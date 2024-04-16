#include <algorithm>
#include <fstream>
#include <filesystem>
#include <future>
#include <cstring>

#include "MavlinkFtpServer.h"
#include "MavlinkSystem.h"
#include "log.h"

namespace fs = std::filesystem;

MavlinkFtpServer::MavlinkFtpServer(MavlinkSystem* mavlink)
    : _mavlink  (mavlink)
    , _homePath (getenv("HOME"))
{
    using namespace std::placeholders;
    _mavlink->subscribeToMessage(MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL, std::bind(&MavlinkFtpServer::_handleFileTransferProtocol, this, _1));
}

MavlinkFtpServer::~MavlinkFtpServer()
{
    std::lock_guard<std::mutex> lock(_mutex);
    _reset();
}

void MavlinkFtpServer::_handleFileTransferProtocol(const mavlink_message_t& msg)
{
    mavlink_file_transfer_protocol_t fileTransferProtocolMsg;
    mavlink_msg_file_transfer_protocol_decode(&msg, &fileTransferProtocolMsg);

    if (fileTransferProtocolMsg.target_system != _mavlink->ourSystemId() || fileTransferProtocolMsg.target_component != _mavlink->ourComponentId()) {
        return;
    }

    const PayloadHeader& payload = *reinterpret_cast<PayloadHeader*>(&fileTransferProtocolMsg.payload[0]);

    // Basic sanity check: validate length before use.
    if (payload.size > maxRequestHeaderDataLength) {
        logWarn() << "FTP: Nak - Invalid incoming data size size:max" << payload.size << ":" << maxRequestHeaderDataLength;
        auto response = PayloadHeader{};
        response.seq_number = payload.seq_number + 1;
        response.req_opcode = payload.opcode;
        response.opcode = Opcode::RSP_NAK;
        response.data[0] = ServerResult::ERR_INVALID_DATA_SIZE;
        response.size = 1;
        _sendMavlinkFtpMessage(response);
        return;
    }

    logDebug() << "_handleFileTransferProtocol opcode: " << _opcodeToString(payload.opcode) 
        << "size:" << payload.size 
        << "offset:" << payload.offset 
        << "seq:" << payload.seq_number;

    switch (payload.opcode) {
    case Opcode::CMD_TERMINATE_SESSION:
        _workTerminate(payload);
        break;
    case Opcode::CMD_RESET_SESSIONS:
        _workReset(payload);
        break;
    case Opcode::CMD_LIST_DIRECTORY:
        _workList(payload);
        break;
    case Opcode::CMD_OPEN_FILE_RO:
        _workOpenFileReadOnly(payload);
        break;
    case Opcode::CMD_BURST_READ_FILE:
        _workBurst(payload);
        break;
    case Opcode::CMD_REMOVE_DIRECTORY:
        _workRemoveDirectory(payload);
        break;
    default:
        // FIXME: Should nak command
        return;
    }
}

void MavlinkFtpServer::_sendMavlinkFtpMessage(const PayloadHeader& payload)
{
    mavlink_message_t message;

    mavlink_msg_file_transfer_protocol_pack(
        _mavlink->ourSystemId().value(),
        _mavlink->ourComponentId(),
        &message,
        0,                          // target_network
        _mavlink->gcsSystemId().value(),
        0,                          // target_component
        reinterpret_cast<const uint8_t*>(&payload));
    _mavlink->sendMessage(message);
}

std::string MavlinkFtpServer::_dataAsString(const PayloadHeader& payload, size_t entry)
{
    size_t start = 0;
    size_t end = 0;
    std::string result;

    for (int i = entry; i >= 0; --i) {
        start = end;
        end +=
            strnlen(reinterpret_cast<const char*>(&payload.data[start]), maxRequestHeaderDataLength - start) +
            1;
    }

    result.resize(end - start);
    std::memcpy(result.data(), &payload.data[start], end - start);

    return result;
}

std::variant<std::string, MavlinkFtpServer::ServerResult>
MavlinkFtpServer::_pathFromPayload(const PayloadHeader& payload, size_t entry)
{
    // Requires lock

    auto data = _dataAsString(payload, entry);
    return _pathFromString(data);
}

std::variant<std::string, MavlinkFtpServer::ServerResult>
MavlinkFtpServer::_pathFromString(const std::string& payload_path)
{
    // Requires lock

    if (payload_path.empty()) {
        return ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
    }

    fs::path combined_path;
    if (payload_path == ".") {
        // This indicates we want home directory
        combined_path = fs::path(_homePath).lexically_normal();
    } else {
        combined_path = (fs::path(_homePath) / payload_path).lexically_normal();
    }

    return combined_path.string();
}

void MavlinkFtpServer::_workList(const PayloadHeader& payload)
{
    auto response = PayloadHeader{};
    response.seq_number = payload.seq_number + 1;
    response.req_opcode = payload.opcode;
    response.size = 0;

    std::lock_guard<std::mutex> lock(_mutex);

    auto maybe_path = _pathFromPayload(payload);
    if (std::holds_alternative<ServerResult>(maybe_path)) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = std::get<ServerResult>(maybe_path);
        _sendMavlinkFtpMessage(response);
        return;
    }

    fs::path path = std::get<std::string>(maybe_path);

    // Move to the requested offset
    uint32_t requested_offset = payload.offset;

    std::error_code ec;
    if (!fs::exists(path, ec)) {
        logWarn() << "FTP: can't open path " << path;
        // this is not an FTP error, abort directory by simulating eof
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
        _sendMavlinkFtpMessage(response);
        return;
    }

    logDebug() << "Directory path: " << path.string();

    for (const auto& entry : fs::directory_iterator(fs::canonical(path))) {
        if (requested_offset > 0) {
            requested_offset--;
            continue;
        }
        const auto name = entry.path().filename();

        std::string payload_str;

        const auto is_regular_file = entry.is_regular_file(ec);
        if (ec) {
            logWarn() << "Could not determine whether '" << entry.path().string()
                      << "' is a file: " << ec.message();
            continue;
        }

        const auto is_directory = entry.is_directory(ec);
        if (ec) {
            logWarn() << "Could not determine whether '" << entry.path().string()
                      << "' is a directory: " << ec.message();
            continue;
        }

        if (is_regular_file) {
            const auto filesize = fs::file_size(entry.path(), ec);
            if (ec) {
                logWarn() << "Could not get file size of '" << entry.path().string()
                          << "': " << ec.message();
                continue;
            }

            logDebug() << "Found file: " << name.string() << ", size: " << filesize << " bytes";

            payload_str += 'F';
            payload_str += name.string();

        } else if (is_directory) {
            logDebug() << "Found directory: " << name.string();

            payload_str += 'D';
            payload_str += name.string();

        } else {
            logDebug() << "Sending S(kip) for non file/directory: " << name.string();

            payload_str += 'S';
            payload_str += name.string();
        }

        auto required_len = payload_str.length() + 1;

        // Do we have room for the dir entry and the null terminator?
        if (response.size + required_len > maxRequestHeaderDataLength) {
            break;
        }

        std::memcpy(&response.data[response.size], payload_str.c_str(), required_len);

        response.size += static_cast<uint8_t>(required_len);
    }

    if (response.size == 0) {
        // We are done and need to respond with EOF.
        logDebug() << "End of dir list - sending NAK";
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_EOF;

    } else {
        response.opcode = Opcode::RSP_ACK;
        response.offset = payload.offset;
    }

    _sendMavlinkFtpMessage(response);
}

void MavlinkFtpServer::_workOpenFileReadOnly(const PayloadHeader& payload)
{
    auto response = PayloadHeader{};
    response.seq_number = payload.seq_number + 1;
    response.req_opcode = payload.opcode;

    std::lock_guard<std::mutex> lock(_mutex);
    if (_session_info.ifstream.is_open()) {
        _reset();
    }

    std::string path;
    {
        std::lock_guard<std::mutex> tmp_lock(_tempFilesMutex);
        const auto it = _tempFiles.find(_dataAsString(payload));
        if (it != _tempFiles.end()) {
            path = it->second;
        } else {
            auto maybe_path = _pathFromPayload(payload);
            if (std::holds_alternative<ServerResult>(maybe_path)) {
                response.opcode = Opcode::RSP_NAK;
                response.size = 1;
                response.data[0] = std::get<ServerResult>(maybe_path);
                _sendMavlinkFtpMessage(response);
                return;
            }

            path = std::get<std::string>(maybe_path);
        }
    }

    logInfo() << "Finding " << path << " in " << _homePath;
    if (path.rfind(_homePath, 0) != 0) {
        logWarn() << "FTP: invalid path " << path;
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL;
        _sendMavlinkFtpMessage(response);
        return;
    }

    logDebug() << "Going to open readonly: " << path;

    std::error_code ec;
    if (!fs::exists(path, ec)) {
        logError() << "FTP: Open failed - file doesn't exist";
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
        _sendMavlinkFtpMessage(response);
        return;
    }

    auto file_size = static_cast<uint32_t>(fs::file_size(path, ec));
    if (ec) {
        logError() << "Could not determine file size of '" << path << "': " << ec.message();
        return;
    }

    logDebug() << "Determined filesize to be: " << file_size << " bytes";

    std::ifstream ifstream;
    ifstream.open(path, std::ios::in | std::ios::binary);

    if (!ifstream.is_open()) {
        logWarn() << "FTP: Open failed";
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL;
        _sendMavlinkFtpMessage(response);
        return;
    }

    _session_info.ifstream = std::move(ifstream);
    _session_info.file_size = file_size;

    response.opcode = Opcode::RSP_ACK;
    response.session = 0;
    response.seq_number = payload.seq_number + 1;
    response.size = sizeof(uint32_t);
    std::memcpy(response.data, &file_size, response.size);

    _sendMavlinkFtpMessage(response);
}

void MavlinkFtpServer::_workBurst(const PayloadHeader& payload)
{
    auto response = PayloadHeader{};
    response.req_opcode = payload.opcode;

    std::lock_guard<std::mutex> lock(_mutex);
    if (payload.session != 0 || !_session_info.ifstream.is_open()) {
        _reset();
    }

    // We have to test seek past EOF ourselves, lseek will allow seek past EOF
    if (payload.offset >= _session_info.file_size) {
        response.seq_number = payload.seq_number + 1;
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_EOF;
        logDebug() << "Reached EOF reading";
        _sendMavlinkFtpMessage(response);
        return;
    }

    logDebug() << "Seek to " << payload.offset;
    _session_info.ifstream.seekg(payload.offset);
    if (_session_info.ifstream.fail()) {
        response.seq_number = payload.seq_number + 1;
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL;
        logError() << "Seek failed";
        _sendMavlinkFtpMessage(response);
        return;
    }

    _session_info.burst_offset = payload.offset;
    _session_info.burst_chunk_size = payload.size;
    _burstSeq = payload.seq_number + 1;

    if (_session_info.burst_thread.joinable()) {
        _session_info.burst_stop = true;
        _session_info.burst_thread.join();
    }

    _session_info.burst_stop = false;

    // Schedule sending out burst messages.
    _session_info.burst_thread = std::thread([this]() {
        while (!_session_info.burst_stop)
            if (_sendBurstPacket())
                break;
    });

    // Don't send response as that's done in the call every burst call above.
}

// Returns true if sending is complete
bool MavlinkFtpServer::_sendBurstPacket()
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_session_info.ifstream.is_open()) {
        return false;
    }

    PayloadHeader burst_packet{};
    burst_packet.req_opcode = Opcode::CMD_BURST_READ_FILE;
    burst_packet.seq_number = _burstSeq++;

    _makeBurstPacket(burst_packet);

    _sendMavlinkFtpMessage(burst_packet);

    if (burst_packet.burst_complete == 1) {
        return true;
    }

    return false;
}

void MavlinkFtpServer::_makeBurstPacket(PayloadHeader& packet)
{
    uint32_t bytes_to_read = std::min(
        static_cast<uint32_t>(_session_info.burst_chunk_size),
        _session_info.file_size - _session_info.burst_offset);

    logDebug() << "Burst read of " << bytes_to_read << " bytes";
    _session_info.ifstream.read(reinterpret_cast<char*>(packet.data), bytes_to_read);

    if (_session_info.ifstream.fail()) {
        packet.opcode = Opcode::RSP_NAK;
        packet.size = 1;
        packet.data[0] = ServerResult::ERR_FAIL;
        logWarn() << "Burst read failed";
        return;
    }

    const uint32_t bytes_read = _session_info.ifstream.gcount();
    packet.size = bytes_read;
    packet.opcode = Opcode::RSP_ACK;

    packet.offset = _session_info.burst_offset;
    _session_info.burst_offset += bytes_read;

    if (_session_info.burst_offset == _session_info.file_size) {
        // Last read, we are done for this burst.
        packet.burst_complete = 1;
        logDebug() << "Burst complete";
    }
}

void MavlinkFtpServer::_workTerminate(const PayloadHeader& payload)
{
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _reset();
    }

    auto response = PayloadHeader{};
    response.seq_number = payload.seq_number + 1;
    response.req_opcode = payload.opcode;
    response.opcode = Opcode::RSP_ACK;
    response.size = 0;
    _sendMavlinkFtpMessage(response);
}

void MavlinkFtpServer::_reset()
{
    // requires lock
    if (_session_info.ifstream.is_open()) {
        _session_info.ifstream.close();
    }

    if (_session_info.ofstream.is_open()) {
        _session_info.ofstream.close();
    }

    _session_info.burst_stop = true;
    if (_session_info.burst_thread.joinable()) {
        _session_info.burst_thread.join();
    }
}

void MavlinkFtpServer::_workReset(const PayloadHeader& payload)
{
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _reset();
    }

    auto response = PayloadHeader{};
    response.seq_number = payload.seq_number + 1;
    response.req_opcode = payload.opcode;
    response.opcode = Opcode::RSP_ACK;
    response.size = 0;
    _sendMavlinkFtpMessage(response);
}

void MavlinkFtpServer::_workRemoveDirectory(const PayloadHeader& payload)
{
    auto response = PayloadHeader{};
    response.seq_number = payload.seq_number + 1;
    response.req_opcode = payload.opcode;

    std::lock_guard<std::mutex> lock(_mutex);

    auto maybe_path = _pathFromPayload(payload);
    if (std::holds_alternative<ServerResult>(maybe_path)) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = std::get<ServerResult>(maybe_path);
        _sendMavlinkFtpMessage(response);
        return;
    }

    fs::path path = std::get<std::string>(maybe_path);

    std::error_code ec;
    if (!fs::exists(path, ec)) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
        _sendMavlinkFtpMessage(response);
        return;
    }
    if (fs::remove(path, ec)) {
        response.opcode = Opcode::RSP_ACK;
        response.size = 0;
    } else {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL;
    }

    _sendMavlinkFtpMessage(response);
}

void MavlinkFtpServer::_workRemoveFile(const PayloadHeader& payload)
{
    auto response = PayloadHeader{};
    response.seq_number = payload.seq_number + 1;
    response.req_opcode = payload.opcode;

    std::lock_guard<std::mutex> lock(_mutex);

    auto maybe_path = _pathFromPayload(payload);
    if (std::holds_alternative<ServerResult>(maybe_path)) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = std::get<ServerResult>(maybe_path);
        _sendMavlinkFtpMessage(response);
        return;
    }

    auto path = std::get<std::string>(maybe_path);

    std::error_code ec;
    if (!fs::exists(path, ec)) {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL_FILE_DOES_NOT_EXIST;
        _sendMavlinkFtpMessage(response);
        return;
    }
    if (fs::remove(path, ec)) {
        response.opcode = Opcode::RSP_ACK;
    } else {
        response.opcode = Opcode::RSP_NAK;
        response.size = 1;
        response.data[0] = ServerResult::ERR_FAIL;
    }

    _sendMavlinkFtpMessage(response);
}

std::string MavlinkFtpServer::_opcodeToString(MavlinkFtpServer::Opcode opcode)
{
    std::string resultStr;

    switch (opcode) {
    case Opcode::CMD_NONE:
        resultStr = "CMD_NONE";
        break;
    case Opcode::CMD_TERMINATE_SESSION:
        resultStr = "CMD_TERMINATE_SESSION";
        break;
    case Opcode::CMD_RESET_SESSIONS:
        resultStr = "CMD_RESET_SESSIONS";
        break;
    case Opcode::CMD_LIST_DIRECTORY:
        resultStr = "CMD_LIST_DIRECTORY";
        break;
    case Opcode::CMD_OPEN_FILE_RO:
        resultStr = "CMD_OPEN_FILE_RO";
        break;
    case Opcode::CMD_READ_FILE:
        resultStr = "CMD_READ_FILE";
        break;
    case Opcode::CMD_CREATE_FILE:
        resultStr = "CMD_CREATE_FILE";
        break;
    case Opcode::CMD_WRITE_FILE:
        resultStr = "CMD_WRITE_FILE";
        break;
    case Opcode::CMD_REMOVE_FILE:
        resultStr = "CMD_REMOVE_FILE";
        break;
    case Opcode::CMD_CREATE_DIRECTORY:
        resultStr = "CMD_CREATE_DIRECTORY";
        break;
    case Opcode::CMD_REMOVE_DIRECTORY:
        resultStr = "CMD_REMOVE_DIRECTORY";
        break;
    case Opcode::CMD_OPEN_FILE_WO:
        resultStr = "CMD_OPEN_FILE_WO";
        break;
    case Opcode::CMD_TRUNCATE_FILE:
        resultStr = "CMD_TRUNCATE_FILE";
        break;
    case Opcode::CMD_RENAME:
        resultStr = "CMD_RENAME";
        break;
    case Opcode::CMD_CALC_FILE_CRC32:
        resultStr = "CMD_CALC_FILE_CRC32";
        break;
    case Opcode::CMD_BURST_READ_FILE:
        resultStr = "CMD_BURST_READ_FILE";
        break;
    default:
        resultStr = "CMD Unknown";
        break;
    }

    return resultStr;
}
