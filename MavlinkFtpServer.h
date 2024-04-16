#pragma once

#include <cinttypes>
#include <fstream>
#include <unordered_map>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>
#include <thread>

#include <mavlink.h>

// As found in
// https://stackoverflow.com/questions/1537964#answer-3312896
#ifdef _MSC_VER // MSVC
#define PACK(__Declaration__) __pragma(pack(push, 1)) __Declaration__ __pragma(pack(pop))
#else
#define PACK(__Declaration__) __Declaration__ __attribute__((__packed__))
#endif

class MavlinkSystem;

class MavlinkFtpServer {
public:
    explicit MavlinkFtpServer(MavlinkSystem* mavlink);
    ~MavlinkFtpServer();

    struct ProgressData {
        uint32_t bytes_transferred{}; /**< @brief The number of bytes already transferred. */
        uint32_t total_bytes{}; /**< @brief The total bytes to transfer. */
    };

private:
    /// @brief Possible server results returned for requests.
    enum ServerResult : uint8_t {
        SUCCESS,
        ERR_FAIL, ///< Unknown failure
        ERR_FAIL_ERRNO, ///< Command failed, errno sent back in PayloadHeader.data[1]
        ERR_INVALID_DATA_SIZE, ///< PayloadHeader.size is invalid
        ERR_INVALID_SESSION, ///< Session is not currently open
        ERR_NO_SESSIONS_AVAILABLE, ///< All available Sessions in use
        ERR_EOF, ///< Offset past end of file for List and Read commands
        ERR_UNKOWN_COMMAND, ///< Unknown command opcode
        ERR_FAIL_FILE_EXISTS, ///< File exists already
        ERR_FAIL_FILE_PROTECTED, ///< File is write protected
        ERR_FAIL_FILE_DOES_NOT_EXIST, ///< File does not exist

        // These error codes are returned to client without contacting the server
        ERR_TIMEOUT = 200, ///< Timeout
        ERR_FILE_IO_ERROR, ///< File IO operation error
    };

    /// @brief Command opcodes
    enum class Opcode : uint8_t {
        CMD_NONE, ///< ignored, always acked
        CMD_TERMINATE_SESSION, ///< Terminates open Read session
        CMD_RESET_SESSIONS, ///< Terminates all open Read sessions
        CMD_LIST_DIRECTORY, ///< List files in <path> from <offset>
        CMD_OPEN_FILE_RO, ///< Opens file at <path> for reading, returns <session>
        CMD_READ_FILE, ///< Reads <size> bytes from <offset> in <session>
        CMD_CREATE_FILE, ///< Creates file at <path> for writing, returns <session>
        CMD_WRITE_FILE, ///< Writes <size> bytes to <offset> in <session>
        CMD_REMOVE_FILE, ///< Remove file at <path>
        CMD_CREATE_DIRECTORY, ///< Creates directory at <path>
        CMD_REMOVE_DIRECTORY, ///< Removes Directory at <path>, must be empty
        CMD_OPEN_FILE_WO, ///< Opens file at <path> for writing, returns <session>
        CMD_TRUNCATE_FILE, ///< Truncate file at <path> to <offset> length
        CMD_RENAME, ///< Rename <path1> to <path2>
        CMD_CALC_FILE_CRC32, ///< Calculate CRC32 for file at <path>
        CMD_BURST_READ_FILE, ///< Burst download session file

        RSP_ACK = 128, ///< Ack response
        RSP_NAK ///< Nak response
    };

    static constexpr uint8_t maxRequestHeaderDataLength = 239; ///< Maximum data size in RequestHeader::data

    /// @brief This is the payload which is in mavlink_file_transfer_protocol_t.payload.
    /// This needs to be packed, because it's typecasted from
    /// mavlink_file_transfer_protocol_t.payload, which starts at a 3 byte offset, causing an
    /// unaligned access to seq_number and offset
    PACK(struct PayloadHeader {
        uint16_t seq_number; ///< sequence number for message
        uint8_t session; ///< Session id for read and write commands
        Opcode opcode; ///< Command opcode
        uint8_t size; ///< Size of data
        Opcode req_opcode; ///< Request opcode returned in RSP_ACK, RSP_NAK message
        uint8_t burst_complete; ///< Only used if req_opcode=CMD_BURST_READ_FILE - 1: set of burst
        ///< packets complete, 0: More burst packets coming.
        uint8_t padding; ///< 32 bit alignment padding
        uint32_t offset; ///< Offsets for List and Read commands
        uint8_t data[maxRequestHeaderDataLength]; ///< command data, varies by Opcode
    });

    static_assert(
        sizeof(PayloadHeader) == sizeof(mavlink_file_transfer_protocol_t::payload),
        "PayloadHeader size is incorrect.");

    struct SessionInfo {
        uint32_t file_size{0};
        uint32_t burst_offset{0};
        uint8_t burst_chunk_size{0};
        std::ifstream ifstream;
        std::ofstream ofstream;
        bool burst_stop{false};
        std::thread burst_thread;
    } _session_info{};



private:
    void        _handleFileTransferProtocol (const mavlink_message_t& msg);
    std::string _opcodeToString             (MavlinkFtpServer::Opcode opcode);
    void        _sendMavlinkFtpMessage      (const PayloadHeader& payload);
    void        _reset                      ();
    void        _workList                   (const PayloadHeader& payload);
    void        _workOpenFileReadOnly       (const PayloadHeader& payload);
    void        _workBurst                  (const PayloadHeader& payload);
    void        _workTerminate              (const PayloadHeader& payload);
    void        _workReset                  (const PayloadHeader& payload);
    void        _workRemoveDirectory        (const PayloadHeader& payload);
    void        _workRemoveFile             (const PayloadHeader& payload);
    bool        _sendBurstPacket            ();
    void        _makeBurstPacket            (PayloadHeader& packet);


    std::string _dataAsString(const PayloadHeader& payload, size_t entry = 0);
    std::variant<std::string, ServerResult>
    _pathFromPayload(const PayloadHeader& payload, size_t entry = 0);
    std::variant<std::string, ServerResult> _pathFromString(const std::string& payload_path);

    MavlinkSystem*  _mavlink;
    std::string     _homePath;
    std::mutex      _mutex{};
    std::mutex      _tempFilesMutex{};
    uint16_t        _burstSeq = 0;
    std::unordered_map<std::string, std::string> _tempFiles{};
};
