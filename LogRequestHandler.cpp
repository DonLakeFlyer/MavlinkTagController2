#include "LogRequestHandler.h"
#include "LogFileManager.h"
#include "MavlinkSystem.h"
#include "log.h"
#include "formatString.h"

#include <filesystem>

LogRequestHandler::LogRequestHandler(MavlinkSystem* mavlink)
    : _mavlink  (mavlink)
    , _homePath (getenv("HOME"))
{
    using namespace std::placeholders;
    _mavlink->subscribeToMessage(MAVLINK_MSG_ID_LOG_REQUEST_LIST, std::bind(&LogRequestHandler::_handleLogRequestList, this, _1));
}

void LogRequestHandler::_handleLogRequestList(const mavlink_message_t& message)
{
    std::thread([this]() {
        log_request_list_t logRequestList;

        mavlink_msg_log_request_list_decode(&message, &logRequestList);
        logInfo() << "LOG_REQUEST_LIST: start/end: " << logRequestList.start << "/" << logRequestList.end;
        bool lastAvailable = logRequestList.end == 0xffff;

        typedef struct {
            std::string                 directory;
            std::vector<std::string>    files;
        } DirEntry_t;
        std::vector<DirEntry_t> dirEntries;

        // Get list of directories in home directory which start with "Logs-"
        uint16_t numLogs = 0;
        for (const auto& entry : std::filesystem::directory_iterator(_homePath)) {
            if (entry.is_directory() && entry.path().filename().string().starts_with(LogFileManager::logDirPrefix())) {
                auto logDir = entry.path().filename().string();
                logDebug() << "Found log directory: " << logDir;

                DirEntry_t dirEntry;
                dirEntry.directory = logDir;
                dirEntries.push_back(dirEntry);

                // Add all the files in the directory to the files vector
                std::string logDirPath = formatString("%s/%s", _homePath, logDir.c_str());
                for (const auto& entry : std::filesystem::directory_iterator(logDirPath)) {
                    if (entry.is_regular_file()) {
                        std::string logFile = entry.path().filename().string();
                        logDebug() << "Found log file: " << logFile;

                        dirEntries.back().files.push_back(logFile);
                        numLogs++;
                    }
                }
            }
        }

        // Send the requested log entries
        int curLogIndex = 0;
        for (const auto& dirEntry : dirEntries) {
            std::string logDirPath = formatString("%s/%s", _homePath, dirEntry.directory.c_str());
            for (const auto& logFile : dirEntry.files) {
                if (curLogIndex >= logRequestList.start && (lastAvailable || curLogIndex <= logRequestList.end)) {
                    mavlink_message_t   message;
                    mavlink_log_entry_t logEntry;

                    memset(&logEntry, 0, sizeof(logEntry));

                    logEntry.id = logIndex

            for (const auto& entry : std::filesystem::directory_iterator(logDirPath)) {
                if (entry.is_regular_file()) {
                    std::string logFile = entry.path().filename().string();
                    logDebug() << "Found log file: " << logFile;

                    if (logIndex >= logRequestList.start && (lastAvailable || logIndex <= logRequestList.end)) {
                        mavlink_message_t   message;
                        mavlink_log_entry_t logEntry;

                        memset(&logEntry, 0, sizeof(logEntry));

                        logEntry.id = logIndex
                        logEntry.num_logs = logDirs.size();
                        tunnel.target_system    = gcsSystemId().value();
                        tunnel.target_component = 0;
                        tunnel.payload_type     = MAV_TUNNEL_PAYLOAD_TYPE_UNKNOWN;
                        tunnel.payload_length   = tunnelPayloadSize;

                        memcpy(tunnel.payload, tunnelPayload, tunnelPayloadSize);

                        mavlink_msg_tunnel_encode(
                            ourSystemId().value(),
                            ourComponentId(),
                            &message,
                            &tunnel);
                    } else {
                        logDebug() << "Skipping log file, due to logIndex" << logIndex;
                    }
                    logIndex++;
                }
            }
        }
    }).detach();
}
