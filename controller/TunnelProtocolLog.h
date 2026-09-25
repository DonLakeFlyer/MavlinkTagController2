#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

// Log-line rendering of TunnelProtocol frames using the names from TunnelProtocol.h.
namespace TunnelProtocolLog {

std::string commandName(uint32_t command);              // COMMAND_ID_*
std::string commandResultName(uint32_t result);         // COMMAND_RESULT_*
std::string collectionStatusName(uint32_t status);      // COLLECTION_STATUS_*
std::string collectionFinishName(uint32_t disposition); // COLLECTION_FINISH_*
std::string operationStateName(uint32_t state);         // OPERATION_STATE_*
std::string heartbeatStatusName(uint32_t status);       // HEARTBEAT_STATUS_*
std::string detectionModeName(uint32_t mode);           // DETECTION_MODE_*
std::string logLevelName(uint32_t level);               // LOG_LEVEL_*

// "START_COLLECTION_SLICE <verb>: request_id:N field:value ..." for a raw tunnel payload.
std::string describe(const char* verb, const void* payload, size_t length);

} // namespace TunnelProtocolLog
