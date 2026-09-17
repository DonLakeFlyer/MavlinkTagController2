#include "TunnelCommandDispatcher.h"
#include "formatString.h"

#include <cstring>

using namespace TunnelProtocol;

TunnelCommandDispatcher::TunnelCommandDispatcher(CommandActions& actions, CommandLog& log,
                                                 DetectionCoordinator::HeartbeatSink onHeartbeatStatus,
                                                 RequestCache requestCache)
    : _actions(actions)
    , _log(log)
    , _requestCache(std::move(requestCache))
    , _detection(std::move(onHeartbeatStatus))
{
}

AckInfo_t TunnelCommandDispatcher::makeAck(uint32_t requestId, uint32_t command, uint32_t result, const std::string& message)
{
    AckInfo_t ack;
    memset(&ack, 0, sizeof(ack));
    ack.header.command    = COMMAND_ID_ACK;
    ack.header.request_id = 0;
    ack.command           = command;
    ack.request_id        = requestId;
    ack.result            = result;
    strncpy(ack.message, message.c_str(), sizeof(ack.message) - 1);
    return ack;
}

std::string TunnelCommandDispatcher::commandName(uint32_t command)
{
    switch (command) {
    case COMMAND_ID_ACK:                    return "ACK";
    case COMMAND_ID_START_TAGS:             return "START_TAGS";
    case COMMAND_ID_END_TAGS:               return "END_TAGS";
    case COMMAND_ID_TAG:                    return "TAG";
    case COMMAND_ID_START_DETECTION:        return "START_DETECTION";
    case COMMAND_ID_STOP_DETECTION:         return "STOP_DETECTION";
    case COMMAND_ID_PULSE:                  return "PULSE";
    case COMMAND_ID_RAW_CAPTURE:            return "RAW_CAPTURE";
    case COMMAND_ID_HEARTBEAT:              return "HEARTBEAT";
    case COMMAND_ID_SAVE_LOGS:              return "SAVE_LOGS";
    case COMMAND_ID_CLEAN_LOGS:             return "CLEAN_LOGS";
    case COMMAND_ID_AIRSPY_STATUS:          return "AIRSPY_STATUS";
    case COMMAND_ID_START_COLLECTION:       return "START_COLLECTION";
    case COMMAND_ID_START_COLLECTION_SLICE: return "START_COLLECTION_SLICE";
    case COMMAND_ID_FINISH_COLLECTION:      return "FINISH_COLLECTION";
    case COMMAND_ID_BEARING_RESULT:         return "BEARING_RESULT";
    case COMMAND_ID_COLLECTION_STATUS:      return "COLLECTION_STATUS";
    case COMMAND_ID_PYTHON_PULSE:           return "PYTHON_PULSE";
    }
    return formatString("UNKNOWN(%u)", command);
}

AckInfo_t TunnelCommandDispatcher::handle(const mavlink_tunnel_t& tunnel)
{
    HeaderInfo_t header {};
    if (tunnel.payload_length < sizeof(header)) {
        _log.error(formatString("Tunnel payload too small for header: %u", tunnel.payload_length));
        return makeAck(0, 0, COMMAND_RESULT_FAILURE, "Payload too small");
    }
    memcpy(&header, tunnel.payload, sizeof(header));

    RequestCache::Entry stored;
    switch (_requestCache.lookup(header.request_id, header.command, &stored)) {
    case RequestCache::Lookup::Replay:
        _log.info(formatString("Replaying ack for retried %s request_id:%u result:%u",
                               commandName(header.command).c_str(), header.request_id, stored.result));
        return makeAck(header.request_id, header.command, stored.result, stored.message);
    case RequestCache::Lookup::CommandMismatch:
        _log.error(formatString("request_id %u reused by %s after %s; rejecting",
                                header.request_id, commandName(header.command).c_str(),
                                commandName(stored.command).c_str()));
        return makeAck(header.request_id, header.command, COMMAND_RESULT_FAILURE, "request_id reuse");
    case RequestCache::Lookup::Miss:
        break;
    }

    const Outcome outcome = _dispatch(header, tunnel);
    const uint32_t result = outcome.success ? COMMAND_RESULT_SUCCESS : COMMAND_RESULT_FAILURE;
    _requestCache.store({header.request_id, header.command, result, outcome.message});
    _log.debug(formatString("ack %s request_id:%u %s%s%s", commandName(header.command).c_str(),
                            header.request_id, outcome.success ? "SUCCESS" : "FAILURE",
                            outcome.message.empty() ? "" : " ", outcome.message.c_str()));
    return makeAck(header.request_id, header.command, result, outcome.message);
}

TunnelCommandDispatcher::Outcome TunnelCommandDispatcher::_dispatch(const HeaderInfo_t& header, const mavlink_tunnel_t& tunnel)
{
    switch (header.command) {
    case COMMAND_ID_START_TAGS:      return _startTags(tunnel);
    case COMMAND_ID_TAG:             return _tag(tunnel);
    case COMMAND_ID_END_TAGS:        return _endTags(tunnel);
    case COMMAND_ID_START_DETECTION: return _startDetection(tunnel);
    case COMMAND_ID_STOP_DETECTION:  return _stopDetection();
    case COMMAND_ID_RAW_CAPTURE: {
        const std::string error = _actions.rawCapture(tunnel);
        return {error.empty(), error};
    }
    case COMMAND_ID_SAVE_LOGS:       return {_actions.saveLogs(), ""};
    case COMMAND_ID_CLEAN_LOGS:      return {_actions.cleanLogs(), ""};
    case COMMAND_ID_AIRSPY_STATUS: {
        const std::string error = _actions.airspyStatus();
        return {error.empty(), error};
    }
    case COMMAND_ID_START_COLLECTION: {
        const std::string error = _actions.startCollection(tunnel);
        return error.empty() ? Outcome{true, _actions.detectionLogDir()} : Outcome{false, error};
    }
    case COMMAND_ID_START_COLLECTION_SLICE: {
        const std::string error = _actions.startCollectionSlice(tunnel);
        return {error.empty(), error};
    }
    case COMMAND_ID_FINISH_COLLECTION: {
        const std::string error = _actions.finishCollection(tunnel);
        return {error.empty(), error};
    }
    }
    _log.error(formatString("Unknown tunnel command %u", header.command));
    return {false, "Unknown command"};
}

bool TunnelCommandDispatcher::_controllerIdle()
{
    return _detection.idle() && !_actions.captureInProgress();
}

TunnelCommandDispatcher::Outcome TunnelCommandDispatcher::_startTags(const mavlink_tunnel_t& tunnel)
{
    if (tunnel.payload_length != sizeof(StartTagsInfo_t)) {
        _log.error(formatString("START_TAGS payload length incorrect expected:%zu actual:%u", sizeof(StartTagsInfo_t), tunnel.payload_length));
        return {false, "Payload length incorrect"};
    }
    StartTagsInfo_t info {};
    memcpy(&info, tunnel.payload, sizeof(info));
    _log.debug(formatString("START_TAGS upload_id:%u tag_count:%u", info.upload_id, info.tag_count));

    if (_tagUpload.startTags(_controllerIdle(), info) == TagUploadCoordinator::Result::WrongState) {
        _log.error("START_TAGS rejected: controller not idle");
        return {false, "Controller in incorrect state"};
    }
    return {true, ""};
}

TunnelCommandDispatcher::Outcome TunnelCommandDispatcher::_tag(const mavlink_tunnel_t& tunnel)
{
    if (tunnel.payload_length != sizeof(TagInfo_t)) {
        _log.error(formatString("TAG payload length incorrect expected:%zu actual:%u", sizeof(TagInfo_t), tunnel.payload_length));
        return {false, "Payload length incorrect"};
    }
    TagInfo_t tagInfo {};
    memcpy(&tagInfo, tunnel.payload, sizeof(tagInfo));
    _log.debug(formatString("TAG id:%u index:%u upload_id:%u freq:%u ip1:%u", tagInfo.id, tagInfo.tag_index,
                            tagInfo.upload_id, tagInfo.frequency_hz, tagInfo.intra_pulse1_msecs));

    using R = TagUploadCoordinator::Result;
    switch (_tagUpload.addTag(tagInfo)) {
    case R::Accepted:
        return {true, ""};
    case R::Retransmit:
        // Must not become a second detector on the same port.
        _log.debug(formatString("TAG %u retransmitted; already stored", tagInfo.id));
        return {true, ""};
    case R::NotReceiving:
        _log.error(formatString("TAG %u outside START_TAGS/END_TAGS", tagInfo.id));
        return {false, "TAG outside START_TAGS/END_TAGS"};
    case R::StaleUpload:
        _log.error(formatString("TAG %u upload_id %u does not match open upload %u", tagInfo.id, tagInfo.upload_id, _tagUpload.uploadId()));
        return {false, "stale upload_id"};
    case R::InvalidIndex:
        _log.error(formatString("TAG %u tag_index %u invalid", tagInfo.id, tagInfo.tag_index));
        return {false, "invalid tag_index"};
    case R::InvalidId:
        _log.error("TAG rejected: ids 0 and 1 are reserved");
        return {false, "invalid tag id"};
    case R::InvalidK:
        _log.error(formatString("TAG %u k must be >= 2, got %u", tagInfo.id, tagInfo.k));
        return {false, "k must be >= 2"};
    case R::Conflict:
        _log.error(formatString("TAG %u already defined with different parameters", tagInfo.id));
        return {false, "tag redefined with different parameters"};
    case R::WrongState:
    case R::Incomplete:
    case R::CountMismatch:
        break;
    }
    _log.error(formatString("TAG %u unexpected result", tagInfo.id));
    return {false, "internal error"};
}

TunnelCommandDispatcher::Outcome TunnelCommandDispatcher::_endTags(const mavlink_tunnel_t& tunnel)
{
    if (tunnel.payload_length != sizeof(EndTagsInfo_t)) {
        _log.error(formatString("END_TAGS payload length incorrect expected:%zu actual:%u", sizeof(EndTagsInfo_t), tunnel.payload_length));
        return {false, "Payload length incorrect"};
    }
    EndTagsInfo_t info {};
    memcpy(&info, tunnel.payload, sizeof(info));
    _log.debug(formatString("END_TAGS upload_id:%u tag_count:%u state:%d tags:%zu", info.upload_id, info.tag_count,
                            static_cast<int>(_tagUpload.state()), _tagUpload.tags().size()));

    using R = TagUploadCoordinator::Result;
    switch (_tagUpload.endTags(info)) {
    case R::Accepted:
        // An empty upload must not leave a stale HAS_TAGS from an earlier list.
        _detection.tagsUploaded(_tagUpload.hasTags());
        return {true, ""};
    case R::Retransmit:
        _log.debug("END_TAGS retransmitted; upload already complete");
        return {true, ""};
    case R::Incomplete: {
        const std::string missing = TagUploadCoordinator::formatIndices(_tagUpload.missingIndices());
        _log.error(formatString("END_TAGS incomplete: missing %s", missing.c_str()));
        return {false, "incomplete: missing " + missing};
    }
    case R::StaleUpload:
        _log.error(formatString("END_TAGS upload_id %u does not match open upload %u", info.upload_id, _tagUpload.uploadId()));
        return {false, "stale upload_id"};
    case R::CountMismatch:
        _log.error(formatString("END_TAGS tag_count %u does not match START_TAGS", info.tag_count));
        return {false, "tag_count mismatch"};
    case R::NotReceiving:
        _log.error("END_TAGS without START_TAGS");
        return {false, "END_TAGS without START_TAGS"};
    default:
        break;
    }
    _log.error("END_TAGS unexpected result");
    return {false, "internal error"};
}

std::string TunnelCommandDispatcher::startDetection(const StartDetectionInfo_t& info)
{
    using R = DetectionCoordinator::Result;
    switch (_detection.requestStart()) {
    case R::Accepted: {
        const std::string error = _actions.startDetectionPipeline(info);
        if (!error.empty()) {
            _detection.startFinished(false);
            _log.error("START_DETECTION failed: " + error);
            return error;
        }
        return "";
    }
    case R::AlreadyStarting:
        // Retry (or GCS restart) while the pipeline is still coming up: the
        // intent is already being carried out, so answer as the original did.
        _log.info("START_DETECTION while start in progress; treating as satisfied");
        return "";
    case R::AlreadyDetecting:
        _log.error("START_DETECTION rejected: already detecting");
        return "Detection already running";
    case R::NoTags:
        _log.error("START_DETECTION rejected: no tags uploaded");
        return "Controller in incorrect state";
    default:
        break;
    }
    return "internal error";
}

TunnelCommandDispatcher::Outcome TunnelCommandDispatcher::_startDetection(const mavlink_tunnel_t& tunnel)
{
    if (tunnel.payload_length != sizeof(StartDetectionInfo_t)) {
        _log.error(formatString("START_DETECTION payload length incorrect expected:%zu actual:%u", sizeof(StartDetectionInfo_t), tunnel.payload_length));
        return {false, "Payload length incorrect"};
    }
    StartDetectionInfo_t info {};
    memcpy(&info, tunnel.payload, sizeof(info));
    const std::string error = startDetection(info);
    return error.empty() ? Outcome{true, _actions.detectionLogDir()} : Outcome{false, error};
}

bool TunnelCommandDispatcher::stopDetection(std::string* error)
{
    using R = DetectionCoordinator::Result;
    switch (_detection.requestStop()) {
    case R::Accepted:
        _actions.stopDetectionPipeline();
        return true;
    case R::AlreadyStopping:
        _log.info("STOP_DETECTION while stop in progress; treating as satisfied");
        return true;
    case R::StartInProgress:
        _log.error("STOP_DETECTION rejected: start still in progress");
        if (error) *error = "Detection start in progress; retry";
        return false;
    case R::NotDetecting:
        _log.error("STOP_DETECTION rejected: not detecting");
        if (error) *error = "Not detecting";
        return false;
    default:
        break;
    }
    if (error) *error = "internal error";
    return false;
}

TunnelCommandDispatcher::Outcome TunnelCommandDispatcher::_stopDetection()
{
    std::string error;
    const bool ok = stopDetection(&error);
    return {ok, error};
}
