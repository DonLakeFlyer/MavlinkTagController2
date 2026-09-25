#include "TunnelCommandDispatcher.h"
#include "TunnelProtocolLog.h"
#include "formatString.h"
#include "logLevel.h"

#include <cstring>

using namespace TunnelProtocol;

TunnelCommandDispatcher::TunnelCommandDispatcher(CommandActions& actions, CommandLog& log, OperationProgressReporter& progress,
                                                 DetectionCoordinator::HeartbeatSink onHeartbeatStatus,
                                                 RequestCache requestCache)
    : _actions(actions)
    , _log(log)
    , _progress(progress)
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
    return TunnelProtocolLog::commandName(command);
}

AckInfo_t TunnelCommandDispatcher::handle(const mavlink_tunnel_t& tunnel)
{
    HeaderInfo_t header {};
    if (tunnel.payload_length < sizeof(header)) {
        _log.error(formatString("Tunnel payload too small for header: %u", tunnel.payload_length));
        return makeAck(0, 0, COMMAND_RESULT_FAILURE, "Payload too small");
    }
    if (tunnel.payload_length > sizeof(tunnel.payload)) {
        _log.error(formatString("Tunnel payload_length %u exceeds frame capacity %zu", tunnel.payload_length, sizeof(tunnel.payload)));
        return makeAck(0, 0, COMMAND_RESULT_FAILURE, "Payload too large");
    }
    memcpy(&header, tunnel.payload, sizeof(header));

    RequestCache::Entry stored;
    switch (_requestCache.lookup(header.request_id, header.command, tunnel.payload, tunnel.payload_length, &stored)) {
    case RequestCache::Lookup::Replay:
        _log.info(formatString("Replaying ack for retried %s request_id:%u result:%u",
                               commandName(header.command).c_str(), header.request_id, stored.result));
        if (stored.result == COMMAND_RESULT_SUCCESS) {
            _replayCollectionFrames(header.command, tunnel);
        }
        return makeAck(header.request_id, header.command, stored.result, stored.message);
    case RequestCache::Lookup::CommandMismatch:
        _log.error(formatString("request_id %u reused by %s after %s; rejecting",
                                header.request_id, commandName(header.command).c_str(),
                                commandName(stored.command).c_str()));
        return makeAck(header.request_id, header.command, COMMAND_RESULT_FAILURE, "request_id reuse");
    case RequestCache::Lookup::PayloadMismatch:
        _log.error(formatString("request_id %u reused by %s with a different payload; rejecting",
                                header.request_id, commandName(header.command).c_str()));
        return makeAck(header.request_id, header.command, COMMAND_RESULT_FAILURE, "request_id reuse");
    case RequestCache::Lookup::Miss:
        break;
    }

    const Outcome outcome = _dispatch(header, tunnel);
    const uint32_t result = outcome.success ? COMMAND_RESULT_SUCCESS : COMMAND_RESULT_FAILURE;
    _requestCache.store({header.request_id, header.command, result, outcome.message,
                         std::vector<uint8_t>(tunnel.payload, tunnel.payload + tunnel.payload_length)});
    _log.debug(formatString("ack %s request_id:%u %s%s%s", commandName(header.command).c_str(),
                            header.request_id, outcome.success ? "SUCCESS" : "FAILURE",
                            outcome.message.empty() ? "" : " ", outcome.message.c_str()));
    return makeAck(header.request_id, header.command, result, outcome.message);
}

TunnelCommandDispatcher::Outcome TunnelCommandDispatcher::_dispatch(const HeaderInfo_t& header, const mavlink_tunnel_t& tunnel)
{
    switch (header.command) {
    case COMMAND_ID_STOP_DETECTION:
    case COMMAND_ID_SAVE_LOGS:
    case COMMAND_ID_CLEAN_LOGS:
    case COMMAND_ID_AIRSPY_STATUS:
        if (tunnel.payload_length != sizeof(HeaderInfo_t)) {
            _log.error(formatString("%s payload length incorrect expected:%zu actual:%u", commandName(header.command).c_str(),
                                    sizeof(HeaderInfo_t), tunnel.payload_length));
            return {false, "Payload length incorrect"};
        }
        break;
    default:
        break;
    }
    // Starting still publishes HAS_TAGS, so the heartbeat alone would let
    // CLEAN_LOGS delete the session directory being opened.
    if ((header.command == COMMAND_ID_SAVE_LOGS || header.command == COMMAND_ID_CLEAN_LOGS) && !_controllerIdle()) {
        _log.error(formatString("%s rejected: controller not idle", commandName(header.command).c_str()));
        return {false, "Controller in incorrect state"};
    }
    // Idle in the detection sense still allows a log save/delete or the
    // post-flight analysis to be running; only one such operation at a time.
    if (header.command == COMMAND_ID_SAVE_LOGS || header.command == COMMAND_ID_CLEAN_LOGS || header.command == COMMAND_ID_RAW_CAPTURE) {
        const std::string busy = _progress.busyMessage();
        if (!busy.empty()) {
            _log.error(formatString("%s rejected: %s", commandName(header.command).c_str(), busy.c_str()));
            return {false, busy};
        }
    }

    switch (header.command) {
    case COMMAND_ID_START_TAGS:      return _startTags(tunnel);
    case COMMAND_ID_TAG:             return _tag(tunnel);
    case COMMAND_ID_END_TAGS:        return _endTags(tunnel);
    case COMMAND_ID_START_DETECTION: return _startDetection(tunnel);
    case COMMAND_ID_STOP_DETECTION:  return _stopDetection(header.request_id);
    case COMMAND_ID_RAW_CAPTURE: {
        const std::string error = _actions.rawCapture(tunnel);
        return {error.empty(), error};
    }
    case COMMAND_ID_SAVE_LOGS: {
        const std::string error = _actions.saveLogs(header.request_id);
        return {error.empty(), error};
    }
    case COMMAND_ID_CLEAN_LOGS: {
        const std::string error = _actions.cleanLogs(header.request_id);
        return {error.empty(), error};
    }
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
    case COMMAND_ID_SET_LOG_LEVEL: {
        if (tunnel.payload_length != sizeof(SetLogLevel_t)) {
            _log.error(formatString("SET_LOG_LEVEL payload length incorrect expected:%zu actual:%u", sizeof(SetLogLevel_t), tunnel.payload_length));
            return {false, "Payload length incorrect"};
        }
        SetLogLevel_t info {};
        memcpy(&info, tunnel.payload, sizeof(info));
        if (info.level != LOG_LEVEL_DEBUG && info.level != LOG_LEVEL_VERBOSE) {
            _log.error(formatString("SET_LOG_LEVEL rejected: unknown level %u", info.level));
            return {false, "Unknown log level"};
        }
        setVerboseLogging(info.level == LOG_LEVEL_VERBOSE);
        _log.info(formatString("Verbose logging %s", info.level == LOG_LEVEL_VERBOSE ? "enabled" : "disabled"));
        return {true, ""};
    }
    }
    _log.error(formatString("Unknown tunnel command %u", header.command));
    return {false, "Unknown command"};
}

// The collection commands push status frames (SLICE_ARMED / SLICE_COMPLETE,
// the FINISH outcome) that have no ACK of their own. A cached-success retry
// means the GCS may have lost those too, so re-send them through paths that
// cannot change collection state.
void TunnelCommandDispatcher::_replayCollectionFrames(uint32_t command, const mavlink_tunnel_t& tunnel)
{
    switch (command) {
    case COMMAND_ID_START_COLLECTION_SLICE:
        // The handler's Duplicate / AlreadyComplete paths re-ARM or replay
        // SLICE_COMPLETE and are retry-safe by design.
        if (tunnel.payload_length == sizeof(StartCollectionSlice_t)) {
            _actions.startCollectionSlice(tunnel);
        }
        break;
    case COMMAND_ID_FINISH_COLLECTION:
        if (tunnel.payload_length == sizeof(FinishCollection_t)) {
            FinishCollection_t info {};
            memcpy(&info, tunnel.payload, sizeof(info));
            _actions.replayFinishOutcome(info.collection_id);
        }
        break;
    default:
        break;
    }
}

bool TunnelCommandDispatcher::_controllerIdle()
{
    return _detection.idle();
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

    using R = TagUploadCoordinator::Result;
    switch (_tagUpload.startTags(_controllerIdle(), info)) {
    case R::WrongState:
        _log.error("START_TAGS rejected: controller not idle");
        return {false, "Controller in incorrect state"};
    case R::InvalidCount:
        _log.error(formatString("START_TAGS rejected: tag_count %u exceeds max %u", info.tag_count, TagUploadCoordinator::kMaxTagCount));
        return {false, formatString("tag_count exceeds max %u", TagUploadCoordinator::kMaxTagCount)};
    default:
        break;
    }
    // The old list is gone; START_DETECTION must wait for END_TAGS.
    _detection.tagsUploaded(false);
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
    case R::InvalidCount:
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

std::string TunnelCommandDispatcher::startDetection(const StartDetectionInfo_t& info, bool checkBusy)
{
    using R = DetectionCoordinator::Result;
    switch (_detection.requestStart()) {
    case R::Accepted: {
        // HasTags is compatible with a running log save/delete or analysis.
        const std::string busy = checkBusy ? _progress.busyMessage() : std::string();
        if (!busy.empty()) {
            _detection.startFinished(false);
            _log.error("START_DETECTION rejected: " + busy);
            return busy;
        }
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
    case R::Capturing:
        _log.error("START_DETECTION rejected: raw capture in progress");
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

bool TunnelCommandDispatcher::stopDetection(std::string* error, uint32_t requestId)
{
    using R = DetectionCoordinator::Result;
    switch (_detection.requestStop()) {
    case R::Accepted:
        _actions.stopDetectionPipeline(requestId);
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

TunnelCommandDispatcher::Outcome TunnelCommandDispatcher::_stopDetection(uint32_t requestId)
{
    std::string error;
    const bool ok = stopDetection(&error, requestId);
    return {ok, error};
}
