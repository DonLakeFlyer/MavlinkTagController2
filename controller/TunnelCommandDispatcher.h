#pragma once

#include "DetectionCoordinator.h"
#include "OperationProgress.h"
#include "RequestCache.h"
#include "TagUploadCoordinator.h"
#include "TunnelProtocol.h"

#include <mavlink.h>

#include <string>

// Side effects the dispatcher asks its owner (CommandHandler) to perform.
// Everything that touches processes, files, the SDR or MAVLink lives behind
// this so the command/ack logic can run in a standalone test.
class CommandActions {
public:
    virtual ~CommandActions() = default;

    /// Launch the detection pipeline. Returns "" on success or an error
    /// message. On success the pipeline must later call
    /// detection().startFinished(true) from its worker.
    virtual std::string startDetectionPipeline(const TunnelProtocol::StartDetectionInfo_t& info) = 0;
    /// Tear the pipeline down; must later call detection().stopFinished().
    /// requestId is the STOP_DETECTION request, 0 for controller-initiated stops.
    virtual void        stopDetectionPipeline(uint32_t requestId) = 0;
    /// Payload of the START_DETECTION success ACK.
    virtual std::string detectionLogDir() = 0;

    virtual std::string rawCapture(const mavlink_tunnel_t& tunnel) = 0;
    /// "" on success or an error message for the NACK.
    virtual std::string saveLogs(uint32_t requestId) = 0;
    virtual std::string cleanLogs(uint32_t requestId) = 0;
    virtual std::string airspyStatus() = 0;
    virtual std::string startCollection(const mavlink_tunnel_t& tunnel) = 0;
    virtual std::string startCollectionSlice(const mavlink_tunnel_t& tunnel) = 0;
    virtual std::string finishCollection(const mavlink_tunnel_t& tunnel) = 0;
    /// Re-send the frames a completed FINISH pushed (candidate replays,
    /// BEARING_RESULTs, STOPPED); no state change.
    virtual void        replayFinishOutcome(uint32_t collectionId) = 0;
};

class CommandLog {
public:
    virtual ~CommandLog() = default;
    virtual void debug(const std::string& message) = 0;
    virtual void info(const std::string& message) = 0;
    virtual void error(const std::string& message) = 0;
};

// Decodes GCS tunnel commands, dedupes retries via RequestCache, drives the
// tag-upload and detection state machines, and returns the ACK to send.
// Collection commands are validated here and delegated whole; their state
// lives in CollectionCoordinator inside CommandHandler. Long-running commands
// (START/STOP_DETECTION, RAW_CAPTURE, SAVE/CLEAN_LOGS) are NACKed "Busy"
// while the OperationProgressReporter has one running.
class TunnelCommandDispatcher {
public:
    TunnelCommandDispatcher(CommandActions& actions, CommandLog& log, OperationProgressReporter& progress,
                            DetectionCoordinator::HeartbeatSink onHeartbeatStatus = {},
                            RequestCache requestCache = RequestCache());

    TunnelProtocol::AckInfo_t handle(const mavlink_tunnel_t& tunnel);

    /// START_DETECTION semantics without the tunnel framing; also used by the
    /// collection path, which already owns the progress reporter for the whole
    /// rotation and so passes checkBusy = false. Returns "" on success
    /// (including an in-flight start).
    std::string startDetection(const TunnelProtocol::StartDetectionInfo_t& info, bool checkBusy = true);
    /// Returns true if detection is stopping or already stopped as a result.
    /// requestId: the STOP_DETECTION request, 0 for controller-initiated stops.
    bool stopDetection(std::string* error = nullptr, uint32_t requestId = 0);

    DetectionCoordinator&       detection()          { return _detection; }
    const DetectionCoordinator& detection()    const { return _detection; }
    OperationProgressReporter&  progress()           { return _progress; }
    TagUploadCoordinator&       tagUpload()          { return _tagUpload; }
    const TagUploadCoordinator& tagUpload()    const { return _tagUpload; }
    const TagDatabase&          tags()         const { return _tagUpload.tags(); }
    RequestCache&               requestCache()       { return _requestCache; }

    static TunnelProtocol::AckInfo_t makeAck(uint32_t requestId, uint32_t command, uint32_t result,
                                             const std::string& message);
    static std::string commandName(uint32_t command);

private:
    struct Outcome {
        bool        success = false;
        std::string message;
    };

    Outcome _dispatch(const TunnelProtocol::HeaderInfo_t& header, const mavlink_tunnel_t& tunnel);
    void    _replayCollectionFrames(uint32_t command, const mavlink_tunnel_t& tunnel);
    Outcome _startTags(const mavlink_tunnel_t& tunnel);
    Outcome _tag(const mavlink_tunnel_t& tunnel);
    Outcome _endTags(const mavlink_tunnel_t& tunnel);
    Outcome _startDetection(const mavlink_tunnel_t& tunnel);
    Outcome _stopDetection(uint32_t requestId);
    bool    _controllerIdle();

    CommandActions&            _actions;
    CommandLog&                _log;
    OperationProgressReporter& _progress;
    RequestCache               _requestCache;
    TagUploadCoordinator       _tagUpload;
    DetectionCoordinator       _detection;
};
