#pragma once

#include "TunnelProtocol.h"
#include "TagDatabase.h"
#include "BearingCalculator.h"
#include "CollectionCoordinator.h"
#include "boost_process_compat.h"
#include "detector_protocol.h"

#include <atomic>
#include <condition_variable>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <mavlink.h>

class MavlinkSystem;
class MonitoredProcess;
class LogFileManager;
class TelemetryCache;

class CommandHandler {
public:
    explicit CommandHandler(MavlinkSystem* mavlink, TelemetryCache* telemetryCache, bool simulatorMode = false, const std::string& simulatorPreset = "strong", bool debugDetector = false);

    static constexpr int kPulseUdpPort = 50000; // UDP port for pulse/heartbeat reports from detectors

    // UDP pulse struct sent by pulse_detector.py over UDP.  Enough for MTU 1500 bytes.
    typedef struct {
        double tag_id;
        double frequency_hz;
        double start_time_seconds;
        double predict_next_start_seconds;
        double snr;
        double stft_score;
        double group_seq_counter;
        double group_ind;
        double group_snr;
        double detection_status;
        double confirmed_status;
        double noise_psd;
    } UDPPulseInfo_T;

    void handlePulse(const UDPPulseInfo_T& udpPulseInfo, uint32_t collectionId = 0, uint32_t sliceId = 0);
    void handlePythonDetectorMessage(const TagTrackerDetectorProtocol::Header& header,
                                     const TagTrackerDetectorProtocol::PulsePayload* pulsePayload,
                                     uint32_t errorCode = 0);

private:
    struct RotationSlice {
        float       heading_deg;
        double      snr_db;
        double      noise_psd;
        uint8_t     confirmed_status;
        uint32_t    tag_id;
        double      latitude;
        double      longitude;
        double      altitude_rel;
    };
    enum class AirSpyDeviceType {
        NONE,
        MINI,
        HF,
        SIMULATOR
    };


    void _sendCommandAck        (uint32_t command, uint32_t result, std::string& ackMessage);
    bool _handleStartTags       (const mavlink_tunnel_t& tunnel);
    bool _handleEndTags         (void);
    bool _handleTag             (const mavlink_tunnel_t& tunnel);
    std::string _handleStartDetection  (const mavlink_tunnel_t& tunnel);
    bool _handleStopDetection   (bool waitForCompletion = false);
    std::string _handleRawCapture      (const mavlink_tunnel_t& tunnel);
    bool _handleSaveLogs        (void);
    bool _handleCleanLogs       (void);
    void _handleTunnelMessage   (const mavlink_message_t& message);
    void _startDetector         (LogFileManager* logFileManager, const TunnelProtocol::TagInfo_t& tagInfo, bool secondaryChannel);
    void _startPythonDetector   (LogFileManager* logFileManager, const TunnelProtocol::TagInfo_t& tagInfo, bool secondaryChannel, bool isHFMode, double detectionMargin, double confidenceRatio, bool debugDetector, bool dumpSpectrogram, int controlPort = 0);
    bool _writeSessionInfo      (const TunnelProtocol::StartDetectionInfo_t& startDetection, AirSpyDeviceType deviceType, bool isHFMode);
    std::string _handleStartCollection      (const mavlink_tunnel_t& tunnel);
    std::string _handleStartCollectionSlice (const mavlink_tunnel_t& tunnel);
    std::string _handleFinishCollection     (const mavlink_tunnel_t& tunnel);
    void _sendCollectionStatus(uint32_t collectionId, uint32_t sliceId, uint32_t status, uint32_t errorCode = 0,
                               std::optional<uint32_t> expectedDetectors = std::nullopt,
                               std::optional<uint32_t> completedDetectors = std::nullopt);
    bool _sendDetectorControl(uint32_t tagId, const TagTrackerDetectorProtocol::ArmMessage& message);
        void _handleDetectorProcessFailure(uint32_t tagId, int exitCode);
    void _runPostFlightAnalysis (const std::string& logDir);
    AirSpyDeviceType _connectedAirSpyType(std::string* errorMessage = nullptr);
    std::string _sdrPathStatusText(AirSpyDeviceType deviceType, double frequencyMhz) const;
    std::string _checkForAirSpy  (void);

    std::string _tunnelCommandIdToString    (uint32_t command);
    std::string _tunnelCommandResultToString(uint32_t result);

    std::string _simulatorCommand(uint32_t radioCenterFrequencyHz);

    MavlinkSystem*                  _mavlink                = nullptr;
    TelemetryCache*                 _telemetryCache         = nullptr;
    TagDatabase                     _tagDatabase;
    bool                            _receivingTags          = false;
    char*                           _homePath               = nullptr;
    std::vector<std::shared_ptr<MonitoredProcess>> _processes;
    bp::pipe*                       _airspyPipe             = nullptr;
    std::string                     _airspyPath;
    int                            _rawCaptureCount         = 0;
    std::atomic<int>                _analysisJobs           { 0 };      // post-flight analyzers still running
    std::atomic<bool>               _detectionStarting      { false };  // start worker has not yet published DETECTING
    std::atomic<bool>               _detectionStopping      { false };  // stop worker owns _processes teardown
    bool                            _simulatorMode          = false;
    std::string                     _simulatorPreset;
    bool                            _debugDetector          = false;
    uint32_t                        _simPhase               = 0;        // 4-phase cycle: 0=A, 1=A→B, 2=B, 3=B→A

    // Rotation detection state — accessed from both MAVLink and UDP threads
    std::mutex                                  _rotationMutex;
    std::condition_variable                     _collectionReady;
    CollectionCoordinator                       _collectionCoordinator;
    std::map<uint32_t, int>                     _detectorControlPorts;
    bool                                        _inRotation             = false;
    float                                       _currentHeadingDeg      = 0;
    std::vector<RotationSlice>                  _rotationSlices;

    static constexpr int kDetectorControlPortBase = 51000;

    static constexpr float kBackSectorMinDeg = 165.0f;   // simulator back-sector hack threshold

    static constexpr int kAirSpyHfFrequencyOffsetHz = 10000; // 10 kHz - takes into account 768 ksps incoming and 3840 Hz outgoing
};
