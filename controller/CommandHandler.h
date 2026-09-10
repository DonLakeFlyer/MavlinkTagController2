#pragma once

#include "TunnelProtocol.h"
#include "TagDatabase.h"
#include "BearingCalculator.h"
#include "CollectionCoordinator.h"
#include "TelemetryCache.h"
#include "boost_process_compat.h"
#include "detector_protocol.h"

#include <atomic>
#include <condition_variable>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <mavlink.h>

class MavlinkSystem;
class MonitoredProcess;
class LogFileManager;

class CommandHandler {
public:
    explicit CommandHandler(MavlinkSystem* mavlink, TelemetryCache* telemetryCache, bool simulatorMode = false, const std::string& simulatorPreset = "strong", bool debugDetector = false, double simulatorSnrDb = 20.0,
                            double simulatorTxBearingDeg = 0.0, double simulatorInterfererSnrDb = std::numeric_limits<double>::quiet_NaN(),
                            const std::string& simulatorAntenna = "ra2a", double simulatorPriPpm = 43.0);

    static constexpr int kPulseUdpPort = 50000; // UDP port for pulse/heartbeat reports from detectors

    // UDP pulse struct sent by uavrt_detection over UDP.  Enough for MTU 1500 bytes.
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

    // uavrt_detection pulses: forwarded to the GCS as PulseInfo_t, never part of a collection.
    void handleUavrtPulse(const UDPPulseInfo_T& udpPulseInfo);
    void handlePythonDetectorMessage(const TagTrackerDetectorProtocol::Header& header,
                                     const TagTrackerDetectorProtocol::PulsePayload* pulsePayload,
                                     uint32_t errorCode = 0);

private:
    struct RotationSlice {
        uint32_t    slice_id;
        uint8_t     candidate_id;   // detector lock candidate this power was measured at
        float       heading_deg;
        bool        detected;       // false: armed heading, detector reported no pulse
        bool        sighted;        // detector's fold search found this candidate here (not just measured at it)
        double      snr_db;
        double      signal_power;
        double      noise_psd;
        uint8_t     confirmed_status;
        uint32_t    tag_id;
        double      latitude;
        double      longitude;
        double      altitude_rel;
        TunnelProtocol::PythonPulseInfo_t pulse_info;  // as built for the GCS; replayed if this candidate wins
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
    void _handlePythonPulse     (const TagTrackerDetectorProtocol::Header& header, const TagTrackerDetectorProtocol::PulsePayload& payload);
    void _sendPythonHeartbeat   (uint32_t tagId);
    void _startDetector         (LogFileManager* logFileManager, const TunnelProtocol::TagInfo_t& tagInfo, bool secondaryChannel);
    void _startPythonDetector   (LogFileManager* logFileManager, const TunnelProtocol::TagInfo_t& tagInfo, bool secondaryChannel, bool isHFMode, double detectionMargin, double confidenceRatio, bool debugDetector, bool dumpSpectrogram, int controlPort = 0);
    bool _writeSessionInfo      (const TunnelProtocol::StartDetectionInfo_t& startDetection, AirSpyDeviceType deviceType, bool isHFMode);
    std::string _handleStartCollection      (const mavlink_tunnel_t& tunnel);
    std::string _handleStartCollectionSlice (const mavlink_tunnel_t& tunnel);
    std::string _handleFinishCollection     (const mavlink_tunnel_t& tunnel);
    void _sendCollectionStatus(uint32_t collectionId, uint32_t sliceId, uint32_t status, uint32_t errorCode = 0,
                               std::optional<uint32_t> expectedDetectors = std::nullopt,
                               std::optional<uint32_t> completedDetectors = std::nullopt,
                               float revisitHeadingDeg = std::numeric_limits<float>::quiet_NaN());
    // Fits every (tag, candidate) from the given slices with the collection's antenna.
    BearingCalculator _bearingCalculatorFor(const std::vector<RotationSlice>& slices) const;
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
    const char*                     _homePath               = nullptr;
    std::vector<std::shared_ptr<MonitoredProcess>> _processes;
    bp::pipe*                       _airspyPipe             = nullptr;
    std::string                     _airspyPath;
    int                            _rawCaptureCount         = 0;
    std::atomic<int>                _analysisJobs           { 0 };      // post-flight analyzers still running
    std::atomic<bool>               _detectionStarting      { false };  // start worker has not yet published DETECTING
    std::atomic<bool>               _detectionStopping      { false };  // stop worker owns _processes teardown
    bool                            _simulatorMode          = false;
    std::string                     _simulatorPreset;
    double                          _simulatorSnrDb = 20.0;
    double                          _simulatorTxBearingDeg = 0.0;       // true bearing of the simulated transmitter from the first vehicle pose
    double                          _simulatorInterfererSnrDb;          // NaN: no interferer; else a flat (pattern-free) pulse train +1 kHz from the tag
    std::string                     _simulatorAntenna;                  // iq_simulator --antenna gain table
    double                          _simulatorPriPpm        = 43.0;     // iq_simulator --pri-ppm collar crystal offset
    bool                            _debugDetector          = false;
    uint32_t                        _simPhase               = 0;        // 4-phase cycle: 0=A, 1=A→B, 2=B, 3=B→A

    // Rotation detection state — accessed from both MAVLink and UDP threads
    std::mutex                                  _rotationMutex;
    std::condition_variable                     _collectionReady;
    CollectionCoordinator                       _collectionCoordinator;
    std::map<uint32_t, int>                     _detectorControlPorts;
    bool                                        _inRotation             = false;
    uint32_t                                    _antennaId              = 0;        // StartCollection_t::antenna_id
    bool                                        _revisitRequested       = false;    // one confirmation revisit per collection
    std::optional<float>                        _pendingRevisitHeadingDeg;          // requested but its slice not yet armed
    float                                       _currentHeadingDeg      = 0;
    std::map<uint32_t, float>                   _rotationSliceHeadings;
    std::map<uint32_t, TelemetryCache::TelemetryCacheEntry_t> _rotationSliceTelemetry;   // vehicle pose captured at ARM
    std::vector<RotationSlice>                  _rotationSlices;
    std::map<uint32_t, uint8_t>                 _liveCandidate;         // tag_id -> lock candidate the GCS is currently shown

    // Live view switches to another candidate only when its pattern fit is
    // clearly better, to avoid flip-flopping on a marginal rotation.
    static constexpr float    kLiveCandidateSwitchMargin = 0.15f;
    static constexpr uint32_t kLiveCandidateMinSlices    = 3;
    // A lock sighted independently on this many headings needs no revisit.
    static constexpr uint32_t kConfirmedSightings        = 2;
    // An ARM within this of the requested revisit heading satisfies the revisit;
    // well inside the 45 deg slice spacing, well outside the GCS heading-hold error.
    static constexpr float    kRevisitHeadingToleranceDeg = 15.0f;

    uint8_t _liveCandidateFor(uint32_t tagId) const;
    // Re-fits all candidates of tagId from _rotationSlices; if a different one
    // now wins, makes it live and returns its slice reports for replay to the
    // GCS. Caller holds _rotationMutex.
    std::vector<TunnelProtocol::PythonPulseInfo_t> _updateLiveCandidate(uint32_t tagId);

    static constexpr int kDetectorControlPortBase = 51000;

    static constexpr int kAirSpyHfFrequencyOffsetHz = 10000; // 10 kHz - takes into account 768 ksps incoming and 3840 Hz outgoing
    static constexpr double kSimulatorTxRangeM = 4000.0;     // simulated transmitter distance from the first vehicle pose
    static constexpr int kSimulatorInterfererOffsetHz = 1000; // inside the +/-2 kHz acquisition band, outside the 200 Hz lock tolerance
};
