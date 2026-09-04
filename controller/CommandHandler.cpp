#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <future>
#include <map>
#include <memory>
#include <thread>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <spawn.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/wait.h>

#include <mavlink.h>

#include <stdio.h>
#include <filesystem>
#include <fstream>
#include <sstream>
#include "CommandHandler.h"
#include "TunnelProtocol.h"
#include "MonitoredProcess.h"
#include "formatString.h"
#include "platformHelpers.h"
#include "log.h"
#include "channelizerTuner.h"
#include "MavlinkSystem.h"
#include "LogFileManager.h"
#include "TelemetryCache.h"
#include "BearingCalculator.h"

// POSIX requires the application to declare this; glibc only does so under _GNU_SOURCE.
extern char **environ;

using namespace TunnelProtocol;

CommandHandler::CommandHandler(MavlinkSystem* mavlink, TelemetryCache* telemetryCache, bool simulatorMode, const std::string& simulatorPreset, bool debugDetector)
    : _mavlink          (mavlink)
    , _telemetryCache   (telemetryCache)
    , _homePath         (getenv("HOME"))
    , _simulatorMode    (simulatorMode)
    , _simulatorPreset  (simulatorPreset)
    , _debugDetector    (debugDetector)
{
    if (isRunningOnRPi()) {
        // When we are running from a crontab entry on the rPi the PATH environment variable is not fully set yet.
        // Because of this, the process fails to find the airspy_rx executable. So we need to explicitly specify
        // where the airspy executables are located.
        logInfo() << "CommandHandler::CommandHandler - Running on rPi. Setting airspy path to /usr/local/bin/";
        _airspyPath = "/usr/bin/";
    }

    using namespace std::placeholders;
    _mavlink->subscribeToMessage(MAVLINK_MSG_ID_TUNNEL, std::bind(&CommandHandler::_handleTunnelMessage, this, _1));
}

void CommandHandler::_sendCommandAck(uint32_t command, uint32_t result, std::string& ackMessage)
{
    AckInfo_t ackInfo;

    logDebug() << "_sendCommandAck command:result" << _tunnelCommandIdToString(command) << _tunnelCommandResultToString(result);

    memset(&ackInfo, 0, sizeof(ackInfo));
    ackInfo.header.command  = COMMAND_ID_ACK;
    ackInfo.command         = command;
    ackInfo.result          = result;
    strncpy(ackInfo.message, ackMessage.c_str(), sizeof(ackInfo.message) - 1);

    _mavlink->sendTunnelMessage(&ackInfo, sizeof(ackInfo));
}

bool CommandHandler::_handleStartTags(const mavlink_tunnel_t& tunnel)
{
    StartTagsInfo_t startTagsInfo;

    if (tunnel.payload_length != sizeof(StartTagsInfo_t)) {
        logError() << "CommandHandler::_handleStartTags ERROR - Payload length incorrect expected:actual" << sizeof(StartTagsInfo_t) << tunnel.payload_length;
        return false;
    }

    if (_mavlink->heartbeatStatus() != HEARTBEAT_STATUS_IDLE && _mavlink->heartbeatStatus() != HEARTBEAT_STATUS_HAS_TAGS) {
        logError() << "CommandHandler::_handleStartTags ERROR - Controller in incorrect state for start tags - heartbeatStatus:" << _mavlink->heartbeatStatus();
        return false;
    }

    memcpy(&startTagsInfo, tunnel.payload, sizeof(startTagsInfo));

    logDebug() << "_handleStartTags";

    _tagDatabase.clear();
    _receivingTags = true;

    return true;
}

bool CommandHandler::_handleTag(const mavlink_tunnel_t& tunnel)
{
    TagInfo_t tagInfo;

    if (tunnel.payload_length != sizeof(tagInfo)) {
        logError() << "CommandHandler::_handleTagCommand ERROR - Payload length incorrect expected:actual" << sizeof(tagInfo) << tunnel.payload_length;
        return false;
    }

    memcpy(&tagInfo, tunnel.payload, sizeof(tagInfo));

    if (tagInfo.id < 2) {
        logError() << "CommandHandler::_handleTagCommand: invalid tag id of 0/1";
        return false;
    }

    logDebug() << "CommandHandler::handleTagCommand: id:freq:intra_pulse1_msecs "
                << tagInfo.id
                << tagInfo.frequency_hz
                << tagInfo.intra_pulse1_msecs;

    _tagDatabase.push_back(tagInfo);

    return true;
}

bool CommandHandler::_handleEndTags(void)
{
    logDebug() << "_handleEndTags _receivingTags" << _receivingTags;

    if (!_receivingTags) {
        return false;
    }

    _receivingTags = false;
    if (_tagDatabase.size() != 0) {
        _mavlink->setHeartbeatStatus(HEARTBEAT_STATUS_HAS_TAGS);
    }

    return true;
}

void CommandHandler::_startDetector(LogFileManager* logFileManager, const TunnelProtocol::TagInfo_t& tagInfo, bool secondaryChannel)
{
    std::string commandStr  = formatString("%s/repos/uavrt_detection/uavrt_detection %s",
                                _homePath,
                                _tagDatabase.detectorConfigFileName(tagInfo, secondaryChannel).c_str());
    std::string logStem     = formatString("detector_%d", tagInfo.id + (secondaryChannel ? 1 : 0));
    std::string logPath     = logFileManager->filename(LogFileManager::DETECTORS, logStem.c_str(), "log");

    auto detectorProc = std::make_shared<MonitoredProcess>(
                                                _mavlink,
                                                "uavrt_detection",
                                                commandStr.c_str(),
                                                logPath.c_str(),
                                                MonitoredProcess::NoPipe,
                                                nullptr);
    detectorProc->start();
    _processes.push_back(detectorProc);
}

void CommandHandler::_startPythonDetector(LogFileManager* logFileManager, const TunnelProtocol::TagInfo_t& tagInfo, bool secondaryChannel, bool isHFMode, double detectionMargin, double confidenceRatio, bool debugDetector, bool dumpSpectrogram, int controlPort)
{
    int     secondaryChannelIncrement   = secondaryChannel ? 1 : 0;
    int     tagId                       = tagInfo.id + secondaryChannelIncrement;
    int     portData                    = isHFMode ? (10000 + secondaryChannelIncrement) : (20000 + ((tagInfo.channelizer_channel_number - 1) * 2) + secondaryChannelIncrement);
    int     sampleRate                  = isHFMode ? 3840 : 3750;
    double  tip                         = tagInfo.intra_pulse1_msecs / 1000.0;
    double  tp                          = tagInfo.pulse_width_msecs / 1000.0;
    double  centerFreqMhz              = double(tagInfo.channelizer_channel_center_frequency_hz) / 1000000.0;

    // Validate K (fold count): must be >= 2 for meaningful integration.
    // Default to 5 if the GCS sends 0 or 1 (unset or invalid).
    uint32_t k = tagInfo.k >= 2 ? tagInfo.k : 5;
    if (tagInfo.k < 2) {
        logWarn() << "Tag " << tagId << " has invalid k=" << tagInfo.k << ", defaulting to 5";
    }

    std::string repoDir     = formatString("%s/repos/MavlinkTagController2", _homePath);
    std::string venvPython  = formatString("%s/.venv/bin/python3", repoDir.c_str());
    std::string pythonCmd   = (access(venvPython.c_str(), X_OK) == 0) ? venvPython : std::string("python3");
    std::string cacheDir    = std::string(_homePath);

    std::string commandStr  = formatString("\"%s\" -u \"%s/detector/pulse_detector.py\""
                                           " --tp %f --tip %f --fs %d --port %d"
                                           " --tag-id %d --freq %u --pulse-port %d"
                                           " --center-freq %f --pf %f"
                                           " --detection-margin %f --confidence-ratio %f"
                                           " --threshold-cache-dir \"%s\""
                                           " --k %u",
                                pythonCmd.c_str(),
                                repoDir.c_str(),
                                tp, tip, sampleRate, portData,
                                tagId, tagInfo.frequency_hz, kPulseUdpPort,
                                centerFreqMhz, tagInfo.false_alarm_probability,
                                detectionMargin, confidenceRatio,
                                cacheDir.c_str(),
                                k);
    if (_debugDetector || debugDetector) {
        commandStr += " --debug";
    }
    if (controlPort > 0) {
        commandStr += formatString(" --control-port %d", controlPort);
    }

    // Always pass the log directory so the detector can write to it
    std::string detectorLogDir = logFileManager->logDir(LogFileManager::DETECTORS);
    if (!detectorLogDir.empty()) {
        commandStr += formatString(" --log-dir \"%s\"", detectorLogDir.c_str());
    }

    if (dumpSpectrogram && !detectorLogDir.empty()) {
        commandStr += " --dump-spectrogram";
    }

    // If the tag has a secondary PRI, pass it as --tip-secondary so the
    // detector runs multi-hypothesis rate-switch detection in a single process
    // instead of requiring a separate detector process for the second rate.
    if (tagInfo.intra_pulse2_msecs != 0) {
        double tipSecondary = tagInfo.intra_pulse2_msecs / 1000.0;
        commandStr += formatString(" --tip-secondary %f", tipSecondary);
    }

    std::string logStem = formatString("py_detector_%d", tagId);
    std::string logPath = logFileManager->filename(LogFileManager::DETECTORS, logStem.c_str(), "log");

    logInfo() << "Starting Python pulse detector:" << commandStr;

    std::string procName = formatString("pulse_detector_%d", tagId);
    auto detectorProc = std::make_shared<MonitoredProcess>(
                                                _mavlink,
                                                procName.c_str(),
                                                commandStr.c_str(),
                                                logPath.c_str(),
                                                MonitoredProcess::NoPipe,
                                                nullptr,
                                                false,
                                                [this, tagId](int exitCode) {
                                                    _handleDetectorProcessFailure(tagId, exitCode);
                                                });
    detectorProc->start();
    _processes.push_back(detectorProc);
}

// session.json: the GCS request as received plus the rates/thresholds the
// detectors were actually launched with. Consumed by analyzer/flight_checks.py.
bool CommandHandler::_writeSessionInfo(const StartDetectionInfo_t& startDetection, AirSpyDeviceType deviceType, bool isHFMode)
{
    auto logFileManager = LogFileManager::instance();
    std::string path = logFileManager->filename(LogFileManager::DETECTORS, "session", "json");

    FILE* fp = fopen(path.c_str(), "w");
    if (fp == NULL) {
        logError() << "_writeSessionInfo fopen failed" << path << "-" << strerror(errno);
        return false;
    }

    auto jsonNumber = [](double value) {
        return std::isfinite(value) ? formatString("%.15g", value) : std::string("null");
    };
    const char* sdr = deviceType == AirSpyDeviceType::HF ? "airspyhf"
                    : deviceType == AirSpyDeviceType::MINI ? "airspy_mini"
                    : deviceType == AirSpyDeviceType::SIMULATOR ? "simulator" : "none";
    const int sampleRate = isHFMode ? 3840 : 3750;
    const double detectionMargin = startDetection.detection_margin > 0 ? startDetection.detection_margin : 0.90;
    const double confidenceRatio = startDetection.confidence_ratio > 0 ? startDetection.confidence_ratio : 1.3;

    fprintf(fp, "{\n");
    fprintf(fp, "  \"detection_mode\": \"python\",\n");
    fprintf(fp, "  \"sdr\": \"%s\",\n", sdr);
    fprintf(fp, "  \"radio_center_frequency_hz\": %u,\n", startDetection.radio_center_frequency_hz);
    fprintf(fp, "  \"gain\": %u,\n", startDetection.gain);
    fprintf(fp, "  \"detector_sample_rate_sps\": %d,\n", sampleRate);
    fprintf(fp, "  \"detection_margin_requested\": %s,\n", jsonNumber(startDetection.detection_margin).c_str());
    fprintf(fp, "  \"detection_margin\": %s,\n", jsonNumber(detectionMargin).c_str());
    fprintf(fp, "  \"confidence_ratio_requested\": %s,\n", jsonNumber(startDetection.confidence_ratio).c_str());
    fprintf(fp, "  \"confidence_ratio\": %s,\n", jsonNumber(confidenceRatio).c_str());
    fprintf(fp, "  \"debug_detector\": %s,\n", startDetection.debug_detector ? "true" : "false");
    fprintf(fp, "  \"dump_spectrogram\": %s,\n", startDetection.dump_spectrogram ? "true" : "false");
    fprintf(fp, "  \"tags\": [\n");

    bool first = true;
    for (const TagInfo_t& tag : _tagDatabase) {
        const uint32_t kEffective = tag.k >= 2 ? tag.k : 5; // mirrors _startPythonDetector
        fprintf(fp, "%s    {\n", first ? "" : ",\n");
        first = false;
        fprintf(fp, "      \"id\": %u,\n", tag.id);
        fprintf(fp, "      \"frequency_hz\": %u,\n", tag.frequency_hz);
        fprintf(fp, "      \"pulse_width_msecs\": %u,\n", tag.pulse_width_msecs);
        fprintf(fp, "      \"intra_pulse1_msecs\": %u,\n", tag.intra_pulse1_msecs);
        fprintf(fp, "      \"intra_pulse2_msecs\": %u,\n", tag.intra_pulse2_msecs);
        fprintf(fp, "      \"intra_pulse_uncertainty_msecs\": %u,\n", tag.intra_pulse_uncertainty_msecs);
        fprintf(fp, "      \"intra_pulse_jitter_msecs\": %u,\n", tag.intra_pulse_jitter_msecs);
        fprintf(fp, "      \"k_requested\": %u,\n", tag.k);
        fprintf(fp, "      \"k\": %u,\n", kEffective);
        fprintf(fp, "      \"false_alarm_probability\": %s,\n", jsonNumber(tag.false_alarm_probability).c_str());
        fprintf(fp, "      \"channelizer_channel_number\": %u,\n", tag.channelizer_channel_number);
        fprintf(fp, "      \"channelizer_channel_center_frequency_hz\": %u,\n", tag.channelizer_channel_center_frequency_hz);
        fprintf(fp, "      \"ip1_mu\": %s,\n", jsonNumber(tag.ip1_mu).c_str());
        fprintf(fp, "      \"ip1_sigma\": %s,\n", jsonNumber(tag.ip1_sigma).c_str());
        fprintf(fp, "      \"ip2_mu\": %s,\n", jsonNumber(tag.ip2_mu).c_str());
        fprintf(fp, "      \"ip2_sigma\": %s\n", jsonNumber(tag.ip2_sigma).c_str());
        fprintf(fp, "    }");
    }
    fprintf(fp, "\n  ]\n}\n");
    const bool writeError = ferror(fp) != 0;
    const bool closeError = fclose(fp) != 0;
    if (writeError || closeError) {
        logError() << "_writeSessionInfo write failed" << path << "-" << strerror(errno);
        return false;
    }

    std::ifstream in(path);
    std::stringstream contents;
    contents << in.rdbuf();
    logInfo() << "SESSION INFO:" << path;
    logInfo() << contents.str();
    return true;
}

std::string CommandHandler::_handleStartDetection(const mavlink_tunnel_t& tunnel)
{
   if (tunnel.payload_length != sizeof(StartDetectionInfo_t)) {
        logError() << "COMMAND_ID_START_DETECTION - ERROR: Payload length incorrect expected:actual" << sizeof(StartDetectionInfo_t) << tunnel.payload_length;
        return "Payload length incorrect";
    }

    if (_mavlink->heartbeatStatus() != HEARTBEAT_STATUS_HAS_TAGS) {
        logError() << "COMMAND_ID_START_DETECTION - ERROR: Start detection failed. Controller in incorrect state - heartbeatStatus" << _mavlink->heartbeatStatus();
        return "Controller in incorrect state";
    }

    // Heartbeat stays HAS_TAGS until the worker finishes, so this is the only
    // thing preventing a second START from launching a duplicate process set.
    if (_detectionStarting.exchange(true)) {
        logError() << "COMMAND_ID_START_DETECTION - ERROR: Detection start already in progress";
        return "Detection start already in progress";
    }

    std::string airspyError;
    auto deviceType = _simulatorMode ? AirSpyDeviceType::SIMULATOR : _connectedAirSpyType(&airspyError);
    if (deviceType == AirSpyDeviceType::NONE) {
        logError() << "COMMAND_ID_START_DETECTION - ERROR: AirSpy detection failed: " << airspyError;
        _detectionStarting.store(false);
        return std::string("AirSpy detection failed: ") + airspyError;
    }

    auto logFileManager = LogFileManager::instance();
    // Freeze before the log dir is named and before DETECTING is published, so a
    // concurrent stop cannot unfreeze and then be re-frozen by this start.
    _mavlink->setVehicleTimeFrozen(true);
    logFileManager->detectorsStarted();
    bool isHFMode = (deviceType == AirSpyDeviceType::HF || deviceType == AirSpyDeviceType::SIMULATOR);
    StartDetectionInfo_t requestedStart {};
    memcpy(&requestedStart, tunnel.payload, sizeof(requestedStart));
    if (requestedStart.detection_mode == DETECTION_MODE_PYTHON) {
        // The Python detector is configured entirely via CLI; the legacy .config
        // files are only read by uavrt_detection. Record the request instead.
        if (!_writeSessionInfo(requestedStart, deviceType, isHFMode)) {
            _mavlink->sendStatusText("Write session info failed", MAV_SEVERITY_ALERT);
        }
    } else if (!_tagDatabase.writeDetectorConfigs(isHFMode)) {
        logError() << "CommandHandler::_handleEndTags: writeDetectorConfigs failed";
        _mavlink->sendStatusText("Write Detector Configs failed", MAV_SEVERITY_ALERT);
    }

    std::thread([this, tunnel, logFileManager, deviceType]() {
        StartDetectionInfo_t    startDetection;
        std::string             commandStr;
        std::string             logPath;
        std::string             airspyChannelizeDir;
        std::string             airspyChannelizeProcessName;
        std::string             airspyChannelizeExecutable;
        std::string             airspyReceiverProcessName;

        memcpy(&startDetection, tunnel.payload, sizeof(startDetection));

        logInfo() << "COMMAND_ID_START_DETECTION:";
        logInfo() << "\tradio_center_frequency_hz:" << startDetection.radio_center_frequency_hz;
        logInfo() << "\tdetection_margin:" << startDetection.detection_margin << " confidence_ratio:" << startDetection.confidence_ratio;
        logInfo() << "\tdebug_detector:" << startDetection.debug_detector;
        const double centerFrequencyMhz = (double)startDetection.radio_center_frequency_hz / 1000000.0;

        std::string sdrPathStatus;
        if (deviceType == AirSpyDeviceType::SIMULATOR) {
                // Simulator Pipeline: iq_simulator.py --(ZMQ)--> airspyhf_decimator -> UDP 10000/10001 -> detectors
                // The simulator generates IQ at 768 kHz on ZMQ PUB, identical to airspyhf_zeromq_rx.
                // Tags are generated at DC (freq_offset_hz=0), so the decimator uses --shift-khz 0.
                sdrPathStatus = _sdrPathStatusText(deviceType, centerFrequencyMhz);

                commandStr = _simulatorCommand(startDetection.radio_center_frequency_hz);
                logInfo() << "COMMAND_ID_START_DETECTION - using IQ Simulator stream source";
                logInfo() << "  command:" << commandStr;

                _mavlink->sendStatusText(sdrPathStatus.c_str(), MAV_SEVERITY_INFO);

                logPath = logFileManager->filename(LogFileManager::DETECTORS, "iq_simulator", "log");
                auto simProc = std::make_shared<MonitoredProcess>(
                                                        _mavlink,
                                                        "iq_simulator",
                                                        commandStr.c_str(),
                                                        logPath.c_str(),
                                                        MonitoredProcess::NoPipe,
                                                        nullptr);
                simProc->start();
                _processes.push_back(simProc);

                // Start airspyhf_decimator (subscribes to ZMQ, decimates by 200 to get 3840 Hz)
                // --shift-khz 0 because the simulator generates tags at DC (no hardware DC spur to avoid)
                commandStr = formatString("%s/repos/MavlinkTagController2/build/decimator/airspyhf_decimator --input-rate 768000 --shift-khz 0 --ports 10000,10001",
                                    _homePath);
                logPath = logFileManager->filename(LogFileManager::DETECTORS, "airspyhf_decimator", "log");
                auto decimatorProc = std::make_shared<MonitoredProcess>(
                                                        _mavlink,
                                                        "airspyhf_decimator",
                                                        commandStr.c_str(),
                                                        logPath.c_str(),
                                                        MonitoredProcess::NoPipe,
                                                        nullptr);
                decimatorProc->start();
                _processes.push_back(decimatorProc);
            } else if (deviceType == AirSpyDeviceType::HF) {
                // HF Pipeline: airspyhf_zeromq_rx --(ZMQ)--> airspyhf_decimator -> UDP 10000/10001 -> detectors
                airspyReceiverProcessName = "airspyhf_zeromq_rx";
                sdrPathStatus = _sdrPathStatusText(deviceType, centerFrequencyMhz);

                // Tune 10 kHz above the requested center to keep the signal away from the AirSpy HF DC bin.
                // Then apply +10 kHz digital shift so the target is re-centered while the hardware DC spike moves to +10 kHz.
                // The -Z flag enables ZeroMQ PUB output on tcp://127.0.0.1:5555 (default).
                commandStr = formatString("%s/repos/MavlinkTagController2/build/airspyhf_zeromq/tools/src/airspyhf_zeromq_rx -Z -f %f -a 768000 -g off -m on",
                                    _homePath,
                                    centerFrequencyMhz + (static_cast<double>(kAirSpyHfFrequencyOffsetHz) / 1000000.0));
                logInfo() << "COMMAND_ID_START_DETECTION - using AirSpy HF ZeroMQ stream source";

                _mavlink->sendStatusText(sdrPathStatus.c_str(), MAV_SEVERITY_INFO);

                logPath     = logFileManager->filename(LogFileManager::DETECTORS, airspyReceiverProcessName.c_str(), "log");
                auto airspyProc = std::make_shared<MonitoredProcess>(
                                                        _mavlink,
                                                        airspyReceiverProcessName.c_str(),
                                                        commandStr.c_str(),
                                                        logPath.c_str(),
                                                        MonitoredProcess::NoPipe,
                                                        nullptr);
                airspyProc->start();
                _processes.push_back(airspyProc);

                // Start airspyhf_decimator (subscribes to ZMQ, decimates by 200 to get 3840 Hz)
                // --shift-khz 10 compensates for the +10 kHz receiver tune offset with this shifter convention.
                const int hfFrequencyShiftKhz = (kAirSpyHfFrequencyOffsetHz / 1000);
                commandStr = formatString("%s/repos/MavlinkTagController2/build/decimator/airspyhf_decimator --input-rate 768000 --shift-khz %d --ports 10000,10001",
                                    _homePath,
                                    hfFrequencyShiftKhz);
                logPath = logFileManager->filename(LogFileManager::DETECTORS, "airspyhf_decimator", "log");
                auto decimatorProc = std::make_shared<MonitoredProcess>(
                                                        _mavlink,
                                                        "airspyhf_decimator",
                                                        commandStr.c_str(),
                                                        logPath.c_str(),
                                                        MonitoredProcess::NoPipe,
                                                        nullptr);
                decimatorProc->start();
                _processes.push_back(decimatorProc);
            } else {
                _airspyPipe = new bp::pipe();
                // Mini Pipeline: airspy_rx -> csdr-uavrt -> channelizer -> UDP 20000+ -> detectors
                airspyChannelizeDir = "airspy_channelize";
                airspyChannelizeProcessName = "airspy_channelize";
                airspyChannelizeExecutable = "airspy_channelize";
                airspyReceiverProcessName = "airspy_rx";
                sdrPathStatus = _sdrPathStatusText(deviceType, centerFrequencyMhz);

                commandStr  = formatString("%sairspy_rx -f %f -a 3000000 -r /dev/stdout -h %d -t 0",
                                    _airspyPath.c_str(),
                                    centerFrequencyMhz,
                                    startDetection.gain);
                logInfo() << "COMMAND_ID_START_DETECTION - using AirSpy Mini stream source";

                _mavlink->sendStatusText(sdrPathStatus.c_str(), MAV_SEVERITY_INFO);

                logPath     = logFileManager->filename(LogFileManager::DETECTORS, airspyReceiverProcessName.c_str(), "log");
                auto airspyProc = std::make_shared<MonitoredProcess>(
                                                        _mavlink,
                                                        airspyReceiverProcessName.c_str(),
                                                        commandStr.c_str(),
                                                        logPath.c_str(),
                                                        MonitoredProcess::OutputPipe,
                                                        _airspyPipe);
                airspyProc->start();
                _processes.push_back(airspyProc);

                logPath = logFileManager->filename(LogFileManager::DETECTORS, "csdr-uavrt", "log");
                auto csdrProc = std::make_shared<MonitoredProcess>(
                                                        _mavlink,
                                                        "csdr-uavrt",
                                                        "csdr-uavrt fir_decimate_cc 8 0.05 HAMMING",
                                                        logPath.c_str(),
                                                        MonitoredProcess::InputPipe,
                                                        _airspyPipe);
                csdrProc->start();
                _processes.push_back(csdrProc);

                commandStr  = formatString("%s/repos/%s/%s %s",
                                    _homePath,
                                    airspyChannelizeDir.c_str(),
                                    airspyChannelizeExecutable.c_str(),
                                    _tagDatabase.channelizerCommandLine().c_str());
                logPath = logFileManager->filename(LogFileManager::DETECTORS, airspyChannelizeProcessName.c_str(), "log");
                auto channelizeProc = std::make_shared<MonitoredProcess>(
                                                            _mavlink,
                                                            airspyChannelizeProcessName.c_str(),
                                                            commandStr.c_str(),
                                                            logPath.c_str(),
                                                            MonitoredProcess::NoPipe,
                                                            nullptr);
                channelizeProc->start();
                _processes.push_back(channelizeProc);
            }

        bool isHFMode = (deviceType == AirSpyDeviceType::HF || deviceType == AirSpyDeviceType::SIMULATOR);

        // Python detector thresholds — use defaults if not set (0 means use default)
        double detectionMargin = startDetection.detection_margin > 0 ? startDetection.detection_margin : 0.90;
        double confidenceRatio = startDetection.confidence_ratio > 0 ? startDetection.confidence_ratio : 1.3;
        if (startDetection.detection_margin < 0) {
            logError() << "Negative detection_margin (" << startDetection.detection_margin << "), using default:" << detectionMargin;
        }
        if (startDetection.confidence_ratio < 0) {
            logError() << "Negative confidence_ratio (" << startDetection.confidence_ratio << "), using default:" << confidenceRatio;
        }
        if (startDetection.detection_mode == DETECTION_MODE_PYTHON) {
            logInfo() << "Python detector thresholds: detectionMargin:" << detectionMargin << " confidenceRatio:" << confidenceRatio;
        }

        _mavlink->setDetectionMode(startDetection.detection_mode);

        for (const TunnelProtocol::TagInfo_t& tagInfo: _tagDatabase) {
            if (startDetection.detection_mode == DETECTION_MODE_PYTHON) {
                // Python detector handles both rates in a single process via
                // --tip-secondary, so only launch once per tag.
                bool debugDet = (startDetection.debug_detector != 0);
                bool dumpSpec = (startDetection.dump_spectrogram != 0);
                int controlPort = 0;
                {
                    std::lock_guard<std::mutex> lock(_rotationMutex);
                    const auto it = _detectorControlPorts.find(tagInfo.id);
                    if (it != _detectorControlPorts.end()) {
                        controlPort = it->second;
                    }
                }
                _startPythonDetector(logFileManager, tagInfo, false /* secondaryChannel */, isHFMode, detectionMargin, confidenceRatio, debugDet, dumpSpec, controlPort);
            } else {
                _startDetector(logFileManager, tagInfo, false /* secondaryChannel */);
                if (tagInfo.intra_pulse2_msecs != 0) {
                    _startDetector(logFileManager, tagInfo, true /* secondaryChannel */);
                }
            }
        }

        std::string startedStr = formatString("All processes started at center hz: %.3f", (double)startDetection.radio_center_frequency_hz / 1000000.0);
        _mavlink->sendStatusText(startedStr.c_str(), MAV_SEVERITY_INFO);

        _mavlink->setHeartbeatStatus(HEARTBEAT_STATUS_DETECTING);
        _detectionStarting.store(false);
    }).detach();

    return ""; // Return empty string to indicate success
}

void CommandHandler::_runPostFlightAnalysis(const std::string& logDir)
{
    std::string repoDir    = formatString("%s/repos/MavlinkTagController2", _homePath);
    std::string venvPython = formatString("%s/.venv/bin/python3", repoDir.c_str());
    std::string pythonCmd  = (access(venvPython.c_str(), X_OK) == 0) ? venvPython : std::string("python3");
    std::string script     = formatString("%s/analyzer/post_flight_analysis.py", repoDir.c_str());

    if (access(script.c_str(), R_OK) != 0) {
        logWarn() << "Post-flight analysis script not found:" << script;
        return;
    }

    logInfo() << "Running post-flight analysis:" << pythonCmd << "-u" << script << logDir;

    // posix_spawnp with an argv array: no shell, and exec failures come back as errno.
    // _analysisJobs keeps SAVE/CLEAN_LOGS from touching the directory mid-run.
    _analysisJobs.fetch_add(1);
    std::thread([this, pythonCmd, script, logDir]() {
        struct JobGuard {
            std::atomic<int>& n;
            ~JobGuard() { n.fetch_sub(1); }
        } guard{_analysisJobs};

        char* const argv[] = {
            const_cast<char*>(pythonCmd.c_str()),
            const_cast<char*>("-u"),
            const_cast<char*>(script.c_str()),
            const_cast<char*>(logDir.c_str()),
            nullptr
        };

        pid_t pid = 0;
        int spawnErr = posix_spawnp(&pid, pythonCmd.c_str(), nullptr, nullptr, argv, environ);
        if (spawnErr != 0) {
            logWarn() << "Post-flight analysis: failed to launch" << pythonCmd << ":" << strerror(spawnErr);
            return;
        }

        int rc = 0;
        if (waitpid(pid, &rc, 0) == -1) {
            logWarn() << "Post-flight analysis: waitpid() failed:" << strerror(errno);
        } else if (WIFEXITED(rc)) {
            int exitCode = WEXITSTATUS(rc);
            if (exitCode == 0) {
                logInfo() << "Post-flight analysis completed successfully";
            } else {
                logWarn() << "Post-flight analysis failed with exit code" << exitCode;
            }
        } else if (WIFSIGNALED(rc)) {
            logWarn() << "Post-flight analysis killed by signal" << WTERMSIG(rc);
        } else {
            logWarn() << "Post-flight analysis exited abnormally (wait status" << rc << ")";
        }
    }).detach();
}

bool CommandHandler::_handleStopDetection(bool waitForCompletion)
{
    logDebug() << "COMMAND_ID_STOP_DETECTION heartbeatStatus" << _mavlink->heartbeatStatus();

    if (_mavlink->heartbeatStatus() != HEARTBEAT_STATUS_DETECTING) {
        logError() << "COMMAND_ID_STOP_DETECTION called when not detecting";
        return false;
    }

    // Heartbeat stays DETECTING until teardown finishes, so claim the stop
    // here or two workers (e.g. STOP_ROTATION racing the UDP auto-stop) would
    // both walk and clear _processes and double-delete _airspyPipe.
    if (_detectionStopping.exchange(true)) {
        if (waitForCompletion) {
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
            while (_mavlink->heartbeatStatus() == HEARTBEAT_STATUS_DETECTING &&
                   std::chrono::steady_clock::now() < deadline) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            return _mavlink->heartbeatStatus() != HEARTBEAT_STATUS_DETECTING;
        }
        logDebug() << "COMMAND_ID_STOP_DETECTION already in progress";
        return false;
    }

    // Snapshot now: the caller may clear _inRotation before the thread below
    // reaches its checks (rotation teardown), which would wrongly unfreeze
    // time and run a per-detector analysis.
    bool inRotation;
    {
        std::lock_guard<std::mutex> lock(_rotationMutex);
        inRotation = _inRotation;
    }

    std::thread stopThread([this, inRotation]() {
        // Signal all first so the exits overlap, then share one deadline so
        // several hung children cost one timeout total, not one each.
        for (const auto& process : _processes) {
            process->terminate();
        }
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        for (const auto& process : _processes) {
            const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now());
            process->waitForExit(std::max(remaining, std::chrono::milliseconds(0)));
        }
        _processes.clear();

        delete _airspyPipe;
        _airspyPipe = NULL;

        _simPhase = 0;

        _mavlink->setDetectionMode(DETECTION_MODE_UAVRT);

        // Rotation keeps time frozen across all its per-heading detection
        // cycles and runs its own analysis after bearing computation.
        if (!inRotation) {
            _mavlink->setVehicleTimeFrozen(false);
        }

        auto logFileManager = LogFileManager::instance();

        // Capture log dir before detectorsStopped() clears it
        std::string detLogDir = logFileManager->logDir(LogFileManager::DETECTORS);

        logFileManager->detectorsStopped();

        if (!inRotation && !detLogDir.empty()) {
            _runPostFlightAnalysis(detLogDir);
        }

        // Last: HAS_TAGS gates new START_* commands, so nothing can begin mid-teardown.
        _mavlink->setHeartbeatStatus(HEARTBEAT_STATUS_HAS_TAGS);
        _detectionStopping.store(false);
    });

    if (waitForCompletion) {
        stopThread.join();
    } else {
        stopThread.detach();
    }

    return true;
}

void CommandHandler::handlePulse(const UDPPulseInfo_T& udpPulseInfo, uint32_t collectionId, uint32_t sliceId)
{
    PulseInfo_t pulseInfo;

    memset(&pulseInfo, 0, sizeof(pulseInfo));

    pulseInfo.header.command                = COMMAND_ID_PULSE;
    pulseInfo.collection_id                 = collectionId;
    pulseInfo.slice_id                      = sliceId;
    pulseInfo.tag_id                        = (uint32_t)udpPulseInfo.tag_id;
    pulseInfo.frequency_hz                  = (uint32_t)udpPulseInfo.frequency_hz;

    if (pulseInfo.frequency_hz == 0) {
        logInfo() << "HEARTBEAT from Detector" << pulseInfo.tag_id;
    } else if (std::isfinite(udpPulseInfo.detection_status)
              && static_cast<uint8_t>(udpPulseInfo.detection_status) == kNoPulseDetectionStatus) {
        // No pulse detected this cycle — forward noise floor to GCS
        pulseInfo.detection_status      = kNoPulseDetectionStatus;
        pulseInfo.confirmed_status      = 0;
        pulseInfo.stft_score            = udpPulseInfo.stft_score;
        pulseInfo.noise_psd             = udpPulseInfo.noise_psd;
        pulseInfo.group_seq_counter     = (uint16_t)udpPulseInfo.group_seq_counter;
        pulseInfo.start_time_seconds    = udpPulseInfo.start_time_seconds;

        auto telemetry = _telemetryCache->telemetryForTime(udpPulseInfo.start_time_seconds);
        logDebug() << formatString("NO DETECTION Id: %2u score_ratio: %.3f noise_psd: %5.1g freq: %9u lat/lon/yaw/alt: %3.6f %3.6f %4.0f %3.0f",
                                   pulseInfo.tag_id,
                                   udpPulseInfo.stft_score,
                                   pulseInfo.noise_psd,
                                   pulseInfo.frequency_hz,
                                   telemetry.position.latitude,
                                   telemetry.position.longitude,
                                   telemetry.attitudeEuler.yawDegrees,
                                   telemetry.position.relativeAltitude);
    } else {
        auto telemetry = _telemetryCache->telemetryForTime(udpPulseInfo.start_time_seconds);

        pulseInfo.start_time_seconds            = udpPulseInfo.start_time_seconds;
        pulseInfo.predict_next_start_seconds    = udpPulseInfo.predict_next_start_seconds;
        pulseInfo.snr                           = udpPulseInfo.snr;
        pulseInfo.stft_score                    = udpPulseInfo.stft_score;
        pulseInfo.group_seq_counter             = (uint16_t)udpPulseInfo.group_seq_counter;
        pulseInfo.group_ind                     = (uint16_t)udpPulseInfo.group_ind;
        pulseInfo.group_snr                     = udpPulseInfo.group_snr;
        pulseInfo.detection_status              = (uint8_t)udpPulseInfo.detection_status;
        pulseInfo.confirmed_status              = (uint8_t)udpPulseInfo.confirmed_status;
        pulseInfo.latitude                      = telemetry.position.latitude;
        pulseInfo.longitude                     = telemetry.position.longitude;
        pulseInfo.altitude_rel                  = telemetry.position.relativeAltitude;
        pulseInfo.roll_deg                      = telemetry.attitudeEuler.rollDegrees;
        pulseInfo.pitch_deg                     = telemetry.attitudeEuler.pitchDegrees;
        pulseInfo.yaw_deg                       = telemetry.attitudeEuler.yawDegrees;
        pulseInfo.noise_psd                     = udpPulseInfo.noise_psd;

        // Simulator hack: when pointing within 45° of directly away from
        // the transmitter (assumed due north), force unconfirmed with low SNR.
        if (_simulatorMode) {
            float yaw = telemetry.attitudeEuler.yawDegrees;
            // Normalize yaw-180 into [-180,180]
            float offBack = std::fmod(yaw - 180.0f + 540.0f, 360.0f) - 180.0f;
            if (std::fabs(offBack) < (180.0f - kBackSectorMinDeg)) {
                pulseInfo.confirmed_status = 0;
                pulseInfo.snr = 20.0;
                pulseInfo.group_snr = 20.0;
                pulseInfo.detection_status = 0; // subthreshold
            }
        }

        bool isPythonDetector = (_mavlink->detectionMode() == DETECTION_MODE_PYTHON);
        std::string pulseStatus = isPythonDetector
            ? formatString("Conf: %u Id: %2u snr: %5.1f heading: %3.1f score_ratio: %.3f noise_psd: %5.1g freq: %9u seq: %u group_ind: %u lat/lon/yaw/alt: %3.6f %3.6f %4.0f %3.0f",
                                        pulseInfo.confirmed_status,
                                        pulseInfo.tag_id,
                                        pulseInfo.snr,
                                        pulseInfo.yaw_deg,
                                        pulseInfo.stft_score,
                                        pulseInfo.noise_psd,
                                        pulseInfo.frequency_hz,
                                        pulseInfo.group_seq_counter,
                                        pulseInfo.group_ind,
                                        telemetry.position.latitude,
                                        telemetry.position.longitude,
                                        telemetry.attitudeEuler.yawDegrees,
                                        telemetry.position.relativeAltitude)
            : formatString("Conf: %u Id: %2u snr: %5.1f heading: %3.1f stft_score: %5.1g noise_psd: %5.1g freq: %9u seq: %u lat/lon/yaw/alt: %3.6f %3.6f %4.0f %3.0f",
                                        pulseInfo.confirmed_status,
                                        pulseInfo.tag_id,
                                        pulseInfo.snr,
                                        pulseInfo.yaw_deg,
                                        pulseInfo.stft_score,
                                        pulseInfo.noise_psd,
                                        pulseInfo.frequency_hz,
                                        pulseInfo.group_seq_counter,
                                        telemetry.position.latitude,
                                        telemetry.position.longitude,
                                        telemetry.attitudeEuler.yawDegrees,
                                        telemetry.position.relativeAltitude);
        if (udpPulseInfo.confirmed_status) {
            logInfo() << pulseStatus;
        } else {
            logDebug() << pulseStatus;
        }
    }

    _mavlink->sendTunnelMessage(&pulseInfo, sizeof(pulseInfo));

    {
        std::lock_guard<std::mutex> lock(_rotationMutex);
        if (_inRotation
            && _collectionCoordinator.state() == CollectionCoordinator::State::CollectingSlice
            && _collectionCoordinator.collectionId() == collectionId
            && _collectionCoordinator.sliceId() == sliceId
            && pulseInfo.frequency_hz != 0) {

            // Store slice data for pulses (both confirmed and low-confidence)
            if (pulseInfo.detection_status != kNoPulseDetectionStatus) {
                RotationSlice slice;
                slice.heading_deg       = _currentHeadingDeg;
                slice.snr_db            = pulseInfo.snr;
                slice.noise_psd         = pulseInfo.noise_psd;
                slice.confirmed_status  = pulseInfo.confirmed_status;
                slice.tag_id            = pulseInfo.tag_id;
                slice.latitude          = pulseInfo.latitude;
                slice.longitude         = pulseInfo.longitude;
                slice.altitude_rel      = pulseInfo.altitude_rel;
                _rotationSlices.push_back(slice);

                logInfo() << "Rotation slice stored: tag_id:" << slice.tag_id
                          << " heading:" << slice.heading_deg
                          << " snr:" << slice.snr_db
                          << " confirmed:" << slice.confirmed_status;
            } else {
                logInfo() << "Rotation no-detection at heading:" << _currentHeadingDeg;
            }
        }
    }
}

void CommandHandler::handlePythonDetectorMessage(
    const TagTrackerDetectorProtocol::Header& header,
    const TagTrackerDetectorProtocol::PulsePayload* pulsePayload,
    uint32_t errorCode)
{
    using TagTrackerDetectorProtocol::MessageType;

    const auto messageType = static_cast<MessageType>(header.message_type);
    if (messageType == MessageType::Heartbeat) {
        UDPPulseInfo_T heartbeat {};
        heartbeat.tag_id = header.tag_id;
        handlePulse(heartbeat);
        return;
    }

    if (messageType == MessageType::Ready) {
        std::lock_guard<std::mutex> lock(_rotationMutex);
        if (_collectionCoordinator.state() == CollectionCoordinator::State::Starting) {
            const auto result = _collectionCoordinator.detectorReady(
                _collectionCoordinator.collectionId(), header.tag_id);
            if (result == CollectionCoordinator::Result::Accepted
                && _collectionCoordinator.state() == CollectionCoordinator::State::Ready) {
                _collectionReady.notify_all();
            }
        }
        return;
    }

    if (messageType == MessageType::Armed) {
        bool allArmed = false;
        {
            std::lock_guard<std::mutex> lock(_rotationMutex);
            const auto result = _collectionCoordinator.detectorArmed(
                header.collection_id, header.slice_id, header.tag_id);
            // Duplicate ARMEDs come from a re-ARM after the GCS missed our
            // SLICE_ARMED; replay it so the retry can complete.
            allArmed = (result == CollectionCoordinator::Result::Accepted
                        || result == CollectionCoordinator::Result::Duplicate)
                && _collectionCoordinator.sliceArmed();
        }
        if (allArmed) {
            _sendCollectionStatus(header.collection_id, header.slice_id,
                                  COLLECTION_STATUS_SLICE_ARMED);
        }
        return;
    }

    if (messageType == MessageType::CycleComplete) {
        bool sliceComplete = false;
        {
            std::lock_guard<std::mutex> lock(_rotationMutex);
            const auto result = _collectionCoordinator.completeDetector(
                header.collection_id, header.slice_id, header.tag_id);
            sliceComplete = result == CollectionCoordinator::Result::Accepted
                && _collectionCoordinator.state() == CollectionCoordinator::State::Ready;
        }
        if (sliceComplete) {
            _sendCollectionStatus(header.collection_id, header.slice_id,
                                  COLLECTION_STATUS_SLICE_COMPLETE);
        }
        return;
    }

    if (messageType == MessageType::Failed) {
        {
            std::lock_guard<std::mutex> lock(_rotationMutex);
            if (_collectionCoordinator.state() != CollectionCoordinator::State::CollectingSlice
                || _collectionCoordinator.collectionId() != header.collection_id
                || _collectionCoordinator.sliceId() != header.slice_id) {
                logWarn() << "Ignoring stale Python detector failure, collection:"
                          << header.collection_id << "slice:" << header.slice_id
                          << "tag:" << header.tag_id;
                return;
            }
        }
        _sendCollectionStatus(header.collection_id, header.slice_id,
                              COLLECTION_STATUS_FAILED, errorCode);
        return;
    }

    if (messageType != MessageType::Pulse && messageType != MessageType::NoDetection) {
        logDebug() << "Python detector lifecycle message type:" << header.message_type
                   << "collection:" << header.collection_id << "slice:" << header.slice_id
                   << "tag:" << header.tag_id;
        return;
    }

    {
        std::lock_guard<std::mutex> lock(_rotationMutex);
        // Standalone START_DETECTION (no collection) reports with ids (0, 0);
        // only collection traffic is subject to the stale-slice check.
        const bool standalone = !_inRotation
            && header.collection_id == 0 && header.slice_id == 0;
        if (!standalone
            && (_collectionCoordinator.state() != CollectionCoordinator::State::CollectingSlice
                || _collectionCoordinator.collectionId() != header.collection_id
                || _collectionCoordinator.sliceId() != header.slice_id)) {
            logWarn() << "Ignoring stale Python detector result, collection:"
                      << header.collection_id << "slice:" << header.slice_id
                      << "tag:" << header.tag_id;
            return;
        }
    }

    if (pulsePayload == nullptr) {
        logError() << "Python detector pulse message missing payload";
        return;
    }

    UDPPulseInfo_T pulse {};
    pulse.tag_id = header.tag_id;
    pulse.frequency_hz = pulsePayload->frequency_hz;
    pulse.start_time_seconds = pulsePayload->start_time_seconds;
    pulse.predict_next_start_seconds = pulsePayload->predict_next_start_seconds;
    pulse.snr = pulsePayload->snr;
    pulse.stft_score = pulsePayload->score_ratio;
    pulse.group_seq_counter = pulsePayload->group_seq_counter;
    pulse.group_ind = pulsePayload->group_ind;
    pulse.group_snr = pulsePayload->group_snr;
    pulse.detection_status = pulsePayload->detection_status;
    pulse.confirmed_status = pulsePayload->confirmed_status;
    pulse.noise_psd = pulsePayload->noise_psd;
    handlePulse(pulse, header.collection_id, header.slice_id);
}

void CommandHandler::_handleDetectorProcessFailure(uint32_t tagId, int exitCode)
{
    uint32_t collectionId = 0;
    uint32_t sliceId = 0;
    uint32_t expectedDetectors = 0;
    uint32_t completedDetectors = 0;
    bool notifyStartup = false;
    {
        std::lock_guard<std::mutex> lock(_rotationMutex);
        const auto state = _collectionCoordinator.state();
        if (state == CollectionCoordinator::State::Inactive) {
            return;
        }
        collectionId = _collectionCoordinator.collectionId();
        sliceId = _collectionCoordinator.sliceId();
        // Snapshot before cancel(): it resets the counts the status reports.
        expectedDetectors = static_cast<uint32_t>(_collectionCoordinator.expectedDetectorCount());
        completedDetectors = static_cast<uint32_t>(_collectionCoordinator.completedDetectorCount());
        if (state == CollectionCoordinator::State::Starting) {
            _collectionCoordinator.cancel(collectionId);
            notifyStartup = true;
        }
    }

    if (notifyStartup) {
        _collectionReady.notify_all();
    }

    logError() << "Python detector process failed, tag:" << tagId
               << "exit code:" << exitCode << "collection:" << collectionId
               << "slice:" << sliceId;
    _sendCollectionStatus(
        collectionId,
        sliceId,
        COLLECTION_STATUS_FAILED,
        static_cast<uint32_t>(TagTrackerDetectorProtocol::ErrorCode::ProcessFailed),
        expectedDetectors,
        completedDetectors);
}

std::string CommandHandler::_handleStartCollection(const mavlink_tunnel_t& tunnel)
{
    if (tunnel.payload_length != sizeof(StartCollection_t)) {
        return "Payload length incorrect";
    }

    StartCollection_t collectionInfo {};
    memcpy(&collectionInfo, tunnel.payload, sizeof(collectionInfo));
    // These reach the detector CLI verbatim; inf/nan would silently disable detection.
    if (!std::isfinite(collectionInfo.detection_margin)
        || !std::isfinite(collectionInfo.confidence_ratio)) {
        return "Detection thresholds must be finite";
    }

    std::vector<uint32_t> tagIds;
    for (const TagInfo_t& tagInfo : _tagDatabase) {
        tagIds.push_back(tagInfo.id);
    }

    bool startPipeline = false;
    {
        std::lock_guard<std::mutex> lock(_rotationMutex);
        const auto result = _collectionCoordinator.start(collectionInfo.collection_id, tagIds);
        if (result == CollectionCoordinator::Result::Conflict) {
            return "Another collection is active";
        }
        if (result != CollectionCoordinator::Result::Duplicate) {
            if (_mavlink->heartbeatStatus() != HEARTBEAT_STATUS_HAS_TAGS) {
                _collectionCoordinator.cancel(collectionInfo.collection_id);
                return "Controller in incorrect state";
            }
            _detectorControlPorts.clear();
            for (size_t index = 0; index < tagIds.size(); ++index) {
                _detectorControlPorts[tagIds[index]] = kDetectorControlPortBase + static_cast<int>(index);
            }
            _inRotation = true;
            _currentHeadingDeg = 0;
            _rotationSlices.clear();
            startPipeline = true;
        }
    }

    if (startPipeline) {
        _mavlink->setVehicleTimeFrozen(true);
        auto logFileManager = LogFileManager::instance();
        logFileManager->rotationStarted();

        StartDetectionInfo_t startDetection {};
        startDetection.header.command = COMMAND_ID_START_DETECTION;
        startDetection.radio_center_frequency_hz = collectionInfo.radio_center_frequency_hz;
        startDetection.detection_mode = DETECTION_MODE_PYTHON;
        startDetection.detection_margin = collectionInfo.detection_margin;
        startDetection.confidence_ratio = collectionInfo.confidence_ratio;
        startDetection.debug_detector = collectionInfo.debug_detector;
        startDetection.dump_spectrogram = collectionInfo.dump_spectrogram;

        mavlink_tunnel_t syntheticTunnel {};
        syntheticTunnel.payload_length = sizeof(startDetection);
        memcpy(syntheticTunnel.payload, &startDetection, sizeof(startDetection));
        const std::string startError = _handleStartDetection(syntheticTunnel);
        if (!startError.empty()) {
            std::lock_guard<std::mutex> lock(_rotationMutex);
            _collectionCoordinator.cancel(collectionInfo.collection_id);
            _detectorControlPorts.clear();
            _inRotation = false;
            logFileManager->rotationStopped();
            _mavlink->setVehicleTimeFrozen(false);
            return startError;
        }
    }

    // A retried START_COLLECTION may arrive mid-slice; any established state
    // (Ready or CollectingSlice) means the collection is up. Only Starting
    // still needs the detectors' READY.
    auto isEstablished = [this]() {
        const auto state = _collectionCoordinator.state();
        return state == CollectionCoordinator::State::Ready
            || state == CollectionCoordinator::State::CollectingSlice;
    };
    std::unique_lock<std::mutex> lock(_rotationMutex);
    const bool ready = _collectionReady.wait_for(
        lock, std::chrono::seconds(30), [this, &isEstablished, collectionId = collectionInfo.collection_id]() {
            return _collectionCoordinator.collectionId() != collectionId || isEstablished();
        });
    if (!ready || !isEstablished()) {
        _collectionCoordinator.cancel(collectionInfo.collection_id);
        _detectorControlPorts.clear();
        lock.unlock();
        logError() << "Collection startup timed out waiting for detector READY messages";
        const auto startDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
        while (_detectionStarting.load() && std::chrono::steady_clock::now() < startDeadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (_mavlink->heartbeatStatus() == HEARTBEAT_STATUS_DETECTING) {
            _handleStopDetection(true);
        }
        {
            std::lock_guard<std::mutex> cleanupLock(_rotationMutex);
            _inRotation = false;
        }
        LogFileManager::instance()->rotationStopped();
        _mavlink->setVehicleTimeFrozen(false);
        return "Detector startup timed out";
    }

    logInfo() << "Collection ready: id:" << collectionInfo.collection_id
              << "detectors:" << tagIds.size() << "slices:" << collectionInfo.n_slices;
    return "";
}

bool CommandHandler::_sendDetectorControl(
    uint32_t tagId, const TagTrackerDetectorProtocol::ArmMessage& message)
{
    int controlPort = 0;
    {
        std::lock_guard<std::mutex> lock(_rotationMutex);
        const auto it = _detectorControlPorts.find(tagId);
        if (it == _detectorControlPorts.end()) {
            return false;
        }
        controlPort = it->second;
    }

    const int socketFd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socketFd < 0) {
        logError() << "Failed to create detector control socket:" << strerror(errno);
        return false;
    }
    sockaddr_in address {};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(controlPort);
    const auto sent = sendto(socketFd, &message, sizeof(message), 0,
                             reinterpret_cast<sockaddr*>(&address), sizeof(address));
    close(socketFd);
    if (sent != sizeof(message)) {
        logError() << "Failed to send detector ARM for tag:" << tagId
                   << "port:" << controlPort << "error:" << strerror(errno);
        return false;
    }
    return true;
}

void CommandHandler::_sendCollectionStatus(
    uint32_t collectionId, uint32_t sliceId, uint32_t status, uint32_t errorCode,
    std::optional<uint32_t> expectedDetectors, std::optional<uint32_t> completedDetectors)
{
    CollectionStatus_t message {};
    message.header.command = COMMAND_ID_COLLECTION_STATUS;
    message.collection_id = collectionId;
    message.slice_id = sliceId;
    message.status = status;
    message.error_code = errorCode;
    {
        std::lock_guard<std::mutex> lock(_rotationMutex);
        message.expected_detectors = expectedDetectors.value_or(
            static_cast<uint32_t>(_collectionCoordinator.expectedDetectorCount()));
        message.completed_detectors = completedDetectors.value_or(
            static_cast<uint32_t>(_collectionCoordinator.completedDetectorCount()));
    }
    _mavlink->sendTunnelMessage(&message, sizeof(message));
}

std::string CommandHandler::_handleStartCollectionSlice(const mavlink_tunnel_t& tunnel)
{
    if (tunnel.payload_length != sizeof(StartCollectionSlice_t)) {
        return "Payload length incorrect";
    }

    StartCollectionSlice_t sliceInfo {};
    memcpy(&sliceInfo, tunnel.payload, sizeof(sliceInfo));
    // Reject before touching state: NaN would never match as a duplicate and
    // later indexes the antenna pattern table in BearingCalculator.
    if (!std::isfinite(sliceInfo.heading_deg)) {
        return "Heading is not a finite number";
    }
    bool replayComplete = false;
    {
        std::lock_guard<std::mutex> lock(_rotationMutex);
        const auto result = _collectionCoordinator.armSlice(
            sliceInfo.collection_id, sliceInfo.slice_id, sliceInfo.heading_deg);
        replayComplete = result == CollectionCoordinator::Result::AlreadyComplete;
        // Duplicate falls through: ARM/ARMED are single UDP datagrams, so a GCS
        // retry must resend ARM or a lost one leaves the slice stuck forever.
        if (!replayComplete
            && result != CollectionCoordinator::Result::Accepted
            && result != CollectionCoordinator::Result::Duplicate) {
            return result == CollectionCoordinator::Result::Busy
                ? "Previous slice is still active" : "Invalid collection or slice id";
        }
        if (!replayComplete) {
            _currentHeadingDeg = sliceInfo.heading_deg;
        }
    }
    if (replayComplete) {
        // The GCS missed our SLICE_COMPLETE; replay it without re-arming.
        _sendCollectionStatus(sliceInfo.collection_id, sliceInfo.slice_id,
                              COLLECTION_STATUS_SLICE_COMPLETE);
        return "";
    }

    for (const TagInfo_t& tagInfo : _tagDatabase) {
        TagTrackerDetectorProtocol::ArmMessage arm {};
        arm.header.magic = TagTrackerDetectorProtocol::kMagic;
        arm.header.message_type = static_cast<uint16_t>(TagTrackerDetectorProtocol::MessageType::Arm);
        arm.header.payload_length = sizeof(arm.payload);
        arm.header.collection_id = sliceInfo.collection_id;
        arm.header.slice_id = sliceInfo.slice_id;
        arm.header.tag_id = tagInfo.id;
        arm.payload.heading_deg = sliceInfo.heading_deg;
        if (!_sendDetectorControl(tagInfo.id, arm)) {
            _sendCollectionStatus(sliceInfo.collection_id, sliceInfo.slice_id,
                                  COLLECTION_STATUS_FAILED);
            return "Failed to arm detector";
        }
    }

    logInfo() << "Collection slice armed: collection:" << sliceInfo.collection_id
              << "slice:" << sliceInfo.slice_id << "heading:" << sliceInfo.heading_deg;
    return "";
}

std::string CommandHandler::_handleFinishCollection(const mavlink_tunnel_t& tunnel)
{
    if (tunnel.payload_length != sizeof(FinishCollection_t)) {
        return "Payload length incorrect";
    }

    FinishCollection_t finishInfo {};
    memcpy(&finishInfo, tunnel.payload, sizeof(finishInfo));
    const bool finalize = finishInfo.disposition == COLLECTION_FINISH_FINALIZE;
    if (!finalize && finishInfo.disposition != COLLECTION_FINISH_CANCEL) {
        return "Invalid finish disposition";
    }

    {
        std::lock_guard<std::mutex> lock(_rotationMutex);
        const auto result = finalize
            ? _collectionCoordinator.finalize(finishInfo.collection_id)
            : _collectionCoordinator.cancel(finishInfo.collection_id);
        if (result == CollectionCoordinator::Result::Duplicate) {
            return "";
        }
        if (result != CollectionCoordinator::Result::Accepted) {
            return result == CollectionCoordinator::Result::Busy
                ? "Collection slice is still active" : "Collection id is not active";
        }
    }

    const auto startDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
    while (_detectionStarting.load() && std::chrono::steady_clock::now() < startDeadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (_detectionStarting.load()) {
        return "Detection start still in progress; retry";
    }
    if (_mavlink->heartbeatStatus() == HEARTBEAT_STATUS_DETECTING) {
        _handleStopDetection(true);
    }

    std::vector<RotationSlice> slicesCopy;
    {
        std::lock_guard<std::mutex> lock(_rotationMutex);
        slicesCopy = std::move(_rotationSlices);
        _rotationSlices.clear();
        _detectorControlPorts.clear();
        _inRotation = false;
    }
    _mavlink->setVehicleTimeFrozen(false);

    auto logFileManager = LogFileManager::instance();
    if (!finalize) {
        logFileManager->rotationStopped();
        _sendCollectionStatus(finishInfo.collection_id, 0, COLLECTION_STATUS_STOPPED);
        return "";
    }

    logInfo() << "Finishing collection" << finishInfo.collection_id << "with"
              << slicesCopy.size() << "detection results";

    // Compute bearing per tag
    BearingCalculator calculator;
    for (const auto& slice : slicesCopy) {
        calculator.addSlice(slice.heading_deg, slice.snr_db, slice.tag_id);
    }

    auto results = calculator.solve();

    // Send bearing results to GCS and log
    std::ofstream bearingLog;
    if (logFileManager->rotationActive()) {
        std::string logPath = logFileManager->filename(LogFileManager::ROTATION, "bearing_result", "log");
        bearingLog.open(logPath);
        if (bearingLog.is_open()) {
            bearingLog << "tag_id,bearing_deg,r_squared,n_valid_slices,best_snr,latitude,longitude\n";
        }
    }

    // Compute mean lat/lon per tag from the rotation slices. TelemetryCache
    // returns (0,0) when it has no fix yet; exclude that sentinel.
    std::map<uint32_t, std::pair<double, double>> tagLatLonSum;
    std::map<uint32_t, int> tagLatLonCount;
    for (const auto& slice : slicesCopy) {
        if (!std::isfinite(slice.latitude) || !std::isfinite(slice.longitude)) {
            continue;
        }
        if (slice.latitude == 0.0 && slice.longitude == 0.0) {
            continue;
        }
        tagLatLonSum[slice.tag_id].first  += slice.latitude;
        tagLatLonSum[slice.tag_id].second += slice.longitude;
        tagLatLonCount[slice.tag_id]++;
    }

    for (const auto& result : results) {
        BearingResult_t bearingResult;
        memset(&bearingResult, 0, sizeof(bearingResult));
        bearingResult.header.command    = COMMAND_ID_BEARING_RESULT;
        bearingResult.collection_id     = finishInfo.collection_id;
        bearingResult.tag_id            = result.tag_id;
        bearingResult.bearing_deg       = result.bearing_deg;
        bearingResult.r_squared         = result.r_squared;
        bearingResult.n_valid_slices    = result.n_valid_slices;
        bearingResult.best_snr          = result.best_snr;

        _mavlink->sendTunnelMessage(&bearingResult, sizeof(bearingResult));

        logInfo() << formatString("Bearing result: tag_id: %u  bearing: %.1f  R²: %.3f  slices: %u  best_snr: %.1f",
                                  result.tag_id, result.bearing_deg, result.r_squared,
                                  result.n_valid_slices, result.best_snr);

        if (bearingLog.is_open()) {
            double lat = 0.0, lon = 0.0;
            auto it = tagLatLonCount.find(result.tag_id);
            if (it != tagLatLonCount.end() && it->second > 0) {
                lat = tagLatLonSum[result.tag_id].first  / it->second;
                lon = tagLatLonSum[result.tag_id].second / it->second;
            }
            bearingLog << result.tag_id << ","
                       << result.bearing_deg << ","
                       << result.r_squared << ","
                       << result.n_valid_slices << ","
                       << result.best_snr << ","
                       << std::fixed << std::setprecision(8)
                       << lat << "," << lon << "\n";
        }
    }

    // Capture rotation log dir before rotationStopped() clears it
    std::string rotLogDir = logFileManager->logDir(LogFileManager::ROTATION);

    // Flush the CSV before the analyzer reads it.
    bearingLog.close();

    logFileManager->rotationStopped();

    // Run post-flight analysis on the rotation log directory
    if (!rotLogDir.empty()) {
        _runPostFlightAnalysis(rotLogDir);
    }

    _sendCollectionStatus(finishInfo.collection_id, 0, COLLECTION_STATUS_STOPPED);

    return "";
}

std::string CommandHandler::_handleRawCapture(const mavlink_tunnel_t& tunnel)
{
    logDebug() << "_handleRawCapture heartbeatStatus" << _mavlink->heartbeatStatus();

    if (tunnel.payload_length != sizeof(RawCaptureInfo_t)) {
        logError() << "COMMAND_ID_RAW_CAPTURE - ERROR: Payload length incorrect expected:actual" << sizeof(RawCaptureInfo_t) << tunnel.payload_length;
        return "Payload length incorrect";
    }

    if (_mavlink->heartbeatStatus() != HEARTBEAT_STATUS_HAS_TAGS) {
        _mavlink->sendStatusText("Command failed. Controller in incorrect state", MAV_SEVERITY_ALERT);
        return "Controller in incorrect state";
    }

    std::string airspyError;
    auto deviceType = _simulatorMode ? AirSpyDeviceType::SIMULATOR : _connectedAirSpyType(&airspyError);
    if (deviceType == AirSpyDeviceType::NONE) {
        logError() << "COMMAND_ID_RAW_CAPTURE - ERROR: AirSpy device detection failed: " << airspyError;
        return airspyError;
    }

    if (deviceType == AirSpyDeviceType::SIMULATOR) {
        logError() << "COMMAND_ID_RAW_CAPTURE - ERROR: Raw capture is not supported in simulator mode";
        return "Raw capture not supported in simulator mode";
    }

    RawCaptureInfo_t rawCaptureCmd;
    memcpy(&rawCaptureCmd, tunnel.payload, sizeof(rawCaptureCmd));

    if (rawCaptureCmd.frequency_hz == 0 && _tagDatabase.size() == 0) {
        logError() << "COMMAND_ID_RAW_CAPTURE - ERROR: No frequency specified and no tags configured";
        return "No frequency specified and no tags configured";
    }

    std::thread([this, rawCaptureCmd, deviceType]() {
        RawCaptureInfo_t    rawCapture = rawCaptureCmd;
        double              frequencyMhz = rawCapture.frequency_hz != 0
                                            ? (double)rawCapture.frequency_hz / 1000000.0
                                            : (double)_tagDatabase[0].frequency_hz / 1000000.0;
        auto                logFileManager = LogFileManager::instance();

        logFileManager->rawCaptureStarted();
        auto logDir = logFileManager->logDir(LogFileManager::RAW_CAPTURE);

        ++_rawCaptureCount;

        std::string commandStr;
        std::string captureLogPath;
        std::string captureDataPath;
        std::string sdrPathStatus;
        std::string processName;
        std::string sdrType;
        int         sampleRate = 0;
        int         gain = 0;
        double      tuneFreqMhz = 0;
        double      dcOffsetHz = 0;

        const int sampleDurationSeconds = 13;
        if (deviceType == AirSpyDeviceType::HF) {
            // AGC off, LNA on
            sampleRate = 768000; // 768 ksps is the default sample rate for AirSpy HF
            const int numSamples = sampleRate * sampleDurationSeconds;
            dcOffsetHz = static_cast<double>(kAirSpyHfFrequencyOffsetHz);
            tuneFreqMhz = frequencyMhz + (dcOffsetHz / 1000000.0);
            // Match start-detection tuning: capture 10 kHz above requested center to avoid the DC spike at baseband.
            // Post-processing can account for the same +10 kHz offset when interpreting frequency bins.
            captureDataPath = formatString("%s/airspy-hf.%d.dat", logDir.c_str(), _rawCaptureCount);
            commandStr = formatString("%s/repos/MavlinkTagController2/build/airspyhf_zeromq/tools/src/airspyhf_zeromq_rx -r %s -f %f -a 768000 -g off -m on -n %d",
                                      _homePath,
                                      captureDataPath.c_str(),
                                      tuneFreqMhz,
                                      numSamples);
            captureLogPath = formatString("%s/airspy-hf.%d.log", logDir.c_str(), _rawCaptureCount);
            sdrPathStatus = _sdrPathStatusText(deviceType, frequencyMhz);
            processName = "airspy-hf-capture";
            sdrType = "airspy_hf";
            logInfo() << "COMMAND_ID_RAW_CAPTURE - using AirSpy HF ZeroMQ capture command";
        } else {
            sampleRate = 3000000; // 3 Msps
            const int numSamples = sampleRate * sampleDurationSeconds;
            tuneFreqMhz = frequencyMhz;
            gain = rawCapture.gain;

            captureDataPath = formatString("%s/airspy-mini.%d.dat", logDir.c_str(), _rawCaptureCount);
            commandStr = formatString("%sairspy_rx -r %s -f %f -a 3000000 -h %d -t 0 -n %d",
                                      _airspyPath.c_str(),
                                      captureDataPath.c_str(),
                                      tuneFreqMhz,
                                      gain,
                                      numSamples);
            captureLogPath = formatString("%s/airspy-mini.%d.log", logDir.c_str(), _rawCaptureCount);
            sdrPathStatus = _sdrPathStatusText(deviceType, frequencyMhz);
            processName = "airspy-mini-capture";
            sdrType = "airspy_mini";
            logInfo() << "COMMAND_ID_RAW_CAPTURE - using AirSpy Mini capture command";
        }

        // Write capture metadata JSON
        {
            auto now = std::chrono::system_clock::now();
            auto epoch_s = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
            std::time_t tt = std::chrono::system_clock::to_time_t(now);
            char timeBuf[64];
            std::strftime(timeBuf, sizeof(timeBuf), "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&tt));

            std::string metaPath = captureDataPath.substr(0, captureDataPath.size() - 4) + ".json";
            std::ofstream metaFile(metaPath);
            if (metaFile.is_open()) {
                metaFile << "{\n";
                metaFile << formatString("  \"sdr\": \"%s\",\n", sdrType.c_str());
                metaFile << formatString("  \"requested_freq_mhz\": %.6f,\n", frequencyMhz);
                metaFile << formatString("  \"tune_freq_mhz\": %.6f,\n", tuneFreqMhz);
                metaFile << formatString("  \"dc_offset_hz\": %.0f,\n", dcOffsetHz);
                metaFile << formatString("  \"sample_rate_hz\": %d,\n", sampleRate);
                metaFile << formatString("  \"bandwidth_hz\": %d,\n", sampleRate);
                metaFile << formatString("  \"duration_seconds\": %d,\n", sampleDurationSeconds);
                metaFile << formatString("  \"gain\": %d,\n", gain);
                metaFile << "  \"format\": \"complex_float32\",\n";
                metaFile << formatString("  \"capture_utc\": \"%s\",\n", timeBuf);
                metaFile << formatString("  \"capture_epoch\": %lld,\n", (long long)epoch_s);
                metaFile << formatString("  \"data_file\": \"%s\"\n", captureDataPath.c_str());
                metaFile << "}\n";
                metaFile.close();
                logInfo() << "COMMAND_ID_RAW_CAPTURE - metadata written to" << metaPath;
            } else {
                logError() << "COMMAND_ID_RAW_CAPTURE - failed to write metadata to" << metaPath;
            }
        }

        _mavlink->sendStatusText(sdrPathStatus.c_str(), MAV_SEVERITY_INFO);

        auto airspyProcess = std::make_shared<MonitoredProcess>(
                                                    _mavlink,
                                                    processName.c_str(),
                                                    commandStr.c_str(),
                                                    captureLogPath.c_str(),
                                                    MonitoredProcess::NoPipe,
                                                    nullptr,
                                                    true /* rawCaptureProcess */);
        airspyProcess->start();

        _mavlink->setHeartbeatStatus(HEARTBEAT_STATUS_CAPTURE);
    }).detach();

    return ""; // Return empty string to indicate success
}

bool CommandHandler::_handleSaveLogs(void)
{
    logDebug() << "COMMAND_ID_SAVE_LOGS";

    if (_mavlink->heartbeatStatus() != HEARTBEAT_STATUS_IDLE && _mavlink->heartbeatStatus() != HEARTBEAT_STATUS_HAS_TAGS) {
        logError() << "COMMAND_ID_SAVE_LOGS called when not idle";
        _mavlink->sendStatusText("Command failed. Controller is not idle", MAV_SEVERITY_ALERT);
        return false;
    }
    if (_analysisJobs.load() > 0) {
        logError() << "COMMAND_ID_SAVE_LOGS called while post-flight analysis is running";
        _mavlink->sendStatusText("Command failed. Post-flight analysis still running", MAV_SEVERITY_ALERT);
        return false;
    }

    std::thread([]() {
        auto logFileManager = LogFileManager::instance();
        logFileManager->saveLogsToSDCard();
    }).detach();

    return true;
}

bool CommandHandler::_handleCleanLogs(void)
{
    logDebug() << "COMMAND_ID_CLEAN_LOGS";

    if (_mavlink->heartbeatStatus() != HEARTBEAT_STATUS_IDLE && _mavlink->heartbeatStatus() != HEARTBEAT_STATUS_HAS_TAGS) {
        logError() << "COMMAND_ID_CLEAN_LOGS called when not idle";
        _mavlink->sendStatusText("Command failed. Controller is not idle", MAV_SEVERITY_ALERT);
        return false;
    }
    if (_analysisJobs.load() > 0) {
        logError() << "COMMAND_ID_CLEAN_LOGS called while post-flight analysis is running";
        _mavlink->sendStatusText("Command failed. Post-flight analysis still running", MAV_SEVERITY_ALERT);
        return false;
    }

    std::thread([]() {
        auto logFileManager = LogFileManager::instance();
        logFileManager->cleanLocalLogs();
        MavlinkSystem::instance()->sendStatusText("#Logs deleted", MAV_SEVERITY_INFO);
    }).detach();

    return true;
}

void CommandHandler::_handleTunnelMessage(const mavlink_message_t& message)
{
    mavlink_tunnel_t tunnel;

    mavlink_msg_tunnel_decode(&message, &tunnel);

    HeaderInfo_t headerInfo;

    if (tunnel.payload_length < sizeof(headerInfo)) {
        logError() << "CommandHandler::_handleTunnelMessage payload too small";
        return;
    }

    memcpy(&headerInfo, tunnel.payload, sizeof(headerInfo));

    bool success = false;
    std::string ackMessage;

    switch (headerInfo.command) {
    case COMMAND_ID_START_TAGS:
        success = _handleStartTags(tunnel);
        break;
    case COMMAND_ID_END_TAGS:
        success = _handleEndTags();
        break;
    case COMMAND_ID_TAG:
        success = _handleTag(tunnel);
        break;
    case COMMAND_ID_START_DETECTION:
        {
            std::string errorMessage = _handleStartDetection(tunnel);
            success = errorMessage.empty();
            if (success) {
                ackMessage = LogFileManager::instance()->logDir(LogFileManager::DETECTORS);
            } else {
                ackMessage = errorMessage;
            }
        }
        break;
    case COMMAND_ID_STOP_DETECTION:
        success = _handleStopDetection();
        break;
    case COMMAND_ID_RAW_CAPTURE:
        {
            std::string errorMessage = _handleRawCapture(tunnel);
            success = errorMessage.empty();
            if (!success) {
                ackMessage = errorMessage;
            }
        }
        break;
    case COMMAND_ID_SAVE_LOGS:
        success = _handleSaveLogs();
        break;
    case COMMAND_ID_CLEAN_LOGS:
        success = _handleCleanLogs();
        break;
    case COMMAND_ID_AIRSPY_STATUS:
        if (_simulatorMode) {
            success = true;
        } else {
            std::string errorMessage = _checkForAirSpy();
            success = errorMessage.empty();
            if (!success) {
                ackMessage = errorMessage;
            }
        }
        break;
    case COMMAND_ID_START_COLLECTION:
        {
            std::string errorMessage = _handleStartCollection(tunnel);
            success = errorMessage.empty();
            if (success) {
                ackMessage = LogFileManager::instance()->logDir(LogFileManager::DETECTORS);
            } else {
                ackMessage = errorMessage;
            }
        }
        break;
    case COMMAND_ID_START_COLLECTION_SLICE:
        {
            std::string errorMessage = _handleStartCollectionSlice(tunnel);
            success = errorMessage.empty();
            if (!success) {
                ackMessage = errorMessage;
            }
        }
        break;
    case COMMAND_ID_FINISH_COLLECTION:
        {
            std::string errorMessage = _handleFinishCollection(tunnel);
            success = errorMessage.empty();
            if (!success) {
                ackMessage = errorMessage;
            }
        }
        break;
    }

    _sendCommandAck(headerInfo.command, success ? COMMAND_RESULT_SUCCESS : COMMAND_RESULT_FAILURE, ackMessage);
}

std::string CommandHandler::_checkForAirSpy(void)
{
    std::string errorMessage;
    auto deviceType = _connectedAirSpyType(&errorMessage);

    if (deviceType == AirSpyDeviceType::MINI) {
        logInfo() << "Detected AirSpy Mini";
        return "";
    }

    if (deviceType == AirSpyDeviceType::HF) {
        logInfo() << "Detected AirSpy HF";
        return "";
    }

    return errorMessage;
}

std::string CommandHandler::_sdrPathStatusText(AirSpyDeviceType deviceType, double frequencyMhz) const
{
    if (deviceType == AirSpyDeviceType::SIMULATOR) {
        return formatString("SDR path selected: IQ Simulator @ %.3f MHz", frequencyMhz);
    }

    if (deviceType == AirSpyDeviceType::HF) {
        return formatString("SDR path selected: AirSpy HF @ %.3f MHz", frequencyMhz);
    }

    if (deviceType == AirSpyDeviceType::MINI) {
        return formatString("SDR path selected: AirSpy Mini @ %.3f MHz", frequencyMhz);
    }

    return formatString("SDR path selected: Unknown AirSpy @ %.3f MHz", frequencyMhz);
}

std::string CommandHandler::_simulatorCommand(uint32_t radioCenterFrequencyHz)
{
    // Build the iq_simulator.py command line.
    // The simulator is a drop-in replacement for airspyhf_zeromq_rx, publishing
    // IQ data at 768 kHz over ZMQ PUB on port 5555.
    //
    // We derive per-tag simulator parameters from the TagDatabase entries:
    //   --freq-offset-hz : tag frequency relative to radio center
    //   --tp             : pulse width (pulse_width_msecs / 1000)
    //   --tip            : inter-pulse interval (intra_pulse1_msecs / 1000)
    //   --snr            : default 20 dB (not available in TagInfo_t)
    //
    // The venv Python is preferred so numpy/pyzmq are available.

    std::string repoDir = formatString("%s/repos/MavlinkTagController2", _homePath);
    std::string venvPython = formatString("%s/.venv/bin/python3", repoDir.c_str());
    std::string simScript  = formatString("%s/simulator/iq_simulator.py", repoDir.c_str());

    // Check if venv python exists; fall back to system python3
    std::string pythonCmd = venvPython;
    if (access(venvPython.c_str(), X_OK) != 0) {
        logInfo() << "_simulatorCommand: venv python not found, using system python3";
        pythonCmd = "python3";
    }

    // If no tags configured, use the preset
    if (_tagDatabase.size() == 0) {
        return formatString("%s -u %s --preset %s -P 5555",
                            pythonCmd.c_str(),
                            simScript.c_str(),
                            _simulatorPreset.c_str());
    }

    // Build per-tag arguments.
    //
    // For dual-rate tags (intra_pulse2_msecs != 0), the controller cycles
    // through a 4-phase pattern across successive pie slices so the detector
    // sees every combination:
    //
    //   Phase 0 (clean A):  --tip tipA                                     → group_ind 0
    //   Phase 1 (A→B):      --tip tipA --tip-secondary tipB --switch-time T → group_ind 2+
    //   Phase 2 (clean B):  --tip tipB                                     → group_ind 1
    //   Phase 3 (B→A):      --tip tipB --tip-secondary tipA --switch-time T → group_ind 2+
    //
    // switch_time places the rate change at the midpoint of the K-group.
    // The detector discards the first warmupSeconds of IQ data before
    // detection starts, so switch_time must land after warmup plus
    // enough lead-in pulses at the initial rate.  We use:
    //   T = warmup + (K + K/2) * tip
    // which gives K full pulses at the old rate (one complete segment)
    // then K/2 more before switching, so the transition falls mid-group
    // in the second detection cycle.
    static constexpr double warmupSeconds = 5.0;  // must match --warmup-seconds default

    uint32_t phase = _simPhase;
    _simPhase = (_simPhase + 1) % 4;

    std::string tagArgs;
    for (const auto& tagInfo : _tagDatabase) {
        int32_t freqOffsetHz = static_cast<int32_t>(tagInfo.frequency_hz) - static_cast<int32_t>(radioCenterFrequencyHz);
        double tp  = tagInfo.pulse_width_msecs / 1000.0;
        double tipA = tagInfo.intra_pulse1_msecs / 1000.0;

        if (tagInfo.intra_pulse2_msecs != 0) {
            double tipB = tagInfo.intra_pulse2_msecs / 1000.0;
            uint32_t k = tagInfo.k >= 2 ? tagInfo.k : 5;

            switch (phase) {
            case 0: // Clean A
                tagArgs += formatString(" --freq-offset-hz %d --snr 20 --tp %f --tip %f",
                                        freqOffsetHz, tp, tipA);
                break;
            case 1: { // A → B transition
                double switchTime = warmupSeconds + (k + k / 2) * tipA;
                tagArgs += formatString(" --freq-offset-hz %d --snr 20 --tp %f --tip %f --tip-secondary %f --switch-time %f",
                                        freqOffsetHz, tp, tipA, tipB, switchTime);
                break;
            }
            case 2: // Clean B
                tagArgs += formatString(" --freq-offset-hz %d --snr 20 --tp %f --tip %f",
                                        freqOffsetHz, tp, tipB);
                break;
            case 3: { // B → A transition
                double switchTime = warmupSeconds + (k + k / 2) * tipB;
                tagArgs += formatString(" --freq-offset-hz %d --snr 20 --tp %f --tip %f --tip-secondary %f --switch-time %f",
                                        freqOffsetHz, tp, tipB, tipA, switchTime);
                break;
            }
            }
        } else {
            tagArgs += formatString(" --freq-offset-hz %d --snr 20 --tp %f --tip %f",
                                    freqOffsetHz, tp, tipA);
        }
    }

    logInfo() << "_simulatorCommand: simPhase=" << phase
              << " (" << (phase == 0 ? "clean A" : phase == 1 ? "A->B" : phase == 2 ? "clean B" : "B->A") << ")";

    return formatString("%s -u %s -P 5555%s",
                        pythonCmd.c_str(),
                        simScript.c_str(),
                        tagArgs.c_str());
}

CommandHandler::AirSpyDeviceType CommandHandler::_connectedAirSpyType(std::string* errorMessage)
{
    logInfo() << "Checking for AirSpy devices";

    // Collect any exception messages so callers can see root cause details
    std::string exceptionDetails;

    // Try airspy_info for Mini detection
    try {
        bp::ipstream pipe_stream;
        bp::ipstream error_stream;

        std::string command = _airspyPath + "airspy_info";
        bp::child child_process(command, bp::std_out > pipe_stream, bp::std_err > error_stream);

        std::string output;
        std::string errorOutput;
        std::string line;

        while (pipe_stream && std::getline(pipe_stream, line)) {
            output += line + "\n";
        }

        while (error_stream && std::getline(error_stream, line)) {
            errorOutput += line + "\n";
        }

        child_process.wait();

        // Check if Mini device is found
        if (errorOutput.find("AIRSPY_ERROR_NOT_FOUND") == std::string::npos &&
            child_process.exit_code() == 0) {
            logInfo() << "Detected AirSpy Mini";
            return AirSpyDeviceType::MINI;
        }

    } catch (const std::exception& e) {
        logError() << "Exception running airspy_info:" << e.what();
        exceptionDetails += std::string("airspy_info exception: ") + e.what() + "\n";
    }

    // Try airspyhf_info for HF detection
    try {
        bp::ipstream pipe_stream;
        bp::ipstream error_stream;

        std::string command = _airspyPath + "airspyhf_info";
        bp::child child_process(command, bp::std_out > pipe_stream, bp::std_err > error_stream);

        std::string output;
        std::string errorOutput;
        std::string line;

        while (pipe_stream && std::getline(pipe_stream, line)) {
            output += line + "\n";
        }

        while (error_stream && std::getline(error_stream, line)) {
            errorOutput += line + "\n";
        }

        child_process.wait();

        // Check if HF device is NOT found
        std::string combinedOutput = output + errorOutput;
        if (combinedOutput.find("No devices attached") == std::string::npos &&
            child_process.exit_code() == 0) {
            logInfo() << "Detected AirSpy HF";
            return AirSpyDeviceType::HF;
        }

    } catch (const std::exception& e) {
        logError() << "Exception running airspyhf_info:" << e.what();
        exceptionDetails += std::string("airspyhf_info exception: ") + e.what() + "\n";
    }

    // No device found - surface any collected exception details when available
    if (errorMessage) {
        if (!exceptionDetails.empty()) {
            *errorMessage = exceptionDetails;
        } else {
            *errorMessage = "No AirSpy device found";
        }
    }
    return AirSpyDeviceType::NONE;
}

std::string CommandHandler::_tunnelCommandIdToString(uint32_t command)
{
    std::string commandStr;

    switch (command) {
    case COMMAND_ID_ACK:
        commandStr = "ACK";
        break;
    case COMMAND_ID_START_TAGS:
        commandStr = "START_TAGS";
        break;
    case COMMAND_ID_END_TAGS:
        commandStr = "END_TAGS";
        break;
    case COMMAND_ID_TAG:
        commandStr = "TAG";
        break;
    case COMMAND_ID_START_DETECTION:
        commandStr = "START_DETECTION";
        break;
    case COMMAND_ID_STOP_DETECTION:
        commandStr = "STOP_DETECTION";
        break;
    case COMMAND_ID_PULSE:
        commandStr = "PULSE";
        break;
    case COMMAND_ID_RAW_CAPTURE:
        commandStr = "RAW_CAPTURE";
        break;
    case COMMAND_ID_SAVE_LOGS:
        commandStr = "SAVE_LOGS";
        break;
    case COMMAND_ID_CLEAN_LOGS:
        commandStr = "CLEAN_LOGS";
        break;
    case COMMAND_ID_AIRSPY_STATUS:
        commandStr = "AIRSPY_STATUS";
        break;
    case COMMAND_ID_START_COLLECTION:
        commandStr = "START_COLLECTION";
        break;
    case COMMAND_ID_START_COLLECTION_SLICE:
        commandStr = "START_COLLECTION_SLICE";
        break;
    case COMMAND_ID_FINISH_COLLECTION:
        commandStr = "FINISH_COLLECTION";
        break;
    case COMMAND_ID_BEARING_RESULT:
        commandStr = "BEARING_RESULT";
        break;
    case COMMAND_ID_COLLECTION_STATUS:
        commandStr = "COLLECTION_STATUS";
        break;
    }

    return commandStr;
}

std::string CommandHandler::_tunnelCommandResultToString(uint32_t result)
{
    std::string resultStr;

    switch (result) {
    case COMMAND_RESULT_SUCCESS:
        resultStr = "SUCCESS";
        break;
    case COMMAND_RESULT_FAILURE:
        resultStr = "FAILURE";
        break;
    }

    return resultStr;
}
