#include <chrono>
#include <cstdint>
#include <iostream>
#include <future>
#include <memory>
#include <thread>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

#include <mavlink.h>

#include <stdio.h>
#include <filesystem>
#include <fstream>
#include "CommandHandler.h"
#include "TunnelProtocol.h"
#include "MonitoredProcess.h"
#include "formatString.h"
#include "log.h"
#include "channelizerTuner.h"
#include "MavlinkSystem.h"
#include "LogFileManager.h"
#include "PulseSimulator.h"

using namespace TunnelProtocol;

CommandHandler::CommandHandler(MavlinkSystem* mavlink, PulseSimulator* pulseSimulator)
    : _mavlink          (mavlink)
    , _pulseSimulator   (pulseSimulator)
    , _homePath         (getenv("HOME"))
{
    if (strcmp(_homePath, "/home/pi") == 0) {
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
    std::string root        = formatString("detector_%d", tagInfo.id + (secondaryChannel ? 1 : 0));
    std::string logPath     = logFileManager->filename(LogFileManager::DETECTORS, root.c_str(), "log");

    MonitoredProcess* detectorProc = new MonitoredProcess(
                                                _mavlink,
                                                "uavrt_detection",
                                                commandStr.c_str(),
                                                logPath.c_str(),
                                                MonitoredProcess::NoPipe,
                                                NULL);
    detectorProc->start();
    _processes.push_back(detectorProc);
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

    std::string airspyError;
    auto deviceType = _connectedAirSpyType(&airspyError);
    if (deviceType == AirSpyDeviceType::NONE) {
        logError() << "COMMAND_ID_START_DETECTION - ERROR: AirSpy detection failed: " << airspyError;
        return std::string("AirSpy detection failed: ") + airspyError;
    }

    auto logFileManager = LogFileManager::instance();
    logFileManager->detectorsStarted();
    if (!_tagDatabase.writeDetectorConfigs()) {
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
        const double centerFrequencyMhz = (double)startDetection.radio_center_frequency_hz / 1000000.0;

        if (_pulseSimulator) {
            _pulseSimulator->startSendingTagPulses(startDetection.radio_center_frequency_hz);
        } else {
            _airspyPipe         = new bp::pipe();
            std::string sdrPathStatus;
            if (deviceType == AirSpyDeviceType::HF) {
                airspyChannelizeDir = "airspyhf_channelize";
                airspyChannelizeProcessName = "airspyhf_channelize";
                airspyChannelizeExecutable = "airspyhf_channelize";
                airspyReceiverProcessName = "airspyhf_rx";
                sdrPathStatus = _sdrPathStatusText(deviceType, centerFrequencyMhz);

                commandStr = formatString("%sairspyhf_rx -f %f -a 768000 -r /dev/stdout -g off -m on",
                                    _airspyPath.c_str(),
                                    centerFrequencyMhz);
                logInfo() << "COMMAND_ID_START_DETECTION - using AirSpy HF stream source";
            } else {
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
            }

            _mavlink->sendStatusText(sdrPathStatus.c_str(), MAV_SEVERITY_INFO);

            logPath     = logFileManager->filename(LogFileManager::DETECTORS, airspyReceiverProcessName.c_str(), "log");
            MonitoredProcess* airspyProc = new MonitoredProcess(
                                                    _mavlink,
                                                    airspyReceiverProcessName.c_str(),
                                                    commandStr.c_str(),
                                                    logPath.c_str(),
                                                    MonitoredProcess::OutputPipe,
                                                    _airspyPipe);
            airspyProc->start();
            _processes.push_back(airspyProc);

            logPath = logFileManager->filename(LogFileManager::DETECTORS, "csdr-uavrt", "log");
            MonitoredProcess* csdrProc = new MonitoredProcess(
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
            MonitoredProcess* channelizeProc = new MonitoredProcess(
                                                        _mavlink,
                                                        airspyChannelizeProcessName.c_str(),
                                                        commandStr.c_str(),
                                                        logPath.c_str(),
                                                        MonitoredProcess::NoPipe,
                                                        NULL);
            channelizeProc->start();
            _processes.push_back(channelizeProc);

            for (const TunnelProtocol::TagInfo_t& tagInfo: _tagDatabase) {
                _startDetector(logFileManager, tagInfo, false /* secondaryChannel */);
                if (tagInfo.intra_pulse2_msecs != 0) {
                    _startDetector(logFileManager, tagInfo, true /* secondaryChannel */);
                }
            }
        }

        std::string startedStr = formatString("All processes started at center hz: %.3f", (double)startDetection.radio_center_frequency_hz / 1000000.0);
        _mavlink->sendStatusText(startedStr.c_str(), MAV_SEVERITY_INFO);

        _mavlink->setHeartbeatStatus(HEARTBEAT_STATUS_DETECTING);
    }).detach();

    return ""; // Return empty string to indicate success
}

bool CommandHandler::_handleStopDetection(void)
{
    logDebug() << "COMMAND_ID_STOP_DETECTION heartbeatStatus" << _mavlink->heartbeatStatus();

    if (_mavlink->heartbeatStatus() != HEARTBEAT_STATUS_DETECTING) {
        logError() << "COMMAND_ID_STOP_DETECTION called when not detecting";
        return false;
    }

    std::thread([this]() {
        if (_pulseSimulator) {
            _pulseSimulator->stopSendingTagPulses();
        } else {
            for (MonitoredProcess* process: _processes) {
                process->stop();
            }
            _processes.clear();

            delete _airspyPipe;
            _airspyPipe = NULL;
        }

        _mavlink->setHeartbeatStatus(HEARTBEAT_STATUS_HAS_TAGS);

        auto logFileManager = LogFileManager::instance();
        logFileManager->detectorsStopped();
    }).detach();

    return true;
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
    auto deviceType = _connectedAirSpyType(&airspyError);
    if (deviceType == AirSpyDeviceType::NONE) {
        logError() << "COMMAND_ID_RAW_CAPTURE - ERROR: AirSpy device detection failed: " << airspyError;
        return airspyError;
    }

    if (_tagDatabase.size() == 0) {
        logError() << "COMMAND_ID_RAW_CAPTURE - ERROR: No tags configured";
        return "No tags configured";
    }

    std::thread([this, tunnel, deviceType]() {
        RawCaptureInfo_t    rawCapture;
        double              frequencyMhz = (double)_tagDatabase[0].frequency_hz / 1000000.0;
        auto                logFileManager = LogFileManager::instance();

        logFileManager->rawCaptureStarted();
        auto logDir = logFileManager->logDir(LogFileManager::RAW_CAPTURE);

        memcpy(&rawCapture, tunnel.payload, sizeof(rawCapture));

        ++_rawCaptureCount;

        std::string commandStr;
        std::string captureLogPath;
        std::string sdrPathStatus;
        std::string processName;

        const int sampleDurationSeconds = 8;
        if (deviceType == AirSpyDeviceType::HF) {
            // AGC off, LNA on
            const int sampleRate = 768000; // 768 ksps is the default sample rate for AirSpy HF
            const int numSamples = sampleRate * sampleDurationSeconds;
            commandStr = formatString("%sairspyhf_rx -r %s/airspy-hf.%d.dat -f %f -a 768000 -g off -m on -n %d",
                                      _airspyPath.c_str(),
                                      logDir.c_str(),
                                      _rawCaptureCount,
                                      frequencyMhz,
                                      numSamples);
            captureLogPath = formatString("%s/airspy-hf.%d.log", logDir.c_str(), _rawCaptureCount);
            sdrPathStatus = _sdrPathStatusText(deviceType, frequencyMhz);
            processName = "airspy-hf-capture";
            logInfo() << "COMMAND_ID_RAW_CAPTURE - using AirSpy HF capture command";
        } else {
            const int sampleRate = 3000000; // 3 Msps
            const int numSamples = sampleRate * sampleDurationSeconds;

            commandStr = formatString("%sairspy_rx -r %s/airspy-mini.%d.dat -f %f -a 3000000 -h %d -t 0 -n %d",
                                      _airspyPath.c_str(),
                                      logDir.c_str(),
                                      _rawCaptureCount,
                                      frequencyMhz,
                                      rawCapture.gain,
                                      numSamples);
            captureLogPath = formatString("%s/airspy-mini.%d.log", logDir.c_str(), _rawCaptureCount);
            sdrPathStatus = _sdrPathStatusText(deviceType, frequencyMhz);
            processName = "airspy-mini-capture";
            logInfo() << "COMMAND_ID_RAW_CAPTURE - using AirSpy Mini capture command";
        }

        _mavlink->sendStatusText(sdrPathStatus.c_str(), MAV_SEVERITY_INFO);

        MonitoredProcess* airspyProcess = new MonitoredProcess(
                                                    _mavlink,
                                                    processName.c_str(),
                                                    commandStr.c_str(),
                                                    captureLogPath.c_str(),
                                                    MonitoredProcess::NoPipe,
                                                    NULL,
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
    if (deviceType == AirSpyDeviceType::HF) {
        return formatString("SDR path selected: AirSpy HF @ %.3f MHz", frequencyMhz);
    }

    if (deviceType == AirSpyDeviceType::MINI) {
        return formatString("SDR path selected: AirSpy Mini @ %.3f MHz", frequencyMhz);
    }

    return formatString("SDR path selected: Unknown AirSpy @ %.3f MHz", frequencyMhz);
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

        // Check if Mini device is NOT found
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
