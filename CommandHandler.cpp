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
    , _airspyCmdLine    ("-h 21 -t 0")
{
    using namespace std::placeholders;
    _mavlink->subscribeToMessage(MAVLINK_MSG_ID_TUNNEL, std::bind(&CommandHandler::_handleTunnelMessage, this, _1));

    namespace fs = std::filesystem;

    std::string configFileName = formatString("%s/airspy_cmdline.txt", _homePath);
    fs::path path(configFileName);
    if (fs::exists(path)) {
        std::ifstream file(configFileName);
        std::string str;
        std::getline(file, str);
        _airspyCmdLine = str;
        logInfo() << "CommandHandler::CommandHandler - Using custom airspy command line:" << _airspyCmdLine;
    } else {
        logInfo() << "CommandHandler::CommandHandler - Using default airspy command line:" << _airspyCmdLine;
    }
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

    logDebug() << "_handleStartTags sdr_type" << startTagsInfo.sdr_type;

    _tagDatabase.clear();
    _receivingTags = true;
    _receivingTagsSdrType = startTagsInfo.sdr_type;

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
    std::string logPath     = logFileManager->filename(root.c_str(), "log");

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

bool CommandHandler::_handleStartDetection(const mavlink_tunnel_t& tunnel)
{
   if (tunnel.payload_length != sizeof(StartDetectionInfo_t)) {
        logError() << "COMMAND_ID_START_DETECTION - ERROR: Payload length incorrect expected:actual" << sizeof(StartDetectionInfo_t) << tunnel.payload_length;
        return false;
    }

    if (_mavlink->heartbeatStatus() != HEARTBEAT_STATUS_HAS_TAGS) {
        logError() << "COMMAND_ID_START_DETECTION - ERROR: Start detection failed. Controller in incorrect state - heartbeatStatus" << _mavlink->heartbeatStatus();
        return false;
    }

    auto logFileManager = LogFileManager::instance();
    logFileManager->detectorsStarted();
    if (!_tagDatabase.writeDetectorConfigs(_receivingTagsSdrType)) {
        logError() << "CommandHandler::_handleEndTags: writeDetectorConfigs failed";
        _mavlink->sendStatusText("Write Detector Configs failed", MAV_SEVERITY_ALERT);
    }

    std::thread([this, tunnel, logFileManager]() {
        StartDetectionInfo_t    startDetection;
        std::string             commandStr;
        std::string             logPath;
        std::string             airspyChannelizeDir;

        memcpy(&startDetection, tunnel.payload, sizeof(startDetection));

        logInfo() << "COMMAND_ID_START_DETECTION:";
        logInfo() << "\tradio_center_frequency_hz:" << startDetection.radio_center_frequency_hz; 
        logInfo() << "\tsdr_type:"                  << startDetection.sdr_type; 

        if (_pulseSimulator) {
            _pulseSimulator->startSendingTagPulses(startDetection.radio_center_frequency_hz);
        } else {
            switch (startDetection.sdr_type) {
            case SDR_TYPE_AIRSPY_MINI:
            {
                _airspyPipe         = new bp::pipe();
                airspyChannelizeDir = "airspy_channelize_mini";

                commandStr  = formatString("/usr/local/bin/airspy_rx -f %f -a 3000000 -r /dev/stdout %s", (double)startDetection.radio_center_frequency_hz / 1000000.0, _airspyCmdLine.c_str());
                logPath     = logFileManager->filename("airspy_rx", "log");
                MonitoredProcess* airspyProc = new MonitoredProcess(
                                                        _mavlink, 
                                                        "airspy_rx", 
                                                        commandStr.c_str(), 
                                                        logPath.c_str(), 
                                                        MonitoredProcess::OutputPipe,
                                                        _airspyPipe);
                airspyProc->start();
                _processes.push_back(airspyProc);

                logPath = logFileManager->filename("csdr-uavrt", "log");
                MonitoredProcess* csdrProc = new MonitoredProcess(
                                                        _mavlink, 
                                                        "csdr-uavrt", 
                                                        "csdr-uavrt fir_decimate_cc 8 0.05 HAMMING", 
                                                        logPath.c_str(), 
                                                        MonitoredProcess::InputPipe,
                                                        _airspyPipe);
                csdrProc->start();
                _processes.push_back(csdrProc);
            }
                break;

            case SDR_TYPE_AIRSPY_HF:
            {
                airspyChannelizeDir = "airspy_channelize_hf";

                commandStr  = formatString("airspyhf_rx_udp -u 10000 -f %f -a 192000 -g on -l low", (double)startDetection.radio_center_frequency_hz / 1000000.0);
                logPath     = logFileManager->filename("airspyhf_rx_udp", "log");
                MonitoredProcess* airspyProc = new MonitoredProcess(
                                                        _mavlink, 
                                                        "airspyhf_rx_udp", 
                                                        commandStr.c_str(), 
                                                        logPath.c_str(), 
                                                        MonitoredProcess::NoPipe,
                                                        NULL);
                airspyProc->start();
                _processes.push_back(airspyProc);
            }
                break;

            default:
                logError() << "_handleStartDetection - Unknown sdr type:" << startDetection.sdr_type;
                _mavlink->sendStatusText("Command failed. Unknown sdr type.", MAV_SEVERITY_ERROR);
                return;
            }

            commandStr  = formatString("%s/repos/%s/airspy_channelize %s", _homePath, airspyChannelizeDir.c_str(), _tagDatabase.channelizerCommandLine().c_str());
            logPath = logFileManager->filename("airspy_channelize", "log");
            MonitoredProcess* channelizeProc = new MonitoredProcess(
                                                        _mavlink, 
                                                        "airspy_channelize", 
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

        std::string startedStr = formatString("#All processes started at center hz: %.3f", (double)startDetection.radio_center_frequency_hz / 1000000.0);
        _mavlink->sendStatusText(startedStr.c_str(), MAV_SEVERITY_INFO);

        _mavlink->setHeartbeatStatus(HEARTBEAT_STATUS_DETECTING);
    }).detach();

    return true;
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
        _mavlink->sendStatusText("#Detectors stopped", MAV_SEVERITY_INFO);

        auto logFileManager = LogFileManager::instance();
        logFileManager->detectorsStopped();
    }).detach();

    return true;
}

bool CommandHandler::_handleRawCapture(const mavlink_tunnel_t& tunnel)
{
    logDebug() << "_handleRawCapture heartbeatStatus" << _mavlink->heartbeatStatus();

    if (tunnel.payload_length != sizeof(RawCapture_t)) {
        logError() << "COMMAND_ID_RAW_CAPTURE - ERROR: Payload length incorrect expected:actual" << sizeof(RawCapture_t) << tunnel.payload_length;
        return false;
    }

    if (_mavlink->heartbeatStatus() != HEARTBEAT_STATUS_HAS_TAGS) {
        _mavlink->sendStatusText("Command failed. Controller in incorrect state", MAV_SEVERITY_ALERT);
        return false;
    }

    std::thread([this, tunnel]() {
        RawCapture_t    rawCapture;
        double          frequencyMhz = (double)_tagDatabase[0].frequency_hz / 1000000.0; 
        std::string     commandStr;
        std::string     logPath;

        memcpy(&rawCapture, tunnel.payload, sizeof(rawCapture));

        switch (rawCapture.sdr_type) {
        case SDR_TYPE_AIRSPY_MINI:
            commandStr = formatString("/usr/local/bin/airspy_rx -r %s/airspy_mini.dat -f %f -a 3000000 -h 21 -t 0 -n 90000000", _homePath, frequencyMhz);
            logPath    = formatString("%s/airspy-mini-capture.log", _homePath);
            break;
        case SDR_TYPE_AIRSPY_HF:
            commandStr = formatString("/usr/local/bin/airspyhf_rx_udp -r %s/airspy_hf.dat -f %f -a 192000 -g on -l low -n 5760000", _homePath, frequencyMhz);
            logPath    = formatString("%s/airspy-hf-capture.log", _homePath);
            break;
        default:
            logError() << "_handleRawCapture - Unknown sdr type:" << rawCapture.sdr_type;
            _mavlink->sendStatusText("Command failed. Unknown sdr type.", MAV_SEVERITY_ERROR);
            return;
        }

        MonitoredProcess* airspyProcess = new MonitoredProcess(
                                                    _mavlink, 
                                                    "airspy-capture", 
                                                    commandStr.c_str(), 
                                                    logPath.c_str(), 
                                                    MonitoredProcess::NoPipe,
                                                    NULL,
                                                    true /* rawCaptureProcess */);
        airspyProcess->start();

        _mavlink->setHeartbeatStatus(HEARTBEAT_STATUS_CAPTURE);
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
        success = _handleStartDetection(tunnel);
        if (success) {
            ackMessage = LogFileManager::instance()->logDir();
        }
        break;
    case COMMAND_ID_STOP_DETECTION:
        success = _handleStopDetection();
        break;
    case COMMAND_ID_RAW_CAPTURE:
        success = _handleRawCapture(tunnel);
        break;
    }

    _sendCommandAck(headerInfo.command, success ? COMMAND_RESULT_SUCCESS : COMMAND_RESULT_FAILURE, ackMessage);
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
