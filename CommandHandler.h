#pragma once

#include "TunnelProtocol.h"
#include "TagDatabase.h"

#include <boost/process.hpp>

#include <string>

#include <mavlink.h>

namespace bp = boost::process;

class MavlinkSystem;
class MonitoredProcess;
class LogFileManager;
class PulseSimulator;

class CommandHandler {
public:
    CommandHandler(MavlinkSystem* mavlink, PulseSimulator* pulseSimulator);

private:
    void _sendCommandAck        (uint32_t command, uint32_t result, std::string& ackMessage);
    bool _handleStartTags       (const mavlink_tunnel_t& tunnel);
    bool _handleEndTags         (void);
    bool _handleTag             (const mavlink_tunnel_t& tunnel);
    bool _handleStartDetection  (const mavlink_tunnel_t& tunnel);
    bool _handleStopDetection   (void);
    bool _handleRawCapture      (const mavlink_tunnel_t& tunnel);
    bool _handleSaveLogs        (void);
    bool _handleCleanLogs       (void);
    void _handleTunnelMessage   (const mavlink_message_t& message);
    void _startDetector         (LogFileManager* logFileManager, const TunnelProtocol::TagInfo_t& tagInfo, bool secondaryChannel);

    std::string _tunnelCommandIdToString    (uint32_t command);
    std::string _tunnelCommandResultToString(uint32_t result);

private:
    MavlinkSystem*                  _mavlink                = nullptr;
    PulseSimulator*                 _pulseSimulator         = nullptr;
    TagDatabase                     _tagDatabase;
    bool                            _receivingTags          = false;
    char*                           _homePath               = nullptr;
    std::vector<MonitoredProcess*>  _processes;
    bp::pipe*                       _airspyPipe             = nullptr;
    std::string                     _airspyPath;
    std::string                     _airspyCmdLine;
    int                            _rawCaptureCount         = 0;
};
