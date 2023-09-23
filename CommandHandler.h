#pragma once

#include "TunnelProtocol.h"
#include "TagDatabase.h"

#include <boost/process.hpp>

#include <mavlink.h>

namespace bp = boost::process;

class MavlinkSystem;
class MonitoredProcess;
class LogFileManager;

class CommandHandler {
public:
    CommandHandler(MavlinkSystem* mavlink);

private:
    void _sendCommandAck        (uint32_t command, uint32_t result);
    bool _handleStartTags       (const mavlink_tunnel_t& tunnel);
    bool _handleEndTags         (void);
    bool _handleTag             (const mavlink_tunnel_t& tunnel);
    bool _handleStartDetection  (const mavlink_tunnel_t& tunnel);
    bool _handleStopDetection   (void);
    bool _handleRawCapture      (const mavlink_tunnel_t& tunnel);
    void _handleTunnelMessage   (const mavlink_message_t& message);
    void _startDetector         (LogFileManager* logFileManager, const TunnelProtocol::TagInfo_t& tagInfo, bool secondaryChannel);

    std::string _tunnelCommandIdToString    (uint32_t command);
    std::string _tunnelCommandResultToString(uint32_t result);

private:
    MavlinkSystem*                  _mavlink;
    TagDatabase                     _tagDatabase;
    bool                            _receivingTags          = false;
    uint32_t                        _receivingTagsSdrType;
    char*                           _homePath               = NULL;
    std::vector<MonitoredProcess*>  _processes;
    bp::pipe*                       _airspyPipe             = NULL;
};
