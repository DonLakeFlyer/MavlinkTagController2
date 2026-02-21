#pragma once

#include <mavlink.h>

class MavlinkSystem;

class LogRequestHandler {
public:
    LogRequestHandler(MavlinkSystem* mavlink);

private:
    void _handleLogRequestList  (const mavlink_message_t& message);

private:
    MavlinkSystem*  _mavlink;
    char*           _homePath = nullptr;
};
