#include "UDPPulseReceiver.h"
#include "TunnelProtocol.h"
#include "formatString.h"
#include "log.h"
#include "MavlinkSystem.h"
#include "TelemetryCache.h"
#include "PulseHandler.h"

#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <errno.h>
#include <unistd.h> // for close()

#include <algorithm>
#include <utility>
#include <iostream>
#include <string.h>
#include <cstddef>
#include <chrono>

using namespace TunnelProtocol;


UDPPulseReceiver::UDPPulseReceiver(std::string localIp, int localPort, PulseHandler* pulseHandler)
    : _localIp	            (std::move(localIp))
    , _localPort            (localPort)
    , _pulseHandler         (pulseHandler)
{

}

UDPPulseReceiver::~UDPPulseReceiver()
{
    // If no one explicitly called stop before, we should at least do it.
    stop();
}


void UDPPulseReceiver::start()
{
    if (!_setupPort()) {
        return;
    }

    _thread = new std::thread(&UDPPulseReceiver::_receive, this);
}

bool UDPPulseReceiver::_setupPort(void)
{
    _fdSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (_fdSocket < 0) {
        logError() << "socket error" << strerror(errno);
        return false;
    }

    struct sockaddr_in addr {};
    addr.sin_family         = AF_INET;
    addr.sin_addr.s_addr    = inet_addr(_localIp.c_str());
    addr.sin_port           = htons(_localPort);

    if (bind(_fdSocket, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        logError() << "bind error:" << strerror(errno);
        return false;
    }

    logDebug() << "UDPPulseReceiver::_setupPort" << _localIp << "port:" << _localPort;

    return true;
}

void UDPPulseReceiver::stop()
{
    // This should interrupt a recv/recvfrom call.
    shutdown(_fdSocket, SHUT_RDWR);

    // But on Mac, closing is also needed to stop blocking recv/recvfrom.
    close(_fdSocket);
}

void UDPPulseReceiver::_receive()
{
    while (true) {
        PulseHandler::UDPPulseInfo_T buffer[sizeof(PulseHandler::UDPPulseInfo_T) * 10];

        auto cBytesReceived = recvfrom(_fdSocket, buffer, sizeof(buffer), 0, NULL, NULL);

        if (cBytesReceived < 0) {
            // This happens on destruction when close(_fdSocket) is called,
            // therefore be quiet.
            logDebug() << "recvfrom error:" << strerror(errno);
            return;
        }

        int pulseCount = cBytesReceived / sizeof(PulseHandler::UDPPulseInfo_T);
        int pulseIndex = 0;

        while (pulseCount--) {
            _pulseHandler->handlePulse(buffer[pulseIndex++]);
        }
    }
}
