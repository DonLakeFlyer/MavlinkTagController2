#include "UDPPulseReceiver.h"
#include "TunnelProtocol.h"
#include "formatString.h"
#include "log.h"
#include "MavlinkSystem.h"
#include "TelemetryCache.h"
#include "CommandHandler.h"

#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <errno.h>
#include <unistd.h> // for close()

#include <algorithm>
#include <utility>
#include <iostream>
#include <cstring>
#include <cstddef>
#include <chrono>
#include <array>

using namespace TunnelProtocol;


UDPPulseReceiver::UDPPulseReceiver(std::string localIp, int localPort, CommandHandler* commandHandler)
    : _localIp	            (std::move(localIp))
    , _localPort            (localPort)
    , _commandHandler       (commandHandler)
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
        std::array<uint8_t, 1500> buffer {};

        auto cBytesReceived = recvfrom(_fdSocket, buffer.data(), buffer.size(), 0, NULL, NULL);

        if (cBytesReceived < 0) {
            // This happens on destruction when close(_fdSocket) is called,
            // therefore be quiet.
            logDebug() << "recvfrom error:" << strerror(errno);
            return;
        }

        if (static_cast<size_t>(cBytesReceived) >= sizeof(TagTrackerDetectorProtocol::Header)) {
            TagTrackerDetectorProtocol::Header header {};
            std::memcpy(&header, buffer.data(), sizeof(header));
            if (header.magic == TagTrackerDetectorProtocol::kMagic) {
                const auto validation = TagTrackerDetectorProtocol::validateHeader(
                    header, static_cast<size_t>(cBytesReceived));
                if (validation != TagTrackerDetectorProtocol::ValidationResult::Valid) {
                    logWarn() << "Rejected malformed Python detector packet, validation:"
                              << static_cast<int>(validation) << "bytes:" << cBytesReceived;
                    continue;
                }

                TagTrackerDetectorProtocol::PulsePayload pulsePayload {};
                TagTrackerDetectorProtocol::FailedPayload failedPayload {};
                const auto messageType = static_cast<TagTrackerDetectorProtocol::MessageType>(header.message_type);
                const bool hasPulsePayload =
                    messageType == TagTrackerDetectorProtocol::MessageType::Pulse
                    || messageType == TagTrackerDetectorProtocol::MessageType::NoDetection;
                if (hasPulsePayload) {
                    std::memcpy(&pulsePayload,
                                buffer.data() + sizeof(header),
                                sizeof(pulsePayload));
                } else if (messageType == TagTrackerDetectorProtocol::MessageType::Failed) {
                    std::memcpy(&failedPayload,
                                buffer.data() + sizeof(header),
                                sizeof(failedPayload));
                }
                _commandHandler->handlePythonDetectorMessage(
                    header, hasPulsePayload ? &pulsePayload : nullptr,
                    failedPayload.error_code);
                continue;
            }
        }

        if (cBytesReceived % sizeof(CommandHandler::UDPPulseInfo_T) != 0) {
            logWarn() << "Rejected malformed legacy detector packet, bytes:" << cBytesReceived;
            continue;
        }

        int pulseCount = static_cast<int>(cBytesReceived / sizeof(CommandHandler::UDPPulseInfo_T));
        for (int pulseIndex = 0; pulseIndex < pulseCount; ++pulseIndex) {
            CommandHandler::UDPPulseInfo_T pulse {};
            std::memcpy(&pulse,
                        buffer.data() + pulseIndex * sizeof(pulse),
                        sizeof(pulse));
            _commandHandler->handlePulse(pulse);
        }
    }
}
