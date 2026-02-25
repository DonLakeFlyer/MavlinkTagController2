#include "log.h"
#include "CommandHandler.h"
#include "UDPPulseReceiver.h"
#include "TunnelProtocol.h"
#include "TelemetryCache.h"
#include "MavlinkSystem.h"
#include "MavlinkFtpServer.h"
#include "PulseHandler.h"
#include "formatString.h"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <future>
#include <memory>
#include <thread>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>

MavlinkSystem* globalMavlinkSystem = nullptr;

int main(int argc, char** argv)
{
	setbuf(stdout, NULL); // Disable stdout buffering

	logDebug() << "**************** MavlinkTagController starting... ****************";

    // Check that TunnelProtocol hasn't exceed limits
    static_assert(TunnelProtocolValidateSizes, "TunnelProtocolValidateSizes failed");

	std::string connectionUrl = "udp://127.0.0.1:14540";    // default to SITL
    if (argc == 2) {
        connectionUrl = argv[1];
    }
    logInfo() << "Connecting to" << connectionUrl;

	MavlinkSystem* mavlink = MavlinkSystem::instance();
	mavlink->init(connectionUrl);


    auto ftpServer 			= MavlinkFtpServer { mavlink };
    auto telemetryCache     = new TelemetryCache(mavlink);
	auto pulseHandler 		= new PulseHandler(mavlink, telemetryCache);
    auto udpPulseReceiver   = UDPPulseReceiver { std::string("127.0.0.1"), 50000, pulseHandler };

	globalMavlinkSystem		= mavlink;

    auto commandHandler 	= CommandHandler { mavlink };

    udpPulseReceiver.start();

	if (!mavlink->start()) {
		logError() << "Mavlink start failed";
		return 1;
	}

	logInfo() << "Waiting for autopilot heartbeat...";
	while (!mavlink->connected()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	bool tunnelHeartbeatsStarted = false;
	while (true) {
		if (!tunnelHeartbeatsStarted && mavlink->gcsSystemId().has_value()) {
			tunnelHeartbeatsStarted = true;
			mavlink->startTunnelHeartbeatSender();
			// Create status text message to indicate ready. Include IP of the rPi for user reference.
			std::string ipAddress = "Unknown IP";

			struct ifaddrs *interfaces = NULL;
			if (getifaddrs(&interfaces) == 0) {
				std::string fallback;
				for (struct ifaddrs *iface = interfaces; iface != NULL; iface = iface->ifa_next) {
					if (!iface->ifa_addr) continue;
					if (iface->ifa_addr->sa_family != AF_INET) continue; // IPv4 only

					char addressBuffer[INET_ADDRSTRLEN];
					void* addrPtr = &((struct sockaddr_in*)iface->ifa_addr)->sin_addr;
					const char* ntopResult = inet_ntop(AF_INET, addrPtr, addressBuffer, sizeof(addressBuffer));
					if (ntopResult == nullptr) {
						continue;
					}
					std::string addr(addressBuffer);

					// Exclude loopback addresses
					if (addr == "127.0.0.1") continue;

					std::string name(iface->ifa_name);
					// Prefer wlan0 (or names that start with wl)
					if (name == "wlan0" || name.rfind("wl", 0) == 0) {
						ipAddress = addr;
						break;
					}

					// Keep the first non-loopback as fallback
					if (fallback.empty()) fallback = addr;
				}
				if (ipAddress == "Unknown IP" && !fallback.empty()) ipAddress = fallback;
				freeifaddrs(interfaces);
			}

			auto statusMessage = formatString("Controller Ready - %s", ipAddress.c_str());
			mavlink->sendStatusText(statusMessage);
		}

		// Do nothing -- message subscription callbacks are asynchronous and run in the connection receiver thread
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	logInfo() << "Exiting...";

    return 0;
}
