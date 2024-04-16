#include "log.h"
#include "CommandHandler.h"
#include "UDPPulseReceiver.h"
#include "TunnelProtocol.h"
#include "TelemetryCache.h"
#include "MavlinkSystem.h"
#include "PulseSimulator.h"
#include "MavlinkFtpServer.h"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <future>
#include <memory>
#include <thread>

int main(int argc, char** argv)
{
	setbuf(stdout, NULL); // Disable stdout buffering

    // Check that TunnelProtocol hasn't exceed limits
    static_assert(TunnelProtocolValidateSizes, "TunnelProtocolValidateSizes failed");

    bool simulatePulse 		= false;
	uint32_t antennaOffset	= 0;
	std::string connectionUrl = "udp://127.0.0.1:14540";    // default to SITL
    if (argc == 2) {
		std::string strArg = argv[1];
		std::string simulatePulsePrefix = "--simulate-pulse:";
        if (strArg.starts_with(simulatePulsePrefix)) {
			std::string strArg = argv[1];
			strArg.erase(strArg.find(simulatePulsePrefix), simulatePulsePrefix.length());

            simulatePulse = true;
			antennaOffset = std::stoi(strArg);

			logInfo() << "Simulating pulses - antenna offset:" << antennaOffset;

        } else {
            connectionUrl = strArg;
        }
    }
    logInfo() << "Connecting to" << connectionUrl;

	auto mavlink 			= new MavlinkSystem(connectionUrl);
    auto commandHandler 	= CommandHandler { mavlink };
    auto ftpServer 			= MavlinkFtpServer { mavlink };
    auto telemetryCache     = new TelemetryCache(mavlink);
    auto udpPulseReceiver   = UDPPulseReceiver { std::string("127.0.0.1"), 50000, mavlink, telemetryCache };

    udpPulseReceiver.start();

	if (!mavlink->start()) {
		logError() << "Mavlink start failed";
		return 1;
	}

	logInfo() << "Waiting for autopilot heartbeat...";
	while (!mavlink->connected()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	PulseSimulator* pulseSimulator = nullptr;
	if (simulatePulse) {
		pulseSimulator = new PulseSimulator(mavlink, antennaOffset);
	}

	bool tunnelHeartbeatsStarted = false;
	while (true) {
		if (!tunnelHeartbeatsStarted && mavlink->gcsSystemId().has_value()) {
			tunnelHeartbeatsStarted = true;
			mavlink->startTunnelHeartbeatSender();
		    mavlink->sendStatusText("MavlinkTagController Ready");
		}

		// Do nothing -- message subscription callbacks are asynchronous and run in the connection receiver thread
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	delete pulseSimulator;

	logInfo() << "Exiting...";

    return 0;
}
