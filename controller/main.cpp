#include "log.h"
#include "CommandHandler.h"
#include "UDPPulseReceiver.h"
#include "TunnelProtocol.h"
#include "TelemetryCache.h"
#include "MavlinkSystem.h"
#include "MavlinkFtpServer.h"
#include "SimulatorTelemetryPublisher.h"
#include "formatString.h"
#include "LogFileManager.h"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <future>
#include <limits>
#include <memory>
#include <optional>
#include <thread>
#include <cstring>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>

MavlinkSystem* globalMavlinkSystem = nullptr;

int main(int argc, char** argv)
{
	setbuf(stdout, NULL); // Disable stdout buffering

	logDebug() << "**************** MavlinkTagController starting... ****************";

	// Prune old log directories if disk space is low (< 25% free)
	LogFileManager::instance()->pruneOnDiskPressure();

    // Check that TunnelProtocol hasn't exceed limits
    static_assert(TunnelProtocolValidateSizes, "TunnelProtocolValidateSizes failed");

	std::string connectionUrl = "udp://127.0.0.1:14540";    // default to SITL
    bool        simulatorMode = false;
    std::string simulatorPreset = "strong";
    double      simulatorSnrDb = 20.0;
    std::optional<double> simulatorTxBearingDegArg;   // explicit --sim-tx-bearing-deg
    // Off the first heading (0) for every level so the lock happens partway
    // round and earlier headings are always filled in retrospectively.
    double      simulatorTxBearingDeg = 135.0;
    double      simulatorInterfererSnrDb = std::numeric_limits<double>::quiet_NaN();
    std::string simulatorAntenna = "ra2a";
    double      simulatorPriPpm = 43.0;      // bench RA-2A collar; 0 = ideal crystal
	std::string simulatorTelemetryEndpoint = "tcp://127.0.0.1:6001";
    bool        debugDetector = false;

    // Whole-string finite numeric parse: atof would turn a typo into 0.0 and
    // silently change the scenario; nan/inf would poison the simulated geometry.
    auto parseDouble = [](const char* text, double& out) {
        char* end = nullptr;
        errno = 0;
        out = strtod(text, &end);
        return *text != '\0' && end != text && *end == '\0' && errno == 0 && std::isfinite(out);
    };

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--simulator") == 0) {
            simulatorMode = true;
            // Optional level/preset following --simulator.
            // Signal levels are SNR at the 768 kHz simulator output. The 200x
            // decimator adds ~23 dB of processing gain before the detector, so
            // detector-side SNR is ~23 dB higher than the number here.
            // Chosen around the K=20 lock ratio of 3.0 (see simulator/README.md):
            //   strong          20 dB (~43 dB at detector)  -> sighted on every heading, confirmed;
            //                  bench-level, strong enough that sidelobe images fill the bank (#148)
            //   moderate        -8 dB (~15 dB at detector)  -> sighted on every heading, confirmed,
            //                  no sidelobe images; long-range realistic level
            //   marginal       -27 dB (~-4 dB at detector)  -> one sighting, revisit requested
            //                  (-21 dB gave three sightings, ratios 21/8/8, on 2026-09-12)
            //   below-marginal -33 dB (~-10 dB at detector) -> never locks; sub-lock hits at the tag
            //                  frequency on 2-3 headings -> "heard, no bearing"
            //   silent         no tag at all (iq_simulator noise-only preset) -> "nothing heard";
            //                  checks that pf false alarms do not become "heard"
            //   competing      -18 dB tag + flat -18 dB interferer +1 kHz:
            //                  the interferer takes the provisional lock on the first
            //                  heading, the tag is only admitted as an alternate near
            //                  its bearing, so earlier headings are filled in
            //                  retrospectively and the finish-time candidate selection
            //                  must pick the tag.
            // Any other word is an iq_simulator preset, used only when no tag is configured.
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                simulatorPreset = argv[++i];
                if (simulatorPreset == "strong") {
                    simulatorSnrDb = 20.0;
                } else if (simulatorPreset == "moderate") {
                    simulatorSnrDb = -8.0;
                } else if (simulatorPreset == "marginal") {
                    simulatorSnrDb = -27.0;
                } else if (simulatorPreset == "below-marginal") {
                    simulatorSnrDb = -33.0;
                } else if (simulatorPreset == "competing") {
                    simulatorSnrDb = -18.0;
                    simulatorInterfererSnrDb = -18.0;
                }
            }
        } else if (strcmp(argv[i], "--sim-tx-bearing-deg") == 0) {
            // Where the simulated transmitter sits relative to the first vehicle pose.
            double value = 0.0;
            if (i + 1 >= argc || !parseDouble(argv[i + 1], value)) {
                logError() << "--sim-tx-bearing-deg requires a numeric value, got" << (i + 1 < argc ? argv[i + 1] : "<none>");
                return 2;
            }
            ++i;
            simulatorTxBearingDegArg = value;
        } else if (strcmp(argv[i], "--sim-antenna") == 0) {
            // iq_simulator gain table for the simulated tag: ra2a or ra23k.
            // Independent of StartCollection_t::antenna_id so a GCS/airframe
            // mismatch can be reproduced.
            if (i + 1 >= argc) {
                logError() << "--sim-antenna requires a value (ra2a or ra23k)";
                return 2;
            }
            simulatorAntenna = argv[++i];
        } else if (strcmp(argv[i], "--sim-pri-ppm") == 0) {
            // Collar crystal offset from the nominal TIP. Default is the bench
            // collar's +43; pass 0 for a perfectly on-nominal collar.
            double value = 0.0;
            if (i + 1 >= argc || !parseDouble(argv[i + 1], value)) {
                logError() << "--sim-pri-ppm requires a numeric value, got" << (i + 1 < argc ? argv[i + 1] : "<none>");
                return 2;
            }
            ++i;
            simulatorPriPpm = value;
		} else if (strcmp(argv[i], "--sim-telemetry-endpoint") == 0) {
			if (i + 1 < argc) {
				simulatorTelemetryEndpoint = argv[++i];
			}
        } else if (strcmp(argv[i], "--debug-detector") == 0) {
            debugDetector = true;
        } else {
            // Treat any other argument as the connection URL
            connectionUrl = argv[i];
        }
    }

    if (simulatorMode) {
        // Both reach iq_simulator.py verbatim; a bad value would otherwise only
        // surface as the simulator process dying at the first START_COLLECTION.
        if (simulatorAntenna != "ra2a" && simulatorAntenna != "ra23k") {
            logError() << "--sim-antenna must be ra2a or ra23k, got" << simulatorAntenna;
            return 2;
        }
        if (!std::isfinite(simulatorPriPpm)) {
            logError() << "--sim-pri-ppm must be finite, got" << simulatorPriPpm;
            return 2;
        }
        if (1.0 + simulatorPriPpm * 1e-6 <= 0.0) {
            logError() << "--sim-pri-ppm must be > -1000000 to keep TIP positive, got" << simulatorPriPpm;
            return 2;
        }
        if (simulatorTxBearingDegArg) {
            simulatorTxBearingDeg = *simulatorTxBearingDegArg;
        }
        const bool isLevelPreset = simulatorPreset == "strong" || simulatorPreset == "moderate" || simulatorPreset == "marginal"
                                   || simulatorPreset == "below-marginal" || simulatorPreset == "competing"
                                   || simulatorPreset == "silent";
        if (isLevelPreset) {
            logInfo() << "Simulator mode enabled (level:" << simulatorPreset << " snr:" << simulatorSnrDb << "dB"
                      << " tx bearing:" << simulatorTxBearingDeg << "deg"
                      << " interferer snr:" << simulatorInterfererSnrDb << "dB"
                      << " antenna:" << simulatorAntenna << " pri ppm:" << simulatorPriPpm << ")";
        } else {
            logInfo() << "Simulator mode enabled (preset:" << simulatorPreset << ", used only when no tag is configured)";
        }
		logInfo() << "Simulator telemetry endpoint:" << simulatorTelemetryEndpoint;
    }
    logInfo() << "Connecting to" << connectionUrl;

	MavlinkSystem* mavlink = MavlinkSystem::instance();
	mavlink->init(connectionUrl);

	std::unique_ptr<SimulatorTelemetryPublisher> simulatorTelemetryPublisher;
	if (simulatorMode) {
		simulatorTelemetryPublisher = std::make_unique<SimulatorTelemetryPublisher>(mavlink, simulatorTelemetryEndpoint, 200);
		if (!simulatorTelemetryPublisher->start()) {
			logWarn() << "Failed to start simulator telemetry publisher; simulator will run without vehicle pose feed";
			simulatorTelemetryPublisher.reset();
		}
	}


    auto ftpServer 			= MavlinkFtpServer { mavlink };
    auto telemetryCache     = new TelemetryCache(mavlink);
    auto commandHandler 	= CommandHandler { mavlink, telemetryCache, simulatorMode, simulatorPreset, debugDetector, simulatorSnrDb,
                                           simulatorTxBearingDeg, simulatorInterfererSnrDb, simulatorAntenna, simulatorPriPpm };
    auto udpPulseReceiver   = UDPPulseReceiver { std::string("127.0.0.1"), CommandHandler::kPulseUdpPort, &commandHandler };

	globalMavlinkSystem		= mavlink;

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
