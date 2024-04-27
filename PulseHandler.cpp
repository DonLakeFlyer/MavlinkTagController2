#include "PulseHandler.h"
#include "TunnelProtocol.h"
#include "formatString.h"
#include "log.h"
#include "MavlinkSystem.h"
#include "TelemetryCache.h"

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


PulseHandler::PulseHandler(MavlinkSystem* mavlink, TelemetryCache* telemetryCache)
    : _mavlink          (mavlink)
    , _telemetryCache   (telemetryCache)
{

}

PulseHandler::~PulseHandler()
{

}

void PulseHandler::handlePulse(const PulseHandler::UDPPulseInfo_T& udpPulseInfo)
{
    PulseInfo_t pulseInfo;

    memset(&pulseInfo, 0, sizeof(pulseInfo));

    pulseInfo.header.command                = COMMAND_ID_PULSE;
    pulseInfo.tag_id                        = (uint32_t)udpPulseInfo.tag_id;
    pulseInfo.frequency_hz                  = (uint32_t)udpPulseInfo.frequency_hz;

    if (pulseInfo.frequency_hz == 0) {
        logInfo() << "HEARTBEAT from Detector" << pulseInfo.tag_id;
    } else {
        auto telemetry = _telemetryCache->telemetryForTime(udpPulseInfo.start_time_seconds);

        pulseInfo.start_time_seconds            = udpPulseInfo.start_time_seconds;
        pulseInfo.predict_next_start_seconds    = udpPulseInfo.predict_next_start_seconds;
        pulseInfo.snr                           = udpPulseInfo.snr;
        pulseInfo.stft_score                    = udpPulseInfo.stft_score;
        pulseInfo.group_seq_counter             = (uint16_t)udpPulseInfo.group_seq_counter;
        pulseInfo.group_ind                     = (uint16_t)udpPulseInfo.group_ind;
        pulseInfo.group_snr                     = udpPulseInfo.group_snr;
        pulseInfo.detection_status              = (uint8_t)udpPulseInfo.detection_status;
        pulseInfo.confirmed_status              = (uint8_t)udpPulseInfo.confirmed_status;
        pulseInfo.position_x                    = telemetry.position.latitude;
        pulseInfo.position_y                    = telemetry.position.longitude;
        pulseInfo.position_z                    = telemetry.position.relativeAltitude;
        pulseInfo.orientation_x                 = telemetry.attitudeEuler.rollDegrees;
        pulseInfo.orientation_y                 = telemetry.attitudeEuler.pitchDegrees;
        pulseInfo.orientation_z                 = telemetry.attitudeEuler.yawDegrees;
        pulseInfo.noise_psd                     = udpPulseInfo.noise_psd;

        std::string pulseStatus = formatString("Conf: %u Id: %2u snr: %5.1f noise_psd: %5.1g freq: %9u lat/lon/yaw/alt: %3.6f %3.6f %4.0f %3.0f",
                                        pulseInfo.confirmed_status,
                                        pulseInfo.tag_id,
                                        pulseInfo.snr,
                                        pulseInfo.noise_psd,
                                        pulseInfo.frequency_hz,
                                        telemetry.position.latitude,
                                        telemetry.position.longitude,
                                        telemetry.attitudeEuler.yawDegrees,
                                        telemetry.position.relativeAltitude);
        if (udpPulseInfo.confirmed_status) {
            logInfo() << pulseStatus;
        } else {
            logDebug() << pulseStatus;
        }
    }

    _mavlink->sendTunnelMessage(&pulseInfo, sizeof(pulseInfo));
}
