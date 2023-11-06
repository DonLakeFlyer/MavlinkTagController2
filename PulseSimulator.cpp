#include "PulseSimulator.h"
#include "Telemetry.h"
#include "timeHelpers.h"
#include "TunnelProtocol.h"
#include "formatString.h"
#include "log.h"

PulseSimulator::PulseSimulator(MavlinkSystem* mavlink, uint32_t antennaOffset)
	: _mavlink      (mavlink)
    , _antennaOffset(antennaOffset)
{
    std::thread([this]()
    {
        int     seqCounter = 1;
        int     intraPulseSeconds = 2.0;
        int     k = 3;

        while (true) {
            Telemetry& telemetry = _mavlink->telemetry();

            TunnelProtocol::PulseInfo_t heartbeatInfo;

            memset(&heartbeatInfo, 0, sizeof(heartbeatInfo));

            heartbeatInfo.header.command    = COMMAND_ID_PULSE;
            heartbeatInfo.frequency_hz      = 0;

            if (telemetry.lastPosition().has_value() && telemetry.lastAttitudeEuler().has_value() && _mavlink->gcsSystemId().has_value()) {
                heartbeatInfo.tag_id = 2;
                _mavlink->sendTunnelMessage(&heartbeatInfo, sizeof(heartbeatInfo));
                heartbeatInfo.tag_id = 3;
                _mavlink->sendTunnelMessage(&heartbeatInfo, sizeof(heartbeatInfo));

				auto vehicleAttitude = telemetry.lastAttitudeEuler().value();
				auto vehiclePosition = telemetry.lastPosition().value();

                double currentTimeInSeconds = secondsSinceEpoch();

                TunnelProtocol::PulseInfo_t pulseInfo;

                memset(&pulseInfo, 0, sizeof(pulseInfo));

                pulseInfo.header.command                = COMMAND_ID_PULSE;
                pulseInfo.tag_id                        = 3;
                pulseInfo.frequency_hz                  = 146000000;
                pulseInfo.snr                           = _snrFromYaw(vehicleAttitude.yawDegrees);
                pulseInfo.group_seq_counter             = seqCounter++;
                pulseInfo.confirmed_status              = 1;
                pulseInfo.position_x                    = vehiclePosition.latitude;
                pulseInfo.position_y                    = vehiclePosition.longitude;
                pulseInfo.position_z                    = vehiclePosition.relativeAltitude;
                pulseInfo.orientation_x                 = vehicleAttitude.rollDegrees;
                pulseInfo.orientation_y                 = vehicleAttitude.pitchDegrees;
                pulseInfo.orientation_z                 = vehicleAttitude.yawDegrees;
                pulseInfo.noise_psd                     = 1e-9;

                for (int i=2; i>=0; i--) {
                    pulseInfo.start_time_seconds            = currentTimeInSeconds - (i * intraPulseSeconds);
                    pulseInfo.group_ind                     = i + 1;

                    std::string pulseStatus = formatString("Conf: %u Id: %2u snr: %5.1f noise_psd: %5.1g freq: %9u lat/lon/yaw/alt: %3.6f %3.6f %4.0f %3.0f",
                                                    pulseInfo.confirmed_status,
                                                    pulseInfo.tag_id,
                                                    pulseInfo.snr,
                                                    pulseInfo.noise_psd,
                                                    pulseInfo.frequency_hz,
                                                    vehiclePosition.latitude,
                                                    vehiclePosition.longitude,
                                                    vehicleAttitude.yawDegrees,
                                                    vehiclePosition.relativeAltitude);
                    logInfo() << pulseStatus;

                    _mavlink->sendTunnelMessage(&pulseInfo, sizeof(pulseInfo));
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(intraPulseSeconds * (k + 1) * 1000));
        }
    }).detach();
}

double PulseSimulator::_normalizeYaw(double yaw)
{
    if (yaw < 0) {
        yaw += 360.0;
    } else if (yaw >= 360) {
        yaw -= 360.0;
    }

    return yaw;
}

double PulseSimulator::_snrFromYaw(double vehicleYawDegrees)
{
    double maxSnr               = 60.0;
    double antennaYawDegrees    = _normalizeYaw(_normalizeYaw(vehicleYawDegrees) + _normalizeYaw(_antennaOffset));

    logDebug() << _normalizeYaw(_normalizeYaw(vehicleYawDegrees) + _normalizeYaw(_antennaOffset)) << _normalizeYaw(vehicleYawDegrees) << _normalizeYaw(_antennaOffset);

    if (antennaYawDegrees > 180.0) {
        antennaYawDegrees = 180.0 - (antennaYawDegrees - 180.0);
    }

    return (antennaYawDegrees / 180.0) * maxSnr;
}
