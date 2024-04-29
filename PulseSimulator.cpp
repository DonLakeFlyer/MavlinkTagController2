#include "PulseSimulator.h"
#include "Telemetry.h"
#include "timeHelpers.h"
#include "TunnelProtocol.h"
#include "formatString.h"
#include "log.h"
#include "PulseHandler.h"

PulseSimulator::PulseSimulator(PulseHandler* pulseHandler, MavlinkSystem* mavlink, uint32_t antennaOffset)
    : _pulseHandler (pulseHandler)
	, _mavlink      (mavlink)
    , _antennaOffset(antennaOffset)
{
    std::thread([this]()
    {
        int     seqCounter = 1;
        int     intraPulseSeconds = 2.0;
        int     k = 3;

        while (true) {
            Telemetry& telemetry = _mavlink->telemetry();

            PulseHandler::UDPPulseInfo_T heartbeatInfo;
            memset(&heartbeatInfo, 0, sizeof(heartbeatInfo));   // frequezy_hz == 0 means heartbeat

            if (telemetry.lastPosition().has_value() && telemetry.lastAttitudeEuler().has_value() && _mavlink->gcsSystemId().has_value()) {
                if (_sendTagPulses) {
                    // Send detector heartbeats
                    heartbeatInfo.tag_id = 2;
                    _pulseHandler->handlePulse(heartbeatInfo);
                    heartbeatInfo.tag_id = 3;
                    _pulseHandler->handlePulse(heartbeatInfo);

                    // Send tag pulses for resting tag
                    
                    auto vehicleAttitude = telemetry.lastAttitudeEuler().value();

                    double currentTimeInSeconds = secondsSinceEpoch();

                    PulseHandler::UDPPulseInfo_T pulseInfo;
                    memset(&pulseInfo, 0, sizeof(pulseInfo));

                    pulseInfo.tag_id            = 3;
                    pulseInfo.frequency_hz      = _frequencyHz;
                    pulseInfo.snr               = _snrFromYaw(vehicleAttitude.yawDegrees);
                    pulseInfo.stft_score        = pulseInfo.snr;
                    pulseInfo.group_seq_counter = seqCounter++;
                    pulseInfo.confirmed_status  = 1;
                    pulseInfo.noise_psd         = 1e-9;

                    for (int i=2; i>=0; i--) {
                        pulseInfo.start_time_seconds    = currentTimeInSeconds - (i * intraPulseSeconds);
                        pulseInfo.group_ind             = i + 1;

                        std::string pulseStatus = formatString("Conf: %u Id: %2u snr: %5.1f noise_psd: %5.1g freq: %9u",
                                                        pulseInfo.confirmed_status,
                                                        pulseInfo.tag_id,
                                                        pulseInfo.snr,
                                                        pulseInfo.noise_psd,
                                                        pulseInfo.frequency_hz);
                        logInfo() << pulseStatus;

                        _pulseHandler->handlePulse(pulseInfo);
                    }
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
