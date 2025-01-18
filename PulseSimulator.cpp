#include "PulseSimulator.h"
#include "Telemetry.h"
#include "timeHelpers.h"
#include "TunnelProtocol.h"
#include "formatString.h"
#include "log.h"
#include "PulseHandler.h"

#include <cmath>

PulseSimulator::PulseSimulator(PulseHandler* pulseHandler, MavlinkSystem* mavlink, int32_t antennaOffset)
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

            // We use the first vehicle position as the tag position
            if (!_tagPosition.has_value()&& telemetry.lastPosition().has_value()) {
                _tagPosition = telemetry.lastPosition().value();
            }

            if (_sendTagPulses && telemetry.lastPosition().has_value() && telemetry.lastAttitudeEuler().has_value() && _mavlink->gcsSystemId().has_value()) {
                // Send detector heartbeats
                PulseHandler::UDPPulseInfo_T heartbeatInfo;
                memset(&heartbeatInfo, 0, sizeof(heartbeatInfo));   // frequezy_hz == 0 means heartbeat
                heartbeatInfo.tag_id = 2;
                _pulseHandler->handlePulse(heartbeatInfo);
                heartbeatInfo.tag_id = 3;
                _pulseHandler->handlePulse(heartbeatInfo);

                // Send tag pulses for resting tag
                
                double currentTimeInSeconds = secondsSinceEpoch();

                PulseHandler::UDPPulseInfo_T pulseInfo;
                memset(&pulseInfo, 0, sizeof(pulseInfo));

                pulseInfo.tag_id            = 3;
                pulseInfo.frequency_hz      = _frequencyHz;
                pulseInfo.snr               = _calcSNR(telemetry.lastPosition().value(), _tagPosition.value(), telemetry.lastAttitudeEuler().value());
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

            std::this_thread::sleep_for(std::chrono::milliseconds(intraPulseSeconds * k * 1000));
        }
    }).detach();
}

double PulseSimulator::_toRadians(double degree) {
    return degree * M_PI / 180.0;
}

double PulseSimulator::_toDegrees(double radian) {
    return radian * (180.0 / M_PI);
}

double PulseSimulator::_calculateAzimuth(double lat1, double lon1, double lat2, double lon2) 
{
    lat1 = _toRadians(lat1);
    lon1 = _toRadians(lon1);
    lat2 = _toRadians(lat2);
    lon2 = _toRadians(lon2);

    double dLon = lon2 - lon1;
    double x = sin(dLon) * cos(lat2);
    double y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    double azimuth = atan2(x, y);

    return fmod(_toDegrees(azimuth) + 360.0, 360.0); // Normalize to 0-360 degrees
}
double PulseSimulator::_haversineDistanceKilometers(double lat1, double lon1, double lat2, double lon2) {
    static const double EARTH_RADIUS_KM = 6371.0;


    double dLat = _toRadians(lat2 - lat1);
    double dLon = _toRadians(lon2 - lon1);

    lat1 = _toRadians(lat1);
    lat2 = _toRadians(lat2);

    double a = std::sin(dLat / 2) * std::sin(dLat / 2) +
               std::sin(dLon / 2) * std::sin(dLon / 2) * std::cos(lat1) * std::cos(lat2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

    return EARTH_RADIUS_KM * c;
}

double PulseSimulator::_normalizeYaw(double yaw)
{
    return fmod(yaw + 360.0, 360.0);
}

double PulseSimulator::_calcSNR(const Telemetry::Position_t& vehiclePosition, const Telemetry::Position_t& tagPosition, const Telemetry::EulerAngle_t& vehicleAttitudeEuler)
{
    const double maxSnr = 60.0;
    const double maxDistance = 6.0;

    double vehicleDistanceToTag = _haversineDistanceKilometers(tagPosition.latitude, tagPosition.longitude, vehiclePosition.latitude, vehiclePosition.longitude);

    double snr = maxSnr;
    if (vehicleDistanceToTag > 0) {
        snr = (fmin(maxDistance, maxDistance - vehicleDistanceToTag) / maxDistance) * maxSnr;
        logDebug() << "Distance" << vehicleDistanceToTag << snr;
    }

    if (_antennaOffset != -1) {
        double antennaYawDegrees = _normalizeYaw(vehicleAttitudeEuler.yawDegrees) + _antennaOffset;
        double vehicleAzimuthToTag = _calculateAzimuth(vehiclePosition.latitude, vehiclePosition.longitude, tagPosition.latitude, tagPosition.longitude);
        double antennaYawDegreesToTag = _normalizeYaw(antennaYawDegrees - vehicleAzimuthToTag);

        logDebug() << "VehicleYaw" << _normalizeYaw(vehicleAttitudeEuler.yawDegrees) << "AntennaYawDegrees" << antennaYawDegrees << "VehicleAzimuthToTag" << vehicleAzimuthToTag << "AntennaYawDegreesToTag" << antennaYawDegreesToTag;

        if (antennaYawDegreesToTag > 180.0) {
            antennaYawDegreesToTag = 180.0 - (antennaYawDegreesToTag - 180.0);
        }
        double maxDegradation = 0.25;
        if (antennaYawDegreesToTag > 0) {
            snr *= maxDegradation + ((1 - maxDegradation) *  (1 - (antennaYawDegreesToTag / 180)));
        }
    }

    logDebug() << "SNR:" << snr;
    return snr;
}
