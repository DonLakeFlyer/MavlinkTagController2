#include "PythonPulseMapper.h"

#include <cstring>

using namespace TunnelProtocol;

PythonPulseInfo_t buildPythonPulseInfo(
    const TagTrackerDetectorProtocol::Header& header,
    const TagTrackerDetectorProtocol::PulsePayload& payload,
    const TelemetryCache::TelemetryCacheEntry_t& telemetry)
{
    PythonPulseInfo_t pulseInfo;
    memset(&pulseInfo, 0, sizeof(pulseInfo));

    pulseInfo.header.command     = COMMAND_ID_PYTHON_PULSE;
    pulseInfo.collection_id      = header.collection_id;
    pulseInfo.slice_id           = header.slice_id;
    pulseInfo.tag_id             = header.tag_id;
    pulseInfo.frequency_hz       = payload.frequency_hz;
    pulseInfo.cycle_counter      = payload.group_seq_counter;
    pulseInfo.start_time_seconds = payload.start_time_seconds;
    pulseInfo.score_ratio        = payload.score_ratio;
    pulseInfo.noise_psd          = payload.noise_psd;
    pulseInfo.detection_status   = payload.detection_status;
    pulseInfo.candidate_id       = payload.candidate_id;

    pulseInfo.latitude      = telemetry.position.latitude;
    pulseInfo.longitude     = telemetry.position.longitude;
    pulseInfo.altitude_rel  = telemetry.position.relativeAltitude;
    pulseInfo.roll_deg      = telemetry.attitudeEuler.rollDegrees;
    pulseInfo.pitch_deg     = telemetry.attitudeEuler.pitchDegrees;
    pulseInfo.yaw_deg       = telemetry.attitudeEuler.yawDegrees;

    if (payload.detection_status == kNoPulseDetectionStatus) {
        pulseInfo.confirmed_status = 0;
        pulseInfo.rate_state       = kRateStateA;
    } else {
        pulseInfo.predict_next_start_seconds = payload.predict_next_start_seconds;
        pulseInfo.snr                        = payload.snr;
        pulseInfo.signal_psd                 = payload.group_snr;
        pulseInfo.confirmed_status           = payload.confirmed_status;
        pulseInfo.rate_state                 = payload.rate_state;
    }

    return pulseInfo;
}
