// Unit tests for buildPythonPulseInfo: the detector TTDP payload -> GCS
// PythonPulseInfo_t field mapping, isolated from CommandHandler.

#include "PythonPulseMapper.h"
#include "test_check.h"

#include <cstdio>

using namespace TunnelProtocol;
using TagTrackerDetectorProtocol::Header;
using TagTrackerDetectorProtocol::MessageType;
using TagTrackerDetectorProtocol::PulsePayload;

namespace {

Header makeHeader(MessageType type, uint32_t collectionId, uint32_t sliceId, uint32_t tagId)
{
    Header header {};
    header.magic          = TagTrackerDetectorProtocol::kMagic;
    header.message_type   = static_cast<uint16_t>(type);
    header.payload_length = sizeof(PulsePayload);
    header.collection_id  = collectionId;
    header.slice_id       = sliceId;
    header.tag_id         = tagId;
    return header;
}

PulsePayload makeDetection()
{
    PulsePayload payload {};
    payload.frequency_hz               = 148515000;
    payload.group_seq_counter          = 42;
    payload.rate_state                 = kRateStateAToB;
    payload.detection_status           = kConfirmedDetectionStatus;
    payload.confirmed_status           = 1;
    payload.candidate_id               = 2;
    payload.start_time_seconds         = 1700000000.25;
    payload.predict_next_start_seconds = 1700000002.25;
    payload.snr                        = 17.5;
    payload.score_ratio                = 4.2;
    payload.group_snr                  = 3.5e-9;   // per-pulse signal PSD
    payload.noise_psd                  = 1.1e-11;
    return payload;
}

TelemetryCache::TelemetryCacheEntry_t makeTelemetry()
{
    TelemetryCache::TelemetryCacheEntry_t telemetry {};
    telemetry.timeInSeconds               = 1700000000.0;
    telemetry.position.latitude           = 47.6;
    telemetry.position.longitude          = -122.3;
    telemetry.position.relativeAltitude   = 55.0;
    telemetry.attitudeEuler.rollDegrees   = 1.0f;
    telemetry.attitudeEuler.pitchDegrees  = -2.0f;
    telemetry.attitudeEuler.yawDegrees    = 135.0f;
    return telemetry;
}

void testDetectionFieldsForwarded()
{
    const auto header    = makeHeader(MessageType::Pulse, 7, 3, 21);
    const auto payload   = makeDetection();
    const auto telemetry = makeTelemetry();

    const PythonPulseInfo_t info = buildPythonPulseInfo(header, payload, telemetry);

    CHECK(info.header.command == COMMAND_ID_PYTHON_PULSE);
    CHECK(info.collection_id == 7);
    CHECK(info.slice_id == 3);
    CHECK(info.tag_id == 21);
    CHECK(info.frequency_hz == 148515000);
    CHECK(info.cycle_counter == 42);
    CHECK(info.rate_state == kRateStateAToB);
    CHECK(info.candidate_id == 2);
    CHECK(info.detection_status == kConfirmedDetectionStatus);
    CHECK(info.confirmed_status == 1);
    CHECK(info.start_time_seconds == 1700000000.25);
    CHECK(info.predict_next_start_seconds == 1700000002.25);
    CHECK(info.snr == 17.5);
    CHECK(info.score_ratio == 4.2);
    CHECK(info.signal_psd == 3.5e-9);
    CHECK(info.noise_psd == 1.1e-11);

    CHECK(info.latitude == 47.6);
    CHECK(info.longitude == -122.3);
    CHECK(info.altitude_rel == 55.0);
    CHECK(info.roll_deg == 1.0f);
    CHECK(info.pitch_deg == -2.0f);
    CHECK(info.yaw_deg == 135.0f);
}

void testEachRateStateForwarded()
{
    const auto header    = makeHeader(MessageType::Pulse, 1, 1, 5);
    const auto telemetry = makeTelemetry();
    for (uint8_t state : {kRateStateA, kRateStateB, kRateStateAToB, kRateStateBToA}) {
        auto payload = makeDetection();
        payload.rate_state = state;
        CHECK(buildPythonPulseInfo(header, payload, telemetry).rate_state == state);
    }
}

void testNoDetectionZeroesPulseFields()
{
    const auto header    = makeHeader(MessageType::NoDetection, 7, 3, 21);
    const auto telemetry = makeTelemetry();
    auto payload = makeDetection();
    payload.detection_status = kNoPulseDetectionStatus;
    // A detector bug that leaves these populated must not leak to the GCS.
    payload.confirmed_status = 1;
    payload.rate_state       = kRateStateB;

    const PythonPulseInfo_t info = buildPythonPulseInfo(header, payload, telemetry);

    CHECK(info.detection_status == kNoPulseDetectionStatus);
    CHECK(info.confirmed_status == 0);
    CHECK(info.rate_state == kRateStateA);
    CHECK(info.predict_next_start_seconds == 0.0);
    CHECK(info.snr == 0.0);
    CHECK(info.signal_psd == 0.0);
    // Diagnostics still flow through so the GCS can show the noise floor.
    CHECK(info.score_ratio == 4.2);
    CHECK(info.noise_psd == 1.1e-11);
    CHECK(info.start_time_seconds == 1700000000.25);
    CHECK(info.cycle_counter == 42);
    CHECK(info.candidate_id == 2);
    CHECK(info.yaw_deg == 135.0f);
}

void testTelemetryIsCallerSupplied()
{
    // The slice-pose override lives in CommandHandler; the mapper must copy
    // whatever it is handed verbatim so that override actually reaches the GCS.
    const auto header  = makeHeader(MessageType::Pulse, 7, 3, 21);
    const auto payload = makeDetection();
    auto armPose = makeTelemetry();
    armPose.position.latitude          = 10.0;
    armPose.position.longitude         = 20.0;
    armPose.position.relativeAltitude  = 30.0;
    armPose.attitudeEuler.yawDegrees   = 270.0f;

    const PythonPulseInfo_t info = buildPythonPulseInfo(header, payload, armPose);

    CHECK(info.latitude == 10.0);
    CHECK(info.longitude == 20.0);
    CHECK(info.altitude_rel == 30.0);
    CHECK(info.yaw_deg == 270.0f);
}

void testCandidateZeroIsProvisionalLock()
{
    const auto header    = makeHeader(MessageType::Pulse, 1, 1, 5);
    const auto telemetry = makeTelemetry();
    auto payload = makeDetection();
    payload.candidate_id = 0;
    CHECK(buildPythonPulseInfo(header, payload, telemetry).candidate_id == 0);
}

} // namespace

int main()
{
    testDetectionFieldsForwarded();
    testEachRateStateForwarded();
    testNoDetectionZeroesPulseFields();
    testTelemetryIsCallerSupplied();
    testCandidateZeroIsProvisionalLock();
    std::printf("test_python_pulse_mapper: all tests passed\n");
    return 0;
}
