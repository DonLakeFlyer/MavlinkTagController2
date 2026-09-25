// TunnelProtocolLog::describe() decodes every wire struct for the log; a
// field/format mismatch here would silently corrupt every flight log line.

#include "TunnelProtocol.h"
#include "TunnelProtocolLog.h"
#include "test_check.h"

#include <cstring>
#include <string>

using namespace TunnelProtocol;

namespace {

bool contains(const std::string& text, const char* needle)
{
    return text.find(needle) != std::string::npos;
}

template <typename T>
std::string describeStruct(const T& message, const char* verb = "received")
{
    return TunnelProtocolLog::describe(verb, &message, sizeof(message));
}

void testNames()
{
    CHECK(TunnelProtocolLog::commandName(COMMAND_ID_SET_LOG_LEVEL) == "SET_LOG_LEVEL");
    CHECK(TunnelProtocolLog::commandName(COMMAND_ID_DETECTOR_HEARTBEAT) == "DETECTOR_HEARTBEAT");
    CHECK(TunnelProtocolLog::commandName(0xFFFF) == "UNKNOWN(65535)");
    CHECK(TunnelProtocolLog::logLevelName(LOG_LEVEL_DEBUG) == "DEBUG");
    CHECK(TunnelProtocolLog::logLevelName(LOG_LEVEL_VERBOSE) == "VERBOSE");
    CHECK(TunnelProtocolLog::logLevelName(7) == "UNKNOWN(7)");
    CHECK(TunnelProtocolLog::detectionModeName(DETECTION_MODE_PYTHON) == "PYTHON");
    CHECK(TunnelProtocolLog::collectionStatusName(COLLECTION_STATUS_REVISIT_REQUESTED) == "REVISIT_REQUESTED");
    CHECK(TunnelProtocolLog::heartbeatStatusName(HEARTBEAT_STATUS_DETECTING) == "DETECTING");
}

void testHeaderTooSmall()
{
    uint8_t bytes[3] = {};
    const std::string text = TunnelProtocolLog::describe("received", bytes, sizeof(bytes));
    CHECK(contains(text, "UNKNOWN received"));
    CHECK(contains(text, "too small"));
}

void testWrongLengthFallsBackToPayloadLength()
{
    StartCollectionSlice_t m {};
    m.header.command = COMMAND_ID_START_COLLECTION_SLICE;
    m.header.request_id = 9;
    const std::string text = TunnelProtocolLog::describe("received", &m, sizeof(m) - 1);
    CHECK(contains(text, "START_COLLECTION_SLICE received: request_id:9"));
    CHECK(contains(text, "payload_length:"));
    CHECK(!contains(text, "heading_deg"));
}

void testAck()
{
    AckInfo_t m {};
    m.header.command = COMMAND_ID_ACK;
    m.header.request_id = 0;
    m.command = COMMAND_ID_START_COLLECTION;
    m.request_id = 42;
    m.result = COMMAND_RESULT_FAILURE;
    std::strncpy(m.message, "Busy", sizeof(m.message) - 1);
    const std::string text = describeStruct(m, "sent");
    CHECK(contains(text, "ACK sent: request_id:0 command:START_COLLECTION request_id:42 result:FAILURE message:Busy"));

    m.message[0] = '\0';
    CHECK(!contains(describeStruct(m), "message:"));
}

void testAckMessageNotNulTerminated()
{
    AckInfo_t m {};
    m.header.command = COMMAND_ID_ACK;
    std::memset(m.message, 'x', sizeof(m.message));
    const std::string text = describeStruct(m);
    CHECK(text.size() < 512);   // describe() terminates the buffer itself
    CHECK(contains(text, "message:x"));
}

void testStartCollectionAndSlice()
{
    StartCollection_t c {};
    c.header.command = COMMAND_ID_START_COLLECTION;
    c.header.request_id = 5;
    c.collection_id = 123;
    c.radio_center_frequency_hz = 146000000;
    c.n_slices = 8;
    c.detection_margin = 1.0f;
    c.confidence_ratio = 1.3f;
    c.antenna_id = ANTENNA_ID_RA23K;
    std::string text = describeStruct(c);
    CHECK(contains(text, "START_COLLECTION received: request_id:5 collection_id:123 radio_center_frequency_hz:146000000 n_slices:8"));
    CHECK(contains(text, "antenna_id:1"));

    StartCollectionSlice_t s {};
    s.header.command = COMMAND_ID_START_COLLECTION_SLICE;
    s.collection_id = 123;
    s.slice_id = 3;
    s.heading_deg = 135.0f;
    text = describeStruct(s);
    CHECK(contains(text, "collection_id:123 slice_id:3 heading_deg:135.0"));

    FinishCollection_t f {};
    f.header.command = COMMAND_ID_FINISH_COLLECTION;
    f.collection_id = 123;
    f.disposition = COLLECTION_FINISH_CANCEL;
    text = describeStruct(f);
    CHECK(contains(text, "FINISH_COLLECTION received: request_id:0 collection_id:123 disposition:CANCEL"));
}

void testCollectionStatusAndBearing()
{
    CollectionStatus_t s {};
    s.header.command = COMMAND_ID_COLLECTION_STATUS;
    s.collection_id = 7;
    s.slice_id = 2;
    s.status = COLLECTION_STATUS_SLICE_COMPLETE;
    s.expected_detectors = 1;
    s.completed_detectors = 1;
    s.revisit_heading_deg = 270.0f;
    std::string text = describeStruct(s, "sent");
    CHECK(contains(text, "COLLECTION_STATUS sent: request_id:0 collection_id:7 slice_id:2 status:SLICE_COMPLETE expected_detectors:1 completed_detectors:1 error_code:0 revisit_heading_deg:270.0"));

    BearingResult_t b {};
    b.header.command = COMMAND_ID_BEARING_RESULT;
    b.collection_id = 7;
    b.tag_id = 2;
    b.bearing_deg = 134.75f;
    b.r_squared = 0.975f;
    b.n_valid_slices = 8;
    b.best_snr = 74.6f;
    b.confirmed = 1;
    text = describeStruct(b, "sent");
    CHECK(contains(text, "tag_id:2 bearing_deg:134.8 r_squared:0.975 n_valid_slices:8 best_snr:74.6 confirmed:1"));
}

void testOperationProgressAndHeartbeats()
{
    OperationProgress_t p {};
    p.header.command = COMMAND_ID_OPERATION_PROGRESS;
    p.command = COMMAND_ID_START_COLLECTION;
    p.request_id = 11;
    p.state = OPERATION_STATE_RUNNING;
    p.step = 3;
    p.step_count = 10;
    std::strncpy(p.message, "1/8 000 deg", sizeof(p.message) - 1);
    std::string text = describeStruct(p, "sent");
    CHECK(contains(text, "OPERATION_PROGRESS sent: request_id:0 command:START_COLLECTION request_id:11 state:RUNNING step:3/10 message:1/8 000 deg"));

    Heartbeat_t h {};
    h.header.command = COMMAND_ID_HEARTBEAT;
    h.protocol_version = TUNNEL_PROTOCOL_VERSION;
    h.system_id = HEARTBEAT_SYSTEM_ID_MAVLINKCONTROLLER;
    h.status = HEARTBEAT_STATUS_HAS_TAGS;
    h.cpu_temp_c = 51.25f;
    text = describeStruct(h, "sent");
    CHECK(contains(text, "protocol_version:7"));
    CHECK(contains(text, "status:HAS_TAGS cpu_temp_c:51.2"));

    DetectorHeartbeat_t d {};
    d.header.command = COMMAND_ID_DETECTOR_HEARTBEAT;
    d.tag_id = 4;
    d.detection_mode = DETECTION_MODE_UAVRT;
    text = describeStruct(d, "sent");
    CHECK(text == "DETECTOR_HEARTBEAT sent: request_id:0 tag_id:4 detection_mode:UAVRT");

    SetLogLevel_t l {};
    l.header.command = COMMAND_ID_SET_LOG_LEVEL;
    l.header.request_id = 3;
    l.level = LOG_LEVEL_VERBOSE;
    text = describeStruct(l);
    CHECK(text == "SET_LOG_LEVEL received: request_id:3 level:VERBOSE");
}

void testPulses()
{
    PulseInfo_t u {};
    u.header.command = COMMAND_ID_PULSE;
    u.tag_id = 2;
    u.frequency_hz = 146000000;
    u.detection_status = 2;
    u.confirmed_status = 1;
    u.snr = 12.5f;
    u.group_seq_counter = 9;
    u.group_ind = 1;
    u.noise_psd = 1e-10f;
    std::string text = describeStruct(u, "sent");
    CHECK(contains(text, "PULSE sent: request_id:0 tag_id:2 frequency_hz:146000000 detection_status:2 confirmed_status:1 snr:12.5 group_seq_counter:9 group_ind:1 noise_psd:1e-10"));

    PythonPulseInfo_t p {};
    p.header.command = COMMAND_ID_PYTHON_PULSE;
    p.collection_id = 7;
    p.slice_id = 2;
    p.tag_id = 2;
    p.frequency_hz = 146000993;
    p.cycle_counter = 8;
    p.detection_status = 2;
    p.confirmed_status = 1;
    p.rate_state = 0;
    p.candidate_id = 1;
    p.snr = 28.6f;
    p.score_ratio = 8.709f;
    text = describeStruct(p, "sent");
    CHECK(contains(text, "PYTHON_PULSE sent: request_id:0 collection_id:7 slice_id:2 tag_id:2 frequency_hz:146000993 cycle_counter:8"));
    CHECK(contains(text, "candidate_id:1 snr:28.6 score_ratio:8.709"));
}

void testHeaderOnlyCommands()
{
    HeaderInfo_t h {};
    h.command = COMMAND_ID_STOP_DETECTION;
    h.request_id = 77;
    CHECK(describeStruct(h) == "STOP_DETECTION received: request_id:77");

    h.command = COMMAND_ID_SAVE_LOGS;
    CHECK(describeStruct(h) == "SAVE_LOGS received: request_id:77");

    // Extra bytes on a header-only command are reported, not ignored.
    uint8_t padded[sizeof(HeaderInfo_t) + 4] = {};
    std::memcpy(padded, &h, sizeof(h));
    const std::string text = TunnelProtocolLog::describe("received", padded, sizeof(padded));
    CHECK(contains(text, "payload_length:12"));
}

void testTagUploadStartDetectionAndRawCapture()
{
    StartTagsInfo_t st {};
    st.header.command = COMMAND_ID_START_TAGS;
    st.header.request_id = 11;
    st.upload_id = 777;
    st.tag_count = 3;
    CHECK(describeStruct(st) == "START_TAGS received: request_id:11 upload_id:777 tag_count:3");

    EndTagsInfo_t et {};
    et.header.command = COMMAND_ID_END_TAGS;
    et.upload_id = 777;
    et.tag_count = 3;
    CHECK(describeStruct(et) == "END_TAGS received: request_id:0 upload_id:777 tag_count:3");

    TagInfo_t t {};
    t.header.command = COMMAND_ID_TAG;
    t.upload_id = 777;
    t.tag_index = 2;
    t.id = 4;
    t.frequency_hz = 146000000;
    t.pulse_width_msecs = 15;
    t.intra_pulse1_msecs = 1333;
    t.intra_pulse2_msecs = 2000;
    t.intra_pulse_uncertainty_msecs = 60;
    t.intra_pulse_jitter_msecs = 20;
    t.k = 20;
    t.false_alarm_probability = 0.05;
    t.channelizer_channel_number = 6;
    t.channelizer_channel_center_frequency_hz = 146001000;
    std::string text = describeStruct(t);
    CHECK(contains(text, "TAG received: request_id:0 upload_id:777 tag_index:2 id:4 frequency_hz:146000000 pulse_width_msecs:15"));
    CHECK(contains(text, " intra_pulse1_msecs:1333 intra_pulse2_msecs:2000 intra_pulse_uncertainty_msecs:60 intra_pulse_jitter_msecs:20"));
    CHECK(contains(text, " k:20 false_alarm_probability:0.05 channelizer_channel_number:6 channelizer_channel_center_frequency_hz:146001000"));

    StartDetectionInfo_t sd {};
    sd.header.command = COMMAND_ID_START_DETECTION;
    sd.header.request_id = 12;
    sd.radio_center_frequency_hz = 146500000;
    sd.gain = 18;
    sd.detection_mode = DETECTION_MODE_PYTHON;
    sd.detection_margin = 0.9;
    sd.confidence_ratio = 1.3;
    sd.debug_detector = 1;
    sd.dump_spectrogram = 0;
    text = describeStruct(sd);
    CHECK(contains(text, "START_DETECTION received: request_id:12 radio_center_frequency_hz:146500000 gain:18 detection_mode:PYTHON"));
    CHECK(contains(text, " detection_margin:0.9 confidence_ratio:1.3 debug_detector:1 dump_spectrogram:0"));

    RawCaptureInfo_t rc {};
    rc.header.command = COMMAND_ID_RAW_CAPTURE;
    rc.gain = 21;
    rc.frequency_hz = 146250000;
    CHECK(describeStruct(rc) == "RAW_CAPTURE received: request_id:0 gain:21 frequency_hz:146250000");
}

} // namespace

int main()
{
    testNames();
    testHeaderTooSmall();
    testWrongLengthFallsBackToPayloadLength();
    testAck();
    testAckMessageNotNulTerminated();
    testStartCollectionAndSlice();
    testCollectionStatusAndBearing();
    testOperationProgressAndHeartbeats();
    testPulses();
    testHeaderOnlyCommands();
    testTagUploadStartDetectionAndRawCapture();
    return 0;
}
