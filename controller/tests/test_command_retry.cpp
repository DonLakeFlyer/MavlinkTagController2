// End-to-end controller-side retry tests: GCS command sequences fed through
// TunnelCommandDispatcher with ACKs "lost" (the GCS re-sends the identical
// frame), against a fake CommandActions that only counts calls. No MAVLink,
// no processes. Every retry-handling path the controller has is exercised
// here: request-id replay, tag upload set integrity, in-flight start/stop
// idempotency, GCS restart and controller restart.

#include "TunnelCommandDispatcher.h"
#include "test_check.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace TunnelProtocol;
using DState = DetectionCoordinator::State;

namespace {

struct FakeActions : CommandActions {
    int          startCalls = 0;
    int          stopCalls = 0;
    int          rawCaptureCalls = 0;
    int          saveLogsCalls = 0;
    int          cleanLogsCalls = 0;
    int          airspyStatusCalls = 0;
    int          startCollectionCalls = 0;
    int          sliceCalls = 0;
    int          finishCalls = 0;
    bool         capture = false;
    std::string  startError;            // returned by startDetectionPipeline
    std::string  collectionError;
    StartDetectionInfo_t lastStart {};

    std::string startDetectionPipeline(const StartDetectionInfo_t& info) override { ++startCalls; lastStart = info; return startError; }
    void        stopDetectionPipeline() override { ++stopCalls; }
    std::string detectionLogDir() override { return "/logs/det"; }
    bool        captureInProgress() override { return capture; }
    std::string rawCapture(const mavlink_tunnel_t&) override { ++rawCaptureCalls; return ""; }
    bool        saveLogs() override { ++saveLogsCalls; return true; }
    bool        cleanLogs() override { ++cleanLogsCalls; return true; }
    std::string airspyStatus() override { ++airspyStatusCalls; return ""; }
    std::string startCollection(const mavlink_tunnel_t&) override { ++startCollectionCalls; return collectionError; }
    std::string startCollectionSlice(const mavlink_tunnel_t&) override { ++sliceCalls; return ""; }
    std::string finishCollection(const mavlink_tunnel_t&) override { ++finishCalls; return ""; }
};

struct VectorLog : CommandLog {
    std::vector<std::string> lines;
    void debug(const std::string& m) override { lines.push_back("D " + m); }
    void info(const std::string& m) override  { lines.push_back("I " + m); }
    void error(const std::string& m) override { lines.push_back("E " + m); }
    bool contains(const char* needle) const
    {
        for (const auto& l : lines) if (l.find(needle) != std::string::npos) return true;
        return false;
    }
};

struct Gcs {
    FakeActions           actions;
    VectorLog             log;
    std::vector<uint16_t> heartbeats;
    TunnelCommandDispatcher dispatcher { actions, log, [this](uint16_t s) { heartbeats.push_back(s); } };
    uint32_t              nextRequestId = 1000;

    template <typename T>
    static mavlink_tunnel_t frame(const T& payload)
    {
        mavlink_tunnel_t t {};
        t.payload_length = sizeof(payload);
        memcpy(t.payload, &payload, sizeof(payload));
        return t;
    }

    template <typename T>
    AckInfo_t send(T payload, uint32_t command)
    {
        payload.header.command    = command;
        payload.header.request_id = nextRequestId++;
        return dispatcher.handle(frame(payload));
    }

    // Same bytes again: what SendTunnelCommandState does after its ACK timer fires.
    AckInfo_t resend(const mavlink_tunnel_t& t) { return dispatcher.handle(t); }

    template <typename T>
    mavlink_tunnel_t prepare(T payload, uint32_t command)
    {
        payload.header.command    = command;
        payload.header.request_id = nextRequestId++;
        return frame(payload);
    }

    uint16_t heartbeat() const { return dispatcher.detection().heartbeatStatus(); }
};

TagInfo_t makeTag(uint32_t id, uint32_t index, uint32_t uploadId)
{
    TagInfo_t tag {};
    tag.upload_id = uploadId;
    tag.tag_index = index;
    tag.id = id;
    tag.frequency_hz = 147970000;
    tag.pulse_width_msecs = 19;
    tag.intra_pulse1_msecs = 1500;
    tag.intra_pulse2_msecs = 2000;
    tag.intra_pulse_uncertainty_msecs = 60;
    tag.intra_pulse_jitter_msecs = 20;
    tag.k = 20;
    tag.false_alarm_probability = 0.05;
    tag.channelizer_channel_number = 1;
    tag.channelizer_channel_center_frequency_hz = 147970000;
    tag.ip1_mu = tag.ip1_sigma = tag.ip2_mu = tag.ip2_sigma = std::nan("");
    return tag;
}

bool ok(const AckInfo_t& ack)   { return ack.result == COMMAND_RESULT_SUCCESS; }
bool nack(const AckInfo_t& ack) { return ack.result == COMMAND_RESULT_FAILURE; }
bool sameAck(const AckInfo_t& a, const AckInfo_t& b)
{
    return a.command == b.command && a.request_id == b.request_id && a.result == b.result
        && strncmp(a.message, b.message, sizeof(a.message)) == 0;
}

void uploadTags(Gcs& g, uint32_t uploadId, std::vector<uint32_t> ids)
{
    StartTagsInfo_t s {}; s.upload_id = uploadId; s.tag_count = static_cast<uint32_t>(ids.size());
    CHECK(ok(g.send(s, COMMAND_ID_START_TAGS)));
    for (uint32_t i = 0; i < ids.size(); ++i) {
        CHECK(ok(g.send(makeTag(ids[i], i, uploadId), COMMAND_ID_TAG)));
    }
    EndTagsInfo_t e {}; e.upload_id = uploadId; e.tag_count = static_cast<uint32_t>(ids.size());
    CHECK(ok(g.send(e, COMMAND_ID_END_TAGS)));
}

void testAckCarriesRequestIdAndCommand()
{
    Gcs g;
    StartTagsInfo_t s {}; s.upload_id = 1; s.tag_count = 0;
    const auto t = g.prepare(s, COMMAND_ID_START_TAGS);
    HeaderInfo_t h; memcpy(&h, t.payload, sizeof(h));
    const auto ack = g.resend(t);
    CHECK(ack.header.command == COMMAND_ID_ACK);
    CHECK(ack.header.request_id == 0);
    CHECK(ack.command == COMMAND_ID_START_TAGS);
    CHECK(ack.request_id == h.request_id);
    CHECK(ok(ack));
}

// Every TAG's ACK lost once: each retry is replayed, the list has one entry per tag.
void testTagUploadWithEveryAckLost()
{
    Gcs g;
    const uint32_t up = 11;
    StartTagsInfo_t s {}; s.upload_id = up; s.tag_count = 2;
    auto t = g.prepare(s, COMMAND_ID_START_TAGS);
    auto a1 = g.resend(t); auto a2 = g.resend(t);
    CHECK(ok(a1) && sameAck(a1, a2));

    for (uint32_t i = 0; i < 2; ++i) {
        t = g.prepare(makeTag(2 + 2 * i, i, up), COMMAND_ID_TAG);
        a1 = g.resend(t); a2 = g.resend(t);
        CHECK(ok(a1) && sameAck(a1, a2));
    }
    EndTagsInfo_t e {}; e.upload_id = up; e.tag_count = 2;
    t = g.prepare(e, COMMAND_ID_END_TAGS);
    a1 = g.resend(t); a2 = g.resend(t); auto a3 = g.resend(t);
    CHECK(ok(a1) && sameAck(a1, a2) && sameAck(a1, a3));

    CHECK(g.dispatcher.tags().size() == 2);
    CHECK(g.heartbeat() == HEARTBEAT_STATUS_HAS_TAGS);
    CHECK((g.heartbeats == std::vector<uint16_t>{HEARTBEAT_STATUS_HAS_TAGS}));
    CHECK(g.log.contains("Replaying ack for retried TAG"));
}

// A TAG itself lost (never reached the controller): END_TAGS NACKs with the
// gap, the GCS resends that TAG, END_TAGS then succeeds.
void testLostTagIsReportedByEndTags()
{
    Gcs g;
    const uint32_t up = 12;
    StartTagsInfo_t s {}; s.upload_id = up; s.tag_count = 3;
    CHECK(ok(g.send(s, COMMAND_ID_START_TAGS)));
    CHECK(ok(g.send(makeTag(2, 0, up), COMMAND_ID_TAG)));
    // index 1 lost in flight
    CHECK(ok(g.send(makeTag(6, 2, up), COMMAND_ID_TAG)));
    EndTagsInfo_t e {}; e.upload_id = up; e.tag_count = 3;
    const auto bad = g.send(e, COMMAND_ID_END_TAGS);
    CHECK(nack(bad));
    CHECK(std::string(bad.message) == "incomplete: missing 1");
    CHECK(g.heartbeat() == HEARTBEAT_STATUS_IDLE);

    CHECK(ok(g.send(makeTag(4, 1, up), COMMAND_ID_TAG)));
    CHECK(ok(g.send(e, COMMAND_ID_END_TAGS)));          // new request id, fresh evaluation
    CHECK(g.dispatcher.tags().size() == 3);
    CHECK(g.heartbeat() == HEARTBEAT_STATUS_HAS_TAGS);
}

// GCS restarts mid-upload and starts over with a new upload_id. Late frames
// from the abandoned set are refused; the new set completes normally.
void testGcsRestartMidUpload()
{
    Gcs g;
    StartTagsInfo_t s {}; s.upload_id = 20; s.tag_count = 2;
    CHECK(ok(g.send(s, COMMAND_ID_START_TAGS)));
    CHECK(ok(g.send(makeTag(2, 0, 20), COMMAND_ID_TAG)));

    // New GCS process: different request-id range, different upload_id.
    g.nextRequestId = 0x7f000000;
    uploadTags(g, 21, {2, 4});
    CHECK(g.dispatcher.tags().size() == 2);

    // Straggler from the old set.
    const auto stale = g.send(makeTag(4, 1, 20), COMMAND_ID_TAG);
    CHECK(nack(stale));
    CHECK(std::string(stale.message) == "TAG outside START_TAGS/END_TAGS");
    CHECK(g.dispatcher.tags().size() == 2);
}

void testStartDetectionRetryWhileStarting()
{
    Gcs g;
    uploadTags(g, 30, {2});
    StartDetectionInfo_t sd {}; sd.radio_center_frequency_hz = 147970000;
    const auto t = g.prepare(sd, COMMAND_ID_START_DETECTION);
    const auto a1 = g.resend(t);
    CHECK(ok(a1));
    CHECK(std::string(a1.message) == "/logs/det");
    CHECK(g.actions.startCalls == 1);
    CHECK(g.dispatcher.detection().state() == DState::Starting);

    // Retry with the same id: replayed from cache, pipeline not touched.
    const auto a2 = g.resend(t);
    CHECK(sameAck(a1, a2));
    CHECK(g.actions.startCalls == 1);

    // GCS restarted: new id, start still in flight -> intent satisfied (option A).
    g.nextRequestId = 0x5000;
    const auto a3 = g.send(sd, COMMAND_ID_START_DETECTION);
    CHECK(ok(a3));
    CHECK(std::string(a3.message) == "/logs/det");
    CHECK(g.actions.startCalls == 1);
    CHECK(g.log.contains("start in progress; treating as satisfied"));

    g.dispatcher.detection().startFinished(true);
    CHECK(g.heartbeat() == HEARTBEAT_STATUS_DETECTING);

    // Once detecting, a genuinely new START is an error, not a duplicate launch.
    const auto a4 = g.send(sd, COMMAND_ID_START_DETECTION);
    CHECK(nack(a4));
    CHECK(std::string(a4.message) == "Detection already running");
    CHECK(g.actions.startCalls == 1);
}

void testStartDetectionPipelineFailureIsRetryable()
{
    Gcs g;
    uploadTags(g, 31, {2});
    g.actions.startError = "AirSpy detection failed: none";
    StartDetectionInfo_t sd {};
    const auto a1 = g.send(sd, COMMAND_ID_START_DETECTION);
    CHECK(nack(a1));
    CHECK(std::string(a1.message) == "AirSpy detection failed: none");
    CHECK(g.dispatcher.detection().state() == DState::HasTags);

    g.actions.startError.clear();
    CHECK(ok(g.send(sd, COMMAND_ID_START_DETECTION)));
    CHECK(g.actions.startCalls == 2);
}

void testStopDetectionRetryWhileStopping()
{
    Gcs g;
    uploadTags(g, 40, {2});
    StartDetectionInfo_t sd {};
    CHECK(ok(g.send(sd, COMMAND_ID_START_DETECTION)));
    g.dispatcher.detection().startFinished(true);

    StopDetectionInfo_t stop {};
    const auto t = g.prepare(stop, COMMAND_ID_STOP_DETECTION);
    const auto a1 = g.resend(t);
    CHECK(ok(a1));
    CHECK(g.actions.stopCalls == 1);
    CHECK(g.dispatcher.detection().state() == DState::Stopping);
    CHECK(g.heartbeat() == HEARTBEAT_STATUS_DETECTING);     // still gating until teardown ends

    CHECK(sameAck(a1, g.resend(t)));                        // replay
    CHECK(g.actions.stopCalls == 1);

    g.nextRequestId = 0x6000;                               // GCS restart
    CHECK(ok(g.send(stop, COMMAND_ID_STOP_DETECTION)));     // option A: satisfied
    CHECK(g.actions.stopCalls == 1);
    CHECK(g.log.contains("stop in progress; treating as satisfied"));

    g.dispatcher.detection().stopFinished();
    CHECK(g.heartbeat() == HEARTBEAT_STATUS_HAS_TAGS);

    // After teardown a new STOP is a real error; the earlier retry ids still replay success.
    const auto late = g.send(stop, COMMAND_ID_STOP_DETECTION);
    CHECK(nack(late));
    CHECK(std::string(late.message) == "Not detecting");
    CHECK(ok(g.resend(t)));
}

void testStopWhileStartingIsRefused()
{
    Gcs g;
    uploadTags(g, 41, {2});
    StartDetectionInfo_t sd {};
    CHECK(ok(g.send(sd, COMMAND_ID_START_DETECTION)));
    StopDetectionInfo_t stop {};
    const auto a = g.send(stop, COMMAND_ID_STOP_DETECTION);
    CHECK(nack(a));
    CHECK(std::string(a.message) == "Detection start in progress; retry");
    CHECK(g.actions.stopCalls == 0);
}

void testTagUploadRefusedWhileDetectingOrCapturing()
{
    Gcs g;
    uploadTags(g, 50, {2});
    StartDetectionInfo_t sd {};
    CHECK(ok(g.send(sd, COMMAND_ID_START_DETECTION)));
    g.dispatcher.detection().startFinished(true);

    StartTagsInfo_t s {}; s.upload_id = 51; s.tag_count = 1;
    const auto a = g.send(s, COMMAND_ID_START_TAGS);
    CHECK(nack(a));
    CHECK(std::string(a.message) == "Controller in incorrect state");
    CHECK(g.dispatcher.tags().size() == 1);

    g.dispatcher.stopDetection();
    g.dispatcher.detection().stopFinished();
    g.actions.capture = true;
    CHECK(nack(g.send(s, COMMAND_ID_START_TAGS)));
    g.actions.capture = false;
    CHECK(ok(g.send(s, COMMAND_ID_START_TAGS)));
}

void testRequestIdReuseWithDifferentCommandIsRejected()
{
    Gcs g;
    StartTagsInfo_t s {}; s.upload_id = 60; s.tag_count = 0;
    auto t = g.prepare(s, COMMAND_ID_START_TAGS);
    CHECK(ok(g.resend(t)));

    EndTagsInfo_t e {}; e.upload_id = 60; e.tag_count = 0;
    e.header.command = COMMAND_ID_END_TAGS;
    HeaderInfo_t h; memcpy(&h, t.payload, sizeof(h));
    e.header.request_id = h.request_id;                     // reused id
    const auto a = g.resend(Gcs::frame(e));
    CHECK(nack(a));
    CHECK(std::string(a.message) == "request_id reuse");
    CHECK(g.dispatcher.tagUpload().state() == TagUploadCoordinator::State::Receiving);   // END not applied
    CHECK(g.log.contains("reused by END_TAGS after START_TAGS"));
}

void testPassThroughCommandsAreDeduped()
{
    Gcs g;
    uploadTags(g, 70, {2});
    StopDetectionInfo_t bare {};   // header-only payloads share this shape

    auto t = g.prepare(bare, COMMAND_ID_SAVE_LOGS);
    CHECK(ok(g.resend(t))); CHECK(ok(g.resend(t)));
    CHECK(g.actions.saveLogsCalls == 1);

    t = g.prepare(bare, COMMAND_ID_CLEAN_LOGS);
    CHECK(ok(g.resend(t))); CHECK(ok(g.resend(t)));
    CHECK(g.actions.cleanLogsCalls == 1);

    t = g.prepare(bare, COMMAND_ID_AIRSPY_STATUS);
    CHECK(ok(g.resend(t))); CHECK(ok(g.resend(t)));
    CHECK(g.actions.airspyStatusCalls == 1);

    RawCaptureInfo_t rc {};
    t = g.prepare(rc, COMMAND_ID_RAW_CAPTURE);
    CHECK(ok(g.resend(t))); CHECK(ok(g.resend(t)));
    CHECK(g.actions.rawCaptureCalls == 1);

    StartCollection_t sc {}; sc.collection_id = 1;
    t = g.prepare(sc, COMMAND_ID_START_COLLECTION);
    auto a1 = g.resend(t); auto a2 = g.resend(t);
    CHECK(ok(a1) && sameAck(a1, a2));
    CHECK(std::string(a1.message) == "/logs/det");
    CHECK(g.actions.startCollectionCalls == 1);

    StartCollectionSlice_t sl {}; sl.collection_id = 1; sl.slice_id = 1;
    t = g.prepare(sl, COMMAND_ID_START_COLLECTION_SLICE);
    CHECK(ok(g.resend(t))); CHECK(ok(g.resend(t)));
    CHECK(g.actions.sliceCalls == 1);

    FinishCollection_t fc {}; fc.collection_id = 1;
    t = g.prepare(fc, COMMAND_ID_FINISH_COLLECTION);
    CHECK(ok(g.resend(t))); CHECK(ok(g.resend(t)));
    CHECK(g.actions.finishCalls == 1);

    // A failure is replayed as a failure, not re-tried by the controller.
    g.actions.collectionError = "Another collection is active";
    t = g.prepare(sc, COMMAND_ID_START_COLLECTION);
    a1 = g.resend(t); a2 = g.resend(t);
    CHECK(nack(a1) && sameAck(a1, a2));
    CHECK(g.actions.startCollectionCalls == 2);
}

void testMalformedFrames()
{
    Gcs g;
    mavlink_tunnel_t t {};
    t.payload_length = 2;                                   // shorter than the header
    auto a = g.dispatcher.handle(t);
    CHECK(nack(a) && a.request_id == 0);

    StartTagsInfo_t s {}; s.header.command = COMMAND_ID_START_TAGS; s.header.request_id = 5;
    t = Gcs::frame(s);
    t.payload_length = sizeof(HeaderInfo_t);                // header only, body missing
    a = g.dispatcher.handle(t);
    CHECK(nack(a) && a.request_id == 5);
    CHECK(std::string(a.message) == "Payload length incorrect");
    CHECK(g.dispatcher.tagUpload().state() == TagUploadCoordinator::State::Idle);

    HeaderInfo_t h {}; h.command = 999; h.request_id = 6;
    a = g.dispatcher.handle(Gcs::frame(h));
    CHECK(nack(a) && std::string(a.message) == "Unknown command");
    // Its retry is replayed too.
    CHECK(sameAck(a, g.dispatcher.handle(Gcs::frame(h))));
}

// Controller restarted between the GCS's send and its retry: the cache is
// gone, so the retry runs for real. The state machines make that harmless.
void testControllerRestartFallsBackToStateMachines()
{
    Gcs g;
    StartTagsInfo_t s {}; s.upload_id = 80; s.tag_count = 1;
    const auto tStart = g.prepare(s, COMMAND_ID_START_TAGS);
    CHECK(ok(g.resend(tStart)));
    const auto tTag = g.prepare(makeTag(2, 0, 80), COMMAND_ID_TAG);
    CHECK(ok(g.resend(tTag)));

    // "Reboot": fresh dispatcher, same GCS frames retried.
    Gcs g2;
    g2.nextRequestId = g.nextRequestId;
    CHECK(ok(g2.resend(tStart)));
    CHECK(ok(g2.resend(tTag)));
    CHECK(ok(g2.resend(tTag)));                             // semantic Retransmit path
    EndTagsInfo_t e {}; e.upload_id = 80; e.tag_count = 1;
    CHECK(ok(g2.send(e, COMMAND_ID_END_TAGS)));
    CHECK(g2.dispatcher.tags().size() == 1);
}

void testLegacyRequestIdZeroIsNeverDeduped()
{
    Gcs g;
    StartTagsInfo_t s {}; s.upload_id = 90; s.tag_count = 0;
    s.header.command = COMMAND_ID_START_TAGS;              // request_id 0
    CHECK(ok(g.dispatcher.handle(Gcs::frame(s))));
    EndTagsInfo_t e {}; e.upload_id = 90; e.tag_count = 0; e.header.command = COMMAND_ID_END_TAGS;
    CHECK(ok(g.dispatcher.handle(Gcs::frame(e))));
    CHECK(g.dispatcher.requestCache().size() == 0);
}

} // namespace

int main()
{
    testAckCarriesRequestIdAndCommand();
    testTagUploadWithEveryAckLost();
    testLostTagIsReportedByEndTags();
    testGcsRestartMidUpload();
    testStartDetectionRetryWhileStarting();
    testStartDetectionPipelineFailureIsRetryable();
    testStopDetectionRetryWhileStopping();
    testStopWhileStartingIsRefused();
    testTagUploadRefusedWhileDetectingOrCapturing();
    testRequestIdReuseWithDifferentCommandIsRejected();
    testPassThroughCommandsAreDeduped();
    testMalformedFrames();
    testControllerRestartFallsBackToStateMachines();
    testLegacyRequestIdZeroIsNeverDeduped();
    std::printf("test_command_retry: all tests passed\n");
    return 0;
}
