// OperationProgressReporter: frame sequence, busy gate, 1 Hz re-send.

#include "OperationProgress.h"
#include "test_check.h"

#include <cstdio>
#include <string>
#include <vector>

using namespace TunnelProtocol;

namespace {

struct Sink {
    std::vector<OperationProgress_t> frames;
    std::vector<std::string>         log;
    OperationProgressReporter reporter {
        [this](const OperationProgress_t& f) { frames.push_back(f); },
        [this](const std::string& l) { log.push_back(l); } };
    const OperationProgress_t& last() const { return frames.back(); }
};

void testBeginUpdateFinishSequence()
{
    Sink s;
    CHECK(!s.reporter.busy());
    CHECK(s.reporter.busyMessage().empty());

    CHECK(s.reporter.begin(COMMAND_ID_RAW_CAPTURE, 42, "Raw capture", 13));
    CHECK(s.frames.size() == 1);
    CHECK(s.last().header.command == COMMAND_ID_OPERATION_PROGRESS);
    CHECK(s.last().header.request_id == 0);
    CHECK(s.last().command == COMMAND_ID_RAW_CAPTURE);
    CHECK(s.last().request_id == 42);
    CHECK(s.last().state == OPERATION_STATE_RUNNING);
    CHECK(s.last().step == 0 && s.last().step_count == 13);
    CHECK(std::string(s.last().message) == "Raw capture");
    CHECK(s.reporter.busy());
    CHECK(s.reporter.busyMessage() == "Busy: Raw capture in progress");

    s.reporter.update(5, "Capturing");
    CHECK(s.frames.size() == 2);
    CHECK(s.last().step == 5 && std::string(s.last().message) == "Capturing");

    s.reporter.update(5, "Capturing");                  // no change: nothing sent
    s.reporter.update(5);
    CHECK(s.frames.size() == 2);

    s.reporter.update(99);                              // clamped to step_count
    CHECK(s.last().step == 13);
    s.reporter.update(99);                              // same clamped value: nothing sent
    CHECK(s.frames.size() == 3);

    s.reporter.finish(true, "Capture complete");
    CHECK(s.frames.size() == 4);
    CHECK(s.last().state == OPERATION_STATE_COMPLETE);
    CHECK(s.last().step == 13);
    CHECK(std::string(s.last().message) == "Capture complete");
    CHECK(!s.reporter.busy());
    CHECK(s.reporter.busyMessage().empty());

    // Nothing running: update/finish are no-ops.
    s.reporter.update(1, "late");
    s.reporter.finish(false, "late");
    CHECK(s.frames.size() == 4);
}

void testFailureKeepsStep()
{
    Sink s;
    CHECK(s.reporter.begin(COMMAND_ID_STOP_DETECTION, 7, "Stopping detection", 4));
    s.reporter.update(2, "decimator");
    s.reporter.finish(false, "Detector process hung");
    CHECK(s.last().state == OPERATION_STATE_FAILED);
    CHECK(s.last().step == 2);
    CHECK(std::string(s.last().message) == "Detector process hung");
    CHECK(!s.reporter.busy());
}

void testBusyGate()
{
    Sink s;
    CHECK(s.reporter.begin(COMMAND_ID_SAVE_LOGS, 1, "Saving logs"));
    CHECK(!s.reporter.begin(COMMAND_ID_CLEAN_LOGS, 2, "Deleting logs"));
    CHECK(s.frames.size() == 1);                        // refused begin sends nothing
    CHECK(s.last().command == COMMAND_ID_SAVE_LOGS);
    CHECK(s.log.back().find("refused") != std::string::npos);
    CHECK(s.log.back().find("Busy: Saving logs in progress") != std::string::npos);

    s.reporter.finish(true);
    CHECK(s.reporter.begin(COMMAND_ID_CLEAN_LOGS, 2, "Deleting logs"));
    CHECK(s.last().command == COMMAND_ID_CLEAN_LOGS && s.last().request_id == 2);
}

void testIndeterminateThenCounted()
{
    Sink s;
    CHECK(s.reporter.begin(COMMAND_ID_SAVE_LOGS, 3, "Saving logs"));
    CHECK(s.last().step_count == 0);
    s.reporter.update(1, 250, "session/a.log");
    CHECK(s.last().step == 1 && s.last().step_count == 250);
    CHECK(std::string(s.last().message) == "session/a.log");
    s.reporter.finish(true, "Logs saved");
    CHECK(s.last().step == 250);                       // success fills the bar
}

void testResendWhileRunningAndBoundedAfterFinish()
{
    Sink s;
    s.reporter.resendIfRunning();
    CHECK(s.frames.empty());

    CHECK(s.reporter.begin(COMMAND_ID_START_DETECTION, 9, "Starting detection", 3));
    s.reporter.update(1, "airspyhf_zeromq_rx");
    const size_t logLines = s.log.size();
    s.reporter.resendIfRunning();
    CHECK(s.frames.size() == 3);
    CHECK(s.last().state == OPERATION_STATE_RUNNING && s.last().step == 1);
    CHECK(std::string(s.last().message) == "airspyhf_zeromq_rx");
    CHECK(s.log.size() == logLines);                   // re-sends are not logged

    // The terminal frame is re-sent for a few ticks so one lost frame cannot
    // leave the GCS showing RUNNING, then stops.
    s.reporter.finish(true, "Detection running");
    CHECK(s.frames.size() == 4);
    for (int i = 0; i < 3; ++i) {
        s.reporter.resendIfRunning();
        CHECK(s.last().state == OPERATION_STATE_COMPLETE);
    }
    CHECK(s.frames.size() == 7);
    s.reporter.resendIfRunning();
    CHECK(s.frames.size() == 7);
    CHECK(!s.reporter.busy());                          // re-sends do not hold the gate

    // A new operation cancels any remaining terminal re-sends.
    CHECK(s.reporter.begin(COMMAND_ID_SAVE_LOGS, 10, "Saving logs"));
    s.reporter.finish(false, "Log save failed");
    CHECK(s.reporter.begin(COMMAND_ID_CLEAN_LOGS, 11, "Deleting logs"));
    s.reporter.resendIfRunning();
    CHECK(s.last().command == COMMAND_ID_CLEAN_LOGS && s.last().state == OPERATION_STATE_RUNNING);
}

void testMessageTruncation()
{
    Sink s;
    const std::string longTitle(200, 'x');
    CHECK(s.reporter.begin(COMMAND_ID_CLEAN_LOGS, 1, longTitle));
    CHECK(s.last().message[sizeof(s.last().message) - 1] == '\0');
    CHECK(std::string(s.last().message).size() == sizeof(s.last().message) - 1);

    // The same over-length message again is not a change.
    s.reporter.update(0, longTitle);
    CHECK(s.frames.size() == 1);
    s.reporter.update(0, longTitle + "y");             // differs only past the cut: still no change
    CHECK(s.frames.size() == 1);
}

void testLogLineIsStable()
{
    Sink s;
    CHECK(s.reporter.begin(COMMAND_ID_RAW_CAPTURE, 5, "Raw capture", 13));
    CHECK(s.log.back() == "operation_progress command=8 request_id=5 state=1 step=0/13 msg=Raw capture");
}

} // namespace

int main()
{
    static_assert(sizeof(OperationProgress_t) <= MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN);
    testBeginUpdateFinishSequence();
    testFailureKeepsStep();
    testBusyGate();
    testIndeterminateThenCounted();
    testResendWhileRunningAndBoundedAfterFinish();
    testMessageTruncation();
    testLogLineIsStable();
    std::printf("test_operation_progress: all tests passed\n");
    return 0;
}
