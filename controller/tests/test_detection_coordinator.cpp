// Unit tests for DetectionCoordinator: the tag-list / detection lifecycle
// that gates START_DETECTION and STOP_DETECTION, including the in-flight
// cases (Starting, Stopping) a GCS retry or restart can land in.

#include "DetectionCoordinator.h"
#include "TunnelProtocol.h"
#include "test_check.h"

#include <chrono>
#include <cstdio>
#include <thread>
#include <vector>

using Result = DetectionCoordinator::Result;
using State  = DetectionCoordinator::State;

namespace {

struct Harness {
    std::vector<uint16_t> heartbeats;
    DetectionCoordinator  coord { [this](uint16_t s) { heartbeats.push_back(s); } };
};

void testHappyPath()
{
    Harness h;
    CHECK(h.coord.state() == State::Idle);
    CHECK(h.coord.heartbeatStatus() == HEARTBEAT_STATUS_IDLE);

    CHECK(h.coord.tagsUploaded(true) == Result::Accepted);
    CHECK(h.coord.state() == State::HasTags);
    CHECK(h.coord.heartbeatStatus() == HEARTBEAT_STATUS_HAS_TAGS);

    CHECK(h.coord.requestStart() == Result::Accepted);
    CHECK(h.coord.state() == State::Starting);
    CHECK(h.coord.heartbeatStatus() == HEARTBEAT_STATUS_HAS_TAGS);   // not yet published

    h.coord.startFinished(true);
    CHECK(h.coord.state() == State::Detecting);
    CHECK(h.coord.heartbeatStatus() == HEARTBEAT_STATUS_DETECTING);

    CHECK(h.coord.requestStop() == Result::Accepted);
    CHECK(h.coord.state() == State::Stopping);
    CHECK(h.coord.heartbeatStatus() == HEARTBEAT_STATUS_DETECTING);  // still gating

    h.coord.stopFinished();
    CHECK(h.coord.state() == State::HasTags);

    // Heartbeat sink fires once per published change only.
    CHECK((h.heartbeats == std::vector<uint16_t>{HEARTBEAT_STATUS_HAS_TAGS, HEARTBEAT_STATUS_DETECTING, HEARTBEAT_STATUS_HAS_TAGS}));
}

void testStartRejections()
{
    Harness h;
    CHECK(h.coord.requestStart() == Result::NoTags);
    CHECK(h.coord.tagsUploaded(false) == Result::Accepted);           // empty list
    CHECK(h.coord.requestStart() == Result::NoTags);
    CHECK(h.coord.tagsUploaded(true) == Result::Accepted);
    CHECK(h.coord.requestStart() == Result::Accepted);
    CHECK(h.coord.requestStart() == Result::AlreadyStarting);         // retry mid-start
    h.coord.startFinished(true);
    CHECK(h.coord.requestStart() == Result::AlreadyDetecting);
    CHECK(h.coord.requestStop() == Result::Accepted);
    CHECK(h.coord.requestStart() == Result::AlreadyDetecting);        // during teardown
}

void testStopRejections()
{
    Harness h;
    CHECK(h.coord.requestStop() == Result::NotDetecting);
    h.coord.tagsUploaded(true);
    CHECK(h.coord.requestStop() == Result::NotDetecting);
    h.coord.requestStart();
    CHECK(h.coord.requestStop() == Result::StartInProgress);
    h.coord.startFinished(true);
    CHECK(h.coord.requestStop() == Result::Accepted);
    CHECK(h.coord.requestStop() == Result::AlreadyStopping);          // retry mid-stop
    h.coord.stopFinished();
    CHECK(h.coord.requestStop() == Result::NotDetecting);
}

void testFailedStartReturnsToHasTags()
{
    Harness h;
    h.coord.tagsUploaded(true);
    CHECK(h.coord.requestStart() == Result::Accepted);
    h.coord.startFinished(false);
    CHECK(h.coord.state() == State::HasTags);
    CHECK(h.heartbeats.size() == 1);                                  // HasTags once; Starting/failed start publish nothing new
    CHECK(h.coord.requestStart() == Result::Accepted);                // can try again
}

void testTagsUploadedRefusedWhileBusy()
{
    Harness h;
    h.coord.tagsUploaded(true);
    h.coord.requestStart();
    CHECK(h.coord.tagsUploaded(false) == Result::Busy);
    h.coord.startFinished(true);
    CHECK(h.coord.tagsUploaded(false) == Result::Busy);
    h.coord.requestStop();
    CHECK(h.coord.tagsUploaded(false) == Result::Busy);
    h.coord.stopFinished();
    CHECK(h.coord.tagsUploaded(false) == Result::Accepted);
    CHECK(h.coord.state() == State::Idle);
}

void testFinishedCallsOutOfStateAreIgnored()
{
    Harness h;
    h.coord.startFinished(true);      // Idle
    CHECK(h.coord.state() == State::Idle);
    h.coord.stopFinished();
    CHECK(h.coord.state() == State::Idle);
    h.coord.tagsUploaded(true);
    h.coord.stopFinished();           // HasTags
    CHECK(h.coord.state() == State::HasTags);
}

void testWaitWhileWakesOnTransition()
{
    Harness h;
    h.coord.tagsUploaded(true);
    h.coord.requestStart();
    CHECK(!h.coord.waitWhile(State::Starting, std::chrono::milliseconds(20)));

    std::thread worker([&h] {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        h.coord.startFinished(true);
    });
    CHECK(h.coord.waitWhile(State::Starting, std::chrono::seconds(5)));
    worker.join();
    CHECK(h.coord.state() == State::Detecting);
    CHECK(h.coord.waitWhile(State::Starting, std::chrono::milliseconds(0)));   // not in that state
}

} // namespace

int main()
{
    testHappyPath();
    testStartRejections();
    testStopRejections();
    testFailedStartReturnsToHasTags();
    testTagsUploadedRefusedWhileBusy();
    testFinishedCallsOutOfStateAreIgnored();
    testWaitWhileWakesOnTransition();
    std::printf("test_detection_coordinator: all tests passed\n");
    return 0;
}
