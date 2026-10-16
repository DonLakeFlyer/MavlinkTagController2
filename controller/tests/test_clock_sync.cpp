#include "ClockStepGate.h"
#include "ClockSync.h"
#include "DetectionCoordinator.h"
#include "test_check.h"

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <optional>
#include <string>

#ifdef __linux__
#include <fstream>
#include <linux/capability.h>
#endif

namespace {

using Kind = ClockSync::Outcome::Kind;

constexpr double kMinEpoch  = 1767225600.0;     // 2026-01-01
constexpr double kGpsEpoch  = 1792100000.0;     // 2026-10-15
constexpr double kPiBehind  = 1340.0;           // 22 min 20 s, the 10-14 field offset

struct FakeClock {
    double  realtime        = kGpsEpoch - kPiBehind;
    bool    ntp             = false;
    int     setError        = 0;
    int     setCalls        = 0;

    ClockSync::Ops ops()
    {
        ClockSync::Ops ops;
        ops.realtimeNow     = [this] { return realtime; };
        ops.setRealtime     = [this](double seconds) {
            ++setCalls;
            if (setError != 0) {
                return setError;
            }
            realtime = seconds;
            return 0;
        };
        ops.ntpSynchronized = [this] { return ntp; };
        return ops;
    }
};

// Drives 1 Hz SYSTEM_TIME with the Pi clock and GPS time both advancing.
struct Feed {
    FakeClock&  clock;
    ClockSync&  sync;
    double      steady  = 100.0;
    double      gps     = kGpsEpoch;

    ClockSync::Outcome tick(bool sessionActive = false)
    {
        steady          += 1.0;
        gps             += 1.0;
        clock.realtime  += 1.0;
        return sync.systemTime(static_cast<uint64_t>(gps * 1e6), steady, sessionActive);
    }

    // Ticks until something other than None or the settle wait comes back.
    ClockSync::Outcome tickUntilEvent(int maxTicks = 20, bool sessionActive = false)
    {
        for (int i = 0; i < maxTicks; ++i) {
            auto outcome = tick(sessionActive);
            if (outcome.kind != Kind::None) {
                return outcome;
            }
        }
        return {};
    }
};

void testStepsAfterFixAndSettle()
{
    FakeClock clock;
    ClockSync sync(clock.ops(), kMinEpoch);
    Feed feed { clock, sync };

    sync.gpsFixType(3);
    // The first kSettleSecs of consistent samples do not step.
    for (int i = 0; i < static_cast<int>(ClockSync::kSettleSecs); ++i) {
        CHECK(feed.tick().kind == Kind::None);
    }
    CHECK(clock.setCalls == 0);

    const auto outcome = feed.tick();
    CHECK(outcome.kind == Kind::Stepped);
    CHECK(std::fabs(outcome.offsetSeconds - kPiBehind) < 1e-3);
    CHECK(outcome.message.find("stepped system clock +1340.000 s") == 0);
    CHECK(clock.setCalls == 1);
    CHECK(std::fabs(clock.realtime - feed.gps) < 1e-6);
    CHECK(sync.state() == ClockSync::State::Stepped);

    // At most once per run: later samples never step again.
    clock.realtime -= 100.0;
    for (int i = 0; i < 10; ++i) {
        CHECK(feed.tick().kind == Kind::None);
    }
    CHECK(clock.setCalls == 1);

    // ... but the session-start line flags the new offset.
    const auto summary = sync.sessionStartSummary(feed.steady);
    CHECK(summary.warn);
    CHECK(summary.message.find("GPS offset +100.000 s") != std::string::npos);
    CHECK(summary.message.find("stepped +1340.000 s earlier this run") != std::string::npos);
}

// Pi ahead of GPS: the step is backward.
void testStepsBackward()
{
    FakeClock clock;
    clock.realtime = kGpsEpoch + kPiBehind;
    ClockSync sync(clock.ops(), kMinEpoch);
    Feed feed { clock, sync };

    sync.gpsFixType(3);
    const auto outcome = feed.tickUntilEvent();
    CHECK(outcome.kind == Kind::Stepped);
    CHECK(std::fabs(outcome.offsetSeconds + kPiBehind) < 1e-3);
    CHECK(outcome.message.find("stepped system clock -1340.000 s") == 0);
    CHECK(std::fabs(clock.realtime - feed.gps) < 1e-6);
}

// Gate against the real DetectionCoordinator heartbeat mapping.
void testClockStepGate()
{
    uint16_t heartbeat = HEARTBEAT_STATUS_IDLE;
    DetectionCoordinator detection([&heartbeat](uint16_t status) { heartbeat = status; });

    CHECK(!clockStepBlocked(false, detection.heartbeatStatus()));
    CHECK(detection.tagsUploaded(true) == DetectionCoordinator::Result::Accepted);
    CHECK(!clockStepBlocked(false, heartbeat));

    CHECK(detection.requestCapture() == DetectionCoordinator::Result::Accepted);
    CHECK(clockStepBlocked(false, heartbeat));
    detection.captureFinished();
    CHECK(!clockStepBlocked(false, heartbeat));

    CHECK(detection.requestStart() == DetectionCoordinator::Result::Accepted);
    detection.startFinished(true);
    CHECK(detection.state() == DetectionCoordinator::State::Detecting);
    CHECK(clockStepBlocked(true, heartbeat));

    // Teardown with a hung child: vehicle time is unfrozen but stopFinished()
    // never comes, so the coordinator stays Stopping.
    CHECK(detection.requestStop() == DetectionCoordinator::Result::Accepted);
    CHECK(detection.state() == DetectionCoordinator::State::Stopping);
    CHECK(clockStepBlocked(false, heartbeat));

    detection.stopFinished();
    CHECK(!clockStepBlocked(false, heartbeat));
    CHECK(clockStepBlocked(true, heartbeat));

    // IQ recording teardown with a hung child: unfrozen, stays RecordStopping.
    CHECK(detection.requestRecordStart() == DetectionCoordinator::Result::Accepted);
    detection.recordStartFinished(true);
    CHECK(clockStepBlocked(false, heartbeat));
    CHECK(detection.requestRecordStop() == DetectionCoordinator::Result::Accepted);
    CHECK(clockStepBlocked(false, heartbeat));
    detection.recordStopFinished();
    CHECK(!clockStepBlocked(false, heartbeat));
}

void testWaitsForThreeDFix()
{
    FakeClock clock;
    ClockSync sync(clock.ops(), kMinEpoch);
    Feed feed { clock, sync };

    sync.gpsFixType(2);
    const auto first = feed.tick();
    CHECK(first.kind == Kind::Waiting);
    CHECK(first.message == "waiting: no 3D GPS fix (fix_type 2)");
    // Logged once, not every second.
    for (int i = 0; i < 20; ++i) {
        CHECK(feed.tick().kind == Kind::None);
    }
    CHECK(clock.setCalls == 0);

    const auto waiting = sync.sessionStartSummary(feed.steady);
    CHECK(!waiting.warn);
    CHECK(waiting.message == "session start: GPS offset n/a (no 3D fix); not synced yet (waiting: no 3D GPS fix (fix_type 2))");

    sync.gpsFixType(3);
    CHECK(feed.tickUntilEvent().kind == Kind::Stepped);
}

void testIgnoresImplausibleTime()
{
    FakeClock clock;
    ClockSync sync(clock.ops(), kMinEpoch);
    sync.gpsFixType(3);

    // Zero, and an autopilot clock still near 1970 or its build date.
    auto outcome = sync.systemTime(0, 1.0, false);
    CHECK(outcome.kind == Kind::Waiting);
    CHECK(outcome.message == "waiting: autopilot time not valid yet");
    for (int i = 0; i < 20; ++i) {
        CHECK(sync.systemTime(static_cast<uint64_t>((1000.0 + i) * 1e6), 2.0 + i, false).kind == Kind::None);
        CHECK(sync.systemTime(static_cast<uint64_t>((kMinEpoch - 86400.0 + i) * 1e6), 2.0 + i, false).kind == Kind::None);
    }
    CHECK(clock.setCalls == 0);
    CHECK(sync.sessionStartSummary(30.0).message.find("GPS offset n/a (no autopilot time)") != std::string::npos);
}

void testReportsMissingGpsRawInt()
{
    FakeClock clock;
    ClockSync sync(clock.ops(), kMinEpoch);
    Feed feed { clock, sync };

    // Silent until kNoGpsMessageSecs, then reported once.
    CHECK(sync.systemTime(0, feed.steady, false).kind == Kind::Waiting);
    for (int i = 0; i < static_cast<int>(ClockSync::kNoGpsMessageSecs); ++i) {
        CHECK(feed.tick().kind == Kind::None);
    }
    // A session starting in the silent window names the real blocker, not the earlier invalid time.
    CHECK(sync.sessionStartSummary(feed.steady).message
          == "session start: GPS offset n/a (no 3D fix); not synced yet (waiting: no GPS_RAW_INT yet)");
    const auto outcome = feed.tick();
    CHECK(outcome.kind == Kind::Waiting);
    CHECK(outcome.message == "waiting: no GPS_RAW_INT received from autopilot");
    CHECK(feed.tick().kind == Kind::None);
    CHECK(clock.setCalls == 0);
}

void testTimeJumpRestartsSettle()
{
    FakeClock clock;
    ClockSync sync(clock.ops(), kMinEpoch);
    Feed feed { clock, sync };

    sync.gpsFixType(3);
    feed.tick();
    feed.tick();
    // The autopilot sets its own clock from GPS just after reporting the fix.
    feed.gps += 600.0;
    const auto jumped = feed.tick();
    CHECK(jumped.kind == Kind::Waiting);
    CHECK(jumped.message == "waiting: autopilot time drifted +600.0 s from the steady clock; restarting 5 s settle");
    for (int i = 0; i < static_cast<int>(ClockSync::kSettleSecs) - 1; ++i) {
        CHECK(feed.tick().kind == Kind::None);
    }
    const auto stepped = feed.tick();
    CHECK(stepped.kind == Kind::Stepped);
    CHECK(std::fabs(clock.realtime - feed.gps) < 1e-6);
}

// Stalled or slow autopilot time must never settle, even though each sample
// differs from the previous one by no more than kConsistencySecs.
void testStalledOrSlowTimeNeverSettles()
{
    for (const double gpsStepPerSecond : { 0.0, 0.5 }) {
        FakeClock clock;
        ClockSync sync(clock.ops(), kMinEpoch);
        sync.gpsFixType(3);
        double steady = 100.0;
        double gps    = kGpsEpoch;
        int    driftLogs = 0;
        for (int i = 0; i < 30; ++i) {
            steady         += 1.0;
            gps            += gpsStepPerSecond;
            clock.realtime += 1.0;
            const auto outcome = sync.systemTime(static_cast<uint64_t>(gps * 1e6), steady, false);
            CHECK(outcome.kind != Kind::Stepped);
            driftLogs += outcome.message.find("drifted") != std::string::npos ? 1 : 0;
        }
        // The settle keeps restarting, but the log line appears once.
        CHECK(driftLogs == 1);
        CHECK(clock.setCalls == 0);
        CHECK(sync.state() == ClockSync::State::Waiting);
    }
}

// A fix lost and regained between two SYSTEM_TIME samples still restarts the settle.
void testFixFlickerBetweenSamplesRestartsSettle()
{
    FakeClock clock;
    ClockSync sync(clock.ops(), kMinEpoch);
    Feed feed { clock, sync };

    sync.gpsFixType(3);
    for (int i = 0; i < 3; ++i) {
        CHECK(feed.tick().kind == Kind::None);
    }
    sync.gpsFixType(1);
    sync.gpsFixType(3);
    for (int i = 0; i < static_cast<int>(ClockSync::kSettleSecs); ++i) {
        CHECK(feed.tick().kind == Kind::None);
    }
    CHECK(feed.tick().kind == Kind::Stepped);
}

void testLosingFixRestartsSettle()
{
    FakeClock clock;
    ClockSync sync(clock.ops(), kMinEpoch);
    Feed feed { clock, sync };

    sync.gpsFixType(3);
    for (int i = 0; i < 3; ++i) {
        feed.tick();
    }
    sync.gpsFixType(1);
    CHECK(feed.tick().kind == Kind::Waiting);
    sync.gpsFixType(3);
    for (int i = 0; i < static_cast<int>(ClockSync::kSettleSecs); ++i) {
        CHECK(feed.tick().kind == Kind::None);
    }
    CHECK(feed.tick().kind == Kind::Stepped);
}

void testSmallOffsetIsLeftAlone()
{
    FakeClock clock;
    clock.realtime = kGpsEpoch - 0.4;
    ClockSync sync(clock.ops(), kMinEpoch);
    Feed feed { clock, sync };

    sync.gpsFixType(3);
    const auto outcome = feed.tickUntilEvent();
    CHECK(outcome.kind == Kind::Skipped);
    CHECK(outcome.message == "skipped: offset +0.400 s is within 1.0 s");
    CHECK(clock.setCalls == 0);
    CHECK(sync.state() == ClockSync::State::InSync);
    // A later large offset is reported, not stepped.
    clock.realtime -= 50.0;
    CHECK(feed.tickUntilEvent().kind == Kind::None);
    CHECK(clock.setCalls == 0);
    CHECK(sync.sessionStartSummary(feed.steady).warn);
}

void testNtpOwnsTheClock()
{
    FakeClock clock;
    clock.ntp = true;
    ClockSync sync(clock.ops(), kMinEpoch);
    Feed feed { clock, sync };

    sync.gpsFixType(3);
    const auto outcome = feed.tickUntilEvent();
    CHECK(outcome.kind == Kind::Skipped);
    CHECK(outcome.message == "skipped: system clock is NTP-synchronized");
    CHECK(clock.setCalls == 0);
    CHECK(sync.state() == ClockSync::State::NtpSynchronized);
}

void testDeferredUntilSessionEnds()
{
    FakeClock clock;
    ClockSync sync(clock.ops(), kMinEpoch);
    Feed feed { clock, sync };

    sync.gpsFixType(3);
    const auto deferred = feed.tickUntilEvent(20, true /* sessionActive */);
    CHECK(deferred.kind == Kind::Deferred);
    CHECK(deferred.message == "deferred: session active (offset +1340.000 s); will step when it ends");
    // Logged once for the whole session, never stepped inside it.
    for (int i = 0; i < 30; ++i) {
        CHECK(feed.tick(true).kind == Kind::None);
    }
    CHECK(clock.setCalls == 0);
    CHECK(sync.sessionStartSummary(feed.steady).message
          == "session start: GPS offset +1340.000 s; not synced yet (deferred: session active)");

    // First sample after the session ends steps.
    const auto stepped = feed.tick(false);
    CHECK(stepped.kind == Kind::Stepped);
    CHECK(clock.setCalls == 1);
}

void testPermissionFailureIsReportedOnce()
{
    FakeClock clock;
    clock.setError = EPERM;
    ClockSync sync(clock.ops(), kMinEpoch);
    Feed feed { clock, sync };

    sync.gpsFixType(3);
    const auto failed = feed.tickUntilEvent();
    CHECK(failed.kind == Kind::Failed);
    CHECK(failed.message.find("failed: clock_settime: ") == 0);
    CHECK(sync.state() == ClockSync::State::Failed);
    // No retry.
    for (int i = 0; i < 10; ++i) {
        CHECK(feed.tick().kind == Kind::None);
    }
    CHECK(clock.setCalls == 1);
    const auto summary = sync.sessionStartSummary(feed.steady);
    CHECK(!summary.warn);
    CHECK(summary.message.find("GPS offset +1340.000 s; sync failed (clock_settime: ") != std::string::npos);
}

void testDisabledStillReportsOffset()
{
    FakeClock clock;
    ClockSync sync(clock.ops(), kMinEpoch, std::string("not Linux"));
    Feed feed { clock, sync };

    sync.gpsFixType(3);
    for (int i = 0; i < 20; ++i) {
        CHECK(feed.tick().kind == Kind::None);
    }
    CHECK(clock.setCalls == 0);
    CHECK(sync.state() == ClockSync::State::Disabled);
    const auto summary = sync.sessionStartSummary(feed.steady);
    CHECK(!summary.warn);
    CHECK(summary.message == "session start: GPS offset +1340.000 s; sync disabled (not Linux)");
}

void testSummaryProjectsLatestSample()
{
    FakeClock clock;
    ClockSync sync(clock.ops(), kMinEpoch, std::string("test"));
    sync.gpsFixType(3);
    sync.systemTime(static_cast<uint64_t>(kGpsEpoch * 1e6), 10.0, false);
    // 2.5 s later, with the Pi clock 2.5 s further on as well.
    clock.realtime = kGpsEpoch - 3.0 + 2.5;
    CHECK(sync.sessionStartSummary(12.5).message.find("GPS offset +3.000 s") != std::string::npos);
}

void testBuildDateParse()
{
    CHECK(ClockSync::buildDateEpochSeconds("Jan  1 2026") == std::optional<double>(1767225600.0));
    CHECK(ClockSync::buildDateEpochSeconds("Oct 15 2026") == std::optional<double>(1792022400.0));
    CHECK(!ClockSync::buildDateEpochSeconds("Foo 15 2026"));
    CHECK(!ClockSync::buildDateEpochSeconds(""));
    CHECK(!ClockSync::buildDateEpochSeconds(nullptr));
    CHECK(ClockSync::minPlausibleEpochSeconds() >= 1767225600.0);
}

#ifdef __linux__
// The kernel's own view of a capability set, from /proc/self/status ("CapEff", "CapAmb").
uint64_t procCapabilities(const std::string& field)
{
    std::ifstream status("/proc/self/status");
    std::string line;
    while (std::getline(status, line)) {
        if (line.rfind(field + ":", 0) == 0) {
            return std::stoull(line.substr(field.size() + 1), nullptr, 16);
        }
    }
    CHECK(false);
    return 0;
}
#endif

// Runs the real platform calls, never setRealtime: that would move the host clock.
void testPlatformOps()
{
    const auto ops = ClockSync::systemOps();
    CHECK(std::fabs(ops.realtimeNow() - static_cast<double>(std::time(nullptr))) < 2.0);
    (void)ops.ntpSynchronized();    // host-dependent; must just not fail

    const auto reason = ClockSync::platformDisabledReason();
    CHECK(!ClockSync::clearAmbientCapabilities());
    // Clearing the ambient set must not drop this process's own capability.
    CHECK(ClockSync::platformDisabledReason() == reason);

#ifdef __linux__
    const bool kernelSaysCapSysTime = (procCapabilities("CapEff") >> CAP_SYS_TIME) & 1;
    CHECK(kernelSaysCapSysTime == !reason);
    if (reason) {
        CHECK(reason->find("CAP_SYS_TIME") != std::string::npos);
    }
    CHECK(procCapabilities("CapAmb") == 0);
    // CI also runs this test with CAP_SYS_TIME granted as an ambient capability,
    // the way mavlink-tag-controller.service grants it.
    if (const char* expect = std::getenv("CLOCK_SYNC_TEST_EXPECT_CAP_SYS_TIME"); expect && *expect == '1') {
        CHECK(!reason);
    }
    std::printf("test_clock_sync: CAP_SYS_TIME %s, NTP-synchronized %s\n",
                reason ? "absent" : "present", ops.ntpSynchronized() ? "yes" : "no");
#else
    CHECK(reason == std::optional<std::string>("not Linux"));
#endif
}

}

int main()
{
    testStepsAfterFixAndSettle();
    testStepsBackward();
    testClockStepGate();
    testWaitsForThreeDFix();
    testIgnoresImplausibleTime();
    testReportsMissingGpsRawInt();
    testTimeJumpRestartsSettle();
    testLosingFixRestartsSettle();
    testStalledOrSlowTimeNeverSettles();
    testFixFlickerBetweenSamplesRestartsSettle();
    testSmallOffsetIsLeftAlone();
    testNtpOwnsTheClock();
    testDeferredUntilSessionEnds();
    testPermissionFailureIsReportedOnce();
    testDisabledStillReportsOffset();
    testSummaryProjectsLatestSample();
    testBuildDateParse();
    testPlatformOps();
    std::printf("test_clock_sync: all tests passed\n");
    return 0;
}
