// TelemetryCache: pose lookups for detector results that arrive a minute or
// more after their pulses, and the dwell-mean pose used for rotation slices.

#include "TelemetryCache.h"
#include "test_check.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <thread>

namespace {

using Entry = TelemetryCache::TelemetryCacheEntry_t;

double nowSeconds()
{
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

Entry makeEntry(double timeSeconds, float yawDegrees, double latitude = 47.7)
{
    Entry entry {};
    entry.timeInSeconds               = timeSeconds;
    entry.position.latitude           = latitude;
    entry.position.longitude          = -122.14;
    entry.position.relativeAltitude   = 120.0;
    entry.attitudeEuler.yawDegrees    = yawDegrees;
    return entry;
}

// Fill [start, end] at the live sampling interval, yaw from yawAt(t).
template <typename YawFn>
void fill(TelemetryCache& cache, double start, double end, YawFn yawAt)
{
    for (double t = start; t <= end + 1e-9; t += 0.5) {
        cache.addEntry(makeEntry(t, yawAt(t)));
    }
}

// A K=20 result arrives ~61 s after its first pulse (40 s segment + compute).
void testPoseOlderThanAMinuteIsFound()
{
    TelemetryCache cache;
    const double start = nowSeconds() - 120.0;
    fill(cache, start, start + 120.0, [start](double t) { return static_cast<float>(t - start); });

    const auto pose = cache.telemetryForTime(start + 30.0);
    CHECK(pose.has_value());
    CHECK(std::fabs(pose->timeInSeconds - (start + 30.0)) < 0.26);
    CHECK(std::fabs(pose->attitudeEuler.yawDegrees - 30.0f) < 0.6f);
    std::printf("PASS: testPoseOlderThanAMinuteIsFound\n");
}

void testLookupFarFromAnySampleIsRejected()
{
    TelemetryCache cache;
    const double start = nowSeconds() - 10.0;
    fill(cache, start, start + 10.0, [](double) { return 90.0f; });

    CHECK(!cache.telemetryForTime(start + 30.0).has_value());
    CHECK(!cache.telemetryForTime(start - 5.0).has_value());
    CHECK(cache.telemetryForTime(start + 5.2).has_value());
    std::printf("PASS: testLookupFarFromAnySampleIsRejected\n");
}

void testEmptyCacheHasNoPose()
{
    TelemetryCache cache;
    CHECK(!cache.telemetryForTime(nowSeconds()).has_value());
    CHECK(!cache.meanTelemetry(nowSeconds() - 10.0, nowSeconds()).has_value());
    std::printf("PASS: testEmptyCacheHasNoPose\n");
}

// Dwell at north: the vehicle wanders either side of 0/360.
void testMeanYawAcrossNorth()
{
    TelemetryCache cache;
    const double start = nowSeconds() - 50.0;
    fill(cache, start, start + 40.0, [start](double t) {
        const int i = static_cast<int>(std::lround((t - start) / 0.5));
        return (i % 2 == 0) ? 358.0f : 2.0f;
    });

    const auto pose = cache.meanTelemetry(start, start + 40.0);
    CHECK(pose.has_value());
    const double yaw = std::remainder(static_cast<double>(pose->attitudeEuler.yawDegrees), 360.0);
    CHECK(std::fabs(yaw) < 0.5);
    CHECK(std::fabs(pose->position.latitude - 47.7) < 1e-9);
    std::printf("PASS: testMeanYawAcrossNorth\n");
}

// The turn into the dwell lies outside the window and must not pull the mean.
void testMeanUsesOnlyTheWindow()
{
    TelemetryCache cache;
    const double start = nowSeconds() - 60.0;
    fill(cache, start, start + 10.0, [](double) { return 45.0f; });
    fill(cache, start + 10.5, start + 50.0, [](double) { return 90.0f; });

    const auto pose = cache.meanTelemetry(start + 11.0, start + 50.0);
    CHECK(pose.has_value());
    CHECK(std::fabs(pose->attitudeEuler.yawDegrees - 90.0f) < 0.01f);
    std::printf("PASS: testMeanUsesOnlyTheWindow\n");
}

void testMeanRejectsWindowWithoutSamples()
{
    TelemetryCache cache;
    const double start = nowSeconds() - 60.0;
    fill(cache, start, start + 20.0, [](double) { return 90.0f; });
    fill(cache, start + 30.0, start + 50.0, [](double) { return 90.0f; });

    CHECK(!cache.meanTelemetry(start, start + 50.0).has_value());          // 10 s hole
    CHECK(!cache.meanTelemetry(start + 40.0, start + 70.0).has_value());   // runs past the newest sample
    CHECK(cache.meanTelemetry(start + 30.0, start + 50.0).has_value());
    std::printf("PASS: testMeanRejectsWindowWithoutSamples\n");
}

// After the clock steps back, pre-step samples at the same timestamps belong to another dwell.
void testBackwardClockStepDoesNotMixDwells()
{
    TelemetryCache cache;
    const double start = nowSeconds() - 120.0;
    fill(cache, start, start + 60.0, [](double) { return 0.0f; });
    const auto stepBack = cache.addEntry(makeEntry(start + 30.0, 90.0f));
    CHECK(stepBack.has_value() && std::fabs(*stepBack - 30.0) < 1e-6);
    fill(cache, start + 30.5, start + 60.0, [](double) { return 90.0f; });

    const auto pose = cache.meanTelemetry(start + 30.0, start + 60.0);
    CHECK(pose.has_value());
    CHECK(std::fabs(pose->attitudeEuler.yawDegrees - 90.0f) < 0.01f);
    std::printf("PASS: testBackwardClockStepDoesNotMixDwells\n");
}

// The sampling thread must stop with the cache, not outlive it.
void testDestructionStopsSampling()
{
    auto calls = std::make_shared<std::atomic<int>>(0);
    {
        TelemetryCache cache([calls]() -> std::optional<Entry> {
            ++*calls;
            return makeEntry(nowSeconds(), 0.0f);
        }, [](double) {});
        std::this_thread::sleep_for(std::chrono::milliseconds(700));
    }
    const int callsAtDestruction = calls->load();
    CHECK(callsAtDestruction > 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1200));
    CHECK(calls->load() == callsAtDestruction);
    std::printf("PASS: testDestructionStopsSampling\n");
}

}

int main()
{
    testPoseOlderThanAMinuteIsFound();
    testLookupFarFromAnySampleIsRejected();
    testEmptyCacheHasNoPose();
    testMeanYawAcrossNorth();
    testMeanUsesOnlyTheWindow();
    testMeanRejectsWindowWithoutSamples();
    testBackwardClockStepDoesNotMixDwells();
    testDestructionStopsSampling();
    std::printf("All TelemetryCache tests passed\n");
    return 0;
}
