#pragma once

#include <condition_variable>
#include <functional>
#include <list>
#include <mutex>
#include <optional>
#include <thread>

#include <mavlink.h>

#include "Telemetry.h"

class TelemetryCache
{
public:
    typedef struct {
        double                  timeInSeconds;
        Telemetry::Position_t   position;
        Telemetry::EulerAngle_t attitudeEuler;
    } TelemetryCacheEntry_t;

    using Sampler = std::function<std::optional<TelemetryCacheEntry_t>()>;
    using ClockStepBackHandler = std::function<void(double stepSeconds)>;

    TelemetryCache() = default;     // no sampling thread; entries come from addEntry()
    TelemetryCache(Sampler sampler, ClockStepBackHandler onClockStepBack);
    ~TelemetryCache();

    // Returns how far the clock stepped back when the entry is older than the newest
    // sample; the earlier samples are then dropped so two dwells can't be averaged together.
    std::optional<double> addEntry(const TelemetryCacheEntry_t& entry);
    // Drops every sample, e.g. after the system clock was stepped.
    void clear();

    // Nearest sample within ~1 s of the time, else nullopt.
    std::optional<TelemetryCacheEntry_t> telemetryForTime(double timeInSeconds);
    // Mean pose (circular-mean yaw) over a window the samples cover without gaps, else nullopt.
    std::optional<TelemetryCacheEntry_t> meanTelemetry(double startSeconds, double endSeconds);

private:
    void _pruneTelemetryCache(double newestSeconds);

    std::list<TelemetryCacheEntry_t>    _telemetryCache;
    std::mutex                          _telemetryCacheMutex;

    std::mutex                          _stopMutex;
    std::condition_variable             _stopCondition;
    bool                                _stopRequested = false;
    std::thread                         _samplerThread;     // last member: joined before the rest is destroyed
};
