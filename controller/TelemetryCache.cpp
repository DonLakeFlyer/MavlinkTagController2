#include "TelemetryCache.h"
#include "MavlinkSystem.h"
#include "log.h"
#include "timeHelpers.h"

#include <functional>
#include <thread>
#include <chrono>
#include <cmath>
#include <ctime>
#include <time.h>

TelemetryCache::TelemetryCache(MavlinkSystem* mavlink)
    : _mavlink(mavlink)
{
    std::thread([this]()
    {
        while (true) {
            Telemetry& telemetry = _mavlink->telemetry();

            if (telemetry.lastPosition().has_value() && telemetry.lastAttitudeEuler().has_value()) {
                std::lock_guard<std::mutex> lock(_telemetryCacheMutex);
                TelemetryCacheEntry_t       entry {};

                entry.timeInSeconds         = secondsSinceEpoch();
                entry.position              = telemetry.lastPosition().value();
                entry.attitudeEuler         = telemetry.lastAttitudeEuler().value();

                _telemetryCache.push_back(entry);

                _pruneTelemetryCache();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }).detach();
}

TelemetryCache::~TelemetryCache()
{
}

TelemetryCache::TelemetryCacheEntry_t TelemetryCache::telemetryForTime(double timeInSeconds)
{
    std::lock_guard<std::mutex>     lock            (_telemetryCacheMutex);
    TelemetryCacheEntry_t           bestEntry       { };
    double                          bestDiffMSecs   = -1;

    // Find the closest matching entry in the cache
    for (auto entry : _telemetryCache) {
        double  entryDiffMSecs  = std::fabs(timeInSeconds - entry.timeInSeconds);

        if (bestDiffMSecs < 0) {
            bestDiffMSecs = entryDiffMSecs;
            bestEntry = entry;
        } else {
            if (entryDiffMSecs < bestDiffMSecs) {
                bestDiffMSecs = entryDiffMSecs;
                bestEntry = entry;
            }
        }
    }

    return bestEntry;
}

void TelemetryCache::_pruneTelemetryCache()
{
    auto    timeSince           = std::chrono::system_clock::now().time_since_epoch();
    auto    nowMSecs            = std::chrono::duration_cast<std::chrono::milliseconds>(timeSince).count();
    double  nowSecs             = nowMSecs / 1000.0;
    double  maxIntraPulseSecs   = 5.0;
    double  maxK                = 3.0;
    double  pruneBeforeSecs     = nowSecs - (((maxK + 1) * maxIntraPulseSecs) * 2);

    while (_telemetryCache.size() > 0) {
        auto& entry = _telemetryCache.front();

        if (entry.timeInSeconds < pruneBeforeSecs) {
            _telemetryCache.pop_front();
        } else {
            break;
        }
    }
}
