#include "TelemetryCache.h"

#include <algorithm>
#include <functional>
#include <thread>
#include <chrono>
#include <cmath>
#include <vector>

namespace {

constexpr double kSampleIntervalSecs = 0.5;
// Detector results arrive a segment plus compute after their first pulse
// (~61 s for K=20 at 2 s, ~76 s with slow compute); keep ample headroom.
constexpr double kRetentionSecs      = 600.0;
// Longer than this from any sample (or between samples) means no pose.
constexpr double kMaxSampleGapSecs   = 1.0;
constexpr size_t kMaxEntries         = static_cast<size_t>(kRetentionSecs / kSampleIntervalSecs) * 2;

constexpr double kPi = 3.14159265358979323846;

}

TelemetryCache::TelemetryCache(Sampler sampler, ClockStepBackHandler onClockStepBack)
{
    _samplerThread = std::thread([this, sampler, onClockStepBack]()
    {
        std::unique_lock<std::mutex> stopLock(_stopMutex);
        while (!_stopRequested) {
            stopLock.unlock();
            if (auto entry = sampler()) {
                if (const auto stepBack = addEntry(*entry)) {
                    onClockStepBack(*stepBack);
                }
            }
            stopLock.lock();
            _stopCondition.wait_for(stopLock, std::chrono::milliseconds(static_cast<int>(kSampleIntervalSecs * 1000)),
                                    [this]() { return _stopRequested; });
        }
    });
}

TelemetryCache::~TelemetryCache()
{
    {
        std::lock_guard<std::mutex> lock(_stopMutex);
        _stopRequested = true;
    }
    _stopCondition.notify_all();
    if (_samplerThread.joinable()) {
        _samplerThread.join();
    }
}

std::optional<double> TelemetryCache::addEntry(const TelemetryCacheEntry_t& entry)
{
    std::lock_guard<std::mutex> lock(_telemetryCacheMutex);
    std::optional<double> stepBack;
    if (!_telemetryCache.empty() && entry.timeInSeconds < _telemetryCache.back().timeInSeconds) {
        stepBack = _telemetryCache.back().timeInSeconds - entry.timeInSeconds;
        _telemetryCache.clear();
    }
    _telemetryCache.push_back(entry);
    _pruneTelemetryCache(entry.timeInSeconds);
    return stepBack;
}

void TelemetryCache::clear()
{
    std::lock_guard<std::mutex> lock(_telemetryCacheMutex);
    _telemetryCache.clear();
}

std::optional<TelemetryCache::TelemetryCacheEntry_t> TelemetryCache::telemetryForTime(double timeInSeconds)
{
    std::lock_guard<std::mutex> lock(_telemetryCacheMutex);
    std::optional<TelemetryCacheEntry_t> bestEntry;
    double bestDiffSecs = kMaxSampleGapSecs;

    for (const auto& entry : _telemetryCache) {
        const double diffSecs = std::fabs(timeInSeconds - entry.timeInSeconds);
        if (diffSecs <= bestDiffSecs) {
            bestDiffSecs = diffSecs;
            bestEntry = entry;
        }
    }
    return bestEntry;
}

std::optional<TelemetryCache::TelemetryCacheEntry_t> TelemetryCache::meanTelemetry(double startSeconds, double endSeconds)
{
    std::lock_guard<std::mutex> lock(_telemetryCacheMutex);
    std::vector<const TelemetryCacheEntry_t*> inWindow;
    for (const auto& entry : _telemetryCache) {
        if (entry.timeInSeconds >= startSeconds && entry.timeInSeconds <= endSeconds) {
            inWindow.push_back(&entry);
        }
    }
    if (inWindow.empty()) {
        return std::nullopt;
    }
    std::sort(inWindow.begin(), inWindow.end(),
              [](const auto* a, const auto* b) { return a->timeInSeconds < b->timeInSeconds; });
    if (inWindow.front()->timeInSeconds - startSeconds > kMaxSampleGapSecs
        || endSeconds - inWindow.back()->timeInSeconds > kMaxSampleGapSecs) {
        return std::nullopt;
    }
    for (size_t i = 1; i < inWindow.size(); ++i) {
        if (inWindow[i]->timeInSeconds - inWindow[i - 1]->timeInSeconds > kMaxSampleGapSecs) {
            return std::nullopt;
        }
    }

    double lat = 0.0, lon = 0.0, alt = 0.0, roll = 0.0, pitch = 0.0, yawSin = 0.0, yawCos = 0.0;
    for (const auto* entry : inWindow) {
        lat   += entry->position.latitude;
        lon   += entry->position.longitude;
        alt   += entry->position.relativeAltitude;
        roll  += entry->attitudeEuler.rollDegrees;
        pitch += entry->attitudeEuler.pitchDegrees;
        const double yawRad = entry->attitudeEuler.yawDegrees * kPi / 180.0;
        yawSin += std::sin(yawRad);
        yawCos += std::cos(yawRad);
    }
    const double n = static_cast<double>(inWindow.size());
    TelemetryCacheEntry_t mean {};
    mean.timeInSeconds                  = 0.5 * (startSeconds + endSeconds);
    mean.position.latitude              = lat / n;
    mean.position.longitude             = lon / n;
    mean.position.relativeAltitude      = alt / n;
    mean.attitudeEuler.rollDegrees      = static_cast<float>(roll / n);
    mean.attitudeEuler.pitchDegrees     = static_cast<float>(pitch / n);
    mean.attitudeEuler.yawDegrees       = static_cast<float>(std::atan2(yawSin, yawCos) * 180.0 / kPi);
    return mean;
}

void TelemetryCache::_pruneTelemetryCache(double newestSeconds)
{
    while (!_telemetryCache.empty()
           && (_telemetryCache.size() > kMaxEntries
               || _telemetryCache.front().timeInSeconds < newestSeconds - kRetentionSecs)) {
        _telemetryCache.pop_front();
    }
}
