#include "RotationProgress.h"
#include "TunnelProtocol.h"
#include "formatString.h"

#include <algorithm>
#include <cmath>

using namespace TunnelProtocol;

RotationProgress::RotationProgress(OperationProgressReporter& reporter)
    : _reporter(reporter)
{
}

bool RotationProgress::begin(uint32_t requestId, uint32_t sliceCount, uint32_t detectorCount,
                             uint32_t startupProcessEstimate, double estimatedDwellSeconds)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (_active) {
        return false;
    }
    _sliceCount        = std::max<uint32_t>(sliceCount, 1);
    _detectorCount     = detectorCount;
    _startupProcesses  = startupProcessEstimate;
    _dwellSteps        = static_cast<uint32_t>(std::max(1.0, std::ceil(estimatedDwellSeconds)));
    _dwellLearned      = false;
    _phase             = Phase::Startup;
    _processesStarted  = 0;
    _detectorsReady    = 0;
    _currentSliceId.reset();
    _slicesArmed       = 0;
    _slicesComplete    = 0;
    _sliceProgressByTag.clear();
    _completedExtraSteps = 0;
    _computeTicks      = 0;
    _finalizeStage     = 0;
    _lastStep          = 0;
    if (!_reporter.begin(COMMAND_ID_START_COLLECTION, requestId, "Rotation", _totalSteps())) {
        return false;
    }
    _reporterTotal = _totalSteps();
    _active = true;
    return true;
}

bool RotationProgress::active() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _active;
}

void RotationProgress::setStartupProcessCount(uint32_t count)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_active || count == _startupProcesses) {
        return;
    }
    _startupProcesses = count;
    _publishLocked("");
}

void RotationProgress::processStarted(const std::string& name)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_active || _phase != Phase::Startup) {
        return;
    }
    _processesStarted = std::min(_processesStarted + 1, _startupProcesses);
    _publishLocked(formatString("Starting %s", name.c_str()));
}

void RotationProgress::detectorReady(uint32_t readyCount)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_active || _phase != Phase::Startup) {
        return;
    }
    // READY implies every process is up even if the pipeline's own count lagged.
    _processesStarted = _startupProcesses;
    _detectorsReady = std::min(readyCount, _detectorCount);
    _publishLocked(formatString("Ready %u/%u", _detectorsReady, _detectorCount));
}

void RotationProgress::sliceArmed(uint32_t sliceId, float headingDeg)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_active) {
        return;
    }
    if (_currentSliceId && *_currentSliceId == sliceId) {
        return;   // GCS retry of the same ARM
    }
    _phase = Phase::Slice;
    _processesStarted = _startupProcesses;
    _detectorsReady = _detectorCount;
    _currentSliceId = sliceId;
    _currentHeadingDeg = headingDeg;
    _sliceProgressByTag.clear();
    _computeTicks = 0;
    ++_slicesArmed;
    if (_slicesArmed > _sliceCount) {
        _sliceCount = _slicesArmed;   // unannounced extra slice: grow rather than clamp
    }
    _publishLocked(formatString("%u/%u %03.0f deg", _slicesArmed, _sliceCount, headingDeg));
}

RotationProgress::ProgressResult RotationProgress::sliceProgress(uint32_t sliceId, uint32_t tagId, uint32_t samplesHave, uint32_t samplesNeeded, uint32_t sampleRateHz)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_active || _phase != Phase::Slice || !_currentSliceId || *_currentSliceId != sliceId
        || sampleRateHz == 0 || samplesNeeded == 0) {
        return ProgressResult::Ignored;
    }
    // Ceil divisions written to avoid uint32 wrap on absurd wire values (samplesNeeded != 0 checked above).
    const uint32_t segmentSeconds = (samplesNeeded - 1) / sampleRateHz + 1;
    const auto it = _sliceProgressByTag.find(tagId);
    if (it != _sliceProgressByTag.end() && it->second.segmentSeconds != segmentSeconds) {
        return ProgressResult::SegmentChanged;
    }
    // Each detector's segment length (K, PRI) differs; the slice lasts as long as the
    // longest one, so the dwell is the largest reported. The begin() estimate only sized the bar.
    _dwellSteps   = _dwellLearned ? std::max(_dwellSteps, segmentSeconds) : segmentSeconds;
    _dwellLearned = true;
    const uint32_t have      = std::min(samplesHave, samplesNeeded);
    const uint32_t remaining = samplesNeeded - have;
    TagProgress progress;
    progress.segmentSeconds   = segmentSeconds;
    progress.haveSeconds      = have / sampleRateHz;
    progress.remainingSeconds = remaining == 0 ? 0 : (remaining - 1) / sampleRateHz + 1;
    progress.discardedSeconds = 0;
    if (it != _sliceProgressByTag.end()) {
        progress.discardedSeconds = it->second.discardedSeconds;
        if (it->second.haveSeconds > progress.haveSeconds + kRestartToleranceSeconds) {
            progress.discardedSeconds += it->second.haveSeconds;   // segment restarted after an IQ gap: that work is redone
        }
    }
    _sliceProgressByTag[tagId] = progress;
    _publishLocked("");   // step only; the slice message stands
    return ProgressResult::Accepted;
}

void RotationProgress::computeTick()
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_active || _phase != Phase::Slice || !_currentSliceId || !_segmentsFullLocked()
        || _computeTicks >= kComputeSteps) {
        return;
    }
    ++_computeTicks;
    _publishLocked("");
}

void RotationProgress::sliceComplete(uint32_t sliceId)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_active || _phase != Phase::Slice || !_currentSliceId || *_currentSliceId != sliceId) {
        return;
    }
    _slicesComplete = _slicesArmed;
    _currentSliceId.reset();
    _completedExtraSteps += _currentExtraLocked();
    _sliceProgressByTag.clear();
    _publishLocked("");   // step only; the next ARM names the next slice
}

void RotationProgress::revisitRequested(float headingDeg)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_active) {
        return;
    }
    _phase = Phase::Slice;
    _sliceCount = std::max(_sliceCount, _slicesArmed) + 1;
    _publishLocked(formatString("Revisit %03.0f deg", headingDeg));
}

void RotationProgress::finalizeStage(uint32_t stage, const std::string& message)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_active) {
        return;
    }
    _phase = Phase::Finalize;
    _slicesComplete = _slicesArmed;
    _currentSliceId.reset();
    _completedExtraSteps += _currentExtraLocked();
    _sliceProgressByTag.clear();
    _finalizeStage = std::min(stage, kFinalizeSteps - 1);
    _publishLocked(message);
}

void RotationProgress::finish(bool success, const std::string& message)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_active) {
        return;
    }
    _active = false;
    _reporter.finish(success, message);
}

uint32_t RotationProgress::_currentExtraLocked() const
{
    // Detectors dwell in parallel, so a shared IQ gap extends the slice by the
    // most any one of them threw away, not by the sum.
    uint32_t extra = 0;
    for (const auto& [tag, progress] : _sliceProgressByTag) {
        extra = std::max(extra, progress.discardedSeconds);
    }
    return extra;
}

uint32_t RotationProgress::_sliceSecondsLocked() const
{
    // Each detector is credited with the work it has actually done in this
    // slice (seconds received plus seconds discarded by its own restarts); the
    // slowest one governs, so a restart elsewhere cannot move the step while a
    // detector that has received nothing is still at zero. A detector whose
    // segment is full has nothing left to wait on and no longer holds the step
    // back while longer segments finish. Unreported detectors count as zero.
    if (_sliceProgressByTag.empty() || _sliceProgressByTag.size() < _detectorCount) {
        return 0;
    }
    const uint32_t sliceDwell = _dwellSteps + _currentExtraLocked();
    uint32_t slowest = sliceDwell;
    for (const auto& [tag, progress] : _sliceProgressByTag) {
        const uint32_t credited = progress.remainingSeconds == 0 ? sliceDwell
                                                                 : progress.discardedSeconds + progress.haveSeconds;
        slowest = std::min(slowest, credited);
    }
    return slowest;
}

bool RotationProgress::_segmentsFullLocked() const
{
    if (_sliceProgressByTag.size() < _detectorCount) {
        return false;
    }
    return std::all_of(_sliceProgressByTag.begin(), _sliceProgressByTag.end(),
                       [](const auto& entry) { return entry.second.remainingSeconds <= kFullToleranceSeconds; });
}

uint32_t RotationProgress::_stepLocked() const
{
    switch (_phase) {
    case Phase::Startup:
        return _processesStarted + _detectorsReady;
    case Phase::Slice: {
        uint32_t step = _startupSteps() + _slicesComplete * _sliceSteps() + _completedExtraSteps;
        if (_currentSliceId) {
            step += 1 + _sliceSecondsLocked() + _computeTicks;
        }
        return step;
    }
    case Phase::Finalize:
        return _startupSteps() + _sliceCount * _sliceSteps() + _completedExtraSteps + _finalizeStage;
    }
    return 0;
}

void RotationProgress::_publishLocked(const std::string& message)
{
    const uint32_t total = _totalSteps();
    // Monotonic unless the layout itself changed (learned dwell, grown slice count, segment restart).
    uint32_t step = _stepLocked();
    if (total == _reporterTotal) {
        step = std::max(step, _lastStep);
    }
    _reporterTotal = total;
    _lastStep = step;
    _reporter.update(step, total, message);
}
