#include "RotationProgress.h"
#include "TunnelProtocol.h"
#include "detector_protocol.h"
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
    _sliceText.clear();
    _sliceHeadings.clear();
    _analysisByTag.clear();
    _computeText.clear();
    _awaitingSliceId.reset();
    _analysisWaitPending = true;
    _analysisWaitTicks = 0;
    _completedWaitSteps = 0;
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
    if (_sliceHeadings.contains(sliceId)) {
        return;   // GCS retry of an ARM already counted (the slice may be captured since)
    }
    _phase = Phase::Slice;
    _processesStarted = _startupProcesses;
    _detectorsReady = _detectorCount;
    _currentSliceId = sliceId;
    _currentHeadingDeg = headingDeg;
    _sliceHeadings[sliceId] = headingDeg;
    _sliceProgressByTag.clear();
    ++_slicesArmed;
    if (_slicesArmed > _sliceCount) {
        _sliceCount = _slicesArmed;   // unannounced extra slice: grow rather than clamp
    }
    _sliceText = formatString("%u/%u %03.0f deg", _slicesArmed, _sliceCount, headingDeg);
    _publishLocked(_sliceMessageLocked());
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

void RotationProgress::sliceCaptured(uint32_t sliceId)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_active || _phase != Phase::Slice || !_currentSliceId || *_currentSliceId != sliceId) {
        return;
    }
    _endCurrentSliceLocked();
    _awaitingSliceId = sliceId;
    _analysisWaitTicks = 0;
    _publishLocked(_sliceMessageLocked());
}

void RotationProgress::sliceComplete(uint32_t sliceId)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_active || _phase != Phase::Slice) {
        return;
    }
    if (_currentSliceId && *_currentSliceId == sliceId) {
        _endCurrentSliceLocked();
    } else if (_awaitingSliceId && *_awaitingSliceId == sliceId) {
        _awaitingSliceId.reset();
        _completedWaitSteps += _analysisWaitTicks;
        _analysisWaitTicks = 0;
        _analysisWaitPending = false;
    } else {
        return;
    }
    _publishLocked("");   // step only; the next ARM names the next slice
}

void RotationProgress::analysisQueued(uint32_t tagId)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_active) {
        return;
    }
    Analysis& analysis = _analysisByTag[tagId];
    if (analysis.pending == 0) {
        analysis.ticksSinceReport = 0;
        analysis.ticksInAnalysis = 0;
        analysis.lastSliceId.reset();
        analysis.lastStageText.clear();
        analysis.stallReported = false;
    }
    ++analysis.pending;
}

void RotationProgress::computeProgress(uint32_t sliceId, uint32_t tagId, uint16_t stage, uint32_t done, uint32_t total)
{
    std::lock_guard<std::mutex> lock(_mutex);
    const auto it = _analysisByTag.find(tagId);
    if (!_active || it == _analysisByTag.end() || it->second.pending == 0) {
        return;
    }
    it->second.ticksSinceReport = 0;
    const auto heading = _sliceHeadings.find(sliceId);
    const std::string stageText = computeStageText(stage, done, total);
    if (it->second.lastSliceId && sliceId > *it->second.lastSliceId) {
        it->second.ticksInAnalysis = 0;   // the earlier slice finished; its CYCLE_COMPLETE was lost
    }
    it->second.lastSliceId = sliceId;
    it->second.lastStageText = stageText;
    if (!it->second.stalled()) {
        it->second.stallReported = false;   // recovered: a later stall is reported again
    }
    _computeText = heading == _sliceHeadings.end()
        ? formatString("slice %u: %s", sliceId, stageText.c_str())
        : formatString("%03.0f deg: %s", heading->second, stageText.c_str());
    _publishLocked(_sliceMessageLocked());
}

void RotationProgress::analysisDone(uint32_t tagId)
{
    std::lock_guard<std::mutex> lock(_mutex);
    const auto it = _analysisByTag.find(tagId);
    if (!_active || it == _analysisByTag.end() || it->second.pending == 0) {
        return;
    }
    --it->second.pending;
    it->second.ticksSinceReport = 0;
    it->second.ticksInAnalysis = 0;
    it->second.lastSliceId.reset();
    it->second.lastStageText.clear();
    it->second.stallReported = false;
    const bool anyPending = std::any_of(_analysisByTag.begin(), _analysisByTag.end(),
                                        [](const auto& entry) { return entry.second.pending > 0; });
    if (!anyPending) {
        _computeText.clear();
    }
    _publishLocked(_sliceMessageLocked());
}

std::vector<std::string> RotationProgress::computeTick()
{
    std::lock_guard<std::mutex> lock(_mutex);
    std::vector<std::string> stalls;
    if (!_active) {
        return stalls;
    }
    for (auto& [tag, analysis] : _analysisByTag) {
        if (analysis.pending > 0) {
            ++analysis.ticksSinceReport;
            ++analysis.ticksInAnalysis;
        }
        if (!analysis.stalled() || analysis.stallReported) {
            continue;
        }
        analysis.stallReported = true;
        const std::string reason = analysis.ticksSinceReport > kComputeProgressTimeoutSeconds
            ? formatString("no COMPUTE_PROGRESS for %u s (limit %u s)", analysis.ticksSinceReport, kComputeProgressTimeoutSeconds)
            : formatString("running %u s (cap %u s)", analysis.ticksInAnalysis, kMaxAnalysisSeconds);
        const std::string slice = analysis.lastSliceId ? formatString("%u", *analysis.lastSliceId) : std::string("?");
        stalls.push_back(formatString("Rotation progress frozen: detector %u analysis stalled, %s, slice %s, last stage '%s', %u pending",
                                      tag, reason.c_str(), slice.c_str(),
                                      analysis.lastStageText.empty() ? "none" : analysis.lastStageText.c_str(),
                                      analysis.pending));
    }
    if (_phase != Phase::Slice || !_awaitingSliceId || _analysisStalledLocked()) {
        return stalls;
    }
    ++_analysisWaitTicks;
    _publishLocked("");
    return stalls;
}

void RotationProgress::revisitRequested(float headingDeg)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_active) {
        return;
    }
    _phase = Phase::Slice;
    _sliceCount = std::max(_sliceCount, _slicesArmed) + 1;
    _analysisWaitPending = true;
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
    _awaitingSliceId.reset();
    _completedWaitSteps += _analysisWaitTicks;
    _analysisWaitTicks = 0;
    _analysisWaitPending = false;
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

bool RotationProgress::_analysisStalledLocked() const
{
    return std::any_of(_analysisByTag.begin(), _analysisByTag.end(),
                       [](const auto& entry) { return entry.second.stalled(); });
}

void RotationProgress::_endCurrentSliceLocked()
{
    _slicesComplete = _slicesArmed;
    _currentSliceId.reset();
    _completedExtraSteps += _currentExtraLocked();
    _sliceProgressByTag.clear();
}

std::string RotationProgress::_sliceMessageLocked() const
{
    if (_awaitingSliceId) {
        return _computeText.empty() ? std::string("Analysing") : "Analysing " + _computeText;
    }
    if (_computeText.empty()) {
        return _sliceText;
    }
    // Between slices an analysis running on is not worth replacing the last slice text.
    return _currentSliceId ? _sliceText + " | " + _computeText : std::string();
}

std::string RotationProgress::computeStageText(uint16_t stage, uint32_t done, uint32_t total)
{
    using TagTrackerDetectorProtocol::ComputeStage;
    std::string name;
    switch (static_cast<ComputeStage>(stage)) {
    case ComputeStage::Spectrogram: name = "spectrogram"; break;
    case ComputeStage::Search:      name = "search"; break;
    case ComputeStage::Null:        name = "null"; break;
    case ComputeStage::Refit:       name = "refit"; break;
    case ComputeStage::Measure:     name = "measure"; break;
    default:                        name = formatString("stage %u", stage); break;
    }
    return total > 0 ? formatString("%s %u/%u", name.c_str(), done, total) : name;
}

uint32_t RotationProgress::_stepLocked() const
{
    switch (_phase) {
    case Phase::Startup:
        return _processesStarted + _detectorsReady;
    case Phase::Slice: {
        uint32_t step = _startupSteps() + _slicesComplete * _sliceSteps() + _completedExtraSteps + _completedWaitSteps;
        if (_currentSliceId) {
            step += 1 + _sliceSecondsLocked();
        }
        if (_awaitingSliceId) {
            step += _analysisWaitTicks;
        }
        return step;
    }
    case Phase::Finalize:
        return _totalSteps() - kFinalizeSteps + _finalizeStage;
    }
    return 0;
}

void RotationProgress::_publishLocked(const std::string& message)
{
    if (_analysisStalledLocked()) {
        // Hold the step so the GCS watchdog sees the hung analysis, even mid-dwell.
        _reporter.update(_lastStep, _reporterTotal, message);
        return;
    }
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
