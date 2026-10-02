#pragma once

#include "OperationProgress.h"

#include <algorithm>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

// Publishes one OPERATION_PROGRESS operation (command START_COLLECTION) that
// spans a whole rotation, advanced only by real events: pipeline processes
// started, detector READY/ARMED/SLICE_PROGRESS/SLICE_CAPTURED/COMPUTE_PROGRESS/
// CYCLE_COMPLETE, finalize stages. The GCS treats a step that has not advanced
// for a while as a stalled rotation.
//
// Step layout:
//   [startup processes][detectors READY]
//   per slice: [ARMED][one step per second of dwell][CAPTURED]
//   [analysis wait, >= kAnalysisEstimateSteps][finalize stages]
// The dwell length is estimated at begin() and corrected from SLICE_PROGRESS
// reports, which carry each detector's real segment length; the longest wins.
// A detector restarts its segment after an IQ gap; the seconds it discards are
// added to the current slice as extra steps so the bar keeps moving instead of
// sitting at its old high-water mark until the detector catches back up.
// Detectors analyse each captured slice in the background while the next one
// is flown; only the last slice waits for the analyses, and those wait steps
// are ticked from the 1 Hz heartbeat. Every running analysis must keep sending
// COMPUTE_PROGRESS: one that goes quiet, or runs past kMaxAnalysisSeconds,
// freezes the step (dwell included) so the GCS stall watchdog fires.
class RotationProgress {
public:
    static constexpr uint32_t kFinalizeSteps = 3;   // stopping detectors, computing bearing, sending results
    // Steps reserved for the wait on the last slice's analysis; a slice measured
    // 23-36 s on the flight computer (#173). A longer wait grows the layout.
    static constexpr uint32_t kAnalysisEstimateSteps = 30;
    // Heartbeat ticks an analysing detector may go without COMPUTE_PROGRESS
    // (sent ~1 Hz; one PRI re-fit step takes up to ~4 s late in a rotation).
    static constexpr uint32_t kComputeProgressTimeoutSeconds = 5;
    // One slice's analysis normally takes 23-36 s; past this it is treated as
    // hung even while it keeps reporting.
    static constexpr uint32_t kMaxAnalysisSeconds = 120;
    // A samples_have drop larger than this is a segment restart; a reordered
    // datagram (reports are <= 1 Hz) regresses by at most about one second.
    static constexpr uint32_t kRestartToleranceSeconds = 2;

    explicit RotationProgress(OperationProgressReporter& reporter);

    /// Claims the reporter. Returns false if another operation is running.
    bool begin(uint32_t requestId, uint32_t sliceCount, uint32_t detectorCount,
               uint32_t startupProcessEstimate, double estimatedDwellSeconds);
    bool active() const;

    /// The pipeline knows its real process count only once the SDR type is known.
    void setStartupProcessCount(uint32_t count);
    void processStarted(const std::string& name);
    void detectorReady(uint32_t readyCount);

    void sliceArmed(uint32_t sliceId, float headingDeg);
    enum class ProgressResult { Accepted, Ignored, SegmentChanged };
    /// SegmentChanged: the detector's segment length differs from its earlier reports
    /// in this slice (a detector bug); the report is dropped so it cannot inflate the layout.
    ProgressResult sliceProgress(uint32_t sliceId, uint32_t tagId, uint32_t samplesHave, uint32_t samplesNeeded, uint32_t sampleRateHz);
    /// Every detector has captured the last slice; the GCS is held until the analyses finish.
    void sliceCaptured(uint32_t sliceId);
    /// SLICE_COMPLETE sent; for a held slice this ends the analysis wait.
    void sliceComplete(uint32_t sliceId);
    /// A detector captured a slice; its analysis is queued.
    void analysisQueued(uint32_t tagId);
    void computeProgress(uint32_t sliceId, uint32_t tagId, uint16_t stage, uint32_t done, uint32_t total);
    /// CYCLE_COMPLETE: the detector finished analysing one slice.
    void analysisDone(uint32_t tagId);
    /// 1 Hz from the heartbeat: ages running analyses and advances the analysis wait.
    /// Returns one diagnostic per analysis that has just become stalled.
    std::vector<std::string> computeTick();
    /// One more slice will be flown; grows step_count.
    void revisitRequested(float headingDeg);

    /// stage is 0..kFinalizeSteps-1.
    void finalizeStage(uint32_t stage, const std::string& message);
    void finish(bool success, const std::string& message);

    static std::string computeStageText(uint16_t stage, uint32_t done, uint32_t total);

private:
    enum class Phase { Startup, Slice, Finalize };

    uint32_t _sliceSteps() const { return 2 + _dwellSteps; }
    uint32_t _startupSteps() const { return _startupProcesses + _detectorCount; }
    uint32_t _analysisWaitSteps() const
    {
        const uint32_t reserved = _analysisWaitPending ? kAnalysisEstimateSteps : 0;
        return std::max(reserved, _awaitingSliceId ? _analysisWaitTicks : 0u);
    }
    uint32_t _totalSteps() const { return _startupSteps() + _sliceCount * _sliceSteps() + _completedExtraSteps + _currentExtraLocked() + _completedWaitSteps + _analysisWaitSteps() + kFinalizeSteps; }
    uint32_t _stepLocked() const;
    uint32_t _sliceSecondsLocked() const;
    uint32_t _currentExtraLocked() const;
    bool     _analysisStalledLocked() const;
    void     _endCurrentSliceLocked();
    std::string _sliceMessageLocked() const;
    void     _publishLocked(const std::string& message);

    OperationProgressReporter&  _reporter;
    mutable std::mutex          _mutex;
    bool                        _active            = false;
    Phase                       _phase             = Phase::Startup;
    uint32_t                    _sliceCount        = 0;
    uint32_t                    _detectorCount     = 0;
    uint32_t                    _startupProcesses  = 0;
    uint32_t                    _dwellSteps        = 0;
    bool                        _dwellLearned      = false;
    // Startup phase
    uint32_t                    _processesStarted  = 0;
    uint32_t                    _detectorsReady    = 0;
    // Slice phase
    struct TagProgress {
        uint32_t segmentSeconds;     // ceil(samples_needed / fs); fixed per detector within a slice
        uint32_t haveSeconds;        // floor(samples_have / fs) of the last report
        uint32_t remainingSeconds;   // ceil((samples_needed - samples_have) / fs)
        uint32_t discardedSeconds;   // work thrown away by this detector's segment restarts in the slice
    };
    std::optional<uint32_t>     _currentSliceId;
    uint32_t                    _slicesArmed       = 0;     // index of the current slice is _slicesArmed - 1
    uint32_t                    _slicesComplete    = 0;
    std::map<uint32_t, TagProgress> _sliceProgressByTag;    // per detector, for the current slice
    uint32_t                    _completedExtraSteps = 0;   // restart seconds of completed slices (max per slice across detectors)
    float                       _currentHeadingDeg = 0.0f;
    std::string                 _sliceText;                 // "3/8 090 deg" for the slice being flown
    std::map<uint32_t, float>   _sliceHeadings;             // slice id -> heading, to name analysed slices
    // Background analysis
    struct Analysis {
        uint32_t pending          = 0;   // slices captured and not yet analysed
        uint32_t ticksSinceReport = 0;
        uint32_t ticksInAnalysis  = 0;   // of the slice being analysed now
        std::optional<uint32_t> lastSliceId;   // from the last COMPUTE_PROGRESS
        std::string lastStageText;
        bool stallReported        = false;
        bool stalled() const
        {
            return pending > 0 && (ticksSinceReport > kComputeProgressTimeoutSeconds || ticksInAnalysis > kMaxAnalysisSeconds);
        }
    };
    std::map<uint32_t, Analysis> _analysisByTag;
    std::string                 _computeText;               // "045 deg: null 24/40" while any analysis runs
    std::optional<uint32_t>     _awaitingSliceId;           // captured slice the GCS is held on
    bool                        _analysisWaitPending = true; // an analysis wait is still ahead in the layout
    uint32_t                    _analysisWaitTicks = 0;
    uint32_t                    _completedWaitSteps = 0;
    // Finalize phase
    uint32_t                    _finalizeStage     = 0;
    uint32_t                    _lastStep          = 0;
    uint32_t                    _reporterTotal     = 0;
};
