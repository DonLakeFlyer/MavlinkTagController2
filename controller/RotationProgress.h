#pragma once

#include "OperationProgress.h"

#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>

// Publishes one OPERATION_PROGRESS operation (command START_COLLECTION) that
// spans a whole rotation, advanced only by real events: pipeline processes
// started, detector READY/ARMED/SLICE_PROGRESS/CYCLE_COMPLETE, finalize stages.
// The GCS treats a step that has not advanced for a while as a stalled rotation.
//
// Step layout:
//   [startup processes][detectors READY]
//   per slice: [ARMED][one step per second of dwell][compute, <= kComputeSteps][COMPLETE]
//   [finalize stages]
// The dwell length is estimated at begin() and corrected from SLICE_PROGRESS
// reports, which carry each detector's real segment length; the longest wins.
// A detector restarts its segment after an IQ gap; the seconds it discards are
// added to the current slice as extra steps so the bar keeps moving instead of
// sitting at its old high-water mark until the detector catches back up.
// Once every detector has a full segment it goes quiet while it computes
// (STFT, fold, permutation null); the compute steps are ticked from the 1 Hz
// heartbeat so that silence is not mistaken for a dead stream, and are bounded
// so a detector hung in compute still trips the GCS watchdog.
class RotationProgress {
public:
    static constexpr uint32_t kFinalizeSteps = 3;   // stopping detectors, computing bearing, sending results
    // Seconds of detector compute the bar keeps moving for after the segment is
    // full: two full --null-time-budget passes plus STFT. Beyond this the step
    // stops and the GCS stall watchdog applies.
    static constexpr uint32_t kComputeSteps = 12;
    // A detector whose last report left at most this much remaining counts as
    // full: it sends one report per second and a final full one, so a lost
    // or slightly early last datagram must not keep the compute steps parked.
    static constexpr uint32_t kFullToleranceSeconds = 1;
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
    /// 1 Hz from the heartbeat: advances a compute step while every detector's segment is full.
    void computeTick();
    void sliceComplete(uint32_t sliceId);
    /// One more slice will be flown; grows step_count.
    void revisitRequested(float headingDeg);

    /// stage is 0..kFinalizeSteps-1.
    void finalizeStage(uint32_t stage, const std::string& message);
    void finish(bool success, const std::string& message);

private:
    enum class Phase { Startup, Slice, Finalize };

    uint32_t _sliceSteps() const { return 2 + _dwellSteps + kComputeSteps; }
    uint32_t _startupSteps() const { return _startupProcesses + _detectorCount; }
    uint32_t _totalSteps() const { return _startupSteps() + _sliceCount * _sliceSteps() + _completedExtraSteps + _currentExtraLocked() + kFinalizeSteps; }
    uint32_t _stepLocked() const;
    uint32_t _sliceSecondsLocked() const;
    bool     _segmentsFullLocked() const;
    uint32_t _currentExtraLocked() const;
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
    uint32_t                    _computeTicks      = 0;     // compute steps published for the current slice
    float                       _currentHeadingDeg = 0.0f;
    // Finalize phase
    uint32_t                    _finalizeStage     = 0;
    uint32_t                    _lastStep          = 0;
    uint32_t                    _reporterTotal     = 0;
};
