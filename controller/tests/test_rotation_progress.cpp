// RotationProgress: step layout, monotonic advance, learned dwell, revisit growth, background analysis, terminal frames.

#include "RotationProgress.h"
#include "detector_protocol.h"
#include "test_check.h"

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

using namespace TunnelProtocol;
using TagTrackerDetectorProtocol::ComputeStage;

namespace {

struct Sink {
    std::vector<OperationProgress_t> frames;
    OperationProgressReporter reporter { [this](const OperationProgress_t& f) { frames.push_back(f); } };
    RotationProgress rotation { reporter };
    const OperationProgress_t& last() const { return frames.back(); }
    std::string message() const { return last().message; }
};

uint16_t stage(ComputeStage value) { return static_cast<uint16_t>(value); }

// begin(): 2 startup procs + 1 detector, 2 slices, 10 s dwell estimate.
constexpr uint32_t kProcs = 3, kDetectors = 1, kSlices = 2, kDwell = 10;
constexpr uint32_t kWait = RotationProgress::kAnalysisEstimateSteps;
constexpr uint32_t kSliceSteps = 2 + kDwell;
constexpr uint32_t kTotal = kProcs + kDetectors + kSlices * kSliceSteps + kWait + RotationProgress::kFinalizeSteps;

void checkMonotonic(const Sink& s)
{
    for (size_t i = 1; i < s.frames.size(); ++i) {
        // A layout change (new step_count) may move the step; otherwise never backwards.
        if (s.frames[i].step_count == s.frames[i - 1].step_count) {
            CHECK(s.frames[i].step >= s.frames[i - 1].step);
        }
    }
}

void testFullRotation()
{
    Sink s;
    CHECK(s.rotation.begin(7, kSlices, kDetectors, kProcs, kDwell));
    CHECK(s.rotation.active());
    CHECK(s.reporter.busy());
    CHECK(s.last().command == COMMAND_ID_START_COLLECTION);
    CHECK(s.last().request_id == 7);
    CHECK(s.last().state == OPERATION_STATE_RUNNING);
    CHECK(s.last().step == 0 && s.last().step_count == kTotal);
    CHECK(s.reporter.busyMessage() == "Busy: Rotation in progress");

    s.rotation.processStarted("iq_simulator");
    s.rotation.processStarted("airspyhf_decimator");
    s.rotation.processStarted("pulse_detector_2");
    CHECK(s.last().step == 3);
    s.rotation.processStarted("extra");                       // clamped to the announced count
    CHECK(s.last().step == 3);

    s.rotation.detectorReady(1);
    CHECK(s.last().step == kProcs + kDetectors);
    CHECK(s.message() == "Ready 1/1");

    // Slice 1
    s.rotation.sliceArmed(1, 90.0f);
    const uint32_t slice1Base = kProcs + kDetectors;
    CHECK(s.last().step == slice1Base + 1);
    CHECK(s.message() == "1/2 090 deg");
    s.rotation.sliceArmed(1, 90.0f);                          // GCS ARM retry: no change
    CHECK(s.last().step == slice1Base + 1);

    // Detector reports a real 12 s segment at 3840 Hz: layout re-sized.
    const uint32_t fs = 3840, needed = 12 * fs;
    s.rotation.sliceProgress(1, 2, 3 * fs, needed, fs);
    const uint32_t learnedSliceSteps = 2 + 12;
    const uint32_t learnedTotal = kProcs + kDetectors + kSlices * learnedSliceSteps + kWait + RotationProgress::kFinalizeSteps;
    CHECK(s.last().step_count == learnedTotal);
    CHECK(s.last().step == slice1Base + 1 + 3);
    CHECK(s.message() == "1/2 090 deg");                    // dwell moves the bar, not the text

    s.rotation.sliceProgress(1, 2, 2 * fs, needed, fs);      // reordered datagram: never backwards
    CHECK(s.last().step == slice1Base + 1 + 3);
    // A detector whose segment length changes mid-slice is buggy; the report is dropped.
    CHECK(s.rotation.sliceProgress(1, 2, 5 * fs, 30 * fs, fs) == RotationProgress::ProgressResult::SegmentChanged);
    CHECK(s.last().step == slice1Base + 1 + 3);
    CHECK(s.last().step_count == learnedTotal);
    CHECK(s.rotation.sliceProgress(9, 2, 5 * fs, needed, fs) == RotationProgress::ProgressResult::Ignored);
    CHECK(s.rotation.sliceProgress(1, 2, 11 * fs, needed, fs) == RotationProgress::ProgressResult::Accepted);
    CHECK(s.last().step == slice1Base + 1 + 11);
    s.rotation.sliceProgress(9, 2, 12 * fs, needed, fs);     // unknown slice ignored
    CHECK(s.last().step == slice1Base + 1 + 11);

    s.rotation.sliceComplete(1);
    CHECK(s.last().step == slice1Base + learnedSliceSteps);
    CHECK(s.message() == "1/2 090 deg");

    // Slice 2, completed without any progress reports.
    s.rotation.sliceArmed(2, 180.0f);
    CHECK(s.last().step == slice1Base + learnedSliceSteps + 1);
    s.rotation.sliceComplete(2);
    CHECK(s.last().step == slice1Base + 2 * learnedSliceSteps);

    // Finalize
    // Finalize without an analysis wait (no held slice): the reserved wait steps are dropped.
    const uint32_t finalTotal = learnedTotal - kWait;
    s.rotation.finalizeStage(0, "Stopping detectors");
    CHECK(s.last().step_count == finalTotal);
    CHECK(s.last().step == finalTotal - 3);
    s.rotation.finalizeStage(1, "Computing bearing");
    CHECK(s.last().step == finalTotal - 2);
    s.rotation.finalizeStage(2, "Sending results");
    CHECK(s.last().step == finalTotal - 1);

    s.rotation.finish(true, "Rotation complete");
    CHECK(!s.rotation.active());
    CHECK(!s.reporter.busy());
    CHECK(s.last().state == OPERATION_STATE_COMPLETE);
    CHECK(s.last().step == finalTotal);
    CHECK(s.message() == "Rotation complete");

    // Post-finish events are ignored, and the reporter is free again.
    const size_t frames = s.frames.size();
    s.rotation.sliceArmed(3, 0.0f);
    s.rotation.finalizeStage(0, "x");
    CHECK(s.frames.size() == frames);
    CHECK(s.reporter.begin(COMMAND_ID_SAVE_LOGS, 1, "Saving logs"));

    checkMonotonic(s);
}

void testRevisitGrowsStepCount()
{
    Sink s;
    CHECK(s.rotation.begin(1, 1, 1, 2, 5));
    const uint32_t total1 = s.last().step_count;
    s.rotation.sliceArmed(1, 0.0f);
    s.rotation.sliceComplete(1);
    const uint32_t afterSlice = s.last().step;

    s.rotation.revisitRequested(45.0f);
    CHECK(s.last().step_count == total1 + 2 + 5);
    CHECK(s.last().step == afterSlice);
    CHECK(s.message() == "Revisit 045 deg");

    s.rotation.sliceArmed(2, 45.0f);
    CHECK(s.message() == "2/2 045 deg");
    s.rotation.sliceComplete(2);
    s.rotation.finalizeStage(2, "Sending results");
    CHECK(s.last().step == s.last().step_count - 1);
    s.rotation.finish(true, "done");
    CHECK(s.last().step == s.last().step_count);
    checkMonotonic(s);
}

void testSlowestDetectorGoverns()
{
    Sink s;
    CHECK(s.rotation.begin(1, 1, 2, 2, 10));
    s.rotation.detectorReady(2);
    s.rotation.sliceArmed(1, 0.0f);
    const uint32_t base = s.last().step;
    const uint32_t fs = 1000, needed = 10 * fs;
    s.rotation.sliceProgress(1, 10, 8 * fs, needed, fs);
    CHECK(s.last().step == base);                              // second detector not heard from yet
    s.rotation.sliceProgress(1, 11, 3 * fs, needed, fs);
    CHECK(s.last().step == base + 3);
    s.rotation.sliceProgress(1, 11, 9 * fs, needed, fs);
    CHECK(s.last().step == base + 8);
}

// Per-tag K/PRI give unequal segment lengths: the dwell is the longest one, and a
// short detector that has finished must not freeze the step while the long one runs.
void testUnequalDetectorDurations()
{
    Sink s;
    constexpr uint32_t procs = 2, detectors = 2;
    CHECK(s.rotation.begin(1, 1, detectors, procs, 10));
    s.rotation.detectorReady(2);
    s.rotation.sliceArmed(1, 0.0f);
    const uint32_t base = s.last().step;
    const uint32_t fs = 1000, shortNeeded = 3 * fs, longNeeded = 8 * fs;

    s.rotation.sliceProgress(1, 10, 1 * fs, shortNeeded, fs);   // short detector first: dwell 3
    CHECK(s.last().step_count == procs + detectors + (2 + 3) + kWait + RotationProgress::kFinalizeSteps);
    CHECK(s.last().step == base);                              // long detector not heard from yet
    s.rotation.sliceProgress(1, 11, 1 * fs, longNeeded, fs);    // dwell grows to 8
    CHECK(s.last().step_count == procs + detectors + (2 + 8) + kWait + RotationProgress::kFinalizeSteps);
    CHECK(s.last().step == base + 1);

    s.rotation.sliceProgress(1, 10, 3 * fs, shortNeeded, fs);   // short detector done
    s.rotation.sliceProgress(1, 11, 3 * fs, longNeeded, fs);
    CHECK(s.last().step == base + 3);
    s.rotation.sliceProgress(1, 11, 6 * fs, longNeeded, fs);    // keeps moving after the short one finished
    CHECK(s.last().step == base + 6);
    s.rotation.sliceProgress(1, 11, 8 * fs, longNeeded, fs);
    CHECK(s.last().step == base + 8);

    s.rotation.sliceComplete(1);
    CHECK(s.last().step == base - 1 + 2 + 8);
    checkMonotonic(s);
}

// After an IQ gap the detector restarts its segment and samples_have drops to ~0.
// The discarded seconds become extra steps for the slice: the step holds on the
// restart frame, then keeps advancing rather than waiting for the detector to
// climb back to its old high-water mark.
void testSegmentRestartAddsWork()
{
    Sink s;
    constexpr uint32_t procs = 2, detectors = 1;
    CHECK(s.rotation.begin(1, 1, detectors, procs, 20));
    s.rotation.detectorReady(1);
    s.rotation.sliceArmed(1, 0.0f);
    const uint32_t base = s.last().step;
    const uint32_t fs = 1000, needed = 20 * fs;
    const uint32_t total = s.last().step_count;

    s.rotation.sliceProgress(1, 5, 12 * fs, needed, fs);
    CHECK(s.last().step == base + 12);
    CHECK(s.last().step_count == total);

    s.rotation.sliceProgress(1, 5, fs / 2, needed, fs);        // restart: 12 s of work discarded
    CHECK(s.last().step_count == total + 12);
    CHECK(s.last().step == base + 12);
    s.rotation.sliceProgress(1, 5, 3 * fs / 2, needed, fs);
    CHECK(s.last().step == base + 13);
    s.rotation.sliceProgress(1, 5, 20 * fs, needed, fs);
    CHECK(s.last().step == base + 32);

    s.rotation.sliceComplete(1);
    CHECK(s.last().step_count == total + 12);
    CHECK(s.last().step == base - 1 + 2 + 20 + 12);
    s.rotation.finalizeStage(2, "Sending results");
    CHECK(s.last().step == s.last().step_count - 1);
    s.rotation.finish(true, "done");
    CHECK(s.last().step == s.last().step_count);
    checkMonotonic(s);
}

// Two detectors hit the same IQ gap: they dwell in parallel, so the slice grows by
// the discarded work once, not once per detector.
void testSimultaneousRestart()
{
    Sink s;
    CHECK(s.rotation.begin(1, 1, 2, 2, 20));
    s.rotation.detectorReady(2);
    s.rotation.sliceArmed(1, 0.0f);
    const uint32_t base = s.last().step;
    const uint32_t fs = 1000, needed = 20 * fs;
    s.rotation.sliceProgress(1, 10, 12 * fs, needed, fs);
    s.rotation.sliceProgress(1, 11, 12 * fs, needed, fs);
    const uint32_t total = s.last().step_count;
    CHECK(s.last().step == base + 12);

    s.rotation.sliceProgress(1, 10, fs / 2, needed, fs);
    CHECK(s.last().step_count == total + 12);
    CHECK(s.last().step == base + 12);
    s.rotation.sliceProgress(1, 11, fs / 2, needed, fs);      // same gap seen by the second detector
    CHECK(s.last().step_count == total + 12);
    CHECK(s.last().step == base + 12);

    s.rotation.sliceProgress(1, 10, 5 * fs, needed, fs);
    CHECK(s.last().step == base + 12);                          // slower detector still governs
    s.rotation.sliceProgress(1, 11, 4 * fs, needed, fs);
    CHECK(s.last().step == base + 16);
    checkMonotonic(s);
}

// One detector restarts while the other has received nothing: the restart credit
// belongs to the detector that earned it, so the slowest (idle) one still holds the step.
void testAsymmetricRestart()
{
    Sink s;
    CHECK(s.rotation.begin(1, 1, 2, 2, 20));
    s.rotation.detectorReady(2);
    s.rotation.sliceArmed(1, 0.0f);
    const uint32_t base = s.last().step;
    const uint32_t fs = 1000, needed = 20 * fs;
    s.rotation.sliceProgress(1, 10, 12 * fs, needed, fs);
    s.rotation.sliceProgress(1, 11, 0, needed, fs);
    const uint32_t total = s.last().step_count;
    CHECK(s.last().step == base);

    s.rotation.sliceProgress(1, 10, fs / 2, needed, fs);      // A restarts; B still idle
    CHECK(s.last().step_count == total + 12);
    CHECK(s.last().step == base);
    s.rotation.sliceProgress(1, 10, 3 * fs, needed, fs);
    CHECK(s.last().step == base);                              // A alone cannot move the step

    s.rotation.sliceProgress(1, 11, 5 * fs, needed, fs);
    CHECK(s.last().step == base + 5);
    s.rotation.sliceProgress(1, 11, 16 * fs, needed, fs);
    CHECK(s.last().step == base + 15);                         // now A (12 + 3) is the slowest
    checkMonotonic(s);
}

// Slice 1 is analysed while slice 2 is flown; the bar only waits on the last
// slice, and those wait steps tick from the heartbeat while the detector
// keeps reporting. The stage shows in the message.
void testBackgroundAnalysis()
{
    Sink s;
    constexpr uint32_t procs = 2, detectors = 1, slices = 2, dwell = 10, tag = 7;
    CHECK(s.rotation.begin(1, slices, detectors, procs, dwell));
    s.rotation.detectorReady(1);
    s.rotation.sliceArmed(1, 90.0f);
    const uint32_t base = s.last().step;
    const uint32_t fs = 1000, needed = dwell * fs;
    const uint32_t total = procs + detectors + slices * (2 + dwell) + kWait + RotationProgress::kFinalizeSteps;
    CHECK(s.last().step_count == total);

    s.rotation.sliceProgress(1, tag, needed, needed, fs);
    CHECK(s.last().step == base + dwell);
    s.rotation.analysisQueued(tag);
    s.rotation.sliceComplete(1);
    const uint32_t slice2Base = base - 1 + 2 + dwell;
    CHECK(s.last().step == slice2Base);
    s.rotation.computeProgress(1, tag, stage(ComputeStage::Null), 3, 40);
    CHECK(s.message() == "1/2 090 deg");                       // between slices the text stands

    s.rotation.sliceArmed(2, 180.0f);
    CHECK(s.message() == "2/2 180 deg | 090 deg: null 3/40");
    for (int i = 0; i < 3; ++i) {
        s.rotation.computeTick();                               // not waiting on analysis: no steps
    }
    CHECK(s.last().step == slice2Base + 1);
    s.rotation.sliceProgress(2, tag, 4 * fs, needed, fs);
    CHECK(s.last().step == slice2Base + 1 + 4);
    s.rotation.analysisDone(tag);
    CHECK(s.message() == "2/2 180 deg");
    s.rotation.computeProgress(1, tag, stage(ComputeStage::Null), 4, 40);   // late report for a finished analysis
    CHECK(s.message() == "2/2 180 deg");
    s.rotation.sliceProgress(2, tag, needed, needed, fs);

    // Last slice: captured, then held until its analysis is done.
    s.rotation.analysisQueued(tag);
    s.rotation.sliceCaptured(2);
    const uint32_t waitBase = slice2Base + 2 + dwell;
    CHECK(s.last().step == waitBase);
    CHECK(s.message() == "Analysing");
    s.rotation.computeProgress(2, tag, stage(ComputeStage::Spectrogram), 0, 0);
    CHECK(s.message() == "Analysing 180 deg: spectrogram");

    const uint32_t ticks = kWait + 5;                           // runs past the estimate: layout grows
    for (uint32_t i = 1; i <= ticks; ++i) {
        s.rotation.computeProgress(2, tag, stage(ComputeStage::Null), i, 40);
        s.rotation.computeTick();
        CHECK(s.last().step == waitBase + i);
        CHECK(s.last().step_count == total - kWait + std::max(kWait, i));
    }
    s.rotation.analysisDone(tag);
    s.rotation.sliceComplete(2);
    const uint32_t finalTotal = total - kWait + ticks;
    CHECK(s.last().step == waitBase + ticks);
    CHECK(s.last().step_count == finalTotal);
    s.rotation.computeTick();                                   // wait over: no more steps
    CHECK(s.last().step == waitBase + ticks);

    s.rotation.finalizeStage(0, "Stopping");
    CHECK(s.last().step == finalTotal - RotationProgress::kFinalizeSteps);
    s.rotation.finish(true, "done");
    CHECK(s.last().step == finalTotal);
    checkMonotonic(s);
}

// An analysis that stops reporting freezes the step even while the next slice's
// dwell is still delivering IQ, so the GCS watchdog sees the hung detector.
void testSilentAnalysisFreezesStep()
{
    Sink s;
    constexpr uint32_t dwell = 20, tag = 7;
    CHECK(s.rotation.begin(1, 3, 1, 2, dwell));
    s.rotation.detectorReady(1);
    s.rotation.sliceArmed(1, 0.0f);
    const uint32_t fs = 1000, needed = dwell * fs;
    s.rotation.sliceProgress(1, tag, needed, needed, fs);
    s.rotation.analysisQueued(tag);
    s.rotation.sliceComplete(1);
    s.rotation.sliceArmed(2, 120.0f);
    const uint32_t slice2Base = s.last().step;
    s.rotation.computeProgress(1, tag, stage(ComputeStage::Search), 0, 0);

    constexpr uint32_t timeout = RotationProgress::kComputeProgressTimeoutSeconds;
    for (uint32_t i = 1; i <= timeout; ++i) {
        CHECK(s.rotation.computeTick().empty());
        s.rotation.sliceProgress(2, tag, i * fs, needed, fs);
        CHECK(s.last().step == slice2Base + i);
    }
    const auto stalls = s.rotation.computeTick();               // one tick too many without a report
    CHECK(stalls.size() == 1);
    CHECK(stalls.front().find("detector 7") != std::string::npos);
    CHECK(stalls.front().find("no COMPUTE_PROGRESS") != std::string::npos);
    CHECK(stalls.front().find("slice 1") != std::string::npos);
    CHECK(stalls.front().find("'search'") != std::string::npos);
    CHECK(s.rotation.computeTick().empty());                    // reported once per stall
    s.rotation.sliceProgress(2, tag, (timeout + 1) * fs, needed, fs);
    CHECK(s.last().step == slice2Base + timeout);
    s.rotation.sliceProgress(2, tag, (timeout + 2) * fs, needed, fs);
    CHECK(s.last().step == slice2Base + timeout);

    s.rotation.computeProgress(1, tag, stage(ComputeStage::Null), 1, 40);   // reporting again
    CHECK(s.last().step == slice2Base + timeout + 2);
    checkMonotonic(s);
}

// An analysis that keeps reporting but never finishes is capped.
void testRunawayAnalysisIsCapped()
{
    Sink s;
    constexpr uint32_t dwell = 10, tag = 7;
    CHECK(s.rotation.begin(1, 1, 1, 2, dwell));
    s.rotation.detectorReady(1);
    s.rotation.sliceArmed(1, 0.0f);
    const uint32_t fs = 1000;
    s.rotation.sliceProgress(1, tag, dwell * fs, dwell * fs, fs);
    s.rotation.analysisQueued(tag);
    s.rotation.sliceCaptured(1);
    const uint32_t waitBase = s.last().step;

    constexpr uint32_t cap = RotationProgress::kMaxAnalysisSeconds;
    for (uint32_t i = 1; i <= cap; ++i) {
        s.rotation.computeProgress(1, tag, stage(ComputeStage::Null), i % 40, 40);
        CHECK(s.rotation.computeTick().empty());
        CHECK(s.last().step == waitBase + i);
    }
    for (int i = 0; i < 3; ++i) {
        s.rotation.computeProgress(1, tag, stage(ComputeStage::Null), 1, 40);
        const auto stalls = s.rotation.computeTick();
        CHECK(stalls.size() == (i == 0 ? 1u : 0u));             // still reporting, so still one stall
        if (i == 0) {
            CHECK(stalls.front().find("cap") != std::string::npos);
        }
        CHECK(s.last().step == waitBase + cap);
    }

    s.rotation.analysisDone(tag);                               // a late finish releases the wait
    s.rotation.sliceComplete(1);
    s.rotation.finalizeStage(2, "Sending results");
    CHECK(s.last().step == s.last().step_count - 1);
    checkMonotonic(s);
}

// A lost CYCLE_COMPLETE must not let two analyses add up against the per-slice cap.
void testCapRestartsOnLaterSlice()
{
    Sink s;
    constexpr uint32_t dwell = 10, tag = 7, fs = 1000;
    CHECK(s.rotation.begin(1, 2, 1, 2, dwell));
    s.rotation.detectorReady(1);
    s.rotation.sliceArmed(1, 0.0f);
    s.rotation.sliceProgress(1, tag, dwell * fs, dwell * fs, fs);
    s.rotation.analysisQueued(tag);
    s.rotation.sliceComplete(1);
    s.rotation.sliceArmed(2, 180.0f);
    s.rotation.sliceProgress(2, tag, dwell * fs, dwell * fs, fs);
    s.rotation.analysisQueued(tag);
    s.rotation.sliceCaptured(2);
    uint32_t step = s.last().step;

    constexpr uint32_t cap = RotationProgress::kMaxAnalysisSeconds;
    for (uint32_t i = 0; i < cap - 10; ++i) {
        s.rotation.computeProgress(1, tag, stage(ComputeStage::Null), 1, 40);
        CHECK(s.rotation.computeTick().empty());
        CHECK(s.last().step == ++step);
    }
    for (uint32_t i = 0; i < 20; ++i) {                        // slice 1's CYCLE_COMPLETE never arrived
        s.rotation.computeProgress(2, tag, stage(ComputeStage::Null), 1, 40);
        CHECK(s.rotation.computeTick().empty());
        CHECK(s.last().step == ++step);
    }
    checkMonotonic(s);
}

void testComputeStageText()
{
    CHECK(RotationProgress::computeStageText(stage(ComputeStage::Null), 24, 40) == "null 24/40");
    CHECK(RotationProgress::computeStageText(stage(ComputeStage::Refit), 1, 3) == "refit 1/3");
    CHECK(RotationProgress::computeStageText(stage(ComputeStage::Spectrogram), 0, 0) == "spectrogram");
    CHECK(RotationProgress::computeStageText(99, 0, 0) == "stage 99");
}

// An analysis that finishes while the vehicle yaws between slices must not
// leave its stage text on the bar until the next ARM.
void testAnalysisTextClearedBetweenSlices()
{
    Sink s;
    constexpr uint32_t dwell = 10, tag = 7, fs = 1000;
    CHECK(s.rotation.begin(1, 3, 1, 2, dwell));
    s.rotation.detectorReady(1);
    s.rotation.sliceArmed(1, 90.0f);
    s.rotation.sliceProgress(1, tag, dwell * fs, dwell * fs, fs);
    s.rotation.analysisQueued(tag);
    s.rotation.sliceComplete(1);
    s.rotation.sliceArmed(2, 180.0f);
    s.rotation.computeProgress(1, tag, stage(ComputeStage::Measure), 2, 8);
    CHECK(s.message() == "2/3 180 deg | 090 deg: measure 2/8");
    s.rotation.sliceProgress(2, tag, dwell * fs, dwell * fs, fs);
    s.rotation.sliceComplete(2);
    CHECK(s.message() == "2/3 180 deg | 090 deg: measure 2/8");
    s.rotation.analysisDone(tag);
    CHECK(s.message() == "2/3 180 deg");
}

void testFailureAndBusyGate()
{
    Sink s;
    CHECK(s.reporter.begin(COMMAND_ID_SAVE_LOGS, 1, "Saving logs"));
    CHECK(!s.rotation.begin(2, 8, 1, 3, 10));                 // reporter owned by another operation
    CHECK(!s.rotation.active());
    s.reporter.finish(true);

    CHECK(s.rotation.begin(2, 8, 1, 3, 10));
    s.rotation.sliceArmed(1, 0.0f);
    s.rotation.finish(false, "Detector 2 exited (code 1)");
    CHECK(s.last().state == OPERATION_STATE_FAILED);
    CHECK(s.message() == "Detector 2 exited (code 1)");
    CHECK(!s.reporter.busy());
}

} // namespace

int main()
{
    testFullRotation();
    testRevisitGrowsStepCount();
    testSlowestDetectorGoverns();
    testUnequalDetectorDurations();
    testSegmentRestartAddsWork();
    testSimultaneousRestart();
    testAsymmetricRestart();
    testBackgroundAnalysis();
    testSilentAnalysisFreezesStep();
    testRunawayAnalysisIsCapped();
    testCapRestartsOnLaterSlice();
    testComputeStageText();
    testAnalysisTextClearedBetweenSlices();
    testFailureAndBusyGate();
    std::printf("test_rotation_progress: all tests passed\n");
    return 0;
}
