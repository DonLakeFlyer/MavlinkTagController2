#include "CollectionCoordinator.h"
#include "test_check.h"

#include <cstdint>
#include <vector>

int main()
{
    CollectionCoordinator coordinator;

    CHECK(coordinator.state() == CollectionCoordinator::State::Inactive);
    CHECK(coordinator.start(100, {2, 4}, 2) == CollectionCoordinator::Result::Accepted);
    CHECK(coordinator.state() == CollectionCoordinator::State::Starting);
    CHECK(coordinator.start(100, {2, 4}, 2) == CollectionCoordinator::Result::Duplicate);
    CHECK(coordinator.start(100, {2, 4}, 3) == CollectionCoordinator::Result::Conflict);   // not a retry: slice count differs
    CHECK(coordinator.start(101, {2, 4}, 2) == CollectionCoordinator::Result::Conflict);

    CHECK(coordinator.detectorReady(100, 2) == CollectionCoordinator::Result::Accepted);
    CHECK(coordinator.state() == CollectionCoordinator::State::Starting);
    CHECK(coordinator.detectorReady(100, 2) == CollectionCoordinator::Result::Duplicate);
    CHECK(coordinator.detectorReady(99, 4) == CollectionCoordinator::Result::Stale);
    CHECK(coordinator.detectorReady(100, 4) == CollectionCoordinator::Result::Accepted);
    CHECK(coordinator.state() == CollectionCoordinator::State::Ready);

    CHECK(coordinator.armSlice(100, 1, 45.0f) == CollectionCoordinator::Result::Accepted);
    CHECK(coordinator.state() == CollectionCoordinator::State::CollectingSlice);
    CHECK(!coordinator.sliceArmed());
    CHECK(coordinator.detectorArmed(100, 1, 2) == CollectionCoordinator::Result::Accepted);
    CHECK(!coordinator.sliceArmed());
    CHECK(coordinator.detectorArmed(100, 1, 2) == CollectionCoordinator::Result::Duplicate);
    CHECK(coordinator.detectorArmed(100, 1, 4) == CollectionCoordinator::Result::Accepted);
    CHECK(coordinator.sliceArmed());
    CHECK(coordinator.armSlice(100, 1, 45.0f) == CollectionCoordinator::Result::Duplicate);
    CHECK(coordinator.armSlice(100, 2, 90.0f) == CollectionCoordinator::Result::Busy);
    CHECK(coordinator.detectorCaptured(100, 1, 2) == CollectionCoordinator::Result::Accepted);
    CHECK(coordinator.detectorCaptured(100, 1, 2) == CollectionCoordinator::Result::Duplicate);
    CHECK(coordinator.detectorCaptured(100, 0, 4) == CollectionCoordinator::Result::Stale);
    CHECK(coordinator.detectorCaptured(100, 1, 4) == CollectionCoordinator::Result::Accepted);
    // Not the last slice: complete for the GCS on capture, analysed in the background.
    CHECK(coordinator.state() == CollectionCoordinator::State::Ready);
    CHECK(coordinator.analysisPending());
    CHECK(coordinator.isLiveSlice(100, 1));
    CHECK(coordinator.detectorCaptured(100, 1, 4) == CollectionCoordinator::Result::Duplicate);

    // GCS missed SLICE_COMPLETE and retries the arm: replay, don't reject.
    CHECK(coordinator.armSlice(100, 1, 45.0f) == CollectionCoordinator::Result::AlreadyComplete);
    CHECK(coordinator.state() == CollectionCoordinator::State::Ready);

    CHECK(coordinator.armSlice(100, 3, 90.0f) == CollectionCoordinator::Result::OutOfOrder);
    CHECK(coordinator.armSlice(100, 2, 90.0f) == CollectionCoordinator::Result::Accepted);
    CHECK(coordinator.finalize(100) == CollectionCoordinator::Result::Busy);
    CHECK(coordinator.cancel(100) == CollectionCoordinator::Result::Accepted);
    CHECK(coordinator.state() == CollectionCoordinator::State::Inactive);
    CHECK(coordinator.cancel(100) == CollectionCoordinator::Result::Duplicate);
    // A FINALIZE after a CANCEL of the same id is not a retry of anything.
    CHECK(coordinator.finalize(100) == CollectionCoordinator::Result::Stale);

    CollectionCoordinator finalizable;
    CHECK(finalizable.start(200, {6}, 1) == CollectionCoordinator::Result::Accepted);
    CHECK(finalizable.detectorReady(200, 6) == CollectionCoordinator::Result::Accepted);
    CHECK(finalizable.finalize(200) == CollectionCoordinator::Result::Accepted);
    CHECK(finalizable.state() == CollectionCoordinator::State::Inactive);
    // Retried FINISH after a lost ACK must be idempotent, as cancel() is.
    CHECK(finalizable.finalize(200) == CollectionCoordinator::Result::Duplicate);
    CHECK(finalizable.finalize(201) == CollectionCoordinator::Result::Stale);
    // ...but the opposite disposition is not a duplicate either.
    CHECK(finalizable.cancel(200) == CollectionCoordinator::Result::Stale);

    // The last announced slice is held until every analysis has finished.
    CollectionCoordinator held;
    CHECK(held.start(300, {6}, 2) == CollectionCoordinator::Result::Accepted);
    CHECK(held.detectorReady(300, 6) == CollectionCoordinator::Result::Accepted);
    CHECK(held.armSlice(300, 1, 0.0f) == CollectionCoordinator::Result::Accepted);
    CHECK(held.detectorArmed(300, 1, 6) == CollectionCoordinator::Result::Accepted);
    CHECK(held.detectorCaptured(300, 1, 6) == CollectionCoordinator::Result::Accepted);
    CHECK(held.state() == CollectionCoordinator::State::Ready);
    CHECK(held.finalize(300) == CollectionCoordinator::Result::Busy);            // slice 1 still analysing
    CHECK(held.armSlice(300, 2, 180.0f) == CollectionCoordinator::Result::Accepted);
    CHECK(held.detectorArmed(300, 2, 6) == CollectionCoordinator::Result::Accepted);
    CHECK(held.detectorCaptured(300, 2, 6) == CollectionCoordinator::Result::Accepted);
    CHECK(held.state() == CollectionCoordinator::State::CollectingSlice);
    CHECK(held.sliceAwaitingAnalysis());
    CHECK(held.armSlice(300, 2, 180.0f) == CollectionCoordinator::Result::Duplicate);
    CHECK(held.detectorAnalysed(300, 1, 6) == CollectionCoordinator::Result::Accepted);
    CHECK(held.state() == CollectionCoordinator::State::CollectingSlice);     // slice 2 still analysing
    CHECK(held.detectorAnalysed(300, 1, 6) == CollectionCoordinator::Result::Duplicate);
    CHECK(!held.isLiveSlice(300, 1));
    CHECK(held.isLiveSlice(300, 2));
    CHECK(held.detectorAnalysed(300, 9, 6) == CollectionCoordinator::Result::Stale);
    CHECK(held.detectorAnalysed(300, 2, 7) == CollectionCoordinator::Result::Conflict);
    CHECK(held.detectorAnalysed(300, 2, 6) == CollectionCoordinator::Result::Accepted);
    CHECK(held.state() == CollectionCoordinator::State::Ready);
    CHECK(!held.analysisPending());
    CHECK(held.armSlice(300, 2, 180.0f) == CollectionCoordinator::Result::AlreadyComplete);
    CHECK(held.detectorCaptured(300, 2, 6) == CollectionCoordinator::Result::Duplicate);   // replay after a re-ARM
    CHECK(held.finalize(300) == CollectionCoordinator::Result::Accepted);

    // A lost SLICE_CAPTURED: the finished analysis proves the capture.
    CollectionCoordinator lostCapture;
    CHECK(lostCapture.start(400, {6}, 1) == CollectionCoordinator::Result::Accepted);
    CHECK(lostCapture.detectorReady(400, 6) == CollectionCoordinator::Result::Accepted);
    CHECK(lostCapture.armSlice(400, 1, 0.0f) == CollectionCoordinator::Result::Accepted);
    CHECK(lostCapture.awaitingCapture(400, 1, 6));
    CHECK(!lostCapture.awaitingCapture(400, 2, 6));
    CHECK(!lostCapture.awaitingCapture(400, 1, 7));
    CHECK(lostCapture.detectorAnalysed(400, 1, 6) == CollectionCoordinator::Result::Accepted);
    CHECK(lostCapture.state() == CollectionCoordinator::State::Ready);
    CHECK(!lostCapture.analysisPending());

    // Both copies of a CYCLE_COMPLETE lost: the next slice's completion retires it.
    CollectionCoordinator lostCompletion;
    CHECK(lostCompletion.start(600, {6}, 2) == CollectionCoordinator::Result::Accepted);
    CHECK(lostCompletion.detectorReady(600, 6) == CollectionCoordinator::Result::Accepted);
    CHECK(lostCompletion.armSlice(600, 1, 0.0f) == CollectionCoordinator::Result::Accepted);
    CHECK(lostCompletion.detectorCaptured(600, 1, 6) == CollectionCoordinator::Result::Accepted);
    CHECK(!lostCompletion.awaitingCapture(600, 1, 6));
    CHECK(lostCompletion.armSlice(600, 2, 180.0f) == CollectionCoordinator::Result::Accepted);
    CHECK(lostCompletion.detectorCaptured(600, 2, 6) == CollectionCoordinator::Result::Accepted);
    CHECK(lostCompletion.sliceAwaitingAnalysis());
    uint32_t retired = 99;
    CHECK(lostCompletion.detectorAnalysed(600, 2, 6, &retired) == CollectionCoordinator::Result::Accepted);
    CHECK(retired == 1);
    CHECK(!lostCompletion.analysisPending());
    CHECK(lostCompletion.state() == CollectionCoordinator::State::Ready);
    CHECK(lostCompletion.detectorAnalysed(600, 1, 6, &retired) == CollectionCoordinator::Result::Duplicate);
    CHECK(retired == 0);
    CHECK(lostCompletion.finalize(600) == CollectionCoordinator::Result::Accepted);

    // Cancel drops pending analyses with the collection.
    CollectionCoordinator cancelled;
    CHECK(cancelled.start(500, {6}, 3) == CollectionCoordinator::Result::Accepted);
    CHECK(cancelled.detectorReady(500, 6) == CollectionCoordinator::Result::Accepted);
    CHECK(cancelled.armSlice(500, 1, 0.0f) == CollectionCoordinator::Result::Accepted);
    CHECK(cancelled.detectorCaptured(500, 1, 6) == CollectionCoordinator::Result::Accepted);
    CHECK(cancelled.cancel(500) == CollectionCoordinator::Result::Accepted);
    CHECK(!cancelled.analysisPending());
    CHECK(!cancelled.isLiveSlice(500, 1));
    CHECK(cancelled.detectorAnalysed(500, 1, 6) == CollectionCoordinator::Result::Stale);

    // Only locked measurements reach the GCS during a Python collection;
    // acquisition, marginal and no-detection reports stay in the vehicle logs.
    CHECK(CollectionCoordinator::forwardPulseToGcs(TunnelProtocol::kConfirmedDetectionStatus));
    CHECK(!CollectionCoordinator::forwardPulseToGcs(TunnelProtocol::kSubthresholdDetectionStatus));
    CHECK(!CollectionCoordinator::forwardPulseToGcs(TunnelProtocol::kSuperthresholdDetectionStatus));
    CHECK(!CollectionCoordinator::forwardPulseToGcs(TunnelProtocol::kNoPulseDetectionStatus));

    return 0;
}
