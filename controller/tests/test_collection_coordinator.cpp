#include "CollectionCoordinator.h"
#include "test_check.h"

#include <cstdint>
#include <vector>

int main()
{
    CollectionCoordinator coordinator;

    CHECK(coordinator.state() == CollectionCoordinator::State::Inactive);
    CHECK(coordinator.start(100, {2, 4}) == CollectionCoordinator::Result::Accepted);
    CHECK(coordinator.state() == CollectionCoordinator::State::Starting);
    CHECK(coordinator.start(100, {2, 4}) == CollectionCoordinator::Result::Duplicate);
    CHECK(coordinator.start(101, {2, 4}) == CollectionCoordinator::Result::Conflict);

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
    CHECK(coordinator.completeDetector(100, 1, 2) == CollectionCoordinator::Result::Accepted);
    CHECK(coordinator.completeDetector(100, 1, 2) == CollectionCoordinator::Result::Duplicate);
    CHECK(coordinator.completeDetector(100, 0, 4) == CollectionCoordinator::Result::Stale);
    CHECK(coordinator.completeDetector(100, 1, 4) == CollectionCoordinator::Result::Accepted);
    CHECK(coordinator.state() == CollectionCoordinator::State::Ready);

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
    CHECK(finalizable.start(200, {6}) == CollectionCoordinator::Result::Accepted);
    CHECK(finalizable.detectorReady(200, 6) == CollectionCoordinator::Result::Accepted);
    CHECK(finalizable.finalize(200) == CollectionCoordinator::Result::Accepted);
    CHECK(finalizable.state() == CollectionCoordinator::State::Inactive);
    // Retried FINISH after a lost ACK must be idempotent, as cancel() is.
    CHECK(finalizable.finalize(200) == CollectionCoordinator::Result::Duplicate);
    CHECK(finalizable.finalize(201) == CollectionCoordinator::Result::Stale);
    // ...but the opposite disposition is not a duplicate either.
    CHECK(finalizable.cancel(200) == CollectionCoordinator::Result::Stale);

    return 0;
}
