#include "CollectionCoordinator.h"

#include <algorithm>

CollectionCoordinator::Result CollectionCoordinator::start(
    uint32_t collectionId, std::vector<uint32_t> detectorTagIds)
{
    const std::set<uint32_t> requestedTagIds(
        detectorTagIds.begin(), detectorTagIds.end());

    if (_state != State::Inactive) {
        if (_collectionId == collectionId && _expectedTagIds == requestedTagIds) {
            return Result::Duplicate;
        }
        return Result::Conflict;
    }
    if (collectionId == 0 || requestedTagIds.empty()) {
        return Result::Conflict;
    }

    _collectionId = collectionId;
    _nextSliceId = 1;
    _expectedTagIds = requestedTagIds;
    _readyTagIds.clear();
    _armedTagIds.clear();
    _completedTagIds.clear();
    _activeSliceId.reset();
    _lastCompletedSliceId.reset();
    _state = State::Starting;
    return Result::Accepted;
}

CollectionCoordinator::Result CollectionCoordinator::detectorReady(
    uint32_t collectionId, uint32_t tagId)
{
    if (!_matchesCollection(collectionId)) {
        return Result::Stale;
    }
    if (_state != State::Starting) {
        return _readyTagIds.contains(tagId) ? Result::Duplicate : Result::Conflict;
    }
    if (!_expectedTagIds.contains(tagId)) {
        return Result::Conflict;
    }
    if (!_readyTagIds.insert(tagId).second) {
        return Result::Duplicate;
    }
    if (_readyTagIds.size() == _expectedTagIds.size()) {
        _state = State::Ready;
    }
    return Result::Accepted;
}

CollectionCoordinator::Result CollectionCoordinator::armSlice(
    uint32_t collectionId, uint32_t sliceId, float headingDeg)
{
    if (!_matchesCollection(collectionId)) {
        return Result::Stale;
    }
    if (_state == State::CollectingSlice) {
        if (_activeSliceId == sliceId && _activeHeadingDeg == headingDeg) {
            return Result::Duplicate;
        }
        return Result::Busy;
    }
    if (_state != State::Ready) {
        return Result::Busy;
    }
    // The SLICE_COMPLETE status is a single datagram; a GCS that missed it
    // retries the arm, and must get a replay rather than OutOfOrder.
    if (_lastCompletedSliceId == sliceId) {
        return Result::AlreadyComplete;
    }
    if (sliceId != _nextSliceId) {
        return Result::OutOfOrder;
    }

    _activeSliceId = sliceId;
    _activeHeadingDeg = headingDeg;
    _armedTagIds.clear();
    _completedTagIds.clear();
    _state = State::CollectingSlice;
    return Result::Accepted;
}

CollectionCoordinator::Result CollectionCoordinator::detectorArmed(
    uint32_t collectionId, uint32_t sliceId, uint32_t tagId)
{
    if (!_matchesCollection(collectionId) || _activeSliceId != sliceId) {
        return Result::Stale;
    }
    if (_state != State::CollectingSlice || !_expectedTagIds.contains(tagId)) {
        return Result::Conflict;
    }
    return _armedTagIds.insert(tagId).second ? Result::Accepted : Result::Duplicate;
}

CollectionCoordinator::Result CollectionCoordinator::completeDetector(
    uint32_t collectionId, uint32_t sliceId, uint32_t tagId)
{
    if (!_matchesCollection(collectionId) || _activeSliceId != sliceId) {
        return Result::Stale;
    }
    if (_state != State::CollectingSlice || !_expectedTagIds.contains(tagId)) {
        return Result::Conflict;
    }
    if (!_completedTagIds.insert(tagId).second) {
        return Result::Duplicate;
    }
    if (_completedTagIds.size() == _expectedTagIds.size()) {
        _state = State::Ready;
        _lastCompletedSliceId = _activeSliceId;
        _activeSliceId.reset();
        ++_nextSliceId;
    }
    return Result::Accepted;
}

CollectionCoordinator::Result CollectionCoordinator::finalize(uint32_t collectionId)
{
    // Mirrors cancel(): a retried FINISH after a lost ACK must be idempotent,
    // but FINALIZE after CANCEL of the same id is a different request.
    if (_state == State::Inactive) {
        return (collectionId == _lastFinishedCollectionId
                && _lastDisposition == Disposition::Finalized)
            ? Result::Duplicate : Result::Stale;
    }
    if (!_matchesCollection(collectionId)) {
        return Result::Stale;
    }
    if (_state != State::Ready) {
        return Result::Busy;
    }
    _reset(Disposition::Finalized);
    return Result::Accepted;
}

CollectionCoordinator::Result CollectionCoordinator::cancel(uint32_t collectionId)
{
    if (_state == State::Inactive) {
        return (collectionId == _lastFinishedCollectionId
                && _lastDisposition == Disposition::Cancelled)
            ? Result::Duplicate : Result::Stale;
    }
    if (!_matchesCollection(collectionId)) {
        return Result::Stale;
    }
    _reset(Disposition::Cancelled);
    return Result::Accepted;
}

bool CollectionCoordinator::_matchesCollection(uint32_t collectionId) const
{
    return _state != State::Inactive && _collectionId == collectionId;
}

void CollectionCoordinator::_reset(Disposition disposition)
{
    _lastFinishedCollectionId = _collectionId;
    _lastDisposition = disposition;
    _collectionId = 0;
    _nextSliceId = 1;
    _activeSliceId.reset();
    _lastCompletedSliceId.reset();
    _expectedTagIds.clear();
    _readyTagIds.clear();
    _armedTagIds.clear();
    _completedTagIds.clear();
    _state = State::Inactive;
}
