#pragma once

#include <cstdint>
#include <optional>
#include <set>
#include <vector>

class CollectionCoordinator {
public:
    enum class State {
        Inactive,
        Starting,
        Ready,
        CollectingSlice,
    };

    enum class Result {
        Accepted,
        Duplicate,
        Conflict,
        Stale,
        Busy,
        OutOfOrder,
        /// armSlice: this slice already completed; caller should resend SLICE_COMPLETE.
        AlreadyComplete,
    };

    Result start(uint32_t collectionId, std::vector<uint32_t> detectorTagIds);
    Result detectorReady(uint32_t collectionId, uint32_t tagId);
    Result armSlice(uint32_t collectionId, uint32_t sliceId, float headingDeg);
    Result detectorArmed(uint32_t collectionId, uint32_t sliceId, uint32_t tagId);
    Result completeDetector(uint32_t collectionId, uint32_t sliceId, uint32_t tagId);
    Result finalize(uint32_t collectionId);
    Result cancel(uint32_t collectionId);

    /// How the last collection ended; distinguishes a FINALIZE retry from a
    /// FINALIZE that follows a CANCEL of the same id (which must not be Duplicate).
    enum class Disposition { None, Finalized, Cancelled };

    State state() const { return _state; }
    uint32_t collectionId() const { return _collectionId; }
    uint32_t sliceId() const { return _activeSliceId.value_or(0); }
    float headingDeg() const { return _activeHeadingDeg; }
    size_t expectedDetectorCount() const { return _expectedTagIds.size(); }
    bool sliceArmed() const { return !_expectedTagIds.empty() && _armedTagIds.size() == _expectedTagIds.size(); }
    size_t completedDetectorCount() const { return _completedTagIds.size(); }

private:
    bool _matchesCollection(uint32_t collectionId) const;
    void _reset(Disposition disposition);

    State _state { State::Inactive };
    uint32_t _collectionId { 0 };
    uint32_t _lastFinishedCollectionId { 0 };
    Disposition _lastDisposition { Disposition::None };
    uint32_t _nextSliceId { 1 };
    std::optional<uint32_t> _activeSliceId;
    std::optional<uint32_t> _lastCompletedSliceId;
    float _activeHeadingDeg { 0.0f };
    std::set<uint32_t> _expectedTagIds;
    std::set<uint32_t> _readyTagIds;
    std::set<uint32_t> _armedTagIds;
    std::set<uint32_t> _completedTagIds;
};
