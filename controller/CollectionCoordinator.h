#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <vector>

#include "TunnelProtocol.h"

// A slice is complete for the GCS once every detector has captured its segment;
// the detectors analyse it in the background while the next heading is flown.
// The last announced slice (and a revisit) is held until every analysis has
// finished, so FINISH_COLLECTION always sees the full set of results.
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

    static const char* resultName(Result result)
    {
        switch (result) {
        case Result::Accepted:        return "Accepted";
        case Result::Duplicate:       return "Duplicate";
        case Result::Conflict:        return "Conflict";
        case Result::Stale:           return "Stale";
        case Result::Busy:            return "Busy";
        case Result::OutOfOrder:      return "OutOfOrder";
        case Result::AlreadyComplete: return "AlreadyComplete";
        }
        return "?";
    }

    /// sliceCount: slices announced by START_COLLECTION; slice ids at or past it are held for analysis.
    Result start(uint32_t collectionId, std::vector<uint32_t> detectorTagIds, uint32_t sliceCount);
    Result detectorReady(uint32_t collectionId, uint32_t tagId);
    Result armSlice(uint32_t collectionId, uint32_t sliceId, float headingDeg);
    Result detectorArmed(uint32_t collectionId, uint32_t sliceId, uint32_t tagId);
    /// SLICE_CAPTURED. The slice completes (state Ready) once every detector captured it, unless it is held.
    Result detectorCaptured(uint32_t collectionId, uint32_t sliceId, uint32_t tagId);
    /// CYCLE_COMPLETE: the detector finished analysing the slice. Implies capture, and the analysis of
    /// every earlier slice by that detector (counted in *retiredEarlier). May complete a held slice.
    Result detectorAnalysed(uint32_t collectionId, uint32_t sliceId, uint32_t tagId, uint32_t* retiredEarlier = nullptr);
    Result finalize(uint32_t collectionId);
    Result cancel(uint32_t collectionId);

    /// Python collections send only locked (confirmed) measurements to the GCS;
    /// acquisition/marginal/no-detection reports stay in vehicle logs.
    static bool forwardPulseToGcs(uint8_t detectionStatus) { return detectionStatus == TunnelProtocol::kConfirmedDetectionStatus; }

    /// How the last collection ended; distinguishes a FINALIZE retry from a
    /// FINALIZE that follows a CANCEL of the same id (which must not be Duplicate).
    enum class Disposition { None, Finalized, Cancelled };

    State state() const { return _state; }
    uint32_t collectionId() const { return _collectionId; }
    uint32_t sliceId() const { return _activeSliceId.value_or(0); }
    float headingDeg() const { return _activeHeadingDeg; }
    size_t expectedDetectorCount() const { return _expectedTagIds.size(); }
    bool isExpectedDetector(uint32_t tagId) const { return _expectedTagIds.contains(tagId); }
    size_t readyDetectorCount() const { return _readyTagIds.size(); }
    size_t armedDetectorCount() const { return _armedTagIds.size(); }
    bool sliceArmed() const { return !_expectedTagIds.empty() && _armedTagIds.size() == _expectedTagIds.size(); }
    size_t completedDetectorCount() const { return _capturedTagIds.size(); }
    /// The active slice is captured by every detector but held until all analyses finish.
    bool sliceAwaitingAnalysis() const { return _state == State::CollectingSlice && _allCaptured(); }
    bool analysisPending() const { return !_pendingAnalysis.empty(); }
    /// The slice is active or still being analysed, so detector reports for it are current.
    bool isLiveSlice(uint32_t collectionId, uint32_t sliceId) const;
    /// The slice is being collected and this detector has not reported capturing it.
    bool awaitingCapture(uint32_t collectionId, uint32_t sliceId, uint32_t tagId) const
    {
        return _matchesCollection(collectionId) && _state == State::CollectingSlice && _activeSliceId == sliceId
            && _expectedTagIds.contains(tagId) && !_capturedTagIds.contains(tagId);
    }

private:
    bool _matchesCollection(uint32_t collectionId) const;
    bool _allCaptured() const { return !_expectedTagIds.empty() && _capturedTagIds.size() == _expectedTagIds.size(); }
    bool _holdsForAnalysis(uint32_t sliceId) const { return sliceId >= _sliceCount; }
    void _maybeCompleteActiveSlice();
    void _reset(Disposition disposition);

    State _state { State::Inactive };
    uint32_t _collectionId { 0 };
    uint32_t _sliceCount { 0 };
    uint32_t _lastFinishedCollectionId { 0 };
    Disposition _lastDisposition { Disposition::None };
    uint32_t _nextSliceId { 1 };
    std::optional<uint32_t> _activeSliceId;
    std::optional<uint32_t> _lastCompletedSliceId;
    float _activeHeadingDeg { 0.0f };
    std::set<uint32_t> _expectedTagIds;
    std::set<uint32_t> _readyTagIds;
    std::set<uint32_t> _armedTagIds;
    std::set<uint32_t> _capturedTagIds;
    std::map<uint32_t, std::set<uint32_t>> _pendingAnalysis;   // slice id -> detectors still analysing it
    std::set<uint32_t> _analysedSliceIds;                       // fully analysed, for duplicate CYCLE_COMPLETE
};
