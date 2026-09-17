#include "TagUploadCoordinator.h"

TagUploadCoordinator::Result TagUploadCoordinator::startTags(bool controllerIdle)
{
    if (!controllerIdle) {
        return Result::WrongState;
    }
    _tags.clear();
    _state = State::Receiving;
    return Result::Accepted;
}

TagUploadCoordinator::Result TagUploadCoordinator::addTag(const TunnelProtocol::TagInfo_t& tagInfo)
{
    if (_state != State::Receiving) {
        return Result::NotReceiving;
    }
    if (tagInfo.id < 2) {
        return Result::InvalidId;
    }
    if (tagInfo.k < 2) {
        return Result::InvalidK;
    }
    switch (_tags.addTag(tagInfo)) {
    case TagDatabase::AddResult::Added:      return Result::Accepted;
    case TagDatabase::AddResult::Retransmit: return Result::Retransmit;
    case TagDatabase::AddResult::Conflict:   return Result::Conflict;
    }
    return Result::Conflict;
}

TagUploadCoordinator::Result TagUploadCoordinator::endTags()
{
    switch (_state) {
    case State::Receiving:
        _state = _tags.empty() ? State::Empty : State::HasTags;
        return Result::Accepted;
    case State::HasTags:
    case State::Empty:
        return Result::Retransmit;
    case State::Idle:
        return Result::NotReceiving;
    }
    return Result::NotReceiving;
}
