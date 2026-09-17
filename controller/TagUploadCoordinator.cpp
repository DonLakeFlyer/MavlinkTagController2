#include "TagUploadCoordinator.h"

#include <cstdio>

using namespace TunnelProtocol;

TagUploadCoordinator::Result TagUploadCoordinator::startTags(bool controllerIdle, const StartTagsInfo_t& info)
{
    if (!controllerIdle) {
        return Result::WrongState;
    }
    _tags.clear();
    _uploadId      = info.upload_id;
    _expectedCount = info.tag_count;
    _indexToId.assign(_expectedCount, 0);
    _state = State::Receiving;
    return Result::Accepted;
}

TagUploadCoordinator::Result TagUploadCoordinator::addTag(const TagInfo_t& tagInfo)
{
    if (_state != State::Receiving) {
        return Result::NotReceiving;
    }
    if (tagInfo.upload_id != _uploadId) {
        return Result::StaleUpload;
    }
    if (tagInfo.tag_index >= _expectedCount) {
        return Result::InvalidIndex;
    }
    if (tagInfo.id < 2) {
        return Result::InvalidId;
    }
    if (tagInfo.k < 2) {
        return Result::InvalidK;
    }
    const uint32_t slotId = _indexToId[tagInfo.tag_index];
    if (slotId != 0 && slotId != tagInfo.id) {
        return Result::InvalidIndex;
    }
    switch (_tags.addTag(tagInfo)) {
    case TagDatabase::AddResult::Added:
        _indexToId[tagInfo.tag_index] = tagInfo.id;
        return Result::Accepted;
    case TagDatabase::AddResult::Retransmit:
        // Same id at a different index would be two slots for one tag.
        return slotId == tagInfo.id ? Result::Retransmit : Result::InvalidIndex;
    case TagDatabase::AddResult::Conflict:
        return Result::Conflict;
    }
    return Result::Conflict;
}

TagUploadCoordinator::Result TagUploadCoordinator::endTags(const EndTagsInfo_t& info)
{
    switch (_state) {
    case State::Receiving:
        if (info.upload_id != _uploadId) {
            return Result::StaleUpload;
        }
        if (info.tag_count != _expectedCount) {
            return Result::CountMismatch;
        }
        if (!missingIndices().empty()) {
            return Result::Incomplete;
        }
        _state = _tags.empty() ? State::Empty : State::HasTags;
        return Result::Accepted;
    case State::HasTags:
    case State::Empty:
        return info.upload_id == _uploadId ? Result::Retransmit : Result::StaleUpload;
    case State::Idle:
        return Result::NotReceiving;
    }
    return Result::NotReceiving;
}

std::vector<uint32_t> TagUploadCoordinator::missingIndices() const
{
    std::vector<uint32_t> missing;
    for (uint32_t i = 0; i < _indexToId.size(); ++i) {
        if (_indexToId[i] == 0) {
            missing.push_back(i);
        }
    }
    return missing;
}

std::string TagUploadCoordinator::formatIndices(const std::vector<uint32_t>& indices)
{
    std::string out;
    for (const uint32_t i : indices) {
        if (!out.empty()) {
            out += ',';
        }
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%u", i);
        out += buf;
    }
    return out;
}

