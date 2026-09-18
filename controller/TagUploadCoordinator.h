#pragma once

#include "TagDatabase.h"
#include "TunnelProtocol.h"

#include <string>
#include <vector>

// START_TAGS / TAG / END_TAGS upload state machine. Takes decoded payloads, not
// tunnel frames, and does no logging so it links into a standalone test;
// CommandHandler owns MAVLink framing, logging and ACKs. Every command is
// idempotent under retransmission after a lost ACK, and the set is checked
// for completeness (upload_id / tag_count / tag_index) before END_TAGS is
// accepted.
class TagUploadCoordinator {
public:
    enum class State {
        Idle,       // nothing uploaded yet
        Receiving,  // inside a START_TAGS/END_TAGS bracket
        HasTags,    // upload complete, list usable
        Empty,      // upload complete with no tags; distinct from Idle so END_TAGS can be retransmitted
    };

    enum class Result {
        Accepted,
        Retransmit,     // duplicate of a command already applied; ACK success
        WrongState,     // startTags: controller not idle
        InvalidCount,   // startTags: tag_count > kMaxTagCount; previous list untouched
        NotReceiving,   // TAG or END_TAGS outside a bracket
        InvalidId,      // tag id 0/1 are reserved
        InvalidK,       // k < 2
        Conflict,       // id already uploaded with different parameters
        StaleUpload,    // TAG/END_TAGS upload_id is not the open bracket's
        InvalidIndex,   // tag_index >= tag_count, or index already used by another id
        Incomplete,     // endTags: some tag_index never arrived; see missingIndices()
        CountMismatch,  // endTags: tag_count differs from START_TAGS
    };

    // tag_count comes off the wire; bound it before it sizes anything.
    static constexpr uint32_t kMaxTagCount = 5;

    Result startTags(bool controllerIdle, const TunnelProtocol::StartTagsInfo_t& info);
    Result addTag(const TunnelProtocol::TagInfo_t& tagInfo);
    Result endTags(const TunnelProtocol::EndTagsInfo_t& info);

    State              state()    const { return _state; }
    bool               hasTags()  const { return _state == State::HasTags; }
    const TagDatabase& tags()     const { return _tags; }
    uint32_t           uploadId() const { return _uploadId; }

    /// Indices in 0..tag_count-1 not yet received (meaningful while Receiving).
    std::vector<uint32_t> missingIndices() const;
    /// "2,5" form of missingIndices(), for the NACK message.
    static std::string formatIndices(const std::vector<uint32_t>& indices);

private:
    State                 _state { State::Idle };
    TagDatabase           _tags;
    uint32_t              _uploadId { 0 };
    uint32_t              _expectedCount { 0 };
    std::vector<uint32_t> _indexToId;      // tag_index -> tag id; 0 = not received
};
