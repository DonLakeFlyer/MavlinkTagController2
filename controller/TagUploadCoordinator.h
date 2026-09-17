#pragma once

#include "TagDatabase.h"
#include "TunnelProtocol.h"

// START_TAGS / TAG / END_TAGS upload state machine. Takes decoded payloads, not
// tunnel frames, and does no logging so it links into a standalone test;
// CommandHandler owns MAVLink framing, logging and ACKs. Every command is
// idempotent under retransmission after a lost ACK.
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
        NotReceiving,   // TAG or END_TAGS outside a bracket
        InvalidId,      // tag id 0/1 are reserved
        InvalidK,       // k < 2
        Conflict,       // id already uploaded with different parameters
    };

    Result startTags(bool controllerIdle);
    Result addTag(const TunnelProtocol::TagInfo_t& tagInfo);
    Result endTags();

    State              state()   const { return _state; }
    bool               hasTags() const { return _state == State::HasTags; }
    const TagDatabase& tags()    const { return _tags; }

private:
    State       _state { State::Idle };
    TagDatabase _tags;
};
