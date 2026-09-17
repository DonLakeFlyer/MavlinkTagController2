// Unit tests for TagUploadCoordinator: the START_TAGS / TAG / END_TAGS state
// machine, isolated from CommandHandler's MAVLink framing. Sequences are the
// ones from the 2026-09-17 controller log where retransmits after lost ACKs
// produced a duplicate tag (15:17) and a spurious END_TAGS failure (15:13).

#include "TagUploadCoordinator.h"
#include "test_check.h"

#include <cmath>
#include <cstdio>

using TunnelProtocol::TagInfo_t;
using Result = TagUploadCoordinator::Result;
using State  = TagUploadCoordinator::State;

namespace {

TagInfo_t makeTag(uint32_t id)
{
    TagInfo_t tag {};
    tag.id                                      = id;
    tag.frequency_hz                            = 147970000;
    tag.pulse_width_msecs                       = 19;
    tag.intra_pulse1_msecs                      = 1500;
    tag.intra_pulse2_msecs                      = 2000;
    tag.intra_pulse_uncertainty_msecs           = 60;
    tag.intra_pulse_jitter_msecs                = 20;
    tag.k                                       = 20;
    tag.false_alarm_probability                 = 0.05;
    tag.channelizer_channel_number              = 1;
    tag.channelizer_channel_center_frequency_hz = 147970000;
    tag.ip1_mu    = std::nan("");
    tag.ip1_sigma = std::nan("");
    tag.ip2_mu    = std::nan("");
    tag.ip2_sigma = std::nan("");
    return tag;
}

// 15:13 upload: START, TAG 2, END (ACK lost), END retransmitted.
void testEndTagsRetransmitIsIdempotent()
{
    TagUploadCoordinator upload;
    CHECK(upload.state() == State::Idle);
    CHECK(upload.startTags(true) == Result::Accepted);
    CHECK(upload.state() == State::Receiving);
    CHECK(upload.addTag(makeTag(2)) == Result::Accepted);
    CHECK(upload.endTags() == Result::Accepted);
    CHECK(upload.state() == State::HasTags);
    CHECK(upload.endTags() == Result::Retransmit);   // was ACK FAILURE in the field
    CHECK(upload.state() == State::HasTags);
    CHECK(upload.tags().size() == 1);
}

// 15:17 upload: START, TAG 2 (ACK lost), TAG 2 retransmitted, END.
void testTagRetransmitIsNotAppended()
{
    TagUploadCoordinator upload;
    CHECK(upload.startTags(true) == Result::Accepted);
    CHECK(upload.addTag(makeTag(2)) == Result::Accepted);
    CHECK(upload.addTag(makeTag(2)) == Result::Retransmit);
    CHECK(upload.endTags() == Result::Accepted);
    CHECK(upload.state() == State::HasTags);
    CHECK(upload.tags().size() == 1);   // one detector, not two on the same port
}

void testStartTagsRequiresIdleController()
{
    TagUploadCoordinator upload;
    CHECK(upload.startTags(true) == Result::Accepted);
    CHECK(upload.addTag(makeTag(2)) == Result::Accepted);
    CHECK(upload.endTags() == Result::Accepted);

    // Controller busy (e.g. detecting): list must survive untouched.
    CHECK(upload.startTags(false) == Result::WrongState);
    CHECK(upload.state() == State::HasTags);
    CHECK(upload.tags().size() == 1);
}

void testStartTagsClearsPreviousUpload()
{
    TagUploadCoordinator upload;
    CHECK(upload.startTags(true) == Result::Accepted);
    CHECK(upload.addTag(makeTag(2)) == Result::Accepted);
    CHECK(upload.addTag(makeTag(4)) == Result::Accepted);
    CHECK(upload.endTags() == Result::Accepted);
    CHECK(upload.tags().size() == 2);

    CHECK(upload.startTags(true) == Result::Accepted);
    CHECK(upload.state() == State::Receiving);
    CHECK(upload.tags().empty());
    CHECK(upload.addTag(makeTag(2)) == Result::Accepted);   // fresh add, not Retransmit
    CHECK(upload.endTags() == Result::Accepted);
    CHECK(upload.tags().size() == 1);
}

// START_TAGS retransmit while Receiving just restarts the (empty) bracket.
void testStartTagsRetransmitWhileReceiving()
{
    TagUploadCoordinator upload;
    CHECK(upload.startTags(true) == Result::Accepted);
    CHECK(upload.startTags(true) == Result::Accepted);
    CHECK(upload.state() == State::Receiving);
    CHECK(upload.tags().empty());
}

void testTagOutsideBracketIsRejected()
{
    TagUploadCoordinator upload;
    CHECK(upload.addTag(makeTag(2)) == Result::NotReceiving);   // Idle
    CHECK(upload.tags().empty());

    CHECK(upload.startTags(true) == Result::Accepted);
    CHECK(upload.addTag(makeTag(2)) == Result::Accepted);
    CHECK(upload.endTags() == Result::Accepted);
    CHECK(upload.addTag(makeTag(4)) == Result::NotReceiving);   // HasTags
    CHECK(upload.tags().size() == 1);
}

void testEndTagsWithoutStartIsRejected()
{
    TagUploadCoordinator upload;
    CHECK(upload.endTags() == Result::NotReceiving);
    CHECK(upload.state() == State::Idle);
}

// Empty upload: START, END (ACK lost), END retransmitted. Clearing the list is a
// legitimate GCS operation and must be as retransmit-safe as a non-empty one.
void testEmptyUploadIsRetransmitSafe()
{
    TagUploadCoordinator upload;
    CHECK(upload.startTags(true) == Result::Accepted);
    CHECK(upload.endTags() == Result::Accepted);
    CHECK(upload.state() == State::Empty);
    CHECK(!upload.hasTags());
    CHECK(upload.endTags() == Result::Retransmit);
    CHECK(upload.state() == State::Empty);
    CHECK(!upload.hasTags());
    CHECK(upload.addTag(makeTag(2)) == Result::NotReceiving);   // still outside a bracket
    CHECK(upload.tags().empty());

    CHECK(upload.startTags(true) == Result::Accepted);          // next upload proceeds normally
    CHECK(upload.state() == State::Receiving);
}

void testTagValidation()
{
    TagUploadCoordinator upload;
    CHECK(upload.startTags(true) == Result::Accepted);

    TagInfo_t id0 = makeTag(0);
    TagInfo_t id1 = makeTag(1);
    CHECK(upload.addTag(id0) == Result::InvalidId);
    CHECK(upload.addTag(id1) == Result::InvalidId);

    TagInfo_t badK = makeTag(2);
    badK.k = 1;
    CHECK(upload.addTag(badK) == Result::InvalidK);

    CHECK(upload.tags().empty());
    CHECK(upload.state() == State::Receiving);   // rejects don't abort the bracket
}

void testConflictingRedefinitionIsRejected()
{
    TagUploadCoordinator upload;
    CHECK(upload.startTags(true) == Result::Accepted);
    CHECK(upload.addTag(makeTag(2)) == Result::Accepted);

    TagInfo_t changed = makeTag(2);
    changed.frequency_hz = 147980000;
    CHECK(upload.addTag(changed) == Result::Conflict);
    CHECK(upload.tags().size() == 1);
    CHECK(upload.tags()[0].frequency_hz == 147970000);
}

} // namespace

int main()
{
    testEndTagsRetransmitIsIdempotent();
    testTagRetransmitIsNotAppended();
    testStartTagsRequiresIdleController();
    testStartTagsClearsPreviousUpload();
    testStartTagsRetransmitWhileReceiving();
    testTagOutsideBracketIsRejected();
    testEndTagsWithoutStartIsRejected();
    testEmptyUploadIsRetransmitSafe();
    testTagValidation();
    testConflictingRedefinitionIsRejected();
    std::printf("test_tag_upload_coordinator: all tests passed\n");
    return 0;
}
