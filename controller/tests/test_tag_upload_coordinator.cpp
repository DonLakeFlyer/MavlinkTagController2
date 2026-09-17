// Unit tests for TagUploadCoordinator: the START_TAGS / TAG / END_TAGS state
// machine, isolated from CommandHandler's MAVLink framing. Sequences are the
// ones from the 2026-09-17 controller log where retransmits after lost ACKs
// produced a duplicate tag (15:17) and a spurious END_TAGS failure (15:13),
// plus the upload-set checks (upload_id / tag_count / tag_index) added in
// protocol v4.

#include "TagUploadCoordinator.h"
#include "test_check.h"

#include <cmath>
#include <cstdio>

using TunnelProtocol::EndTagsInfo_t;
using TunnelProtocol::StartTagsInfo_t;
using TunnelProtocol::TagInfo_t;
using Result = TagUploadCoordinator::Result;
using State  = TagUploadCoordinator::State;

namespace {

constexpr uint32_t kUpload = 0xA1;

StartTagsInfo_t start(uint32_t count, uint32_t uploadId = kUpload)
{
    StartTagsInfo_t s {};
    s.upload_id = uploadId;
    s.tag_count = count;
    return s;
}

EndTagsInfo_t end(uint32_t count, uint32_t uploadId = kUpload)
{
    EndTagsInfo_t e {};
    e.upload_id = uploadId;
    e.tag_count = count;
    return e;
}

TagInfo_t makeTag(uint32_t id, uint32_t index = 0, uint32_t uploadId = kUpload)
{
    TagInfo_t tag {};
    tag.upload_id                               = uploadId;
    tag.tag_index                               = index;
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
    CHECK(upload.startTags(true, start(1)) == Result::Accepted);
    CHECK(upload.state() == State::Receiving);
    CHECK(upload.addTag(makeTag(2)) == Result::Accepted);
    CHECK(upload.endTags(end(1)) == Result::Accepted);
    CHECK(upload.state() == State::HasTags);
    CHECK(upload.endTags(end(1)) == Result::Retransmit);   // was ACK FAILURE in the field
    CHECK(upload.state() == State::HasTags);
    CHECK(upload.tags().size() == 1);
}

// 15:17 upload: START, TAG 2 (ACK lost), TAG 2 retransmitted, END.
void testTagRetransmitIsNotAppended()
{
    TagUploadCoordinator upload;
    CHECK(upload.startTags(true, start(1)) == Result::Accepted);
    CHECK(upload.addTag(makeTag(2)) == Result::Accepted);
    CHECK(upload.addTag(makeTag(2)) == Result::Retransmit);
    CHECK(upload.endTags(end(1)) == Result::Accepted);
    CHECK(upload.state() == State::HasTags);
    CHECK(upload.tags().size() == 1);   // one detector, not two on the same port
}

void testStartTagsRequiresIdleController()
{
    TagUploadCoordinator upload;
    CHECK(upload.startTags(true, start(1)) == Result::Accepted);
    CHECK(upload.addTag(makeTag(2)) == Result::Accepted);
    CHECK(upload.endTags(end(1)) == Result::Accepted);

    // Controller busy (e.g. detecting): list must survive untouched.
    CHECK(upload.startTags(false, start(1, kUpload + 1)) == Result::WrongState);
    CHECK(upload.state() == State::HasTags);
    CHECK(upload.tags().size() == 1);
}

void testStartTagsClearsPreviousUpload()
{
    TagUploadCoordinator upload;
    CHECK(upload.startTags(true, start(2)) == Result::Accepted);
    CHECK(upload.addTag(makeTag(2, 0)) == Result::Accepted);
    CHECK(upload.addTag(makeTag(4, 1)) == Result::Accepted);
    CHECK(upload.endTags(end(2)) == Result::Accepted);
    CHECK(upload.tags().size() == 2);

    CHECK(upload.startTags(true, start(1, kUpload + 1)) == Result::Accepted);
    CHECK(upload.state() == State::Receiving);
    CHECK(upload.tags().empty());
    CHECK(upload.addTag(makeTag(2, 0, kUpload + 1)) == Result::Accepted);   // fresh add, not Retransmit
    CHECK(upload.endTags(end(1, kUpload + 1)) == Result::Accepted);
    CHECK(upload.tags().size() == 1);
}

// START_TAGS retransmit while Receiving just restarts the (empty) bracket.
void testStartTagsRetransmitWhileReceiving()
{
    TagUploadCoordinator upload;
    CHECK(upload.startTags(true, start(1)) == Result::Accepted);
    CHECK(upload.startTags(true, start(1)) == Result::Accepted);
    CHECK(upload.state() == State::Receiving);
    CHECK(upload.tags().empty());
}

void testTagOutsideBracketIsRejected()
{
    TagUploadCoordinator upload;
    CHECK(upload.addTag(makeTag(2)) == Result::NotReceiving);   // Idle
    CHECK(upload.tags().empty());

    CHECK(upload.startTags(true, start(1)) == Result::Accepted);
    CHECK(upload.addTag(makeTag(2)) == Result::Accepted);
    CHECK(upload.endTags(end(1)) == Result::Accepted);
    CHECK(upload.addTag(makeTag(4)) == Result::NotReceiving);   // HasTags
    CHECK(upload.tags().size() == 1);
}

void testEndTagsWithoutStartIsRejected()
{
    TagUploadCoordinator upload;
    CHECK(upload.endTags(end(0)) == Result::NotReceiving);
    CHECK(upload.state() == State::Idle);
}

// Empty upload: START, END (ACK lost), END retransmitted. Clearing the list is a
// legitimate GCS operation and must be as retransmit-safe as a non-empty one.
void testEmptyUploadIsRetransmitSafe()
{
    TagUploadCoordinator upload;
    CHECK(upload.startTags(true, start(0)) == Result::Accepted);
    CHECK(upload.endTags(end(0)) == Result::Accepted);
    CHECK(upload.state() == State::Empty);
    CHECK(!upload.hasTags());
    CHECK(upload.endTags(end(0)) == Result::Retransmit);
    CHECK(upload.state() == State::Empty);
    CHECK(!upload.hasTags());
    CHECK(upload.addTag(makeTag(2)) == Result::NotReceiving);   // still outside a bracket
    CHECK(upload.tags().empty());

    CHECK(upload.startTags(true, start(1, kUpload + 1)) == Result::Accepted);   // next upload proceeds normally
    CHECK(upload.state() == State::Receiving);
}

void testTagValidation()
{
    TagUploadCoordinator upload;
    CHECK(upload.startTags(true, start(3)) == Result::Accepted);

    TagInfo_t id0 = makeTag(0, 0);
    TagInfo_t id1 = makeTag(1, 1);
    CHECK(upload.addTag(id0) == Result::InvalidId);
    CHECK(upload.addTag(id1) == Result::InvalidId);

    TagInfo_t badK = makeTag(2, 2);
    badK.k = 1;
    CHECK(upload.addTag(badK) == Result::InvalidK);

    CHECK(upload.tags().empty());
    CHECK(upload.state() == State::Receiving);   // rejects don't abort the bracket
}

void testConflictingRedefinitionIsRejected()
{
    TagUploadCoordinator upload;
    CHECK(upload.startTags(true, start(1)) == Result::Accepted);
    CHECK(upload.addTag(makeTag(2)) == Result::Accepted);

    TagInfo_t changed = makeTag(2);
    changed.frequency_hz = 147980000;
    CHECK(upload.addTag(changed) == Result::Conflict);
    CHECK(upload.tags().size() == 1);
    CHECK(upload.tags()[0].frequency_hz == 147970000);
}

// A TAG lost in flight (not just its ACK): END_TAGS must not close the set.
void testIncompleteUploadIsRejectedUntilFilled()
{
    TagUploadCoordinator upload;
    CHECK(upload.startTags(true, start(3)) == Result::Accepted);
    CHECK(upload.addTag(makeTag(2, 0)) == Result::Accepted);
    CHECK(upload.addTag(makeTag(6, 2)) == Result::Accepted);          // index 1 never arrived
    CHECK(upload.endTags(end(3)) == Result::Incomplete);
    CHECK(upload.state() == State::Receiving);
    const auto missing = upload.missingIndices();
    CHECK(missing.size() == 1 && missing[0] == 1);
    CHECK(TagUploadCoordinator::formatIndices(missing) == "1");

    CHECK(upload.addTag(makeTag(4, 1)) == Result::Accepted);          // GCS fills the gap
    CHECK(upload.endTags(end(3)) == Result::Accepted);
    CHECK(upload.tags().size() == 3);
    CHECK(TagUploadCoordinator::formatIndices({0, 2, 5}) == "0,2,5");
}

// GCS restarted mid-upload and began a new set; leftovers from the old one
// must be refused rather than folded into the new bracket.
void testStaleUploadIdIsRejected()
{
    TagUploadCoordinator upload;
    CHECK(upload.startTags(true, start(1, 7)) == Result::Accepted);
    CHECK(upload.addTag(makeTag(2, 0, 9)) == Result::StaleUpload);
    CHECK(upload.tags().empty());
    CHECK(upload.endTags(end(1, 9)) == Result::StaleUpload);
    CHECK(upload.state() == State::Receiving);

    CHECK(upload.addTag(makeTag(2, 0, 7)) == Result::Accepted);
    CHECK(upload.endTags(end(1, 7)) == Result::Accepted);
    // A late END_TAGS from the abandoned set is not a retransmit of this one.
    CHECK(upload.endTags(end(1, 9)) == Result::StaleUpload);
    CHECK(upload.endTags(end(1, 7)) == Result::Retransmit);
}

void testIndexValidation()
{
    TagUploadCoordinator upload;
    CHECK(upload.startTags(true, start(2)) == Result::Accepted);
    CHECK(upload.addTag(makeTag(2, 2)) == Result::InvalidIndex);     // >= tag_count
    CHECK(upload.addTag(makeTag(2, 0)) == Result::Accepted);
    CHECK(upload.addTag(makeTag(4, 0)) == Result::InvalidIndex);     // slot 0 already holds id 2
    CHECK(upload.addTag(makeTag(2, 1)) == Result::InvalidIndex);     // id 2 already at slot 0
    CHECK(upload.addTag(makeTag(4, 1)) == Result::Accepted);
    CHECK(upload.endTags(end(2)) == Result::Accepted);
    CHECK(upload.tags().size() == 2);
}

void testCountMismatchIsRejected()
{
    TagUploadCoordinator upload;
    CHECK(upload.startTags(true, start(1)) == Result::Accepted);
    CHECK(upload.addTag(makeTag(2, 0)) == Result::Accepted);
    CHECK(upload.endTags(end(2)) == Result::CountMismatch);
    CHECK(upload.state() == State::Receiving);
    CHECK(upload.endTags(end(1)) == Result::Accepted);
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
    testIncompleteUploadIsRejectedUntilFilled();
    testStaleUploadIdIsRejected();
    testIndexValidation();
    testCountMismatchIsRejected();
    std::printf("test_tag_upload_coordinator: all tests passed\n");
    return 0;
}
