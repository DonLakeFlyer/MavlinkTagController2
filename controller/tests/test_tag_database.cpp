// Unit tests for TagDatabase: the tag-payload equality used to classify a
// COMMAND_ID_TAG re-send as retransmit vs conflict, and the detector UDP port
// formula / collision check used by START_DETECTION. The upload command
// sequences themselves are exercised in test_tag_upload_coordinator.cpp.

#include "TagDatabase.h"
#include "test_check.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

using TunnelProtocol::TagInfo_t;

namespace {

// Tag 2 as uploaded on 2026-09-17 (session.json, Logs-Rotation-2026-09-17_15-50-14).
TagInfo_t makeTag2()
{
    TagInfo_t tag {};
    tag.id                                      = 2;
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
    // session.json shows these as null: unset priors come across as NaN.
    tag.ip1_mu    = std::nan("");
    tag.ip1_sigma = std::nan("");
    tag.ip2_mu    = std::nan("");
    tag.ip2_sigma = std::nan("");
    return tag;
}

void testFirstTagIsAdded()
{
    TagDatabase db;
    CHECK(db.addTag(makeTag2()) == TagDatabase::AddResult::Added);
    CHECK(db.size() == 1);
    CHECK(db[0].id == 2);
}

// The field failure: START_TAGS, TAG 2, TAG 2 (retransmit), END_TAGS.
void testRetransmitOfIdenticalTagIsNotAppended()
{
    TagDatabase db;
    db.clear();                                                    // START_TAGS
    CHECK(db.addTag(makeTag2()) == TagDatabase::AddResult::Added); // TAG, ACK lost
    CHECK(db.addTag(makeTag2()) == TagDatabase::AddResult::Retransmit); // TAG again
    CHECK(db.size() == 1);                                         // END_TAGS: one detector
}

// NaN priors must not defeat the identical-payload check (NaN != NaN under
// operator==, so the comparison has to be NaN-aware).
void testRetransmitWithNanPriorsIsStillRetransmit()
{
    TagDatabase db;
    TagInfo_t tag = makeTag2();
    CHECK(std::isnan(tag.ip1_mu));
    CHECK(db.addTag(tag) == TagDatabase::AddResult::Added);
    CHECK(db.addTag(tag) == TagDatabase::AddResult::Retransmit);
    CHECK(db.size() == 1);
}

// A retransmit from a sender that encodes "unset" with a different NaN bit
// pattern is still the same tag.
void testRetransmitWithDifferentNanEncodingIsStillRetransmit()
{
    TagDatabase db;
    CHECK(db.addTag(makeTag2()) == TagDatabase::AddResult::Added);

    TagInfo_t other = makeTag2();
    other.ip1_mu    = std::numeric_limits<double>::signaling_NaN();
    other.ip2_sigma = -std::numeric_limits<double>::quiet_NaN();
    CHECK(std::memcmp(&other, &db[0], sizeof(other)) != 0);   // bytes differ...
    CHECK(db.addTag(other) == TagDatabase::AddResult::Retransmit);   // ...value does not
    CHECK(db.size() == 1);
}

// A prior that is set on one copy and unset on the other is a real difference.
void testNanVersusValueIsConflict()
{
    TagDatabase db;
    CHECK(db.addTag(makeTag2()) == TagDatabase::AddResult::Added);

    TagInfo_t withPrior = makeTag2();
    withPrior.ip1_mu = 1.5;
    CHECK(db.addTag(withPrior) == TagDatabase::AddResult::Conflict);
    CHECK(db.size() == 1);
}

void testSameIdDifferentPayloadIsConflict()
{
    TagDatabase db;
    CHECK(db.addTag(makeTag2()) == TagDatabase::AddResult::Added);

    TagInfo_t changed = makeTag2();
    changed.frequency_hz = 147980000;
    CHECK(db.addTag(changed) == TagDatabase::AddResult::Conflict);
    CHECK(db.size() == 1);
    CHECK(db[0].frequency_hz == 147970000); // original definition kept
}

void testDistinctIdsAreAllAdded()
{
    TagDatabase db;
    TagInfo_t tag4 = makeTag2();
    tag4.id = 4;
    tag4.channelizer_channel_number = 2;
    CHECK(db.addTag(makeTag2()) == TagDatabase::AddResult::Added);
    CHECK(db.addTag(tag4) == TagDatabase::AddResult::Added);
    CHECK(db.size() == 2);
    CHECK(db[0].id == 2);
    CHECK(db[1].id == 4);
}

// A new START_TAGS bracket clears the list; the same tag is a fresh add.
void testClearResetsRetransmitDetection()
{
    TagDatabase db;
    CHECK(db.addTag(makeTag2()) == TagDatabase::AddResult::Added);
    db.clear();
    CHECK(db.addTag(makeTag2()) == TagDatabase::AddResult::Added);
    CHECK(db.size() == 1);
}

// Port formula shared by _startPythonDetector and _writeDetectorConfig.
void testDetectorDataPort()
{
    TagInfo_t tag = makeTag2();
    CHECK(TagDatabase::detectorDataPort(tag, true, false) == 10000);   // HF ignores the channel
    CHECK(TagDatabase::detectorDataPort(tag, true, true) == 10001);
    tag.channelizer_channel_number = 1;
    CHECK(TagDatabase::detectorDataPort(tag, false, false) == 20000);
    CHECK(TagDatabase::detectorDataPort(tag, false, true) == 20001);
    tag.channelizer_channel_number = 3;
    CHECK(TagDatabase::detectorDataPort(tag, false, false) == 20004);
}

// The HF pipeline is one decimator channel: every tag would bind UDP 10000.
void testHfModeTwoTagsCollide()
{
    TagDatabase db;
    TagInfo_t tag4 = makeTag2();
    tag4.id = 4;
    tag4.channelizer_channel_number = 2;   // distinct channel does not help in HF mode
    CHECK(db.addTag(makeTag2()) == TagDatabase::AddResult::Added);
    CHECK(db.addTag(tag4) == TagDatabase::AddResult::Added);

    const auto collision = db.findPortCollision(true);
    CHECK(collision.has_value());
    CHECK(collision->tagIdA == 2);
    CHECK(collision->tagIdB == 4);
    CHECK(collision->port == 10000);
}

void testHfModeSingleTagNoCollision()
{
    TagDatabase db;
    CHECK(db.addTag(makeTag2()) == TagDatabase::AddResult::Added);
    CHECK(!db.findPortCollision(true).has_value());
}

void testMiniModeSameChannelCollides()
{
    TagDatabase db;
    TagInfo_t tag4 = makeTag2();
    tag4.id = 4;                             // same channelizer_channel_number (1) as tag 2
    CHECK(db.addTag(makeTag2()) == TagDatabase::AddResult::Added);
    CHECK(db.addTag(tag4) == TagDatabase::AddResult::Added);

    const auto collision = db.findPortCollision(false);
    CHECK(collision.has_value());
    CHECK(collision->tagIdA == 2);
    CHECK(collision->tagIdB == 4);
    CHECK(collision->port == 20000);
}

void testMiniModeDistinctChannelsNoCollision()
{
    TagDatabase db;
    TagInfo_t tag4 = makeTag2();
    tag4.id = 4;
    tag4.channelizer_channel_number = 2;
    CHECK(db.addTag(makeTag2()) == TagDatabase::AddResult::Added);
    CHECK(db.addTag(tag4) == TagDatabase::AddResult::Added);
    CHECK(!db.findPortCollision(false).has_value());
}

void testEmptyDatabaseNoCollision()
{
    TagDatabase db;
    CHECK(!db.findPortCollision(true).has_value());
    CHECK(!db.findPortCollision(false).has_value());
}

} // namespace

int main()
{
    testFirstTagIsAdded();
    testRetransmitOfIdenticalTagIsNotAppended();
    testRetransmitWithNanPriorsIsStillRetransmit();
    testRetransmitWithDifferentNanEncodingIsStillRetransmit();
    testNanVersusValueIsConflict();
    testSameIdDifferentPayloadIsConflict();
    testDistinctIdsAreAllAdded();
    testClearResetsRetransmitDetection();
    testDetectorDataPort();
    testHfModeTwoTagsCollide();
    testHfModeSingleTagNoCollision();
    testMiniModeSameChannelCollides();
    testMiniModeDistinctChannelsNoCollision();
    testEmptyDatabaseNoCollision();
    std::printf("test_tag_database: all tests passed\n");
    return 0;
}
