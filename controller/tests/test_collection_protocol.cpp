#include "TunnelProtocol.h"
#include "test_check.h"

#include <cstddef>
#include <cstdint>

using namespace TunnelProtocol;

int main()
{
    static_assert(TUNNEL_PROTOCOL_VERSION == 1);
    static_assert(COMMAND_ID_START_COLLECTION == 15);
    static_assert(COMMAND_ID_START_COLLECTION_SLICE == 16);
    static_assert(COMMAND_ID_FINISH_COLLECTION == 17);
    static_assert(COMMAND_ID_BEARING_RESULT == 18);
    static_assert(COMMAND_ID_COLLECTION_STATUS == 19);

    static_assert(sizeof(StartCollection_t) <= MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN);
    static_assert(sizeof(StartCollectionSlice_t) <= MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN);
    static_assert(sizeof(FinishCollection_t) <= MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN);
    static_assert(sizeof(CollectionStatus_t) <= MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN);
    static_assert(sizeof(BearingResult_t) <= MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN);
    static_assert(sizeof(PulseInfo_t) <= MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN);
    static_assert(sizeof(Heartbeat_t) == 16);

    Heartbeat_t heartbeat {};
    heartbeat.protocol_version = TUNNEL_PROTOCOL_VERSION;
    CHECK(heartbeat.protocol_version == 1);

    StartCollectionSlice_t slice {};
    slice.header.command = COMMAND_ID_START_COLLECTION_SLICE;
    slice.collection_id = 0x11223344;
    slice.slice_id = 7;
    slice.heading_deg = 135.0f;

    CHECK(slice.collection_id == 0x11223344);
    CHECK(slice.slice_id == 7);
    CHECK(slice.heading_deg == 135.0f);
    CHECK(TunnelProtocolValidateSizes);
    return 0;
}
