#include "detector_protocol.h"
#include "test_check.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

using namespace TagTrackerDetectorProtocol;

int main()
{
    static_assert(sizeof(Header) == 20);
    static_assert(sizeof(FailedPayload) == 4);
    static_assert(sizeof(ArmPayload) == 4);
    static_assert(sizeof(ArmMessage) == 24);
    static_assert(sizeof(PulsePayload) == 60);
    static_assert(sizeof(PulseReport) == 80);
    static_assert(offsetof(Header, collection_id) == 8);
    static_assert(offsetof(ArmMessage, payload) == 20);
    static_assert(offsetof(PulsePayload, start_time_seconds) == 12);

    Header header {};
    header.magic = kMagic;
    header.message_type = static_cast<uint16_t>(MessageType::Heartbeat);
    header.payload_length = 0;
    header.collection_id = 0x11223344;
    header.slice_id = 0x55667788;
    header.tag_id = 0x99AABBCC;

    const std::array<uint8_t, sizeof(Header)> expected {
        0x54, 0x54, 0x44, 0x50,
        0x07, 0x00,
        0x00, 0x00,
        0x44, 0x33, 0x22, 0x11,
        0x88, 0x77, 0x66, 0x55,
        0xCC, 0xBB, 0xAA, 0x99,
    };

    CHECK(std::memcmp(&header, expected.data(), expected.size()) == 0);
    CHECK(validateHeader(header, sizeof(Header)) == ValidationResult::Valid);

    header.magic = 0;
    CHECK(validateHeader(header, sizeof(Header)) == ValidationResult::BadMagic);
    header.magic = kMagic;
    CHECK(validateHeader(header, sizeof(Header) - 1) == ValidationResult::BadLength);
    header.message_type = 0xFFFF;
    CHECK(validateHeader(header, sizeof(Header)) == ValidationResult::BadMessageType);

    header.message_type = static_cast<uint16_t>(MessageType::Pulse);
    CHECK(validateHeader(header, sizeof(Header)) == ValidationResult::BadPayloadLength);

    header.message_type = static_cast<uint16_t>(MessageType::Ready);
    header.payload_length = 1;
    CHECK(validateHeader(header, sizeof(Header) + 1) == ValidationResult::BadPayloadLength);

    header.message_type = static_cast<uint16_t>(MessageType::Failed);
    header.payload_length = sizeof(FailedPayload);
    CHECK(validateHeader(header, sizeof(Header) + sizeof(FailedPayload)) == ValidationResult::Valid);

    header.message_type = static_cast<uint16_t>(MessageType::Arm);
    header.payload_length = 0;
    CHECK(validateHeader(header, sizeof(Header)) == ValidationResult::BadPayloadLength);
    header.payload_length = sizeof(ArmPayload);
    CHECK(validateHeader(header, sizeof(ArmMessage)) == ValidationResult::Valid);

    return 0;
}
