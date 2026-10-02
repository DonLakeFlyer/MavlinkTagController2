#pragma once

#include <cstddef>
#include <cstdint>

namespace TagTrackerDetectorProtocol {

constexpr uint32_t kMagic = 0x50445454; // "TTDP" on the wire

enum class MessageType : uint16_t {
    Ready = 1,
    Pulse = 2,
    NoDetection = 3,
    CycleComplete = 4,   // the slice's analysis is done and every report for it has been sent
    Failed = 5,
    Arm = 6,
    Heartbeat = 7,
    Armed = 8,
    SliceProgress = 9,   // armed detector, ~1 Hz: IQ accumulated toward the slice's segment
    SliceCaptured = 10,  // the slice's segment is in; its analysis runs in the background
    ComputeProgress = 11, // analysing detector, from the compute thread: each stage change, else <= 1 Hz
};

// ComputeProgressPayload::stage
enum class ComputeStage : uint16_t {
    Spectrogram = 1,
    Search = 2,      // fold search
    Null = 3,        // permutation null; done/total = permutations
    Refit = 4,       // PRI re-fit; done/total = lock candidates
    Measure = 5,     // per-slice measurement; done/total = measurements
};

enum class ValidationResult {
    Valid,
    BadLength,
    BadMagic,
    BadMessageType,
    BadPayloadLength,
};

enum class ErrorCode : uint32_t {
    ProcessFailed = 1,
    UnexpectedException = 2,
    ReportSendFailed = 3,
    LogOpenFailed = 4,
};

#pragma pack(push, 1)
struct Header {
    uint32_t magic;
    uint16_t message_type;
    uint16_t payload_length;
    uint32_t collection_id;
    uint32_t slice_id;
    uint32_t tag_id;
};

struct FailedPayload {
    uint32_t error_code;
};

struct ArmPayload {
    float heading_deg; // detector writes this slice's output under heading-XXX/
};

struct ArmMessage {
    Header header;
    ArmPayload payload;
};

struct SliceProgressPayload {
    uint32_t samples_have;    // IQ samples buffered since ARM
    uint32_t samples_needed;  // segment length that ends the slice's cycle
    uint32_t sample_rate_hz;
};

struct ComputeProgressPayload {
    uint16_t stage;     // ComputeStage
    uint16_t reserved;
    uint32_t done;
    uint32_t total;     // 0 = not countable
};

struct PulsePayload {
    uint32_t frequency_hz;
    uint32_t group_seq_counter;
    uint8_t rate_state;   // TunnelProtocol kRateStateXxx
    uint8_t detection_status;
    uint8_t confirmed_status;
    uint8_t candidate_id; // 0 = provisional lock; >0 = alternate lock candidate
    double start_time_seconds;
    double predict_next_start_seconds;
    double snr;
    double score_ratio;
    double group_snr;
    double noise_psd;
};

struct PulseReport {
    Header header;
    PulsePayload payload;
};
#pragma pack(pop)

constexpr bool isKnownMessageType(uint16_t value)
{
    return value >= static_cast<uint16_t>(MessageType::Ready)
        && value <= static_cast<uint16_t>(MessageType::ComputeProgress);
}

constexpr uint16_t expectedPayloadLength(MessageType type)
{
    switch (type) {
    case MessageType::Pulse:
    case MessageType::NoDetection:
        return sizeof(PulsePayload);
    case MessageType::Failed:
        return sizeof(uint32_t);
    case MessageType::Arm:
        return sizeof(ArmPayload);
    case MessageType::SliceProgress:
        return sizeof(SliceProgressPayload);
    case MessageType::ComputeProgress:
        return sizeof(ComputeProgressPayload);
    case MessageType::Ready:
    case MessageType::CycleComplete:
    case MessageType::Heartbeat:
    case MessageType::Armed:
    case MessageType::SliceCaptured:
        return 0;
    }
    return 0;
}

constexpr ValidationResult validateHeader(const Header& header, size_t packetLength)
{
    if (packetLength != sizeof(Header) + header.payload_length) {
        return ValidationResult::BadLength;
    }
    if (header.magic != kMagic) {
        return ValidationResult::BadMagic;
    }
    if (!isKnownMessageType(header.message_type)) {
        return ValidationResult::BadMessageType;
    }
    const auto type = static_cast<MessageType>(header.message_type);
    if (header.payload_length != expectedPayloadLength(type)) {
        return ValidationResult::BadPayloadLength;
    }
    return ValidationResult::Valid;
}

static_assert(sizeof(Header) == 20);
static_assert(sizeof(FailedPayload) == 4);
static_assert(sizeof(ArmPayload) == 4);
static_assert(sizeof(ArmMessage) == 24);
static_assert(sizeof(SliceProgressPayload) == 12);
static_assert(sizeof(ComputeProgressPayload) == 12);
static_assert(sizeof(PulsePayload) == 60);
static_assert(sizeof(PulseReport) == 80);

} // namespace TagTrackerDetectorProtocol
