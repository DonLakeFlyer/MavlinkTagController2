#pragma once

#include <mavlink.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

// The outgoing queue drains one message per 100 ms, so each chunk delays the
// tunnel heartbeat behind it; keep this small (8 chunks = 0.8 s). Must be <= 256 (uint8_t chunk_seq).
constexpr size_t kStatusTextMaxChunks = 8;

// Splits text into STATUSTEXT messages per the MAVLink long-message scheme: a
// message that fits in one 50-byte field is sent with id 0; longer text is
// sent as chunks sharing a non-zero id with increasing chunk_seq, the last
// chunk being shorter than 50 bytes (a lone space is appended if the text is
// an exact multiple of 50). Text beyond kStatusTextMaxChunks chunks is truncated.
inline std::vector<mavlink_statustext_t> splitStatusText(const std::string& text, MAV_SEVERITY severity, uint16_t id)
{
    constexpr size_t kChunkSize = sizeof(mavlink_statustext_t::text);
    constexpr size_t kMaxChunks = kStatusTextMaxChunks;

    // -1: the final chunk must be short, so kMaxChunks full chunks would need one more terminator.
    std::string body = text.substr(0, kChunkSize * kMaxChunks - 1);
    // The GCS renders an empty terminator chunk as a missing-chunk marker, so end with a 1-byte chunk instead.
    if (body.size() > kChunkSize && body.size() % kChunkSize == 0) {
        body.push_back(' ');
    }
    const size_t length = body.size();
    const bool single = length <= kChunkSize;

    std::vector<mavlink_statustext_t> chunks;
    for (size_t offset = 0, seq = 0; ; offset += kChunkSize, seq++) {
        mavlink_statustext_t chunk {};
        chunk.severity  = severity;
        chunk.id        = single ? 0 : id;
        chunk.chunk_seq = static_cast<uint8_t>(seq);

        const size_t count = std::min(kChunkSize, length - offset);
        std::memcpy(chunk.text, body.data() + offset, count);
        chunks.push_back(chunk);

        if (count < kChunkSize) {
            break;
        }
        if (offset + count == length && single) {
            break;
        }
    }
    return chunks;
}
