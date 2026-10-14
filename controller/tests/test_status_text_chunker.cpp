// splitStatusText: MAVLink long-STATUSTEXT chunking (single message, exact
// multiples of the chunk size, truncation at the chunk cap).

#include "StatusTextChunker.h"
#include "test_check.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

constexpr size_t kChunk = sizeof(mavlink_statustext_t::text);

std::string reassemble(const std::vector<mavlink_statustext_t>& chunks)
{
    std::string text;
    for (const auto& chunk : chunks) {
        text.append(chunk.text, strnlen(chunk.text, kChunk));
    }
    return text;
}

// Reassembly is complete when the last chunk is short; every earlier chunk must be full.
void checkWellFormed(const std::vector<mavlink_statustext_t>& chunks, const std::string& text, size_t expectedChunks)
{
    CHECK(chunks.size() == expectedChunks);
    for (size_t i = 0; i < chunks.size(); i++) {
        CHECK(chunks[i].severity == MAV_SEVERITY_WARNING);
        CHECK(chunks[i].chunk_seq == i);
        CHECK(chunks[i].id == chunks[0].id);
        const bool last = i + 1 == chunks.size();
        CHECK(last || strnlen(chunks[i].text, kChunk) == kChunk);
    }
    if (chunks.size() > 1) {
        CHECK(chunks[0].id == 42);
        CHECK(strnlen(chunks.back().text, kChunk) < kChunk);
    }
    CHECK(reassemble(chunks) == text);
}

struct Row {
    const char* name;
    size_t      length;
    size_t      expectedChunks;
};

// A message that fits in one field is never chunked (id 0), even at exactly 50 bytes.
void testShortTextIsASingleUnchunkedMessage()
{
    for (const Row& row : { Row { "empty", 0, 1 }, Row { "short", 10, 1 }, Row { "exactly one chunk", kChunk, 1 } }) {
        const std::string text(row.length, 'a');
        const auto chunks = splitStatusText(text, MAV_SEVERITY_WARNING, 42);
        checkWellFormed(chunks, text, row.expectedChunks);
        CHECK(chunks[0].id == 0);
    }
    std::printf("PASS: testShortTextIsASingleUnchunkedMessage\n");
}

// Long text is sent as chunks sharing one id; an exact multiple of 50 ends with a 1-byte space chunk, never an empty one.
void testLongTextIsChunkedAndReassembles()
{
    for (const Row& row : { Row { "one over", kChunk + 1, 2 }, Row { "two full chunks", 2 * kChunk, 3 }, Row { "mid third chunk", 2 * kChunk + 7, 3 }, Row { "seven full chunks", 7 * kChunk, 8 } }) {
        std::string text;
        for (size_t i = 0; i < row.length; i++) {
            text.push_back(static_cast<char>('a' + i % 26));
        }
        const std::string expected = row.length % kChunk == 0 ? text + " " : text;
        const auto chunks = splitStatusText(text, MAV_SEVERITY_WARNING, 42);
        checkWellFormed(chunks, expected, row.expectedChunks);
        CHECK(strnlen(chunks.back().text, kChunk) > 0);
    }
    std::printf("PASS: testLongTextIsChunkedAndReassembles\n");
}

// Each chunk holds the outgoing queue for 100 ms, so text beyond the cap is dropped.
void testOverlongTextIsTruncatedToTheChunkCap()
{
    const std::string text(kChunk * (kStatusTextMaxChunks + 5), 'x');
    const auto chunks = splitStatusText(text, MAV_SEVERITY_WARNING, 42);
    CHECK(chunks.size() == kStatusTextMaxChunks);
    CHECK(chunks.back().chunk_seq == kStatusTextMaxChunks - 1);
    CHECK(reassemble(chunks) == std::string(kChunk * kStatusTextMaxChunks - 1, 'x'));
    std::printf("PASS: testOverlongTextIsTruncatedToTheChunkCap\n");
}

}

int main()
{
    testShortTextIsASingleUnchunkedMessage();
    testLongTextIsChunkedAndReassembles();
    testOverlongTextIsTruncatedToTheChunkCap();
    std::printf("All StatusTextChunker tests passed\n");
    return 0;
}
