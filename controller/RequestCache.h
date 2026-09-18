#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <string>
#include <vector>

// Remembers the ACK sent for recent GCS request ids so a retry after a lost
// ACK is answered with the original result instead of re-executing the
// command. No logging; the caller owns diagnostics.
class RequestCache {
public:
    using Clock = std::chrono::steady_clock;

    struct Entry {
        uint32_t             requestId = 0;
        uint32_t             command   = 0;
        uint32_t             result    = 0;
        std::string          message;
        std::vector<uint8_t> payload;   // whole tunnel payload; a retry must match it byte for byte
    };

    enum class Lookup {
        Miss,               // never seen (or request_id 0, which is never cached)
        Replay,             // same id, command and payload: resend the stored ACK
        CommandMismatch,    // same id, different command: protocol error
        PayloadMismatch,    // same id and command, different payload: protocol error
    };

    static constexpr size_t kDefaultCapacity = 16;
    // Must exceed the longest GCS retry window: START_COLLECTION retries up to
    // 2x at 35 s, so the last retry can arrive ~70 s after the first send.
    static constexpr std::chrono::seconds kDefaultMaxAge { 120 };

    explicit RequestCache(std::function<Clock::time_point()> now = Clock::now,
                          size_t capacity = kDefaultCapacity,
                          Clock::duration maxAge = kDefaultMaxAge);

    /// On any non-Miss result, `stored` (if given) receives the cached entry.
    Lookup lookup(uint32_t requestId, uint32_t command, const void* payload, size_t payloadSize,
                  Entry* stored = nullptr) const;
    void   store(const Entry& entry);
    size_t size() const { return _entries.size(); }

private:
    struct Timed {
        Entry             entry;
        Clock::time_point storedAt;
    };

    bool _expired(const Timed& t) const { return _now() - t.storedAt > _maxAge; }

    std::function<Clock::time_point()> _now;
    size_t                             _capacity;
    Clock::duration                    _maxAge;
    std::deque<Timed>                  _entries;    // oldest first
};
