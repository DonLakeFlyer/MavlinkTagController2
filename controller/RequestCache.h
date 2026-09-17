#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <string>

// Remembers the ACK sent for recent GCS request ids so a retry after a lost
// ACK is answered with the original result instead of re-executing the
// command. No logging; the caller owns diagnostics.
class RequestCache {
public:
    using Clock = std::chrono::steady_clock;

    struct Entry {
        uint32_t    requestId = 0;
        uint32_t    command   = 0;
        uint32_t    result    = 0;
        std::string message;
    };

    enum class Lookup {
        Miss,               // never seen (or request_id 0, which is never cached)
        Replay,             // same id and command: resend the stored ACK
        CommandMismatch,    // same id, different command: protocol error
    };

    static constexpr size_t kDefaultCapacity = 16;
    static constexpr std::chrono::seconds kDefaultMaxAge { 30 };

    explicit RequestCache(std::function<Clock::time_point()> now = Clock::now,
                          size_t capacity = kDefaultCapacity,
                          Clock::duration maxAge = kDefaultMaxAge);

    /// On Replay and CommandMismatch, `stored` (if given) receives the cached entry.
    Lookup lookup(uint32_t requestId, uint32_t command, Entry* stored = nullptr) const;
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
