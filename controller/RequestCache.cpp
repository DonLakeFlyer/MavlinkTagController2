#include "RequestCache.h"

#include <cstring>

RequestCache::RequestCache(std::function<Clock::time_point()> now, size_t capacity, Clock::duration maxAge)
    : _now(std::move(now))
    , _capacity(capacity)
    , _maxAge(maxAge)
{
}

RequestCache::Lookup RequestCache::lookup(uint32_t requestId, uint32_t command, const void* payload, size_t payloadSize,
                                          Entry* stored) const
{
    if (requestId == 0) {
        return Lookup::Miss;
    }
    for (auto it = _entries.rbegin(); it != _entries.rend(); ++it) {
        if (it->entry.requestId != requestId || _expired(*it)) {
            continue;
        }
        if (stored) {
            *stored = it->entry;
        }
        if (it->entry.command != command) {
            return Lookup::CommandMismatch;
        }
        const auto& p = it->entry.payload;
        const bool same = p.size() == payloadSize && (payloadSize == 0 || std::memcmp(p.data(), payload, payloadSize) == 0);
        return same ? Lookup::Replay : Lookup::PayloadMismatch;
    }
    return Lookup::Miss;
}

void RequestCache::store(const Entry& entry)
{
    if (entry.requestId == 0) {
        return;
    }
    for (auto it = _entries.begin(); it != _entries.end(); ++it) {
        if (it->entry.requestId == entry.requestId) {
            _entries.erase(it);
            break;
        }
    }
    _entries.push_back({entry, _now()});
    while (_entries.size() > _capacity) {
        _entries.pop_front();
    }
}
