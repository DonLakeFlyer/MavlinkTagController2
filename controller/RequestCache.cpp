#include "RequestCache.h"

RequestCache::RequestCache(std::function<Clock::time_point()> now, size_t capacity, Clock::duration maxAge)
    : _now(std::move(now))
    , _capacity(capacity)
    , _maxAge(maxAge)
{
}

RequestCache::Lookup RequestCache::lookup(uint32_t requestId, uint32_t command, Entry* stored) const
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
        return it->entry.command == command ? Lookup::Replay : Lookup::CommandMismatch;
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
