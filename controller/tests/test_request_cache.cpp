// Unit tests for RequestCache: the retry-dedupe layer in front of every GCS
// command. A retry after a lost ACK must be answered with the original ACK;
// a reused id carrying a different command must be refused.

#include "RequestCache.h"
#include "test_check.h"

#include <cstdio>

using Lookup = RequestCache::Lookup;

namespace {

struct FakeClock {
    RequestCache::Clock::time_point now { RequestCache::Clock::duration(0) };
    RequestCache make(size_t capacity = RequestCache::kDefaultCapacity,
                      RequestCache::Clock::duration maxAge = RequestCache::kDefaultMaxAge)
    {
        return RequestCache([this] { return now; }, capacity, maxAge);
    }
};

void testMissThenReplay()
{
    FakeClock clock;
    RequestCache cache = clock.make();
    RequestCache::Entry stored;
    CHECK(cache.lookup(1, 5, &stored) == Lookup::Miss);

    cache.store({1, 5, 1, "logdir"});
    CHECK(cache.lookup(1, 5, &stored) == Lookup::Replay);
    CHECK(stored.result == 1);
    CHECK(stored.message == "logdir");
    CHECK(cache.lookup(2, 5, &stored) == Lookup::Miss);
}

void testFailureIsReplayedToo()
{
    FakeClock clock;
    RequestCache cache = clock.make();
    cache.store({3, 6, 0, "Not detecting"});
    RequestCache::Entry stored;
    CHECK(cache.lookup(3, 6, &stored) == Lookup::Replay);
    CHECK(stored.result == 0);
    CHECK(stored.message == "Not detecting");
}

void testCommandMismatch()
{
    FakeClock clock;
    RequestCache cache = clock.make();
    cache.store({7, 5, 1, ""});
    RequestCache::Entry stored;
    CHECK(cache.lookup(7, 6, &stored) == Lookup::CommandMismatch);
    CHECK(stored.command == 5);     // the earlier command, for the diagnostic
}

void testRequestIdZeroIsNeverCached()
{
    FakeClock clock;
    RequestCache cache = clock.make();
    cache.store({0, 5, 1, ""});
    CHECK(cache.size() == 0);
    CHECK(cache.lookup(0, 5) == Lookup::Miss);
}

void testEvictionOldestFirst()
{
    FakeClock clock;
    RequestCache cache = clock.make(3);
    cache.store({1, 5, 1, ""});
    cache.store({2, 5, 1, ""});
    cache.store({3, 5, 1, ""});
    cache.store({4, 5, 1, ""});
    CHECK(cache.size() == 3);
    CHECK(cache.lookup(1, 5) == Lookup::Miss);
    CHECK(cache.lookup(2, 5) == Lookup::Replay);
    CHECK(cache.lookup(4, 5) == Lookup::Replay);
}

void testRestoreMovesEntryToNewest()
{
    FakeClock clock;
    RequestCache cache = clock.make(2);
    cache.store({1, 5, 1, ""});
    cache.store({2, 5, 1, ""});
    cache.store({1, 5, 0, "again"});    // same id re-stored: one entry, newest
    CHECK(cache.size() == 2);
    cache.store({3, 5, 1, ""});
    CHECK(cache.lookup(2, 5) == Lookup::Miss);
    RequestCache::Entry stored;
    CHECK(cache.lookup(1, 5, &stored) == Lookup::Replay);
    CHECK(stored.message == "again");
}

void testAgeExpiry()
{
    FakeClock clock;
    RequestCache cache = clock.make(16, std::chrono::seconds(30));
    cache.store({1, 5, 1, ""});
    clock.now += std::chrono::seconds(29);
    CHECK(cache.lookup(1, 5) == Lookup::Replay);
    clock.now += std::chrono::seconds(2);
    CHECK(cache.lookup(1, 5) == Lookup::Miss);
    // Expired entries do not raise a mismatch either: a GCS restart may
    // legitimately reuse an old id for a new command.
    CHECK(cache.lookup(1, 6) == Lookup::Miss);
}

} // namespace

int main()
{
    testMissThenReplay();
    testFailureIsReplayedToo();
    testCommandMismatch();
    testRequestIdZeroIsNeverCached();
    testEvictionOldestFirst();
    testRestoreMovesEntryToNewest();
    testAgeExpiry();
    std::printf("test_request_cache: all tests passed\n");
    return 0;
}
