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
    CHECK(cache.lookup(1, 5, nullptr, 0, &stored) == Lookup::Miss);

    cache.store({1, 5, 1, "logdir", {}});
    CHECK(cache.lookup(1, 5, nullptr, 0, &stored) == Lookup::Replay);
    CHECK(stored.result == 1);
    CHECK(stored.message == "logdir");
    CHECK(cache.lookup(2, 5, nullptr, 0, &stored) == Lookup::Miss);
}

void testFailureIsReplayedToo()
{
    FakeClock clock;
    RequestCache cache = clock.make();
    cache.store({3, 6, 0, "Not detecting", {}});
    RequestCache::Entry stored;
    CHECK(cache.lookup(3, 6, nullptr, 0, &stored) == Lookup::Replay);
    CHECK(stored.result == 0);
    CHECK(stored.message == "Not detecting");
}

void testCommandMismatch()
{
    FakeClock clock;
    RequestCache cache = clock.make();
    cache.store({7, 5, 1, "", {}});
    RequestCache::Entry stored;
    CHECK(cache.lookup(7, 6, nullptr, 0, &stored) == Lookup::CommandMismatch);
    CHECK(stored.command == 5);     // the earlier command, for the diagnostic
}

void testPayloadMismatch()
{
    FakeClock clock;
    RequestCache cache = clock.make();
    const std::vector<uint8_t> a = {1, 2, 3}, b = {1, 2, 4}, longer = {1, 2, 3, 0};
    cache.store({8, 5, 1, "", a});
    CHECK(cache.lookup(8, 5, a.data(), a.size()) == Lookup::Replay);
    CHECK(cache.lookup(8, 5, b.data(), b.size()) == Lookup::PayloadMismatch);
    CHECK(cache.lookup(8, 5, longer.data(), longer.size()) == Lookup::PayloadMismatch);
    CHECK(cache.lookup(8, 6, b.data(), b.size()) == Lookup::CommandMismatch);    // command checked first
}

void testRequestIdZeroIsNeverCached()
{
    FakeClock clock;
    RequestCache cache = clock.make();
    cache.store({0, 5, 1, "", {}});
    CHECK(cache.size() == 0);
    CHECK(cache.lookup(0, 5, nullptr, 0) == Lookup::Miss);
}

void testEvictionOldestFirst()
{
    FakeClock clock;
    RequestCache cache = clock.make(3);
    cache.store({1, 5, 1, "", {}});
    cache.store({2, 5, 1, "", {}});
    cache.store({3, 5, 1, "", {}});
    cache.store({4, 5, 1, "", {}});
    CHECK(cache.size() == 3);
    CHECK(cache.lookup(1, 5, nullptr, 0) == Lookup::Miss);
    CHECK(cache.lookup(2, 5, nullptr, 0) == Lookup::Replay);
    CHECK(cache.lookup(4, 5, nullptr, 0) == Lookup::Replay);
}

void testRestoreMovesEntryToNewest()
{
    FakeClock clock;
    RequestCache cache = clock.make(2);
    cache.store({1, 5, 1, "", {}});
    cache.store({2, 5, 1, "", {}});
    cache.store({1, 5, 0, "again", {}});    // same id re-stored: one entry, newest
    CHECK(cache.size() == 2);
    cache.store({3, 5, 1, "", {}});
    CHECK(cache.lookup(2, 5, nullptr, 0) == Lookup::Miss);
    RequestCache::Entry stored;
    CHECK(cache.lookup(1, 5, nullptr, 0, &stored) == Lookup::Replay);
    CHECK(stored.message == "again");
}

void testAgeExpiry()
{
    FakeClock clock;
    RequestCache cache = clock.make(16, std::chrono::seconds(30));
    cache.store({1, 5, 1, "", {}});
    clock.now += std::chrono::seconds(29);
    CHECK(cache.lookup(1, 5, nullptr, 0) == Lookup::Replay);
    clock.now += std::chrono::seconds(2);
    CHECK(cache.lookup(1, 5, nullptr, 0) == Lookup::Miss);
    // Expired entries do not raise a mismatch either: a GCS restart may
    // legitimately reuse an old id for a new command.
    CHECK(cache.lookup(1, 6, nullptr, 0) == Lookup::Miss);
}

// The default must outlive the GCS's slowest retry: START_COLLECTION's second
// retry lands ~70 s after the first send.
void testDefaultAgeCoversGcsRetryWindow()
{
    static_assert(RequestCache::kDefaultMaxAge >= std::chrono::seconds(70));
    FakeClock clock;
    RequestCache cache = clock.make();
    cache.store({1, 5, 1, "", {}});
    clock.now += std::chrono::seconds(70);
    CHECK(cache.lookup(1, 5, nullptr, 0) == Lookup::Replay);
}

} // namespace

int main()
{
    testMissThenReplay();
    testFailureIsReplayedToo();
    testCommandMismatch();
    testPayloadMismatch();
    testRequestIdZeroIsNeverCached();
    testEvictionOldestFirst();
    testRestoreMovesEntryToNewest();
    testAgeExpiry();
    testDefaultAgeCoversGcsRetryWindow();
    std::printf("test_request_cache: all tests passed\n");
    return 0;
}
