// Maps Raspberry Pi firmware throttle bits to the heartbeat's thermal / under-voltage
// flags; getting this wrong either hides an enclosure problem or blames heat for a power fault.

#include "CpuThrottle.h"
#include "test_check.h"

namespace {

constexpr uint8_t kThermal = CPU_THROTTLE_FLAG_THERMAL;
constexpr uint8_t kUnder   = CPU_THROTTLE_FLAG_UNDERVOLTAGE;

void testFlagMapping()
{
    CHECK(cpuThrottleFlags(0x0) == 0);
    CHECK(cpuThrottleFlags(0x1) == kUnder);
    CHECK(cpuThrottleFlags(0x2) == kThermal);                // frequency capped
    CHECK(cpuThrottleFlags(0x4) == kThermal);                // throttled
    CHECK(cpuThrottleFlags(0x8) == kThermal);                // soft temperature limit
    CHECK(cpuThrottleFlags(0x5) == kUnder);                  // under-voltage also sets throttled
    CHECK(cpuThrottleFlags(0x3) == kUnder);                  // under-voltage also caps frequency
    CHECK(cpuThrottleFlags(0x9) == (kThermal | kUnder));     // soft limit is thermal even with under-voltage
    CHECK(cpuThrottleFlags(0xF) == (kThermal | kUnder));
    CHECK(cpuThrottleFlags(0x50000) == 0);                   // sticky "has occurred" bits only
    CHECK(cpuThrottleFlags(0xF0000) == 0);
    CHECK(cpuThrottleFlags(0x50005) == kUnder);
}

void testSinceBootFlagMapping()
{
    CHECK(cpuThrottleSinceBootFlags(0x0) == 0);
    CHECK(cpuThrottleSinceBootFlags(0xF) == 0);                      // current-state bits only
    CHECK(cpuThrottleSinceBootFlags(0x10000) == kUnder);
    CHECK(cpuThrottleSinceBootFlags(0x50000) == kUnder);             // under-voltage also sets throttled
    CHECK(cpuThrottleSinceBootFlags(0x40000) == kThermal);
    CHECK(cpuThrottleSinceBootFlags(0x80000) == kThermal);           // soft temperature limit
    CHECK(cpuThrottleSinceBootFlags(0xF0000) == (kThermal | kUnder));
    CHECK(cpuThrottleSinceBootFlags(0x50005) == kUnder);
    CHECK(cpuThrottleSinceBootFlags(0x100000) == 0);                 // undocumented high bits ignored
}

void testParse()
{
    CHECK(parseFirmwareThrottled("throttled=0x50005\n") == 0x50005u);
    CHECK(parseFirmwareThrottled("throttled=0x0\n") == 0u);
    CHECK(parseFirmwareThrottled("0x8") == 0x8u);
    CHECK(parseFirmwareThrottled("a\n") == 0xAu);

    CHECK(!parseFirmwareThrottled(""));
    CHECK(!parseFirmwareThrottled("\n"));
    CHECK(!parseFirmwareThrottled("throttled="));
    CHECK(!parseFirmwareThrottled("throttled=0xZZ"));
    CHECK(!parseFirmwareThrottled("throttled=0x5 junk"));
    CHECK(!parseFirmwareThrottled("-1"));
    CHECK(!parseFirmwareThrottled("VCHI initialization failed"));
    CHECK(!parseFirmwareThrottled("throttled=0x1ffffffff"));
}

void testHoldThroughFailedPolls()
{
    CpuThrottleHold hold;

    CHECK(hold.update(std::nullopt) == 0);                  // never polled: nothing to hold
    CHECK(hold.update(kThermal) == kThermal);
    CHECK(hold.update(std::nullopt) == kThermal);           // 1st failure held
    CHECK(hold.update(std::nullopt) == kThermal);           // 2nd failure held
    CHECK(hold.update(std::nullopt) == 0);                  // 3rd consecutive failure clears
    CHECK(hold.update(std::nullopt) == 0);

    CHECK(hold.update(kUnder) == kUnder);
    CHECK(hold.update(std::nullopt) == kUnder);
    CHECK(hold.update(kUnder) == kUnder);                   // a good poll resets the failure count
    CHECK(hold.update(std::nullopt) == kUnder);
    CHECK(hold.update(std::nullopt) == kUnder);
    CHECK(hold.update(0) == 0);                             // a good poll is believed immediately
    CHECK(hold.update(std::nullopt) == 0);
}

void testSimulatedSchedule()
{
    SimulatedCpuThrottle sim;
    sim.flags      = kThermal;
    sim.atSeconds  = 10.0;
    sim.forSeconds = 5.0;

    CHECK(simulatedCpuThrottleFlags(sim, 0.0) == 0);
    CHECK(simulatedCpuThrottleFlags(sim, 9.99) == 0);
    CHECK(simulatedCpuThrottleFlags(sim, 10.0) == kThermal);
    CHECK(simulatedCpuThrottleFlags(sim, 14.99) == kThermal);
    CHECK(simulatedCpuThrottleFlags(sim, 15.0) == 0);

    sim.atSeconds = 0.0;
    sim.flags     = kThermal | kUnder;
    CHECK(simulatedCpuThrottleFlags(sim, 0.0) == (kThermal | kUnder));
}

void testNewStickyBits()
{
    CHECK(newStickyThrottleBits(0, 0x0).empty());
    CHECK(newStickyThrottleBits(0, 0x5).empty());                       // current-state bits are not sticky
    CHECK(newStickyThrottleBits(0, 0x10000) == "under-voltage");
    CHECK(newStickyThrottleBits(0, 0x50005) == "under-voltage,throttled");
    CHECK(newStickyThrottleBits(0, 0xF0000) == "under-voltage,freq-capped,throttled,soft-temp-limit");
    CHECK(newStickyThrottleBits(0x50000, 0x50000).empty());             // already reported
    CHECK(newStickyThrottleBits(0x10000, 0x50000) == "throttled");      // only the newly set one
    CHECK(newStickyThrottleBits(0, 0x100000).empty());                  // undocumented high bits ignored
}

} // namespace

int main()
{
    testFlagMapping();
    testSinceBootFlagMapping();
    testParse();
    testSimulatedSchedule();
    testHoldThroughFailedPolls();
    testNewStickyBits();
    return 0;
}
