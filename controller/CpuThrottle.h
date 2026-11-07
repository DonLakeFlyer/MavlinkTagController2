#pragma once

#include "TunnelProtocol.h"

#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <string>

// Raspberry Pi firmware `vcgencmd get_throttled` bits, current state only.
constexpr uint32_t kFwThrottleUnderVoltage  = 1u << 0;
constexpr uint32_t kFwThrottleFreqCapped    = 1u << 1;
constexpr uint32_t kFwThrottleThrottled     = 1u << 2;
constexpr uint32_t kFwThrottleSoftTempLimit = 1u << 3;
// Sticky "has occurred since boot" copies of the four bits above.
constexpr uint32_t kFwThrottleStickyShift   = 16;
constexpr uint32_t kFwThrottleStickyMask    = 0xFu << kFwThrottleStickyShift;

// Comma-separated names of the sticky bits set in `now` but not in `before`; empty if none.
inline std::string newStickyThrottleBits(uint32_t before, uint32_t now)
{
    static const char* const kNames[] = { "under-voltage", "freq-capped", "throttled", "soft-temp-limit" };
    const uint32_t added = (now & ~before & kFwThrottleStickyMask) >> kFwThrottleStickyShift;
    std::string names;
    for (uint32_t bit = 0; bit < 4; ++bit) {
        if (added & (1u << bit)) {
            if (!names.empty()) {
                names += ',';
            }
            names += kNames[bit];
        }
    }
    return names;
}

// Under-voltage also sets the frequency-capped and throttled bits, so those count as heat only when it is clear.
inline uint8_t cpuThrottleFlags(uint32_t firmwareBits)
{
    uint8_t flags = 0;
    const bool underVoltage = (firmwareBits & kFwThrottleUnderVoltage) != 0;
    const bool slowed       = (firmwareBits & (kFwThrottleFreqCapped | kFwThrottleThrottled)) != 0;

    if ((firmwareBits & kFwThrottleSoftTempLimit) || (slowed && !underVoltage)) {
        flags |= CPU_THROTTLE_FLAG_THERMAL;
    }
    if (underVoltage) {
        flags |= CPU_THROTTLE_FLAG_UNDERVOLTAGE;
    }
    return flags;
}

// The sticky copies mapped the same way; the conditions may have occurred at different times.
inline uint8_t cpuThrottleSinceBootFlags(uint32_t firmwareBits)
{
    return cpuThrottleFlags((firmwareBits & kFwThrottleStickyMask) >> kFwThrottleStickyShift);
}

// Parses "throttled=0x50005" (vcgencmd) or a bare hex value; nullopt for anything else.
inline std::optional<uint32_t> parseFirmwareThrottled(const std::string& text)
{
    size_t start = text.find('=');
    start = (start == std::string::npos) ? 0 : start + 1;

    const char* begin = text.c_str() + start;
    if (!std::isxdigit(static_cast<unsigned char>(*begin))) {
        return std::nullopt;
    }

    char* end = nullptr;
    const unsigned long value = std::strtoul(begin, &end, 16);
    for (const char* p = end; *p; ++p) {
        if (!std::isspace(static_cast<unsigned char>(*p))) {
            return std::nullopt;
        }
    }
    if (value > UINT32_MAX) {
        return std::nullopt;
    }
    return static_cast<uint32_t>(value);
}

// Keeps nonzero flags through brief poll failures so a transient error does not read as recovery.
class CpuThrottleHold {
public:
    static constexpr int kMaxFailedPolls = 3;   // consecutive failures that clear held flags

    uint8_t update(std::optional<uint8_t> polled)
    {
        if (polled) {
            _flags       = *polled;
            _failedPolls = 0;
        } else if (_flags != 0 && ++_failedPolls >= kMaxFailedPolls) {
            _flags       = 0;
            _failedPolls = 0;
        }
        return _flags;
    }

private:
    uint8_t _flags       = 0;
    int     _failedPolls = 0;
};

// Simulator mode: raise `flags` for `forSeconds` starting `atSeconds` after the poller starts.
struct SimulatedCpuThrottle {
    uint8_t flags      = 0;
    double  atSeconds  = 10.0;
    double  forSeconds = 20.0;
};

inline uint8_t simulatedCpuThrottleFlags(const SimulatedCpuThrottle& sim, double elapsedSeconds)
{
    return (elapsedSeconds >= sim.atSeconds && elapsedSeconds < sim.atSeconds + sim.forSeconds) ? sim.flags : 0;
}
