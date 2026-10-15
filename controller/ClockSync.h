#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>

// Steps the system clock to the autopilot's GPS time, at most once per run.
//
// The rPi has no RTC and boots at its last saved time, so every battery swap
// leaves it behind by the time it was off. The decimator anchors its packet
// timestamps to CLOCK_REALTIME once at start and the telemetry cache stamps
// with it too, so a step must never land inside a session: the caller passes
// sessionActive and the step waits until it is false.
//
// Decision logic only; the clock, NTP state and steady time are injected so
// tests never touch the real clock. Not thread-safe: the caller serialises.
class ClockSync
{
public:
    static constexpr double  kStepThresholdSecs = 1.0;     // smaller offsets are left alone
    static constexpr double  kSettleSecs        = 5.0;     // 3D fix and self-consistent time for this long first
    static constexpr double  kConsistencySecs   = 1.0;     // max drift of SYSTEM_TIME from the steady clock during the settle
    static constexpr double  kNoGpsMessageSecs  = 30.0;    // report a missing GPS_RAW_INT stream after this
    static constexpr uint8_t kMinFixType        = 3;       // GPS_FIX_TYPE_3D_FIX

    struct Ops {
        std::function<double()>     realtimeNow;        // CLOCK_REALTIME, seconds since epoch
        std::function<int(double)>  setRealtime;        // 0, or errno
        std::function<bool()>       ntpSynchronized;    // kernel clock disciplined by NTP
    };

    enum class State {
        Disabled,           // this process may not step the clock; offsets are still reported
        Waiting,            // no trustworthy GPS time yet, or deferred by a session
        Stepped,
        InSync,             // first trustworthy offset was within kStepThresholdSecs
        NtpSynchronized,    // NTP owns the clock
        Failed,
    };

    struct Outcome {
        enum class Kind { None, Waiting, Deferred, Skipped, Stepped, Failed };
        Kind        kind            = Kind::None;
        double      offsetSeconds   = 0.0;      // GPS minus system clock, when known
        std::string message;                    // log text, follows the "CLOCK_SYNC" key
    };

    struct Summary {
        std::string message;
        bool        warn = false;   // synced earlier but the offset is over the threshold again
    };

    // With disabledReason the clock is never stepped, but offsets are still tracked.
    ClockSync(Ops ops, double minPlausibleEpochSeconds, std::optional<std::string> disabledReason = std::nullopt);

    // GPS_RAW_INT.fix_type from the autopilot.
    void    gpsFixType(uint8_t fixType);
    // SYSTEM_TIME.time_unix_usec from the autopilot, received at steadyNowSeconds.
    Outcome systemTime(uint64_t timeUnixUsec, double steadyNowSeconds, bool sessionActive);
    // One line for the session log, so each session explains its own timestamps.
    Summary sessionStartSummary(double steadyNowSeconds) const;

    State   state() const { return _state; }

    // __DATE__ ("Oct 15 2026") as 00:00 UTC that day; nullopt if it does not parse.
    static std::optional<double>        buildDateEpochSeconds(const char* compilerDate);
    // Lower bound for a plausible autopilot time: the date this file was compiled.
    static double                       minPlausibleEpochSeconds();
    static Ops                          systemOps();
    // nullopt when this process may step the clock.
    static std::optional<std::string>   platformDisabledReason();
    // Keep CAP_SYS_TIME out of spawned processes; the controller keeps its own.
    static std::optional<std::string>   clearAmbientCapabilities();

private:
    enum class WaitReason { None, TimeInvalid, NoGpsMessage, NoFix, Drift };

    struct Sample {
        double  gpsSeconds;
        double  steadySeconds;
        bool    fix3d;
    };

    struct Settle {
        double  startSteady;
        double  startGps;
    };

    Outcome _waiting(WaitReason reason, std::string message);

    Ops                         _ops;
    double                      _minPlausibleEpochSeconds;
    std::optional<std::string>  _disabledReason;
    State                       _state;

    bool                        _gpsMessageSeen     = false;
    uint8_t                     _fixType            = 0;
    std::optional<double>       _firstValidSteady;
    std::optional<Sample>       _latest;
    std::optional<Settle>       _settle;
    WaitReason                  _loggedWaitReason   = WaitReason::None;
    std::string                 _waitMessage;
    bool                        _deferredLogged     = false;
    double                      _steppedSeconds     = 0.0;
    std::string                 _failure;
};
