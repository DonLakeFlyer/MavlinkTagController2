#include "ClockSync.h"
#include "formatString.h"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <utility>

#ifdef __linux__
#include <linux/capability.h>
#include <sys/prctl.h>
#include <sys/syscall.h>
#include <sys/timex.h>
#include <unistd.h>
#endif

namespace {

// 2026-01-01 00:00:00 UTC, used only if __DATE__ does not parse.
constexpr double kFallbackMinEpochSeconds = 1767225600.0;

std::string utcString(double epochSeconds)
{
    const auto t = static_cast<std::time_t>(std::floor(epochSeconds));
    struct tm tmBuf {};
    gmtime_r(&t, &tmBuf);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tmBuf);
    return buf;
}

}

ClockSync::ClockSync(Ops ops, double minPlausibleEpochSeconds, std::optional<std::string> disabledReason)
    : _ops                      (std::move(ops))
    , _minPlausibleEpochSeconds (minPlausibleEpochSeconds)
    , _disabledReason           (std::move(disabledReason))
    , _state                    (_disabledReason ? State::Disabled : State::Waiting)
{

}

void ClockSync::gpsFixType(uint8_t fixType)
{
    _gpsMessageSeen = true;
    _fixType        = fixType;
    // A fix lost between two SYSTEM_TIME samples must still restart the settle.
    if (fixType < kMinFixType) {
        _settle.reset();
    }
}

ClockSync::Outcome ClockSync::_waiting(WaitReason reason, std::string message)
{
    _waitMessage = message;
    if (reason == _loggedWaitReason) {
        return {};
    }
    _loggedWaitReason = reason;
    return { Outcome::Kind::Waiting, 0.0, std::move(message) };
}

ClockSync::Outcome ClockSync::systemTime(uint64_t timeUnixUsec, double steadyNowSeconds, bool sessionActive)
{
    const double gpsSeconds = static_cast<double>(timeUnixUsec) / 1e6;
    // Before GPS lock the autopilot may send 0 or its own boot-time clock.
    const bool   valid      = timeUnixUsec != 0 && gpsSeconds >= _minPlausibleEpochSeconds;
    const bool   fix3d      = _fixType >= kMinFixType;

    if (valid) {
        _latest = Sample { gpsSeconds, steadyNowSeconds, fix3d };
        if (!_firstValidSteady) {
            _firstValidSteady = steadyNowSeconds;
        }
    }

    if (_state != State::Waiting) {
        return {};
    }

    if (!valid) {
        _settle.reset();
        return _waiting(WaitReason::TimeInvalid, "waiting: autopilot time not valid yet");
    }

    if (!fix3d) {
        _settle.reset();
        if (!_gpsMessageSeen) {
            if (steadyNowSeconds - *_firstValidSteady < kNoGpsMessageSecs) {
                _waitMessage = "waiting: no GPS_RAW_INT yet";
                return {};
            }
            return _waiting(WaitReason::NoGpsMessage, "waiting: no GPS_RAW_INT received from autopilot");
        }
        return _waiting(WaitReason::NoFix, formatString("waiting: no 3D GPS fix (fix_type %u)", static_cast<unsigned>(_fixType)));
    }

    // The autopilot may set its own clock from GPS a moment after reporting the
    // fix, so the time must also advance in step with the steady clock for
    // kSettleSecs. Measured from the start of the window, not sample to sample,
    // so a stalled or slow time cannot creep through one small step at a time.
    if (_settle) {
        const double drift = (gpsSeconds - _settle->startGps) - (steadyNowSeconds - _settle->startSteady);
        if (std::fabs(drift) > kConsistencySecs) {
            _settle = Settle { steadyNowSeconds, gpsSeconds };
            // Logged once per episode: a stalled or slow clock restarts the settle every few samples.
            return _waiting(WaitReason::Drift,
                            formatString("waiting: autopilot time drifted %+.1f s from the steady clock; restarting %.0f s settle",
                                         drift, kSettleSecs));
        }
    } else {
        _settle = Settle { steadyNowSeconds, gpsSeconds };
    }
    if (steadyNowSeconds - _settle->startSteady < kSettleSecs) {
        _waitMessage = "waiting: autopilot time settling";
        return {};
    }

    if (_ops.ntpSynchronized()) {
        _state = State::NtpSynchronized;
        return { Outcome::Kind::Skipped, 0.0, "skipped: system clock is NTP-synchronized" };
    }

    const double systemSeconds = _ops.realtimeNow();
    const double offset        = gpsSeconds - systemSeconds;
    if (std::fabs(offset) <= kStepThresholdSecs) {
        _state = State::InSync;
        return { Outcome::Kind::Skipped, offset,
                 formatString("skipped: offset %+.3f s is within %.1f s", offset, kStepThresholdSecs) };
    }

    if (sessionActive) {
        _waitMessage = "deferred: session active";
        if (_deferredLogged) {
            return {};
        }
        _deferredLogged = true;
        return { Outcome::Kind::Deferred, offset,
                 formatString("deferred: session active (offset %+.3f s); will step when it ends", offset) };
    }

    // The sample was received microseconds ago; serial link latency (~10 ms) is not corrected.
    const int err = _ops.setRealtime(gpsSeconds);
    if (err != 0) {
        _state   = State::Failed;
        _failure = formatString("clock_settime: %s", std::strerror(err));
        return { Outcome::Kind::Failed, offset, "failed: " + _failure };
    }

    _state          = State::Stepped;
    _steppedSeconds = offset;
    return { Outcome::Kind::Stepped, offset,
             formatString("stepped system clock %+.3f s to GPS time (%s -> %s UTC)",
                          offset, utcString(systemSeconds).c_str(), utcString(gpsSeconds).c_str()) };
}

ClockSync::Summary ClockSync::sessionStartSummary(double steadyNowSeconds) const
{
    std::string status;
    switch (_state) {
    case State::Disabled:
        status = "sync disabled (" + _disabledReason.value_or("") + ")";
        break;
    case State::Waiting:
        status = "not synced yet (" + (_waitMessage.empty() ? std::string("no autopilot time") : _waitMessage) + ")";
        break;
    case State::Stepped:
        status = formatString("stepped %+.3f s earlier this run", _steppedSeconds);
        break;
    case State::InSync:
        status = "no step needed";
        break;
    case State::NtpSynchronized:
        status = "NTP-synchronized";
        break;
    case State::Failed:
        status = "sync failed (" + _failure + ")";
        break;
    }

    Summary summary;
    if (!_latest) {
        summary.message = "session start: GPS offset n/a (no autopilot time); " + status;
    } else if (!_latest->fix3d) {
        summary.message = "session start: GPS offset n/a (no 3D fix); " + status;
    } else {
        const double gpsNow = _latest->gpsSeconds + (steadyNowSeconds - _latest->steadySeconds);
        const double offset = gpsNow - _ops.realtimeNow();
        summary.message = formatString("session start: GPS offset %+.3f s; ", offset) + status;
        const bool synced = _state == State::Stepped || _state == State::InSync || _state == State::NtpSynchronized;
        summary.warn = synced && std::fabs(offset) > kStepThresholdSecs;
    }
    return summary;
}

std::optional<double> ClockSync::buildDateEpochSeconds(const char* compilerDate)
{
    static const char* const kMonths[] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun",
                                           "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };
    char monthName[4] {};
    int  day  = 0;
    int  year = 0;
    if (compilerDate == nullptr || std::sscanf(compilerDate, "%3s %d %d", monthName, &day, &year) != 3) {
        return std::nullopt;
    }
    int month = -1;
    for (int i = 0; i < 12; ++i) {
        if (std::strcmp(monthName, kMonths[i]) == 0) {
            month = i;
            break;
        }
    }
    if (month < 0 || day < 1 || day > 31 || year < 1970) {
        return std::nullopt;
    }
    struct tm tmBuf {};
    tmBuf.tm_year = year - 1900;
    tmBuf.tm_mon  = month;
    tmBuf.tm_mday = day;
    return static_cast<double>(timegm(&tmBuf));
}

double ClockSync::minPlausibleEpochSeconds()
{
    // Only refreshed when this file recompiles, so it can be older than the binary; still a valid lower bound.
    return buildDateEpochSeconds(__DATE__).value_or(kFallbackMinEpochSeconds);
}

ClockSync::Ops ClockSync::systemOps()
{
    Ops ops;
    ops.realtimeNow = [] {
        timespec ts {};
        clock_gettime(CLOCK_REALTIME, &ts);
        return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
    };
#ifdef __linux__
    ops.setRealtime = [](double epochSeconds) {
        const double whole = std::floor(epochSeconds);
        timespec ts {};
        ts.tv_sec  = static_cast<time_t>(whole);
        ts.tv_nsec = static_cast<long>((epochSeconds - whole) * 1e9);
        return clock_settime(CLOCK_REALTIME, &ts) == 0 ? 0 : errno;
    };
    ops.ntpSynchronized = [] {
        timex tx {};
        const int clockState = adjtimex(&tx);
        return clockState != -1 && clockState != TIME_ERROR;
    };
#else
    ops.setRealtime     = [](double) { return ENOTSUP; };
    ops.ntpSynchronized = [] { return false; };
#endif
    return ops;
}

std::optional<std::string> ClockSync::platformDisabledReason()
{
#ifdef __linux__
    __user_cap_header_struct header { _LINUX_CAPABILITY_VERSION_3, 0 };
    __user_cap_data_struct   data[_LINUX_CAPABILITY_U32S_3] {};
    if (syscall(SYS_capget, &header, data) != 0) {
        return formatString("capget failed: %s", std::strerror(errno));
    }
    if ((data[CAP_TO_INDEX(CAP_SYS_TIME)].effective & CAP_TO_MASK(CAP_SYS_TIME)) == 0) {
        return std::string("no CAP_SYS_TIME; granted by the mavlink-tag-controller service");
    }
    return std::nullopt;
#else
    return std::string("not Linux");
#endif
}

std::optional<std::string> ClockSync::clearAmbientCapabilities()
{
#ifdef __linux__
    if (prctl(PR_CAP_AMBIENT, PR_CAP_AMBIENT_CLEAR_ALL, 0, 0, 0) != 0) {
        return formatString("prctl(PR_CAP_AMBIENT_CLEAR_ALL) failed: %s", std::strerror(errno));
    }
#endif
    return std::nullopt;
}
