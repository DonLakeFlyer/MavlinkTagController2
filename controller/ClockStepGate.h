#pragma once

#include "TunnelProtocol.h"

#include <cstdint>

// The decimator and detectors anchor their timestamps to the wall clock when
// they start, so ClockSync must not step it while any of them may still run:
// detection or a collection (vehicle time frozen), a raw capture, or a
// detection or IQ recording teardown that has not finished. A teardown whose
// child process hangs unfreezes vehicle time but stays Stopping /
// RecordStopping, which report DETECTING / RECORDING.
inline bool clockStepBlocked(bool vehicleTimeFrozen, uint16_t heartbeatStatus)
{
    return vehicleTimeFrozen
        || heartbeatStatus == HEARTBEAT_STATUS_DETECTING
        || heartbeatStatus == HEARTBEAT_STATUS_CAPTURE
        || heartbeatStatus == HEARTBEAT_STATUS_RECORDING;
}
