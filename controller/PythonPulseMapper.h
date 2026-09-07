#pragma once

#include "TunnelProtocol.h"
#include "TelemetryCache.h"
#include "detector_protocol.h"

// Pure translation of a detector TTDP pulse report into the GCS-facing
// PythonPulseInfo_t. Split out of CommandHandler::_handlePythonPulse so the
// field mapping (rate_state, candidate_id, cycle_counter, signal_psd, the
// no-detection zeroing) can be unit-tested without a MavlinkSystem or the
// rotation state machine; those stay in CommandHandler.
//
// `telemetry` is whatever pose the caller has chosen (live cache or the
// slice's ARM-time pose); this function does not decide between them.
TunnelProtocol::PythonPulseInfo_t buildPythonPulseInfo(
    const TagTrackerDetectorProtocol::Header& header,
    const TagTrackerDetectorProtocol::PulsePayload& payload,
    const TelemetryCache::TelemetryCacheEntry_t& telemetry);
