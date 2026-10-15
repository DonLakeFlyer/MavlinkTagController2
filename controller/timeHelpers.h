#pragma once

#include <cstdint>

// Wall clock: timestamps that must match other processes (detector, telemetry cache).
uint64_t    msecsSinceEpoch();
double      secondsSinceEpoch();
// Monotonic: intervals and deadlines, unaffected when ClockSync steps the wall clock.
uint64_t    steadyMsecs();