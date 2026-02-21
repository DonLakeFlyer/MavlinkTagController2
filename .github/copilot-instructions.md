# Repository Instructions

This is a monorepo containing the full UAV radio-tag tracking signal pipeline:

| Component | Directory | Description |
| --- | --- | --- |
| Controller | `controller/` | MAVLink tag controller, detector management, pulse relay |
| Decimator | `decimator/` | ZeroMQ→UDP IQ decimator (8×5×5) |
| ZeroMQ publisher | `airspyhf_zeromq/` | Airspy HF+ SDR → ZeroMQ PUB stream |
| Wire format | `shared/tagtracker_wireformat/` | Shared packet header (header-only C library) |

## Wire-format contract

The packet header in `shared/tagtracker_wireformat/zmq_iq_packet.h` is the single source of truth for all three components. Any change to field layout, sizes, endianness, magic, or version semantics is a **breaking contract change** that must:

- Bump `TTWF_ZMQ_IQ_VERSION`.
- Update all three components atomically in the same commit/PR.
- Update the ZeroMQ packet format table in README.md.

## Coding focus

- Keep edits minimal and interface-safe.
- Do not introduce silent behavior changes in stream contracts.
- Wire-format changes require updating the shared header, all consumers, and tests.

## Rate and timing invariants

- Preserve controller assumptions tied to packet cadence, timestamp progression, and continuity.
- Any change to timeout/rate thresholds must be documented with rationale.
- Keep behavior deterministic under intermittent packet loss.

## Failure behavior matrix

- Define behavior for upstream anomalies:
   - malformed packets,
   - dropped packets,
   - out-of-order/duplicate packets,
   - sustained sample-rate mismatch.
- For each anomaly, specify whether to warn, degrade, retry, ignore, or fail.

## Observability requirements

- Expose stable counters/log keys for continuity and timing anomalies.
- Keep enough diagnostic context to correlate issues across components.
- Avoid silent error absorption in control paths.

## Build system

The top-level CMakeLists.txt supports selective builds:

- `BUILD_CONTROLLER` (ON by default)
- `BUILD_DECIMATOR` (ON by default)
- `BUILD_AIRSPYHF_ZMQ` (OFF by default — requires system libairspyhf)

## Test requirements by change type

- Wire-format change: update `shared/test_zmq_iq_packet.c` and `decimator/tests/test_main.cpp`.
- Timing/rate change: validate behavior under nominal, drifted, and degraded-rate scenarios.
- Error-handling change: verify anomaly-specific action paths and operator-visible logs.

## Performance and backpressure constraints

- Keep controller loops bounded and responsive under degraded input conditions.
- Avoid unbounded queues that can mask prolonged upstream problems.
- If introducing buffering/retry logic, ensure loss/lag remains visible in diagnostics.

## Dependencies

- **MAVLink C headers** — fetched via CPM at configure time from `mavlink/c_library_v2`. Pinned to a specific commit in the top-level `CMakeLists.txt`. To update: change the `GIT_TAG` in the `CPMAddPackage(NAME mavlink ...)` call.
- `libs/uavrt_interfaces/` — TunnelProtocol.h. Git submodule from `dynamic-and-active-systems-lab/uavrt_interfaces`.

`uavrt_interfaces` is a git submodule — clone with `--recurse-submodules`. CPM caches downloads in `~/.cache/CPM` by default.
