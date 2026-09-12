# Repository Instructions

This is a monorepo containing the full UAV radio-tag tracking signal pipeline
(SDR publisher, decimator, Python detector, MAVLink controller, simulator,
analyzers). The component table and layout are in the root [README.md](../README.md);
the end-to-end flow is in `docs/design/SYSTEM_OVERVIEW.md`.

## Wire-format contract

The packet header in `shared/tagtracker_wireformat/zmq_iq_packet.h` is the single source of truth for the SDR publisher, decimator and simulator. Any change to field layout, sizes, endianness, magic, or version semantics is a **breaking contract change** that must:

- Bump `TTWF_ZMQ_IQ_VERSION`.
- Update all consumers atomically in the same commit/PR.
- Update the ZeroMQ packet format table in `shared/README.md`.

The same rule applies to `shared/detector_protocol.h` (controller ↔ detector): update `detector/detector_protocol.py`, `controller/tests/test_detector_protocol.cpp` and `detector/tests/test_detector_protocol.py` together.

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

All three C/C++ components are built by the top-level CMakeLists.txt (libairspyhf is a required system dependency). Per-component CMake presets (`controller`, `decimator`, `airspyhf-zeromq`, and `-release` variants) filter the build to one target; `debug`/`release` enable `BUILD_TESTING`. `make` / `make test` wrap these.

## Documentation placement

- Component READMEs answer only "what is this and how do I run it" (sections: purpose, build, usage with CLI table from the argument parser, inputs/outputs, key files, tests, further reading, optional troubleshooting). No algorithms, protocol semantics, field results, proposals or source line numbers.
- `docs/design/` describes current code only; `docs/analysis/` is dated and never rewritten; every `docs/proposals/` doc has a `Status:` line and a row in `docs/proposals/README.md`; superseded material goes to `docs/archive/` with a banner. Index: `docs/README.md`.
- When implementing a proposal, update the design doc and the proposal status row in the same PR.
- Test list: `TESTING.md`.

## Test requirements by change type

- Wire-format change: update `shared/tests/test_zmq_iq_packet.c` and `decimator/tests/test_main.cpp`.
- Timing/rate change: validate behavior under nominal, drifted, and degraded-rate scenarios.
- Error-handling change: verify anomaly-specific action paths and operator-visible logs.

## Performance and backpressure constraints

- Keep controller loops bounded and responsive under degraded input conditions.
- Avoid unbounded queues that can mask prolonged upstream problems.
- If introducing buffering/retry logic, ensure loss/lag remains visible in diagnostics.

## Dependencies

- **MAVLink C headers** — fetched via CPM at configure time from `mavlink/c_library_v2`. Pinned to a specific commit in the top-level `CMakeLists.txt`. To update: change the `GIT_TAG` in the `CPMAddPackage(NAME mavlink ...)` call.
- **TunnelProtocol.h** — fetched via CPM at configure time from `DonLakeFlyer/TagTrackerTunnelProtocol` (`CPMAddPackage(NAME TunnelProtocol ...)`). Its `GIT_TAG` must match the pin in TagTracker's `custom/CMakeLists.txt`; bump both together.

CPM caches downloads in `~/.cache/CPM` by default.
