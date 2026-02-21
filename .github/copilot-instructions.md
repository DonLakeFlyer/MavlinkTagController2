# Repository Instructions

This repository is part of a 3-repo signal pipeline. Keep code and docs aligned across all three:

1. AirspyHFDecimate
   - https://github.com/DonLakeFlyer/AirspyHFDecimate
2. airspyhf-zeromq
   - https://github.com/DonLakeFlyer/airspyhf-zeromq
3. MavlinkTagController2 (this repo)
   - https://github.com/DonLakeFlyer/MavlinkTagController2

## System contract (cross-repo)

- `airspyhf-zeromq` publishes IQ over ZeroMQ PUB with the documented packet header and float32 IQ payload.
- `AirspyHFDecimate` subscribes to that ZeroMQ stream, validates header/sequence/rate, decimates, and emits UDP packets in the downstream expected format.
- `MavlinkTagController2` consumes downstream telemetry/detection products and relies on stream timing and packet continuity assumptions.

## Change policy

When changing packet assumptions, timing, continuity handling, or detector/controller interfaces in this repo:

- Treat it as a cross-repo contract change.
- Update README and runtime configuration docs in this repo.
- Call out required companion changes in:
  - `airspyhf-zeromq` (publisher side), and/or
  - `AirspyHFDecimate` (subscriber/decimator side).
- Prefer backward-compatible transitions when practical (feature flags, version checks, soft warnings before hard-fail).

## Integration checks to preserve

- Maintain compatibility with decimator output framing and timestamp behavior.
- Handle upstream packet continuity assumptions explicitly (drops/out-of-order conditions).
- Keep controller behavior robust to expected stream rate and timing constraints.

## Coding focus

- Keep edits minimal and interface-safe.
- Do not introduce silent behavior changes in stream contracts.
- If ambiguity exists between repos, document assumptions in PR description and README notes.

## Versioning policy

- Any incompatible change to expected decimator output framing, timestamps, or continuity handling must be explicitly versioned.
- Prefer backward-compatible handling paths before requiring new upstream behavior.
- Do not change default assumptions in a silent way.

## Wire-format and ABI contract

- Treat incoming payload structure, timestamp encoding, and field interpretation as stable contracts.
- If parser assumptions change, update docs and call out upstream dependencies.
- Keep binary parsing robust to malformed/incomplete input.

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
- Keep enough diagnostic context to correlate issues back to publisher or decimator.
- Avoid silent error absorption in control paths.

## Cross-repo change checklist

When changing this repo, verify and document impact on:

- `airspyhf-zeromq`: assumptions on source timing/rate quality and continuity semantics.
- `AirspyHFDecimate`: expected UDP framing, timestamp behavior, and rate/continuity diagnostics.
- This repo README/config docs: updated thresholds, flags, fallback behavior, and compatibility notes.

## Test requirements by change type

- Interface/parsing change: add/update compatibility and malformed-input tests.
- Timing/rate change: validate behavior under nominal, drifted, and degraded-rate scenarios.
- Error-handling change: verify anomaly-specific action paths and operator-visible logs.

## Performance and backpressure constraints

- Keep controller loops bounded and responsive under degraded input conditions.
- Avoid unbounded queues that can mask prolonged upstream problems.
- If introducing buffering/retry logic, ensure loss/lag remains visible in diagnostics.

## PR expectations

- Include a cross-repo impact section in PR descriptions.
- State backward-compatibility status and migration guidance.
- Include representative logs/metrics for changed failure-handling behavior.
