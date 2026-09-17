# Controller Tests

Plain-executable C++ tests (no framework; `test_check.h` provides `CHECK`),
registered with ctest by `controller/CMakeLists.txt`.

## Run

```bash
cmake --preset debug && cmake --build --preset debug
ctest --test-dir build -R 'test_(collection_protocol|collection_coordinator|detector_protocol|bearing_calculator|python_pulse_mapper|tag_database|tag_upload_coordinator|request_cache|detection_coordinator|command_retry)' --output-on-failure
```

## Tests

| Target / file | Covers |
| --- | --- |
| `test_collection_protocol` — `test_collection_protocol.cpp` | Compile-time contract with `TunnelProtocol.h`: `TUNNEL_PROTOCOL_VERSION`, `HeaderInfo_t`/`AckInfo_t` `request_id` offsets, tag upload set fields, command ids (`START_COLLECTION` … `PYTHON_PULSE`), `COLLECTION_STATUS_REVISIT_REQUESTED`, antenna ids, every tunnel struct fits `MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN`. Fails the build when the CPM pin drifts. |
| `test_collection_coordinator` — `test_collection_coordinator.cpp` | `CollectionCoordinator` state machine: start / duplicate / conflict, per-detector READY and ARMED, slice ordering (`Busy`, `OutOfOrder`, `AlreadyComplete` replay), `completeDetector`, `finalize` and `cancel` idempotence and stale ids. |
| `test_detector_protocol` — `test_detector_protocol.cpp` | Layout of `shared/detector_protocol.h`: `sizeof` and `offsetof` for `Header` (20), `ArmPayload` (4), `PulsePayload` (60), `PulseReport` (80). Mirror of `detector/tests/test_detector_protocol.py`; change both together. |
| `test_bearing_calculator` — `test_bearing_calculator.cpp` | `BearingCalculator` fit against the RA-2A pattern with synthetic slices: empty / single / too-few slices, reset, `best_snr`, pattern symmetry, bearings at 0°, 90°, 225°, 350° (wraparound), 22° (off-grid), two tags at once, ±2 dB noise, 16-slice R². |
| `test_python_pulse_mapper` — `test_python_pulse_mapper.cpp` | TTDP `PulsePayload` → tunnel `PythonPulseInfo_t`: detection fields forwarded, each `rate_state`, no-detection zeroes pulse fields, telemetry supplied by caller, `candidate_id 0` is the provisional lock. |
| `test_tag_database` — `test_tag_database.cpp` | `TagDatabase::addTag` under `COMMAND_ID_TAG` retransmission: identical re-send of an id is not appended (`Retransmit`, incl. NaN priors), same id with different payload is `Conflict`, distinct ids all added, `clear()` (new START_TAGS) resets. `detectorDataPort` formula (HF 10000/10001, Mini 20000 + 2·(channel−1)) and `findPortCollision`: two tags in HF mode, two Mini tags on one channel → collision; single HF tag, distinct Mini channels, empty → none. |
| `test_tag_upload_coordinator` — `test_tag_upload_coordinator.cpp` | `TagUploadCoordinator` START_TAGS / TAG / END_TAGS state machine: `Idle` → `Receiving` → `HasTags` / `Empty`; retransmitted TAG and END_TAGS after a lost ACK are idempotent (`Retransmit`), including END_TAGS after an empty upload; TAG/END_TAGS outside a bracket, id 0/1, `k < 2`, conflicting redefinition and START_TAGS on a busy controller are rejected; START_TAGS clears the previous list. Upload set: END_TAGS is `Incomplete` (with `missingIndices()`) until every `tag_index` arrived, `StaleUpload` for another `upload_id`, `InvalidIndex` / `CountMismatch` on bad indices or counts. Drives the 15:13 and 15:17 command sequences from the 2026-09-17 controller log. |
| `test_request_cache` — `test_request_cache.cpp` | `RequestCache`: miss then replay of the stored ACK (success and failure alike), same id with a different command is `CommandMismatch`, `request_id 0` never cached, oldest-first eviction, re-store moves to newest, age expiry via injected clock. |
| `test_detection_coordinator` — `test_detection_coordinator.cpp` | `DetectionCoordinator`: `Idle` → `HasTags` → `Starting` → `Detecting` → `Stopping` → `HasTags`; `AlreadyStarting` / `AlreadyStopping` for in-flight retries, `NoTags`, `AlreadyDetecting`, `NotDetecting`, `StartInProgress`, `Busy`; failed start returns to `HasTags`; stray `*Finished` calls ignored; heartbeat sink fires once per published change; `waitWhile` wakes on transition. |
| `test_command_retry` — `test_command_retry.cpp` | `TunnelCommandDispatcher` against a fake `CommandActions`: ACK echoes `request_id`; every TAG/END_TAGS ACK lost → replayed, one tag per id; TAG lost in flight → END_TAGS NACKs `incomplete: missing N`, then succeeds; GCS restart mid-upload / while Starting / while Stopping (new ids, intent satisfied, pipeline invoked once); pipeline start failure retryable; STOP while Starting refused; tag upload refused while detecting or capturing; request-id reuse with another command NACKed; all pass-through commands deduped (failures replayed as failures); short / header-only / unknown frames; controller restart falls back to the state machines; `request_id 0` never deduped. |

Adding a test: add the source under `controller/tests/`, an `add_executable` +
`add_test(NAME …)` in `controller/CMakeLists.txt`, and a row above.
