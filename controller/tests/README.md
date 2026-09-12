# Controller Tests

Plain-executable C++ tests (no framework; `test_check.h` provides `CHECK`),
registered with ctest by `controller/CMakeLists.txt`.

## Run

```bash
cmake --preset debug && cmake --build --preset debug
ctest --test-dir build -R 'test_(collection_protocol|collection_coordinator|detector_protocol|bearing_calculator|python_pulse_mapper)' --output-on-failure
```

## Tests

| Target / file | Covers |
| --- | --- |
| `test_collection_protocol` — `test_collection_protocol.cpp` | Compile-time contract with `TunnelProtocol.h`: `TUNNEL_PROTOCOL_VERSION`, command ids (`START_COLLECTION` … `PYTHON_PULSE`), `COLLECTION_STATUS_REVISIT_REQUESTED`, antenna ids, every tunnel struct fits `MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN`. Fails the build when the CPM pin drifts. |
| `test_collection_coordinator` — `test_collection_coordinator.cpp` | `CollectionCoordinator` state machine: start / duplicate / conflict, per-detector READY and ARMED, slice ordering (`Busy`, `OutOfOrder`, `AlreadyComplete` replay), `completeDetector`, `finalize` and `cancel` idempotence and stale ids. |
| `test_detector_protocol` — `test_detector_protocol.cpp` | Layout of `shared/detector_protocol.h`: `sizeof` and `offsetof` for `Header` (20), `ArmPayload` (4), `PulsePayload` (60), `PulseReport` (80). Mirror of `detector/tests/test_detector_protocol.py`; change both together. |
| `test_bearing_calculator` — `test_bearing_calculator.cpp` | `BearingCalculator` fit against the RA-2A pattern with synthetic slices: empty / single / too-few slices, reset, `best_snr`, pattern symmetry, bearings at 0°, 90°, 225°, 350° (wraparound), 22° (off-grid), two tags at once, ±2 dB noise, 16-slice R². |
| `test_python_pulse_mapper` — `test_python_pulse_mapper.cpp` | TTDP `PulsePayload` → tunnel `PythonPulseInfo_t`: detection fields forwarded, each `rate_state`, no-detection zeroes pulse fields, telemetry supplied by caller, `candidate_id 0` is the provisional lock. |

Adding a test: add the source under `controller/tests/`, an `add_executable` +
`add_test(NAME …)` in `controller/CMakeLists.txt`, and a row above.
