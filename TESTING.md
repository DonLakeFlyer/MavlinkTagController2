# Testing

Two test systems: **ctest** for the C/C++ components and **pytest** for the
Python ones. CI runs both in `.github/workflows/tests.yml`; the `ci.yml` build
workflow behind the README badge runs ctest only, as does `make test`.

## Prerequisites

```bash
./setup_venv.sh                          # Python: numpy, scipy, pyzmq, matplotlib, pytest
cmake --preset debug && cmake --build --preset debug    # C/C++ with BUILD_TESTING=ON
```

## C / C++ (ctest)

```bash
ctest --preset debug --output-on-failure          # everything
ctest --test-dir build -R bearing                 # one target by regex
make test                                         # configure + build + ctest, release preset
```

| Target | Source | Covers | Notes |
| --- | --- | --- | --- |
| `tagtracker_wireformat_tests` | `shared/tests/test_zmq_iq_packet.c` | ZMQ IQ header layout, magic, version, sizes | Must be updated with any wire-format change |
| `airspyhf_decimator_tests` | `decimator/tests/test_main.cpp` | CLI parsing, FIR/decimation, frequency shift, ZMQ frame handling, rate tracking | |
| `test_collection_protocol` | `controller/tests/test_collection_protocol.cpp` | `TunnelProtocol.h` version, command ids and struct sizes the controller depends on (`SetLogLevel_t`, `OperationProgress_t`, collection structs) | Guards the CPM pin |
| `test_collection_coordinator` | `controller/tests/test_collection_coordinator.cpp` | ARM/ARMED/CYCLE_COMPLETE state machine, duplicates, cancel/finalize | |
| `test_detector_protocol` | `controller/tests/test_detector_protocol.cpp` | TTDP header/payload layout (`shared/detector_protocol.h`) | Mirror of `detector/tests/test_detector_protocol.py` |
| `test_bearing_calculator` | `controller/tests/test_bearing_calculator.cpp` | Pattern fit, weighted headings and residuals, candidate selection, wraparound, noise tolerance, confirmed/revisit semantics; run for both antenna patterns | |
| `test_python_pulse_mapper` | `controller/tests/test_python_pulse_mapper.cpp` | TTDP pulse → tunnel `PythonPulseInfo_t` mapping | |
| `test_tag_database` | `controller/tests/test_tag_database.cpp` | `COMMAND_ID_TAG` payload equality (retransmit vs conflict, NaN-aware); detector UDP port formula and START_DETECTION port-collision detection | |
| `test_tag_upload_coordinator` | `controller/tests/test_tag_upload_coordinator.cpp` | START_TAGS / TAG / END_TAGS state machine, retransmit idempotence, upload-set integrity (`upload_id`, `tag_count`, `tag_index`, missing-index NACK), out-of-bracket and validation rejects | Reproduces the 2026-09-17 duplicate-tag collection failure |
| `test_request_cache` | `controller/tests/test_request_cache.cpp` | Request-id ACK replay cache: miss / replay / command mismatch / payload mismatch, eviction, age expiry | |
| `test_detection_coordinator` | `controller/tests/test_detection_coordinator.cpp` | Idle / HasTags / Starting / Detecting / Stopping / Capturing transitions, in-flight start/stop idempotence, start–capture exclusion, heartbeat publication | |
| `test_command_retry` | `controller/tests/test_command_retry.cpp` | Whole controller command/ACK path (`TunnelCommandDispatcher`) under lost ACKs: every command retried, lost TAG, GCS restart mid-upload / mid-start / mid-stop, request-id reuse, malformed frames, controller restart fallback, `Busy` NACK of long-running commands while an operation runs | Fake `CommandActions`; no MAVLink or processes |
| `test_operation_progress` | `controller/tests/test_operation_progress.cpp` | `OperationProgressReporter`: begin/update/finish frame sequence, busy gate, indeterminate → counted, 1 Hz re-send while RUNNING and bounded re-send of the terminal frame, clamped-step and truncated-message dedup, stable log line | |
| `test_rotation_progress` | `controller/tests/test_rotation_progress.cpp` | `RotationProgress`: step layout across startup/slices/finalize, dwell length = longest segment reported by any detector, step driven by the least-credited detector, segment restarts add extra steps (single, simultaneous and asymmetric detectors), mid-slice segment-length change rejected, bounded compute ticks after the segment fills, monotonic step within a layout, revisit grows `step_count`, ARM-retry and stale-slice dedup, FAILED/COMPLETE and reporter release | |
| `test_tunnel_protocol_log` | `controller/tests/test_tunnel_protocol_log.cpp` | `TunnelProtocolLog::describe()`: command/status names, every wire struct decoded to its log fields in both directions (ACK, tag upload, START_DETECTION, RAW_CAPTURE, START_COLLECTION/slice, collection status, bearing, operation progress, heartbeats, pulses, header-only commands), header-too-small and wrong-length fallbacks, non-NUL-terminated messages | |
| `zmq_timestamp_test`, `zmq_loss_detection_test` | `airspyhf_zeromq/tests/` | Live SDR timestamp continuity and loss detection | **Hardware**: exit 77 (skipped) without an Airspy HF+ |

## Python (pytest)

There is no top-level pytest configuration; `detector/tests` and
`simulator/tests` carry their own `conftest.py`. Run the three directories
separately.

```bash
.venv/bin/python -m pytest detector/tests  -v
.venv/bin/python -m pytest simulator/tests -v
.venv/bin/python -m pytest analyzer/tests  -v
```

| Directory | Files | Covers |
| --- | --- | --- |
| `detector/tests/` | `test_end_to_end.py`, `test_rate_switch.py`, `test_threshold_null.py`, `test_iq_stream.py`, `test_udp_receiver.py`, `test_collection_control.py`, `test_detector_protocol.py`, `test_pulse_reporting.py`, `test_structured_logging.py` | Full STFT→fold→threshold pipeline on synthetic IQ, dual-rate hypotheses, permutation-null threshold and impulse blanking, gap handling, ARM handling and `SLICE_PROGRESS` send policy, TTDP encode/decode, `.jsonl` schema |
| `simulator/tests/` | `test_simulator.py` | Pulse timing (fixed and rate-switch), ZMQ header encoding, SNR-vs-distance model |
| `analyzer/tests/` | `test_flight_checks.py`, `test_post_flight_analysis.py`, `test_iq_replay.py`, `test_psd_spectrum.py`, `test_signal_analyzer.py`, `test_ipi_analyzer.py` | Log parsers (legacy and `.jsonl` layouts), report generation, offline replay, PSD, live analyzers' DSP |

Per-directory details: [detector/tests/README.md](detector/tests/README.md),
[simulator/tests/README.md](simulator/tests/README.md),
[decimator/tests/README.md](decimator/tests/README.md),
[controller/tests/README.md](controller/tests/README.md),
[shared/tests/README.md](shared/tests/README.md),
[airspyhf_zeromq/tests/README.md](airspyhf_zeromq/tests/README.md).

## Test requirements by change type

- **Wire-format change** (`shared/tagtracker_wireformat/zmq_iq_packet.h`): bump
  `TTWF_ZMQ_IQ_VERSION`; update `shared/tests/test_zmq_iq_packet.c`,
  `decimator/tests/test_main.cpp`, the simulator encoder and
  `simulator/tests/test_simulator.py`; update the table in `shared/README.md`.
- **TTDP change** (`shared/detector_protocol.h`): update
  `detector/detector_protocol.py`, `controller/tests/test_detector_protocol.cpp`
  and `detector/tests/test_detector_protocol.py` in the same commit.
- **Timing / rate thresholds**: cover nominal, drifted and degraded-rate cases
  (decimator tests; `test_iq_stream.py` for gaps).
- **Detector decision logic**: add an end-to-end case in
  `detector/tests/test_end_to_end.py` that fails before the change.

## Running against PX4 SITL

1. Set up PX4 SITL per the [PX4 developer guide](https://docs.px4.io/main/en/dev_setup/getting_started.html).
2. Start the controller with the default connection (`udp://127.0.0.1:14540`):
   ```bash
   ./build/controller/MavlinkTagController2
   ```
3. Without an SDR, add `--simulator [strong|moderate|marginal|below-marginal|silent|competing|power-line]`;
   the controller spawns `iq_simulator.py`, the decimator and detectors when
   detection is started from the GCS. See
   [simulator/README.md](simulator/README.md) and
   [controller/README.md](controller/README.md).
