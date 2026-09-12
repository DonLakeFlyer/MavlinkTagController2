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
| `test_collection_protocol` | `controller/tests/test_collection_protocol.cpp` | `TunnelProtocol.h` command ids and struct sizes the controller depends on | Guards the CPM pin |
| `test_collection_coordinator` | `controller/tests/test_collection_coordinator.cpp` | ARM/ARMED/CYCLE_COMPLETE state machine, duplicates, cancel/finalize | |
| `test_detector_protocol` | `controller/tests/test_detector_protocol.cpp` | TTDP header/payload layout (`shared/detector_protocol.h`) | Mirror of `detector/tests/test_detector_protocol.py` |
| `test_bearing_calculator` | `controller/tests/test_bearing_calculator.cpp` | Pattern fit, weighted headings and residuals, candidate selection, wraparound, noise tolerance, confirmed/revisit semantics; run for both antenna patterns | |
| `test_python_pulse_mapper` | `controller/tests/test_python_pulse_mapper.cpp` | TTDP pulse → tunnel `PythonPulseInfo_t` mapping | |
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
| `detector/tests/` | `test_end_to_end.py`, `test_rate_switch.py`, `test_threshold_null.py`, `test_iq_stream.py`, `test_udp_receiver.py`, `test_collection_control.py`, `test_detector_protocol.py`, `test_pulse_reporting.py`, `test_structured_logging.py` | Full STFT→fold→threshold pipeline on synthetic IQ, dual-rate hypotheses, permutation-null threshold and impulse blanking, gap handling, ARM handling, TTDP encode/decode, `.jsonl` schema |
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
3. Without an SDR, add `--simulator [strong|moderate|marginal|below-marginal|silent|competing]`;
   the controller spawns `iq_simulator.py`, the decimator and detectors when
   detection is started from the GCS. See
   [simulator/README.md](simulator/README.md) and
   [controller/README.md](controller/README.md).
