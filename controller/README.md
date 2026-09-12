# MavlinkTagController2 (controller)

Companion-computer process that sits between the GCS (TagTracker) and the SDR
pipeline. It receives tag definitions and collection commands over a MAVLink
tunnel, spawns and supervises the SDR publisher, decimator and one Python
detector per tag, receives their pulse reports over UDP, fits the antenna pattern
to produce a bearing per rotation, and relays pulses and bearings back to the GCS.

```
GCS ⇄ MAVLink tunnel ⇄ MavlinkTagController2 ⇄ UDP (TTDP) ⇄ pulse_detector.py × N
                              │ spawns
                              ├── airspyhf_zeromq_rx  (or iq_simulator.py)
                              └── airspyhf_decimator
```

## Build

Built by the default `make`. Needs Boost (system, filesystem), Threads.
MAVLink headers and `TunnelProtocol.h` are fetched by CPM at configure time
(pins in the top-level `CMakeLists.txt`; the tunnel-protocol pin must match
TagTracker's `custom/CMakeLists.txt`).

```bash
make controller           # or: cmake --preset controller && cmake --build --preset controller
```

## Usage

```bash
./build/controller/MavlinkTagController2 [options] [connection_url]
```

| Argument | Default | Description |
| --- | --- | --- |
| `connection_url` | `udp://127.0.0.1:14540` | MAVLink connection, e.g. `serial:///dev/serial0:921600` |
| `--simulator [level\|preset]` | off | Replace the SDR with `simulator/iq_simulator.py`. Level (`strong`, `moderate`, `marginal`, `below-marginal`, `silent`, `competing`) sets the SNR of the configured tag; any other word is an `iq_simulator` preset used only when no tag is configured |
| `--sim-tx-bearing-deg <deg>` | `135` | True bearing of the simulated transmitter from the first vehicle pose |
| `--sim-antenna ra2a\|ra23k` | `ra2a` | Gain pattern applied to the simulated tag (independent of the GCS `antenna_id`) |
| `--sim-pri-ppm <ppm>` | `43` | Collar crystal offset; `0` = perfect crystal |
| `--sim-telemetry-endpoint <uri>` | internal default | TCP endpoint the simulator reads vehicle pose from |
| `--debug-detector` | off | Pass the verbose flag to `pulse_detector.py` |

```bash
./build/controller/MavlinkTagController2                                  # SITL
./build/controller/MavlinkTagController2 serial:///dev/ttyACM0:115200
./build/controller/MavlinkTagController2 --simulator competing            # no SDR
```

Levels set the SNR of the GCS-configured tag and are calibrated against the
K=20 lock ratio; presets define their own tags and apply only when no tag is
configured. Table of both: [simulator/README.md](../simulator/README.md#controller-signal-levels).

## Inputs / outputs

| | |
| --- | --- |
| GCS ⇄ controller | MAVLink tunnel, messages from `TunnelProtocol.h` (`StartDetection`, `StartCollection_t`, `StartCollectionSlice_t`, `FinishCollection_t`, `BearingResult_t`, pulse reports). The heartbeat advertises `TUNNEL_PROTOCOL_VERSION`; TagTracker requires an exact match |
| Detector → controller | TTDP over UDP on `CommandHandler::kPulseUdpPort` (see [shared/README.md](../shared/README.md#ttdp-detector-protocol)) |
| Spawned processes | `airspyhf_zeromq_rx -Z -f <radio_center+0.010> -a 768000 -g off -m on` (centre from `StartDetectionInfo_t::radio_center_frequency_hz`), `airspyhf_decimator --input-rate 768000 --shift-khz 10 --ports 10000,10001`, `pulse_detector.py --tag-id … --k <tag K> …` (one per tag), or `iq_simulator.py` in simulator mode |

### Logs

Everything lives under `~/Logs/`, one directory per session
(`Logs-Detectors-<UTC>`, `Logs-Rotation-<UTC>`, `Logs-RawCapture-<UTC>`).
A rotation directory holds `MavlinkTagController.log`, `airspyhf_decimator.log`,
`airspyhf_zeromq_rx.log` and the detectors' text logs at the root, plus a
`heading-NNN/` subdirectory per slice with each detector's structured
`detector_<tag>.jsonl` (schema `shared/log_schema.py`) and, if the GCS enabled
`dump_spectrogram`, `tag<T>_cycle_NNNN_{power.npy,iq.npy,meta.json}` (~0.9 MB per
cycle per tag at K=5, ~3.7 MB at K=20 — scales with K). A revisit that lands on
an already-flown heading is written to
`heading-NNN-sSS/`. `bearing_result.log` and `bearing_candidates.log` are CSVs of
the pattern fit. When a session stops the controller runs
`analyzer/post_flight_analysis.py` on the directory and writes `analysis.md`.

## Key files

| File | Role |
| --- | --- |
| `main.cpp` | CLI parsing, wiring |
| `CommandHandler.cpp` | Tunnel command dispatch; starts/stops the pipeline; per-tag detector launch; pulse forwarding; `FinishCollection` |
| `CollectionCoordinator.cpp` | ARM/ARMED/CYCLE_COMPLETE bookkeeping per slice and per detector |
| `BearingCalculator.cpp` | Levenberg–Marquardt fit of slice SNRs to the antenna pattern; candidate selection; revisit decision |
| `AntennaPattern.cpp` | RA-2A / RA-23K gain tables and `confidenceFloor` |
| `UDPPulseReceiver.cpp`, `PythonPulseMapper.cpp` | TTDP receive and mapping to tunnel pulse messages |
| `MonitoredProcess.cpp` | Child-process supervision and log redirection |
| `TagDatabase.cpp` | Tags received from the GCS |
| `MavlinkSystem.cpp`, `Connection.cpp`, `UdpConnection.cpp`, `SerialConnection.cpp` | MAVLink link and heartbeat |
| `LogFileManager.cpp` | Session directory creation and `heading-NNN` rotation |
| `MavlinkFtpServer.cpp` | Serves log files to the GCS over MAVLink FTP |
| `Telemetry.cpp`, `TelemetryCache.cpp`, `SimulatorTelemetryPublisher.cpp` | Vehicle pose for slices; pose feed to the simulator |

## Tests

The component build preset builds only `MavlinkTagController2`; build the test
targets first:

```bash
cmake --preset debug && cmake --build --preset debug
ctest --test-dir build -R 'test_(collection_protocol|collection_coordinator|detector_protocol|bearing_calculator|python_pulse_mapper)'
```

See [controller/tests/README.md](tests/README.md) and [TESTING.md](../TESTING.md).

## Further reading

- [docs/design/SYSTEM_OVERVIEW.md](../docs/design/SYSTEM_OVERVIEW.md) — end-to-end data and control flow
- [docs/design/COLLECTION_FLOW.md](../docs/design/COLLECTION_FLOW.md) — acquisition → lock → per-slice measurement → bearing
- [setup/README.md](../setup/README.md) — Raspberry Pi install and auto-start
