# System overview

**Scope.** How the processes in this repository fit together on the companion
computer, what flows between them, and where each artifact ends up. Per-stage
detail lives in the documents linked from each section.

## Processes and data path

```
                    ┌───────────────────────────── companion computer (Raspberry Pi) ─────────────────────────────┐
                    │                                                                                             │
 Airspy HF+ ──USB──▶│ airspyhf_zeromq_rx ──ZMQ PUB tcp://127.0.0.1:5555──▶ airspyhf_decimator ──UDP :10000──▶ pulse_detector.py (tag A) │
                    │   -f centre+10 kHz, 768 kHz IQ  40-byte header + float32 IQ    shift +10 kHz, ÷200     │            :10001──▶ pulse_detector.py (tag B) │
                    │                                                                                     │                       │             │
                    │                                                                          TTDP/UDP :50000 ◀───────────┘             │
                    │                                                                                     │                                   │
                    │                                                              ┌──────────────────────▼──────────────────────┐            │
 Pixhawk ◀─serial──▶│ MavlinkTagController2  ◀──── MAVLink tunnel ────▶ GCS       │ spawns & supervises all of the above,       │            │
 (TELEM2 921600)    │                          (via Pixhawk forwarding)           │ ARMs detectors per heading, stores slices,  │            │
                    │                                                              │ fits antenna pattern → BEARING_RESULT       │            │
                    │                                                              └─────────────────────────────────────────────┘            │
                    └─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
```

In simulator mode (`MavlinkTagController2 --simulator`) `iq_simulator.py`
replaces the SDR process on the same ZMQ endpoint and the decimator runs with
`--shift-khz 0`; everything downstream is unchanged.

| Hop | Transport | Contract | Rate |
| --- | --- | --- | --- |
| SDR → decimator | ZeroMQ PUB/SUB | `shared/tagtracker_wireformat/zmq_iq_packet.h` (v1, 40-byte header) | 768 kS/s complex float32 |
| Decimator → detectors | UDP, one port per detector | `complex64` array; first element is a header with `uint32` seconds / nanoseconds bit-cast into its float lanes, rest are IQ; short frame before a hole | 3840 S/s |
| Detector → controller | UDP `CommandHandler::kPulseUdpPort` (50000) | TTDP, `shared/detector_protocol.h` | per cycle + 1 Hz heartbeat |
| Controller → detector | UDP `--control-port` per detector | TTDP `ARM` | per slice |
| Controller ⇄ GCS | MAVLink tunnel through the autopilot | `TunnelProtocol.h` (CPM-pinned; exact `TUNNEL_PROTOCOL_VERSION` match required) | commands + pulses + bearings |

## Control flow of a flight

1. **Boot.** `setup/crontab-start-controller.sh` starts the controller on
   `serial:///dev/serial0:921600`. It heartbeats on the tunnel; TagTracker
   checks the protocol version.
2. **Tags.** The GCS sends tag definitions (frequency, `tp`, `tip`, optional
   secondary `tip`, K, `pf`, thresholds) → `TagDatabase`.
3. **START_DETECTION.** The controller creates
   `~/Logs/Logs-Detectors-<UTC>/`, starts `airspyhf_zeromq_rx` tuned to the
   requested radio centre (`radio_center_frequency_hz`, normally the tag) +
   10 kHz, `airspyhf_decimator`, and one `pulse_detector.py` per tag with
   the tag's parameters. Detectors warm up (5 s), send `READY`, and free-run:
   one report per cycle, forwarded to the GCS (HIGH at Info, LOW/no-detection
   at Debug). The EVT threshold is loaded or generated on the first cycle, so
   that cycle is slower.
4. **START_COLLECTION** (a rotation) → `~/Logs/Logs-Rotation-<UTC>/`. For each
   heading the GCS sends `START_COLLECTION_SLICE`; the controller `ARM`s every
   detector, which reopens its `.jsonl` in `heading-NNN/`, runs one K-fold
   cycle, may lock, measures every buffered slice at every lock candidate, and
   sends `CYCLE_COMPLETE`. During the rotation only locked (`CONFIRMED`)
   measurements of the live candidate reach the GCS.
5. **FINISH_COLLECTION.** The controller may first ask for one revisit slice,
   then fits the antenna pattern to each candidate, sends `BEARING_RESULT` per
   tag, writes `bearing_result.log` / `bearing_candidates.log`.
6. **Stop.** Processes are torn down; `analyzer/post_flight_analysis.py` runs
   on the session directory and writes `analysis.md`.

Steps 3–5 in detail: [COLLECTION_FLOW.md](COLLECTION_FLOW.md). Inside one
cycle: [DETECTOR_PIPELINE.md](DETECTOR_PIPELINE.md).

## Frequency plan

The HF+ has a DC spur. The radio is tuned 10 kHz **above** the tag, so the tag
sits at −10 kHz in the raw baseband; the decimator mixes by **+10 kHz**
(`FrequencyShifter`, `exp(+j2πft)`) before filtering, bringing the tag to DC of
the 3840 Hz channel and pushing the spur to +10 kHz, outside the ±1920 Hz
passband.
Detector bins are ~33 Hz wide. The simulator has no spur, hence
`--shift-khz 0`. Details: [FREQUENCY_RANGE.md](FREQUENCY_RANGE.md).

## Where things are logged

| Artifact | Location | Written by |
| --- | --- | --- |
| Controller text log | `~/MavlinkTagController.log` (boot script) and `<session>/MavlinkTagController.log` | controller |
| SDR / decimator stderr | `<session>/airspyhf_zeromq_rx.log`, `airspyhf_decimator.log` (1 Hz `perf` counters: `dropped`, `malformed`, `out_of_order`, `queue_drops`) | those processes |
| Detector text log | `<session>/py_detector_<tag>.log` | detector stdout |
| Detector structured log | `<session>/heading-NNN/detector_<tag>.jsonl` (or session root outside a rotation) | detector (`shared/log_schema.py`) |
| Spectrogram dumps | `heading-NNN/tag<T>_cycle_NNNN_{power.npy,iq.npy,meta.json}` when the GCS sets `dump_spectrogram` (~0.9 MB/cycle/tag at K=5, ~3.7 MB at K=20; scales with K) | detector |
| Bearing CSVs | `<rotation>/bearing_result.log`, `bearing_candidates.log` | controller |
| Post-flight report | `<session>/analysis.md` | `analyzer/post_flight_analysis.py` |
| EVT threshold cache | `~/*.pythreshold` (`--threshold-cache-dir` = home) | detector |

The GCS can fetch any of these over MAVLink FTP (`MavlinkFtpServer`).

## Failure behaviour at each hop

| Anomaly | Where detected | Action |
| --- | --- | --- |
| USB sample loss | `airspyhf_zeromq_rx` | `sequence` skips ahead; logged |
| Malformed / out-of-order / dropped ZMQ packet | decimator | counted in `perf`; stream continues |
| Sample-rate mismatch beyond `--rate-tol-ppm` (packet header or measured rate) | decimator | warning; with `--strict-input-rate` the process exits on the **first** such mismatch |
| UDP gap < 2·tp | detector `iq_stream` | zero-fill, segment flagged |
| UDP gap ≥ 2·tp | detector `iq_stream` | barrier; segment restarts after the hole |
| Detector `FAILED` during a slice | controller | forwarded to the GCS as `COLLECTION_STATUS_FAILED` with the error code (only if it names the current slice); the GCS decides whether to cancel |
| Detector process exits during a collection (crash or unrequested exit) | controller `MonitoredProcess` → `_handleDetectorProcessFailure` | `COLLECTION_STATUS_FAILED` with `ErrorCode::ProcessFailed` and expected/completed detector counts sent to the GCS; if still in `Starting`, the collection is cancelled. Outside a collection the exit is only logged and announced as a status text |
| Detector silent | controller | heartbeats are logged on receipt only; there is no controller-side timeout on `CYCLE_COMPLETE`, so a stalled detector holds the slice open until the GCS times out or cancels |
| No candidate clears `confidenceFloor` | `BearingCalculator` | `BEARING_RESULT` with NaN bearing, `confirmed = 0` |

## Related

- Component READMEs: [controller](../../controller/README.md), [decimator](../../decimator/README.md), [airspyhf_zeromq](../../airspyhf_zeromq/README.md), [detector](../../detector/README.md), [simulator](../../simulator/README.md), [analyzer](../../analyzer/README.md), [shared](../../shared/README.md)
- [TESTING.md](../../TESTING.md)
