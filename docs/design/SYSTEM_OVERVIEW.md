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
 Pixhawk ◀─serial──▶│ mavlink-routerd ⇄ MavlinkTagController2 ◀── tunnel ──▶ GCS  │ spawns & supervises all of the above,       │            │
 (TELEM2 921600)    │ WiFi GCS UDP :14550      (via Pixhawk forwarding)           │ ARMs detectors per heading, stores slices,  │            │
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
| Pixhawk ⇄ controller, WiFi GCS | `mavlink-routerd` (`setup/mavlink-router.conf`): UART `/dev/serial0` 921600 ⇄ UDP `127.0.0.1:14540` (controller) and UDP server `:14550` (WiFi GCS) | MAVLink v2 | all MAVLink traffic |
| Controller ⇄ GCS | MAVLink tunnel through the autopilot | `TunnelProtocol.h` (CPM-pinned; exact `TUNNEL_PROTOCOL_VERSION` match required) | commands + pulses + bearings; each detector's 1 Hz TTDP heartbeat (uavrt: `frequency_hz == 0` UDP report) is relayed as `DETECTOR_HEARTBEAT` |

## Control flow of a flight

1. **Boot.** The `mavlink-router` systemd service opens `/dev/serial0`, and the
   `mavlink-tag-controller` service (`setup/start-controller.sh`) starts the controller on
   `udp://127.0.0.1:14540`, which the router forwards to (or directly on
   `serial:///dev/serial0:921600` if the service is not enabled). The controller
   learns the router's address from the first packet received. It heartbeats on the tunnel; TagTracker
   checks the protocol version.
   Every GCS command carries `HeaderInfo_t::request_id`, unique per command and
   repeated byte-for-byte on retry. `TunnelCommandDispatcher` answers a
   repeated id from `RequestCache` (16 entries, 120 s — longer than the
   GCS's slowest retry window, START_COLLECTION's second retry at ~70 s) with the original ACK
   instead of re-executing, and NACKs an id reused for a different command
   or with a different payload.
   Below that, each state machine is also idempotent on its own. A controller
   restart mid-upload is not transparent — the outstanding `TAG`/`END_TAGS`
   retry meets a fresh coordinator and is NACKed `outside START_TAGS/END_TAGS`,
   and the heartbeat drops to `IDLE` — but the re-upload the operator then
   sends is clean.
2. **Tags.** The GCS sends `START_TAGS` (`upload_id`, `tag_count`), one `TAG`
   per tag definition (`tag_index`, frequency, `tp`, `tip`, optional secondary
   `tip`, K, `pf`, thresholds), then `END_TAGS` (`upload_id`, `tag_count`).
   `TagUploadCoordinator` (`Idle` → `Receiving` → `HasTags`, or `Empty` when
   the list was cleared) validates each command against the bracket state and
   fills `TagDatabase`. An accepted `START_TAGS` clears the previous list and
   drops `DetectionCoordinator` to `Idle`, so `START_DETECTION` is NACKed
   until `END_TAGS` completes the new one; a `tag_count` above
   `TagUploadCoordinator::kMaxTagCount` (5) is NACKed before anything is
   cleared. A retransmitted `TAG` or `END_TAGS` after a lost ACK is
   accepted without effect; a `TAG` outside a bracket, one carrying another
   `upload_id`, or a conflicting redefinition of an id is NACKed. `END_TAGS`
   is NACKed `incomplete: missing i,j` until every `tag_index` has arrived, so
   a `TAG` lost in flight cannot silently shorten the list.
3. **START_DETECTION.** `DetectionCoordinator` (`HasTags` → `Starting` →
   `Detecting` → `Stopping` → `HasTags`) admits one start; a retry or a
   restarted GCS sending START while `Starting`, or STOP while `Stopping`, is
   ACKed success because the intent is already being carried out. Raw capture
   is a state of the same machine (`HasTags` → `Capturing` → `HasTags`,
   claimed synchronously before the RAW_CAPTURE ACK), so START and RAW_CAPTURE
   cannot both hold the SDR. IQ recording for antenna-pattern calibration is
   another (`HasTags` → `RecordStarting` → `Recording` → `RecordStopping` →
   `HasTags`, heartbeat `RECORDING` once up): `START_IQ_RECORDING` starts the
   same SDR front end plus one `detector/iq_recorder.py` per tag writing raw
   decimated IQ to `~/Logs/Logs-CalibrationSweep-<UTC>/`, with the clock frozen
   as for detection; it is refused while a collection is still open. A repeat
   START while recording is ACKed only if its frequency and gain match.
   `STOP_IQ_RECORDING` tears it down; no post-flight analysis runs. The
   controller probes the SDR, then rejects the
   command (NACK with message) if two tags would bind the same detector UDP
   port — AirSpy HF is a single decimator channel, so it supports one tag;
   Mini tags must have distinct channelizer channels. Otherwise it creates
   `~/Logs/Logs-Detectors-<UTC>/`, starts `airspyhf_zeromq_rx` tuned to the
   requested radio centre (`radio_center_frequency_hz`, normally the tag) +
   10 kHz, `airspyhf_decimator`, and one `pulse_detector.py` per tag with
   the tag's parameters. Detectors warm up (5 s), send `READY`, and free-run:
   one report per cycle, forwarded to the GCS (HIGH at Info, LOW/no-detection
   at Debug). The detection threshold is derived from each cycle's own
   spectrogram, so there is no warm-up cycle and nothing cached on disk.
4. **START_COLLECTION** (a rotation) → `~/Logs/Logs-Rotation-<UTC>/`. For each
   heading the GCS sends `START_COLLECTION_SLICE`; the controller `ARM`s every
   detector, which opens its `.jsonl` in `heading-NNN/`, captures one K-fold
   segment and sends `SLICE_CAPTURED`, which completes the slice for the GCS.
   The detector analyses the segment while the next heading is flown: one
   K-fold cycle, may lock, measures every buffered slice at every lock
   candidate, and sends `CYCLE_COMPLETE`. The last slice is held until every
   analysis is done. During the rotation only locked (`CONFIRMED`)
   measurements of the live candidate reach the GCS.
5. **FINISH_COLLECTION.** The controller may first ask for one revisit slice,
   then fits the antenna pattern to each candidate, sends `BEARING_RESULT` per
   tag, writes `bearing_result.log` / `bearing_candidates.log`.
6. **Stop.** Processes are torn down; `analyzer/post_flight_analysis.py` runs
   on the session directory and writes `analysis.md`.

Steps 3–5 in detail: [COLLECTION_FLOW.md](COLLECTION_FLOW.md). Inside one
cycle: [DETECTOR_PIPELINE.md](DETECTOR_PIPELINE.md).

### Long-running commands and progress

The ACK for `START_DETECTION`, `STOP_DETECTION`, `START_IQ_RECORDING`,
`STOP_IQ_RECORDING`, `RAW_CAPTURE`, `SAVE_LOGS` and `CLEAN_LOGS` means *accepted*; the work runs on a worker thread. Its
state reaches the GCS as `OPERATION_PROGRESS` frames
(`OperationProgressReporter`, `controller/OperationProgress.cpp`) carrying the
originating command and `request_id`, `RUNNING` / `COMPLETE` / `FAILED`,
`step` of `step_count` (0 = indeterminate) and a short message. A frame is
sent on begin, on every step change and on finish, and the RUNNING frame is
re-sent from the 1 Hz heartbeat thread (the COMPLETE/FAILED frame for three
more ticks) so a lost frame cannot strand the GCS
indicator. Steps: one per pipeline process for start/stop, one per elapsed
second of the 13 s raw capture, one per file for save logs (plus the unmount
on the rPi), one per session directory for clean logs; the post-flight
analysis that follows a stop is reported indeterminate under
`STOP_DETECTION` with `request_id` 0. A collection is one operation under
`START_COLLECTION` spanning the whole rotation, advanced by detector
`READY`/`ARMED`/`SLICE_PROGRESS`/`SLICE_CAPTURED`/`COMPUTE_PROGRESS`/`CYCLE_COMPLETE` and the finalize stages
(`RotationProgress`; see COLLECTION_FLOW.md § Rotation progress); the pipeline
start/stop inside it do not open their own operations.

The reporter is also the concurrency gate: `TunnelCommandDispatcher` NACKs a
second long-running command with `Busy: <operation> in progress` while one is
running (`STOP_DETECTION` is exempt — `DetectionCoordinator` already refuses
it while `Starting`, and it must be able to interrupt detection). Begin,
message changes and finish are logged as `operation_progress command=…
request_id=… state=… step=x/y msg=…`; step-only updates and the 1 Hz re-sends
appear only in the verbose `OPERATION_PROGRESS sent:` frame log. Spoken status texts remain only for completion and failure
(`#Log save complete`, `#Logs deleted`, `#Capture complete`, the hung-process
alerts).

Tunnel commands are handled one at a time, in arrival order, on a dedicated
command thread fed by a bounded queue (32) from the MAVLink receive thread.
Commands that block until done (`START_COLLECTION` waits for detector
`READY`, `FINISH_COLLECTION` for teardown) therefore cannot stall the receive
loop, which also sends the controller's MAVLink `HEARTBEAT`; PX4 reports
"Connection to mission computer lost" after `COM_OBC_LOSS_T` (5 s) without it.
A command arriving behind another is logged as `queued behind N command(s)`;
one arriving at a full queue is dropped unACKed with `Command queue full`
(the GCS retries).

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
| Controller text log | `~/MavlinkTagController.log` (whole run; `.1` … `.20` are the previous runs, rotated by `setup/start-controller.sh`, with the systemd exit status appended by `setup/controller-exited.sh`) and `<session>/MavlinkTagController.log` | controller |
| SDR / decimator stderr | `<session>/airspyhf_zeromq_rx.log`, `airspyhf_decimator.log` (1 Hz `perf` counters: `dropped`, `malformed`, `out_of_order`, `queue_drops`) | those processes |
| Detector text log | `<session>/py_detector_<tag>.log` | detector stdout |
| Detector structured log | `<session>/heading-NNN/detector_<tag>.jsonl` (or session root outside a rotation) | detector (`shared/log_schema.py`) |
| Spectrogram dumps | `heading-NNN/tag<T>_cycle_NNNN_{power.npy,iq.npy,meta.json}` when the GCS sets `dump_spectrogram` (~0.9 MB/cycle/tag at K=5, ~3.7 MB at K=20; scales with K) | detector |
| Bearing CSVs | `<rotation>/bearing_result.log`, `bearing_candidates.log` | controller |
| Post-flight report | `<session>/analysis.md` | `analyzer/post_flight_analysis.py` |

The GCS can fetch any of these over MAVLink FTP (`MavlinkFtpServer`).

### Controller log levels

Every line is `[HH:MM:SS|L]  message  (file:line)` with `L` one of `D`
(normal), `E` (internal error) or `V` (verbose). `D` and `E` are always
written. `V` carries the high-rate lines: unconfirmed pulses, `NO DETECTION`
cycles, detector heartbeats and `PULSE`/`PYTHON_PULSE` tunnel frames; it is
off by default and enabled either by `--verbose` at startup or at runtime by
the GCS `SET_LOG_LEVEL` command (`SetLogLevel_t::level`), which TagTracker
sends from its *Controller verbose logging* setting on every change and on
every (re)connect, since the controller reverts to the default on restart.
Every other tunnel frame in either direction is logged at `D` as
`<NAME> sent:`/`<NAME> received:` with its fields (`TunnelProtocolLog`); the
1 Hz `HEARTBEAT` is summarized once a minute and on status change.

The `HEARTBEAT` carries `cpu_throttle_flags` (`CPU_THROTTLE_FLAG_THERMAL`,
`CPU_THROTTLE_FLAG_UNDERVOLTAGE`), the current state from a background poll of
`vcgencmd get_throttled` every 5 s (`CpuThrottle.h`), each call limited to 2 s
(killed after 3 s). Heat is the soft
temperature limit, or frequency-capped / throttled with no under-voltage
(which sets those bits too). Where `vcgencmd` is missing the flags are 0. A
failed or timed-out poll keeps nonzero flags for two more polls (logged at `E`)
so a transient error does not read as recovery, then reports 0 on the third
consecutive failure. Each change is logged at `E` with the raw firmware bits and the CPU
temperature, and TagTracker shows it on the controller indicator. With
`--simulator`, `--sim-cpu-throttle` replaces the poll with a scripted 0 → flags
→ 0 transition for exercising that UI.

## Time base

Every timestamp on the Pi — controller log lines, telemetry cache samples,
the decimator's packet timestamps (anchored to `CLOCK_REALTIME` once at
start, then advanced by sample count), detector `timestamp_ns`, raw-capture
`capture_utc` — comes from the Pi's system clock. The Pi has no RTC and no
network in the field, so it boots at its last saved time and every battery
swap leaves it behind by the time it was off.

`ClockSync` (owned by `MavlinkSystem`) steps the system clock to the
autopilot's GPS time **at most once per controller run**:

- Only `SYSTEM_TIME` and `GPS_RAW_INT` from the autopilot (its sysid,
  `MAV_COMP_ID_AUTOPILOT1`) count; mavlink-router also forwards the GCS's
  `SYSTEM_TIME`, which carries the laptop clock.
- `GPS_RAW_INT.fix_type` ≥ 3 continuously for 5 s (any lower fix, even
  between two `SYSTEM_TIME` samples, restarts the wait), `time_unix_usec`
  non-zero and not before the build date, and over those 5 s the time must
  advance with the steady clock to within 1 s of where the window started, so
  a jump, a stall or a slow clock restarts it. The autopilot may set its own
  clock just after reporting the fix.
- Offsets of 1 s or less are left alone; so is a clock NTP already disciplines
  (kernel `adjtimex` state not `TIME_ERROR`).
- Never inside a session: while vehicle time is frozen (detection or
  collection) or the heartbeat status is `DETECTING` or `CAPTURE`
  (`ClockStepGate.h`), the step is deferred to the first `SYSTEM_TIME` after
  the session ends. `DETECTING` covers a detection teardown that has not
  finished, including one stuck in `Stopping` because a child process did not
  exit; then the clock is not stepped for the rest of the run. The check and the step hold the
  same mutex as the setters that start a session, so a step either completes
  before the session's processes start or waits for its end.
- After a step the telemetry cache is cleared. Heartbeat send and receive
  times in `Connection` use the steady clock, so a step in either direction
  does not delay the controller's MAVLink `HEARTBEAT`.
- Steps need `CAP_SYS_TIME`, which only the `mavlink-tag-controller` service
  grants (`AmbientCapabilities`); the controller clears its ambient set at
  startup so spawned processes do not inherit it. Without it (desktop, SITL,
  manual runs) or off Linux, sync is disabled and offsets are still reported.

Log key `CLOCK_SYNC`: `enabled`/`disabled: <reason>` at startup;
`waiting: …`, `deferred: …`, `skipped: …`, `stepped …`, `failed: …` once each
as they occur (a `waiting` reason is logged again only after a different one
intervened); and at every session start `session start: GPS offset <s>; <state>`
into the session's log (an `E` line if a synced clock has drifted past 1 s
again). Raw captures record the same text as `clock_sync` in their metadata
JSON. A step and a failure are also sent to the GCS as status text.

## Failure behaviour at each hop

| Anomaly | Where detected | Action |
| --- | --- | --- |
| USB sample loss | `airspyhf_zeromq_rx` | `sequence` skips ahead; logged |
| Malformed / out-of-order / dropped ZMQ packet | decimator | counted in `perf`; stream continues |
| Sample-rate mismatch beyond `--rate-tol-ppm` (packet header or measured rate) | decimator | warning; with `--strict-input-rate` the process exits on the **first** such mismatch |
| UDP gap < 2·tp | detector `iq_stream` | zero-fill, segment flagged |
| UDP gap ≥ 2·tp | detector `iq_stream` | barrier; segment restarts after the hole |
| Detector `FAILED` during a slice | controller | forwarded to the GCS as `COLLECTION_STATUS_FAILED` with the error code (only if it names the slice being captured or one still being analysed); the GCS decides whether to cancel |
| Detector process exits during a collection (crash or unrequested exit) | controller `MonitoredProcess` → `_handleDetectorProcessFailure` | `COLLECTION_STATUS_FAILED` with `ErrorCode::ProcessFailed` and expected/completed detector counts sent to the GCS; if still in `Starting`, the collection is cancelled. Outside a collection the exit is only logged and announced as a status text |
| Detector silent | controller | heartbeats are logged on receipt only; there is no controller-side timeout on `CYCLE_COMPLETE`. During a collection the rotation `OPERATION_PROGRESS` step stops advancing (no `SLICE_PROGRESS` while capturing; an analysis without `COMPUTE_PROGRESS` for > 5 s, or running > 120 s, freezes it), which the GCS treats as a stalled rotation and cancels |
| No candidate clears `confidenceFloor` | `BearingCalculator` | `BEARING_RESULT` with NaN bearing, `confirmed = 0` |
| No 3D fix, implausible, jumping, stalled or slow autopilot time | controller `ClockSync` | no step; the 5 s settle restarts; `CLOCK_SYNC waiting: …` logged once per reason until a different reason intervenes |
| No `GPS_RAW_INT` 30 s after the first valid `SYSTEM_TIME` | controller `ClockSync` | `CLOCK_SYNC waiting: no GPS_RAW_INT received from autopilot`; no step |
| Clock step needed during a session | controller `ClockSync` | deferred until the session ends; logged once |
| Detection teardown left a hung child (coordinator stays `Stopping`) | controller `ClockStepGate` | no clock step for the rest of the run |
| `clock_settime` fails (no `CAP_SYS_TIME`) | controller `ClockSync` | `CLOCK_SYNC failed` (`E`) and a status text to the GCS; not retried this run |
| Clock off by > 1 s again after the sync | controller `ClockSync` | session-start `CLOCK_SYNC` line logged as `E`; no second step |

## Related

- Component READMEs: [controller](../../controller/README.md), [decimator](../../decimator/README.md), [airspyhf_zeromq](../../airspyhf_zeromq/README.md), [detector](../../detector/README.md), [simulator](../../simulator/README.md), [analyzer](../../analyzer/README.md), [shared](../../shared/README.md)
- [TESTING.md](../../TESTING.md)
