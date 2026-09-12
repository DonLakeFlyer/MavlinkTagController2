# VHF Pulse Detector

Python K-fold pulse detector for crystal-oscillator wildlife radio collars.
Consumes decimated IQ over UDP, integrates K pulses at the known interval,
thresholds against an empirically calibrated (EVT) noise model and reports
each cycle to the controller over the TTDP protocol. During a rotation it also
banks lock candidates and measures every heading at each of them for the
bearing fit.

```
airspyhf_zeromq_rx → ZMQ → airspyhf_decimator → UDP :10000, :10001, … → pulse_detector.py → TTDP/UDP :50000 → controller
```

## Install

```bash
./setup_venv.sh          # from the repo root: creates .venv from simulator/requirements.txt (numpy, scipy, pyzmq, matplotlib, pytest)
```

The controller launches the detector with `.venv/bin/python3` when present,
else `python3`.

## Usage

```bash
.venv/bin/python detector/pulse_detector.py --tip 2.0 --tp 0.015 --center-freq 146.000
```

| Flag | Default | Meaning |
| --- | --- | --- |
| `--tip` | *required* | Inter-pulse interval, s |
| `--tp` | `0.015` | Pulse width, s |
| `--tip-secondary` | off | Second PRI for dual-rate collars; enables the multi-hypothesis fold |
| `--k` | `5` | Pulses folded per cycle (the controller passes the tag's K, typically 20) |
| `--fs` | `3840` | Decimated sample rate, Hz |
| `--port` | `10000` | UDP port to receive IQ on |
| `--center-freq` | `0.0` | Channel centre, MHz (display and frequency gate) |
| `--freq` | `0` | Absolute tag frequency, Hz (reported in pulses; with `--center-freq` enables the ±2 kHz search gate) |
| `--pf` | `5e-2` | False-alarm probability per cycle |
| `--detection-margin` | `0.90` | Multiplier on the EVT threshold; lower = more sensitive |
| `--confidence-ratio` | `1.3` | `score/threshold` at or above → HIGH (`confirmed_status = 1`), unless one fold carries > 80 % of the score (dominant-fold gate → LOW) |
| `--lock-score-ratio` | `3.0` | Minimum `score/threshold` for a lock candidate |
| `--warmup-seconds` | `5.0` | IQ discarded before the first cycle |
| `--tag-id` | `0` | Tag id carried in reports |
| `--pulse-port` | `0` | UDP port for TTDP reports (`0` = none) |
| `--control-port` | `0` | Local UDP port for `ARM` commands (`0` = free-running, no collections) |
| `--threshold-cache-dir` | none | Where `*.pythreshold` EVT caches live |
| `--log-dir` | none | Write `detector_<tag>.jsonl` (and per-heading subdirs when armed) here |
| `--dump-spectrogram` | off | Save `tag<T>_cycle_NNNN_{power.npy,iq.npy,meta.json}` per cycle (~0.9 MB at K=5, ~3.7 MB at K=20) |
| `--debug` | off | Per-stage diagnostics |

`pf` presets used by TagTracker: Aggressive `5e-2` (default), Moderate `1e-2`,
Conservative `1e-3`. Lower `pf` = fewer false dots, slightly less range.

Standalone pipeline against real hardware at 146.000 MHz (starts SDR,
decimator and detector; Ctrl-C stops all three):

```bash
detector/run_detector.sh
```

Without hardware use the simulator: `simulator/run_sim_pipeline.sh` or
`MavlinkTagController2 --simulator` (see [simulator/README.md](../simulator/README.md)).

## Inputs / outputs

| | |
| --- | --- |
| IQ in | UDP datagrams of `complex64` at `--fs`; the first sample is a header whose float lanes are bit-cast `uint32` seconds / nanoseconds (`decode_timestamp`), the rest are IQ (decimator format) |
| Control in | TTDP `ARM(heading_deg)` on `--control-port` |
| Reports out | TTDP `READY`, `ARMED`, `PULSE`, `NO_DETECTION`, `CYCLE_COMPLETE`, `FAILED`, `HEARTBEAT` (1 Hz) to `--pulse-port` — layout in [shared/README.md](../shared/README.md#ttdp-detector-protocol) |
| Console | one line per cycle, e.g. `[   7 08:43:10]  DETECTED  146.609080 MHz  (-1920.0 Hz)  SNR 18.4 dB  score_ratio 1.599  noise 5.230e-12  171 ms  [LOW]`; `MEASURED …` lines for locked measurements; `no detection … best=…` otherwise |
| Structured log | `detector_<tag>.jsonl`, entry types in `shared/log_schema.py`; when armed, one file per `heading-NNN/` |
| EVT cache | `<cache-dir>/…-F<bins>-K<K>-Trials100-S2.pythreshold`, keyed by geometry |

## Key files

| File | Role |
| --- | --- |
| `pulse_detector.py` | STFT·W, K-fold, EVT threshold, peak selection, lock-candidate bank, per-slice measurement, PRI refit, reporting |
| `iq_stream.py` | Continuous sample timeline, segment cutting, gap zero-fill / barrier |
| `udp_receiver.py` | Receive thread and bounded ring |
| `collection_control.py` | `ARM` handling and slice bookkeeping |
| `detector_protocol.py` | TTDP encode/decode (mirror of `shared/detector_protocol.h`) |
| `run_detector.sh` | Standalone hardware pipeline |

## Tests

```bash
.venv/bin/python -m pytest detector/tests -v
```

See [detector/tests/README.md](tests/README.md) and [TESTING.md](../TESTING.md).

## Further reading

- [docs/design/DETECTOR_PIPELINE.md](../docs/design/DETECTOR_PIPELINE.md) — one cycle end to end: stream, STFT·W, fold, EVT, peaks, SNR
- [docs/design/COLLECTION_FLOW.md](../docs/design/COLLECTION_FLOW.md) — acquisition → lock → per-slice measurement → bearing
- [docs/design/CONFIDENCE_PIPELINE.md](../docs/design/CONFIDENCE_PIPELINE.md) — `pf` / margin / confidence ratio / dominant-fold gate
- [docs/design/RATE_SWITCH_DETECTOR.md](../docs/design/RATE_SWITCH_DETECTOR.md) — dual-rate collars
- [docs/design/FREQUENCY_RANGE.md](../docs/design/FREQUENCY_RANGE.md) — bandwidth and bin resolution
- [docs/design/PYTHON_VS_UAVRT.md](../docs/design/PYTHON_VS_UAVRT.md) — differences from the MATLAB reference

## Troubleshooting

| Symptom | Cause | Fix |
| --- | --- | --- |
| Process dies at once with `Illegal instruction` (SIGILL, controller logs "Process fail: 4") | aarch64 VM advertises SME; numpy's OpenBLAS picks its `armv9sme` kernel and the guest faults. Verify: `OPENBLAS_VERBOSE=2 .venv/bin/python -c "import numpy"` prints `Core: armv9sme` | `OPENBLAS_CORETYPE=ARMV8` in the environment of every numpy process (`/etc/environment` or `Environment=` in the systemd unit) |
| Hangs after the startup banner | No IQ arriving | Decimator running? Its `--ports` includes this `--port`? Firewall on localhost UDP? |
| No detections | Tag off / wrong frequency / wrong `--tp` `--tip` | Confirm with a handheld receiver; check `--center-freq` and the tune offset; check decimator log shows `locked input rate` |
| Detections on nearly every cycle at scattered offsets, SNR ≈ 15–18 dB | EVT threshold not matching the field noise; these are noise maxima (see the Apr-11 record in `docs/analysis/`) | Lower `--pf`, set `--detection-margin 1.0` (the default 0.90 lowers the threshold); a stable collar clusters within a bin or two of one offset |
| Frequent `GAP ≥` barriers | CPU starvation or UDP buffer loss between decimator and detector | Check `top`; check decimator `perf` counters (`queue_drops`, `dropped`); reduce other load |
| First cycle takes many seconds | EVT cache being generated for a new geometry | Expected once per (K, tp, fs, hypotheses); cached in `--threshold-cache-dir` afterwards |
