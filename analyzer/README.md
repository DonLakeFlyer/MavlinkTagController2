# Signal Analyzer

Tools in this folder:

| Tool | Input | Purpose |
|------|-------|---------|
| `signal_analyzer.py` (`run_analyzer.sh`) | live SDR | Measure pulse width and repetition rate of a strong signal |
| `ipi_analyzer.py` (`run_ipi_capture.sh`) | live SDR | Inter-pulse-interval measurement |
| `flight_checks.py` | recorded logs | Run the FLIGHT_DATA_ANALYSIS.md checks against flight logs |
| `iq_replay.py` | raw IQ capture | Offline detector replay + fixed-offset amplitude (Check 6) |

Detect a strong pulsed signal at a specific frequency and measure its **pulse width** (ms) and **repetition rate** (Hz / interval in seconds).

## Pipeline

Uses the same Airspy HF+ SDR pipeline as the pulse detector:

```
airspyhf_zeromq_rx  →  [ZMQ]  →  decimator  →  [UDP]  →  signal_analyzer.py
```

`run_analyzer.sh` starts all three processes automatically.

## Algorithm

1. Accumulate IQ samples for the analysis window (default 10 s)
2. Find the strongest frequency via full-segment FFT (excluding DC spike)
3. Mix to baseband at that frequency
4. Compute instantaneous power (`I² + Q²`), then lowpass filter to get a smooth envelope
5. Threshold the envelope to find on/off transitions
6. Measure pulse widths and start-to-start intervals from transitions

## Usage

```bash
# Basic — specify the expected signal frequency:
./run_analyzer.sh --expected-freq 148.515

# With a longer analysis window:
./run_analyzer.sh --expected-freq 148.515 --duration 15

# Override detection threshold (dB above noise):
./run_analyzer.sh --expected-freq 148.515 --threshold 12
```

Ctrl-C stops all three pipeline processes.

## DC Spike Avoidance

The radio is automatically tuned 10 kHz above `--expected-freq` so the signal of interest lands away from the SDR's DC spike. The decimator's `--shift-khz 10` moves the radio's DC to the edge of the decimated band.

## Options

| Flag | Default | Description |
|------|---------|-------------|
| `--expected-freq` | *(required)* | Expected signal frequency in MHz |
| `--duration` | 10 | Analysis window in seconds |
| `--threshold` | 10 | Detection threshold (dB above noise floor) |
| `--freq-offset` | auto | Baseband frequency offset in Hz (auto-detects strongest peak) |
| `--port` | 10001 | UDP port for decimated IQ |
| `--fs` | 3840 | Decimated sample rate in Hz |

## Prerequisites

- Built `airspyhf_zeromq_rx` and `airspyhf_decimator` (`cmake --build build`)
- Python venv with `numpy` (`./setup_venv.sh`)
- Airspy HF+ SDR connected

## Output

```
[ana] === Signal Analyzer (Airspy HF+) ===
[ana]   Expected freq   148.515000 MHz
[ana]   Sample rate     3840.0 Hz
[ana]   Analysis window 10.0 s
[ana] [   1 17:02:14]  148.515012 MHz (+12.3 Hz)  5 pulses  width=14.8 ms  [14.6, 15.1, 14.8, 15.0, 14.6]  interval=2.001 s  rate=0.500 Hz  (10.0s window, 25 ms)
```

On exit, prints aggregate statistics:

```
--- Analysis complete: 6 cycles ---
  Pulse width:     mean=14.82 ms  median=14.84 ms  std=0.21 ms  (n=30)
  Repetition:      mean=2.0008 s  (0.4998 Hz)  std=0.0012 s  (n=24)
```

---

# Offline Flight-Data Tools

Both tools are offline: they need no SDR or running pipeline. They implement
the checks defined in `FLIGHT_DATA_ANALYSIS.md` (results from the April 2026
PDC testing data are recorded in that document).

## flight_checks.py

Walks a directory tree of recorded controller/detector logs
(`MavlinkTagController.log`, `py_detector_*.log`, `detector_*.config`) and
produces the evidence for Checks 0–5:

- **Check 0/3** — detections and SNR per commanded heading (heading_XXX dwell
  directories)
- **Check 1** — reported-SNR histograms split by K, against the predicted
  noise floor
- **Check 2** — detection frequency offsets vs the entered tag frequency
  (noise vs multipath discriminator)
- **Check 4** — detection-cycle period only; true PRI is *not* measurable
  from these logs (1 s timestamps, per-cycle reports) — use `ipi_analyzer.py`
- **Check 5** — `noise_psd` per session/heading

```bash
python3 analyzer/flight_checks.py "/path/to/PDC Testing"
```

Plain-Python (no venv needed). Controller logs contain ANSI colour codes;
the parser strips them.

## iq_replay.py

Check 6: replays a raw airspy-hf capture (`complex_float32` at 768 kHz)
through the detector's own processing offline — mixes the tag band to
baseband, decimates 200× to 3840 Hz, runs the STFT + K-fold max-search
(the "current metric"), then applies the fixed-offset
`amplitude_at_known_pulse` estimator from `DETECTOR_AMPLITUDE_ANALYSIS.md`
at the locked (bin, phase).

```bash
# Tune convention tune = tag + 10 kHz assumed if --tune-freq omitted:
.venv/bin/python analyzer/iq_replay.py capture.dat --tag-freq 146.170 --tip 2.0 --k 4

# Explicit tune frequency (from the capture's .json sidecar):
.venv/bin/python analyzer/iq_replay.py capture.dat --tag-freq 146.664 \
    --tune-freq 146.674 --tip 1.333 --k 9
```

| Flag | Default | Description |
|------|---------|-------------|
| `--tag-freq` | *(required)* | Entered tag frequency in MHz |
| `--tune-freq` | tag + 0.010 | SDR tune frequency in MHz |
| `--raw-fs` | 768000 | Raw capture sample rate in Hz |
| `--tip` | 1.333 | Inter-pulse interval in s (try both collar rates) |
| `--tp` | 0.015 | Pulse width in s |
| `--k` | 5 | Fold count (clamped to what the capture length supports) |
| `--top` | 5 | Number of top candidates to print |
| `--lock-offset` | *(self-lock)* | Fold-offset index from an independent lock; without it the phase is re-maximized on the data being measured, which positively biases the amplitude |
| `--lock-bin` | *(entered freq)* | Frequency bin from an independent lock (use with `--lock-offset`) |

For an unbiased fixed-offset validation, obtain the lock (bin and fold
offset) from held-out data of the **same continuous stream** — e.g. lock on
the first portion, measure the rest. A fold-offset index from an
independently started capture is NOT reusable: the index is relative to that
file's STFT origin and separate captures share no sample timebase, so the
result samples an arbitrary pulse phase (unless converted through
synchronized sample timestamps and the PRI). Fold offset 0 is anchored to
the start of each file, so a lock carried across files split from one
recording must be adjusted by the split point (in STFT windows).

Requires the repo venv (`./setup_venv.sh`) for numpy/scipy, and imports fold
machinery from `detector/pulse_detector.py`.

Output per candidate: frequency offset, current-metric SNR, absolute
fixed-offset signal power `p` (sum over folds minus K×noise, unclamped),
the per-bin noise estimate `N` reported separately (so bearing fits can use
`p` directly without per-heading noise normalization), the `p/N` ratio as a
diagnostic, and per-fold amplitudes — a
real pulse train shows energy spread across folds; a single hot fold is a
transient.
