# Signal Analyzer

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
