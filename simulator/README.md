# IQ Signal Simulator

Drop-in replacement for `airspyhf_zeromq_rx` that generates synthetic IQ data
with configurable tag pulses, noise, and anomalies.  Publishes over ZeroMQ PUB
using the exact tagtracker wire format, so the downstream decimator and pulse
detector work unmodified.

## Pipeline

```
iq_simulator.py → [ZMQ PUB] → decimator → [UDP] → pulse_detector.py
```

## Setup

```bash
# From the repo root — create venv and install dependencies
python3 -m venv .venv
source .venv/bin/activate
pip install -r simulator/requirements.txt
```

## Quick start

```bash
# Run the full pipeline with a strong tag signal
./simulator/run_sim_pipeline.sh --preset strong

# Run with a weak (marginal) signal
./simulator/run_sim_pipeline.sh --preset weak

# Pure noise — verify no false detections
./simulator/run_sim_pipeline.sh --preset noise-only --duration 60
```

## Presets

| Preset       | Description                                    |
|-------------|------------------------------------------------|
| `strong`     | Single tag, 25 dB SNR — clear detection        |
| `weak`       | Single tag, 6 dB SNR — marginal detection      |
| `noise-only` | No tags, pure noise — false alarm test         |
| `two-tags`   | Two tags at different freq offsets and rates    |
| `distant`    | Single tag, 3 dB SNR — below typical threshold |
| `dropout`    | Strong tag with 5% random packet drops         |
| `gap`        | Strong tag with 100 ms gaps every 10 s         |

## Simulator options

```
--preset NAME           Load a named scenario (see table above)
--sample-rate HZ        IQ sample rate (default: 768000)
--zmq-port PORT         ZMQ PUB port (default: 5555)
--samples-per-packet N  Complex samples per packet (default: 4096)
--noise-power-dbfs DB   Noise floor in dBFS (default: -40)

Tag parameters (repeat for multiple tags):
--freq-offset-hz HZ    Frequency offset from center
--snr DB               Signal-to-noise ratio in dB
--tp SEC               Pulse width in seconds
--tip SEC              Inter-pulse interval in seconds
--phase-offset SEC     Pulse phase offset in seconds

Distance model:
--distance-m METERS    Transmitter distance (free-space path loss)
--ref-distance-m M     Reference distance for --snr (default: 100)

Simulation control:
--duration SEC         Run for N seconds then stop (0 = indefinite)
--no-realtime          Send packets as fast as possible
--drop-probability P   Random packet drop probability (0..1)
--gap-seconds SEC      Duration of injected gaps
--gap-interval SEC     Inject a gap every N seconds
--seed INT             RNG seed for reproducible runs
```

## Examples

### Custom tag parameters
```bash
# Single tag: 12 dB SNR, 20 ms pulses, 1.5 s interval
./simulator/run_sim_pipeline.sh --freq-offset-hz 0 --snr 12 --tp 0.020 --tip 1.5

# Two tags at different frequencies
./simulator/run_sim_pipeline.sh \
    --freq-offset-hz 0 --snr 20 --tp 0.015 --tip 2.0 \
    --freq-offset-hz 800 --snr 12 --tp 0.020 --tip 3.0
```

### Distance simulation
```bash
# Tag at 500 m (reference: 30 dB SNR at 100 m)
# Free-space loss: -14 dB → effective SNR ≈ 16 dB
./simulator/run_sim_pipeline.sh --freq-offset-hz 0 --snr 30 --distance-m 500

# Tag at 2 km — probably below detection threshold
./simulator/run_sim_pipeline.sh --freq-offset-hz 0 --snr 30 --distance-m 2000
```

### Robustness testing
```bash
# 5% random packet drops
./simulator/run_sim_pipeline.sh --preset dropout

# Periodic gaps (tests detector gap handling)
./simulator/run_sim_pipeline.sh --preset gap

# Reproducible run with fixed seed
./simulator/run_sim_pipeline.sh --preset strong --seed 42 --duration 30
```

### Standalone simulator (without the full pipeline)
```bash
source .venv/bin/activate
python simulator/iq_simulator.py --preset strong -P 5555
```

## Signal model

- **Noise**: Complex Gaussian white noise at configurable power level
- **Tag signal**: Pulsed CW tone (on/off keyed) at a frequency offset from DC
  - Amplitude derived from SNR: $A = \sigma_n \sqrt{10^{SNR_{dB}/10}}$
  - Pulse envelope: rectangular, active for `tp` seconds every `tip` seconds
- **Path loss**: Free-space inverse-square law
  - $SNR(d) = SNR_{ref} - 20 \log_{10}(d / d_{ref})$
  - 6 dB loss per distance doubling

## Wire format

Each ZMQ message: 40-byte header + interleaved float32 I/Q payload.
Matches `shared/tagtracker_wireformat/zmq_iq_packet.h` exactly.

| Field          | Type     | Value                    |
|---------------|----------|--------------------------|
| magic          | uint32   | 0x5a514941               |
| version        | uint16   | 1                        |
| header_size    | uint16   | 40                       |
| sequence       | uint64   | Monotonic counter        |
| timestamp_us   | uint64   | CLOCK_MONOTONIC µs       |
| sample_rate    | uint32   | 768000                   |
| sample_count   | uint32   | samples_per_packet       |
| payload_bytes  | uint32   | sample_count × 8         |
| flags          | uint32   | 0                        |
