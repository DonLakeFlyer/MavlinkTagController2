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

A preset defines its own tag(s). Used by `run_sim_pipeline.sh --preset`, by
`iq_simulator.py --preset`, and by the controller only when **no tag is
configured** from the GCS.

| Preset       | Description                                    |
|-------------|------------------------------------------------|
| `strong`     | Single tag, 25 dB SNR — clear detection        |
| `weak`       | Single tag, 6 dB SNR — marginal detection      |
| `noise-only` | No tags, pure noise — false alarm test         |
| `two-tags`   | Two tags at different freq offsets and rates    |
| `distant`    | Single tag, 3 dB SNR — below typical threshold |
| `dropout`    | Strong tag with 5% random packet drops         |
| `gap`        | Strong tag with 100 ms gaps every 10 s         |

## Controller signal levels

When the controller runs `--simulator <level>` **and a tag is configured**, the
tag's own frequency, `tp`, `tip` (and secondary rate) come from the GCS and the
level only sets its SNR. SNR is specified at the 768 kHz simulator output; the
200× decimator adds ~23 dB of processing gain before the detector. The level
SNRs are set in `controller/main.cpp` and were chosen around the detector's
`--lock-score-ratio` of 3.0 at K=20; the per-dwell permutation-null threshold
(see [DETECTOR_PIPELINE.md](../docs/design/DETECTOR_PIPELINE.md)) moves the
exact `score_ratio` each level produces, so treat the behaviours below as the
intent to verify, not a guarantee.

The simulated transmitter sits at bearing 135° by default for every level
(`--sim-tx-bearing-deg` overrides). With TagTracker's clockwise sweep from 0°
that is the fourth heading, so the first three headings are always measured
retrospectively once a lock exists.

| Level | Simulator SNR | ≈ Detector SNR | Interferer | Expected outcome |
|---|---|---|---|---|
| `strong` (default) | 20 dB | 43 dB | — | Locks on the first heading that sees the tag (0° is 45° off-axis, still tens of dB up); sighted on every heading → **confirmed**, sector 3, bearing ≈ 135°. Bench level: the tag is 50–70 dB over noise, so its spectral sidelobe images are admitted as extra candidates (issue #148) |
| `moderate` | −8 dB | 15 dB | — | Long-range realistic level: sighted on every heading (deepest pattern null still ≈ 20 dB over noise) → **confirmed**, bearing ≈ 135°, with a single candidate and no sidelobe images |
| `marginal` | −27 dB | −4 dB | — | Sighted on one heading only (135°, `score_ratio` ≈ 5 against the lock ratio 3; ±45° ≈ 2, below it); `FINISH_COLLECTION` gets `COLLECTION_STATUS_REVISIT_REQUESTED`, the GCS flies that heading again, then **confirmed** if the revisit sights it, otherwise **unconfirmed**. (−21 dB gave three sightings and a confirm without a revisit) |
| `below-marginal` | −33 dB | −10 dB | — | Never locks (best `score_ratio` ≈ 1.6 at 135°, under the lock ratio 3) but the tag's bin clears the `pf` threshold on the two or three headings nearest 135°. No bearing is fitted from such hits; because they agree in frequency the result is **heard, no bearing** (`bearing_deg` NaN, `n_valid_slices` > 0) |
| `silent` | no tag | — | — | Runs the `noise-only` preset whatever tags are configured. Expect no lock, `mu` ≈ 68 with `refined: false` on every heading, at most a few scattered `pf` hits (≈ 0.4 per rotation), and **nothing heard** (`bearing_deg` NaN, `n_valid_slices` 0). Two noise hits in one bin would wrongly read as *heard*; that is ~1e-3 per rotation |
| `competing` | −18 dB | 5 dB | flat −18 dB tone at tag +1 kHz (`kSimulatorInterfererOffsetHz`, heading-independent) | Interferer takes the provisional lock (candidate 0) on the first heading; the tag is admitted as an alternate near 135°; every buffered heading is measured at both; the finish-time fit must select the tag (pattern-shaped) over the interferer (flat) |

Any other word after `--simulator` is treated as a preset name. `strong` is
both a level (20 dB) and a preset (25 dB); which applies depends on whether a
tag is configured. `--sim-tx-bearing-deg`, `--sim-antenna` and `--sim-pri-ppm`
modify the configured tag under any level (see
[controller/README.md](../controller/README.md#usage)). Lock mechanics
(candidate bank, sightings, revisit) are described in
[COLLECTION_FLOW.md](../docs/design/COLLECTION_FLOW.md).

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

## Controller-integrated mode

The controller binary supports a `--simulator` flag that replaces the SDR
hardware with `iq_simulator.py` inside the managed pipeline. No separate
script is needed — the controller spawns the simulator, decimator, and
detectors automatically when detection is started via the MAVLink tunnel.

```bash
# Build the controller
make controller

# Start with the default level ("strong")
./build/controller/MavlinkTagController2 --simulator

# Level: sets the SNR of the GCS-configured tag (see "Controller signal levels")
./build/controller/MavlinkTagController2 --simulator competing

# Preset: used only when no tag is configured (see "Presets")
./build/controller/MavlinkTagController2 --simulator weak

# Preset plus a connection URL
./build/controller/MavlinkTagController2 --simulator two-tags serial:///dev/ttyACM0:115200
```

In this mode:
- SDR hardware detection is bypassed
- `iq_simulator.py` publishes IQ on ZMQ PUB port 5555
- The decimator uses `--shift-khz 0` (no DC-spur offset)
- `pulse_detector.py` processes run unchanged
- Tag parameters from the MAVLink tag database are mapped to simulator
  `--freq-offset-hz`, `--tp`, `--tip` (and `--tip-secondary`) arguments; the
  level sets `--snr`
- If no tags are configured when detection starts, the selected preset is used
- The Python venv at `$REPO/.venv` is preferred; falls back to system `python3`

## Signal model

- **Noise**: Complex Gaussian white noise at configurable power level
- **Tag signal**: Pulsed CW tone (on/off keyed) at a frequency offset from DC
  - Amplitude derived from SNR: $A = \sigma_n \sqrt{10^{SNR_{dB}/10}}$
  - Pulse envelope: rectangular, active for `tp` seconds every `tip` seconds
- **Path loss**: Free-space inverse-square law
  - $SNR(d) = SNR_{ref} - 20 \log_{10}(d / d_{ref})$
  - 6 dB loss per distance doubling

## Wire format

Each ZMQ message: 40-byte header + interleaved float32 I/Q payload, exactly
as defined in `shared/tagtracker_wireformat/zmq_iq_packet.h` (table in
[shared/README.md](../shared/README.md#zeromq-iq-packet-format)). The
simulator sets `sample_rate = 768000` and `flags = 0`. A wire-format change
must update `simulator/iq_simulator.py` and `simulator/tests/test_simulator.py`
in the same commit.

## Tests

```bash
.venv/bin/python -m pytest simulator/tests -v
```

See [simulator/tests/README.md](tests/README.md) and [TESTING.md](../TESTING.md).
