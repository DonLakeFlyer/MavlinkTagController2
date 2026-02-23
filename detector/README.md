# VHF Pulse Detector

Python-based pulse detector for Sirtrack wildlife radio collars. Implements K-fold coherent integration with Extreme Value Theory (EVT) thresholding for robust detection under non-Gaussian noise conditions.

## Overview

This detector is designed for crystal-oscillator VHF tags with **no timing uncertainty** (ti_pu=0, ti_pj=0). It receives decimated IQ data via UDP from the `airspyhf_zeromq → decimator` pipeline and performs stateless detection cycles optimized for single-tag tracking scenarios.

**Pipeline:**
```
airspyhf_zeromq_rx  →  ZMQ PUB  →  decimator  →  UDP  →  pulse_detector.py
```

## Algorithm Pipeline

### 1. Data Accumulation (~10 seconds per cycle)

The detector accumulates IQ samples until it has enough data to fold K=5 pulses:

```
samples_needed = STFT_step × (K × PRI_windows + 1) + STFT_overlap
               ≈ 38,400 samples at 3840 Hz
               ≈ 10 seconds
```

Each cycle is **independent**—all state is discarded between cycles for simplicity and robustness against drift.

### 2. STFT + Spectral Weighting (W Matrix)

**Window sizing:** Matched to pulse width
- `n_w = ⌈pulse_width × sample_rate⌉` (e.g., 15 ms → 58 samples)
- 50% overlap (`n_ol = n_w / 2`)
- Rectangular window (matched to on/off keyed pulse shape, matching uavrt_detection's `rectwin`)

**Spectral weighting matrix W** (from uavrt_detection's `weightingmatrix.m`):

Instead of zero-padding the FFT for sub-bin resolution, we use a Toeplitz-based matched filter matrix that provides both interpolation and optimal SNR gain:

1. For each sub-bin shift `zeta` in `[0, 0.5]`:
   - Create frequency-shifted pulse template: `s[n] = exp(2jπ·zeta·n/n_w)`
   - Compute normalised, DC-centred DFT: `Xs = fftshift(fft(s)) / ‖fft(s)‖`
   - Build circulant Toeplitz matrix from `Xs`
2. Stack and reshape (Fortran order) to interleave columns
3. Sort columns by ascending frequency

**Result:** `W` is `(n_w, 2·n_w)` — doubles frequency resolution to half-bin spacing.

**STFT computation with W:**
```
S = fftshift(fft(segment, n=n_w))    # n_w-point FFT per window (no zero-padding)
scores = W^H · S                      # matched-filter at each sub-bin frequency
power = |scores|²                     # detection statistic
```

**Output:** Power spectrogram `(n_freq, n_time)` where:
- `n_freq = 2·n_w` (116 bins for 15 ms pulse at 3840 Hz)
- Frequency axis: DC-centred, from `Wf` vector
- Time axis: STFT window indices at 50% overlap step

### 3. K-Fold Pulse Integration

For each frequency bin, the detector searches all possible first-pulse offsets within one PRI:

```python
search_range = min(PRI_windows, max_valid_start)
pulse_idx = first_offset + [0, N, 2N, 3N, 4N]  # K=5 folds
```

**Folding operation:** Sum power at each candidate pattern:
```
fold_score[f, offset] = Σ(k=0 to K-1) power[f, offset + k×N]
```

This provides **coherent integration gain** proportional to K when the pulse timing is correct, while random noise averages out.

### 4. EVT (Extreme Value Theory) Threshold

Unlike analytical Gamma-distribution thresholds, EVT learns the **actual noise statistics** empirically.

**Monte Carlo calibration (first cycle only):**
1. Generate 100 synthetic complex Gaussian noise trials (unit-variance)
2. Run each through the **full STFT + W matrix pipeline** (captures correlations from window overlap and spectral weighting)
3. Fold and search each trial identically to real data
4. Record maximum score across all frequencies/offsets per trial
5. Fit **Gumbel distribution** (`gumbel_r`) to the max scores — matching MATLAB's `evfit`
6. Compute threshold at desired P<sub>f</sub> from fitted distribution

**Noise estimation** (per frequency bin, matching uavrt_detection `wfmstft.m`):
1. 3-window moving mean along time axis (smooths transients)
2. Median of smoothed power per frequency bin
3. Mask bins where power > 10× median (excludes signal energy and strong interference)
4. Mean of unmasked bins → `noise_power[f]`

**Per-frequency scaling:**
```python
threshold[f] = base_threshold × noise_power[f]
```

**Why EVT?**
- Handles non-Gaussian noise (impulsive interference, power line arcs)
- Empirically adapts to actual noise tail behavior
- More robust than parametric assumptions in harsh RF environments

**Threshold cache:** Regenerated only if data geometry changes (n_freq, n_time).

### 5. Peak Detection & Sidelobe Suppression

**Candidate selection:**
```python
det_bins = where(fold_scores > threshold)
```

**Spectral sidelobe merging:**
- Detections within `max(15, nfft // 4)` bins (~29 bins ≈ 960 Hz at default settings) are merged
- Only the **strongest peak** is retained per cluster
- Prevents reporting multiple detections for a single tag due to FFT sidelobes

**Output limit:** Top 1 detection by SNR (configurable for multi-tag scenarios)

### 6. SNR Calculation

For each detected frequency bin:

```python
signal = fold_score[f]
noise  = K × noise_power[f]
SNR_dB = 10 × log10(signal / noise)
```

The SNR reflects the **K-fold integration gain**—expect 7 dB improvement for K=5 compared to single-pulse detection (10×log10(5) ≈ 7 dB).

## Gap Handling (Conservative Strategy)

The detector monitors timestamp continuity between UDP packets to detect data loss.

### Two-Tier Gap Classification

A single threshold at **2×t_p** (30 ms for default 15 ms pulse width) determines handling:

| Gap Size | Action | Rationale |
|----------|--------|-----------|
| **< 2×t_p** (< 30 ms) | **Zero-fill** | Maintains STFT continuity, prevents phase jumps |
| **≥ 2×t_p** (≥ 30 ms) | **Discard & reset** | Likely missed pulse(s), better to start fresh |

**No "accept as-is" category:** All gaps are handled actively—either compensated via zero-fill or rejected via buffer reset.

### Zero-Fill Implementation

When any gap below the threshold is detected:
1. Calculate missing sample count: `gap_seconds × sample_rate`
2. Insert `np.zeros(N, dtype=complex64)` into buffer
3. Flag segment with `[ZEROFILLED]` in output
4. Continue processing (STFT sees continuous data)

**Rationale:** A 15 ms gap in 10 seconds of data degrades SNR by ~0.006 dB—negligible compared to discarding 10 seconds and losing the detection entirely.

### Large Gap Handling

When a gap ≥30 ms is detected:
1. Discard all buffered samples (`buf_parts.clear()`)
2. Reset segment timestamp
3. Log event with detailed diagnostics
4. Start accumulating fresh data

**Rationale:** Missing >2% of segment data corrupts fold timing. Processing would yield false negatives or spurious detections.

### Gap Logging

**Startup banner:**
```
Gap handling:
  < 30.0 ms: zero-fill missing samples
  ≥ 30.0 ms: discard segment and reset buffer
```

**Runtime logging:**
```
GAP < THRESHOLD: 15.3 ms (< 30.0 ms) - zero-filling 59 samples

*** GAP ≥ RESET THRESHOLD: 125.5 ms (≥30.0 ms) ***
    Discarding 18432 buffered samples and resetting segment
    Expected packet after 266.7 ms, got 392.2 ms
```

**Final statistics:**
```
--- Detection stopped after 42 cycles (420 s) ---
  Detections:        38
  Zero-filled gaps:  3 (< 30.0 ms)
  Reset gaps:        1 (≥ 30.0 ms, segments discarded)
  Total gap events:  4
```

## Configuration Parameters

### Required

| Parameter | Description | Example |
|-----------|-------------|---------|
| `--tip` | Inter-pulse interval (seconds) | `2.0` |

### Optional

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--tp` | `0.015` | Pulse duration (seconds) |
| `--fs` | `3840.0` | Decimated sample rate (Hz) |
| `--port` | `10000` | UDP port for IQ data |
| `--pf` | `1e-4` | False-alarm probability (per cycle) |
| `--center-freq` | `0.0` | Channel center freq (MHz, display only) |

### False Alarm Probability (P<sub>f</sub>)

The `--pf` parameter controls detection sensitivity:

- **1e-4** (default): ~0.5 false alarms/hour at 20s cycles
- **1e-5**: Conservative, 1 FA every 10 hours
- **1e-6**: Very conservative, 1 FA every 100 hours
- **1e-3**: Aggressive, 5 FA/hour (use for very weak signals)

**Trade-off:** Lower P<sub>f</sub> reduces false alarms but increases risk of missing weak tags.

## Usage Examples

### Basic (single 146 MHz tag, 2s interval)
```bash
python3 pulse_detector.py --tp 0.015 --tip 2.0 --center-freq 146.000
```

### Conservative (low false-alarm rate)
```bash
python3 pulse_detector.py --tp 0.015 --tip 2.0 --pf 1e-6 --center-freq 146.000
```

### Custom sample rate and port
```bash
python3 pulse_detector.py --tp 0.020 --tip 3.0 --fs 7680 --port 10002
```

### Full pipeline via run_detector.sh
```bash
cd detector
./run_detector.sh
```

This script automatically starts:
1. `airspyhf_zeromq_rx` at 146.010 MHz (radio hardware)
2. `decimator` with 10 kHz shift (768 kHz → 3840 Hz, removes DC spur)
3. `pulse_detector.py` with configured tag parameters

## Output Format

### Detection Output

**With center frequency:**
```
[   1 18:51:43]  DETECTED  145.999768 MHz  (-231.7 Hz)  SNR 76.1 dB  11 ms  Δt=10.000s
```

**Without center frequency:**
```
[   1 18:51:43]  DETECTED  -231.7 Hz  SNR 76.1 dB  11 ms  Δt=10.000s
```

**No detection:**
```
[   1 18:51:43]  no detection  11 ms
```

**With zero-fill warning:**
```
[   2 18:51:53]  DETECTED  145.999768 MHz  SNR 74.2 dB  12 ms  Δt=10.050s [ZEROFILLED]
```

### Output Fields

| Field | Description |
|-------|-------------|
| `[cycle time]` | Cycle index and UTC timestamp |
| `freq MHz` | Absolute frequency (if `--center-freq` provided) |
| `(offset Hz)` | Frequency offset from channel center |
| `SNR dB` | Signal-to-noise ratio after K-fold integration |
| `proc_ms` | Processing time for this cycle |
| `Δt` | Time since previous detection (cycle-to-cycle) |
| `[ZEROFILLED]` | Flag indicating segment had medium gaps |

### Interpreting Δt (Inter-Detection Time)

**Expected:** `Δt ≈ K × t_ip = 5 × 2.0s = 10.0s`

The detector reports once per cycle after folding K pulses. To verify tag timing:

- **Δt ≈ 10.0s** → Tag is transmitting at 2.0s intervals (5 pulses detected and folded)
- **Δt ≈ 20.0s** → Missed one detection cycle (weak signal, interference, or gap)
- **Δt ≈ 12.0s** → Tag might be at 2.4s intervals (5 × 2.4 = 12s)
- **Δt ≈ 8.0s** → Tag might be at 1.6s intervals (5 × 1.6 = 8s)

Variation of ±0.1s is normal (packet jitter, processing time).

## Comparison to uavrt_detection

| Feature | pulse_detector.py | uavrt_detection (MATLAB) |
|---------|-------------------|--------------------------|
| **Spectral weighting** | W matrix (Toeplitz matched filter) | W matrix (weightingmatrix.m) |
| **Window** | Rectangular (rectwin) | Rectangular (rectwin) |
| **Sub-bin resolution** | W matrix with zetas=[0, 0.5] | W matrix with configurable zetas |
| **Noise estimation** | 10× median masking | 10× median masking (wfmstft.m) |
| **Threshold method** | EVT Gumbel (Monte Carlo through full pipeline) | EVT Gumbel (evfit/evthresh) |
| **Temporal validation** | None (stateless) | Confirmation state machine |
| **State persistence** | Discarded per cycle | Tracked across segments |
| **Gap handling** | Zero-fill + discard | Zero-fill + stale-data reset |
| **Frequency tracking** | None | Adaptive ±100 Hz lock |
| **Timing uncertainty** | Assumes zero (ti_pu=0) | Configurable (ti_pu, ti_pj) |
| **Multi-signal peeling** | Not needed (1 tag/channel) | Iterative peak removal |
| **Complexity** | Low (single file) | High (state machine, posteriori) |

### When to Use Each

**pulse_detector.py:**
- Single-tag tracking
- Crystal-oscillator tags (precise timing)
- Simple deployment scenarios
- Quick field validation
- Lower computational overhead

**uavrt_detection:**
- Multi-tag environments
- Oscillator drift compensation needed
- Production UAV deployments
- Requires high confirmation confidence
- Integration with full UAV-RT stack

## Performance Characteristics

### Computational Cost

At 3840 Hz decimated rate with default parameters:

| Component | Time (per cycle) | Notes |
|-----------|-----------------|-------|
| EVT threshold generation | ~800 ms | First cycle only (cached) |
| STFT + W matrix | ~5 ms | ~1.3k windows (10 s segment), W^H multiplication |
| Pulse folding | ~3 ms | Vectorized NumPy operations |
| Peak detection | ~1 ms | Per-frequency threshold comparison |
| **Total** | **~10 ms** | (800 ms first cycle) |

**Duty cycle:** 10 ms processing / 10 s collection = **0.1% CPU**

### Detection Sensitivity

**Theoretical SNR improvement from K=5 folding:**
- Single pulse: SNR_in
- K=5 fold: SNR_out = SNR_in + 10×log10(5) ≈ **SNR_in + 7 dB**

**Practical detection limits:**
- Strong tags (local): 60-80 dB SNR (easily detected)
- Moderate tags (1-2 km): 20-40 dB SNR (reliable)
- Weak tags (3-5 km): 10-20 dB SNR (marginal, depends on P<sub>f</sub>)
- Very weak (> 5 km): < 10 dB SNR (missed or requires lower P<sub>f</sub>)

### False Alarm Rate

With `--pf 1e-4` and 20-second cycles:

```
FA_rate = (3600 s/hour) / (20 s/cycle) × 1e-4 = 0.018 FA/hour
        ≈ 1 false alarm every 6 hours
```

Actual rate may be higher in non-Gaussian noise environments (power lines, urban RF).

## Troubleshooting

### No Detections

**Check:**
1. Tag is powered and transmitting (verify with handheld radio)
2. Radio is tuned correctly (`run_detector.sh` expects 146.000 MHz tag)
3. Decimator is receiving data (look for `[dec] locked input rate` message)
4. Detector shows startup banner (if not, check Python dependencies)
5. Tag parameters match (`--tp` and `--tip` must be correct)

**Try:**
- Increase `--pf` to `1e-3` for more sensitivity
- Reduce distance to tag
- Check for RF interference

### Too Many False Alarms

**Solutions:**
- Decrease `--pf` to `1e-5` or `1e-6`
- Increase minimum SNR threshold in code (line 160)
- Increase sidelobe merging distance (line 152)
- Move away from power lines / urban RF

### Frequent Large Gaps

**Causes:**
- CPU overload (other processes competing)
- Network congestion (ZMQ → decimator → detector)
- Insufficient UDP receive buffer

**Solutions:**
- Monitor CPU usage (`top`/`htop`)
- Increase UDP receive buffer (already set to 1 MB at line 306)
- Check decimator performance stats
- Reduce system load

### Detector Hangs After Startup

**Likely cause:** No data arriving via UDP

**Check:**
- Decimator is running and bound to correct port
- `--ports` argument to decimator includes detector's `--port`
- Firewall not blocking UDP localhost traffic

## Dependencies

- Python 3.7+
- NumPy (vectorized operations, FFT)
- SciPy (Gumbel distribution fitting, Toeplitz matrix construction)

**Install:**
```bash
pip3 install numpy scipy
```

## Implementation Notes

### Why K=5 (not K=10 as originally)?

The code was simplified from K=10 to K=5 to:
- Reduce cycle time (10s instead of 20s)
- Lower computational cost
- Still achieve 7 dB SNR gain
- Provide faster feedback in field testing

For very weak tags, increasing K back to 10 would improve sensitivity at the cost of longer detection latency.

### Why Stateless?

Unlike uavrt_detection's stateful tracking, this detector discards all state between cycles because:
1. **Simplicity:** No state machine, no priori/posteriori bookkeeping
2. **Robustness:** Immune to state corruption from transient interference
3. **Single-tag optimization:** Stateless is sufficient when tracking one known tag
4. **Predictable timing:** Crystal oscillators have minimal drift over 10-second windows

For drift-prone tags or multi-tag scenarios, use the full uavrt_detection pipeline.

### EVT vs Gamma Distribution

The switch from analytical Gamma to empirical EVT thresholding was motivated by field observations in the African bush:
- Power line arcs create impulsive noise with fat tails
- Lightning static doesn't follow exponential assumptions
- EVT adapts to actual noise without parametric assumptions

In clean RF environments, both methods perform similarly. In harsh environments, EVT is more robust.

## References

- **UAV-RT Detection System:** https://github.com/dynamic-and-active-systems-lab/uavrt_detection
- **Extreme Value Theory:** Coles, S. (2001). *An Introduction to Statistical Modeling of Extreme Values*. Springer.
- **Sirtrack Tags:** https://www.sirtrack.com/

## License

See repository root LICENSE file.

## Authors

Detector implementation: Dynamic and Active Systems Lab contributors
UAV-RT system design: Michael W. Shafer
Integration: Don Lake (Dynamic and Active Systems Lab)
