# Python vs uavrt_detection Detector Comparison

Source-level analysis of the Python pulse detector (`detector/pulse_detector.py`) and the MATLAB/C++ uavrt_detection detector (`uavrt_detection/uavrt_detection.m` → MATLAB Coder → C++).

## Architecture

| Aspect | Python | uavrt_detection (MATLAB/C++) |
|--------|--------|------------------------------|
| Language | Python 3 (NumPy/SciPy) | MATLAB → MATLAB Coder → C++ |
| Process model | One process per tag | One process per tag per rate |
| Dual-rate tags | Single process, multi-hypothesis `--tip-secondary` | Two separate processes (tag ID and ID+1) |
| State machine | Stateless — no cross-cycle memory | D (Discovery) → C (Confirmation) → T (Tracking) |
| Input | UDP: 8-byte timestamp + 1024 complex64 IQ samples | Same UDP format |
| Input sample rate | `--fs` (HF: 3840 Hz, Mini: 3750 Hz) | `Config.Fs` from `.config` file (same rates) |
| Output | 12 little-endian doubles (96 bytes) to UDP 50000 | Same 12-double format to UDP 50000 |
| Configuration | CLI arguments | `.config` file written by `TagDatabase::_writeDetectorConfig()` |
| Segment overlap | None — segments are non-overlapping | Overlaps by `2*(K*M+J)` windows |
| IQ recording | None | Writes raw IQ to binary file per segment |

## STFT

Both use effectively identical STFT computation:

| Parameter | Python | uavrt_detection |
|-----------|--------|-----------------|
| Window function | Rectangular (`np.ones`) | Rectangular (`rectwin`) |
| Window length | `ceil(tp * Fs)` | `ceil(t_p * Fs)` |
| Overlap factor | 50% (`n_w // 2`) | 50% (`floor(0.5 * n_w)`) |
| FFT length | `n_w` (no zero-padding) | `n_w` (same) |
| Centering | `fftshift` after FFT | MATLAB `stft()` centered mode |
| Output | Power (`|W^H · S|²`) as float32 | Complex `S` (power computed downstream) |

**Difference:** Python computes `|W^H · S|²` immediately in `compute_stft_power()`, outputting power. MATLAB keeps the complex STFT and applies `|W^H · S|²` later in `incohsumtoeplitz`.

## Weighting Matrix (W)

Algorithmically identical:

| Step | Python (`build_weighting_matrix`) | MATLAB (`weightingmatrix.m`) |
|------|-----------------------------------|------------------------------|
| Zetas | `[0.0, 0.5]` | `[0, 0.5]` |
| Template | `exp(2jπ·zeta·n/n_w) * window` | `exp(j·2π·zeta·n/nw) .* x_of_n` |
| Normalisation | `Xs / ‖Xs‖` (L2) | `currDFT / norm(currDFT)` (L2) |
| Toeplitz | `scipy.linalg.toeplitz` | MATLAB `toeplitz` |
| Interleave | `vstack → reshape(n_w, n_zetas*n_w, order='F')` | `reshape(stacked, nw, numZetas*nw)` (column-major) |
| Sorting | `argsort(Wf, kind='stable')` ascending | `sort(Wf)` ascending |
| Output | `(n_w, 2*n_w)` — doubles freq bins | `(nw, numZetas*nw)` — same |

The W matrix doubles the effective frequency resolution via sub-bin shifting (zeta=0.5), providing half-bin resolution without zero-padding.

## Fold / Integration

| Aspect | Python | uavrt_detection |
|--------|--------|-----------------|
| Method | Direct index gather + summation | Sparse Toeplitz matrix multiply via `incohsumtoeplitz` |
| Score | `Σ power[:, pulse_indices]` per freq per offset | `abs(W' * S)² * IR * Wq` then max across Wq columns |
| PRI uncertainty (M) | **Not supported** (`tipu=0`) | `M = ceil(n_ipu / n_ws)` — Wq explores N±M offsets |
| PRI jitter (J) | **Not supported** (`tipj=0`) | `J = ceil(n_ipj / n_ws)` — additional jitter expansion |
| Fractional PRI | Cumulative rounding: `offsets[k] = round(Σ spacings)` | `N = floor(n_ip / n_ws)` integer-only (M/J compensate) |
| Search range | One full PRI period | `N+M+J` windows (discovery), narrower in tracking |
| Freq blinder (Fb) | None — all freq bins searched | Diagonal mask excludes bands, supports informed search |
| Time blinder (Tb) | None | Diagonal mask constrains time search in tracking mode |
| Repetition rejector (IR) | Not implemented | Identity (disabled), but code exists for harmonics `[2,3,5,10]` |

**Key takeaway:** Python uses simple direct indexing with no uncertainty margin. uavrt_detection builds a full Wq sparse matrix exploring PRI deviations. For crystal-oscillator tags (`tipu=tipj=0`), both converge to the same result. For drifty oscillators, uavrt_detection is more robust.

## Noise Estimation

Nearly identical algorithm:

| Step | Python | uavrt_detection (`wfmstft.updatepsd`) |
|------|--------|----------------------------------------|
| Smoothing | 3-window moving mean | `movmean(magSqrd, 3, 2)` |
| Outlier detection | `power > 10 × median` → mask as NaN | `magSqrd > 10 × medMovMeanMagSqrdMat` → NaN |
| Central tendency | `nanmean` of unmasked power per freq bin | `mean(magSqrd, 2, 'omitnan')` |
| PSD scaling | Deferred: `noise_power / (Fs * n_w)` at output | Inline: `dt²/T × mean(...)` |
| Signal exclusion | None | Additional mask around detected-pulse regions |

**Difference:** uavrt_detection builds a signal-exclusion mask in `findpulse` that removes detected pulse regions from the noise estimate. Python does not — strong pulses may slightly bias noise upward.

## EVT Threshold

| Aspect | Python | uavrt_detection (`threshold.m`) |
|--------|--------|----------------------------------|
| Monte Carlo trials | 100 | 100 |
| Noise generation | Unit-variance complex Gaussian | `wgn(nSamps, trials, P, 'linear', 'complex')` at 1W/bin |
| Pipeline | Full STFT → W^H → power → fold → max per trial | Full STFT → `abs(W' * S)² * IR * Wq` → max per trial |
| Distribution | `gumbel_r.fit(max_scores)` → `gumbel_r.ppf(1-pf)` | `evfit(-scores)` → `fzero(1 - exp(-exp(...)) - pf)` |
| Per-bin scaling | `threshold[f] = base_threshold × noise_power[f]` | `interp1(powGrid, threshGrid, freqBinPow)` |
| Cache key | `N-Nb-H-K-Trials` | `N-M-J-K-Trials` |
| Recompute trigger | `n_freq` or `n_time` change | `N`, `M`, `J`, or `K` change |
| Detection margin | `base_threshold *= 0.90` (configurable) | None — raw EVT threshold used |

Both fit Gumbel distributions to Monte Carlo max-scores. Python normalises each trial by its own median noise, then multiplies by per-bin noise at runtime. uavrt_detection generates at a known power and interpolates linearly.

The Python detector applies a `detection_margin` (default 0.90) multiplier to lower the threshold by 10%, increasing sensitivity at the cost of more marginal detections. uavrt_detection has no equivalent.

Python includes the multi-hypothesis bank in EVT calibration when `--tip-secondary` is active, correctly accounting for the expanded search space.

## Detection Decision

| Aspect | Python | uavrt_detection |
|--------|--------|-----------------|
| Score test | `best_scores > threshold` | `peak_masked_scores >= thresh` |
| Sidelobe suppression | Proximity merge: `max(15, nfft//4)` bins | Full peeling: slope analysis, time-correlation, sideband masking, iterative removal |
| Max detections per cycle | 1 (top score) | All peaks found by peeling, ranked by `selectpeakindex` |
| Detection status values | 0=sub, 1=super, 2=confirmed, 3=no-detection | `det_dec` (bool), `con_dec` (bool) |
| Detection margin | 0.90× applied to threshold (configurable) | None |
| No-detection reports | Sent with `detection_status=3` + noise PSD | **Not sent** — silence when nothing found |

**Key difference:** uavrt_detection's peeling algorithm can resolve multiple overlapping frequencies in a single cycle. Python reports only the single best detection and merges nearby peaks.

## Confirmation Logic

| Aspect | Python | uavrt_detection (`confirmpulses.m`) |
|--------|--------|-------------------------------------|
| Cross-segment state | None — fully stateless | Carries priori (`ps_pre`) across segments |
| Time alignment check | Not performed | `t_0 ∈ [tref + k(tip − tipu) − tipj, tref + k(tip + tipu) + tipj]` |
| Frequency alignment | Not performed | `fp ∈ [prev_fp − 100, prev_fp + 100]` Hz |
| SNR check | Not performed | All K pulses must have `SNR ≠ −Inf` |
| Confirmed definition | `score_ratio ≥ confidence_ratio` (single-cycle metric) | `confirmpulses` returns true for all K pulses across segments |

**Critical difference:** Python's "confirmed" is a **same-cycle confidence metric** — the detection score significantly exceeds the threshold. uavrt_detection's confirmation requires **inter-segment temporal and spectral alignment** — true physical consistency of the pulse train over time. uavrt_detection is much more robust against false confirmations but requires state persistence.

## State Machine (uavrt_detection only)

uavrt_detection has a D→C→T state machine:

1. **Discovery (D):** Full frequency/time search. All bins, all offsets via Wq.
2. **Confirmation (C):** Narrow frequency search around detected pulse. `confirmpulses` checks temporal/spectral alignment with prior.
3. **Tracking (T):** Tight frequency and time windows. Lowest compute. Reverts to C on miss.

Python has no equivalent — every cycle is a full discovery search. This means Python does more work per cycle but never loses a tag due to failed state transitions.

## Rate-Switch Handling

| Aspect | Python | uavrt_detection |
|--------|--------|-----------------|
| Approach | Multi-hypothesis fold bank within single process | Two separate processes, one per rate |
| Hypotheses (K=5) | 8: pure-A, pure-B, 3× A→B, 3× B→A | 1 per process (always single-rate) |
| Switch detection | `group_ind` encodes winning hypothesis | Not available — controller must correlate two processes externally |
| Uniformity filter | `min/max ≥ 0.25` rejects cross-rate leakage | Not implemented |
| Transition tracking | Immediate detection of mid-segment switch | Switch causes missed detections until steady state |
| Transition fold gain | Full K-fold gain preserved (0 dB loss) | Degraded — only aligned pulses contribute signal |

### Sensitivity loss during rate transitions (uavrt two-process scheme)

With two independent single-rate processes, neither can align all K pulses during the segment in which the tag switches rates. Each process folds at its own fixed PRI, so pulses that arrived at the *other* rate land on noise windows instead of signal. The fold score's signal component drops from $K \cdot P_\text{signal}$ to $c \cdot P_\text{signal}$ (or $(K-c) \cdot P_\text{signal}$), where $c$ is the number of gaps at that process's rate.

The EVT threshold is calibrated for the full K-fold noise distribution and does not change, so the effective SNR loss relative to a steady-rate segment is:

$$\Delta\text{SNR} = 10\log_{10}\!\bigl(c / K\bigr) \quad \text{dB}$$

for the better-aligned of the two processes.

**Example (K=5, switch at change-point c=2):**

| Process | Aligned pulses | Misaligned (noise) | SNR loss |
|---------|---------------|---------------------|----------|
| Rate A  | 3             | 2                   | $10\log_{10}(3/5) = -2.2$ dB |
| Rate B  | 2             | 3                   | $10\log_{10}(2/5) = -4.0$ dB |

The best available detection comes from the rate-A process at −2.2 dB. The rate-B process fares worse at −4.0 dB.

**Worst case** is a mid-segment switch ($c \approx K/2$): neither process gets more than ~$K/2$ aligned pulses, losing ~3 dB. For weak long-range signals already near threshold, this is enough to cause missed detections during every transition segment — a gap in coverage precisely when the animal's behavior is changing.

The Python multi-hypothesis detector avoids this entirely. The correct switch hypothesis (e.g. `A_to_B_c2`) places all K pulse windows at their true positions regardless of where the switch occurred, preserving the full K-fold integration gain with **zero SNR loss**. This is the primary sensitivity motivation for the multi-hypothesis approach and represents a significant range advantage during rate transitions.

## Output Packet Fields

Both send 12 little-endian doubles (96 bytes) to UDP 50000, but several fields have **different semantics**:

| Index | Field | Python meaning | uavrt_detection meaning |
|-------|-------|----------------|-------------------------|
| 0 | `tag_id` | `args.tag_id` (same ID for both rates) | `Config.ID` (primary) or `Config.ID + 1` (secondary rate) |
| 1 | `frequency_hz` | `args.freq` (absolute, from GCS) | `channelCenterFreqMHz × 1e6 + detectorPulse.fp` |
| 2 | `start_time_seconds` | `current_ts / 1e9` (wall clock) | `detectorPulse.t_0` (STFT-relative time) |
| 3 | `predict_next` | `start_time + tip` (or `tip_secondary` for B) | Lower bound of uncertainty range |
| 4 | `snr` | `10 log10(fold_score / noise_power)` | Signal PSD / noise PSD in dB |
| 5 | `stft_score` | **Score/threshold ratio** (dimensionless, ~1.0 at threshold) | **Raw spectral score `yw`** (absolute units) |
| 6 | `group_seq_counter` | `cycle` (monotonic counter) | Detection group counter |
| 7 | `group_ind` | **Hypothesis encoding** (0=A, 1=B, 2+=switch) | **Pulse index within K-group** (1-based) |
| 8 | `group_snr` | Same as per-pulse SNR | Group mean linear SNR → dB |
| 9 | `detection_status` | 0/1/2/3 (includes no-detection) | Boolean `det_dec` |
| 10 | `confirmed_status` | 1 if score_ratio ≥ confidence_ratio | 1 if `confirmpulses` passes |
| 11 | `noise_psd` | `noise_power / (Fs × n_w)` | `yw × dt²/T × (1 + 10^(SNR/10))⁻¹` |

**Incompatible fields:** `stft_score`, `group_ind`, and `confirmed_status` have fundamentally different semantics. The controller's `PulseHandler` forwards all fields identically regardless of detector mode — it is the GCS that must interpret them differently when `detection_mode == DETECTION_MODE_PYTHON` (1).

## Heartbeat

| Aspect | Python | uavrt_detection |
|--------|--------|-----------------|
| Mechanism | Dedicated 1 Hz thread | Called before each segment |
| Format | `send_pulse_udp(frequency_hz=0, ...)` | `sendHeartbeatOverUDP` (same zero-frequency convention) |
| Rate | 1 per second (independent of detection cycle) | 1 per segment (~every K×tip seconds) |

## Gap Handling

| Aspect | Python | uavrt_detection |
|--------|--------|-----------------|
| Small gap threshold | `< 2×tp` (~30 ms): zero-fill | `< tip/2` (~1.0 s): zero-fill |
| Large gap | `≥ 2×tp`: discard buffer, reset | `≥ tip`: stale flag, full reset |
| Negative timestamp | Explicit detection + reset | `< -tp/2`: reset buffers |
| Integrated error | Per-packet checking | `sum(diff(t) − 1/Fs)` vs `tipu + tipj` |

Python resets more aggressively on gaps (30 ms vs 1.0 s for zero-fill threshold).

## Configuration Parameters

### Python (CLI args from `_startPythonDetector`)

| Parameter | Default | From controller |
|-----------|---------|-----------------|
| `--tp` | 0.015 s | `pulse_width_msecs / 1000` |
| `--tip` | required | `intra_pulse1_msecs / 1000` |
| `--fs` | 3840 | HF=3840, Mini=3750 |
| `--port` | 10000 | HF: 10000, Mini: 20000+ |
| `--k` | 5 | `tagInfo.k` (validated ≥ 2) |
| `--pf` | 0.05 | `false_alarm_probability` |
| `--detection-margin` | 0.90 | `startDetection.detection_margin` |
| `--confidence-ratio` | 1.3 | `startDetection.confidence_ratio` |
| `--tip-secondary` | None | `intra_pulse2_msecs / 1000` (if ≠ 0) |
| `--min-uniformity` | 0.0 | 0.25 (when tip-secondary active) |
| `--tag-id` | 0 | `tagInfo.id` |
| `--freq` | 0 | `tagInfo.frequency_hz` |

### uavrt_detection (.config file from `_writeDetectorConfig`)

| Parameter | Default | From controller |
|-----------|---------|-----------------|
| `tp` | 0.02 s | `pulse_width_msecs / 1000` |
| `tip` | 1.0 s | `intra_pulse1_msecs / 1000` (or `intra_pulse2_msecs` for secondary) |
| `tipu` | 0 s | `intra_pulse_uncertainty_msecs / 1000` |
| `tipj` | 0 s | `intra_pulse_jitter_msecs / 1000` |
| `K` | 1 | `tagInfo.k` (no validation) |
| `Fs` | 192000 | HF=3840, Mini=3750 |
| `falseAlarmProb` | 0.01 | `false_alarm_probability` |
| `opMode` | `freqSearchHardLock` | Always `freqSearchHardLock` |
| `excldFreqs` | `[Inf, -Inf]` | Always `[Inf, -Inf]` |

**Differences:** Python lacks `tipu`/`tipj` support. Python validates K ≥ 2; MATLAB allows K=1. Python has `detection_margin` and `confidence_ratio`; uavrt does not. uavrt has `opMode` and `excldFreqs`; Python does not.

## Summary of Key Differences

1. **PRI uncertainty/jitter:** Python ignores (`tipu=tipj=0`). uavrt_detection handles via Wq matrix expansion. Equivalent for crystal tags; uavrt more robust for drifty oscillators.

2. **Cross-cycle state:** Python is stateless. uavrt carries priori across segments for D→C→T progression and true temporal confirmation. Python compensates with `confidence_ratio`.

3. **Rate-switch:** Python handles in one process with multi-hypothesis bank, preserving full K-fold gain during transitions (0 dB loss). uavrt requires two processes, each seeing degraded fold SNR during the switch segment (up to −3 dB at mid-segment switch) — enough to miss weak long-range signals. Python encodes the winning hypothesis in `group_ind`; uavrt cannot determine which rate produced a pulse without external correlation.

4. **Segment overlap:** uavrt overlaps by `2(KM+J)` windows so boundary pulses aren't missed. Python has no overlap — a pulse straddling segment boundaries is lost.

5. **Multi-frequency:** uavrt's peeling algorithm resolves overlapping frequencies. Python reports only the top-1 detection per cycle.

6. **Detection margin:** Python applies 0.90× threshold reduction. uavrt uses raw EVT threshold.

7. **No-detection reports:** Python sends `detection_status=3` every cycle with noise floor. uavrt is silent when nothing is detected. This gives operators better situational awareness with the Python detector.

8. **`stft_score` and `group_ind` semantics are incompatible** between detectors. The GCS must interpret these based on `detection_mode`.

9. **Gap handling:** Python resets more aggressively (30 ms threshold) vs uavrt (1.0 s). Python recovers faster from short transients but may unnecessarily discard data during brief dropouts.

10. **Noise estimation:** Identical core algorithm (3-window movmean, 10× median mask). uavrt additionally masks detected-pulse regions from the noise estimate.
