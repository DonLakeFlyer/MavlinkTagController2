# Cross-Rate False Detection Rejection via On-Window Uniformity Test

## Problem

When a tag has two configured pulse rates (e.g., resting at 2.0 s and moving at 1.333 s), two detector instances are launched — one folding at each rate. A strong transmitter pulsing at rate A can produce false detections in the detector configured for rate B, because partial fold alignment captures enough energy to exceed the EVT threshold.

### Example: 2.0 s transmitter detected by 1.333 s detector

The ratio 2.0 / 1.333 = 1.5 = 3/2. With K=5 folds at 1.333 s spacing, some fold windows coincide with real pulses:

| Fold index | Time offset | Nearest real pulse | Captures energy? |
|------------|-------------|-------------------|-----------------|
| 0 | 0.000 s | 0.0 s | **yes** |
| 1 | 1.333 s | — | no |
| 2 | 2.667 s | — | no |
| 3 | 4.000 s | 4.0 s | **yes** |
| 4 | 5.333 s | — | no |

2 of 5 on-windows capture real pulse energy. For a strong tag, `2 × signal_power + 3 × noise_power` can exceed the EVT threshold, which was calibrated against `5 × noise_power`.

### Why the EVT threshold alone doesn't prevent this

The EVT threshold is set for a false alarm probability against pure noise. It answers: "how likely is this fold score from noise alone?" It does not ask whether the score came from K uniformly-contributing pulses or from a few very strong contributions. A strong signal leaking into 2/5 windows produces the same total score as a moderate signal present in all 5.

### Why CW rejection doesn't help

The CW contrast test (see `CW_REJECTION.md`) compares mean on-window power vs. mean off-window power. A pulsed signal at the wrong rate still has high contrast — power is concentrated in a few on-windows rather than spread uniformly across all time windows. The contrast test correctly passes it as "pulsed, not CW." The issue is that it's pulsed at the *wrong rate*.

## Solution: On-Window Uniformity Test

After K-fold detection exceeds the EVT threshold, check whether the power is distributed uniformly across all K on-windows. A real detection at the correct rate has signal energy in every on-window. Cross-rate leakage has signal in only a subset.

### Algorithm

For each candidate detection at frequency bin `f` with best first-pulse offset `t0`:

```python
on_idx = t0 + np.arange(K) * N
on_powers = power[f, on_idx]

uniformity = on_powers.min() / on_powers.max()

if uniformity < MIN_UNIFORMITY:
    # Reject: power is concentrated in a subset of on-windows
    detection = None
```

### Threshold Selection

| `MIN_UNIFORMITY` | Meaning | Behavior |
|-------------------|---------|----------|
| 0.10 | Weakest on-window ≥ 10% of strongest | Very permissive — only rejects extreme cases |
| 0.25 | Weakest ≥ 25% of strongest | Moderate — recommended starting point |
| 0.50 | Weakest ≥ 50% of strongest | Strict — may reject detections with natural power variation |

**Recommended starting point: 0.25.** This is permissive enough to tolerate natural pulse-to-pulse amplitude variation (fading, antenna pattern, slight frequency drift) while rejecting the cross-rate case where 2-3 windows are at the noise floor.

### Expected values by scenario

| Scenario | On-window powers | min/max | Result |
|----------|-----------------|---------|--------|
| Strong pulse, correct rate | [100, 95, 105, 98, 102] | 0.90 | **Pass** |
| Weak pulse, correct rate | [1.2, 1.0, 1.3, 1.1, 1.15] | 0.77 | **Pass** |
| Faded pulse, correct rate | [80, 40, 90, 50, 70] | 0.44 | **Pass** |
| Strong pulse, wrong rate (2/5 aligned) | [100, 0.5, 0.3, 95, 0.4] | 0.003 | **Rejected** |
| Strong pulse, wrong rate (3/5 aligned) | [100, 0.4, 90, 0.3, 85] | 0.003 | **Rejected** |

### Why it doesn't hurt weak detections

The test is a **relative** measure (ratio), not an absolute threshold. A weak pulse that the K-fold can detect has signal present in all K windows — each window contributes a small but roughly equal amount above the noise floor. The ratio min/max stays high even though the absolute values are low. Only when some windows are pure noise and others contain signal does the ratio collapse.

Consider a barely-detectable pulse with 3 dB per-window SNR:
- Each on-window: noise + signal ≈ 2× noise power
- min/max across windows ≈ 0.5–0.8 (noise fluctuation dominates, but signal keeps all windows elevated)
- Uniformity check passes

Compare with cross-rate leakage from a 20 dB signal:
- 2 on-windows: noise + strong signal ≈ 100× noise power
- 3 on-windows: noise only ≈ 1× noise power
- min/max ≈ 0.01
- Uniformity check rejects

## Edge Cases

### Scintillation and deep fades
In environments with strong multipath, a real pulse could experience a deep fade on one of K pulses, dropping min/max below the threshold. The 0.25 threshold allows a 4:1 power variation between the weakest and strongest pulse, which accommodates moderate fading. For extreme scintillation environments, the threshold could be lowered to 0.10.

### Very weak signals near noise floor
When per-window signal power is comparable to noise variance, the min/max ratio is dominated by noise fluctuation rather than signal uniformity. At per-window SNR < 0 dB (signal below noise), the ratio naturally stays in the 0.3–0.7 range because noise power in each window is similar. The uniformity test remains safe for weak signals.

### Harmonic rate relationships
When tip₁/tip₂ is exactly an integer (e.g., 1.0s and 2.0s), every fold window of the slower-rate detector aligns with a real pulse. The uniformity test would pass, and this is actually the hardest cross-rate case. However, integer-ratio rates produce `K/1 = K` aligned windows, meaning the fold score at the wrong rate equals the fold score at the correct rate — there is no way to distinguish them from fold statistics alone. Cross-cycle frequency consistency or explicit rate-pair exclusion logic would be needed for this case.

## Computational Cost

Negligible. Indexes K values from the existing power spectrogram (already in memory), computes min and max of a 5-element array. Total: nanoseconds per candidate. Runs only on detections that already passed the EVT threshold.

## Integration Point

Insert after the EVT threshold test and before (or alongside) the CW contrast test. Both are independent post-detection filters on the candidate list.

```
STFT + W matrix  →  K-fold  →  EVT threshold  →  uniformity test  →  contrast test  →  SNR calc  →  report
                                                  ^^^^^^^^^^^^^^^^    ^^^^^^^^^^^^^^
                                                  cross-rate          CW rejection
                                                  rejection
```
