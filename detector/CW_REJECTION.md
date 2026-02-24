# CW Interference Rejection via On/Off Contrast Test

## Problem

Narrowband CW interference (repeaters, LO spurs, intermodulation products) is the #1 false alarm source for the stateless pulse detector. A continuous tone puts constant power in a frequency bin across all STFT windows. When the K-fold operation sums 5 evenly-spaced windows, it gets `K × signal_power` — indistinguishable from 5 real pulses. Because the detector is stateless (no cross-cycle confirmation), every cycle independently reports the CW signal as a detection.

## Insight

A real 15 ms pulse at a 2-second PRI has a **0.75% duty cycle**. Power is concentrated in K=5 STFT windows out of ~1,300 total. A CW signal has equal power in all ~1,300 windows. The fold score is identical, but the **power distribution across time** is completely different — and the detector already has the full power spectrogram available.

## Solution: On/Off Contrast Ratio

After the fold identifies a candidate detection at frequency bin `f` with best first-pulse offset `t0`:

```python
on_idx = t0 + np.arange(K) * N          # K windows where pulses should be
off_mask = np.ones(n_time, dtype=bool)
off_mask[on_idx] = False                 # everything else

on_power  = power[f, on_idx].mean()
off_power = power[f, off_mask].mean()

contrast_dB = 10 * np.log10(on_power / off_power)

if contrast_dB < MIN_CONTRAST_DB:
    # Reject: signal is present in off-windows too (CW interference)
    detection = None
```

### Threshold Selection

| `MIN_CONTRAST_DB` | Behavior |
|--------------------|----------|
| 3 dB | Conservative — rejects only obvious CW, minimal risk of dropping weak pulses |
| 6 dB | Moderate — good default, rejects CW and slow-fading tones |
| 10 dB | Aggressive — may reject very weak real pulses near the noise floor |

**Recommended starting point: 3–6 dB.** Any real pulse that the K-fold can detect has signal power well above the noise floor in its on-windows and near-zero signal contribution in off-windows, so the contrast is typically >> 10 dB. CW interference has contrast ≈ 0 dB by definition.

### Why It Works

| Signal Type | On-Window Power | Off-Window Power | Contrast | Result |
|-------------|-----------------|------------------|----------|--------|
| Real pulse (strong) | High | Noise floor | >> 10 dB | Pass |
| Real pulse (weak, barely detectable) | Moderate | Noise floor | ~3–8 dB | Pass |
| CW tone | Constant | Constant | ≈ 0 dB | **Rejected** |
| CW with slow fading | ~Constant | ~Constant | 0–2 dB | **Rejected** |
| AM-modulated CW | Varies | Varies | 0–3 dB | **Rejected** |

### What It Does Not Reject

- **Pulsed interferers with matching PRI** — these are genuinely pulsed signals with high contrast. Frequency discrimination or cross-cycle consistency checks would be needed.
- **Other wildlife tags** with similar pulse parameters — same issue, the signal is real and pulsed.
- **Periodic EMI at exact PRI submultiples** — if the EMI is truly pulsed (short bursts), it will have high contrast and pass.

These are much rarer scenarios than CW interference and represent fundamentally different problems (signal discrimination rather than noise rejection).

## Computational Cost

Negligible. The operation indexes into the existing power spectrogram matrix (already computed for the fold step), computes two means, and performs one division. Total: microseconds per detection candidate. No impact on cycle time.

## Integration Point

Insert after fold peak detection (README Section 5) and before SNR calculation (README Section 6). The check should run only on candidates that already exceed the EVT threshold — it is a post-detection filter, not a replacement for the threshold.

```
STFT + W matrix  →  K-fold  →  EVT threshold  →  contrast test  →  SNR calc  →  report
                                                  ^^^^^^^^^^^^^^
                                                  new step here
```
