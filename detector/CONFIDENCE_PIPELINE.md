# Detection Confidence Pipeline

How `false_alarm_prob`, `detection_margin`, `confidence_ratio`, and `uniformity`
interact to produce detection classifications.

## Overview

The detector uses a layered threshold system. Each layer serves a distinct role:

| Layer | Parameter | Default | Purpose |
|-------|-----------|---------|---------|
| 1 | `false_alarm_prob` (pf) | 0.05 | Sets the fundamental noise-floor threshold via EVT |
| 2 | `detection_margin` | 0.90 | Lowers the threshold to increase sensitivity |
| 3 | `confidence_ratio` | 1.30 | Classifies detections as HIGH or LOW confidence |
| — | `uniformity` | — | Diagnostic metric (not used in classification) |

## Pipeline flow

```
               false_alarm_prob
                     │
            ┌────────▼────────┐
            │  EVT threshold  │  Gumbel quantile from Monte Carlo
            └────────┬────────┘
                     │
            ┌────────▼────────┐
            │ × detection_    │  0.90 → lower threshold by 10%
            │   margin        │
            └────────┬────────┘
                     │
            ┌────────▼────────┐
            │ score_ratio =   │
            │ score/threshold │
            └────────┬────────┘
                     │
        ┌────────────┼────────────┐
        │            │            │
   < 1.0         1.0–1.3       ≥ 1.3
  No det.     SUBTHRESHOLD   SUPERTHRESHOLD
              Conf:0 [LOW]     Conf:1

        uniformity ──── logged but not used in decision
```

---

## Layer 1: EVT threshold from `false_alarm_prob`

The detector generates a per-frequency-bin detection threshold using Extreme Value
Theory (EVT). This calibrates the threshold to the noise statistics of the actual
STFT pipeline rather than relying on analytic assumptions.

**How it works:**

1. Run 100 Monte Carlo trials of pure complex Gaussian noise through the full
   STFT detection pipeline (same window size, overlap, fold structure, and
   Toeplitz W matrix as real data).
2. Collect the maximum detection score from each trial.
3. Fit a Gumbel distribution (extreme value Type I) to these maxima.
4. Set the threshold at the `(1 - pf)` quantile:

```python
loc, scale = gumbel_r.fit(max_scores)
threshold = gumbel_r.ppf(1.0 - pf, loc=loc, scale=scale)
```

**Effect of `pf`:**

| `pf` | Quantile | Threshold | Sensitivity | False alarm rate |
|------|----------|-----------|-------------|------------------|
| 0.10 | 90th     | Lower     | Higher      | Higher           |
| 0.01 | 99th     | Moderate  | Moderate    | Moderate         |
| 0.001 | 99.9th  | Higher    | Lower       | Lower            |

The EVT threshold is computed once at startup and cached. It depends on `K`,
`n_w`, `n_ol`, `nfft`, and the fold structure — but NOT on incoming signal data.

---

## Layer 2: `detection_margin` lowers the threshold

After the EVT threshold is computed, `detection_margin` scales it down:

```python
base_threshold *= detection_margin   # e.g., 0.90
```

A margin of 0.90 drops the threshold by 10%. This deliberately allows more
detections through — including marginal ones near the noise floor that would
otherwise be missed. The rationale:

- At long range, real tag signals may only barely exceed the EVT threshold.
- Lowering the threshold captures these weak detections.
- The two-tier confidence system (Layer 3) then sorts marginal detections
  from confident ones.

**Without `detection_margin`**, a score_ratio of 1.0 means the signal exactly
matched the noise-calibrated threshold. With `margin=0.90`, signals that would
have scored 0.90–1.00 now score above 1.0 and appear as detections.

---

## Layer 3: `confidence_ratio` classifies HIGH vs LOW

Each detection's score is compared to the (margined) threshold to produce a
score ratio:

```python
score_ratio = best_scores[b] / max(threshold[b], 1e-30)
```

This ratio is then compared to `confidence_ratio` (default 1.3):

```python
is_marginal = score_ratio < confidence_ratio
det_status = (DETECTION_STATUS_SUBTHRESHOLD if is_marginal
              else DETECTION_STATUS_SUPERTHRESHOLD)
```

| score_ratio | Classification | `detection_status` | `confirmed_status` | Log tag |
|-------------|----------------|--------------------|--------------------|---------|
| ≥ 1.3 | Confident | `SUPERTHRESHOLD` (1) | 1 | *(none)* |
| 1.0 – 1.3 | Marginal | `SUBTHRESHOLD` (0) | 0 | `[LOW]` |
| < 1.0 | Below threshold | Not reported | — | — |

The `[LOW]` tag in detector output and `Conf:0` in controller logs both indicate
a marginal detection — one that only passed because `detection_margin` lowered
the threshold, and whose score_ratio didn't reach `confidence_ratio`.

### The sensitivity band

The combination of `detection_margin` and `confidence_ratio` creates a
deliberate sensitivity band between `1.0` and `confidence_ratio`:

```
          margin lowers              confidence_ratio
          threshold here             classifies here
               │                           │
    ───────────┼───────────────────────────┼──────────▶ score_ratio
             1.0                         1.3
         ◄─────── sensitivity band ──────►
         Detections in this range are
         flagged [LOW] / Conf:0
```

**Tuning guidance:**
- Widen the band (lower margin or raise confidence_ratio) → more LOW detections,
  fewer missed weak signals, but more noise in results.
- Narrow the band (raise margin or lower confidence_ratio) → fewer LOW detections,
  cleaner results, but may miss weak signals at range.

---

## Uniformity: diagnostic metric (not yet in decision path)

Uniformity measures how evenly the detection energy is distributed across
K folds:

```python
uniformity = on_powers.min() / max(on_powers.max(), 1e-30)
```

| Uniformity | Meaning |
|------------|---------|
| ~1.0 | Equal power in all folds — consistent pulsed signal |
| ~0.5 | 2:1 power variation — moderate, possibly real at low SNR |
| ~0.0 | One fold dominates — spike-driven, likely transient/RFI |

### Current status

Uniformity is:
- **Computed** for every detection in `pulse_detector.py`
- **Logged** in the `[FOLDS]` diagnostic line
- **Stored** in the in-process `fold_info` dict for local downstream use
- **NOT sent** in the current UDP packet format, so it is not available to the controller/GCS unless the wire format is extended
- **NOT used** in the SUBTHRESHOLD/SUPERTHRESHOLD classification

A detection with `uniformity=0.000` will still be classified as `Conf:1` if
its `score_ratio ≥ 1.3`. This is a known gap.

### Planned use

The on-window uniformity test described in [CROSS_RATE_REJECTION.md](CROSS_RATE_REJECTION.md)
would add uniformity as a rejection criterion: detections with uniformity below
a threshold (e.g., 0.10) could be downgraded or rejected. Similarly, the on/off
contrast test in [CW_REJECTION.md](CW_REJECTION.md) would reject continuous-wave
interference. Neither test is implemented in the detection decision path yet.

---

## UDP packet fields

Each detection (or no-detection) is sent to the controller as a 96-byte UDP
packet containing 12 double-precision floats:

| Field | Value (detection) | Value (no-detection) |
|-------|-------------------|----------------------|
| `tag_id` | Tag ID | Tag ID |
| `frequency_hz` | Detected frequency | Expected frequency |
| `start_time_seconds` | Detection timestamp | Current timestamp |
| `predict_next_start_seconds` | timestamp + PRI | 0.0 |
| `snr` | SNR in dB | 0.0 |
| `stft_score` | score_ratio | Best candidate score_ratio |
| `group_seq_counter` | Cycle number | Cycle number |
| `group_ind` | 0 | 0 |
| `group_snr` | SNR in dB | 0.0 |
| `detection_status` | 0 (SUB) or 1 (SUPER) | 3 (NO_DETECTION) |
| `confirmed_status` | 0 or 1 | 0 |
| `noise_psd` | Noise PSD (W/Hz) | Noise PSD (W/Hz) |

The controller reads `confirmed_status` to decide `Conf:0` vs `Conf:1` in logs.

---

## Interpretation guide

When reviewing field data:

1. **score_ratio ≥ 1.3 + consistent frequency across headings** → high-confidence
   real detection.
2. **score_ratio 1.0–1.3 + consistent frequency** → real signal at edge of range,
   correctly flagged LOW.
3. **score_ratio ≥ 1.3 + uniformity ≈ 0 + scattered frequencies** → likely
   transient RFI or interference. The confidence gate passes it, but uniformity
   and frequency spread indicate it's not a real pulsed tag.
4. **score_ratio < 1.0** → below threshold, no detection reported.

Frequency consistency across rotation headings is currently the strongest
post-hoc discriminator between real tags and false positives, since uniformity
is not yet used in the classification logic.
