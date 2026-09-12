# Detection Confidence Pipeline

How `false_alarm_prob`, `detection_margin`, `confidence_ratio` and the
dominant-fold gate (`DOMINANT_FOLD_THRESHOLD`) interact to classify each
cycle's strongest fold peak. Applies to acquisition reports; locked
measurements (`detection_status` 2) bypass this classification — see
[COLLECTION_FLOW.md](COLLECTION_FLOW.md).

## Overview

The detector uses a layered threshold system. Each layer serves a distinct role:

| Layer | Parameter | Default | Purpose |
|-------|-----------|---------|---------|
| 1 | `false_alarm_prob` (pf) | 0.05 | Sets the fundamental noise-floor threshold via EVT |
| 2 | `detection_margin` | 0.90 | Lowers the threshold to increase sensitivity |
| 3 | `confidence_ratio` | 1.30 | Classifies detections as HIGH or LOW confidence |
| 3 | `DOMINANT_FOLD_THRESHOLD` | 0.80 | Downgrades to LOW when one fold carries most of the score |

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
 NO_DETECTION  SUBTHRESHOLD      │
   (3)         Conf:0 [LOW]      │
                          ┌──────▼──────┐
                          │ max_fold_   │
                          │ fraction    │
                          └──┬──────┬──┘
                        > 0.8        ≤ 0.8
                    SUBTHRESHOLD  SUPERTHRESHOLD
                    Conf:0           Conf:1
                    [DOMINANT_FOLD]
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

## Layer 3: `confidence_ratio` and the dominant-fold gate classify HIGH vs LOW

Each detection's score is compared to the (margined) threshold to produce a
score ratio:

```python
score_ratio = best_scores[b] / max(threshold[b], 1e-30)
```

This ratio is compared to `confidence_ratio` (default 1.3), and the fold-shape
metric `max_fold_fraction` to `DOMINANT_FOLD_THRESHOLD` (0.8):

```python
has_dominant_fold = det.fold_info['max_fold_fraction'] > DOMINANT_FOLD_THRESHOLD
is_marginal = det.score_ratio < confidence_ratio or has_dominant_fold
det_status = (DETECTION_STATUS_SUBTHRESHOLD if is_marginal
              else DETECTION_STATUS_SUPERTHRESHOLD)
```

| score_ratio | max_fold_fraction | Classification | `detection_status` | `confirmed_status` | Log tag |
|-------------|-------------------|----------------|--------------------|--------------------|---------|
| ≥ 1.3 | ≤ 0.8 | Confident | `SUPERTHRESHOLD` (1) | 1 | *(none)* |
| ≥ 1.3 | > 0.8 | Single-fold transient | `SUBTHRESHOLD` (0) | 0 | `[DOMINANT_FOLD]` |
| 1.0 – 1.3 | any | Marginal | `SUBTHRESHOLD` (0) | 0 | `[LOW]` |
| < 1.0 | — | Below threshold | `NO_DETECTION` (3), carrying `noise_psd` and the best sub-threshold `score_ratio` | 0 | `no detection … best=…` |

The `[LOW]` tag in detector output and `Conf:0` in controller logs both indicate
a marginal detection — one that only passed because `detection_margin` lowered
the threshold, and whose score_ratio didn't reach `confidence_ratio`.
`[DOMINANT_FOLD]` marks a detection that cleared the ratio but whose energy sits
in one fold. The same `max_fold_fraction` gate also bars a peak from becoming a
lock candidate.

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

## Fold-shape metric: `max_fold_fraction`

How much of the K-fold score comes from a single fold:

```python
max_fold_fraction = max(on_powers) / sum(on_powers)
```

| max_fold_fraction | Meaning | Action |
|-------------------|---------|--------|
| ≈ 1/K | Equal power in all folds — consistent pulsed signal | pass |
| 0.2 – 0.8 | Uneven, normal for a weak signal in noise | pass |
| > 0.8 | One fold dominates — transient or RFI spike | downgrade to `SUBTHRESHOLD`; ineligible for lock |

It replaced the earlier `min/max` "uniformity" ratio, which fell to ~0.01 even
for strong real signals and could not be thresholded. Per-fold SNRs
(`fold_snrs`) and `max_fold_fraction` are written to the `FOLDS` `.jsonl`
entry and the `[FOLDS]` console line. They are **not** in the TTDP
`PulsePayload`, so the controller and GCS see only the resulting
`detection_status` (putting them on the wire is Change 3 in
[CONFIDENCE_IMPROVEMENTS.md](../proposals/CONFIDENCE_IMPROVEMENTS.md)).

Limits: the gate catches single-transient spikes only. Noise maxima whose
energy is spread across folds pass it — on the Apr-11 data `max_fold_fraction`
ranged 0.11–0.75 for all 69 noise detections. The on/off contrast test in
[CW_REJECTION.md](../proposals/CW_REJECTION.md) (continuous-wave interference)
and a cross-heading frequency-consistency check remain open proposals.

---

## UDP packet fields

Each detection (or no-detection) is sent to the controller as a typed TTDP
packet (`detector_protocol.py` / `shared/detector_protocol.h`): a header
(`collection_id`, `slice_id`, `tag_id`, message type `PULSE` or
`NO_DETECTION`) followed by the pulse payload:

| Field | Value (detection) | Value (no-detection) |
|-------|-------------------|----------------------|
| `frequency_hz` | Tag frequency (`--freq`) | Tag frequency (`--freq`) |
| `group_seq_counter` | Cycle number | Cycle number |
| `rate_state` | Winning rate hypothesis (`kRateStateXxx`) | 0 (`kRateStateA`) |
| `detection_status` | 0 (SUB) or 1 (SUPER) | 3 (NO_DETECTION) |
| `confirmed_status` | 0 or 1 | 0 |
| `candidate_id` | 0 (provisional lock) or >0 (alternate) | 0 |
| `start_time_seconds` | Segment start timestamp | Segment start timestamp |
| `predict_next_start_seconds` | timestamp + PRI | 0.0 |
| `snr` | SNR in dB | 0.0 |
| `score_ratio` | score_ratio | Best candidate score_ratio |
| `group_snr` | Per-pulse signal power (PSD units) | 0.0 |
| `noise_psd` | Noise PSD (W/Hz) | Noise PSD (W/Hz) |

The controller reads `confirmed_status` to decide `Conf:0` vs `Conf:1` in logs.

---

## Interpretation guide

When reviewing field data:

1. **score_ratio ≥ 1.3 + consistent frequency across headings** → high-confidence
   real detection.
2. **score_ratio 1.0–1.3 + consistent frequency** → real signal at edge of range,
   correctly flagged LOW.
3. **score_ratio ≥ 1.3 + `[DOMINANT_FOLD]`** → single transient; already downgraded
   to `Conf:0`.
4. **score_ratio ≥ 1.3, folds spread, but frequencies scattered across headings
   (±kHz) and SNR sitting at the K-fold noise floor (~17 dB at K=20)** → noise
   maxima passing every per-cycle gate (the Apr-11 pattern). Only cross-heading
   frequency consistency separates these from a tag.
5. **score_ratio < 1.0** → `NO_DETECTION` report; `noise_psd` and the best
   sub-threshold `score_ratio` are still available for trend plots.

Frequency consistency across rotation headings is the strongest post-hoc
discriminator between real tags and false positives; no per-cycle gate
currently uses it.
