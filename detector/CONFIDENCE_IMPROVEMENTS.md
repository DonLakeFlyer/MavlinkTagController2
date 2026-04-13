# Confidence Classification Improvements

Changes needed to reduce false-positive detections that currently pass the
confidence gate. Motivated by Apr-11 field data where Flight 2 produced 8/8
Conf:1 detections that were all transient/RFI artifacts.

## Problem Summary

The current confidence classification uses a single test:

```python
is_marginal = score_ratio < confidence_ratio   # default 1.3
```

This is necessary but insufficient. Flight 2 (5.75 km, 8 headings) produced
score_ratios of 1.3–2.6 on every heading — all classified Conf:1 — yet:

- Uniformity was 0.000–0.004 on all 8 headings (single-fold spikes)
- Frequency offsets ranged from −1192 to +1622 Hz (2,814 Hz spread)
- Peak bearing was 48° off the true collar direction

Each heading independently detected a different noise/RFI transient at a
different frequency. The score_ratio gate cannot catch this.

---

## Change 1: Dominant-Fold Fraction Gate in Detector

**Priority: High — biggest single impact, minimal code change**

### What

Add a dominant-fold fraction check to the detection decision. Detections
where a single fold dominates the K-fold score (max_fold_fraction above
`DOMINANT_FOLD_THRESHOLD`) are downgraded to SUBTHRESHOLD regardless of
score_ratio.

### Where

[pulse_detector.py](pulse_detector.py), in the detection reporting loop
(currently ~line 988).

### Current code

```python
is_marginal = score_ratio < confidence_ratio
det_status = (DETECTION_STATUS_SUBTHRESHOLD if is_marginal
              else DETECTION_STATUS_SUPERTHRESHOLD)
```

### Proposed code

```python
is_marginal = (score_ratio < confidence_ratio
               or fold_info['max_fold_fraction'] > DOMINANT_FOLD_THRESHOLD)
det_status = (DETECTION_STATUS_SUBTHRESHOLD if is_marginal
              else DETECTION_STATUS_SUPERTHRESHOLD)
```

### Threshold

No CLI argument — the `DOMINANT_FOLD_THRESHOLD` (0.8) constant is built
into the detector.

### Max-fold-fraction rationale

`max_fold_fraction = max(fold_powers) / sum(fold_powers)`.  Higher values
are worse (a single fold dominates the score).

| max_fold_fraction | Meaning | Action |
|-------------------|---------|--------|
| > 0.8 | Single fold dominates — likely transient/spike | Downgrade to SUBTHRESHOLD |
| 0.2–0.8 | Some variation across folds — normal for weak signals | Pass |
| ~1/K | Uniform power across all folds — ideal pulse train | Pass |

This replaces the old `min/max` uniformity metric which was fundamentally
broken at high SNR (all detections had U≈0.01 and were rejected).  The
fraction-of-sum approach matches uavrt_detection's `selectpeakindex`
heuristic and only flags genuine single-transient false alarms.

### Impact on existing detections

Reviewing all field data (uniformity values are from the legacy `min/max`
metric, now replaced by `max_fold_fraction`):

| Flight | Real signal? | Uniformity (legacy) | Affected? |
|--------|-------------|----------------------|-----------|
| Apr-9 K=10 (4 km) | Yes | Not available (pre-uniformity) | No |
| Apr-11 Flight 2 (5.75 km) | No — RFI | 0.000–0.004 | **Yes — all 8 downgraded** |
| Apr-11 Flight 3 (7.03 km) | Marginal | 0.001–0.029 | Some at 0.001–0.004 |
| Apr-11 Flight 4 (2.84 km) | Partial | 0.001–0.009 | Conf:1 headings at 0.001 |

Flight 4's three Conf:1 headings (90°, 135°, −135°) have uniformity 0.001,
which is below 0.10. These would be downgraded. This is a debatable call —
they had the correct bearing pattern — but their uniformity values are
indistinguishable from the Flight 2 garbage. If real signal at 2.84 km
produces uniformity this low, the threshold may need to be lowered to 0.005,
or the uniformity metric itself may need refinement (see open questions below).

---

## Change 2: On/Off Contrast Test in Detector

**Priority: Medium — covers CW interference, different failure mode**

### What

After a detection exceeds the EVT threshold, compare mean power in the K
on-windows vs mean power in all off-windows. CW interference has near-equal
power everywhere; a real pulse has high on/off contrast.

### Where

[pulse_detector.py](pulse_detector.py), in the fold detection function,
after computing `on_powers` and before building `fold_info`.

### Proposed code

```python
off_mask = np.ones(power.shape[1], dtype=bool)
off_mask[on_idx] = False
off_power = power[b, off_mask].mean()
on_power = on_powers.mean()
contrast_db = 10.0 * np.log10(on_power / max(off_power, 1e-30))

fold_info['contrast_db'] = float(contrast_db)
```

Then in the classification:

```python
is_marginal = (score_ratio < confidence_ratio
               or fold_info['max_fold_fraction'] > DOMINANT_FOLD_THRESHOLD
               or fold_info['contrast_db'] < min_contrast_db)
```

### New CLI argument

```
--min-contrast-db  (default: 3.0)
```

### Threshold rationale

| Contrast | Signal type | Action |
|----------|-------------|--------|
| < 3 dB | CW or near-CW interference | Downgrade to LOW |
| 3–10 dB | Weak pulse or fading | Pass |
| > 10 dB | Strong pulsed signal | Pass |

See [CW_REJECTION.md](CW_REJECTION.md) for the full design.

---

## Change 3: Add Uniformity to UDP Packet

**Priority: High — needed for controller-side logging and future use**

### What

Add `uniformity` as a 13th field in the UDP pulse packet so the controller
can log it and downstream consumers (TagTracker) can use it.

### Wire format change

This is a **breaking change** to the detector↔controller interface.

**Detector side** ([pulse_detector.py](pulse_detector.py)):

```python
# Current: struct.pack('<12d', ...)
# Proposed: struct.pack('<13d', ..., uniformity)
```

Add `uniformity` parameter to `send_pulse_udp()` and append it as field 12
(0-indexed). Packet grows from 96 to 104 bytes.

**Controller side** ([../controller/PulseHandler.h](../controller/PulseHandler.h)):

```cpp
typedef struct {
    double tag_id;
    double frequency_hz;
    double start_time_seconds;
    double predict_next_start_seconds;
    double snr;
    double stft_score;
    double group_seq_counter;
    double group_ind;
    double group_snr;
    double detection_status;
    double confirmed_status;
    double noise_psd;
    double uniformity;        // NEW
} UDPPulseInfo_T;
```

**Controller side** ([../controller/PulseHandler.cpp](../controller/PulseHandler.cpp)):

Add `uniformity` to the `PulseInfo_t` population and to the log format string:

```
Conf: %u Id: %2u snr: %5.1f ... uniformity: %.3f ...
```

**TunnelProtocol** ([../tunnel-protocol/TunnelProtocol.h](../tunnel-protocol/TunnelProtocol.h)):

Add `uniformity` field to `PulseInfo_t` if it needs to be forwarded to the
GCS via MAVLink tunnel.

### Both sides must be updated in the same commit.

---

## Change 4: Frequency Consistency Check at Controller Level

**Priority: High — strongest cross-heading discriminator, but more complex**

### What

After a full rotation (8 headings), compare the detected frequencies across
all headings that reported detections. A real tag produces the same frequency
(±~100 Hz). Scattered frequencies indicate independent noise/RFI hits.

### Where

This belongs in the **controller**, not the detector, because the detector
runs as independent per-heading instances with no cross-heading awareness.
The controller already accumulates per-heading results.

### Proposed logic

```
After 8-heading rotation completes:
  detected_freqs = [freq_hz for each heading with detection_status != NO_DETECTION]
  if len(detected_freqs) >= 3:
      freq_spread = max(detected_freqs) - min(detected_freqs)
      if freq_spread > max_freq_spread_hz:   # default: 200 Hz
          # Downgrade all detections in this rotation to LOW
          for each detection: confirmed_status = 0
```

### Where in controller

[../controller/PulseHandler.cpp](../controller/PulseHandler.cpp) or a new
rotation-level aggregation class. The controller currently processes each
pulse independently — it would need to buffer a rotation's worth of pulses
before applying this check.

This is a larger architectural change than the detector-side fixes and should
be designed carefully. The controller needs to know when a rotation starts
and ends, which headings belong together, and how to retroactively downgrade
already-forwarded pulses.

### Alternative: do it in TagTracker

TagTracker already receives all 8 headings for a rotation and fits a bearing
pattern. The frequency consistency check could live there instead, alongside
the R² pattern-fit quality metric. This avoids architectural changes in the
controller.

---

## Change Summary

| # | Change | Where | Priority | Effort | Catches |
|---|--------|-------|----------|--------|---------|
| 1 | Uniformity gate | Detector | High | Low | Transient/spike false positives |
| 2 | On/off contrast | Detector | Medium | Low | CW interference |
| 3 | Uniformity in UDP | Detector + Controller | High | Low–Med | Enables logging and downstream use |
| 4 | Frequency consistency | Controller or TagTracker | High | Medium | Cross-heading RFI |

### Recommended implementation order

1. **Change 1** (uniformity gate) — immediate, ~10 lines, biggest impact
2. **Change 3** (UDP field) — needed for visibility; coordinate with controller
3. **Change 2** (on/off contrast) — secondary filter, same code area
4. **Change 4** (frequency consistency) — requires architectural decision on
   where it lives

---

## Open Questions

### Uniformity threshold vs K

With K=20, a real weak signal might produce very uneven fold powers due to
noise fluctuation at low per-fold SNR. If per-fold SNR is ~0 dB, noise
dominates and fold powers vary widely even with signal present. The fixed
0.10 threshold may be too aggressive for K=20 at long range.

Options:
- Lower the threshold (e.g., 0.01) — but then it only catches the most
  extreme cases
- Use a K-dependent threshold — e.g., simulate expected uniformity vs
  per-fold SNR for a given K
- Use a different statistic — e.g., fraction of folds above the noise
  floor, rather than min/max ratio

This needs testing against more field data at different ranges and K values.

### What does uniformity look like for real weak signals?

We don't have a confirmed real detection with measured uniformity at long
range (the Apr-9 K=10 data predates the uniformity metric). Flight 4 at
2.84 km had uniformity 0.001 on the best headings — but those same headings
had 2,800+ Hz frequency spread, casting doubt on whether they're real.

**Action needed**: collect uniformity data from a known-range test (tag at
known position, drone at multiple ranges) to establish the expected
uniformity vs range/SNR relationship empirically.

### Confirmed vs unconfirmed semantics

Currently `confirmed_status` is a binary mirror of `is_marginal`:
- `score_ratio >= 1.3` → confirmed_status=1
- `score_ratio < 1.3` → confirmed_status=0

With the uniformity and contrast gates added, `confirmed_status=0` would
also cover:
- score_ratio < 1.3 (current)
- uniformity < 0.10 (new)
- contrast_db < 3.0 (new)

The GCS currently only sees confirmed_status as 0 or 1. If it needs to
distinguish *why* a detection is LOW, additional status bits or a quality
score would be needed. For now, 0/1 is sufficient — a detection is either
trustworthy or it isn't.
