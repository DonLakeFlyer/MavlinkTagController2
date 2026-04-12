# VHF Multipath Analysis & Mitigation

## Context

This document covers multipath propagation problems encountered during aerial VHF collar telemetry surveys of African wild dog at 5+ km range, using a 680-class quadrotor at 400 ft AGL with an Airspy HF+ and Telenics RA antenna. The bearing solution is computed in TagTracker using an 8-point rotation sweep and Levenberg-Marquardt pattern fitting.

---

## The Geometry Problem

At 400 ft AGL (122 m) and 5 km range, the elevation angle to a ground-level collar is approximately **1.4°**. At this angle:

- The ground-reflected signal path is nearly equal in length to the direct path
- Over flat savanna, the ground reflection coefficient approaches unity
- The reflected signal picks up a ~180° phase shift on reflection (vertical polarisation, low angle)
- The path length difference between direct and reflected is approximately **6 metres** (~3 wavelengths at 146 MHz)

This means small changes in position or altitude shift the interference pattern between constructive and destructive. The system sits at the boundary of a standing wave pattern, producing significant signal fading.

---

## How Multipath Manifests in This System

### In the K-fold Detector (`pulse_detector.py`)

The K=20 integration window spans **40 seconds** at a 2-second PRI. During a static hover, the multipath geometry is completely fixed — every fold sees identical fading conditions. This means:

- **Fold uniformity stays high** even in a deep destructive null — the system cannot distinguish a consistently weak direct signal from a consistently faded one
- The SNR reported to TagTracker will be uniformly suppressed across all K folds
- The `uniformity` metric (min/max of `on_powers`) is **not diagnostic** in static hover mode

### In the Bearing Calculation (`RotationInfo.cc`, TagTracker)

As the drone yaws through the 8 bearing slices, the Yagi/RA antenna pattern rotates relative to the fixed multipath geometry. The ground-reflected signal arrives from a roughly fixed horizontal direction. This means:

- At some headings the reflected signal is near the antenna's main lobe → inflated SNR for that slice
- At other headings the reflected signal is off-axis → attenuated
- The LM fit sees a per-slice SNR distribution that does **not** conform to the RA-2AK pattern shape
- This degrades R² and can pull the fitted bearing toward the reflection, not the collar

The **180° ambiguity check** does not catch this — ground reflection arrives from roughly the same horizontal direction as the direct signal and will not trigger the front/back SNR ratio test.

### Current Indicator

**R² is the primary existing diagnostic.** Persistent yellow (R² 0.6–0.85) or red (R² < 0.6) bearing estimates during surveys at this geometry are multipath, not noise. Per-heading residuals from the LM fit identify which specific slices are contaminated.

---

## Proposed Mitigations

### 1. Hardware: Circular Polarisation (Highest Impact)

The fundamental fix. A **right-hand circularly polarised (RHCP)** antenna on the drone causes the ground-reflected signal to arrive as LHCP, which an RHCP antenna strongly rejects. The direct signal from the linearly polarised collar is received with a 3 dB penalty, but the ground reflection suppression is typically 15–25 dB — a net gain under multipath conditions.

For a 3-element Yagi at 146 MHz (half-wave elements, ~102 cm driven element, ~100 cm boom), a phased cross-dipole feed achieves RHCP. The antenna is mechanically simple — single boom, no switching required — and robust for bush operations.

**Note:** Switching to a Yagi requires updating the antenna pattern LUT in `RotationInfo.cc`. The RA-2AK pattern currently hardcoded does not match a Yagi's sharper forward lobe (~60° 3 dB beamwidth) and lower sidelobes. The mounted pattern on the 680 frame should be characterised empirically to account for prop wash and frame reflections.

---

### 2. Targeted Dual-Altitude Re-test (Highest Software Impact)

Instead of a full second 8-heading sweep at a second altitude (which doubles hover time), use the LM residuals from the first sweep to identify only the suspect slices, then re-test just those at a different altitude.

#### Decision Flow

```
First sweep completes → fitBearing() runs
    │
    ├─ R² ≥ 0.85 → Bearing is clean. Done.
    │
    └─ R² < 0.85 → Rank slices by |residual| (measured − modelled SNR)
                        │
                        └─ Re-test top 2–3 worst-fitting slices at alt ± ~30 ft
                                │
                                ├─ Alt1/Alt2 SNR agree within ~2–3 dB
                                │    → Consistent: true signal. Keep full weight.
                                │
                                └─ Alt1/Alt2 SNR diverge significantly
                                     → Multipath. Down-weight or clamp to lower value.
                                          │
                                          └─ Re-run fitBearing() with updated weights
```

#### Time Cost

| Approach | Extra Time |
|---|---|
| Full second sweep (K=20, 8 headings) | ~5–6 minutes |
| Targeted re-test of 2–3 slices | ~80–120 seconds |
| R² ≥ 0.85 (no re-test needed) | 0 seconds |

#### Required Code Changes

**`RotationInfo.cc` / `RotationInfo.h`**
- Store per-slice final residuals after LM convergence (currently computed but discarded)
- Expose a `sliceResiduals()` accessor for use by the state machine
- Add optional per-slice weight vector to `fitBearing()` (default all 1.0)

**`SliceInfo.cc`**
- Add a second SNR value per slice to hold the re-test result, alongside the original value
- The dual-altitude re-test provides the only opportunity for multiple values per slice — there is one K=20 run per heading per sweep, so no within-sweep averaging is possible

**`RotateAndCaptureStateBase`**
- After initial sweep and fit, check R² against threshold (suggest 0.85)
- If below threshold, read per-slice residuals, build targeted heading list
- Command altitude step, revisit only flagged headings, update slice SNR with weighted values
- Re-trigger `fitBearing()` with updated slice weights

---

### 3. Altitude as an Operational Variable

The path length difference between direct and reflected signals is:

```
Δr ≈ 2h × sin(θ)
```

where `h` is AGL height and `θ` is elevation angle to the collar. At 400 ft / 5 km, `Δr ≈ 6 m`. A **30 ft (~9 m) altitude change** shifts the path length difference by ~0.5 m — roughly a quarter wavelength at 146 MHz — which is enough to move significantly within the interference fringe pattern.

This is why the targeted re-test at a second altitude is effective: it does not need to fully escape the multipath zone, only to sample a sufficiently different point on the standing wave pattern to distinguish consistent signal from fading.

---

### 4. Fold-Level Diagnostics in the Detector

While fold uniformity is not diagnostic during a static hover (all folds see the same geometry), the **variance of `fold_snrs_db` within a single heading** does carry useful information about within-PRI signal stability. Adding this to the UDP pulse report would allow TagTracker to use it as an additional per-detection quality weight.

Suggested addition to `PulseInfo_t`:
- `fold_snr_variance` — variance of per-fold SNR in dB across K folds
- `fold_uniformity` — existing min/max ratio

High variance at a given heading, relative to other headings in the sweep, is a secondary multipath indicator even in static hover.

---

### 5. Single-Rotation Bearing Accuracy Improvements

These changes do not address multipath directly but improve the bearing estimate from a clean single rotation.

#### Confirmed-Status Weighting in the LM Fit

The LM fit in `RotationInfo.cc` currently assigns equal weight to all slices regardless of detection quality. `PulseInfo_t` already carries `confirmed_status` (0 = unconfirmed, 1 = confirmed), which indicates whether the detected pulse aligned with a prior pulse prediction. A confirmed detection is a higher-quality measurement than an unconfirmed one.

Adding a per-slice weight vector to the LM cost function — confirmed slices weight 1.0, unconfirmed slices weight 0.5 — would give higher-confidence observations more influence on the fitted bearing. The practical benefit is limited if most detections at 5+ km are unconfirmed, but costs almost nothing to implement.

The change is localised to the cost function and Jacobian computation in `RotationInfo.cc`, and the weight vector would be populated in `RotateAndCaptureStateBase` from the `confirmed_status` field of each received `PulseInfo_t`.

#### Null-Detection Slice Handling

When a heading produces no detection above threshold, `SliceInfo` holds SNR = 0 for that slice, and the current ≥ 3 valid slices check excludes it from the fit. This is the correct behaviour — including SNR = 0 as a real measurement would incorrectly signal to the LM fit that the antenna pattern minimum is at that heading.

However, the K=20 detector computes a fold score for every heading regardless of whether it crosses threshold. That sub-threshold score contains real information: it tells the fit that the signal at that heading is *below* the detection threshold, which constrains the pattern fit even if it is not a confirmed detection. Passing the sub-threshold fold score to TagTracker as a soft measurement with low weight (e.g., 0.25) would allow the LM fit to use this information rather than discarding it entirely.

This requires a change in `pulse_detector.py` to report the fold score and noise estimate even on non-detections, and a corresponding change in `SliceInfo` to store and apply soft measurements separately from hard detections.

#### Pattern LUT Interpolation

The RA-2AK pattern LUT is sampled at 10° resolution. Linear interpolation between 10° steps introduces small errors in the main lobe region where the pattern changes most rapidly. Replacing linear with spline interpolation over the existing LUT would improve fit accuracy near the bearing peak with no new measurements required.

---

## Summary of Recommended Changes by Repository

### `MavlinkTagController2` (`pulse_detector.py`)

| Change | Priority | Effort |
|---|---|---|
| Add `fold_snr_variance` to UDP pulse output | Medium | Low |
| Add `fold_uniformity` to UDP pulse output | Medium | Low |
| Report sub-threshold fold score and noise estimate on non-detections | Medium | Low–Medium |

### TagTracker (`RotationInfo.cc`, `SliceInfo.cc`, `RotateAndCaptureStateBase`)

| Change | Priority | Effort |
|---|---|---|
| Expose per-slice LM residuals after `fitBearing()` | High | Low–Medium |
| Add second SNR slot per slice for dual-altitude re-test result | High | Low |
| Implement targeted dual-altitude re-test in state machine | High | Medium |
| Add per-slice weight vector to LM fit; populate from confirmed status | Low–Medium | Low |
| Add soft-measurement slot per slice for sub-threshold fold scores | Medium | Low–Medium |
| Replace linear with spline interpolation on RA-2AK pattern LUT | Low | Low |
| Update antenna pattern LUT when switching to Yagi | Required for Yagi | Medium |

### Hardware

| Change | Priority |
|---|---|
| RHCP feed on Yagi (3-element, 146 MHz) | High — fundamental fix |
| Characterise mounted Yagi pattern on 680 frame | Required for LUT update |

---

## Notes on Receiver

The Airspy HF+ has high dynamic range and low noise figure at 146 MHz, and is not the limiting factor in this scenario. The multipath problem is geometric and must be addressed at the antenna or the bearing estimation layer. The EVT thresholding in the detector handles non-Gaussian noise statistics well and does not require changes for multipath mitigation.
