# Review of MULTIPATH_ANALYSIS.md (2026-09-12)

[MULTIPATH_ANALYSIS.md](../archive/MULTIPATH_ANALYSIS.md) was written in the field on 2026-04-11 to explain the
bearing failures seen that day. This review checks it against (a) the recorded
Apr-11 Morning logs, re-run through `analyzer/flight_checks.py`, and (b) the
two-ray ground-reflection model it relies on. Section references point at the
original document.

## Summary

- The Apr-11 detections were **noise, not a multipath-faded signal**. The
  multipath mechanism the document describes never operated on that data.
- The path-difference formula the document is built on is wrong for a
  ground-level transmitter. At 5 km there is no fringe pattern to move through,
  so the dual-altitude re-test and the RHCP justification do not hold.
- Several of the proposed software changes are still worth doing for reasons
  unrelated to multipath. Those are listed with next steps at the end.

---

## Evidence from the Apr-11 logs

69 detections across 4 flights (flight-1: K=10 @ 146.664 MHz; flights 2–4:
K=20 @ 146.611 MHz). Detector settings were `--pf 0.05 --detection-margin 0.9
--confidence-ratio 1.3`, identical to the current defaults.

| Observation | Value | Implication |
|---|---|---|
| Offsets within ±200 Hz of entered frequency | 7 / 69 (10%) | Offsets are spread uniformly across the full ±2 kHz search window. Multipath changes amplitude/phase, not frequency; a faded collar still clusters in frequency. |
| Detections on the same frequency from two detectors with different PRIs (1.333 s and 2.0 s) | Both detect on nearly every heading, at unrelated offsets | A collar transmits one PRI at a time. |
| Median reported SNR vs predicted noise-only K-fold floor | K=20: 17.5 vs 17.2 dB; K=10: 15.1 vs ~14.2 dB | Reported SNR is the noise maximum statistic. |
| Single-cycle dwells producing a detection | 56 / 64 (88%) vs nominal `pf` = 5% | Threshold not calibrated to the noise environment. |
| Per-heading SNR spread | 2–4 dB, no directional pattern | No antenna-pattern signature. |
| `max_fold_fraction` (recomputed from logged `per_fold_snr`) | 0.11–0.75, median 0.36 | Energy spread across folds — the current dominant-fold gate (> 0.8) rejects none of them. |
| Best `score_ratio` | 2.55 | Below the current lock floor of 3.0, so the current detector would emit the same pulses but never lock or produce a bearing. |

What the logs cannot decide: *why* no collar signal was present (range, collar
state, frequency, ground loss).

---

## The geometry error

Line references below point into the plain-text source of the archived
document; GitHub's rendered view ignores `#L` fragments on Markdown files, so
open it with `?plain=1` (or in an editor) to follow them.

[The Geometry Problem](../archive/MULTIPATH_ANALYSIS.md#L9-L20) and
[§3 Altitude](../archive/MULTIPATH_ANALYSIS.md#L112-L122) use

$$\Delta r \approx 2h \sin\theta \approx 6\ \text{m}$$

That is the path difference for a plane wave from a distant *elevated* source
reflecting off the ground beneath the receiver. For a transmitter on the ground
the reflection point is near the collar and the two-ray path difference is

$$\Delta r \approx \frac{2\,h_{\text{drone}}\,h_{\text{collar}}}{d}
= \frac{2 \cdot 122 \cdot 0.5}{5000} \approx 0.024\ \text{m} \approx 0.012\lambda$$

Consequences:

- Fringes exist only inside the break distance
  $d_b = 4h_{\text{drone}}h_{\text{collar}}/\lambda \approx 120$ m. Beyond it
  the direct and reflected rays are nearly anti-phase everywhere, giving a
  smooth, geometry-fixed excess loss of
  $20\log_{10}\!\left(2\sin\frac{\pi\Delta r}{\lambda}\right) \approx -22$ dB
  relative to free space. That is a constant link-budget penalty, not fading.
- A ±30 ft altitude step changes $\Delta r$ by ~2 mm and received power by
  ~±0.6 dB. It cannot discriminate signal from fade.
- In this regime received power scales as $h_{\text{drone}}^2$: doubling
  altitude gains ~6 dB. Altitude is a fixed link-budget lever, not a per-slice
  test.
- At a 1.4° grazing angle — well below the ground Brewster angle (~15°) — the
  reflected wave keeps the same circular-polarisation sense. An RHCP antenna
  receives the reflection at full strength. The claimed 15–25 dB suppression
  does not apply; only the 3 dB linear-to-circular penalty remains.

---

## Section-by-section verdict

### Incorrect — drop

| Section | Problem |
|---|---|
| [The Geometry Problem](../archive/MULTIPATH_ANALYSIS.md#L9-L20) | Wrong $\Delta r$; no standing-wave boundary at 5 km. |
| [In the Bearing Calculation](../archive/MULTIPATH_ANALYSIS.md#L32-L41) | Never operated on Apr-11 data. The reflection also arrives from the same azimuth as the direct path, so it scales all slices by a common factor rather than distorting the pattern shape. |
| [§1 Circular Polarisation](../archive/MULTIPATH_ANALYSIS.md#L51-L59) | No rejection below the Brewster angle. A Yagi may still be justified by gain and beamwidth, not by this. |
| [§2 Dual-Altitude Re-test](../archive/MULTIPATH_ANALYSIS.md#L61-L91) incl. decision flow and time-cost table | Built on the fringe model. |
| Summary rows [L179–L180](../archive/MULTIPATH_ANALYSIS.md#L179-L180), [L190](../archive/MULTIPATH_ANALYSIS.md#L190) | Depend on the above. |

### Partially correct — keep the mechanism, drop the inference

| Section | Correct part | Incorrect part |
|---|---|---|
| [Current Indicator](../archive/MULTIPATH_ANALYSIS.md#L43-L45) | R² and per-slice residuals are the right fit-quality diagnostic. | "Low R² at this geometry is multipath, not noise" — on Apr-11 it was noise. |
| [§3 Altitude](../archive/MULTIPATH_ANALYSIS.md#L112-L122) | Altitude matters. | Via $h^2$ ground-loss scaling, not fringe hopping. |
| [In the K-fold Detector](../archive/MULTIPATH_ANALYSIS.md#L24-L30) | Fold uniformity cannot detect a static fade. | Moot — there was no signal to fade. |
| [Notes on Receiver](../archive/MULTIPATH_ANALYSIS.md#L195-L197) | HF+ is not the limiting factor; EVT method is fine. | EVT *calibration* was the actual Apr-11 problem and is not mentioned. |

### Correct — keep

| Section | Why it stands on its own |
|---|---|
| [§5 Confirmed-Status Weighting](../archive/MULTIPATH_ANALYSIS.md#L142-L148) | Higher-confidence slices should carry more weight regardless of why others are weak. |
| [§5 Null-Detection Slice Handling](../archive/MULTIPATH_ANALYSIS.md#L150-L156) | SNR=0 must not enter the fit as a measurement; sub-threshold fold score is real information. |
| [§5 Pattern LUT Interpolation](../archive/MULTIPATH_ANALYSIS.md#L158-L160) | Generic accuracy improvement. |
| [§4 Fold-Level Diagnostics](../archive/MULTIPATH_ANALYSIS.md#L126-L136) | Per-detection quality signal (useful for RFI/transient cases). Not a multipath indicator. |
| [§2 Required Code Changes](../archive/MULTIPATH_ANALYSIS.md#L93-L98) — `RotationInfo` bullets only | Residual exposure and a per-slice weight vector are the plumbing §5 needs. The `SliceInfo` second-SNR slot and altitude-step state-machine logic do not survive. |
| Summary rows [L170–L172](../archive/MULTIPATH_ANALYSIS.md#L170-L172), [L178](../archive/MULTIPATH_ANALYSIS.md#L178), [L181–L184](../archive/MULTIPATH_ANALYSIS.md#L181-L184), [L191](../archive/MULTIPATH_ANALYSIS.md#L191) | Follow from the above. L184/L191 apply only if the antenna is changed. |

---

## Next steps

### A. Fix the actual Apr-11 failure first (`MavlinkTagController2`)

1. **EVT threshold calibration.** An 88% per-cycle detection rate against
   `pf = 0.05` means the cached `mu/sigma` (loaded from `/home/pi`) plus the
   0.9 detection margin did not match the field noise. Decide whether the
   cache should be keyed on measured noise statistics, re-generated at session
   start, or invalidated when `noise_psd` departs from the calibration value.
   Validate against nominal, drifted, and degraded-noise scenarios per the
   repository test rules.
2. **Frequency-consistency gate.** Noise peaks wander across the ±2 kHz
   acquisition window; a collar does not. Reject or downgrade consecutive
   cycles whose offsets do not agree within a few bins. This is the single
   discriminator that would have flagged every Apr-11 detection.
3. **Re-verify with the existing tooling.** Re-run
   `analyzer/flight_checks.py` against Apr-11 after any threshold change to
   confirm the false-positive count drops; use `analyzer/iq_replay.py` on the
   `raw-capture/` data if it contains a raw IQ file.

### B. Keep and implement the surviving software items

4. **`RotationInfo.cc` / `.h` (TagTracker):** store per-slice residuals after
   LM convergence, expose `sliceResiduals()`, add an optional per-slice weight
   vector to `fitBearing()` (default 1.0).
5. **Confirmed-status weighting:** populate the weight vector from
   `PulseInfo_t.confirmed_status` (1.0 confirmed, 0.5 unconfirmed) in
   `RotateAndCaptureStateBase`.
6. **Sub-threshold soft measurements:** the detector already reports the best
   sub-threshold `score_ratio` and `noise_psd` on every `NO_DETECTION`; the
   controller stores those slices only as censored nulls. Retain the values in
   `RotationSlice` and add a low-weight (~0.25) soft-measurement path in the
   fit so headings that fell below threshold still constrain the pattern.
7. **Fold diagnostics in the pulse report:** add `fold_snr_variance` and
   `fold_uniformity` to the UDP `PulseInfo_t`. Document them as detection
   quality fields, not multipath indicators.
8. **Spline interpolation** over the RA-2AK LUT.

### C. Operational / hardware, re-justified

9. **Fly higher, as a fixed choice.** Received power at long range scales as
   $h_{\text{drone}}^2$; put the ground-loss term (~22 dB at 400 ft / 5 km /
   0.5 m collar) into the link budget explicitly and pick altitude from it.
10. **Antenna change only on gain grounds.** If a 3-element Yagi is adopted,
    justify it by forward gain and beamwidth, keep it linearly polarised, and
    characterise the mounted pattern on the 680 frame before updating the LUT.
    Do not add an RHCP feed for multipath rejection.

### D. Amend the original document

11. ~~Add a header to `MULTIPATH_ANALYSIS.md` pointing to this review~~ — done
    2026-09-12: moved to `docs/archive/` with a superseded banner; the RHCP
    section was removed from `YAGI_ANTENNA_DESIGN.md`.

Items A–C are tracked in [docs/proposals/README.md](../proposals/README.md).
