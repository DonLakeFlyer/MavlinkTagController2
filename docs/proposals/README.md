# Proposals — status

One row per proposed change. A proposal is **open** until the code lands; then
the relevant `docs/design/` document is updated to describe the shipped
behaviour, the row here becomes **implemented**, and the proposal file moves to
`docs/archive/`. **invalidated** means analysis showed the premise was wrong.

| Proposal | Doc | Status | Evidence / notes |
| --- | --- | --- | --- |
| End-to-end 8×20 hovering listening post (per-slice data-derived threshold, impulse blanking, confirmation rule, weighted fit, measured pattern) | [END_TO_END_8x20_SYSTEM.md](END_TO_END_8x20_SYSTEM.md) | open | Design hand-off 2026-09-12. Subsumes the "EVT threshold calibration" and "Frequency-consistency gate" rows below. Acceptance: Apr-11 replay detection rate 88 % → ≈ pf |
| Dominant-fold fraction gate | [CONFIDENCE_IMPROVEMENTS.md](CONFIDENCE_IMPROVEMENTS.md) Ch1 | implemented | `DOMINANT_FOLD_THRESHOLD` in `detector/pulse_detector.py`. Does not catch the Apr-11 noise detections. |
| On/off contrast (CW rejection) | [CW_REJECTION.md](CW_REJECTION.md), Ch2 | open | No `--min-contrast-db` in code |
| Uniformity / fold diagnostics in `PulsePayload` | Ch3 | partial | Computed and logged in `.jsonl` `FOLDS`; not on the wire (`PulsePayload` is 60 bytes). Wire-format change: detector, `shared/detector_protocol.h`, controller, tests in one commit |
| Frequency-consistency gate across cycles/headings | Ch4; [2026-09_MULTIPATH_ANALYSIS_REVIEW.md](../analysis/2026-09_MULTIPATH_ANALYSIS_REVIEW.md) A.2 | open | Strongest discriminator for the Apr-11 data (10 % of 69 detections within ±200 Hz). Persistent detector makes a detector-side implementation possible |
| EVT threshold calibration / cache validity | Review A.1 | open | Apr-11: 88 % of single-cycle dwells detected against `pf = 0.05`. Decide cache keying (noise stats, per-session regeneration, or invalidation on `noise_psd` drift) |
| Per-slice LM residuals + weight vector in the bearing fit | Review B.4, B.5 | open (TagTracker / `controller/BearingCalculator.cpp`) | Weight by `confirmed_status`; expose residuals for diagnostics |
| Sub-threshold soft measurements on no-detection | Review B.6 | open (controller / fit side) | Detector already sends best sub-threshold `score_ratio` + `noise_psd` on `NoDetection`; controller keeps them only as censored nulls. Remaining work: retain the values and weight them low in the fit |
| Spline interpolation over the antenna LUT | Review B.8 | open | `controller/AntennaPattern.cpp` |
| Linear-power inputs to the pattern fit | [2026-09_DETECTOR_AMPLITUDE_ANALYSIS.md](../analysis/2026-09_DETECTOR_AMPLITUDE_ANALYSIS.md) Ch6 | implemented | Fit consumes linear `group_snr` (→ `signal_psd` → `signal_power`); `snr_db` is diagnostic / `best_snr` only |
| 3-element Yagi (linear) | [YAGI_ANTENNA_DESIGN.md](YAGI_ANTENNA_DESIGN.md) | open (hardware) | Justified by gain/beamwidth only; needs mounted-pattern LUT before flying |
| RHCP feed for multipath rejection | — | invalidated | Reflection keeps CP sense below the Brewster angle; removed from the Yagi doc 2026-09-12 |
| Dual-altitude re-test of suspect slices | [archive/MULTIPATH_ANALYSIS.md](../archive/MULTIPATH_ANALYSIS.md) §2 | invalidated | No fringe pattern at 5 km for a ground-level transmitter (Δr ≈ 2 cm) |
| Fly higher as a link-budget choice | Review C.9 | open (operational) | Received power ∝ h² in the two-ray regime; ~22 dB ground loss at 400 ft / 5 km / 0.5 m collar |
| Test-collar capture campaign | [FIELD_TEST_PLAN.md](FIELD_TEST_PLAN.md) | open (field work) | Closes Checks 4, 5 and the full-rotation IQ gap from the April analysis |
| Fixed-offset amplitude, persistent detector, retrospective lock, frequency priors, single K, whole-rotation PRI fit | 2026-09_DETECTOR_AMPLITUDE_ANALYSIS.md Ch1–5, 7 | implemented | See the status table at the top of that document and [COLLECTION_FLOW.md](../design/COLLECTION_FLOW.md) |

Adding a proposal: create the doc here with a `**Status: open.**` line under the
title, and add a row above.
