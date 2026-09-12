# End-to-end system: 8×20 hovering listening post

> **IMPLEMENTED / ARCHIVED (2026-09-12).** The code items in §10 have landed;
> current behaviour is described in
> [docs/design/DETECTOR_PIPELINE.md](../design/DETECTOR_PIPELINE.md) §6 and
> [docs/design/COLLECTION_FLOW.md](../design/COLLECTION_FLOW.md). Source line
> numbers below refer to the code as it was when this was written. Deviations
> from the text: the permutation null gets a second pass with a detected
> train's windows redrawn (a tag inflated its own null far more than
> "slightly"); no new antenna id was added — measured pattern values will
> replace the existing `kRa2a` / `kRa23k` tables in place; the revisit dwell
> stays mandatory (no operator opt-out); vehicle climb/descent tuning was
> discussion only. Field items still open are listed in
> [docs/proposals/README.md](../proposals/README.md).

**Status: implemented.**

This document is the hand-off record of a design discussion (2026-09-12). It
states the operational requirements, the physics assumptions, the decisions
taken, exactly what the current code already does, and the concrete work
items that turn the current code into the target system. It is written so
that the work can continue on a different machine with no other context.

Related current-code descriptions (read these first when picking this up):

- [docs/design/COLLECTION_FLOW.md](../design/COLLECTION_FLOW.md) — acquisition → lock → per-slice measurement → bearing, as implemented
- [docs/design/DETECTOR_PIPELINE.md](../design/DETECTOR_PIPELINE.md) — per-cycle STFT / fold / EVT threshold
- [docs/design/SYSTEM_OVERVIEW.md](../design/SYSTEM_OVERVIEW.md) — processes, data path, log artifacts
- [docs/analysis/2026-04_FLIGHT_DATA_ANALYSIS.md](../analysis/2026-04_FLIGHT_DATA_ANALYSIS.md) and
  [docs/analysis/2026-09_MULTIPATH_ANALYSIS_REVIEW.md](../analysis/2026-09_MULTIPATH_ANALYSIS_REVIEW.md) — the April-11 false-detection evidence
- [FIELD_TEST_PLAN.md](../proposals/FIELD_TEST_PLAN.md) — capture campaign; this doc adds a 5 km acceptance flight to it

---

## 1. Operational requirements (given, not derived)

| # | Requirement | Source |
| --- | --- | --- |
| R1 | Crews currently search from Land Rovers with an omni antenna and headset; **absolute max range ≈ 1.5 km**. On a hit they switch to a hand-held directional antenna for bearing. | operator |
| R2 | The drone is worth launching only if it reaches **≥ 5 km**; below that, driving on is a better use of the time. | operator (ROI analysis) |
| R3 | The output the crew needs is **"tag present, drive that way"** — a sector-level bearing (±20–30° is plenty), not a survey-grade fix. The Land Rover does the close-in work once inside ~1.5 km. | operator |
| R4 | **Total flight time target ≈ 6 min** (7 min accepted for 8×20, see §4). Longer flights lose to "just drive further". | operator |
| R5 | The vehicle **only translates vertically**; max altitude **400 ft** (122 m); it can be commanded to hold arbitrary headings. | operator |
| R6 | Antenna: **RA-2AHS** (treated as the RA-2A pattern table). | operator |
| R7 | **Collar PRI ≤ 2 s** (worst case for dwell time). Crystal-timed collars; bench RA-2A collar measured at +43 ppm. | operator; `PRI_FIT_SPAN_PPM` comment in `pulse_detector.py` |
| R8 | Noise environment varies per launch site: sometimes clear field, sometimes **near power lines and/or telephone lines**. | operator |
| R9 | The single-cycle K=20 fold detector is considered sound and is **not** to be redesigned. | operator |

## 2. What the vertical-only constraint implies

These are the consequences that drive every later decision.

1. **One launch = one bearing line from one point.** No airborne triangulation.
   A second bearing costs land → drive → relaunch. Per-launch bearing quality
   is therefore what is being bought, but only up to sector level (R3).
2. **No spatial diversity against fading.** If the tag is in a fade at this
   ground spot, rotation does not help (the antenna phase centre barely
   moves). The only spatial lever is altitude, and altitude is *monotonic*:
   in the two-ray regime received power ∝ h_tx² h_rx² / R⁴ (see the Sept
   multipath review: path difference ≈ 2 cm at 5 km, no fringe pattern), so
   **always detect at 400 ft**, never vary altitude during a flight.
3. **Zero Doppler, zero range-rate.** Hovering means the tag's frequency and
   pulse-phase progression are exactly predictable across dwells; 1 m of
   GPS wander is 3 ns. The cross-heading frequency/phase gate (§6) is
   therefore near-perfect for a real tag and near-useless for noise.
4. **Altitude also helps noise.** Power-line noise is a near-field ground
   source; at 400 ft the drone is farther from it while the tag path gains h².
5. **Multipath is the same at every heading** (ground reflection geometry is
   fixed during a rotation). The amplitude-pattern fit is *more* trustworthy
   on a hovering platform than on a moving one. What *does* change with
   heading is the antenna gain toward the reflection point (side null −27.5 dB
   vs boresight 0 dB) — an uncharacterised pattern distortion that only a
   calibration flight on the real airframe can capture (§8).
6. **Descent speed is capped.** The safe way to descend fast is to add lateral
   motion to leave the downwash; vertical-only cannot. Practical ceiling
   ≈ 2.5–3 m/s before vortex-ring-state risk.

## 3. Range: what actually moves it

In the two-ray regime **range ∝ P^¼**: 6 dB ≈ +41 % range, 1.5 dB ≈ +9 %.
(Free-space near the vehicle would be P^½; two-ray is the regime at 5 km for
a ground-level collar.) Levers, ordered by dB per effort:

| Lever | Gain | Time cost | Notes |
| --- | --- | --- | --- |
| Always 400 ft | fixed cap | 0 | R5 makes this the ceiling; use it every flight |
| Honest threshold (§7) | 0 dB directly; lets `pf` be run truthfully at clean sites instead of conservatively everywhere | 0 | Also the single biggest false-positive fix |
| Impulse blanking (§7) | several dB of floor at impulsive sites, ~0 at clean ones | 0 | Standard for corona / switching noise |
| Cross-heading confirmation (§6) | "a few dB equivalent": lets single-dwell candidates be admitted loosely and confirmed structurally | 0 | Already largely in code (`locks_agree`) |
| STFT window = pulse width | 0 if already matched | 0 | **Verified matched:** `n_w = ceil(tp · fs)` ([pulse_detector.py L1796](../../detector/pulse_detector.py#L1796)) |
| Larger K on the sweep | ~5 log₁₀(K) dB at low SNR: 20→40 ≈ +1.5 dB | doubles hover | **Rejected** — does not fit R4 |
| 3-element Yagi | ≈ +3.5 dB, narrower beam | 0 | Hardware; see [YAGI_ANTENNA_DESIGN.md](../proposals/YAGI_ANTENNA_DESIGN.md) |
| Coherent inter-pulse integration | up to +3 dB / doubling | — | **Not feasible** — collar oscillator is not phase-coherent pulse to pulse |

## 4. Flight profile decision: 8 headings × K=20

### Constraint line

Hover budget ≈ 4–5 min at 2 s PRI → N_headings × K ≈ 120–160. Every option is
a point on that line. Worst-case detection is a tag exactly between two
boresights, at 180°/N off-axis. Pattern values from `kRa2a`
([AntennaPattern.cpp L15–20](../../controller/AntennaPattern.cpp#L15-L20)):
0 dB @ 0°, −0.5 @ 20°, −1.0 @ 30°, −2.5 @ 40°, −5.0 @ 50°, −10.5 @ 60°,
−14.5 @ 70°, −27.5 @ 90°, −10 @ 180°.

| N | K | off-axis worst | pattern loss | K gain vs 20 (5–10 log) | net worst | neighbours see tag at worst |
| --- | --- | --- | --- | --- | --- | --- |
| 4 | 30 | 45° | −3.75 dB | +0.9 … +1.8 | −2.0 … −2.9 | −27.5 dB (never confirms) |
| 6 | 20 | 30° | −1.0 dB | 0 | −1.0 | −10.5 dB (only with ≥10.5 dB margin) |
| **8** | **20** | **22.5°** | **≈ −0.6 dB** | **0** | **≈ −0.6** | **−3.75 dB (confirms with ≥3.75 dB margin)** |
| 8 | 15 | 22.5° | −0.6 dB | −0.6 … −1.25 | −1.2 … −1.9 | −3.75 dB |
| 12 | 10 | 15° | −0.25 dB | −1.5 … −3.0 | −1.8 … −3.3 | −1.0 dB |

(Linear interpolation in dB between the 10° table points, as the controller
does. An earlier in-chat estimate used −0.5 dB at 10° and gave −1.9 / −4.4 dB
for the 8-heading row; the table values above supersede it.)

Worst-case sensitivity is essentially flat in N — the beam is so wide that
pattern loss and integration loss trade one-for-one. N is chosen on
*confirmation coverage* and *bearing fit conditioning*, not on range.

### Decision

**8 headings at 45°, K = 20, one dwell each.** Rationale:

- Keeps R9 (K=20 untouched).
- Any tag is within 22.5° of a boresight (≤ 0.6 dB loss).
- Both neighbours are ≤ 45° off → ≥ −3.75 dB, so every detection with
  ≥ 3.75 dB margin is confirmed on a second heading *for free* (§6). At 6
  headings this needed ≥ 10.5 dB margin, i.e. edge detections never confirmed
  without an extra dwell.
- 8 points at 45° always include a near-peak sample and both slopes, so the
  pattern fit is well-posed without slope-targeting headings.
- 45° is already the slice spacing TagTracker uses (see
  `kRevisitHeadingToleranceDeg` comment,
  [CommandHandler.h L160–162](../../controller/CommandHandler.h#L160-L162)).

### Time budget

| Segment | Stock autopilot | Tuned |
| --- | --- | --- |
| Climb 122 m | 2.5–3 m/s → 40–50 s | 6–8 m/s → 15–20 s |
| 8 dwells × 40 s | 320 s | 320 s |
| 7 yaws × ~3–4 s | ~25 s | ~25 s |
| Descent 122 m | 1–1.5 m/s → 80–120 s | 2.5–3 m/s → 40–50 s (VRS cap, §2.6) |
| Final approach (~10 m at 0.5 m/s) | ~20 s | ~20 s |
| **Total** | **≈ 8 min** | **≈ 7 min** |

Start heading 1's dwell during the last ~100 ft of climb: at 300 ft the signal
is already within ~1 dB of its 400 ft value (h²), so a fast climb costs
nothing on detection. Results are computed during the sweep; the operator has
an answer before touchdown, including a clean "nothing heard".

Optional fallback (only when §6 leaves a single-heading hit): one more 40 s
dwell on that heading with phase prediction. Expected cost is low because it
runs only when there is something to confirm.

## 5. Per-dwell processing (detector) — target behaviour

Per slice, in order. Items marked **NEW** do not exist today.

1. **NEW — Impulse blanking on raw IQ** before the STFT: zero (or clip)
   samples whose magnitude exceeds N× a running median. Log the blanked
   fraction per slice as an operator-visible "site noise" indicator (new
   `.jsonl` key; see §10 observability).
2. STFT: `n_w = ceil(tp·fs)` (58 samples at tp 15 ms, fs 3840 Hz), 50 %
   overlap, hop `n_ws = 29` samples = 7.55 ms. Unchanged.
3. Per-bin noise estimate: masked median
   (`estimate_noise_power`, [L556](../../detector/pulse_detector.py#L556)). Unchanged.
4. K-fold search over (frequency bin, PRI hypothesis, phase offset) with
   local-max pooling (`fold_multi_hypothesis` [L417](../../detector/pulse_detector.py#L417),
   `_compute_fold_scores` [L488](../../detector/pulse_detector.py#L488)). Unchanged.
5. **NEW — Data-derived threshold** (§7) replacing the synthetic-noise Monte
   Carlo (`generate_evt_threshold`, [L1004](../../detector/pulse_detector.py#L1004))
   and its cache. Computed fresh **per slice**.
6. Candidate emission: frequency, PRI, anchor, `score_ratio`,
   `max_fold_fraction`. Dominant-fold gate (`DOMINANT_FOLD_THRESHOLD = 0.8`)
   retained as a cheap single-window-transient filter. Unchanged.
7. Slice spectrogram appended to `buffered_slices`
   (`MAX_BUFFERED_SLICES = 64`) for retro-measurement. **Already exists.**

## 6. Cross-heading confirmation — what exists, what changes

### Existing mechanism (keep)

- `locks_agree()` ([L816](../../detector/pulse_detector.py#L816)) declares two
  candidates the same when: |Δf| ≤ `LOCKED_SEARCH_HZ` = 200 Hz; |ΔPRI| ≤ one
  STFT step (`n_ws/fs`); and projected phase error
  `|(Δt mod PRI)| ≤ max(n_ws/fs, pri_ppm_uncertainty·1e-6·|Δt|)`.
- `combine_agreeing_locks()` ([L832](../../detector/pulse_detector.py#L832))
  refines PRI as `elapsed / round(elapsed / PRI)`; `fit_lock_timing` later
  refits PRI over all buffered slices (±300 ppm, 2 ppm step).
- Bank of `MAX_LOCK_CANDIDATES = 4`; per-candidate sightings
  `{slice_id: score_ratio}`; every banked candidate is retro-measured on
  every buffered slice (`measure_at_lock_psd`, [L652](../../detector/pulse_detector.py#L652);
  `pulse_indices_at_known_phase`, [L539](../../detector/pulse_detector.py#L539)).
- Controller: `confirmed = !rejected && n_sighted_slices ≥ kConfirmedSightings (2)`;
  a winner sighted on one heading only triggers one
  `COLLECTION_STATUS_REVISIT_REQUESTED` at the fitted bearing.

### Why the phase axis is selective

Noise candidates have anchors uniform over the PRI. Tolerance ≈ 7.5 ms
(growing to `150 ppm × Δt` ≈ 45 ms over a 300 s sweep) against PRI 2000 ms →
a noise peak agrees in phase by chance 0.4–2.3 %. Frequency ±200 Hz against
the observed ±2 kHz noise spread (Apr 11) → ~10 %. Both axes → 0.05–0.2 % per
heading. One neighbour hit at −3.75 dB is therefore sufficient confirmation.

Projection validity over the sweep: PRI must be known to
≲ 7.5 ms / 300 s ≈ 25 ppm to hold within one hop end-to-end; a single 40 s
dwell gives ~150 ppm (`INITIAL_PRI_PPM_UNCERTAINTY`), the tolerance widens
accordingly, and every merge tightens it. Cycle-count ambiguity needs error
< PRI/2 = 1000 ms over 300 s (500 ppm) — comfortably met. If a collar model
turns out to be RC-timed (percent-level PRI), the phase axis degrades and
frequency alone carries the gate — **check which collar models are in use**.

### Changes

- **C1 — Confirmation rule is the primary gate for the sweep.** A tag reported
  to the GCS as *confirmed* requires ≥ 2 headings with independent fold
  sightings of the same bank entry (already `kConfirmedSightings = 2`). A
  single-heading candidate is reported as *unconfirmed*, never as a
  detection. Verify the GCS-facing `BEARING_RESULT.confirmed` and live
  pulse-report stream make this distinction visible to the operator.
- **C2 — Fallback dwell semantics.** The existing revisit (at most one,
  at the fitted bearing) is the fallback dwell of §4. Keep; make it
  operator-optional at the GCS given the 7 min baseline.
- **C3 — Admission threshold review.** `--lock-score-ratio` = 3.0 was raised
  so that a false provisional lock is unlikely under the *current*
  (mis-calibrated) threshold. Once §7 makes `score_ratio = 1` mean pf
  honestly, revisit whether 3.0 is still right or is now throwing away real
  edge candidates; the confirmation rule (C1) is what should carry the
  false-alarm burden, not the admission ratio. Decide with the Apr-11 replay
  numbers (§9).

## 7. Threshold: derive the null from the dwell's own data

### What is wrong today

`generate_evt_threshold` runs 100 trials of **synthetic unit-variance complex
Gaussian** noise through the STFT + fold search, fits `gumbel_r` to the
per-trial maxima normalised by median per-bin power, and returns
`ppf(1 − pf)`. The result is scaled per bin (`threshold = base_threshold ×
noise_power`, [L1463](../../detector/pulse_detector.py#L1463)), multiplied by
`--detection-margin` (0.90), and **cached** keyed on
`(n_freq, n_time, n_search_bins)` in memory and on disk
(`--threshold-cache-dir`).

Apr 11: 88 % of single-cycle dwells "detected" at nominal pf = 5 %, while
the noise-floor *level* matched the Gaussian prediction within ~1 dB. The
level is tracked; something else is not. Real noise differs from the
synthetic model in five dimensions:

| Dimension | Clear field | Near lines | Tracked today? |
| --- | --- | --- | --- |
| Level | thermal + NF | + several dB | yes (per-bin scaling) |
| Colour (spurs, harmonics) | flat | comb of narrowband lines | partly (per-bin normalisation) |
| Tail shape (impulsiveness) | Gaussian | corona / gap discharge → heavy tails | **no** — leading explanation for 88 % |
| Directionality | isotropic | the line is a source *with a bearing*; RA-2A pointed at it sees 10–27 dB more of it than pointed away | **no** (threshold cached across headings) |
| Stationarity within 40 s | stationary | bursty (wind on insulators, switching) | no (masked median absorbs bursts) |

Caveat recorded honestly: heavy tails is an *inference*, not demonstrated by
the analysis docs. Alternative explanations (stale cache, calibration /
live search-space mismatch) are fixed by the same data-derived threshold
but would not be helped by impulse blanking. The Apr-11 replay (§9)
discriminates between them.

### Construction (a) — window permutation (preferred)

After the STFT of the slice, randomly permute the order of the time windows
and re-run the identical fold search (same hypotheses, same frequency mask,
same local-max pooling, same per-bin normalisation). Permutation destroys
periodicity at the PRI while preserving exactly: per-bin level, colour, spur
bins, tail shape, impulsiveness. Each permutation yields one sample of
"max normalised fold score under no-tag". M ≈ 30–50 permutations → fit
`gumbel_r` (reuse the existing fit code) → threshold at `pf`.

Properties:
- Computed **per slice, per heading** — no cache. Directional noise is
  captured automatically.
- If a real tag is present its pulses remain in the permuted data and inflate
  the null slightly → threshold errs conservative, never optimistic.
- 60 Hz mains harmonics (period 16.7 ms) are shorter than the hop; they
  raise the floor rather than form a pulse train, so they land in the null
  correctly.
- Cost: M fold searches on an already-computed STFT. The STFT dominates
  today; expected to fit inside a 40 s dwell on the Pi, **to be verified with
  a timing run** (§9).

### Construction (b) — decoy PRIs (alternative)

Run the identical search with PRI hypotheses no collar uses (e.g. 1.73 s,
1.91 s, …); each decoy search yields one null-max sample. Cheaper to reason
about but harder to guarantee decoys do not alias onto the true PRI grid;
heavy-tailed impulses can still line up by chance across few pulses.
Use only if (a) proves too slow.

### Around the threshold

- Impulse blanking on raw IQ (§5.1) runs *before* the STFT; the permutation
  null then describes the residual noise.
- Per-bin normalisation stays. Optionally exclude bins whose noise estimate
  exceeds X× the band median from the search entirely (spur bins).
- `--detection-margin` should become unnecessary; do **not** tune it against
  Apr 11 (that fits one site's noise). Leave the flag in place, default 1.0
  once §9 passes, or remove it — decide then.
- Log the fitted Gumbel `(mu, sigma)` and M per slice under the existing
  `EVT_THRESHOLD` `.jsonl` key so per-heading threshold drift is visible.

### Bearing-side consequence

With directional noise, the per-heading noise subtraction inside
`measure_at_lock_psd` is doing real work, and per-heading measurement
variance differs. The fit's single floor term `B` is not a substitute for
per-heading subtraction. Hence the weighted fit in §8.

## 8. Bearing (controller) — target behaviour

Existing (keep): on lock, every banked candidate is measured on every
buffered slice → the controller has a linear-power sample on *all 8 headings*
for every candidate, not just headings that detected. Least squares of
`P_i = A·G(θ_i − φ) + B` over φ on a 1° grid ±90° around the strongest
sample ([BearingCalculator.cpp L192–257](../../controller/BearingCalculator.cpp#L192-L257)),
`r_squared = fitFactor × spanFactor × dofFactor`, rejected below
`confidenceFloor` = 0.2. Best `r_squared` across candidates wins.

Changes:

- **B1 — Weighted least squares.** Weight each heading by the inverse of its
  measurement variance (derived from the slice `noise_psd` and pulse count).
  Headings at the floor contribute with low weight instead of either being
  dropped or dominating. (Existing open proposal: "Per-slice LM residuals +
  weight vector", review B.4/B.5.)
- **B2 — Measured installed pattern.** Replace the eyeballed `kRa2a` table
  with a LUT measured on the real airframe (calibration flight, §9). Keep the
  same 19-point 0–180° format and `byId()` selection so the controller code
  is unchanged; add a new `antenna_id` rather than silently redefining RA-2A.
  Absorbs the heading-dependent reflection-gain distortion of §2.5.
- **B3 — Sector output.** In addition to `bearing_deg` and `r_squared`,
  report the 45° sector aligned to the flown headings ("heading 3"). This is
  a GCS presentation change (TagTracker) — the controller output already
  carries what is needed.
- Bearing precision beyond sector level is **out of scope** (R3). Slope-
  targeted extra headings and repeated sweeps were considered and dropped
  for time.

Expected accuracy: the slope region (50–70°) is ~0.5 dB/deg, so a 1 dB
per-heading power error maps to a few degrees of φ; 3 dB → < 10°. Sector
output is robust to that.

## 9. Validation plan and acceptance criteria

1. **Offline — Apr-11 replay (acceptance for §7).** Run the April-11
   recordings (`/Users/don/Documents/PDC Testing`; replay path in
   [analyzer/iq_replay.py](../../analyzer/iq_replay.py) and
   [analyzer/flight_checks.py](../../analyzer/flight_checks.py)) through the
   new per-slice threshold. **Pass:** single-cycle detection rate falls from
   88 % to ≈ pf (5 %) with `--detection-margin` = 1.0 and no other tuning.
   Run with and without impulse blanking to settle the "heavy tails vs
   calibration" question in §7.
2. **Offline — timing.** Per-slice permutation null (M = 30 and 50) on the
   Pi: must complete well inside one dwell. Record wall time per slice.
3. **Offline — simulator regression.** `iq_simulator` at known SNR with
   `--pri-ppm 43`, 8 × 45° headings, `--antenna` RA-2A: confirm Pd vs SNR is
   unchanged or better at the same true pf, and that `confirmed` requires
   ≥ 2 sightings.
4. **Field — 5 km acceptance flight (add to FIELD_TEST_PLAN).** Known tag at
   5 km, drone at 400 ft, full 8 × 45° sweep, **once at a clean site and once
   at a power-line site**. Simultaneously: (a) measures the installed pattern
   for B2 (extend to a 10°-step 360° rotation on the same flight), (b) checks
   the link budget at the ROI range, (c) exercises the threshold on real
   directional noise. **Pass:** confirmed detection with correct sector at
   both sites within the 7 min profile.

## 10. Work items

Grouped by component. Order within a group is the suggested implementation
order. Test files per repository convention are listed alongside.

### Detector (`detector/pulse_detector.py`)

| # | Item | Tests |
| --- | --- | --- |
| D1 | Impulse blanking on raw IQ before `compute_stft_power`; CLI flag for the clip factor; `.jsonl` key for blanked fraction per slice | `detector/tests/` unit test with injected impulses: blanked fraction and floor reduction |
| D2 | `generate_evt_threshold` replacement: permutation null on the slice's own STFT power; reuse `gumbel_r` fit; return `(threshold, mu, sigma, n_perm)`; call per slice; delete/disable the disk cache path and `(n_freq, n_time, n_search_bins)` in-memory cache | unit: synthetic Gaussian slice → threshold within tolerance of the old Monte Carlo (sanity); heavy-tailed synthetic slice → higher threshold; slice with an injected tag → threshold not lower than without |
| D3 | Log `EVT_THRESHOLD` per slice with `mu`, `sigma`, `n_perm`, blanked fraction | `shared/log_schema.py` if a key is added |
| D4 | `--detection-margin` default → 1.0 after §9.1 passes (or remove) | update `detector/tests/test_end_to_end.py` expectations |
| D5 | Review `--lock-score-ratio` = 3.0 against §9.1 numbers (C3) | — |

### Controller

| # | Item | Tests |
| --- | --- | --- |
| K1 | Weighted LS in `BearingCalculator` (B1): per-slice weight from `noise_psd` / pulse count; expose residuals | `controller/tests/test_bearing_calculator.cpp` |
| K2 | New `antenna_id` + LUT for the measured RA-2AHS installed pattern (B2); `confidenceFloor` for it | `test_bearing_calculator.cpp`, `AntennaPattern` byId rejection of unknown ids stays |
| K3 | Confirm `BEARING_RESULT.confirmed` semantics (C1) and that unconfirmed single-heading results are visibly distinct on the tunnel protocol | `controller/tests/test_detector_protocol.cpp` only if wire fields change (then also `shared/detector_protocol.h`, `detector/detector_protocol.py`, `detector/tests/test_detector_protocol.py` atomically) |

### GCS (TagTracker — separate repo)

| # | Item |
| --- | --- |
| G1 | Fixed 8 × 45° sweep profile as the default collection; start heading 1 dwell on the final climb segment |
| G2 | Sector display (B3); "unconfirmed" vs "confirmed" vs "nothing heard" presentation |
| G3 | Revisit dwell (C2) operator-optional |
| G4 | Vehicle tuning: climb 6–8 m/s, descent ≤ 3 m/s |

### Docs (same PR as the code that lands)

- Update [docs/design/DETECTOR_PIPELINE.md](../design/DETECTOR_PIPELINE.md)
  (threshold section) and [COLLECTION_FLOW.md](../design/COLLECTION_FLOW.md)
  (confirmation rule, weighted fit).
- Flip this proposal's row in [README.md](../proposals/README.md) to *implemented* and
  move this file to `docs/archive/` when D1–D4, K1–K2 have landed; the
  "EVT threshold calibration / cache validity" and "Frequency-consistency
  gate" rows are subsumed by this doc and should be marked accordingly.
- Add the 5 km acceptance flight to [FIELD_TEST_PLAN.md](../proposals/FIELD_TEST_PLAN.md).

## 11. Explicitly rejected / out of scope

| Idea | Why not |
| --- | --- |
| K = 40 on the sweep | +1.5 dB for +4 min; fails R4 |
| 4 or 6 headings | Same worst-case range as 8; neighbours at −27.5 / −10.5 dB so edge hits never self-confirm |
| Separate bearing phase (slope-targeted headings, repeated sweeps) | Time; R3 only needs sector level |
| Varying altitude during a flight | Two-ray regime, no fringe; altitude is monotonic (invalidated proposal "Dual-altitude re-test") |
| Tuning `--detection-margin` / `pf` on Apr-11 | Fits one site's noise; the data-derived null makes the knob unnecessary |
| Coherent inter-pulse integration | Collar not phase-coherent |
| RHCP feed | Already invalidated (reflection keeps CP sense below Brewster angle) |
| Relying on the dominant-fold gate for false alarms | Rejected 0 of 69 Apr-11 noise events |

## 12. Assumptions register

Every non-obvious assumption behind the above, with what changes if wrong.

| # | Assumption | Basis | If wrong |
| --- | --- | --- | --- |
| A1 | Two-ray regime at 5 km for a ground-level collar (range ∝ P^¼) | Sept multipath review | Near free-space (P^½) makes every dB lever *more* valuable; ordering unchanged |
| A2 | Apr-11 false alarms are due to heavy-tailed real noise | Inference from 88 % vs a correctly predicted floor level | If cache/calibration mismatch instead: §7 still fixes it; impulse blanking (D1) becomes low-value. §9.1 decides |
| A3 | Power-line noise is impulsive | Corona / gap discharge physics | 60 Hz hum at VHF is Gaussian-like; then D1 gains nothing, D2 still needed |
| A4 | Permutation null fits within a 40 s dwell on the Pi | STFT dominates today | Fall back to construction (b) or reduce M |
| A5 | Collars are crystal-timed (ppm-level PRI) | Bench collar +43 ppm; `PRI_FIT_SPAN_PPM` design | RC-timed collars → phase axis useless; frequency-only gate (weaker, ~10 % per heading) |
| A6 | Installed RA-2AHS pattern ≈ Telonics RA-2A table to within a few dB | Unmeasured | B2 calibration flight replaces it regardless |
| A7 | Hover budget 15–20 min per battery; 7 min profile fits with reserve | Typical multirotor; airframe not checked | If shorter, drop to 6 × 20 with mandatory revisit dwell |
| A8 | 400 ft is the legal/operational hard cap | R5 | Higher altitude would be the single best range lever (h²) |
| A9 | Per-heading locked-frequency power is unbiased after noise subtraction | `measure_at_lock_psd` subtracts slice noise | Biased upward at the floor; K1 weighting mitigates |
| A10 | Neighbour-heading confirmation math uses the *table* pattern | `kRa2a` | Real pattern (B2) changes the −3.75 dB figure; recheck after the calibration flight |
| A11 | Land-Rover omni range 1.5 km, ROI threshold 5 km, 6–7 min flight | Operator | Changes the profile trade, not the architecture |

## 13. Glossary of code names used above

| Name | Where | Meaning |
| --- | --- | --- |
| slice / dwell | `COLLECTION_FLOW.md` | One heading's K·PRI of IQ; `(collection_id, slice_id)` |
| `score_ratio` | `Detection`, `PulseLock` | fold score / threshold; 1.0 = at threshold |
| `anchor_seconds` | `PulseLock` | absolute time of one projected pulse |
| sighting | `admit_lock_candidate` | an independent fold detection agreeing with a banked candidate on a slice |
| measurement | `measure_at_lock_psd` | fixed-coordinate power at a candidate on a slice, detection or not |
| `confirmed` | `BEARING_RESULT` | `!rejected && n_sighted_slices ≥ 2` |
| `n_ws` | `pulse_detector.py` main | STFT hop in samples (29 at defaults; 7.55 ms) |
