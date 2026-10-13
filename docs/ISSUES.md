# Known issues

Problems found in field data and the changes suggested for them. Living
document: items are added as they are found and removed when fixed (the fix
then lands in `design/`). Nothing here has been implemented yet.

Sources, unless stated otherwise: 60-acres flight, 2026-10-13, ~122 m AGL.

| Session | Mode | Geometry |
| --- | --- | --- |
| `Logs-Rotation-2026-10-13_20-35-56` | Manual rotation | Collar ≈260 m south (true bearing 180.6° ± 1.1°; collar ≈10 ft south of the auto-rotation point) |
| `Logs-Rotation-2026-10-13_20-50-12` | Auto rotation | Collar ≈10 ft (3 m) south of the rotation point, i.e. ≈88.6° below the horizon |
| `Logs-RawCapture-2026-10-13_20-34-27` | 13 s raw capture, 400 ft hover | Archived as `flying-400ft.dat` |

All three are archived in `/Volumes/Media/PDC Testing/60-acres 10-13-26 - DShot Flight/`
(rotation folders unchanged). The same day's office bench captures are in
`/Volumes/Media/PDC Testing/Office 10-13-26 - GR01 Noise Testing/`.

Tag 2: 147.970 MHz, 19 ms pulses, dual rate 1.5 s / 2.0 s, K = 20, 8 headings.

---

## Suggested changes

Not implemented. Numbers in brackets are the issues below that each change
addresses.

### Detector / bearing

| # | Change | Addresses |
| --- | --- | --- |
| S1 | **Remove projection and all re-processing of earlier headings.** Each heading's result is final when its dwell ends. If the fold search sighted the tag, measure it at that dwell's own phase and rate. If not, report a non-detection, which the bearing fit already handles as a censored slice. This removes the re-anchor, the measurement of earlier headings when a candidate is admitted after lock, and the PRI refit re-measure. | 1, 2, 5 |
| S2 | **Frequency-only re-search** (optional, after S1). Once a tag's frequency is known, search each dwell, including earlier buffered ones when the frequency is first found later, at that bin only, with its own threshold. Gives ≈1 dB more per heading, i.e. more headings for the bearing fit near range limits. First measure the real threshold gain (`permutation_null_threshold` with a one-bin `frequency_mask`) and the upward max-search bias on weak headings. | 2 |
| S3 | **Candidate identity by frequency.** For a configured dual-rate tag: frequency within tolerance and PRI ∈ {tip, tip_secondary}. No phase, no single PRI. | 3 |
| S4 | **Fold sideband candidates into their carrier.** A candidate with the same pulse timing as a much stronger candidate in the same dwell, at another frequency, is not a separate emitter. Exclude it from candidate slots and bearing selection. | 4 |
| S5 | **Fix the null-refinement train test** so a strong train is always removed from the first-pass null. A 60 dB train must never go unsighted. Find the cause first (item 6). | 6 |

When S1/S3 are built, update `design/COLLECTION_FLOW.md` (candidate bank,
locked cycles, re-anchor) and `design/RATE_SWITCH_DETECTOR.md`. The
fixed-offset estimator rationale in
`analysis/2026-09_DETECTOR_AMPLITUDE_ANALYSIS.md` stays as dated analysis,
with a pointer to this decision.

### Simulator / tests

| # | Change | Addresses |
| --- | --- | --- |
| S6 | Simulator option to switch rate at a chosen point in a rotation, ideally a short A→B→A burst. | 7 |
| S7 | Detector test with a rate switch between and within dwells. It must fail on current code: earlier headings lose power or wrong values are reported. | 1, 2, 7 |

### Field data collection

| # | Change | Addresses |
| --- | --- | --- |
| S8 | Enable `dump_spectrogram` on every test flight (≈30 MB per 8-heading rotation; check `dump_ms` against the real-time margin). Lets every item above be replayed offline on real data. **The TagTracker (GCS) setting that sends `dump_spectrogram` in the start command should default to on.** It was off for both 10-13 rotations, so those dwells cannot be replayed. | all |
| S9 | Archive rotation log folders with the raw captures in `/Volumes/Media/PDC Testing/` after every outing (done for 10-13). | — |
| S13 | Put vehicle (GPS) time on detector and controller log lines, or at least log the RPi-clock-to-vehicle-time offset at session start and on every change. | 11 |

### RF noise / hardware tests

| # | Test | Addresses |
| --- | --- | --- |
| S10 | PWM vs DShot A/B: same site and day, low hover (≈3 m) **and** 122 m in each mode, ESCs recalibrated after switching, same capture settings, ULog hover power and per-motor output. | 8 |
| S11 | Discovery on the desktop: 50 Ω terminator at 147.980 MHz, then antenna at 147.900 MHz. | 9 |
| S12 | Office capture with only the GR01 power lead unplugged; if the bumps vanish, go beyond the existing 2-turn clamp-on (stacked cores, antenna-lead ferrites, filtered supply). | 10 |

---

## Detector / bearing

### 1. Re-anchor overwrites correct measurements on earlier headings

**Seen live:** heading 0 showed a strong detection on the tracker display,
then dropped to nothing partway through the rotation.

**What happened** (rotation 1, candidate 0):

| Cycle (dwell) | Event | Heading 0 value reported |
| --- | --- | --- |
| 1 (000°) | Locked at 2.000 s PRI | 68.5 dB |
| 2 (045°) | Collar switched 2.0 → 1.5 → 2.0 s inside this dwell (`B_to_A_c9` hypothesis won; candidate 3 admitted at 1.5 s) | 68.5 dB |
| 3 (090°) | `reanchored`: anchor moved 86.503 s = 43.25 PRIs, i.e. the train restarted ≈0.5 s out of phase (tolerance 19.3 ms) | **16.0 dB (noise)** |
| 6, 8 | PRI refit, re-measure | 15.6 dB |

Heading 45 was also reduced (61.0 → 55.7 dB). The controller upserts per
slice, so the latest (wrong) value replaced the real one.

**Mechanism.** In `admit_lock_candidate` (`detector/pulse_detector.py`),
once the bank is locked, a sighting with the same frequency and PRI as a
banked candidate but a phase outside tolerance is `reanchored`: the
candidate's anchor is replaced, and the caller clears `measured` for that
candidate on every buffered slice, so all earlier headings are re-measured at
the new timing. `fit_lock_timing` then refits PRI across all buffered slices,
with the same single-train assumption.

**Why it was added.** `design/COLLECTION_FLOW.md`: on the 2026-09-12
simulator `moderate` run, a two-dwell PRI fit was ~90 ppm off and the same tag
was admitted as a duplicate candidate. That was a detector *estimate* error.
The same commit (`6c28399`) also widened the phase tolerance by
`pri_ppm_uncertainty × |Δt|`, which covers that case on its own.

**Why it is wrong in the field.** Collar crystal drift over a rotation is
~9 ms at the fitted +25 ppm (~17 ms at 50 ppm), under the 19.3 ms tolerance.
So the realistic trigger for a large phase miss is a rate switch, which
restarts the train. Every heading before the switch was measured at timing
that was correct for it; re-measuring them at the new timing can only turn
real readings into noise.

**Bearing impact (this session).** The reported 181.0° (confidence 0.90;
truth 180.6°) used heading 0 = noise. **The collar was being handled around
13:37 local** (confirmed by Don), during the 045° dwell, which is what
triggered the rate switch. So the live 68.5 dB at heading 0 (6 dB above the
180° front) was probably handling too, not a clean measurement. The bearing
came out right because the bug overwrote the disturbed heading with noise.
That is not evidence the system copes with a rate switch: on an undisturbed
collar the same bug would wipe a real reading.

**Suggested change.** Superseded by S1 (remove projection entirely). The
earlier, narrower suggestion was to drop only the backward re-measure and keep
same-bin identity and a forward timing update; once projection goes, there is
no timing to update, and identity is covered by S3.

### 2. Projecting lock timing onto other headings assumes the collar never switches rate

**The assumption.** Locked measurement (`measure_at_lock_psd`) projects the
candidate's anchor and PRI ($t_n = \text{anchor} + n \cdot \text{PRI}$) into
every buffered heading and sums power at those windows. This requires the
collar to have stayed on one rate, with an unbroken train, from the last
confirmed sighting (≥ one dwell + turn ≈ 45 s earlier) through the entire
40 s dwell being measured.

A dual-rate collar on a moving animal can switch any number of times, inside a
dwell or unseen between dwells. The Lotek collar switches to the moving rate
as soon as the animal moves and falls back within a few seconds of it stopping.

**Why projection exists.**
1. To measure weak (back-lobe / null) headings that the fold search cannot
   find.
2. To avoid the upward bias of max-search on weak headings (see
   `analysis/2026-09_DETECTOR_AMPLITUDE_ANALYSIS.md`).

The fold search fails on a weak heading not because it looks in the wrong
place but because the true cell's score is below the permutation-null
threshold, which must exceed the largest score noise produces anywhere among
~10⁶ (frequency × phase × hypothesis) combinations. Projection reads that
same true cell and reports it with no threshold. Its whole advantage is
knowing which cell is the true one, and that knowledge exists only if the
timing carried over from another heading is still valid.

**Failure modes:**

| What the collar did | Projected reading |
| --- | --- |
| Same rate, train unbroken | Correct |
| Switched before the dwell (during the turn) | Every window misses → noise |
| Switched partway through the dwell | Only pre-switch pulses hit → too low |
| On the other rate for the whole dwell | Windows rarely align → mostly noise |

Every failure reads *too low*. At a weak heading, "tag weak here" and "timing
no longer valid" give the same near-noise number; nothing can tell them
apart, and the bearing fit treats it as a real null.

**Where projection adds anything at all.** On a heading the search sights,
nothing: that heading is measured at its own phase. In this session the
carrier was sighted on 7 of 8 headings in both rotations. The two exceptions
were rotation 1, 045° (rate switch inside the dwell) and rotation 2, 315° (a
bug, item 6). So projection only matters on headings genuinely below the
search threshold, which is exactly where its timing cannot be verified.

`BearingCalculator` already has an assumption-free way to use a heading where
the tag was not found: a *censored* slice, meaning the model must predict
power below a fraction of the weakest detection. Projection's only gain over
censoring is a number instead of an upper bound, and that number is valid only
if the collar did not switch rate.

**The fit does not distinguish projected from sighted values.** Every
locked measurement is reported as a detection. `BearingCalculator` weights a
slice only by its noise level (`sliceWeights`: (median noise / slice noise)²).
`sighted` is only counted, not used as a quality input. So a sub-threshold
projected value whose timing assumption may be false enters the fit with the
same weight as a 70 dB sighted value at the same noise level. The noise
weight covers random error only, not the systematic error of measuring at the
wrong timing.

**Conclusion.** As implemented, projection has no trustworthy value for a
dual-rate collar and can inject wrong data into the bearing fit. The only
case that could be made verifiable is a weak heading *bracketed* by sighted
headings that confirm the same unbroken timing on both sides. Even then a
moving-rate burst inside the gap can slip through: a 2.0 → 1.5 → 2.0 s burst
of a multiple of 4 pulses (6 s, 12 s, …) shifts phase by a whole 2.0 s PRI and
leaves the timing looking unchanged.

**Consequence for re-processing earlier slices.** Every current reason to go
back to a buffered slice is a projection: a candidate admitted after lock
being measured on all earlier headings, a PRI refit, and a re-anchor. Without
projection, each heading's result is final when its own dwell ends: sighted at
its own phase, or censored. The only remaining reason to revisit buffered
data would be the frequency-only re-search (above): when a frequency is first
found on a later heading, earlier dwells could be searched again at that one
bin with a somewhat lower threshold (≈1 dB, see below). That uses only each dwell's own data
and carries no phase. Dropping projection also removes the per-heading
processing growth of item 5.

| | Fold search | Projection |
| --- | --- | --- |
| Rate assumption within the dwell | One rate, or one switch (hypotheses) | One rate, no switch hypotheses |
| Phase | Searched freely in this dwell's data | Fixed, carried from another heading |
| Needs anything outside the dwell | No | No switch since the last sighting |
| On failure | Score drops → no detection | Wrong value reported as valid |

**Suggested direction.** The only thing that reliably carries across headings
is the **frequency** (a rate switch changes timing, not the carrier). Keep
the known bin; on each dwell re-search phase and rate hypotheses at that bin
only:
- ≈200 phases × 38 hypotheses ≈ 8,000 combinations vs ≈10⁶ for the full
  search. The threshold must exceed the largest noise score among the cells
  searched, so fewer cells means a lower threshold at the same `pf`, but only
  slowly (the noise maximum grows roughly with log M). Illustrative Gamma(K=20)
  score model at `pf` = 0.05: threshold 53.6 → 45.8 (noise mean 20), i.e. the
  signal needed above noise drops 33.6 → 25.8, **≈1.1 dB**. A modest gain,
  not a large one. The real figure can be measured with
  `permutation_null_threshold` and a one-bin `frequency_mask`.
- **Why it matters (Don, 10-13: "super important"):** it does not extend
  first-detection range, but it raises the number of headings that reach the
  threshold for a tag already detected, i.e. real data points for the bearing
  fit. A lone detection scores confidence 0. In rotation 1 the ±45°
  neighbours of the front lobe were ≈3.5 dB down, so a 3-heading bearing
  (front + both neighbours) needs ≈3.5 dB more than first detection: with
  $1/r^4$ propagation that is ≈0.82 × detection range (≈4.9 km if detection
  is ≈6 km). +1.1 dB moves that to ≈5.2 km.
- No cross-heading phase assumption.
- Cost: some upward max-search bias on weak headings, to be quantified.

Alternatives considered:
- Drop projection: measure sighted headings at their own phase and send
  unsighted headings to the bearing fit as censored (non-detections).
- Measure sighted headings at their own phase and treat projected-only weak
  headings as *no information* rather than near-zero power.
- Keep projection but flag the result whenever a rate switch was seen in the
  rotation.

### 3. The same tag at its other rate becomes a separate candidate

Candidate identity (`locks_agree`, and the `reanchored` branch) requires the
same PRI. In rotation 1, candidate 3 (157.8 Hz, PRI 1.5 s) is the same tag, in
the same frequency bin, as candidate 0 (157.8 Hz, PRI 2.0 s), but was admitted
separately because it was sighted at the other configured rate.

**Suggested change.** For a configured dual-rate tag, identity = frequency
within tolerance and PRI ∈ {tip, tip_secondary}. Phase plays no part in
identity.

### 4. Sideband components become lock candidates; one was selected

A strong collar (60–75 dB) shows components at ≈ ±975 Hz (and +1525 Hz in
rotation 2), 30–35 dB below the carrier, with the same anchor and PRI. They
took 3 of the 4 `MAX_LOCK_CANDIDATES` slots in both rotations.

| Rotation | Candidate | Offset | Bearing | Confidence | Selected |
| --- | --- | --- | --- | --- | --- |
| 2 | 0 (carrier) | +184 Hz | 225.0° | 0.000 | |
| 2 | 1 (sideband) | +1157 Hz | 207.5° | 0.591 | **yes** |
| 2 | 2 (sideband) | −815 Hz | 225.0° | 0.000 | |
| 2 | 3 (sideband) | +1710 Hz | 209.8° | 0.501 | |

In rotation 2 the collar was nearly straight below. The carrier pattern is a
front/back-symmetric figure-8 (peaks 045°/225° and 000°/180° within 1 dB,
nulls −12 dB at 135° and −16 dB at 315°) that carries no azimuth information;
the carrier candidate correctly scored confidence 0. The selector then picked
a sideband with the same pattern shape and reported 207.5°. It should have
reported no bearing.

**Suggested change.** Fold candidates that share anchor/PRI with a much
stronger candidate into it (or exclude them from selection).

### 5. Detector real-time margin shrinks with every heading

`total_ms` minus (`stft_ms` + `fold_ms`) grows ≈1.9 s per heading. Total
rose from 22.4 s to 36.2 s of the 40.1 s segment over 8 headings, a margin of
1.1× at heading 8, in both rotations. The growth is the re-measurement of
every buffered slice × every candidate after refits. A 12-heading rotation or
more candidates would fall behind real time. Fixing 1 and 4 removes most of
it.

### 6. Rotation 2, heading 315: a 60 dB train went undetected (null threshold not refined)

`cycle_threshold`: `refined = false`, `dropped_windows = 0`, threshold
3.89e7 (others ≈73), null time 10.2 s (others ≈20.5 s). The first-pass
null was inflated by the tag's own train, and the refinement that exists to
remove it did not run (its train test rejected a 60 dB train). With a
threshold five orders of magnitude too high, the fold search could not sight
it. This is a bug: a 60 dB train must never be missed.

Projection then measured 59.7 dB there, which **masked the bug**: the bearing
input looked normal. Without the bug the search would have sighted the train
and measured it at its own phase; projection added nothing for this heading.
Cause not yet known. Candidates: a rate switch inside the dwell (half the
folds miss, failing the "most folds above noise" test), or the
`DOMINANT_FOLD_THRESHOLD` test.

---

## Simulator

### 7. A simulated rotation never contains a rate switch

The dual-rate 4-phase pattern (clean A / A→B / clean B / B→A) in
`CommandHandler::_simulatorCommand` advances `_simPhase` once per
`START_DETECTION`. A rotation sends one (`_simPhase` resets to 0 on stop), so
every simulated rotation is phase 0: a single-rate, continuous train. The
pattern was designed for the older one-start-per-slice flow.

The default `--sim-pri-ppm 43` is a real, constant offset (bench Lotek
collar, stable to < 25 ppm over 12.7 min), not drift.

**Suggested change.**
- A simulator option to switch rate at a chosen point in the rotation,
  ideally a short A→B→A burst like a collar being bumped.
- A detector unit test that feeds buffered headings with a phase restart and
  checks that pre-switch headings keep their measured power.

---

## RF noise / hardware

### 8. In-flight noise floor regressed ≈1.5 dB since 10-01 (DShot and altitude confounded)

| Flight | Site | Hover altitude (ULog) | Mid-band median (dBFS/Hz) |
| --- | --- | --- | --- |
| 9-17 (short legs, PWM) | 60 acres | no ULog | −114.7 |
| 9-21 (short legs, PWM) | Lake front | **≈4 m** | −115.1 |
| 9-22 (long legs, PWM) | Lake front | **≈1 m** | −117.4 |
| 10-02 (DShot), `log_79` | ? | 114 m (loiter, at capture time) | −115.0 |
| **10-13 (long legs, DShot), `log_83`** | 60 acres | **123.6 m** (loiter, at capture time) | **−115.0** |

The 60-acres site reads ≈0.8 dB hotter than the lake front (9-16 vs 9-17), so
≈1.5 dB is left over. Candidates from 10-01 onward:
- DShot,
- polymer props,
- arm tilt,
- FC rotation.

**But altitude is fully confounded with DShot.** Every PWM flight with a ULog
was a low hover (1–4 m, antenna about half a metre above the ground). Both
DShot flights hovered at ≈115–125 m. Height changes what the antenna sees: the
ground, and line of sight to distant noise sources. So the ≈1.5 dB cannot be
assigned to DShot on current data.

**Hover load is unchanged** (ULog, airborne, first and last 10 s trimmed):

| Flight | Current | Voltage | Power | Motor outputs | Yaw torque sp |
| --- | --- | --- | --- | --- | --- |
| 9-21 | 20.3 A | 24.36 V | 495 W | 0.84 / 0.83 / 0.54 / 0.55 (unbalanced) | +0.145 |
| 9-22 | 20.7 A | 23.96 V | 497 W | 0.69 / 0.67 / 0.69 / 0.75 | −0.020 |
| 10-02 | 22.2 A | 23.51 V | 523 W | 0.58 / 0.62 / 0.63 / 0.66 | −0.020 |
| 10-13 rot 1 | 21.4 A | 23.20 V | 497 W | 0.60 / 0.59 / 0.63 / 0.64 | −0.022 |
| 10-13 rot 2 | 21.2 A | 23.33 V | 495 W | 0.61 / 0.57 / 0.64 / 0.63 | −0.022 |

- **Electrical power at hover matches 9-22 within 0.5%.** The ≈3% higher
  current is the lower pack voltage. So the new props and arm tilt did not
  change the load.
- **Motor balance on 10-13 is comparable to 9-22.**
- **ESC RPM (DShot telemetry):** 4116–4486, a ≈9% spread.
- **Motor output values can't be compared across the PWM → DShot change**,
  because the output-to-throttle mapping changed.
- **ESC current telemetry reads 0** on all ESCs.

The lift is tilted: ≈+2.5 dB below 147.95 MHz, ≈+1.3 dB above 148.1 MHz.

The DShot frames and bidirectional replies run continuously when disarmed, and
the powered-at-rest office capture showed no floor lift. So if DShot is the
cause, it acts through ESC switching under load, not the signal wires.

**Planned:** PWM vs DShot A/B on 10-14:
- same site, same day;
- **in each mode, capture at a low hover (≈3 m) and at 122 m.** This separates
  altitude from DShot in one outing;
- ESCs recalibrated after switching to PWM, with motor balance checked;
- same capture settings;
- ULog hover power and per-motor output compared with 10-13 (≈496 W).

### 9. Narrow humps at 147.705 and 147.995 MHz in every October capture

≈15 kHz wide, ≈ −116 dBFS/Hz absolute. They appear at the same level:
- in the office with the vehicle off (Discovery on the desktop over coax),
- in the office with the vehicle powered,
- at 400 ft.

They are absent from every September capture at the same tune (9-13 office;
9-16, 9-17, 9-21 and 9-22 flights), with the same Discovery serial
`0xC852AA80BE44463D` throughout. In flight they add +1.2 / +2.3 dB; the upper
one is 25 kHz from the tag. Suspected to be inside the SDR.

**Test** (desktop, vehicle not involved):
1. 50 Ω terminator on the input at 147.980 MHz.
2. Antenna, tuned to 147.900 MHz.

If they survive the terminator and move with the tune, they are SDR-internal.

### 10. GR01 at rest: small bumps, ferrite had no effect

Office, 147.98 MHz tune. With the vehicle powered (via RPi), the floor rises
≈0.4–0.6 dB over the vehicle-off baseline (−121.5 dBFS/Hz), plus broad ≈1.6 dB
bumps ≈110–120 kHz apart that moved ≈250 kHz between captures. A 2-turn
clamp-on ferrite on the GR01 power lead, right outside its shielded box, left
their height unchanged. It was also fitted for all of the 10-13 flights. The
source (GR01 vs residual from the UBEC DUO) is not identified.

**Test:** capture in the same spot with only the GR01 power lead unplugged.
If the bumps vanish, the GR01 is the source. Since 2 turns already did
nothing, the next steps would be:
- stacked cores / more turns;
- a ferrite on its two antenna leads, which also leave the box;
- a filtered supply instead of raw 6S from the PDB.

The office ambient at 147.98 MHz on 10-13 (−121.5) was ≈7 dB above the
September office baselines at 146.000 MHz, so the office cannot currently
resolve vehicle noise below that level.

---

## Logging / observability

### 11. RPi clock is not vehicle time; logs cannot be lined up with the ULog

Log folder names use vehicle time (`LogFileManager`: `vehicleTimeNow()`).
Everything inside uses the RPi system clock, which in the field has no network
time source:
- controller log lines,
- detector `timestamp_ns` and cycle times,
- raw-capture `capture_utc`.

| | RPi clock | Vehicle / GPS time (folder name or ULog) | RPi behind by |
| --- | --- | --- | --- |
| Office raw captures (on the network) | 19:51:15 | 19:51:29 | 14 s |
| 60-acres raw capture | 20:12:07 | 20:34:27 | 22 min 20 s |
| Rotation 1, slice 2 armed vs 045° dwell start | 20:14:32 | 20:36:53 | 22 min 21 s |
| Rotation 2, slice 1 armed vs 000° dwell start | 20:26:45 | 20:50:23 | 23 min 38 s |

The offset also **changed by ≈77 s between the two rotations**, so even a
single correction per outing is not enough. Lining detector events up with
the ULog (attitude, altitude, motors), or with what people at the collar did,
currently needs dwell-by-dwell manual mapping.

Example: the rotation-1 rate switch happened in the 045° dwell, which is
20:14:32–20:15:12 by the RPi clock but **20:36:53–20:37:32 UTC (13:37 local)**
by GPS.
