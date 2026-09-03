# Flight Data Analysis Plan

Companion to `DETECTOR_AMPLITUDE_ANALYSIS.md`. That document is entirely
simulation; this one lists the checks against real flight data that confirm it,
falsify it, or settle the questions simulation cannot answer.

Run **Check 0 first.** It is a one-line histogram, and if it comes out wrong the
rest of the analysis is void.

## Results — April 2026 PDC testing data (analysed)

All checks below were run against the recorded data in `PDC Testing/`
(Apr-8 through Apr-11 sessions: ground range tests at 3/4/5 km with raw IQ,
plus four flights of manual 8-heading dwells). Tools:
`analyzer/flight_checks.py` (log-driven, Checks 0–5) and
`analyzer/iq_replay.py` (offline IQ replay, Check 6).

**Pulse-width caveat.** All flights and the original replay ran the detector
at `--tp 0.015`; the collar's pulse width is 19 ms. The mismatched
rectangular window captures 15/19 of the pulse energy, so every SNR
measured from this data is ~1.0 dB pessimistic (10·log10(15/19)). Re-running
the replay at `--tp 0.019` confirmed the gain is that 1 dB and nothing else.
The floor (Check 1) is unaffected — it is a max-selection artefact
independent of pulse width, and the simulation it was compared against also
used `--tp 0.015`, so the comparison is like-for-like. Range figures below are
correspondingly ~6–9 % conservative; no conclusion changes.

| Check | Verdict |
|---|---|
| 0 — slices/rotation | **Model holds.** Genuine detections cluster on 1–2 adjacent headings; the rest are floor hits. |
| 1 — SNR floor | **Confirmed to ~0.1 dB.** K=20 edge 16.3 / median 17.3 dB vs predicted ~17.2. Floor moves with K. |
| 2 — noise vs multipath | **Consistent with noise, not conclusive.** Floor-hit frequencies scatter across the full ±1920 Hz span; only 20 % fall within ±200 Hz. Needs rear-heading filtering around the empirical carrier + collar-OFF control (see detail). |
| 3 — front/back contrast | Real SNR above floor only on the front lobe; consistent with the compression model. |
| 4 — true PRI | **Not measurable from these logs** (per-cycle 1 s timestamps only). Needs raw IQ / `ipi_analyzer`. |
| 5 — noise floor | No systematic heading dependence; no disarmed baseline exists in this data — inconclusive. |
| 6 — offline IQ replay | **Done — core claim confirmed on real data.** See below. |

**Net conclusion: every prediction in the companion document that could be
tested survived.** The confirmed findings: the metric floor and the
fixed-offset amplitude readout, which recovers range-tracking signal the
current metric buries. The rear-heading energy is consistent with that floor
but its classification stays open until the Check 2 rear-heading/collar-OFF
tests run. The recommended changes (1–4) stand; antenna changes remain not
worth buying on the available evidence.

### Check 6 detail — 3/4/5 km replay

The 3/4/5 km raw captures contain a real collar train — at **PRI 2.0 s** (the
tip-2.0 collar) and **+630…+660 Hz** off the entered 146.170 MHz. Replaying
the detector's own fold versus the fixed-offset estimator at the locked
(bin, phase):

| range | metric SNR | above K=3 floor (~12.5) | fixed-offset amp | real-time detector reported |
|---|---|---|---|---|
| 3 km | 16.9 dB | +4.4 dB | 22.6× noise (13.5 dB) | 13.9 dB [LOW], wrong bin — missed |
| 4 km | 15.6 dB | +3.1 dB | ~13× (~11 dB) | 13.9–14.5 dB at +662 [LOW] |
| 5 km | 13.2 dB | +0.7 dB | 5.9× (7.7 dB) | 13.5–15.8 dB at +662 [LOW] |

The fixed-offset estimator tracks range monotonically (−5.8 dB over
3→5 km, effective path exponent ~2.6); the metric compresses to within
~1 dB of floor at 5 km. The real-time detector locked the true collar
repeatedly at 5 km but reported it indistinguishably from its own noise
false-locks — the front/back ambiguity mechanism, observed directly.

The Apr-11 13 s capture shows no detectable train at either rate
(consistent with the Apr-11 flights being almost entirely floor hits).

### Estimated maximum detection range

Per-pulse SNR from the replay: +8.8 / +6.4 / +2.9 dB at 3/4/5 km. Against
the fold detector's requirement (Pd=0.9: K=10 → 5.1 dB, K=20 → 2.6 dB),
boresight detection range on that day's geometry (~1 m collar height):

- K=10: **~4 km** — K=20: **~5 km** — K=40: ~6 km

The collars are on **African wild dogs**, which lowers this via the two-ray
collar-height term (h² → 20 dB/decade):

- dog standing/trotting (~0.5 m): **~3 km** at K=20
- dog lying/resting (~0.2 m, plus body shadowing): **~1.5–2 km**
- denning: effectively overhead only

Wild dogs rest through the heat of the day, so midday flights face the
worst posture. Flying at dawn/dusk when the pack moves is worth +6–10 dB —
more than any hardware change on the table.

### Additional findings not anticipated by the plan

- **Cross-detector agreement is a cheap lock qualifier.** Each flight runs
  two detectors (tip 1.333 / 2.0) on the same feed. When both report the
  *same* frequency offset in a dwell, it is real energy; disagreement marks
  floor hits. Already logged — worth adding to the companion document's
  change 4 qualification list.
- **The collar's true offset was ~+650 Hz**, so a naive ±200 Hz frequency
  gate would have rejected the real collar on first acquisition. The
  wide-then-tighten ordering in change 4.2 is load-bearing; keep it.
- Controller logs contain ANSI colour codes — strip before parsing.
- No bearing CSVs existed in this data set (the flights predate that
  logging), so Checks 0/3 used the per-heading dwell logs instead.

---

## Data sources

From `CommandHandler.cpp`, each pulse is logged as:

```
Conf: %u Id: %2u snr: %5.1f heading: %3.1f score_ratio: %.3f noise_psd: %5.1g
freq: %9u seq: %u group_ind: %u lat/lon/yaw/alt: %3.6f %3.6f %4.0f %3.0f
```

and each solved bearing as:

```
tag_id,bearing_deg,r_squared,n_valid_slices,best_snr
```

Everything below except Check 5 and Check 6 comes out of those two.

---

## Check 0 — Falsification: slices per rotation

**The prediction to break.** Simulation says that at long range roughly **1 of 8
headings** produces a detection (2.2 at 6 dB peak SNR, 3.1 at 9 dB), and that the
ones which do are clustered on the front lobe.

**How.** Histogram `n_valid_slices` from the bearing CSV, split by range or by
`best_snr`. You already log exactly this field — no new instrumentation needed.

**Decision:**

- Long-range rotations centre on 1–3 slices → model holds, proceed.
- Long-range rotations routinely show 5–8 slices → **the model is wrong.** Stop
  and re-derive; every downstream conclusion in the companion document depends
  on this.

**Result: model holds — with a trap.** Naively 5–8 of 8 headings "detect",
which would falsify the model. But almost all are floor hits (SNR 14.5–17,
score_ratio ≈ 1.0–1.3, random frequency offset). Filtering to genuine
detections (score_ratio ≥ ~2.5 and a consistent offset) leaves 1–2 adjacent
headings per rotation — e.g. Apr-10 flight-1: 45°/90° at SNR 22.6–27.5,
score_ratio 6.5–20, both detectors at +463 Hz. The "detections" elsewhere
are the pf=0.05 noise-scraping the companion document predicted.

Also worth pulling: the fraction of rotations where `n_valid_slices` falls below
whatever minimum your fit requires, i.e. how often a rotation produces no real
fit at all.

---

## Check 1 — Is there a floor in the reported SNR?

**Prediction.** At K=20, reported `snr` should pile up against ~17–18 dB and
rarely go below, because that value is what pure noise produces. At K=5 the same
floor sits near 12.5 dB.

**How.** Histogram `snr` across all logged pulses. Overlay by K if you have
flights at both settings — the floor should move by ~4.7 dB between them, which
is a strong signature.

**Decision:**

- Sharp left edge near 17 dB with a tail to the right → the pedestal is real and
  is the dominant effect.
- Values spread smoothly well below 17 dB → the floor is lower than modelled
  (likely because overlapping STFT windows reduce the effective number of
  independent offsets). Recompute the compression figures before acting.

**Result: confirmed, strikingly.** K=20: sharp left edge at 16.3 dB, median
17.3 dB (predicted ~17.2), heavy pile-up at the edge with a thin tail. K=10:
edge 14.5, median 15.2. K=5: edge ~13.2 (predicted 12.5). The floor moves
with K in the predicted direction and roughly the predicted magnitude. The
pedestal is real and dominant.

---

## Check 2 — Noise or multipath? (the open question)

This is the one simulation cannot settle, and the one that decides whether the
rear-heading energy is a measurement artefact or physically real.

**Why it matters.** At 5 km the direct path is 13–22 dB below free space from
two-ray cancellation, so terrain scatter is genuinely competitive with it. Real
multipath into the back lobe is physically plausible. But the current metric
saturates: a strong scatterer and *nothing at all* both report ~17.2 dB. The
field observation "front ≈ back" is consistent with either, so it cannot
discriminate.

**The discriminator: is rear-heading energy at the collar's frequency?**

- Noise is not. A false alarm lands in a random bin somewhere in the ±1920 Hz
  search span.
- Multipath is. A scattered copy of the collar's pulse carries the collar's
  frequency and, within microseconds, its timing — far inside a 7.55 ms window.

**How.** Filter logged pulses to those recorded at headings ≥90° from the
rotation's final `bearing_deg`. Plot their `freq` against the collar's known
frequency.

**Decision:**

- Scattered across the search span → noise. The floor explains the symptom, and
  the changes in the companion document address it.
- Clustered at the collar's frequency → **real multipath.** The floor is still
  present and still worth fixing, but there is a genuine physical back-lobe
  signal on top of it, and no estimator removes that. It would then be worth
  revisiting antenna front/back (shortened elements, 14–17 dB) which the
  companion document currently argues against.
- Both, at different rates → likely, and the ratio tells you how much of each.

**Supporting tests if the logs are ambiguous:**

- Fly a rotation with the collar powered off. If rear headings report the same
  values, it is the floor.
- Repeat a rotation 200 m laterally. Multipath geometry shifts; a noise floor
  does not.

**Result: consistent with noise, not conclusive.** Only 20 % of detections
fall within ±200 Hz of the entered frequency; the rest scatter across the
full ±1920 Hz search span. Two caveats keep this from being a verdict: the
collar's true carrier sat ~+650 Hz off the entered frequency, so a window
centered on the entered frequency excludes genuine direct/multipath copies
(re-run against the empirical carrier), and `flight_checks.py` aggregates
all headings rather than only rear headings (≥90° from the known bearing).
The collar-OFF rotation in FIELD_TEST_PLAN.md is the decisive test. On the
available evidence no antenna front/back purchase is warranted, and the
companion document's changes address the symptom.

---

## Check 3 — Empirical front/back contrast

**Prediction.** Reported `snr` should vary by only ~1.2 dB between boresight and
180° at 3 dB peak SNR, ~3.0 dB at 6 dB. This is the number that started the whole
investigation.

**How.** For each rotation, compute each pulse's heading offset from that
rotation's final `bearing_deg`, then bin reported `snr` by offset (0/45/90/…).
Average across many rotations, grouped by `best_snr` as a proxy for range.

**Decision:** this is a direct measurement of the compression curve. Compare
against the table in the companion document. It also gives you, for free, the
**empirical installed antenna pattern as seen through the current metric** —
which is not the same as the antenna's real pattern, but is what the solve
actually sees.

**Result: consistent with the compression model, but under-sampled.** With no
bearing CSV, the Apr-10/11 heading dwells stand in for rotations. Genuine
detections rise 5–10 dB above floor on 1–2 front-lobe headings; every other
heading reports floor-level values with random frequencies, so the empirical
"pattern" through the current metric is flat-plus-one-bump — exactly the
degenerate shape the companion document predicts. A proper compression curve
needs the fixed-offset readout (or logged IQ) across full rotations.

---

## Check 4 — True PRI

**Why.** Lock lifetime is set entirely by PRI accuracy: ~50 ppm to hold a lock
across a rotation, ~5 ppm across a flight. The existing K=20 fold already relies
on roughly 190 ppm.

**How.** `start_time_seconds` across consecutive detections of the same tag, or
`ipi_analyzer` directly. Fit a line to pulse time vs pulse index; the slope is
the true PRI and the residual scatter is the jitter.

**Decision:**

- Within ~50 ppm of the entered `--tip` → locks will survive a rotation; changes
  3 and 7 in the companion document are safe as written.
- Worse than that → either measure and enter the true value per collar, or
  implement the phase-residual tracking loop so the lock corrects itself.

Also check whether the offset is stable per collar. If it is, it belongs in the
tag database rather than being re-measured each flight.

**Result: not measurable from the current logs.** Timestamps are per K-fold
cycle at 1 s resolution, and `start_time_seconds` is not in the logged pulse
line. From the raw IQ, coarse pulse timing at 3 km gave spacings of
2.005 s against an entered 2.000 — ~2500 ppm if taken at face value, far
worse than the 50 ppm lock budget, but the 7.55 ms window quantisation of
that measurement spans the difference. Measure properly with `ipi_analyzer`
on a live capture before trusting any lock across a rotation.

---

## Check 5 — Noise floor and self-interference

**Why.** At 146 MHz external noise normally dominates, so receiver noise figure
is not usually a range lever — unless the airframe is generating its own.

**How.** Log `noise_psd` (already in the pulse packet, and in the
`detection_status=3` no-detection reports) across a flight. Compare motors
disarmed, armed on the ground, and hovering. Also compare across headings within
a rotation.

**Decision:**

- Floor rises when armed → drone RF noise is costing range directly, and the
  likely coupling path is common-mode current on the feedline, which is also a
  prime suspect for the front/back loss. Both point at the same fix: a proper
  choke at 146 MHz (mix 43 ferrite, or a λ/4 sleeve balun ≈ 35 cm).
- Floor varies systematically with heading → the airframe or feedline is part of
  the antenna. Same fix.
- Flat across arming and heading → self-noise is not a factor; skip the choke work.

**Result: inconclusive, no red flags.** `noise_psd` varies ~5e-12 to 3e-11
across sessions with no systematic heading dependence within rotations. No
disarmed/ground baseline exists in this data set, so the armed-vs-disarmed
comparison is still outstanding — record one before the next campaign.

---

## Check 6 — If you have raw IQ

`PYTHON_VS_UAVRT_COMPARISON.md` notes the Python detector does not record IQ but
uavrt_detection writes it per segment. **If any flight has raw IQ, this is the
definitive test and needs no further flying.**

Implement `amplitude_at_known_pulse()` from the companion document, run it
offline over a recorded rotation, and compare the resulting 8 amplitudes against
the `snr` values the detector reported for the same rotation.

This settles simultaneously:

- whether the fixed-offset estimator really recovers the antenna's front/back
  contrast on real data, not just in simulation;
- whether the rear-heading energy is collar energy (Check 2) — because measuring
  at the locked bin and phase measures collar energy specifically and rejects
  noise by construction;
- what the **real installed antenna pattern** looks like, which is otherwise a
  separate field exercise.

If no IQ exists, consider adding an option to record it for a few rotations. The
data volume is modest against what it settles.

**Result: done — see the Results section at the top.** Raw IQ existed
(3/4/5 km captures plus one Apr-11 capture); `analyzer/iq_replay.py`
implements the replay. The fixed-offset estimator recovers a
range-monotonic amplitude the max-searched metric compresses to floor,
confirming the companion document's central claim on real data. No full
rotation with IQ exists yet, so the installed-pattern measurement is still
outstanding — record IQ for a few rotations next campaign.

---

## Priority

| Check | Cost | What it decides |
|---|---|---|
| 0 — slices/rotation | minutes, existing CSV | whether the analysis is valid at all |
| 2 — rear-heading freq | minutes, existing logs | noise vs multipath — the open question |
| 6 — offline IQ replay | hours, if IQ exists | everything above, definitively |
| 1 — SNR histogram | minutes | confirms the floor |
| 3 — SNR vs heading | an hour | measures the actual compression |
| 4 — true PRI | an hour | whether locks survive a rotation |
| 5 — noise floor | one flight | whether self-noise is costing range |

## What would change the conclusions

- **Check 0 shows 5–8 slices per rotation.** The companion document's central
  argument collapses; the bearing problem is elsewhere.
- **Check 2 shows rear energy at the collar frequency.** Multipath is real and
  additional antenna front/back becomes worth buying after all.
- **Check 1 shows no floor.** The compression figures are overstated; the
  fixed-offset readout still helps but by less than claimed.

None of these occurred: Check 0 held (once floor hits are filtered),
Check 2 showed scattered frequencies (consistent with noise, though not yet
conclusive — see the Check 2 detail), and Check 1 matched the predicted
floor to ~0.1 dB. The companion document's conclusions stand as written.

## Still outstanding after this analysis

See `FIELD_TEST_PLAN.md` for the capture campaign that closes these.

- **True PRI per collar** (Check 4) — needs `ipi_analyzer` on a live UDP
  stream (it has no raw-file input; an offline raw-IQ timing path is not yet
  implemented — a raw capture would have to be replayed into UDP); the
  50 ppm lock budget is unverified.
- **Disarmed noise baseline** (Check 5) — one ground session settles it.
- **Raw IQ over a full rotation** (Check 6) — settles the installed antenna
  pattern; the existing captures are single ground dwells.
- Bearing CSV logging (`tag_id,bearing_deg,r_squared,n_valid_slices,best_snr`)
  was not present in this data; future campaigns get Checks 0/3 for free.
