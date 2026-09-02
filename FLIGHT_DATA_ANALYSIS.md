# Flight Data Analysis Plan

Companion to `DETECTOR_AMPLITUDE_ANALYSIS.md`. That document is entirely
simulation; this one lists the checks against real flight data that confirm it,
falsify it, or settle the questions simulation cannot answer.

Run **Check 0 first.** It is a one-line histogram, and if it comes out wrong the
rest of the analysis is void.

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
