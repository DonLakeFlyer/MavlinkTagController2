# Field Test Plan — Test Collar Capture Campaign

Companion to `FLIGHT_DATA_ANALYSIS.md` (results of the April 2026 analysis) and
`DETECTOR_AMPLITUDE_ANALYSIS.md` (the proposed changes). This plan covers the
next field session with a **real collar, not on a dog**, and lists exactly the
captures needed to convert the remaining simulation-only conclusions into
measured fact.

## Why this session

The April 2026 analysis confirmed the SNR floor and the fixed-offset
amplitude estimator on real IQ, and found the rear-heading energy consistent
with noise (not multipath) — though Check 2 remains inconclusive without
rear-heading filtering around the empirical carrier; capture 3 below is what
establishes that explanation. Those ground changes 1, 2, 4.1–4.3 and 6 —
implement them without waiting.

Two changes still rest on simulation-only numbers:

| Change | Missing evidence |
|---|---|
| 3 — retrospective lock / R² pattern fit | The **installed antenna pattern** has never been measured; the R² separation table used an eyeballed free-space pattern. |
| 7 — lock at K=20, measure at K=5 | The **true collar PRI** is unverified; the 50 ppm lock-lifetime budget is theory. |

This session exists to close those two, plus three cheap controls the April
data could not provide.

## Ground rules for every capture

- **Survey both ends.** Record collar GPS, collar height above ground (tape
  measure, not estimate), and drone GPS/altitude. Every conclusion downstream
  divides by geometry.
- **Record raw IQ generously.** 768 kHz cf32 is ~6 MB/s (~2 GB per 5-minute
  rotation). Storage is not the constraint — missing data is.
- **Write the tune frequency down** (or keep the `.json` sidecar). The April
  3/4/5 km captures had no sidecar and the tune convention had to be assumed.
- **Note collar identity, entered frequency, and expected tip** per capture.
  The April data contained a tip-2.0 collar where tip-1.333 was assumed, which
  cost a day of confusion.
- Enable **bearing CSV logging** (`tag_id,bearing_deg,r_squared,n_valid_slices,best_snr`)
  for the whole session so Checks 0/3 come free.

## Captures, in priority order

### 1. Raw IQ through full 8-heading rotations at known bearing (highest value)

**Settles:** the installed antenna pattern; validates change 3's R² fit
against known truth; measures the real front/back contrast through both the
current metric and the fixed-offset estimator.

**Procedure:**

1. Place the collar at a surveyed position, ~1 m height, clear line of sight.
2. Fly to ~2 km, hover at normal operating altitude (~100–120 m).
3. Record raw IQ as **one uninterrupted stream** across the full rotation
   (start/stop per heading resets the file-relative sample origin, so a fold
   offset locked in one dwell cannot be reused in the next — every file would
   self-lock, re-introducing the max-over-phase bias). Record heading changes
   as **sample-indexed annotations** (raw sample count at each change — the
   1 s controller log timestamps miss the ~7.6 ms STFT hop by orders of
   magnitude). Prerequisite: extend `iq_replay.py` with continuous
   replay/segmentation (one decimator/STFT pass, heading-binned measurements)
   before this capture can validate the installed pattern; splitting into
   per-dwell files loses decimator state at each boundary.
4. Repeat at ~4 km.
5. Repeat one rotation offset ~200 m laterally at the same range (multipath
   geometry control from Check 2's supporting tests).

**Analysis:** offline replay with `analyzer/iq_replay.py` per heading; fit
`p(θ) = A·pattern(θ−φ) + B` in linear power. True bearing is known from the
survey, so the fit error is measured, not estimated.

Also extract the **per-heading noise floor N(θ)** from the same captures
(per-bin noise estimate per dwell, AGC is off so absolute levels are
comparable). Compare its shape against the direction of the nearest known
RFI source (camp, road, town). Flat N(θ) → noise is isotropic and SNR-driven
fitting was merely inelegant; pattern-shaped N(θ) → the absolute-power
readout in change 1 is load-bearing, not optional.

### 2. Long continuous IQ of the collar — true PRI

**Settles:** lock lifetime (change 4's table, change 7's viability).

**Procedure:** ground capture, close range (strong signal), 10–15 minutes of
continuous raw IQ (guarantees 300+ pulses even for a tip-2.0 collar). No
flying needed.

**Analysis:** pulse-edge timing over ~300+ pulses gives PRI to a few ppm.
Target question: is the collar within 50 ppm of its nominal tip (lock survives
a rotation) or 5 ppm (survives a flight)? Also check whether PRI drifts over
the 10 minutes.

### 3. Collar-OFF rotation — floor control

**Settles:** the clean noise-floor baseline Check 2 wanted.

**Procedure:** identical to one rotation of capture 1, collar powered off.
Every detection reported is by definition a floor hit; its SNR / score_ratio /
frequency-offset distribution is the false-lock fingerprint to gate against.

### 4. Noise baseline: disarmed → armed → hover

**Settles:** Check 5 (drone self-interference), still open from April.

**Procedure:** one minute of `noise_psd` logging (or raw IQ) in each state:
motors disarmed, armed on the ground, hovering. Same location, back to back.

**Decision:** floor rises when armed → feedline choke work is worth doing;
flat → skip it.

### 5. Collar height ladder — two-ray verification

**Settles:** the h² height term used for the wild-dog range estimates
(currently theory: standing ~3 km, resting ~1.5–2 km at K=20).

**Procedure:** at a fixed range (~2 km), repeat a short capture with the
collar at ground level, ~0.2 m, ~0.5 m, and ~1.5–2 m (pole). Fixed-offset
amplitude vs height should follow 20 dB/decade if the two-ray model holds.

### 6. Frequency drift vs temperature (opportunistic)

**Settles:** whether per-collar frequency offset belongs in the tag database.

**Procedure:** repeat a short close-range capture with the collar cold
(shade/morning) and hot (sun/afternoon). The April data showed one collar at
~+650 Hz offset and another drifting +463 → +331 Hz across a morning; a
±200 Hz acquisition gate would have rejected the real collar.

## Time budget

| Capture | Flying? | Est. time |
|---|---|---|
| 1 — rotations at 2 km + 4 km + lateral offset | yes | 3 flights, ~45 min total |
| 2 — long IQ for PRI | no | 15 min ground |
| 3 — collar-off rotation | yes | 1 flight, ~10 min |
| 4 — noise baseline | minimal | 10 min |
| 5 — height ladder | yes (hover) | 1 flight, ~20 min |
| 6 — temperature drift | no | 2 × 5 min, hours apart |

Captures 2, 4, and 6 need no flight window and can fill gaps.

## What each outcome changes

- **Capture 1 shows a pattern-shaped fit with usable R² separation** →
  implement change 3 (retrospective lock) as designed.
- **Capture 1 shows the installed pattern badly degraded from free space**
  (feedline/airframe coupling) → the choke work from Check 5 moves up the
  priority list ahead of change 3.
- **Capture 1 shows pattern-shaped N(θ)** → the noise field is anisotropic;
  bearing fits must use absolute signal power (change 1's third rule), and
  historical SNR-based bearings near RFI sources are suspect.
- **Capture 2 shows ≤50 ppm** → change 7 is safe as written. **Worse** →
  implement the phase-residual PRI tracking loop first, or measure and enter
  per-collar tip values.
- **Capture 5 confirms 20 dB/decade** → the wild-dog operational guidance
  (fly dawn/dusk when the pack moves) stands quantitatively.
- **Capture 6 shows >±200 Hz thermal drift** → per-collar offsets must be
  re-acquired each flight, not cached in the tag database.

## Analysis tooling (already in place)

- `analyzer/flight_checks.py <data-root>` — log-driven checks, no venv needed.
- `analyzer/iq_replay.py` — offline IQ replay + fixed-offset amplitude
  (needs `./setup_venv.sh` once).
- Keep the session directory layout used in April (per-heading subdirectories,
  configs alongside logs) so the existing parsers work unchanged.
