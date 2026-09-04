# Detector Analysis: Amplitude vs Detection

**Status: modelled, and since validated against flight data.** Every number
below comes from Monte-Carlo simulation of the detector's own algorithm at its
real parameters. The checks in `FLIGHT_DATA_ANALYSIS.md` have now been run
against the April 2026 PDC testing data (see its Results section): the SNR
floor was confirmed to ~0.1 dB at K=20, the rear-heading energy is consistent
with noise rather than multipath (not yet conclusive — needs rear-heading
filtering around the empirical carrier and a collar-OFF control), and offline
IQ replay confirmed the fixed-offset estimator recovers range-tracking
amplitude the current metric compresses to floor.

---

## Summary

`pulse_detector.py` reports a **detection statistic** and the bearing solve
consumes it as an **amplitude**. Those are different things, and the difference
is large enough to explain the front/back ambiguity on its own.

At K=20 the reported `snr_db` has a floor of **~17.2 dB with no tag present at
all**. A genuine 10 dB antenna front-to-back therefore appears as **~1.2 dB** at
long range. That is the observed symptom — 0° reading nearly the same as 180° —
and it is arithmetic, not antenna.

---

## What the detector does well

Worth stating, because the finding below is narrow and shouldn't be read as a
general criticism:

- **The W matrix is a proper matched filter.** Toeplitz construction with
  sub-bin interpolation, rectangular window matched to `tp`. This is
  near-optimal for on/off-keyed pulses; there is no meaningful processing gain
  left on the table here.
- **EVT/Gumbel thresholding is well-founded** and, importantly, *self-consistent*:
  `generate_evt_threshold` (line 594) runs the identical fold with the same
  `FOLD_LOCAL_RADIUS` as detection (line 771), so the threshold is correctly
  calibrated for the statistic it gates. No bug there.
- **K-fold integration gain is real and larger than the README claims.**
  Measured at Pfa=1e-4, Pd=0.9:

  | K | required per-pulse SNR | gain vs K=1 | `10log10(K)` |
  |---|---|---|---|
  | 1 | 19.37 dB | — | 0 |
  | 5 | 8.00 dB | 11.37 dB | 7.0 |
  | 10 | 5.07 dB | 14.30 dB | 10.0 |
  | 20 | 2.61 dB | 16.75 dB | 13.0 |
  | 40 | 0.46 dB | 18.90 dB | 16.0 |

  The operational gain exceeds `10log10(K)` because a single-pulse detector
  needs a punishing threshold margin against the exponential tail. Running
  K=20 is the right call; K=40 buys only 2.1 dB more for double the dwell.

- The multi-hypothesis rate-switch fold and its Occam preference are sound.

---

## The core finding

### Mechanism

```python
# fold_detect, line 773
best_scores = np.max(fold_scores, axis=1)
...
# line 959
snr_db = 10.0 * np.log10(best_scores[b] / noise_power[b])
```

`best_scores` is a **max over `search_range` fold offsets** (265 at
tp=15 ms / Fs=3840 / tip=2.0) of a K-term power sum, each term itself a **local
max over 3 windows** (`FOLD_LOCAL_RADIUS = 1`, line 43). The denominator is the
*single-window* noise estimate (line 789).

That is exactly right for deciding whether a pulse train is present. It is not
an amplitude. A max-selected statistic stops tracking signal strength the moment
the signal no longer wins the search: below that point the score is simply the
largest of 265 noise sums, and it does not depend on the tag at all.

### Geometry at the real settings

```
n_w  = ceil(0.015 * 3840) = 58 samples
n_ol = 29,  n_ws = 29   ->  one STFT window = 7.552 ms
PRI  = 2.0 s = 265 windows
K=5  : n_time = 1326, search_range = 265, dwell 10 s
K=20 : n_time = 5301, search_range = 265, dwell 40 s
```

### The floor

Median reported `snr_db` vs true per-pulse SNR:

| true per-pulse SNR | K=5 | K=20 |
|---|---|---|
| **no signal at all** | **12.5 dB** | **17.2 dB** |
| 0 dB | 12.6 | 17.4 |
| 3 dB | 12.9 | 18.4 |
| 6 dB | 14.0 | 20.2 |
| 10 dB | 17.2 | 23.4 |

### Consequent front/back compression

For a true 10 dB antenna front-to-back:

| front-lobe per-pulse SNR | reported gap, K=5 | reported gap, K=20 |
|---|---|---|
| 3 dB | 0.3 dB | 1.2 dB |
| 6 dB | 1.5 dB | 3.0 dB |
| 10 dB | 4.6 dB | 6.0 dB |

K=20 roughly doubles the recovered contrast versus K=5 — lower fold variance
means the max-over-265 inflates the pedestal less — but 1.2 dB of apparent
front/back on a 10 dB antenna is not usable.

### Second consequence: most headings never report

At K=20, pf=0.05, `detection_margin=0.90`, the base threshold lands at 17.7 dB
against a per-bin noise median of 17.2 dB. Pure noise clears it 1.2% of the
time. Per heading, at 3 dB peak SNR at boresight:

| heading | true signal | P(detect) |
|---|---|---|
| 0° (at the collar) | +3.0 dB | **79%** |
| 45° | −0.8 dB | 8% |
| 90° | −24.5 dB | 2% — noise floor |
| 135° | −10.0 dB | 1% — noise floor |
| **180°** | −7.0 dB | **2% — noise floor** |
| 225° | −10.0 dB | 1% |
| 270° | −24.5 dB | 2% |
| 315° | −0.8 dB | 8% |

**~1.0 of 8 headings reports.** At 6 dB peak it is 2.2 (0°, 45°, 315° — all on
the front lobe). At 9 dB, 3.1, still clustered around boresight.

The rear of the pattern never reports at any SNR the system operates at. The
evidence that would distinguish 0° from 180° is below threshold every time.

This is also why pf=0.05 is in use: it is an attempt to scrape together enough
slices to fit at all, not a considered sensitivity preference.

---

## Suggested improvements

Ordered by payoff. Change 1 is what makes 2 and 3 possible.

### 1. Add an amplitude readout that does not search

Once a detection has given you the tag's frequency bin and pulse phase, measure
at those known coordinates: no max over offsets, no local-max pooling, minus the
K×noise pedestal.

```python
def amplitude_at_known_pulse(power, freq_bin, pulse_indices, noise_power):
    """Absolute signal energy sum(power) - K*noise. May be negative — do not clamp."""
    idx = np.asarray(pulse_indices, dtype=np.int64)
    idx = idx[(idx >= 0) & (idx < power.shape[1])]
    if idx.size == 0:
        return float('nan')
    n = float(noise_power[freq_bin])
    return float(power[freq_bin, idx].sum()) - idx.size * n
```

`pulse_indices` is one row of the `(search_range, K)` array from
`build_hypothesis_indices()`, selected by the `best_offsets` recorded at lock.

Verified over 200k trials — the estimator is exactly linear in signal power
(`K·S`) and unbiased. The table shows estimate/N (noise units) for scale
only; the returned value is absolute power:

| true per-pulse SNR | estimate (K=5) |
|---|---|
| no signal | 0.002 |
| −7 dB | 0.998 |
| −4 dB | 1.999 |
| −1 dB | 3.989 |
| +3 dB | 9.966 |
| +6 dB | 19.855 |
| +10 dB | 49.917 |

Recovered front/back for a true 10 dB antenna: **10.0 dB at every SNR tested**,
against 0.4 dB and 1.5 dB from the current metric.

Two rules that matter:

- **Do not clamp at zero.** The negative excursions are what make it unbiased.
  Clamping re-introduces exactly the pedestal this removes.
- **Use `local_radius=0` on this path.** `FOLD_LOCAL_RADIUS=1` is correct for
  detection robustness but `max(S+N, N, N)` is not linear in S, so it
  re-introduces a signal-dependent bias.

A third rule, for the bearing consumer: **report absolute power, not a noise
ratio.** At 146 MHz the noise is external and the antenna samples it through
its own pattern, so an anisotropic noise field (a town or repeater on one
horizon) makes the per-heading noise floor pattern-shaped:
N(θ) ≈ N₀ + G(θ−φₛᵣᶜ)·Nₛᵣᶜ. Dividing by N(θ) then biases the bearing fit
toward or away from the noise source. Keep the noise *subtraction* (the
pedestal removal) but drop the division: send
`sum(power) − K·noise` in absolute units, with `noise` alongside as a
separate diagnostic. This is valid because the receiver runs AGC-off with a
fixed gain chain, so absolute power is comparable across headings within a
rotation. (Drone self-noise is exempt from the concern either way — it
rotates with the airframe, costing range but not skewing bearings.)

Keep `snr` as-is — the README documents it as deliberately uavrt-comparable.
Send this as an additional field.

### 2. Measure every heading, not only the ones that detect

Follows directly from 1: measurement has no threshold, so all 8 headings produce
a valid number every rotation — including the rear ones the solve has never seen.
This turns ~1 usable point per rotation into 8.

**Architecture prerequisite: the detector must survive the rotation.** The
current flight procedure (custom QGC → controller) starts a fresh detector at
each heading stop and kills it after K captures. That breaks everything above:

- A lock's phase (`t₀ + n·PRI`) is expressed in the detector's own sample
  timebase; a restarted process has a fresh counter, so the phase from the lock
  heading is meaningless at the next one. Every heading degenerates back to a
  cold-start max-over-offsets search.
- Escalating K over "already-buffered samples" (change 7) is impossible — the
  buffer dies with the process, so a deeper look means a full re-dwell.
- Startup settling and all pulses arriving during rotation between stops are
  discarded — paid-for integration thrown away.

Fix: run **one detector instance across the whole rotation** on an unbroken
sample stream, and have the controller send heading annotations ("heading 090
from t=X to t=Y") instead of start/stop. The readout stage bins per-pulse
amplitudes by heading using the annotations. A lock established at any heading
is then valid for the entire rotation — that is exactly the 50 ppm / 2.5 min
lock-lifetime budget in change 4. This is a controller/QGC protocol change,
not detector math, and it removes process churn at every stop.

(The alternative — expressing t₀ in absolute stream time so a restarted
detector can inherit it — smears timing correctness across three components
and still loses the buffer and the inter-stop pulses. Not recommended.)

### 3. Select the lock retrospectively

Do not commit to a lock in real time. Buffer the rotation, then treat each
heading's detected (bin, phase) as a candidate: re-measure all 8 headings at that
candidate and fit the antenna pattern. A real tag yields one candidate whose 8
amplitudes are pattern-shaped; a noise candidate yields scatter.

R² separation, K=20, 4000 trials:

| peak SNR | true lock R² (5/50/95) | false lock R² (5/50/95) | true locks kept |
|---|---|---|---|
| 0 dB | 0.37 / 0.79 / 0.95 | 0.15 / 0.40 / 0.79 | 50% |
| 3 dB | 0.72 / 0.92 / 0.98 | 0.16 / 0.41 / 0.79 | 89% |
| 6 dB | 0.88 / 0.97 / 0.99 | 0.16 / 0.40 / 0.79 | 99% |
| 10 dB | 0.93 / 0.99 / 1.00 | 0.15 / 0.41 / 0.78 | 100% |

Note a false lock still reaches R² ≈ 0.4 median — three free parameters against
eight points always fit *something*. Clean from 3 dB up; at 0 dB it is a
coin flip, and the honest output there is "no bearing".

### 4. Define what qualifies as a lock

A lock is three numbers plus a time reference: frequency bin, pulse phase
anchored to absolute time, and the true PRI. `frequency_hz`,
`start_time_seconds` and `predict_next` already carry the first two.

Qualification tests, cheapest first:

1. `score_ratio` ≥ ~3 (not the 1.3 `confidence_ratio` used for "probably real").
2. Frequency within tolerance of the entered tag frequency — ±2 kHz on first
   acquisition, ±100–200 Hz once that collar's offset is known.
3. Two consecutive cycles agreeing on bin and phase.
4. The pattern-fit test in change 3, retrospectively.

**Lock lifetime is set by PRI accuracy.** Phase error accumulates as
`(elapsed / PRI) × PRI_error`, and must stay under one 7.55 ms window:

| PRI error | lock valid for |
|---|---|
| 1 ms (500 ppm) | 15 s — unusable |
| 100 µs (50 ppm) | 2.5 min — covers a rotation |
| 10 µs (5 ppm) | 25 min — covers a flight |

The existing K=20 fold already spans 40 s and so already relies on ~190 ppm.
This is a tightening of an existing requirement, not a new one — but it is worth
measuring with `ipi_analyzer` rather than trusting the entered `--tip`. Better
still, track the phase residual across cycles and correct the PRI estimate so the
lock maintains itself.

### 5. Use the frequency prior

The detector searches all bins every cycle with no frequency constraint;
uavrt_detection uses a ±100 Hz adaptive lock (documented in
`PYTHON_VS_UAVRT_COMPARISON.md` as "Frequency tracking: None"). A ±200 Hz gate on
a ±1920 Hz search span keeps ~10% of the band — roughly 10× fewer false locks at
**zero sensitivity cost**, because it is a prior, not a higher bar.

### 6. Fit the pattern in linear power

Whatever consumes the amplitudes: convert to linear power, then fit
`p(θ) = A · pattern(θ − φ) + B`. In that space `B` is literally the noise
pedestal, which is what it physically is, and for any fixed φ the best `(A, B)`
is closed-form linear least squares — a brute-force φ scan with an analytic
solve, no Levenberg-Marquardt, no local minima, no step clamping.

Fitting a linear-power pattern shape to dB-valued data is dimensionally
inconsistent and degenerates to "flat plus one bump at boresight", discarding
precisely the rear-lobe structure that separates front from back.

Fit **absolute** power, per the third rule in change 1 — not SNR. If the
inputs are noise ratios, an anisotropic noise floor N(θ) leaks into the
pattern fit as a spurious lobe pointed at the noise source. With absolute
power, `B` absorbs any residual pedestal and the per-heading noise N(θ)
becomes a separate curve worth plotting on its own: structure in N(θ) is an
RFI-direction diagnostic, flat N(θ) validates the simpler ratio treatment
retrospectively.

### 7. Lock at K=20, measure at K=5

Measurement has no threshold to clear, so only the lock heading needs the full
40 s dwell. The other seven need ~10 s. Rotation time drops from ~5.3 minutes to
~1.5, and bearings improve at the same time.

First rotation: dwell K=20 per heading only until a candidate passes tests 1–3,
then drop to K=5. From the second rotation on, lock on the previous bearing.

**Large K as escalation, not default.** The integration window is K × PRI, and
"no detection" cannot be declared before the window closes — the whole premise of
a large-K fold is that the signal is invisible in shorter ones, so there is no
early exit on absence (a strong signal *can* fire early via a shorter sub-fold).
At the 2 s collar:

| K | min window per look | 8-heading rotation (windows only) |
|---|---|---|
| 5 | 10 s | ~80 s |
| 10 | 20 s | ~160 s |
| 20 | 40 s | ~320 s |
| 40 | 80 s | ~640 s (~11 min) |

A blanket K=40 lock therefore multiplies the "move on, tag's not here" verdict
time by 4–8×, which dominates aerial search cost. Instead, escalate: run K=10
continuously
(20 s verdicts); only when K=10 is negative *and* the search plan says the tag
may be near max range, extend the fold over the already-buffered samples to
K=20/K=40 — extending is more folding on a longer buffer, not a restart. K=40 is
an acquisition-range play (~+1–2 km); it never enters the rotation loop, because
per-heading measurement uses the fixed-offset readout at small K regardless.

### 8. On pf — leave it at 0.05, narrow the search instead

**Recommendation: do not tighten pf.** It is the one lever here that costs
range, and change 5 does the same job with the opposite sign.

Tightening it is cheap, but it is not free:

| pf | threshold | SNR for Pd=0.5 | range cost |
|---|---|---|---|
| 0.05 (current) | 17.73 dB | 1.32 dB | — |
| 0.01 | 17.86 dB | 1.66 dB | −2% |

0.34 dB of sensitivity. On the 1/d⁴ two-ray slope `d ∝ P^(-1/4)`, so
`10^(-0.34/40) = 0.981` — about **−1.9%**, ~95 m at 5 km. For scale, the
3-element Yagi upgrade rejected under "What not to change" is +3 dB for +19%
range; this gives away a tenth of that.

The threshold barely moves because the max over 116 bins has a tight Gumbel
distribution — which is also why the change buys so little. Monte-Carlo could
not resolve the tail past 1e-2 at 200k trials, so treat deeper settings as
"flat", not as exact figures.

**Change 5 is strictly the better lever.** Tightening pf raises the bar;
narrowing the search reduces how many chances noise gets. For a max-over-bins
statistic the threshold scales roughly as `ln(M / pf)`, so cutting M from ~116
bins to ~12 is a *larger* move than the pf change, in the direction that
**lowers** the threshold:

| lever | false-lock reduction | threshold | range |
|---|---|---|---|
| pf 0.05 → 0.01 | ~5× | +0.13 dB | −2% |
| ±200 Hz prior (change 5) | ~10× | ≈ −0.2 dB (est.) | ≈ +1% (est.) |

The prior's figures are an `ln(M/pf)` scaling, not measured — treat the
magnitude as soft and the sign as solid. Combined with the change 4
qualification tests, false locks are handled without touching sensitivity at
all.

**Why this matters more than it looks.** Changes 1–3 decouple *bearing* quality
from the detection threshold: measurement has no threshold, so rear headings
report regardless of pf. But **acquisition** range — "is there a tag out here at
all?" — still lives entirely on that threshold, and that is the K=40 long-range
play in change 7. Sensitivity spent on false-alarm control is spent exactly
where nothing else can recover it.

---

## What not to change

- **The antenna.** The RA-2AHS is published at 4 dBd / 10 dB F/B, and a NEC model
  of a full-size 2-element reproduces that within 0.1 dB (6.19 dBi, 10.5 dB F/B).
  It is performing to spec; the loss is downstream.
- **The Arrow 146-3 / a 3-element upgrade.** The design in
  `YAGI_ANTENNA_DESIGN.md` (1026/965/910 mm, 82 cm boom, 9.5 mm tube) models at
  9.15 dBi, 58° HPBW, **10.3 dB F/B** — beamwidth and gain improve, front/back
  does not. And the gain advantage over the RA-2AHS is +3.0 dB, not the +5–6 dBd
  the doc assumes (it rates the RA-2AK at 1–2 dBd; Telonics publishes 4 dBd).
  On the two-ray slope, range ∝ G^(1/4), so +3 dB is **+19% range**.
- **K.** You are at 20. K=40 is +2.1 dB for double the dwell.
- If more front/back is wanted later, it comes from *shortened* elements, not a
  longer boom: 61 cm loaded elements model at 5.49 dBi with 14.3 / 16.6 / 13.1 dB
  F/B across 146 / 147 / 148 MHz. Below ~50 cm the F/B bandwidth collapses.

## Range context

At 5 km and 122 m AGL the direct path is suppressed by two-ray cancellation at
grazing incidence (Γ ≈ −1 for both polarisations below the pseudo-Brewster angle):

| collar height | vs free space | breakpoint |
|---|---|---|
| 1.5 m | −12.8 dB | 366 m |
| 1.0 m | −16.3 dB | 244 m |
| 0.5 m | −22.3 dB | 122 m |

You are well past the breakpoint, i.e. on the 1/d⁴ slope, where **range ∝ G^(1/4)**.
This is why antenna gain is a weak range lever and why processing changes are
worth more than aperture. It is also why terrain scatter is competitive with the
direct path — see the multipath question in `FLIGHT_DATA_ANALYSIS.md`.

## Modelling caveats

- STFT windows were modelled as independent exponentials. Real windows overlap
  50%, so the effective number of independent offsets is lower and the true floor
  is somewhat below the quoted figures — not qualitatively different. The
  fixed-offset estimator in change 1 is exact regardless.
- The per-cycle max over frequency bins was modelled by order statistics over
  116 independent bins. Adjacent sub-bins are correlated through the W matrix, so
  this slightly overstates the threshold.
- Antenna patterns used the `kPatternDb` table, which is eyeballed from a
  free-space Telonics plot. The installed pattern with the airframe will differ,
  especially in the side nulls. Measuring it is the highest-value field task
  regardless of which changes are made.
