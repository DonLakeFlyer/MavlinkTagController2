# Detector pipeline: one detection cycle

**Scope.** The signal processing `detector/pulse_detector.py` performs on one
segment of decimated IQ, from UDP datagrams to a per-cycle report. What happens
across cycles (lock candidates, per-slice re-measurement, bearing) is in
[COLLECTION_FLOW.md](COLLECTION_FLOW.md); the threshold layers and HIGH/LOW
classification are in [CONFIDENCE_PIPELINE.md](CONFIDENCE_PIPELINE.md); the
dual-rate hypothesis bank is in [RATE_SWITCH_DETECTOR.md](RATE_SWITCH_DETECTOR.md).

```
UDP (3840 Hz complex64)  →  ring  →  IQ stream  →  segment  →  STFT·W  →  fold  →  EVT  →  peaks  →  report
                        udp_receiver.py  iq_stream.py        (K·PRI)
```

The detector targets crystal-oscillator collars: pulse width `tp` and interval
`tip` are known, timing uncertainty (`tipu`, `tipj` in uavrt terms) is assumed
zero.

## 1. Input stream and gap handling

Datagrams from the decimator are `complex64` arrays whose first element is a
timestamp header — `uint32` seconds and `uint32` nanoseconds bit-cast into the
two float lanes (`decode_timestamp`) — followed by the IQ samples. They are
received on a dedicated thread into a bounded ring (`udp_receiver.py`) so a slow
fold or spectrogram dump cannot back up the kernel socket. Ring overflow is
counted (`rx_ring_dropped`) and surfaces downstream as a timestamp gap.

`iq_stream.py` keeps one continuous timeline indexed by absolute sample
position. An `ARM` records the stream index where a slice begins; nothing on
the control plane discards samples. Segments are cut back-to-back so
consecutive segments are sample-contiguous; history more than one segment
behind the read cursor is retired (`retired_samples` in `SESSION_END`).

Timestamp continuity between datagrams is checked against one threshold,
**2 × tp** (30 ms at the default 15 ms pulse):

| Gap | Action | Why |
| --- | --- | --- |
| < 2·tp | **Zero-fill** the missing samples; segment flagged `had_gap` / `[ZEROFILLED]`; the affected STFT windows are recorded as `gap_windows` and skipped by the PRI fit | A 15 ms hole in a 40 s segment costs ~0.006 dB; discarding the segment would cost the detection |
| ≥ 2·tp | **Barrier**: the hole is not filled, the segment cursor moves to the first post-gap sample, pre-gap samples stay as history | A missing pulse corrupts fold timing |

There is no "accept as-is" category. Both kinds are logged (`GAP_EVENT`
entries) and counted in `SESSION_END`.

## 2. Segment length

```
n_w   = ceil(tp · fs)            STFT window (58 samples at 15 ms / 3840 Hz)
n_ol  = n_w // 2                 50 % overlap
n_ws  = n_w − n_ol               step (29)
N     = tip · fs / n_ws          PRI in STFT steps (fractional, e.g. 264.8 at 2.0 s)
samples = n_ws · (K · ceil(N) + 1) + n_ol         (compute_segment_samples)
```

At K=20 and tip=2.0 s a segment is ≈ 40 s. With `--tip-secondary` the larger
of the two PRIs sets the length.

## 3. STFT with spectral weighting matrix W

Rectangular window matched to the on/off-keyed pulse (as uavrt `rectwin`).
Instead of zero-padding for sub-bin resolution, a Toeplitz matched-filter
matrix `W` (`n_w × 2·n_w`, from uavrt `weightingmatrix.m`) is built once from
frequency-shifted pulse templates at sub-bin shifts ζ ∈ {0, 0.5}:

```
S     = fftshift(fft(window, n_w))     per window
score = Wᴴ · S                          matched filter at each half-bin frequency
power = |score|²
```

Result: a power spectrogram `(n_freq = 2·n_w, n_time)` — 116 bins ≈ 33 Hz
apart across ±1920 Hz at the defaults. The frequency axis `Wf` is DC-centred.

## 4. Noise estimate per bin

Following uavrt `wfmstft.m`: 3-window moving mean along time → median per bin
→ mask bins whose power exceeds 10 × that median → mean of the unmasked bins
gives `noise_power[f]` in STFT-bin power units. Reports carry
`noise_psd = noise_power[f] / (Fs · n_w)` (W/Hz; `psd_scale` in `fold_detect`).

## 5. K-fold integration

For every frequency bin and every first-pulse offset within one PRI:

```
fold_score[f, o] = Σ_{k=0}^{K−1} power[f, o + round(k·N)]
```

The fractional `N` is carried exactly and rounded per fold, so alignment holds
at any K. Gain is ~`10·log10(K)` dB over a single pulse (13 dB at K=20). With a
secondary PRI, the fold runs a bank of rate-switch hypotheses and the best
hypothesis wins (`fold_multi_hypothesis`).

The search is masked to `ACQUISITION_SEARCH_HZ` = ±2000 Hz around the entered
tag offset when `--freq` and `--center-freq` are given.

## 6. EVT threshold

Rather than an analytic Gamma tail, the null distribution of the fold maximum is
learned empirically (`generate_evt_threshold`):

1. 100 unit-variance complex-Gaussian noise trials are pushed through the *same*
   STFT·W → fold → masked search as real data (this captures window overlap and
   W-matrix correlations).
2. The maximum score of each trial is recorded; a Gumbel (`scipy.stats.gumbel_r`)
   is fitted to the 100 maxima (as MATLAB `evfit`).
3. `base_threshold = gumbel_r.ppf(1 − pf)`, then × `--detection-margin` (0.90).
4. Per bin: `threshold[f] = base_threshold × noise_power[f]`.

The trial result depends only on geometry (`n_freq`, `n_time`, K, number of
hypotheses, number of searched bins), so it is cached in
`--threshold-cache-dir` under a name encoding those parameters
(`…-F<bins>-K<K>-Trials100-S2.pythreshold`) and regenerated only when the
geometry changes. Generation takes on the order of a second per hypothesis on
a Pi.

Caveat established from the Apr-11 data: the threshold is calibrated to
Gaussian noise of the *estimated* per-bin level. If the field noise is not
well described by that estimate the realised false-alarm rate can be far above
`pf` (88 % of dwells vs 5 % nominal on 2026-04-11). See the calibration item in
[docs/proposals/README.md](../proposals/README.md).

## 7. Peak selection

`det_bins = fold_score > threshold`. Peaks closer than `max(15, nfft // 4)`
bins (29 bins ≈ 960 Hz at the defaults) are merged and only the strongest is
kept, suppressing STFT sidelobes of one tag. The strongest surviving peak is the
cycle's detection; during a collection up to `MAX_LOCK_CANDIDATES` peaks are
also passed to the candidate bank (see COLLECTION_FLOW.md), but the acquisition
report is always single-peak.

For each detection:

```
score_ratio       = fold_score / threshold            drives HIGH/LOW and lock eligibility
snr_db            = 10·log10(fold_score / noise_power[f])   includes the K-fold gain
max_fold_fraction = max(fold_powers) / Σ fold_powers  > DOMINANT_FOLD_THRESHOLD (0.8) ⇒ LOW
fold_snrs_db      = per-fold 10·log10(power / noise)  logged (FOLDS entry), not gated
```

`snr_db` uses the single-window noise in the denominator (uavrt convention), so
it is comparable to uavrt output but is a *max-over-search* statistic: on noise
alone it sits at a K-dependent floor (~17 dB at K=20). It is not the amplitude
the bearing fit uses; that is `group_snr` from the locked measurement.

## 8. Report

Per cycle the detector emits one TTDP `PULSE` (detection_status 0/1) or
`NO_DETECTION` (3) carrying `frequency_hz`, `snr`, `score_ratio`, `noise_psd`,
`rate_state`, `start_time_seconds`, `predict_next_start_seconds`. When the cycle
was armed by a collection (`--control-port`) it is followed by `CYCLE_COMPLETE`;
free-running cycles send none. Heartbeats are separate header-only messages at
1 Hz.
Message layout: [shared/README.md](../../shared/README.md#ttdp-detector-protocol).

Console line and structured `.jsonl` (`shared/log_schema.py`) are written in
parallel:

```
[   1 18:51:43]  DETECTED  145.999768 MHz  (-231.7 Hz)  SNR 18.4 dB  score_ratio 1.599  noise 5.230e-12  171 ms  [LOW]
  [FOLDS] score_ratio=1.599  max_fold_fraction=0.31  per_fold_snr=[…] dB
[   2 18:52:23]  no detection  182 ms  best=146.609974 MHz SNR 16.2 dB ratio 0.964 noise 6.097e-12
```

`Δt` between consecutive reports is one cycle (≈ K·tip), not the collar PRI;
a doubled Δt means a missed cycle. True inter-pulse timing needs raw IQ
(`analyzer/ipi_analyzer.py`).

## Performance (defaults, K=5; figures carried over from the original detector README, platform not recorded)

| Stage | Per cycle |
| --- | --- |
| EVT threshold generation | ~0.8 s, first cycle per geometry (then cached) |
| STFT + W | ~5 ms |
| Fold | ~3 ms |
| Peak selection | ~1 ms |

At K=20 the STFT/fold cost scales with segment length (~4×) and remains far
below the cycle period. Memory: one power spectrogram per buffered slice
(~0.5 MB at the defaults), up to `MAX_BUFFERED_SLICES` = 64.

## Rationale

- **EVT rather than Gamma.** The trials push *Gaussian* noise through the real
  STFT / W-matrix / fold / masked-search geometry, so the fitted maximum
  accounts for window overlap, W-matrix correlation and the size of the search
  space — things an analytic Gamma tail on a single bin gets wrong. It does
  **not** adapt to non-Gaussian field noise (arcs, static); the realised
  false-alarm rate in such noise can be far above `pf`, as on 2026-04-11. That
  gap is the calibration item in [docs/proposals/README.md](../proposals/README.md).
- **W matrix rather than zero-padding.** Same sub-bin resolution with the
  matched-filter SNR gain, and identical to the uavrt reference so results are
  comparable.
- **Per-cycle fold search stays stateless.** Each cycle's fold is independent
  (no priori/posteriori tracking as in uavrt), which keeps a transient from
  poisoning later cycles. Cross-cycle state exists only in the lock-candidate
  bank and buffered slices described in COLLECTION_FLOW.md.
- **Why K matters.** K=20 buys +6 dB over K=5 and halves the scatter of the
  fixed-offset amplitude; the cost is a 40 s dwell per heading (320 s per
  8-heading rotation).

## Related

- [COLLECTION_FLOW.md](COLLECTION_FLOW.md), [CONFIDENCE_PIPELINE.md](CONFIDENCE_PIPELINE.md), [RATE_SWITCH_DETECTOR.md](RATE_SWITCH_DETECTOR.md), [FREQUENCY_RANGE.md](FREQUENCY_RANGE.md)
- [PYTHON_VS_UAVRT.md](PYTHON_VS_UAVRT.md) — where this differs from the MATLAB reference
- [2026-09_DETECTOR_AMPLITUDE_ANALYSIS.md](../analysis/2026-09_DETECTOR_AMPLITUDE_ANALYSIS.md) — the SNR-floor analysis behind §7
- `detector/tests/test_end_to_end.py`, `test_iq_stream.py`, `test_udp_receiver.py`
