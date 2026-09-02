#!/usr/bin/env python3
"""Check 6 from FLIGHT_DATA_ANALYSIS.md — offline IQ replay.

Loads a raw airspy-hf capture (complex_float32 at 768 kHz), shifts the tag
band to baseband, decimates to the detector rate (3840 Hz) using a streaming
mirror of the production decimator (decimator/src/main.cpp filter taps and
output phase), runs the detector's own STFT + K-fold search (the "current
metric"), and then applies the fixed-offset amplitude estimator
(amplitude_at_known_pulse) from DETECTOR_AMPLITUDE_ANALYSIS.md at the locked
(bin, phase), reporting absolute signal power and noise separately.

Usage:
  .venv/bin/python analyzer/iq_replay.py <capture.dat> --tag-freq 146.170 \
      [--tune-freq 146.180] [--tip 1.333] [--k 5] [--raw-fs 768000]

If --tune-freq is omitted, the pipeline convention tune = tag + 10 kHz is
assumed (airspyhf_zeromq_rx dc_offset).
"""

import argparse
import sys
from pathlib import Path

import numpy as np
from scipy.signal import lfilter

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "detector"))
from pulse_detector import (  # noqa: E402
    build_weighting_matrix, build_hypothesis_indices, compute_stft_power,
    _compute_fold_scores, FOLD_LOCAL_RADIUS,
)

DETECTOR_FS = 3840.0
CHUNK_SAMPLES = 1 << 22  # 4M samples (32 MB) per chunk keeps peak memory bounded


def design_lowpass(taps, cutoff):
    """Mirror of designLowpass in decimator/src/main.cpp (Hamming-windowed sinc)."""
    taps = max(taps, 3)
    if taps % 2 == 0:
        taps += 1
    n = np.arange(taps)
    m = n - (taps - 1) / 2.0
    window = 0.54 - 0.46 * np.cos(2.0 * np.pi * n / (taps - 1))
    center = np.abs(m) < 1e-6
    m_safe = np.where(center, 1.0, m)
    sinc = np.where(center, 2.0 * cutoff,
                    np.sin(2.0 * np.pi * cutoff * m_safe) / (np.pi * m_safe))
    b = window * sinc
    return b / b.sum()


class FirDecimStage:
    """Streaming mirror of the production FirDecimator (decimator/src/main.cpp):
    16q+1-tap Hamming sinc at 0.45/q cycles/sample, first output after q inputs."""

    def __init__(self, q):
        self.q = q
        self.b = design_lowpass(q * 16, 0.45 / q)
        self.zi = np.zeros(len(self.b) - 1, dtype=np.complex128)
        self.phase = q - 1

    def process(self, x):
        if len(x) == 0:
            return np.array([], dtype=np.complex128)
        y, self.zi = lfilter(self.b, 1.0, x, zi=self.zi)
        out = y[self.phase::self.q]
        self.phase = (self.phase - len(y)) % self.q
        return out


def amplitude_at_known_pulse(power, freq_bin, pulse_indices, noise_power):
    """Absolute signal energy sum(power) - K*noise. May be negative — do not clamp."""
    idx = np.asarray(pulse_indices, dtype=np.int64)
    idx = idx[(idx >= 0) & (idx < power.shape[1])]
    if idx.size == 0:
        return float('nan')
    n = float(noise_power[freq_bin])
    return float(power[freq_bin, idx].sum()) - idx.size * n


def estimate_noise(power):
    """Replicate fold_detect's noise estimate (moving-mean median, outlier mask)."""
    if power.shape[1] >= 3:
        padded = np.pad(power, ((0, 0), (1, 1)), mode='edge')
        mov_mean = (padded[:, :-2] + padded[:, 1:-1] + padded[:, 2:]) / 3.0
    else:
        mov_mean = power.copy()
    med_per_freq = np.median(mov_mean, axis=1, keepdims=True)
    masked = np.where(power > 10.0 * med_per_freq, np.nan, power)
    noise = np.nanmean(masked, axis=1)
    bad = np.isnan(noise)
    if np.any(bad):
        noise[bad] = np.nanmedian(power[bad, :], axis=1)
    return np.maximum(noise, 1e-30)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("capture")
    ap.add_argument("--tag-freq", type=float, required=True, help="MHz")
    ap.add_argument("--tune-freq", type=float, default=None,
                    help="MHz (default: tag + 0.010)")
    ap.add_argument("--raw-fs", type=float, default=768000.0)
    ap.add_argument("--tip", type=float, default=1.333)
    ap.add_argument("--tp", type=float, default=0.015)
    ap.add_argument("--k", type=int, default=5)
    ap.add_argument("--top", type=int, default=5, help="show top-N candidates")
    ap.add_argument("--lock-offset", type=int, default=None,
                    help="fold-offset index from an independent lock; skips "
                         "the biased same-data phase maximization")
    ap.add_argument("--lock-bin", type=int, default=None,
                    help="frequency bin from an independent lock (use with "
                         "--lock-offset)")
    args = ap.parse_args()

    if args.raw_fs <= 0:
        sys.exit(f"Error: --raw-fs must be positive (got {args.raw_fs}).")
    if args.tip <= 0:
        sys.exit(f"Error: --tip must be positive (got {args.tip}).")
    if args.tp <= 0:
        sys.exit(f"Error: --tp must be positive (got {args.tp}).")
    if args.tp >= args.tip:
        sys.exit(f"Error: --tp ({args.tp}s) must be < --tip ({args.tip}s), "
                 f"as the production detector requires.")
    if args.k < 2:
        sys.exit(f"Error: --k must be >= 2 (got {args.k}); the production "
                 f"detector normalizes K < 2 to 5.")
    if args.top < 1:
        sys.exit(f"Error: --top must be >= 1 (got {args.top}).")
    if args.lock_bin is not None and args.lock_offset is None:
        sys.exit("Error: --lock-bin requires --lock-offset; without it the "
                 "phase would still be maximized on the measured data.")

    tune_mhz = args.tune_freq if args.tune_freq is not None else args.tag_freq + 0.010
    shift_hz = (tune_mhz - args.tag_freq) * 1e6  # tag sits at -shift in raw stream

    n_total = Path(args.capture).stat().st_size // 8  # complex64
    dur = n_total / args.raw_fs
    print(f"Capture: {n_total} samples, {dur:.2f} s @ {args.raw_fs:.0f} Hz")
    print(f"Tune {tune_mhz:.6f} MHz, tag {args.tag_freq:.6f} MHz "
          f"-> mixing by {+shift_hz:.0f} Hz")

    # Mix tag band to DC, then decimate 768k -> 3840 (8 x 5 x 5).
    # Chunked so raw-side peak memory stays bounded on multi-GB captures
    # (the accumulated decimated output is 200x smaller); oscillator phase
    # and FIR state carry across chunk boundaries.
    fs = args.raw_fs / 200.0
    if abs(fs - DETECTOR_FS) > 1e-6:
        sys.exit(f"Error: hard-coded 8x5x5 decimation requires --raw-fs such "
                 f"that raw_fs/200 == {DETECTOR_FS:.0f} Hz (got {args.raw_fs:.0f}).")
    stages = [FirDecimStage(q) for q in (8, 5, 5)]
    cycles_per_sample = shift_hz / args.raw_fs
    phase0 = 0.0
    out = []
    with open(args.capture, "rb") as f:
        while True:
            x = np.fromfile(f, dtype=np.complex64, count=CHUNK_SAMPLES)
            if x.size == 0:
                break
            frac = (phase0 + cycles_per_sample * np.arange(x.size)) % 1.0
            y = x * np.exp(2j * np.pi * frac)
            phase0 = (phase0 + cycles_per_sample * x.size) % 1.0
            for st in stages:
                y = st.process(y)
            out.append(y.astype(np.complex64))
    if not out:
        sys.exit(f"Error: {args.capture} contains no complex64 samples.")
    iq = np.concatenate(out)
    print(f"Decimated: {len(iq)} samples @ {fs:.0f} Hz ({len(iq)/fs:.2f} s)")

    # Detector geometry (identical to pulse_detector.py)
    n_w = int(np.ceil(args.tp * fs))          # 58
    n_ol = n_w // 2                            # 29
    n_ws = n_w - n_ol
    N_exact = args.tip * fs / n_ws            # fractional PRI in STFT windows
    W, Wf = build_weighting_matrix(n_w, fs)
    power, n_time = compute_stft_power(iq, n_w, n_ol, n_w, W=W)
    # K pulses span (K-1)*N_exact windows; count offsets that fit, using the
    # same per-offset rounding as build_hypothesis_indices
    offsets = np.round(np.arange(int((n_time - 1) / N_exact) + 2) * N_exact)
    k_max = int(np.sum(offsets <= n_time - 1))
    K = min(args.k, k_max)
    print(f"STFT: {n_time} windows, PRI={N_exact:.2f} windows, K={K} "
          f"(max supported by capture: {k_max})")
    if K < 2:
        sys.exit("Capture too short for folding.")

    hyps = build_hypothesis_indices(N_exact, K, n_time)
    label, pulse_idx = hyps[0]
    noise = estimate_noise(power)

    # --- Current metric: max over offsets, local-max pooling, /noise ---
    fold_scores = _compute_fold_scores(power, pulse_idx,
                                       local_radius=FOLD_LOCAL_RADIUS)
    best_scores = np.max(fold_scores, axis=1)
    best_offsets = np.argmax(fold_scores, axis=1)
    if args.lock_offset is not None:
        # phase from an independent lock: unbiased fixed-offset measurement
        if not 0 <= args.lock_offset < pulse_idx.shape[0]:
            sys.exit(f"Error: --lock-offset must be in [0, "
                     f"{pulse_idx.shape[0] - 1}] for this capture "
                     f"(got {args.lock_offset}).")
        best_offsets = np.full_like(best_offsets, args.lock_offset)
        print(f"Measuring at externally supplied fold offset "
              f"{args.lock_offset} (fixed-offset, unbiased)")
    snr_metric = 10.0 * np.log10(best_scores / noise)

    order = np.argsort(snr_metric)[::-1]
    print(f"\n{'rank':>4s} {'freq offset':>12s} {'metric snr':>10s} "
          f"{'signal p':>12s} {'noise N':>12s} {'p/N':>8s}   per-fold amps (units of noise)")
    for r in range(min(args.top, len(order))):
        b = int(order[r])
        idx = pulse_idx[best_offsets[b]]
        amp = amplitude_at_known_pulse(power, b, idx, noise)
        ratio = amp / noise[b]
        # same in-range mask as amplitude_at_known_pulse
        idx_valid = idx[(idx >= 0) & (idx < n_time)]
        per_fold = (power[b, idx_valid] - noise[b]) / noise[b]
        pf_str = " ".join(f"{p:6.1f}" for p in per_fold)
        print(f"{r+1:4d} {Wf[b]:+10.1f} Hz {snr_metric[b]:10.1f} "
              f"{amp:12.4e} {noise[b]:12.4e} {ratio:8.2f}   [{pf_str}]")

    # --- Also measure at the exact entered tag frequency (bin nearest 0 Hz),
    # or at an externally supplied lock bin ---
    b0 = args.lock_bin if args.lock_bin is not None else int(np.argmin(np.abs(Wf)))
    if not 0 <= b0 < power.shape[0]:
        sys.exit(f"Error: --lock-bin must be in [0, {power.shape[0] - 1}] "
                 f"(got {b0}).")
    idx0 = pulse_idx[best_offsets[b0]]
    amp0 = amplitude_at_known_pulse(power, b0, idx0, noise)
    which = "lock bin" if args.lock_bin is not None else "entered tag freq"
    print(f"\nAt {which} (bin {b0}, {Wf[b0]:+.1f} Hz): "
          f"metric snr {snr_metric[b0]:.1f} dB, signal p {amp0:.4e}, "
          f"noise N {noise[b0]:.4e}, p/N {amp0 / noise[b0]:.2f}")

    # --- Distribution context: what does pure-noise fixed-offset give? ---
    amps = np.array([
        amplitude_at_known_pulse(power, b, pulse_idx[best_offsets[b]], noise)
        / noise[b]
        for b in range(power.shape[0])
    ])
    med = np.median(amps)
    p95 = np.percentile(amps, 95)
    print(f"\nFixed-offset p/N across all {len(amps)} bins (mostly noise): "
          f"median {med:.2f}, p95 {p95:.2f}  "
          f"(unbiased estimator -> median should sit near 0... but note these "
          f"are max-offset-selected, so slight positive bias remains)")


if __name__ == "__main__":
    main()
