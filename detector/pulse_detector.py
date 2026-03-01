#!/usr/bin/env python3
"""
Simplified VHF Pulse Detector for Sirtrack Crystal-Oscillator Collars

Reads decimated IQ data from the airspyhf_zeromq + decimator pipeline via UDP.
Assumes crystal-oscillator timing (ti_pu=0, ti_pj=0) — no uncertainty or jitter.
Performs K=5 pulse folding and resets completely after each cycle.

Pipeline:
  airspyhf_zeromq_rx ---> [ZMQ PUB] ---> decimator ---> [UDP] ---> this script

The decimator must include this script's --port in its --ports list.

Algorithm (per cycle):
  1. Accumulate ~10 s of decimated IQ (enough for 5 pulse intervals)
  2. Compute STFT with window matched to pulse width (50% overlap)
  3. Fold the power spectrogram at the exact PRI (no M/J expansion)
  4. Threshold using Extreme Value Theory (Monte Carlo noise trials)
  5. Report detections, discard all state, repeat

Examples:
  python pulse_detector.py --tp 0.02 --tip 2.0
  python pulse_detector.py --tp 0.02 --tip 2.0 --port 10002 --pf 1e-5
  python pulse_detector.py --tp 0.02 --tip 2.0 --center-freq 148.515
"""

import argparse
import datetime
import signal
import socket
import struct
import sys
import time

import numpy as np
from scipy.linalg import toeplitz as scipy_toeplitz
from scipy.stats import gumbel_r

K = 5  # Always 5-fold

# Global flag for graceful shutdown
_should_stop = False

def _signal_handler(signum, frame):
    """Handle Ctrl+C gracefully."""
    global _should_stop
    _should_stop = True


# ---------------------------------------------------------------------------
# Pulse reporting (UDP → controller)
# ---------------------------------------------------------------------------

def send_pulse_udp(pulse_sock, dest_addr, tag_id, frequency_hz,
                   start_time_seconds, predict_next_start_seconds,
                   snr, stft_score, group_seq_counter, group_ind,
                   group_snr, detection_status, confirmed_status,
                   noise_psd):
    """Send a detected pulse to the controller as a UDPPulseInfo_T packet.

    The controller's UDPPulseReceiver expects 12 consecutive IEEE-754
    double-precision floats (96 bytes) in little-endian byte order.
    """
    packet = struct.pack('<12d',
                         float(tag_id),
                         float(frequency_hz),
                         start_time_seconds,
                         predict_next_start_seconds,
                         snr,
                         stft_score,
                         float(group_seq_counter),
                         float(group_ind),
                         group_snr,
                         float(detection_status),
                         float(confirmed_status),
                         noise_psd)
    pulse_sock.sendto(packet, dest_addr)


def send_heartbeat_udp(pulse_sock, dest_addr, tag_id):
    """Send a detector heartbeat to the controller.

    A heartbeat is a UDPPulseInfo_T packet with frequency_hz = 0.
    The controller recognises frequency_hz == 0 as a heartbeat rather
    than a real pulse (see PulseHandler::handlePulse).
    """
    send_pulse_udp(
        pulse_sock, dest_addr,
        tag_id=tag_id,
        frequency_hz=0,
        start_time_seconds=0.0,
        predict_next_start_seconds=0.0,
        snr=0.0,
        stft_score=0.0,
        group_seq_counter=0,
        group_ind=0,
        group_snr=0.0,
        detection_status=0,
        confirmed_status=0,
        noise_psd=0.0,
    )


# ---------------------------------------------------------------------------
# UDP packet decoding
# ---------------------------------------------------------------------------

def decode_timestamp(raw_8bytes):
    """Decode wall-clock timestamp from the decimator's first-sample header.

    The decimator encodes the timestamp as a complex<float> whose raw bytes
    represent uint32 seconds (real part) and uint32 nanoseconds (imag part).

    Returns:
        Timestamp in nanoseconds as an integer (uint64 semantics).
    """
    i_f, q_f = struct.unpack('<ff', raw_8bytes)
    sec  = struct.unpack('<I', struct.pack('<f', i_f))[0]
    nsec = struct.unpack('<I', struct.pack('<f', q_f))[0]
    return sec * 1_000_000_000 + nsec


# ---------------------------------------------------------------------------
# Spectral weighting matrix (matched-filter sub-bin resolution)
# ---------------------------------------------------------------------------

def build_weighting_matrix(n_w, Fs, zetas=None):
    """Build spectral weighting matrix W (uavrt_detection weightingmatrix.m).

    Creates a Toeplitz-based matched filter matrix providing sub-bin
    frequency resolution.  For the default zetas=[0, 0.5] each DFT bin
    is split into two half-bin positions, doubling frequency resolution
    compared to a plain n_w-point FFT.

    Args:
        n_w:   STFT window length (= pulse width in samples)
        Fs:    Sample rate in Hz
        zetas: Sub-bin shifts as fractions of DFT bin width.
               Default [0.0, 0.5] gives half-bin resolution.

    Returns:
        W:  (n_w, n_zetas*n_w) complex128 weighting matrix
        Wf: (n_zetas*n_w,) frequency vector in Hz, sorted ascending, DC-centred
    """
    if zetas is None:
        zetas = [0.0, 0.5]

    n_zetas = len(zetas)
    n = np.arange(n_w, dtype=np.float64)

    # Rectangular window — matches our pulse template
    window = np.ones(n_w, dtype=np.float64)

    toeplitz_blocks = []
    for zeta in zetas:
        # Frequency-shifted pulse template
        template = np.exp(2j * np.pi * zeta * n / n_w) * window

        # Normalised, DC-centred DFT
        Xs = np.fft.fftshift(np.fft.fft(template))
        Xs = Xs / np.linalg.norm(Xs)

        # Circulant Toeplitz matrix from the DFT vector
        T = scipy_toeplitz(Xs, np.concatenate(([Xs[0]], Xs[1:][::-1])))
        toeplitz_blocks.append(T)

    # Stack vertically then Fortran-order reshape to interleave columns
    stacked = np.vstack(toeplitz_blocks)                   # (n_zetas*n_w, n_w)
    W = stacked.reshape(n_w, n_zetas * n_w, order='F')     # (n_w, n_zetas*n_w)

    # Frequency vector: base DFT bins + sub-bin offsets, interleaved
    base_freqs = np.fft.fftshift(np.fft.fftfreq(n_w, d=1.0 / Fs))
    bin_width = Fs / n_w

    Wf = np.empty(n_zetas * n_w, dtype=np.float64)
    for i, zeta in enumerate(zetas):
        Wf[i::n_zetas] = base_freqs + zeta * bin_width

    # Sort by ascending frequency; reorder W columns to match
    sort_idx = np.argsort(Wf, kind='stable')
    Wf = Wf[sort_idx]
    W = W[:, sort_idx]

    return W, Wf


# ---------------------------------------------------------------------------
# STFT
# ---------------------------------------------------------------------------

def compute_stft_power(iq, n_w, n_ol, nfft, W=None):
    """Compute power spectrogram via overlapped short-time FFT.

    When W (spectral weighting matrix) is provided, the FFT is computed at
    window length n_w and then multiplied by W^H to produce sub-bin
    frequency resolution — matching uavrt_detection's incohsumtoeplitz path.
    When W is None, a zero-padded FFT of size nfft is used directly.

    Args:
        iq:   1-D complex64 array of IQ samples
        n_w:  STFT window length (= pulse width in samples)
        n_ol: overlap in samples
        nfft: FFT length (used only when W is None)
        W:    Optional (n_w, n_freq) spectral weighting matrix

    Returns:
        power:     (n_freq, n_windows) float32 power spectrogram, DC-centred
        n_windows: number of time windows
    """
    n_ws = n_w - n_ol
    n_windows = (len(iq) - n_ol) // n_ws
    n_freq = W.shape[1] if W is not None else nfft

    if n_windows < K:
        return np.empty((n_freq, 0), dtype=np.float32), 0

    # Rectangular window — matched to rectangular VHF pulse shape (matches
    # uavrt_detection's rectwin).  Maximises SNR for on/off keyed pulses.
    window = np.ones(n_w, dtype=np.float32)

    # Vectorised gather: (n_windows, n_w)
    starts = np.arange(n_windows) * n_ws
    idx = starts[:, None] + np.arange(n_w)[None, :]
    segments = iq[idx] * window

    if W is not None:
        # FFT at window length (no zero-pad), DC-centred: (n_w, n_windows)
        S = np.fft.fftshift(np.fft.fft(segments, n=n_w, axis=1), axes=1).T
        # Apply spectral weighting: W^H @ S → (n_freq, n_windows)
        scores = W.conj().T @ S
        return (scores.real**2 + scores.imag**2).astype(np.float32), n_windows
    else:
        # Zero-padded FFT fallback
        S = np.fft.fftshift(np.fft.fft(segments, n=nfft, axis=1), axes=1).T
        return (S.real**2 + S.imag**2).astype(np.float32), n_windows


# ---------------------------------------------------------------------------
# EVT Threshold Generation
# ---------------------------------------------------------------------------

def generate_evt_threshold(n_w, n_ol, nfft, samples_needed, N, K, pf,
                           W=None, n_trials=100, debug=False):
    """Generate detection threshold via Extreme Value Theory.

    Runs Monte Carlo simulation with synthetic complex Gaussian noise through
    the full STFT pipeline, matching uavrt_detection's approach.  This captures
    correlations introduced by the STFT window and overlap that would be missed
    by generating exponential power values directly.

    Args:
        n_w:             STFT window length
        n_ol:            STFT overlap
        nfft:            FFT size
        samples_needed:  IQ samples per segment
        N:               PRI in STFT windows
        K:               Number of pulse folds
        pf:              False alarm probability
        n_trials:        Number of Monte Carlo noise trials

    Returns:
        threshold: Detection threshold (scalar, normalised to unit-variance
                   complex Gaussian noise)
    """
    max_scores = []

    for _ in range(n_trials):
        # Unit-variance complex Gaussian noise (matching MATLAB's wgn)
        noise_iq = (np.random.randn(samples_needed).astype(np.float32)
                    + 1j * np.random.randn(samples_needed).astype(np.float32)) \
                   * np.float32(1.0 / np.sqrt(2.0))

        # Run through the exact same STFT pipeline as real data
        power, n_time = compute_stft_power(noise_iq, n_w, n_ol, nfft, W=W)
        if n_time == 0:
            continue

        max_start = n_time - (K - 1) * N
        if max_start <= 0:
            continue
        search_range = min(N, max_start)

        pulse_idx = (np.arange(search_range)[:, None]
                     + np.arange(K)[None, :] * N)

        fold_scores = np.sum(power[:, pulse_idx], axis=2)
        max_scores.append(np.max(fold_scores))

    if len(max_scores) < 10:
        return np.inf

    max_scores = np.array(max_scores)

    # Fit Gumbel distribution for maxima (matches MATLAB's evfit approach).
    # MATLAB uses evfit(-scores) which fits a Gumbel-minimum to the negated
    # scores — equivalent to fitting Gumbel-maximum (gumbel_r) to the scores.
    try:
        loc, scale = gumbel_r.fit(max_scores)
        threshold = gumbel_r.ppf(1.0 - pf, loc=loc, scale=scale)
        if debug:
            print(f'[DEBUG EVT] {len(max_scores)} trials  '
                  f'max_scores: min={np.min(max_scores):.4e}  '
                  f'max={np.max(max_scores):.4e}  mean={np.mean(max_scores):.4e}')
            print(f'[DEBUG EVT] Gumbel fit: loc={loc:.4e}  scale={scale:.4e}  '
                  f'threshold(pf={pf:.0e})={threshold:.4e}')
    except Exception:
        threshold = np.percentile(max_scores, 100.0 * (1.0 - pf))
        if debug:
            print(f'[DEBUG EVT] Gumbel fit failed, using percentile: {threshold:.4e}')

    return max(threshold, 0.0)


# ---------------------------------------------------------------------------
# Fold & detect
# ---------------------------------------------------------------------------

def fold_detect(power, N, pf, Fs, nfft, n_w, n_ol, samples_needed,
                evt_threshold_cache, W=None, Wf=None, debug=False):
    """Fold power spectrogram at PRI and detect pulses.

    With tipu=0, tipj=0 the only free parameter is the first-pulse offset
    within one PRI (0 … N-1 STFT windows).  For each frequency bin we sum
    K=5 power values spaced exactly N apart and compare against an
    EVT-derived threshold.

    Args:
        evt_threshold_cache: dict with 'threshold' key (or None to regenerate)

    Returns:
        List of (freq_hz, snr_db, phase_offset) sorted by SNR descending.
    """
    _, n_time = power.shape

    # Maximum valid first-pulse position
    max_start = n_time - (K - 1) * N
    if max_start <= 0:
        return []
    search_range = min(N, max_start)

    # Index matrix: (search_range, K) — each row is one candidate pattern
    pulse_idx = (np.arange(search_range)[:, None]
                 + np.arange(K)[None, :] * N)

    # Fold across all frequencies at once: (n_freq, search_range)
    fold_scores = np.sum(power[:, pulse_idx], axis=2)

    best_scores  = np.max(fold_scores, axis=1)       # (n_freq,)
    best_offsets = np.argmax(fold_scores, axis=1)

    # Noise estimation matching uavrt_detection (wfmstft.m lines 141-157):
    #   1. 3-window moving mean along time axis
    #   2. Median of smoothed power per frequency bin
    #   3. Mask bins where power > 10× median (exclude signal energy)
    #   4. Mean of unmasked bins (no ln(2) correction)

    # 3-point moving average along time (axis=1), edges use available data
    if n_time >= 3:
        padded = np.pad(power, ((0, 0), (1, 1)), mode='edge')
        mov_mean = (padded[:, :-2] + padded[:, 1:-1] + padded[:, 2:]) / 3.0
    else:
        mov_mean = power.copy()

    # Median of smoothed power per frequency bin
    med_per_freq = np.median(mov_mean, axis=1, keepdims=True)

    # Mask: raw power > 10× median (excludes signal and strong interference)
    outlier_mask = power > 10.0 * med_per_freq

    # Mean of unmasked power per frequency bin
    masked_power = power.copy()
    masked_power[outlier_mask] = np.nan
    noise_power = np.nanmean(masked_power, axis=1)

    # Fall back to median if all windows were masked in a bin
    all_masked = np.isnan(noise_power)
    if np.any(all_masked):
        noise_power[all_masked] = np.nanmedian(power[all_masked, :], axis=1)

    noise_power = np.maximum(noise_power, 1e-30)

    if debug:
        n_outlier = np.sum(outlier_mask)
        n_total = outlier_mask.size
        n_all_masked_bins = np.sum(all_masked) if np.any(all_masked) else 0
        print(f'[DEBUG NOISE] noise_power: min={noise_power.min():.6e}  '
              f'max={noise_power.max():.6e}  mean={noise_power.mean():.6e}  '
              f'std={noise_power.std():.6e}')
        print(f'[DEBUG NOISE] outlier mask: {n_outlier}/{n_total} '
              f'({100.0*n_outlier/n_total:.1f}%) masked  |  '
              f'{n_all_masked_bins} freq bins fully masked')

    # EVT threshold (Monte Carlo-derived, cached)
    # Generate or retrieve from cache
    if evt_threshold_cache.get('threshold') is None:
        print('  [Generating EVT threshold via 100 noise trials...]', flush=True)
        base_threshold = generate_evt_threshold(
            n_w, n_ol, nfft, samples_needed, N, K, pf, W=W, n_trials=100,
            debug=debug
        )
        if np.isinf(base_threshold):
            print(f'ERROR: Insufficient data for EVT threshold. '
                  f'Segment length ({n_time} samples) too short for K={K} folds.',
                  flush=True)
            return []
        evt_threshold_cache['threshold'] = base_threshold
    else:
        base_threshold = evt_threshold_cache['threshold']

    # Scale threshold by per-bin noise power (frequency-dependent)
    threshold = base_threshold * noise_power

    if debug:
        score_thresh_ratio = best_scores / np.maximum(threshold, 1e-30)
        print(f'[DEBUG THRESH] base_threshold={base_threshold:.6e}')
        print(f'[DEBUG THRESH] scaled threshold: min={threshold.min():.6e}  '
              f'max={threshold.max():.6e}  mean={threshold.mean():.6e}')
        print(f'[DEBUG THRESH] best_scores: min={best_scores.min():.6e}  '
              f'max={best_scores.max():.6e}  mean={best_scores.mean():.6e}')
        print(f'[DEBUG THRESH] score/threshold ratio: max={score_thresh_ratio.max():.4f}  '
              f'({"DETECT" if score_thresh_ratio.max() > 1.0 else "NO DETECT"})')

        freq_axis_dbg = Wf if Wf is not None else np.fft.fftshift(np.fft.fftfreq(nfft, d=1.0 / Fs))
        top5_idx = np.argsort(score_thresh_ratio)[::-1][:5]
        print(f'[DEBUG TOP5] Top 5 freq bins by score/threshold ratio:')
        for rank, idx in enumerate(top5_idx):
            print(f'  #{rank+1}: freq={freq_axis_dbg[idx]:+8.1f} Hz  '
                  f'score={best_scores[idx]:.4e}  thresh={threshold[idx]:.4e}  '
                  f'ratio={score_thresh_ratio[idx]:.4f}  '
                  f'noise={noise_power[idx]:.4e}')

    det_bins = np.where(best_scores > threshold)[0]
    if len(det_bins) == 0:
        return []

    # Frequency axis (DC-centred)
    freq_axis = Wf if Wf is not None else np.fft.fftshift(np.fft.fftfreq(nfft, d=1.0 / Fs))

    results = []
    for b in det_bins:
        snr_db = 10.0 * np.log10(best_scores[b] / (K * noise_power[b]))
        results.append((freq_axis[b], snr_db, int(best_offsets[b])))

    results.sort(key=lambda x: -x[1])

    # Merge peaks closer than min_sep bins (STFT sidelobe suppression)
    # The heuristic uses both an absolute minimum (15 bins) and a relative
    # separation (nfft // 4) to suppress sidelobes around strong narrowband
    # signals. This is a fixed function of nfft independent of signal strength.
    min_sep = max(15, nfft // 4)
    merged = []
    used_bins = []
    for freq_hz, snr_db, offset in results:
        b = int(np.argmin(np.abs(freq_axis - freq_hz)))
        if any(abs(b - ub) <= min_sep for ub in used_bins):
            continue
        merged.append((freq_hz, snr_db, offset))
        used_bins.append(b)
        # Report only the strongest detection
        if len(merged) >= 1:
            break

    return merged


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    global _should_stop

    ap = argparse.ArgumentParser(
        description='Sirtrack VHF Pulse Detector (K=5, EVT threshold)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument('--tp',  type=float, default=0.015,
                    help='Pulse duration in seconds (default: 0.015)')
    ap.add_argument('--tip', type=float, required=True,
                    help='Inter-pulse interval in seconds (e.g. 2.0)')
    ap.add_argument('--fs',  type=float, default=3840.0,
                    help='Decimated sample rate in Hz (default: 3840)')
    ap.add_argument('--port', type=int, default=10000,
                    help='UDP port for decimated IQ (default: 10000)')
    ap.add_argument('--pf',  type=float, default=1e-4,
                    help='False-alarm probability per cycle via EVT (default: 1e-4)')
    ap.add_argument('--center-freq', type=float, default=0.0,
                    help='Channel center frequency in MHz (display only)')
    ap.add_argument('--debug', action='store_true', default=False,
                    help='Print diagnostic info at each pipeline stage')
    ap.add_argument('--tag-id', type=int, default=0,
                    help='Tag ID sent with pulse reports (default: 0)')
    ap.add_argument('--freq', type=int, default=0,
                    help='Absolute tag frequency in Hz for pulse reports (default: 0)')
    ap.add_argument('--pulse-port', type=int, default=0,
                    help='UDP port to send detected pulses to (0 = disabled)')
    args = ap.parse_args()

    # --- Validate parameters ---
    if args.pf <= 0 or args.pf >= 1:
        sys.exit(f'Error: --pf must be in (0, 1), got {args.pf}')
    if args.tp <= 0:
        sys.exit(f'Error: --tp (pulse width) must be positive, got {args.tp}')
    if args.tp >= args.tip:
        sys.exit(f'Error: --tp ({args.tp}s) must be < --tip ({args.tip}s)')
    if args.tip <= 0:
        sys.exit(f'Error: --tip (inter-pulse interval) must be positive, got {args.tip}')
    if args.fs <= 0:
        sys.exit(f'Error: --fs (sample rate) must be positive, got {args.fs}')

    # --- STFT geometry ---
    n_w  = int(np.ceil(args.tp * args.fs))
    if n_w < 4:
        sys.exit(f'Error: STFT window too short (n_w={n_w}). '
                 f'Check --tp and --fs.')
    n_ol  = n_w // 2
    n_ws  = n_w - n_ol
    N     = int(np.floor(args.tip * args.fs / n_ws))

    # Validate N is sufficient for K-fold integration
    if N < K:
        min_tip = (K * n_ws) / args.fs
        sys.exit(
            f'Error: inter-pulse interval --tip is too small for '
            f'K={K} folds with --tp={args.tp}s and --fs={args.fs}Hz. '
            f'Increase --tip to at least {min_tip:.6f}s '
            f'(currently {args.tip}s).'
        )

    # Spectral weighting matrix (sub-bin matched filter, uavrt_detection style)
    W, Wf = build_weighting_matrix(n_w, args.fs)
    nfft = W.shape[1]   # output frequency bins (= 2*n_w for default zetas)

    if args.debug:
        col_norms = np.linalg.norm(W, axis=0)
        print(f'[DEBUG W] shape={W.shape}  dtype={W.dtype}')
        print(f'[DEBUG W] column norms: min={col_norms.min():.6f}  max={col_norms.max():.6f}  '
              f'mean={col_norms.mean():.6f}  std={col_norms.std():.6f}')
        print(f'[DEBUG W] max/min column norm ratio={col_norms.max()/col_norms.min():.4f}')
        print(f'[DEBUG W] row norms: min={np.linalg.norm(W, axis=1).min():.6f}  '
              f'max={np.linalg.norm(W, axis=1).max():.6f}')
        dead_cols = np.sum(col_norms < 1e-10)
        if dead_cols > 0:
            print(f'[DEBUG W] WARNING: {dead_cols} columns have near-zero norm!')

    samples_needed = n_ws * (K * N + 1) + n_ol
    seg_sec  = samples_needed / args.fs
    freq_res = args.fs / nfft
    fa_per_hour = (3600.0 / seg_sec) * args.pf

    print('=== Sirtrack VHF Pulse Detector ===')
    print(f'  Pulse width     {args.tp * 1000:.1f} ms')
    print(f'  Inter-pulse     {args.tip:.3f} s')
    print(f'  Folds (K)       {K}')
    print(f'  Sample rate     {args.fs:.1f} Hz')
    print(f'  STFT            window={n_w}  overlap={n_ol}  step={n_ws}')
    print(f'  PRI (N)         {N} STFT windows')
    print(f'  Freq bins       {nfft}  ({freq_res:.1f} Hz resolution, W matrix)')
    print(f'  Segment         {samples_needed} samples  ({seg_sec:.1f} s)')
    print(f'  Pf              {args.pf:.0e}  '
          f'(~{fa_per_hour:.2f} false alarms/hour)')
    print(f'  UDP port        {args.port}')
    if args.center_freq > 0:
        print(f'  Center freq     {args.center_freq:.6f} MHz')
    print()

    # --- UDP socket ---
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Configure receive buffer size with error checking
    recv_buf_bytes = 1 << 20
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, recv_buf_bytes)
        actual_buf = sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        if actual_buf < recv_buf_bytes:
            print(f'Warning: requested UDP receive buffer {recv_buf_bytes} bytes, '
                  f'but kernel applied {actual_buf} bytes', file=sys.stderr, flush=True)
    except OSError as e:
        print(f'Warning: failed to set UDP receive buffer to {recv_buf_bytes} bytes: {e}',
              file=sys.stderr, flush=True)

    sock.bind(('0.0.0.0', args.port))
    sock.settimeout(2.0)

    # Pulse reporting socket (separate from IQ receive)
    pulse_sock = None
    pulse_dest = None
    if args.pulse_port > 0:
        pulse_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        pulse_dest = ('127.0.0.1', args.pulse_port)
        print(f'  Pulse reports  → 127.0.0.1:{args.pulse_port}')
        if args.tag_id:
            print(f'  Tag ID          {args.tag_id}')
        if args.freq:
            print(f'  Tag frequency   {args.freq} Hz')

    # Install signal handler for graceful shutdown
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    buf_parts  = []
    buf_len    = 0
    seg_ts     = None
    cycle      = 0
    det_total  = 0
    run_start  = time.monotonic()
    last_detection_ts = None  # Track last detection time for inter-pulse delta

    # Packet continuity tracking
    last_packet_ts = None
    last_packet_samples = 0
    gap_zerofill_count = 0
    gap_reset_count = 0
    segment_has_gap = False

    # Single gap threshold (conservative), in nanoseconds
    gap_threshold_reset = args.tp * 2.0  # ≥ 2×tp: reset, < 2×tp: zero-fill
    gap_threshold_reset_ns = int(gap_threshold_reset * 1e9)

    # EVT threshold cache (regenerated if geometry changes)
    evt_threshold_cache = {}

    # Heartbeat timing — send a heartbeat every ~1 second
    heartbeat_interval = 1.0  # seconds
    last_heartbeat_time = time.monotonic()

    print('Gap handling:')
    print(f'  < {gap_threshold_reset*1000:.1f} ms: zero-fill missing samples')
    print(f'  ≥ {gap_threshold_reset*1000:.1f} ms: discard segment and reset buffer')
    print('Waiting for data ...\n')

    try:
        while not _should_stop:
            # ---- receive one UDP packet from the decimator ----
            try:
                data = sock.recv(65536)
            except socket.timeout:
                continue
            if len(data) < 16:
                continue

            # Decode timestamp with error handling
            try:
                ts = decode_timestamp(data[:8])
            except (struct.error, ValueError) as e:
                print(f'Warning: malformed timestamp in packet: {e}', file=sys.stderr, flush=True)
                continue

            if seg_ts is None:
                seg_ts = ts

            # Skip the 8-byte timestamp sample; rest is IQ payload
            try:
                payload = data[8:]
                n_samp = len(payload) // 8
                if n_samp == 0:
                    continue
                # Verify we can actually unpack this as IQ data
                iq_test = np.frombuffer(payload[:min(16, len(payload))], dtype=np.complex64)
            except (ValueError, struct.error) as e:
                print(f'Warning: malformed IQ data in packet: {e}', file=sys.stderr, flush=True)
                continue

            # Check for dropped packets via timestamp discontinuity
            if last_packet_ts is not None and last_packet_samples > 0:
                # All calculations in nanoseconds (exact integer arithmetic)
                expected_delta_ns = int(round(last_packet_samples / args.fs * 1e9))
                actual_delta_ns = ts - last_packet_ts
                gap_size_ns = actual_delta_ns - expected_delta_ns

                # Handle timestamp regression (negative gap) explicitly
                if gap_size_ns < 0:
                    gap_reset_count += 1
                    gap_ms = gap_size_ns / 1_000_000.0
                    print(f'\n*** NEGATIVE GAP (timestamp regression): {gap_ms:.1f} ms ***',
                          flush=True)
                    print(f'    Last packet ts={last_packet_ts} ns, '
                          f'current ts={ts} ns', flush=True)
                    print(f'    Expected delta={expected_delta_ns / 1_000_000.0:.1f} ms, '
                          f'got {actual_delta_ns / 1_000_000.0:.1f} ms\n', flush=True)

                    # Reset buffer and start a new segment from this packet
                    buf_parts.clear()
                    buf_len = 0
                    seg_ts = ts
                    segment_has_gap = False
                else:
                    # Calculate missing samples (only act if ≥1 sample missing)
                    missing_samples = int(round(gap_size_ns / (1e9 / args.fs)))

                    if missing_samples > 0:
                        gap_ms = gap_size_ns / 1_000_000.0

                        if gap_size_ns >= gap_threshold_reset_ns:
                            # LARGE GAP: Discard segment and reset
                            gap_reset_count += 1
                            print(f'\n*** GAP ≥ RESET THRESHOLD: {gap_ms:.1f} ms '
                                  f'(≥{gap_threshold_reset*1000:.1f} ms) ***',
                                  flush=True)
                            print(f'    Discarding {buf_len} buffered samples and resetting segment',
                                  flush=True)
                            print(f'    Expected packet after {expected_delta_ns / 1_000_000.0:.1f} ms, '
                                  f'got {actual_delta_ns / 1_000_000.0:.1f} ms\n', flush=True)

                            # Reset buffer
                            buf_parts.clear()
                            buf_len = 0
                            seg_ts = ts
                            segment_has_gap = False

                        else:
                            # GAP BELOW THRESHOLD: Zero-fill
                            gap_zerofill_count += 1
                            segment_has_gap = True

                            print(f'GAP < THRESHOLD: {gap_ms:.1f} ms '
                                  f'(< {gap_threshold_reset*1000:.1f} ms) - '
                                  f'zero-filling {missing_samples} samples', flush=True)

                            # Insert zeros to maintain continuity
                            zeros = np.zeros(missing_samples, dtype=np.complex64)
                            buf_parts.append(zeros)
                            buf_len += missing_samples

            # Store for next packet's continuity check
            last_packet_ts = ts
            last_packet_samples = n_samp

            iq = np.frombuffer(payload[:n_samp * 8],
                               dtype=np.complex64).copy()
            buf_parts.append(iq)
            buf_len += n_samp

            # ---- process when we have a full segment ----
            if buf_len < samples_needed:
                continue

            cycle += 1
            t0 = time.monotonic()

            # Send periodic heartbeat so the controller knows we're alive
            if pulse_sock is not None and (t0 - last_heartbeat_time) >= heartbeat_interval:
                send_heartbeat_udp(pulse_sock, pulse_dest, args.tag_id)
                last_heartbeat_time = t0

            segment = np.concatenate(buf_parts)[:samples_needed]
            buf_parts.clear()
            buf_len = 0

            # Check if this segment had gaps
            had_gap = segment_has_gap
            segment_has_gap = False

            power, n_win = compute_stft_power(segment, n_w, n_ol, nfft, W=W)

            if args.debug:
                seg_mag = np.abs(segment)
                print(f'[DEBUG STFT] IQ: len={len(segment)}  '
                      f'mean_mag={seg_mag.mean():.6e}  max_mag={seg_mag.max():.6e}')
                print(f'[DEBUG STFT] power: shape={power.shape}  '
                      f'min={power.min():.6e}  max={power.max():.6e}  '
                      f'mean={power.mean():.6e}  median={np.median(power):.6e}')
                pwr_per_freq = power.mean(axis=1)
                print(f'[DEBUG STFT] per-freq mean power: min={pwr_per_freq.min():.6e}  '
                      f'max={pwr_per_freq.max():.6e}  '
                      f'ratio={pwr_per_freq.max()/max(pwr_per_freq.min(), 1e-30):.1f}')

            # Invalidate EVT cache if geometry changed
            n_freq_cur = power.shape[0]
            n_time_cur = power.shape[1]
            if (evt_threshold_cache.get('n_freq') != n_freq_cur or
                evt_threshold_cache.get('n_time') != n_time_cur):
                evt_threshold_cache['threshold'] = None
                evt_threshold_cache['n_freq'] = n_freq_cur
                evt_threshold_cache['n_time'] = n_time_cur

            detections = fold_detect(power, N, args.pf, args.fs, nfft,
                                     n_w, n_ol, samples_needed,
                                     evt_threshold_cache, W=W, Wf=Wf,
                                     debug=args.debug)
            proc_ms = (time.monotonic() - t0) * 1000.0

            # Timestamp string (UTC) for the start of this segment
            # seg_ts is in nanoseconds; convert to seconds for datetime
            ts_str = ''
            current_ts = seg_ts  # Save before clearing
            if seg_ts and seg_ts > 1e9:
                ts_str = datetime.datetime.fromtimestamp(
                    seg_ts / 1e9, tz=datetime.timezone.utc
                ).strftime('%H:%M:%S')
            seg_ts = None

            # Append gap warning to output if segment had discontinuities
            gap_flag = ' [ZEROFILLED]' if had_gap else ''

            if detections:
                det_total += len(detections)

                # Calculate inter-pulse delta
                delta_str = ''
                if last_detection_ts is not None and current_ts is not None:
                    delta_s = (current_ts - last_detection_ts) / 1e9  # Convert ns to seconds
                    delta_str = f'  Δt={delta_s:.3f}s'
                if current_ts is not None:
                    last_detection_ts = current_ts

                for freq_hz, snr_db, offset in detections:
                    # Send pulse to controller via UDP if configured
                    if pulse_sock is not None:
                        start_time_s = current_ts / 1e9 if current_ts else time.time()
                        predict_next_s = start_time_s + args.tip
                        report_freq_hz = args.freq if args.freq else int(freq_hz)

                        send_pulse_udp(
                            pulse_sock, pulse_dest,
                            tag_id=args.tag_id,
                            frequency_hz=report_freq_hz,
                            start_time_seconds=start_time_s,
                            predict_next_start_seconds=predict_next_s,
                            snr=snr_db,
                            stft_score=snr_db,
                            group_seq_counter=cycle,
                            group_ind=0,
                            group_snr=snr_db,
                            detection_status=1,
                            confirmed_status=1,
                            noise_psd=0.0,
                        )

                    if args.center_freq > 0:
                        abs_mhz = args.center_freq + freq_hz / 1e6
                        print(f'[{cycle:4d} {ts_str}]  DETECTED  '
                              f'{abs_mhz:.6f} MHz  '
                              f'({freq_hz:+.1f} Hz)  '
                              f'SNR {snr_db:.1f} dB  '
                              f'{proc_ms:.0f} ms{delta_str}{gap_flag}')
                    else:
                        print(f'[{cycle:4d} {ts_str}]  DETECTED  '
                              f'{freq_hz:+.1f} Hz  '
                              f'SNR {snr_db:.1f} dB  '
                              f'{proc_ms:.0f} ms{delta_str}{gap_flag}')
            else:
                print(f'[{cycle:4d} {ts_str}]  no detection  '
                      f'{proc_ms:.0f} ms{gap_flag}')

    except KeyboardInterrupt:
        pass  # Handled by signal handler
    finally:
        elapsed = time.monotonic() - run_start
        print(f'\n--- Detection stopped after {cycle} cycles ({elapsed:.0f} s) ---', flush=True)
        print(f'  Detections:        {det_total}', flush=True)
        print(f'  Zero-filled gaps:  {gap_zerofill_count} (< {gap_threshold_reset*1000:.1f} ms)', flush=True)
        print(f'  Reset gaps:        {gap_reset_count} (≥ {gap_threshold_reset*1000:.1f} ms, segments discarded)', flush=True)
        print(f'  Total gap events:  {gap_zerofill_count + gap_reset_count}', flush=True)
        sock.close()
        if pulse_sock is not None:
            pulse_sock.close()


if __name__ == '__main__':
    main()
