#!/usr/bin/env python3
"""
VHF Pulse Detector for Crystal-Oscillator Radio Collars

Reads decimated IQ data from the airspyhf_zeromq + decimator pipeline via UDP.
Assumes crystal-oscillator timing (ti_pu=0, ti_pj=0) — no uncertainty or jitter.
Performs K-fold pulse integration (configurable via --k) and resets completely after each cycle.

Pipeline:
  airspyhf_zeromq_rx ---> [ZMQ PUB] ---> decimator ---> [UDP] ---> this script

The decimator must include this script's --port in its --ports list.

Algorithm (per cycle):
  1. Accumulate decimated IQ (enough for K pulse intervals)
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
import json
import os
import signal
import socket
import struct
import sys
import threading
import time
from typing import NamedTuple

# Structured logging (shared contract with analyzer)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'shared'))
from log_schema import (StructuredLogger, STARTUP, DETECTION, NO_DETECTION,
                        FOLDS, TIMING, NOISE_STATS, NOISE_ELEVATED,
                        GAP_EVENT, EVT_THRESHOLD, HYPOTHESIS, SESSION_END,
                        STFT_DEBUG, SPECTROGRAM_DUMP)

import numpy as np
from scipy.linalg import toeplitz as scipy_toeplitz
from scipy.stats import gumbel_r

K = 5  # Default fold count, overridden by --k
FOLD_LOCAL_RADIUS = 1  # Use local max over [idx-r, ..., idx+r] per fold
OCCAM_MARGIN = 0.05    # Prefer pure hypothesis if within 5% of switch winner

# Detection status values (mirrors TunnelProtocol.h detection_status field)
DETECTION_STATUS_SUBTHRESHOLD  = 0  # Subthreshold pulse
DETECTION_STATUS_SUPERTHRESHOLD = 1  # Superthreshold pulse
DETECTION_STATUS_CONFIRMED     = 2  # Confirmed pulse
DETECTION_STATUS_NO_DETECTION   = 3  # Searched but no pulse found

# Hypothesis encoding carried in PulseInfo_t.group_ind (uint16_t).
# The GCS interprets this to determine tag activity state.
#   0 = rate A only (resting)
#   1 = rate B only (moving)
#   2..K-1 = A→B switch at change-point c  (group_ind = 1 + c)
#   K..2K-3 = B→A switch at change-point c  (group_ind = K - 1 + c)
# For single-rate tags group_ind is always 0.
HYP_GROUP_IND_A = 0
HYP_GROUP_IND_B = 1

# If any single fold carries more than this fraction of the total K-fold
# score, the detection is likely a transient rather than a real pulse train.
# Matches uavrt_detection's selectpeakindex heuristic.
DOMINANT_FOLD_THRESHOLD = 0.8


class Detection(NamedTuple):
    """Single pulse detection result from fold_detect."""
    freq_hz: float          # Detected frequency (Hz, DC-centred)
    snr_db: float           # K-fold integrated SNR (dB)
    offset: int             # Best first-pulse STFT window index
    noise_psd: float        # Per-bin noise PSD (W/Hz)
    stft_score: float       # Fold score / PSD scale
    score_ratio: float      # Fold score / threshold (>1 = above threshold)
    hyp_label: str          # Winning hypothesis label (e.g. "A", "A_to_B_c2")
    fold_info: dict         # {'max_fold_fraction': float, 'fold_snrs': list[float]}


def hyp_label_to_group_ind(label, K=5):
    """Map a hypothesis label to the group_ind encoding.

    Returns (group_ind, 'last_rate') where last_rate is 'A' or 'B'
    indicating which PRI governs the *last* gap in the hypothesis
    (used for predict_next_start_seconds).
    """
    if label == 'A':
        return (HYP_GROUP_IND_A, 'A')
    if label == 'B':
        return (HYP_GROUP_IND_B, 'B')
    # Switch labels: "A_to_B_c{c}" or "B_to_A_c{c}"
    if label.startswith('A_to_B_c'):
        c = int(label.split('c')[1])
        return (1 + c, 'B')       # last gap uses rate B
    if label.startswith('B_to_A_c'):
        c = int(label.split('c')[1])
        return (K - 1 + c, 'A')   # last gap uses rate A
    # Fallback for unknown labels — treat as rate A
    return (HYP_GROUP_IND_A, 'A')

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
    try:
        pulse_sock.sendto(packet, dest_addr)
    except OSError as e:
        print(f'Warning: pulse send failed: {e}', file=sys.stderr, flush=True)


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
# Multi-hypothesis fold (rate-switch aware detection)
# ---------------------------------------------------------------------------

def build_hypothesis_indices(N_A, K, n_time, N_B=None):
    """Build pulse-index matrices for all rate-switch hypotheses.

    For a single rate (N_B is None), returns one hypothesis identical to the
    current fixed-PRI fold.  When N_B is provided, returns pure-A, pure-B,
    and single-switch change-point hypotheses (A→B and B→A at each interior
    change-point).

    N_A and N_B may be float (fractional PRI in STFT windows).  Each
    cumulative pulse offset is independently rounded to the nearest integer,
    keeping drift bounded to ±0.5 windows regardless of K.

    Args:
        N_A:    PRI spacing in STFT windows for rate A (int or float).
        K:      Fold count (number of pulses to sum).
        n_time: Number of STFT time windows available.
        N_B:    PRI spacing for rate B (None = single-rate mode).

    Returns:
        List of (label, pulse_idx) tuples where:
          label:     str — hypothesis name (e.g. "A", "B", "A_to_B_c2")
          pulse_idx: int64 ndarray of shape (search_range, K) — each row
                     gives the K integer window indices for one offset
                     candidate.  Fractional PRI is resolved by independently
                     rounding each cumulative offset.
        The list is empty if no hypothesis has a valid search range.
    """
    hypotheses = []

    def _make_schedule(spacings):
        """Convert a list of K-1 fractional spacings into rounded cumulative offsets."""
        offsets = np.zeros(K, dtype=np.int64)
        cumulative = 0.0
        for k in range(1, K):
            cumulative += spacings[k - 1]
            offsets[k] = int(np.round(cumulative))
        return offsets

    def _add_hypothesis(label, spacings):
        offsets = _make_schedule(spacings)
        span = int(offsets[-1])  # total span from first to last pulse
        max_start = n_time - span
        if max_start <= 0:
            return
        # Pure-rate hypotheses are periodic in t0, so searching one full PRI
        # is sufficient. Mixed (switch) hypotheses are not periodic in the
        # same way, so we must search all valid t0 values to avoid missing
        # transition alignments near the end of the segment.
        is_pure = np.allclose(spacings, spacings[0])
        if is_pure:
            first_N = int(np.round(spacings[0]))
            search_range = min(first_N, max_start)
        else:
            search_range = max_start
        t0 = np.arange(search_range, dtype=np.int64)
        pulse_idx = t0[:, None] + offsets[None, :]
        hypotheses.append((label, pulse_idx))

    # Convert to float so fractional arithmetic works uniformly
    N_A_f = float(N_A)

    # Pure A
    _add_hypothesis("A", [N_A_f] * (K - 1))

    if N_B is not None and N_B != N_A:
        N_B_f = float(N_B)
        # Pure B
        _add_hypothesis("B", [N_B_f] * (K - 1))

        # A→B at change-point c (first c gaps use N_A, rest use N_B)
        for c in range(1, K - 1):
            spacings = [N_A_f] * c + [N_B_f] * (K - 1 - c)
            _add_hypothesis(f"A_to_B_c{c}", spacings)

        # B→A at change-point c
        for c in range(1, K - 1):
            spacings = [N_B_f] * c + [N_A_f] * (K - 1 - c)
            _add_hypothesis(f"B_to_A_c{c}", spacings)

    return hypotheses


def fold_multi_hypothesis(power, hypotheses):
    """Fold power spectrogram across all hypotheses and return best per bin.

    Applies an Occam preference: if a switch hypothesis wins but the
    corresponding pure hypothesis (A or B, matching the switch's last rate)
    scores within OCCAM_MARGIN of the winner, the pure hypothesis is
    preferred.  This avoids spurious switch labels caused by sub-window
    phase alignment at high SNR.

    Args:
        power:      (n_freq, n_time) float32 power spectrogram.
        hypotheses: list from build_hypothesis_indices().

    Returns:
        best_scores:  (n_freq,) — best fold score per frequency bin.
        best_offsets: (n_freq,) — best starting offset per frequency bin.
        best_labels:  (n_freq,) — hypothesis label for the winning hypothesis.
    """
    n_freq = power.shape[0]
    best_scores = np.full(n_freq, -np.inf, dtype=np.float64)
    best_offsets = np.zeros(n_freq, dtype=np.int64)
    best_labels = np.empty(n_freq, dtype=object)
    best_labels[:] = ""

    # Track pure-hypothesis scores for Occam preference
    pure_scores = {}   # label -> (scores_array, offsets_array)

    for label, pulse_idx in hypotheses:
        # fold_scores shape: (n_freq, search_range)
        fold_scores = _compute_fold_scores(power, pulse_idx,
                                           local_radius=FOLD_LOCAL_RADIUS)
        hyp_best = np.max(fold_scores, axis=1)
        hyp_offsets = np.argmax(fold_scores, axis=1)

        if label in ('A', 'B'):
            pure_scores[label] = (hyp_best, hyp_offsets)

        improved = hyp_best > best_scores
        best_scores[improved] = hyp_best[improved]
        best_offsets[improved] = hyp_offsets[improved]
        best_labels[improved] = label

    # Occam preference: for bins where a switch hypothesis won, check if the
    # corresponding pure hypothesis scored within OCCAM_MARGIN.  Switch
    # hypotheses have extra degrees of freedom (the change point), so they
    # can overfit sub-window alignment at high SNR.
    if pure_scores:
        for b in range(n_freq):
            lbl = best_labels[b]
            if lbl in ('A', 'B', ''):
                continue  # already pure or empty
            # Determine which pure hypothesis corresponds to this switch's
            # last rate (the one used for next-pulse prediction).
            if lbl.startswith('A_to_B'):
                pure_lbl = 'B'
            elif lbl.startswith('B_to_A'):
                pure_lbl = 'A'
            else:
                continue
            if pure_lbl not in pure_scores:
                continue
            pure_sc, pure_off = pure_scores[pure_lbl]
            if best_scores[b] > 0 and pure_sc[b] >= best_scores[b] * (1.0 - OCCAM_MARGIN):
                best_scores[b] = pure_sc[b]
                best_offsets[b] = pure_off[b]
                best_labels[b] = pure_lbl

    return best_scores, best_offsets, best_labels



def _compute_fold_scores(power, pulse_idx, local_radius=0):
    """Compute fold scores with optional local-max pooling per fold index.

    Args:
        power:        (n_freq, n_time) power spectrogram.
        pulse_idx:    (search_range, K) integer fold window indices.
        local_radius: Radius in STFT windows for local max pooling.

    Returns:
        (n_freq, search_range) fold scores.
    """
    n_time = power.shape[1]
    idx0 = np.clip(pulse_idx, 0, n_time - 1)
    local_powers = power[:, idx0]
    if local_radius <= 0:
        return np.sum(local_powers, axis=2)

    for d in range(1, local_radius + 1):
        idx_m = np.clip(pulse_idx - d, 0, n_time - 1)
        idx_p = np.clip(pulse_idx + d, 0, n_time - 1)
        local_powers = np.maximum(local_powers, power[:, idx_m])
        local_powers = np.maximum(local_powers, power[:, idx_p])
    return np.sum(local_powers, axis=2)


def _local_peak_powers_1d(power_row, pulse_indices, local_radius=0):
    """Return per-fold powers with optional local-max pooling in time."""
    n_time = power_row.shape[0]
    idx0 = np.clip(np.asarray(pulse_indices, dtype=np.int64), 0, n_time - 1)
    powers = power_row[idx0]
    if local_radius <= 0:
        return powers

    for d in range(1, local_radius + 1):
        idx_m = np.clip(idx0 - d, 0, n_time - 1)
        idx_p = np.clip(idx0 + d, 0, n_time - 1)
        powers = np.maximum(powers, power_row[idx_m])
        powers = np.maximum(powers, power_row[idx_p])
    return powers


def compute_segment_samples(n_ws, n_ol, K, N_A, N_B=None):
    """Compute IQ samples needed for one detection segment.

    The segment must contain enough STFT windows for the longest hypothesis
    span *plus* a full search range (one PRI of the largest rate).  For
    pure-A that total is K*N_A windows; for mixed hypotheses the worst case
    is K * max(N_A, N_B).

    N_A and N_B may be float (fractional PRI).  The ceiling is used to
    ensure the segment is large enough after rounding.

    Args:
        n_ws: STFT window step in samples.
        n_ol: STFT overlap in samples.
        K:    Fold count.
        N_A:  PRI spacing for rate A (in STFT windows, int or float).
        N_B:  PRI spacing for rate B (None = single-rate).

    Returns:
        samples_needed: int — number of IQ samples per segment.
    """
    max_N = N_A
    if N_B is not None and N_B != N_A:
        max_N = max(N_A, N_B)
    # Use ceil so fractional PRI doesn't truncate the segment
    max_N_int = int(np.ceil(max_N))
    return n_ws * (K * max_N_int + 1) + n_ol


# ---------------------------------------------------------------------------
# STFT
# ---------------------------------------------------------------------------

def compute_stft_power(iq, n_w, n_ol, nfft, W=None, min_windows=1):
    """Compute power spectrogram via overlapped short-time FFT.

    When W (spectral weighting matrix) is provided, the FFT is computed at
    window length n_w and then multiplied by W^H to produce sub-bin
    frequency resolution — matching uavrt_detection's incohsumtoeplitz path.
    When W is None, a zero-padded FFT of size nfft is used directly.

    Args:
        iq:          1-D complex64 array of IQ samples
        n_w:         STFT window length (= pulse width in samples)
        n_ol:        overlap in samples
        nfft:        FFT length (used only when W is None)
        W:           Optional (n_w, n_freq) spectral weighting matrix
        min_windows: Minimum number of STFT windows required (default: 1)

    Returns:
        power:     (n_freq, n_windows) float32 power spectrogram, DC-centred
        n_windows: number of time windows
    """
    n_ws = n_w - n_ol
    n_windows = (len(iq) - n_ol) // n_ws
    n_freq = W.shape[1] if W is not None else nfft

    if n_windows < min_windows:
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
                           fold_offsets=None, W=None, n_trials=100,
                           debug=False, hypotheses=None):
    """Generate detection threshold via Extreme Value Theory.

    Runs Monte Carlo simulation with synthetic complex Gaussian noise through
    the full STFT pipeline, matching uavrt_detection's approach.  This captures
    correlations introduced by the STFT window and overlap that would be missed
    by generating exponential power values directly.

    When *hypotheses* is provided (from build_hypothesis_indices), the fold
    search spans all hypotheses — matching the detection-time search space
    so that the EVT threshold correctly controls the false alarm rate.

    Args:
        n_w:             STFT window length
        n_ol:            STFT overlap
        nfft:            FFT size
        samples_needed:  IQ samples per segment
        N:               PRI in STFT windows (used only when hypotheses is None)
        K:               Number of pulse folds
        pf:              False alarm probability
        n_trials:        Number of Monte Carlo noise trials
        hypotheses:      Optional list from build_hypothesis_indices().
                         When None, falls back to single-rate fold using N.

    Returns:
        (threshold, mu, sigma) where:
          threshold: Detection threshold (scalar, normalised to 1W/bin)
          mu:        Gumbel location parameter (or None if fit failed)
          sigma:     Gumbel scale parameter (or None if fit failed)
    """
    max_scores = []

    for _ in range(n_trials):
        # Unit-variance complex Gaussian noise (matching MATLAB's wgn)
        noise_iq = (np.random.randn(samples_needed).astype(np.float32)
                    + 1j * np.random.randn(samples_needed).astype(np.float32)) \
                   * np.float32(1.0 / np.sqrt(2.0))

        # Run through the exact same STFT pipeline as real data
        power, n_time = compute_stft_power(noise_iq, n_w, n_ol, nfft, W=W,
                                           min_windows=K)
        if n_time == 0:
            continue

        if hypotheses is not None and len(hypotheses) > 0:
            # Multi-hypothesis fold: search all hypotheses
            best_scores, _, _ = fold_multi_hypothesis(power, hypotheses)
        else:
            # Single-rate fold using precomputed offsets (handles fractional PRI)
            _fo = fold_offsets if fold_offsets is not None else np.arange(K) * N
            max_start = n_time - _fo[-1]
            if max_start <= 0:
                continue
            effective_period = int(_fo[1]) if len(_fo) > 1 else int(N)
            search_range = min(effective_period, max_start)

            pulse_idx = np.arange(search_range)[:, None] + _fo[None, :]

            fold_scores = _compute_fold_scores(power, pulse_idx,
                                               local_radius=FOLD_LOCAL_RADIUS)
            best_scores = np.max(fold_scores, axis=1)

        # Normalize to 1W/bin reference: divide by median per-bin noise power.
        # This matches uavrt_detection's medPowAllFreqBins=1 calibration so
        # that base_threshold × noise_power gives the correct per-bin threshold.
        noise_per_bin = np.mean(power, axis=1)
        med_noise = np.median(noise_per_bin)
        if med_noise > 0:
            max_scores.append(np.max(best_scores) / med_noise)

    if len(max_scores) < 10:
        return np.inf, None, None

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
        return max(threshold, 0.0), loc, scale
    except Exception:
        threshold = np.percentile(max_scores, 100.0 * (1.0 - pf))
        if debug:
            print(f'[DEBUG EVT] Gumbel fit failed, using percentile: {threshold:.4e}')
        return max(threshold, 0.0), None, None


# ---------------------------------------------------------------------------
# EVT threshold disk cache
# ---------------------------------------------------------------------------

def _evt_cache_path(cache_dir, N_A, K, N_B=None, n_hypotheses=1, n_trials=100):
    """Build cache filename for EVT threshold parameters.

    Uses a naming scheme that is distinct from the legacy single-rate format
    (N...-M...-J...-K...-Trials....pythreshold) so that old and new detectors
    can coexist in the same cache directory without collision.

    N_A and N_B should be the *fractional* PRI values (N_exact, N_B_exact),
    not the integer floors.  This prevents cache collisions between
    configurations whose fractional PRIs differ but share the same floor
    (e.g. N_exact=264.83 vs 264.96 both floor to 264).  The Nb and H
    fields ensure single-rate and dual-rate configurations never share
    entries, since dual-rate EVT thresholds are computed over a larger
    multi-hypothesis search space.

    Legacy format (still read by old code):
        N265.000000-M0.000000-J0.000000-K5.000000-Trials100.pythreshold

    New format:
        N264.827586-Nb200.413793-H8-K5.000000-Trials100.pythreshold
        N264.827586-Nb0-H1-K5.000000-Trials100.pythreshold   (single-rate)
    """
    nb_str = f'{float(N_B):.6f}' if N_B is not None else '0'
    return os.path.join(cache_dir,
                        f'N{float(N_A):.6f}-Nb{nb_str}-H{n_hypotheses}'
                        f'-K{float(K):.6f}-Trials{n_trials}.pythreshold')


def load_evt_cache(cache_dir, N_A, K, N_B=None, n_hypotheses=1, n_trials=100):
    """Load Gumbel mu/sigma from disk. Returns (mu, sigma) or (None, None)."""
    if not cache_dir:
        return None, None
    path = _evt_cache_path(cache_dir, N_A, K, N_B=N_B,
                           n_hypotheses=n_hypotheses, n_trials=n_trials)
    try:
        with open(path, 'r') as f:
            values = [float(line.strip()) for line in f if line.strip()]
        if len(values) >= 2:
            return values[0], values[1]
    except (OSError, ValueError):
        pass
    return None, None


def save_evt_cache(cache_dir, N_A, K, mu, sigma, N_B=None,
                   n_hypotheses=1, n_trials=100):
    """Save Gumbel mu/sigma to disk cache."""
    if not cache_dir or mu is None or sigma is None:
        return
    os.makedirs(cache_dir, exist_ok=True)
    path = _evt_cache_path(cache_dir, N_A, K, N_B=N_B,
                           n_hypotheses=n_hypotheses, n_trials=n_trials)
    try:
        with open(path, 'w') as f:
            f.write(f'{mu:.15e}\n')
            f.write(f'{sigma:.15e}\n')
        print(f'  [Saved EVT cache: {path}]', flush=True)
    except OSError as e:
        print(f'WARNING: Failed to write EVT cache {path}: {e}', flush=True)


# ---------------------------------------------------------------------------
# Fold & detect
# ---------------------------------------------------------------------------

def fold_detect(power, N, pf, Fs, nfft, n_w, n_ol, samples_needed,
                evt_threshold_cache, fold_offsets=None, W=None, Wf=None,
                debug=False, detection_margin=0.90,
                hypotheses=None, N_B=None,
                N_A_exact=None, N_B_exact=None,
                slog=None):
    """Fold power spectrogram and detect pulses.

    Supports both single-rate and multi-hypothesis rate-switch detection.
    When *hypotheses* is provided, folds across all hypotheses.  When
    hypotheses is None, falls back to fold_offsets-based (or basic
    N-stride) single-rate fold.

    Each detection includes a ``max_fold_fraction`` diagnostic — the
    largest single fold's fraction of the total K-fold score.  This is
    used by the caller to downgrade confidence (not to discard
    detections).  A value above ``DOMINANT_FOLD_THRESHOLD`` indicates a
    single transient dominates the score.

    Args:
        power:               (n_freq, n_time) float32 power spectrogram.
        N:                   PRI in STFT windows for rate A.
        pf:                  False alarm probability.
        Fs:                  Sample rate in Hz.
        nfft:                FFT size (number of frequency bins).
        n_w:                 STFT window length.
        n_ol:                STFT overlap.
        samples_needed:      IQ samples per segment.
        evt_threshold_cache: dict — in-memory / disk cache for EVT params.
        W:                   Spectral weighting matrix (or None).
        Wf:                  Frequency axis vector (or None).
        debug:               Print diagnostic info.
        hypotheses:          List from build_hypothesis_indices() (or None).
        N_B:                 Secondary PRI spacing (for EVT cache key; None
                             if single-rate).
        N_A_exact:           Fractional PRI for rate A (for EVT cache key).
                             Defaults to N if not provided.
        N_B_exact:           Fractional PRI for rate B (for EVT cache key).
                             Defaults to N_B if not provided.

    Returns:
        (detections, noise_psd, best_candidate) where:
          detections: list of (freq_hz, snr_db, offset, noise_psd, stft_score,
                      score_ratio, hyp_label, fold_info) sorted by SNR
                      descending.  fold_info is a dict with
                      'max_fold_fraction' (float) and 'fold_snrs'
                      (list of per-fold SNRs in dB).
          noise_psd:  float — median noise PSD across frequency bins when no
                      detections are found; None when detections are present;
                      NaN on early-exit error paths (insufficient data).
          best_candidate: dict with keys (freq_hz, snr_db, score_ratio,
                      noise_psd) for the strongest sub-threshold bin when
                      no detections; None otherwise.
    """
    _, n_time = power.shape

    # --- Fold ---
    # Per-fold window offsets (fractional PRI, independently rounded)
    _fo = fold_offsets if fold_offsets is not None else np.arange(K) * N

    if hypotheses is not None and len(hypotheses) > 0:
        best_scores, best_offsets, best_labels = fold_multi_hypothesis(
            power, hypotheses)
    else:
        # Single-rate fold using precomputed offsets
        max_start = n_time - _fo[-1]
        if max_start <= 0:
            return [], float('nan'), None
        effective_period = int(_fo[1]) if len(_fo) > 1 else int(N)
        search_range = min(effective_period, max_start)

        pulse_idx = np.arange(search_range)[:, None] + _fo[None, :]
        fold_scores = _compute_fold_scores(power, pulse_idx,
                           local_radius=FOLD_LOCAL_RADIUS)

        best_scores  = np.max(fold_scores, axis=1)
        best_offsets = np.argmax(fold_scores, axis=1)
        best_labels  = np.array(["A"] * power.shape[0], dtype=object)

    # --- Noise estimation (unchanged) ---
    if n_time >= 3:
        padded = np.pad(power, ((0, 0), (1, 1)), mode='edge')
        mov_mean = (padded[:, :-2] + padded[:, 1:-1] + padded[:, 2:]) / 3.0
    else:
        mov_mean = power.copy()

    med_per_freq = np.median(mov_mean, axis=1, keepdims=True)
    outlier_mask = power > 10.0 * med_per_freq

    masked_power = power.copy()
    masked_power[outlier_mask] = np.nan
    noise_power = np.nanmean(masked_power, axis=1)

    all_masked = np.isnan(noise_power)
    if np.any(all_masked):
        noise_power[all_masked] = np.nanmedian(power[all_masked, :], axis=1)

    noise_power = np.maximum(noise_power, 1e-30)

    if debug:
        n_outlier = np.sum(outlier_mask)
        n_total = outlier_mask.size
        n_all_masked_bins = np.sum(all_masked) if np.any(all_masked) else 0
        _noise_human = (
            f'[DEBUG NOISE] noise_power: min={noise_power.min():.6e}  '
            f'max={noise_power.max():.6e}  mean={noise_power.mean():.6e}  '
            f'std={noise_power.std():.6e}\n'
            f'[DEBUG NOISE] outlier mask: {n_outlier}/{n_total} '
            f'({100.0*n_outlier/n_total:.1f}%) masked  |  '
            f'{n_all_masked_bins} freq bins fully masked'
        )
        if slog:
            slog.emit(NOISE_STATS, _noise_human,
                      noise_min=float(noise_power.min()),
                      noise_max=float(noise_power.max()),
                      noise_mean=float(noise_power.mean()),
                      noise_std=float(noise_power.std()),
                      outlier_masked=int(n_outlier),
                      outlier_total=int(n_total),
                      fully_masked_bins=int(n_all_masked_bins))
        else:
            print(_noise_human)
        med_noise_dbg = np.median(noise_power)
        elevated_mask = noise_power > 5.0 * med_noise_dbg
        n_elevated = int(np.sum(elevated_mask))
        if n_elevated > 0:
            freq_axis_noise = Wf if Wf is not None else np.fft.fftshift(np.fft.fftfreq(nfft, d=1.0 / Fs))
            elev_idx = np.where(elevated_mask)[0]
            elev_parts = []
            elev_data = []
            for ei in elev_idx:
                ratio = noise_power[ei] / med_noise_dbg
                elev_parts.append(f'#{ei} ({freq_axis_noise[ei]:+.1f} Hz) '
                                  f'{noise_power[ei]:.3e} ({ratio:.0f}x median)')
                elev_data.append({'bin': int(ei),
                                  'freq_hz': float(freq_axis_noise[ei]),
                                  'power': float(noise_power[ei]),
                                  'ratio_to_median': float(ratio)})
            _elev_human = (f'[DEBUG NOISE] {n_elevated} elevated bins: '
                  + '  '.join(elev_parts))
            if slog:
                slog.emit(NOISE_ELEVATED, _elev_human,
                          n_elevated=n_elevated, bins=elev_data)
            else:
                print(_elev_human)

    # --- EVT threshold ---
    n_hypotheses = len(hypotheses) if hypotheses else 1
    cache_N_A = N_A_exact if N_A_exact is not None else N
    cache_N_B = N_B_exact if N_B_exact is not None else N_B
    if evt_threshold_cache.get('threshold') is None:
        cache_dir = evt_threshold_cache.get('cache_dir')
        mu, sigma = load_evt_cache(cache_dir, cache_N_A, K, N_B=cache_N_B,
                                   n_hypotheses=n_hypotheses)
        if mu is not None and sigma is not None and np.isfinite(mu) and np.isfinite(sigma) and sigma > 0:
            base_threshold = max(gumbel_r.ppf(1.0 - pf, loc=mu, scale=sigma), 0.0)
            _evt_human = (f'  [Loaded EVT cache: mu={mu:.4e}, sigma={sigma:.4e}, '
                  f'threshold={base_threshold:.4e}]')
            if slog:
                slog.emit(EVT_THRESHOLD, _evt_human,
                          source='cache', mu=float(mu), sigma=float(sigma),
                          threshold=float(base_threshold))
            else:
                print(_evt_human, flush=True)
        else:
            n_hyp_str = f' ({n_hypotheses} hypotheses)' if n_hypotheses > 1 else ''
            print(f'  [Generating EVT threshold via 100 noise trials{n_hyp_str}...]',
                  flush=True)
            base_threshold, mu, sigma = generate_evt_threshold(
                n_w, n_ol, nfft, samples_needed, N, K, pf,
                fold_offsets=_fo, W=W, n_trials=100, debug=debug,
                hypotheses=hypotheses,
            )
            if np.isinf(base_threshold):
                print(f'ERROR: Insufficient data for EVT threshold. '
                      f'Segment length ({n_time} samples) too short for K={K} folds.',
                      flush=True)
                return [], float('nan'), None
            save_evt_cache(cache_dir, cache_N_A, K, mu, sigma, N_B=cache_N_B,
                           n_hypotheses=n_hypotheses)
        evt_threshold_cache['threshold'] = base_threshold
    else:
        base_threshold = evt_threshold_cache['threshold']

    # Apply detection margin to lower the EVT threshold, increasing
    # sensitivity at the cost of more near-threshold (marginal) detections.
    # The two-tier confidence system classifies these via confidence_ratio.
    base_threshold *= detection_margin
    if detection_margin != 1.0 and not evt_threshold_cache.get('margin_logged'):
        _margin_human = (f'  [Detection margin={detection_margin:.2f} applied: '
              f'effective threshold={base_threshold:.4e}]')
        if slog:
            slog.emit(EVT_THRESHOLD, _margin_human,
                      source='margin', detection_margin=detection_margin,
                      effective_threshold=float(base_threshold))
        else:
            print(_margin_human, flush=True)
        evt_threshold_cache['margin_logged'] = True

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

    freq_axis = Wf if Wf is not None else np.fft.fftshift(np.fft.fftfreq(nfft, d=1.0 / Fs))
    psd_scale = float(Fs * n_w)

    # Best sub-threshold candidate for diagnostics (computed unconditionally
    # so it is available when detections exist but are all filtered out).
    score_ratios = best_scores / np.maximum(threshold, 1e-30)
    best_idx = int(np.argmax(score_ratios))
    best_cand = {
        'freq_hz': float(freq_axis[best_idx]),
        'snr_db': float(10.0 * np.log10(best_scores[best_idx] / noise_power[best_idx])),
        'score_ratio': float(score_ratios[best_idx]),
        'noise_psd': float(noise_power[best_idx] / psd_scale),
    }

    det_bins = np.where(best_scores > threshold)[0]

    # --- Per-hypothesis diagnostic for the strongest frequency bin ---
    # Keep this behind --debug to avoid high-volume logging in normal runs.
    if debug and hypotheses is not None and len(hypotheses) > 0:
        # Find the frequency bin with the highest score/threshold ratio
        diag_bin = best_idx  # from best_cand computation above
        diag_label = str(best_labels[diag_bin])
        diag_offset = int(best_offsets[diag_bin])
        diag_ratio = float(score_ratios[diag_bin])

        # Compute per-hypothesis scores at this frequency bin
        hyp_diag = []
        hyp_detail = []  # (label, ratio, offset, pulse_windows, fold_powers, mff)
        for label, pidx in hypotheses:
            fold_scores_h = _compute_fold_scores(
                power[diag_bin:diag_bin + 1, :], pidx,
                local_radius=FOLD_LOCAL_RADIUS)[0]
            h_best_score = float(np.max(fold_scores_h))
            h_best_offset = int(np.argmax(fold_scores_h))
            h_ratio = h_best_score / max(float(threshold[diag_bin]), 1e-30)

            # Compute max_fold_fraction for this hypothesis at its best offset
            on_idx = pidx[h_best_offset]
            on_powers = _local_peak_powers_1d(
                power[diag_bin], on_idx, local_radius=FOLD_LOCAL_RADIUS)
            p_sum = float(np.sum(on_powers))
            mff = float(np.max(on_powers) / max(p_sum, 1e-30))

            hyp_diag.append((label, h_ratio, mff))
            hyp_detail.append((label, h_ratio, h_best_offset, on_idx.tolist(),
                               on_powers.tolist(), mff))

        # Sort by score ratio descending and show all hypotheses.
        hyp_diag.sort(key=lambda x: -x[1])
        top_str = '  '.join(
            f'{lbl}:{rat:.1f}/F{mff:.2f}{"*" if mff > DOMINANT_FOLD_THRESHOLD else ""}'
            for lbl, rat, mff in hyp_diag
        )
        n_above = len(det_bins)
        _hyp_human = (f'  [HYP] best_bin={diag_bin} winner={diag_label} '
              f'ratio={diag_ratio:.1f} above_thresh={n_above}  '
              f'top: {top_str}')
        _hyp_data = [{'label': lbl, 'ratio': rat, 'max_fold_frac': mff}
                     for lbl, rat, mff in hyp_diag]
        if slog:
            slog.emit(HYPOTHESIS, _hyp_human,
                      best_bin=int(diag_bin), winner=diag_label,
                      winner_ratio=diag_ratio, above_thresh=n_above,
                      hypotheses=_hyp_data)
        else:
            print(_hyp_human, flush=True)

        # Show per-fold SNR vectors for every hypothesis so we can inspect
        # which specific folds are weak/strong across the full bank.
        hyp_detail.sort(key=lambda x: -x[1])  # sort by score ratio
        for lbl, rat, off, widx, fpow, mff in hyp_detail:
            noise_at_bin = float(noise_power[diag_bin])
            fold_snrs = [f'{10*np.log10(p/noise_at_bin):.1f}' if p > 0
                         else '-inf' for p in fpow]
            _hd_human = (f'  [HYP detail] {lbl} ratio={rat:.1f} t0={off} wins={widx} '
                  f'n_time={n_time} F={mff:.4f} '
                  f'fold_pwr=[{", ".join(f"{p:.3e}" for p in fpow)}] '
                  f'fold_snr=[{", ".join(fold_snrs)}] dB')
            if slog:
                slog.emit_raw(_hd_human)
            else:
                print(_hd_human, flush=True)

    if len(det_bins) == 0:
        median_noise_psd = float(np.median(noise_power) / psd_scale)
        return [], median_noise_psd, best_cand

    # --- Build detection results ---
    # Build a lookup from hypothesis label to its pulse_idx matrix so we can
    # recover the exact pulse window indices for the winning hypothesis.
    hyp_lookup = {}
    if hypotheses is not None:
        for label, pidx in hypotheses:
            hyp_lookup[label] = pidx

    results = []
    for b in det_bins:
        label = str(best_labels[b])
        offset = int(best_offsets[b])

        snr_db = 10.0 * np.log10(best_scores[b] / noise_power[b])
        # Score-to-threshold ratio: how far above threshold this detection is.
        # Values near 1.0 are marginal (likely false alarm); >>1 is confident.
        fold_score_psd = float(best_scores[b] / psd_scale)
        score_ratio = float(best_scores[b] / max(threshold[b], 1e-30))
        # Per-fold diagnostics: individual on-window powers and SNRs.
        # Use winning hypothesis pulse_idx if available, else fold_offsets.
        t0 = int(best_offsets[b])
        if label in hyp_lookup and t0 < hyp_lookup[label].shape[0]:
            on_idx = hyp_lookup[label][t0]
        else:
            on_idx = t0 + _fo
        on_powers = _local_peak_powers_1d(
            power[b], on_idx, local_radius=FOLD_LOCAL_RADIUS)
        fold_snrs_db = (10.0 * np.log10(on_powers / noise_power[b])).tolist()
        p_sum = float(np.sum(on_powers))
        max_fold_fraction = float(np.max(on_powers) / max(p_sum, 1e-30))
        fold_info = {'max_fold_fraction': max_fold_fraction,
                     'fold_snrs': fold_snrs_db,
                     'fold_windows': [int(x) for x in on_idx]}
        results.append(Detection(
            freq_hz=freq_axis[b], snr_db=snr_db, offset=offset,
            noise_psd=float(noise_power[b] / psd_scale),
            stft_score=fold_score_psd, score_ratio=score_ratio,
            hyp_label=label, fold_info=fold_info))

    results.sort(key=lambda d: -d.snr_db)

    # Merge peaks closer than min_sep bins (STFT sidelobe suppression)
    min_sep = max(15, nfft // 4)
    merged = []
    used_bins = []
    for det in results:
        b = int(np.argmin(np.abs(freq_axis - det.freq_hz)))
        if any(abs(b - ub) <= min_sep for ub in used_bins):
            continue
        merged.append(det)
        used_bins.append(b)
        if len(merged) >= 1:
            break

    if len(merged) == 0:
        median_noise_psd = float(np.median(noise_power) / psd_scale)
        return [], median_noise_psd, best_cand

    # Per-detection hypothesis detail (complement to the diag_bin block
    # which only covers the global best bin, not actual detection bins).
    if debug and hypotheses is not None and len(hypotheses) > 0:
        for det in merged:
            det_bin = int(np.argmin(np.abs(freq_axis - det.freq_hz)))
            if det_bin == best_idx:
                continue  # already covered by diag_bin block above
            det_thresh = float(threshold[det_bin])
            hyp_det = []
            for label, pidx in hypotheses:
                fold_scores_h = _compute_fold_scores(
                    power[det_bin:det_bin + 1, :], pidx,
                    local_radius=FOLD_LOCAL_RADIUS)[0]
                h_best_score = float(np.max(fold_scores_h))
                h_best_offset = int(np.argmax(fold_scores_h))
                h_ratio = h_best_score / max(det_thresh, 1e-30)
                on_idx_h = pidx[h_best_offset]
                on_powers_h = _local_peak_powers_1d(
                    power[det_bin], on_idx_h, local_radius=FOLD_LOCAL_RADIUS)
                p_sum_h = float(np.sum(on_powers_h))
                mff_h = float(np.max(on_powers_h) / max(p_sum_h, 1e-30))
                noise_at_bin = float(noise_power[det_bin])
                fold_snrs_h = [f'{10*np.log10(p/noise_at_bin):.1f}' if p > 0
                               else '-inf' for p in on_powers_h]
                hyp_det.append((label, h_ratio, h_best_offset,
                                on_idx_h.tolist(), on_powers_h.tolist(),
                                mff_h, fold_snrs_h))
            hyp_det.sort(key=lambda x: -x[1])
            top_str = '  '.join(
                f'{lbl}:{rat:.1f}/F{mff:.2f}{"*" if mff > DOMINANT_FOLD_THRESHOLD else ""}'
                for lbl, rat, _, _, _, mff, _ in hyp_det
            )
            _hdet_human = (f'  [HYP DET] bin={det_bin} freq={det.freq_hz:+.1f} Hz '
                  f'winner={det.hyp_label} ratio={det.score_ratio:.1f}  '
                  f'top: {top_str}')
            if slog:
                slog.emit_raw(_hdet_human)
            else:
                print(_hdet_human, flush=True)
            for lbl, rat, off, widx, fpow, mff, fsnrs in hyp_det:
                _hddet_human = (f'  [HYP DET detail] {lbl} ratio={rat:.1f} t0={off} '
                      f'wins={widx} F={mff:.4f} '
                      f'fold_pwr=[{", ".join(f"{p:.3e}" for p in fpow)}] '
                      f'fold_snr=[{", ".join(fsnrs)}] dB')
                if slog:
                    slog.emit_raw(_hddet_human)
                else:
                    print(_hddet_human, flush=True)

    return merged, None, None


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    global _should_stop

    ap = argparse.ArgumentParser(
        description='VHF Pulse Detector (configurable K, EVT threshold)',
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
    ap.add_argument('--pf',  type=float, default=5e-2,
                    help='False-alarm probability per cycle via EVT (default: 5e-2)')
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
    ap.add_argument('--threshold-cache-dir', type=str, default=None,
                    help='Directory for EVT threshold cache files (default: no disk cache)')
    ap.add_argument('--detection-margin', type=float, default=0.90,
                    help='EVT threshold multiplier, lower = more sensitive (default: 0.90)')
    ap.add_argument('--confidence-ratio', type=float, default=1.3,
                    help='Score/threshold ratio for confirmed status (default: 1.3)')
    ap.add_argument('--k', type=int, default=5,
                    help='Number of pulses to fold/integrate (default: 5)')
    ap.add_argument('--tip-secondary', type=float, default=None,
                    help='Secondary inter-pulse interval in seconds. '
                         'Enables multi-hypothesis rate-switch detection. '
                         'Omit to use single-rate mode (backwards compatible).')
    ap.add_argument('--warmup-seconds', type=float, default=5.0,
                    help='Seconds of initial IQ data to discard before '
                         'detection starts (default: 5.0).')
    ap.add_argument('--log-dir', type=str, default=None,
                    help='Directory for log and diagnostic output files.')
    ap.add_argument('--dump-spectrogram', action='store_true', default=False,
                    help='Save per-cycle spectrogram, IQ, and metadata '
                         'to --log-dir (requires --log-dir).')
    args = ap.parse_args()

    global K
    K = args.k

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
    if args.detection_margin <= 0:
        sys.exit(f'Error: --detection-margin must be positive, got {args.detection_margin}')
    if args.confidence_ratio <= 0:
        sys.exit(f'Error: --confidence-ratio must be positive, got {args.confidence_ratio}')
    if args.k < 2:
        sys.exit(f'Error: --k must be >= 2, got {args.k}')
    if args.dump_spectrogram and not args.log_dir:
        sys.exit('Error: --dump-spectrogram requires --log-dir')
    if args.dump_spectrogram:
        os.makedirs(args.log_dir, exist_ok=True)
        print(f'Spectrogram dump → {args.log_dir}/', flush=True)
    if args.warmup_seconds < 0:
        sys.exit(f'Error: --warmup-seconds must be >= 0, got {args.warmup_seconds}')
    if args.tip_secondary is not None:
        if args.tip_secondary <= 0:
            sys.exit(f'Error: --tip-secondary must be positive, got {args.tip_secondary}')
        if args.tp >= args.tip_secondary:
            sys.exit(f'Error: --tp ({args.tp}s) must be < --tip-secondary ({args.tip_secondary}s)')

    # --- Structured logger ---
    _jsonl_path = None
    if args.log_dir:
        os.makedirs(args.log_dir, exist_ok=True)
        _tag_suffix = f'_{args.tag_id}' if args.tag_id else ''
        _jsonl_path = os.path.join(args.log_dir, f'detector{_tag_suffix}.jsonl')
    slog = StructuredLogger(jsonl_path=_jsonl_path)

    # --- STFT geometry ---
    n_w  = int(np.ceil(args.tp * args.fs))
    if n_w < 4:
        sys.exit(f'Error: STFT window too short (n_w={n_w}). '
                 f'Check --tp and --fs.')
    n_ol  = n_w // 2
    n_ws  = n_w - n_ol
    N_exact = args.tip * args.fs / n_ws   # fractional PRI in STFT windows
    N     = int(np.floor(N_exact))

    # Precompute per-fold window offsets using fractional PRI.
    # Using integer N (= floor(N_exact)) would accumulate drift of
    # (N_exact - N) windows per fold — at fold k the pulse lands
    # (k * N_exact) windows from the start, but integer stride puts
    # it at (k * N), off by k * (N_exact - N).  For K=20 and
    # N_exact=176.51 this reaches 9.6 windows by fold 19, missing
    # the pulse entirely.  Rounding each offset independently keeps
    # the error bounded to ±0.5 windows regardless of K.
    fold_offsets = np.round(np.arange(K) * N_exact).astype(int)

    # Validate N is sufficient for K-fold integration
    if N < K:
        min_tip = (K * n_ws) / args.fs
        sys.exit(
            f'Error: inter-pulse interval --tip is too small for '
            f'K={K} folds with --tp={args.tp}s and --fs={args.fs}Hz. '
            f'Increase --tip to at least {min_tip:.6f}s '
            f'(currently {args.tip}s).'
        )

    # --- Secondary rate and hypothesis bank ---
    N_B = None
    N_B_exact = None
    rate_switch_hypotheses = None
    if args.tip_secondary is not None:
        N_B_exact = args.tip_secondary * args.fs / n_ws
        N_B = int(np.floor(N_B_exact))
        if N_B < K:
            min_tip2 = (K * n_ws) / args.fs
            sys.exit(
                f'Error: --tip-secondary is too small for '
                f'K={K} folds with --tp={args.tp}s and --fs={args.fs}Hz. '
                f'Increase --tip-secondary to at least {min_tip2:.6f}s '
                f'(currently {args.tip_secondary}s).'
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

    samples_needed = compute_segment_samples(n_ws, n_ol, K, N_exact,
                                              N_B_exact)

    # Build hypothesis bank (after nfft is known so we can compute n_time
    # for the hypothesis index builder).  We need n_time to set bounds;
    # compute it from samples_needed the same way compute_stft_power does.
    _n_time_est = (samples_needed - n_ol) // n_ws
    if N_B_exact is not None:
        rate_switch_hypotheses = build_hypothesis_indices(
            N_exact, K, _n_time_est, N_B_exact)
    else:
        rate_switch_hypotheses = None

    seg_sec  = samples_needed / args.fs
    freq_res = args.fs / nfft
    fa_per_hour = (3600.0 / seg_sec) * args.pf
    n_hyp = len(rate_switch_hypotheses) if rate_switch_hypotheses else 1

    _tip2_line = (f'\n  Inter-pulse 2   {args.tip_secondary:.3f} s  (N_B={N_B} windows)'
                  if N_B is not None else '')
    _cfreq_line = (f'\n  Center freq     {args.center_freq:.6f} MHz'
                   if args.center_freq > 0 else '')
    _cache_line = (f'\n  EVT cache dir   {args.threshold_cache_dir}'
                   if args.threshold_cache_dir else '')
    _startup_human = (
        f'=== VHF Pulse Detector ===\n'
        f'  Pulse width     {args.tp * 1000:.1f} ms\n'
        f'  Inter-pulse     {args.tip:.3f} s'
        f'{_tip2_line}\n'
        f'  Folds (K)       {K}\n'
        f'  Hypotheses      {n_hyp}\n'
        f'  Sample rate     {args.fs:.1f} Hz\n'
        f'  STFT            window={n_w}  overlap={n_ol}  step={n_ws}\n'
        f'  PRI (N)         {N} STFT windows  (exact {N_exact:.4f})\n'
        f'  Freq bins       {nfft}  ({freq_res:.1f} Hz resolution, W matrix)\n'
        f'  Segment         {samples_needed} samples  ({seg_sec:.1f} s)\n'
        f'  False alarm %   {args.pf * 100:.2g}%  '
        f'(~{fa_per_hour:.2f} false alarms/hour)\n'
        f'  Det margin      {args.detection_margin:.2f}\n'
        f'  Conf ratio      {args.confidence_ratio:.2f}\n'
        f'  UDP port        {args.port}\n'
        f'  Warmup discard  {args.warmup_seconds:.1f} s'
        f'{_cfreq_line}'
        f'{_cache_line}\n'
    )
    slog.emit(STARTUP, _startup_human,
              tp=args.tp, tip=args.tip,
              tip_secondary=args.tip_secondary,
              K=K, n_hypotheses=n_hyp,
              fs=args.fs, n_w=n_w, n_ol=n_ol, n_ws=n_ws,
              N=N, N_exact=N_exact, N_B=N_B,
              nfft=nfft, freq_res=freq_res,
              samples_needed=samples_needed, seg_sec=seg_sec,
              pf=args.pf, fa_per_hour=fa_per_hour,
              detection_margin=args.detection_margin,
              confidence_ratio=args.confidence_ratio,
              port=args.port, warmup_seconds=args.warmup_seconds,
              center_freq=args.center_freq,
              tag_id=args.tag_id, freq=args.freq,
              log_dir=args.log_dir,
              dump_spectrogram=args.dump_spectrogram)

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
        else:
            print('  Warning: --freq not set; pulse and no-detection reports '
                  'will not be sent to the controller', file=sys.stderr,
                  flush=True)

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
    segment_gap_fills = []  # list of (sample_offset, n_zeros) for current segment

    # Single gap threshold (conservative), in nanoseconds
    gap_threshold_reset = args.tp * 2.0  # ≥ 2×tp: reset, < 2×tp: zero-fill
    gap_threshold_reset_ns = int(gap_threshold_reset * 1e9)

    # Startup warmup: discard initial IQ before detection starts.
    warmup_remaining_samples = int(round(args.warmup_seconds * args.fs))

    # EVT threshold cache (regenerated if geometry changes)
    evt_threshold_cache = {}
    if args.threshold_cache_dir:
        evt_threshold_cache['cache_dir'] = args.threshold_cache_dir

    # Start a dedicated heartbeat thread (pure timer, 1 Hz)
    heartbeat_stop = threading.Event()
    def _heartbeat_loop():
        while not heartbeat_stop.wait(1.0):
            if pulse_sock is not None:
                send_heartbeat_udp(pulse_sock, pulse_dest, args.tag_id)
    if pulse_sock is not None:
        heartbeat_thread = threading.Thread(target=_heartbeat_loop, daemon=True)
        heartbeat_thread.start()
    else:
        heartbeat_thread = None

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

            # Discard initial IQ while front-end/stream settle.
            if warmup_remaining_samples > 0:
                warmup_remaining_samples -= n_samp
                if warmup_remaining_samples <= 0:
                    warmup_remaining_samples = 0
                    seg_ts = None
                    buf_parts.clear()
                    buf_len = 0
                    print('Warmup complete: starting detection', flush=True)
                # Keep continuity bookkeeping current during warmup.
                last_packet_ts = ts
                last_packet_samples = n_samp
                continue

            if seg_ts is None:
                seg_ts = ts

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
                    slog.emit(GAP_EVENT,
                              f'\n*** NEGATIVE GAP (timestamp regression): {gap_ms:.1f} ms ***\n'
                              f'    Last packet ts={last_packet_ts} ns, current ts={ts} ns\n'
                              f'    Expected delta={expected_delta_ns / 1_000_000.0:.1f} ms, '
                              f'got {actual_delta_ns / 1_000_000.0:.1f} ms\n',
                              kind='negative', gap_ms=gap_ms,
                              last_ts_ns=last_packet_ts, current_ts_ns=ts,
                              expected_delta_ms=expected_delta_ns / 1e6,
                              actual_delta_ms=actual_delta_ns / 1e6)

                    # Reset buffer and start a new segment from this packet
                    buf_parts.clear()
                    buf_len = 0
                    seg_ts = ts
                    segment_has_gap = False
                    segment_gap_fills.clear()
                else:
                    # Calculate missing samples (only act if ≥1 sample missing)
                    missing_samples = int(round(gap_size_ns / (1e9 / args.fs)))

                    if missing_samples > 0:
                        gap_ms = gap_size_ns / 1_000_000.0

                        if gap_size_ns >= gap_threshold_reset_ns:
                            # LARGE GAP: Discard segment and reset
                            gap_reset_count += 1
                            slog.emit(GAP_EVENT,
                                      f'\n*** GAP ≥ RESET THRESHOLD: {gap_ms:.1f} ms '
                                      f'(≥{gap_threshold_reset*1000:.1f} ms) ***\n'
                                      f'    Discarding {buf_len} buffered samples and resetting segment\n'
                                      f'    Expected packet after {expected_delta_ns / 1_000_000.0:.1f} ms, '
                                      f'got {actual_delta_ns / 1_000_000.0:.1f} ms\n',
                                      kind='reset', gap_ms=gap_ms,
                                      threshold_ms=gap_threshold_reset * 1000,
                                      discarded_samples=buf_len,
                                      expected_delta_ms=expected_delta_ns / 1e6,
                                      actual_delta_ms=actual_delta_ns / 1e6)

                            # Reset buffer
                            buf_parts.clear()
                            buf_len = 0
                            seg_ts = ts
                            segment_has_gap = False
                            segment_gap_fills.clear()

                        else:
                            # GAP BELOW THRESHOLD: Zero-fill
                            gap_zerofill_count += 1
                            segment_has_gap = True

                            slog.emit(GAP_EVENT,
                                      f'GAP < THRESHOLD: {gap_ms:.1f} ms '
                                      f'(< {gap_threshold_reset*1000:.1f} ms) - '
                                      f'zero-filling {missing_samples} samples',
                                      kind='zerofill', gap_ms=gap_ms,
                                      threshold_ms=gap_threshold_reset * 1000,
                                      missing_samples=missing_samples)

                            # Insert zeros to maintain continuity
                            segment_gap_fills.append((buf_len, missing_samples))
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

            segment = np.concatenate(buf_parts)[:samples_needed]
            buf_parts.clear()
            buf_len = 0

            # Check if this segment had gaps
            had_gap = segment_has_gap
            had_gap_fills = list(segment_gap_fills)
            segment_has_gap = False
            segment_gap_fills.clear()

            t_stft_start = time.monotonic()
            power, n_win = compute_stft_power(segment, n_w, n_ol, nfft, W=W,
                                               min_windows=K)
            t_stft_end = time.monotonic()

            if args.debug:
                seg_mag = np.abs(segment)
                pwr_per_freq = power.mean(axis=1)
                _stft_human = (
                    f'[DEBUG STFT] IQ: len={len(segment)}  '
                    f'mean_mag={seg_mag.mean():.6e}  max_mag={seg_mag.max():.6e}\n'
                    f'[DEBUG STFT] power: shape={power.shape}  '
                    f'min={power.min():.6e}  max={power.max():.6e}  '
                    f'mean={power.mean():.6e}  median={np.median(power):.6e}\n'
                    f'[DEBUG STFT] per-freq mean power: min={pwr_per_freq.min():.6e}  '
                    f'max={pwr_per_freq.max():.6e}  '
                    f'ratio={pwr_per_freq.max()/max(pwr_per_freq.min(), 1e-30):.1f}'
                )
                slog.emit(STFT_DEBUG, _stft_human, cycle=cycle,
                          iq_len=len(segment),
                          iq_mean_mag=float(seg_mag.mean()),
                          iq_max_mag=float(seg_mag.max()),
                          power_shape=list(power.shape),
                          power_min=float(power.min()),
                          power_max=float(power.max()),
                          power_mean=float(power.mean()),
                          power_median=float(np.median(power)),
                          freq_power_min=float(pwr_per_freq.min()),
                          freq_power_max=float(pwr_per_freq.max()),
                          freq_power_ratio=float(pwr_per_freq.max()/max(pwr_per_freq.min(), 1e-30)))
                if had_gap_fills:
                    for gf_offset, gf_len in had_gap_fills:
                        win_start = max(0, (gf_offset - n_ol) // n_ws)
                        win_end = (gf_offset + gf_len - 1) // n_ws
                        slog.emit_raw(f'[DEBUG GAP] zero-fill: offset={gf_offset} '
                              f'len={gf_len} samples '
                              f'(covers STFT windows {win_start}-{win_end})')

            # Invalidate EVT cache if geometry changed
            n_freq_cur = power.shape[0]
            n_time_cur = power.shape[1]
            if (evt_threshold_cache.get('n_freq') != n_freq_cur or
                evt_threshold_cache.get('n_time') != n_time_cur):
                evt_threshold_cache['threshold'] = None
                evt_threshold_cache['margin_logged'] = False
                evt_threshold_cache['n_freq'] = n_freq_cur
                evt_threshold_cache['n_time'] = n_time_cur

            t_fold_start = time.monotonic()
            detections, nodet_noise_psd, best_candidate = fold_detect(
                                     power, N, args.pf, args.fs, nfft,
                                     n_w, n_ol, samples_needed,
                                     evt_threshold_cache,
                                     fold_offsets=fold_offsets,
                                     W=W, Wf=Wf,
                                     debug=args.debug,
                                     detection_margin=args.detection_margin,
                                     hypotheses=rate_switch_hypotheses,
                                     N_B=N_B,
                                     N_A_exact=N_exact,
                                     N_B_exact=N_B_exact,
                                     slog=slog)
            t_fold_end = time.monotonic()
            proc_ms = (t_fold_end - t0) * 1000.0
            if args.debug:
                stft_ms = (t_stft_end - t_stft_start) * 1000.0
                fold_ms = (t_fold_end - t_fold_start) * 1000.0
                slog.emit(TIMING,
                          f'[DEBUG TIMING] stft={stft_ms:.0f} ms  '
                          f'fold={fold_ms:.0f} ms  '
                          f'total={proc_ms:.0f} ms',
                          cycle=cycle, stft_ms=stft_ms,
                          fold_ms=fold_ms, total_ms=proc_ms)

            # ---- per-cycle spectrogram / IQ dump ----
            if args.dump_spectrogram and args.log_dir:
                try:
                    prefix = os.path.join(args.log_dir, f'cycle_{cycle:04d}')
                    np.save(f'{prefix}_power.npy', power.astype(np.float32))
                    np.save(f'{prefix}_iq.npy', segment.astype(np.complex64))
                    meta = {
                        'cycle': cycle,
                        'timestamp_ns': seg_ts,
                        'fs': args.fs,
                        'nfft': nfft,
                        'n_w': n_w,
                        'n_ol': n_ol,
                        'power_shape': list(power.shape),
                        'had_gap': had_gap,
                        'gap_fills': had_gap_fills,
                        'detections': [
                            {
                                'freq_hz': d.freq_hz,
                                'snr_db': d.snr_db,
                                'score_ratio': d.score_ratio,
                                'noise_psd': d.noise_psd,
                                'hyp_label': d.hyp_label,
                            } for d in detections
                        ] if detections else [],
                        'best_candidate': best_candidate,
                    }
                    with open(f'{prefix}_meta.json', 'w') as f:
                        json.dump(meta, f, indent=1)
                except OSError as exc:
                    print(f'WARNING: disabling spectrogram dump after I/O '
                          f'error on cycle {cycle}: {exc}',
                          file=sys.stderr, flush=True)
                    args.dump_spectrogram = False

            # Timestamp string (UTC) for the start of this segment
            # seg_ts is in nanoseconds; convert to seconds for datetime
            ts_str = ''
            current_ts = seg_ts  # Save before clearing
            if seg_ts and seg_ts > 1e9:
                dt = datetime.datetime.fromtimestamp(
                    seg_ts / 1e9, tz=datetime.timezone.utc
                )
                ts_str = dt.strftime('%H:%M:%S') + f'.{dt.microsecond // 1000:03d}'
            seg_ts = None

            # Append gap warning to output if segment had discontinuities
            gap_flag = ' [ZEROFILLED]' if had_gap else ''

            confidence_ratio = args.confidence_ratio

            if detections:
                det_total += len(detections)

                # Calculate inter-pulse delta
                delta_str = ''
                inter_pulse_delta_ms = None
                if last_detection_ts is not None and current_ts is not None:
                    delta_s = (current_ts - last_detection_ts) / 1e9  # Convert ns to seconds
                    inter_pulse_delta_ms = delta_s * 1000.0
                    delta_str = f'  Δt={delta_s:.3f}s'
                if current_ts is not None:
                    last_detection_ts = current_ts

                # Score/threshold ratio < confidence_ratio indicates a marginal
                # detection that is more likely to be a false alarm.  Report
                # these as SUBTHRESHOLD so the GCS can display them differently.

                tip_secondary = getattr(args, 'tip_secondary', None)

                for det in detections:
                    # Fold quality: if any single fold carries >80% of the
                    # total score, this is likely a transient (not a real
                    # pulse train).  Matches uavrt_detection's
                    # selectpeakindex heuristic.  Downgrade to SUBTHRESHOLD
                    # rather than discarding.
                    has_dominant_fold = det.fold_info['max_fold_fraction'] > DOMINANT_FOLD_THRESHOLD
                    is_marginal = det.score_ratio < confidence_ratio or has_dominant_fold
                    det_status = (DETECTION_STATUS_SUBTHRESHOLD if is_marginal
                                  else DETECTION_STATUS_SUPERTHRESHOLD)
                    confidence_tag = ''
                    confidence_flag = ''
                    if has_dominant_fold:
                        confidence_tag = 'DOMINANT_FOLD'
                        confidence_flag = '  [DOMINANT_FOLD]'
                    elif det.score_ratio < confidence_ratio:
                        confidence_tag = 'LOW'
                        confidence_flag = '  [LOW]'
                    hyp_flag = f'  hyp={det.hyp_label}' if det.hyp_label != 'A' else ''

                    # Map hypothesis label → group_ind encoding and
                    # determine which PRI to use for next-pulse prediction.
                    gind, last_rate = hyp_label_to_group_ind(det.hyp_label, K=K)
                    if last_rate == 'B' and tip_secondary:
                        predict_tip = tip_secondary
                    else:
                        predict_tip = args.tip

                    # Send pulse to controller via UDP if configured
                    if pulse_sock is not None:
                        start_time_s = current_ts / 1e9 if current_ts else time.time()
                        predict_next_s = start_time_s + predict_tip
                        report_freq_hz = args.freq if args.freq else int(det.freq_hz)

                        send_pulse_udp(
                            pulse_sock, pulse_dest,
                            tag_id=args.tag_id,
                            frequency_hz=report_freq_hz,
                            start_time_seconds=start_time_s,
                            predict_next_start_seconds=predict_next_s,
                            snr=det.snr_db,
                            stft_score=det.score_ratio,
                            group_seq_counter=cycle,
                            group_ind=gind,
                            group_snr=det.snr_db,
                            detection_status=det_status,
                            confirmed_status=1 if not is_marginal else 0,
                            noise_psd=det.noise_psd,
                        )

                    if args.center_freq > 0:
                        abs_mhz = args.center_freq + det.freq_hz / 1e6
                        _det_human = (f'[{cycle:4d} {ts_str}]  DETECTED  '
                              f'{abs_mhz:.6f} MHz  '
                              f'({det.freq_hz:+.1f} Hz)  '
                              f'SNR {det.snr_db:.1f} dB  '
                              f'score_ratio {det.score_ratio:.3f}  '
                              f'noise {det.noise_psd:.3e}  '
                              f'{proc_ms:.0f} ms{delta_str}{gap_flag}{confidence_flag}{hyp_flag}')
                    else:
                        _det_human = (f'[{cycle:4d} {ts_str}]  DETECTED  '
                              f'{det.freq_hz:+.1f} Hz  '
                              f'SNR {det.snr_db:.1f} dB  '
                              f'score_ratio {det.score_ratio:.3f}  '
                              f'noise {det.noise_psd:.3e}  '
                              f'{proc_ms:.0f} ms{delta_str}{gap_flag}{confidence_flag}{hyp_flag}')
                    slog.emit(DETECTION, _det_human,
                              cycle=cycle, timestamp_ns=current_ts,
                              freq_hz=det.freq_hz,
                              snr_db=det.snr_db,
                              score_ratio=det.score_ratio,
                              noise_psd=det.noise_psd,
                              proc_ms=proc_ms,
                              had_gap=had_gap,
                              confidence=confidence_tag,
                              hyp_label=det.hyp_label,
                              detection_status=det_status,
                              inter_pulse_delta_ms=inter_pulse_delta_ms)
                    fold_snrs_str = ', '.join(f'{s:.1f}' for s in det.fold_info['fold_snrs'])
                    fold_wins_str = ', '.join(str(w) for w in det.fold_info['fold_windows'])
                    slog.emit(FOLDS,
                          f'  [FOLDS] score_ratio={det.score_ratio:.3f}  '
                          f'max_fold_frac={det.fold_info["max_fold_fraction"]:.3f}  '
                          f'per_fold_snr=[{fold_snrs_str}] dB  '
                          f'windows=[{fold_wins_str}]',
                          cycle=cycle,
                          score_ratio=det.score_ratio,
                          max_fold_fraction=det.fold_info['max_fold_fraction'],
                          fold_snrs=det.fold_info['fold_snrs'],
                          fold_windows=det.fold_info['fold_windows'])
            else:
                # Send a no-detection report so the controller/GCS knows
                # we searched this cycle and found nothing.  Uses
                # detection_status=3 (no pulse detected) — a dedicated
                # status value that cannot be confused with a real pulse.
                if pulse_sock is not None and args.freq:
                    start_time_s = current_ts / 1e9 if current_ts else time.time()
                    nodet_score_ratio = best_candidate['score_ratio'] if best_candidate is not None else 0.0
                    send_pulse_udp(
                        pulse_sock, pulse_dest,
                        tag_id=args.tag_id,
                        frequency_hz=args.freq,
                        start_time_seconds=start_time_s,
                        predict_next_start_seconds=0.0,
                        snr=0.0,
                        stft_score=nodet_score_ratio,
                        group_seq_counter=cycle,
                        group_ind=0,
                        group_snr=0.0,
                        detection_status=DETECTION_STATUS_NO_DETECTION,
                        confirmed_status=0,
                        noise_psd=nodet_noise_psd,
                    )
                cand_str = ''
                if best_candidate is not None:
                    cand_freq = best_candidate['freq_hz']
                    if args.center_freq > 0:
                        cand_freq_str = f'{args.center_freq + cand_freq / 1e6:.6f} MHz'
                    else:
                        cand_freq_str = f'{cand_freq:+.1f} Hz'
                    cand_str = (f'  best={cand_freq_str} '
                                f'SNR {best_candidate["snr_db"]:.1f} dB '
                                f'ratio {best_candidate["score_ratio"]:.3f} '
                                f'noise {best_candidate["noise_psd"]:.3e}')
                slog.emit(NO_DETECTION,
                          f'[{cycle:4d} {ts_str}]  no detection  '
                          f'{proc_ms:.0f} ms{gap_flag}{cand_str}',
                          cycle=cycle, timestamp_ns=current_ts,
                          proc_ms=proc_ms, had_gap=had_gap,
                          best_candidate=best_candidate)

    except KeyboardInterrupt:
        pass  # Handled by signal handler
    finally:
        elapsed = time.monotonic() - run_start
        slog.emit(SESSION_END,
                  f'\n--- Detection stopped after {cycle} cycles ({elapsed:.0f} s) ---\n'
                  f'  Detections:        {det_total}\n'
                  f'  Zero-filled gaps:  {gap_zerofill_count} (< {gap_threshold_reset*1000:.1f} ms)\n'
                  f'  Reset gaps:        {gap_reset_count} (≥ {gap_threshold_reset*1000:.1f} ms, segments discarded)\n'
                  f'  Total gap events:  {gap_zerofill_count + gap_reset_count}',
                  cycles=cycle, elapsed_s=elapsed,
                  detections=det_total,
                  gap_zerofill_count=gap_zerofill_count,
                  gap_reset_count=gap_reset_count)
        slog.close()
        heartbeat_stop.set()
        if heartbeat_thread is not None:
            heartbeat_thread.join(timeout=2.0)
        sock.close()
        if pulse_sock is not None:
            pulse_sock.close()


if __name__ == '__main__':
    main()
