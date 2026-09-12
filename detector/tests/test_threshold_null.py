"""Data-derived detection threshold and impulse blanking.

Covers ``permutation_null_threshold`` (window-permutation null of the slice's
own STFT) and ``blank_impulses`` (short-burst zeroing before the STFT).
"""

import numpy as np
import pytest

from pulse_detector import (
    NULL_MIN_PERMUTATIONS,
    blank_impulses,
    build_hypothesis_indices,
    build_weighting_matrix,
    compute_segment_samples,
    compute_stft_power,
    estimate_noise_power,
    fold_detect,
    fold_multi_hypothesis,
    permutation_null_threshold,
    resample_windows,
    _compute_fold_scores,
    FOLD_LOCAL_RADIUS,
)

FS = 3840.0
TP = 0.015
TIP = 2.0
K = 5
PF = 5e-2

N_W = int(np.ceil(TP * FS))
N_OL = N_W // 2
N_WS = N_W - N_OL
N_EXACT = TIP * FS / N_WS
N = int(np.floor(N_EXACT))
W, Wf = build_weighting_matrix(N_W, FS)
NFFT = W.shape[1]
FOLD_OFFSETS = np.round(np.arange(K) * N_EXACT).astype(int)
SAMPLES = compute_segment_samples(N_WS, N_OL, K, N_EXACT)


def _gaussian_iq(rng, n, sigma=0.01):
    return ((rng.standard_normal(n) + 1j * rng.standard_normal(n))
            * (sigma / np.sqrt(2.0))).astype(np.complex64)


def _inject_pulses(iq, amplitude, start_s=0.1, tip=TIP, count=K, f0=0.0):
    pulse_len = int(np.ceil(TP * FS))
    t = np.arange(pulse_len) / FS
    tone = amplitude * np.exp(2j * np.pi * f0 * t)
    for k in range(count):
        start = int(round((start_s + k * tip) * FS))
        end = min(start + pulse_len, iq.size)
        if start < iq.size:
            iq[start:end] += tone[:end - start].astype(np.complex64)


def _single_rate_search(power):
    n_time = power.shape[1]
    search_range = min(int(FOLD_OFFSETS[1]), n_time - FOLD_OFFSETS[-1])
    pulse_idx = np.arange(search_range)[:, None] + FOLD_OFFSETS[None, :]
    scores = np.max(_compute_fold_scores(power, pulse_idx, local_radius=FOLD_LOCAL_RADIUS), axis=1)
    return pulse_idx, scores


def _null_for(iq, n_perm=40, seed=0):
    power, _ = compute_stft_power(iq, N_W, N_OL, NFFT, W=W)
    noise = estimate_noise_power(power)
    pulse_idx, _ = _single_rate_search(power)
    return permutation_null_threshold(power, noise, PF, n_perm,
                                      pulse_idx=pulse_idx,
                                      rng=np.random.default_rng(seed))


# ---------------------------------------------------------------------------
# permutation_null_threshold
# ---------------------------------------------------------------------------

def test_null_returns_gumbel_fit_and_count():
    iq = _gaussian_iq(np.random.default_rng(1), SAMPLES)
    threshold, mu, sigma, n_perm = _null_for(iq, n_perm=30)
    assert n_perm == 30
    assert np.isfinite(threshold) and threshold > 0
    assert mu is not None and sigma is not None and sigma > 0
    assert threshold > mu  # pf = 5 % sits in the upper tail


def test_gaussian_null_controls_false_alarm_rate():
    # Fresh Gaussian slices scored against their own null must clear the
    # threshold about pf of the time, not 88 %.
    rng = np.random.default_rng(7)
    trials = 60
    false_alarms = 0
    for _ in range(trials):
        iq = _gaussian_iq(rng, SAMPLES)
        power, _ = compute_stft_power(iq, N_W, N_OL, NFFT, W=W)
        noise = estimate_noise_power(power)
        pulse_idx, scores = _single_rate_search(power)
        threshold, _, _, _ = permutation_null_threshold(
            power, noise, PF, 30, pulse_idx=pulse_idx, rng=rng)
        if np.max(scores / noise) > threshold:
            false_alarms += 1
    # Binomial(60, 0.05): mean 3, P(>10) < 1e-3.
    assert false_alarms <= 10, f'{false_alarms}/{trials} false alarms at pf={PF}'


def test_heavy_tailed_slice_raises_threshold():
    rng = np.random.default_rng(3)
    gaussian = _gaussian_iq(rng, SAMPLES)
    # Student-t (3 dof) noise: same scale, much heavier tails.
    heavy = ((rng.standard_t(3, SAMPLES) + 1j * rng.standard_t(3, SAMPLES))
             * (0.01 / np.sqrt(2.0))).astype(np.complex64)
    t_gauss, _, _, _ = _null_for(gaussian, seed=1)
    t_heavy, _, _, _ = _null_for(heavy, seed=1)
    assert t_heavy > 1.2 * t_gauss, (t_gauss, t_heavy)


def test_tag_present_never_lowers_threshold():
    rng = np.random.default_rng(11)
    clean = _gaussian_iq(rng, SAMPLES)
    with_tag = clean.copy()
    _inject_pulses(with_tag, amplitude=0.05)
    t_clean, _, _, _ = _null_for(clean, seed=2)
    t_tag, _, _, _ = _null_for(with_tag, seed=2)
    assert t_tag >= 0.98 * t_clean


def test_resampling_detected_windows_removes_tag_from_null():
    rng = np.random.default_rng(11)
    clean = _gaussian_iq(rng, SAMPLES)
    with_tag = clean.copy()
    _inject_pulses(with_tag, amplitude=0.05)
    power, _ = compute_stft_power(with_tag, N_W, N_OL, NFFT, W=W)
    noise = estimate_noise_power(power)
    pulse_idx, scores = _single_rate_search(power)
    b = int(np.argmax(scores / noise))
    t0 = int(np.argmax(_compute_fold_scores(power[b:b + 1], pulse_idx,
                                             local_radius=FOLD_LOCAL_RADIUS)[0]))
    on_idx = pulse_idx[t0]
    drop = np.concatenate([on_idx + d for d in range(-2, 3)])
    refined = resample_windows(power, drop, rng=np.random.default_rng(1))
    assert refined is not power and refined.shape == power.shape

    t_clean, _, _, _ = _null_for(clean, seed=2)
    t_inflated, _, _, _ = permutation_null_threshold(
        power, noise, PF, 40, pulse_idx=pulse_idx, rng=np.random.default_rng(2))
    t_refined, _, _, _ = permutation_null_threshold(
        refined, noise, PF, 40, pulse_idx=pulse_idx, rng=np.random.default_rng(2))
    assert t_inflated > 2.0 * t_clean
    assert t_refined < 1.3 * t_clean, (t_clean, t_inflated, t_refined)


def test_resample_windows_ignores_empty_or_total_drop():
    power = np.arange(12, dtype=np.float32).reshape(3, 4)
    assert resample_windows(power, []) is power
    assert resample_windows(power, [0, 1, 2, 3]) is power
    assert resample_windows(power, [-1, 99]) is power


def test_fold_detect_refines_null_when_tag_clears_first_pass():
    rng = np.random.default_rng(31)
    iq = _gaussian_iq(rng, SAMPLES)
    _inject_pulses(iq, amplitude=0.05)
    power, _ = compute_stft_power(iq, N_W, N_OL, NFFT, W=W)

    class Log:
        def __init__(self):
            self.records = []

        def emit(self, entry_type, human, **data):
            self.records.append((entry_type, data))

        def emit_raw(self, text):
            pass

    log = Log()
    detections, _, _ = fold_detect(
        power, N, PF, FS, NFFT, N_W, N_OL, SAMPLES, {},
        fold_offsets=FOLD_OFFSETS, W=W, Wf=Wf, k_folds=K,
        n_null_permutations=20, slog=log, cycle=7)
    assert len(detections) >= 1
    thr = [d for t, d in log.records if t == 'cycle_threshold']
    assert len(thr) == 1
    assert thr[0]['cycle'] == 7
    assert thr[0]['refined'] is True
    assert thr[0]['dropped_windows'] > 0
    assert thr[0]['n_perm'] == 20
    assert 'blanked_fraction' in thr[0]


def test_multi_hypothesis_null_matches_search_space():
    rng = np.random.default_rng(5)
    iq = _gaussian_iq(rng, SAMPLES)
    power, n_time = compute_stft_power(iq, N_W, N_OL, NFFT, W=W)
    noise = estimate_noise_power(power)
    hyps = build_hypothesis_indices(N_EXACT, K, n_time, N_B=None)
    threshold, mu, sigma, n_perm = permutation_null_threshold(
        power, noise, PF, 30, hypotheses=hyps, rng=np.random.default_rng(0))
    assert n_perm == 30 and np.isfinite(threshold) and sigma > 0


def test_frequency_mask_restricts_null_to_searched_bins():
    rng = np.random.default_rng(9)
    iq = _gaussian_iq(rng, SAMPLES)
    power, _ = compute_stft_power(iq, N_W, N_OL, NFFT, W=W)
    noise = estimate_noise_power(power)
    pulse_idx, _ = _single_rate_search(power)
    narrow = np.abs(Wf) <= 200.0
    t_full, _, _, _ = permutation_null_threshold(
        power, noise, PF, 40, pulse_idx=pulse_idx, rng=np.random.default_rng(0))
    t_narrow, _, _, _ = permutation_null_threshold(
        power, noise, PF, 40, pulse_idx=pulse_idx, frequency_mask=narrow,
        rng=np.random.default_rng(0))
    # Max over fewer bins is stochastically smaller.
    assert t_narrow < t_full


def test_time_budget_caps_permutations_but_keeps_minimum():
    rng = np.random.default_rng(12)
    iq = _gaussian_iq(rng, SAMPLES)
    power, _ = compute_stft_power(iq, N_W, N_OL, NFFT, W=W)
    noise = estimate_noise_power(power)
    pulse_idx, _ = _single_rate_search(power)
    threshold, _, _, n_perm = permutation_null_threshold(
        power, noise, PF, 200, pulse_idx=pulse_idx, time_budget_s=0.0,
        rng=np.random.default_rng(0))
    assert n_perm == NULL_MIN_PERMUTATIONS
    assert np.isfinite(threshold)


def test_too_few_permutations_is_rejected():
    rng = np.random.default_rng(2)
    iq = _gaussian_iq(rng, SAMPLES)
    threshold, mu, sigma, n_perm = _null_for(iq, n_perm=NULL_MIN_PERMUTATIONS - 1)
    assert np.isinf(threshold) and mu is None and sigma is None


def test_requires_exactly_one_search_description():
    power = np.ones((4, 100), dtype=np.float32)
    noise = np.ones(4)
    with pytest.raises(ValueError):
        permutation_null_threshold(power, noise, PF, 10)


def test_fold_detect_false_alarm_rate_on_pure_noise():
    # End to end, including the refinement step: pure Gaussian slices must
    # trip fold_detect about pf of the time.
    rng = np.random.default_rng(17)
    trials = 60
    false_alarms = 0
    for _ in range(trials):
        iq = _gaussian_iq(rng, SAMPLES)
        power, _ = compute_stft_power(iq, N_W, N_OL, NFFT, W=W)
        detections, _, _ = fold_detect(
            power, N, PF, FS, NFFT, N_W, N_OL, SAMPLES, {},
            fold_offsets=FOLD_OFFSETS, W=W, Wf=Wf, k_folds=K,
            n_null_permutations=20)
        false_alarms += bool(detections)
    assert false_alarms <= 10, f'{false_alarms}/{trials} false alarms at pf={PF}'


def test_fold_detect_uses_per_call_null_and_detects_tag():
    rng = np.random.default_rng(21)
    iq = _gaussian_iq(rng, SAMPLES)
    _inject_pulses(iq, amplitude=0.2)
    power, _ = compute_stft_power(iq, N_W, N_OL, NFFT, W=W)
    detections, _, _ = fold_detect(
        power, N, PF, FS, NFFT, N_W, N_OL, SAMPLES, {},
        fold_offsets=FOLD_OFFSETS, W=W, Wf=Wf, k_folds=K,
        n_null_permutations=20)
    assert len(detections) >= 1
    assert abs(detections[0].freq_hz) < 100.0
    assert detections[0].score_ratio > 1.0


def test_fold_detect_fixed_threshold_hook_bypasses_null():
    power = np.ones((8, 400), dtype=np.float32)
    power[2, 10] = 5000.0
    hyps = build_hypothesis_indices(50, K, 400, N_B=None)
    det, _, _ = fold_detect(power, 50, PF, FS, 8, 58, 29, 400 * 29 + 29,
                            {'fixed_threshold': 50.0}, hypotheses=hyps, k_folds=K)
    assert len(det) == 1


# ---------------------------------------------------------------------------
# blank_impulses
# ---------------------------------------------------------------------------

def test_blanking_disabled_returns_input():
    iq = np.ones(100, dtype=np.complex64)
    out, fraction = blank_impulses(iq, 0.0, 14)
    assert out is iq and fraction == 0.0


def test_blanking_removes_short_bursts_and_reports_fraction():
    rng = np.random.default_rng(4)
    iq = _gaussian_iq(rng, 20000)
    for pos in (100, 5000, 12345):
        iq[pos:pos + 3] += 1.0  # ~140x the median magnitude
    out, fraction = blank_impulses(iq, 6.0, 14)
    assert fraction == pytest.approx(9 / 20000)
    for pos in (100, 5000, 12345):
        assert np.all(out[pos:pos + 3] == 0)
    assert np.count_nonzero(out == 0) == 9


def test_blanking_leaves_full_pulses_alone():
    rng = np.random.default_rng(6)
    iq = _gaussian_iq(rng, SAMPLES)
    _inject_pulses(iq, amplitude=1.0)   # far above any blanking factor
    out, fraction = blank_impulses(iq, 6.0, N_W // 4)
    assert fraction == 0.0
    np.testing.assert_array_equal(out, iq)


def test_blanking_lowers_noise_floor_under_impulsive_noise():
    rng = np.random.default_rng(8)
    iq = _gaussian_iq(rng, SAMPLES)
    impulses = rng.choice(SAMPLES - 2, 200, replace=False)
    for pos in impulses:
        iq[pos:pos + 2] += 0.5
    power_raw, _ = compute_stft_power(iq, N_W, N_OL, NFFT, W=W)
    blanked, fraction = blank_impulses(iq, 6.0, N_W // 4)
    power_blank, _ = compute_stft_power(blanked, N_W, N_OL, NFFT, W=W)
    assert fraction > 0
    assert np.median(estimate_noise_power(power_blank)) < np.median(estimate_noise_power(power_raw))
    assert power_blank.max() < power_raw.max()
