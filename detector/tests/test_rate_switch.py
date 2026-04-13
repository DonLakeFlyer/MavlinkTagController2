"""Unit tests for multi-hypothesis rate-switch detection.

Tests cover:
  - Hypothesis index generation (pure and switch schedules)
  - Multi-hypothesis fold correctness
  - Fold quality (max_fold_fraction / dominant-fold check)
  - Segment length computation
  - EVT cache naming isolation
  - End-to-end fold_detect with synthetic signals
  - Regression: single-rate mode matches legacy behavior
"""

import os
import tempfile

import numpy as np
import pytest

from pulse_detector import (
    K,
    Detection,
    DOMINANT_FOLD_THRESHOLD,
    FOLD_LOCAL_RADIUS,
    HYP_GROUP_IND_A,
    HYP_GROUP_IND_B,
    OCCAM_MARGIN,
    build_hypothesis_indices,
    build_weighting_matrix,
    compute_segment_samples,
    compute_stft_power,
    fold_detect,
    fold_multi_hypothesis,
    hyp_label_to_group_ind,
    _evt_cache_path,
    load_evt_cache,
    save_evt_cache,
)


# ---------------------------------------------------------------------------
# Hypothesis index generation
# ---------------------------------------------------------------------------

class TestBuildHypothesisIndices:
    """Tests for build_hypothesis_indices()."""

    N_A = 265
    N_B = 200
    N_TIME = 1400  # comfortably larger than any hypothesis span

    def test_single_rate_produces_one_hypothesis(self):
        hyps = build_hypothesis_indices(self.N_A, K, self.N_TIME, N_B=None)
        assert len(hyps) == 1
        assert hyps[0][0] == "A"

    def test_two_rates_produce_eight_hypotheses(self):
        hyps = build_hypothesis_indices(self.N_A, K, self.N_TIME, N_B=self.N_B)
        assert len(hyps) == 8
        labels = [h[0] for h in hyps]
        assert "A" in labels
        assert "B" in labels
        for c in range(1, K - 1):
            assert f"A_to_B_c{c}" in labels
            assert f"B_to_A_c{c}" in labels

    def test_pure_A_indices(self):
        hyps = build_hypothesis_indices(self.N_A, K, self.N_TIME, N_B=None)
        label, idx = hyps[0]
        assert label == "A"
        # Every row should be t0, t0+N_A, t0+2*N_A, ...
        for row in range(idx.shape[0]):
            t0 = idx[row, 0]
            for k in range(K):
                assert idx[row, k] == t0 + k * self.N_A

    def test_pure_B_indices(self):
        hyps = build_hypothesis_indices(self.N_A, K, self.N_TIME, N_B=self.N_B)
        b_hyps = [(l, i) for l, i in hyps if l == "B"]
        assert len(b_hyps) == 1
        _, idx = b_hyps[0]
        for row in range(idx.shape[0]):
            t0 = idx[row, 0]
            for k in range(K):
                assert idx[row, k] == t0 + k * self.N_B

    def test_switch_A_to_B_indices(self):
        hyps = build_hypothesis_indices(self.N_A, K, self.N_TIME, N_B=self.N_B)
        for c in range(1, K - 1):
            matched = [(l, i) for l, i in hyps if l == f"A_to_B_c{c}"]
            assert len(matched) == 1
            _, idx = matched[0]
            row = idx[0]
            t0 = row[0]
            # First c gaps use N_A, remaining gaps use N_B
            expected = [t0]
            for k in range(K - 1):
                spacing = self.N_A if k < c else self.N_B
                expected.append(expected[-1] + spacing)
            np.testing.assert_array_equal(row, expected)

    def test_switch_B_to_A_indices(self):
        hyps = build_hypothesis_indices(self.N_A, K, self.N_TIME, N_B=self.N_B)
        for c in range(1, K - 1):
            matched = [(l, i) for l, i in hyps if l == f"B_to_A_c{c}"]
            assert len(matched) == 1
            _, idx = matched[0]
            row = idx[0]
            t0 = row[0]
            expected = [t0]
            for k in range(K - 1):
                spacing = self.N_B if k < c else self.N_A
                expected.append(expected[-1] + spacing)
            np.testing.assert_array_equal(row, expected)

    def test_all_indices_in_bounds(self):
        hyps = build_hypothesis_indices(self.N_A, K, self.N_TIME, N_B=self.N_B)
        for label, idx in hyps:
            assert np.all(idx >= 0), f"{label}: negative indices"
            assert np.all(idx < self.N_TIME), f"{label}: index >= n_time"

    def test_same_rate_gives_single_hypothesis(self):
        """N_B == N_A should produce only pure-A (no switch hypotheses)."""
        hyps = build_hypothesis_indices(self.N_A, K, self.N_TIME, N_B=self.N_A)
        assert len(hyps) == 1
        assert hyps[0][0] == "A"

    def test_small_n_time_reduces_hypotheses(self):
        """n_time too small for the longest hypothesis should drop it."""
        # Span for pure-B with N_B=500: (K-1)*500 = 2000 windows
        # With n_time=1500, pure-B should be dropped
        hyps = build_hypothesis_indices(200, K, 1500, N_B=500)
        labels = [h[0] for h in hyps]
        assert "A" in labels
        assert "B" not in labels  # span 2000 > 1500

    def test_hypothesis_shape(self):
        """Each pulse_idx should be (search_range, K)."""
        hyps = build_hypothesis_indices(self.N_A, K, self.N_TIME, N_B=self.N_B)
        for label, idx in hyps:
            assert idx.ndim == 2, f"{label}: wrong ndim"
            assert idx.shape[1] == K, f"{label}: wrong K dimension"
            assert idx.shape[0] > 0, f"{label}: empty search range"

    def test_switch_hypotheses_search_full_valid_offset_range(self):
        """Switch hypotheses must search all valid t0 values up to max_start.

        For mixed schedules (A->B / B->A), limiting search to one period of
        the first rate can miss valid alignments later in the segment.
        """
        hyps = build_hypothesis_indices(self.N_A, K, self.N_TIME, N_B=self.N_B)
        hyp_map = {label: idx for label, idx in hyps}

        # A_to_B_c1 gaps: [A, B, B, B]
        idx_ab_c1 = hyp_map["A_to_B_c1"]
        span_ab_c1 = self.N_A + (K - 2) * self.N_B
        max_start_ab_c1 = self.N_TIME - span_ab_c1
        assert idx_ab_c1.shape[0] == max_start_ab_c1

        # B_to_A_c1 gaps: [B, A, A, A]
        idx_ba_c1 = hyp_map["B_to_A_c1"]
        span_ba_c1 = self.N_B + (K - 2) * self.N_A
        max_start_ba_c1 = self.N_TIME - span_ba_c1
        assert idx_ba_c1.shape[0] == max_start_ba_c1

    def test_fractional_pri_rounds_independently(self):
        """Fractional PRI: each offset independently rounded, drift ≤ 0.5 windows."""
        N_A_exact = 264.8276  # realistic REST rate
        hyps = build_hypothesis_indices(N_A_exact, K, 2000, N_B=None)
        assert len(hyps) == 1
        _, idx = hyps[0]
        row = idx[0]
        t0 = row[0]
        for k in range(K):
            expected = int(np.round(k * N_A_exact))
            assert row[k] == t0 + expected, (
                f"fold {k}: got {row[k] - t0}, expected {expected}")

    def test_fractional_pri_matches_fold_offsets(self):
        """Hypothesis indices should match the fold_offsets rounding strategy."""
        N_A_exact = 264.8276
        hyps = build_hypothesis_indices(N_A_exact, K, 2000, N_B=None)
        _, idx = hyps[0]
        offsets_from_hyp = idx[0] - idx[0, 0]
        fold_offsets = np.round(np.arange(K) * N_A_exact).astype(int)
        np.testing.assert_array_equal(offsets_from_hyp, fold_offsets)

    def test_fractional_switch_offsets(self):
        """Switch hypothesis with fractional PRI uses correct cumulative rounding."""
        N_A_exact = 264.8276
        N_B_exact = 176.5076
        hyps = build_hypothesis_indices(N_A_exact, K, 2000, N_B=N_B_exact)
        matched = [(l, i) for l, i in hyps if l == "A_to_B_c2"]
        assert len(matched) == 1
        _, idx = matched[0]
        row = idx[0]
        t0 = row[0]
        # c=2: gaps are [A, A, B, B]
        spacings = [N_A_exact, N_A_exact, N_B_exact, N_B_exact]
        cumulative = 0.0
        for k in range(1, K):
            cumulative += spacings[k - 1]
            expected = int(np.round(cumulative))
            assert row[k] - t0 == expected, (
                f"fold {k}: got {row[k] - t0}, expected {expected}")


# ---------------------------------------------------------------------------
# Multi-hypothesis fold
# ---------------------------------------------------------------------------

class TestFoldMultiHypothesis:

    def test_known_signal_correct_hypothesis_wins(self):
        """Inject signal at A_to_B_c2 positions; that hypothesis must win."""
        N_A, N_B = 50, 30
        n_time = 500
        n_freq = 10
        rng = np.random.RandomState(42)

        power = rng.exponential(1.0, (n_freq, n_time)).astype(np.float32)

        # Place strong signal at freq bin 3, with A_to_B_c2 schedule
        signal_bin = 3
        signal_power = 100.0
        t0 = 5
        # c=2: first 2 gaps N_A, last 2 gaps N_B
        positions = [t0, t0 + N_A, t0 + 2*N_A, t0 + 2*N_A + N_B, t0 + 2*N_A + 2*N_B]
        for p in positions:
            power[signal_bin, p] += signal_power

        hyps = build_hypothesis_indices(N_A, K, n_time, N_B=N_B)
        best_scores, best_offsets, best_labels = fold_multi_hypothesis(power, hyps)

        assert best_labels[signal_bin] == "A_to_B_c2"
        # Score should be approximately 5 * signal_power (plus noise)
        assert best_scores[signal_bin] > 4 * signal_power

    def test_pure_signal_detected_by_pure_hypothesis(self):
        """Signal at pure-A positions should be found by hypothesis A."""
        N_A, N_B = 50, 30
        n_time = 500
        n_freq = 10
        rng = np.random.RandomState(99)

        power = rng.exponential(1.0, (n_freq, n_time)).astype(np.float32)
        signal_bin = 7
        t0 = 10
        for k in range(K):
            power[signal_bin, t0 + k * N_A] += 80.0

        hyps = build_hypothesis_indices(N_A, K, n_time, N_B=N_B)
        best_scores, _, best_labels = fold_multi_hypothesis(power, hyps)

        assert best_labels[signal_bin] == "A"

    def test_single_rate_fold_matches_manual(self):
        """Single-rate fold via multi-hypothesis matches manual sum."""
        N_A = 50
        n_time = 300
        n_freq = 5
        rng = np.random.RandomState(7)
        power = rng.exponential(1.0, (n_freq, n_time)).astype(np.float32)

        hyps = build_hypothesis_indices(N_A, K, n_time, N_B=None)
        best_scores, best_offsets, best_labels = fold_multi_hypothesis(power, hyps)

        # Manual computation
        search_range = hyps[0][1].shape[0]
        pulse_idx = (np.arange(search_range)[:, None]
                     + np.arange(K)[None, :] * N_A)
        # Match detector statistic: per-fold local max over idx±radius.
        idx0 = np.clip(pulse_idx, 0, n_time - 1)
        local = power[:, idx0]
        for d in range(1, FOLD_LOCAL_RADIUS + 1):
            idx_m = np.clip(pulse_idx - d, 0, n_time - 1)
            idx_p = np.clip(pulse_idx + d, 0, n_time - 1)
            local = np.maximum(local, power[:, idx_m])
            local = np.maximum(local, power[:, idx_p])
        expected_scores = np.sum(local, axis=2)
        expected_best = np.max(expected_scores, axis=1)
        expected_offsets = np.argmax(expected_scores, axis=1)

        np.testing.assert_allclose(best_scores, expected_best, rtol=1e-5)
        np.testing.assert_array_equal(best_offsets, expected_offsets)

    def test_occam_prefers_pure_when_close(self):
        """When a switch hypothesis barely beats a pure hypothesis,
        Occam preference should select the pure one."""
        N_A, N_B = 50, 30
        n_time = 500
        n_freq = 5
        rng = np.random.RandomState(88)

        power = rng.exponential(1.0, (n_freq, n_time)).astype(np.float32)

        # Inject a pure-B signal at bin 2
        signal_bin = 2
        t0 = 10
        for k in range(K):
            power[signal_bin, t0 + k * N_B] += 80.0

        # Inject a pure-B signal at bin 3 as well, then add a tiny bump
        # at the one A-rate position that A_to_B_c1 sees but pure-B doesn't.
        # This makes A_to_B_c1 barely beat pure B (within OCCAM_MARGIN),
        # so Occam should override and select "B".
        occam_bin = 3
        t0_b = 5
        for k in range(K):
            power[occam_bin, t0_b + k * N_B] += 80.0
        # A_to_B_c1 first fold is at A-rate offset from t0.
        # Add a small bump so the switch scores ~2% higher than pure B.
        power[occam_bin, t0_b + N_A] += 2.0

        hyps = build_hypothesis_indices(N_A, K, n_time, N_B=N_B)
        _, _, best_labels = fold_multi_hypothesis(power, hyps)

        # Bin 2: pure B signal → must be labeled "B" (no switch)
        assert best_labels[signal_bin] == "B"
        # Bin 3: marginal switch competition → Occam prefers pure B
        assert best_labels[occam_bin] == "B"

    def test_occam_does_not_override_clear_switch(self):
        """When a switch hypothesis wins by more than OCCAM_MARGIN,
        the switch label should be preserved."""
        N_A, N_B = 50, 30
        n_time = 500
        n_freq = 5
        rng = np.random.RandomState(42)

        power = rng.exponential(1.0, (n_freq, n_time)).astype(np.float32)

        # Inject a clear A_to_B_c2 signal at bin 3
        signal_bin = 3
        signal_power = 100.0
        t0 = 5
        positions = [t0, t0 + N_A, t0 + 2*N_A,
                     t0 + 2*N_A + N_B, t0 + 2*N_A + 2*N_B]
        for p in positions:
            power[signal_bin, p] += signal_power

        hyps = build_hypothesis_indices(N_A, K, n_time, N_B=N_B)
        _, _, best_labels = fold_multi_hypothesis(power, hyps)

        # The switch signal is genuinely better than pure B at this bin,
        # so Occam should NOT override it.
        assert best_labels[signal_bin] == "A_to_B_c2"


# ---------------------------------------------------------------------------
# Max fold fraction (dominant-fold check)
# ---------------------------------------------------------------------------

class TestMaxFoldFraction:
    """Verify the dominant-fold ratio used to derive max_fold_fraction from on_powers."""

    def test_uniform_signal_low_fraction(self):
        """Equal-power folds should produce max_fold_fraction = 1/K."""
        on_powers = np.ones(K) * 10.0
        mff = float(np.max(on_powers) / np.sum(on_powers))
        assert mff == pytest.approx(1.0 / K, rel=1e-5)

    def test_one_hot_high_fraction(self):
        """One dominant fold should produce max_fold_fraction near 1.0."""
        on_powers = np.ones(K) * 0.01
        on_powers[2] = 100.0
        mff = float(np.max(on_powers) / np.sum(on_powers))
        assert mff > 0.95

    def test_moderate_variation_below_threshold(self):
        """Real-world power variation (e.g. 62-81 dB) should be well below 0.8."""
        # Simulate the real-world case from logs
        on_powers = np.array([3.858e-03, 4.892e-03, 5.726e-03, 2.893e-02,
                              2.649e-01, 2.802e-01, 4.865e-02, 1.119e-01,
                              5.899e-02, 5.587e-02, 4.953e-02, 4.277e-02,
                              5.757e-02, 6.722e-02, 6.539e-02, 3.279e-02,
                              2.956e-02, 2.576e-02, 2.378e-02, 1.773e-02])
        mff = float(np.max(on_powers) / np.sum(on_powers))
        assert mff < 0.3  # 0.22 in practice
        assert mff < 0.8  # well below dominant-fold threshold


# ---------------------------------------------------------------------------
# Segment length computation
# ---------------------------------------------------------------------------

class TestComputeSegmentSamples:

    def test_single_rate(self):
        n_ws, n_ol, N_A = 29, 29, 265
        result = compute_segment_samples(n_ws, n_ol, K, N_A, N_B=None)
        expected = n_ws * (K * N_A + 1) + n_ol
        assert result == expected

    def test_dual_rate_uses_max_span(self):
        n_ws, n_ol = 29, 29
        N_A, N_B = 200, 300
        result = compute_segment_samples(n_ws, n_ol, K, N_A, N_B=N_B)
        # max N = 300, total windows needed = K * 300
        expected = n_ws * (K * N_B + 1) + n_ol
        assert result == expected

    def test_dual_rate_A_larger(self):
        n_ws, n_ol = 29, 29
        N_A, N_B = 300, 200
        result = compute_segment_samples(n_ws, n_ol, K, N_A, N_B=N_B)
        expected = n_ws * (K * N_A + 1) + n_ol
        assert result == expected


# ---------------------------------------------------------------------------
# EVT cache naming
# ---------------------------------------------------------------------------

class TestEvtCacheNaming:

    def test_new_format_differs_from_legacy(self):
        """New cache path must not match legacy format."""
        new_path = _evt_cache_path('/tmp', 265, 5, N_B=200, n_hypotheses=8)
        # Legacy format would contain 'M0.000000-J0.000000'
        assert 'M0.000000' not in new_path
        assert 'Nb' in new_path
        assert 'H8' in new_path

    def test_single_rate_new_differs_from_legacy(self):
        """Even single-rate new format must not collide with legacy."""
        new_path = _evt_cache_path('/tmp', 265, 5, N_B=None, n_hypotheses=1)
        assert 'Nb0' in new_path
        assert 'H1' in new_path
        assert 'M0.000000' not in new_path

    def test_different_N_B_gives_different_path(self):
        path1 = _evt_cache_path('/tmp', 265, 5, N_B=200, n_hypotheses=8)
        path2 = _evt_cache_path('/tmp', 265, 5, N_B=300, n_hypotheses=8)
        assert path1 != path2

    def test_round_trip_save_load(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            save_evt_cache(tmpdir, 265, 5, 1.234, 0.567,
                           N_B=200, n_hypotheses=8)
            mu, sigma = load_evt_cache(tmpdir, 265, 5,
                                       N_B=200, n_hypotheses=8)
            assert mu == pytest.approx(1.234, abs=1e-10)
            assert sigma == pytest.approx(0.567, abs=1e-10)

    def test_load_mismatched_N_B_returns_none(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            save_evt_cache(tmpdir, 265, 5, 1.234, 0.567,
                           N_B=200, n_hypotheses=8)
            mu, sigma = load_evt_cache(tmpdir, 265, 5,
                                       N_B=300, n_hypotheses=8)
            assert mu is None
            assert sigma is None


# ---------------------------------------------------------------------------
# End-to-end fold_detect with synthetic data
# ---------------------------------------------------------------------------

class TestFoldDetectEndToEnd:
    """Integration tests using compute_stft_power → fold_detect."""

    FS = 3840.0
    TP = 0.015
    TIP_A = 2.0
    TIP_B = 1.5

    @staticmethod
    def _make_geometry(fs, tp, tip):
        n_w = int(np.ceil(tp * fs))
        n_ol = n_w // 2
        n_ws = n_w - n_ol
        N = int(np.floor(tip * fs / n_ws))
        return n_w, n_ol, n_ws, N

    def _inject_pulses(self, iq, fs, tp, positions_sec, amplitude=0.5):
        """Inject rectangular pulses at given times into IQ."""
        pulse_len = int(np.ceil(tp * fs))
        for t in positions_sec:
            start = int(round(t * fs))
            end = min(start + pulse_len, len(iq))
            if start < len(iq):
                iq[start:end] += amplitude

    def test_single_rate_detects_pure_A(self):
        n_w, n_ol, n_ws, N_A = self._make_geometry(self.FS, self.TP, self.TIP_A)
        samples = compute_segment_samples(n_ws, n_ol, K, N_A)
        W, Wf = build_weighting_matrix(n_w, self.FS)
        nfft = W.shape[1]

        rng = np.random.RandomState(42)
        iq = (rng.randn(samples) + 1j * rng.randn(samples)).astype(np.complex64) * 0.01

        # Inject 5 pulses at rate A near DC — strong enough to exceed EVT threshold
        for k in range(K):
            self._inject_pulses(iq, self.FS, self.TP, [k * self.TIP_A + 0.1], amplitude=2.0)

        power, n_time = compute_stft_power(iq, n_w, n_ol, nfft, W=W)
        evt_cache = {}

        detections, _, _ = fold_detect(
            power, N_A, 5e-2, self.FS, nfft, n_w, n_ol, samples,
            evt_cache, W=W, Wf=Wf, hypotheses=None, N_B=None)

        assert len(detections) >= 1
        assert detections[0].hyp_label == "A"

    def test_dual_rate_detects_switch(self):
        n_w, n_ol, n_ws, N_A = self._make_geometry(self.FS, self.TP, self.TIP_A)
        _, _, _, N_B = self._make_geometry(self.FS, self.TP, self.TIP_B)
        samples = compute_segment_samples(n_ws, n_ol, K, N_A, N_B)
        W, Wf = build_weighting_matrix(n_w, self.FS)
        nfft = W.shape[1]
        n_time_est = (samples - n_ol) // n_ws

        rng = np.random.RandomState(42)
        iq = (rng.randn(samples) + 1j * rng.randn(samples)).astype(np.complex64) * 0.01

        # Inject A_to_B_c2: pulses 0,1,2 at rate A, then 3,4 at rate B
        times = [0.1]
        for k in range(1, K):
            if k < 3:  # change-point at c=2
                times.append(times[-1] + self.TIP_A)
            else:
                times.append(times[-1] + self.TIP_B)
        for t in times:
            self._inject_pulses(iq, self.FS, self.TP, [t], amplitude=2.0)

        power, n_time = compute_stft_power(iq, n_w, n_ol, nfft, W=W)
        hyps = build_hypothesis_indices(N_A, K, n_time, N_B=N_B)
        evt_cache = {}

        detections, _, _ = fold_detect(
            power, N_A, 5e-2, self.FS, nfft, n_w, n_ol, samples,
            evt_cache, W=W, Wf=Wf, hypotheses=hyps, N_B=N_B)

        assert len(detections) >= 1
        # Should detect with a switch hypothesis (A_to_B_c2)
        assert "A_to_B" in detections[0].hyp_label, \
            f"Expected A_to_B switch hypothesis, got {detections[0].hyp_label}"

    def test_no_regression_single_rate(self):
        """Single-rate with hypotheses=None must produce same scores as legacy."""
        n_w, n_ol, n_ws, N_A = self._make_geometry(self.FS, self.TP, self.TIP_A)
        samples = compute_segment_samples(n_ws, n_ol, K, N_A)
        W, Wf = build_weighting_matrix(n_w, self.FS)
        nfft = W.shape[1]

        rng = np.random.RandomState(123)
        iq = (rng.randn(samples) + 1j * rng.randn(samples)).astype(np.complex64) * 0.01
        for k in range(K):
            self._inject_pulses(iq, self.FS, self.TP, [k * self.TIP_A + 0.1], amplitude=2.0)

        power, n_time = compute_stft_power(iq, n_w, n_ol, nfft, W=W)

        # Run with hypotheses=None (legacy path)
        cache1 = {}
        det1, _, _ = fold_detect(
            power, N_A, 5e-2, self.FS, nfft, n_w, n_ol, samples,
            cache1, W=W, Wf=Wf, hypotheses=None)

        # Run with explicit single-rate hypotheses
        hyps = build_hypothesis_indices(N_A, K, n_time, N_B=None)
        cache2 = {}
        det2, _, _ = fold_detect(
            power, N_A, 5e-2, self.FS, nfft, n_w, n_ol, samples,
            cache2, W=W, Wf=Wf, hypotheses=hyps)

        # Both should detect (or not) at the same bins with same SNR
        assert len(det1) == len(det2)
        if det1:
            # Compare freq and SNR of top detection
            assert det1[0].freq_hz == pytest.approx(det2[0].freq_hz, abs=1.0)
            assert det1[0].snr_db == pytest.approx(det2[0].snr_db, abs=0.5)

    def test_one_hot_has_dominant_fold(self):
        """A one-hot signal should produce max_fold_fraction > 0.8."""
        N_A = 50
        n_time = 500
        n_freq = 20

        rng = np.random.RandomState(77)
        power = rng.exponential(0.5, (n_freq, n_time)).astype(np.float32)

        # Place signal in one window only at bin 5
        signal_bin = 5
        t0 = 10
        power[signal_bin, t0] += 500.0

        hyps = build_hypothesis_indices(N_A, K, n_time, N_B=None)

        # Use a trivially low threshold to force detection
        evt_cache = {'threshold': 1.0}
        n_w_fake = 58
        n_ol_fake = 29
        det, _, _ = fold_detect(
            power, N_A, 5e-2, 3840.0, n_freq, n_w_fake, n_ol_fake,
            n_time * 29 + 29, evt_cache, hypotheses=hyps)

        # The one-hot signal bin should be detected (not rejected)
        # but must have max_fold_fraction > 0.8 indicating a dominant fold.
        freq_axis = np.fft.fftshift(np.fft.fftfreq(n_freq, d=1.0 / 3840.0))
        signal_freq = freq_axis[signal_bin]
        found_one_hot = False
        for d in det:
            if abs(d.freq_hz - signal_freq) < 1.0:
                found_one_hot = True
                assert d.fold_info['max_fold_fraction'] > 0.8, (
                    f"One-hot detection should have max_fold_fraction > 0.8, "
                    f"got {d.fold_info['max_fold_fraction']:.3f}"
                )
        assert found_one_hot, "One-hot signal bin should be detected (not discarded)"

    def test_detection_not_discarded(self):
        """Detections are never discarded by fold quality — only downgraded.
        A one-hot spike that exceeds threshold must still be returned."""
        N_A = 50
        n_time = 500
        n_freq = 20

        power = np.ones((n_freq, n_time), dtype=np.float32) * 0.5

        signal_bin = 5
        power[signal_bin, 10] = 5000.0

        hyps = build_hypothesis_indices(N_A, K, n_time, N_B=None)

        evt_cache = {'threshold': 50.0}
        n_w_fake = 58
        n_ol_fake = 29
        det, noise_psd, best_cand = fold_detect(
            power, N_A, 5e-2, 3840.0, n_freq, n_w_fake, n_ol_fake,
            n_time * 29 + 29, evt_cache, hypotheses=hyps)

        # Detection should NOT be filtered out — fold quality only affects
        # confidence, not whether the detection is returned.
        assert len(det) >= 1
        assert det[0].fold_info['max_fold_fraction'] > DOMINANT_FOLD_THRESHOLD

    def test_one_hot_yields_subthreshold_confidence(self):
        """A one-hot spike with high score_ratio should still be classified as
        SUBTHRESHOLD because max_fold_fraction exceeds DOMINANT_FOLD_THRESHOLD.
        This tests the confidence classification policy."""
        N_A = 50
        n_time = 500
        n_freq = 20

        power = np.ones((n_freq, n_time), dtype=np.float32) * 0.5
        signal_bin = 5
        power[signal_bin, 10] = 5000.0

        hyps = build_hypothesis_indices(N_A, K, n_time, N_B=None)
        evt_cache = {'threshold': 50.0}
        det, _, _ = fold_detect(
            power, N_A, 5e-2, 3840.0, n_freq, 58, 29,
            n_time * 29 + 29, evt_cache, hypotheses=hyps)

        assert len(det) >= 1
        d = det[0]
        # The detection has a high score_ratio (well above any confidence_ratio)
        # but the dominant fold should trigger SUBTHRESHOLD classification.
        has_dominant_fold = d.fold_info['max_fold_fraction'] > DOMINANT_FOLD_THRESHOLD
        assert has_dominant_fold, (
            f"Expected dominant fold (>{DOMINANT_FOLD_THRESHOLD}), "
            f"got {d.fold_info['max_fold_fraction']:.3f}")
        assert d.score_ratio > 1.3, (
            f"Expected high score_ratio for classification test, got {d.score_ratio:.3f}")
        # Apply the same classification logic as the runtime reporting loop:
        is_marginal = d.score_ratio < 1.3 or has_dominant_fold
        assert is_marginal, "One-hot spike should be classified as marginal"
        # Verify the derived outputs match expected policy:
        # detection_status = SUBTHRESHOLD, confirmed_status = 0
        from pulse_detector import DETECTION_STATUS_SUBTHRESHOLD
        det_status = DETECTION_STATUS_SUBTHRESHOLD if is_marginal else 1
        confirmed = 0 if is_marginal else 1
        assert det_status == DETECTION_STATUS_SUBTHRESHOLD
        assert confirmed == 0


# ---------------------------------------------------------------------------
# Fold multi-hypothesis regression test
# ---------------------------------------------------------------------------

class TestFoldRegressionSingleRate:
    """Verify fold_multi_hypothesis with N_B=None matches direct computation."""

    def test_scores_match_direct(self):
        N_A = 100
        n_time = 600
        n_freq = 20
        rng = np.random.RandomState(55)
        power = rng.exponential(1.0, (n_freq, n_time)).astype(np.float32)

        # Direct computation (legacy style)
        max_start = n_time - (K - 1) * N_A
        search_range = min(N_A, max_start)
        pulse_idx = (np.arange(search_range)[:, None]
                     + np.arange(K)[None, :] * N_A)
        idx0 = np.clip(pulse_idx, 0, n_time - 1)
        local = power[:, idx0]
        for d in range(1, FOLD_LOCAL_RADIUS + 1):
            idx_m = np.clip(pulse_idx - d, 0, n_time - 1)
            idx_p = np.clip(pulse_idx + d, 0, n_time - 1)
            local = np.maximum(local, power[:, idx_m])
            local = np.maximum(local, power[:, idx_p])
        fold_scores = np.sum(local, axis=2)
        expected_best = np.max(fold_scores, axis=1)
        expected_offsets = np.argmax(fold_scores, axis=1)

        # Multi-hypothesis (single rate)
        hyps = build_hypothesis_indices(N_A, K, n_time, N_B=None)
        best_scores, best_offsets, _ = fold_multi_hypothesis(power, hyps)

        np.testing.assert_allclose(best_scores, expected_best, rtol=1e-5)
        np.testing.assert_array_equal(best_offsets, expected_offsets)


# ---------------------------------------------------------------------------
# Hypothesis → group_ind encoding and predict_next_s logic
# ---------------------------------------------------------------------------

class TestHypLabelToGroupInd:
    """Tests for hyp_label_to_group_ind() mapping."""

    def test_pure_a(self):
        gind, last_rate = hyp_label_to_group_ind('A', K=5)
        assert gind == HYP_GROUP_IND_A
        assert last_rate == 'A'

    def test_pure_b(self):
        gind, last_rate = hyp_label_to_group_ind('B', K=5)
        assert gind == HYP_GROUP_IND_B
        assert last_rate == 'B'

    def test_a_to_b_c1(self):
        gind, last_rate = hyp_label_to_group_ind('A_to_B_c1', K=5)
        assert gind == 2  # 1 + c = 1 + 1
        assert last_rate == 'B'

    def test_a_to_b_c3(self):
        gind, last_rate = hyp_label_to_group_ind('A_to_B_c3', K=5)
        assert gind == 4  # 1 + 3
        assert last_rate == 'B'

    def test_b_to_a_c1(self):
        gind, last_rate = hyp_label_to_group_ind('B_to_A_c1', K=5)
        assert gind == 5  # K - 1 + c = 4 + 1
        assert last_rate == 'A'

    def test_b_to_a_c3(self):
        gind, last_rate = hyp_label_to_group_ind('B_to_A_c3', K=5)
        assert gind == 7  # K - 1 + c = 4 + 3
        assert last_rate == 'A'

    def test_unknown_label_defaults_a(self):
        gind, last_rate = hyp_label_to_group_ind('something_weird', K=5)
        assert gind == HYP_GROUP_IND_A
        assert last_rate == 'A'

    def test_single_rate_always_zero(self):
        """Single-rate tags always produce label 'A'."""
        gind, last_rate = hyp_label_to_group_ind('A', K=5)
        assert gind == 0

    def test_group_ind_ranges_no_overlap_k5(self):
        """With K=5, A→B and B→A group_ind ranges should not collide
        with pure A (0), pure B (1), or each other."""
        a_to_b_inds = set()
        b_to_a_inds = set()
        for c in range(1, 4):  # K-1 = 4, c ranges 1..3
            g, _ = hyp_label_to_group_ind(f'A_to_B_c{c}', K=5)
            a_to_b_inds.add(g)
            g, _ = hyp_label_to_group_ind(f'B_to_A_c{c}', K=5)
            b_to_a_inds.add(g)
        assert HYP_GROUP_IND_A not in a_to_b_inds
        assert HYP_GROUP_IND_B not in a_to_b_inds
        assert HYP_GROUP_IND_A not in b_to_a_inds
        assert HYP_GROUP_IND_B not in b_to_a_inds
        # A→B and B→A ranges must be disjoint
        assert a_to_b_inds.isdisjoint(b_to_a_inds), (
            f"Overlap between A→B {a_to_b_inds} and B→A {b_to_a_inds}"
        )

    def test_last_rate_determines_predict_tip(self):
        """Verify the last_rate field correctly selects the right TIP."""
        tip_a = 1.0
        tip_b = 1.5
        # A_to_B → last gap is B → predict uses tip_b
        _, last = hyp_label_to_group_ind('A_to_B_c2', K=5)
        predict_tip = tip_b if last == 'B' else tip_a
        assert predict_tip == tip_b
        # B_to_A → last gap is A → predict uses tip_a
        _, last = hyp_label_to_group_ind('B_to_A_c2', K=5)
        predict_tip = tip_b if last == 'B' else tip_a
        assert predict_tip == tip_a
