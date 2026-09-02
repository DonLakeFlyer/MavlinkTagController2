"""Deterministic tests for the iq_replay decimator mirror and the
external-lock fixed-offset amplitude path.

Guards the claim that FirDecimStage matches the production decimator
(decimator/src/main.cpp): filter design, output phase (first output after q
inputs), and chunk-boundary state/phase continuity. Also verifies
amplitude_at_known_pulse at a supplied (bin, fold offset) without
re-maximization, including on a held-out segment with a translated offset.
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from iq_replay import (  # noqa: E402
    DETECTOR_FS, FirDecimStage, amplitude_at_known_pulse, design_lowpass,
    estimate_noise,
)
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "detector"))
from pulse_detector import (  # noqa: E402
    build_hypothesis_indices, build_weighting_matrix, compute_stft_power,
)

RNG = np.random.default_rng(12345)

# Golden coefficients frozen from an independent evaluation of the production
# formula (decimator/src/main.cpp designLowpass): Hamming-windowed sinc,
# 16q+1 taps, cutoff 0.45/q, DC-normalized. Guards against a filter-design
# regression shifting both sides of the parity tests below.
# Indices sampled: 0, q-1, center-1, center, center+7.
GOLDEN_TAPS = {
    8: {0: -0.0002335454671943469, 7: 0.0005736994169394769,
        63: 0.10995754673666425, 64: 0.11234277298381225,
        71: 0.027356675555238602},
    5: {0: -0.0003737358495574011, 4: 0.0009035224316659152,
        39: 0.17010779954080812, 40: 0.17977879085874043,
        47: -0.030863325405345254},
}


def reference_decimate(x, q):
    """Independent reference: full convolution + take every q-th output,
    starting at index q-1 (production emits after q inputs)."""
    b = design_lowpass(q * 16, 0.45 / q)
    y = np.convolve(x, b)[: len(x)]
    return y[q - 1 :: q]


def run_chain(x, chunk):
    stages = [FirDecimStage(q) for q in (8, 5, 5)]
    out = []
    for i in range(0, len(x), chunk):
        y = x[i : i + chunk]
        for st in stages:
            y = st.process(y)
        if len(y):
            out.append(y)
    return np.concatenate(out) if out else np.array([], dtype=np.complex128)


def test_lowpass_design_properties():
    for q in (8, 5):
        b = design_lowpass(q * 16, 0.45 / q)
        assert len(b) == q * 16 + 1  # odd taps, 16q+1
        np.testing.assert_allclose(b.sum(), 1.0, atol=1e-12)  # DC-normalized
        np.testing.assert_allclose(b, b[::-1], atol=1e-15)  # linear phase


def test_lowpass_matches_golden_coefficients():
    for q, golden in GOLDEN_TAPS.items():
        b = design_lowpass(q * 16, 0.45 / q)
        for i, want in golden.items():
            np.testing.assert_allclose(b[i], want, rtol=1e-14,
                                       err_msg=f"q={q} tap {i}")


def test_single_stage_matches_reference():
    x = (RNG.standard_normal(4096) + 1j * RNG.standard_normal(4096))
    for q in (8, 5):
        got = FirDecimStage(q).process(x)
        np.testing.assert_allclose(got, reference_decimate(x, q), atol=1e-12)


def test_impulse_response_phase():
    # impulse at n=0: first stage output (index 0, taken at input q-1) must
    # equal b[q-1] — verifies the "first output after q inputs" phase
    for q in (8, 5):
        x = np.zeros(64, dtype=np.complex128)
        x[0] = 1.0
        out = FirDecimStage(q).process(x)
        b = design_lowpass(q * 16, 0.45 / q)
        np.testing.assert_allclose(out[0], b[q - 1], atol=1e-15)


def test_chunked_equals_one_shot():
    # full 8x5x5 chain; chunk sizes chosen to be ragged vs every stage ratio
    n = 200 * 64 + 137
    x = (RNG.standard_normal(n) + 1j * RNG.standard_normal(n))
    ref = run_chain(x, chunk=len(x))
    for chunk in (1, 7, 200, 4096, 12345):
        got = run_chain(x, chunk=chunk)
        assert got.shape == ref.shape, f"chunk={chunk}"
        np.testing.assert_allclose(got, ref, atol=1e-12, err_msg=f"chunk={chunk}")


def test_tone_passes_chain_at_unity_gain():
    # 100 Hz tone at 768 kHz is deep in-band for every stage; after 200x
    # decimation it must survive at ~unity amplitude and correct frequency
    fs, f0, n = 768000.0, 100.0, 768000
    t = np.arange(n) / fs
    x = np.exp(2j * np.pi * f0 * t)
    y = run_chain(x, chunk=4096)
    y = y[200:]  # drop filter transient
    spec = np.fft.fft(y * np.hanning(len(y)))
    peak = np.argmax(np.abs(spec))
    freqs = np.fft.fftfreq(len(y), d=200.0 / fs)
    assert abs(freqs[peak] - f0) < 1.0
    np.testing.assert_allclose(np.abs(y).mean(), 1.0, atol=0.01)


def _pulse_train_stft(r0, k, n_pri_windows, n_time, snr_amp, rng):
    """Detector-rate pulse train: tone gated on for exactly one STFT window
    at fold offsets r0 + i*n_pri_windows. Returns (power, Wf, b_true)."""
    fs = DETECTOR_FS
    n_w, n_ol = 58, 29
    n_ws = n_w - n_ol
    n_samp = (n_time - 1) * n_ws + n_w
    iq = (rng.standard_normal(n_samp) + 1j * rng.standard_normal(n_samp)) \
        * np.sqrt(0.5)
    W, Wf = build_weighting_matrix(n_w, fs)
    f0 = 500.0
    tone = np.exp(2j * np.pi * f0 * np.arange(n_w) / fs)
    # Ground-truth bin from a noise-free probe (external calibration lock)
    probe, _ = compute_stft_power(tone.astype(complex), n_w, n_ol, n_w, W=W)
    b_true = int(np.argmax(probe[:, 0]))
    for i in range(k):
        s = (r0 + i * n_pri_windows) * n_ws
        iq[s:s + n_w] += snr_amp * tone
    power, _ = compute_stft_power(iq, n_w, n_ol, n_w, W=W)
    return iq, power, Wf, b_true


def test_fixed_offset_amplitude_at_external_lock():
    # PRI chosen integer in windows so the ground-truth phase is exact:
    # tip = 66*29/3840 s -> N_exact = 66.0
    rng = np.random.default_rng(999)
    k, pri_w, r0, n_time = 5, 66, 10, 400
    iq, power, Wf, b_true = _pulse_train_stft(r0, k, pri_w, n_time, 3.0, rng)
    noise = estimate_noise(power)
    hyps = build_hypothesis_indices(float(pri_w), k, power.shape[1])
    _, pulse_idx = hyps[0]
    # supplied lock: no re-maximization anywhere in this path
    amp_true = amplitude_at_known_pulse(power, b_true, pulse_idx[r0], noise)
    amp_wrong = amplitude_at_known_pulse(power, b_true,
                                         pulse_idx[r0 + pri_w // 2], noise)
    assert amp_true > 0
    assert abs(amp_wrong) < 0.2 * amp_true  # wrong phase sees only noise

    # held-out segment: lock from the first portion, measure the rest of the
    # SAME continuous stream with the fold offset translated by the split
    n_ws = 29
    w_split = r0 + 2 * pri_w  # first two pulses used for the "lock"
    tail = iq[w_split * n_ws:]
    power2, n_time2 = compute_stft_power(tail, 58, 29, 58,
                                         W=build_weighting_matrix(58, DETECTOR_FS)[0])
    noise2 = estimate_noise(power2)
    k2 = k - 2
    _, pulse_idx2 = build_hypothesis_indices(float(pri_w), k2, n_time2)[0]
    r0_tail = r0 + 2 * pri_w - w_split  # = 0: first held-out pulse window
    amp_tail = amplitude_at_known_pulse(power2, b_true,
                                        pulse_idx2[r0_tail], noise2)
    amp_tail_wrong = amplitude_at_known_pulse(power2, b_true,
                                              pulse_idx2[r0_tail + pri_w // 2],
                                              noise2)
    assert amp_tail > 0
    assert abs(amp_tail_wrong) < 0.2 * amp_tail
    # per-pulse amplitude consistent across the split (same train, same gain)
    np.testing.assert_allclose(amp_tail / k2, amp_true / k, rtol=0.35)


def test_fixed_offset_amplitude_unbiased_on_noise():
    # pedestal subtraction must be mean-zero on pure noise (no clamping)
    rng = np.random.default_rng(1234)
    k, pri_w, n_time = 5, 66, 400
    n_samp = (n_time - 1) * 29 + 58
    iq = (rng.standard_normal(n_samp) + 1j * rng.standard_normal(n_samp)) \
        * np.sqrt(0.5)
    W, _ = build_weighting_matrix(58, DETECTOR_FS)
    power, _ = compute_stft_power(iq, 58, 29, 58, W=W)
    noise = estimate_noise(power)
    _, pulse_idx = build_hypothesis_indices(float(pri_w), k, power.shape[1])[0]
    amps = np.array([amplitude_at_known_pulse(power, b, pulse_idx[7], noise)
                     / noise[b] for b in range(power.shape[0])])
    assert np.any(amps < 0)  # unclamped
    assert abs(amps.mean()) < 0.5  # ~zero-mean in noise units (K=5)


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"PASS {name}")
