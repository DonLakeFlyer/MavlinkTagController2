"""Tests for signal_analyzer pulse-width measurement.

_half_max_width measures each pulse at half of its own peak (above noise),
bounded to the threshold-crossing interval [rise, fall). These tests pin
down the edge cases that were fixed during development: sub-sample
interpolation, plateau handling, array boundaries, adjacent-pulse
isolation, and weak pulses whose half-max sits below the threshold.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from signal_analyzer import _half_max_width, extract_envelope, measure_pulses  # noqa: E402

NOISE = 1.0
DT = 1.0  # one time unit per sample so widths read directly in samples


def _rect(n, start, width, amp, noise=NOISE):
    p = np.full(n, noise, dtype=np.float64)
    p[start:start + width] = amp
    return p


def _rise_fall(power, thresh):
    on = power > thresh
    d = np.diff(on.astype(np.int8))
    rise = int(np.where(d == 1)[0][0] + 1)
    fall = int(np.where(d == -1)[0][0] + 1)
    return rise, fall


# ---------------------------------------------------------------------------
# _half_max_width
# ---------------------------------------------------------------------------

def test_clean_rect_pulse_width_is_exact():
    p = _rect(100, 40, 20, amp=101.0)
    rise, fall = _rise_fall(p, 10.0)
    start, width = _half_max_width(p, rise, fall, NOISE, DT)
    # Half-max = 51. Crossing at 39.5 (between 1 and 101) and 59.5.
    assert start == pytest.approx(39.5)
    assert width == pytest.approx(20.0)


def test_sub_sample_interpolation_on_ramped_edges():
    # Edges ramp linearly over one sample: 1 -> 26 -> 101 ... 101 -> 26 -> 1.
    p = np.full(100, NOISE)
    p[40] = 26.0
    p[41:60] = 101.0
    p[60] = 26.0
    rise, fall = _rise_fall(p, 10.0)
    start, width = _half_max_width(p, rise, fall, NOISE, DT)
    # Half = 51. Left crossing between p[40]=26 and p[41]=101: 40 + 25/75.
    # Right crossing between p[59]=101 and p[60]=26: 59 + 50/75.
    left = 40 + 25 / 75
    right = 59 + 50 / 75
    assert start == pytest.approx(left)
    assert width == pytest.approx(right - left)


def test_plateau_edges_do_not_divide_by_zero():
    # Both edge sample pairs are equal, so the interpolation hits pb == pa.
    # rise=20 and fall=30 come from the 10x-noise threshold; p[19]==p[20]
    # and p[29]==p[30] force the plateau branch on both sides.
    p = np.full(60, NOISE)
    p[19:31] = 101.0
    start, width = _half_max_width(p, 20, 30, NOISE, DT)
    assert np.isfinite(start) and np.isfinite(width)
    assert width == pytest.approx(10.0)


def test_pulse_touching_array_start():
    p = np.full(50, NOISE)
    p[0:10] = 101.0
    rise, fall = 0, 10
    start, width = _half_max_width(p, rise, fall, NOISE, DT)
    assert start == pytest.approx(0.0)
    assert 9.0 <= width <= 10.0


def test_pulse_touching_array_end():
    p = np.full(50, NOISE)
    p[40:50] = 101.0
    rise, fall = 40, 50
    start, width = _half_max_width(p, rise, fall, NOISE, DT)
    assert start == pytest.approx(39.5)
    assert 9.5 <= width <= 10.5


def test_adjacent_pulses_do_not_bleed():
    # Two strong pulses separated by a single below-threshold sample.
    p = np.full(100, NOISE)
    p[40:50] = 101.0
    p[50] = 5.0  # below 10x noise threshold, but above noise
    p[51:61] = 101.0
    _, w1 = _half_max_width(p, 40, 50, NOISE, DT)
    _, w2 = _half_max_width(p, 51, 61, NOISE, DT)
    assert w1 == pytest.approx(10.0, abs=0.1)
    assert w2 == pytest.approx(10.0, abs=0.1)


def test_weak_pulse_is_bounded_by_threshold_interval():
    # Peak 12x noise -> half-max 6.5x, below the 10x threshold. Surround the
    # pulse with noise samples above half-max that an unbounded walk would
    # absorb.
    p = np.full(100, NOISE)
    p[30:40] = 8.0   # above half-max (6.5) but below threshold (10)
    p[40:50] = 12.0  # the pulse
    p[50:60] = 8.0
    rise, fall = 40, 50
    start, width = _half_max_width(p, rise, fall, NOISE, DT)
    assert rise - 1 <= start <= rise
    assert width <= (fall - rise) + 1.0


def test_width_is_never_negative_on_random_noise():
    rng = np.random.default_rng(0)
    for _ in range(200):
        p = rng.exponential(NOISE, size=200)
        i = rng.integers(20, 170)
        p[i:i + 8] += 50.0
        rise, fall = i, i + 8
        _, width = _half_max_width(p, rise, fall, float(np.median(p)), DT)
        assert width >= 0.0


# ---------------------------------------------------------------------------
# End-to-end via extract_envelope + measure_pulses
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("snr_db", [20, 40])
def test_measured_width_matches_true_width(snr_db):
    fs = 3840.0
    n = int(fs * 10)
    t = np.arange(n) / fs
    gate = np.zeros(n)
    for s in np.arange(0.5, 10, 2.0):
        gate[(t >= s) & (t < s + 0.019)] = 1
    rng = np.random.default_rng(1)
    amp = 0.3 * 10 ** (snr_db / 20)
    noise = (rng.standard_normal(n) + 1j * rng.standard_normal(n)) * 0.3
    iq = (gate * np.exp(2j * np.pi * 300 * t) * amp + noise).astype(np.complex64)

    m = measure_pulses(extract_envelope(iq, fs, 300), 1.0 / fs, 10.0)
    assert m["n_pulses"] == 5
    assert m["median_pulse_ms"] == pytest.approx(19.0, abs=0.6)
    assert m["mean_interval_s"] == pytest.approx(2.0, abs=0.002)
