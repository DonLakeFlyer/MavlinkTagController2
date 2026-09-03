"""Tests for ipi_analyzer streaming pieces.

Covers the fixes made while bench-testing against a Lotek collar: STFT
carry-over across UDP packet boundaries, tracker global indexing across
chunks, refractory suppression, gap reset, and stream-anchored timestamps.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from ipi_analyzer import (  # noqa: E402
    PulseEdgeTracker, _report_ipi, classify_ipi, extract_power_at_freq,
    stft_stream_step,
)

FS = 3840.0
TP = 0.019
N_W = int(np.ceil(TP * FS))     # 73
N_OL = N_W // 2                 # 36
N_WS = N_W - N_OL               # 37
STEP = N_WS / FS                # 9.64 ms
TONE_HZ = 300.0


def _pulsed_iq(duration_s, pulse_starts_s, amp=10.0, noise=0.3, seed=0):
    n = int(FS * duration_s)
    t = np.arange(n) / FS
    gate = np.zeros(n)
    for s in pulse_starts_s:
        gate[(t >= s) & (t < s + TP)] = 1.0
    rng = np.random.default_rng(seed)
    nz = (rng.standard_normal(n) + 1j * rng.standard_normal(n)) * noise
    return (gate * amp * np.exp(2j * np.pi * TONE_HZ * t) + nz).astype(np.complex64)


def _stream_power(iq, chunk_sizes):
    """Feed iq through stft_stream_step in the given chunk sizes."""
    carry = np.zeros(0, dtype=np.complex64)
    out = []
    pos = 0
    i = 0
    while pos < len(iq):
        size = chunk_sizes[i % len(chunk_sizes)]
        p, carry = stft_stream_step(carry, iq[pos:pos + size], FS, TONE_HZ, N_W, N_OL)
        out.append(p)
        pos += size
        i += 1
    return np.concatenate(out)


def _run_tracker(power, chunk_sizes, min_pulse_windows=1, **kw):
    tr = PulseEdgeTracker(STEP, threshold_db=15.0,
                          min_pulse_windows=min_pulse_windows,
                          max_pulse_windows=10, **kw)
    ipis = []
    pos = 0
    i = 0
    while pos < len(power):
        size = chunk_sizes[i % len(chunk_sizes)]
        ipis.extend(tr.process(power[pos:pos + size]))
        pos += size
        i += 1
    return ipis


# ---------------------------------------------------------------------------
# STFT streaming / carry
# ---------------------------------------------------------------------------

def test_extract_power_reports_consumed_samples():
    iq = np.zeros(1000, dtype=np.complex64)
    p, consumed = extract_power_at_freq(iq, FS, TONE_HZ, N_W, N_OL)
    n_windows = (1000 - N_OL) // N_WS
    assert len(p) == n_windows
    assert consumed == n_windows * N_WS


def test_extract_power_too_short_consumes_nothing():
    p, consumed = extract_power_at_freq(np.zeros(10, dtype=np.complex64), FS, TONE_HZ, N_W, N_OL)
    assert len(p) == 0 and consumed == 0


@pytest.mark.parametrize("chunks", [[1023], [500, 777, 61], [37], [4096]])
def test_chunked_stft_equals_one_shot(chunks):
    iq = _pulsed_iq(6.0, [0.5, 2.5, 4.5])
    one_shot, _ = extract_power_at_freq(iq, FS, TONE_HZ, N_W, N_OL)
    streamed = _stream_power(iq, chunks)
    n = min(len(one_shot), len(streamed))
    assert n >= len(one_shot) - 1  # tail may be one window short
    np.testing.assert_allclose(streamed[:n], one_shot[:n], rtol=1e-4)


def test_pulse_straddling_packet_boundary_is_not_lost():
    # Put a pulse exactly across a 1023-sample packet edge and confirm the
    # streamed power at that pulse matches a one-shot STFT over the whole
    # buffer (i.e. the boundary windows were not dropped).
    edge_s = 1023 / FS
    iq = _pulsed_iq(3.0, [edge_s - TP / 2, edge_s - TP / 2 + 2.0], amp=30.0)
    one_shot, _ = extract_power_at_freq(iq, FS, TONE_HZ, N_W, N_OL)
    streamed = _stream_power(iq, [1023])
    w_edge = int(edge_s / STEP)
    win = slice(w_edge - 3, w_edge + 3)
    assert one_shot[win].max() > 100 * np.median(one_shot)
    np.testing.assert_allclose(streamed[win], one_shot[win], rtol=1e-4)


# ---------------------------------------------------------------------------
# PulseEdgeTracker
# ---------------------------------------------------------------------------

def _clean_power(pulse_windows, n_total, on=1000.0, off=1.0):
    p = np.full(n_total, off, dtype=np.float32)
    for w in pulse_windows:
        p[w:w + 2] = on
    return p


@pytest.mark.parametrize("chunks", [[10_000], [50], [7], [1]])
def test_tracker_intervals_independent_of_chunking(chunks):
    starts = [200, 407, 614, 821]  # 207 windows = 1.996 s
    p = _clean_power(starts, 1100)
    ipis = _run_tracker(p, chunks, refractory_windows=50)
    assert len(ipis) == 3
    for ipi_s, _ in ipis:
        assert ipi_s == pytest.approx(207 * STEP, abs=1e-9)


def test_tracker_rise_indices_are_global():
    starts = [200, 407, 614]
    p = _clean_power(starts, 800)
    ipis = _run_tracker(p, [50], refractory_windows=50)
    assert [idx for _, idx in ipis] == starts[1:]


def _pulse_with_tail_spike():
    # A pulse whose envelope dips below threshold then pops back up for a
    # single window, followed by the next real pulse 207 windows later.
    p = np.full(600, 1.0, dtype=np.float32)
    p[200:203] = 1000.0
    p[203] = 1.0           # dip
    p[204] = 1000.0        # 1-window tail re-crossing
    p[407:409] = 1000.0    # next real pulse
    return p


def test_refractory_suppresses_tail_recrossing():
    ipis = _run_tracker(_pulse_with_tail_spike(), [600], refractory_windows=50)
    assert len(ipis) == 1
    assert ipis[0][0] == pytest.approx(207 * STEP, abs=1e-9)


def test_refractory_suppression_does_not_disturb_tracker_state():
    # With min_pulse_windows=2 the 1-window tail would be classed as a glitch
    # and wipe last_rise_idx *if* the refractory path had started tracking it
    # as a new pulse. The next real pulse must still yield an IPI anchored on
    # the original rise at 200.
    ipis = _run_tracker(_pulse_with_tail_spike(), [600],
                        min_pulse_windows=2, refractory_windows=50)
    assert len(ipis) == 1
    assert ipis[0][1] == 407
    assert ipis[0][0] == pytest.approx((407 - 200) * STEP, abs=1e-9)


def test_reset_after_gap_discards_spanning_interval():
    tr = PulseEdgeTracker(STEP, threshold_db=15.0, refractory_windows=50)
    p1 = _clean_power([200], 300)
    p2 = _clean_power([100, 307], 400)
    list(tr.process(p1))
    tr.reset_after_gap()
    ipis = list(tr.process(p2))
    # Only the interval fully inside p2 survives.
    assert len(ipis) == 1
    assert ipis[0][0] == pytest.approx(207 * STEP, abs=1e-9)


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

def test_report_timestamps_advance_with_rise_index(capsys):
    import datetime
    t0_ns = 1_756_929_795 * 10**9
    counts = {'A': 0, 'B': 0, 'ANOM': 0}
    ipis, tl = [], []
    last = None
    rises_s = [2.0, 4.0, 5.5]
    for ipi, rise_s in zip([2.0, 2.0, 1.5], rises_s):
        last = _report_ipi(ipi, int(round(rise_s / STEP)), t0_ns, STEP,
                           2.0, 1.5, 0.15, counts, ipis, tl, last)
    lines = [ln for ln in capsys.readouterr().out.splitlines() if ln.strip()]
    times = [ln.split()[1] for ln in lines]
    expected = [
        datetime.datetime.fromtimestamp(
            t0_ns / 1e9 + int(round(r / STEP)) * STEP, tz=datetime.timezone.utc
        ).strftime('%H:%M:%S')
        for r in rises_s
    ]
    assert times == expected
    assert times[0] != times[1] != times[2]  # not all stamped with one time
    assert counts == {'A': 2, 'B': 1, 'ANOM': 0}
    assert tl == [(3, 'A', 'B', 1.5)]
    assert last == 'B'


def test_classify_ipi_tolerance():
    # 15 % bands: A covers [1.70, 2.30], B covers [1.275, 1.725]
    assert classify_ipi(2.0, 2.0, 1.5) == 'A'
    assert classify_ipi(1.5, 2.0, 1.5) == 'B'
    assert classify_ipi(1.2, 2.0, 1.5) == 'ANOM'
    assert classify_ipi(3.0, 2.0, 1.5) == 'ANOM'
