"""IqStream: continuous, never-cleared IQ timeline with gap accounting."""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from iq_stream import IqStream  # noqa: E402

FS = 3840.0
NS_PER_SAMPLE = 1e9 / FS
RESET_S = 0.030          # 2 x tp at tp = 15 ms
T0 = 5_000_000_000


def _ramp(start, n):
    return (np.arange(start, start + n) + 0j).astype(np.complex64)


def _ts(sample_index):
    return T0 + int(round(sample_index * NS_PER_SAMPLE))


def _feed(stream, start, n):
    stream.append(_ramp(start, n), _ts(start))


def _make(retain=4000):
    return IqStream(fs=FS, gap_reset_threshold_s=RESET_S, retain_samples=retain)


def test_take_returns_contiguous_samples_and_first_packet_timestamp():
    s = _make()
    for i in range(0, 3000, 1023):
        _feed(s, i, 1023)
    seg, ts, fills = s.take(0, 1000)
    np.testing.assert_array_equal(seg.real, np.arange(1000))
    assert ts == _ts(0)
    assert fills == []


def test_consecutive_takes_never_skip_or_repeat_samples():
    s = _make(retain=100)
    cursor = 0
    src = 0
    for _ in range(60):
        _feed(s, src, 1023)
        src += 1023
        while s.head - cursor >= 1000:
            seg, ts, _ = s.take(cursor, 1000)
            assert seg[0].real == cursor
            assert seg[-1].real == cursor + 999
            assert abs(ts - _ts(cursor)) <= 1   # packet stamps are already rounded
            cursor += 1000
            s.retire(cursor)


def test_small_gap_is_zero_filled_and_reported_relative_to_segment():
    s = _make()
    _feed(s, 0, 1023)
    skipped = 40                       # ~10 ms < 30 ms threshold
    _feed(s, 1023 + skipped, 1023)
    assert s.head == 2046 + skipped
    seg, ts, fills = s.take(500, 1000)
    assert fills == [(1023 - 500, skipped)]
    assert np.all(seg[523:523 + skipped] == 0)
    assert seg[523 + skipped].real == 1023 + skipped
    assert s.zerofill_count == 1


def test_large_gap_sets_barrier_but_keeps_earlier_samples():
    s = _make()
    _feed(s, 0, 1023)
    skipped = 400                      # ~104 ms >= threshold
    _feed(s, 1023 + skipped, 1023)
    assert s.reset_count == 1
    assert s.barrier == 1023           # index of first sample after the hole
    assert s.head == 2046              # hole is not zero-filled
    seg, ts, _ = s.take(1023, 1023)
    assert seg[0].real == 1023 + skipped
    assert ts == _ts(1023 + skipped)   # time jumps with the data
    # earlier data still readable for history/lock purposes
    old, old_ts, _ = s.take(0, 1023)
    assert old[0].real == 0 and old_ts == _ts(0)


def test_negative_gap_sets_barrier():
    s = _make()
    _feed(s, 0, 1023)
    s.append(_ramp(1023, 1023), _ts(1023) - 50_000_000)
    assert s.reset_count == 1
    assert s.barrier == 1023


def test_take_rejects_span_across_barrier():
    s = _make()
    _feed(s, 0, 1023)
    _feed(s, 1023 + 400, 1023)          # barrier at 1023
    try:
        s.take(1000, 100)               # [1000, 1100) straddles 1023
    except ValueError:
        pass
    else:
        raise AssertionError('take() across barrier must raise')
    # ending exactly at the barrier, or starting on it, is fine
    s.take(23, 1000)
    s.take(1023, 1000)


def test_head_marks_where_an_arm_starts_and_take_excludes_earlier_samples():
    s = _make()
    _feed(s, 0, 1023)
    mark = s.head
    _feed(s, 1023, 1023)
    _feed(s, 2046, 1023)
    seg, ts, _ = s.take(mark, 1500)
    assert seg[0].real == 1023
    assert ts == _ts(1023)


def test_retire_drops_history_and_counts_it():
    s = _make(retain=500)
    for i in range(0, 4092, 1023):
        _feed(s, i, 1023)
    s.retire(3000)                     # target base 2500; lands on part boundary
    assert s.base == 2046
    assert s.retired_samples == 2046
    seg, _, _ = s.take(2046, 100)
    assert seg[0].real == 2046
    try:
        s.take(2000, 100)
    except IndexError:
        pass
    else:
        raise AssertionError('take() before base must fail loudly')


def test_retire_never_concatenates(monkeypatch):
    s = _make(retain=500)
    for i in range(0, 20 * 1023, 1023):
        _feed(s, i, 1023)
    n_parts_before = len(s._parts)

    def boom(*a, **k):
        raise AssertionError('retire() must not concatenate')
    monkeypatch.setattr(np, 'concatenate', boom)
    s.retire(s.head)
    assert 0 < len(s._parts) < n_parts_before
    assert s.base % 1023 == 0
    assert s.head - s.base >= 500


def test_take_rejects_non_positive_length():
    s = _make()
    for bad in (0, -5):
        try:
            s.take(0, bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f'take(0, {bad}) must raise ValueError')


def test_gap_events_are_observable_via_callback():
    events = []
    s = IqStream(fs=FS, gap_reset_threshold_s=RESET_S, retain_samples=4000,
                 on_gap=lambda kind, **kw: events.append((kind, kw)))
    _feed(s, 0, 1023)
    _feed(s, 1023 + 10, 1023)
    _feed(s, 2056 + 1000, 1023)
    kinds = [k for k, _ in events]
    assert kinds == ['zerofill', 'reset']
    assert events[0][1]['missing_samples'] == 10
