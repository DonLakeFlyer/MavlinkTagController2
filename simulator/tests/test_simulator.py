"""Unit tests for iq_simulator core functions.

Covers:
  - pulse_on() fixed-rate and rate-switch behavior
  - encode_header() wire-format correctness
  - snr_at_distance() path-loss model
"""

import math
import struct

import numpy as np
import pytest

from iq_simulator import (
    HEADER_FMT,
    TTWF_ZMQ_IQ_HEADER_SIZE,
    TTWF_ZMQ_IQ_MAGIC,
    TTWF_ZMQ_IQ_VERSION,
    TagSignal,
    encode_header,
    snr_at_distance,
)


# ---------------------------------------------------------------------------
# pulse_on — fixed rate
# ---------------------------------------------------------------------------

class TestPulseOnFixedRate:

    def test_basic_pulse_count(self):
        """10 seconds at tip=2.0 → 5 pulses (at t=0,2,4,6,8)."""
        tag = TagSignal(tip=2.0, tp=0.015)
        fs = 768000
        t = np.arange(10 * fs) / fs
        mask = tag.pulse_on(t)
        # Count distinct pulses (gaps > 1 sample between groups)
        on = np.where(mask)[0]
        pulse_starts = [on[0]]
        for i in range(1, len(on)):
            if on[i] - on[i - 1] > 1:
                pulse_starts.append(on[i])
        assert len(pulse_starts) == 5

    def test_pulse_width(self):
        """Pulse width matches tp within one sample."""
        tag = TagSignal(tip=2.0, tp=0.015)
        fs = 100000  # easier math
        t = np.arange(int(2.5 * fs)) / fs
        mask = tag.pulse_on(t)
        # Find first pulse edges
        on_indices = np.where(mask)[0]
        first_pulse = on_indices[on_indices < int(0.1 * fs)]
        duration_s = len(first_pulse) / fs
        assert abs(duration_s - 0.015) < 2 / fs

    def test_phase_offset(self):
        """First pulse should start at phase_offset."""
        tag = TagSignal(tip=2.0, tp=0.015, phase_offset=0.5)
        fs = 100000
        t = np.arange(int(3 * fs)) / fs
        mask = tag.pulse_on(t)
        first_on = np.where(mask)[0][0]
        first_on_time = first_on / fs
        assert abs(first_on_time - 0.5) < 2 / fs


# ---------------------------------------------------------------------------
# pulse_on — rate switch (one-shot)
# ---------------------------------------------------------------------------

def _find_pulse_starts(mask, fs, min_gap_s=0.05):
    """Return list of pulse start times from a boolean mask."""
    on = np.where(mask)[0]
    if len(on) == 0:
        return []
    starts = [on[0] / fs]
    for i in range(1, len(on)):
        if (on[i] - on[i - 1]) / fs > min_gap_s:
            starts.append(on[i] / fs)
    return starts


class TestPulseOnRateSwitch:

    def test_pre_switch_matches_fixed(self):
        """Before switch_time, switched tag must match fixed-rate tag."""
        tag_fixed = TagSignal(tip=2.0, tp=0.015)
        tag_sw = TagSignal(tip=2.0, tp=0.015, tip_secondary=1.5, switch_time=10.0)
        fs = 100000
        t = np.arange(int(10 * fs)) / fs  # exactly up to switch_time
        pre = t < 10.0
        np.testing.assert_array_equal(
            tag_fixed.pulse_on(t[pre]),
            tag_sw.pulse_on(t)[pre])

    def test_post_switch_uses_secondary_rate(self):
        """After switch, inter-pulse interval should be tip_secondary."""
        tip_a, tip_b = 2.0, 1.0
        tag = TagSignal(tip=tip_a, tp=0.01, tip_secondary=tip_b, switch_time=5.0)
        fs = 100000
        t = np.arange(int(20 * fs)) / fs
        starts = _find_pulse_starts(tag.pulse_on(t), fs, min_gap_s=0.5 * tip_b)

        post_starts = [p for p in starts if p >= 5.0]
        assert len(post_starts) >= 3, "Need at least 3 post-switch pulses"
        for i in range(1, len(post_starts)):
            delta = post_starts[i] - post_starts[i - 1]
            assert abs(delta - tip_b) < 0.01, \
                f"Post-switch IPI {delta:.4f} != {tip_b}"

    def test_anchor_continuity(self):
        """First post-switch pulse should be tip_secondary after last pre-switch pulse."""
        tip_a, tip_b = 2.0, 1.5
        switch_time = 7.0
        tag = TagSignal(tip=tip_a, tp=0.01, tip_secondary=tip_b,
                        switch_time=switch_time)
        fs = 100000
        t = np.arange(int(15 * fs)) / fs
        starts = _find_pulse_starts(tag.pulse_on(t), fs)

        pre_pulses = [p for p in starts if p < switch_time]
        post_pulses = [p for p in starts if p >= switch_time]
        assert len(pre_pulses) >= 1 and len(post_pulses) >= 1

        gap = post_pulses[0] - pre_pulses[-1]
        assert abs(gap - tip_b) < 0.02, \
            f"Anchor gap {gap:.4f} != tip_secondary {tip_b}"

    def test_anchor_on_boundary(self):
        """When switch_time lands exactly on a pulse boundary,
        the pulse at switch_time must still fire and the next pulse comes
        tip_secondary later."""
        tip_a, tip_b = 2.0, 1.5
        switch_time = 10.0  # exactly 5 * tip_a
        tag = TagSignal(tip=tip_a, tp=0.01, tip_secondary=tip_b,
                        switch_time=switch_time)
        fs = 100000
        t = np.arange(int(15 * fs)) / fs
        starts = _find_pulse_starts(tag.pulse_on(t), fs)

        boundary_pulses = [p for p in starts if abs(p - switch_time) < 0.001]
        assert len(boundary_pulses) == 1, \
            f"Expected pulse at switch_time={switch_time}"

        post_pulses = [p for p in starts if p > switch_time + 0.1]
        assert len(post_pulses) >= 1
        gap = post_pulses[0] - switch_time
        assert abs(gap - tip_b) < 0.02, \
            f"Gap after boundary pulse {gap:.4f} != tip_secondary {tip_b}"

    def test_no_switch_fields_unchanged(self):
        """tip_secondary=0 should behave identically to fixed rate."""
        tag_plain = TagSignal(tip=2.0, tp=0.015)
        tag_none = TagSignal(tip=2.0, tp=0.015, tip_secondary=0.0, switch_time=0.0)
        t = np.linspace(0, 10, 50000)
        np.testing.assert_array_equal(
            tag_plain.pulse_on(t),
            tag_none.pulse_on(t))

    def test_switch_time_zero_is_fixed_rate(self):
        """switch_time=0 with tip_secondary should behave as fixed rate."""
        tag_fixed = TagSignal(tip=2.0, tp=0.015)
        tag_zero = TagSignal(tip=2.0, tp=0.015, tip_secondary=1.0, switch_time=0.0)
        t = np.linspace(0, 10, 50000)
        np.testing.assert_array_equal(
            tag_fixed.pulse_on(t),
            tag_zero.pulse_on(t))


# ---------------------------------------------------------------------------
# encode_header
# ---------------------------------------------------------------------------

class TestEncodeHeader:

    def test_size(self):
        hdr = encode_header(0, 0, 768000, 4096, 32768)
        assert len(hdr) == TTWF_ZMQ_IQ_HEADER_SIZE

    def test_magic_and_version(self):
        hdr = encode_header(0, 0, 768000, 4096, 32768)
        magic, version, hdr_size = struct.unpack_from('<IHH', hdr, 0)
        assert magic == TTWF_ZMQ_IQ_MAGIC
        assert version == TTWF_ZMQ_IQ_VERSION
        assert hdr_size == TTWF_ZMQ_IQ_HEADER_SIZE

    def test_field_values(self):
        seq, ts, sr, sc, pb, fl = 42, 1234567890, 768000, 4096, 32768, 0
        hdr = encode_header(seq, ts, sr, sc, pb, fl)
        parsed = struct.unpack(HEADER_FMT, hdr)
        assert parsed[3] == seq         # sequence
        assert parsed[4] == ts          # timestamp_us
        assert parsed[5] == sr          # sample_rate
        assert parsed[6] == sc          # sample_count
        assert parsed[7] == pb          # payload_bytes
        assert parsed[8] == fl          # flags

    def test_round_trip(self):
        """Encode then decode should recover all fields."""
        hdr = encode_header(99, 5_000_000, 768000, 2048, 16384, 1)
        m, v, hs, seq, ts, sr, sc, pb, fl = struct.unpack(HEADER_FMT, hdr)
        assert (m, v, hs) == (TTWF_ZMQ_IQ_MAGIC, TTWF_ZMQ_IQ_VERSION, TTWF_ZMQ_IQ_HEADER_SIZE)
        assert (seq, ts, sr, sc, pb, fl) == (99, 5_000_000, 768000, 2048, 16384, 1)


# ---------------------------------------------------------------------------
# snr_at_distance
# ---------------------------------------------------------------------------

class TestSnrAtDistance:

    def test_same_distance_no_loss(self):
        assert snr_at_distance(20.0, 100.0, 100.0) == pytest.approx(20.0)

    def test_double_distance_6db_loss(self):
        """Doubling distance → 20*log10(2) ≈ 6.02 dB loss."""
        result = snr_at_distance(20.0, 200.0, 100.0)
        assert result == pytest.approx(20.0 - 20 * math.log10(2), abs=0.01)

    def test_half_distance_6db_gain(self):
        result = snr_at_distance(20.0, 50.0, 100.0)
        assert result == pytest.approx(20.0 + 20 * math.log10(2), abs=0.01)

    def test_ten_x_distance_20db_loss(self):
        result = snr_at_distance(30.0, 1000.0, 100.0)
        assert result == pytest.approx(10.0, abs=0.01)

    def test_zero_distance_returns_ref(self):
        assert snr_at_distance(20.0, 0.0, 100.0) == 20.0

    def test_negative_distance_returns_ref(self):
        assert snr_at_distance(20.0, -5.0, 100.0) == 20.0
