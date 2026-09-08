"""Tests for psd_spectrum: Welch PSD metrics and sidecar-driven loaders."""

import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from psd_spectrum import (  # noqa: E402
    DEFAULT_DC_EXCLUDE_HZ_RAW, MINI_PASSBAND_FRACTION, MINI_RAW_FS,
    WINDOW_SECONDS, analyze, compute_metrics, compute_psd,
    load_detector_dump, load_iq, load_raw_capture, maybe_decimate_mini,
    spectral_flatness_db,
)

def white_noise(n, seed, scale=0.01):
    # Per-call seed: a shared generator would make each test's samples depend on
    # which tests ran before it.
    rng = np.random.default_rng(seed)
    return ((rng.standard_normal(n) + 1j * rng.standard_normal(n))
            * (scale / np.sqrt(2))).astype(np.complex64)


def tone(n, fs, freq_hz, amp):
    t = np.arange(n) / fs
    return (amp * np.exp(2j * np.pi * freq_hz * t)).astype(np.complex64)


# ---------------------------------------------------------------------------
# PSD geometry
# ---------------------------------------------------------------------------

def test_psd_axis_is_two_sided_ascending_with_expected_bin_width():
    fs = 3840.0
    f, psd_db = compute_psd(white_noise(int(fs * 2), seed=1), fs)
    nperseg = int(fs * WINDOW_SECONDS)
    assert f.size == nperseg
    assert psd_db.shape == f.shape
    assert np.all(np.diff(f) > 0)
    assert f[0] == pytest.approx(-fs / 2, abs=1e-6)
    assert f[1] - f[0] == pytest.approx(fs / nperseg, rel=1e-9)


def test_psd_rejects_too_short_input():
    with pytest.raises(SystemExit):
        compute_psd(white_noise(10, seed=2), 3840.0)


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------

def test_white_noise_is_flat_and_not_spiky():
    fs = 3840.0
    f, psd_db = compute_psd(white_noise(int(fs * 10), seed=3), fs)
    m = compute_metrics(f, psd_db)
    assert m.flatness_db == pytest.approx(0.0, abs=0.5)
    assert m.spikiness_db < 3.0
    assert m.dc_spike_db is None


def test_tone_raises_spikiness_and_lowers_flatness():
    fs = 3840.0
    n = int(fs * 10)
    f, psd_db = compute_psd(white_noise(n, seed=4) + tone(n, fs, 500.0, 0.05), fs)
    m = compute_metrics(f, psd_db)
    assert m.spikiness_db > 20.0
    assert m.flatness_db < -1.0
    assert f[np.argmax(psd_db)] == pytest.approx(500.0, abs=f[1] - f[0])


def test_spur_near_nyquist_edge_is_still_spiky():
    # medfilt's default zero padding would drag the baseline toward 0 dB in the
    # outer bins and hide this spur; circular padding keeps it visible.
    fs = 3840.0
    n = int(fs * 10)
    df = fs / int(fs * WINDOW_SECONDS)
    edge_hz = -fs / 2 + 2 * df
    f, psd_db = compute_psd(white_noise(n, seed=12) + tone(n, fs, edge_hz, 0.05), fs)
    m = compute_metrics(f, psd_db)
    assert m.spikiness_db > 20.0
    assert f[np.argmax(psd_db)] == pytest.approx(edge_hz, abs=df)


def test_spur_near_dc_is_still_spiky():
    # psd.py filters Welch's native (0, +f, -f) order, so DC sits at an array
    # edge and its zero padding distorts the baseline there too. The DC-centred
    # spectrum here has no seam at DC.
    fs = 3840.0
    n = int(fs * 10)
    df = fs / int(fs * WINDOW_SECONDS)
    near_dc_hz = 3 * df
    f, psd_db = compute_psd(white_noise(n, seed=13) + tone(n, fs, near_dc_hz, 0.05), fs)
    m = compute_metrics(f, psd_db)
    assert m.spikiness_db > 20.0
    assert f[np.argmax(psd_db)] == pytest.approx(near_dc_hz, abs=df)


def test_dc_exclusion_removes_spike_from_metrics_and_reports_it():
    fs = 3840.0
    n = int(fs * 10)
    df = fs / int(fs * WINDOW_SECONDS)
    # One bin off centre: Welch's per-segment mean removal would erase an
    # exact-DC tone, whereas a real HF+ spike has width and survives.
    iq = white_noise(n, seed=5) + tone(n, fs, df, 0.05)
    f, psd_db = compute_psd(iq, fs)

    m_all = compute_metrics(f, psd_db, dc_exclude_hz=0.0)
    m_excl = compute_metrics(f, psd_db, dc_exclude_hz=3 * df)

    assert m_all.spikiness_db > 20.0
    assert m_all.dc_spike_db is None
    assert m_excl.spikiness_db < 3.0
    assert m_excl.dc_spike_db is not None and m_excl.dc_spike_db > 20.0
    assert m_excl.flatness_db > m_all.flatness_db


def test_flatness_reference_values():
    assert spectral_flatness_db(np.full(64, -80.0)) == pytest.approx(0.0, abs=1e-9)
    two_level = np.concatenate([np.full(32, -80.0), np.full(32, -70.0)])
    # geo = 10^-7.5 W/Hz (-75 dB), arith = (10^-8 + 10^-7)/2 = 5.5e-8 (-72.60 dB)
    assert spectral_flatness_db(two_level) == pytest.approx(-2.404, abs=0.001)


def test_mini_decimation_only_at_mini_rate():
    iq = white_noise(40_000, seed=6)
    y, fs, band = maybe_decimate_mini(iq, float(MINI_RAW_FS))
    assert fs == MINI_RAW_FS / 4
    assert y.size == iq.size // 4
    assert band == pytest.approx(MINI_PASSBAND_FRACTION * fs / 2)
    y2, fs2, band2 = maybe_decimate_mini(iq, 768_000.0)
    assert fs2 == 768_000.0 and y2 is iq and band2 is None


def test_mini_and_hf_share_bin_width():
    # Intentional deviation from psd.py, which sizes the Mini window at 3 MHz
    # before decimating (80 ms / 12.5 Hz). Here both SDRs get 20 ms / 50 Hz.
    mini_iq, mini_fs, _ = maybe_decimate_mini(white_noise(3_000_000, seed=10), float(MINI_RAW_FS))
    f_mini, _ = compute_psd(mini_iq, mini_fs)
    f_hf, _ = compute_psd(white_noise(768_000, seed=11), 768_000.0)
    assert f_mini[1] - f_mini[0] == pytest.approx(50.0)
    assert f_hf[1] - f_hf[0] == pytest.approx(50.0)
    assert f_mini.size == int(750_000 * WINDOW_SECONDS)


def test_mini_passband_mask_removes_decimator_rolloff():
    # White noise through the IIR decimator: only the passband is flat. Without
    # the mask the roll-off skirts drag flatness well below zero.
    mini_iq, mini_fs, band = maybe_decimate_mini(white_noise(3_000_000, seed=14), float(MINI_RAW_FS))
    f, psd_db = compute_psd(mini_iq, mini_fs)
    unmasked = compute_metrics(f, psd_db)
    masked = compute_metrics(f, psd_db, valid_band_hz=band)
    assert unmasked.flatness_db < -2.0
    assert masked.flatness_db == pytest.approx(0.0, abs=0.5)
    assert masked.noise_floor_db > unmasked.noise_floor_db
    assert masked.valid_band_hz == band


# ---------------------------------------------------------------------------
# Loaders
# ---------------------------------------------------------------------------

def write_raw_capture(tmp_path, name='airspy-hf.1', fs=768_000, n=20_000):
    dat = tmp_path / f'{name}.dat'
    white_noise(n, seed=7).tofile(dat)
    (tmp_path / f'{name}.json').write_text(json.dumps({
        'sdr': 'airspy_hf', 'requested_freq_mhz': 146.17,
        'tune_freq_mhz': 146.18, 'dc_offset_hz': 10000.0,
        'sample_rate_hz': fs, 'format': 'complex_float32',
    }))
    return str(dat)


def test_load_raw_capture_reads_sidecar(tmp_path):
    rec = load_raw_capture(write_raw_capture(tmp_path))
    assert rec.fs == 768_000.0
    assert rec.iq.dtype == np.complex64 and rec.iq.size == 20_000
    assert rec.center_freq_mhz == pytest.approx(146.18)
    assert rec.tag_freq_hz == pytest.approx(146_170_000.0)
    assert rec.is_raw_capture


def test_load_raw_capture_without_sidecar_exits(tmp_path):
    dat = tmp_path / 'airspy-hf.2.dat'
    white_noise(100, seed=8).tofile(dat)
    with pytest.raises(SystemExit):
        load_raw_capture(str(dat))


def write_detector_dump(tmp_path, with_center=True):
    prefix = tmp_path / 'tag3_cycle_0007'
    np.save(f'{prefix}_iq.npy', white_noise(3840 * 2, seed=9))
    meta = {'cycle': 7, 'fs': 3840.0, 'nfft': 116}
    if with_center:
        meta.update(center_freq_mhz=146.17, tag_freq_hz=146_160_000)
    Path(f'{prefix}_meta.json').write_text(json.dumps(meta))
    return f'{prefix}_iq.npy'


def test_load_detector_dump_reads_meta(tmp_path):
    rec = load_detector_dump(write_detector_dump(tmp_path))
    assert rec.fs == 3840.0
    assert rec.center_freq_mhz == pytest.approx(146.17)
    assert rec.tag_freq_hz == 146_160_000.0
    assert not rec.is_raw_capture


def test_load_detector_dump_tolerates_old_meta(tmp_path):
    rec = load_detector_dump(write_detector_dump(tmp_path, with_center=False))
    assert rec.center_freq_mhz is None and rec.tag_freq_hz is None


def test_load_iq_dispatches_on_suffix(tmp_path):
    assert load_iq(write_raw_capture(tmp_path)).is_raw_capture
    assert not load_iq(write_detector_dump(tmp_path)).is_raw_capture
    with pytest.raises(SystemExit):
        load_iq(str(tmp_path / 'something.wav'))


def test_analyze_end_to_end_on_raw_capture(tmp_path):
    rec = load_iq(write_raw_capture(tmp_path, n=768_000))
    f, psd_db, m, fs = analyze(rec, DEFAULT_DC_EXCLUDE_HZ_RAW)
    assert fs == 768_000.0
    assert f.size == psd_db.size == int(768_000 * WINDOW_SECONDS)
    assert m.dc_spike_db is not None
    assert m.flatness_db == pytest.approx(0.0, abs=0.5)
