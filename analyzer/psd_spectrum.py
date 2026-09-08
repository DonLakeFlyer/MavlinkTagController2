#!/usr/bin/env python3
"""Noise-floor PSD analysis for IQ produced by this pipeline.

Port of https://github.com/DonLakeFlyer/psdSpectrum psd.py. Computes a Welch
power spectral density and three scalar metrics (noise floor, spectral
flatness, spikiness), then plots the spectrum.

Inputs (sample rate and centre frequency come from the sidecar; no --sdr flag):
  airspy-hf.N.dat / airspy-mini.N.dat   raw capture + .json from COMMAND_ID_RAW_CAPTURE
  tagN_cycle_NNNN_iq.npy                detector --dump-spectrogram IQ + _meta.json

The AirSpy HF+ has a DC spike at baseband 0 Hz, which is why raw captures are
tuned 10 kHz above the tag. By default the spike is excluded from the flatness
and spikiness metrics and reported separately (--dc-exclude-hz 0 to disable).

Deviations from psd.py (see spectral_flatness_db, maybe_decimate_mini,
baseline_db):
  - flatness geometric mean is taken in the log domain (no absolute epsilon);
  - AirSpy Mini uses the same 20 ms window as HF+ after decimation, so both
    SDRs get 50 Hz bins. psd.py sizes the window at 3 MHz and only then
    decimates, giving Mini an 80 ms window / 12.5 Hz bins;
  - the spikiness baseline is fitted on the DC-centred spectrum with circular
    padding. psd.py median-filters Welch's native order (0, +f..., -f...) with
    zero padding, so its baseline is distorted in the ~50 bins either side of
    DC and of the +Nyquist/-Nyquist seam;
  - Mini metrics are restricted to the decimator's flat passband (|f| <= 0.8 x
    output Nyquist, i.e. +/-300 kHz). psd.py averages over the full +/-375 kHz,
    so the outer 75 kHz of filter roll-off depresses its noise floor and
    flatness.
HF+ noise floor matches psd.py exactly with --dc-exclude-hz 0; spikiness can
differ when the largest excursion is within 50 bins of DC or +/-Nyquist.

Examples:
  python psd_spectrum.py ~/Logs/Logs-RawCapture-*/airspy-hf.1.dat
  python psd_spectrum.py ~/Logs/Logs-Detectors-*/tag2_cycle_0012_iq.npy --png out.png
"""

import argparse
import json
import math
import os
import sys
from dataclasses import dataclass
from typing import Optional

import numpy as np
from scipy import signal
from scipy.signal import medfilt

WINDOW_SECONDS = 0.02          # Welch segment length, as psd.py
MEDFILT_KERNEL = 101           # spikiness baseline, as psd.py
MINI_RAW_FS = 3_000_000
MINI_DECIMATION = 4            # 3 MHz -> 750 kHz, as psd.py (not exactly 768 kHz)
# scipy.signal.decimate's default IIR anti-alias filter is flat to 0.8 x the
# output Nyquist; beyond that the spectrum is filter roll-off, not receiver noise.
MINI_PASSBAND_FRACTION = 0.8
# Raw captures: wide enough to cover the spike's Welch leakage at 20 ms windows.
DEFAULT_DC_EXCLUDE_HZ_RAW = 200.0


@dataclass
class IqRecord:
    iq: np.ndarray
    fs: float
    label: str
    center_freq_mhz: Optional[float] = None   # RF frequency at baseband 0
    tag_freq_hz: Optional[float] = None       # marker on the plot
    is_raw_capture: bool = False


@dataclass
class PsdMetrics:
    noise_floor_db: float
    flatness_db: float
    spikiness_db: float
    dc_spike_db: Optional[float]   # None when nothing was excluded
    delta_f_hz: float
    valid_band_hz: Optional[float] = None   # metrics computed over |f| <= this


# ---------------------------------------------------------------------------
# Loaders
# ---------------------------------------------------------------------------

def _read_sidecar(path):
    try:
        with open(path, 'r', encoding='utf-8') as f:
            return json.load(f)
    except FileNotFoundError:
        sys.exit(f'Error: sidecar {path} not found; cannot determine sample rate')
    except json.JSONDecodeError as exc:
        sys.exit(f'Error: sidecar {path} is not valid JSON: {exc}')


def load_raw_capture(path):
    """airspy-*.N.dat + airspy-*.N.json written by CommandHandler::_handleRawCapture."""
    meta = _read_sidecar(os.path.splitext(path)[0] + '.json')
    fmt = meta.get('format', 'complex_float32')
    if fmt != 'complex_float32':
        sys.exit(f'Error: unsupported capture format {fmt!r}')
    fs = meta.get('sample_rate_hz')
    if not fs:
        sys.exit('Error: sidecar has no sample_rate_hz')
    iq = np.fromfile(path, dtype=np.complex64)
    if iq.size == 0:
        sys.exit(f'Error: {path} contains no samples')
    center = meta.get('tune_freq_mhz')
    tag_mhz = meta.get('requested_freq_mhz')
    return IqRecord(
        iq=iq, fs=float(fs), label=os.path.basename(path),
        center_freq_mhz=float(center) if center is not None else None,
        tag_freq_hz=float(tag_mhz) * 1e6 if tag_mhz is not None else None,
        is_raw_capture=True)


def load_detector_dump(path):
    """tagN_cycle_NNNN_iq.npy + _meta.json written by pulse_detector.write_cycle_dump."""
    if not path.endswith('_iq.npy'):
        sys.exit(f'Error: expected a *_iq.npy detector dump, got {path}')
    meta = _read_sidecar(path[:-len('_iq.npy')] + '_meta.json')
    fs = meta.get('fs')
    if not fs:
        sys.exit('Error: _meta.json has no fs')
    iq = np.load(path)
    if iq.size == 0:
        sys.exit(f'Error: {path} contains no samples')
    # Older dumps predate these keys; 0 means "not set" on the detector CLI.
    center = meta.get('center_freq_mhz') or None
    tag_hz = meta.get('tag_freq_hz') or None
    return IqRecord(
        iq=iq.astype(np.complex64, copy=False), fs=float(fs),
        label=os.path.basename(path),
        center_freq_mhz=float(center) if center is not None else None,
        tag_freq_hz=float(tag_hz) if tag_hz is not None else None,
        is_raw_capture=False)


def load_iq(path):
    if path.endswith('.npy'):
        return load_detector_dump(path)
    if path.endswith('.dat'):
        return load_raw_capture(path)
    sys.exit(f'Error: unrecognised input {path} (expected .dat raw capture or _iq.npy dump)')


# ---------------------------------------------------------------------------
# PSD and metrics
# ---------------------------------------------------------------------------

def maybe_decimate_mini(iq, fs):
    """psd.py brings AirSpy Mini captures down by 4 before Welch.

    Returns (iq, fs, valid_band_hz). valid_band_hz is the half-width of the
    decimator's flat passband, or None when no decimation was applied. The
    Welch window is sized from the returned rate so Mini and HF+ share the
    same 20 ms / 50 Hz geometry; psd.py sized it from 3 MHz.
    """
    if int(round(fs)) != MINI_RAW_FS:
        return iq, fs, None
    fs_out = fs / MINI_DECIMATION
    return (signal.decimate(iq, MINI_DECIMATION, ftype='iir'), fs_out,
            MINI_PASSBAND_FRACTION * fs_out / 2.0)


def compute_psd(iq, fs):
    """Welch PSD in dB, two-sided, DC-centred. Returns (f_hz, psd_db)."""
    nperseg = math.floor(fs * WINDOW_SECONDS)
    if nperseg < 8:
        sys.exit(f'Error: sample rate {fs} Hz too low for a {WINDOW_SECONDS}s window')
    if iq.size < nperseg:
        sys.exit(f'Error: only {iq.size} samples; need at least {nperseg} for one window')
    window = signal.get_window('hann', nperseg, fftbins=False)
    noverlap = math.floor(0.5 * nperseg)
    f, pxx = signal.welch(iq, fs, window=window, noverlap=noverlap,
                          return_onesided=False, scaling='density')
    order = np.argsort(f)
    f = f[order]
    pxx = pxx[order]
    psd_db = 10.0 * np.log10(np.maximum(pxx, np.finfo(float).tiny))
    return f, psd_db


def spectral_flatness_db(psd_db):
    """10log10(geometric mean / arithmetic mean) of the linear PSD; 0 dB = flat.

    Geometric mean taken in the log domain. psd.py adds an absolute 1e-10 to
    the linear PSD first, which biases the result positive once the floor is
    anywhere near 1e-10 W/Hz (typical at 768 kHz).
    """
    geo_db = float(np.mean(psd_db))
    arith_db = float(10.0 * np.log10(np.mean(10.0 ** (psd_db / 10.0))))
    return geo_db - arith_db


def baseline_db(psd_db, kernel=MEDFILT_KERNEL):
    """Median-filter baseline on the DC-centred spectrum, circular padding.

    The two-sided spectrum's endpoints are adjacent in the DFT, so wrap
    padding is the physically correct edge treatment. psd.py filters Welch's
    native (0, +f..., -f...) order with zero padding, which distorts the
    baseline for ~kernel/2 bins around DC and around the Nyquist seam.
    """
    k = min(kernel, psd_db.size if psd_db.size % 2 else psd_db.size - 1)
    if k < 3:
        return psd_db.copy()
    half = k // 2
    padded = np.pad(psd_db, half, mode='wrap')
    return medfilt(padded, kernel_size=k)[half:-half]


def compute_metrics(f, psd_db, dc_exclude_hz=0.0, valid_band_hz=None):
    """Noise floor / flatness / spikiness over bins outside |f| <= dc_exclude_hz
    and, when valid_band_hz is given, inside |f| <= valid_band_hz.

    The baseline is fitted on the full spectrum (a median filter is robust to
    the spike) so the reported DC height is measured against the same
    reference as spikiness.
    """
    base = baseline_db(psd_db)
    deviation = psd_db - base
    excluded = np.abs(f) <= dc_exclude_hz if dc_exclude_hz > 0 else np.zeros(f.shape, bool)
    kept = ~excluded
    if valid_band_hz is not None:
        kept &= np.abs(f) <= valid_band_hz
    if not np.any(kept):
        sys.exit('Error: exclusions removed every bin')
    dc_spike = float(np.max(deviation[excluded])) if np.any(excluded) else None
    return PsdMetrics(
        noise_floor_db=float(np.mean(psd_db[kept])),
        flatness_db=spectral_flatness_db(psd_db[kept]),
        spikiness_db=float(np.max(deviation[kept])),
        dc_spike_db=dc_spike,
        delta_f_hz=float(f[1] - f[0]) if f.size > 1 else float('nan'),
        valid_band_hz=valid_band_hz)


def analyze(record, dc_exclude_hz):
    iq, fs, valid_band_hz = maybe_decimate_mini(record.iq, record.fs)
    f, psd_db = compute_psd(iq, fs)
    return f, psd_db, compute_metrics(f, psd_db, dc_exclude_hz, valid_band_hz), fs


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------

def plot_psd(f, psd_db, metrics, record, png=None, title=None):
    try:
        import matplotlib
        if png:
            matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.ticker import FuncFormatter
    except ImportError:
        sys.exit('Error: matplotlib not installed (see simulator/requirements.txt); '
                 'use --no-plot for metrics only')

    absolute = record.center_freq_mhz is not None
    x = f / 1e6 + (record.center_freq_mhz if absolute else 0.0)

    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(x, psd_db, '-', linewidth=1)
    if metrics.valid_band_hz is not None:
        # Decimator roll-off: shown but excluded from the metrics.
        offset = record.center_freq_mhz if absolute else 0.0
        vb = metrics.valid_band_hz / 1e6
        ax.axvspan(x[0], offset - vb, color='0.85', zorder=0)
        ax.axvspan(offset + vb, x[-1], color='0.85', zorder=0)
    if record.tag_freq_hz is not None and absolute:
        ax.axvline(record.tag_freq_hz / 1e6, color='r', linestyle='--',
                   linewidth=1, label=f'tag {record.tag_freq_hz / 1e6:.6f} MHz')
        ax.legend(loc='upper right')
    y_min = metrics.noise_floor_db - 2.0
    ax.set_ylim(y_min, y_min + 20.0)
    ax.set_xlabel('Frequency (MHz)' if absolute else 'Baseband offset (kHz)')
    ax.set_ylabel('PSD (dBFS/Hz)')
    ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f'{v:.3f}' if absolute else f'{v * 1e3:.2f}'))
    ax.grid(True)

    line2 = (f'Noise Floor: {metrics.noise_floor_db:.2f} dBFS/Hz | '
             f'Flatness: {metrics.flatness_db:.2f} dB | '
             f'Spikiness: {metrics.spikiness_db:.2f} dB')
    if metrics.dc_spike_db is not None:
        line2 += f' | DC spike: {metrics.dc_spike_db:.2f} dB'
    ax.set_title(f'{title or record.label}\n{line2}')
    fig.tight_layout()

    if png:
        fig.savefig(png, dpi=120)
        print(f'Saved {png}')
    else:
        plt.show()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(
        description='Welch PSD noise-floor analysis of pipeline IQ captures',
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument('file', help='airspy-*.N.dat raw capture or tagN_cycle_NNNN_iq.npy dump')
    ap.add_argument('--png', metavar='PATH', help='save plot to PATH instead of showing it')
    ap.add_argument('--title', help='plot title (default: file name)')
    ap.add_argument('--dc-exclude-hz', type=float, default=None,
                    help=f'exclude |f| <= this from metrics (default: '
                         f'{DEFAULT_DC_EXCLUDE_HZ_RAW:g} for raw captures, 0 for detector dumps)')
    ap.add_argument('--no-plot', action='store_true', help='print metrics only')
    args = ap.parse_args(argv)

    record = load_iq(args.file)
    dc_exclude = args.dc_exclude_hz
    if dc_exclude is None:
        dc_exclude = DEFAULT_DC_EXCLUDE_HZ_RAW if record.is_raw_capture else 0.0
    if dc_exclude < 0:
        sys.exit('Error: --dc-exclude-hz must be >= 0')

    f, psd_db, m, fs = analyze(record, dc_exclude)

    print(f'{record.label}: {record.iq.size} samples @ {record.fs:.0f} Hz'
          + (f' (decimated to {fs:.0f} Hz)' if fs != record.fs else ''))
    if record.center_freq_mhz is not None:
        print(f'Center frequency: {record.center_freq_mhz:.6f} MHz')
    print(f'Bin Bandwidth (Δf): {m.delta_f_hz:.2f} Hz')
    print(f'Average Noise Floor: {m.noise_floor_db:.2f} dBFS/Hz')
    print(f'Spectral Flatness: {m.flatness_db:.2f} dB (closer to 0 = flatter)')
    print(f'Spikiness: {m.spikiness_db:.2f} dB (max peak above median baseline)')
    if m.dc_spike_db is not None:
        print(f'DC spike: {m.dc_spike_db:.2f} dB above baseline '
              f'(|f| <= {dc_exclude:g} Hz excluded from metrics above)')
    if m.valid_band_hz is not None:
        print(f'Metrics restricted to |f| <= {m.valid_band_hz / 1e3:.0f} kHz '
              f'(decimator passband); outer band is filter roll-off')

    if not args.no_plot:
        plot_psd(f, psd_db, m, record, png=args.png, title=args.title)


if __name__ == '__main__':
    main()
