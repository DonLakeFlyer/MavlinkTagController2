#!/usr/bin/env python3
"""Post-flight log analyzer.

Reads structured .jsonl from the Python detector(s) and decimator stderr
logs to produce a Markdown analysis report.

Usage:
    python3 post_flight_analysis.py <log-dir>

The report is written to <log-dir>/analysis.md.
"""

import datetime
import glob
import json
import os
import re
import statistics
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional

# Allow importing shared log_schema from the repo
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'shared'))
from log_schema import (read_jsonl, entries_by_type,
                        STARTUP, DETECTION, NO_DETECTION, FOLDS, TIMING,
                        NOISE_ELEVATED, GAP_EVENT,
                        EVT_THRESHOLD, HYPOTHESIS, SESSION_END)

# Decimator rate warnings beyond this count are treated as a sustained mismatch
# rather than a startup transient (measured-rate check runs ~1/s).
RATE_WARNING_TRANSIENT_MAX = 10


# ---------------------------------------------------------------------------
# Spectrogram plotting
# ---------------------------------------------------------------------------

def generate_spectrogram_png(power_path: str, out_png: str,
                             heading: str = '', tag: str = '') -> bool:
    """Generate a spectrogram PNG from a power .npy file.

    Returns True on success, False if dependencies are missing or data
    cannot be loaded.
    """
    try:
        import numpy as np
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return False

    try:
        power = np.load(power_path)
    except Exception:
        return False

    if power.ndim != 2 or power.size == 0:
        return False

    # Load companion metadata if available
    meta_path = power_path.replace('_power.npy', '_meta.json')
    meta = {}
    fs = None
    nfft = None
    detections = []
    if os.path.exists(meta_path):
        try:
            with open(meta_path) as f:
                meta = json.load(f)
            fs = meta.get('fs')
            nfft = meta.get('nfft')
            detections = meta.get('detections', [])
        except Exception:
            meta = {}

    n_freq, n_time = power.shape

    fig, ax = plt.subplots(figsize=(10, 3))

    power_db = 10 * np.log10(np.maximum(power, 1e-20))
    vmin = np.percentile(power_db, 5)
    vmax = np.percentile(power_db, 99)

    if fs and nfft:
        freq_axis = np.fft.fftshift(np.fft.fftfreq(n_freq, d=1.0 / fs))
        # Time axis: use STFT geometry from metadata when available
        n_w = meta.get('n_w', nfft // 2)
        n_ol = meta.get('n_ol', n_w // 2)
        hop = n_w - n_ol
        t_axis = np.arange(n_time) * hop / fs
        # imshow extent is pixel edges, so pad the bin centers by half a bin
        df = fs / n_freq
        dt = hop / fs
        extent = [t_axis[0] - dt / 2, t_axis[-1] + dt / 2,
                  freq_axis[0] - df / 2, freq_axis[-1] + df / 2]
        ax.imshow(power_db, aspect='auto', origin='lower',
                  extent=extent, cmap='viridis', vmin=vmin, vmax=vmax)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Frequency (Hz)')
    else:
        ax.imshow(power_db, aspect='auto', origin='lower',
                  cmap='viridis', vmin=vmin, vmax=vmax)
        ax.set_xlabel('Time bin')
        ax.set_ylabel('Frequency bin')

    title = 'Spectrogram'
    if tag:
        title += f' — tag {tag}'
    if heading:
        title += f' — heading {heading}°'
    if detections:
        det_info = detections[0]
        snr = det_info.get('snr_db', 0)
        title += f'  (SNR {snr:.1f} dB)'
    ax.set_title(title, fontsize=10)

    cbar = fig.colorbar(ax.images[0], ax=ax, pad=0.01)
    cbar.set_label('dB')

    # Mark detected frequency with a horizontal line
    for d in detections:
        freq = d.get('freq_hz')
        if freq is not None and fs:
            ax.axhline(freq, color='red', linewidth=0.8, linestyle='--',
                       alpha=0.8)

    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    plt.close(fig)
    return True


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass
class DecimatorPerf:
    out_sps: float = 0.0
    cpu_duty_pct: float = 0.0
    queue_depth: int = 0
    queue_drops: int = 0
    dropped: int = 0
    malformed: int = 0
    out_of_order: int = 0
    zmq_complex_sps: float = 0.0


@dataclass
class DetectionCycle:
    cycle: int = 0
    timestamp_ns: Optional[int] = None
    timestamp_str: str = ''
    detected: bool = False
    freq_hz: float = 0.0
    snr_db: float = 0.0
    score_ratio: float = 0.0
    noise_psd: float = 0.0
    proc_ms: float = 0.0
    had_gap: bool = False
    confidence: str = ''
    hyp_label: str = ''
    detection_status: int = 0
    # Interval between consecutive detected segments (not individual pulses)
    inter_detection_delta_ms: Optional[float] = None
    # Folds (populated from FOLDS entry)
    max_fold_fraction: float = 0.0
    fold_snrs: List[float] = field(default_factory=list)
    fold_windows: List[int] = field(default_factory=list)
    # Best candidate (no-detection cycles)
    best_candidate: Optional[dict] = None


@dataclass
class DetectorSummary:
    """All info for one detector, populated from .jsonl entries."""
    tag_id: Optional[int] = None
    port: int = 0
    center_freq_mhz: float = 0.0
    tp: float = 0.0
    tip: float = 0.0
    tip_secondary: Optional[float] = None
    fs: float = 0.0
    K: int = 0
    n_hypotheses: int = 0
    N: int = 0
    N_exact: float = 0.0
    N_B: Optional[int] = None
    nfft: int = 0
    freq_res: float = 0.0
    n_w: int = 0
    n_ol: int = 0
    segment_samples: int = 0
    segment_seconds: float = 0.0
    detection_margin: float = 0.0
    confidence_ratio: float = 0.0
    warmup_seconds: float = 0.0
    pf: float = 0.0
    dump_spectrogram: bool = False
    # Session stats (from SESSION_END)
    total_cycles: int = 0
    total_detections: int = 0
    elapsed_s: float = 0.0
    gap_zerofill: int = 0
    gap_reset: int = 0
    # Collected per-cycle data
    cycles: List[DetectionCycle] = field(default_factory=list)
    # Timing (from TIMING entries)
    timing_stft_ms: List[float] = field(default_factory=list)
    timing_fold_ms: List[float] = field(default_factory=list)
    timing_total_ms: List[float] = field(default_factory=list)
    # Events
    noise_elevated_events: int = 0
    gap_events: List[dict] = field(default_factory=list)
    hypothesis_summaries: List[dict] = field(default_factory=list)
    evt_info: List[dict] = field(default_factory=list)
    heading: Optional[str] = None  # e.g. '000', '045' for rotation headings


@dataclass
class BearingResult:
    tag_id: int = 0
    bearing_deg: float = 0.0
    r_squared: float = 0.0
    n_valid_slices: int = 0
    best_snr: float = 0.0
    latitude: float = 0.0
    longitude: float = 0.0


# ---------------------------------------------------------------------------
# Parsers
# ---------------------------------------------------------------------------

def parse_decimator_log(path: str) -> dict:
    """Parse airspyhf_decimator.log (light regex, stable key=value format)."""
    perfs: List[DecimatorPerf] = []
    rate_warnings: List[str] = []
    drop_events: List[str] = []
    queue_drop_events: List[str] = []
    input_rate = 0.0
    output_rate = 0.0
    # Decimator prints only the first 10 warnings then every 100th, with a
    # cumulative warnings= counter; the max counter is the true count.
    rate_warning_count = 0
    # Drop/queue-drop event lines carry cumulative totals too; they can
    # postdate the last perf line or exist in a run with no perf line at all.
    event_dropped = 0
    event_queue_drops = 0

    perf_pat = re.compile(
        r'perf\s+.*?out_sps=([\d.e+-]+).*?cpu_duty_pct=([\d.e+-]+).*?'
        r'queue_depth=(\d+).*?queue_drops=(\d+)')
    perf2_pat = re.compile(
        r'zmq_complex_sps=([\d.e+-]+).*?malformed=(\d+).*?'
        r'dropped=(\d+).*?out_of_order=(\d+)')
    rate_warn_pat = re.compile(
        r'bad incoming.*(?:sample.rate|sample_rate).*?=([\d.e+-]+).*?'
        r'error_ppm=([\d.e+-]+)(?:.*?warnings=(\d+))?')
    lock_pat = re.compile(
        r'locked input rate=([\d.e+-]+)\s+outputRate=([\d.e+-]+)')
    drop_pat = re.compile(
        r'dropped\s+(\d+)\s+packet.*?total_dropped=(\d+)')
    queue_drop_pat = re.compile(
        r'queue full.*?total_queue_drops=(\d+)')

    try:
        with open(path) as f:
            for line in f:
                lm = lock_pat.search(line)
                if lm:
                    input_rate = float(lm.group(1))
                    output_rate = float(lm.group(2))

                pm = perf_pat.search(line)
                p2m = perf2_pat.search(line)
                if pm:
                    p = DecimatorPerf(
                        out_sps=float(pm.group(1)),
                        cpu_duty_pct=float(pm.group(2)),
                        queue_depth=int(pm.group(3)),
                        queue_drops=int(pm.group(4)),
                    )
                    if p2m:
                        p.zmq_complex_sps = float(p2m.group(1))
                        p.malformed = int(p2m.group(2))
                        p.dropped = int(p2m.group(3))
                        p.out_of_order = int(p2m.group(4))
                    perfs.append(p)

                rw = rate_warn_pat.search(line)
                if rw:
                    rate_warnings.append(
                        f'rate={rw.group(1)} error_ppm={rw.group(2)}')
                    if rw.group(3):
                        rate_warning_count = max(rate_warning_count,
                                                 int(rw.group(3)))
                    else:
                        rate_warning_count = max(rate_warning_count,
                                                 len(rate_warnings))

                dm = drop_pat.search(line)
                if dm:
                    drop_events.append(
                        f'dropped {dm.group(1)} packet(s), '
                        f'total={dm.group(2)}')
                    event_dropped = max(event_dropped, int(dm.group(2)))

                qm = queue_drop_pat.search(line)
                if qm:
                    queue_drop_events.append(
                        f'queue drop total={qm.group(1)}')
                    event_queue_drops = max(event_queue_drops,
                                            int(qm.group(1)))
    except FileNotFoundError:
        pass

    last = perfs[-1] if perfs else DecimatorPerf()
    return {
        'perfs': perfs,
        'rate_warnings': rate_warnings,
        'rate_warning_count': rate_warning_count,
        'dropped': max(last.dropped, event_dropped),
        'malformed': last.malformed,
        'queue_drops': max(last.queue_drops, event_queue_drops),
        'out_of_order': last.out_of_order,
        'drop_events': drop_events,
        'queue_drop_events': queue_drop_events,
        'input_rate': input_rate,
        'output_rate': output_rate,
    }


def _ts_str_from_ns(ts_ns):
    """Convert nanosecond timestamp to HH:MM:SS.mmm UTC string."""
    if ts_ns and ts_ns > 1e9:
        dt = datetime.datetime.fromtimestamp(
            ts_ns / 1e9, tz=datetime.timezone.utc)
        return dt.strftime('%H:%M:%S') + f'.{dt.microsecond // 1000:03d}'
    return ''


def parse_detector_jsonl(path: str) -> DetectorSummary:
    """Parse a detector .jsonl file into a DetectorSummary."""
    entries = read_jsonl(path)
    det = DetectorSummary()

    # --- STARTUP ---
    for e in entries_by_type(entries, STARTUP):
        det.tag_id = e.get('tag_id')
        det.port = e.get('port', 0)
        det.center_freq_mhz = e.get('center_freq', 0.0)
        det.tp = e.get('tp', 0.0)
        det.tip = e.get('tip', 0.0)
        det.tip_secondary = e.get('tip_secondary')
        det.fs = e.get('fs', 0.0)
        det.K = e.get('K', 0)
        det.n_hypotheses = e.get('n_hypotheses', 1)
        det.N = e.get('N', 0)
        det.N_exact = e.get('N_exact', 0.0)
        det.N_B = e.get('N_B')
        det.nfft = e.get('nfft', 0)
        det.freq_res = e.get('freq_res', 0.0)
        det.n_w = e.get('n_w', 0)
        det.n_ol = e.get('n_ol', 0)
        det.segment_samples = e.get('samples_needed', 0)
        det.segment_seconds = e.get('seg_sec', 0.0)
        det.detection_margin = e.get('detection_margin', 0.0)
        det.confidence_ratio = e.get('confidence_ratio', 0.0)
        det.warmup_seconds = e.get('warmup_seconds', 0.0)
        det.pf = e.get('pf', 0.0)
        det.dump_spectrogram = e.get('dump_spectrogram', False)

    # --- DETECTION cycles ---
    folds_by_cycle: Dict[int, dict] = {}
    for e in entries_by_type(entries, FOLDS):
        folds_by_cycle[e.get('cycle', -1)] = e

    for e in entries_by_type(entries, DETECTION):
        cycle_num = e.get('cycle', 0)
        folds_e = folds_by_cycle.get(cycle_num, {})
        c = DetectionCycle(
            cycle=cycle_num,
            timestamp_ns=e.get('timestamp_ns'),
            timestamp_str=_ts_str_from_ns(e.get('timestamp_ns')),
            detected=True,
            freq_hz=e.get('freq_hz', 0.0),
            snr_db=e.get('snr_db', 0.0),
            score_ratio=e.get('score_ratio', 0.0),
            noise_psd=e.get('noise_psd', 0.0),
            proc_ms=e.get('proc_ms', 0.0),
            had_gap=e.get('had_gap', False),
            confidence=e.get('confidence', ''),
            hyp_label=e.get('hyp_label', ''),
            detection_status=e.get('detection_status', 0),
            inter_detection_delta_ms=e.get('inter_detection_delta_ms'),
            max_fold_fraction=folds_e.get('max_fold_fraction', 0.0),
            fold_snrs=folds_e.get('fold_snrs', []),
            fold_windows=folds_e.get('fold_windows', []),
        )
        det.cycles.append(c)

    # --- NO_DETECTION cycles ---
    for e in entries_by_type(entries, NO_DETECTION):
        c = DetectionCycle(
            cycle=e.get('cycle', 0),
            timestamp_ns=e.get('timestamp_ns'),
            timestamp_str=_ts_str_from_ns(e.get('timestamp_ns')),
            detected=False,
            proc_ms=e.get('proc_ms', 0.0),
            had_gap=e.get('had_gap', False),
            best_candidate=e.get('best_candidate'),
        )
        det.cycles.append(c)

    det.cycles.sort(key=lambda c: c.cycle)

    # --- TIMING ---
    for e in entries_by_type(entries, TIMING):
        if 'stft_ms' in e:
            det.timing_stft_ms.append(e['stft_ms'])
        if 'fold_ms' in e:
            det.timing_fold_ms.append(e['fold_ms'])
        if 'total_ms' in e:
            det.timing_total_ms.append(e['total_ms'])

    # --- GAP_EVENT ---
    det.gap_events = entries_by_type(entries, GAP_EVENT)

    # --- NOISE_ELEVATED ---
    det.noise_elevated_events = len(entries_by_type(entries, NOISE_ELEVATED))

    # --- HYPOTHESIS ---
    det.hypothesis_summaries = entries_by_type(entries, HYPOTHESIS)

    # --- EVT_THRESHOLD ---
    det.evt_info = entries_by_type(entries, EVT_THRESHOLD)

    # --- SESSION_END ---
    for e in entries_by_type(entries, SESSION_END):
        det.total_cycles = e.get('cycles', 0)
        det.total_detections = e.get('detections', 0)
        det.elapsed_s = e.get('elapsed_s', 0.0)
        det.gap_zerofill = e.get('gap_zerofill_count', 0)
        det.gap_reset = e.get('gap_reset_count', 0)

    return det


def parse_bearing_log(path: str) -> List[BearingResult]:
    """Parse bearing_result.log CSV."""
    results = []
    try:
        with open(path) as f:
            header = True
            for line in f:
                if header:
                    header = False
                    continue
                parts = line.strip().split(',')
                if len(parts) >= 5:
                    br = BearingResult(
                        tag_id=int(parts[0]),
                        bearing_deg=float(parts[1]),
                        r_squared=float(parts[2]),
                        n_valid_slices=int(parts[3]),
                        best_snr=float(parts[4]),
                    )
                    if len(parts) >= 7:
                        br.latitude = float(parts[5])
                        br.longitude = float(parts[6])
                    results.append(br)
    except FileNotFoundError:
        pass
    return results


# ---------------------------------------------------------------------------
# Report generation
# ---------------------------------------------------------------------------

def _stat_line(values: List[float], unit: str = '', fmt: str = '.1f') -> str:
    if not values:
        return 'N/A'
    mn = format(min(values), fmt)
    mx = format(max(values), fmt)
    md = format(statistics.median(values), fmt)
    if len(values) >= 2:
        sd = format(statistics.stdev(values), fmt)
        return (f'min={mn}  median={md}  max={mx}  std={sd}'
                f'{" " + unit if unit else ""}')
    return f'min={mn}  median={md}  max={mx}{" " + unit if unit else ""}'


def generate_report(log_dir: str) -> str:
    """Generate the full Markdown analysis report."""
    lines: List[str] = []

    def w(s: str = ''):
        lines.append(s)

    # --- Discover files ---
    # For rotation sessions, files live in heading-NNN/ subdirectories.
    # For plain detection sessions, they're at the top level.
    bearing_path = os.path.join(log_dir, 'bearing_result.log')

    heading_dirs = sorted(glob.glob(os.path.join(log_dir, 'heading-*')))
    heading_dirs = [d for d in heading_dirs if os.path.isdir(d)]

    if heading_dirs:
        # Rotation: aggregate across all heading subdirectories
        dec_paths = [os.path.join(d, 'airspyhf_decimator.log')
                     for d in heading_dirs]
        jsonl_files = []
        for d in heading_dirs:
            jsonl_files.extend(sorted(
                glob.glob(os.path.join(d, 'detector*.jsonl'))))
        dump_files = []
        for d in heading_dirs:
            dump_files.extend(sorted(
                glob.glob(os.path.join(d, 'tag*_cycle_*_power.npy'))))
    else:
        # Plain detection: single directory
        dec_paths = [os.path.join(log_dir, 'airspyhf_decimator.log')]
        jsonl_files = sorted(glob.glob(
            os.path.join(log_dir, 'detector*.jsonl')))
        dump_files = sorted(glob.glob(
            os.path.join(log_dir, 'tag*_cycle_*_power.npy')))

    # --- Parse ---
    # Merge decimator perf from all heading directories. Each heading runs a
    # fresh decimator, so the cumulative counters restart per log: sum each
    # log's final counters rather than reading the last log only.
    dec: dict = {'perfs': [], 'rate_warnings': [], 'drop_events': [],
                 'queue_drop_events': [], 'input_rate': 0.0,
                 'output_rate': 0.0,
                 'dropped': 0, 'malformed': 0, 'queue_drops': 0,
                 'out_of_order': 0, 'rate_warning_count': 0,
                 'rate_warning_max': 0}
    for dp in dec_paths:
        d = parse_decimator_log(dp)
        dec['perfs'].extend(d['perfs'])
        dec['rate_warnings'].extend(d['rate_warnings'])
        # Sum for display; classify on the per-log max since each heading's
        # decimator restarts its counter (N startup transients != sustained).
        dec['rate_warning_count'] += d['rate_warning_count']
        dec['rate_warning_max'] = max(dec['rate_warning_max'],
                                      d['rate_warning_count'])
        dec['drop_events'].extend(d['drop_events'])
        dec['queue_drop_events'].extend(d['queue_drop_events'])
        dec['dropped'] += d['dropped']
        dec['malformed'] += d['malformed']
        dec['queue_drops'] += d['queue_drops']
        dec['out_of_order'] += d['out_of_order']
        if d['input_rate'] > 0:
            dec['input_rate'] = d['input_rate']
        if d['output_rate'] > 0:
            dec['output_rate'] = d['output_rate']

    detectors = []
    for jp in jsonl_files:
        det = parse_detector_jsonl(jp)
        # Tag with heading if inside a heading subdirectory
        parent = os.path.basename(os.path.dirname(jp))
        m = re.match(r'heading-(\d+)', parent)
        if m:
            det.heading = m.group(1)
        detectors.append(det)
    bearings = parse_bearing_log(bearing_path)

    is_rotation = (os.path.exists(bearing_path)
                   or 'Rotation' in os.path.basename(log_dir))

    # Extract timestamp from directory name
    dir_name = os.path.basename(log_dir)
    dir_ts_match = re.search(
        r'(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})', dir_name)
    dir_ts = (dir_ts_match.group(1).replace('_', ' ')
              if dir_ts_match else 'unknown')

    # Derive center frequency from first detector startup
    center_freq_mhz = 0.0
    for det in detectors:
        if det.center_freq_mhz:
            center_freq_mhz = det.center_freq_mhz
            break

    # ========== Session Summary ==========
    w(f'# Log Analysis — {dir_ts}')
    w()
    w('## Session Summary')
    w()
    w('| Parameter | Value |')
    w('|---|---|')

    mode_str = 'rotation/bearing' if is_rotation else 'detection'
    w(f'| Mode | {mode_str} |')
    if center_freq_mhz:
        w(f'| Center frequency | {center_freq_mhz:.3f} MHz |')

    tag_ids = sorted({det.tag_id for det in detectors
                       if det.tag_id is not None})
    if tag_ids:
        tag_strs = []
        for tid in tag_ids:
            # Get tip from first detector with this tag_id
            for det in detectors:
                if det.tag_id == tid:
                    tip_str = f'tip={det.tip:.1f} s'
                    if det.tip_secondary is not None:
                        tip_str += f'/{det.tip_secondary:.1f} s'
                    tag_strs.append(f'ID {tid}: {tip_str}')
                    break
        w(f'| Tags | {len(tag_ids)} ({", ".join(tag_strs)}) |')

    # K is constant across all detectors — show once in summary
    k_vals = {det.K for det in detectors if det.K > 0}
    if k_vals:
        w(f'| K (folds) | {", ".join(str(k) for k in sorted(k_vals))} |')

    if heading_dirs:
        w(f'| Headings | {len(heading_dirs)} |')

    dump_on = any(det.dump_spectrogram for det in detectors)
    w(f'| Spectrogram dump | {"ON" if dump_on else "OFF"} |')
    if dump_files:
        # Each cycle has _power.npy, _iq.npy and _meta.json siblings.
        total_bytes = 0
        for pf in dump_files:
            prefix = pf[:-len('_power.npy')]
            for suffix in ('_power.npy', '_iq.npy', '_meta.json'):
                try:
                    total_bytes += os.path.getsize(prefix + suffix)
                except OSError:
                    pass
        w(f'| Dump files | {len(dump_files)} cycles, '
          f'{total_bytes / 1048576:.1f} MB |')

    total_cycles = sum(
        det.total_cycles or len(det.cycles) for det in detectors)
    total_dets = sum(
        det.total_detections or sum(1 for c in det.cycles if c.detected)
        for det in detectors)
    w(f'| Detection cycles | {total_cycles} |')
    w(f'| Detections | {total_dets} |')
    if total_cycles > 0:
        w(f'| Detection rate | {total_dets}/{total_cycles} '
          f'({100 * total_dets / total_cycles:.0f}%) |')
    # Detectors for different tags run concurrently, so take the max per
    # heading and sum headings (plain sessions are a single None heading).
    elapsed_by_heading: Dict[Optional[str], float] = {}
    for det in detectors:
        elapsed_by_heading[det.heading] = max(
            elapsed_by_heading.get(det.heading, 0.0), det.elapsed_s)
    total_elapsed = sum(elapsed_by_heading.values())
    if total_elapsed > 0:
        w(f'| Session duration | {total_elapsed:.0f} s |')
    w()

    # ========== Pipeline Health (Decimator) ==========
    if dec['perfs']:
        w('## Pipeline Health — Decimator')
        w()
        perfs = dec['perfs']
        total_drops = dec['dropped']
        total_malformed = dec['malformed']
        total_queue_drops = dec['queue_drops']
        total_ooo = dec['out_of_order']
        max_queue = max(p.queue_depth for p in perfs)
        # Measured-rate check runs once per second; more than this many
        # warnings in a single decimator run means the mismatch outlasted startup.
        rate_sustained = dec['rate_warning_max'] > RATE_WARNING_TRANSIENT_MAX

        if (total_drops == 0 and total_malformed == 0
                and total_queue_drops == 0 and total_ooo == 0
                and not rate_sustained):
            w('- **Status:** Healthy — no drops, no malformed packets')
        else:
            issues = []
            if total_drops:
                issues.append(f'{total_drops} ZMQ drops')
            if total_malformed:
                issues.append(f'{total_malformed} malformed')
            if total_queue_drops:
                issues.append(f'{total_queue_drops} queue drops')
            if total_ooo:
                issues.append(f'{total_ooo} out-of-order')
            if rate_sustained:
                issues.append(f'sustained sample-rate mismatch '
                              f'({dec["rate_warning_count"]} warnings)')
            w(f'- **Status:** WARNING — {", ".join(issues)}')

        out_rates = [p.out_sps for p in perfs if p.out_sps > 0]
        cpu_duties = [p.cpu_duty_pct for p in perfs]
        if out_rates:
            w(f'- **Output rate:** {min(out_rates):.0f}–'
              f'{max(out_rates):.0f} sps '
              f'(nominal {dec["output_rate"]:.0f})')
        if cpu_duties:
            w(f'- **CPU duty cycle:** {min(cpu_duties):.1f}–'
              f'{max(cpu_duties):.1f}%')
        w(f'- **Max queue depth:** {max_queue}')

        if dec['drop_events']:
            w(f'- **ZMQ drop events:** {len(dec["drop_events"])}')
            for ev in dec['drop_events'][:5]:
                w(f'  - {ev}')
            if len(dec['drop_events']) > 5:
                w(f'  - ... and {len(dec["drop_events"]) - 5} more')

        if dec['queue_drop_events']:
            w(f'- **Queue overflow events:** '
              f'{len(dec["queue_drop_events"])}')

        if dec['rate_warning_count']:
            label = ('sustained' if rate_sustained else 'startup transient')
            w(f'- **Rate warnings:** {dec["rate_warning_count"]} ({label})')
        w()

    # ========== Rotation Overview ==========
    if is_rotation and heading_dirs:
        w('## Rotation Overview')
        w()
        w('| Heading | Tag | Cycles | Detections | '
          'SNR (dB) | Score Ratio | Gaps |')
        w('|---|---|---|---|---|---|---|')
        for det in detectors:
            if det.heading is None:
                continue
            n_cyc = det.total_cycles or len(det.cycles)
            n_det = det.total_detections or sum(
                1 for c in det.cycles if c.detected)
            det_cycles = [c for c in det.cycles if c.detected]
            if det_cycles:
                snr_str = f'{statistics.median(c.snr_db for c in det_cycles):.1f}'
                ratio_str = f'{statistics.median(c.score_ratio for c in det_cycles):.1f}'
            else:
                snr_str = '—'
                ratio_str = '—'
            total_gaps = det.gap_zerofill + det.gap_reset
            gap_str = str(total_gaps) if total_gaps else '0'
            w(f'| {det.heading}° | {det.tag_id} | {n_cyc} | {n_det} '
              f'| {snr_str} | {ratio_str} | {gap_str} |')
        w()

        # Bing Maps satellite embed from bearing result lat/lon
        for b in bearings:
            if b.latitude != 0.0 and b.longitude != 0.0:
                w(f'**Location:** {b.latitude:.6f}, {b.longitude:.6f}')
                w()
                w(f'<iframe width="600" height="400" frameborder="0" '
                  f'src="https://www.bing.com/maps/embed?h=400&w=600'
                  f'&cp={b.latitude}~{b.longitude}'
                  f'&lvl=18&typ=a&sty=a"></iframe>')
                w()
                break

    # ========== Spectrograms (both session types) ==========
    if dump_files:
        w('## Spectrograms')
        w()
        for pf in dump_files:
            hm = re.match(r'heading-(\d+)',
                          os.path.basename(os.path.dirname(pf)))
            heading_label = hm.group(1) if hm else ''
            fm = re.search(r'tag(\d+)_cycle_(\d+)_power',
                           os.path.basename(pf))
            tag_label = fm.group(1) if fm else ''
            cycle_label = fm.group(2) if fm else ''
            png_name = ('spectrogram'
                        + (f'_h{heading_label}' if heading_label else '')
                        + f'_tag{tag_label}_cycle_{cycle_label}.png')
            png_path = os.path.join(log_dir, png_name)
            if generate_spectrogram_png(pf, png_path, heading=heading_label,
                                        tag=tag_label):
                alt = f'tag {tag_label} cycle {cycle_label}'
                if heading_label:
                    alt = f'heading {heading_label}° ' + alt
                w(f'![{alt}]({png_name})')
                w()

    # ========== Per-Detector Analysis ==========
    for det in detectors:
        if not det.cycles and det.total_cycles == 0:
            continue

        tag_label = (f'Tag {det.tag_id}' if det.tag_id is not None
                     else f'Port {det.port}')
        if det.heading is not None:
            tag_label += f' @ heading {det.heading}°'
        w(f'## Detector: {tag_label}')
        w()

        # Config
        if det.center_freq_mhz:
            w(f'- **Frequency:** {det.center_freq_mhz:.3f} MHz')
        w(f'- **PRI:** {det.tip:.3f} s (N={det.N} windows, '
          f'exact {det.N_exact:.4f})')
        if det.tip_secondary:
            w(f'- **Secondary PRI:** {det.tip_secondary:.3f} s')
        w(f'- **K (folds):** {det.K}')
        if det.n_hypotheses > 1:
            w(f'- **Hypotheses:** {det.n_hypotheses}')
        w(f'- **STFT:** window={det.n_w}  overlap={det.n_ol}  '
          f'freq_bins={det.nfft}  resolution={det.freq_res:.1f} Hz')
        w(f'- **Segment:** {det.segment_samples} samples '
          f'({det.segment_seconds:.1f} s)')
        w(f'- **Detection margin:** {det.detection_margin:.2f}')
        w(f'- **Warmup:** {det.warmup_seconds:.1f} s')
        w()

        # Detection summary
        n_cycles = det.total_cycles or len(det.cycles)
        n_det = det.total_detections or sum(
            1 for c in det.cycles if c.detected)
        n_nodet = n_cycles - n_det

        w('### Detection Summary')
        w()
        w(f'- **Cycles:** {n_cycles}')
        w(f'- **Detections:** {n_det} '
          f'({100 * n_det / max(n_cycles, 1):.0f}%)')
        if n_nodet:
            w(f'- **No-detection cycles:** {n_nodet}')

        det_cycles = [c for c in det.cycles if c.detected]
        if det_cycles:
            snrs = [c.snr_db for c in det_cycles]
            ratios = [c.score_ratio for c in det_cycles]
            noises = [c.noise_psd for c in det_cycles]
            w(f'- **SNR:** {_stat_line(snrs, "dB")}')
            w(f'- **Score ratio:** {_stat_line(ratios, "", ".1f")}')
            w(f'- **Noise PSD:** min={min(noises):.3e}  '
              f'max={max(noises):.3e}')

            _MARGINAL_FLAGS = {'LOW', 'DOMINANT_FOLD'}
            marginal = sum(1 for c in det_cycles
                           if c.confidence in _MARGINAL_FLAGS)
            if marginal:
                confidence_counts: Dict[str, int] = {}
                for c in det_cycles:
                    if c.confidence in _MARGINAL_FLAGS:
                        confidence_counts[c.confidence] = \
                            confidence_counts.get(c.confidence, 0) + 1
                parts = [f'{v} {k}' for k, v in confidence_counts.items()]
                w(f'- **Marginal detections:** {marginal} '
                  f'({", ".join(parts)})')

        # Gap stats
        total_gaps = det.gap_zerofill + det.gap_reset
        if total_gaps:
            w(f'- **Gaps:** {det.gap_zerofill} zero-filled, '
              f'{det.gap_reset} resets')
        if det.elapsed_s > 0:
            w(f'- **Session:** {det.elapsed_s:.0f} s')
        w()

        # EVT threshold info
        if det.evt_info:
            w('### EVT Threshold')
            w()
            for e in det.evt_info:
                src = e.get('source', '?')
                if src == 'cache':
                    w(f'- Loaded from cache: \u03bc={e.get("mu", 0):.4e}, '
                      f'\u03c3={e.get("sigma", 0):.4e}, '
                      f'threshold={e.get("threshold", 0):.4e}')
                elif src == 'margin':
                    w(f'- Detection margin={e.get("detection_margin", 0):.2f}'
                      f' applied: effective '
                      f'threshold={e.get("effective_threshold", 0):.4e}')
            w()

        # Timing stats
        if det.timing_total_ms:
            w('### Processing Timing')
            w()
            w(f'- **STFT:** '
              f'{_stat_line(det.timing_stft_ms, "ms", ".0f")}')
            w(f'- **Fold:** '
              f'{_stat_line(det.timing_fold_ms, "ms", ".0f")}')
            w(f'- **Total:** '
              f'{_stat_line(det.timing_total_ms, "ms", ".0f")}')
            if det.segment_seconds > 0:
                max_time_s = max(det.timing_total_ms) / 1000.0
                margin = det.segment_seconds / max(max_time_s, 0.001)
                w(f'- **Real-time margin:** {margin:.0f}\u00d7 '
                  f'(segment={det.segment_seconds:.1f} s, '
                  f'worst cycle={max(det.timing_total_ms):.0f} ms)')
                if margin < 2:
                    w(f'- **WARNING:** Real-time margin is dangerously '
                      f'low ({margin:.1f}\u00d7)')
            w()

        # Cycle-by-cycle table
        if det.cycles:
            w('### Cycle Detail')
            w()
            w('| Cycle | Time | Result | SNR (dB) | Score Ratio | '
              'Noise PSD | Time (ms) | Flags |')
            w('|---|---|---|---|---|---|---|---|')
            for c in det.cycles:
                flags = []
                if c.had_gap:
                    flags.append('GAP')
                if c.detected:
                    if c.confidence:
                        flags.append(c.confidence)
                    if c.hyp_label:
                        flags.append(f'hyp={c.hyp_label}')
                    flag_str = ' '.join(flags)
                    w(f'| {c.cycle} | {c.timestamp_str} | DETECTED '
                      f'| {c.snr_db:.1f} | {c.score_ratio:.3f} '
                      f'| {c.noise_psd:.3e} | {c.proc_ms:.0f} '
                      f'| {flag_str} |')
                else:
                    cand = c.best_candidate
                    if cand and cand.get('score_ratio', 0) > 0:
                        flags.append(
                            f'best ratio='
                            f'{cand["score_ratio"]:.3f}')
                    flag_str = ' '.join(flags)
                    w(f'| {c.cycle} | {c.timestamp_str} | no det '
                      f'| \u2014 | \u2014 | \u2014 | {c.proc_ms:.0f} '
                      f'| {flag_str} |')
            w()

        # Fold quality
        if det_cycles and any(c.fold_snrs for c in det_cycles):
            w('### Fold Quality')
            w()
            all_spreads = []
            all_max_fracs = []
            for c in det_cycles:
                if c.fold_snrs:
                    spread = max(c.fold_snrs) - min(c.fold_snrs)
                    all_spreads.append(spread)
                    all_max_fracs.append(c.max_fold_fraction)
            if all_spreads:
                w(f'- **SNR spread (max-min within fold):** '
                  f'{_stat_line(all_spreads, "dB")}')
            if all_max_fracs:
                w(f'- **Max fold fraction:** '
                  f'{_stat_line(all_max_fracs, "", ".3f")}')
                bad_folds = sum(1 for f in all_max_fracs if f > 0.8)
                if bad_folds:
                    w(f'- **WARNING:** {bad_folds} cycle(s) with '
                      f'dominant fold (>0.8)')
            w()

        # Hypothesis analysis
        if det.hypothesis_summaries:
            w('### Hypothesis Analysis')
            w()
            for h in det.hypothesis_summaries:
                winner = h.get('winner', '?')
                ratio = h.get('winner_ratio', 0)
                n_above = h.get('above_thresh', 0)
                w(f'- Winner: **{winner}** ratio={ratio:.1f}  '
                  f'above_thresh={n_above}')
                hyps = h.get('hypotheses', [])
                if hyps:
                    for hh in hyps[:5]:
                        lbl = hh.get('label', '?')
                        r = hh.get('ratio', 0)
                        mff = hh.get('max_fold_frac', 0)
                        w(f'  - {lbl}: ratio={r:.1f}  '
                          f'max_fold_frac={mff:.3f}')
            w()

        # Elevated noise events
        if det.noise_elevated_events > 0:
            w('### Elevated Noise')
            w()
            w(f'{det.noise_elevated_events} event(s) with elevated '
              f'noise bins detected.')
            w()

        # Gap events
        if det.gap_events:
            w('### Gap Events')
            w()
            w(f'{len(det.gap_events)} gap event(s):')
            w()
            for ev in det.gap_events[:20]:
                kind = ev.get('kind', '?')
                gap_ms = ev.get('gap_ms', 0)
                w(f'- **{kind}:** {gap_ms:.1f} ms')
                if kind == 'zerofill':
                    w(f'  Missing samples: '
                      f'{ev.get("missing_samples", "?")}')
                elif kind == 'reset':
                    w(f'  Discarded: '
                      f'{ev.get("discarded_samples", "?")} samples')
            if len(det.gap_events) > 20:
                w(f'- ... and {len(det.gap_events) - 20} more')
            w()

    # ========== Bearing / Rotation ==========
    if bearings:
        w('## Bearing Results')
        w()
        w('| Tag ID | Bearing (deg) | R\u00b2 | Valid Slices | '
          'Best SNR (dB) |')
        w('|---|---|---|---|---|')
        for b in bearings:
            quality = ''
            if b.r_squared < 0.5:
                quality = ' \u26a0 low R\u00b2'
            w(f'| {b.tag_id} | {b.bearing_deg:.1f} | '
              f'{b.r_squared:.3f}{quality} | {b.n_valid_slices} '
              f'| {b.best_snr:.1f} |')
        w()

    # ========== Anomalies ==========
    anomalies: List[str] = []

    for det in detectors:
        for c in det.cycles:
            if (c.detected and det.confidence_ratio > 0
                    and c.score_ratio < det.confidence_ratio * 1.5):
                anomalies.append(
                    f'Cycle {c.cycle} (tag {det.tag_id}): '
                    f'score_ratio {c.score_ratio:.3f} is close to '
                    f'threshold ({det.confidence_ratio:.1f})')

    for det in detectors:
        if det.timing_total_ms and det.segment_seconds > 0:
            worst = max(det.timing_total_ms)
            margin = det.segment_seconds / (worst / 1000.0)
            if margin < 5:
                anomalies.append(
                    f'Tag {det.tag_id}: real-time margin only '
                    f'{margin:.1f}\u00d7 (worst {worst:.0f} ms vs '
                    f'{det.segment_seconds:.1f} s segment)')

    for det in detectors:
        if det.noise_elevated_events > 5:
            anomalies.append(
                f'Tag {det.tag_id}: '
                f'{det.noise_elevated_events} elevated noise events '
                f'\u2014 possible sustained RFI')

    for det in detectors:
        total = det.total_cycles or len(det.cycles)
        if total > 0 and det.gap_reset > 0:
            pct = 100 * det.gap_reset / total
            if pct > 10:
                anomalies.append(
                    f'Tag {det.tag_id}: {det.gap_reset} gap resets '
                    f'({pct:.0f}% of cycles) \u2014 possible USB or '
                    f'scheduling issue')

    for det in detectors:
        n_dom = sum(1 for c in det.cycles
                    if c.confidence == 'DOMINANT_FOLD')
        if n_dom > 0:
            anomalies.append(
                f'Tag {det.tag_id}: {n_dom} detection(s) with '
                f'DOMINANT_FOLD \u2014 likely transient interference')

    # Counters can come from event lines even when no perf line was logged.
    if dec_paths:
        if dec['dropped'] > 0:
            anomalies.append(
                f'Decimator: {dec["dropped"]} dropped ZMQ packets')
        if dec['queue_drops'] > 0:
            anomalies.append(
                f'Decimator: {dec["queue_drops"]} queue overflow drops')
        if dec['out_of_order'] > 0:
            anomalies.append(
                f'Decimator: {dec["out_of_order"]} out-of-order packets')
        if dec['rate_warning_max'] > RATE_WARNING_TRANSIENT_MAX:
            anomalies.append(
                f'Decimator: sustained sample-rate mismatch '
                f'({dec["rate_warning_count"]} warnings) — check SDR '
                f'clock / --input-rate')

    for b in bearings:
        if b.r_squared < 0.5:
            anomalies.append(
                f'Tag {b.tag_id}: bearing R\u00b2={b.r_squared:.3f} '
                f'\u2014 result may be unreliable')

    for det in detectors:
        switch_cycles = [c for c in det.cycles
                         if c.detected and c.hyp_label
                         and c.hyp_label != 'A']
        if switch_cycles:
            labels = {c.hyp_label for c in switch_cycles}
            anomalies.append(
                f'Tag {det.tag_id}: {len(switch_cycles)} cycle(s) '
                f'detected with non-primary hypothesis ({labels})')

    w('## Anomalies & Warnings')
    w()
    if anomalies:
        for a in anomalies:
            w(f'- **{a}**')
    else:
        w('No anomalies detected.')
    w()

    # ========== Observations ==========
    observations: List[str] = []

    if 0 < dec['rate_warning_max'] <= RATE_WARNING_TRANSIENT_MAX:
        observations.append(
            f'Decimator had {dec["rate_warning_count"]} rate '
            f'warning(s) during startup \u2014 transient, '
            f'self-corrects within seconds.')

    dump_on = any(det.dump_spectrogram for det in detectors)
    if not dump_on and not dump_files:
        observations.append(
            'Spectrogram dump was not enabled. To capture per-cycle '
            'data, set dump_spectrogram in the GCS start command.')

    for det in detectors:
        n = det.total_cycles or len(det.cycles)
        nd = det.total_detections or sum(
            1 for c in det.cycles if c.detected)
        if n >= 5 and nd / max(n, 1) < 0.1:
            observations.append(
                f'Tag {det.tag_id}: very low detection rate '
                f'({nd}/{n} = {100 * nd / max(n, 1):.0f}%). '
                f'Check tag parameters, antenna, and noise floor.')

    if observations:
        w('## Observations')
        w()
        for i, obs in enumerate(observations, 1):
            w(f'{i}. {obs}')
        w()

    return '\n'.join(lines)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    if len(sys.argv) != 2:
        print(f'Usage: {sys.argv[0]} <log-dir>', file=sys.stderr)
        sys.exit(1)

    log_dir = sys.argv[1]
    if not os.path.isdir(log_dir):
        print(f'Error: {log_dir} is not a directory', file=sys.stderr)
        sys.exit(1)

    report = generate_report(log_dir)

    out_path = os.path.join(log_dir, 'analysis.md')
    with open(out_path, 'w') as f:
        f.write(report)

    print(f'Analysis written to {out_path}')


if __name__ == '__main__':
    main()
