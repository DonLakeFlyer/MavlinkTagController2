#!/usr/bin/env python3
"""Replay a rotation's per-heading detector dumps at several lock score ratios.

Input is a rotation log directory recorded with spectrogram dumping on
(heading-NNN/tag<T>_cycle_NNNN_{power.npy,meta.json} and detector_<T>.jsonl).
For each tag the full-band fold search is re-run once per heading on the
dumped spectrogram; it does not depend on the lock ratio. Then, for each
--lock-ratios value, the detector's candidate bank and per-heading
measurements (pulse_detector.RotationCandidates / measure_candidate) and the
controller's bearing fit (a port of controller/BearingCalculator.cpp plus the
heard/confirmed rules of CommandHandler) are replayed, showing whether the
rotation would have locked, on which heading, and with what bearing.

Permutation-null thresholds are random. Each search gets its own fixed seed,
so a replay is reproducible and independent of the sweep, but it will not
match the flight's thresholds bit for bit. The full search uses the
permutation count each heading actually reached in flight (time-limited on
the vehicle) unless --null-permutations overrides it.

Usage:
  .venv/bin/python analyzer/lock_ratio_replay.py <rotation_dir> [<rotation_dir> ...] \\
      [--lock-ratios 1.5,2,2.5,3] [--null-permutations N] [--json out.json]
"""

import argparse
import glob
import json
import math
import os
import re
import sys
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "detector"))
import pulse_detector as pd  # noqa: E402

DEFAULT_LOCK_RATIOS = (1.5, 2.0, 2.5, 3.0)
# pulse_detector --lock-score-ratio default when startup records did not yet log it (before #200).
DEFAULT_FLOWN_LOCK_RATIO = 3.0

# controller/AntennaPattern.cpp, dB relative to boresight, 0..180 deg in 10 deg steps.
ANTENNA_PATTERNS = {
    0: ("RA-2A-tuned-147", (
        0.0, 0.0, 0.0, -1.0, -2.8, -8.5, -12.5, -16.5, -20.5, -24.2,
        -23.5, -22.0, -20.5, -19.0, -17.9, -15.0, -11.5, -8.0, -6.4), 0.2),
    1: ("RA-23K", (
        0.0, -0.5, -1.0, -2.0, -3.0, -5.0, -8.0, -10.5, -16.0, -18.0,
        -17.5, -16.0, -14.0, -13.5, -11.5, -10.5, -10.0, -9.5, -10.0), 0.2),
}

# controller/CommandHandler.h
CONFIRMED_SIGHTINGS = 2
HEARD_MIN_AGREEING_HITS = 2
HEARD_FREQUENCY_TOLERANCE_HZ = 200.0


# ---------------------------------------------------------------------------
# Bearing fit: port of controller/BearingCalculator.cpp
# ---------------------------------------------------------------------------

SCAN_STEP_DEG = 0.5
MIN_DETECTED_FOR_FLOOR = 3
CENSOR_FRACTION = 0.5
SPAN_FLOOR_FRACTION = 0.005
FULL_DOF = 5.0
CANDIDATE_CONFIDENCE_TOLERANCE = 0.05


@dataclass
class Observation:
    """One stored rotation report, as CommandHandler hands it to the fit."""
    heading_deg: float
    detected: bool
    candidate_id: int = 0
    signal_power: float = 0.0
    snr_db: float = 0.0
    sighted: bool = False
    noise_psd: float = 0.0
    freq_hz: float = float('nan')
    slice_index: int = -1


@dataclass
class BearingResult:
    candidate_id: int
    bearing_deg: float
    confidence: float
    n_valid_slices: int
    n_sighted_slices: int
    best_snr: float
    rejected: bool = False


def pattern_linear(pattern_db, offset_deg):
    angle = math.fmod(offset_deg, 360.0)
    if angle < 0.0:
        angle += 360.0
    if angle > 180.0:
        angle = 360.0 - angle
    index_f = angle / 10.0
    idx0 = int(index_f)
    idx1 = min(idx0 + 1, len(pattern_db) - 1)
    frac = index_f - idx0
    db = pattern_db[idx0] * (1.0 - frac) + pattern_db[idx1] * frac
    return 10.0 ** (db / 10.0)


def slice_weights(observations):
    known = sorted(o.noise_psd for o in observations
                   if math.isfinite(o.noise_psd) and o.noise_psd > 0.0)
    if not known:
        return [1.0] * len(observations)
    reference = known[len(known) // 2]
    weights = []
    for o in observations:
        if math.isfinite(o.noise_psd) and o.noise_psd > 0.0:
            weights.append((reference / o.noise_psd) ** 2)
        else:
            weights.append(1.0)
    return weights


def _fit_amplitude(g, p, w, fit_floor):
    sw = sum(w)
    swg = sum(wi * gi for wi, gi in zip(w, g))
    swgg = sum(wi * gi * gi for wi, gi in zip(w, g))
    swp = sum(wi * pi for wi, pi in zip(w, p))
    swgp = sum(wi * gi * pi for wi, gi, pi in zip(w, g, p))
    if fit_floor:
        det = sw * swgg - swg * swg
        if abs(det) > 1e-12 and sw > 0.0:
            a = (sw * swgp - swg * swp) / det
            b = (swp - a * swg) / sw
            if a >= 0.0 and b >= 0.0:
                return a, b
            if a >= 0.0:
                return (max(0.0, swgp / swgg) if swgg > 1e-12 else 0.0), 0.0
        return 0.0, (max(0.0, swp / sw) if sw > 0.0 else 0.0)
    return (max(0.0, swgp / swgg) if swgg > 1e-12 else 0.0), 0.0


def _solve_for_candidate(pattern_db, observations, candidate_id):
    weights = slice_weights(observations)
    det_h, det_p, det_w, cens_h, cens_w = [], [], [], [], []
    best_snr = -1e9
    best_power = -math.inf
    best_evidence = -math.inf
    min_positive = math.inf
    best_heading = 0.0
    n_sighted = 0
    for o, w in zip(observations, weights):
        if not o.detected:
            cens_h.append(o.heading_deg)
            cens_w.append(w)
            continue
        det_h.append(o.heading_deg)
        det_p.append(o.signal_power)
        det_w.append(w)
        best_snr = max(best_snr, o.snr_db)
        if o.sighted:
            n_sighted += 1
        best_power = max(best_power, o.signal_power)
        evidence = o.signal_power * math.sqrt(w)
        if evidence > best_evidence:
            best_evidence = evidence
            best_heading = o.heading_deg
        if o.signal_power > 0.0:
            min_positive = min(min_positive, o.signal_power)

    n_det, n_cens = len(det_h), len(cens_h)
    result = BearingResult(candidate_id=candidate_id, bearing_deg=float('nan'),
                           confidence=0.0, n_valid_slices=n_det,
                           n_sighted_slices=n_sighted,
                           best_snr=best_snr if n_det else 0.0)
    if n_det == 0 or best_power <= 0.0:
        if n_det:
            result.bearing_deg = best_heading
        return result

    censor_limit = CENSOR_FRACTION * min_positive
    fit_floor = n_det >= MIN_DETECTED_FOR_FLOOR
    sum_power_sq = sum(w * p * p for w, p in zip(det_w, det_p))

    n_steps = int(round(180.0 / SCAN_STEP_DEG)) + 1
    costs = []
    for k in range(n_steps):
        phi = best_heading - 90.0 + k * SCAN_STEP_DEG
        g = [pattern_linear(pattern_db, h - phi) for h in det_h]
        a, b = _fit_amplitude(g, det_p, det_w, fit_floor)
        cost = sum(w * (p - (a * gi + b)) ** 2 for w, p, gi in zip(det_w, det_p, g))
        for h, w in zip(cens_h, cens_w):
            excess = a * pattern_linear(pattern_db, h - phi) + b - censor_limit
            if excess > 0.0:
                cost += w * excess * excess
        costs.append(cost)
    best_idx = int(np.argmin(costs))
    min_cost = costs[best_idx]

    threshold = min_cost + max(min_cost, SPAN_FLOOR_FRACTION * sum_power_sq)
    lo = hi = best_idx
    while lo > 0 and costs[lo - 1] <= threshold:
        lo -= 1
    while hi < n_steps - 1 and costs[hi + 1] <= threshold:
        hi += 1
    span_deg = (hi - lo) * SCAN_STEP_DEG
    phi = math.fmod(best_heading - 90.0 + 0.5 * (lo + hi) * SCAN_STEP_DEG, 360.0)
    if phi < 0.0:
        phi += 360.0

    n_params = 3 if fit_floor else 2
    fit_factor = min(max(1.0 - min_cost / sum_power_sq, 0.0), 1.0)
    span_factor = min(max(1.0 - span_deg / 180.0, 0.0), 1.0)
    dof_factor = min(max((n_det + n_cens - n_params + 1) / FULL_DOF, 0.0), 1.0)
    result.bearing_deg = phi
    result.confidence = fit_factor * span_factor * dof_factor
    return result


def solve_candidates(pattern_db, observations):
    """One result per candidate. A candidate's no-detections are its own censored headings."""
    ids = sorted({o.candidate_id for o in observations if o.detected})
    if not ids:
        ids = [0]
    return [_solve_for_candidate(pattern_db,
                                 [o for o in observations if o.candidate_id == cid], cid)
            for cid in ids]


def _is_better(candidate, incumbent, floor):
    clears, incumbent_clears = candidate.confidence >= floor, incumbent.confidence >= floor
    if clears != incumbent_clears:
        return clears
    delta = candidate.confidence - incumbent.confidence
    if abs(delta) > CANDIDATE_CONFIDENCE_TOLERANCE:
        return delta > 0.0
    if candidate.n_sighted_slices != incumbent.n_sighted_slices:
        return candidate.n_sighted_slices > incumbent.n_sighted_slices
    if candidate.n_valid_slices != incumbent.n_valid_slices:
        return candidate.n_valid_slices > incumbent.n_valid_slices
    return candidate.best_snr > incumbent.best_snr


def select_result(results, floor):
    best = None
    for r in results:
        if best is None or _is_better(r, best, floor):
            best = r
    if best is not None and best.n_valid_slices > 0 and best.confidence < floor:
        best = BearingResult(**{**best.__dict__, 'rejected': True, 'bearing_deg': float('nan')})
    return best


def largest_frequency_cluster(freqs_hz, tolerance_hz):
    finite = [f for f in freqs_hz if math.isfinite(f)]
    return max((sum(1 for f in finite if abs(f - c) <= tolerance_hz) for c in finite), default=0)


@dataclass
class ControllerOutcome:
    """What CommandHandler would put in BEARING_RESULT for the tag."""
    selected: Optional[BearingResult]
    bearing_deg: float
    confidence: float
    heard: bool
    confirmed: bool
    # This tag qualifies for the confirmation revisit; the controller asks
    # for one per collection, for the lowest eligible tag id.
    revisit_eligible: bool
    reason: str
    reported_valid_slices: int = 0   # n_valid_slices on the wire: 0 when not heard


def controller_outcome(pattern_db, floor, observations):
    results = solve_candidates(pattern_db, observations)
    selected = select_result(results, floor)
    heard = selected.n_valid_slices > 0
    no_bearing = selected.rejected
    reason = 'below confidence floor' if selected.rejected else ''
    if selected.n_sighted_slices == 0 and selected.n_valid_slices > 0:
        agreeing = largest_frequency_cluster(
            [o.freq_hz for o in observations if o.detected], HEARD_FREQUENCY_TOLERANCE_HZ)
        heard = agreeing >= HEARD_MIN_AGREEING_HITS
        no_bearing = True
        reason = ('no lock; acquisition hits agree in frequency' if heard
                  else 'no lock; acquisition hits do not agree in frequency')
    if selected.n_valid_slices == 0:
        reason = 'nothing heard'
    bearing = float('nan') if no_bearing else selected.bearing_deg
    revisit_eligible = (not selected.rejected and math.isfinite(selected.bearing_deg)
                        and selected.n_sighted_slices == 1 and not no_bearing)
    return ControllerOutcome(
        selected=selected, bearing_deg=bearing,
        confidence=0.0 if no_bearing else selected.confidence,
        heard=heard,
        confirmed=not no_bearing and selected.n_sighted_slices >= CONFIRMED_SIGHTINGS,
        revisit_eligible=revisit_eligible, reason=reason,
        reported_valid_slices=selected.n_valid_slices if heard else 0)


# ---------------------------------------------------------------------------
# Detector setup and rotation loading
# ---------------------------------------------------------------------------

@dataclass
class DetectorSetup:
    tag_id: int
    fs: float
    tip: float
    tip_secondary: Optional[float]
    k: int
    pf: float
    detection_margin: float
    null_permutations: int
    flown_lock_ratio: float
    n_w: int
    n_ol: int
    n_ws: int
    N: int
    N_exact: float
    N_B: Optional[int]
    N_B_exact: Optional[float]
    nfft: int
    samples_needed: int
    fold_offsets: np.ndarray
    W: np.ndarray
    Wf: np.ndarray
    hypotheses: Optional[list]
    usable_band_mask: np.ndarray
    frequency_mask: np.ndarray
    expected_offset_hz: Optional[float]


def build_setup(startup):
    """Rebuild pulse_detector main()'s search geometry from its startup record.

    Raises ValueError if the rebuilt geometry disagrees with what the flight
    logged, which means main() changed and this mirror must follow.
    """
    fs, tp, tip = float(startup['fs']), float(startup['tp']), float(startup['tip'])
    tip_secondary = startup.get('tip_secondary')
    k = int(startup['K'])
    n_w = int(np.ceil(tp * fs))
    n_ol = n_w // 2
    n_ws = n_w - n_ol
    N_exact = tip * fs / n_ws
    N = int(np.floor(N_exact))
    fold_offsets = np.round(np.arange(k) * N_exact).astype(int)
    N_B = N_B_exact = None
    if tip_secondary is not None:
        N_B_exact = tip_secondary * fs / n_ws
        N_B = int(np.floor(N_B_exact))
    W, Wf = pd.build_weighting_matrix(n_w, fs)
    nfft = W.shape[1]
    usable = np.abs(Wf) <= pd.USABLE_BAND_FRACTION * fs / 2.0
    samples_needed = pd.compute_segment_samples(n_ws, n_ol, k, N_exact, N_B_exact)
    hypotheses = (pd.build_hypothesis_indices(N_exact, k, (samples_needed - n_ol) // n_ws, N_B_exact)
                  if N_B_exact is not None else None)

    freq, center = startup.get('freq'), startup.get('center_freq') or 0.0
    expected = (freq - center * 1e6) if freq and center > 0 else None
    frequency_mask = usable.copy()
    if expected is not None:
        frequency_mask &= np.abs(Wf - expected) <= pd.ACQUISITION_SEARCH_HZ

    rebuilt = {'n_w': n_w, 'n_ol': n_ol, 'n_ws': n_ws, 'N': N, 'N_B': N_B,
               'nfft': nfft, 'samples_needed': samples_needed}
    drift = {key: (startup[key], value) for key, value in rebuilt.items()
             if key in startup and startup[key] != value}
    if drift:
        raise ValueError(f'detector geometry differs from the flight startup record '
                         f'(logged, rebuilt): {drift}')

    return DetectorSetup(
        tag_id=int(startup['tag_id']), fs=fs, tip=tip, tip_secondary=tip_secondary,
        k=k, pf=float(startup['pf']),
        detection_margin=float(startup.get('detection_margin', 1.0)),
        null_permutations=int(startup.get('null_permutations', 40)),
        flown_lock_ratio=float(startup.get('lock_score_ratio', DEFAULT_FLOWN_LOCK_RATIO)),
        n_w=n_w, n_ol=n_ol, n_ws=n_ws, N=N, N_exact=N_exact, N_B=N_B, N_B_exact=N_B_exact,
        nfft=nfft, samples_needed=samples_needed, fold_offsets=fold_offsets, W=W, Wf=Wf,
        hypotheses=hypotheses, usable_band_mask=usable, frequency_mask=frequency_mask,
        expected_offset_hz=expected)


@dataclass
class HeadingSlice:
    cycle: int
    slice_id: int
    heading_deg: float
    label: str
    segment_start_s: float
    power: np.ndarray
    gap_windows: List[Tuple[int, int]]
    had_gap: bool
    flown_n_perm: Optional[int]


@dataclass
class TagRotation:
    rotation_dir: str
    setup: DetectorSetup
    slices: List[HeadingSlice]
    antenna_id: int
    center_freq_mhz: float
    flown_bearing: Optional[dict]


def _read_jsonl(path):
    """Records of a detector .jsonl log. Malformed or non-object lines (a line
    cut short by a power loss) are skipped with one warning per file."""
    records = []
    if not os.path.exists(path):
        return records
    bad, first_bad = 0, None
    with open(path, encoding='utf-8') as f:
        for line_no, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                record = None
            if not isinstance(record, dict):
                bad += 1
                first_bad = first_bad or line_no
                continue
            records.append(record)
    if bad:
        print(f'WARNING: {path}: skipped {bad} malformed line(s) (first at line {first_bad})',
              file=sys.stderr, flush=True)
    return records


STARTUP_SETUP_KEYS = ('tp', 'tip', 'tip_secondary', 'K', 'fs', 'pf', 'detection_margin',
                      'null_permutations', 'lock_score_ratio', 'freq', 'center_freq',
                      'n_w', 'n_ol', 'nfft', 'samples_needed', 'N', 'N_B')


def _startup_record(rotation_dir, tag_id, heading_dirs, read_jsonl=_read_jsonl):
    """The tag's detector startup record. A restarted detector logs another;
    every one must agree on what the replay rebuilds from it."""
    records = [record
               for path in [os.path.join(rotation_dir, f'detector_{tag_id}.jsonl')] + [
                   os.path.join(d, f'detector_{tag_id}.jsonl') for d in heading_dirs]
               for record in read_jsonl(path) if record.get('type') == 'startup']
    if not records:
        raise ValueError(f'{rotation_dir}: no detector startup record for tag {tag_id}')
    first = records[0]
    for record in records[1:]:
        changed = [key for key in STARTUP_SETUP_KEYS if record.get(key) != first.get(key)]
        if changed:
            raise ValueError(f'{rotation_dir}: tag {tag_id}: detector startup configuration '
                             f'changed within the rotation ({", ".join(changed)})')
    return first


def _flown_bearing(rotation_dir, tag_id):
    path = os.path.join(rotation_dir, 'bearing_result.log')
    if not os.path.exists(path):
        return None
    with open(path, encoding='utf-8') as f:
        lines = [line.strip().split(',') for line in f if line.strip()]
    if not lines:
        return None
    header = lines[0]
    for row in lines[1:]:
        entry = dict(zip(header, row))
        if entry.get('tag_id') == str(tag_id):
            return entry
    return None


def load_rotation(rotation_dir):
    """One TagRotation per tag with dumps under heading-*/, slices in flight order."""
    if not os.path.isdir(rotation_dir):
        raise FileNotFoundError(f'not a directory: {rotation_dir}')
    heading_dirs = sorted(d for d in glob.glob(os.path.join(rotation_dir, 'heading-*'))
                          if os.path.isdir(d))
    session = {}
    session_path = os.path.join(rotation_dir, 'session.json')
    if os.path.exists(session_path):
        with open(session_path, encoding='utf-8') as f:
            session = json.load(f)
    antenna_id = int((session.get('collection') or {}).get('antenna_id', 0))

    by_tag: Dict[int, List[Tuple[str, str]]] = {}
    for d in heading_dirs:
        for meta_path in glob.glob(os.path.join(d, 'tag*_cycle_*_meta.json')):
            m = re.match(r'tag(\d+)_cycle_\d+_meta\.json$', os.path.basename(meta_path))
            if m:
                by_tag.setdefault(int(m.group(1)), []).append((d, meta_path))

    rotations = []
    jsonl_cache = {}

    def read_jsonl(path):
        if path not in jsonl_cache:
            jsonl_cache[path] = _read_jsonl(path)
        return jsonl_cache[path]

    # A tag whose first dump already failed has logs but no dumps at all.
    if by_tag:
        for d in heading_dirs:
            for log_path in glob.glob(os.path.join(d, 'detector_*.jsonl')):
                m = re.match(r'detector_(\d+)\.jsonl$', os.path.basename(log_path))
                if m and any(r.get('type') == 'cycle_threshold' for r in read_jsonl(log_path)):
                    by_tag.setdefault(int(m.group(1)), [])

    for tag_id, entries in sorted(by_tag.items()):
        startup = _startup_record(rotation_dir, tag_id, heading_dirs, read_jsonl)
        setup = build_setup(startup)
        # The detector stops dumping after a write error but keeps flying
        # cycles; replaying without those headings would be a different rotation.
        dumped = {(d, int(re.search(r'_cycle_(\d+)_meta', meta_path).group(1)))
                  for d, meta_path in entries}
        undumped = sorted(f'{os.path.basename(d)} cycle {r["cycle"]}' for d in heading_dirs
                          for r in read_jsonl(os.path.join(d, f'detector_{tag_id}.jsonl'))
                          if r.get('type') == 'cycle_threshold'
                          and (d, r.get('cycle')) not in dumped)
        if undumped:
            raise ValueError(f'{rotation_dir}: tag {tag_id}: no spectrogram dump for flown '
                             f'cycle(s) {", ".join(undumped)}')
        # Retro-measured reports land in the heading log open when the lock
        # came, not in the slice's own directory.
        all_reports = [r for d in heading_dirs
                       for r in read_jsonl(os.path.join(d, f'detector_{tag_id}.jsonl'))
                       if 'slice_id' in r and 'heading_deg' in r]
        slices = []
        for d, meta_path in entries:
            with open(meta_path, encoding='utf-8') as f:
                meta = json.load(f)
            cycle = int(meta['cycle'])
            timestamp_ns = int(meta['timestamp_ns'])
            records = [r for r in read_jsonl(os.path.join(d, f'detector_{tag_id}.jsonl'))
                       if r.get('cycle') == cycle]
            threshold = next((r for r in records if r.get('type') == 'cycle_threshold'), {})
            report = next((r for r in records if 'slice_id' in r and 'heading_deg' in r), None)
            if report is None:
                # Match the timestamp too: a restarted detector reuses cycle numbers.
                report = next((r for r in all_reports if r.get('cycle') == cycle
                               and abs(int(r.get('timestamp_ns', timestamp_ns)) - timestamp_ns)
                               < 1_000_000), {})
            label = os.path.basename(d)
            heading_match = re.match(r'heading-(\d+)', label)
            heading = report.get('heading_deg')
            if heading is None:
                heading = float(heading_match.group(1)) if heading_match else 0.0
            n_ol = int(meta.get('n_ol', setup.n_ol))
            n_ws = int(meta.get('n_w', setup.n_w)) - n_ol
            gap_fills = meta.get('gap_fills') or []
            slices.append(HeadingSlice(
                cycle=cycle,
                slice_id=int(report.get('slice_id', cycle)),
                heading_deg=float(heading),
                label=label,
                segment_start_s=timestamp_ns / 1e9,
                power=np.load(meta_path.replace('_meta.json', '_power.npy')),
                gap_windows=[(max(0, (off - n_ol) // n_ws), (off + length - 1) // n_ws)
                             for off, length in gap_fills],
                had_gap=bool(meta.get('had_gap')),
                flown_n_perm=threshold.get('n_perm')))
        slices.sort(key=lambda s: (s.segment_start_s, s.cycle))
        # A restarted detector starts with an empty candidate bank and slice
        # buffer; the replay keeps one per rotation, so it cannot follow.
        for prev, cur in zip(slices, slices[1:]):
            if cur.cycle <= prev.cycle:
                raise ValueError(f'{rotation_dir}: tag {tag_id}: detector restarted within '
                                 f'the rotation (cycle {prev.cycle} at {prev.label} -> '
                                 f'cycle {cur.cycle} at {cur.label}); not replayable')
        rotations.append(TagRotation(
            rotation_dir=rotation_dir, setup=setup, slices=slices, antenna_id=antenna_id,
            center_freq_mhz=float(startup.get('center_freq') or 0.0),
            flown_bearing=_flown_bearing(rotation_dir, tag_id)))
    return rotations


# ---------------------------------------------------------------------------
# Replay
# ---------------------------------------------------------------------------

def full_search(setup, sl, n_perm):
    """The flight's full-band fold search on one heading (collection mode)."""
    state = {'rng': np.random.default_rng([setup.tag_id, sl.cycle, 0])}
    detections, nodet_noise, best = pd.fold_detect(
        sl.power, setup.N, setup.pf, setup.fs, setup.nfft, setup.n_w, setup.n_ol,
        setup.samples_needed, state, fold_offsets=setup.fold_offsets, W=setup.W, Wf=setup.Wf,
        detection_margin=setup.detection_margin, hypotheses=setup.hypotheses,
        N_B=setup.N_B, N_A_exact=setup.N_exact, N_B_exact=setup.N_B_exact,
        frequency_mask=setup.frequency_mask, max_detections=setup.nfft, k_folds=setup.k,
        n_null_permutations=n_perm, null_time_budget_s=None, cycle=sl.cycle)
    return detections, nodet_noise, best


def make_research(setup, n_perm):
    """research(slice, power, bins) with one cached, fixed-seed search per (slice, bins)."""
    cache = {}

    def research(sl, power, bins):
        bins = bins[setup.usable_band_mask[bins]]
        key = (id(sl), tuple(int(b) for b in bins))
        if key not in cache:
            if bins.size == 0:
                cache[key] = ([], None)
            else:
                state = {'rng': np.random.default_rng([setup.tag_id, sl.cycle, int(bins[0]) + 1])}
                found, _, best = pd.fold_detect(
                    power[bins], setup.N, setup.pf, setup.fs, setup.nfft, setup.n_w,
                    setup.n_ol, setup.samples_needed, state,
                    fold_offsets=setup.fold_offsets, Wf=setup.Wf[bins],
                    detection_margin=setup.detection_margin, hypotheses=setup.hypotheses,
                    N_B=setup.N_B, N_A_exact=setup.N_exact, N_B_exact=setup.N_B_exact,
                    max_detections=1, k_folds=setup.k, n_null_permutations=n_perm,
                    null_time_budget_s=None)
                cache[key] = (found, best)
        return cache[key]
    return research


@dataclass
class RatioReplay:
    lock_ratio: float
    lock_slice: Optional[int]                     # index into slices of the first lock
    lock_score: Optional[float]
    bank: list                                    # pd.PulseLock per candidate
    reports: Dict[Tuple[int, int], object]        # (slice index, candidate) -> CandidateReport
    acquisition: Dict[int, Optional[object]]      # slice index -> strongest Detection before lock
    # slice index -> noise PSD the no-detection acquisition report carried
    acquisition_noise: Dict[int, float] = field(default_factory=dict)
    observations: List[Observation] = field(default_factory=list)
    outcome: Optional[ControllerOutcome] = None


def replay_lock_ratio(setup, slices, full_results, lock_ratio,
                      research: Callable, pattern_db, confidence_floor, nodet_noise=None):
    """Detector candidate bank + measurements, then the controller's verdict.

    full_results[i] is slice i's list of full-search Detections and
    nodet_noise[i] the noise PSD its full search reports when nothing
    cleared. research is research(slice, power, bins) -> fold_detect's
    (detections, best).
    """
    candidates = pd.RotationCandidates(
        tip=setup.tip, tip_secondary=setup.tip_secondary, n_ws=setup.n_ws, fs=setup.fs,
        lock_score_ratio=lock_ratio, expected_offset_hz=setup.expected_offset_hz)
    buffered = deque(maxlen=pd.MAX_BUFFERED_SLICES)
    reports, acquisition, acquisition_noise = {}, {}, {}
    lock_slice = lock_score = None
    for i, sl in enumerate(slices):
        current = {
            'index': i, 'slice': sl, 'slice_id': sl.slice_id, 'cycle': sl.cycle,
            'power': sl.power, 'noise_power': pd.slice_noise_power(sl.power, sl.gap_windows),
            'segment_start_s': sl.segment_start_s, 'had_gap': sl.had_gap,
            'gap_windows': sl.gap_windows, 'heading_deg': sl.heading_deg,
            'sightings': {}, 'measured': set(),
        }
        buffered.append(current)
        was_locked = bool(candidates.bank)
        candidates.offer(current, full_results[i], sl.segment_start_s)
        if not candidates.bank:
            acquisition[i] = full_results[i][0] if full_results[i] else None
            if nodet_noise is not None:
                acquisition_noise[i] = nodet_noise[i]
            continue
        if not was_locked:
            lock_slice = i
            lock_score = candidates.bank[0].score_ratio
        for b, candidate_id, candidate in candidates.pending(buffered):
            result = pd.measure_candidate(
                b, candidate_id, candidate,
                lambda power, bins, _sl=b['slice']: research(_sl, power, bins),
                setup.Wf, setup.n_ws, setup.fs, setup.n_w, setup.tip, setup.tip_secondary)
            b['measured'].add(candidate_id)
            if result.measurable:
                reports[(b['index'], candidate_id)] = result

    replay = RatioReplay(lock_ratio=lock_ratio, lock_slice=lock_slice, lock_score=lock_score,
                         bank=list(candidates.bank), reports=reports, acquisition=acquisition,
                         acquisition_noise=acquisition_noise)
    replay.observations = rotation_observations(slices, replay)
    replay.outcome = controller_outcome(pattern_db, confidence_floor, replay.observations)
    return replay


def rotation_observations(slices, replay):
    """The controller's stored reports: locked results replace acquisition ones."""
    observations = []
    for i, sl in enumerate(slices):
        for candidate_id in range(max(1, len(replay.bank))):
            result = replay.reports.get((i, candidate_id))
            if result is not None:
                observations.append(Observation(
                    heading_deg=sl.heading_deg, detected=result.detected,
                    candidate_id=candidate_id,
                    signal_power=result.signal_power_psd if result.detected else 0.0,
                    snr_db=result.snr_db, sighted=result.detected and result.sighted,
                    noise_psd=result.noise_psd, freq_hz=result.freq_hz, slice_index=i))
            elif candidate_id == 0 and i in replay.acquisition:
                det = replay.acquisition[i]
                if det is None:
                    observations.append(Observation(
                        heading_deg=sl.heading_deg, detected=False,
                        noise_psd=replay.acquisition_noise.get(i, 0.0), slice_index=i))
                else:
                    observations.append(Observation(
                        heading_deg=sl.heading_deg, detected=True,
                        signal_power=det.signal_power, snr_db=det.snr_db,
                        noise_psd=det.noise_psd, freq_hz=det.freq_hz, slice_index=i))
    return observations


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------

def _fmt_heading(deg):
    return f'{deg:03.0f}°'


def _fmt_float(value, fmt):
    return '—' if value is None or not math.isfinite(value) else format(value, fmt)


def print_full_search(rot, full_results, best_below, n_perms):
    print(f'Full-band search per heading (independent of the lock ratio; '
          f'offset from {rot.center_freq_mhz:.6f} MHz / score ratio):')
    print(f'  {"heading":<8} {"cycle":>5} {"perms":>5}  detections ≥ threshold'
          f'                       best below threshold')
    for sl, dets, best, n_perm in zip(rot.slices, full_results, best_below, n_perms):
        det_str = '  '.join(
            f'{d.freq_hz:+.0f}/{d.score_ratio:.2f}{"[DF]" if d.fold_info["dominant_fold"] else ""}'
            for d in dets) or '—'
        best_str = (f'{best["freq_hz"]:+.0f}/{best["score_ratio"]:.2f}' if best else '')
        print(f'  {_fmt_heading(sl.heading_deg):<8} {sl.cycle:>5} {n_perm:>5}  '
              f'{det_str:<44} {best_str}')
    print('  [DF] = dominant fold (one fold carries the score); never locks.')


def print_sweep(rot, replays):
    print('Lock-ratio sweep (controller verdict for the selected candidate):')
    print(f'  {"ratio":>5}  {"first lock":<16} {"cands":>5}  {"bearing":>7}  {"conf":>5}  '
          f'{"valid":>5}  {"sighted":>7}  {"confirmed":>9}  {"heard":>5}  {"revisit?":>8}  note')
    for r in replays:
        o = r.outcome
        lock = (f'{_fmt_heading(rot.slices[r.lock_slice].heading_deg)} ({r.lock_score:.2f})'
                if r.lock_slice is not None else 'none')
        sel = o.selected
        flown = '  (as flown)' if math.isclose(r.lock_ratio, rot.setup.flown_lock_ratio) else ''
        print(f'  {r.lock_ratio:>5.2f}  {lock:<16} {len(r.bank):>5}  '
              f'{_fmt_float(o.bearing_deg, ".1f"):>7}  {_fmt_float(o.confidence, ".3f"):>5}  '
              f'{o.reported_valid_slices:>5}  {sel.n_sighted_slices:>7}  '
              f'{"yes" if o.confirmed else "no":>9}  {"yes" if o.heard else "no":>5}  '
              f'{"yes" if o.revisit_eligible else "no":>8}  {o.reason}{flown}')


def selected_heading_results(replay):
    """(candidate id, per-heading results) of the controller's selected candidate.
    source is 'sighted', 'research', 'acquisition' or None when not found."""
    cid = replay.outcome.selected.candidate_id if replay.outcome.selected else 0
    results = []
    for o in replay.observations:
        if o.candidate_id != cid:
            continue
        report = replay.reports.get((o.slice_index, cid))
        if not o.detected:
            source = None
        elif report is None:
            source = 'acquisition'
        else:
            source = 'sighted' if report.sighted else 'research'
        results.append({
            'heading_deg': o.heading_deg, 'detected': o.detected,
            'snr_db': o.snr_db if o.detected else None,
            'freq_hz': o.freq_hz if o.detected else None, 'source': source,
            'research_ratio': report.research_ratio if source == 'research' else None})
    return cid, results


def print_headings(rot, replays):
    """Per-heading result of the selected candidate, one row per distinct outcome."""
    print('Per-heading results of the selected candidate '
          '(S = sighted by full search, R = re-search ratio, A = acquisition hit, · = not found):')
    seen = {}
    for r in replays:
        cells = []
        for h in selected_heading_results(r)[1]:
            if h['source'] is None:
                cells.append(f'{_fmt_heading(h["heading_deg"])} ·')
                continue
            if h['source'] == 'research':
                tag = f'R{h["research_ratio"]:.2f}'
            else:
                tag = 'S' if h['source'] == 'sighted' else 'A'
            cells.append(f'{_fmt_heading(h["heading_deg"])} {h["snr_db"]:.1f}dB {tag}')
        line = '  |  '.join(cells)
        seen.setdefault(line, []).append(r.lock_ratio)
    for line, ratios in seen.items():
        print(f'  ratio {",".join(f"{x:g}" for x in ratios):<14} {line}')


def replay_to_json(rot, full_results, best_below, n_perms, replays):
    return {
        'rotation_dir': rot.rotation_dir,
        'tag_id': rot.setup.tag_id,
        'center_freq_mhz': rot.center_freq_mhz,
        'flown_lock_ratio': rot.setup.flown_lock_ratio,
        'flown_bearing': rot.flown_bearing,
        'headings': [{
            'heading_deg': sl.heading_deg, 'cycle': sl.cycle, 'n_perm': n_perm,
            'detections': [{'freq_hz': d.freq_hz, 'score_ratio': d.score_ratio,
                            'snr_db': d.snr_db, 'dominant_fold': bool(d.fold_info['dominant_fold'])}
                           for d in dets],
            # Strongest sub-threshold bin when nothing cleared (fold_detect's
            # best_candidate); None when there were detections.
            'best_below': ({'freq_hz': best['freq_hz'], 'score_ratio': best['score_ratio'],
                            'snr_db': best['snr_db']} if best else None),
        } for sl, dets, best, n_perm in zip(rot.slices, full_results, best_below, n_perms)],
        'sweep': [{
            'lock_ratio': r.lock_ratio,
            'lock_heading_deg': (rot.slices[r.lock_slice].heading_deg
                                 if r.lock_slice is not None else None),
            'lock_score': r.lock_score,
            'candidates': [{'freq_hz': c.freq_hz, 'score_ratio': c.score_ratio} for c in r.bank],
            'selected_candidate_id': cid,
            'selected_headings': heading_results,
            'bearing_deg': None if not math.isfinite(r.outcome.bearing_deg) else r.outcome.bearing_deg,
            'confidence': r.outcome.confidence,
            'n_valid_slices': r.outcome.reported_valid_slices,
            'n_sighted_slices': r.outcome.selected.n_sighted_slices,
            'confirmed': r.outcome.confirmed,
            'heard': r.outcome.heard,
            'revisit_eligible': r.outcome.revisit_eligible,
            'reason': r.outcome.reason,
        } for r, (cid, heading_results) in zip(
            replays, (selected_heading_results(r) for r in replays))],
    }


def run_rotation(rot, lock_ratios, n_perm_override):
    setup = rot.setup
    if rot.antenna_id not in ANTENNA_PATTERNS:
        raise ValueError(f'unknown antenna_id {rot.antenna_id}')
    antenna_name, pattern_db, floor = ANTENNA_PATTERNS[rot.antenna_id]

    full_results, nodet_noise, best_below, n_perms = [], [], [], []
    for sl in rot.slices:
        n_perm = n_perm_override or sl.flown_n_perm or setup.null_permutations
        if not n_perm_override and sl.flown_n_perm is None:
            print(f'WARNING: {sl.label} cycle {sl.cycle}: no cycle_threshold record; '
                  f'full search uses the startup default of {n_perm} permutations, '
                  f'not the flown count', file=sys.stderr, flush=True)
        print(f'  searching {sl.label} (cycle {sl.cycle}, {n_perm} permutations)...',
              file=sys.stderr, flush=True)
        dets, noise, best = full_search(setup, sl, n_perm)
        full_results.append(dets)
        nodet_noise.append(noise)
        best_below.append(best)
        n_perms.append(n_perm)

    research = make_research(setup, n_perm_override or setup.null_permutations)
    replays = [replay_lock_ratio(setup, rot.slices, full_results, ratio, research,
                                 pattern_db, floor, nodet_noise)
               for ratio in sorted(lock_ratios)]

    print(f'\n=== {os.path.basename(os.path.normpath(rot.rotation_dir))} — tag {setup.tag_id} '
          f'({len(rot.slices)} headings, antenna {antenna_name}) ===')
    if rot.flown_bearing:
        fb = rot.flown_bearing
        print(f'Flown result: bearing {fb.get("bearing_deg")}  confidence {fb.get("r_squared")}  '
              f'valid {fb.get("n_valid_slices")}  sighted {fb.get("n_sighted_slices")}  '
              f'confirmed {fb.get("confirmed")}  heard {fb.get("heard")}')
    print_full_search(rot, full_results, best_below, n_perms)
    print_sweep(rot, replays)
    print_headings(rot, replays)
    return replay_to_json(rot, full_results, best_below, n_perms, replays)


def parse_ratios(text):
    try:
        ratios = [float(x) for x in text.split(',') if x.strip()]
    except ValueError:
        raise argparse.ArgumentTypeError(f'not a comma-separated list of numbers: {text!r}')
    if not ratios or any(not math.isfinite(r) or r <= 0 for r in ratios):
        raise argparse.ArgumentTypeError('lock ratios must be finite and positive')
    return ratios


def permutation_count(text):
    try:
        value = int(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f'not an integer: {text!r}')
    if value < pd.NULL_MIN_PERMUTATIONS:
        raise argparse.ArgumentTypeError(
            f'must be at least {pd.NULL_MIN_PERMUTATIONS} (the detector\'s minimum), got {value}')
    return value


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('rotation_dirs', nargs='+',
                    help='Rotation log directories (Logs-Rotation-*) recorded with spectrogram dumps')
    ap.add_argument('--lock-ratios', type=parse_ratios, default=list(DEFAULT_LOCK_RATIOS),
                    help='Comma-separated lock score ratios to replay '
                         f'(default: {",".join(f"{r:g}" for r in DEFAULT_LOCK_RATIOS)})')
    ap.add_argument('--null-permutations', type=permutation_count, default=None,
                    help='Permutations for every null (default: each heading\'s flown count '
                         'for the full search, the startup record\'s for re-searches)')
    ap.add_argument('--json', default=None, help='Also write the results to this JSON file')
    args = ap.parse_args(argv)

    output, failed = [], 0
    for rotation_dir in args.rotation_dirs:
        try:
            rotations = load_rotation(rotation_dir)
            if not rotations:
                failed += 1
                print(f'ERROR: {rotation_dir}: no heading-*/tag*_cycle_*_meta.json dumps '
                      f'(was the rotation flown with spectrogram dumping on?); skipped',
                      file=sys.stderr, flush=True)
                continue
            for rot in rotations:
                output.append(run_rotation(rot, args.lock_ratios, args.null_permutations))
        except (OSError, ValueError, KeyError) as exc:
            # One unreadable rotation (older log layout, changed detector
            # geometry) must not end a sweep over a whole campaign.
            failed += 1
            print(f'ERROR: {rotation_dir}: {type(exc).__name__}: {exc}; skipped',
                  file=sys.stderr, flush=True)
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as f:
            json.dump(output, f, indent=1)
    if failed:
        print(f'{failed} of {len(args.rotation_dirs)} rotation(s) failed', file=sys.stderr)
    return 0 if output and not failed else 1


if __name__ == '__main__':
    sys.exit(main())
