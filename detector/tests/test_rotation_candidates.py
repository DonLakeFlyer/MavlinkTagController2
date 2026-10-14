"""Per-heading candidate results across a rotation (S1/S2/S3/S7, docs/ISSUES.md).

A dual-rate collar restarts its pulse train whenever it switches rate, so a
heading's result may only come from that heading's own data: its own fold
sighting, or a frequency-only re-search of that dwell. These tests drive
RotationCandidates + measure_candidate over synthetic headings with rate
switches between and inside dwells and check that every heading reports the
power that was actually there, exactly once. Projecting one heading's timing
onto another (the pre-2026-10 locked measurement) reads noise on every
heading after a restart and overwrote earlier headings when it re-anchored.
"""

import numpy as np
import pytest

from pulse_detector import (
    RotationCandidates,
    build_hypothesis_indices,
    build_weighting_matrix,
    compute_segment_samples,
    compute_stft_power,
    fold_detect,
    measure_candidate,
    slice_noise_power,
)

FS = 3840.0
TP = 0.015
TIP_A = 2.0
TIP_B = 1.5
K = 5
PF = 5e-2
TAG_HZ = 300.0

N_W = int(np.ceil(TP * FS))
N_OL = N_W // 2
N_WS = N_W - N_OL
N_A = TIP_A * FS / N_WS
N_B = TIP_B * FS / N_WS
W, Wf = build_weighting_matrix(N_W, FS)
NFFT = W.shape[1]
SAMPLES = compute_segment_samples(N_WS, N_OL, K, N_A, N_B)
SEG_S = SAMPLES / FS
HYPS = build_hypothesis_indices(N_A, K, (SAMPLES - N_OL) // N_WS, N_B)


def _train(start, rates):
    """Pulse start times from `start`, one gap per entry of `rates`, clipped to the segment."""
    times = [start]
    for gap in rates:
        times.append(times[-1] + gap)
    return [t for t in times if t + TP < SEG_S]


def _power(pulse_times, amplitude, seed):
    rng = np.random.default_rng(seed)
    iq = ((rng.standard_normal(SAMPLES) + 1j * rng.standard_normal(SAMPLES))
          * (0.01 / np.sqrt(2.0))).astype(np.complex64)
    n = int(np.ceil(TP * FS))
    tone = (amplitude * np.exp(2j * np.pi * TAG_HZ * np.arange(n) / FS)).astype(np.complex64)
    for t in pulse_times:
        start = int(round(t * FS))
        iq[start:start + n] += tone[:max(0, min(n, SAMPLES - start))]
    power, _ = compute_stft_power(iq, N_W, N_OL, NFFT, W=W, min_windows=K)
    return power


def _search(power, rows=None, seed=0):
    rows = np.arange(power.shape[0]) if rows is None else rows
    found, _, best = fold_detect(
        power[rows], int(N_A), PF, FS, NFFT, N_W, N_OL, SAMPLES,
        {'rng': np.random.default_rng(seed)}, Wf=Wf[rows],
        hypotheses=HYPS, N_B=int(N_B), max_detections=4, k_folds=K,
        n_null_permutations=20)
    return found, best


def _slice(slice_id, power):
    return {'collection_id': 1, 'slice_id': slice_id, 'cycle': slice_id,
            'power': power, 'noise_power': slice_noise_power(power, []),
            'segment_start_s': 100.0 * slice_id, 'had_gap': False,
            'gap_windows': [], 'heading_deg': 90.0 * slice_id,
            'sightings': {}, 'measured': set()}


def _run(headings, hide_from_full_search=()):
    """Analyse headings in order as the detector main loop does.

    `headings` is a list of (pulse_times, amplitude). Returns
    {slice_id: CandidateReport} for candidate 0 and the bank, failing if any
    (slice, candidate) result is produced twice.
    """
    candidates = RotationCandidates(TIP_A, TIP_B, N_WS, FS, lock_score_ratio=3.0)
    buffered, results = [], {}
    for slice_id, (times, amplitude) in enumerate(headings, start=1):
        power = _power(times, amplitude, seed=slice_id)
        current = _slice(slice_id, power)
        buffered.append(current)
        detections = [] if slice_id in hide_from_full_search else _search(power)[0]
        candidates.offer(current, detections, current['segment_start_s'])
        for buf, candidate_id, candidate in candidates.pending(buffered):
            report = measure_candidate(
                buf, candidate_id, candidate,
                lambda p, bins: _search(p, bins, seed=buf['slice_id']),
                Wf, N_WS, FS, N_W, TIP_A, TIP_B)
            key = (buf['slice_id'], candidate_id)
            assert key not in results, f'{key} reported twice'
            results[key] = report
            buf['measured'].add(candidate_id)
        assert candidates.pending(buffered) == []
    return {s: r for (s, c), r in results.items() if c == 0}, candidates


def _db(x):
    return 10.0 * np.log10(x)


def test_rate_switches_between_and_inside_dwells_keep_each_headings_power():
    headings = [
        # 1: rate A from 0.10 s.
        (_train(0.10, [TIP_A] * 8), 0.05),
        # 2: the train restarted between headings (0.95 s off the old grid),
        #    then switched A -> B inside the dwell.
        (_train(0.95, [TIP_A] * 2 + [TIP_B] * 8), 0.03),
        # 3: all B, another restart.
        (_train(0.40, [TIP_B] * 8), 0.02),
        # 4: back to A, short B burst inside the dwell (A -> B -> A).
        (_train(1.30, [TIP_A, TIP_B, TIP_B, TIP_A, TIP_A, TIP_A]), 0.05),
    ]
    reports, candidates = _run(headings)

    # One emitter whatever rate each heading caught it at (S3).
    assert len(candidates.bank) == 1
    assert sorted(reports) == [1, 2, 3, 4]
    assert all(r.detected and r.sighted for r in reports.values())

    # Power tracks each heading's own amplitude (the antenna pattern), to
    # within 1 dB of the truth relative to heading 1.
    ref = reports[1].signal_power_psd
    for slice_id, (_, amplitude) in enumerate(headings, start=1):
        expected_db = _db((amplitude / headings[0][1]) ** 2)
        measured_db = _db(reports[slice_id].signal_power_psd / ref)
        assert measured_db == pytest.approx(expected_db, abs=1.0), slice_id


def test_candidate_found_late_is_searched_for_on_earlier_headings():
    # Heading 1 holds the train but its full search is withheld (as if it fell
    # below the lock ratio): when heading 2 admits the candidate, heading 1 is
    # re-searched at the candidate's frequency only and measured at its own
    # phase, which is unrelated to heading 2's.
    headings = [
        (_train(1.70, [TIP_B] * 8), 0.04),
        (_train(0.20, [TIP_A] * 8), 0.04),
    ]
    reports, _ = _run(headings, hide_from_full_search={1})

    assert reports[1].detected and not reports[1].sighted
    assert reports[1].research_ratio > 1.0
    assert reports[2].sighted
    assert _db(reports[1].signal_power_psd / reports[2].signal_power_psd) == pytest.approx(0.0, abs=1.0)


def test_heading_without_the_tag_is_a_non_detection_not_a_projection():
    headings = [
        (_train(0.10, [TIP_A] * 8), 0.05),
        ([], 0.0),          # deep pattern null: nothing there
        (_train(0.60, [TIP_A] * 8), 0.05),
    ]
    reports, _ = _run(headings)

    assert reports[1].detected and reports[3].detected
    assert not reports[2].detected
    assert np.isnan(reports[2].signal_power_psd)
    assert reports[2].noise_psd > 0.0
