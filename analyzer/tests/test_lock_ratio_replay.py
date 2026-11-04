"""Tests for lock_ratio_replay: the Python port of the controller's bearing
fit and reporting rules, the antenna tables it mirrors, the lock-ratio gate,
rotation loading, and the guard that the rebuilt detector geometry still
matches the flight's startup record."""

import json
import math
import re
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from lock_ratio_replay import (  # noqa: E402
    ANTENNA_PATTERNS, HeadingSlice, Observation, _read_jsonl, build_setup,
    controller_outcome, load_rotation, main, make_research, replay_lock_ratio,
)
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "detector"))
from pulse_detector import Detection  # noqa: E402

REPO = Path(__file__).resolve().parent.parent.parent

STARTUP = {
    'type': 'startup', 'tp': 0.019, 'tip': 1.5, 'tip_secondary': 2.0, 'K': 5,
    'fs': 3840.0, 'pf': 0.05, 'detection_margin': 1.0, 'null_permutations': 40,
    'center_freq': 146.788, 'freq': 146788000, 'tag_id': 4,
}


def test_bearing_fit_matches_controller_on_flight():
    # 2026-11-04 14-51-10 rotation: per-pulse power and noise per heading as
    # reported by the detector; the controller logged 195.5 deg, 0.950373.
    flight = [
        (0.0, 2.1225745585312827e-11, 1.4861347059108702e-12, 25.13, True),
        (45.0, 7.10655915646669e-12, 1.3012593539479837e-12, 21.74, False),
        (90.0, None, 1.221556899266181e-12, 0.0, False),
        (135.0, 6.268937555771769e-12, 1.2517965355305147e-12, 21.47, False),
        (180.0, 8.994698871060583e-11, 1.4094947431431138e-12, 31.19, True),
        (225.0, 7.348434964535084e-11, 1.219176256652647e-12, 30.95, True),
        (270.0, 5.096890864716821e-12, 1.1734043467982311e-12, 21.03, False),
        (315.0, None, 1.2379109406749473e-12, 0.0, False),
    ]
    observations = [
        Observation(heading_deg=h, detected=p is not None, signal_power=p or 0.0,
                    snr_db=snr, sighted=sighted, noise_psd=noise, freq_hz=-263.0)
        for h, p, noise, snr, sighted in flight]
    _, pattern_db, floor = ANTENNA_PATTERNS[0]
    outcome = controller_outcome(pattern_db, floor, observations)
    assert outcome.bearing_deg == pytest.approx(195.5)
    assert outcome.confidence == pytest.approx(0.950373, abs=1e-6)
    assert outcome.selected.n_valid_slices == 6
    assert outcome.selected.n_sighted_slices == 3
    assert outcome.confirmed and outcome.heard and not outcome.revisit_eligible


C_NUMBER = r'[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?'


def test_antenna_tables_match_controller():
    source = (REPO / 'controller' / 'AntennaPattern.cpp').read_text()
    for antenna_id, (name, table, floor) in ANTENNA_PATTERNS.items():
        block = re.search(r'\{\s*ANTENNA_ID_\w+,\s*"' + re.escape(name)
                          + r'",\s*\{(.*?)\},\s*(' + C_NUMBER + r')f?', source, re.S)
        assert block, f'{name} not found in AntennaPattern.cpp'
        body = re.sub(r'//[^\n]*', '', block.group(1))
        values = [float(v) for v in re.findall(C_NUMBER, body)]
        assert values == list(table), name
        assert float(block.group(2)) == floor, name


def test_geometry_drift_from_startup_record_is_rejected():
    build_setup({**STARTUP, 'nfft': 146})
    with pytest.raises(ValueError, match='nfft'):
        build_setup({**STARTUP, 'nfft': 128})


def test_malformed_jsonl_lines_are_skipped_with_one_warning(tmp_path, capsys):
    log = tmp_path / 'detector_4.jsonl'
    log.write_text('{"type": "startup", "K": 20}\n'
                   '\n'
                   '42\n'                              # parses, but not a record
                   '{"type": "cycle_threshold", "n_p\n')  # truncated by power loss
    assert _read_jsonl(str(log)) == [{'type': 'startup', 'K': 20}]
    err = capsys.readouterr().err
    assert err.count('WARNING') == 1
    assert 'detector_4.jsonl: skipped 2 malformed line(s) (first at line 3)' in err


def _slice(setup, index, heading, rng):
    n_time = (setup.samples_needed - setup.n_ol) // setup.n_ws
    power = rng.exponential(1.0, size=(setup.nfft, n_time)).astype(np.float32)
    return HeadingSlice(cycle=index + 1, slice_id=index + 1, heading_deg=heading,
                        label=f'heading-{heading:03.0f}', segment_start_s=100.0 * index,
                        power=power, gap_windows=[], had_gap=False, flown_n_perm=None)


def _train(setup, slice_, freq_hz, score_ratio, amplitude):
    """A K-pulse train injected at freq_hz, reported as a full-search detection."""
    freq_bin = int(np.argmin(np.abs(setup.Wf - freq_hz)))
    windows = [int(10 + w) for w in setup.fold_offsets]
    slice_.power[freq_bin, windows] += amplitude
    return Detection(freq_hz=float(setup.Wf[freq_bin]), snr_db=20.0, signal_power=amplitude,
                     offset=windows[0], noise_psd=1.0, stft_score=1.0,
                     score_ratio=score_ratio, hyp_label='A',
                     fold_info={'max_fold_fraction': 0.2, 'dominant_fold': False,
                                'fold_snrs': [], 'fold_windows': windows})


def _no_research(_slice, _power, _bins):
    return [], None


@pytest.mark.parametrize('lock_ratio, locks', [
    (2.0, True),    # best heading scores 2.5: clears the gate
    (3.0, False),   # same data: never locks, acquisition hits only
])
def test_lock_ratio_gates_bearing(lock_ratio, locks):
    setup = build_setup(STARTUP)
    rng = np.random.default_rng(7)
    headings = [0.0, 90.0, 180.0, 270.0]
    slices = [_slice(setup, i, h, rng) for i, h in enumerate(headings)]
    full = [[_train(setup, slices[0], -263.0, 2.5, 400.0)],
            [],
            [_train(setup, slices[2], -263.0, 1.4, 60.0)],
            []]
    _, pattern_db, floor = ANTENNA_PATTERNS[0]
    replay = replay_lock_ratio(setup, slices, full, lock_ratio, _no_research,
                               pattern_db, floor)
    outcome = replay.outcome
    if locks:
        assert replay.lock_slice == 0 and replay.lock_score == pytest.approx(2.5)
        assert outcome.selected.n_sighted_slices == 1
        assert math.isfinite(outcome.bearing_deg)
        assert outcome.revisit_eligible and not outcome.confirmed
    else:
        assert replay.lock_slice is None and not replay.bank
        assert math.isnan(outcome.bearing_deg)
        # Two acquisition hits in the same bin: heard, but no bearing without a lock.
        assert outcome.heard
        assert outcome.reason == 'no lock; acquisition hits agree in frequency'


def test_locked_results_replace_earlier_acquisition_reports():
    # Heading 0 is only an acquisition hit (1.5 < 3.0); heading 1 locks. The
    # controller then stores the lock's own verdict on heading 0, not the hit.
    setup = build_setup(STARTUP)
    rng = np.random.default_rng(11)
    slices = [_slice(setup, i, h, rng) for i, h in enumerate([0.0, 180.0])]
    full = [[_train(setup, slices[0], -263.0, 1.5, 50.0)],
            [_train(setup, slices[1], -263.0, 4.0, 400.0)]]
    _, pattern_db, floor = ANTENNA_PATTERNS[0]
    replay = replay_lock_ratio(setup, slices, full, 3.0, _no_research, pattern_db, floor)
    assert replay.lock_slice == 1
    heading0 = [o for o in replay.observations if o.slice_index == 0]
    assert len(heading0) == 1
    assert (0, 0) in replay.reports
    assert not heading0[0].detected   # the re-search found nothing; the hit is gone


def test_no_detection_acquisition_reports_carry_noise():
    # The controller weights every heading by the noise its report carried,
    # including a no-detection acquisition report.
    setup = build_setup(STARTUP)
    rng = np.random.default_rng(13)
    slices = [_slice(setup, i, h, rng) for i, h in enumerate([0.0, 180.0])]
    _, pattern_db, floor = ANTENNA_PATTERNS[0]
    replay = replay_lock_ratio(setup, slices, [[], []], 3.0, _no_research, pattern_db, floor,
                               nodet_noise=[2e-12, 3e-12])
    assert [o.noise_psd for o in replay.observations] == [2e-12, 3e-12]


@pytest.mark.parametrize('second_hit_hz, heard, reported_valid', [
    (-250.0, True, 2),   # within 200 Hz of the first hit: heard, no bearing
    (900.0, False, 0),   # scattered hits: controller reports nothing heard
])
def test_unlocked_hits_report_valid_slices_only_when_heard(second_hit_hz, heard,
                                                           reported_valid):
    observations = [
        Observation(heading_deg=0.0, detected=True, signal_power=1e-12, freq_hz=-263.0),
        Observation(heading_deg=90.0, detected=True, signal_power=1e-12, freq_hz=second_hit_hz),
        Observation(heading_deg=180.0, detected=False),
    ]
    _, pattern_db, floor = ANTENNA_PATTERNS[0]
    outcome = controller_outcome(pattern_db, floor, observations)
    assert math.isnan(outcome.bearing_deg)
    assert outcome.heard == heard
    assert outcome.reported_valid_slices == reported_valid


@pytest.mark.parametrize('startup, expected', [
    (STARTUP, 3.0),                               # older logs: detector default
    ({**STARTUP, 'lock_score_ratio': 2.5}, 2.5),
])
def test_flown_lock_ratio_comes_from_startup_record(startup, expected):
    assert build_setup(startup).flown_lock_ratio == expected


def _write_rotation(root, headings, startup=STARTUP, n_perm=10, extra_records=()):
    """A minimal dumped rotation: noise-only spectrograms, one cycle per heading.

    headings is a list of (dir name, cycle, report[, segment start s]) where
    report, if given, is the heading's detection record (slice_id,
    heading_deg); the start defaults to 50 s per cycle. extra_records is a
    list of (dir name, record) appended to that heading's log, e.g. a
    retro-measured report filed in the log open when the lock came.
    """
    setup = build_setup(STARTUP)   # file shapes; `startup` is what gets logged
    n_time = (setup.samples_needed - setup.n_ol) // setup.n_ws
    rng = np.random.default_rng(5)
    root.mkdir()
    (root / 'session.json').write_text(json.dumps({'collection': {'antenna_id': 0}}))
    (root / 'detector_4.jsonl').write_text(json.dumps(startup) + '\n')
    for name, cycle, report, *start in headings:
        d = root / name
        d.mkdir()
        prefix = d / f'tag4_cycle_{cycle:04d}'
        np.save(f'{prefix}_power.npy',
                rng.exponential(1.0, size=(setup.nfft, n_time)).astype(np.float32))
        Path(f'{prefix}_meta.json').write_text(json.dumps({
            'cycle': cycle, 'timestamp_ns': int((start[0] if start else cycle * 50) * 1e9),
            'n_w': setup.n_w, 'n_ol': setup.n_ol,
            'gap_fills': [[370, 100]] if cycle == 1 else [], 'had_gap': cycle == 1}))
        records = [{'type': 'cycle_threshold', 'cycle': cycle, 'n_perm': n_perm}]
        if report:
            records.append({'type': 'detection', 'cycle': cycle, **report})
        records += [record for extra_dir, record in extra_records if extra_dir == name]
        (d / 'detector_4.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in records))
    return root


def test_load_rotation_orders_by_cycle_and_reads_heading_records(tmp_path):
    root = _write_rotation(tmp_path / 'rot', [
        ('heading-000', 2, None),
        ('heading-045', 3, {'slice_id': 7, 'heading_deg': 45.5}),
        ('heading-270', 1, None),
    ], extra_records=[('heading-045', {'type': 'measured', 'cycle': 2, 'slice_id': 5,
                                       'heading_deg': 0.4, 'timestamp_ns': 100_000_000_000})])
    [rot] = load_rotation(str(root))
    assert [s.cycle for s in rot.slices] == [1, 2, 3]
    # Dir name unless logged; cycle 2's retro report is in heading-045's log.
    assert [s.heading_deg for s in rot.slices] == [270.0, 0.4, 45.5]
    assert [s.slice_id for s in rot.slices] == [1, 5, 7]
    assert [s.flown_n_perm for s in rot.slices] == [10, 10, 10]
    # Zero-fill of samples [370, 470) -> STFT windows (370-36)//37 .. 469//37.
    assert rot.slices[0].gap_windows == [(9, 12)] and rot.slices[0].had_gap
    assert rot.slices[1].gap_windows == [] and rot.slices[0].segment_start_s == 50.0


def test_research_cache_is_per_slice():
    # Two slices with the same cycle number must not share a cached re-search.
    setup = build_setup(STARTUP)
    rng = np.random.default_rng(3)
    a, b = _slice(setup, 0, 0.0, rng), _slice(setup, 0, 180.0, rng)
    assert a.cycle == b.cycle
    research = make_research(setup, 10)
    bins = np.flatnonzero(setup.usable_band_mask)[:4]
    assert research(a, a.power, bins)[1]['score_ratio'] != research(b, b.power, bins)[1]['score_ratio']


def test_failed_rotation_does_not_stop_the_sweep(tmp_path, capsys):
    bad = _write_rotation(tmp_path / 'bad', [('heading-000', 1, None)],
                          startup={**STARTUP, 'nfft': 128})
    good = _write_rotation(tmp_path / 'good', [('heading-000', 1, None),
                                               ('heading-180', 2, None)])
    out = tmp_path / 'sweep.json'
    missing = tmp_path / 'not-there'
    undumped = tmp_path / 'no-dumps'
    undumped.mkdir()
    # Dumping stopped after heading-000: heading-090 was flown but has no dump.
    partial = _write_rotation(tmp_path / 'partial', [('heading-000', 1, None)])
    (partial / 'heading-090').mkdir()
    (partial / 'heading-090' / 'detector_4.jsonl').write_text(
        json.dumps({'type': 'cycle_threshold', 'cycle': 2, 'n_perm': 10}) + '\n')
    # A restarted detector came back with a different K.
    restarted = _write_rotation(tmp_path / 'restarted',
                                [('heading-000', 1, None), ('heading-180', 2, None)],
                                extra_records=[('heading-180', {**STARTUP, 'K': 6})])
    # Same config, but the restart reset the detector's candidate bank.
    relaunched = _write_rotation(tmp_path / 'relaunched', [
        ('heading-000', 1, None, 50), ('heading-090', 2, None, 100),
        ('heading-180', 1, None, 150)])
    # Tag 5 flew alongside tag 4, but its first dump write already failed.
    undumped_tag = _write_rotation(tmp_path / 'undumped_tag', [('heading-000', 1, None)])
    (undumped_tag / 'heading-000' / 'detector_5.jsonl').write_text(''.join(
        json.dumps(r) + '\n' for r in [{**STARTUP, 'tag_id': 5},
                                       {'type': 'cycle_threshold', 'cycle': 1, 'n_perm': 10}]))
    assert main([str(bad), str(missing), str(undumped), str(partial), str(restarted),
                 str(relaunched), str(undumped_tag), str(good), '--lock-ratios', '3',
                 '--null-permutations', '10', '--json', str(out)]) == 1
    err = capsys.readouterr().err
    assert f'ERROR: {bad}' in err and f'ERROR: {missing}' in err
    assert f'ERROR: {undumped}' in err
    assert 'no spectrogram dump for flown cycle(s) heading-090 cycle 2' in err
    assert 'startup configuration changed within the rotation (K)' in err
    assert ('detector restarted within the rotation '
            '(cycle 2 at heading-090 -> cycle 1 at heading-180)') in err
    assert 'tag 5: no spectrogram dump for flown cycle(s) heading-000 cycle 1' in err
    assert '7 of 8 rotation(s) failed' in err
    [result] = json.loads(out.read_text())
    assert result['rotation_dir'] == str(good)
    for heading in result['headings']:
        assert heading['n_perm'] == 10                # the count actually used
        assert 'best_below' in heading                # noise maximum for no-tag flights
    [sweep] = result['sweep']
    assert sweep['selected_candidate_id'] == 0
    assert [h['heading_deg'] for h in sweep['selected_headings']] == [0.0, 180.0]


@pytest.mark.parametrize('value', ['0', '-2', '9'])
def test_null_permutations_must_reach_detector_minimum(tmp_path, value):
    with pytest.raises(SystemExit):
        main([str(tmp_path), '--null-permutations', value])


@pytest.mark.parametrize('value', ['0', '-1', 'nan', 'inf', '2,nan'])
def test_lock_ratios_must_be_finite_and_positive(tmp_path, value):
    with pytest.raises(SystemExit):
        main([str(tmp_path), '--lock-ratios', value])
