"""Fixture tests for post_flight_analysis.py.

Builds synthetic plain-detection and rotation log directories and pins the
report's conclusions (decimator aggregation across headings, sustained vs.
transient rate-warning classification, sidecar sizing, truncated JSONL,
session duration) to representative inputs.
"""

import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from post_flight_analysis import (  # noqa: E402
    RATE_WARNING_TRANSIENT_MAX, generate_report, parse_decimator_log,
)


def _jsonl(path: Path, entries):
    with open(path, 'w', encoding='utf-8') as f:
        for e in entries:
            f.write(json.dumps(e) + '\n')


def _detector_entries(tag_id, cycles, elapsed_s):
    out = [{'type': 'startup', 'tag_id': tag_id, 'fs': 3840.0, 'nfft': 116,
            'n_w': 58, 'n_ol': 29, 'tip': 2.0, 'K': 5,
            'confidence_ratio': 1.3, 'dump_spectrogram': False}]
    for c in range(1, cycles + 1):
        out.append({'type': 'detection', 'cycle': c, 'freq_hz': 10.0,
                    'snr_db': 12.0, 'score_ratio': 3.0, 'noise_psd': 1e-6,
                    'proc_ms': 100.0, 'confidence': '', 'hyp_label': 'A',
                    'detection_status': 1})
        out.append({'type': 'timing', 'cycle': c, 'stft_ms': 10.0,
                    'fold_ms': 50.0, 'total_ms': 100.0})
    out.append({'type': 'session_end', 'cycles': cycles,
                'detections': cycles, 'elapsed_s': elapsed_s})
    return out


def _decimator_log(path: Path, rate_warnings=0, dropped=0):
    lines = []
    for i in range(1, rate_warnings + 1):
        lines.append(f'airspyhf_decimator: bad incoming measured sample '
                     f'rate=767000 expected=768000 error_ppm=1302 '
                     f'warnings={i}')
    lines.append(f'airspyhf_decimator: perf zmq_Bps=1 zmq_complex_sps=768000 '
                 f'malformed=0 dropped={dropped} out_of_order=0 '
                 f'out_sps=3840 cpu_duty_pct=20 queue_depth=1 queue_drops=0')
    path.write_text('\n'.join(lines) + '\n')


def _dump(dir_: Path, tag, cycle):
    # Large enough that omitting the IQ sidecar changes the reported MB.
    prefix = dir_ / f'tag{tag}_cycle_{cycle:04d}'
    np.save(f'{prefix}_power.npy', np.ones((116, 2000), dtype=np.float32))
    np.save(f'{prefix}_iq.npy', np.ones(200_000, dtype=np.complex64))
    (Path(f'{prefix}_meta.json')).write_text(json.dumps(
        {'fs': 3840.0, 'nfft': 116, 'n_w': 58, 'n_ol': 29,
         'detections': []}))
    return prefix


# ---------------------------------------------------------------------------

class TestDecimatorParsing:
    def test_rate_warning_count_uses_cumulative_counter(self, tmp_path):
        log = tmp_path / 'airspyhf_decimator.log'
        # Decimator prints the first 10 and then every 100th warning.
        log.write_text(
            'airspyhf_decimator: bad incoming measured sample rate=1 '
            'expected=2 error_ppm=5 warnings=10\n'
            'airspyhf_decimator: bad incoming measured sample rate=1 '
            'expected=2 error_ppm=5 warnings=300\n')
        d = parse_decimator_log(str(log))
        assert d['rate_warning_count'] == 300
        assert len(d['rate_warnings']) == 2

    def test_drop_totals_include_events_after_last_perf(self, tmp_path):
        log = tmp_path / 'airspyhf_decimator.log'
        log.write_text(
            'airspyhf_decimator: perf zmq_Bps=1 zmq_complex_sps=768000 '
            'malformed=0 dropped=2 out_of_order=0 out_sps=3840 '
            'cpu_duty_pct=20 queue_depth=1 queue_drops=1\n'
            'airspyhf_decimator: dropped 3 packets total_dropped=5\n'
            'airspyhf_decimator: queue full total_queue_drops=4\n')
        d = parse_decimator_log(str(log))
        assert d['dropped'] == 5
        assert d['queue_drops'] == 4

    def test_drop_totals_without_any_perf_line(self, tmp_path):
        log = tmp_path / 'airspyhf_decimator.log'
        log.write_text('airspyhf_decimator: dropped 1 packet total_dropped=1\n')
        d = parse_decimator_log(str(log))
        assert d['dropped'] == 1


class TestPlainSession:
    def test_transient_rate_warnings_are_healthy(self, tmp_path):
        _jsonl(tmp_path / 'detector_3.jsonl', _detector_entries(3, 2, 24.0))
        _decimator_log(tmp_path / 'airspyhf_decimator.log',
                       rate_warnings=RATE_WARNING_TRANSIENT_MAX)
        md = generate_report(str(tmp_path))
        assert '**Status:** Healthy' in md
        assert 'startup transient' in md
        assert 'sustained' not in md

    def test_sustained_rate_warnings_flagged_everywhere(self, tmp_path):
        _jsonl(tmp_path / 'detector_3.jsonl', _detector_entries(3, 2, 24.0))
        _decimator_log(tmp_path / 'airspyhf_decimator.log',
                       rate_warnings=RATE_WARNING_TRANSIENT_MAX + 1)
        md = generate_report(str(tmp_path))
        assert '**Status:** WARNING' in md
        assert 'sustained sample-rate mismatch' in md
        # The observations section must not contradict the health status.
        assert 'self-corrects' not in md

    def test_duration_is_max_of_concurrent_detectors(self, tmp_path):
        _jsonl(tmp_path / 'detector_3.jsonl', _detector_entries(3, 2, 24.0))
        _jsonl(tmp_path / 'detector_5.jsonl', _detector_entries(5, 2, 26.0))
        _decimator_log(tmp_path / 'airspyhf_decimator.log')
        md = generate_report(str(tmp_path))
        assert '| Session duration | 26 s |' in md

    def test_dump_size_includes_all_sidecars(self, tmp_path):
        _jsonl(tmp_path / 'detector_3.jsonl', _detector_entries(3, 1, 12.0))
        _decimator_log(tmp_path / 'airspyhf_decimator.log')
        prefix = _dump(tmp_path, 3, 1)
        expected = sum(Path(f'{prefix}{s}').stat().st_size
                       for s in ('_power.npy', '_iq.npy', '_meta.json'))
        md = generate_report(str(tmp_path))
        assert f'| Dump files | 1 cycles, {expected / 1048576:.1f} MB |' in md

    def test_truncated_jsonl_still_reports(self, tmp_path):
        p = tmp_path / 'detector_3.jsonl'
        _jsonl(p, _detector_entries(3, 2, 24.0))
        with open(p, 'a', encoding='utf-8') as f:
            f.write('{"type": "detection", "cyc')
        _decimator_log(tmp_path / 'airspyhf_decimator.log')
        md = generate_report(str(tmp_path))
        assert '| Detections | 2 |' in md

    def test_drops_without_perf_line_are_an_anomaly(self, tmp_path):
        _jsonl(tmp_path / 'detector_3.jsonl', _detector_entries(3, 1, 12.0))
        (tmp_path / 'airspyhf_decimator.log').write_text(
            'airspyhf_decimator: dropped 1 packet total_dropped=1\n')
        md = generate_report(str(tmp_path))
        assert 'Decimator: 1 dropped ZMQ packets' in md
        assert 'No anomalies detected' not in md


class TestRotationSession:
    @pytest.fixture
    def rotation_dir(self, tmp_path):
        for h, dropped, elapsed in ((0, 2, 12.0), (90, 3, 14.0)):
            hd = tmp_path / f'heading-{h:03d}'
            hd.mkdir()
            _jsonl(hd / 'detector_3.jsonl', _detector_entries(3, 1, elapsed))
            _decimator_log(hd / 'airspyhf_decimator.log', dropped=dropped)
            _dump(hd, 3, 1)
        (tmp_path / 'bearing_result.log').write_text(
            'tag_id,bearing_deg,r_squared,n_valid_slices,best_snr,'
            'latitude,longitude\n3,45.0,0.9,2,12.0,38.1,-122.2\n')
        return tmp_path

    def test_decimator_counters_summed_across_headings(self, rotation_dir):
        md = generate_report(str(rotation_dir))
        assert '5 ZMQ drops' in md
        assert 'Decimator: 5 dropped ZMQ packets' in md

    def test_duration_sums_headings(self, rotation_dir):
        md = generate_report(str(rotation_dir))
        assert '| Session duration | 26 s |' in md

    def test_per_heading_transients_are_not_sustained(self, tmp_path):
        # Each heading restarts its decimator: 6 + 6 warnings is two startup
        # transients, not a 12-warning sustained mismatch.
        for h in (0, 90):
            hd = tmp_path / f'heading-{h:03d}'
            hd.mkdir()
            _jsonl(hd / 'detector_3.jsonl', _detector_entries(3, 1, 12.0))
            _decimator_log(hd / 'airspyhf_decimator.log', rate_warnings=6)
        md = generate_report(str(tmp_path))
        assert '**Status:** Healthy' in md
        assert '- **Rate warnings:** 12 (startup transient)' in md
        assert 'sustained' not in md

    def test_spectrograms_labelled_per_heading_and_tag(self, rotation_dir):
        md = generate_report(str(rotation_dir))
        assert '| Headings | 2 |' in md
        assert '| Mode | rotation/bearing |' in md
        if (rotation_dir / 'spectrogram_h000_tag3_cycle_0001.png').exists():
            assert '![heading 000° tag 3 cycle 0001]' in md


class TestPersistentRotationSession:
    """One detector process across all headings: decimator log at the root,
    per-heading heading-NNN/ dirs, and a single process-wide SESSION_END that
    lands in whichever file was open at exit."""

    def test_session_end_totals_not_double_counted(self, tmp_path):
        n_headings = 4
        for i, h in enumerate(range(0, 360, 360 // n_headings), start=1):
            hd = tmp_path / f'heading-{h:03d}'
            hd.mkdir()
            entries = _detector_entries(3, 1, 12.0)
            entries = [e for e in entries if e['type'] != 'session_end']
            if i == 2:  # one real per-slice gap event on heading 090
                entries.insert(1, {'type': 'gap_event', 'kind': 'zerofill',
                                   'gap_ms': 5.0})
            if i == n_headings:  # last file gets the rotation-wide totals
                entries.append({'type': 'session_end', 'cycles': n_headings,
                                'detections': n_headings, 'elapsed_s': 60.0,
                                'gap_zerofill_count': 2, 'gap_reset_count': 1})
            _jsonl(hd / 'detector_3.jsonl', entries)
        # session.json marks the persistent layout for every SDR; no decimator
        # log here mirrors an AirSpy Mini collection
        (tmp_path / 'session.json').write_text('{"detection_mode": "python"}')
        (tmp_path / 'bearing_result.log').write_text(
            'tag_id,bearing_deg,r_squared,n_valid_slices,best_snr,'
            'latitude,longitude\n3,45.0,0.9,4,12.0,38.1,-122.2\n')
        md = generate_report(str(tmp_path))
        assert f'| Headings | {n_headings} |' in md
        # (N-1) + N would be 7; per-slice counting from records gives N
        assert f'| Detection cycles | {n_headings} |' in md
        assert f'| Detections | {n_headings} |' in md
        # rotation-wide fields from SESSION_END shown once at rotation level
        assert '| Session duration | 60 s |' in md
        assert '| Gaps (rotation) | 2 zero-filled, 1 resets |' in md
        # ...and NOT attributed to the heading whose file happened to hold them:
        # per-heading gap column comes from that heading's own GAP_EVENTs
        rows = {line.split('|')[1].strip(): line for line in md.splitlines()
                if line.startswith('| ') and '° |' in line}
        assert rows['090°'].rstrip().endswith('| 1 |')
        assert rows['270°'].rstrip().endswith('| 0 |')
