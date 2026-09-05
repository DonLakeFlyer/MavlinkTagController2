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
    RATE_WARNING_TRANSIENT_MAX, correlate_holes_with_gaps, generate_report,
    parse_decimator_log, parse_rx_log,
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

    def test_holes_carry_sample_counts(self, tmp_path):
        log = tmp_path / 'airspyhf_decimator.log'
        log.write_text(
            'airspyhf_decimator: dropped 5 packet(s) before sequence=25 '
            'missing_input_samples=20480 skipped_output_samples=102 '
            'total_dropped=5\n'
            'airspyhf_decimator: dropped 2 packet(s) before sequence=90 '
            'missing_input_samples=8192 skipped_output_samples=41 '
            'total_dropped=7\n')
        d = parse_decimator_log(str(log))
        assert d['holes'] == [
            {'packets': 5, 'missing_input_samples': 20480,
             'skipped_output_samples': 102},
            {'packets': 2, 'missing_input_samples': 8192,
             'skipped_output_samples': 41},
        ]
        assert d['lost_input_samples'] == 28672


class TestRxLogParsing:
    def test_usb_drops_are_summed(self, tmp_path):
        log = tmp_path / 'airspyhf_zeromq_rx.log'
        log.write_text(
            'airspyhf_rx: USB dropped_samples=4096, skipping 1 sequence number(s)\n'
            'some unrelated line\n'
            'airspyhf_rx: USB dropped_samples=12288, skipping 3 sequence number(s)\n')
        d = parse_rx_log(str(log))
        assert d['usb_dropped_samples'] == 16384
        assert d['usb_drop_events'] == 2

    def test_missing_log_is_zero(self, tmp_path):
        d = parse_rx_log(str(tmp_path / 'nope.log'))
        assert d == {'usb_dropped_samples': 0, 'usb_drop_events': 0}


class TestHoleGapCorrelation:
    FS = 3840.0

    def test_every_hole_matched_by_a_detector_gap(self):
        holes = [{'skipped_output_samples': 102},   # 26.6 ms
                 {'skipped_output_samples': 41}]    # 10.7 ms
        gaps = [{'kind': 'zerofill', 'gap_ms': 10.9},
                {'kind': 'reset', 'gap_ms': 26.4}]
        unmatched = correlate_holes_with_gaps(holes, gaps, self.FS)
        assert unmatched == []

    def test_unmatched_hole_is_reported(self):
        holes = [{'skipped_output_samples': 102},
                 {'skipped_output_samples': 3840}]  # 1 s hole, nothing downstream
        gaps = [{'kind': 'reset', 'gap_ms': 26.6}]
        unmatched = correlate_holes_with_gaps(holes, gaps, self.FS)
        assert unmatched == [{'skipped_output_samples': 3840}]

    def test_each_gap_matches_at_most_one_hole(self):
        holes = [{'skipped_output_samples': 102}, {'skipped_output_samples': 102}]
        gaps = [{'kind': 'reset', 'gap_ms': 26.6}]
        assert len(correlate_holes_with_gaps(holes, gaps, self.FS)) == 1


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

    def test_lost_iq_summary_by_stage(self, tmp_path):
        entries = _detector_entries(3, 1, 12.0)
        entries[-1]['rx_ring_dropped'] = 2
        entries[-1]['retired_samples'] = 5000
        entries.insert(-1, {'type': 'gap_event', 'kind': 'reset', 'gap_ms': 26.6})
        _jsonl(tmp_path / 'detector_3.jsonl', entries)
        (tmp_path / 'airspyhf_zeromq_rx.log').write_text(
            'airspyhf_rx: USB dropped_samples=4096, skipping 1 sequence number(s)\n')
        (tmp_path / 'airspyhf_decimator.log').write_text(
            'airspyhf_decimator: perf zmq_Bps=1 zmq_complex_sps=768000 '
            'malformed=0 dropped=5 out_of_order=0 out_sps=3840 '
            'cpu_duty_pct=20 queue_depth=1 queue_drops=0\n'
            'airspyhf_decimator: dropped 5 packet(s) before sequence=25 '
            'missing_input_samples=20480 skipped_output_samples=102 '
            'total_dropped=5\n')
        md = generate_report(str(tmp_path))
        assert '**Lost IQ by stage:**' in md
        assert 'USB: 4096 samples (5.3 ms)' in md
        assert 'ZMQ/queue: 20480 input samples (26.7 ms) in 1 hole(s)' in md
        assert 'detector rx ring: 2 packet(s)' in md
        assert 'all 1 upstream hole(s) observed by detector' in md
        assert '**Status:** WARNING' in md

    def test_hole_not_seen_by_detector_is_flagged(self, tmp_path):
        _jsonl(tmp_path / 'detector_3.jsonl', _detector_entries(3, 1, 12.0))
        (tmp_path / 'airspyhf_decimator.log').write_text(
            'airspyhf_decimator: perf zmq_Bps=1 zmq_complex_sps=768000 '
            'malformed=0 dropped=5 out_of_order=0 out_sps=3840 '
            'cpu_duty_pct=20 queue_depth=1 queue_drops=0\n'
            'airspyhf_decimator: dropped 5 packet(s) before sequence=25 '
            'missing_input_samples=20480 skipped_output_samples=102 '
            'total_dropped=5\n')
        md = generate_report(str(tmp_path))
        assert '1 upstream hole(s) NOT observed by detector' in md
        assert 'continuity accounting' in md


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
