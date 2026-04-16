"""Tests for structured logging and --dump-spectrogram sidecar files.

Validates:
  1. StructuredLogger produces valid JSONL with correct schema.
  2. Key entry types (STARTUP, DETECTION, NO_DETECTION, SESSION_END) contain
     required fields.
  3. --dump-spectrogram requires --log-dir (argparse validation).
  4. Spectrogram dump produces expected sidecar files.
"""

import json
import os
import subprocess
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'shared'))
from log_schema import (
    StructuredLogger, read_jsonl,
    STARTUP, DETECTION, NO_DETECTION, SESSION_END,
)


# ---------------------------------------------------------------------------
# StructuredLogger unit tests
# ---------------------------------------------------------------------------

class TestStructuredLogger:
    def test_jsonl_written_with_type(self, tmp_path):
        path = str(tmp_path / 'test.jsonl')
        log = StructuredLogger(jsonl_path=path)
        log.emit(STARTUP, 'hello', tp=0.015, tip=2.0)
        log.close()

        entries = read_jsonl(path)
        assert len(entries) == 1
        assert entries[0]['type'] == STARTUP
        assert entries[0]['tp'] == 0.015
        assert entries[0]['tip'] == 2.0

    def test_no_file_when_path_is_none(self):
        log = StructuredLogger(jsonl_path=None)
        assert not log.active
        log.emit(STARTUP, 'test')
        log.close()

    def test_multiple_entries(self, tmp_path):
        path = str(tmp_path / 'multi.jsonl')
        log = StructuredLogger(jsonl_path=path)
        log.emit(STARTUP, 'start', k=5)
        log.emit(DETECTION, 'det', snr_db=12.3, freq_hz=100.0)
        log.emit(NO_DETECTION, 'nodet', cycle=2)
        log.emit(SESSION_END, 'end', cycles=2)
        log.close()

        entries = read_jsonl(path)
        assert len(entries) == 4
        types = [e['type'] for e in entries]
        assert types == [STARTUP, DETECTION, NO_DETECTION, SESSION_END]

    def test_emit_raw_not_in_jsonl(self, tmp_path):
        path = str(tmp_path / 'raw.jsonl')
        log = StructuredLogger(jsonl_path=path)
        log.emit(STARTUP, 'start', k=5)
        log.emit_raw('debug line')
        log.emit(SESSION_END, 'end', cycles=0)
        log.close()

        entries = read_jsonl(path)
        assert len(entries) == 2

    def test_utf8_encoding(self, tmp_path):
        path = str(tmp_path / 'utf8.jsonl')
        log = StructuredLogger(jsonl_path=path)
        log.emit(DETECTION, 'Δt=2.0s  freq=±100 Hz', delta='Δt')
        log.close()

        with open(path, 'rb') as f:
            raw = f.read()
        assert b'\xce\x94' in raw  # UTF-8 for Δ

    def test_numpy_serialization(self, tmp_path):
        path = str(tmp_path / 'numpy.jsonl')
        log = StructuredLogger(jsonl_path=path)
        log.emit(DETECTION, 'test',
                 val_int=np.int64(42),
                 val_float=np.float32(3.14),
                 val_arr=np.array([1, 2, 3]))
        log.close()

        entries = read_jsonl(path)
        assert entries[0]['val_int'] == 42
        assert abs(entries[0]['val_float'] - 3.14) < 0.01
        assert entries[0]['val_arr'] == [1, 2, 3]

    def test_write_failure_degrades_to_stdout_only(self, tmp_path, capsys):
        """ENOSPC on the .jsonl must not propagate out of emit()."""
        class Broken:
            def write(self, _):
                raise OSError(28, 'No space left on device')

            def flush(self):
                pass

            def close(self):
                pass

        log = StructuredLogger(jsonl_path=str(tmp_path / 'x.jsonl'))
        log._jsonl = Broken()
        log.emit(DETECTION, 'still printed', cycle=1)
        log.emit(DETECTION, 'and again', cycle=2)
        log.close()

        assert not log.active
        out, err = capsys.readouterr()
        assert 'still printed' in out and 'and again' in out
        assert err.count('WARNING') == 1


# ---------------------------------------------------------------------------
# read_jsonl robustness
# ---------------------------------------------------------------------------

class TestReadJsonl:
    def test_skips_truncated_last_line(self, tmp_path):
        """A detector killed mid-write leaves a partial trailing line."""
        path = tmp_path / 'trunc.jsonl'
        path.write_text(
            '{"type": "startup", "tp": 0.015}\n'
            '{"type": "detection", "cycle": 1}\n'
            '{"type": "detection", "cyc',
            encoding='utf-8')

        entries = read_jsonl(str(path))
        assert [e['type'] for e in entries] == ['startup', 'detection']


# ---------------------------------------------------------------------------
# JSONL schema field validation
# ---------------------------------------------------------------------------

class TestJsonlSchema:
    def _make_log(self, tmp_path):
        path = str(tmp_path / 'schema.jsonl')
        log = StructuredLogger(jsonl_path=path)
        return log, path

    def test_startup_fields(self, tmp_path):
        log, path = self._make_log(tmp_path)
        log.emit(STARTUP, 'start',
                 tp=0.015, tip=2.0, K=5, fs=3840.0,
                 n_w=58, nfft=116, port=10000)
        log.close()

        e = read_jsonl(path)[0]
        for field in ('tp', 'tip', 'K', 'fs', 'n_w', 'nfft', 'port'):
            assert field in e, f'Missing field: {field}'

    def test_detection_fields(self, tmp_path):
        log, path = self._make_log(tmp_path)
        log.emit(DETECTION, 'det',
                 cycle=1, freq_hz=50.0, snr_db=15.0,
                 score_ratio=2.1, noise_psd=1e-6,
                 confidence='', hyp_label='A',
                 detection_status=1)
        log.close()

        e = read_jsonl(path)[0]
        for field in ('cycle', 'freq_hz', 'snr_db', 'score_ratio',
                      'noise_psd', 'confidence', 'hyp_label',
                      'detection_status'):
            assert field in e, f'Missing field: {field}'

    def test_confidence_no_brackets(self, tmp_path):
        log, path = self._make_log(tmp_path)
        log.emit(DETECTION, 'det [LOW]', confidence='LOW')
        log.emit(DETECTION, 'det [DOMINANT_FOLD]', confidence='DOMINANT_FOLD')
        log.emit(DETECTION, 'det', confidence='')
        log.close()

        entries = read_jsonl(path)
        for e in entries:
            val = e['confidence']
            assert '[' not in val and ']' not in val, \
                f'Brackets in confidence field: {val!r}'

    def test_no_detection_fields(self, tmp_path):
        log, path = self._make_log(tmp_path)
        log.emit(NO_DETECTION, 'nodet',
                 cycle=1, proc_ms=50.0, had_gap=False,
                 best_candidate={'freq_hz': 10.0, 'snr_db': 5.0,
                                 'score_ratio': 0.3, 'noise_psd': 1e-5})
        log.close()

        e = read_jsonl(path)[0]
        assert 'best_candidate' in e
        assert e['best_candidate']['freq_hz'] == 10.0

    def test_session_end_fields(self, tmp_path):
        log, path = self._make_log(tmp_path)
        log.emit(SESSION_END, 'end',
                 cycles=10, elapsed_s=60.0, detections=3,
                 gap_zerofill_count=1, gap_reset_count=0)
        log.close()

        e = read_jsonl(path)[0]
        for field in ('cycles', 'elapsed_s', 'detections'):
            assert field in e, f'Missing field: {field}'


# ---------------------------------------------------------------------------
# --dump-spectrogram validation
# ---------------------------------------------------------------------------

class TestDumpSpectrogram:
    def test_dump_spectrogram_requires_log_dir(self):
        """--dump-spectrogram without --log-dir must fail."""
        result = subprocess.run(
            [sys.executable, '-m', 'pulse_detector',
             '--tip', '2.0', '--dump-spectrogram'],
            capture_output=True, text=True,
            cwd=os.path.join(os.path.dirname(__file__), '..'))
        assert result.returncode != 0
        assert 'log-dir' in result.stderr.lower() or 'log-dir' in result.stdout.lower()

    def test_dump_produces_sidecar_files(self, tmp_path):
        """write_cycle_dump writes power/iq/meta with the STFT geometry
        the analyzer relies on."""
        import pulse_detector as pd

        log_dir = str(tmp_path / 'dump')
        os.makedirs(log_dir)

        n_w = 58
        n_ol = 29
        fs = 3840.0
        W, _ = pd.build_weighting_matrix(n_w, fs)
        nfft = W.shape[1]
        iq = (np.random.randn(int(fs * 12))
              + 1j * np.random.randn(int(fs * 12))).astype(np.complex64)
        power, _ = pd.compute_stft_power(iq, n_w, n_ol, nfft, W=W)

        meta = {'cycle': 7, 'fs': fs, 'nfft': nfft, 'n_w': n_w,
                'n_ol': n_ol, 'detections': []}
        prefix = pd.write_cycle_dump(log_dir, 3, 7, power, iq, meta)

        assert prefix == os.path.join(log_dir, 'tag3_cycle_0007')
        assert np.load(f'{prefix}_power.npy').dtype == np.float32
        assert np.load(f'{prefix}_power.npy').shape == power.shape
        assert np.load(f'{prefix}_iq.npy').dtype == np.complex64
        with open(f'{prefix}_meta.json') as f:
            loaded = json.load(f)
        for key in ('fs', 'nfft', 'n_w', 'n_ol', 'detections'):
            assert loaded[key] == meta[key]
