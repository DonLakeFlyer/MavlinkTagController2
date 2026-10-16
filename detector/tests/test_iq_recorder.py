"""IqRecorder: datagram payloads and index rows written for offline calibration fits."""

import json
import struct

import numpy as np

from iq_recorder import IqRecorder


def _datagram(sec, nsec, samples):
    return struct.pack('<II', sec, nsec) + samples.astype(np.complex64).tobytes()


def test_records_samples_and_index(tmp_path):
    recorder = IqRecorder(str(tmp_path), tag_id=3, fs=3840)
    first = np.arange(4) + 1j * np.arange(4)
    second = -np.arange(2) + 0.5j
    assert recorder.write(_datagram(10, 500, first))
    assert recorder.write(_datagram(11, 0, second))
    # On disk before close: the controller stops the recorder with SIGKILL
    assert (tmp_path / 'iq_tag3.c64').stat().st_size == recorder.sample_count * 8
    recorder.close()

    samples = np.fromfile(tmp_path / 'iq_tag3.c64', dtype=np.complex64)
    np.testing.assert_array_equal(samples, np.concatenate([first, second]).astype(np.complex64))

    rows = (tmp_path / 'iq_tag3_index.csv').read_text().splitlines()
    assert rows == ['sample_index,timestamp_ns,n_samples',
                    '0,10000000500,4',
                    '4,11000000000,2']

    meta = json.loads((tmp_path / 'iq_tag3.json').read_text())
    assert meta['fs'] == 3840
    assert meta['tag_id'] == 3


def test_rejects_datagram_without_samples(tmp_path):
    recorder = IqRecorder(str(tmp_path), tag_id=1, fs=3840)
    assert not recorder.write(struct.pack('<II', 1, 2))
    recorder.close()
    assert recorder.sample_count == 0
    assert (tmp_path / 'iq_tag1.c64').stat().st_size == 0


def test_rejects_datagram_with_partial_sample(tmp_path):
    recorder = IqRecorder(str(tmp_path), tag_id=1, fs=3840)
    assert not recorder.write(_datagram(1, 2, np.ones(4)) + b'\x00\x00\x00')
    recorder.close()
    assert recorder.sample_count == 0
    assert recorder.datagram_count == 0
    assert (tmp_path / 'iq_tag1.c64').stat().st_size == 0
    assert (tmp_path / 'iq_tag1_index.csv').read_text().splitlines() == ['sample_index,timestamp_ns,n_samples']
