"""pulse_detector main(): a captured slice is analysed in the background while
the next slice is armed and recorded (#173). Runs the detector as a process
fed with timestamped noise over UDP, as the decimator would."""

import json
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from detector_protocol import (  # noqa: E402
    ComputeStage,
    MessageType,
    decode_compute_progress,
    decode_header,
    encode_arm,
)

DETECTOR = Path(__file__).resolve().parents[1] / 'pulse_detector.py'
FS = 3840
FRAME = 1023
TAG_ID = 7
COLLECTION = 11


def _free_udp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


class _IqFeeder(threading.Thread):
    """Timestamped noise frames at 4x real time, contiguous in stream time."""

    def __init__(self, port):
        super().__init__(daemon=True)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._dest = ('127.0.0.1', port)
        self._rng = np.random.default_rng(1)
        self._t_ns = 1_700_000_000 * 1_000_000_000
        self._halt = threading.Event()

    def run(self):
        while not self._halt.is_set():
            sec, nsec = divmod(self._t_ns, 1_000_000_000)
            iq = (0.01 * (self._rng.standard_normal(FRAME)
                          + 1j * self._rng.standard_normal(FRAME))).astype(np.complex64)
            self._sock.sendto(struct.pack('<II', sec, nsec) + iq.tobytes(), self._dest)
            self._t_ns += round(FRAME * 1e9 / FS)
            time.sleep(FRAME / FS / 4)

    def stop(self):
        self._halt.set()
        self.join(timeout=2.0)
        self._sock.close()


class _Reports:
    """Everything the detector sends to its --pulse-port, in arrival order."""

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 0))
        self.sock.settimeout(0.2)
        self.port = self.sock.getsockname()[1]
        self.messages = []   # (header, packet)

    def wait_for(self, message_type, slice_id=None, count=1, timeout=30.0):
        deadline = time.monotonic() + timeout
        while True:
            matches = [i for i, (h, _) in enumerate(self.messages)
                       if h.message_type == message_type
                       and (slice_id is None or h.slice_id == slice_id)]
            if len(matches) >= count:
                return matches[count - 1]
            if time.monotonic() > deadline:
                seen = [(h.message_type.name, h.slice_id) for h, _ in self.messages
                        if h.message_type != MessageType.HEARTBEAT]
                raise AssertionError(f'no {message_type.name} for slice {slice_id} '
                                     f'(x{count}); saw {seen}')
            try:
                packet = self.sock.recv(2048)
            except socket.timeout:
                continue
            self.messages.append((decode_header(packet), packet))


@pytest.fixture
def detector(tmp_path):
    reports = _Reports()
    iq_port, control_port = _free_udp_port(), _free_udp_port()
    log_file = open(tmp_path / 'detector.log', 'w')
    process = subprocess.Popen(
        [sys.executable, str(DETECTOR),
         '--tip', '0.5', '--k', '2', '--fs', str(FS),
         '--port', str(iq_port), '--pulse-port', str(reports.port),
         '--control-port', str(control_port), '--tag-id', str(TAG_ID),
         '--freq', '146000000', '--center-freq', '146.0',
         '--warmup-seconds', '0',
         # Noise only: a single null pass, held to ~2 s by the budget, so the
         # analysis reliably outlasts an ARM round trip.
         '--null-permutations', '100000', '--null-time-budget', '2',
         '--log-dir', str(tmp_path / 'logs')],
        stdout=log_file, stderr=subprocess.STDOUT)
    feeder = _IqFeeder(iq_port)
    feeder.start()
    control = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def arm(slice_id, heading_deg):
        control.sendto(encode_arm(COLLECTION, slice_id, TAG_ID, heading_deg),
                       ('127.0.0.1', control_port))

    try:
        yield reports, arm, tmp_path / 'logs'
    finally:
        process.terminate()
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        feeder.stop()
        control.close()
        reports.sock.close()
        log_file.close()
    assert process.returncode == 0, (tmp_path / 'detector.log').read_text()[-4000:]


def test_next_slice_is_armed_while_previous_is_analysed(detector):
    reports, arm, log_dir = detector
    reports.wait_for(MessageType.READY, timeout=60.0)

    arm(1, 0.0)
    reports.wait_for(MessageType.ARMED, slice_id=1)
    captured_1 = reports.wait_for(MessageType.SLICE_CAPTURED, slice_id=1)
    arm(2, 90.0)
    armed_2 = reports.wait_for(MessageType.ARMED, slice_id=2)
    complete_1 = reports.wait_for(MessageType.CYCLE_COMPLETE, slice_id=1)
    assert captured_1 < armed_2 < complete_1

    # The analysis reported its progress from the compute thread, null included.
    stages = [decode_compute_progress(packet)[1:]
              for header, packet in reports.messages[captured_1:complete_1]
              if header.message_type == MessageType.COMPUTE_PROGRESS and header.slice_id == 1]
    assert stages and stages[0][0] == ComputeStage.SPECTROGRAM
    assert any(stage == ComputeStage.NULL and total == 100000 for stage, _, total in stages)

    reports.wait_for(MessageType.SLICE_CAPTURED, slice_id=2)
    reports.wait_for(MessageType.CYCLE_COMPLETE, slice_id=2)

    # Each status goes out twice (the second copy 1 s later); a re-ARM after a
    # lost status replays both once more, without recollecting.
    reports.wait_for(MessageType.SLICE_CAPTURED, slice_id=2, count=2, timeout=5.0)
    reports.wait_for(MessageType.CYCLE_COMPLETE, slice_id=2, count=2, timeout=5.0)
    arm(2, 90.0)
    reports.wait_for(MessageType.SLICE_CAPTURED, slice_id=2, count=3, timeout=5.0)
    reports.wait_for(MessageType.CYCLE_COMPLETE, slice_id=2, count=3, timeout=5.0)
    assert not [h for h, _ in reports.messages if h.message_type == MessageType.ARMED
                and h.slice_id == 2][1:]

    # Each slice's analysis records land in its own heading's log, even though
    # the next heading's file was already open for capture.
    def no_detection_slices(heading_dir):
        lines = (log_dir / heading_dir / f'detector_{TAG_ID}.jsonl').read_text().splitlines()
        return [json.loads(line)['cycle'] for line in lines
                if json.loads(line)['type'] == 'no_detection']

    assert no_detection_slices('heading-000') == [1]
    assert no_detection_slices('heading-090') == [2]
