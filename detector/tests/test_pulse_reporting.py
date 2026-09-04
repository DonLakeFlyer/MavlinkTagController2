import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pulse_detector import send_pulse_udp  # noqa: E402


class FakeSocket:
    def __init__(self, error=None):
        self.error = error

    def sendto(self, packet, destination):
        if self.error is not None:
            raise self.error
        return len(packet)


def send_test_pulse(pulse_socket):
    return send_pulse_udp(
        pulse_socket,
        ('127.0.0.1', 50000),
        tag_id=42,
        frequency_hz=146_170_000,
        start_time_seconds=12.5,
        predict_next_start_seconds=14.5,
        snr=18.25,
        stft_score=2.75,
        group_seq_counter=9,
        group_ind=2,
        group_snr=18.25,
        detection_status=1,
        confirmed_status=0,
        noise_psd=1.5e-10,
        collection_id=7,
        slice_id=3,
    )


def test_pulse_send_reports_success():
    assert send_test_pulse(FakeSocket())


def test_pulse_send_reports_socket_failure():
    assert not send_test_pulse(FakeSocket(OSError('send failed')))
