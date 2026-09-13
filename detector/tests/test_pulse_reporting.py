import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pulse_detector import send_pulse_udp, report_frequency_hz  # noqa: E402


def test_report_frequency_is_measured_offset_from_channel_center():
    # The controller clusters these to tell "heard" from scattered noise
    # hits, so the reported value must move with the detected bin.
    assert report_frequency_hz(146.0, 146_170_000, 650.0) == 146_000_650
    assert report_frequency_hz(146.0, 146_170_000, -1200.4) == 145_998_800
    # No channel centre: fall back to the configured tag frequency.
    assert report_frequency_hz(0.0, 146_170_000, 650.0) == 146_170_000
    assert report_frequency_hz(0.0, 0, 650.4) == 650


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
        rate_state=2,
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


def test_pulse_send_carries_candidate_id():
    from detector_protocol import decode_pulse_report

    class CapturingSocket(FakeSocket):
        def sendto(self, packet, destination):
            self.packet = packet
            return len(packet)

    sock = CapturingSocket()
    send_pulse_udp(
        sock, ('127.0.0.1', 50000), tag_id=42, frequency_hz=146_170_000,
        start_time_seconds=12.5, predict_next_start_seconds=14.5, snr=18.25,
        stft_score=0.0, group_seq_counter=9, rate_state=0, group_snr=1e-9,
        detection_status=2, confirmed_status=1, noise_psd=1.5e-10,
        collection_id=7, slice_id=3, candidate_id=3)
    report = decode_pulse_report(sock.packet)
    assert report.candidate_id == 3
    assert report.slice_id == 3

    send_test_pulse(sock)
    assert decode_pulse_report(sock.packet).candidate_id == 0
