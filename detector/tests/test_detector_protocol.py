import struct
import sys
from pathlib import Path

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from detector_protocol import (  # noqa: E402
    HEADER_SIZE,
    MAGIC,
    PULSE_REPORT_SIZE,
    ErrorCode,
    MessageType,
    ProtocolError,
    PulseReport,
    decode_arm,
    decode_header,
    decode_failed_report,
    decode_pulse_report,
    encode_arm,
    encode_failed_report,
    encode_header,
    encode_pulse_report,
)


def test_header_has_stable_little_endian_layout():
    encoded = encode_header(
        MessageType.HEARTBEAT,
        payload_length=0,
        collection_id=0x11223344,
        slice_id=0x55667788,
        tag_id=0x99AABBCC,
    )

    assert HEADER_SIZE == 20
    assert encoded == struct.pack(
        '<IHHIII',
        MAGIC,
        MessageType.HEARTBEAT,
        0,
        0x11223344,
        0x55667788,
        0x99AABBCC,
    )
    assert decode_header(encoded).message_type == MessageType.HEARTBEAT


def test_arm_round_trips_heading_with_stable_layout():
    encoded = encode_arm(collection_id=7, slice_id=3, tag_id=42, heading_deg=135.0)

    assert encoded == struct.pack('<IHHIIIf', MAGIC, MessageType.ARM, 4, 7, 3, 42, 135.0)
    header, heading_deg = decode_arm(encoded)
    assert header.message_type == MessageType.ARM
    assert (header.collection_id, header.slice_id, header.tag_id) == (7, 3, 42)
    assert heading_deg == 135.0


def test_arm_without_payload_is_rejected():
    with pytest.raises(ProtocolError):
        decode_header(encode_header(MessageType.ARM, 0, 7, 3, 42))
    with pytest.raises(ProtocolError):
        decode_arm(encode_header(MessageType.HEARTBEAT, 0, 7, 3, 42))


def test_pulse_report_round_trips_with_integer_status_fields():
    report = PulseReport(
        collection_id=7,
        slice_id=3,
        tag_id=42,
        frequency_hz=146_170_650,
        group_seq_counter=9,
        group_ind=2,
        detection_status=1,
        confirmed_status=0,
        start_time_seconds=12.5,
        predict_next_start_seconds=14.5,
        snr=18.25,
        score_ratio=2.75,
        group_snr=18.25,
        noise_psd=1.5e-10,
    )

    encoded = encode_pulse_report(report)

    assert PULSE_REPORT_SIZE == 80
    assert decode_pulse_report(encoded) == report


def test_no_detection_uses_distinct_message_type():
    report = PulseReport(
        collection_id=7,
        slice_id=3,
        tag_id=42,
        frequency_hz=146_170_000,
        group_seq_counter=9,
        group_ind=0,
        detection_status=3,
        confirmed_status=0,
        start_time_seconds=12.5,
        predict_next_start_seconds=0.0,
        snr=0.0,
        score_ratio=0.8,
        group_snr=0.0,
        noise_psd=1.5e-10,
    )

    encoded = encode_pulse_report(report, MessageType.NO_DETECTION)

    assert decode_header(encoded).message_type == MessageType.NO_DETECTION
    assert decode_pulse_report(encoded) == report


def test_error_codes_match_controller_header():
    # shared/detector_protocol.h ErrorCode enum; both sides must agree
    assert ErrorCode.PROCESS_FAILED == 1
    assert ErrorCode.UNEXPECTED_EXCEPTION == 2
    assert ErrorCode.REPORT_SEND_FAILED == 3
    assert ErrorCode.LOG_OPEN_FAILED == 4


def test_failed_report_round_trips_error_code():
    encoded = encode_failed_report(
        error_code=23,
        collection_id=7,
        slice_id=3,
        tag_id=42,
    )

    header, error_code = decode_failed_report(encoded)

    assert header.message_type == MessageType.FAILED
    assert header.collection_id == 7
    assert header.slice_id == 3
    assert header.tag_id == 42
    assert error_code == 23


def test_header_only_message_rejects_payload():
    packet = encode_header(MessageType.READY, 1, 7, 0, 42) + b'x'

    with pytest.raises(ProtocolError):
        decode_header(packet)


@pytest.mark.parametrize('packet', [
    b'',
    b'\x00' * (HEADER_SIZE - 1),
    struct.pack('<IHHIII', 0, MessageType.READY, 0, 1, 0, 2),
    struct.pack('<IHHIII', MAGIC, 0xFFFF, 0, 1, 0, 2),
    struct.pack('<IHHIII', MAGIC, MessageType.READY, 4, 1, 0, 2),
])
def test_malformed_headers_are_rejected(packet):
    with pytest.raises(ProtocolError):
        decode_header(packet)
