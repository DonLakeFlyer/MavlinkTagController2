"""Typed local wire protocol between the controller and Python detector."""

from dataclasses import dataclass
from enum import IntEnum
import struct


MAGIC = 0x50445454  # "TTDP" on the wire
_HEADER = struct.Struct('<IHHIII')
_PULSE_PAYLOAD = struct.Struct('<IIHBB6d')
_ARM_PAYLOAD = struct.Struct('<f')
HEADER_SIZE = _HEADER.size
PULSE_REPORT_SIZE = HEADER_SIZE + _PULSE_PAYLOAD.size


class MessageType(IntEnum):
    READY = 1
    PULSE = 2
    NO_DETECTION = 3
    CYCLE_COMPLETE = 4
    FAILED = 5
    ARM = 6
    HEARTBEAT = 7
    ARMED = 8


class ErrorCode(IntEnum):
    PROCESS_FAILED = 1
    UNEXPECTED_EXCEPTION = 2
    REPORT_SEND_FAILED = 3
    LOG_OPEN_FAILED = 4


class ProtocolError(ValueError):
    pass


@dataclass(frozen=True)
class Header:
    message_type: MessageType
    payload_length: int
    collection_id: int
    slice_id: int
    tag_id: int


@dataclass(frozen=True)
class PulseReport:
    collection_id: int
    slice_id: int
    tag_id: int
    frequency_hz: int
    group_seq_counter: int
    group_ind: int
    detection_status: int
    confirmed_status: int
    start_time_seconds: float
    predict_next_start_seconds: float
    snr: float
    score_ratio: float
    group_snr: float
    noise_psd: float


def encode_header(message_type, payload_length, collection_id, slice_id, tag_id):
    return _HEADER.pack(
        MAGIC,
        int(message_type),
        payload_length,
        collection_id,
        slice_id,
        tag_id,
    )


def decode_header(packet):
    if len(packet) < HEADER_SIZE:
        raise ProtocolError('packet is shorter than the detector protocol header')

    magic, raw_type, payload_length, collection_id, slice_id, tag_id = _HEADER.unpack_from(packet)
    if magic != MAGIC:
        raise ProtocolError('invalid detector protocol magic')
    try:
        message_type = MessageType(raw_type)
    except ValueError as exc:
        raise ProtocolError(f'unknown detector message type: {raw_type}') from exc
    if len(packet) != HEADER_SIZE + payload_length:
        raise ProtocolError('detector packet length does not match its header')
    expected_payload_length = {
        MessageType.READY: 0,
        MessageType.PULSE: _PULSE_PAYLOAD.size,
        MessageType.NO_DETECTION: _PULSE_PAYLOAD.size,
        MessageType.CYCLE_COMPLETE: 0,
        MessageType.FAILED: 4,
        MessageType.ARM: _ARM_PAYLOAD.size,
        MessageType.HEARTBEAT: 0,
        MessageType.ARMED: 0,
    }[message_type]
    if payload_length != expected_payload_length:
        raise ProtocolError('detector message has the wrong payload size')

    return Header(message_type, payload_length, collection_id, slice_id, tag_id)


def encode_pulse_report(report, message_type=MessageType.PULSE):
    if message_type not in (MessageType.PULSE, MessageType.NO_DETECTION):
        raise ProtocolError('pulse report requires a pulse message type')
    payload = _PULSE_PAYLOAD.pack(
        report.frequency_hz,
        report.group_seq_counter,
        report.group_ind,
        report.detection_status,
        report.confirmed_status,
        report.start_time_seconds,
        report.predict_next_start_seconds,
        report.snr,
        report.score_ratio,
        report.group_snr,
        report.noise_psd,
    )
    return encode_header(
        message_type,
        len(payload),
        report.collection_id,
        report.slice_id,
        report.tag_id,
    ) + payload


def encode_arm(collection_id, slice_id, tag_id, heading_deg):
    payload = _ARM_PAYLOAD.pack(heading_deg)
    return encode_header(
        MessageType.ARM,
        len(payload),
        collection_id,
        slice_id,
        tag_id,
    ) + payload


def decode_arm(packet):
    header = decode_header(packet)
    if header.message_type != MessageType.ARM:
        raise ProtocolError('detector control channel accepts only ARM messages')
    (heading_deg,) = _ARM_PAYLOAD.unpack_from(packet, HEADER_SIZE)
    return header, heading_deg


def encode_failed_report(error_code, collection_id, slice_id, tag_id):
    payload = struct.pack('<I', error_code)
    return encode_header(
        MessageType.FAILED,
        len(payload),
        collection_id,
        slice_id,
        tag_id,
    ) + payload


def decode_failed_report(packet):
    header = decode_header(packet)
    if header.message_type != MessageType.FAILED:
        raise ProtocolError('packet is not a failed report')
    return header, struct.unpack_from('<I', packet, HEADER_SIZE)[0]


def decode_pulse_report(packet):
    header = decode_header(packet)
    if header.message_type not in (MessageType.PULSE, MessageType.NO_DETECTION):
        raise ProtocolError('packet is not a pulse report')
    if header.payload_length != _PULSE_PAYLOAD.size:
        raise ProtocolError('pulse payload has the wrong size')

    values = _PULSE_PAYLOAD.unpack_from(packet, HEADER_SIZE)
    return PulseReport(
        collection_id=header.collection_id,
        slice_id=header.slice_id,
        tag_id=header.tag_id,
        frequency_hz=values[0],
        group_seq_counter=values[1],
        group_ind=values[2],
        detection_status=values[3],
        confirmed_status=values[4],
        start_time_seconds=values[5],
        predict_next_start_seconds=values[6],
        snr=values[7],
        score_ratio=values[8],
        group_snr=values[9],
        noise_psd=values[10],
    )
