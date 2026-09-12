# shared/

Header-only contracts shared by more than one component. Any change here is a
cross-component change: update every consumer and its tests in the same commit
(see [Wire-format contract](#wire-format-contract)).

## Contents

| File | Consumers | Purpose |
| --- | --- | --- |
| `tagtracker_wireformat/zmq_iq_packet.h` | `airspyhf_zeromq_rx`, `airspyhf_decimator`, `simulator/iq_simulator.py` | ZeroMQ IQ packet header (SDR → decimator) |
| `detector_protocol.h` | `controller/UDPPulseReceiver`, `detector/detector_protocol.py` | TTDP control/report protocol (controller ↔ Python detector) |
| `log_schema.py` | `detector/pulse_detector.py`, `analyzer/post_flight_analysis.py` | Structured `.jsonl` log entry types |
| `tests/test_zmq_iq_packet.c` | — | Wire-format unit tests (`tagtracker_wireformat_tests`) |

## ZeroMQ IQ packet format

Each PUB message is a fixed 40-byte header followed by interleaved `float32` I/Q
(8 bytes per complex sample). Packed little-endian, no padding.

| Field | Type | Description |
| --- | --- | --- |
| `magic` | `uint32` | `0x5a514941` — wire bytes `41 49 51 5a` (`"AIQZ"`; reads `"ZQIA"` as a big-endian word) |
| `version` | `uint16` | `TTWF_ZMQ_IQ_VERSION` = `1` |
| `header_size` | `uint16` | `40` |
| `sequence` | `uint64` | Increments per packet; skips ahead by one per packet's worth of samples the SDR driver dropped, so a gap always means lost IQ |
| `timestamp_us` | `uint64` | Monotonic clock microseconds |
| `sample_rate` | `uint32` | Sample rate in Hz |
| `sample_count` | `uint32` | Number of complex samples |
| `payload_bytes` | `uint32` | Byte count of IQ payload |
| `flags` | `uint32` | `0x1` = final chunk |

### Wire-format contract

`zmq_iq_packet.h` is the single source of truth. Any change to field layout,
sizes, endianness, magic, or version semantics is a breaking change and must:

- bump `TTWF_ZMQ_IQ_VERSION`;
- update publisher, decimator and simulator atomically in one commit/PR;
- update `tests/test_zmq_iq_packet.c` and `decimator/tests/test_main.cpp`;
- update the table above.

## TTDP detector protocol

`detector_protocol.h` (magic `0x50445454`, `"TTDP"`) carries the
controller ↔ detector handshake and pulse reports over UDP. 20-byte header
(`magic`, `message_type`, `payload_length`, `collection_id`, `slice_id`,
`tag_id`) followed by an optional payload:

| Message | Direction | Payload |
| --- | --- | --- |
| `Ready` | detector → controller | none |
| `Arm` | controller → detector | `ArmPayload` (`heading_deg` f32) |
| `Armed` | detector → controller | none |
| `Pulse`, `NoDetection` | detector → controller | `PulsePayload` (60 bytes) |
| `CycleComplete` | detector → controller | none |
| `Failed` | detector → controller | `FailedPayload` (`error_code` u32) |
| `Heartbeat` | detector → controller | none (1 Hz) |

The Python mirror is `detector/detector_protocol.py`; the C++ side is checked by
`controller/tests/test_detector_protocol.cpp` and the Python side by
`detector/tests/test_detector_protocol.py`. Field semantics
(`detection_status`, `confirmed_status`, `candidate_id`, `rate_state`) are
described in [docs/design/COLLECTION_FLOW.md](../docs/design/COLLECTION_FLOW.md).

## Tests

```bash
ctest --test-dir build -R tagtracker_wireformat
```

See [TESTING.md](../TESTING.md) for the full list.
