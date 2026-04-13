# Wire-Format Tests

Unit tests for the `tagtracker_wireformat` header-only library (`shared/tagtracker_wireformat/zmq_iq_packet.h`), which defines the ZeroMQ IQ packet header shared by all three pipeline components.

## Run

```bash
cd build && ctest -R tagtracker_wireformat
```

## Tests

### `test_zmq_iq_packet.c`

| Test | Description |
|------|-------------|
| `test_valid_roundtrip` | Encode a header with known field values, decode it, and verify all fields survive the round-trip |
| `test_bad_magic` | Encode a header with an invalid magic number and verify `ttwf_validate_zmq_iq_frame` returns `TTWF_ZMQ_ERR_BAD_MAGIC` |
