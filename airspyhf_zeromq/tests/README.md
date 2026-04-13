# Airspy HF+ ZeroMQ Publisher Tests

Integration tests for the `airspyhf_zeromq_rx` ZeroMQ publisher. These require a connected Airspy HF+ device — they return exit code 77 (skipped) if no hardware is present.

## Run

```bash
cd build && ctest -R zmq_
```

## Tests

### `test_zmq_timestamp.c`

Subscribes to the live ZeroMQ publisher, receives 8 packets, and verifies:

| Check | Description |
|-------|-------------|
| Sequence continuity | Sequence numbers are contiguous (no gaps) |
| Header validity | Magic, version, header size, and payload size fields are correct |
| Timestamp monotonicity | Timestamps strictly increase between consecutive packets |

### `test_zmq_loss_detection.c`

Subscribes with `ZMQ_CONFLATE=1` and deliberately sleeps 1.2 s between reads to force the publisher to drop packets, then verifies:

| Check | Description |
|-------|-------------|
| Packet loss detection | Sequence gaps are observed, confirming loss-detection logic works |
