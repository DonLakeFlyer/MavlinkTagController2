# airspyhf_decimator

Subscribes to the `airspyhf_zeromq_rx` (or `iq_simulator.py`) ZeroMQ PUB stream,
applies a frequency shift to move the Airspy HF+ DC spur out of band, decimates
by 8×5×5 = 200 (768 kHz → 3840 Hz), and emits UDP datagrams of `--frame`
samples (shorter immediately before an upstream hole) to one or more pulse
detectors.

```
ZMQ PUB (768 kHz IQ)  →  airspyhf_decimator  →  UDP :10000, :10001 (3840 Hz IQ)
```

## Build

Built by the default `make` / `cmake --preset release`. Needs `libzmq`.

```bash
make decimator            # or: cmake --preset decimator && cmake --build --preset decimator
```

## Usage

```
./build/decimator/airspyhf_decimator [options]
```

| Option | Default | Description |
| --- | --- | --- |
| `--input-rate <Hz>` | `0` | Expected incoming sample rate; `0` auto-learns from the first packet |
| `--strict-input-rate` | off | Exit on sample-rate mismatch beyond `--rate-tol-ppm` |
| `--shift-khz <kHz>` | `10` | Frequency shift before decimation (`0` in simulator mode) |
| `--frame <samples>` | `1024` | Complex samples per UDP packet (timestamp + payload). The frame immediately before an upstream hole is flushed short so the next timestamp can jump past the hole; consumers must size the payload from the datagram length |
| `--zmq-endpoint <uri>` | `tcp://127.0.0.1:5555` | ZeroMQ SUB endpoint |
| `--rate-tol-ppm <ppm>` | `5000` | Allowed sample-rate error before warning |
| `--ip <addr>` | `127.0.0.1` | Destination IPv4 address |
| `--ports <p0,p1,…>` | `10000,10001` | UDP destination ports, one per detector |

The controller launches it as
`airspyhf_decimator --input-rate 768000 --shift-khz 10 --ports 10000,10001`
and redirects stderr to `airspyhf_decimator.log` in the session log directory.

## Inputs / outputs

| | Format |
| --- | --- |
| Input | ZeroMQ PUB messages with the header in [shared/README.md](../shared/README.md#zeromq-iq-packet-format) |
| Output | UDP datagram of `complex64` samples. The first sample is a timestamp header, not IQ: its real lane holds `uint32` seconds and its imaginary lane `uint32` nanoseconds, bit-cast into the two `float32`s (`TimestampEncoder::headerForSample`; decoded by `pulse_detector.py::decode_timestamp`) |
| Diagnostics (stderr, 1 Hz) | `perf zmq_Bps= zmq_complex_sps= in_sps= out_sps= frames_per_s= cpu_duty_pct= buffer_samples= zmq_packets= malformed= dropped= out_of_order= queue_depth= queue_drops=` and `zmq_timestamp_rate_sps= first_ts_us= last_ts_us=` |

`malformed`, `dropped`, `out_of_order` and `queue_drops` are the continuity
counters `analyzer/post_flight_analysis.py` reads.

## Key files

| File | Role |
| --- | --- |
| `src/main.cpp` | CLI, ZMQ receive thread, FIR stages, frequency shifter, UDP sender, perf counters |
| `tests/test_main.cpp` | Unit tests (`airspyhf_decimator_tests`) |

## Tests

The component build preset builds only `airspyhf_decimator`; build the test
target first:

```bash
cmake --preset debug && cmake --build --preset debug
ctest --test-dir build -R airspyhf_decimator
```

See [decimator/tests/README.md](tests/README.md) and [TESTING.md](../TESTING.md).

## Further reading

- [docs/design/FREQUENCY_RANGE.md](../docs/design/FREQUENCY_RANGE.md) — why the 10 kHz shift, resulting bandwidth and bin resolution
- [docs/design/SYSTEM_OVERVIEW.md](../docs/design/SYSTEM_OVERVIEW.md) — where the decimator sits in the pipeline
