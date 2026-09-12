# airspyhf_zeromq

Airspy HF+ SDR receiver that publishes raw IQ over a ZeroMQ PUB socket using the
shared wire format. Fork of the upstream `airspyhf` tools with a `-Z` ZeroMQ
output added to `airspyhf_rx`.

```
Airspy HF+ (USB)  →  airspyhf_zeromq_rx -Z  →  ZMQ PUB tcp://127.0.0.1:5555
```

## Build

Requires a **system** `libairspyhf` (`apt install libairspyhf-dev` /
`brew install airspyhf`) plus `libusb-1.0` and `libzmq`. The `libairspyhf/`
directory is the upstream library source kept for reference and packaging; the
build links the system library.

```bash
make airspyhf_zeromq      # or: cmake --preset airspyhf-zeromq && cmake --build --preset airspyhf-zeromq
```

## Usage

Flags the controller uses (`controller/CommandHandler.cpp`):

| Flag | Meaning |
| --- | --- |
| `-Z` | Publish IQ over ZeroMQ instead of writing a file |
| `-f <MHz>` | Tune frequency. The controller tunes **requested radio centre (`radio_center_frequency_hz`, normally the tag) + 0.010 MHz** so the DC spur lands 10 kHz off the channel centre; the decimator shifts it back |
| `-a 768000` | Sample rate (Hz) |
| `-g off` | AGC off |
| `-m on` | Manual gain on |
| `-r <file> -n <count>` | Raw capture mode (writes `complex_float32` to a file; used for offline replay with `analyzer/iq_replay.py`) |

Example (what the controller runs for a 146.611 MHz tag):

```bash
./build/airspyhf_zeromq/tools/src/airspyhf_zeromq_rx -Z -f 146.621 -a 768000 -g off -m on
```

## Inputs / outputs

| | Format |
| --- | --- |
| Output | ZeroMQ PUB, header per [shared/README.md](../shared/README.md#zeromq-iq-packet-format), `sequence` skips on USB-level sample loss |
| Log | stderr → `airspyhf_zeromq_rx.log` in the session directory (device info, gain settings, loss events) |

## Key files

| Path | Role |
| --- | --- |
| `tools/src/airspyhf_rx.c` | Receiver tool with the `-Z` ZeroMQ publisher |
| `tools/src/CMakeLists.txt` | Builds `airspyhf_zeromq_rx` |
| `libairspyhf/` | Upstream libairspyhf source (not linked; system library is used) |
| `LICENSE.BSD`, `LICENSE.GPL-2.0` | Upstream licences |

## Tests

Hardware integration tests; they exit 77 (skipped) when no Airspy HF+ is attached.
The component build preset builds only `airspyhf_zeromq_rx`; build the test
targets first:

```bash
cmake --preset debug && cmake --build --preset debug
ctest --test-dir build -R zmq_
```

See [airspyhf_zeromq/tests/README.md](tests/README.md) and [TESTING.md](../TESTING.md).
