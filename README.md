# MavlinkTagController2

[![CI](https://github.com/DonLakeFlyer/MavlinkTagController2/actions/workflows/ci.yml/badge.svg)](https://github.com/DonLakeFlyer/MavlinkTagController2/actions/workflows/ci.yml)

Monorepo for the UAV radio-tag tracking signal pipeline. Contains three co-deployed components and a shared wire-format header.

## Architecture

```
Airspy HF+ SDR
     │
     ▼
airspyhf_zeromq_rx        ─── publishes IQ over ZeroMQ PUB
     │
     ▼
airspyhf_decimator        ─── subscribes ZMQ, decimates (8×5×5), emits UDP
     │
     ▼
MavlinkTagController2     ─── MAVLink controller, spawns detectors, relays pulses
```

All three share a single packet header defined in [`shared/tagtracker_wireformat/zmq_iq_packet.h`](shared/tagtracker_wireformat/zmq_iq_packet.h).

## Repository layout

```
shared/                        Shared wire-format header (header-only library)
  tagtracker_wireformat/
    zmq_iq_packet.h            ZeroMQ IQ packet header definition
  test_zmq_iq_packet.c         Wire-format unit tests

controller/                    MAVLink tag controller
  CMakeLists.txt
  main.cpp, *.cpp, *.h

decimator/                     ZeroMQ → UDP decimator (was AirspyHFDecimate)
  CMakeLists.txt
  src/main.cpp
  tests/test_main.cpp

airspyhf_zeromq/               Airspy HF+ ZeroMQ publisher (was airspyhf-zeromq)
  CMakeLists.txt
  libairspyhf/                 Vendored libairspyhf source
  tools/                       airspyhf_zeromq_rx source
  tests/                       Integration tests (require hardware)

libs/                          Third-party dependencies
  uavrt_interfaces/            TunnelProtocol.h (git submodule)

cmake/                         Build utilities
  CPM.cmake                    CPM.cmake package manager (downloads mavlink at configure time)

setup/                         Raspberry Pi setup and boot scripts
```

## Build prerequisites

Top-level CMake now checks these at configure time:

- Boost
- Threads (pthreads)
- pkg-config
- libzmq
- libusb-1.0
- libairspyhf (system-installed)

Install on Ubuntu/Debian:

```bash
sudo apt install build-essential cmake pkg-config libboost-all-dev libzmq3-dev libusb-1.0-0-dev libairspyhf-dev
```

Install on macOS (Homebrew):

```bash
brew install cmake pkg-config boost zeromq libusb airspyhf
```

## Quick start

### Build controller + decimator (default)

```bash
make
```

### Build individual components

```bash
make controller
make decimator
make airspyhf_zeromq
```

### Run tests

```bash
make test
```

### Raw CMake

```bash
cmake -S . -B build \
    -DBUILD_TESTING=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

### CMake options

| Option | Default | Description |
| --- | --- | --- |
| `BUILD_TESTING` | `OFF` | Build and register tests |

## Component details

### Controller (MavlinkTagController2)

The MAVLink controller receives tag definitions over a MAVLink tunnel, writes per-tag detector configurations, spawns `uavrt_detection` processes, and relays pulse detections back over MAVLink.

**Dependencies:** libboost (system, filesystem)

### Decimator (airspyhf_decimator)

Consumes complex IQ samples from the `airspyhf_zeromq_rx` ZeroMQ publisher, performs three-stage FIR decimation (8×5×5 = 200× total), frequency-shifts to dodge the HF DC spur, and emits reduced-rate UDP packets for `uavrt_detection`.

**Dependencies:** libzmq

See [decimator usage details](#decimator-usage) below.

### Airspy HF+ ZeroMQ publisher (airspyhf_zeromq_rx)

Streams Airspy HF+ SDR IQ data over ZeroMQ PUB. Each message contains a fixed 40-byte header followed by interleaved float32 IQ payload.

**Dependencies:** system `libairspyhf`, libusb-1.0, libzmq

## Decimator usage

```
./build/airspyhf_decimator [options]
```

| Option | Default | Description |
| --- | --- | --- |
| `--input-rate <Hz>` | `0` | Expected incoming sample rate; `0` auto-learns from first packet |
| `--strict-input-rate` | off | Exit on sample-rate mismatch beyond `--rate-tol-ppm` |
| `--shift-khz <kHz>` | `10` | Frequency shift before decimation |
| `--frame <samples>` | `1024` | Total complex samples per UDP packet (timestamp + payload) |
| `--zmq-endpoint <uri>` | `tcp://127.0.0.1:5555` | ZeroMQ SUB endpoint |
| `--rate-tol-ppm <ppm>` | `5000` | Allowed sample-rate error before warning |
| `--ip <addr>` | `127.0.0.1` | Destination IPv4 address |
| `--ports <p0,p1>` | `10000,10001` | Comma-separated UDP destination ports |

## ZeroMQ packet format

Each PUB message is a fixed header + IQ payload:

| Field | Type | Description |
| --- | --- | --- |
| `magic` | `uint32` | `0x5a514941` |
| `version` | `uint16` | `1` |
| `header_size` | `uint16` | `40` |
| `sequence` | `uint64` | Monotonically increasing per packet |
| `timestamp_us` | `uint64` | Monotonic clock microseconds |
| `sample_rate` | `uint32` | Sample rate in Hz |
| `sample_count` | `uint32` | Number of complex samples |
| `payload_bytes` | `uint32` | Byte count of IQ payload |
| `flags` | `uint32` | `0x1` = final chunk |

Wire header is packed little-endian, 40 bytes, no padding.

## Linux / Raspberry Pi setup

```bash
cd ~/Downloads
wget https://raw.githubusercontent.com/DonLakeFlyer/MavlinkTagController2/main/setup/full_setup.sh
sh full_setup.sh
```

### Timezone and serial port

Set rPi timezone to UTC:
* `sudo raspi-config` → Localization → Timezone → None of the above → UTC

Enable serial port:
* `sudo raspi-config` → Interface Options → Serial Port → No login shell → Yes hardware enabled

### Pixhawk serial setup

* `MAV_1_CONFIG`: TELEM2
* `MAV_1_MODE`: Onboard
* `MAV_1_FORWARD`: On
* `SER_TEL2_BAUD`: 921600 8N1
* Reboot Pixhawk

### Auto-start at boot

```bash
crontab -e
# Add: @reboot /bin/bash /home/pi/repos/MavlinkTagController2/setup/crontab-start-controller.sh
```

### Check if running

```bash
ps -aux | grep Mav
```

## Testing with PX4 SITL

1. Follow [PX4 getting started](https://docs.px4.io/main/en/dev_setup/getting_started.html) for SITL.
2. Start the controller:
   ```bash
   ./build/MavlinkTagController2
   ```

## Migration from multi-repo

This repo consolidates what were previously three separate repositories:

| Component | Former repo | Now located at |
| --- | --- | --- |
| Controller | `MavlinkTagController2` | `controller/` |
| Decimator | `AirspyHFDecimate` | `decimator/` |
| ZeroMQ publisher | `airspyhf-zeromq` | `airspyhf_zeromq/` |
| Wire format | `TagTrackerWireFormat` (submodule) | `shared/tagtracker_wireformat/` |
| MAVLink headers | `c_library_v2` (submodule) | CPM package (auto-downloaded at configure time) |
| uavrt_interfaces | `uavrt_interfaces` (submodule) | `libs/uavrt_interfaces/` (git submodule) |

MAVLink headers are fetched automatically via CPM — no manual download needed. `uavrt_interfaces` remains a git submodule; clone with `--recurse-submodules` or run `git submodule update --init --recursive` after cloning.
