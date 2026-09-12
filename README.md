# MavlinkTagController2

[![CI](https://github.com/DonLakeFlyer/MavlinkTagController2/actions/workflows/ci.yml/badge.svg)](https://github.com/DonLakeFlyer/MavlinkTagController2/actions/workflows/ci.yml)

Companion-computer software for UAV radio-collar tracking. An Airspy HF+ SDR
streams IQ over ZeroMQ, a decimator narrows it to a 3.84 kHz channel per tag, a
Python K-fold detector finds the collar pulses, and a MAVLink controller turns
per-heading detections into a bearing for the ground station (TagTracker).

## Architecture

```
Airspy HF+ SDR
     │
     ▼
airspyhf_zeromq_rx        publishes 768 kHz IQ over ZeroMQ PUB      (or simulator/iq_simulator.py)
     │
     ▼
airspyhf_decimator        shifts 10 kHz, decimates 8×5×5, emits UDP IQ per detector
     │
     ▼
detector/pulse_detector.py   one per tag: STFT → K-fold → EVT threshold → TTDP pulse reports
     │
     ▼
MavlinkTagController2     supervises the above, fits antenna pattern → bearing, MAVLink tunnel to GCS
```

Wire contracts between these live in [shared/](shared/README.md). The full data
and control flow is described in
[docs/design/SYSTEM_OVERVIEW.md](docs/design/SYSTEM_OVERVIEW.md).

## Repository layout

| Directory | Contents | README |
| --- | --- | --- |
| `controller/` | MAVLink tag controller (C++) | [controller/README.md](controller/README.md) |
| `decimator/` | ZeroMQ → UDP decimator (C++) | [decimator/README.md](decimator/README.md) |
| `airspyhf_zeromq/` | Airspy HF+ → ZeroMQ publisher (C) | [airspyhf_zeromq/README.md](airspyhf_zeromq/README.md) |
| `detector/` | Python pulse detector | [detector/README.md](detector/README.md) |
| `simulator/` | Synthetic IQ source, drop-in SDR replacement | [simulator/README.md](simulator/README.md) |
| `analyzer/` | Offline log/IQ analysis and live signal characterisation tools | [analyzer/README.md](analyzer/README.md) |
| `shared/` | Wire-format and protocol headers, log schema | [shared/README.md](shared/README.md) |
| `setup/` | Raspberry Pi install and auto-start scripts | [setup/README.md](setup/README.md) |
| `docs/` | Design references, analysis records, proposals, archive | [docs/README.md](docs/README.md) |
| `cmake/` | `CPM.cmake` (fetches MAVLink headers and `TunnelProtocol.h` at configure time) | — |
| `Antennas/` | Reference photos of the RA-2AHS and RA-23K antennas | — |

## Build

### Prerequisites

Boost, Threads, pkg-config, libzmq, libusb-1.0, libairspyhf (system), Python 3 + venv.

```bash
# Ubuntu / Debian / Raspberry Pi OS
sudo apt install build-essential cmake pkg-config libboost-all-dev libzmq3-dev libusb-1.0-0-dev libairspyhf-dev python3 python3-venv
# macOS
brew install cmake pkg-config boost zeromq libusb airspyhf
```

### Quick start

```bash
make                 # release build of controller, decimator, airspyhf_zeromq
./setup_venv.sh      # Python venv for detector, simulator, analyzer
make test            # all ctest targets
```

Individual components: `make controller`, `make decimator`, `make airspyhf_zeromq`.

### CMake presets

| Preset | Description |
| --- | --- |
| `debug` / `release` / `relwithdebinfo` | Full build; `debug` and `release` enable `BUILD_TESTING` |
| `controller`, `decimator`, `airspyhf-zeromq` | Debug build filtered to one target (`-release` variants exist) |

```bash
cmake --preset debug && cmake --build --preset debug && ctest --preset debug
```

Dependencies fetched by CPM (cache `~/.cache/CPM`): MAVLink `c_library_v2` and
`DonLakeFlyer/TagTrackerTunnelProtocol`, both pinned by `GIT_TAG` in
`CMakeLists.txt`. The tunnel-protocol pin must match TagTracker's
`custom/CMakeLists.txt`; bump both together.

## Running

- On the aircraft: [setup/README.md](setup/README.md) (Raspberry Pi, auto-start on boot, Pixhawk serial settings).
- On a desk with SITL or no SDR: [TESTING.md](TESTING.md#running-against-px4-sitl) and `MavlinkTagController2 --simulator`.
- Per-component CLI reference: the component READMEs above.

## Testing

`make test` runs every ctest target; Python tests are run per directory with
pytest. [TESTING.md](TESTING.md) lists all of them.

## Documentation

[docs/README.md](docs/README.md) is the index. Layout:

| Directory | Question it answers |
| --- | --- |
| `docs/design/` | How does the current code work? |
| `docs/analysis/` | What did we measure? (dated, never rewritten) |
| `docs/proposals/` | What might we change? (each has a status) |
| `docs/archive/` | Superseded or historical material |

Component READMEs answer only "what is this and how do I run it".

## License

`airspyhf_zeromq/` carries the upstream BSD and GPL-2.0 licences
(`LICENSE.BSD`, `LICENSE.GPL-2.0`). The rest of the repository has no licence
file yet.
