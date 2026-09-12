# Migration from multi-repo (historical)

> **Archived.** Kept for anyone tracing history across the former repositories.
> The current layout is described in the root [README.md](../../README.md).

This repository consolidated what were previously three separate repositories:

| Component | Former repo | Now located at |
| --- | --- | --- |
| Controller | `MavlinkTagController2` | `controller/` |
| Decimator | `AirspyHFDecimate` | `decimator/` |
| ZeroMQ publisher | `airspyhf-zeromq` | `airspyhf_zeromq/` |
| Wire format | `TagTrackerWireFormat` (submodule) | `shared/tagtracker_wireformat/` |
| MAVLink headers | `c_library_v2` (submodule) | CPM package (auto-downloaded at configure time) |
| Tunnel protocol | `TagTrackerTunnelProtocol` (submodule) | CPM package (auto-downloaded at configure time) |

MAVLink headers and `TagTrackerTunnelProtocol` are both fetched automatically via
CPM at configure time — no submodules, no manual download. Each is pinned to a
commit by `GIT_TAG` in the top-level `CMakeLists.txt`; the tunnel protocol pin
must match the one in TagTracker's `custom/CMakeLists.txt`.
