# setup/ — Raspberry Pi install and boot

Scripts that turn a fresh Raspberry Pi OS image into a running tag-tracking
companion computer.

## Scripts

| Script | What it does |
| --- | --- |
| `install.sh` | Stable bootstrap. Installs git, clones or updates `~/repos/MavlinkTagController2`, then runs `full_setup.sh` **from the checkout** so the steps are never stale. Safe to re-run to update. |
| `full_setup.sh` | `apt install` build deps, `make` all components, `./setup_venv.sh`. On a Pi additionally: timezone → UTC, hardware serial enabled with no login shell, mavlink-router built from a pinned commit in `~/repos/mavlink-router` and enabled as the `mavlink-router` systemd service (always restarted, 1 s apart, no start limit), the `mavlink-tag-controller` systemd service (removes the `@reboot` crontab entry of older setups), VNC (`wayvnc`) with desktop autologin at 1920×1080. Reboot afterwards. |
| `mavlink-router.conf` | Installed to `/etc/mavlink-router/main.conf`. Routes `/dev/serial0` at 921600 to the controller (UDP `127.0.0.1:14540`) and to a WiFi GCS (UDP server on port 14550). TCP server disabled. |
| `mavlink-tag-controller.service` | Installed to `/etc/systemd/system`. Runs `start-controller.sh` as `pi` at boot with `CAP_SYS_TIME`, so the controller can set the clock to GPS time. Not restarted if it exits: a restarted controller would have lost its uploaded tags. |
| `start-controller.sh` | The service's `ExecStart`. Rotates `~/MavlinkTagController.log` to `.1` … `.20` (one file per run, so per boot; nothing truncated), sends its own and the controller's output there, activates the venv, then `exec`s `MavlinkTagController2 udp://127.0.0.1:14540` when the `mavlink-router` service is enabled, otherwise `serial:///dev/serial0:921600`. |
| `controller-exited.sh` | The service's `ExecStopPost`. Appends systemd's exit result, code and status (e.g. `core-dump` / `SEGV`) to `~/MavlinkTagController.log`. |

## Fresh install

```bash
cd ~/Downloads
wget https://raw.githubusercontent.com/DonLakeFlyer/MavlinkTagController2/main/setup/install.sh
bash install.sh
```

Update an existing install:

```bash
bash ~/repos/MavlinkTagController2/setup/install.sh
```

Do not download and run `full_setup.sh` directly — always go through `install.sh`.

## Pixhawk serial setup

* `MAV_1_CONFIG`: TELEM2
* `MAV_1_MODE`: Onboard
* `MAV_1_FORWARD`: On
* `SER_TEL2_BAUD`: 921600 8N1
* Reboot the Pixhawk

## Manual equivalents

Everything `full_setup.sh` does on the Pi can be done by hand with `sudo raspi-config`:

| Setting | raspi-config path |
| --- | --- |
| Timezone UTC | Localization → Timezone → None of the above → UTC |
| Serial port | Interface Options → Serial Port → No login shell → Yes hardware enabled |
| VNC | Interface Options → VNC → Yes; System Options → Boot → Desktop; System Options → Auto Login → Desktop; Display Options → VNC Resolution |
| Auto-start | Not in raspi-config: `sudo install -m 644 ~/repos/MavlinkTagController2/setup/mavlink-tag-controller.service /etc/systemd/system/ && sudo systemctl daemon-reload && sudo systemctl enable mavlink-tag-controller` |

## Remote desktop

Flash Raspberry Pi OS **with desktop** (Bookworm or later). Connect from a Mac
with the TigerVNC viewer to `raspberrypi.local:5900`, log in with the Pi user's
Linux credentials and accept the self-signed certificate on first connect.

## Download logs over WiFi

Join the Pi to the network the laptop is on (for example a phone hotspot),
either when flashing with Raspberry Pi Imager or with `sudo nmcli device wifi connect <SSID> --ask`.

* **Autopilot logs:** QGC Comm Link, Type UDP:
  * Port: `14551`
  * Add Server: `raspberrypi.local:14550`
* **Pi logs:** `rsync -av pi@raspberrypi.local:Logs/ ./Logs/`.

QGC on the G16 and on the laptop both default to MAVLink system ID 255, and
mavlink-router remembers a sender's link until the Pi reboots. If the G16 has
connected since power-on, the laptop's requests will not reach the Pixhawk:
power-cycle the drone first, or set a different system ID in the laptop QGC
(Application Settings → MAVLink).

## Check it is running

```bash
systemctl status mavlink-router
systemctl status mavlink-tag-controller
tail -f ~/MavlinkTagController.log
grep CLOCK_SYNC ~/MavlinkTagController.log
```

Stop the service before running the controller by hand
(`sudo systemctl stop mavlink-tag-controller`), otherwise two controllers
share the router port. A hand-started controller has no `CAP_SYS_TIME` and
logs `CLOCK_SYNC disabled`.

## Clock

The Pi has no RTC, so it boots at its last saved time and falls behind by
however long it was powered off. Once the autopilot has a 3D GPS fix, the
controller sets the Pi clock to GPS time (once per boot, never during a
session) and announces it to the GCS as `Pi clock set to GPS time (+N s)`.
`CLOCK_SYNC` lines in the log say what it did or why it is waiting. Details:
[SYSTEM_OVERVIEW.md](../docs/design/SYSTEM_OVERVIEW.md#time-base).

Session logs (detectors, decimator, rotations) are under `~/Logs/` — see
[controller/README.md](../controller/README.md#logs).
