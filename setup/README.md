# setup/ — Raspberry Pi install and boot

Scripts that turn a fresh Raspberry Pi OS image into a running tag-tracking
companion computer.

## Scripts

| Script | What it does |
| --- | --- |
| `install.sh` | Stable bootstrap. Installs git, clones or updates `~/repos/MavlinkTagController2`, then runs `full_setup.sh` **from the checkout** so the steps are never stale. Safe to re-run to update. |
| `full_setup.sh` | `apt install` build deps, `make` all components, `./setup_venv.sh`. On a Pi additionally: timezone → UTC, hardware serial enabled with no login shell, `@reboot` crontab entry, VNC (`wayvnc`) with desktop autologin at 1920×1080. Reboot afterwards. |
| `crontab-start-controller.sh` | The `@reboot` entry. Activates the venv, rotates `~/MavlinkTagController.log` (keeps the last 500 lines), then runs `MavlinkTagController2 serial:///dev/serial0:921600`. |

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
| Auto-start | `crontab -e` and add `@reboot /bin/bash /home/pi/repos/MavlinkTagController2/setup/crontab-start-controller.sh >> /home/pi/MavlinkTagController-boot.log 2>&1` |

## Remote desktop

Flash Raspberry Pi OS **with desktop** (Bookworm or later). Connect from a Mac
with the TigerVNC viewer to `raspberrypi.local:5900`, log in with the Pi user's
Linux credentials and accept the self-signed certificate on first connect.

## Check it is running

```bash
pgrep -af MavlinkTagController2
tail -f ~/MavlinkTagController.log
```

Session logs (detectors, decimator, rotations) are under `~/Logs/` — see
[controller/README.md](../controller/README.md#logs).
