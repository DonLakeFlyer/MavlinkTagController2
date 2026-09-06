#!/bin/bash
set -euo pipefail

# Do not download and run this file directly: it may be stale. Use setup/install.sh, which
# clones/updates the repo and then runs this script from the checked-out tree.
#
# wget https://raw.githubusercontent.com/DonLakeFlyer/MavlinkTagController2/main/setup/install.sh
# bash install.sh

REPO_DIR="$HOME/repos/MavlinkTagController2"

echo "*** Install tools"
sudo apt update
sudo apt install build-essential git gh cmake libboost-all-dev libairspyhf-dev airspy airspyhf libzmq3-dev libusb-1.0-0-dev pkg-config python3 python3-venv -y
git config --global pull.rebase false

echo "*** Build all components (controller, decimator, airspyhf_zeromq)"
cd "$REPO_DIR"
rm -rf build
make

echo "*** Set up Python virtual environment (detector + simulator)"
cd "$REPO_DIR"
./setup_venv.sh

if command -v raspi-config >/dev/null 2>&1; then
    echo "*** Configure Raspberry Pi: UTC timezone, hardware serial without login shell"
    sudo timedatectl set-timezone UTC
    # raspi-config nonint: 0 = enable, 1 = disable
    sudo raspi-config nonint do_serial_hw 0
    sudo raspi-config nonint do_serial_cons 1

    echo "*** Install crontab entry to start controller at boot"
    CRON_LINE="@reboot /bin/bash \"$REPO_DIR/setup/crontab-start-controller.sh\" >> \"$HOME/MavlinkTagController-boot.log\" 2>&1"
    # Replace any existing entry for this script so a re-run never yields two @reboot controllers.
    # grep exits 1 when nothing survives the filter (empty crontab); without "|| true" set -e
    # would kill the subshell before the echo and install an empty crontab.
    (crontab -l 2>/dev/null | grep -Fv "crontab-start-controller.sh" || true; echo "$CRON_LINE") | crontab -
    echo "*** Installed crontab:"
    crontab -l

    echo "*** Enable VNC (wayvnc) for remote desktop; desktop autologin so the session exists headless"
    # Best-effort: no desktop (Lite image) or an older raspi-config must not fail the whole setup
    sudo raspi-config nonint do_boot_behaviour B4 || echo "*** WARNING: desktop autologin not available; skipping"
    sudo raspi-config nonint do_vnc 0 || echo "*** WARNING: VNC enable failed; skipping"
    sudo raspi-config nonint do_vnc_resolution 1920x1080 || echo "*** WARNING: VNC resolution not supported; skipping"

    echo "*** Setup complete. Reboot to apply serial port and VNC changes and start the controller."
fi
