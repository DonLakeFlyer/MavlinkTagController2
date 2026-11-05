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

    echo "*** Keep the system journal across reboots (kernel under-voltage events), capped at 100 MB"
    # Raspberry Pi OS ships 40-rpi-volatile-storage.conf (Storage=volatile); a later-sorting drop-in overrides it
    sudo mkdir -p /etc/systemd/journald.conf.d
    printf '[Journal]\nStorage=persistent\nSystemMaxUse=100M\n' \
        | sudo tee /etc/systemd/journald.conf.d/50-persistent.conf >/dev/null
    sudo systemctl restart systemd-journald
    # Move this boot's RAM-only entries to disk so they survive the reboot after setup
    sudo journalctl --flush
    # The distro's tmpfiles rules for /var/log/journal only adjust existing paths (z/a+), so run them after
    # journald has created the directories; this applies the group, mode and ACLs non-root users need to read it
    sudo systemd-tmpfiles --create --prefix /var/log/journal

    echo "*** Build and install mavlink-router (owns /dev/serial0; controller and WiFi GCS connect over UDP)"
    MAVLINK_ROUTER_DIR="$HOME/repos/mavlink-router"
    # Pinned past v4 for the <cstdint> fix newer GCC needs
    MAVLINK_ROUTER_SHA="2362c620f483cef1edd574fb962a373a288e4b9e"
    sudo apt install meson ninja-build -y
    if [ ! -d "$MAVLINK_ROUTER_DIR" ]; then
        git clone https://github.com/mavlink-router/mavlink-router.git "$MAVLINK_ROUTER_DIR"
    fi
    git -C "$MAVLINK_ROUTER_DIR" fetch origin
    git -C "$MAVLINK_ROUTER_DIR" checkout --detach "$MAVLINK_ROUTER_SHA"
    git -C "$MAVLINK_ROUTER_DIR" submodule update --init --recursive
    rm -rf "$MAVLINK_ROUTER_DIR/build"
    meson setup "$MAVLINK_ROUTER_DIR/build" "$MAVLINK_ROUTER_DIR" --buildtype=release -Dsystemdsystemunitdir=/etc/systemd/system
    ninja -C "$MAVLINK_ROUTER_DIR/build"
    sudo ninja -C "$MAVLINK_ROUTER_DIR/build" install
    sudo install -D -m 644 "$REPO_DIR/setup/mavlink-router.conf" /etc/mavlink-router/main.conf
    # Default restart limits give up after ~0.5 s of UART failures, leaving the controller with no link
    sudo mkdir -p /etc/systemd/system/mavlink-router.service.d
    printf '[Unit]\nStartLimitIntervalSec=0\n\n[Service]\nRestart=always\nRestartSec=1\n' \
        | sudo tee /etc/systemd/system/mavlink-router.service.d/restart.conf >/dev/null
    sudo systemctl daemon-reload
    sudo systemctl enable mavlink-router

    echo "*** Install mavlink-tag-controller systemd service to start the controller at boot"
    # Older setups started it from an @reboot crontab entry; remove that so the controller never starts twice.
    # grep exits 1 when nothing survives the filter (empty crontab); without "|| true" set -e
    # would kill the subshell.
    (crontab -l 2>/dev/null | grep -Fv "crontab-start-controller.sh" || true) | crontab -
    sudo install -m 644 "$REPO_DIR/setup/mavlink-tag-controller.service" /etc/systemd/system/mavlink-tag-controller.service
    sudo systemctl daemon-reload
    sudo systemctl enable mavlink-tag-controller

    echo "*** Enable VNC (wayvnc) for remote desktop; desktop autologin so the session exists headless"
    # Best-effort: no desktop (Lite image) or an older raspi-config must not fail the whole setup
    sudo raspi-config nonint do_boot_behaviour B4 || echo "*** WARNING: desktop autologin not available; skipping"
    sudo raspi-config nonint do_vnc 0 || echo "*** WARNING: VNC enable failed; skipping"
    sudo raspi-config nonint do_vnc_resolution 1920x1080 || echo "*** WARNING: VNC resolution not supported; skipping"

    echo "*** Setup complete. Reboot to apply serial port and VNC changes and start mavlink-router and the controller service."
fi
