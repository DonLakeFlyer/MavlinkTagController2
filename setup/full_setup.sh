#!/bin/bash
set -e

# wget https://raw.githubusercontent.com/DonLakeFlyer/MavlinkTagController2/main/setup/full_setup.sh

echo "*** Install tools"
sudo apt install build-essential git cmake libboost-all-dev libairspyhf-dev libzmq3-dev libusb-1.0-0-dev pkg-config python3 python3-venv -y
git config --global pull.rebase false

echo "*** Create repos directory"
cd ~
if [ ! -d repos ]; then
    mkdir repos
fi
cd ~/repos

echo "*** Clone and build MavlinkTagController2 (controller + decimator + airspyhf_zeromq)"
cd ~/repos
if [ ! -d MavlinkTagController2 ]; then
    git clone https://github.com/DonLakeFlyer/MavlinkTagController2.git
fi
cd ~/repos/MavlinkTagController2
git pull origin main

echo "*** Build all components (controller, decimator, airspyhf_zeromq)"
rm -rf build
make

echo "*** Set up Python virtual environment (detector + simulator)"
cd ~/repos/MavlinkTagController2
./setup_venv.sh

if command -v raspi-config >/dev/null 2>&1; then
    echo "*** Configure Raspberry Pi: UTC timezone, hardware serial without login shell"
    sudo timedatectl set-timezone UTC
    # raspi-config nonint: 0 = enable, 1 = disable
    sudo raspi-config nonint do_serial_hw 0
    sudo raspi-config nonint do_serial_cons 1

    echo "*** Install crontab entry to start controller at boot"
    CRON_LINE="@reboot /bin/bash $HOME/repos/MavlinkTagController2/setup/crontab-start-controller.sh >> $HOME/MavlinkTagController-boot.log 2>&1"
    # Replace any existing entry for this script so a re-run never yields two @reboot controllers
    (crontab -l 2>/dev/null | grep -Fv "crontab-start-controller.sh"; echo "$CRON_LINE") | crontab -

    echo "*** Enable VNC (wayvnc) for remote desktop; desktop autologin so the session exists headless"
    # Best-effort: no desktop (Lite image) or an older raspi-config must not fail the whole setup
    sudo raspi-config nonint do_boot_behaviour B4 || echo "*** WARNING: desktop autologin not available; skipping"
    sudo raspi-config nonint do_vnc 0 || echo "*** WARNING: VNC enable failed; skipping"
    sudo raspi-config nonint do_vnc_resolution 1920x1080 || echo "*** WARNING: VNC resolution not supported; skipping"

    echo "*** Setup complete. Reboot to apply serial port and VNC changes and start the controller."
fi
