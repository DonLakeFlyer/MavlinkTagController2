#!/bin/bash
set -euo pipefail

REPO_DIR="/home/pi/repos/MavlinkTagController2"
VENV_ACTIVATE="$REPO_DIR/.venv/bin/activate"

if [ ! -f "$VENV_ACTIVATE" ]; then
    echo "Virtual environment not found at $VENV_ACTIVATE" >&2
    echo "Run $REPO_DIR/setup_venv.sh first." >&2
    exit 1
fi

# shellcheck disable=SC1091
source "$VENV_ACTIVATE"

if [ -f /home/pi/MavlinkTagController.log ]; then
    mv -f /home/pi/MavlinkTagController.log /home/pi/MavlinkTagController.log.save
    tail -n 500 /home/pi/MavlinkTagController.log.save >> /home/pi/MavlinkTagController.log
    rm /home/pi/MavlinkTagController.log.save
fi
# Only use the router once full_setup.sh has installed it; checking is-active instead would race it at boot
if systemctl is-enabled --quiet mavlink-router 2>/dev/null; then
    CONNECTION_URL="udp://127.0.0.1:14540"
else
    CONNECTION_URL="serial:///dev/serial0:921600"
fi
echo "Connection: $CONNECTION_URL" >>/home/pi/MavlinkTagController.log
/home/pi/repos/MavlinkTagController2/build/controller/MavlinkTagController2 "$CONNECTION_URL" >>/home/pi/MavlinkTagController.log 2>&1
