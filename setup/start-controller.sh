#!/bin/bash
set -euo pipefail

# ExecStart of mavlink-tag-controller.service (installed by full_setup.sh).

REPO_DIR="/home/pi/repos/MavlinkTagController2"
VENV_ACTIVATE="$REPO_DIR/.venv/bin/activate"
LOG="/home/pi/MavlinkTagController.log"
# One log per controller start, i.e. per boot: every battery swap is a boot.
KEEP_LOGS=20

# .1 is the previous run, .20 the oldest. Nothing is truncated, so a crash stays readable.
rm -f "$LOG.$KEEP_LOGS"
for ((i = KEEP_LOGS - 1; i >= 1; i--)); do
    if [ -f "$LOG.$i" ]; then
        mv -f "$LOG.$i" "$LOG.$((i + 1))"
    fi
done
if [ -f "$LOG" ]; then
    mv -f "$LOG" "$LOG.1"
fi
exec >>"$LOG" 2>&1

if [ ! -f "$VENV_ACTIVATE" ]; then
    echo "Virtual environment not found at $VENV_ACTIVATE"
    echo "Run $REPO_DIR/setup_venv.sh first."
    exit 1
fi

# shellcheck source=/dev/null
source "$VENV_ACTIVATE"

# Only use the router once full_setup.sh has installed it; checking is-active instead would race it at boot
if systemctl is-enabled --quiet mavlink-router 2>/dev/null; then
    CONNECTION_URL="udp://127.0.0.1:14540"
    # Diagnostic only: a router that is not "active" yet (e.g. "activating" while it retries the UART)
    # explains a controller stuck waiting for the autopilot heartbeat. Non-zero exit when not active.
    ROUTER_STATE="$(systemctl is-active mavlink-router 2>/dev/null || true)"
    echo "Connection: $CONNECTION_URL (mavlink-router: ${ROUTER_STATE:-unknown})"
else
    CONNECTION_URL="serial:///dev/serial0:921600"
    echo "Connection: $CONNECTION_URL"
fi
# exec: the controller becomes the service's main process, so systemd sees its exit status
exec "$REPO_DIR/build/controller/MavlinkTagController2" "$CONNECTION_URL"
