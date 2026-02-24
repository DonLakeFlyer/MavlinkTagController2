#!/usr/bin/env bash
#
# Run the full Sirtrack VHF pulse detection pipeline at 146 MHz.
#
#   airspyhf_zeromq_rx  →  [ZMQ]  →  decimator  →  [UDP]  →  pulse_detector.py
#
# Radio is tuned to 146.010 MHz so the decimator's +10 kHz shift places
# 146.000 MHz at DC in the decimated output.
#
# Usage:  ./run_detector.sh
# Stop:   Ctrl-C (kills all three processes)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$REPO_DIR/build"

RX="$BUILD_DIR/airspyhf_zeromq/tools/src/airspyhf_zeromq_rx"
DEC="$BUILD_DIR/decimator/airspyhf_decimator"
DET="$SCRIPT_DIR/pulse_detector.py"

RADIO_FREQ=146.010    # MHz — offset +10 kHz so tag at 146.000 lands at DC
TAG_CENTER=146.000    # MHz — for display
ZMQ_PORT=5555
UDP_PORT=10000
TIP=2.0
TP=0.015

# --- preflight checks ---
for bin in "$RX" "$DEC"; do
    if [ ! -x "$bin" ]; then
        echo "Error: $bin not found or not executable. Run cmake --build build" >&2
        exit 1
    fi
done
if ! command -v python3 &>/dev/null; then
    echo "Error: python3 not found" >&2
    exit 1
fi

# --- cleanup on exit ---
PIDS=()
cleanup() {
    echo ""
    echo "Stopping pipeline..."
    # Send SIGTERM to all processes
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done

    # Wait with timeout, then escalate to SIGKILL if needed
    for pid in "${PIDS[@]}"; do
        # Check if process still running
        if kill -0 "$pid" 2>/dev/null; then
            sleep 0.5
            if kill -0 "$pid" 2>/dev/null; then
                echo "Process $pid did not terminate, sending SIGKILL..."
                kill -9 "$pid" 2>/dev/null || true
            fi
        fi
    done
    wait 2>/dev/null || true
    echo "Done."
}
trap cleanup EXIT INT TERM

# --- 1. airspy HF+ ZMQ reader ---
echo "Starting airspyhf_zeromq_rx at ${RADIO_FREQ} MHz (ZMQ port ${ZMQ_PORT})..."
"$RX" -f "$RADIO_FREQ" -P "$ZMQ_PORT" 2>&1 | sed 's/^/[rx]  /' &
PIDS+=($!)
sleep 1

# --- 2. decimator ---
echo "Starting decimator (ZMQ → UDP:${UDP_PORT})..."
"$DEC" \
    --zmq-endpoint "tcp://127.0.0.1:${ZMQ_PORT}" \
    --ports "$UDP_PORT" \
    --shift-khz 10 \
    2>&1 | grep -v '^.*: perf ' | sed 's/^/[dec] /' &
PIDS+=($!)
sleep 1

# --- 3. pulse detector ---
echo "Starting pulse detector (tp=${TP}s, tip=${TIP}s, center=${TAG_CENTER} MHz)..."
echo ""
python3 -u "$DET" \
    --tp "$TP" \
    --tip "$TIP" \
    --port "$UDP_PORT" \
    --center-freq "$TAG_CENTER" \
    "$@" \
    2>&1 | sed -u 's/^/[det] /'

