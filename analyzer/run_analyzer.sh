#!/usr/bin/env bash
#
# Run the full signal analysis pipeline with Airspy HF+:
#
#   airspyhf_zeromq_rx  →  [ZMQ]  →  decimator  →  [UDP]  →  signal_analyzer.py
#
# The radio is tuned 10 kHz above the expected frequency so the signal
# lands away from the DC spike in the decimated output.
#
# Usage:
#   ./run_analyzer.sh --expected-freq 148.515
#   ./run_analyzer.sh --expected-freq 148.515 --duration 15
#
# Stop:   Ctrl-C (kills all three processes)
#
# All extra arguments are forwarded to signal_analyzer.py.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$REPO_DIR/build"

RX="$BUILD_DIR/airspyhf_zeromq/tools/src/airspyhf_zeromq_rx"
DEC="$BUILD_DIR/decimator/airspyhf_decimator"
ANALYZER="$SCRIPT_DIR/signal_analyzer.py"

ZMQ_PORT=5555
UDP_PORT=10001        # Different from detector default to allow both
SHIFT_KHZ=10          # Tune offset to avoid DC spike

# --- Activate venv if present ---
if [ -f "$REPO_DIR/.venv/bin/activate" ]; then
    # shellcheck disable=SC1091
    source "$REPO_DIR/.venv/bin/activate"
fi

# --- Parse --expected-freq from args so we can compute radio freq ---
EXPECTED_FREQ=""
EXTRA_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --expected-freq)
            EXPECTED_FREQ="$2"
            EXTRA_ARGS+=("$1" "$2")
            shift 2
            ;;
        --port)
            UDP_PORT="$2"
            shift 2
            ;;
        *)
            EXTRA_ARGS+=("$1")
            shift
            ;;
    esac
done

if [ -z "$EXPECTED_FREQ" ]; then
    echo "Error: --expected-freq is required (e.g. --expected-freq 148.515)" >&2
    echo "Usage: $0 --expected-freq <MHz> [signal_analyzer.py options]" >&2
    exit 1
fi

# Radio tunes 10 kHz above expected so decimator --shift-khz 10 places
# the expected frequency at DC in the decimated output, away from the
# radio's own DC spike.
RADIO_FREQ=$(python3 -c "print(f'{${EXPECTED_FREQ} + ${SHIFT_KHZ}/1000.0:.6f}')")

# --- preflight checks ---
for bin in "$RX" "$DEC"; do
    if [ ! -x "$bin" ]; then
        echo "Error: $bin not found or not executable. Run: cmake --build build" >&2
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
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    for pid in "${PIDS[@]}"; do
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

echo "=== Signal Analyzer Pipeline ==="
echo "  Expected freq   ${EXPECTED_FREQ} MHz"
echo "  Radio tuned to  ${RADIO_FREQ} MHz (+${SHIFT_KHZ} kHz offset)"
echo "  ZMQ port        ${ZMQ_PORT}"
echo "  UDP port        ${UDP_PORT}"
echo ""

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
    --shift-khz "$SHIFT_KHZ" \
    2>&1 | grep -v '^.*: perf ' | sed 's/^/[dec] /' &
PIDS+=($!)
sleep 1

# --- 3. signal analyzer ---
echo "Starting signal analyzer (expected ${EXPECTED_FREQ} MHz)..."
echo ""
python3 -u "$ANALYZER" \
    --port "$UDP_PORT" \
    "${EXTRA_ARGS[@]}" \
    2>&1 | sed -u 's/^/[ana] /'
