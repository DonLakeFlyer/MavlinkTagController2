#!/usr/bin/env bash
#
# Run the simulated VHF pulse detection pipeline.
#
#   iq_simulator.py  →  [ZMQ]  →  decimator  →  [UDP]  →  pulse_detector.py
#
# This is the test/simulation equivalent of run_detector.sh — it replaces the
# real Airspy HF+ radio with a configurable IQ signal generator.
#
# The simulator publishes at 768 kHz on ZMQ PUB.  The decimator applies 200×
# decimation with a 0 kHz frequency shift in this simulated pipeline, since the
# simulator already generates tags at DC; the real Airspy HF+ pipeline uses +10 kHz.
#
# Usage:
#   ./run_sim_pipeline.sh [--preset strong|weak|noise-only|...]
#   ./run_sim_pipeline.sh --snr 12 --tp 0.015 --tip 2.0
#   ./run_sim_pipeline.sh --preset strong --duration 60
#
# All extra arguments are forwarded to iq_simulator.py.
# Stop: Ctrl-C (kills all three processes)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$REPO_DIR/build"
VENV_DIR="$REPO_DIR/.venv"

SIM="$SCRIPT_DIR/iq_simulator.py"
DEC="$BUILD_DIR/decimator/airspyhf_decimator"
DET="$REPO_DIR/detector/pulse_detector.py"

ZMQ_PORT=5555
UDP_PORT=10000
TAG_CENTER=146.000    # MHz — for display in detector output
TIP=2.0
TP=0.015

# --- preflight checks ---
if [ ! -f "$SIM" ]; then
    echo "Error: $SIM not found." >&2
    exit 1
fi
if [ ! -x "$DEC" ]; then
    echo "Error: $DEC not found or not executable. Run: cmake --build build" >&2
    exit 1
fi
if [ ! -f "$DET" ]; then
    echo "Error: $DET not found." >&2
    exit 1
fi

# Activate virtual environment
if [ -d "$VENV_DIR" ]; then
    # shellcheck disable=SC1091
    source "$VENV_DIR/bin/activate"
    echo "Using venv: $VENV_DIR"
else
    echo "Warning: No .venv found at $VENV_DIR — using system Python." >&2
    echo "  To create: python3 -m venv $VENV_DIR && $VENV_DIR/bin/pip install -r $SCRIPT_DIR/requirements.txt" >&2
fi

# Verify Python dependencies
if ! python3 -c "import zmq, numpy" 2>/dev/null; then
    echo "Error: Missing Python dependencies. Install with:" >&2
    echo "  pip install -r $SCRIPT_DIR/requirements.txt" >&2
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

# --- Parse preset for detector tp/tip override ---
# Extract --tp and --tip from arguments if provided, for the detector
SIM_ARGS=("$@")
for ((i=0; i<${#SIM_ARGS[@]}; i++)); do
    case "${SIM_ARGS[$i]}" in
        --tp)
            if ((i+1 < ${#SIM_ARGS[@]})); then
                TP="${SIM_ARGS[$((i+1))]}"
            fi
            ;;
        --tip)
            if ((i+1 < ${#SIM_ARGS[@]})); then
                TIP="${SIM_ARGS[$((i+1))]}"
            fi
            ;;
    esac
done

# --- 1. IQ Simulator (replaces airspyhf_zeromq_rx) ---
echo "Starting IQ simulator (ZMQ port ${ZMQ_PORT})..."
python3 -u "$SIM" -P "$ZMQ_PORT" "${SIM_ARGS[@]+"${SIM_ARGS[@]}"}" 2>&1 | sed -u 's/^/[sim] /' &
PIDS+=($!)
sleep 1

# --- 2. Decimator ---
# NOTE: The simulator generates tags at freq_offset_hz=0 (DC).  The decimator
# applies --shift-khz 10 which shifts the DC signal to +10 kHz in the
# decimated output.  But the real pipeline tunes the radio to center+10kHz
# so the tag at center lands at -10 kHz, and the decimator shifts +10 kHz
# to bring it to DC.  For the simulator the tag is already at DC, so we
# set --shift-khz 0 to keep it there.
echo "Starting decimator (ZMQ → UDP:${UDP_PORT})..."
"$DEC" \
    --zmq-endpoint "tcp://127.0.0.1:${ZMQ_PORT}" \
    --ports "$UDP_PORT" \
    --shift-khz 0 \
    2>&1 | grep -v '^.*: perf ' | sed 's/^/[dec] /' &
PIDS+=($!)
sleep 1

# --- 3. Pulse detector ---
echo "Starting pulse detector (tp=${TP}s, tip=${TIP}s, center=${TAG_CENTER} MHz)..."
echo ""
python3 -u "$DET" \
    --tp "$TP" \
    --tip "$TIP" \
    --port "$UDP_PORT" \
    --center-freq "$TAG_CENTER" \
    2>&1 | sed -u 's/^/[det] /'
