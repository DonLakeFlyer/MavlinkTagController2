#!/usr/bin/env bash
#
# Run the inter-pulse-interval analyzer live against the Airspy HF+:
#
#   airspyhf_zeromq_rx  →  [ZMQ]  →  decimator  →  [UDP]  →  ipi_analyzer.py
#
# Usage:
#   ./run_ipi_analyzer.sh --expected-freq 147.970 --tip-a 2.0 --tip-b 1.5 [--tp 0.019 ...]
#
# Start with the collar still, then move it, then set it down, to see the
# rate switch in the per-interval log.  Stop with Ctrl-C.
#
# All extra arguments are forwarded to ipi_analyzer.py.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$REPO_DIR/build"

RX="$BUILD_DIR/airspyhf_zeromq/tools/src/airspyhf_zeromq_rx"
DEC="$BUILD_DIR/decimator/airspyhf_decimator"
ANALYZER="$SCRIPT_DIR/ipi_analyzer.py"

ZMQ_PORT=5555
UDP_PORT=10001        # Different from detector default to allow both
SHIFT_KHZ=10          # Tune offset to avoid DC spike

if [ -f "$REPO_DIR/.venv/bin/activate" ]; then
    # shellcheck disable=SC1091
    source "$REPO_DIR/.venv/bin/activate"
fi

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
    echo "Error: --expected-freq is required (e.g. --expected-freq 147.970)" >&2
    echo "Usage: $0 --expected-freq <MHz> --tip-a <s> --tip-b <s> [ipi_analyzer.py options]" >&2
    exit 1
fi

# Radio tunes SHIFT_KHZ above expected so the decimator's shift places the
# expected frequency at DC in the decimated output, away from the radio's DC spike.
RADIO_FREQ=$(python3 -c 'import sys; print(f"{float(sys.argv[1]) + float(sys.argv[2]) / 1000.0:.6f}")' \
    "$EXPECTED_FREQ" "$SHIFT_KHZ" 2>/dev/null) || {
    echo "Error: --expected-freq must be numeric, got '$EXPECTED_FREQ'" >&2
    exit 1
}

for bin in "$RX" "$DEC"; do
    if [ ! -x "$bin" ]; then
        echo "Error: $bin not found or not executable. Run: cmake --build build" >&2
        exit 1
    fi
done

PIDS=()
cleanup() {
    trap - EXIT INT TERM  # run once even when INT/TERM is followed by EXIT
    echo ""
    echo "Stopping pipeline..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    for pid in "${PIDS[@]}"; do
        # analyzer may be blocked in a 2 s socket timeout before it can exit
        for _ in 1 2 3 4 5 6; do
            kill -0 "$pid" 2>/dev/null || break
            sleep 0.5
        done
        kill -9 "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    echo "Done."
}
trap cleanup EXIT INT TERM

# Fail fast if a pipeline stage died during its startup grace period
# (e.g. no Airspy on USB), instead of letting the analyzer wait forever.
require_alive() {
    if ! kill -0 "$1" 2>/dev/null; then
        echo "Error: $2 exited during startup (see its log lines above)" >&2
        exit 1
    fi
}

echo "=== IPI Analyzer Pipeline ==="
echo "  Expected freq   ${EXPECTED_FREQ} MHz"
echo "  Radio tuned to  ${RADIO_FREQ} MHz (+${SHIFT_KHZ} kHz offset)"
echo "  ZMQ port        ${ZMQ_PORT}"
echo "  UDP port        ${UDP_PORT}"
echo ""

# Log prefixes via process substitution so $! is the producer, not sed.
echo "Starting airspyhf_zeromq_rx at ${RADIO_FREQ} MHz (ZMQ port ${ZMQ_PORT})..."
"$RX" -f "$RADIO_FREQ" -P "$ZMQ_PORT" > >(sed 's/^/[rx]  /') 2>&1 &
RX_PID=$!
PIDS+=("$RX_PID")
sleep 1
require_alive "$RX_PID" "airspyhf_zeromq_rx"

echo "Starting decimator (ZMQ → UDP:${UDP_PORT})..."
"$DEC" \
    --zmq-endpoint "tcp://127.0.0.1:${ZMQ_PORT}" \
    --ports "$UDP_PORT" \
    --shift-khz "$SHIFT_KHZ" \
    > >(grep -v '^.*: perf ' | sed 's/^/[dec] /') 2>&1 &
DEC_PID=$!
PIDS+=("$DEC_PID")
sleep 1
require_alive "$DEC_PID" "airspyhf_decimator"

echo "Starting IPI analyzer (expected ${EXPECTED_FREQ} MHz)..."
echo ""
# Backgrounded + wait so INT/TERM traps fire promptly (bash defers traps
# while a foreground command runs).
python3 -u "$ANALYZER" \
    --port "$UDP_PORT" \
    "${EXTRA_ARGS[@]}" \
    > >(sed -u 's/^/[ipi] /') 2>&1 &
ANA_PID=$!
PIDS+=("$ANA_PID")
# Propagate the analyzer's status (130 on Ctrl-C, nonzero on bind/arg
# failure); the EXIT trap still runs cleanup.
wait "$ANA_PID" && exit_code=0 || exit_code=$?
exit "$exit_code"
