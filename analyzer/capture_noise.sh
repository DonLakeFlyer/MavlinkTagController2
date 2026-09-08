#!/usr/bin/env bash
#
# Capture baseline noise IQ from an Airspy HF+ with airspyhf_rx, write the
# .json sidecar psd_spectrum.py needs, then run the PSD analysis.
#
# Mirrors the controller's COMMAND_ID_RAW_CAPTURE: the radio is tuned 10 kHz
# above the requested frequency (AGC off, LNA on) so the requested frequency
# sits away from the DC spike at baseband 0 Hz.
#
# Usage:
#   ./capture_noise.sh <freq_mhz> [output_base] [duration_s] [psd_spectrum.py options]
#
#   ./capture_noise.sh 147.970
#   ./capture_noise.sh 147.970 ~/Downloads/noise 10 --png ~/Downloads/noise.png

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

SAMPLE_RATE=768000
DC_OFFSET_HZ=10000     # kAirSpyHfFrequencyOffsetHz in controller

if [ $# -lt 1 ]; then
    echo "Usage: $0 <freq_mhz> [output_base] [duration_s] [psd_spectrum.py options]" >&2
    exit 1
fi

FREQ_MHZ="$1"; shift
OUT_BASE="${1:-$HOME/Downloads/noise}"; [ $# -gt 0 ] && shift
DURATION_S="${1:-10}"; [ $# -gt 0 ] && shift

if ! command -v airspyhf_rx >/dev/null; then
    echo "Error: airspyhf_rx not found in PATH" >&2
    exit 1
fi
if ! command -v python3 >/dev/null; then
    echo "Error: python3 not found in PATH" >&2
    exit 1
fi

# Normalize both values so the JSON sidecar always gets canonical float tokens.
FREQS=$(python3 -c 'import sys; f = float(sys.argv[1]); print(f"{f:.6f} {f + float(sys.argv[2]) / 1e6:.6f}")' \
    "$FREQ_MHZ" "$DC_OFFSET_HZ" 2>/dev/null) || {
    echo "Error: freq_mhz must be numeric, got '$FREQ_MHZ'" >&2
    exit 1
}
read -r FREQ_MHZ TUNE_MHZ <<< "$FREQS"
case "$DURATION_S" in
    ''|*[!0-9]*) echo "Error: duration_s must be a positive integer, got '$DURATION_S'" >&2; exit 1 ;;
esac
# Force base 10 so leading zeros are not read as octal.
DURATION_S=$((10#$DURATION_S))
if [ "$DURATION_S" -le 0 ]; then
    echo "Error: duration_s must be a positive integer, got '$DURATION_S'" >&2
    exit 1
fi
NUM_SAMPLES=$((SAMPLE_RATE * DURATION_S))

DAT="$OUT_BASE.dat"
JSON="$OUT_BASE.json"
mkdir -p "$(dirname "$DAT")"

echo "Capturing ${DURATION_S}s at ${TUNE_MHZ} MHz (requested ${FREQ_MHZ} MHz + ${DC_OFFSET_HZ} Hz DC offset) -> $DAT"
CAPTURE_UTC=$(date -u +%Y-%m-%dT%H:%M:%SZ)
CAPTURE_EPOCH=$(date +%s)
airspyhf_rx -r "$DAT" -f "$TUNE_MHZ" -a "$SAMPLE_RATE" -g off -m on -n "$NUM_SAMPLES"

cat > "$JSON" <<EOF
{
  "sdr": "airspy_hf",
  "requested_freq_mhz": $FREQ_MHZ,
  "tune_freq_mhz": $TUNE_MHZ,
  "dc_offset_hz": $DC_OFFSET_HZ,
  "sample_rate_hz": $SAMPLE_RATE,
  "bandwidth_hz": $SAMPLE_RATE,
  "duration_seconds": $DURATION_S,
  "gain": 0,
  "format": "complex_float32",
  "capture_utc": "$CAPTURE_UTC",
  "capture_epoch": $CAPTURE_EPOCH,
  "data_file": "$DAT"
}
EOF
echo "Wrote $JSON"

if [ -f "$REPO_DIR/.venv/bin/activate" ]; then
    # shellcheck disable=SC1091
    source "$REPO_DIR/.venv/bin/activate"
fi
python3 "$SCRIPT_DIR/psd_spectrum.py" "$DAT" "$@"
