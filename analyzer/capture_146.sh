#!/usr/bin/env bash
# 13 s Airspy HF+ noise capture at 146 MHz, same settings as the controller's raw capture.
# Also writes the .json sidecar psd_spectrum.py needs.
set -euo pipefail
cd "$(dirname "$0")/.."

FREQ_MHZ=146.000000
SAMPLE_RATE=768000
DURATION_S=13
OUT_BASE=~/Downloads/airspy-hf.1
DAT="$OUT_BASE.dat"
JSON="$OUT_BASE.json"

CAPTURE_UTC=$(date -u +%Y-%m-%dT%H:%M:%SZ)
CAPTURE_EPOCH=$(date +%s)
./build/airspyhf_zeromq/tools/src/airspyhf_zeromq_rx \
    -r "$DAT" -f "$FREQ_MHZ" -a "$SAMPLE_RATE" -g off -m on -n $((SAMPLE_RATE * DURATION_S))

cat > "$JSON" <<EOF
{
  "sdr": "airspy_hf",
  "requested_freq_mhz": $FREQ_MHZ,
  "tune_freq_mhz": $FREQ_MHZ,
  "dc_offset_hz": 0,
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
echo "Wrote $DAT and $JSON"
