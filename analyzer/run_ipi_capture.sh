#!/usr/bin/env bash
#
# Capture IQ data from the Airspy HF+ for a fixed duration, then analyze
# inter-pulse intervals offline.
#
# Usage:
#   ./analyzer/run_ipi_capture.sh [duration_seconds]
#
# Default duration: 40 seconds.  Set the collar to the desired state before
# running, and move it during the capture to observe rate-switch behavior.
#
set -euo pipefail

DURATION=${1:-40}
REPO="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="$REPO/build"
VENV="$REPO/.venv/bin/python3"

SDR="$BUILD/airspyhf_zeromq/tools/src/airspyhf_zeromq_rx"
DEC="$BUILD/decimator/airspyhf_decimator"
ANALYZER="$REPO/analyzer/ipi_analyzer.py"

# Pipeline parameters (from last controller run)
TUNE_MHZ=146.649000
SHIFT_KHZ=10
INPUT_RATE=768000
FS=3840
PORT=10001
FREQ_OFFSET=-1755
EXPECTED_FREQ=146.639
TIP_A=1.333
TIP_B=2.0
TP=0.015

# Output file for raw IQ (binary, complex64)
CAPTURE_FILE="/tmp/ipi_capture_$(date +%Y%m%d_%H%M%S).raw"

echo "=== IPI Capture ==="
echo "  Duration:       ${DURATION}s"
echo "  Tune freq:      ${TUNE_MHZ} MHz"
echo "  Shift:          ${SHIFT_KHZ} kHz"
echo "  Capture file:   ${CAPTURE_FILE}"
echo ""

# PID variables — initialised so cleanup() is safe under set -u
CAP_PID=""
SDR_PID=""
DEC_PID=""

# Cleanup function
cleanup() {
    echo ""
    echo "Stopping pipeline..."
    [[ -n "$DEC_PID" ]] && kill "$DEC_PID" 2>/dev/null && wait "$DEC_PID" 2>/dev/null || true
    [[ -n "$SDR_PID" ]] && kill "$SDR_PID" 2>/dev/null && wait "$SDR_PID" 2>/dev/null || true
    [[ -n "$CAP_PID" ]] && kill "$CAP_PID" 2>/dev/null && wait "$CAP_PID" 2>/dev/null || true
}
trap cleanup EXIT

# 1. Start UDP capture using an inline Python UDP receiver
#    Binds to the decimator UDP port and writes raw payloads to file
"$VENV" -u -c "
import socket, sys, time, struct

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
except OSError:
    pass
sock.bind(('0.0.0.0', $PORT))
sock.settimeout(2.0)

duration = $DURATION
outfile = '$CAPTURE_FILE'
start = None
total_bytes = 0

print(f'  Listening on UDP port $PORT ...')

with open(outfile, 'wb') as f:
    while True:
        try:
            data = sock.recv(65536)
        except socket.timeout:
            if start is not None and time.monotonic() - start >= duration:
                break
            continue
        if start is None:
            start = time.monotonic()
            print(f'  First packet received, capturing for {duration}s ...')
        f.write(data)
        total_bytes += len(data)
        elapsed = time.monotonic() - start
        if elapsed >= duration:
            break

print(f'  Captured {total_bytes} bytes ({total_bytes/1024:.1f} KB) in {elapsed:.1f}s')
sock.close()
" &
CAP_PID=$!

sleep 0.5

# 2. Start SDR
echo "Starting SDR..."
"$SDR" -Z -f "$TUNE_MHZ" -a "$INPUT_RATE" -g off -m on > /dev/null 2>&1 &
SDR_PID=$!

# Wait for SDR to initialize
sleep 2

if ! kill -0 "$SDR_PID" 2>/dev/null; then
    echo "ERROR: SDR failed to start. Check USB connection."
    exit 1
fi
echo "  SDR running (PID $SDR_PID)"

# 3. Start decimator
echo "Starting decimator..."
"$DEC" --input-rate "$INPUT_RATE" --shift-khz "$SHIFT_KHZ" --ports "$PORT" > /dev/null 2>&1 &
DEC_PID=$!
sleep 1

if ! kill -0 "$DEC_PID" 2>/dev/null; then
    echo "ERROR: Decimator failed to start."
    exit 1
fi
echo "  Decimator running (PID $DEC_PID)"

echo ""
echo ">>> Capturing for ${DURATION}s — move the collar when ready <<<"
echo ""

# Wait for capture to finish
wait "$CAP_PID" 2>/dev/null || true

# Pipeline stops via cleanup trap

echo ""
echo "=== Analyzing captured data ==="
echo ""

# 4. Analyze the captured data offline
"$VENV" -u -c "
import struct, sys, numpy as np

capture_file = '$CAPTURE_FILE'
Fs = $FS
freq_hz = $FREQ_OFFSET
tip_a = $TIP_A
tip_b = $TIP_B
tp = $TP
tolerance = 0.15

# STFT parameters
n_w = int(np.ceil(tp * Fs))
n_ol = n_w // 2
n_ws = n_w - n_ol
time_step = n_ws / Fs

print(f'STFT: window={n_w}  overlap={n_ol}  step={n_ws}  time_step={time_step*1000:.2f} ms')
print()

# Read captured UDP packets and reassemble IQ stream
with open(capture_file, 'rb') as f:
    raw = f.read()

# Parse packets: each is [8-byte timestamp] [N * 8-byte complex64]
# The capture file contains raw UDP payloads concatenated.
# We need to figure out packet boundaries. The decimator sends fixed-size
# packets. Parse by scanning for valid timestamps.
# Simpler approach: the file is a stream of UDP payloads.
# Each payload = 8 bytes timestamp + N*8 bytes IQ.
# Decimator frame size is known from the pipeline.

# Actually, we wrote raw UDP recv() data sequentially. Each recv() returns
# one full UDP packet. We need to re-parse them.
# The decimator sends packets of consistent size. Let's detect the size
# from the first few packets by looking at timestamp patterns.

# For simplicity, strip timestamps and concatenate IQ.
# Packet size from decimator: frame=1024 complex samples + 1 timestamp sample
# = 1025 * 8 = 8200 bytes per packet... but after decimation it's smaller.
# Let's just try to parse greedily.

# The decimator output rate is 3840 sps. With default frame=1024 input samples
# and 200x decimation: output is ~1024/200 ≈ 5 samples per frame.
# But frame may vary. Let's parse by trying known sizes.

# Better: just decode all data as complex64, skip every Nth sample that's
# a timestamp. We know timestamps have a specific pattern.

# Actually simplest: reconstruct from raw packets. We need packet boundaries.
# Since we used sock.recv() and wrote sequentially, packet boundaries are lost.
# Let's use a different approach: we know the timestamp is the first sample
# of each packet, encoded as two float32s representing uint32 seconds and
# nanoseconds. The seconds field will be ~1.7e9 (year ~2026). As a float32
# reinterpreted from uint32, this will have a very specific bit pattern.

# Parse all as complex64 first
all_samples = np.frombuffer(raw, dtype=np.complex64)
print(f'Total samples (including timestamps): {len(all_samples)}')

# Find timestamp samples: reinterpret as pairs of uint32
raw_u32 = np.frombuffer(raw, dtype=np.uint32)
# Timestamps have seconds ~ 1.7e9 (2024-2026 range: 1704067200 - 1767225600)
# These are stored as float32 bit patterns reinterpreted as uint32
# We need to find indices where the float32 value, when bit-cast to uint32,
# falls in the valid seconds range.

# Each complex64 sample = 8 bytes = 2 float32s = 2 uint32s
# Timestamp is at index 0 of each packet. Its real part (first float32)
# is a uint32 seconds value bit-reinterpreted as float32.
# uint32 ~1.7e9 as float32 bits: struct.unpack('<f', struct.pack('<I', 1712000000))
# = roughly some large float. Let's just detect by checking if the real part
# of a sample, when reinterpreted as uint32, is in [1.6e9, 1.8e9].

real_parts = all_samples.real.copy()
real_as_u32 = real_parts.view(np.uint32)

valid_ts_mask = (real_as_u32 >= 1_600_000_000) & (real_as_u32 <= 1_900_000_000)
ts_indices = np.where(valid_ts_mask)[0]

print(f'Detected {len(ts_indices)} timestamp samples')

if len(ts_indices) < 2:
    print('ERROR: Could not detect packet boundaries')
    sys.exit(1)

# Infer packet size from spacing between timestamps
diffs = np.diff(ts_indices)
packet_size = int(np.median(diffs))
print(f'Packet size: {packet_size} samples (1 timestamp + {packet_size-1} IQ)')

# Extract IQ by removing timestamp samples
iq_mask = np.ones(len(all_samples), dtype=bool)
iq_mask[ts_indices] = False
iq = all_samples[iq_mask]

print(f'IQ samples: {len(iq)} ({len(iq)/Fs:.2f} s)')
print()

# STFT at target frequency
n_windows = (len(iq) - n_ol) // n_ws
if n_windows < 2:
    print('ERROR: Not enough data for STFT')
    sys.exit(1)

starts = np.arange(n_windows) * n_ws
idx = starts[:, None] + np.arange(n_w)[None, :]
segments = iq[idx]

# DFT at target frequency
k = np.arange(n_w, dtype=np.float64)
tone = np.exp(-2j * np.pi * freq_hz * k / Fs).astype(np.complex64)
scores = segments @ tone.conj()
power_ts = (scores.real**2 + scores.imag**2).astype(np.float32)

print(f'STFT windows: {n_windows} ({n_windows * time_step:.2f} s)')

# Threshold: adaptive noise floor
noise_floor = float(np.percentile(power_ts, 25))
if noise_floor <= 0:
    noise_floor = float(np.mean(power_ts)) * 0.1
threshold = noise_floor * (10.0 ** (25.0 / 10.0))  # 25 dB above noise

print(f'Noise floor: {noise_floor:.4e}  Threshold: {threshold:.4e}')
print(f'Peak power:  {power_ts.max():.4e}  ({10*np.log10(power_ts.max()/noise_floor):.1f} dB above noise)')
print()

# Detect rising edges
is_on = power_ts > threshold
transitions = np.diff(is_on.astype(np.int8))
rise_indices = np.where(transitions == 1)[0] + 1

if is_on[0]:
    rise_indices = np.concatenate(([0], rise_indices))

# Debounce: suppress rising edges within 0.5s of previous (min expected IPI)
min_gap_windows = int(0.5 / time_step)
filtered = [rise_indices[0]] if len(rise_indices) > 0 else []
for ri in rise_indices[1:]:
    if ri - filtered[-1] >= min_gap_windows:
        filtered.append(ri)
rise_indices = np.array(filtered)

print(f'Pulses detected: {len(rise_indices)} (after debounce, min gap {0.5}s)')
print()

if len(rise_indices) < 2:
    print('Not enough pulses for IPI analysis')
    sys.exit(0)

# Compute IPIs
rise_times = rise_indices * time_step
ipis = np.diff(rise_times)

# Classify
print(f'{\"#\":>4}  {\"time (s)\":>9}  {\"IPI (s)\":>9}  {\"class\":>5}  '
      f'{\"delta_A\":>9}  {\"delta_B\":>9}  notes')
print('-' * 80)

last_label = None
transitions_log = []
anom_list = []

for i, ipi in enumerate(ipis):
    t = rise_times[i+1]
    da = ipi - tip_a
    db = ipi - tip_b

    if abs(ipi - tip_a) / tip_a <= tolerance:
        label = 'A'
    elif abs(ipi - tip_b) / tip_b <= tolerance:
        label = 'B'
    else:
        label = 'ANOM'
        anom_list.append((i+1, ipi))

    notes = ''
    if last_label is not None and label != last_label:
        notes = f'  *** {last_label}->{label} ***'
        transitions_log.append((i+1, last_label, label, ipi))

    print(f'{i+1:4d}  {t:9.4f}  {ipi:9.4f}  {label:>5}  '
          f'{da:+9.4f}  {db:+9.4f}{notes}')
    last_label = label

# Summary
print()
print('=' * 80)
a_ipis = ipis[np.abs(ipis - tip_a) / tip_a <= tolerance]
b_ipis = ipis[np.abs(ipis - tip_b) / tip_b <= tolerance]
anom_ipis = [v for _, v in anom_list]

print(f'Rate A intervals:  {len(a_ipis)}  (expected {tip_a:.3f} s)')
if len(a_ipis) > 0:
    print(f'  mean={np.mean(a_ipis):.4f}  std={np.std(a_ipis):.4f}  '
          f'min={np.min(a_ipis):.4f}  max={np.max(a_ipis):.4f}')

print(f'Rate B intervals:  {len(b_ipis)}  (expected {tip_b:.3f} s)')
if len(b_ipis) > 0:
    print(f'  mean={np.mean(b_ipis):.4f}  std={np.std(b_ipis):.4f}  '
          f'min={np.min(b_ipis):.4f}  max={np.max(b_ipis):.4f}')

print(f'Anomalous:         {len(anom_ipis)}')
if anom_ipis:
    for n, v in anom_list:
        print(f'  #{n:4d}  IPI={v:.4f} s  '
              f'(delta_A={v-tip_a:+.4f}  delta_B={v-tip_b:+.4f})')

if transitions_log:
    print()
    print('Transitions:')
    for n, fr, to, ipi in transitions_log:
        print(f'  #{n:4d}  {fr}->{to}  IPI={ipi:.4f} s')
else:
    print()
    print('No rate transitions detected.')

print()
"

echo "Capture file: ${CAPTURE_FILE}"
