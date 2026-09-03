#!/usr/bin/env python3
"""
Signal Analyzer — detect a strong signal at a specific frequency and measure
pulse width (ms) and repetition rate (Hz / interval in seconds).

Reads decimated IQ data from the Airspy HF+ pipeline (same as pulse_detector.py):

  airspyhf_zeromq_rx  →  [ZMQ]  →  decimator  →  [UDP]  →  this script

Algorithm:
  1. Compute STFT of incoming IQ stream
  2. Find the strongest frequency bin (or use --freq-offset)
  3. Extract the power time-series at that bin
  4. Threshold the power to find on/off transitions
  5. Measure pulse widths and inter-pulse intervals from transitions

The radio must be tuned offset from the expected frequency to avoid the DC
spike.  The decimator's --shift-khz places a specific offset at DC in the
decimated output.  This script computes the expected frequency's position
in the decimated baseband automatically.

Examples:
  # Radio at 148.525 MHz, decimator --shift-khz 10, tag at 148.515 MHz:
  python signal_analyzer.py --expected-freq 148.515 --port 10000

  # Override the frequency offset manually:
  python signal_analyzer.py --expected-freq 148.515 --freq-offset 200 --port 10000
"""

import argparse
import datetime
import signal
import socket
import struct
import sys
import time

import numpy as np

# Global flag for graceful shutdown
_should_stop = False


def _signal_handler(signum, frame):
    global _should_stop
    _should_stop = True


# ---------------------------------------------------------------------------
# UDP packet decoding (same as pulse_detector.py)
# ---------------------------------------------------------------------------

def decode_timestamp(raw_8bytes):
    """Decode wall-clock timestamp from the decimator's first-sample header.

    Returns timestamp in nanoseconds as an integer.
    """
    i_f, q_f = struct.unpack('<ff', raw_8bytes)
    sec  = struct.unpack('<I', struct.pack('<f', i_f))[0]
    nsec = struct.unpack('<I', struct.pack('<f', q_f))[0]
    return sec * 1_000_000_000 + nsec


# ---------------------------------------------------------------------------
# Envelope detection (mix to baseband + lowpass)
# ---------------------------------------------------------------------------

def extract_envelope(iq, Fs, freq_hz, lpf_cutoff_hz=500.0):
    """Extract the power envelope at a specific frequency.

    Mixes the signal to baseband at freq_hz, lowpass filters, and returns
    the power envelope at the full sample rate.

    Args:
        iq:             1-D complex64 array
        Fs:             Sample rate in Hz
        freq_hz:        Frequency to extract (Hz offset from DC)
        lpf_cutoff_hz:  Lowpass filter cutoff in Hz (default: 500)

    Returns:
        envelope: 1-D float32 array of power values at sample rate Fs
    """
    n = len(iq)
    t = np.arange(n, dtype=np.float64) / Fs

    # Mix to baseband: shift the target frequency to DC
    mixed = iq * np.exp(-2j * np.pi * freq_hz * t).astype(np.complex64)

    # Compute instantaneous power first, then smooth.
    # This avoids residual carrier ripple that would occur from filtering
    # I/Q separately when the frequency estimate is imperfect.
    inst_power = (mixed.real**2 + mixed.imag**2).astype(np.float64)

    # Moving-average lowpass on the power envelope. The kernel must be much
    # shorter than the pulse (15–20 ms for typical collars) or the measured
    # width is biased toward the kernel length. 500 Hz at 3840 Hz → 7 samples
    # (~1.8 ms).
    kernel_len = max(int(Fs / lpf_cutoff_hz), 3)
    if kernel_len % 2 == 0:
        kernel_len += 1
    kernel = np.ones(kernel_len, dtype=np.float64) / kernel_len

    envelope = np.convolve(inst_power, kernel, mode='same')
    return envelope.astype(np.float32)


def find_peak_frequency_fft(iq, Fs, exclude_dc_hz=50.0):
    """Find the strongest frequency in the IQ segment using a full FFT.

    Args:
        iq:             1-D complex array
        Fs:             Sample rate in Hz
        exclude_dc_hz:  Exclude frequencies within this range of DC

    Returns:
        peak_freq_hz: float
    """
    n = len(iq)
    S = np.fft.fftshift(np.fft.fft(iq))
    power = (S.real**2 + S.imag**2).astype(np.float64)
    freq_axis = np.fft.fftshift(np.fft.fftfreq(n, d=1.0 / Fs))

    # Mask DC region
    dc_mask = np.abs(freq_axis) < exclude_dc_hz
    power[dc_mask] = 0

    peak_bin = np.argmax(power)
    return float(freq_axis[peak_bin])


# ---------------------------------------------------------------------------
# Pulse measurement
# ---------------------------------------------------------------------------

def measure_pulses(power_time_series, time_step_s, threshold_db_above_noise=6.0):
    """Detect on/off transitions in a power time series and measure pulses.

    Args:
        power_time_series: 1-D array of power values at one frequency bin
        time_step_s:       Time between consecutive power samples (seconds)
        threshold_db_above_noise: dB above noise floor to consider "on"

    Returns:
        dict with:
          pulse_widths_ms:   list of pulse widths in milliseconds
          intervals_s:       list of pulse-to-pulse intervals in seconds
          rep_rate_hz:       repetition rate (1/mean_interval), or None
          mean_pulse_ms:     mean pulse width in ms, or None
          median_pulse_ms:   median pulse width in ms, or None
          mean_interval_s:   mean interval in seconds, or None
    """
    if len(power_time_series) < 3:
        return _empty_result()

    # Estimate noise floor as the 25th percentile of power (robust to pulses)
    noise_floor = np.percentile(power_time_series, 25)
    if noise_floor <= 0:
        noise_floor = np.min(power_time_series[power_time_series > 0])
        if noise_floor <= 0:
            return _empty_result()

    threshold_linear = noise_floor * (10.0 ** (threshold_db_above_noise / 10.0))

    # Boolean mask: True where signal is above threshold
    is_on = power_time_series > threshold_linear

    # Find transitions
    transitions = np.diff(is_on.astype(np.int8))
    rise_indices = np.where(transitions == 1)[0] + 1   # rising edge
    fall_indices = np.where(transitions == -1)[0] + 1   # falling edge

    # If signal starts high, add a rising edge at index 0
    if is_on[0]:
        rise_indices = np.concatenate(([0], rise_indices))
    # If signal ends high, add a falling edge at the end
    if is_on[-1]:
        fall_indices = np.concatenate((fall_indices, [len(is_on)]))

    # Pair up rises and falls. Width is measured at half of each pulse's own
    # peak (above noise) so it is independent of SNR and threshold choice.
    pulse_widths_ms = []
    pulse_starts = []

    n_pulses = min(len(rise_indices), len(fall_indices))
    for i in range(n_pulses):
        rise = rise_indices[i]
        # Find the first fall after this rise
        falls_after = fall_indices[fall_indices > rise]
        if len(falls_after) == 0:
            break
        fall = falls_after[0]
        start_s, width_s = _half_max_width(power_time_series, rise, fall,
                                           noise_floor, time_step_s)
        pulse_widths_ms.append(width_s * 1000.0)
        pulse_starts.append(start_s)

    # Inter-pulse intervals (start-to-start)
    intervals_s = []
    for i in range(1, len(pulse_starts)):
        intervals_s.append(pulse_starts[i] - pulse_starts[i - 1])

    result = {
        'pulse_widths_ms': pulse_widths_ms,
        'intervals_s': intervals_s,
        'noise_floor': noise_floor,
        'threshold': threshold_linear,
        'n_pulses': len(pulse_widths_ms),
    }

    if pulse_widths_ms:
        result['mean_pulse_ms'] = np.mean(pulse_widths_ms)
        result['median_pulse_ms'] = np.median(pulse_widths_ms)
    else:
        result['mean_pulse_ms'] = None
        result['median_pulse_ms'] = None

    if intervals_s:
        result['mean_interval_s'] = np.mean(intervals_s)
        result['rep_rate_hz'] = 1.0 / np.mean(intervals_s)
    else:
        result['mean_interval_s'] = None
        result['rep_rate_hz'] = None

    return result


def _half_max_width(power, rise, fall, noise_floor, time_step_s):
    """Return (start_s, width_s) of the pulse in [rise, fall) at half-max.

    Half-max is relative to the noise floor. The peak search and the outward
    walk are confined to [rise, fall) so a weak pulse (half-max below the
    threshold) cannot wander into neighbouring noise; each edge crossing is
    then linearly interpolated between the boundary sample and its neighbour
    just outside, so start_s may land in [rise-1, rise].
    """
    n = len(power)
    hi = min(fall, n)
    peak_idx = rise + int(np.argmax(power[rise:hi]))
    half = noise_floor + 0.5 * (power[peak_idx] - noise_floor)

    def _cross(a, b):
        # Fractional position between samples a and b where power crosses half
        pa, pb = power[a], power[b]
        if pb == pa:
            return float(b)
        return float(np.clip(a + (half - pa) / (pb - pa), a, b))

    left = peak_idx
    while left > rise and power[left - 1] >= half:
        left -= 1
    t_start = _cross(left - 1, left) if left > 0 else float(left)

    right = peak_idx
    last = min(fall, n) - 1
    while right < last and power[right + 1] >= half:
        right += 1
    t_end = _cross(right, right + 1) if right < n - 1 else float(right)

    return t_start * time_step_s, (t_end - t_start) * time_step_s


def _empty_result():
    return {
        'pulse_widths_ms': [],
        'intervals_s': [],
        'noise_floor': None,
        'threshold': None,
        'n_pulses': 0,
        'mean_pulse_ms': None,
        'median_pulse_ms': None,
        'mean_interval_s': None,
        'rep_rate_hz': None,
    }


# ---------------------------------------------------------------------------
# Display
# ---------------------------------------------------------------------------

def print_measurements(cycle, ts_str, meas, freq_display, proc_ms, segment_s):
    """Print a summary line for one analysis cycle."""
    if meas['n_pulses'] == 0:
        print(f'[{cycle:4d} {ts_str}]  {freq_display}  '
              f'no pulses detected  ({segment_s:.1f}s window, {proc_ms:.0f} ms)',
              flush=True)
        return

    pw_str = f'{meas["median_pulse_ms"]:.1f}' if meas['median_pulse_ms'] else '?'
    interval_str = f'{meas["mean_interval_s"]:.3f}' if meas['mean_interval_s'] else '?'
    rate_str = f'{meas["rep_rate_hz"]:.3f}' if meas['rep_rate_hz'] else '?'

    # Show individual pulse widths if few enough
    if meas['n_pulses'] <= 8:
        widths = ', '.join(f'{w:.1f}' for w in meas['pulse_widths_ms'])
        pw_detail = f'  [{widths}]'
    else:
        pw_detail = ''

    print(f'[{cycle:4d} {ts_str}]  {freq_display}  '
          f'{meas["n_pulses"]} pulses  '
          f'width={pw_str} ms{pw_detail}  '
          f'interval={interval_str} s  '
          f'rate={rate_str} Hz  '
          f'({segment_s:.1f}s window, {proc_ms:.0f} ms)',
          flush=True)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def run(args):
    """Run analyzer reading from the Airspy HF+ decimator UDP pipeline."""
    global _should_stop

    Fs = args.fs
    analysis_duration = args.duration
    samples_needed = int(Fs * analysis_duration)
    time_step = 1.0 / Fs  # Envelope is at full sample rate

    # The decimator centers DC at the radio tune frequency + shift-khz.
    # The expected signal will appear at an offset from DC in the decimated
    # baseband.  If --freq-offset is not given, auto-detect the peak.
    expected_freq_mhz = args.expected_freq

    print('=== Signal Analyzer (Airspy HF+) ===')
    print(f'  Expected freq   {expected_freq_mhz:.6f} MHz')
    print(f'  Sample rate     {Fs:.1f} Hz')
    print(f'  Analysis window {analysis_duration:.1f} s')
    print(f'  Time resolution {time_step*1000:.2f} ms (envelope at sample rate)')
    print(f'  Freq resolution {Fs/samples_needed:.4f} Hz (full-segment FFT)')
    print(f'  Samples/window  {samples_needed}')
    print(f'  UDP port        {args.port}')
    if args.freq_offset is not None:
        print(f'  Freq offset     {args.freq_offset:.1f} Hz (manual)')
    else:
        print(f'  Freq offset     auto-detect (avoid DC spike)')
    print(f'  Threshold       {args.threshold:.1f} dB above noise')
    print()

    # UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_buf_bytes = 1 << 20
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, recv_buf_bytes)
    except OSError:
        pass
    sock.bind(('0.0.0.0', args.port))
    sock.settimeout(2.0)

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    buf_parts = []
    buf_len = 0
    seg_ts = None
    cycle = 0
    total_packets = 0
    total_samples = 0
    last_status_time = time.monotonic()

    # Running stats for summary
    all_widths = []
    all_intervals = []

    print('Waiting for data ...\n')

    try:
        while not _should_stop:
            try:
                data = sock.recv(65536)
            except socket.timeout:
                now = time.monotonic()
                if total_packets == 0 and now - last_status_time > 5.0:
                    print('  (no data received yet — check pipeline is running)',
                          flush=True)
                    last_status_time = now
                elif total_packets > 0 and now - last_status_time > 5.0:
                    print(f'  (timeout — no data for 2s, '
                          f'{total_packets} packets/{total_samples} samples so far)',
                          flush=True)
                    last_status_time = now
                continue
            if len(data) < 16:
                continue

            try:
                ts = decode_timestamp(data[:8])
            except (struct.error, ValueError):
                continue

            if seg_ts is None:
                seg_ts = ts

            payload = data[8:]
            n_samp = len(payload) // 8
            if n_samp == 0:
                continue

            iq = np.frombuffer(payload[:n_samp * 8],
                               dtype=np.complex64).copy()
            buf_parts.append(iq)
            buf_len += n_samp
            total_packets += 1
            total_samples += n_samp

            # Log first packet and periodic status
            if total_packets == 1:
                print(f'  Receiving data: first packet {n_samp} samples',
                      flush=True)
            now = time.monotonic()
            if now - last_status_time > 5.0:
                pct = 100.0 * buf_len / samples_needed
                print(f'  Buffering: {buf_len}/{samples_needed} samples '
                      f'({pct:.0f}%)  [{total_packets} packets received]',
                      flush=True)
                last_status_time = now

            if buf_len < samples_needed:
                continue

            cycle += 1
            t0 = time.monotonic()

            segment = np.concatenate(buf_parts)[:samples_needed]
            buf_parts.clear()
            buf_len = 0

            # Find or use specified frequency
            if args.freq_offset is not None:
                peak_freq = args.freq_offset
            else:
                peak_freq = find_peak_frequency_fft(segment, Fs)

            # Extract power envelope at the target frequency
            envelope = extract_envelope(segment, Fs, peak_freq)

            # Measure pulses from the envelope
            meas = measure_pulses(envelope, time_step, args.threshold)

            proc_ms = (time.monotonic() - t0) * 1000.0

            # Format frequency display
            abs_mhz = expected_freq_mhz + peak_freq / 1e6
            freq_display = f'{abs_mhz:.6f} MHz ({peak_freq:+.1f} Hz)'

            # Timestamp
            ts_str = ''
            if seg_ts and seg_ts > 1e9:
                ts_str = datetime.datetime.fromtimestamp(
                    seg_ts / 1e9, tz=datetime.timezone.utc
                ).strftime('%H:%M:%S')
            seg_ts = None

            print_measurements(cycle, ts_str, meas, freq_display,
                               proc_ms, analysis_duration)

            all_widths.extend(meas['pulse_widths_ms'])
            all_intervals.extend(meas['intervals_s'])

    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        _print_summary(cycle, all_widths, all_intervals)


# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------

def _print_summary(cycle, all_widths, all_intervals):
    """Print aggregate statistics."""
    print(f'\n--- Analysis complete: {cycle} cycles ---', flush=True)

    if all_widths:
        w = np.array(all_widths)
        print(f'  Pulse width:     mean={np.mean(w):.2f} ms  '
              f'median={np.median(w):.2f} ms  '
              f'std={np.std(w):.2f} ms  '
              f'(n={len(w)})', flush=True)
    else:
        print('  Pulse width:     no pulses detected', flush=True)

    if all_intervals:
        iv = np.array(all_intervals)
        print(f'  Repetition:      mean={np.mean(iv):.4f} s  '
              f'({1.0/np.mean(iv):.4f} Hz)  '
              f'std={np.std(iv):.4f} s  '
              f'(n={len(iv)})', flush=True)
    else:
        print('  Repetition:      insufficient data', flush=True)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description='Signal Analyzer — measure pulse width and repetition rate',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)

    ap.add_argument('--expected-freq', type=float, required=True,
                    help='Expected signal frequency in MHz (e.g. 148.515). '
                         'The radio should be tuned offset from this to avoid '
                         'the DC spike.')
    ap.add_argument('--duration', type=float, default=10.0,
                    help='Analysis window duration in seconds (default: 10)')
    ap.add_argument('--threshold', type=float, default=10.0,
                    help='Detection threshold in dB above noise floor (default: 10)')
    ap.add_argument('--freq-offset', type=float, default=None,
                    help='Frequency offset in Hz to monitor in decimated baseband '
                         '(default: auto-detect strongest peak, excluding DC)')
    ap.add_argument('--port', type=int, default=10000,
                    help='UDP port for decimated IQ (default: 10000)')
    ap.add_argument('--fs', type=float, default=3840.0,
                    help='Decimated sample rate in Hz (default: 3840)')

    args = ap.parse_args()
    run(args)


if __name__ == '__main__':
    main()
