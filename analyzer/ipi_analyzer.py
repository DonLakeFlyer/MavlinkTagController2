#!/usr/bin/env python3
"""
Inter-Pulse Interval (IPI) Analyzer — measure every pulse-to-pulse gap in
real time from the decimated IQ stream and classify each interval as
rate A (resting), rate B (moving), or anomalous (transition artifact).

Reads from the same UDP pipeline as pulse_detector.py:

  airspyhf_zeromq_rx  →  [ZMQ]  →  decimator  →  [UDP]  →  this script

Unlike pulse_detector.py (which operates on fixed K-pulse segments), this
script uses a rolling buffer and reports each inter-pulse interval as it
occurs, so no pulse is lost at a segment boundary.

Examples:
  # Real collar, auto-detect frequency:
  python ipi_analyzer.py --tip-a 1.333 --tip-b 2.0 --expected-freq 146.639 --port 10001

  # With known frequency offset:
  python ipi_analyzer.py --tip-a 1.333 --tip-b 2.0 --expected-freq 146.639 \\
      --freq-offset -1755 --port 10001
"""

import argparse
import datetime
import signal
import socket
import struct
import sys
import time

import numpy as np

_should_stop = False


def _signal_handler(signum, frame):
    global _should_stop
    _should_stop = True


# ---------------------------------------------------------------------------
# UDP helpers (identical to signal_analyzer.py / pulse_detector.py)
# ---------------------------------------------------------------------------

def decode_timestamp(raw_8bytes):
    """Decode wall-clock timestamp from the decimator's first-sample header."""
    i_f, q_f = struct.unpack('<ff', raw_8bytes)
    sec  = struct.unpack('<I', struct.pack('<f', i_f))[0]
    nsec = struct.unpack('<I', struct.pack('<f', q_f))[0]
    return sec * 1_000_000_000 + nsec


# ---------------------------------------------------------------------------
# Envelope extraction
# ---------------------------------------------------------------------------

def extract_power_at_freq(iq, Fs, freq_hz, n_w, n_ol):
    """Extract power time-series at a specific frequency using STFT.

    Returns (power_ts, consumed) where power_ts has one sample per STFT
    window step and consumed is the number of leading IQ samples fully
    accounted for (the remainder must be carried to the next call).
    """
    n_ws = n_w - n_ol
    n_windows = (len(iq) - n_ol) // n_ws
    if n_windows < 1:
        return np.array([], dtype=np.float32), 0

    # Gather windowed segments
    starts = np.arange(n_windows) * n_ws
    idx = starts[:, None] + np.arange(n_w)[None, :]
    segments = iq[idx]  # (n_windows, n_w)

    # DFT at the single target frequency (no full FFT needed)
    k = np.arange(n_w, dtype=np.float64)
    tone = np.exp(-2j * np.pi * freq_hz * k / Fs).astype(np.complex64)
    # Dot product of each segment with the tone
    scores = segments @ tone.conj()
    power_ts = (scores.real**2 + scores.imag**2).astype(np.float32)

    return power_ts, n_windows * n_ws


def stft_stream_step(carry, iq, Fs, freq_hz, n_w, n_ol):
    """Run the STFT over carry+iq; return (power_ts, new_carry).

    Consecutive calls produce exactly the window sequence a single call over
    the concatenated stream would, regardless of how it is chunked.
    """
    buf = np.concatenate((carry, iq)) if len(carry) else iq
    power_ts, consumed = extract_power_at_freq(buf, Fs, freq_hz, n_w, n_ol)
    return power_ts, buf[consumed:]


def find_peak_frequency_fft(iq, Fs, exclude_dc_hz=50.0):
    """Find the strongest frequency in the IQ segment (excluding DC)."""
    n = len(iq)
    S = np.fft.fftshift(np.fft.fft(iq))
    power = (S.real**2 + S.imag**2).astype(np.float64)
    freq_axis = np.fft.fftshift(np.fft.fftfreq(n, d=1.0 / Fs))
    dc_mask = np.abs(freq_axis) < exclude_dc_hz
    power[dc_mask] = 0
    peak_bin = np.argmax(power)
    return float(freq_axis[peak_bin])


# ---------------------------------------------------------------------------
# Pulse-edge detector (rolling)
# ---------------------------------------------------------------------------

class PulseEdgeTracker:
    """Tracks rising/falling edges in a power time-series and emits IPIs.

    Operates on STFT-rate power samples (one per window step).
    """

    def __init__(self, time_step_s, threshold_db=10.0, min_pulse_windows=1,
                 max_pulse_windows=10, refractory_windows=0):
        self.time_step = time_step_s
        self.threshold_db = threshold_db
        self.min_pulse_windows = min_pulse_windows
        self.max_pulse_windows = max_pulse_windows
        self.refractory_windows = refractory_windows

        self.noise_floor = None
        self.noise_alpha = 0.02

        self.is_on = False
        self.on_start_idx = 0
        self.last_rise_idx = None
        self.global_idx = 0

        self._init_buf = []
        self._init_count = 0
        self._init_target = int(1.0 / time_step_s)  # ~1 s warmup

    def _update_noise(self, off_values):
        if len(off_values) == 0:
            return
        mean_off = float(np.mean(off_values))
        if self.noise_floor is None:
            self.noise_floor = mean_off
        else:
            self.noise_floor = (1.0 - self.noise_alpha) * self.noise_floor \
                               + self.noise_alpha * mean_off

    def _threshold(self):
        if self.noise_floor is None or self.noise_floor <= 0:
            return np.inf
        return self.noise_floor * (10.0 ** (self.threshold_db / 10.0))

    def process(self, power_ts):
        """Process a chunk of STFT power samples.

        Yields (ipi_seconds, rise_global_idx) for each completed IPI.
        """
        if self.noise_floor is None:
            self._init_buf.append(power_ts)
            self._init_count += len(power_ts)
            if self._init_count < self._init_target:
                return
            warmup = np.concatenate(self._init_buf)
            # Per-window noise power is ~exponential; median/ln2 estimates its
            # mean robustly even when pulses are present in the warmup.
            self.noise_floor = float(np.median(warmup)) / np.log(2.0)
            if self.noise_floor <= 0:
                self.noise_floor = float(np.mean(warmup)) * 0.1
            self._init_buf.clear()
            yield from self._scan(warmup, 0)
            self.global_idx = len(warmup)
            return

        yield from self._scan(power_ts, self.global_idx)  # _scan advances global_idx

    def reset_after_gap(self):
        """Forget the in-progress pulse so no IPI is measured across lost data."""
        self.is_on = False
        self.last_rise_idx = None

    def _scan(self, power_ts, base_idx):
        thresh = self._threshold()
        off_samples = []

        for i in range(len(power_ts)):
            idx = base_idx + i
            val = float(power_ts[i])

            if not self.is_on:
                if val > thresh:
                    if (self.last_rise_idx is not None
                            and idx - self.last_rise_idx < self.refractory_windows):
                        continue  # tail of the same pulse; ignore, keep state

                    self.is_on = True
                    self.on_start_idx = idx

                    if self.last_rise_idx is not None:
                        ipi_windows = idx - self.last_rise_idx
                        yield (ipi_windows * self.time_step, idx)

                    self.last_rise_idx = idx
                else:
                    off_samples.append(val)
            else:
                pulse_len = idx - self.on_start_idx
                if val <= thresh or pulse_len >= self.max_pulse_windows:
                    self.is_on = False
                    if pulse_len < self.min_pulse_windows:
                        self.last_rise_idx = None

        if off_samples:
            self._update_noise(np.array(off_samples, dtype=np.float32))

        self.global_idx = base_idx + len(power_ts)


# ---------------------------------------------------------------------------
# IPI classifier
# ---------------------------------------------------------------------------

def classify_ipi(ipi_s, tip_a, tip_b, tolerance=0.15):
    """Classify an IPI as rate A, rate B, or anomalous.

    Args:
        ipi_s:     Measured inter-pulse interval in seconds.
        tip_a:     Expected rate A interval (seconds).
        tip_b:     Expected rate B interval (seconds).
        tolerance: Fractional tolerance (default 15%).

    Returns:
        label: str — 'A', 'B', or 'ANOM'
    """
    if abs(ipi_s - tip_a) / tip_a <= tolerance:
        return 'A'
    if abs(ipi_s - tip_b) / tip_b <= tolerance:
        return 'B'
    return 'ANOM'


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    global _should_stop

    ap = argparse.ArgumentParser(
        description='Inter-Pulse Interval Analyzer — measure and classify '
                    'every pulse gap in real time',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument('--tip-a', type=float, required=True,
                    help='Rate A (resting) inter-pulse interval in seconds')
    ap.add_argument('--tip-b', type=float, required=True,
                    help='Rate B (moving) inter-pulse interval in seconds')
    ap.add_argument('--expected-freq', type=float, required=True,
                    help='Expected signal frequency in MHz (e.g. 146.639)')
    ap.add_argument('--freq-offset', type=float, default=None,
                    help='Frequency offset in Hz within decimated baseband '
                         '(default: auto-detect from first segment)')
    ap.add_argument('--port', type=int, default=10000,
                    help='UDP port for decimated IQ (default: 10000)')
    ap.add_argument('--fs', type=float, default=3840.0,
                    help='Decimated sample rate in Hz (default: 3840)')
    ap.add_argument('--threshold', type=float, default=15.0,
                    help='Pulse detection threshold in dB above mean noise; '
                         'single-window STFT noise needs >=15 dB to avoid '
                         'false edges (default: 15)')
    ap.add_argument('--tolerance', type=float, default=0.15,
                    help='Fractional tolerance for A/B classification '
                         '(default: 0.15)')
    ap.add_argument('--tp', type=float, default=0.015,
                    help='Expected pulse width in seconds (default: 0.015)')
    args = ap.parse_args()

    tip_a = args.tip_a
    tip_b = args.tip_b
    Fs = args.fs

    # STFT geometry (same as pulse_detector.py)
    n_w = int(np.ceil(args.tp * Fs))
    if n_w < 4:
        sys.exit(f'Error: STFT window too short (n_w={n_w})')
    n_ol = n_w // 2
    n_ws = n_w - n_ol
    time_step_s = n_ws / Fs

    print('=== Inter-Pulse Interval Analyzer ===')
    print(f'  Rate A (resting)  {tip_a:.3f} s')
    print(f'  Rate B (moving)   {tip_b:.3f} s')
    print(f'  Expected freq     {args.expected_freq:.6f} MHz')
    print(f'  Sample rate       {Fs:.1f} Hz')
    print(f'  STFT              window={n_w}  overlap={n_ol}  step={n_ws}')
    print(f'  Time resolution   {time_step_s*1000:.2f} ms per STFT window')
    print(f'  Threshold         {args.threshold:.1f} dB above noise')
    print(f'  Tolerance         {args.tolerance*100:.0f}%')
    print(f'  Pulse width       {args.tp*1000:.1f} ms')
    print(f'  UDP port          {args.port}')
    if args.freq_offset is not None:
        print(f'  Freq offset       {args.freq_offset:.1f} Hz (manual)')
    else:
        print(f'  Freq offset       auto-detect')
    print()

    # UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    except OSError:
        pass
    sock.bind(('0.0.0.0', args.port))
    sock.settimeout(2.0)

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    # Auto-detect frequency from first N seconds of data
    freq_hz = args.freq_offset
    autodetect_buf = []
    # Need several pulses in the FFT for the tag to beat noise peaks
    autodetect_samples = int(5.0 * max(tip_a, tip_b) * Fs)
    autodetect_done = freq_hz is not None

    tracker = PulseEdgeTracker(
        time_step_s,
        threshold_db=args.threshold,
        min_pulse_windows=1,
        max_pulse_windows=max(int(args.tp * 5.0 / time_step_s), 3),
        # Suppresses tail re-crossings of one pulse, but also hides any real
        # interval shorter than half the fast rate (e.g. 750 ms at 1.5 s).
        refractory_windows=int(0.5 * min(tip_a, tip_b) / time_step_s),
    )

    # Samples not yet covered by a full STFT window; carried to the next packet
    carry = np.zeros(0, dtype=np.complex64)
    stream_t0_ns = None  # wall-clock of the first IQ sample (STFT index 0)
    expected_ts_ns = None  # where the next packet should start if no loss
    # UDP loss shows up as a jump in the packet timestamp. Anything beyond one
    # STFT step would misplace every later timestamp and shorten the IPI that
    # spans it, so re-anchor and drop the in-progress pulse.
    gap_tol_ns = time_step_s * 1e9
    n_gaps = 0

    # Stats
    counts = {'A': 0, 'B': 0, 'ANOM': 0}
    all_ipis = []
    run_start = time.monotonic()
    last_ipi_label = None
    transition_log = []  # (pulse_number, from_label, to_label, ipi_s)

    def report(ipi_s, rise_idx):
        nonlocal last_ipi_label
        last_ipi_label = _report_ipi(
            ipi_s, rise_idx, stream_t0_ns, time_step_s, tip_a, tip_b,
            args.tolerance, counts, all_ipis, transition_log, last_ipi_label)

    print('Waiting for data ...\n')
    print(f'{"#":>5}  {"time":>8}  {"IPI (s)":>9}  {"class":>5}  '
          f'{"delta_A":>9}  {"delta_B":>9}  notes')
    print('-' * 80)

    try:
        while not _should_stop:
            try:
                data = sock.recv(65536)
            except socket.timeout:
                continue
            if len(data) < 16:
                continue

            try:
                ts = decode_timestamp(data[:8])
            except (struct.error, ValueError):
                continue
            if stream_t0_ns is None:
                stream_t0_ns = ts

            payload = data[8:]
            n_samp = len(payload) // 8
            if n_samp == 0:
                continue

            if expected_ts_ns is not None and ts > 1e9:
                gap_ns = ts - expected_ts_ns
                if abs(gap_ns) > gap_tol_ns:
                    n_gaps += 1
                    print(f'  *** GAP {gap_ns / 1e6:+.1f} ms in IQ stream '
                          f'(#{n_gaps}); next interval discarded ***',
                          flush=True)
                    # Keep later timestamps honest: index 0 now maps to a
                    # later wall-clock instant by the missing span plus the
                    # received-but-unwindowed carry we are discarding.
                    stream_t0_ns += gap_ns + int(len(carry) / Fs * 1e9)
                    carry = np.zeros(0, dtype=np.complex64)
                    tracker.reset_after_gap()
            expected_ts_ns = ts + int(n_samp / Fs * 1e9)

            iq = np.frombuffer(payload[:n_samp * 8],
                               dtype=np.complex64).copy()

            # Auto-detect frequency if needed
            if not autodetect_done:
                autodetect_buf.append(iq)
                total = sum(len(b) for b in autodetect_buf)
                if total < autodetect_samples:
                    continue
                combined = np.concatenate(autodetect_buf)
                freq_hz = find_peak_frequency_fft(combined, Fs)
                autodetect_buf.clear()
                abs_mhz = args.expected_freq + freq_hz / 1e6
                print(f'  Auto-detected signal at {abs_mhz:.6f} MHz '
                      f'({freq_hz:+.1f} Hz offset)\n', flush=True)
                autodetect_done = True

                # Process the autodetect buffer through STFT power + tracker
                power_ts, carry = stft_stream_step(carry, combined, Fs, freq_hz,
                                                   n_w, n_ol)
                for ipi_s, rise_idx in tracker.process(power_ts):
                    report(ipi_s, rise_idx)
                continue

            # Normal streaming: STFT power at target freq → edge tracker
            power_ts, carry = stft_stream_step(carry, iq, Fs, freq_hz, n_w, n_ol)
            if len(power_ts) == 0:
                continue
            for ipi_s, rise_idx in tracker.process(power_ts):
                report(ipi_s, rise_idx)

    except KeyboardInterrupt:
        pass
    finally:
        elapsed = time.monotonic() - run_start
        sock.close()
        _print_summary(elapsed, counts, all_ipis, tip_a, tip_b,
                        transition_log)
        if n_gaps:
            print(f'  IQ stream gaps: {n_gaps} (one interval discarded each)\n',
                  flush=True)


def _report_ipi(ipi_s, rise_idx, stream_t0_ns, time_step_s, tip_a, tip_b,
                tolerance, counts, all_ipis, transition_log, last_label):
    """Print and record one IPI. Returns its label."""
    label = classify_ipi(ipi_s, tip_a, tip_b, tolerance)
    n = len(all_ipis) + 1
    notes = ''
    if last_label is not None and label != last_label:
        notes = f'  *** {last_label}→{label} ***'
        transition_log.append((n, last_label, label, ipi_s))
    ts_str = ''
    if stream_t0_ns and stream_t0_ns > 1e9:
        pulse_ns = stream_t0_ns + rise_idx * time_step_s * 1e9
        ts_str = datetime.datetime.fromtimestamp(
            pulse_ns / 1e9, tz=datetime.timezone.utc
        ).strftime('%H:%M:%S')
    print(f'{n:5d}  {ts_str:>8}  {ipi_s:9.4f}  '
          f'{label:>5}  {ipi_s - tip_a:+9.4f}  {ipi_s - tip_b:+9.4f}'
          f'{notes}', flush=True)
    counts[label] += 1
    all_ipis.append(ipi_s)
    return label


def _print_summary(elapsed, counts, all_ipis, tip_a, tip_b, transition_log):
    total = sum(counts.values())
    print(f'\n{"="*80}')
    print(f'=== Summary ({elapsed:.0f} s, {total} intervals) ===')
    print(f'  Rate A intervals:  {counts["A"]:4d}  '
          f'(expected {tip_a:.3f} s)')
    print(f'  Rate B intervals:  {counts["B"]:4d}  '
          f'(expected {tip_b:.3f} s)')
    print(f'  Anomalous:         {counts["ANOM"]:4d}')

    if all_ipis:
        ipis = np.array(all_ipis)
        print(f'\n  IPI stats:  min={ipis.min():.4f}  max={ipis.max():.4f}  '
              f'mean={ipis.mean():.4f}  std={ipis.std():.4f} s')

        # Separate stats for A and B
        a_ipis = ipis[np.abs(ipis - tip_a) / tip_a <= 0.15]
        b_ipis = ipis[np.abs(ipis - tip_b) / tip_b <= 0.15]
        if len(a_ipis) > 0:
            print(f'  Rate A:     min={a_ipis.min():.4f}  '
                  f'max={a_ipis.max():.4f}  mean={a_ipis.mean():.4f}  '
                  f'std={a_ipis.std():.4f} s  (n={len(a_ipis)})')
        if len(b_ipis) > 0:
            print(f'  Rate B:     min={b_ipis.min():.4f}  '
                  f'max={b_ipis.max():.4f}  mean={b_ipis.mean():.4f}  '
                  f'std={b_ipis.std():.4f} s  (n={len(b_ipis)})')

    if transition_log:
        print(f'\n  Transitions detected:')
        for pulse_n, from_l, to_l, ipi_s in transition_log:
            print(f'    #{pulse_n:4d}  {from_l}→{to_l}  '
                  f'IPI={ipi_s:.4f} s')
    else:
        print(f'\n  No rate transitions detected.')

    # List all anomalous IPIs
    anom_ipis = [(i+1, v) for i, v in enumerate(all_ipis)
                 if classify_ipi(v, tip_a, tip_b) == 'ANOM']
    if anom_ipis:
        print(f'\n  Anomalous intervals:')
        for n, v in anom_ipis:
            print(f'    #{n:4d}  IPI={v:.4f} s  '
                  f'(delta_A={v-tip_a:+.4f}  delta_B={v-tip_b:+.4f})')

    print()


if __name__ == '__main__':
    main()
