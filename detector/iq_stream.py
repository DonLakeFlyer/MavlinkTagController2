"""Continuous IQ timeline for the pulse detector.

Samples are indexed by absolute stream position and are never cleared by
control-plane events. Packet-timestamp discontinuities are handled here:

  * gap < threshold  -> zero-filled, recorded as (offset, n) in stream coords
  * gap >= threshold -> not filled; a *barrier* is placed at the first
                        post-gap sample so no segment may span the hole
  * negative gap     -> treated as a barrier

Old history is released only by an explicit retire(), which is counted.
"""

import collections

import numpy as np


class IqStream:
    def __init__(self, fs, gap_reset_threshold_s, retain_samples, on_gap=None):
        self._fs = float(fs)
        self._reset_ns = int(round(gap_reset_threshold_s * 1e9))
        self._retain = int(retain_samples)
        self._on_gap = on_gap

        self._parts = collections.deque()   # np.ndarray chunks, contiguous from base
        self._fills = []            # (abs_offset, n_zeros)
        self._anchors = []          # (abs_index, ts_ns) — time is linear between

        self.base = 0               # abs index of first retained sample
        self.head = 0               # abs index of the next sample to arrive
        self.barrier = 0            # segments must start at or after this
        self._last_ts = None
        self._last_n = 0

        self.zerofill_count = 0
        self.reset_count = 0
        self.retired_samples = 0

    # -- ingest -------------------------------------------------------------

    def append(self, iq, ts_ns):
        n = int(iq.size)
        if self._last_ts is not None and self._last_n > 0:
            expected = int(round(self._last_n / self._fs * 1e9))
            gap_ns = (ts_ns - self._last_ts) - expected
            if gap_ns < 0:
                self._reset('negative', gap_ns, expected, ts_ns)
            else:
                missing = int(round(gap_ns / (1e9 / self._fs)))
                if missing > 0:
                    if gap_ns >= self._reset_ns:
                        self._reset('reset', gap_ns, expected, ts_ns)
                    else:
                        self._zero_fill(missing, gap_ns)
        if not self._anchors or self._anchors[-1][0] != self.head:
            self._anchors.append((self.head, int(ts_ns)))
        self._parts.append(np.asarray(iq, dtype=np.complex64))
        self.head += n
        self._last_ts = ts_ns
        self._last_n = n

    def _zero_fill(self, missing, gap_ns):
        self.zerofill_count += 1
        self._fills.append((self.head, missing))
        self._parts.append(np.zeros(missing, dtype=np.complex64))
        self.head += missing
        self._emit('zerofill', gap_ms=gap_ns / 1e6, missing_samples=missing,
                   threshold_ms=self._reset_ns / 1e6)

    def _reset(self, kind, gap_ns, expected_ns, ts_ns):
        self.reset_count += 1
        self.barrier = self.head
        self._emit(kind, gap_ms=gap_ns / 1e6, threshold_ms=self._reset_ns / 1e6,
                   expected_delta_ms=expected_ns / 1e6,
                   actual_delta_ms=(ts_ns - self._last_ts) / 1e6,
                   last_ts_ns=self._last_ts, current_ts_ns=ts_ns,
                   barrier=self.barrier)

    def _emit(self, kind, **kw):
        if self._on_gap is not None:
            self._on_gap(kind, **kw)

    # -- read ---------------------------------------------------------------

    def available(self, start):
        return self.head - max(start, self.barrier)

    def take(self, start, n):
        """Return (samples, ts_ns, gap_fills) for [start, start+n)."""
        start = int(start)
        if n <= 0:
            raise ValueError(f'take() requires n > 0, got {n}')
        if start < self.base:
            raise IndexError(f'take({start}) precedes retained base {self.base}')
        if start + n > self.head:
            raise IndexError(f'take({start}, {n}) exceeds head {self.head}')
        if start < self.barrier < start + n:
            raise ValueError(f'take({start}, {n}) spans barrier at {self.barrier}')
        seg = self._gather(start, n)
        fills = []
        for off, cnt in self._fills:
            a, b = max(off, start), min(off + cnt, start + n)
            if b > a:
                fills.append((a - start, b - a))
        return seg, self.time_at(start), fills

    def time_at(self, index):
        anchor_idx, anchor_ts = self._anchors[0]
        for a_idx, a_ts in self._anchors:
            if a_idx > index:
                break
            anchor_idx, anchor_ts = a_idx, a_ts
        return anchor_ts + int(round((index - anchor_idx) * 1e9 / self._fs))

    def _gather(self, start, n):
        """Copy [start, start+n) out of the part list; parts stay as received."""
        out = np.empty(n, dtype=np.complex64)
        pos = self.base
        filled = 0
        for part in self._parts:
            end = pos + part.size
            if end > start and filled < n:
                lo = max(start, pos) - pos
                hi = min(start + n, end) - pos
                out[filled:filled + hi - lo] = part[lo:hi]
                filled += hi - lo
            pos = end
            if filled >= n:
                break
        return out

    # -- release ------------------------------------------------------------

    def retire(self, consumed_to):
        """Drop whole leading parts that lie entirely more than retain_samples
        behind *consumed_to*. base lands on a part boundary at or below the
        target, so retention is a lower bound and nothing is ever copied."""
        target = min(consumed_to - self._retain, self.head)
        while self._parts and self.base + self._parts[0].size <= target:
            n = self._parts.popleft().size
            self.retired_samples += n
            self.base += n
        self._fills = [(o, c) for o, c in self._fills if o + c > self.base]
        keep = [a for a in self._anchors if a[0] >= self.base]
        if self._anchors and (not keep or keep[0][0] != self.base):
            keep.insert(0, (self.base, self.time_at(self.base)))
        self._anchors = keep
