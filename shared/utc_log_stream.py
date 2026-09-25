"""Prefix every stdout/stderr line with a UTC wall-clock stamp.

The controller logs in UTC (HH:MM:SS); child processes redirected to their own
log files carry no clock of their own, so post-flight the two cannot be lined
up. Installing this on both streams gives ``HH:MM:SS.mmm <original line>``.
"""

import datetime
import sys
import threading


class UtcLinePrefixStream:
    def __init__(self, stream):
        self._stream = stream
        self._at_line_start = True
        self._lock = threading.Lock()

    def write(self, text):
        if not text:
            return 0
        with self._lock:
            out = []
            for piece in text.splitlines(keepends=True):
                if self._at_line_start:
                    now = datetime.datetime.now(datetime.timezone.utc)
                    out.append(now.strftime('%H:%M:%S') + f'.{now.microsecond // 1000:03d} ')
                out.append(piece)
                self._at_line_start = piece.endswith('\n')
            self._stream.write(''.join(out))
            return len(text)

    def flush(self):
        self._stream.flush()

    def __getattr__(self, name):
        return getattr(self._stream, name)


def install_utc_line_prefix():
    if not isinstance(sys.stdout, UtcLinePrefixStream):
        sys.stdout = UtcLinePrefixStream(sys.stdout)
    if not isinstance(sys.stderr, UtcLinePrefixStream):
        sys.stderr = UtcLinePrefixStream(sys.stderr)
