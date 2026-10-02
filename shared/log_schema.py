"""Structured logging schema shared between pulse_detector and post_flight_analysis.

Provides a StructuredLogger that writes human-readable text to stdout and
structured JSON Lines (.jsonl) to a sidecar file.  The analyzer reads the
.jsonl — no regex, no fragile coupling to printf format strings.

Contract: the .jsonl carries the *analyzable events* named by the entry type
constants below.  Free-form diagnostics (EVT trial progress, weighting-matrix
checks, per-hypothesis detail, malformed-packet notices, ...) are stdout-only
via emit_raw()/print and are intentionally not part of the schema.
Adding a field to an emit() call automatically appears in the .jsonl record.
"""

import json
import os
import sys
import threading
from typing import Dict, List, Optional, Sequence, TextIO

# ---------------------------------------------------------------------------
# Entry type constants — shared between detector (writer) and analyzer (reader)
# ---------------------------------------------------------------------------

STARTUP          = 'startup'
DETECTION        = 'detection'
NO_DETECTION     = 'no_detection'
FOLDS            = 'folds'
TIMING           = 'timing'
NOISE_STATS      = 'noise_stats'
NOISE_ELEVATED   = 'noise_elevated'
GAP_EVENT        = 'gap_event'
# One-off threshold notes (detection margin); replayed as preamble.
EVT_THRESHOLD    = 'evt_threshold'
# Per-cycle data-derived threshold: Gumbel (mu, sigma) of the window-permutation
# null, n_perm, base threshold, impulse-blanked IQ fraction. One record per
# dwell, so per-heading threshold drift is visible.
CYCLE_THRESHOLD  = 'cycle_threshold'
HYPOTHESIS       = 'hypothesis'
SESSION_END      = 'session_end'
STFT_DEBUG       = 'stft_debug'
# Lock-candidate bank: admission/merge/lock events and per-candidate
# fixed-coordinate measurements (issue #134). DETECTION records carry only
# the provisional lock (candidate 0) so existing per-heading summaries are
# unchanged.
LOCK_CANDIDATE   = 'lock_candidate'
CANDIDATE_MEASUREMENT = 'candidate_measurement'


# ---------------------------------------------------------------------------
# JSON serialization helper
# ---------------------------------------------------------------------------

def _json_default(obj):
    """Handle numpy and other non-JSON-native types."""
    try:
        import numpy as np
        if isinstance(obj, np.integer):
            return int(obj)
        if isinstance(obj, np.floating):
            return float(obj)
        if isinstance(obj, np.ndarray):
            return obj.tolist()
    except ImportError:
        pass
    return str(obj)


# ---------------------------------------------------------------------------
# StructuredLogger
# ---------------------------------------------------------------------------

# Shared by forked loggers: the preamble and whole-line .jsonl writes.
_WRITE_LOCK = threading.Lock()


class _Preamble:
    def __init__(self):
        self.lines: List[str] = []
        self.written: Dict[str, int] = {}   # path -> preamble lines already in that file


class StructuredLogger:
    """Dual-output logger: human text to stdout, structured JSON to .jsonl.

    Usage::

        log = StructuredLogger(jsonl_path='/tmp/det.jsonl')
        log.emit(DETECTION, f'[{cycle}] DETECTED ...', cycle=1, snr_db=34.2, ...)
        log.close()

    If *jsonl_path* is None, only stdout output is produced (backward compat).

    Entry types listed in *preamble_types* are remembered and replayed at the
    top of every file opened via :meth:`reopen`, so each file is self-contained.
    One recorded after a file was opened is written there before its next
    record, by whichever logger (this one or a fork) writes to it.

    Files are written in append mode (after truncation on a fresh open), and
    every .jsonl line is written and flushed under one lock, so a second
    logger from :meth:`fork` may append whole lines to the same file from
    another thread. stdout is not locked.
    """

    def __init__(self, jsonl_path: Optional[str] = None,
                 preamble_types: Sequence[str] = ()):
        self._jsonl: Optional[TextIO] = None
        self._path: Optional[str] = None
        self._preamble_types = frozenset(preamble_types)
        self._preamble = _Preamble()
        if jsonl_path:
            self._jsonl = self._open(jsonl_path, truncate=True)
            self._path = os.path.abspath(jsonl_path)
            self._preamble.written[self._path] = 0

    @staticmethod
    def _open(path: str, truncate: bool) -> TextIO:
        if truncate:
            open(path, 'w', encoding='utf-8').close()
        return open(path, 'a', encoding='utf-8', newline='\n')

    def fork(self) -> 'StructuredLogger':
        """A logger with no file of its own that shares this one's preamble."""
        other = StructuredLogger(preamble_types=self._preamble_types)
        other._preamble = self._preamble
        return other

    def reopen(self, jsonl_path: str, append: bool = False):
        """Switch output to the .jsonl at *jsonl_path*.

        A fresh file (the default) is truncated and gets the preamble; with
        *append* the file is joined as is. The current file stays open until
        the new one is ready, so a failed reopen raises OSError and leaves
        logging untouched.
        """
        path = os.path.abspath(jsonl_path)
        new_file = self._open(jsonl_path, truncate=not append)
        if not append:
            try:
                with _WRITE_LOCK:
                    for line in self._preamble.lines:
                        new_file.write(line)
                    new_file.flush()
                    self._preamble.written[path] = len(self._preamble.lines)
            except OSError:
                new_file.close()
                raise
        with _WRITE_LOCK:
            self._preamble.written.setdefault(path, len(self._preamble.lines))
            old_file, self._jsonl = self._jsonl, new_file
            self._path = path
        if old_file is not None:
            old_file.close()

    @property
    def active(self) -> bool:
        """True if a .jsonl file is being written."""
        return self._jsonl is not None

    def emit(self, entry_type: str, human: Optional[str], flush: bool = True, **data):
        """Write a log entry.

        *human* is printed to stdout as-is; pass None to record to .jsonl only.
        *entry_type* + *data* are written as a JSON object to the .jsonl file.
        """
        if human is not None:
            print(human, flush=flush)
        keep_for_preamble = entry_type in self._preamble_types
        if self._jsonl is None and not keep_for_preamble:
            return
        record = {'type': entry_type}
        record.update(data)
        line = json.dumps(record, default=_json_default, ensure_ascii=False) + '\n'
        with _WRITE_LOCK:
            preamble = self._preamble
            if keep_for_preamble:
                preamble.lines.append(line)
            if self._jsonl is None:
                return
            # A preamble record is itself the last unwritten preamble line.
            pending = preamble.lines[preamble.written[self._path]:]
            if not keep_for_preamble:
                pending.append(line)
            try:
                self._jsonl.write(''.join(pending))
                preamble.written[self._path] = len(preamble.lines)
                if flush:
                    self._jsonl.flush()
                return
            except OSError as exc:
                failure = exc
                broken, self._jsonl = self._jsonl, None
        # Losing log storage (e.g. ENOSPC) must not stop detection.
        print(f'WARNING: structured log write failed ({failure}); '
              f'continuing stdout-only', file=sys.stderr, flush=True)
        try:
            broken.close()
        except OSError:
            pass

    def emit_raw(self, human: str, flush: bool = True):
        """Write a human-only line (not recorded in .jsonl)."""
        print(human, flush=flush)

    def close(self):
        """Flush and close the .jsonl file."""
        with _WRITE_LOCK:
            old_file, self._jsonl = self._jsonl, None
            if old_file is not None:
                old_file.close()


# ---------------------------------------------------------------------------
# Reader (used by analyzer)
# ---------------------------------------------------------------------------

def read_jsonl(path: str):
    """Read all structured log entries from a .jsonl file.

    Returns a list of dicts, each with at least a ``'type'`` key.
    Malformed lines (e.g. a partial trailing line from a killed writer)
    are skipped with a warning on stderr.
    """
    entries = []
    bad_lines = []
    with open(path, encoding='utf-8') as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                bad_lines.append(lineno)
                continue
            if not isinstance(record, dict):
                bad_lines.append(lineno)  # a bare scalar/list is not a record
                continue
            entries.append(record)
    if bad_lines:
        print(f'WARNING: {path}: skipped {len(bad_lines)} malformed line(s) '
              f'at {bad_lines[:10]}', file=sys.stderr)
    return entries


def entries_by_type(entries, entry_type: str):
    """Filter entries to those matching *entry_type*."""
    return [e for e in entries if e.get('type') == entry_type]
