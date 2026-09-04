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
import sys
from typing import List, Optional, Sequence, TextIO

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
EVT_THRESHOLD    = 'evt_threshold'
HYPOTHESIS       = 'hypothesis'
SESSION_END      = 'session_end'
STFT_DEBUG       = 'stft_debug'


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

class StructuredLogger:
    """Dual-output logger: human text to stdout, structured JSON to .jsonl.

    Usage::

        log = StructuredLogger(jsonl_path='/tmp/det.jsonl')
        log.emit(DETECTION, f'[{cycle}] DETECTED ...', cycle=1, snr_db=34.2, ...)
        log.close()

    If *jsonl_path* is None, only stdout output is produced (backward compat).

    Entry types listed in *preamble_types* are remembered and replayed at the
    top of every file opened via :meth:`reopen`, so each file is self-contained.
    """

    def __init__(self, jsonl_path: Optional[str] = None,
                 preamble_types: Sequence[str] = ()):
        self._jsonl: Optional[TextIO] = None
        self._preamble_types = frozenset(preamble_types)
        self._preamble: List[str] = []
        if jsonl_path:
            self._jsonl = open(jsonl_path, 'w',
                               encoding='utf-8', newline='\n')

    def reopen(self, jsonl_path: str):
        """Switch output to a new .jsonl at *jsonl_path*, replaying the preamble.

        The current file stays open until the new one is ready, so a failed
        reopen raises OSError and leaves logging untouched.
        """
        new_file = open(jsonl_path, 'w', encoding='utf-8', newline='\n')
        try:
            for line in self._preamble:
                new_file.write(line)
            new_file.flush()
        except OSError:
            new_file.close()
            raise
        self.close()
        self._jsonl = new_file

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
        if keep_for_preamble:
            self._preamble.append(line)
        if self._jsonl is not None:
            try:
                self._jsonl.write(line)
                if flush:
                    self._jsonl.flush()
            except OSError as exc:
                # Losing log storage (e.g. ENOSPC) must not stop detection.
                print(f'WARNING: structured log write failed ({exc}); '
                      f'continuing stdout-only', file=sys.stderr, flush=True)
                try:
                    self._jsonl.close()
                except OSError:
                    pass
                self._jsonl = None

    def emit_raw(self, human: str, flush: bool = True):
        """Write a human-only line (not recorded in .jsonl)."""
        print(human, flush=flush)

    def close(self):
        """Flush and close the .jsonl file."""
        if self._jsonl is not None:
            self._jsonl.close()
            self._jsonl = None


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
