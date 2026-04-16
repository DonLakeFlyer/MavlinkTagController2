"""Structured logging schema shared between pulse_detector and post_flight_analysis.

Provides a StructuredLogger that writes human-readable text to stdout and
structured JSON Lines (.jsonl) to a sidecar file.  The analyzer reads the
.jsonl — no regex, no fragile coupling to printf format strings.

Entry type constants below define the contract between writer and reader.
Adding a field to an emit() call automatically appears in the .jsonl record.
"""

import json
import sys
from typing import Optional, TextIO

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
SPECTROGRAM_DUMP = 'spectrogram_dump'


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
    """

    def __init__(self, jsonl_path: Optional[str] = None):
        self._jsonl: Optional[TextIO] = None
        if jsonl_path:
            self._jsonl = open(jsonl_path, 'w')

    @property
    def active(self) -> bool:
        """True if a .jsonl file is being written."""
        return self._jsonl is not None

    def emit(self, entry_type: str, human: str, flush: bool = True, **data):
        """Write a log entry.

        *human* is printed to stdout as-is.
        *entry_type* + *data* are written as a JSON object to the .jsonl file.
        """
        print(human, flush=flush)
        if self._jsonl is not None:
            record = {'type': entry_type}
            record.update(data)
            self._jsonl.write(json.dumps(record, default=_json_default) + '\n')
            if flush:
                self._jsonl.flush()

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
    """
    entries = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                entries.append(json.loads(line))
    return entries


def entries_by_type(entries, entry_type: str):
    """Filter entries to those matching *entry_type*."""
    return [e for e in entries if e.get('type') == entry_type]
