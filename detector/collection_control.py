"""State for arming one detector collection slice at a time."""

import threading
from enum import Enum, auto

from detector_protocol import ProtocolError, decode_arm


class ArmResult(Enum):
    ARMED = auto()
    DUPLICATE = auto()
    BUSY = auto()
    # This detector already captured these ids; the controller's re-ARM is a
    # retry after a lost status. Replay SLICE_CAPTURED (and CYCLE_COMPLETE once
    # analysed), don't collect again.
    ALREADY_CAPTURED = auto()


class CollectionControl:
    """A slice is armed from ARM until its segment is captured; its analysis
    then runs in the background and the next ARM is accepted at once."""

    def __init__(self):
        self._active_ids = None
        self._captured_ids = None

    @property
    def armed(self):
        return self._active_ids is not None

    @property
    def active_ids(self):
        return self._active_ids

    @property
    def last_ids(self):
        """The armed ids, else the last captured ones (whose analysis may still run)."""
        return self._active_ids if self._active_ids is not None else self._captured_ids

    def arm(self, collection_id, slice_id):
        requested_ids = (collection_id, slice_id)
        if self._active_ids is None:
            if self._captured_ids == requested_ids:
                return ArmResult.ALREADY_CAPTURED
            self._active_ids = requested_ids
            return ArmResult.ARMED
        if self._active_ids == requested_ids:
            return ArmResult.DUPLICATE
        return ArmResult.BUSY

    def captured(self, collection_id, slice_id):
        if self._active_ids != (collection_id, slice_id):
            return False
        self._captured_ids = self._active_ids
        self._active_ids = None
        return True

    def cancel(self):
        self._active_ids = None
        self._captured_ids = None


class SliceProgressPolicy:
    """When a SLICE_PROGRESS report goes out while a slice is armed.

    The controller turns these into the GCS stall watchdog's evidence, so the
    rules matter: at most one periodic report per INTERVAL_S, and only when the
    caller has just received IQ (a stalled stream must go quiet); one final
    report the moment the segment is full, regardless of the interval, because
    the detector is then silent while it computes.
    """

    INTERVAL_S = 1.0

    def __init__(self):
        self._last_sent = float('-inf')

    def reset(self):
        """New slice armed: the first periodic report is not throttled."""
        self._last_sent = float('-inf')

    def periodic_due(self, now):
        """Returns True (and records the send) if a periodic report is due at *now*."""
        if now - self._last_sent < self.INTERVAL_S:
            return False
        self._last_sent = now
        return True

    def final_sent(self, now):
        """The full-segment report was sent at *now*."""
        self._last_sent = now


class StatusRepeat:
    """A second copy, DELAY_S later, of a one-shot status datagram.

    SLICE_CAPTURED and CYCLE_COMPLETE each gate the GCS: one lost datagram
    would stall the rotation until the GCS cancels it. The controller treats
    the copy as a duplicate. Scheduled from either thread, sent by the capture
    thread.
    """

    DELAY_S = 1.0

    def __init__(self):
        self._lock = threading.Lock()
        self._due = []   # (send_at, message_type, collection_id, slice_id)

    def schedule(self, message_type, collection_id, slice_id, now):
        with self._lock:
            self._due.append((now + self.DELAY_S, message_type, collection_id, slice_id))

    def take_due(self, now):
        """Returns the (message_type, collection_id, slice_id) copies due at *now*."""
        with self._lock:
            due = [entry[1:] for entry in self._due if entry[0] <= now]
            self._due = [entry for entry in self._due if entry[0] > now]
        return due


class ComputeProgressPolicy:
    """When a COMPUTE_PROGRESS report goes out while a slice is analysed.

    The controller freezes the rotation's progress step when an analysing
    detector goes quiet for a few seconds, so reports come from the compute
    thread itself: one at every stage change, otherwise at most one per
    INTERVAL_S.
    """

    INTERVAL_S = 1.0

    def __init__(self):
        self._last_sent = float('-inf')
        self._last_stage = None

    def due(self, stage, now):
        """Returns True (and records the send) if a report for *stage* is due at *now*."""
        if stage == self._last_stage and now - self._last_sent < self.INTERVAL_S:
            return False
        self._last_stage = stage
        self._last_sent = now
        return True


def handle_control_packet(control, packet, expected_tag_id=None):
    """Decode an ARM and apply it. If *expected_tag_id* is given, a packet
    addressed to another detector is rejected before touching state."""
    header, heading_deg = decode_arm(packet)
    if expected_tag_id is not None and header.tag_id not in (0, expected_tag_id):
        raise ProtocolError(
            f'ARM tag_id {header.tag_id} does not match detector tag_id {expected_tag_id}')
    return header, control.arm(header.collection_id, header.slice_id), heading_deg
