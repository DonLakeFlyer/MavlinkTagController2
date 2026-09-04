"""State for arming one detector collection slice at a time."""

from enum import Enum, auto

from detector_protocol import ProtocolError, decode_arm


class ArmResult(Enum):
    ARMED = auto()
    DUPLICATE = auto()
    BUSY = auto()
    # This detector already completed these ids; the controller's re-ARM is a
    # retry on behalf of a slower detector. Replay CYCLE_COMPLETE, don't collect.
    ALREADY_COMPLETE = auto()


class CollectionControl:
    def __init__(self):
        self._active_ids = None
        self._completed_ids = None

    @property
    def armed(self):
        return self._active_ids is not None

    @property
    def active_ids(self):
        return self._active_ids

    def arm(self, collection_id, slice_id):
        requested_ids = (collection_id, slice_id)
        if self._active_ids is None:
            if self._completed_ids == requested_ids:
                return ArmResult.ALREADY_COMPLETE
            self._active_ids = requested_ids
            return ArmResult.ARMED
        if self._active_ids == requested_ids:
            return ArmResult.DUPLICATE
        return ArmResult.BUSY

    def complete(self, collection_id, slice_id):
        if self._active_ids != (collection_id, slice_id):
            return False
        self._completed_ids = self._active_ids
        self._active_ids = None
        return True

    def cancel(self):
        self._active_ids = None
        self._completed_ids = None


def handle_control_packet(control, packet, expected_tag_id=None):
    """Decode an ARM and apply it. If *expected_tag_id* is given, a packet
    addressed to another detector is rejected before touching state."""
    header, heading_deg = decode_arm(packet)
    if expected_tag_id is not None and header.tag_id not in (0, expected_tag_id):
        raise ProtocolError(
            f'ARM tag_id {header.tag_id} does not match detector tag_id {expected_tag_id}')
    return header, control.arm(header.collection_id, header.slice_id), heading_deg
