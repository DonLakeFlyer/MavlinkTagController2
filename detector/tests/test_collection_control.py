import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from collection_control import (  # noqa: E402
    ArmResult,
    CollectionControl,
    handle_control_packet,
)
from detector_protocol import MessageType, ProtocolError, encode_arm, encode_header  # noqa: E402


def test_arm_complete_and_rearm():
    control = CollectionControl()

    assert not control.armed
    assert control.arm(10, 1) == ArmResult.ARMED
    assert control.armed
    assert control.active_ids == (10, 1)

    assert control.complete(10, 1)
    assert not control.armed

    assert control.arm(10, 2) == ArmResult.ARMED
    assert control.active_ids == (10, 2)


def test_duplicate_arm_is_idempotent():
    control = CollectionControl()

    assert control.arm(10, 4) == ArmResult.ARMED
    assert control.arm(10, 4) == ArmResult.DUPLICATE
    assert control.active_ids == (10, 4)


def test_rearm_of_completed_slice_is_a_replay_not_a_new_slice():
    # Controller re-ARMs everyone when the GCS retries for a slower detector;
    # a detector that already finished must not reopen/recollect.
    control = CollectionControl()
    assert control.arm(10, 4) == ArmResult.ARMED
    assert control.complete(10, 4)

    assert control.arm(10, 4) == ArmResult.ALREADY_COMPLETE
    assert not control.armed

    # only the most recent completion is remembered; the next slice arms normally
    assert control.arm(10, 5) == ArmResult.ARMED
    assert control.complete(10, 5)
    assert control.arm(10, 4) == ArmResult.ARMED  # 4 is no longer "completed"
    control.cancel()
    assert control.arm(10, 5) == ArmResult.ARMED  # cancel forgets completions too


def test_conflicting_arm_is_rejected_while_collecting():
    control = CollectionControl()

    assert control.arm(10, 4) == ArmResult.ARMED
    assert control.arm(10, 5) == ArmResult.BUSY
    assert control.arm(11, 4) == ArmResult.BUSY
    assert control.active_ids == (10, 4)


def test_stale_completion_does_not_close_active_slice():
    control = CollectionControl()

    control.arm(10, 4)
    assert not control.complete(10, 3)
    assert not control.complete(9, 4)
    assert control.active_ids == (10, 4)


def test_cancel_returns_to_idle():
    control = CollectionControl()
    control.arm(10, 4)

    control.cancel()

    assert not control.armed
    assert control.active_ids is None


def test_arm_packet_updates_collection_state():
    control = CollectionControl()
    packet = encode_arm(10, 4, 42, heading_deg=45.0)

    header, result, heading_deg = handle_control_packet(control, packet)

    assert header.tag_id == 42
    assert result == ArmResult.ARMED
    assert heading_deg == 45.0
    assert control.active_ids == (10, 4)


def test_non_arm_control_packet_is_rejected():
    control = CollectionControl()
    packet = encode_header(MessageType.READY, 0, 10, 4, 42)

    try:
        handle_control_packet(control, packet)
    except ProtocolError as error:
        assert 'ARM' in str(error)
    else:
        raise AssertionError('non-ARM control packet was accepted')


def test_arm_for_another_tag_does_not_touch_state():
    # A misdirected/delayed ARM for a different detector must be rejected
    # before it can arm, re-arm or cancel this detector's slice.
    control = CollectionControl()
    assert control.arm(10, 4) == ArmResult.ARMED

    try:
        handle_control_packet(control, encode_arm(10, 5, 99, 0.0), expected_tag_id=42)
    except ProtocolError as error:
        assert 'tag_id 99' in str(error)
    else:
        raise AssertionError('ARM for another tag was accepted')
    assert control.active_ids == (10, 4)

    # tag_id 0 is the broadcast form and is still accepted
    _, result, _ = handle_control_packet(control, encode_arm(10, 4, 0, 0.0), expected_tag_id=42)
    assert result == ArmResult.DUPLICATE
