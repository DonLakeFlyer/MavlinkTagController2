import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from collection_control import (  # noqa: E402
    ArmResult,
    CollectionControl,
    ComputeProgressPolicy,
    SliceProgressPolicy,
    StatusRepeat,
    handle_control_packet,
)
from detector_protocol import MessageType, ProtocolError, encode_arm, encode_header  # noqa: E402


def test_arm_capture_and_rearm():
    control = CollectionControl()

    assert not control.armed
    assert control.arm(10, 1) == ArmResult.ARMED
    assert control.armed
    assert control.active_ids == (10, 1)

    # Captured: the next heading can be armed while this one is analysed.
    assert control.captured(10, 1)
    assert not control.armed

    assert control.arm(10, 2) == ArmResult.ARMED
    assert control.active_ids == (10, 2)


def test_duplicate_arm_is_idempotent():
    control = CollectionControl()

    assert control.arm(10, 4) == ArmResult.ARMED
    assert control.arm(10, 4) == ArmResult.DUPLICATE
    assert control.active_ids == (10, 4)


def test_rearm_of_captured_slice_is_a_replay_not_a_new_slice():
    # The controller re-ARMs when the GCS retries after a lost status; a
    # detector that already captured the slice must not reopen/recollect.
    control = CollectionControl()
    assert control.arm(10, 4) == ArmResult.ARMED
    assert control.captured(10, 4)

    assert control.arm(10, 4) == ArmResult.ALREADY_CAPTURED
    assert not control.armed

    # only the most recent capture is remembered; the next slice arms normally
    assert control.arm(10, 5) == ArmResult.ARMED
    assert control.captured(10, 5)
    assert control.arm(10, 4) == ArmResult.ARMED  # 4 is no longer "captured"
    control.cancel()
    assert control.arm(10, 5) == ArmResult.ARMED  # cancel forgets captures too


def test_last_ids_falls_back_to_the_last_capture():
    control = CollectionControl()
    assert control.last_ids is None
    control.arm(10, 4)
    assert control.last_ids == (10, 4)
    control.captured(10, 4)
    assert control.active_ids is None
    assert control.last_ids == (10, 4)
    control.arm(10, 5)
    assert control.last_ids == (10, 5)


def test_conflicting_arm_is_rejected_while_collecting():
    control = CollectionControl()

    assert control.arm(10, 4) == ArmResult.ARMED
    assert control.arm(10, 5) == ArmResult.BUSY
    assert control.arm(11, 4) == ArmResult.BUSY
    assert control.active_ids == (10, 4)


def test_stale_capture_does_not_close_active_slice():
    control = CollectionControl()

    control.arm(10, 4)
    assert not control.captured(10, 3)
    assert not control.captured(9, 4)
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


def test_slice_progress_periodic_is_throttled_to_one_hertz():
    policy = SliceProgressPolicy()

    assert policy.periodic_due(100.0)          # first report of a slice goes out at once
    assert not policy.periodic_due(100.5)
    assert not policy.periodic_due(100.99)
    assert policy.periodic_due(101.0)
    assert not policy.periodic_due(101.5)
    assert policy.periodic_due(102.3)          # interval is measured from the last send, not a grid
    assert not policy.periodic_due(103.2)


def test_slice_progress_reset_on_arm_unthrottles_first_report():
    policy = SliceProgressPolicy()
    assert policy.periodic_due(100.0)
    assert not policy.periodic_due(100.2)

    policy.reset()                              # new slice armed
    assert policy.periodic_due(100.3)


def test_slice_progress_final_report_restarts_the_clock():
    policy = SliceProgressPolicy()
    assert policy.periodic_due(100.0)

    # The full-segment report is unconditional; the policy only records it so a
    # periodic report cannot follow it within the interval.
    policy.final_sent(100.4)
    assert not policy.periodic_due(101.0)
    assert policy.periodic_due(101.4)


def test_compute_progress_sends_each_stage_change_and_throttles_within_a_stage():
    policy = ComputeProgressPolicy()

    assert policy.due(1, 100.0)
    assert policy.due(2, 100.1)                # stage change goes out at once
    assert policy.due(3, 100.2)
    assert not policy.due(3, 100.7)
    assert not policy.due(3, 101.1)
    assert policy.due(3, 101.2)                # one second after the last send
    assert not policy.due(3, 101.5)
    assert policy.due(4, 101.6)


def test_status_repeat_sends_one_copy_after_the_delay():
    repeat = StatusRepeat()
    repeat.schedule(MessageType.SLICE_CAPTURED, 10, 4, now=100.0)
    repeat.schedule(MessageType.CYCLE_COMPLETE, 10, 3, now=100.5)

    assert repeat.take_due(100.9) == []
    assert repeat.take_due(101.0) == [(MessageType.SLICE_CAPTURED, 10, 4)]
    assert repeat.take_due(101.2) == []
    assert repeat.take_due(102.0) == [(MessageType.CYCLE_COMPLETE, 10, 3)]
    assert repeat.take_due(110.0) == []
