"""Recovery feedback cannot revive or extend an expired verification budget."""
import pytest

from v3.adapters.room_cruise_recovery import RoomCruiseRecovery


@pytest.mark.parametrize("feedback_delay_s", [0., 6., 8.])
def test_delayed_first_feedback_keeps_original_verification_deadline(feedback_delay_s):
    recovery = RoomCruiseRecovery((-.1, .26, -.18, .18), .05)
    started = 1_000_000_000
    recovery.begin([1., 0., None], started, "navigation_failed")
    recovery.selected = {"name": "reverse"}
    recovery.action_finished(True, (0., 0., 0.), started)
    original_deadline = recovery.deadline_ns
    received = started+round(feedback_delay_s*1e9)
    recovery.feedback(1., received)
    assert recovery.deadline_ns <= original_deadline
    assert recovery.verify((0., 0., 0.), 1., received, original_deadline) is False
    assert recovery.state == "READY"
    assert recovery.counts["verification_failed"] == 1


def test_fresh_translation_and_goal_progress_resolve_recovery():
    recovery = RoomCruiseRecovery((-.1, .26, -.18, .18), .05)
    recovery.begin([1., 0., None], 1_000_000_000, "navigation_failed")
    recovery.selected = {"name": "reverse"}
    recovery.action_finished(True, (0., 0., 0.), 2_000_000_000)
    recovery.feedback(1., 2_100_000_000)
    assert recovery.verify((.3, 0., 0.), .8, 2_900_000_000, 3_000_000_000) is True
    assert recovery.state == "RESOLVED"


@pytest.mark.parametrize("pose,remaining", [(None, None), ((.3, 0., 0.), .8)])
def test_expired_verification_fails_with_missing_pose_or_late_progress(pose, remaining):
    recovery = RoomCruiseRecovery((-.1, .26, -.18, .18), .05)
    recovery.begin([1., 0., None], 1_000_000_000, "navigation_failed")
    recovery.selected = {"name": "reverse"}
    recovery.action_finished(True, (0., 0., 0.), 2_000_000_000)
    recovery.feedback(1., 2_100_000_000)
    assert recovery.verify(pose, remaining, recovery.deadline_ns, recovery.deadline_ns) is False
    assert recovery.state == "READY"
