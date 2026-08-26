from contextlib import ExitStack
from types import SimpleNamespace
from unittest.mock import patch

from controller.commands import set_twist


def _controller():
    return SimpleNamespace(
        motion_task_status={},
        waypoint_mission_status={},
        motion_contract_status={},
        input_vector={},
    )


def _set_twist_without_unrelated_layers(ctrl, v, omega):
    patches = {
        "set_motion_source": {"return_value": True},
        "_preempt_waypoint_mission": {},
        "_note_motion_command_activity": {},
        "_clear_all_explicit_motion_layers": {},
        "_set_requested_motion_intent": {},
        "_set_requested_track_reference": {},
        "_clear_motion_public_target": {},
        "_set_active_motion_command": {},
        "_apply_motion_state_from_twist": {},
    }
    with ExitStack() as stack:
        for name, kwargs in patches.items():
            stack.enter_context(patch(f"controller.commands.{name}", **kwargs))
        return set_twist(ctrl, v, omega, source="STATE")


def test_nonzero_twist_starts_running_motion_task():
    ctrl = _controller()

    assert _set_twist_without_unrelated_layers(ctrl, 0.15, 0.0)

    task = dict(ctrl.motion_task_status)
    assert task["command_type"] == "set_twist"
    assert task["execution_state"] == "running"
    assert task["terminal_reason"] == ""
    assert task["task_id"]


def test_zero_twist_closes_new_task_as_successful_stop():
    ctrl = _controller()
    _set_twist_without_unrelated_layers(ctrl, 0.15, 0.0)
    moving_task_id = str(ctrl.motion_task_status["task_id"])

    assert _set_twist_without_unrelated_layers(ctrl, 0.0, 0.0)

    task = dict(ctrl.motion_task_status)
    assert task["task_id"] != moving_task_id
    assert task["command_type"] == "set_twist"
    assert task["execution_state"] == "succeeded"
    assert task["terminal_reason"] == "SEGMENT_COMPLETED"
    assert task["retryable"] is False
    assert task["details"] == {
        "v_target": 0.0,
        "omega_target": 0.0,
        "zero_twist_stop": True,
    }


def test_zero_linear_nonzero_angular_twist_remains_running():
    ctrl = _controller()

    assert _set_twist_without_unrelated_layers(ctrl, 0.0, 0.2)

    assert ctrl.motion_task_status["execution_state"] == "running"
    assert ctrl.motion_task_status["terminal_reason"] == ""
