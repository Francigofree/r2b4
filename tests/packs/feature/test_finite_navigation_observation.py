"""Finite public actions expose admission identity before their completion wait."""
from __future__ import annotations

import threading
from contextlib import contextmanager
from types import SimpleNamespace

import pytest

from v3.adapters.v3_control import V3ControlInterfaceAdapter
from v3.contracts.localization import LOCAL_FRAME_ID
from v3.robot_interface import RobotInterface


class Controller:
    def __init__(self):
        self.locked = False
        self.admitted = False
        self.observations = []
        self.order = []
        self.navigate_parameters = None

    def status(self):
        return {"runtime_running": True}

    def snapshot(self):
        return SimpleNamespace(runtime_pid=123)

    @contextmanager
    def finite_motion_transaction(self, **parameters):
        self.locked = True
        try:
            yield SimpleNamespace(runtime_pid=123)
        finally:
            self.locked = False

    def navigate(self, **parameters):
        assert self.locked
        self.admitted = True
        self.navigate_parameters = parameters
        self.order.append("ADMITTED")
        return {"command_id": "finite-1"}

    def live_runtime_status(self):
        status = {
            "monotonic_ns": 123456, "state": "RUNNING", "ready_for_active": True,
            "fault_layer": None, "safety_decision": "ALLOW", "safety_reason": "ALLOW",
            "estimate": {
                "local_pose": {"frame_id": LOCAL_FRAME_ID, "x_m": 1.0, "y_m": 2.0, "yaw_rad": 0.0},
                "localization_quality": {"generation": 7},
            },
        }
        if self.admitted:
            assert self.observations, "completion wait began before passive admission publication"
            self.order.append("WAIT")
            status.update(
                mission={"mission_id": "mission-finite-1", "mode": "NAVIGATE", "lifecycle": "COMPLETED"},
                navigation={"mission_id": "mission-finite-1", "status": "COMPLETE", "reason": "COMPLETE"},
                safety_decision="STOP", safety_reason="NOT_ACTIVE",
            )
        return status

    def stop(self):
        self.order.append("STOP")


@pytest.mark.parametrize("action, parameters", [
    ("v3.command.move_relative", {"forward_m": 0.5}),
    ("v3.command.turn_by", {"angle_deg": 90}),
    ("v3.command.navigate", {"x_m": 3.0, "y_m": 4.0, "frame_id": LOCAL_FRAME_ID,
                             "wait_for_completion": True}),
])
@pytest.mark.parametrize("sink_fails", [False, True])
def test_real_finite_execution_publishes_early_compact_lineage_and_tolerates_sink_failure(
    action, parameters, sink_fails,
):
    controller = Controller()
    robot = RobotInterface(controller=controller, adapters=(V3ControlInterfaceAdapter(controller),), upper_runtime=False)

    def observe(admission):
        assert not controller.locked, "passive publication must happen outside the operator transaction"
        controller.order.append("OBSERVATION")
        controller.observations.append(admission)
        if sink_fails:
            raise OSError("observation unavailable")

    result = robot.execute(action, **parameters, admission_sink=observe)
    admission, = controller.observations
    assert admission == {
        "command_id": "finite-1", "mission_id": "mission-finite-1", "frame_id": LOCAL_FRAME_ID,
        "runtime_pid": 123, "localization_generation": 7, "pose_status_monotonic_ns": 123456,
    }
    assert controller.order == ["ADMITTED", "OBSERVATION", "WAIT", "STOP"]
    assert result["status"] == "COMPLETED"
    assert result["command_id"] == admission["command_id"]
    assert result["mission_id"] == admission["mission_id"]
    assert result["frame_provenance"]["pose_status_monotonic_ns"] == admission["pose_status_monotonic_ns"]
    assert "admission_sink" not in controller.navigate_parameters
    assert "admission_sink" not in result["requested"]


def test_pre_admission_cancellation_never_claims_a_command_identity():
    controller = Controller()
    robot = RobotInterface(controller=controller, adapters=(V3ControlInterfaceAdapter(controller),), upper_runtime=False)
    cancelled = threading.Event()
    cancelled.set()
    result = robot.execute("v3.command.move_relative", forward_m=0.5, cancel_event=cancelled,
                           admission_sink=controller.observations.append)
    assert result["reason"] == "CANCELLED"
    assert result["command_id"] is None
    assert controller.observations == []
    assert controller.order == ["STOP"]
