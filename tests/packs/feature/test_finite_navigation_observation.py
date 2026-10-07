"""Finite public actions expose admission identity before their completion wait."""
from __future__ import annotations

import threading
from contextlib import contextmanager
from types import SimpleNamespace

import pytest
from rig import resolved_config

from v3.adapters.v3_control import V3ControlInterfaceAdapter
from v3.contracts.localization import LOCAL_FRAME_ID
from v3.robot_interface import RobotInterface


class Controller:
    def __init__(self):
        self.resolved = resolved_config()
        self.effective_config = self.resolved.as_dict()
        self.locked = False
        self.admitted = False
        self.observations = []
        self.order = []
        self.navigate_parameters = None

    def status(self):
        return {"runtime_running": True}

    def current_capture_mode(self):
        return "full"

    def current_capture_hz(self):
        return 10

    def snapshot(self):
        return SimpleNamespace(runtime_pid=123)

    @contextmanager
    def finite_motion_transaction(self, **parameters):
        self.capture_parameters = parameters
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
            "effective_config": self.effective_config,
            "config_snapshot_id": self.resolved.snapshot_id,
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
            if self.navigate_parameters is not None:
                status["estimate"]["local_pose"].update({key: self.navigate_parameters[key]
                    for key in ("x_m", "y_m", "yaw_rad") if key in self.navigate_parameters})
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
    assert result["distance_remaining_m"] == 0
    assert controller.capture_parameters["capture_mode"] == "full"
    assert controller.capture_parameters["capture_hz"] == 10


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


def test_stalled_navigation_is_bounded_and_never_completed(monkeypatch):
    from v3.finite_navigation import FiniteNavigationExecutor
    controller = Controller()
    controller.admitted = True
    controller.observations.append({})
    status = controller.live_runtime_status()
    status.update(safety_decision="ALLOW", safety_reason="ALLOW")
    status["mission"]["lifecycle"] = "ACTIVE"
    status["navigation"].update(status="ACTIVE", reason="NO_PROGRESS_VIABLE_TRAJECTORY")
    monkeypatch.setattr(controller, "live_runtime_status", lambda: status)
    clock = [0.0]
    executor = FiniteNavigationExecutor(controller, monotonic=lambda: clock[0],
        sleep=lambda dt: clock.__setitem__(0, clock[0] + dt), progress_timeout_s=.2)
    result = executor.execute("v3.command.move_relative", {"forward_m": 1})
    assert result["reason"] == "NAVIGATION_STALLED" and result["status"] == "INTERRUPTED"
    assert result["elapsed_s"] < .3
    assert controller.order[-1] == "STOP"


def test_stop_error_preserves_command_identity_and_original_error(monkeypatch):
    from v3.finite_navigation import FiniteNavigationSafetyError
    controller = Controller()
    events = []
    robot = RobotInterface(controller=controller, adapters=(V3ControlInterfaceAdapter(controller),),
                           upper_runtime=False, observation_sink=events.append)
    def broken_stop():
        raise RuntimeError("IDLE confirmation failed")
    monkeypatch.setattr(controller, "stop", broken_stop)
    with pytest.raises(FiniteNavigationSafetyError, match="IDLE confirmation failed"):
        robot.execute("v3.command.move_relative", forward_m=1, admission_sink=controller.observations.append)
    error = events[-1]
    assert error.kind == "ACTION_ERROR"
    assert error.command_id == "finite-1" and error.mission_id == "mission-finite-1"
    assert error.runtime_pid == 123 and error.status == "INTERRUPTED"
    assert error.error_cause_type == "RuntimeError"
    assert "IDLE confirmation failed" in error.error_message


def test_frame_generation_change_never_produces_comparable_distance_evidence():
    class ChangedFrameController(Controller):
        def live_runtime_status(self):
            status = super().live_runtime_status()
            if self.admitted:
                status["estimate"]["localization_quality"]["generation"] = 8
            return status
    controller = ChangedFrameController()
    robot = RobotInterface(controller=controller, adapters=(V3ControlInterfaceAdapter(controller),), upper_runtime=False)
    result = robot.execute("v3.command.move_relative", forward_m=.5, admission_sink=controller.observations.append)
    assert result["status"] == "INTERRUPTED" and result["reason"] == "LOCALIZATION_FRAME_CHANGED"
    assert result["distance_requested_m"] == .5
    assert result["distance_executed_m"] is result["distance_remaining_m"] is None


def test_default_turn_uses_running_config_spin_envelope_and_signed_final_evidence():
    controller = Controller()
    robot = RobotInterface(controller=controller, adapters=(V3ControlInterfaceAdapter(controller),), upper_runtime=False)
    result = robot.execute("v3.command.turn_by", angle_deg=-90,
                           admission_sink=controller.observations.append)
    wheels = controller.resolved.runtime.composition.live_control.control.motion_realization.wheel_limits
    assert controller.navigate_parameters["max_omega_rad_s"] >= wheels.minimum_center_spin_rad_s
    assert controller.navigate_parameters["max_omega_rad_s"] == wheels.target_center_spin_rad_s
    assert result["status"] == "COMPLETED"
    assert result["angle_requested_rad"] < 0
    assert result["angle_executed_rad"] == result["angle_requested_rad"]
    assert result["angle_remaining_rad"] == 0
    assert result["runtime_pid"] == 123
    assert result["completion_status_monotonic_ns"] == 123456
    assert result["frame_provenance"]["config_snapshot_id"] == controller.resolved.snapshot_id


@pytest.mark.parametrize("action, parameters", [
    ("v3.command.turn_by", {"angle_deg": -90}),
    ("v3.command.move_relative", {"forward_m": 1, "final_yaw_rad": 1}),
    ("v3.command.move_relative", {"forward_m": 0, "left_m": 1}),
])
def test_explicit_unrealizable_spin_cap_stops_before_admission(action, parameters):
    controller = Controller()
    robot = RobotInterface(controller=controller, adapters=(V3ControlInterfaceAdapter(controller),), upper_runtime=False)
    wheels = controller.resolved.runtime.composition.live_control.control.motion_realization.wheel_limits
    result = robot.execute(action, **parameters, max_omega_rad_s=wheels.minimum_center_spin_rad_s / 2,
                           admission_sink=controller.observations.append)
    assert result["status"] == "INTERRUPTED"
    assert result["reason"] == "ANGULAR_LIMIT_UNREALIZABLE"
    assert result["command_id"] is result["mission_id"] is None
    assert not controller.admitted and controller.observations == []
    assert controller.order == ["STOP"]


def test_default_turn_respects_unrealizable_running_platform_cap():
    controller = Controller()
    control = controller.effective_config["runtime"]["composition"]["live_control"]["control"]
    control["operational_constraints"]["max_omega_rad_s"] = .1
    robot = RobotInterface(controller=controller, adapters=(V3ControlInterfaceAdapter(controller),), upper_runtime=False)
    result = robot.execute("v3.command.turn_by", angle_deg=-90)
    assert result["reason"] == "ANGULAR_LIMIT_UNREALIZABLE"
    assert controller.navigate_parameters is None
    assert controller.order == ["STOP"]


def test_missing_runtime_motion_config_never_admits_relative_motion():
    controller = Controller()
    original = controller.live_runtime_status
    def status_without_config():
        status = original()
        status.pop("effective_config")
        return status
    controller.live_runtime_status = status_without_config
    robot = RobotInterface(controller=controller, adapters=(V3ControlInterfaceAdapter(controller),), upper_runtime=False)
    with pytest.raises(ValueError, match="RUNTIME_MOTION_CONFIG_UNAVAILABLE"):
        robot.execute("v3.command.turn_by", angle_deg=-90)
    assert not controller.admitted
    assert controller.order == ["STOP"]
