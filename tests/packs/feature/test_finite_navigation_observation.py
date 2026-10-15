"""Finite public actions expose admission identity before their completion wait."""
from __future__ import annotations

import threading
from contextlib import contextmanager
from types import SimpleNamespace

import pytest
from rig import resolved_config

from v3.adapters.v3_control import V3ControlInterfaceAdapter
from v3.contracts.localization import GLOBAL_FRAME_ID, LOCAL_FRAME_ID
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
        self.stopped = False

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
            "monotonic_ns": 123458 if self.stopped else 123456,
            "state": "RUNNING", "ready_for_active": True,
            "fault_layer": None, "safety_decision": "ALLOW", "safety_reason": "ALLOW",
            "enabled": False, "left_output": 0., "right_output": 0.,
            "estimate": {
                "frame_id": GLOBAL_FRAME_ID, "x_m": 1.0, "y_m": 2.0, "yaw_rad": 0.0,
                "v_mps": 0., "omega_rad_s": 0.,
                "local_pose": {"frame_id": LOCAL_FRAME_ID, "x_m": 1.0, "y_m": 2.0, "yaw_rad": 0.0},
                "localization_quality": {"generation": 7, "global_position": "GOOD", "heading": "GOOD",
                    "global_fix_age_ns": 0,
                    "global_fix_last_accepted_ns": 123457 if self.stopped else (123456 if self.admitted else 123455)},
            },
        }
        if self.admitted:
            if not self.stopped:
                assert self.observations, "completion wait began before passive admission publication"
                self.order.append("WAIT")
            status.update(
                mission={"mission_id": "mission-finite-1", "mode": "NAVIGATE", "lifecycle": "COMPLETED"},
                navigation={"mission_id": "mission-finite-1", "status": "COMPLETE", "reason": "COMPLETE"},
                safety_decision="STOP", safety_reason="NOT_ACTIVE",
            )
            if self.navigate_parameters is not None:
                pose = (status["estimate"]["local_pose"] if self.navigate_parameters["frame_id"] == LOCAL_FRAME_ID
                        else status["estimate"])
                pose.update({key: self.navigate_parameters[key]
                    for key in ("x_m", "y_m", "yaw_rad") if key in self.navigate_parameters})
        return status

    def stop(self):
        self.order.append("STOP")
        self.stopped = True


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
    frame_id = GLOBAL_FRAME_ID if action == "v3.command.move_relative" else LOCAL_FRAME_ID
    assert admission == {
        "command_id": "finite-1", "mission_id": "mission-finite-1", "frame_id": frame_id,
        "runtime_pid": 123, "localization_generation": None if frame_id == GLOBAL_FRAME_ID else 7,
        "pose_status_monotonic_ns": 123456,
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
    result = robot.execute("v3.command.navigate", x_m=1.5, y_m=2., frame_id=LOCAL_FRAME_ID,
                           wait_for_completion=True, admission_sink=controller.observations.append)
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


@pytest.mark.parametrize("action,parameters,error_field,remaining_distance", [
    ("v3.command.move_relative", {"forward_m": .2}, "distance_remaining_m", .06246),
    ("v3.command.move_relative", {"forward_m": .3}, "distance_remaining_m", .07337),
    ("v3.command.turn_by", {"angle_deg": 5}, "angle_remaining_rad", 0.),
    ("v3.command.turn_by", {"angle_deg": -5}, "angle_remaining_rad", 0.),
])
def test_small_relative_request_cannot_complete_outside_request_tolerance(
    action, parameters, error_field, remaining_distance,
):
    class UndertravelController(Controller):
        def live_runtime_status(self):
            status = super().live_runtime_status()
            # Reproduce captured translation residuals and the unmoved 5° turn.
            if self.admitted:
                pose = (status["estimate"] if self.navigate_parameters["frame_id"] == GLOBAL_FRAME_ID
                        else status["estimate"]["local_pose"])
                pose["x_m"] -= remaining_distance
                pose["yaw_rad"] = 0.
            return status
    controller = UndertravelController()
    robot = RobotInterface(controller=controller, adapters=(V3ControlInterfaceAdapter(controller),), upper_runtime=False)
    result = robot.execute(action, **parameters, admission_sink=controller.observations.append)
    assert result["status"] == "INTERRUPTED"
    assert result["reason"] == "COMPLETION_OUTSIDE_TOLERANCE"
    assert abs(result[error_field]) > result["goal_tolerance_m" if error_field == "distance_remaining_m" else "yaw_tolerance_rad"]
    assert controller.order[-1] == "STOP"


def test_request_tolerances_cross_operator_cli_resident_and_l5(tmp_path, monkeypatch):
    import time
    from rig import ROOT
    from v3 import control_cli
    from v3.adapters.resident_command import AtomicResidentCommandGateway, ResidentCommandMailboxConfig
    from v3.contracts import CommandMode, TickContext
    from v3.layers.l5_command_mission import MissionManager
    from v3.operator_controller import OperatorController

    resolved = resolved_config()
    controller = OperatorController(ROOT)
    calls = []
    monkeypatch.setattr(controller, "_start_motion", lambda *args, **kwargs: calls.append(args[3]) or (321, "nincs"))
    controller.navigate(x_m=.2, y_m=0., yaw_rad=.087, frame_id=LOCAL_FRAME_ID,
                        goal_tolerance_m=.02, yaw_tolerance_rad=.0087, capture=False)
    monkeypatch.setattr(control_cli, "_runtime_path", lambda value: tmp_path / value.rsplit("/", 1)[-1])
    monkeypatch.setattr(control_cli, "_active_preflight", lambda path: None)
    monkeypatch.setattr(control_cli, "_run_active", lambda client, publish, **kwargs: 0 if publish(kwargs["command_id"]) else 1)
    assert control_cli.main(calls[0][3:]) == 0
    mailbox = ResidentCommandMailboxConfig.from_policy(tmp_path / "v3_command.json", resolved.edges.command_ingress)
    command = AtomicResidentCommandGateway(mailbox).snapshot(TickContext(0, time.monotonic_ns()))
    assert command.mode is CommandMode.NAVIGATE
    mission = MissionManager(resolved.runtime.composition.live_control.control.mission).evaluate(command)
    assert mission.constraints.goal_tolerance_m == .02
    assert mission.constraints.yaw_tolerance_rad == .0087


def test_relative_tolerance_preserves_tighter_running_config_and_evidence():
    controller = Controller()
    defaults = controller.effective_config["runtime"]["composition"]["live_control"]["control"]["mission"]["default_constraints"]
    defaults.update(goal_tolerance_m=.003, yaw_tolerance_rad=.002)
    robot = RobotInterface(controller=controller, adapters=(V3ControlInterfaceAdapter(controller),), upper_runtime=False)
    result = robot.execute("v3.command.move_relative", forward_m=.2, final_yaw_rad=.087,
                           admission_sink=controller.observations.append)
    assert result["status"] == "COMPLETED"
    assert result["goal_tolerance_m"] == controller.navigate_parameters["goal_tolerance_m"] == .003
    assert result["yaw_tolerance_rad"] == controller.navigate_parameters["yaw_tolerance_rad"] == .002


@pytest.mark.parametrize("distance,tolerance", ((1., .02), (.3, .02), (.1, .01)))
def test_relative_translation_uses_corrected_global_arrival_and_bounded_tolerance(distance, tolerance):
    class CorrectedController(Controller):
        def live_runtime_status(self):
            status = super().live_runtime_status()
            status["estimate"]["local_pose"]["x_m"] += .065
            if self.admitted:
                # A corrected map pose remains comparable when odometry and
                # map_T_odom change; the fixed global command is not re-anchored.
                status["estimate"]["transform_revision"] = 2
                status["estimate"]["localization_quality"]["generation"] = 8
            return status

    controller = CorrectedController()
    events = []
    robot = RobotInterface(controller=controller, adapters=(V3ControlInterfaceAdapter(controller),),
                           upper_runtime=False, observation_sink=events.append)
    result = robot.execute("v3.command.move_relative", forward_m=distance,
                           admission_sink=controller.observations.append)
    assert result["status"] == "COMPLETED"
    assert controller.navigate_parameters["frame_id"] == GLOBAL_FRAME_ID
    assert controller.navigate_parameters["x_m"] == pytest.approx(1. + distance)
    assert controller.navigate_parameters["goal_tolerance_m"] == pytest.approx(tolerance)
    assert result["goal_tolerance_m"] == pytest.approx(tolerance)
    assert result["distance_executed_m"] == pytest.approx(distance)
    assert result["distance_remaining_m"] == 0.
    assert result["frame_provenance"]["frame_id"] == GLOBAL_FRAME_ID
    assert result["completion_status_monotonic_ns"] == 123456
    assert result["final_pose_status_monotonic_ns"] == 123458
    observed = next(event for event in events if event.kind == "ACTION_RESULT")
    assert observed.completion_status_monotonic_ns == 123456
    assert observed.final_pose_status_monotonic_ns == observed.measurement_time_ns == 123458
    assert result["navigation_reason"] == "COMPLETE"
    assert controller.order.count("ADMITTED") == 1


@pytest.mark.parametrize("quality", (
    {"global_position": "LOST", "global_fix_age_ns": 0},
    {"global_position": "DEGRADED", "global_fix_age_ns": 0},
    {"global_position": "GOOD", "global_fix_age_ns": "stale"},
    {"global_position": "GOOD", "global_fix_age_ns": -1},
    {"global_position": "GOOD", "global_fix_age_ns": 0, "heading": "LOST"},
    {"global_position": "GOOD", "global_fix_age_ns": 0, "global_fix_last_accepted_ns": None},
    {},
))
@pytest.mark.parametrize("recovers", (False, True))
def test_relative_translation_waits_for_fresh_good_global_anchor_without_local_fallback(quality, recovers):
    from v3.finite_navigation import FiniteNavigationExecutor

    class FixController(Controller):
        def __init__(self):
            super().__init__()
            self.pose_reads = 0

        def live_runtime_status(self):
            status = super().live_runtime_status()
            if not self.admitted:
                self.pose_reads += 1
                if not recovers or self.pose_reads == 1:
                    status["estimate"]["x_m"] = 100.
                    status["estimate"]["localization_quality"] = {
                        "generation": 7, "heading": "GOOD", "global_fix_last_accepted_ns": 123455, **quality}
                    if quality.get("global_fix_age_ns") == "stale":
                        maximum_age = self.resolved.runtime.composition.live_control.control.estimation.quality.global_fix_max_age_ns
                        status["estimate"]["localization_quality"]["global_fix_age_ns"] = maximum_age + 1
            return status

    controller = FixController()
    clock = [0.]
    executor = FiniteNavigationExecutor(controller, monotonic=lambda: clock[0],
        sleep=lambda dt: clock.__setitem__(0, clock[0] + dt))
    result = executor.execute("v3.command.move_relative", {"forward_m": 1.}, finite_timeout_s=.2,
                              admission_sink=controller.observations.append)
    if recovers:
        assert result["status"] == "COMPLETED"
        assert controller.pose_reads == 2
        assert controller.navigate_parameters["frame_id"] == GLOBAL_FRAME_ID
        assert controller.navigate_parameters["x_m"] == 2.
        assert result["start_pose"]["x_m"] == 1.
    else:
        assert result["status"] == "INTERRUPTED" and result["reason"] == "TIMEOUT"
        assert controller.navigate_parameters is None
        assert result["command_id"] is None
        assert result["final_pose"]["frame_id"] == result["frame_provenance"]["frame_id"] == GLOBAL_FRAME_ID
    assert controller.order[-1] == "STOP"


@pytest.mark.parametrize("change", ("drift", "fix_lost"))
def test_relative_translation_cannot_complete_from_local_arrival_or_untrusted_global_pose(change):
    class WrongArrivalController(Controller):
        def live_runtime_status(self):
            status = super().live_runtime_status()
            if self.admitted:
                status["estimate"]["local_pose"]["x_m"] = self.navigate_parameters["x_m"]
                if change == "drift":
                    status["estimate"]["x_m"] -= .065
                else:
                    status["estimate"]["localization_quality"]["global_position"] = "LOST"
            return status

    controller = WrongArrivalController()
    robot = RobotInterface(controller=controller, adapters=(V3ControlInterfaceAdapter(controller),), upper_runtime=False)
    result = robot.execute("v3.command.move_relative", forward_m=1.,
                           admission_sink=controller.observations.append)
    assert result["status"] == "INTERRUPTED"
    assert result["reason"] == ("COMPLETION_OUTSIDE_TOLERANCE" if change == "drift"
                                else "GLOBAL_LOCALIZATION_UNAVAILABLE")
    assert controller.order[-1] == "STOP"


def test_relative_zero_translation_keeps_local_heading_without_global_fix():
    class NoFixController(Controller):
        def live_runtime_status(self):
            status = super().live_runtime_status()
            status["estimate"]["localization_quality"].update(global_position="LOST", global_fix_age_ns=None)
            return status

    controller = NoFixController()
    robot = RobotInterface(controller=controller, adapters=(V3ControlInterfaceAdapter(controller),), upper_runtime=False)
    result = robot.execute("v3.command.move_relative", forward_m=0., final_yaw_rad=.2,
                           admission_sink=controller.observations.append)
    assert result["status"] == "COMPLETED"
    assert controller.navigate_parameters["frame_id"] == LOCAL_FRAME_ID


def test_relative_translation_rejects_repeated_anchor_fix_as_completion_evidence():
    class RepeatedFixController(Controller):
        def live_runtime_status(self):
            status = super().live_runtime_status()
            status["estimate"]["localization_quality"]["global_fix_last_accepted_ns"] = 123455
            return status

    controller = RepeatedFixController()
    robot = RobotInterface(controller=controller, adapters=(V3ControlInterfaceAdapter(controller),), upper_runtime=False)
    result = robot.execute("v3.command.move_relative", forward_m=1.,
                           admission_sink=controller.observations.append)
    assert result["status"] == "INTERRUPTED"
    assert result["reason"] == "GLOBAL_LOCALIZATION_UNAVAILABLE"
    assert controller.order.count("ADMITTED") == controller.order.count("STOP") == 1


@pytest.mark.parametrize("change,reason", (
    ("coast", "POST_STOP_OUTSIDE_TOLERANCE"),
    ("repeated_fix", "FINAL_POSE_UNCONFIRMED"),
    ("moving", "FINAL_POSE_UNCONFIRMED"),
    ("missing_status", "STATUS_UNAVAILABLE"),
    ("session", "RUNTIME_SESSION_CHANGED"),
    ("output_active", "POST_STOP_MOTION_ACTIVE"),
    ("stale_lineage", "FINAL_POSE_UNCONFIRMED"),
))
def test_relative_translation_verifies_post_stop_fix_pose_and_stationarity_without_motion_retry(change, reason):
    from v3.finite_navigation import FiniteNavigationExecutor

    class PostStopController(Controller):
        def snapshot(self):
            return SimpleNamespace(runtime_pid=456 if self.stopped and change == "session" else 123)

        def live_runtime_status(self):
            status = super().live_runtime_status()
            if self.stopped:
                if change == "missing_status":
                    return None
                if change == "coast":
                    status["estimate"]["x_m"] += .03
                if change == "repeated_fix":
                    status["estimate"]["localization_quality"]["global_fix_last_accepted_ns"] = 123456
                if change == "moving":
                    threshold = self.resolved.runtime.composition.live_control.control.estimation.still_velocity_threshold_mps
                    status["estimate"]["v_mps"] = threshold * 2
                if change == "output_active":
                    status.update(enabled=True, left_output=.1, safety_decision="ALLOW")
                if change == "stale_lineage":
                    maximum_age = self.resolved.runtime.composition.live_control.control.estimation.quality.global_fix_max_age_ns
                    status["monotonic_ns"] += maximum_age
                    # The scalar claims freshness; accepted measurement lineage
                    # proves the final fix is actually beyond its age budget.
                    status["estimate"]["localization_quality"]["global_fix_age_ns"] = 0
                status["navigation"].update(status="IDLE", reason="COMMAND_STOP", progress=0.)
            return status

    controller = PostStopController()
    clock = [0.]
    executor = FiniteNavigationExecutor(controller, monotonic=lambda: clock[0],
        sleep=lambda dt: clock.__setitem__(0, clock[0] + dt))
    result = executor.execute("v3.command.move_relative", {"forward_m": 1.}, finite_timeout_s=.2,
                              admission_sink=controller.observations.append)
    assert result["status"] == "INTERRUPTED" and result["reason"] == reason
    assert controller.order.count("ADMITTED") == controller.order.count("STOP") == 1
    assert clock[0] <= .2
    assert result["completion_status_monotonic_ns"] == 123456
    assert result["navigation_reason"] == "COMPLETE"
    if change == "coast":
        assert result["distance_remaining_m"] == pytest.approx(.03)
        assert result["final_pose"]["x_m"] == pytest.approx(2.03)
