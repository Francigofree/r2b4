"""Motor-free acceptance of the canonical finite-motion host boundary."""
from __future__ import annotations

import copy
import math
import threading
from types import SimpleNamespace

import pytest

from r2b4_er2.config import Er2Config
from r2b4_er2.tool_bridge import Er2RobotTools, Er2SafetyError
from r2b4_orchestration.agent_runner import _receipt
from r2b4_voice.action_executor import VoiceActionExecutor
from r2b4_voice.voice_service import _action_receipt_text
from v3.adapters import v3_control
from v3.adapters.operator import OperatorInterfaceAdapter
from v3.capture_rate import DEFAULT_CAPTURE_HZ
from v3.external_gateway import ExternalRobotGateway, GatewayPolicy
from v3.finite_navigation import FiniteNavigationExecutor, FiniteNavigationSafetyError
from v3.robot_interface import RobotInterface


class Clock:
    def __init__(self):
        self.now = 0.0
        self.on_sleep = None

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds
        if self.on_sleep is not None:
            self.on_sleep()


class Controller:
    """Runtime truth at the public controller boundary, without any device I/O."""
    def __init__(self, outcome="COMPLETE", *, running=True):
        self.outcome = outcome
        self.running = running
        self.calls = []
        self.target = None
        self.samples = 0
        self.stop_error = False
        self.activation_error = False
        self.navigation_error = False
        self.no_identity = False
        self.pose_delay = 0
        self.initial = {
            "state": "RUNNING", "ready_for_active": True,
            "safety_decision": "ALLOW", "fault_layer": None,
            "estimate": {
                "frame_id": "R2B4_BOOT_ROBOT_MAP", "x_m": 100.0, "y_m": 200.0, "yaw_rad": 0.0,
                "local_pose": {"frame_id": "R2B4_ODOM_LOCAL", "x_m": 2.0, "y_m": -1.0, "yaw_rad": math.pi / 2},
                "localization_quality": {"generation": 7},
            },
        }

    def status(self):
        return {"runtime_running": self.running, "capture_mode": "nincs", "capture_hz": DEFAULT_CAPTURE_HZ}

    def current_capture_mode(self):
        return "nincs"

    def current_capture_path(self):
        return None

    def ensure_runtime(self, mode, hz):
        self.calls.append(("runtime", mode, hz))
        if self.activation_error:
            raise RuntimeError("runtime activation failed")
        self.running = True

    def navigate(self, **parameters):
        self.calls.append(("navigate", parameters))
        if self.navigation_error:
            raise RuntimeError("command producer failed")
        self.target = parameters
        return SimpleNamespace(command_id=None if self.no_identity else "finite-test")

    def stop(self):
        self.calls.append(("stop",))
        if self.stop_error:
            raise RuntimeError("STOP acknowledgement lost")

    def live_runtime_status(self):
        if not self.running:
            return None
        if self.target is None:
            if self.pose_delay:
                self.pose_delay -= 1
                return None
            return copy.deepcopy(self.initial)
        self.samples += 1
        status = copy.deepcopy(self.initial)
        status["mission"] = {"mission_id": "mission-finite-test", "mode": "NAVIGATE", "lifecycle": "ACTIVE"}
        status["navigation"] = {"mission_id": "mission-finite-test", "status": "ACTIVE", "progress": 0.0}
        outcome = self.outcome
        if outcome == "COMPLETE" and self.samples >= 2:
            status["navigation"].update(status="COMPLETE", progress=1.0)
            status.update(safety_decision="STOP", safety_reason="NOT_ACTIVE")
            pose = status["estimate"]["local_pose"] if self.target["frame_id"] == "R2B4_ODOM_LOCAL" else status["estimate"]
            for name in ("x_m", "y_m", "yaw_rad"):
                if name in self.target:
                    pose[name] = self.target[name]
        elif outcome == "STATUS_UNAVAILABLE":
            return None
        elif outcome == "MISSION_REPLACED":
            status["mission"]["mission_id"] = "another-mission"
        elif outcome == "NAVIGATION_IDENTITY_MISMATCH":
            status["navigation"]["mission_id"] = "another-mission"
        elif outcome in {"MISSION_FAILED", "MISSION_CANCELLED"}:
            status["mission"]["lifecycle"] = outcome.removeprefix("MISSION_")
        elif outcome == "LOCALIZATION_FRAME_CHANGED":
            status["estimate"]["local_pose"]["frame_id"] = "different-frame"
        elif outcome == "GENERATION_CHANGED":
            status["estimate"]["localization_quality"]["generation"] += 1
        elif outcome == "RUNTIME_FAULT":
            status["fault_layer"] = "L12"
            status["safety_decision"] = "FAULT"
        elif outcome == "SAFETY_STOP":
            status.update(safety_decision="STOP", safety_reason="OBSTACLE")
            status["navigation"]["status"] = "COMPLETE"  # even COMPLETE must not hide a safety stop
        elif outcome == "SAFETY_STATUS_UNAVAILABLE":
            status["safety_decision"] = None
        elif outcome == "MISSION_INVALID":
            status["mission"]["mode"] = "EXPLORE"
        elif outcome == "NAVIGATION_STATUS_UNAVAILABLE":
            status["navigation"]["status"] = None
        elif outcome in {"NO_PATH", "INVALIDATED"}:
            status["navigation"]["status"] = outcome
        elif outcome == "LOCALIZATION_HOLD":
            if self.samples < 3:
                status.update(safety_decision="STOP", safety_reason="NOT_ACTIVE")
                status["navigation"].update(status="IDLE", reason="LOCALIZATION_HOLD")
            else:
                status.update(safety_decision="STOP", safety_reason="NOT_ACTIVE")
                status["navigation"]["status"] = "COMPLETE"
        return status


def interface_for(monkeypatch, controller, clock):
    monkeypatch.setattr(v3_control, "FiniteNavigationExecutor", lambda c: FiniteNavigationExecutor(
        c, sleep=clock.sleep, monotonic=clock.monotonic,
    ))
    return RobotInterface(controller=controller, adapters=(
        v3_control.V3ControlInterfaceAdapter(controller), OperatorInterfaceAdapter(controller),
    ))


def er2_for(interface):
    gateway = ExternalRobotGateway(interface, policy=GatewayPolicy(
        allow_execute=True, session_owner_pid=77, session_watchdog_s=3.0,
    ))
    return Er2RobotTools(gateway, Er2Config(session_watchdog_s=3.0))


def test_finite_motion_canonical_and_er2_share_targets_identity_completion_and_stop(monkeypatch):
    scenarios = (
        ("move_relative", {"forward_m": 1.2}, (2.0, 0.2, None)),
        ("move_relative", {"forward_m": -0.5}, (2.0, -1.5, None)),
        ("turn_by", {"angle_deg": 90.0}, (2.0, -1.0, -math.pi)),
        ("turn_by", {"angle_deg": -45.0}, (2.0, -1.0, math.pi / 4)),
        ("move_relative", {"forward_m": 0.3, "left_m": 0.4, "final_yaw_rad": math.pi}, (1.6, -0.7, -math.pi / 2)),
    )
    for primitive, params, expected in scenarios:
        receipts = []
        for route in ("canonical", "er2"):
            controller, clock = Controller(), Clock()
            interface = interface_for(monkeypatch, controller, clock)
            caps = interface.capabilities()["capabilities"]
            name = "v3.command." + primitive
            assert all(caps[name][key] is True for key in ("voice_exposed", "session_watchdog", "completion_required"))
            if route == "canonical":
                result = interface.execute(name, **params, capture=False, capture_mode="nincs",
                                           session_owner_pid=77, session_watchdog_s=3.0)
            else:
                result = er2_for(interface).execute("robot_" + primitive, params)
            assert result["status"] == "COMPLETED" and result["reason"] == "COMPLETE"
            assert result["command_id"] == "finite-test"
            assert result["mission_id"] == "mission-finite-test"
            target = result["target_pose"]
            assert target["frame_id"] == "R2B4_ODOM_LOCAL"
            assert (target["x_m"], target["y_m"]) == pytest.approx(expected[:2])
            if expected[2] is None:
                assert "yaw_rad" not in target
            else:
                assert target["yaw_rad"] == pytest.approx(expected[2])
                assert result["final_pose"]["yaw_rad"] == pytest.approx(expected[2])
            assert controller.target["session_owner_pid"] == 77
            assert controller.target["session_watchdog_s"] == 3.0
            assert controller.calls[-1] == ("stop",)
            assert clock.now > 0 and controller.samples >= 2
            receipts.append(result)
        assert receipts[0]["target_pose"] == receipts[1]["target_pose"]
        assert receipts[0]["final_pose"] == receipts[1]["final_pose"]

    # The absolute ER2 tool shares the lifecycle but retains the global frame.
    controller, clock = Controller(), Clock()
    result = er2_for(interface_for(monkeypatch, controller, clock)).robot_navigate_to_pose(x_m=1.0, y_m=2.0)
    assert result["status"] == "COMPLETED"
    assert result["target_pose"] == {"x_m": 1.0, "y_m": 2.0, "frame_id": "R2B4_BOOT_ROBOT_MAP"}
    assert controller.calls[-1] == ("stop",)


def test_finite_motion_failure_and_acceptance_never_report_voice_success(monkeypatch):
    reasons = (
        "MISSION_REPLACED", "NAVIGATION_IDENTITY_MISMATCH", "MISSION_FAILED", "MISSION_CANCELLED",
        "NO_PATH", "INVALIDATED", "SAFETY_STOP", "RUNTIME_FAULT", "STATUS_UNAVAILABLE",
        "LOCALIZATION_FRAME_CHANGED", "GENERATION_CHANGED", "TIMEOUT", "CANCELLED",
        "SAFETY_STATUS_UNAVAILABLE", "MISSION_INVALID", "NAVIGATION_STATUS_UNAVAILABLE",
        "COMPLETE", "LOCALIZATION_HOLD",
    )
    for reason in reasons:
        controller, clock = Controller(reason), Clock()
        interface = interface_for(monkeypatch, controller, clock)
        cancel = threading.Event()
        if reason == "CANCELLED":
            clock.on_sleep = cancel.set
        # Carry host cancellation without exposing it to the proposal schema.
        class CancellableInterface:
            capabilities = interface.capabilities
            read = interface.read
            def execute(self, action, **params):
                return interface.execute(action, cancel_event=cancel, **params)
        execution = VoiceActionExecutor(CancellableInterface(), mode="execute", session_watchdog_s=1.0).execute_proposal(
            {"name": "v3.command.turn_by", "parameters": {"angle_deg": 90}},
        )
        succeeded = reason in {"COMPLETE", "LOCALIZATION_HOLD"}
        expected_reason = "LOCALIZATION_FRAME_CHANGED" if reason == "GENERATION_CHANGED" else reason
        assert execution.executed is succeeded
        assert execution.status == ("COMPLETED" if succeeded else "FAILED:" + expected_reason)
        assert execution.command_id == "finite-test" and execution.mission_id == "mission-finite-test"
        assert controller.calls[-1] == ("stop",)
        assert (_receipt(execution.status, execution.executed) == "Rendben.") is succeeded
        assert (_action_receipt_text(status=execution.status, executed=execution.executed, action_name=execution.action_name) == "Rendben.") is succeeded

    for status in ("ACCEPTED", "INTERRUPTED", "FAILED", None):
        class AcceptedInterface:
            capabilities = interface.capabilities
            read = interface.read
            def execute(self, action, **params):
                return {"status": status, "command_id": "accepted-only"}
        result = VoiceActionExecutor(AcceptedInterface(), mode="execute").execute_proposal(
            {"name": "v3.command.move_relative", "parameters": {"forward_m": 1.2}},
        )
        assert result.executed is False and result.status.startswith("FAILED:")


def test_finite_motion_cold_readiness_cancellation_validation_and_stop_failure(monkeypatch):
    controller, clock = Controller(running=False), Clock()
    interface = interface_for(monkeypatch, controller, clock)
    controller.pose_delay = 2
    result = interface.execute("v3.command.move_relative", forward_m=0.5)
    assert result["status"] == "COMPLETED" and clock.now >= 0.15 - 1e-9
    assert [call[0] for call in controller.calls] == ["runtime", "navigate", "stop"]

    controller, clock = Controller(running=False), Clock()
    cancel = threading.Event()
    cancel.set()
    result = interface_for(monkeypatch, controller, clock).execute("v3.command.turn_by", angle_deg=45, cancel_event=cancel)
    assert result["reason"] == "CANCELLED" and controller.calls == [("stop",)]

    for params in ({}, {"angle_deg": 181}, {"angle_deg": float("nan")}, {"angle_deg": True},
                   {"angle_deg": 1, "max_omega_rad_s": 0}, {"angle_deg": 1, "max_v_mps": 0.4}):
        controller, clock = Controller(), Clock()
        with pytest.raises(ValueError):
            interface_for(monkeypatch, controller, clock).execute("v3.command.turn_by", **params)
        assert controller.calls == [("stop",)]

    for failure in ("activation_error", "navigation_error", "no_identity"):
        controller, clock = Controller(), Clock()
        setattr(controller, failure, True)
        with pytest.raises(RuntimeError):
            interface_for(monkeypatch, controller, clock).execute("v3.command.move_relative", forward_m=0.5)
        assert controller.calls[-1] == ("stop",)

    for route in ("canonical", "er2"):
        controller, clock = Controller(), Clock()
        controller.stop_error = True
        interface = interface_for(monkeypatch, controller, clock)
        with pytest.raises(FiniteNavigationSafetyError if route == "canonical" else Er2SafetyError):
            if route == "canonical":
                interface.execute("v3.command.move_relative", forward_m=0.5)
            else:
                er2_for(interface).execute("robot_move_relative", {"forward_m": 0.5})
        assert [call[0] for call in controller.calls].count("stop") == 1
