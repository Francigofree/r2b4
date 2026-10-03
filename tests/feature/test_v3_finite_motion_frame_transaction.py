from __future__ import annotations

from contextlib import contextmanager
import math
from types import SimpleNamespace

import pytest

from v3.contracts.localization import LOCAL_FRAME_ID
from v3.finite_navigation import FiniteNavigationExecutor
from v3.operator_controller import MotionPreparation, OperatorController


class _FiniteController:
    def __init__(self) -> None:
        self.runtime_pid = 222
        self.in_transaction = False
        self.command_id: str | None = None
        self.target: dict[str, object] | None = None
        self.stop_count = 0
        self.start_pose = {
            "frame_id": LOCAL_FRAME_ID,
            "x_m": 0.17699,
            "y_m": -0.11322,
            "yaw_rad": -0.5512140628277933,
        }

    @contextmanager
    def finite_motion_transaction(self, *, capture, capture_mode, capture_hz):
        self.in_transaction = True
        try:
            yield MotionPreparation(self.runtime_pid, capture_mode, capture_hz, capture)
        finally:
            self.in_transaction = False

    def snapshot(self):
        return SimpleNamespace(runtime_pid=self.runtime_pid)

    def live_runtime_status(self):
        base = {
            "state": "RUNNING",
            "ready_for_active": True,
            "fault_layer": None,
            "safety_decision": "STOP",
            "safety_reason": "NOT_ACTIVE",
            "monotonic_ns": 123456789,
            "estimate": {
                "local_pose": dict(self.start_pose),
                "localization_quality": {"generation": 0},
            },
        }
        if self.command_id is None:
            return base
        base["mission"] = {
            "mission_id": f"mission-{self.command_id}",
            "mode": "NAVIGATE",
            "lifecycle": "ACTIVE",
        }
        base["navigation"] = {
            "mission_id": f"mission-{self.command_id}",
            "status": "COMPLETE",
            "reason": None,
            "progress": 1.0,
        }
        return base

    def navigate(self, **kwargs):
        assert self.in_transaction
        preparation = kwargs.pop("preparation")
        assert preparation.runtime_pid == self.runtime_pid
        self.target = dict(kwargs)
        self.command_id = "finite-test"
        return {"command_id": self.command_id}

    def stop(self):
        self.stop_count += 1


def test_relative_target_uses_post_preparation_pose_and_exports_provenance() -> None:
    controller = _FiniteController()
    result = FiniteNavigationExecutor(controller, sleep=lambda _: None).execute(
        "v3.command.move_relative",
        {"forward_m": 1.2},
        capture=True,
        capture_mode="alap",
        capture_hz=10,
        session_watchdog_s=30.0,
    )

    assert result["status"] == "COMPLETED"
    assert result["reason"] == "COMPLETE"
    start = result["start_pose"]
    target = result["target_pose"]
    assert start == controller.start_pose
    assert math.hypot(
        float(target["x_m"]) - float(start["x_m"]),
        float(target["y_m"]) - float(start["y_m"]),
    ) == pytest.approx(1.2)
    assert result["frame_provenance"] == {
        "frame_id": LOCAL_FRAME_ID,
        "runtime_pid": 222,
        "localization_generation": 0,
        "pose_status_monotonic_ns": 123456789,
    }
    assert controller.stop_count == 1


def test_runtime_session_change_fails_closed() -> None:
    controller = _FiniteController()
    original_navigate = controller.navigate

    def navigate_and_replace_runtime(**kwargs):
        result = original_navigate(**kwargs)
        controller.runtime_pid = 333
        return result

    controller.navigate = navigate_and_replace_runtime
    result = FiniteNavigationExecutor(controller, sleep=lambda _: None).execute(
        "v3.command.move_relative",
        {"forward_m": 0.5},
        capture=True,
        capture_mode="alap",
        capture_hz=10,
        session_watchdog_s=30.0,
    )
    assert result["status"] == "INTERRUPTED"
    assert result["reason"] == "RUNTIME_SESSION_CHANGED"
    assert controller.stop_count == 1


def test_operator_finite_transaction_rearms_before_yield(monkeypatch, tmp_path) -> None:
    controller = OperatorController(tmp_path)
    events: list[str] = []
    pid = {"value": 100}

    monkeypatch.setattr(
        controller,
        "ensure_runtime",
        lambda mode, hz: events.append("ensure") or pid["value"],
    )
    monkeypatch.setattr(controller, "stop", lambda: events.append("stop"))
    monkeypatch.setattr(controller, "current_capture_mode", lambda: "alap")
    monkeypatch.setattr(controller, "current_capture_hz", lambda: 10)

    def rearm():
        events.append("rearm")
        pid["value"] = 200

    monkeypatch.setattr(controller, "_ensure_fresh_capture_slot", rearm)
    monkeypatch.setattr(controller, "_wait_ready", lambda: events.append("ready"))
    monkeypatch.setattr(controller, "_runtime_pid", lambda: pid["value"])

    with controller.finite_motion_transaction(
        capture=True, capture_mode="alap", capture_hz=10,
    ) as preparation:
        events.append("snapshot")
        assert preparation.runtime_pid == 200

    assert events == ["ensure", "stop", "rearm", "ready", "snapshot"]
