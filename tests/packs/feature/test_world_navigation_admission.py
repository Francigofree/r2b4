"""World coordinates cannot cross runtime/capture identity at motion admission."""
from __future__ import annotations

from contextlib import contextmanager
from types import SimpleNamespace

import pytest

from v3.adapters.v3_control import V3ControlInterfaceAdapter
from v3.robot_interface import RobotInterface


class Controller:
    locked = False
    require_lock = True

    def __init__(self):
        self.navigations = []

    def status(self):
        return {"runtime_running": True}

    def live_runtime_status(self):
        return {"state": "RUNNING", "ready_for_active": True}

    def snapshot(self):
        assert self.locked, "session identity must be checked under the operator transaction"
        return SimpleNamespace(runtime_pid=123)

    def current_capture_mode(self):
        return "full"

    def current_capture_hz(self):
        return 10

    @contextmanager
    def operator_transition(self):
        self.locked = True
        try:
            yield
        finally:
            self.locked = False

    def navigate(self, **parameters):
        if self.require_lock:
            assert self.locked, "the world goal must be admitted in the checked session"
        self.navigations.append(parameters)
        return {"command_id": "navigation-bound"}


@pytest.mark.parametrize("expected_pid, capture, capture_mode, capture_hz, accepted", [
    (123, False, "full", 10, True),
    (456, False, "full", 10, False),
    (123, True, "full", 10, False),
    (123, False, "alap", 10, False),
    (123, False, "full", 5, False),
])
def test_world_goal_identity_check_and_request_share_one_public_admission_transaction(
    expected_pid, capture, capture_mode, capture_hz, accepted,
):
    controller = Controller()
    facade = RobotInterface(controller=controller, adapters=(V3ControlInterfaceAdapter(controller),), upper_runtime=False)
    parameters = dict(x_m=1.0, y_m=2.0, expected_runtime_pid=expected_pid, capture=capture,
                      capture_mode=capture_mode, capture_hz=capture_hz)
    if accepted:
        assert facade.execute("v3.command.navigate", **parameters)["command_id"] == "navigation-bound"
        assert len(controller.navigations) == 1
    else:
        with pytest.raises(RuntimeError, match="world goal"):
            facade.execute("v3.command.navigate", **parameters)
        assert controller.navigations == []


def test_ordinary_navigation_preserves_existing_capture_and_request_semantics():
    controller = Controller()
    controller.require_lock = False
    adapter = V3ControlInterfaceAdapter(controller)
    result = adapter.execute("v3.command.navigate", x_m=1.0, y_m=2.0,
                             capture=True, capture_mode="alap", capture_hz=10)
    assert result["command_id"] == "navigation-bound"
    assert len(controller.navigations) == 1
    assert controller.navigations[0]["capture"] is True
    assert controller.navigations[0]["capture_mode"] == "alap"
    assert "expected_runtime_pid" not in controller.navigations[0]
