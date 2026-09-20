from r2b4_voice.robot_context import ROBOT_CONTEXT_SCHEMA, RobotContextBuilder


class StoppedInterface:
    def capabilities(self):
        return {
            "capabilities": {
                "operator.status": {"kind": "read", "available": True},
                "v3.status": {"kind": "read", "available": False},
                "v3.command.stop": {"kind": "action", "supported": True, "available": True, "ready": True},
            }
        }
    def read(self, name):
        if name == "operator.status":
            return {"runtime_running": False, "runtime_pid": None}
        raise AssertionError(name)


def test_stopped_runtime_is_not_reported_as_unavailable_or_faulted():
    context = RobotContextBuilder(StoppedInterface()).build().to_jsonable()
    assert context["schema"] == ROBOT_CONTEXT_SCHEMA == "R2B4_ROBOT_CONTEXT_V3"
    assert context["host"]["runtime_running"] is False
    assert context["host"]["runtime_state"] == "STOPPED"
    assert context["runtime"]["state"] == "STOPPED"
    assert context["runtime"]["live_status_available"] is False
    assert context["runtime"]["fault_layer"] is None
    assert context["runtime"]["interpretation"] == "RUNTIME_STOPPED"


class RunningWithoutFreshStatus:
    def capabilities(self):
        return {
            "capabilities": {
                "operator.status": {"kind": "read", "available": True},
                "v3.status": {"kind": "read", "available": False},
            }
        }
    def read(self, name):
        if name == "operator.status":
            return {"runtime_running": True, "runtime_pid": 12}
        raise AssertionError(name)


def test_running_without_live_status_is_unknown_not_fault():
    context = RobotContextBuilder(RunningWithoutFreshStatus()).build().to_jsonable()
    assert context["host"]["runtime_running"] is True
    assert context["runtime"]["state"] == "UNAVAILABLE"
    assert context["runtime"]["interpretation"] == "LIVE_STATUS_UNAVAILABLE"
    assert context["runtime"]["fault_layer"] is None
