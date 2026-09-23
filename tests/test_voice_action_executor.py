from r2b4_voice.action_executor import VoiceActionExecutor


class FakeInterface:
    def __init__(self, *, ready=True, fault=False):
        self.ready = ready
        self.fault = fault
        self.calls = []

    def capabilities(self):
        action = lambda name: {
            "kind": "action", "supported": True, "available": True,
            "ready": self.ready, "reason": None if self.ready else "NOT_READY",
        }
        return {
            "capabilities": {
                "operator.status": {"kind": "read", "supported": True, "available": True, "ready": True},
                "v3.status": {"kind": "read", "supported": True, "available": True, "ready": True},
                "v3.command.stop": action("stop"),
                "v3.command.face_person": action("face"),
                "v3.command.follow_person": action("follow"),
            }
        }

    def read(self, resource):
        if resource == "operator.status":
            return {"runtime_running": True}
        if resource == "v3.status":
            return {
                "state": "RUNNING",
                "ready_for_active": True,
                "tick_id": 42,
                "enabled": False,
                "fault_layer": "L9" if self.fault else None,
                "world": {"person_tracks": [{"track_id": "person-1"}]},
            }
        raise KeyError(resource)

    def execute(self, action, **parameters):
        self.calls.append((action, parameters))
        return {"status": "ok"}


def test_positive_action_gets_owner_and_watchdog_only_after_fresh_check():
    interface = FakeInterface()
    executor = VoiceActionExecutor(
        interface, mode="execute", session_owner_pid=123, session_watchdog_s=15.0
    )
    result = executor.execute_proposal(
        {"name": "v3.command.follow_person", "parameters": {"max_v_mps": 0.10, "max_omega_rad_s": 0.20}}
    )
    assert result.executed is True
    assert interface.calls == [
        (
            "v3.command.follow_person",
            {
                "max_omega_rad_s": 0.2,
                "max_v_mps": 0.1,
                "session_owner_pid": 123,
                "session_watchdog_s": 15.0,
            },
        )
    ]


def test_shadow_and_not_ready_never_execute():
    interface = FakeInterface()
    shadow = VoiceActionExecutor(interface, mode="shadow", session_owner_pid=123)
    result = shadow.execute_proposal({"name": "v3.command.face_person", "parameters": {"max_omega_rad_s": 0.2}})
    assert result.status == "SHADOW_ACCEPTED"
    assert interface.calls == []

    blocked_interface = FakeInterface(ready=False)
    live = VoiceActionExecutor(blocked_interface, mode="execute", session_owner_pid=123)
    blocked = live.execute_proposal({"name": "v3.command.face_person", "parameters": {"max_omega_rad_s": 0.2}})
    assert blocked.status == "REJECTED:ACTION_NOT_READY"
    assert blocked_interface.calls == []


def test_fault_and_non_allowlisted_action_fail_closed():
    faulty = FakeInterface(fault=True)
    executor = VoiceActionExecutor(faulty, mode="execute", session_owner_pid=123)
    result = executor.execute_proposal({"name": "v3.command.face_person", "parameters": {"max_omega_rad_s": 0.2}})
    assert result.status == "REJECTED:RUNTIME_FAULT"
    assert faulty.calls == []

    unknown = executor.execute_proposal({"name": "v3.command.forward", "parameters": {}})
    assert unknown.status == "REJECTED:ACTION_NOT_ADVERTISED"
    assert faulty.calls == []
