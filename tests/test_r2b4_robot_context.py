from r2b4_voice.robot_context import RobotContextBuilder


class FakeInterface:
    def capabilities(self):
        return {
            "schema": "x",
            "capabilities": {
                "v3.status": {"kind": "read", "available": True},
                "v3.pose": {"kind": "read", "available": True},
                "v3.safety": {"kind": "read", "available": True},
                "v3.health": {"kind": "read", "available": True},
                "v3.command.stop": {
                    "kind": "action",
                    "supported": True,
                    "available": True,
                    "ready": True,
                },
                "v3.command.face_person": {
                    "kind": "action",
                    "supported": True,
                    "available": False,
                    "ready": False,
                    "reason": "NO_PERSON",
                },
                "host.shutdown": {
                    "kind": "action",
                    "supported": True,
                    "available": True,
                    "ready": True,
                },
            },
        }

    def read(self, name):
        return {
            "v3.status": {
                "state": "RUNNING",
                "ready_for_active": True,
                "tick_id": 10,
                "enabled": False,
                "fault_layer": None,
            },
            "v3.pose": {"x_m": 1.0, "y_m": 2.0, "yaw_rad": 0.2},
            "v3.safety": {"decision": "STOP"},
            "v3.health": [{"device": "lidar", "state": "OK"}],
        }[name]


def test_context_is_compact_and_only_allowlists_llm_actions():
    context = RobotContextBuilder(FakeInterface()).build().to_jsonable()
    assert context["runtime"]["state"] == "RUNNING"
    assert context["pose"]["x_m"] == 1.0
    names = {item["name"] for item in context["available_actions"]}
    assert names == {"v3.command.stop", "v3.command.face_person"}
    assert "host.shutdown" not in names


class NoRuntimeInterface:
    def capabilities(self):
        return {"capabilities": {"v3.status": {"kind": "read", "available": False}}}

    def read(self, name):
        raise AssertionError("read must not be called")


def test_context_handles_runtime_unavailable_without_starting_it():
    context = RobotContextBuilder(NoRuntimeInterface()).build()
    assert context.runtime["state"] == "UNAVAILABLE"
    assert context.pose is None
