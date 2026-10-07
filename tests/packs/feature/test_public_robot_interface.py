from __future__ import annotations

import json
from types import SimpleNamespace

import pytest

from r2b4_orchestration.robot_runtime import PublicRobotClient
from r2b4_voice.robot_context import RobotContextBuilder
from v3.adapters.vision_media_socket import VisionClient
from v3.robot_interface import RobotInterface


class Controller:
    def __init__(self, project_root, **_kwargs):
        self.root = project_root
        self.calls = []
        self.status_value = {
            "state": "RUNNING", "ready_for_active": True, "tick_id": 4,
            "monotonic_ns": 10, "enabled": False, "fault_layer": None,
            "world": {"blocked": True, "person_tracks": []},
            "mission": {"mission_id": "mission-local"}, "navigation": {},
        }

    def status(self):
        return {"runtime_running": True, "runtime_pid": 123, "status": self.status_value}

    def live_runtime_status(self):
        return self.status_value

    def current_capture_mode(self):
        return "nincs"

    def current_capture_hz(self):
        return 10

    def current_capture_path(self):
        return None

    def stop(self):
        self.calls.append(("stop", {}))

    def forward(self, speed, **parameters):
        self.calls.append(("forward", {"speed": speed, **parameters}))
        return SimpleNamespace(command_id="local-forward")

    def roomcruise(self, **_parameters):
        raise AssertionError("public Room Cruise must enter the Behavior System")

    def followperson(self, **_parameters):
        raise AssertionError("public Follow Person must enter the Behavior System")


@pytest.fixture
def composed(monkeypatch, tmp_path):
    calls = []
    world = {"facts": [{"entity_id": "door", "value": "open", "age_ns": 100,
                        "confidence": 0.4, "state": "LIKELY", "lineage": ["frame-1"]}]}
    behavior = {"name": "room_cruise", "lifecycle": "ACTIVE", "behavior_id": "behavior-1"}

    def request(client, operation, **arguments):
        calls.append((client.socket_path, operation, arguments))
        if operation == "read":
            return {"world.snapshot": world, "behavior.state": behavior,
                    "robot.state": {"world": world, "active_behavior": behavior}}[arguments["resource"]]
        return {"command_id": "upper-command", "mission_id": "mission-upper", "lifecycle": "STARTING"}

    monkeypatch.setattr(PublicRobotClient, "request", request)
    monkeypatch.setattr(PublicRobotClient, "preempt", lambda client, reason: calls.append((client.socket_path, "preempt", {"reason": reason})))
    monkeypatch.setattr(PublicRobotClient, "revoke", lambda client, reason: calls.append((client.socket_path, "revoke", {"reason": reason})))
    monkeypatch.setattr(VisionClient, "status", lambda client: {"camera_state": "OFF"})
    controller = Controller(tmp_path)
    interface = RobotInterface(project_root=tmp_path, controller=controller)
    return SimpleNamespace(interface=interface, controller=controller, calls=calls, world=world, behavior=behavior)


def test_default_interface_routes_legacy_behaviors_to_one_public_service(composed):
    first = composed.interface.execute("v3.command.explore", max_v_mps=0.1)
    second = composed.interface.execute("v3.command.follow_person", max_v_mps=0.1)
    requests = [(operation, arguments) for _, operation, arguments in composed.calls if operation == "execute"]
    assert requests == [
        ("execute", {"action": "behavior.room_cruise", "parameters": {"max_v_mps": 0.1}}),
        ("execute", {"action": "behavior.follow_person", "parameters": {"max_v_mps": 0.1}}),
    ]
    assert first["command_id"] == second["command_id"] == "upper-command"
    other = RobotInterface(project_root=composed.controller.root, controller=composed.controller)
    assert other.read("world.snapshot") == composed.world
    assert len({str(path) for path, _, _ in composed.calls}) == 1
    assert composed.controller.calls == []


def test_local_v3_action_survives_public_world_service_failure(monkeypatch, composed):
    def unavailable(*_args, **_kwargs):
        raise RuntimeError("PUBLIC_ROBOT_RUNTIME_UNAVAILABLE")
    monkeypatch.setattr(PublicRobotClient, "request", unavailable)
    monkeypatch.setattr(PublicRobotClient, "preempt", unavailable)
    with pytest.raises(RuntimeError, match="PUBLIC_ROBOT_RUNTIME_UNAVAILABLE"):
        composed.interface.read("world.snapshot")
    handle = composed.interface.execute("v3.command.forward", speed_mps=0.1)
    assert handle.command_id == "local-forward"
    assert [name for name, _ in composed.controller.calls] == ["stop", "forward"]


def test_default_interface_stop_does_not_read_capability_or_world_state(monkeypatch, composed):
    def unavailable():
        raise AssertionError("STOP cannot depend on observation or capability reads")
    monkeypatch.setattr(composed.controller, "status", unavailable)
    monkeypatch.setattr(composed.controller, "live_runtime_status", unavailable)
    result = composed.interface.stop()
    assert result == {"status": "STOPPED"}
    assert composed.controller.calls == [("stop", {})]
    assert composed.calls
    assert all(operation in {"revoke", "preempt"} for _, operation, _ in composed.calls)
    assert all(arguments.get("reason") == "STOP" for _, _, arguments in composed.calls)


def test_default_robot_context_keeps_public_memory_on_demand(composed):
    before = len(composed.calls)
    context = RobotContextBuilder(composed.interface).build()
    environment = context.host["environment"]
    assert "world" not in environment
    assert environment["local_world"] == composed.controller.status_value["world"]
    assert environment["behavior"] == composed.behavior
    assert "robot_state" not in context.host
    reads = [arguments["resource"] for _, operation, arguments in composed.calls[before:] if operation == "read"]
    assert "world.snapshot" not in reads
    assert "robot.state" not in reads


def test_live_behavior_capabilities_reflect_runtime_and_vision_failure(monkeypatch, composed):
    composed.controller.status_value["safety_decision"] = "FAULT"
    capabilities = composed.interface.capabilities()["capabilities"]
    assert capabilities["behavior.room_cruise"]["available"] is False
    assert capabilities["v3.command.explore"]["available"] is False
    assert capabilities["world.snapshot"]["available"] is True
    composed.controller.status_value["safety_decision"] = "STOP"
    monkeypatch.setattr(VisionClient, "status", lambda client: {"camera_state": "FAILED"})
    capabilities = composed.interface.capabilities()["capabilities"]
    assert capabilities["behavior.room_cruise"]["available"] is True
    assert capabilities["behavior.follow_person"]["available"] is False
    assert capabilities["v3.command.follow_person"]["available"] is False
    assert capabilities["behavior.search_person"]["available"] is False
    assert composed.controller.calls == []


def test_conversation_reuses_the_same_public_robot_adapters(monkeypatch, composed):
    import r2b4_voice.conversation_interface as conversation
    import v3.operator_controller as operator

    class Model:
        model = "fake"
        def complete_agent_step(self, *_args, **_kwargs):
            raise AssertionError("composition test must not call a provider")

    class Service:
        def __init__(self, **arguments):
            self.context = arguments["robot_context"]
            self.agent = arguments["agent"]
        def close(self):
            pass

    monkeypatch.setattr(operator, "OperatorController", Controller)
    monkeypatch.setattr(conversation, "build_llm_client", lambda **_kwargs: Model())
    monkeypatch.setattr(conversation, "PromptAssembler", lambda *_args, **_kwargs: object())
    monkeypatch.setattr(conversation, "ConversationJournal", lambda *_args: object())
    monkeypatch.setattr(conversation, "ConversationService", Service)
    with conversation.build_voice_interface(composed.controller.root) as bundle:
        core = bundle.conversation.context._interface
        assert bundle.interface.adapters[:-1] == core.adapters
        public_adapter = next(adapter for adapter in core.adapters if adapter.name == "public_robot")
        assert public_adapter is next(adapter for adapter in bundle.interface.adapters if adapter.name == "public_robot")
        assert bundle.interface.read("world.snapshot") == composed.world
        assert "config.patch" not in {tool["name"] for tool in bundle.conversation.agent.tool_catalog}


def test_cli_reads_and_executes_the_public_robot_surface(monkeypatch, composed, capsys):
    from v3 import interface_cli
    monkeypatch.setattr(interface_cli, "RobotInterface", lambda **_kwargs: composed.interface)
    assert interface_cli.main(["read", "world.snapshot", "--json"], project_root=composed.controller.root) == 0
    assert json.loads(capsys.readouterr().out) == composed.world
    assert interface_cli.main([
        "execute", "behavior.room_cruise", "--parameters", '{"max_duration_s":5}', "--json",
    ], project_root=composed.controller.root) == 0
    assert json.loads(capsys.readouterr().out)["command_id"] == "upper-command"
    assert any(operation == "execute" and arguments["parameters"] == {"max_duration_s": 5}
               for _, operation, arguments in composed.calls)
    assert composed.controller.calls == []


def test_legacy_operator_stop_revokes_upper_intent_without_observation(monkeypatch, composed):
    from v3 import operator_cli

    monkeypatch.setattr(operator_cli, "RobotInterface", lambda **_kwargs: composed.interface)
    def unavailable():
        raise AssertionError("STOP must not read readiness or world state")
    monkeypatch.setattr(composed.controller, "status", unavailable)
    monkeypatch.setattr(composed.controller, "live_runtime_status", unavailable)

    assert operator_cli.main(["stop"]) == 0
    assert composed.controller.calls == [("stop", {})]
    assert [(operation, arguments["reason"]) for _, operation, arguments in composed.calls] == [
        ("revoke", "STOP"), ("preempt", "STOP"),
    ]


def test_legacy_operator_motion_preempts_brain_before_canonical_action(monkeypatch, composed):
    from v3 import operator_cli

    monkeypatch.setattr(operator_cli, "RobotInterface", lambda **_kwargs: composed.interface)
    assert operator_cli.main(["forward", "start", "0.15", "c", "nincs", "nocapture"]) == 0
    assert [(operation, arguments) for _, operation, arguments in composed.calls] == [
        ("preempt", {"reason": "PREEMPTED_BY:v3.command.forward"}),
    ]
    assert composed.controller.calls[0][0] == "forward"
    assert composed.controller.calls[0][1]["speed"] == 0.15
    assert composed.controller.calls[0][1]["capture"] is False
    assert composed.controller.calls[0][1]["capture_mode"] == "nincs"


@pytest.mark.parametrize("command, action", [
    ("roomcruise", "behavior.room_cruise"),
    ("explore", "behavior.room_cruise"),
    ("followperson", "behavior.follow_person"),
])
def test_legacy_operator_behaviors_use_the_shared_owner(monkeypatch, composed, command, action):
    from v3 import operator_cli

    monkeypatch.setattr(operator_cli, "RobotInterface", lambda **_kwargs: composed.interface)
    assert operator_cli.main([command, "c", "nincs"]) == 0
    assert composed.controller.calls == []
    assert len(composed.calls) == 1
    _, operation, arguments = composed.calls[0]
    assert operation == "execute" and arguments["action"] == action


@pytest.mark.parametrize("arguments, action, method", [
    (["shutdown"], "operator.shutdown", "runtime_stop"),
    (["runtime", "stop"], "operator.runtime.stop", "runtime_stop"),
    (["panic", "--quiet"], "operator.panic", "panic"),
    (["proba", "c", "nincs"], "operator.proba", "run_proba"),
])
def test_legacy_operator_lifecycle_and_test_preempt_upper_owner(monkeypatch, composed, arguments, action, method):
    from v3 import operator_cli

    monkeypatch.setattr(operator_cli, "RobotInterface", lambda **_kwargs: composed.interface)
    monkeypatch.setattr(composed.controller, method,
                        lambda **parameters: composed.controller.calls.append((method, parameters)), raising=False)
    assert operator_cli.main(arguments) == 0
    assert [(operation, values) for _, operation, values in composed.calls] == [
        ("preempt", {"reason": "PREEMPTED_BY:" + action}),
    ]
    assert len(composed.controller.calls) == 1 and composed.controller.calls[0][0] == method


def test_private_runtime_session_keeps_its_backend_entry(monkeypatch, tmp_path):
    from v3 import operator_cli

    calls = []
    class WorkerController:
        def run_runtime_session(self, *arguments):
            calls.append(arguments)
            return 0
    monkeypatch.setattr(operator_cli, "OperatorController", lambda **_kwargs: WorkerController())
    def no_public_request(**_kwargs):
        raise AssertionError("The runtime worker must not preempt the task that launched it")
    monkeypatch.setattr(operator_cli, "RobotInterface", no_public_request)
    path = tmp_path / "capture.mcap"
    assert operator_cli.main([
        "__runtime-session", "--capture-path", str(path), "--capture-mode", "nincs", "--capture-hz", "10",
    ]) == 0
    assert calls == [(path, "nincs", 10)]
