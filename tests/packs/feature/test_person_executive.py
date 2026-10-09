"""Person teaching and execution through the production host composition."""
import json
import threading

import pytest

from r2b4_orchestration.local_task_planner import LocalTaskPlanner
from r2b4_orchestration.outcome_learning import read_learning_snapshot
from r2b4_orchestration.robot_runtime import PublicRobotRuntime
from r2b4_orchestration.world_model import PublicWorldModel, WorldQuery
from tests.packs.feature.test_brain_core import wait_for
from tests.packs.feature.test_local_task_execution import Backend, Clock
from v3.robot_interface import RobotInterface


class PersonBackend(Backend):
    capability_names = Backend.capability_names | {"camera.status"}

    def __init__(self, clock):
        self.person_visible = True
        self.vision_generation = "camera-person-a"
        self.mode = "NAVIGATE"
        super().__init__(clock)

    def make_status(self, *, active=False):
        status = super().make_status(active=active)
        status["mission"]["mode"] = self.mode
        status["world"]["person_tracks"] = [{
            "track_id": "person-7", "x_m": 4., "y_m": 3., "confidence": .9,
            "measurement_monotonic_ns": self.clock.now,
            "prediction_valid_until_ns": self.clock.now + 500_000_000,
            "estimate_status": "OBSERVED",
        }] if self.person_visible else []
        return status

    def read(self, resource):
        if resource == "camera.status":
            return {"running": True, "detector_running": True,
                    "owner_generation": self.vision_generation, "owner_pid": 456,
                    "owner_generation_started_ns": 1}
        return super().read(resource)

    def publish(self, *, active=False):
        self.clock.now += 10_000_000
        self.tick += 1
        self.status = self.make_status(active=active)
        if self.owner is not None:
            self.owner.ingest_status(self.status, runtime_pid=self.runtime_pid,
                                     vision_status=self.read("camera.status"))

    def execute(self, action, **parameters):
        if action == "v3.command.follow_person":
            self.actions.append((action, parameters))
            self.commands += 1
            command = "follow-command-" + str(self.commands)
            self.mission_id = "mission-" + command
            self.mode = "FOLLOW_PERSON"
            self.publish(active=True)
            return {"command_id": command, "mission_id": self.mission_id}
        return super().execute(action, **parameters)


def setup_person(root=None):
    clock = Clock()
    backend = PersonBackend(clock)
    interface = RobotInterface(controller=backend, adapters=(backend,), upper_runtime=False, clock_ns=clock)
    world = PublicWorldModel(clock_ns=clock, clock_epoch="person-executive-test")
    owner = PublicRobotRuntime(interface, root=root, world=world, clock_ns=clock)
    backend.owner = owner
    owner.ingest_status(backend.status, runtime_pid=123, vision_status=backend.read("camera.status"))
    return clock, backend, owner


def adopt(owner, text, *, asynchronous=False):
    pending = owner.brain.submit(text)
    plan = LocalTaskPlanner().resolve(text, owner, goal_id=pending["goal_id"]).plan
    assert plan is not None
    return owner.brain.adopt(pending["goal_id"], plan, asynchronous=asynchronous)


def teach_goal(owner, text):
    goal = adopt(owner, text)
    wait_for(lambda: owner.brain.goal(goal["goal_id"])["lifecycle"] in {"COMPLETED", "FAILED"})
    return owner.brain.goal(goal["goal_id"])


def test_teaching_and_named_search_follow_share_person_and_runtime(tmp_path):
    _, backend, owner = setup_person(tmp_path)
    taught = teach_goal(owner, "Ezt a személyt nevezd Annának")
    assert taught["lifecycle"] == "COMPLETED", taught
    assert taught["result"]["durability"] == "SAVED"
    entity = taught["result"]["entity_id"]
    assert backend.actions == [] and backend.stops == 0
    assert owner.person_identity.resolve("Anna") == entity
    following = adopt(owner, "Kövesd Annát 1 másodpercig")
    assert following["lifecycle"] == "ACTIVE", following
    assert [(action, params["target_track_id"], params["expected_runtime_pid"])
            for action, params in backend.actions] == [("v3.command.follow_person", "person-7", 123)]
    completed = [event.state for event in owner.brain.history() if event.kind == "SUBTASK_COMPLETED"]
    found = next(state for state in completed if state.to_jsonable()["current_subtask"] == "behavior.search_person")
    assert dict(found.result)["target_entity_id"] == entity
    assert dict(found.result)["bound_track"] is True
    owner._project_goal_history()
    snapshot = read_learning_snapshot(owner)
    assert any(row[:3] == ("person.named_search", "1", 1) for row in snapshot.methods)
    # A later follow failure cannot replace the already completed search result.
    backend.status.update(safety_decision="STOP", safety_reason="OBSTACLE")
    owner.behaviors.step()
    owner.brain.step()
    assert owner.brain.goal(following["goal_id"])["lifecycle"] == "FAILED"
    assert dict(found.result)["target_entity_id"] == entity
    assert owner.world.read(entity, "identity").value["name"] == "Anna"


def test_restore_keeps_taught_name_but_cannot_follow_restored_track(tmp_path):
    _, _, owner = setup_person(tmp_path)
    taught = teach_goal(owner, "Name this person Anna")
    assert taught["lifecycle"] == "COMPLETED"
    entity = taught["result"]["entity_id"]
    _, backend, restored = setup_person(tmp_path)
    assert restored.person_identity.resolve("Anna") == entity
    binding = restored.world.read(entity, "person_binding")
    assert binding.freshness == "RESTORED_UNVALIDATED"
    following = adopt(restored, "Follow Anna for 1 second")
    assert following["lifecycle"] == "FAILED"
    assert not any(action == "v3.command.follow_person" for action, _ in backend.actions)
    assert json.loads((tmp_path / "runtime/public_world/state.json").read_text())["facts"]


def test_follow_refreshes_continuous_named_measurement_before_host_poll():
    clock, backend, owner = setup_person()
    assert teach_goal(owner, "Name this person Anna")["lifecycle"] == "COMPLETED"
    # The raw runtime advances during admission STOP. The producer does not
    # synchronously update the host; production host polling happens separately.
    def publish_without_host(*, active=False):
        clock.now += 10_000_000
        backend.tick += 1
        backend.status = backend.make_status(active=active)
    backend.publish = publish_without_host
    following = adopt(owner, "Follow Anna for 1 second")
    assert following["lifecycle"] == "ACTIVE", following
    assert backend.actions[0][1]["target_track_id"] == "person-7"


def test_source_restart_during_teaching_read_cannot_create_identity(monkeypatch):
    _, backend, owner = setup_person()
    read = backend.read
    reads = 0
    def changed_source(resource):
        nonlocal reads
        result = read(resource)
        if resource == "camera.status":
            reads += 1
            if reads == 2:
                result["owner_generation"] = "restarted-camera"
        return result
    monkeypatch.setattr(backend, "read", changed_source)
    with pytest.raises(ValueError, match="SESSION_CHANGED"):
        owner.execute("person.teach", {"name": "Anna", "request_id": "explicit-human"})
    assert owner.world.query(WorldQuery(domain="person_identity")).facts == ()
    assert backend.actions == []


def test_failed_storage_is_reported_as_memory_only_without_false_durability(tmp_path, monkeypatch):
    _, backend, owner = setup_person(tmp_path)
    def unavailable(*_):
        raise OSError("test disk unavailable")
    monkeypatch.setattr("r2b4_orchestration.robot_runtime.os.replace", unavailable)
    result = owner.execute("person.teach", {"name": "Anna", "request_id": "explicit-human"})
    assert result["status"] == "TAUGHT" and result["durability"] == "MEMORY_ONLY"
    assert result["storage_error"].startswith("WORLD_SAVE_FAILED:")
    assert owner.person_identity.resolve("Anna") == result["entity_id"]
    assert backend.actions == [] and backend.stops == 0


def test_stop_does_not_wait_for_teaching_storage_and_late_result_does_not_revive_goal(monkeypatch):
    _, backend, owner = setup_person()
    entered, release = threading.Event(), threading.Event()
    def storage(*, force=False):
        entered.set()
        assert release.wait(3)
        return False
    monkeypatch.setattr(owner, "_persist", storage)
    goal = adopt(owner, "Name this person Anna", asynchronous=True)
    assert entered.wait(2)
    stopped = threading.Event()
    thread = threading.Thread(target=lambda: (owner.preempt("USER_STOP"), stopped.set()), daemon=True)
    thread.start()
    try:
        assert stopped.wait(1), "STOP waited for storage"
        assert backend.stops >= 1
        assert owner.brain.goal(goal["goal_id"])["lifecycle"] == "CANCELLED"
    finally:
        release.set()
        thread.join(2)
    wait_for(lambda: any(event.kind == "ACTION_RESULT_AFTER_REVOCATION" for event in owner.brain.history()))
    assert owner.brain.goal(goal["goal_id"])["lifecycle"] == "CANCELLED"
    assert backend.actions == []


@pytest.mark.parametrize("text", ["Ezt a személyt nevezd Annának", "Name this person Anna"])
def test_teaching_with_multiple_people_returns_clarification_without_name_or_motion(text):
    clock, backend, owner = setup_person()
    other = dict(backend.status["world"]["person_tracks"][0], track_id="person-8")
    backend.status["world"]["person_tracks"].append(other)
    goal = teach_goal(owner, text)
    assert goal["lifecycle"] == "FAILED"
    assert "CLARIFICATION" in goal["reason"]
    assert owner.world.query(WorldQuery(domain="person_identity")).facts == ()
    assert backend.actions == [] and backend.stops == 0


def test_r_execute_and_read_use_the_same_canonical_person_owner(monkeypatch, capsys, tmp_path):
    from v3 import interface_cli, launcher_cli
    _, backend, owner = setup_person()
    monkeypatch.setattr(launcher_cli, "project_root", lambda: tmp_path)
    monkeypatch.setattr("v3.runtime_performance.apply_host_affinity", lambda *_: None)
    monkeypatch.setattr(interface_cli, "RobotInterface", lambda **_: owner.brain.robot)
    assert launcher_cli.main(["execute", "person.teach", "--parameters",
        json.dumps({"name": "Anna", "request_id": "launcher-explicit-teaching"}), "--json"]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result["status"] == "TAUGHT"
    assert owner.person_identity.resolve("Anna") == result["entity_id"]
    assert launcher_cli.main(["read", "world.snapshot", "--json"]) == 0
    world = json.loads(capsys.readouterr().out)
    assert any(fact["entity_id"] == result["entity_id"] and fact["attribute"] == "identity"
               for fact in world["facts"])
    assert backend.actions == [] and backend.stops == 0


def test_r_discovers_installed_teaching_and_evidence_resources_without_runtime(capsys):
    from v3 import launcher_cli, launcher_extras
    assert launcher_cli.main(["commands", "--json"]) == 0
    catalog = json.loads(capsys.readouterr().out)
    assert "person.teach" in catalog["public_robot"]["actions"]
    skill = next(skill for skill in catalog["public_robot"]["person_skills"] if skill["name"] == "person.teach")
    assert skill["requires_motion"] is False
    assert "brain.history" in catalog["public_robot"]["resources"]
    assert "person.teach" in launcher_extras._robot_completion("execute", (), "person.")[1]
    assert "world.snapshot" in launcher_extras._robot_completion("read", (), "world.")[1]


@pytest.mark.parametrize("operation, required", [
    ("query", {"--field", "--topic", "--message-id", "--tick-id", "--limit"}),
    ("verify", {"--source"}),
    ("verify-evidence", {"--source"}),
])
def test_r_evi_completion_uses_the_selected_cli_operation_parser(operation, required, tmp_path):
    from tools.mcap_evidence.cli import parser
    from v3.launcher_extras import completion

    hint, candidates = completion(4, ["evi", operation, "BUNDLE", "--"], tmp_path)
    assert required.issubset(candidates)
    assert not {"--output", "--workers", "--overwrite", "--shard-bytes"}.intersection(candidates)
    assert ("r evi query" if operation == "query" else "r evi verify") in hint
    # Suggested flags are the flags the invoked CLI actually accepts.
    arguments = (["BUNDLE", "--field", "/payload/value/task_graph/method_id", "--topic", "/r2b4/event"]
                 if operation == "query" else ["BUNDLE", "--source", "CAPTURE.mcap"])
    parsed = parser(operation).parse_args(arguments)
    assert parsed.bundle == "BUNDLE"
    _, compilation = completion(3, ["evi", "CAPTURE.mcap", "--"], tmp_path)
    assert {"--output", "--workers"}.issubset(compilation)
