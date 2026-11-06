"""Executive semantic targets and shared outcomes through the public runtime."""
from __future__ import annotations

import threading
import json
from contextlib import contextmanager
from types import SimpleNamespace

import pytest

from r2b4_orchestration.robot_runtime import PublicRobotRuntime
from r2b4_orchestration.world_model import PublicWorldModel, WorldQuery
from v3.action_catalog import ACTION_CATALOG


class Clock:
    now = 20_000_000_000

    def __call__(self):
        return self.now


class Robot:
    def __init__(self, clock):
        self.clock = clock
        self.actions = []
        self.stops = 0
        self.status = {}
        self.runtime = {"runtime_running": True, "runtime_pid": 123,
                        "capture_mode": "full", "capture_hz": 10}

    def capabilities(self):
        return {"capabilities": {action: {"available": True} for action in ACTION_CATALOG}}

    def read(self, resource):
        if resource == "operator.status":
            return self.runtime
        if resource == "v3.status":
            return self.status
        raise KeyError(resource)

    def execute(self, action, **params):
        self.actions.append((action, params))
        command = f"command-{len(self.actions)}"
        self.status = {"monotonic_ns": self.clock.now, "state": "RUNNING",
                       "safety_decision": "ALLOW", "safety_reason": "ALLOW", "fault_layer": None,
                       "mission": {"mission_id": "mission-" + command, "mode": "NAVIGATE",
                                   "lifecycle": "ACTIVE"}}
        return {"command_id": command, "mission_id": "mission-" + command,
                "status": "COMPLETED", "reason": "COMPLETE"}

    def stop(self):
        self.stops += 1


def setup():
    clock = Clock()
    robot = Robot(clock)
    world = PublicWorldModel(clock_ns=clock, clock_epoch="brain-world-test")
    owner = PublicRobotRuntime(robot, world=world, clock_ns=clock)
    return clock, robot, world, owner


def observe_place(world, clock, *, x=1.0, runtime_pid=123, frame_id="R2B4_BOOT_ROBOT_MAP",
                  confidence=1.0, source="semantic-map", lineage=("map:1",), sequence=1, **extra):
    return world.observe("konyha", "location", {"x_m": x, "y_m": 2.0,
        "frame_id": frame_id, "runtime_pid": runtime_pid}, domain="room_topology",
        measurement_time_ns=clock.now, confidence=confidence, source=source,
        lineage=lineage, sequence=sequence, **extra)


def plan():
    return {"steps": [{"action": "v3.command.navigate", "target_entity_id": "konyha",
                       "parameters": {"max_v_mps": 0.2}}]}


def test_brain_resolves_semantic_place_without_agent_coordinate_authority():
    clock, robot, world, owner = setup()
    observed = observe_place(world, clock)
    pending = owner.brain.submit("Menj a konyhába.")
    goal = owner.brain.adopt(pending["goal_id"], plan())
    assert goal["lifecycle"] == "ACTIVE"
    action, params = robot.actions[0]
    assert action == "v3.command.navigate"
    assert (params["x_m"], params["y_m"], params["frame_id"]) == (1.0, 2.0, "R2B4_BOOT_ROBOT_MAP")
    assert params["expected_runtime_pid"] == 123
    assert params["capture"] is False and params["capture_mode"] == "full"
    fact = goal["world_target"]
    assert fact["world_revision"] == observed.world_revision
    assert fact["measurement_time_ns"] == clock.now
    assert fact["source"] == "semantic-map" and fact["lineage"] == ["map:1"]
    assert fact["sequence"] == 1 and fact["clock_epoch"] == "brain-world-test"
    assert fact["validity_scope"]["runtime_pid"] == 123
    event = next(event for event in owner.brain.history() if event.kind == "SUBTASK_DISPATCHED")
    assert event.state.world_target.observation is observed.observation


@pytest.mark.parametrize("invalid", ["missing", "old_runtime", "clock_epoch", "unknown_frame",
                                    "no_runtime", "no_lineage", "low_confidence", "conflicting"])
def test_unqualified_semantic_place_never_becomes_motion(invalid):
    clock, robot, world, owner = setup()
    if invalid != "missing":
        extras = {}
        if invalid == "old_runtime":
            extras["runtime_pid"] = 456
        elif invalid == "clock_epoch":
            previous = PublicWorldModel(clock_ns=clock, clock_epoch="old-boot")
            observe_place(previous, clock)
            world.restore(previous.export_state())
        elif invalid == "unknown_frame":
            extras["frame_id"] = "unbound-map"
        elif invalid == "no_lineage":
            extras["lineage"] = ()
        elif invalid == "low_confidence":
            extras["confidence"] = 0.0
        if invalid != "clock_epoch":
            observe_place(world, clock, **extras)
        if invalid == "conflicting":
            observe_place(world, clock, x=10, source="other-map")
    if invalid == "no_runtime":
        robot.runtime["runtime_running"] = False
    goal = owner.brain.submit("Menj a konyhába.")
    result = owner.brain.adopt(goal["goal_id"], plan())
    assert result["lifecycle"] == "FAILED"
    assert "WORLD_TARGET_UNAVAILABLE" in result["reason"]
    assert robot.actions == []


def test_each_semantic_subtask_resolves_latest_world_instead_of_plan_time_coordinates():
    clock, robot, world, owner = setup()
    observe_place(world, clock)
    execute = robot.execute

    def move_then_update(action, **params):
        result = execute(action, **params)
        if action == "v3.command.move_relative":
            clock.now += 1
            observe_place(world, clock, x=3.0, sequence=2)
        return result

    robot.execute = move_then_update
    pending = owner.brain.submit("Menj előre, aztán a konyhába.")
    proposal = plan()
    proposal["steps"].insert(0, {"action": "v3.command.move_relative", "parameters": {"forward_m": 0.5}})
    goal = owner.brain.adopt(pending["goal_id"], proposal)
    assert goal["lifecycle"] == "ACTIVE" and goal["step_index"] == 1
    assert robot.actions[-1][1]["x_m"] == 3.0
    assert goal["world_target"]["sequence"] == 2


def test_stop_during_semantic_lookup_revokes_the_resolved_target_before_execution(monkeypatch):
    clock, robot, world, owner = setup()
    observe_place(world, clock)
    entered, release = threading.Event(), threading.Event()
    query = world.query

    def delayed_query(value):
        entered.set()
        assert release.wait(2)
        return query(value)

    monkeypatch.setattr(world, "query", delayed_query)
    pending = owner.brain.submit("Menj a konyhába.")
    worker = threading.Thread(target=lambda: owner.brain.adopt(pending["goal_id"], plan()))
    worker.start()
    try:
        assert entered.wait(1)
        owner.revoke("STOP")
        assert owner.brain.goal(pending["goal_id"])["lifecycle"] == "CANCELLED"
    finally:
        release.set()
        worker.join(2)
    assert not worker.is_alive()
    assert robot.actions == []


@pytest.mark.parametrize("row", [
    {"action": "v3.command.navigate", "target_entity_id": "konyha", "parameters": {"x_m": 1, "y_m": 2}},
    {"action": "behavior.follow_person", "target_entity_id": "Laci"},
])
def test_semantic_target_cannot_silently_override_coordinates_or_other_actions(row):
    _, robot, _, owner = setup()
    pending = owner.brain.submit("Semantic goal")
    result = owner.brain.adopt(pending["goal_id"], {"steps": [row]})
    assert result["lifecycle"] == "FAILED"
    assert "WORLD_TARGET" in result["reason"]
    assert robot.actions == []


def test_only_terminal_goal_truth_enters_shared_task_memory_even_if_observation_is_broken(monkeypatch):
    _, robot, world, owner = setup()
    class BrokenHub:
        def publish(self, *args, **kwargs):
            raise OSError("observation unavailable")

    monkeypatch.setattr(owner, "observation_hub", BrokenHub())
    pending = owner.brain.submit("Menj előre.")
    owner.brain.adopt(pending["goal_id"], {"steps": [{"action": "v3.command.move_relative",
                                                   "parameters": {"forward_m": 0.5}}]})
    assert world.query(WorldQuery(domain="task_outcome")).facts == ()
    owner.poll()
    fact = world.query(WorldQuery(entity_id="goal:" + pending["goal_id"], domain="task_outcome")).facts[0]
    assert fact.value["lifecycle"] == "COMPLETED"
    assert fact.value["command_id"] == "command-1" and fact.value["mission_id"] == "mission-command-1"
    assert fact.observation.lineage and fact.observation.source.startswith("brain:")
    assert len(robot.actions) == 1
    sequence = world.export_state()["event_sequence"]
    owner.poll()
    assert world.export_state()["event_sequence"] == sequence


def test_command_acceptance_is_not_a_task_outcome_and_stop_records_cancellation():
    clock, robot, world, owner = setup()
    observe_place(world, clock)
    pending = owner.brain.submit("Menj a konyhába.")
    owner.brain.adopt(pending["goal_id"], plan())
    owner.poll()
    assert world.query(WorldQuery(domain="task_outcome")).facts == ()
    owner.preempt("STOP")
    owner.poll()
    outcome = world.query(WorldQuery(domain="task_outcome")).facts[0]
    assert outcome.value["lifecycle"] == "CANCELLED" and outcome.value["reason"] == "STOP"
    assert outcome.value["world_target"]["entity_id"] == "konyha"
    assert robot.stops > 0


def test_memory_projection_failure_does_not_revoke_independent_execution(monkeypatch):
    _, robot, _, owner = setup()
    monkeypatch.setattr(owner.projector, "completed_goal", lambda event: (_ for _ in ()).throw(OSError()))
    pending = owner.brain.submit("Menj előre.")
    result = owner.brain.adopt(pending["goal_id"], {"steps": [{"action": "v3.command.move_relative",
                                                           "parameters": {"forward_m": 0.5}}]})
    owner.poll_safely()
    assert result["lifecycle"] == owner.brain.goal(pending["goal_id"])["lifecycle"] == "COMPLETED"
    assert owner.read("robot.state")["health"]["reason"].startswith("BRAIN_MEMORY_PROJECTION_FAILED")
    before = robot.stops
    owner.preempt("STOP")
    assert robot.stops > before


def test_restart_preserves_target_provenance_without_restarting_semantic_motion():
    clock, _, world, owner = setup()
    observe_place(world, clock)
    pending = owner.brain.submit("Menj a konyhába.")
    before = owner.brain.adopt(pending["goal_id"], plan())
    _, other_robot, _, restored = setup()
    assert restored.brain.restore(owner.brain.export_state())
    after = restored.brain.goal(pending["goal_id"])
    assert after["lifecycle"] == "INTERRUPTED"
    assert after["world_target"] == before["world_target"]
    assert after["steps"][0]["target_entity_id"] == "konyha"
    assert other_robot.actions == []


def test_brain_plan_cannot_admit_operator_wheel_setpoints():
    _, robot, _, owner = setup()
    pending = owner.brain.submit("Move")
    result = owner.brain.adopt(pending["goal_id"], {"steps": [{"action": "v3.command.wheels",
        "parameters": {"left_mps": 0.2, "right_mps": 0.2}}]})
    assert result["lifecycle"] == "FAILED"
    assert result["reason"] == "CAPABILITY_UNSUPPORTED:v3.command.wheels"
    assert robot.actions == []


@pytest.mark.parametrize("was_running", [False, True])
def test_reboot_preserves_old_outcome_epoch_and_records_only_new_interruption(was_running):
    clock, _, world, owner = setup()
    pending = owner.brain.submit("Executive task")
    if was_running:
        observe_place(world, clock)
        owner.brain.adopt(pending["goal_id"], plan())
    else:
        owner.brain.fail(pending["goal_id"], "ANSWERED")
    owner.poll()
    entity = "goal:" + pending["goal_id"]
    before = world.read(entity, "outcome")
    saved_world, saved_brain = world.export_state(), owner.brain.export_state()
    new_clock = Clock()
    new_clock.now = clock.now + 1_000_000_000
    new_world = PublicWorldModel(clock_ns=new_clock, clock_epoch="new-boot")
    new_world.restore(saved_world)
    new_robot = Robot(new_clock)
    restored = PublicRobotRuntime(new_robot, world=new_world, clock_ns=new_clock)
    assert restored.brain.restore(saved_brain) is was_running
    restored.poll()
    after = new_world.read(entity, "outcome")
    if was_running:
        assert after.state.value == "KNOWN" and after.value["lifecycle"] == "INTERRUPTED"
        assert after.observation.clock_epoch == "new-boot"
        assert after.observation.measurement_time_ns == new_clock.now
        assert after.value["reason"] == "RUNTIME_RESTART"
    else:
        assert after.state.value == "STALE" and after.freshness == "CLOCK_MISMATCH"
        assert after.observation.to_jsonable() == before.observation.to_jsonable()
    assert new_robot.actions == []

    # Restoring an already recorded interruption again is historical knowledge.
    later_clock = Clock()
    later_clock.now = new_clock.now + 1_000_000_000
    later_world = PublicWorldModel(clock_ns=later_clock, clock_epoch="later-boot")
    later_world.restore(new_world.export_state())
    later = PublicRobotRuntime(Robot(later_clock), world=later_world, clock_ns=later_clock)
    assert later.brain.restore(restored.brain.export_state()) is False
    later.poll()
    assert later_world.read(entity, "outcome").observation.to_jsonable() == after.observation.to_jsonable()


def test_large_place_description_does_not_amplify_into_brain_history_wire_payload():
    from r2b4_orchestration.robot_runtime import MAX_REPLY_BYTES

    clock, robot, world, owner = setup()
    observation = world.observe("konyha", "location", {"x_m": 1, "y_m": 2,
        "frame_id": "R2B4_BOOT_ROBOT_MAP", "runtime_pid": 123, "description": "x" * 15_000},
        domain="room_topology", measurement_time_ns=clock.now, confidence=1.0,
        source="semantic-map", lineage=("map:1",))
    for index in range(28):
        pending = owner.brain.submit(f"Menj a konyhába {index}")
        result = owner.brain.adopt(pending["goal_id"], plan())
        assert result["lifecycle"] == "ACTIVE"
    history = owner.read("brain.history")
    assert len(json.dumps(history).encode()) < MAX_REPLY_BYTES
    assert owner.brain.history()[-1].state.world_target.observation is observation.observation
    assert "description" not in history[-1]["world_target"]["value"]
    assert world.read("konyha", "location").value["description"] == "x" * 15_000
    assert len(robot.actions) == 28


def test_oversized_world_lineage_fails_explicitly_instead_of_truncating_provenance():
    clock, robot, world, owner = setup()
    observe_place(world, clock, lineage=("x" * 10_000,))
    pending = owner.brain.submit("Menj a konyhába.")
    result = owner.brain.adopt(pending["goal_id"], plan())
    assert result["lifecycle"] == "FAILED"
    assert "WORLD_TARGET_EVIDENCE_EXCEEDS_BOUND" in result["reason"]
    assert robot.actions == []


@pytest.mark.parametrize("admission_pid", [123, 456])
def test_semantic_brain_goal_uses_real_public_navigation_session_transaction(admission_pid):
    from v3.adapters.operator import OperatorInterfaceAdapter
    from v3.adapters.v3_control import V3ControlInterfaceAdapter
    from v3.robot_interface import RobotInterface

    class Controller:
        locked = False

        def __init__(self):
            self.navigations = []

        def status(self):
            return {"runtime_running": True, "runtime_pid": 123,
                    "capture_mode": "full", "capture_hz": 10}

        def live_runtime_status(self):
            return {"state": "RUNNING", "ready_for_active": True}

        def snapshot(self):
            assert self.locked
            return SimpleNamespace(runtime_pid=admission_pid)

        def current_capture_mode(self):
            return "full"

        def current_capture_path(self):
            return None

        def current_capture_hz(self):
            return 10

        @contextmanager
        def operator_transition(self):
            self.locked = True
            try:
                yield
            finally:
                self.locked = False

        def navigate(self, **params):
            assert self.locked
            self.navigations.append(params)
            return {"command_id": "world-command"}

        def stop(self):
            pass

    clock = Clock()
    world = PublicWorldModel(clock_ns=clock)
    observe_place(world, clock)
    controller = Controller()
    facade = RobotInterface(controller=controller,
                            adapters=(OperatorInterfaceAdapter(controller), V3ControlInterfaceAdapter(controller)),
                            upper_runtime=False)
    owner = PublicRobotRuntime(facade, world=world, clock_ns=clock)
    owner.ingest_status({"monotonic_ns": clock.now, "tick_id": 1,
        "estimate": {"frame_id": "R2B4_BOOT_ROBOT_MAP", "localization_quality": {"generation": 1}},
        "world": {"frame_id": "R2B4_BOOT_ROBOT_MAP"}}, runtime_pid=123)
    pending = owner.brain.submit("Menj a konyhába.")
    goal = owner.brain.adopt(pending["goal_id"], plan())
    if admission_pid == 123:
        assert goal["lifecycle"] == "ACTIVE" and goal["command_id"] == "world-command"
        assert len(controller.navigations) == 1
        assert controller.navigations[0]["capture"] is False
        assert controller.navigations[0]["x_m"] == 1.0
    else:
        assert goal["lifecycle"] == "FAILED"
        assert "runtime session changed before admission" in goal["reason"]
        assert controller.navigations == []
