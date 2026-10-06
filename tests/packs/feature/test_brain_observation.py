"""Passive Brain/Behavior lineage without a live robot or existing capture data."""
from __future__ import annotations

import threading
import time
from dataclasses import FrozenInstanceError
from types import SimpleNamespace

import pytest

from r2b4_orchestration.behavior_system import BehaviorEvent
from r2b4_orchestration.brain_core import BrainEvent
from r2b4_orchestration.robot_runtime import PublicRobotRuntime
from r2b4_orchestration.world_model import PublicWorldModel
from v3.observation import ObservationHub


class Clock:
    def __init__(self):
        self.now = time.monotonic_ns()

    def __call__(self):
        return self.now


class Robot:
    """Only the public execution/status surface; no hardware or subprocesses."""
    def __init__(self, clock):
        self.clock = clock
        self.actions = []
        self.stops = 0
        self.status = {}

    def capabilities(self):
        return {"capabilities": {}}

    def execute(self, action, **parameters):
        self.actions.append((action, parameters))
        command_id = f"command-{len(self.actions)}"
        self.status = {
            "monotonic_ns": self.clock.now, "state": "RUNNING", "fault_layer": None,
            "safety_decision": "ALLOW", "safety_reason": "ALLOW",
            "mission": {"mission_id": "mission-" + command_id, "mode": "EXPLORE", "lifecycle": "ACTIVE"},
            "navigation": {"mission_id": "mission-" + command_id, "status": "ACTIVE", "reason": None},
        }
        return {"command_id": command_id}

    def read(self, name):
        if name == "v3.status":
            return self.status
        if name == "operator.status":
            return {"runtime_pid": 123}
        raise KeyError(name)

    def stop(self):
        self.stops += 1


def setup_runtime(*, hub=None, root=None):
    clock = Clock()
    robot = Robot(clock)
    world = PublicWorldModel(clock_ns=clock, clock_epoch="observation-test")
    runtime = PublicRobotRuntime(robot, root=root, world=world, clock_ns=clock, observation_hub=hub)
    return clock, robot, runtime


def start(runtime):
    submitted = runtime.execute("brain.submit", {"text": "Menj körbe 2 másodpercig.", "request_id": "goal-observed"})
    admitted = runtime.execute("brain.adopt", {"goal_id": submitted["goal_id"], "plan": {"steps": [
        {"action": "behavior.room_cruise", "parameters": {"max_duration_s": 2}, "completion": "duration"},
    ]}})
    goal = wait_goal(runtime, admitted["goal_id"], {"ACTIVE", "COMPLETED", "FAILED", "CANCELLED"})
    assert goal["lifecycle"] == "ACTIVE", goal
    return goal


def wait_goal(runtime, goal_id, lifecycles):
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        goal = runtime.brain.goal(goal_id)
        if goal["lifecycle"] in lifecycles:
            return goal
        time.sleep(0.001)
    pytest.fail(f"Brain goal did not reach {lifecycles}: {goal}")


def complete(clock, robot, runtime):
    runtime.behaviors.step()
    clock.now = runtime.behaviors.snapshot().deadline_ns
    robot.status["monotonic_ns"] = clock.now
    runtime.behaviors.step()
    runtime.brain.step()


def test_brain_and_downstream_lineage_reach_hub_without_capture():
    hub = ObservationHub()
    subscription = hub.subscribe_reliable("observer", capacity=64, topics={"r2b4.brain", "r2b4.behavior"})
    _, robot, runtime = setup_runtime(hub=hub)
    goal = start(runtime)
    frames = subscription.drain()
    brain = [frame.payload for frame in frames if frame.topic == "r2b4.brain"]
    behavior = [frame.payload for frame in frames if frame.topic == "r2b4.behavior"]
    assert all(isinstance(event, BrainEvent) for event in brain)
    assert all(isinstance(event, BehaviorEvent) for event in behavior)
    assert all(event is expected for event, expected in zip(brain, runtime.brain.history()))
    dispatch = next(event for event in brain if event.kind == "SUBTASK_DISPATCHED")
    result = next(event for event in behavior if event.kind == "BEHAVIOR_ACTION_RESULT")
    assert result.state.goal_id == dispatch.state.goal_id == goal["goal_id"]
    assert result.state.subtask_id == dispatch.state.subtask_id
    assert result.state.decision_id == dispatch.decision_id
    assert result.action_command_id == goal["command_id"] == "command-1"
    assert result.action_mission_id == goal["mission_id"] == "mission-command-1"
    with pytest.raises(FrozenInstanceError):
        dispatch.kind = "modified"
    assert robot.actions[0][0] == "v3.command.explore"
    assert runtime.root is None


def test_slow_finite_action_admits_promptly_and_publishes_later_completion(monkeypatch):
    hub = ObservationHub()
    subscription = hub.subscribe_reliable("observer", capacity=64, topics={"r2b4.brain"})
    _, robot, runtime = setup_runtime(hub=hub)
    entered, release = threading.Event(), threading.Event()
    monkeypatch.setattr(robot, "capabilities", lambda: {"capabilities": {
        "v3.command.move_relative": {"available": True, "supported": True, "kind": "action"},
    }})

    def slow_physical_action(action, **parameters):
        assert action == "v3.command.move_relative"
        entered.set()
        assert release.wait(2)
        return {"command_id": "late-command", "mission_id": "mission-late-command",
                "status": "COMPLETED", "reason": "COMPLETE"}

    monkeypatch.setattr(robot, "execute", slow_physical_action)
    submitted = runtime.execute("brain.submit", {"text": "Menj előre fél métert."})
    started = time.monotonic()
    try:
        admitted = runtime.execute("brain.adopt", {"goal_id": submitted["goal_id"], "plan": {"steps": [
            {"action": "v3.command.move_relative", "parameters": {"forward_m": 0.5}},
        ]}})
        assert time.monotonic() - started < 0.5
        assert admitted["lifecycle"] == "STARTING"
        assert entered.wait(1)
        before = [frame.payload for frame in subscription.drain()]
        assert any(event.kind == "PLAN_ADOPTED" for event in before)
        assert all(event.kind != "GOAL_COMPLETED" for event in before)
    finally:
        release.set()
    completed = wait_goal(runtime, submitted["goal_id"], {"COMPLETED", "FAILED", "CANCELLED"})
    assert completed["lifecycle"] == "COMPLETED", completed
    after = [frame.payload for frame in subscription.drain()]
    result = next(event for event in after if event.kind == "ACTION_RESULT")
    terminal = next(event for event in after if event.kind == "GOAL_COMPLETED")
    assert result.state.command_id == terminal.state.command_id == "late-command"
    assert result.state.mission_id == terminal.state.mission_id == "mission-late-command"
    assert result.state.goal_id == submitted["goal_id"]


@pytest.mark.parametrize("failure", ["overflow", "closed", "broken"])
def test_observation_or_journal_failure_cannot_change_goal_execution(failure, monkeypatch):
    hub = ObservationHub()
    subscription = hub.subscribe_reliable("slow", capacity=1, required=True)
    if failure == "closed":
        hub.close()
    elif failure == "broken":
        class BrokenHub:
            def publish(self, *args, **kwargs):
                raise OSError("observation unavailable")
        hub = BrokenHub()
    clock, robot, runtime = setup_runtime(hub=hub)
    if failure == "overflow":
        for index in range(300):
            runtime._queue_evidence("maintenance", index)
        assert subscription.snapshot().lost_count > 0
        assert runtime._evidence_dropped > 0

    def unavailable_journal(record):
        raise OSError("storage unavailable")

    monkeypatch.setattr(runtime, "_evidence", unavailable_journal)
    goal = start(runtime)
    runtime._flush_evidence()
    assert runtime.brain.goal(goal["goal_id"])["lifecycle"] == "ACTIVE"
    complete(clock, robot, runtime)
    assert runtime.brain.goal(goal["goal_id"])["lifecycle"] == "COMPLETED"
    assert len(robot.actions) == 1
    assert robot.stops >= 1


def test_brain_behavior_command_lineage_survives_real_passive_capture_edge(tmp_path):
    from rig import resolved_config
    from v3.composition.full_fake import OfflineMotorSink
    from v3.composition.native_control import NativeControlComposition
    from v3.contracts import CommandMode, CommandRequest, LifecycleState, RawDeviceBatch, TickContext
    from v3.engine import TickInputs
    from v3.execution import ExecutionRecord
    from v3.mcap_capture import EVENT_TOPIC, TICK_TOPIC, McapCaptureConfig
    from v3.mcap_reader import McapReader
    from v3.public_runtime_evidence import PUBLIC_RUNTIME_EVENT_TOPIC
    from v3_process_runtime import McapCaptureSession

    _, _, runtime = setup_runtime(root=tmp_path)
    session = McapCaptureSession("brain-lineage", tmp_path / "brain.mcap", configuration={},
                                project_root=tmp_path,
                                config=McapCaptureConfig(mode="append_only", tick_sample_hz=10))
    session.start()
    try:
        goal = start(runtime)
        runtime._flush_evidence()
        composition = NativeControlComposition(OfflineMotorSink(), resolved_config().runtime.composition.live_control.control)
        context = TickContext(0, time.monotonic_ns())
        inputs = composition.close_inputs(TickInputs(
            context, RawDeviceBatch(context, (), ()),
            CommandRequest(context, goal["command_id"], CommandMode.EXPLORE, (), 0), LifecycleState.ACTIVE))
        session.observe(ExecutionRecord(inputs, composition.run_tick(inputs)))
    finally:
        output = session.finalize(SimpleNamespace(status=0))
    reader = McapReader(output)
    public = [row["payload"] for _, row in reader.iter_json_messages(topics=[EVENT_TOPIC])
              if row.get("source_topic") == PUBLIC_RUNTIME_EVENT_TOPIC]
    brain = [row["value"] for row in public if row["kind"] == "brain"]
    behavior = [row["value"] for row in public if row["kind"] == "behavior"]
    dispatch = next(event for event in brain if event["kind"] == "SUBTASK_DISPATCHED")
    result = next(event for event in behavior if event["kind"] == "BEHAVIOR_ACTION_RESULT")
    assert result["goal_id"] == dispatch["goal_id"] == goal["goal_id"]
    assert result["subtask_id"] == dispatch["subtask_id"]
    assert result["decision_id"] == dispatch["decision_id"]
    assert result["action_command_id"] == goal["command_id"]
    assert result["action_mission_id"] == goal["mission_id"]
    tick = next(row for _, row in reader.iter_json_messages(topics=[TICK_TOPIC]))
    assert tick["inputs"]["command"]["command_id"] == result["action_command_id"]
    assert tick["inputs"]["command"]["mode"] == "EXPLORE"
    assert reader.capture_integrity()["integrity"]["complete"] is True
