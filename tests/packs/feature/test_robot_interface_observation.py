"""Compact passive interface evidence without hardware or filesystem writes."""
from __future__ import annotations

from dataclasses import FrozenInstanceError
import json
from types import SimpleNamespace

import pytest

from r2b4_orchestration.robot_runtime import PublicRobotRuntime
from v3.observation import ObservationHub
from v3.robot_interface import RobotInterface, RobotInterfaceEvent


class Adapter:
    name = "fake"
    capability_names = frozenset({"x.action", "v3.command.stop"})

    def __init__(self, *, result=None, error=None, calls=None):
        self.result = result if result is not None else {"command_id": "command-1", "status": "ACCEPTED"}
        self.error = error
        self.calls = calls if calls is not None else []

    def capabilities(self):
        return {name: {"kind": "action", "supported": True, "available": True}
                for name in self.capability_names}

    def execute(self, action, **parameters):
        self.calls.append(("dispatch", action))
        if self.error is not None:
            raise self.error
        return self.result


def interface(tmp_path, adapter, sink=None, *, clock_ns=lambda: 100):
    return RobotInterface(project_root=tmp_path, controller=SimpleNamespace(),
                          adapters=(adapter,), upper_runtime=False,
                          observation_sink=sink, clock_ns=clock_ns)


def test_action_intent_and_result_preserve_identity_time_and_compact_payload(tmp_path):
    class RawPayload:
        def __repr__(self):
            raise AssertionError("raw payload must never be inspected")

    raw = RawPayload()
    result = {"command_id": "command-1", "mission_id": "mission-1", "runtime_pid": 123,
              "status": "COMPLETED", "reason": "COMPLETE", "raw": raw,
              "world": {"map": raw}}
    adapter = Adapter(result=result)
    events = []
    clock = iter((100, 110, 200))
    robot = interface(tmp_path, adapter, events.append, clock_ns=lambda: next(clock))
    received = robot.execute("x.action", goal_id="goal-1", subtask_id="task-1",
                             decision_id="decision-1", behavior_id="behavior-1", raw=raw)

    assert received is result
    assert adapter.calls == [("dispatch", "x.action")]
    assert [event.kind for event in events] == ["ACTION_REQUESTED", "ACTION_RESULT"]
    assert all(isinstance(event, RobotInterfaceEvent) for event in events)
    assert events[0].request_id == events[1].request_id
    assert all(event.measurement_time_ns == 100 for event in events)
    assert [event.observation_time_ns for event in events] == [110, 200]
    assert events[1].goal_id == "goal-1"
    assert events[1].subtask_id == "task-1"
    assert events[1].decision_id == "decision-1"
    assert events[1].behavior_id == "behavior-1"
    assert events[1].command_id == "command-1"
    assert events[1].mission_id == "mission-1"
    assert events[1].runtime_pid == 123
    assert events[1].status == "COMPLETED"
    assert events[1].reason == "COMPLETE"
    encoded = json.dumps(events[1].to_jsonable(), allow_nan=False)
    assert "raw" not in encoded and "world" not in encoded
    with pytest.raises(FrozenInstanceError):
        events[1].status = "changed"


def test_legacy_alias_records_one_request_and_one_dispatch(tmp_path):
    class PublicAdapter(Adapter):
        name = "public_robot"
        capability_names = frozenset({"behavior.room_cruise", "behavior.follow_person"})

    adapter = PublicAdapter()
    events = []
    robot = interface(tmp_path, adapter, events.append)
    received = robot.execute("v3.command.explore", max_v_mps=0.1)

    assert received is adapter.result
    assert adapter.calls == [("dispatch", "behavior.room_cruise")]
    assert [event.kind for event in events] == ["ACTION_REQUESTED", "ACTION_RESULT"]
    assert all(event.action == "v3.command.explore" for event in events)
    assert all(event.resolved_action == "behavior.room_cruise" for event in events)
    assert len({event.request_id for event in events}) == 1


@pytest.mark.parametrize("invoke", [lambda robot: robot.stop(), lambda robot: robot.execute("v3.command.stop")])
def test_stop_reaches_owner_before_evidence_and_preserves_result(tmp_path, invoke):
    order = []
    adapter = Adapter(calls=order)
    def sink(event):
        order.append(("observation", event.kind))
        raise OSError("observer unavailable")

    robot = interface(tmp_path, adapter, sink)
    assert invoke(robot) is adapter.result
    assert order == [("dispatch", "v3.command.stop"), ("observation", "ACTION_RESULT")]


def test_action_error_keeps_original_exception_and_request_lineage(tmp_path):
    error = RuntimeError("adapter failed")
    events = []
    robot = interface(tmp_path, Adapter(error=error), events.append)
    with pytest.raises(RuntimeError) as caught:
        robot.execute("x.action", command_id="attempt-1")

    assert caught.value is error
    assert [event.kind for event in events] == ["ACTION_REQUESTED", "ACTION_ERROR"]
    assert len({event.request_id for event in events}) == 1
    assert events[1].command_id == "attempt-1"
    assert events[1].error_type == "RuntimeError"


def test_broken_observer_cannot_change_ordinary_action(tmp_path):
    adapter = Adapter()
    def sink(_event):
        raise OSError("observer unavailable")

    robot = interface(tmp_path, adapter, sink)
    assert robot.execute("x.action") is adapter.result
    assert adapter.calls == [("dispatch", "x.action")]


def test_runtime_backend_and_brain_facade_share_existing_evidence_edge(tmp_path):
    hub = ObservationHub()
    consumer = hub.subscribe_reliable("interface-evidence", capacity=8, topics={"r2b4.interface"})
    adapter = Adapter()
    backend = interface(tmp_path, adapter)
    runtime = PublicRobotRuntime(backend, observation_hub=hub, clock_ns=lambda: 150)
    persisted = []
    runtime._evidence = persisted.append

    assert runtime.interface is backend
    assert backend.execute("x.action") is adapter.result
    assert runtime.brain.robot.execute("x.action") is adapter.result
    events = [frame.payload for frame in consumer.drain()]
    assert len(events) == 4
    assert all(isinstance(event, RobotInterfaceEvent) for event in events)
    assert all(event.measurement_time_ns == 150 for event in events)
    assert len({event.request_id for event in events}) == 2
    # Publication only enqueues; the existing host drain performs persistence.
    assert persisted == []
    runtime._flush_evidence()
    assert len(persisted) == 4
    assert all(row["kind"] == "interface" for row in persisted)
    assert [row["value"] for row in persisted] == [event.to_jsonable() for event in events]


def test_optional_observation_does_not_read_clock_without_consumer(tmp_path):
    def unavailable_clock():
        raise AssertionError("an unobserved action needs no evidence clock")

    adapter = Adapter()
    robot = interface(tmp_path, adapter, clock_ns=unavailable_clock)
    assert robot.execute("x.action") is adapter.result
