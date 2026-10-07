"""Shared host state and STOP races through simulated public robot operations."""

from __future__ import annotations

import threading
import time

import pytest

from r2b4_orchestration.robot_runtime import (
    MAX_REQUEST_BYTES,
    PublicRobotClient,
    PublicRobotInterfaceAdapter,
    PublicRobotRuntime,
)
from r2b4_orchestration.world_model import PublicWorldModel
from v3.robot_interface import RobotInterface


class Clock:
    now = 20_000_000_000

    def __call__(self):
        return self.now


class Backend:
    def __init__(self, clock):
        self.clock = clock
        self.actions = []
        self.operations = []
        self.stopped = threading.Event()
        self.status = self.snapshot()

    def snapshot(self, *, mission_id="idle", mode="STOP", lifecycle="IDLE", tracks=()):
        return {
            "monotonic_ns": self.clock.now - 10,
            "tick_id": 1, "state": "RUNNING", "fault_layer": None,
            "safety_decision": "ALLOW" if lifecycle == "ACTIVE" else "STOP",
            "safety_reason": "ALLOW" if lifecycle == "ACTIVE" else "NOT_ACTIVE",
            "source_health": [{"device_id": "lidar", "state": "READY", "reason": None}],
            "estimate": {"frame_id": "R2B4_ODOM_LOCAL", "x_m": 1.0, "y_m": 2.0,
                         "yaw_rad": 0.2, "localization_quality": {"global_position": "GOOD"}},
            "world": {"frame_id": "R2B4_ODOM_LOCAL", "person_tracks": list(tracks)},
            "mission": {"mission_id": mission_id, "mode": mode, "lifecycle": lifecycle},
            "navigation": {"mission_id": mission_id,
                           "status": "ACTIVE" if lifecycle == "ACTIVE" else "IDLE", "reason": None},
        }

    def read(self, resource):
        if resource == "v3.status":
            return self.status
        if resource == "operator.status":
            return {"runtime_pid": 123}
        raise KeyError(resource)

    def capabilities(self):
        return {"capabilities": {}}

    def execute(self, action, **parameters):
        self.operations.append("EXECUTE:" + action)
        self.actions.append((action, parameters))
        command = "command-" + str(len(self.actions))
        mode = {"v3.command.explore": "EXPLORE", "v3.command.follow_person": "FOLLOW_PERSON"}.get(action, "NAVIGATE")
        self.status = self.snapshot(mission_id="mission-" + command, mode=mode, lifecycle="ACTIVE")
        return {"command_id": command}

    def stop(self):
        self.operations.append("STOP")
        self.status = self.snapshot(mission_id="stopped", lifecycle="CANCELLED")
        self.stopped.set()
        return {"status": "STOPPED"}


class SharedClient:
    """Two facades address one owner, with no process/hardware launch."""

    def __init__(self, runtime):
        self.runtime = runtime
        self.down = False

    def request(self, operation, **arguments):
        if self.down:
            raise RuntimeError("PUBLIC_WORLD_UNAVAILABLE")
        if operation == "read":
            return self.runtime.read(arguments["resource"])
        return self.runtime.execute(arguments["action"], arguments.get("parameters", {}))

    def preempt(self, reason):
        if self.down:
            raise RuntimeError("PUBLIC_WORLD_UNAVAILABLE")
        return self.runtime.preempt(reason)

    def revoke(self, reason):
        if self.down:
            raise RuntimeError("PUBLIC_WORLD_UNAVAILABLE")
        return self.runtime.revoke(reason)


class LocalAdapter:
    name = "local"
    capability_names = frozenset({"v3.command.stop", "v3.command.move_relative",
                                  "v3.command.explore", "v3.command.follow_person"})

    def __init__(self, backend):
        self.backend = backend

    def capabilities(self):
        return {name: {"kind": "action", "available": True, "supported": True}
                for name in self.capability_names}

    def execute(self, action, **parameters):
        if action == "v3.command.stop":
            return self.backend.stop()
        return self.backend.execute(action, **parameters)


def setup_runtime():
    clock = Clock()
    backend = Backend(clock)
    world = PublicWorldModel(clock_ns=clock, clock_epoch="test-boot")
    runtime = PublicRobotRuntime(backend, world=world, clock_ns=clock)
    return clock, backend, runtime


def wait_behavior(runtime, lifecycle):
    deadline = time.monotonic() + 2
    while runtime.behaviors.snapshot().lifecycle.value != lifecycle and time.monotonic() < deadline:
        time.sleep(.001)
    assert runtime.behaviors.snapshot().lifecycle.value == lifecycle


def test_independent_public_facades_share_behavior_mission_and_world():
    clock, backend, runtime = setup_runtime()
    client = SharedClient(runtime)
    adapters = (LocalAdapter(backend), PublicRobotInterfaceAdapter(client))
    human = RobotInterface(controller=backend, adapters=adapters)
    agent = RobotInterface(controller=backend, adapters=adapters)
    starting = human.execute("v3.command.explore", max_v_mps=0.2)
    assert agent.read("behavior.state")["behavior_id"] == starting["behavior_id"]
    assert backend.actions[0][0] == "v3.command.explore"
    runtime.poll()
    wait_behavior(runtime, "ACTIVE")
    state = agent.read("robot.state")
    assert state["active_behavior"]["lifecycle"] == "ACTIVE"
    assert state["active_mission"]["value"]["mission_id"] == starting["mission_id"]
    assert state["active_mission"]["measurement_time_ns"] == clock.now - 10
    assert human.read("world.snapshot") == agent.read("world.snapshot")
    agent.stop()
    assert human.read("behavior.state")["lifecycle"] == "CANCELLED"
    assert human.read("behavior.state")["reason"] == "STOP"


def test_person_knowledge_retains_physical_measurement_time_across_prediction():
    clock, backend, runtime = setup_runtime()
    measured = clock.now - 100_000_000
    observed = {"track_id": "person-7", "x_m": 1.0, "y_m": 2.0, "confidence": 0.9,
                "measurement_monotonic_ns": measured, "estimate_status": "OBSERVED"}
    status = backend.snapshot(tracks=(observed,))
    runtime.ingest_status(status, runtime_pid=123)
    fact = runtime.world.read("person:123:person-7", "location").to_jsonable()
    assert fact["measurement_time_ns"] == measured
    assert fact["observation_time_ns"] == clock.now
    assert fact["sequence"] == status["tick_id"]  # The producer is completed V3 status.
    assert fact["lineage"] == ["runtime:123", "tick:1"]
    assert fact["value"]["frame_id"] == "R2B4_ODOM_LOCAL"
    clock.now += 1_000_000_000
    predicted = {**observed, "x_m": 4.0, "estimate_status": "PREDICTED"}
    status = backend.snapshot(tracks=(predicted,))
    status["tick_id"] = 2
    runtime.ingest_status(status, runtime_pid=123)
    later = runtime.world.read("person:123:person-7", "location").to_jsonable()
    assert later["measurement_time_ns"] == measured
    assert later["observation_time_ns"] == fact["observation_time_ns"]
    assert later["value"]["x_m"] == 1.0
    assert later["age_ns"] == clock.now - measured
    # A track from another runtime is a separate identity, not a current person.
    status["world"]["person_tracks"] = [observed]
    runtime.ingest_status(status, runtime_pid=456)
    other = runtime.world.read("person:456:person-7", "location").to_jsonable()
    assert other["value"]["runtime_pid"] == 456
    assert other["measurement_time_ns"] == measured


def test_unavailable_public_world_does_not_block_local_action_or_stop():
    _, backend, runtime = setup_runtime()
    client = SharedClient(runtime)
    client.down = True
    interface = RobotInterface(controller=backend, adapters=(
        LocalAdapter(backend), PublicRobotInterfaceAdapter(client),
    ))
    with pytest.raises(RuntimeError, match="PUBLIC_WORLD_UNAVAILABLE"):
        interface.read("world.snapshot")
    result = interface.execute("v3.command.move_relative", forward_m=0.3)
    assert result["command_id"]
    assert backend.actions == [("v3.command.move_relative", {"forward_m": 0.3})]
    assert interface.stop() == {"status": "STOPPED"}
    assert backend.operations[-1] == "STOP"


def test_room_cruise_to_follow_person_explicitly_preempts_one_mission():
    _, backend, runtime = setup_runtime()
    first = runtime.execute("behavior.room_cruise", {})
    runtime.poll()
    following = runtime.execute("behavior.follow_person", {})
    assert following["behavior_id"] != first["behavior_id"]
    assert following["name"] == "follow_person"
    assert following["mission_id"] != first["mission_id"]
    history = runtime.read("behavior.history")
    ended = [event for event in history if event["behavior_id"] == first["behavior_id"]
             and event["lifecycle"] == "CANCELLED"]
    assert len(ended) == 1
    assert "PREEMPTED" in ended[0]["reason"]
    assert "STOP" in backend.operations[1:-1]
    assert backend.actions[-1][0] == "v3.command.follow_person"


def test_stop_during_start_repeats_after_late_public_action_and_never_reactivates():
    _, backend, runtime = setup_runtime()
    entered, release = threading.Event(), threading.Event()
    original = backend.execute
    results, errors = {}, []

    def blocked_execute(action, **parameters):
        entered.set()
        assert release.wait(2)
        return original(action, **parameters)

    backend.execute = blocked_execute

    def start():
        try:
            results["start"] = runtime.execute("behavior.room_cruise", {})
        except Exception as exc:
            errors.append(exc)

    def stop():
        try:
            results["stop"] = runtime.preempt("STOP")
        except Exception as exc:
            errors.append(exc)

    starter = threading.Thread(target=start)
    starter.start()
    assert entered.wait(2)
    assert runtime.read("behavior.state")["lifecycle"] == "STARTING"
    stopper = threading.Thread(target=stop)
    stopper.start()
    assert backend.stopped.wait(2), "STOP must reach the backend while start is blocked"
    assert runtime.read("behavior.state")["lifecycle"] == "CANCELLED"
    release.set()
    starter.join(2)
    stopper.join(2)
    assert not starter.is_alive() and not stopper.is_alive()
    assert not errors
    assert backend.operations[-1] == "STOP"
    assert backend.operations.index("STOP") < backend.operations.index("EXECUTE:v3.command.explore")
    assert runtime.read("behavior.state")["lifecycle"] == "CANCELLED"
    assert results["start"]["lifecycle"] == "CANCELLED"


def test_start_queued_before_stop_cannot_restart_after_stop():
    _, backend, runtime = setup_runtime()
    arrived = threading.Event()
    held_lock = threading.Lock()
    held_lock.acquire()

    class AdmissionLock:
        def __enter__(self):
            if threading.current_thread().name == "queued-start":
                arrived.set()  # Generation was captured before entering this lock.
            held_lock.acquire()
            return self

        def __exit__(self, *args):
            held_lock.release()

    runtime._action_lock = AdmissionLock()
    errors = []

    def start():
        try:
            runtime.execute("behavior.room_cruise", {})
        except RuntimeError as exc:
            errors.append(str(exc))

    starter = threading.Thread(target=start, name="queued-start")
    starter.start()
    assert arrived.wait(2)
    stopper = threading.Thread(target=runtime.preempt, args=("STOP",))
    stopper.start()
    assert backend.stopped.wait(2)
    held_lock.release()
    starter.join(2)
    stopper.join(2)
    assert not starter.is_alive() and not stopper.is_alive()
    assert errors and "revoked" in errors[0]
    assert backend.actions == []
    assert not runtime.behaviors.active


def test_public_poll_failure_revokes_active_behavior_and_stops_without_retry(monkeypatch):
    _, backend, runtime = setup_runtime()
    runtime.execute("behavior.room_cruise", {})
    runtime.poll()

    def failed_observation(*args, **kwargs):
        raise RuntimeError("world storage unavailable")

    monkeypatch.setattr(runtime.world, "observe", failed_observation)
    backend.status["tick_id"] += 1  # A new completed status reaches the projector.
    runtime.poll_safely()
    state = runtime.read("behavior.state")
    assert state["lifecycle"] == "CANCELLED"
    assert state["reason"] == "HOST_STEP_FAILED"
    assert backend.operations[-1] == "STOP"
    assert len(backend.actions) == 1
    assert "PUBLIC_POLL_FAILED" in runtime._health_error


def test_stop_does_not_wait_for_passive_evidence_output(monkeypatch):
    _, backend, runtime = setup_runtime()

    def forbidden_write(*args, **kwargs):
        raise AssertionError("STOP waited for evidence output")

    monkeypatch.setattr(runtime, "_evidence", forbidden_write)
    runtime.execute("behavior.room_cruise", {})
    state = runtime.preempt("STOP")
    assert state["lifecycle"] == "CANCELLED"
    assert backend.operations[-1] == "STOP"
    assert runtime.read("behavior.history")[-1]["kind"] == "BEHAVIOR_CANCELLED"


def test_oversized_public_request_is_rejected_before_socket_or_process_start(monkeypatch, tmp_path):
    from r2b4_orchestration import robot_runtime

    def forbidden_external_operation(*args, **kwargs):
        raise AssertionError("oversized input reached transport")

    monkeypatch.setattr(robot_runtime.socket, "socket", forbidden_external_operation)
    monkeypatch.setattr(robot_runtime.subprocess, "Popen", forbidden_external_operation)
    client = PublicRobotClient(tmp_path)
    with pytest.raises(ValueError, match="exceeded its bound"):
        client.request("execute", action="world.observe", parameters={"raw": "x" * MAX_REQUEST_BYTES})


def test_storage_stall_cannot_block_active_behavior_revocation(monkeypatch):
    clock, backend, runtime = setup_runtime()
    runtime.execute("behavior.room_cruise", {})
    storing, release, stopped = threading.Event(), threading.Event(), threading.Event()
    errors, results = [], []

    def storage_stall(**_kwargs):
        storing.set()
        assert release.wait(2.0)

    def observe():
        try:
            runtime.execute("world.observe", {
                "entity_id": "cup", "attribute": "location", "value": "table",
                "domain": "object_position", "measurement_time_ns": clock.now,
                "confidence": 0.9, "source": "semantic_vision", "lineage": ["frame:9"],
            })
        except Exception as exc:
            errors.append(exc)

    def stop():
        try:
            results.append(runtime.preempt("STOP"))
        except Exception as exc:
            errors.append(exc)
        finally:
            stopped.set()

    monkeypatch.setattr(runtime, "_persist", storage_stall)
    writer, stopper = threading.Thread(target=observe), threading.Thread(target=stop)
    writer.start()
    try:
        assert storing.wait(1.0)
        stopper.start()
        assert stopped.wait(0.5), "STOP waited for public world storage"
    finally:
        release.set()
        writer.join(2.0)
        if stopper.ident is not None:
            stopper.join(2.0)
    assert errors == []
    assert results[0]["lifecycle"] == "CANCELLED"
    assert backend.operations[-1] == "STOP"


def test_search_person_uses_runtime_injected_public_world_through_real_facade():
    clock = Clock()
    backend = Backend(clock)
    world = PublicWorldModel(clock_ns=clock, clock_epoch="test-boot")
    world.observe("room:lounge", "location",
                  {"frame_id": "R2B4_BOOT_ROBOT_MAP", "runtime_pid": 123, "x_m": 1.0, "y_m": 0.0},
                  domain="room_topology", measurement_time_ns=1, confidence=1.0,
                  source="room_map", lineage=("map:7",))

    class PublicBackendAdapter(LocalAdapter):
        capability_names = LocalAdapter.capability_names | frozenset({
            "operator.status", "v3.status", "v3.command.navigate", "vision.observe",
        })

        def capabilities(self):
            return {name: {"kind": "read" if name in {"operator.status", "v3.status"} else "action",
                           "available": True, "supported": True} for name in self.capability_names}

        def read(self, resource):
            if resource == "operator.status":
                return {"runtime_running": True, "runtime_pid": 123, "capture_mode": "full", "capture_hz": 10}
            return backend.read(resource)

        def execute(self, action, **parameters):
            if action == "vision.observe":
                backend.operations.append("EXECUTE:vision.observe")
                world.observe("person:laci", "location", {"place_id": "room:lounge"},
                              domain="person_position", measurement_time_ns=clock.now,
                              confidence=0.95, source="semantic_vision", lineage=("camera:18", "identity:4"))
                return object()
            return super().execute(action, **parameters)

    facade = RobotInterface(controller=backend, adapters=(PublicBackendAdapter(backend),), upper_runtime=False)
    runtime = PublicRobotRuntime(facade, world=world, clock_ns=clock)
    # Real resident status includes both pose frames and localization generation.
    # A semantic navigation request needs a completed current spatial context.
    backend.status["estimate"]["localization_quality"]["generation"] = 0
    backend.status["estimate"]["global_pose"] = {
        "frame_id": "R2B4_BOOT_ROBOT_MAP", "x_m": 1.0, "y_m": 2.0, "yaw_rad": 0.2}
    runtime.ingest_status(backend.status, runtime_pid=123)
    starting = runtime.execute("behavior.start", {"name": "search_person", "entity_id": "person:laci"})
    assert starting["lifecycle"] == "STARTING"
    assert backend.actions[0][0] == "v3.command.navigate"
    assert backend.actions[0][1]["expected_runtime_pid"] == 123
    backend.status["navigation"]["status"] = "COMPLETE"
    backend.status.update(safety_decision="STOP", safety_reason="NOT_ACTIVE")
    runtime.poll()
    wait_behavior(runtime, "COMPLETED")
    completed = runtime.read("behavior.state")
    assert completed["lifecycle"] == "COMPLETED"
    assert "TARGET_OBSERVED:person:laci" in completed["reason"]
    assert "EXECUTE:vision.observe" in backend.operations
    assert backend.operations[-1] == "STOP"
    intent = next(event for event in runtime.read("behavior.history") if event["kind"] == "BEHAVIOR_INTENT")
    assert intent["world_revision"] is not None
    assert intent["clock_epoch"] == "test-boot"
    assert runtime.world.read("person:laci", "location").to_jsonable()["measurement_time_ns"] == clock.now
