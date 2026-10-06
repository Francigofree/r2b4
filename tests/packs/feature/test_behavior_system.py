"""Host lifecycle evidence through a simulated public RobotInterface only."""

from __future__ import annotations

import threading

import pytest

from r2b4_orchestration.behavior_system import (
    BehaviorLifecycle,
    BehaviorSystem,
    BehaviorUpdate,
)


class Clock:
    now = 10_000_000_000

    def __call__(self):
        return self.now


class Robot:
    def __init__(self, clock):
        self.clock = clock
        self.actions = []
        self.stops = 0
        self.status = {}
        self.read_error = None

    def capabilities(self):
        return {"capabilities": {}}

    def execute(self, action, **parameters):
        self.actions.append((action, parameters))
        return {"command_id": f"command-{len(self.actions)}"}

    def read(self, resource):
        assert resource == "v3.status"
        if self.read_error:
            raise self.read_error
        return self.status

    def stop(self):
        self.stops += 1

    def observe(self, state, *, mode="EXPLORE", lifecycle="ACTIVE", navigation="ACTIVE", reason=None):
        self.status = {
            "monotonic_ns": self.clock.now - 1,
            "state": "RUNNING", "fault_layer": None,
            "safety_decision": "ALLOW", "safety_reason": "ALLOW",
            "mission": {"mission_id": state.mission_id, "mode": mode, "lifecycle": lifecycle},
            "navigation": {"mission_id": state.mission_id, "status": navigation, "reason": reason},
        }


def setup_system(**kwargs):
    clock = Clock()
    robot = Robot(clock)
    return clock, robot, BehaviorSystem(robot, clock_ns=clock, **kwargs)


def test_room_cruise_uses_public_action_and_correlated_completed_status():
    clock, robot, system = setup_system()
    starting = system.start("room_cruise", {"max_v_mps": 0.2})
    assert robot.actions == [("v3.command.explore", {"max_v_mps": 0.2})]
    assert starting.lifecycle is BehaviorLifecycle.STARTING
    # A completion from a previous mission never completes the new behavior.
    robot.observe(starting, navigation="COMPLETE")
    robot.status["mission"]["mission_id"] = "previous-mission"
    assert system.step().lifecycle is BehaviorLifecycle.STARTING
    robot.observe(starting)
    active = system.step()
    assert active.lifecycle is BehaviorLifecycle.ACTIVE
    assert active.measurement_time_ns == clock.now - 1
    assert active.observation_time_ns == clock.now
    assert active.to_jsonable()["lineage"]["mission_id"] == starting.mission_id
    events = len(system.history())
    clock.now += 1_000_000
    robot.observe(active)
    assert system.step().measurement_time_ns == clock.now - 1
    assert len(system.history()) == events  # No repeated unchanged lifecycle evidence.
    robot.observe(active, navigation="COMPLETE")
    completed = system.step()
    assert completed.lifecycle is BehaviorLifecycle.COMPLETED
    assert completed.measurement_time_ns == robot.status["monotonic_ns"]
    assert robot.stops == 1
    assert [event.kind for event in system.history()] == [
        "BEHAVIOR_STARTING", "BEHAVIOR_INTENT", "BEHAVIOR_ACTION_RESULT", "BEHAVIOR_ACTION_ACCEPTED",
        "BEHAVIOR_OBSERVED", "BEHAVIOR_COMPLETED",
    ]


def test_foreign_mission_preempts_without_stopping_the_new_owner():
    _, robot, system = setup_system()
    starting = system.start("room_cruise")
    with pytest.raises(RuntimeError, match="preempted"):
        system.start("follow_person")
    robot.observe(starting)
    system.step()
    robot.status["mission"]["mission_id"] = "new-owner-mission"
    robot.status["fault_layer"] = "NEW_OWNER_FAULT"
    cancelled = system.step()
    assert cancelled.lifecycle is BehaviorLifecycle.CANCELLED
    assert cancelled.reason == "MISSION_PREEMPTED"
    assert robot.stops == 0
    next_state = system.start("follow_person")
    assert next_state.behavior_id != cancelled.behavior_id
    assert robot.actions[-1][0] == "v3.command.follow_person"


@pytest.mark.parametrize("failure, reason", [
    ("stale", "STATUS_STALE"),
    ("future", "STATUS_STALE"),
    ("read", "STATUS_ERROR:RuntimeError"),
    ("fault", "RUNTIME_FAULT"),
    ("safety", "LIDAR_SAFETY_STALE"),
])
def test_status_failures_stop_once_and_do_not_retry(failure, reason):
    clock, robot, system = setup_system()
    state = system.start("room_cruise")
    robot.observe(state)
    if failure == "stale":
        robot.status["monotonic_ns"] = clock.now - 500_000_000
    elif failure == "future":
        robot.status["monotonic_ns"] = clock.now + 1
    elif failure == "read":
        robot.read_error = RuntimeError("lost host observation")
    elif failure == "fault":
        robot.status["fault_layer"] = "L12"
    else:
        robot.status.update(safety_decision="STOP", safety_reason=reason)
    failed = system.step()
    assert failed.lifecycle is BehaviorLifecycle.FAILED
    assert failed.reason == reason
    assert robot.stops == 1
    assert system.step() is failed
    assert robot.stops == 1
    assert len(robot.actions) == 1


def test_follow_person_observes_hold_and_lost_using_the_same_lifecycle():
    _, robot, system = setup_system()
    state = system.start("follow_person")
    robot.observe(state, mode="FOLLOW_PERSON", navigation="INVALIDATED", reason="PERSON_TARGET_NOT_AVAILABLE")
    robot.status.update(safety_decision="STOP", safety_reason="NOT_ACTIVE")
    assert system.step().lifecycle is BehaviorLifecycle.ACTIVE
    robot.observe(state, mode="FOLLOW_PERSON", navigation="IDLE", reason="PERSON_OCCLUDED_HOLD")
    assert system.step().reason == "PERSON_OCCLUDED_HOLD"
    robot.observe(state, mode="FOLLOW_PERSON", navigation="INVALIDATED", reason="PERSON_TARGET_LOST")
    assert system.step().lifecycle is BehaviorLifecycle.FAILED
    assert system.snapshot().reason == "PERSON_TARGET_LOST"
    assert robot.stops == 1


def test_duration_and_command_acknowledgement_are_bounded():
    clock, robot, system = setup_system()
    state = system.start("room_cruise", {"session_watchdog_s": 2.0}, max_duration_s=100)
    assert state.deadline_ns - state.started_ns == 2_000_000_000
    clock.now = state.deadline_ns
    assert system.step().reason == "DURATION_LIMIT"
    assert robot.stops == 1
    _, robot, system = setup_system()
    state = system.start("room_cruise")
    robot.observe(state)
    robot.status["mission"]["mission_id"] = "unacknowledged-command"
    system._clock_ns.now += 5_000_000_000
    robot.status["monotonic_ns"] = system._clock_ns.now
    assert system.step().reason == "MISSION_NOT_ACKNOWLEDGED"
    assert robot.stops == 1


def test_stop_revokes_starting_program_before_late_action_returns():
    _, robot, system = setup_system()
    entered, release = threading.Event(), threading.Event()
    original = robot.execute

    def waiting_action(action, **parameters):
        entered.set()
        assert release.wait(2)
        return original(action, **parameters)

    robot.execute = waiting_action
    thread = threading.Thread(target=system.start, args=("room_cruise",))
    thread.start()
    assert entered.wait(2)
    assert system.snapshot().lifecycle is BehaviorLifecycle.STARTING
    system.revoke("STOP")
    # The resident owner performs canonical STOP serialized with the action
    # boundary. The late callback cannot make the cancelled lifecycle active.
    release.set()
    thread.join(2)
    robot.stop()
    assert not thread.is_alive()
    assert system.snapshot().lifecycle is BehaviorLifecycle.CANCELLED
    assert system.snapshot().reason == "STOP"
    assert system.snapshot().command_id is None
    assert robot.stops == 1
    assert len(system.history()) == 4
    result = system.history()[-1]
    assert result.kind == "BEHAVIOR_ACTION_RESULT"
    assert result.state.behavior_id == system.snapshot().behavior_id
    assert result.action_command_id == "command-1"


def test_new_program_uses_public_operations_and_cannot_use_wheel_action():
    _, robot, system = setup_system()

    class Visit:
        def start(self, port, parameters):
            self.port = port
            return port.execute("v3.command.navigate", **parameters)

        def step(self, port, state, status):
            return BehaviorUpdate(BehaviorLifecycle.COMPLETED, "VISITED")

    program = Visit()
    system.register("visit", lambda: program)
    state = system.start("visit", {"x_m": 1.0, "y_m": 2.0})
    assert robot.actions == [("v3.command.navigate", {"x_m": 1.0, "y_m": 2.0})]
    with pytest.raises(ValueError, match="public robot"):
        program.port.execute("v3.command.wheels", left_mps=0.2, right_mps=0.2)
    robot.observe(state, mode="NAVIGATE")
    assert system.step().reason == "VISITED"
    with pytest.raises(RuntimeError, match="revoked"):
        program.port.execute("v3.command.navigate", x_m=2.0, y_m=3.0)
    assert len(robot.actions) == 1


def test_evidence_sink_failure_cannot_prevent_canonical_stop():
    def failing_sink(event):
        raise OSError("evidence output unavailable")

    _, robot, system = setup_system(event_sink=failing_sink, history_limit=2)
    system.start("room_cruise")
    cancelled = system.cancel()
    assert cancelled.lifecycle is BehaviorLifecycle.CANCELLED
    assert robot.stops == 1
    assert len(system.history()) == 2
    event = system.history()[-1].to_jsonable()
    assert event["schema"] == "R2B4_BEHAVIOR_EVENT_V1"
    assert event["behavior_id"] == cancelled.behavior_id
    assert event["mission_id"] == cancelled.mission_id


def test_caller_parameter_mutation_cannot_change_behavior_state_or_evidence():
    _, robot, system = setup_system()
    received = []

    class Program:
        def start(self, port, parameters):
            received.append(parameters["candidate_places"])
            return port.execute("v3.command.explore")

        def step(self, port, state, status):
            return None

    system.register("places", Program)
    places = ["room:lounge", "room:kitchen"]
    parameters = {"candidate_places": places}
    state = system.start("places", parameters)
    original = state.to_jsonable()
    event = system.history()[0].to_jsonable()
    places.append("room:hall")
    parameters["candidate_places"] = ["room:garage"]
    assert system.snapshot().to_jsonable() == original
    assert system.history()[0].to_jsonable() == event
    assert original["parameters"]["candidate_places"] == ["room:lounge", "room:kitchen"]
    assert received == [("room:lounge", "room:kitchen")]
    assert len(robot.actions) == 1


def test_brain_lineage_and_every_program_command_are_observable_and_immutable():
    clock, robot, system = setup_system()

    class Visit:
        def start(self, port, parameters):
            return port.execute("v3.command.navigate", x_m=1.0, y_m=2.0)

        def step(self, port, state, status):
            result = port.execute("v3.command.navigate", x_m=3.0, y_m=4.0)
            return BehaviorUpdate(BehaviorLifecycle.ACTIVE, "NEXT_PLACE", result["command_id"])

    system.register("visit", Visit)
    lineage = {"goal_id": "goal-1", "subtask_id": "subtask-2", "decision_id": "decision-3"}
    state = system.start("visit", lineage=lineage)
    lineage["goal_id"] = "mutated"
    robot.observe(state, mode="NAVIGATE")
    system.step()
    results = [event.to_jsonable() for event in system.history()
               if event.kind == "BEHAVIOR_ACTION_RESULT"]
    assert [event["action_command_id"] for event in results] == ["command-1", "command-2"]
    assert [event["action_mission_id"] for event in results] == ["mission-command-1", "mission-command-2"]
    assert all(event["lineage"]["goal_id"] == "goal-1" for event in results)
    assert all(event["subtask_id"] == "subtask-2" and event["decision_id"] == "decision-3"
               for event in results)
    assert system.snapshot().command_id == "command-2"


@pytest.mark.parametrize("name, mode", [("room_cruise", "EXPLORE"), ("follow_person", "FOLLOW_PERSON")])
def test_explicit_requested_duration_completes_only_with_current_execution(name, mode):
    clock, robot, system = setup_system()
    state = system.start(name, max_duration_s=50, completion_on_duration=True)
    robot.observe(state, mode=mode)
    system.step()
    clock.now = state.deadline_ns
    robot.observe(state, mode=mode)
    completed = system.step()
    assert completed.lifecycle is BehaviorLifecycle.COMPLETED
    assert completed.reason == "REQUESTED_DURATION_REACHED"
    assert completed.measurement_time_ns == robot.status["monotonic_ns"]
    assert robot.stops == 1


def test_watchdog_and_missing_execution_never_prove_requested_duration():
    clock, robot, system = setup_system()
    state = system.start("room_cruise", {"session_watchdog_s": 2},
                         max_duration_s=50, completion_on_duration=True)
    clock.now = state.deadline_ns
    assert system.step().lifecycle is BehaviorLifecycle.FAILED
    assert system.snapshot().reason == "DURATION_LIMIT"

    clock, robot, system = setup_system()
    state = system.start("room_cruise", max_duration_s=50, completion_on_duration=True)
    robot.observe(state)
    system.step()
    clock.now = state.deadline_ns
    assert system.step().reason == "STATUS_STALE"
    assert system.snapshot().lifecycle is BehaviorLifecycle.FAILED
    assert robot.stops == 1


def test_program_terminal_result_is_preserved_without_parallel_mutable_world():
    _, robot, system = setup_system()
    result = {"target_track_id": "person-7", "target_entity_id": "person:123:person-7",
              "runtime_pid": 123, "measurement_time_ns": 12345}

    class Search:
        def start(self, port, parameters):
            return BehaviorUpdate(BehaviorLifecycle.COMPLETED, "PERSON_FOUND", result=result)

        def step(self, port, state, status):
            raise AssertionError("completed search must not step")

    system.register("search_any", Search)
    completed = system.start("search_any")
    result["target_track_id"] = "person-8"
    assert completed.to_jsonable()["result"]["target_track_id"] == "person-7"
    assert system.history()[-1].to_jsonable()["result"]["runtime_pid"] == 123
    assert robot.actions == []


def test_program_action_error_remains_passive_correlated_evidence():
    _, robot, system = setup_system(event_sink=lambda event: (_ for _ in ()).throw(OSError("hub gone")))

    def failed_action(action, **parameters):
        raise RuntimeError("canonical operation failed")

    robot.execute = failed_action
    state = system.start("room_cruise", lineage={"goal_id": "goal-1"})
    assert state.lifecycle is BehaviorLifecycle.FAILED
    errors = [event.to_jsonable() for event in system.history() if event.kind == "BEHAVIOR_ACTION_FAILED"]
    assert errors[0]["action"] == "v3.command.explore"
    assert errors[0]["action_error"] == "RuntimeError"
    assert errors[0]["goal_id"] == "goal-1"
    assert robot.stops == 1


def test_requested_duration_never_reports_success_when_canonical_stop_fails():
    clock, robot, system = setup_system()
    state = system.start("room_cruise", max_duration_s=50, completion_on_duration=True)
    robot.observe(state)
    state = system.step()
    clock.now = state.deadline_ns
    robot.observe(state)

    def failed_stop():
        raise RuntimeError("canonical STOP failed")

    robot.stop = failed_stop
    with pytest.raises(RuntimeError, match="canonical STOP failed"):
        system.step()
    assert system.snapshot().lifecycle is BehaviorLifecycle.FAILED
    assert system.snapshot().reason == "CANONICAL_STOP_FAILED:RuntimeError"
    assert all(event.kind != "BEHAVIOR_COMPLETED" for event in system.history())
    assert system.history()[-1].kind == "BEHAVIOR_FAILED"


def test_follow_duration_starts_with_target_execution_and_cannot_finish_without_acquisition():
    clock, robot, system = setup_system()
    state = system.start("follow_person", max_duration_s=50, completion_on_duration=True)
    robot.observe(state, mode="FOLLOW_PERSON", navigation="INVALIDATED", reason="PERSON_TARGET_NOT_AVAILABLE")
    assert system.step().execution_started_ns is None
    clock.now = state.deadline_ns
    robot.observe(state, mode="FOLLOW_PERSON", navigation="INVALIDATED", reason="PERSON_TARGET_NOT_AVAILABLE")
    assert system.step().reason == "REQUESTED_EXECUTION_UNPROVEN"
    assert system.snapshot().lifecycle is BehaviorLifecycle.FAILED

    clock, robot, system = setup_system()
    state = system.start("follow_person", max_duration_s=50, completion_on_duration=True)
    clock.now += 2_000_000_000  # Acquisition time is not requested follow time.
    robot.observe(state, mode="FOLLOW_PERSON", navigation="IDLE", reason="PERSON_DISTANCE_HOLD")
    active = system.step()
    assert active.execution_started_ns == robot.status["monotonic_ns"]
    assert active.deadline_ns == active.execution_started_ns + 50_000_000_000
    clock.now = state.deadline_ns
    robot.observe(active, mode="FOLLOW_PERSON", navigation="IDLE", reason="PERSON_DISTANCE_HOLD")
    assert system.step().lifecycle is BehaviorLifecycle.ACTIVE
