"""User-goal truth, preemption and stale results through the host runtime."""
import threading
import time

import pytest

from r2b4_orchestration.robot_runtime import PublicRobotRuntime
from r2b4_orchestration.world_model import PublicWorldModel
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
        self.pid = 123
        self.status = self.make_status()

    def capabilities(self):
        return {"capabilities": {name: {"available": True, "supported": True, "kind": "action"}
                                  for name in (*ACTION_CATALOG, "vision.observe")}}

    def make_status(self, command="idle", mode="STOP", lifecycle="IDLE"):
        return {"monotonic_ns": self.clock.now, "state": "RUNNING", "fault_layer": None,
                "safety_decision": "ALLOW", "safety_reason": "ALLOW",
                "mission": {"mission_id": "mission-" + command, "mode": mode, "lifecycle": lifecycle},
                "navigation": {"mission_id": "mission-" + command,
                               "status": "ACTIVE", "reason": None}}

    def read(self, resource):
        if resource == "operator.status":
            return {"runtime_pid": self.pid, "runtime_running": True,
                    "capture_mode": "nincs", "capture_hz": 10}
        if resource == "v3.status":
            return self.status
        raise KeyError(resource)

    def execute(self, action, **parameters):
        self.actions.append((action, parameters))
        command = "cmd-" + str(len(self.actions))
        mode = {"v3.command.explore": "EXPLORE", "v3.command.follow_person": "FOLLOW_PERSON"}.get(action, "NAVIGATE")
        self.status = self.make_status(command, mode, "ACTIVE")
        if action in {"v3.command.move_relative", "v3.command.turn_by"}:
            return {"command_id": command, "mission_id": "mission-" + command,
                    "status": "COMPLETED", "reason": "COMPLETE"}
        return {"command_id": command, "mission_id": "mission-" + command}

    def stop(self):
        self.stops += 1


def runtime():
    clock = Clock()
    robot = Robot(clock)
    owner = PublicRobotRuntime(robot, clock_ns=clock, world=PublicWorldModel(clock_ns=clock))
    return clock, robot, owner


def adopt(owner, text, steps, source="HUMAN", **plan_fields):
    pending = owner.execute("brain.submit", {"text": text, "source": source})
    result = owner.execute("brain.adopt", {"goal_id": pending["goal_id"],
                         "plan": {"steps": steps, **plan_fields}})
    deadline = time.monotonic() + 2
    while result["lifecycle"] == "STARTING" and time.monotonic() < deadline:
        time.sleep(0.001)
        result = owner.brain.goal(result["goal_id"])
    return result


def test_requested_50_seconds_is_owned_until_correlated_duration_completion():
    clock, robot, owner = runtime()
    goal = adopt(owner, "Menj körbe 50 másodpercig.", [
        {"action": "behavior.room_cruise", "parameters": {"max_duration_s": 50}}])
    assert goal["lifecycle"] == "ACTIVE"
    assert robot.actions[-1][1]["session_watchdog_s"] > 50
    assert owner.behaviors.snapshot().goal_id == goal["goal_id"]
    assert owner.behaviors.snapshot().decision_id
    owner.poll()  # First correlated physical execution acknowledgement.
    clock.now += 49_000_000_000
    robot.status["monotonic_ns"] = clock.now
    owner.poll()
    assert owner.brain.goal(goal["goal_id"])["lifecycle"] == "ACTIVE"
    clock.now += 1_000_000_000
    robot.status["monotonic_ns"] = clock.now
    owner.poll()
    assert owner.brain.goal(goal["goal_id"])["lifecycle"] == "COMPLETED"
    assert owner.brain.goal(goal["goal_id"])["reason"] == "REQUESTED_DURATION_REACHED"


def test_navigation_acceptance_does_not_complete_observation_goal():
    _, robot, owner = runtime()
    goal = adopt(owner, "Menj oda és nézd meg a lámpát.", [
        {"action": "v3.command.navigate", "parameters": {"x_m": 1, "y_m": 2}},
        {"action": "vision.observe", "parameters": {}}])
    assert goal["lifecycle"] == "ACTIVE" and goal["step_index"] == 0
    robot.status["mission"]["lifecycle"] = "COMPLETED"
    owner.poll()
    final = owner.brain.goal(goal["goal_id"])
    assert final["step_index"] == 1
    assert final["lifecycle"] == "FAILED"  # Fake camera supplied no calibrated evidence.
    assert "OBSERVATION_EVIDENCE_UNAVAILABLE" in final["reason"]


def test_mission_that_ends_early_does_not_prove_requested_duration():
    _, robot, owner = runtime()
    goal = adopt(owner, "Menj körbe 50 másodpercig.", [
        {"action": "behavior.room_cruise", "parameters": {"max_duration_s": 50}}])
    robot.status["mission"]["lifecycle"] = "COMPLETED"
    owner.poll()
    assert owner.brain.goal(goal["goal_id"])["lifecycle"] == "FAILED"
    assert owner.brain.goal(goal["goal_id"])["reason"] == "REQUESTED_DURATION_UNPROVEN"


@pytest.mark.parametrize("plan", [
    {"steps": [{"action": "behavior.follow_person", "parameters": {}}]},
    {"steps": [{"action": "behavior.follow_person", "parameters": {"follow_distance_m": 1.5}}]},
])
def test_explicit_unsupported_follow_distance_never_starts(plan):
    _, robot, owner = runtime()
    goal = owner.brain.submit("Kövesd 1,5 méterről.")
    result = owner.brain.adopt(goal["goal_id"], plan)
    assert result["lifecycle"] == "FAILED"
    assert "CONSTRAINT_UNSUPPORTED" in result["reason"]
    assert robot.actions == []


def test_duration_constraint_cannot_be_discarded_by_proposal():
    _, robot, owner = runtime()
    goal = adopt(owner, "Menj körbe 50 másodpercig.", [
        {"action": "behavior.room_cruise", "parameters": {"max_duration_s": 30}}])
    assert goal["lifecycle"] == "FAILED"
    assert robot.actions == []


def test_human_preempts_autonomous_goal_and_lower_priority_cannot_replace_human():
    _, robot, owner = runtime()
    auto = adopt(owner, "Explore", [{"action": "behavior.room_cruise"}], source="AUTONOMOUS")
    human = adopt(owner, "Follow", [{"action": "behavior.follow_person"}])
    assert owner.brain.goal(auto["goal_id"])["lifecycle"] == "CANCELLED"
    assert human["lifecycle"] == "ACTIVE"
    before = len(robot.actions)
    rejected = adopt(owner, "Explore", [{"action": "behavior.room_cruise"}], source="AUTONOMOUS")
    assert rejected["reason"] == "LOWER_PRIORITY"
    assert len(robot.actions) == before
    assert owner.brain.snapshot()["primary_goal"]["goal_id"] == human["goal_id"]


def test_general_question_does_not_preempt_physical_goal():
    _, robot, owner = runtime()
    active = adopt(owner, "Explore", [{"action": "behavior.room_cruise"}])
    before = robot.stops
    question = owner.brain.submit("Mennyi kétszer kettő?")
    owner.brain.fail(question["goal_id"], "ANSWERED")
    assert owner.brain.goal(active["goal_id"])["lifecycle"] == "ACTIVE"
    assert robot.stops == before


def test_stop_revokes_plan_waiting_on_canonical_execution_and_stops_late_return():
    _, robot, owner = runtime()
    entered, release = threading.Event(), threading.Event()
    execute = robot.execute

    def blocked(action, **parameters):
        entered.set()
        assert release.wait(2)
        return execute(action, **parameters)

    robot.execute = blocked
    pending = owner.brain.submit("Explore")
    results = []
    worker = threading.Thread(target=lambda: results.append(owner.brain.adopt(
        pending["goal_id"], {"steps": [{"action": "behavior.room_cruise"}]})))
    worker.start()
    assert entered.wait(1)
    stopper = threading.Thread(target=lambda: owner.preempt("STOP"))
    stopper.start()
    assert owner.brain.goal(pending["goal_id"])["lifecycle"] == "CANCELLED"
    release.set()
    worker.join(2)
    stopper.join(2)
    assert not worker.is_alive() and not stopper.is_alive()
    assert results[0]["lifecycle"] == "CANCELLED"
    assert not owner.behaviors.active
    with pytest.raises(RuntimeError, match="REVOKED"):
        owner.brain.adopt(pending["goal_id"], {"steps": [{"action": "behavior.room_cruise"}]})


def test_stale_status_at_requested_duration_is_failure_without_retry():
    clock, robot, owner = runtime()
    goal = adopt(owner, "Explore for 50 seconds", [{"action": "behavior.room_cruise",
        "parameters": {"max_duration_s": 50}, "max_retries": 2}])
    clock.now += 50_000_000_000
    owner.poll()
    assert owner.brain.goal(goal["goal_id"])["lifecycle"] == "FAILED"
    assert len(robot.actions) == 1


def test_restart_restores_memory_but_never_restarts_physical_goal():
    _, robot, owner = runtime()
    goal = adopt(owner, "Explore", [{"action": "behavior.room_cruise"}])
    saved = owner.brain.export_state()
    _, other_robot, restored = runtime()
    assert restored.brain.restore(saved)
    assert restored.brain.snapshot()["primary_goal"]["lifecycle"] == "INTERRUPTED"
    assert restored.brain.snapshot()["primary_goal"]["goal_id"] == goal["goal_id"]
    assert other_robot.actions == []


def test_search_follow_binding_fails_closed_after_runtime_restart():
    _, robot, owner = runtime()
    pending = owner.brain.submit("Find and follow")
    # A completed search event must keep both track and runtime identity.
    from r2b4_orchestration.behavior_system import BehaviorUpdate, BehaviorLifecycle

    class Search:
        def start(self, port, parameters):
            robot.pid = 456
            return BehaviorUpdate(BehaviorLifecycle.COMPLETED, "PERSON_FOUND",
                                  result={"target_track_id": "person-7", "runtime_pid": 123})

    owner.behaviors._factories["search_any_person"] = Search
    result = owner.brain.adopt(pending["goal_id"], {"steps": [
        {"action": "behavior.search_any_person", "bind_target": True},
        {"action": "behavior.follow_person", "use_bound_target": True,
         "parameters": {"max_duration_s": 300}}]})
    assert result["lifecycle"] == "FAILED" and result["reason"] == "TARGET_BINDING_STALE"
    assert robot.actions == []


def test_all_explicit_step_durations_survive_plan_admission():
    _, robot, owner = runtime()
    goal = adopt(owner, "Search for 10 seconds then follow for 5 minutes", [
        {"action": "behavior.search_any_person", "parameters": {"max_duration_s": 20}, "bind_target": True},
        {"action": "behavior.follow_person", "parameters": {"max_duration_s": 300}, "use_bound_target": True}])
    assert goal["lifecycle"] == "FAILED"
    assert goal["reason"] == "CONSTRAINT_UNSUPPORTED:durations_s"
    assert robot.actions == []


def test_raw_observation_cannot_satisfy_requested_visual_conclusion():
    _, robot, owner = runtime()
    goal = adopt(owner, "Menj a konyhába és nézd meg, ég-e a lámpa.", [
        {"action": "v3.command.navigate", "parameters": {"x_m": 1, "y_m": 2}},
        {"action": "vision.observe"}])
    assert goal["lifecycle"] == "FAILED"
    assert goal["reason"] == "CONSTRAINT_UNSUPPORTED:observation_query"
    assert robot.actions == []


def test_metadata_poll_does_not_wait_behind_finite_action_dispatch():
    _, robot, owner = runtime()
    entered, release = threading.Event(), threading.Event()
    original = robot.execute

    def blocked(action, **params):
        entered.set()
        assert release.wait(2)
        return original(action, **params)

    robot.execute = blocked
    pending = owner.brain.submit("Move")
    owner.execute("brain.adopt", {"goal_id": pending["goal_id"],
        "plan": {"steps": [{"action": "v3.command.move_relative", "parameters": {"forward_m": 1}}]}})
    assert entered.wait(1)
    start = time.monotonic()
    owner.poll()
    assert time.monotonic() - start < 0.5
    assert owner.brain.goal(pending["goal_id"])["lifecycle"] == "STARTING"
    release.set()


def test_stop_failure_cannot_kill_dispatcher_or_create_success():
    _, robot, owner = runtime()
    stop = robot.stop
    failed = threading.Event()

    def broken():
        failed.set()
        raise RuntimeError("canonical STOP unavailable")

    robot.stop = broken
    first = adopt(owner, "Move", [{"action": "v3.command.move_relative", "parameters": {"forward_m": 1}}])
    assert first["lifecycle"] == "FAILED" and failed.is_set()
    assert robot.actions == []
    robot.stop = stop
    second = adopt(owner, "Move", [{"action": "v3.command.move_relative", "parameters": {"forward_m": 1}}])
    assert second["lifecycle"] == "COMPLETED"
    assert owner.brain._dispatch_thread.is_alive()


def test_revoked_behavior_cannot_install_after_stop_or_preempting_plan(monkeypatch):
    _, robot, owner = runtime()
    entered, release = threading.Event(), threading.Event()
    start = owner.behaviors.start

    def delayed_start(*args, **kwargs):
        entered.set()
        assert release.wait(2)
        return start(*args, **kwargs)

    monkeypatch.setattr(owner.behaviors, "start", delayed_start)
    pending = owner.brain.submit("Explore")
    old_results = []
    worker = threading.Thread(target=lambda: old_results.append(owner.brain.adopt(
        pending["goal_id"], {"steps": [{"action": "behavior.room_cruise"}]})))
    worker.start()
    assert entered.wait(1)
    # Admission has passed the Brain check, but has not installed a Behavior.
    replacement = owner.brain.submit("Follow")
    owner.execute("brain.adopt", {"goal_id": replacement["goal_id"],
        "plan": {"steps": [{"action": "behavior.follow_person"}]}})
    assert owner.brain.goal(pending["goal_id"])["lifecycle"] == "CANCELLED"
    release.set()
    worker.join(2)
    assert not worker.is_alive()
    deadline = time.monotonic() + 2
    while owner.brain.goal(replacement["goal_id"])["lifecycle"] == "STARTING" and time.monotonic() < deadline:
        time.sleep(0.001)
    assert old_results[0]["lifecycle"] == "CANCELLED"
    assert owner.brain.goal(replacement["goal_id"])["lifecycle"] == "ACTIVE"
    assert owner.behaviors.snapshot().goal_id == replacement["goal_id"]
    assert [action for action, _ in robot.actions] == ["v3.command.follow_person"]


def test_finite_admission_lineage_is_visible_before_completion_and_stop_cancels_wait(monkeypatch):
    _, robot, owner = runtime()
    observer = owner.observation_hub.subscribe_reliable("finite-lineage", capacity=64, topics={"r2b4.brain"})
    entered, release = threading.Event(), threading.Event()
    cancellations = []

    def delayed_action(action, **params):
        assert action == "v3.command.move_relative"
        cancellations.append(params["cancel_event"])
        params["admission_sink"]({"command_id": "finite-1", "mission_id": "mission-finite-1",
            "runtime_pid": 123, "pose_status_monotonic_ns": robot.clock.now})
        entered.set()
        assert release.wait(2)
        # Even an incorrectly late success cannot reactivate a revoked goal.
        return {"command_id": "finite-1", "mission_id": "mission-finite-1",
                "status": "COMPLETED", "reason": "COMPLETE"}

    monkeypatch.setattr(robot, "execute", delayed_action)
    pending = owner.brain.submit("Move")
    owner.execute("brain.adopt", {"goal_id": pending["goal_id"], "plan": {"steps": [
        {"action": "v3.command.move_relative", "parameters": {"forward_m": .5}}]}})
    assert entered.wait(1)
    admission = next(frame.payload for frame in observer.drain() if frame.payload.kind == "ACTION_ACCEPTED")
    assert admission.state.goal_id == pending["goal_id"]
    assert admission.state.command_id == "finite-1"
    assert admission.state.mission_id == "mission-finite-1"
    assert admission.state.lifecycle.value == "STARTING"
    stopper = threading.Thread(target=lambda: owner.preempt("STOP"))
    stopper.start()
    assert cancellations[0].wait(1)
    release.set()
    stopper.join(2)
    assert not stopper.is_alive()
    assert owner.brain.goal(pending["goal_id"])["lifecycle"] == "CANCELLED"
    late = next(frame.payload for frame in observer.drain() if frame.payload.kind == "ACTION_RESULT_AFTER_REVOCATION")
    assert late.state.lifecycle.value == "CANCELLED"
    assert late.state.command_id == admission.state.command_id
