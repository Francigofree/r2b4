"""Human person teaching preserves a running physical goal and verified feedback."""
import threading

import pytest

from r2b4_orchestration.brain_hri import BrainAdoption, BrainGoalObserver, adopt_brain_result, wait_for_brain_goal
from tests.packs.feature.test_brain_core import wait_for
from tests.packs.feature.test_person_executive import adopt, setup_person, teach_goal


def active_follow(owner, duration_s=20):
    pending = owner.brain.submit(f"Kövesd az embert {duration_s} másodpercig")
    goal = owner.brain.adopt(pending["goal_id"], {"steps": [{"action": "behavior.follow_person",
        "parameters": {"max_duration_s": duration_s}, "completion": "duration"}]})
    assert goal["lifecycle"] == "ACTIVE"
    return goal


@pytest.mark.parametrize("asynchronous", [False, True])
def test_human_name_teaching_keeps_physical_goal_behavior_and_command(asynchronous):
    _, backend, owner = setup_person()
    physical = active_follow(owner)
    behavior, stops, actions = owner.behaviors.snapshot(), backend.stops, list(backend.actions)
    knowledge = adopt(owner, "Name this person Anna", asynchronous=asynchronous)
    wait_for(lambda: owner.brain.goal(knowledge["goal_id"])["lifecycle"] == "COMPLETED")
    primary = owner.brain.snapshot()["primary_goal"]
    assert primary["goal_id"] == physical["goal_id"] and primary["lifecycle"] == "ACTIVE"
    for field in ("command_id", "mission_id", "current_node_id", "constraints"):
        assert primary[field] == physical[field]
    assert owner.behaviors.snapshot().behavior_id == behavior.behavior_id
    assert owner.behaviors.snapshot().command_id == behavior.command_id
    assert owner.behaviors.active and backend.stops == stops and backend.actions == actions
    remembered = owner.brain.goal(knowledge["goal_id"])
    assert owner.person_identity.resolve("Anna") == remembered["result"]["entity_id"]
    feedback = wait_for_brain_goal(owner.brain.robot, BrainAdoption("ACTIVE", "", knowledge["goal_id"]), timeout_s=1)
    assert feedback.status == "COMPLETED" and "Anna" in feedback.text


def test_teaching_clarification_and_read_only_answer_leave_physical_goal_active():
    _, backend, owner = setup_person()
    physical = active_follow(owner)
    backend.status["world"]["person_tracks"].append(dict(
        backend.status["world"]["person_tracks"][0], track_id="person-8"))
    stops, actions = backend.stops, list(backend.actions)
    knowledge = teach_goal(owner, "Name this person Anna")
    assert knowledge["lifecycle"] == "FAILED" and "CLARIFICATION" in knowledge["reason"]
    feedback = wait_for_brain_goal(owner.brain.robot, BrainAdoption("ACTIVE", "", knowledge["goal_id"]), timeout_s=1)
    assert "Több személyt látok" in feedback.text and "melyik" in feedback.text
    question = owner.brain.submit("Mennyi kétszer kettő?")
    owner.brain.fail(question["goal_id"], "ANSWERED", pending_only=True)
    assert owner.brain.goal(physical["goal_id"])["lifecycle"] == "ACTIVE"
    assert owner.brain.snapshot()["primary_goal"]["goal_id"] == physical["goal_id"]
    assert backend.stops == stops and backend.actions == actions


def test_stop_cancels_late_interjection_without_waiting_for_storage_or_restarting_motion(monkeypatch):
    _, backend, owner = setup_person()
    physical = active_follow(owner)
    entered, release = threading.Event(), threading.Event()

    def storage(*, force=False):
        entered.set()
        assert release.wait(3)
        return False

    monkeypatch.setattr(owner, "_persist", storage)
    knowledge = adopt(owner, "Name this person Anna", asynchronous=True)
    assert entered.wait(2)
    stops, actions = backend.stops, list(backend.actions)
    busy = adopt(owner, "Name this person Bela", asynchronous=True)
    assert busy["lifecycle"] == "FAILED" and busy["reason"] == "PERSON_TEACHING_BUSY"
    assert owner.brain.goal(physical["goal_id"])["lifecycle"] == "ACTIVE" and backend.stops == stops
    stopped = threading.Event()
    thread = threading.Thread(target=lambda: (owner.preempt("USER_STOP"), stopped.set()), daemon=True)
    thread.start()
    try:
        assert stopped.wait(1), "STOP waited for an informational teaching save"
        assert owner.brain.goal(physical["goal_id"])["lifecycle"] == "CANCELLED"
        assert owner.brain.goal(knowledge["goal_id"])["lifecycle"] == "CANCELLED"
    finally:
        release.set()
        thread.join(2)
    wait_for(lambda: any(event.kind == "ACTION_RESULT_AFTER_REVOCATION" and event.state.goal_id == knowledge["goal_id"]
                         for event in owner.brain.history()))
    assert owner.brain.goal(knowledge["goal_id"])["lifecycle"] == "CANCELLED"
    assert backend.actions == actions and backend.stops > stops and not owner.behaviors.active


@pytest.mark.parametrize("visible", [False, True])
def test_feedback_distinguishes_absent_track_from_multiple_visible_people(visible):
    _, backend, owner = setup_person()
    if visible:
        backend.status["world"]["person_tracks"].append(dict(
            backend.status["world"]["person_tracks"][0], track_id="person-8"))
    else:
        backend.status["world"]["person_tracks"] = []
    knowledge = teach_goal(owner, "Name this person Anna")
    feedback = wait_for_brain_goal(owner.brain.robot, BrainAdoption("ACTIVE", "", knowledge["goal_id"]), timeout_s=1)
    assert ("Több személyt látok" in feedback.text) is visible
    assert ("Nem látok friss" in feedback.text) is not visible


def test_feedback_reports_memory_only_from_the_actual_teaching_result(monkeypatch):
    _, _, owner = setup_person()
    monkeypatch.setattr(owner, "_persist", lambda **_: False)
    knowledge = teach_goal(owner, "Name this person Anna")
    assert knowledge["result"]["durability"] == "MEMORY_ONLY"
    feedback = wait_for_brain_goal(owner.brain.robot, BrainAdoption("ACTIVE", "", knowledge["goal_id"]), timeout_s=1)
    assert "Anna" in feedback.text and "tartós mentés nem sikerült" in feedback.text

    class Interface:
        def execute(self, action, **parameters):
            return {"lifecycle": "COMPLETED", "current_subtask": "person.teach", "result": {"name": "Anna"}}

    unsupported = adopt_brain_result(Interface(), {"goal_id": "unverified", "proposed_plan": {"steps": []}})
    assert "Anna" not in unsupported.text


@pytest.mark.parametrize("inflight", [False, True])
def test_physical_failure_cancels_pending_or_inflight_teaching_and_releases_slot(monkeypatch, inflight):
    _, backend, owner = setup_person()
    physical = active_follow(owner)
    entered, release = threading.Event(), threading.Event()

    def storage(*, force=False):
        entered.set()
        assert release.wait(3)
        return False

    monkeypatch.setattr(owner, "_persist", storage)
    if not inflight:
        dispatch = owner.brain._dispatch_teaching_interjection

        def pending_dispatch(*arguments):
            entered.set()
            assert release.wait(3)
            dispatch(*arguments)

        monkeypatch.setattr(owner.brain, "_dispatch_teaching_interjection", pending_dispatch)
    knowledge = adopt(owner, "Name this person Anna", asynchronous=True)
    assert entered.wait(2)
    if not inflight:
        assert owner.brain.goal(knowledge["goal_id"])["lifecycle"] == "STARTING"
    failed = threading.Event()
    thread = threading.Thread(target=lambda: (owner.brain.fail(physical["goal_id"], "SAFETY_STOP:TEST"), failed.set()), daemon=True)
    thread.start()
    try:
        assert failed.wait(1), "physical failure waited for teaching storage"
        cancelled = owner.brain.goal(knowledge["goal_id"])
        assert cancelled["lifecycle"] == "CANCELLED"
        assert "PHYSICAL_GOAL_FAILED" in cancelled["reason"]
    finally:
        release.set()
        thread.join(2)
    if inflight:
        wait_for(lambda: any(event.kind == "ACTION_RESULT_AFTER_REVOCATION" and event.state.goal_id == knowledge["goal_id"]
                             for event in owner.brain.history()))
    actions = list(backend.actions)
    owner.brain._dispatch_teaching_interjection(knowledge["goal_id"], owner.brain._generation)
    assert owner.brain.goal(knowledge["goal_id"])["lifecycle"] == "CANCELLED"
    assert backend.actions == actions
    monkeypatch.setattr(owner, "_persist", lambda **_: False)
    active_follow(owner)
    confirmed = adopt(owner, "Name this person Anna")
    assert confirmed["lifecycle"] == "COMPLETED", confirmed


def test_long_teaching_save_cannot_complete_after_node_deadline(monkeypatch):
    clock, backend, owner = setup_person()
    physical = active_follow(owner, duration_s=300)
    stops = backend.stops

    def slow_storage(*, force=False):
        clock.now += 121_000_000_000
        backend.publish(active=True)
        return False

    monkeypatch.setattr(owner, "_persist", slow_storage)
    knowledge = adopt(owner, "Name this person Anna")
    assert knowledge["lifecycle"] == "FAILED" and "TIMEOUT" in knowledge["reason"]
    assert knowledge["failure_code"] == "TIMEOUT"
    failures = [event.state for event in owner.brain.history()
                if event.kind == "SUBTASK_FAILED" and event.state.goal_id == knowledge["goal_id"]]
    assert len(failures) == 1 and failures[0].current_node_id == knowledge["current_node_id"]
    assert failures[0].failure_code.value == "TIMEOUT"
    assert owner.brain.goal(physical["goal_id"])["lifecycle"] == "ACTIVE" and backend.stops == stops


def test_physical_failure_during_capability_read_prevents_the_unstarted_teaching(monkeypatch):
    _, _, owner = setup_person()
    physical = active_follow(owner)
    entered, release, finished = threading.Event(), threading.Event(), threading.Event()
    capabilities = owner.brain.robot.capabilities
    dispatch = owner.brain._dispatch_teaching_interjection

    def finished_dispatch(*arguments):
        try:
            dispatch(*arguments)
        finally:
            finished.set()

    def delayed_capabilities():
        result = capabilities()
        if owner.brain._teaching_interjection is not None:
            entered.set()
            assert release.wait(3)
        return result

    monkeypatch.setattr(owner.brain.robot, "capabilities", delayed_capabilities)
    monkeypatch.setattr(owner.brain, "_dispatch_teaching_interjection", finished_dispatch)
    knowledge = adopt(owner, "Name this person Anna", asynchronous=True)
    assert entered.wait(2)
    try:
        owner.brain.fail(physical["goal_id"], "SAFETY_STOP:TEST")
        assert owner.brain.goal(knowledge["goal_id"])["lifecycle"] == "CANCELLED"
    finally:
        release.set()
    assert finished.wait(1)
    assert owner.person_identity.resolve("Anna") is None


def test_goal_feedback_preserves_physical_watch_through_one_teaching_interjection(monkeypatch):
    _, _, owner = setup_person()
    physical = active_follow(owner)
    feedback = []
    observer = BrainGoalObserver(owner.brain.robot, feedback_sink=lambda text, fields: feedback.append((text, fields)))
    entered, release = threading.Event(), threading.Event()

    def storage(*, force=False):
        entered.set()
        assert release.wait(3)
        return False

    monkeypatch.setattr(owner, "_persist", storage)
    observer.observe(physical["goal_id"], turn_id="physical-turn")
    knowledge = adopt(owner, "Name this person Anna", asynchronous=True)
    assert entered.wait(2)
    observer.observe(knowledge["goal_id"], turn_id="teaching-turn")
    observer.observe(knowledge["goal_id"], turn_id="duplicate-turn")
    try:
        assert set(observer._watches) == {physical["goal_id"], knowledge["goal_id"]}
        release.set()
        wait_for(lambda: any(fields["goal_id"] == knowledge["goal_id"] for _, fields in feedback))
        assert not any(fields["goal_id"] == physical["goal_id"] for _, fields in feedback)
        owner.preempt("USER_STOP")
        wait_for(lambda: len(feedback) == 2)
        by_goal = {fields["goal_id"]: (text, fields) for text, fields in feedback}
        assert by_goal[knowledge["goal_id"]][1]["turn_id"] == "teaching-turn"
        assert by_goal[knowledge["goal_id"]][1]["lifecycle"] == "COMPLETED"
        assert "Anna" in by_goal[knowledge["goal_id"]][0]
        assert by_goal[physical["goal_id"]][1]["turn_id"] == "physical-turn"
        assert by_goal[physical["goal_id"]][1]["lifecycle"] == "CANCELLED"
        assert observer._watches == {}
    finally:
        release.set()
        observer.close()
