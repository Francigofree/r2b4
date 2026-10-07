"""Offline evidence for local decomposition and proposal-only HRI ingress."""
from types import SimpleNamespace

import pytest

from r2b4_orchestration.local_task_planner import LocalTaskPlanner, explicit_metric_constraints
from r2b4_orchestration.world_model import PublicWorldModel
from r2b4_voice.conversation_contracts import LLMDecision, RobotContextSnapshot
from r2b4_voice.conversation_interface import _OnDemandLLM
from r2b4_voice.conversation_journal import ConversationJournal
from r2b4_voice.conversation_service import ConversationService


class Interface:
    def __init__(self):
        self.world = PublicWorldModel(clock_ns=lambda: 1_000_000_000, clock_epoch="planner-test")
        self.executed = []
        self.history = []

    def query(self, query):
        return self.world.query(query)

    def read(self, resource):
        return {"primary_goal": None} if resource == "brain.state" else self.history

    def execute(self, action, **params):
        self.executed.append((action, params))
        assert action in {"brain.submit", "brain.fail"}, "interpretation never executes robot proposals"
        return {"goal_id": params.get("request_id", params.get("goal_id")), "lifecycle": "PENDING"}

    def observe(self, entity, domain, value, attribute="location"):
        return self.world.observe(entity, attribute, value, domain=domain,
            measurement_time_ns=1_000_000_000, confidence=1, source="semantic-test",
            lineage=("source:1",), sequence=1)


@pytest.mark.parametrize("text", [
    "Menj előre 1 métert, fordulj jobbra 90 fokot, menj még fél métert.",
    "Move forward 1 metre, turn right 90 degrees, then move another half metre.",
])
def test_three_step_motion_keeps_order_direction_and_exact_distances(text):
    result = LocalTaskPlanner().resolve(text, Interface())
    assert result.plan == {"steps": [
        {"action": "v3.command.move_relative", "parameters": {"forward_m": 1}},
        {"action": "v3.command.turn_by", "parameters": {"angle_deg": -90}},
        {"action": "v3.command.move_relative", "parameters": {"forward_m": .5}},
    ]}


@pytest.mark.parametrize("text,expected", [
    ("Menj 1 m-t, aztán még fél métert.", {"distances_m": (1, .5)}),
    ("Menj előre 20 centimétert, majd kövesd két percig.", {"distance_m": .2, "duration_s": 120}),
    ("Move forward one metre, follow them for two minutes.", {"distance_m": 1, "duration_s": 120}),
    ("Kövesd 1,5 méterről öt másodpercig.", {"follow_distance_m": 1.5, "duration_s": 5}),
    ("Menj 1 métert, kövesd 2 méterről 30 másodpercig.", {"distance_m": 1, "follow_distance_m": 2, "duration_s": 30}),
    ("Barangolj 10 másodpercig, majd kövesd fél percig.", {"durations_s": (10, 30)}),
])
def test_pure_request_constraints_preserve_units_words_and_action_ownership(text, expected):
    actual = explicit_metric_constraints(text)
    assert {name: actual[name] for name in expected} == expected


@pytest.mark.parametrize("text,value", [
    ("menj előre 0.3m-t", .3), ("indulj előre 1m-t", 1),
    ("hátra 1m", -1), ("menj 1 métert hátra", -1), ("menj 0,3m-t előre", .3),
])
def test_observed_metric_aliases_preserve_signed_distance_locally(text, value):
    result = LocalTaskPlanner().resolve(text, Interface())
    assert result.plan["steps"] == [{"action": "v3.command.move_relative", "parameters": {"forward_m": value}}]
    assert explicit_metric_constraints(text)["motion_sequence"] == (("move", value),)


@pytest.mark.parametrize("text,action", [
    ("kövesd az embert", "behavior.follow_person"),
    ("szoba felfedezés", "behavior.room_cruise"),
    ("csinálj egy képet", "vision.observe"),
])
def test_observed_behavior_aliases_stay_local(text, action):
    assert LocalTaskPlanner().resolve(text, Interface()).plan["steps"][0]["action"] == action


@pytest.mark.parametrize("text,angles", [
    ("fordulj balra 185 fokot", [180, 5]), ("fordulj 195 fokot jobbra", [-180, -15]),
])
def test_large_turn_retains_requested_direction_with_bounded_primitives(text, angles):
    plan = LocalTaskPlanner().resolve(text, Interface()).plan
    assert [step["parameters"]["angle_deg"] for step in plan["steps"]] == angles
    assert explicit_metric_constraints(text)["motion_sequence"] == (("turn", sum(angles)),)


def test_unsupported_large_turn_and_missing_direction_request_clarification_locally():
    for text in ("fordulj balra 720 fokot", "185 fok"):
        result = LocalTaskPlanner().resolve(text, Interface())
        assert result.unfulfilled and result.spoken_text and result.plan is None


def test_known_place_resolution_emits_entity_without_coordinate_authority():
    interface = Interface()
    interface.observe("room:kitchen", "room_topology", {"name": "konyha", "x_m": 1,
        "y_m": 2, "frame_id": "R2B4_BOOT_ROBOT_MAP", "runtime_pid": 123})
    result = LocalTaskPlanner().resolve("Menj a konyhába.", interface)
    assert result.plan == {"steps": [{"action": "v3.command.navigate", "parameters": {},
                                     "target_entity_id": "room:kitchen"}]}
    assert interface.executed == []


def test_named_search_follow_binds_same_target_and_preserves_duration():
    interface = Interface()
    interface.observe("person:peter", "person_identity", {"name": "Péter"}, attribute="identity")
    result = LocalTaskPlanner().resolve("Keresd meg Pétert és kövesd 2 percig.", interface)
    assert result.plan["steps"] == [
        {"action": "behavior.search_person", "parameters": {"entity_id": "person:peter"}, "bind_target": True},
        {"action": "behavior.follow_person", "parameters": {"max_duration_s": 120}, "use_bound_target": True},
    ]


def test_look_around_uses_only_canonical_turn_and_new_observation_at_each_view():
    rows = LocalTaskPlanner().resolve("Nézz körül.", Interface()).plan["steps"]
    assert [row["action"] for row in rows] == ["vision.observe"] + ["v3.command.turn_by", "vision.observe"] * 4
    assert sum(row["parameters"].get("angle_deg", 0) for row in rows) == 360


def test_house_search_visits_known_places_and_only_exact_bound_person_can_follow():
    interface = Interface()
    interface.observe("room:kitchen", "room_topology", {"name": "konyha"})
    interface.observe("room:lounge", "room_topology", {"name": "nappali"})
    result = LocalTaskPlanner().resolve("Keress valakit a lakásban és kövesd két percig.", interface)
    nodes = result.plan["nodes"]
    assert [node["target_entity_id"] for node in nodes if node.get("action") == "v3.command.navigate"] == ["room:kitchen", "room:lounge"]
    search_nodes = [node for node in nodes if node.get("action") == "behavior.search_any_person"]
    assert all(node["bind_target"] and node["on_success"] == "follow" for node in search_nodes)
    assert all(node["failure_on"] == ["TARGET_LOST"] for node in search_nodes)
    follow = next(node for node in nodes if node["node_id"] == "follow")
    assert follow["use_bound_target"] and follow["parameters"]["max_duration_s"] == 120


def test_long_local_goal_composes_observation_search_follow_and_canonical_return():
    interface = Interface()
    interface.observe("room:lounge", "room_topology", {"name": "nappali"})
    result = LocalTaskPlanner().resolve("Menj a nappaliba, nézz körül, keress valakit. Ha találsz valakit, kövesd két percig, aztán gyere vissza.", interface)
    rows = [node for node in result.plan["nodes"] if node["kind"] == "action"]
    assert rows[0]["target_entity_id"] == "room:lounge"
    assert sum(row["action"] == "vision.observe" for row in rows) == 5
    search = next(row for row in rows if row["action"] == "behavior.search_any_person")
    follow = next(row for row in rows if row["action"] == "behavior.follow_person")
    assert search["bind_target"] and search["on_failure"] == "not-found"
    assert follow["use_bound_target"] and follow["parameters"]["max_duration_s"] == 120
    assert rows[-1]["return_to_origin"] and rows[-1]["parameters"] == {}


def test_go_observe_return_never_places_origin_coordinates_in_planner():
    interface = Interface()
    interface.observe("room:kitchen", "room_topology", {"name": "konyha"})
    result = LocalTaskPlanner().resolve("Menj a konyhába, készíts képet, gyere vissza.", interface)
    assert [row["action"] for row in result.plan["steps"]] == ["v3.command.navigate", "vision.observe", "v3.command.navigate"]
    assert result.plan["steps"][-1] == {"action": "v3.command.navigate", "parameters": {}, "return_to_origin": True}


def test_unknown_semantics_is_scoped_and_does_not_drop_local_steps():
    result = LocalTaskPlanner().resolve("Menj előre 1 métert, majd értékeld a rendetlenséget, majd fordulj balra.", Interface())
    assert not result.resolved
    assert result.unresolved_text == "értékeld a rendetlenséget"
    merged = result.merge_specialist_plan({"steps": [{"action": "vision.observe", "parameters": {}}]})
    assert [row["action"] for row in merged["steps"]] == ["v3.command.move_relative", "vision.observe", "v3.command.turn_by"]
    assert merged["steps"][0]["parameters"] == {"forward_m": 1}


def test_conditional_navigation_proposes_no_path_refresh_retry_and_report():
    interface = Interface()
    interface.observe("door:hall", "object_identity", {"name": "ajtó"}, attribute="identity")
    result = LocalTaskPlanner().resolve("Menj az ajtóhoz. Ha nem lehet odajutni, próbálj másik utat. Ha az sem sikerül, szólj.", interface)
    nodes = result.plan["nodes"]
    assert result.plan["entry"] == "navigate"
    assert nodes[0]["target_entity_id"] == nodes[2]["target_entity_id"] == "door:hall"
    assert nodes[0]["failure_on"] == ["NO_PATH"]
    assert nodes[1]["condition"]["require_current"] is True
    assert nodes[3]["failure_code"] == "NO_PATH"


def test_previous_failure_answer_uses_correlated_history_and_never_invents_reason():
    interface = Interface()
    interface.history = [{"goal_id": "old", "text": "Keresd meg Pétert", "lifecycle": "FAILED",
                          "reason": "SEARCH_PLACES_EXHAUSTED"},
                         {"goal_id": "unrelated", "text": "Menj a konyhába", "lifecycle": "FAILED", "reason": "NO_PATH"},
                         {"goal_id": "current", "text": "Miért?", "lifecycle": "PENDING"}]
    result = LocalTaskPlanner().resolve("Miért nem találtad meg Pétert?", interface, goal_id="current")
    assert "SEARCH_PLACES_EXHAUSTED" in result.spoken_text
    assert result.plan is None and interface.executed == []


def test_task_introspection_distinguishes_finished_primary_and_correlates_subtask_evidence():
    interface = Interface()
    final = {"goal_id": "find", "text": "Keresd meg Pétert", "lifecycle": "FAILED", "reason": "SEARCH_PLACES_EXHAUSTED"}
    interface.history = [
        {**final, "kind": "BEHAVIOR_RESULT", "subtask_id": "find:node:search-kitchen",
         "current_subtask": "behavior.search_person", "world_target": {"entity_id": "room:kitchen"},
         "result": {"reason": "SEARCH_VIEWS_EXHAUSTED"}},
        {"goal_id": "other", "kind": "BEHAVIOR_RESULT", "subtask_id": "other:node:search",
         "current_subtask": "unrelated", "result": {"reason": "OTHER_REASON"}},
        final,
    ]
    interface.read = lambda resource: {"primary_goal": final} if resource == "brain.state" else interface.history
    status = LocalTaskPlanner().resolve("Mit csinálsz?", interface).spoken_text
    assert "utolsó feladat" in status and "jelenlegi feladat" not in status
    answer = LocalTaskPlanner().resolve("Miért nem találtad meg Pétert?", interface).spoken_text
    assert "room:kitchen" in answer and "SEARCH_VIEWS_EXHAUSTED" in answer
    assert "OTHER_REASON" not in answer and "unrelated" not in answer


def service(tmp_path, interface, model, *, context=None):
    return ConversationService(llm=model, brain_interface=interface,
        robot_context=SimpleNamespace(build=context or (lambda: RobotContextSnapshot("test", {}, None, None, (), ()))),
        prompt_assembler=SimpleNamespace(build_messages=lambda turn, *a, **k: [{"role": "user", "content": turn.text}]),
        journal=ConversationJournal(tmp_path / "journal"))


def test_production_conversation_local_motion_does_not_touch_provider_or_context(tmp_path):
    class OfflineModel:
        model = "offline"
        def complete(self, _messages):
            raise AssertionError("known local task cannot need an LLM")
    def no_context():
        raise AssertionError("local primitive cannot need provider context")
    interface = Interface()
    conversation = service(tmp_path, interface, OfflineModel(), context=no_context)
    try:
        turn = conversation.submit_text("Menj előre 1 métert, fordulj jobbra 90 fokot, menj még fél métert.")
        result = conversation.wait_for_turn(turn, timeout_s=2)
        assert result["model"] == "local-task-planner"
        assert result["action_status"] == "PLAN_PROPOSED" and result["error"] is None
        assert len(result["proposed_plan"]["steps"]) == 3
        assert [name for name, _ in interface.executed] == ["brain.submit"]
    finally:
        conversation.close()


def test_scoped_specialist_cannot_omit_locally_resolved_motion(tmp_path):
    class Model:
        model = "specialist"
        def complete(self, messages):
            assert messages[-1]["content"] == "értékeld a rendetlenséget"
            assert "surrounding canonical steps" in messages[0]["content"]
            return LLMDecision("Ehhez nincs megfelelő képesség.", None, self.model, unfulfilled=True)
    interface = Interface()
    conversation = service(tmp_path, interface, Model())
    try:
        turn = conversation.submit_text("Menj előre 1 métert, majd értékeld a rendetlenséget.")
        result = conversation.wait_for_turn(turn, timeout_s=2)
        assert result["action_status"] == "FAILED:REQUEST_UNFULFILLED"
        assert result["proposed_plan"] is None
        assert [name for name, _ in interface.executed] == ["brain.submit", "brain.fail"]
    finally:
        conversation.close()


@pytest.mark.parametrize("text", ["Menj oda.", "Menj a konyhába.", "Keress valakit a lakásban."])
def test_clarification_or_missing_search_places_cannot_claim_goal_completed(tmp_path, text):
    class Model:
        model = "offline"
        def complete(self, _messages):
            raise AssertionError("local capability precondition must be answered locally")
    interface = Interface()
    conversation = service(tmp_path, interface, Model())
    try:
        turn = conversation.submit_text(text)
        result = conversation.wait_for_turn(turn, timeout_s=2)
        assert result["action_status"] == "FAILED:REQUEST_UNFULFILLED"
        assert result["proposed_plan"] is None and result["error"] is None
        assert interface.executed[-1][1]["reason"] == "REQUEST_UNFULFILLED"
    finally:
        conversation.close()


def test_no_credentials_needed_until_specialist_is_called(monkeypatch, tmp_path):
    import r2b4_voice.conversation_interface as composition
    initialized = []
    def unavailable(**_settings):
        initialized.append(True)
        raise RuntimeError("no usable LLM authentication")
    monkeypatch.setattr(composition, "build_llm_client", unavailable)
    model = _OnDemandLLM(provider="openai", project_root=tmp_path)
    assert initialized == []
    conversation = service(tmp_path, Interface(), model)
    try:
        local_turn = conversation.submit_text("Nézz körül.")
        assert conversation.wait_for_turn(local_turn, timeout_s=2)["error"] is None
        assert initialized == []
        open_turn = conversation.submit_text("Beszélgessünk az irodalomról.")
        result = conversation.wait_for_turn(open_turn, timeout_s=2)
        assert result["action_status"] == "ERROR" and "authentication" in result["error"]
        assert initialized == [True]
    finally:
        conversation.close()


def test_graph_proposals_cross_conversation_without_execution(tmp_path):
    interface = Interface()
    interface.observe("door:hall", "object_identity", {"name": "ajtó"}, attribute="identity")
    class Model:
        model = "offline"
        def complete(self, _messages):
            raise AssertionError("local recovery needs no LLM")
    conversation = service(tmp_path, interface, Model())
    try:
        turn = conversation.submit_text("Menj az ajtóhoz. Ha nem lehet odajutni, próbálj másik utat. Ha az sem sikerül, szólj.")
        result = conversation.wait_for_turn(turn, timeout_s=2)
        assert result["proposed_plan"]["entry"] == "navigate"
        assert result["action_status"] == "PLAN_PROPOSED"
        assert [name for name, _ in interface.executed] == ["brain.submit"]
    finally:
        conversation.close()


@pytest.mark.parametrize("text", [
    "Menj előre 1 métert, fordulj jobbra 90 fokot, menj még fél métert.",
    "Menj előre 20 centimétert, majd kövesd két percig.",
    "Keress valakit a lakásban és kövesd két percig.",
    "Menj a nappaliba, nézz körül, keress valakit. Ha találsz valakit, kövesd két percig, aztán gyere vissza.",
    "Menj a nappaliba, készíts képet, gyere vissza.",
])
def test_local_plans_preserve_source_request_constraints_at_brain_admission(text):
    from tests.packs.feature.test_brain_core import runtime
    clock, robot, owner = runtime()
    for index, name in enumerate(("nappali", "konyha", "eloszoba", "halo", "dolgozo", "etkezo")):
        owner.world.observe("room:" + name, "location", {"name": name, "x_m": index,
            "y_m": 0, "frame_id": "R2B4_BOOT_ROBOT_MAP", "runtime_pid": 123},
            domain="room_topology", measurement_time_ns=clock.now, confidence=1,
            source="public-semantic-source", lineage=("map:1",))
    interface = SimpleNamespace(query=owner.world.query, spatial_query=owner.spatial.query)
    result = LocalTaskPlanner().resolve(text, interface)
    pending = owner.brain.submit(text)
    goal = owner.brain._goals[pending["goal_id"]]
    if "nodes" in result.plan:
        _, steps, constraints = owner.brain._graph_plan(result.plan, goal)
    else:
        steps, constraints = owner.brain._plan(result.plan, goal)
    assert steps and constraints == goal.constraints
    assert robot.actions == [], "plan admission does not dispatch hardware"
