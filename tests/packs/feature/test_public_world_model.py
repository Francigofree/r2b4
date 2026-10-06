from __future__ import annotations

import json
from dataclasses import FrozenInstanceError
from threading import Thread

import pytest

from r2b4_orchestration.world_model import (
    FreshnessPolicy, KnowledgeState, PublicWorldModel, ValidityScope,
    WorldLocation, WorldQuery, WorldQueryResult,
)


SECOND = 1_000_000_000


def model_at(now: list[int], **kwargs) -> PublicWorldModel:
    return PublicWorldModel(clock_ns=lambda: now[0], clock_epoch="boot-a", **kwargs)


def person(model: PublicWorldModel, value="kitchen", **kwargs):
    parameters = {"domain": "person_position", "measurement_time_ns": SECOND,
                  "confidence": 0.95, "source": "vision", "sequence": 1,
                  "lineage": {"frame_id": "camera-1", "detector_revision": 7}}
    parameters.update(kwargs)
    return model.observe("person:laci", "last_observed_location", value, **parameters)


def test_domain_freshness_uses_measurement_time_and_preserves_lineage():
    now = [3 * SECOND]
    model = model_at(now)
    event = person(model, observation_time_ns=3 * SECOND, revision="camera-session")
    model.observe("room:kitchen", "topology", {"door": "hall"}, domain="room_topology",
                  measurement_time_ns=SECOND, confidence=0.9, source="mapper")

    now[0] = 5 * SECOND
    fact = model.read("person:laci", "last_observed_location")
    assert fact.state is KnowledgeState.STALE
    assert fact.age_ns == 4 * SECOND
    assert fact.observation.measurement_time_ns == SECOND
    assert fact.observation.observation_time_ns == 3 * SECOND
    assert fact.observation.sequence == 1
    assert fact.observation.revision == "camera-session"
    assert fact.observation.lineage["frame_id"] == "camera-1"
    assert model.read("room:kitchen", "topology").state is KnowledgeState.KNOWN
    assert event.event_id == "world:boot-a:1"

    # Polling the same measurement cannot renew its age or create new knowledge.
    repeated = person(model, observation_time_ns=now[0], revision="camera-session")
    assert not repeated.accepted
    assert repeated.reason == "DUPLICATE"
    assert model.read("person:laci", "last_observed_location").age_ns == 4 * SECOND


def test_snapshots_and_observations_are_deeply_immutable_and_detached():
    now = [2 * SECOND]
    model = model_at(now)
    value = {"room": "kitchen", "position": [1, 2]}
    event = person(model, value)
    snapshot = model.snapshot()
    value["position"][0] = 999
    value["room"] = "hall"
    fact = snapshot.facts[("person:laci", "last_observed_location")]
    assert fact.value["position"] == (1, 2)
    with pytest.raises(TypeError):
        fact.value["room"] = "hall"
    with pytest.raises(TypeError):
        snapshot.facts[("other", "key")] = fact
    with pytest.raises(TypeError):
        event.observation.lineage["frame_id"] = "changed"

    person(model, "hall", measurement_time_ns=2 * SECOND, sequence=2)
    assert fact.value["room"] == "kitchen"
    json_view = snapshot.to_jsonable()
    json_view["facts"][0]["value"]["position"][0] = 99
    assert fact.value["position"] == (1, 2)


def test_older_and_lower_sequence_observations_are_evidence_not_current_truth():
    now = [10 * SECOND]
    model = model_at(now)
    person(model, "hall", measurement_time_ns=8 * SECOND, sequence=20, revision="session-a")
    old = person(model, "kitchen", measurement_time_ns=7 * SECOND, sequence=21, revision="session-a")
    assert not old.accepted and old.reason == "OUT_OF_ORDER"
    invalid_sequence = person(model, "garden", measurement_time_ns=9 * SECOND,
                              sequence=19, revision="session-a")
    assert not invalid_sequence.accepted and invalid_sequence.reason == "OUT_OF_ORDER_SEQUENCE"
    assert model.read("person:laci", "last_observed_location").value == "hall"
    assert [event.accepted for event in model.history()] == [True, False, False]

    # A separately identified restarted producer may restart its sequence.
    restarted = person(model, "kitchen", measurement_time_ns=9 * SECOND,
                       sequence=1, revision="session-b")
    assert restarted.accepted
    assert model.read("person:laci", "last_observed_location").value == "kitchen"


def test_uncertainty_and_conflict_are_visible_until_new_measurement():
    now = [2 * SECOND]
    model = model_at(now)
    assert model.read("person:laci", "location").state is KnowledgeState.UNKNOWN
    person(model, confidence=0.4)
    assert model.read("person:laci", "last_observed_location").state is KnowledgeState.LIKELY
    conflict = person(model, "hall", source="external-camera", confidence=0.99)
    fact = model.read("person:laci", "last_observed_location")
    assert conflict.accepted
    assert fact.state is KnowledgeState.CONFLICTING
    assert fact.conflicts[0].value == "kitchen"
    assert fact.to_jsonable()["conflicts"][0]["source"] == "vision"
    person(model, "hall", source="external-camera", measurement_time_ns=2 * SECOND, sequence=2)
    assert model.read("person:laci", "last_observed_location").state is KnowledgeState.KNOWN
    now[0] = 6 * SECOND
    assert model.read("person:laci", "last_observed_location").state is KnowledgeState.STALE

    model.observe("object:cup", "location", None, domain="object_position",
                  measurement_time_ns=now[0], confidence=0, source="human")
    assert model.read("object:cup", "location").state is KnowledgeState.UNKNOWN


def test_bounded_snapshot_and_history_report_eviction_and_history_loss():
    now = [SECOND]
    model = model_at(now, max_facts=2, history_capacity=2)
    for number in range(4):
        event = model.observe(f"room:{number}", "topology", number, domain="room_topology",
                              measurement_time_ns=SECOND, confidence=0.9, source="mapper")
    snapshot = model.snapshot()
    assert len(snapshot.facts) == 2
    assert model.read("room:0", "topology").state is KnowledgeState.UNKNOWN
    assert event.evicted == ("room:1", "topology")
    assert snapshot.history_dropped == 2
    assert snapshot.history_first_sequence == 3
    assert snapshot.history_last_sequence == 4
    assert [item.event_sequence for item in model.history(after_sequence=3)] == [4]


def test_durable_export_preserves_timestamps_conflicts_and_new_boot_invalidates_age():
    now = [2 * SECOND]
    model = model_at(now)
    person(model, "kitchen", revision=7)
    person(model, "hall", source="human", revision=8)
    state = json.loads(json.dumps(model.export_state()))
    restored = PublicWorldModel.from_state(state, clock_ns=lambda: now[0], clock_epoch="boot-a")
    assert restored.export_state() == state
    assert restored.read("person:laci", "last_observed_location").state is KnowledgeState.CONFLICTING
    assert restored.history()[0].observation.measurement_time_ns == SECOND

    # Monotonic timestamps from a previous boot never acquire a new freshness age.
    now[0] = 100
    rebooted = PublicWorldModel.from_state(state, clock_ns=lambda: now[0], clock_epoch="boot-b")
    stale = rebooted.read("person:laci", "last_observed_location")
    assert stale.state is KnowledgeState.STALE
    assert stale.freshness == "CLOCK_MISMATCH"
    assert stale.age_ns is None
    assert stale.observation.measurement_time_ns == SECOND
    assert stale.observation.clock_epoch == "boot-a"
    replaced = person(rebooted, "garden", measurement_time_ns=100, sequence=1)
    assert replaced.accepted and replaced.reason == "NEW_CLOCK_EPOCH"
    assert rebooted.read("person:laci", "last_observed_location").state is KnowledgeState.KNOWN
    assert rebooted.history()[0].event_id == "world:boot-a:1"
    assert rebooted.history()[-1].event_id == "world:boot-b:3"


def test_explicit_domain_policy_survives_export_and_restore():
    now = [10 * SECOND]
    model = model_at(now, policies={"semantic_test": FreshnessPolicy(SECOND, known_confidence=0.6)})
    model.observe("test", "meaning", True, domain="semantic_test", measurement_time_ns=10 * SECOND,
                  confidence=0.7, source="external")
    restored = PublicWorldModel.from_state(model.export_state(), clock_ns=lambda: now[0], clock_epoch="boot-a")
    assert restored.read("test", "meaning").state is KnowledgeState.KNOWN
    now[0] += 2 * SECOND
    assert restored.read("test", "meaning").state is KnowledgeState.STALE


def test_invalid_clock_payload_domain_and_confidence_cannot_enter_snapshot():
    now = [2 * SECOND]
    model = model_at(now)
    for parameters in ({"confidence": float("nan")}, {"confidence": True},
                       {"clock_epoch": "other-boot"}, {"domain": "unregistered"},
                       {"measurement_time_ns": 3 * SECOND}, {"observation_time_ns": 3 * SECOND}):
        with pytest.raises(ValueError):
            person(model, **parameters)
    for value in (b"raw image", {"raw": "x" * 16_385}, list(range(600)), {"value": float("inf")}):
        with pytest.raises(ValueError):
            person(model, value)
    assert not model.snapshot().facts
    assert not model.history()


def test_invalid_restore_is_atomic_and_does_not_rewrite_existing_facts():
    now = [SECOND]
    model = model_at(now, max_facts=1)
    person(model)
    original = model.export_state()
    invalid = json.loads(json.dumps(original))
    invalid["facts"].append(invalid["facts"][0])
    with pytest.raises(ValueError, match="capacity"):
        model.restore(invalid)
    assert model.export_state() == original
    invalid = json.loads(json.dumps(original))
    invalid["facts"][0]["observation"]["domain"] = "unregistered"
    with pytest.raises(ValueError, match="domain"):
        model.restore(invalid)
    assert model.export_state() == original


def test_aggregate_byte_limits_keep_snapshot_history_and_durable_state_under_host_envelope():
    now = [SECOND]
    model = model_at(now)
    # JSON escaping makes the wire payload substantially larger than its Python
    # text length. Count bounds alone cannot bound a public reply or saved state.
    value = "\x01" * 16_000
    for number in range(20):
        event = model.observe(f"object:{number}", "identity", value, domain="object_identity",
                              measurement_time_ns=SECOND, confidence=0.9, source="external")
    snapshot = model.snapshot()
    assert snapshot.facts_evicted > 0
    assert snapshot.history_dropped > 0
    assert event.evicted_facts
    assert len(json.dumps(snapshot.to_jsonable()).encode()) < 256 * 1024
    assert len(json.dumps([item.to_jsonable() for item in model.history()]).encode()) < 256 * 1024
    state = model.export_state()
    assert len(json.dumps(state).encode()) < 512 * 1024
    restored = PublicWorldModel.from_state(json.loads(json.dumps(state)),
                                          clock_ns=lambda: now[0], clock_epoch="boot-a")
    assert restored.export_state() == state


def test_byte_budget_rejection_and_restore_are_atomic():
    now = [SECOND]
    model = model_at(now, max_facts_bytes=4096, max_history_bytes=4096)
    person(model)
    original = model.export_state()
    with pytest.raises(ValueError, match="byte budget"):
        person(model, "x" * 8_000, measurement_time_ns=SECOND, sequence=2)
    assert model.export_state() == original

    larger = model_at(now)
    person(larger, "x" * 8_000)
    with pytest.raises(ValueError, match="byte budget"):
        model.restore(larger.export_state())
    assert model.export_state() == original


def test_repeat_physical_measurement_does_not_rewrite_measurement_lineage_on_new_status_tick():
    now = [2 * SECOND]
    model = model_at(now)
    person(model, sequence=10)
    now[0] += SECOND
    repeated = person(model, sequence=11)
    assert repeated.reason == "DUPLICATE"
    assert not repeated.accepted
    fact = model.read("person:laci", "last_observed_location")
    assert fact.observation.sequence == 10
    assert fact.observation.observation_time_ns == 2 * SECOND
    assert fact.age_ns == 2 * SECOND


def test_typed_query_roundtrip_is_immutable_exact_and_does_not_renew_evidence():
    now = [2 * SECOND]
    model = model_at(now)
    person(model, {"place_id": "room:kitchen"})
    model.observe("room:kitchen", "topology", {"door": "hall"}, domain="room_topology",
                  measurement_time_ns=SECOND, confidence=1.0, source="mapper")
    query = WorldQuery(entity_id="person:laci", attribute="last_observed_location",
                       domain="person_position", limit=1, require_current=True)
    assert WorldQuery.from_jsonable(json.loads(json.dumps(query.to_jsonable()))) == query
    result = model.query(query)
    restored = WorldQueryResult.from_jsonable(json.loads(json.dumps(result.to_jsonable())))
    assert restored == result
    assert result.facts[0].location == WorldLocation(place_id="room:kitchen")
    assert result.facts[0].domain == "person_position"
    with pytest.raises(FrozenInstanceError):
        restored.query.limit = 2
    with pytest.raises(TypeError):
        restored.facts[0].value["place_id"] = "room:hall"
    now[0] += 2 * SECOND
    assert model.query(query).facts  # At the inclusive domain age budget.
    now[0] += 1
    assert not model.query(query).facts
    evidence = model.query(WorldQuery(entity_id="person:laci")).facts[0]
    assert evidence.observation.measurement_time_ns == SECOND
    assert evidence.observation.observation_time_ns == 2 * SECOND
    assert evidence.age_ns == 3 * SECOND + 1
    with pytest.raises(ValueError, match="policy"):
        model.query(WorldQuery(domain="unregistered"))
    with pytest.raises(ValueError, match="limit"):
        WorldQuery(limit=65)


def test_spatial_queries_require_all_declared_scope_fields_and_reject_previous_runtime():
    now = [2 * SECOND]
    model = model_at(now)
    model.observe("room:kitchen", "location",
                  {"x_m": 1.0, "y_m": 2.0, "frame_id": "map", "runtime_pid": 7, "map_revision": 3},
                  domain="room_topology", measurement_time_ns=SECOND, confidence=1.0, source="mapper")
    scope = ValidityScope(frame_id="map", runtime_pid=7, map_revision=3)
    fact = model.query(WorldQuery(entity_id="room:kitchen", scope=scope)).facts[0]
    assert fact.observation.validity_scope == scope
    assert fact.state is KnowledgeState.KNOWN
    assert fact.location == WorldLocation(x_m=1, y_m=2, frame_id="map")
    for invalid_context in (None, ValidityScope(runtime_pid=7),
                            ValidityScope(frame_id="map", runtime_pid=8, map_revision=3),
                            ValidityScope(frame_id="map", runtime_pid=7, map_revision=4)):
        query = WorldQuery(entity_id="room:kitchen", scope=invalid_context)
        invalid = model.query(query).facts[0]
        assert invalid.state is KnowledgeState.UNKNOWN
        assert invalid.freshness == "SCOPE_MISMATCH"
        assert invalid.observation == fact.observation
        assert not model.query(WorldQuery(entity_id="room:kitchen", scope=invalid_context,
                                          require_current=True)).facts
    assert WorldQueryResult.from_jsonable(model.query(WorldQuery(scope=scope)).to_jsonable()).facts == (fact,)
    state = model.export_state()
    restored = PublicWorldModel.from_state(state, clock_ns=lambda: now[0], clock_epoch="boot-a")
    assert restored.query(WorldQuery(scope=scope)).facts == (fact,)
    # Old durable records carry their session/frame in the value rather than a
    # typed scope. Reading them must enforce the same qualification boundary.
    for raw in state["facts"]:
        raw["observation"].pop("validity_scope")
    for raw in state["history"]:
        raw["observation"].pop("validity_scope")
    legacy = PublicWorldModel.from_state(state, clock_ns=lambda: now[0], clock_epoch="boot-a")
    assert legacy.query(WorldQuery(scope=scope)).facts[0].state is KnowledgeState.KNOWN
    assert legacy.query(WorldQuery()).facts[0].freshness == "SCOPE_MISMATCH"


def test_explicit_scope_is_detached_and_locations_reject_partial_or_invalid_coordinates():
    now = [SECOND]
    model = model_at(now)
    scope = {"runtime_pid": 9}
    event = person(model, {"place_id": "room:kitchen"}, validity_scope=scope)
    scope["runtime_pid"] = 10
    assert event.observation.validity_scope == ValidityScope(runtime_pid=9)
    assert not model.query(WorldQuery(require_current=True)).facts
    assert model.query(WorldQuery(scope=ValidityScope(runtime_pid=9), require_current=True)).facts
    for value in ({"x_m": 1, "frame_id": "map"}, {"x_m": 1, "y_m": 2},
                  {"x_m": True, "y_m": 2, "frame_id": "map"},
                  {"x_m": float("nan"), "y_m": 2, "frame_id": "map"}):
        with pytest.raises(ValueError):
            WorldLocation.from_jsonable(value)
    person(model, {"x_m": 1, "frame_id": "map"}, measurement_time_ns=SECOND,
           sequence=2, revision="new")
    assert model.query(WorldQuery()).facts[0].location is None


def test_episodic_queries_preserve_rejections_and_report_cursor_gap_and_truncation():
    now = [3 * SECOND]
    model = model_at(now, history_capacity=3)
    person(model, "hall", measurement_time_ns=2 * SECOND)
    person(model, "kitchen", measurement_time_ns=SECOND)
    person(model, "hall", measurement_time_ns=2 * SECOND)
    person(model, "garden", measurement_time_ns=3 * SECOND, sequence=2)
    query = WorldQuery(kind="episodes", entity_id="person:laci", limit=2)
    result = model.query(query)
    assert result.history_dropped == 1
    assert result.history_first_sequence == 2
    assert result.history_last_sequence == 4
    assert result.history_gap and result.truncated
    assert [event.reason for event in result.events] == ["OUT_OF_ORDER", "DUPLICATE"]
    assert all(not event.accepted for event in result.events)
    assert WorldQueryResult.from_jsonable(json.loads(json.dumps(result.to_jsonable()))) == result
    next_result = model.query(WorldQuery(kind="episodes", after_sequence=3))
    assert [event.event_sequence for event in next_result.events] == [4]
    assert not next_result.history_gap and not next_result.truncated
    model.observe("room:kitchen", "topology", "hall", domain="room_topology",
                  measurement_time_ns=SECOND, confidence=1.0, source="mapper")
    assert model.query(WorldQuery(limit=1)).truncated


def test_passive_sink_receives_every_committed_event_outside_lock_and_restore_is_silent():
    now = [2 * SECOND]
    received = []
    visible = []
    model = model_at(now)

    def sink(event):
        received.append(event)
        reader = Thread(target=lambda: visible.append(model.history()[-1].event_sequence))
        reader.start()
        reader.join(timeout=1.0)
        assert not reader.is_alive(), "model lock must be released before passive evidence delivery"
        raise OSError("capture unavailable")

    model.set_event_sink(sink)
    person(model)
    person(model)
    person(model, "hall", measurement_time_ns=0)
    assert [event.reason for event in received] == ["NEW", "DUPLICATE", "OUT_OF_ORDER"]
    assert visible == [1, 2, 3]
    assert model.event_sink_errors == 3
    assert model.read("person:laci", "last_observed_location").value == "kitchen"
    model.restore(model.export_state())
    assert len(received) == 3
    model.set_event_sink(None)
    person(model, "hall", measurement_time_ns=2 * SECOND, sequence=2)
    assert model.read("person:laci", "last_observed_location").value == "hall"
    assert model.event_sink_errors == 3
