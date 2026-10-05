from __future__ import annotations

import json

import pytest

from r2b4_orchestration.world_model import FreshnessPolicy, KnowledgeState, PublicWorldModel


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
