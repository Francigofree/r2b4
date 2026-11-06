"""Bounded semantic retention cannot turn historical evidence into current motion."""
from __future__ import annotations

import json

from r2b4_orchestration.spatial_service import SpatialQuery, SpatialService
from r2b4_orchestration.world_model import KnowledgeState, PublicWorldModel, ValidityScope, WorldQuery


class Clock:
    now = 20_000_000_000

    def __call__(self):
        return self.now


def observe(world, clock, entity, domain, value, *, attribute="value", **extra):
    return world.observe(entity, attribute, value, domain=domain, measurement_time_ns=clock.now,
                         confidence=1.0, source="human" if domain in {"person_identity", "room_topology", "user_preference"} else "qualified-result",
                         lineage=("original:measurement",), **extra)


def test_pose_and_mission_churn_cannot_evict_taught_semantics_or_other_retention_classes():
    clock = Clock()
    limits = {group: {"max_facts": count, "max_bytes": 4096}
              for group, count in (("transient", 3), ("knowledge", 3), ("experience", 2))}
    world = PublicWorldModel(clock_ns=clock, clock_epoch="boot-a", max_facts=8,
                             max_facts_bytes=12 * 1024, retention_limits=limits)
    teachings = [observe(world, clock, "person:anna", "person_identity", {"name": "Anna"}),
                 observe(world, clock, "room:kitchen", "room_topology", {"name": "konyha"}),
                 observe(world, clock, "person:anna", "user_preference", "formal", attribute="address")]
    for index in range(100):
        clock.now += 1
        observe(world, clock, f"robot-pose:{index}", "pose", {"x_m": index})
        observe(world, clock, f"mission:{index}", "mission_outcome", {"lifecycle": "COMPLETED"})
    for event in teachings:
        fact = world.read(event.observation.entity_id, event.observation.attribute)
        assert fact.observation == event.observation
        assert fact.state is KnowledgeState.KNOWN
    snapshot = world.snapshot().to_jsonable()
    assert len(snapshot["facts"]) <= world.max_facts
    assert snapshot["retention"]["knowledge"]["facts_evicted"] == 0
    assert snapshot["retention"]["transient"]["facts_evicted"] > 0
    assert snapshot["retention"]["experience"]["facts_evicted"] > 0
    for pool in snapshot["retention"].values():
        assert pool["fact_count"] <= pool["max_facts"]
        assert pool["bytes_used"] <= pool["max_bytes"]
    restored = PublicWorldModel.from_state(json.loads(json.dumps(world.export_state())),
                                          clock_ns=clock, clock_epoch="boot-b")
    assert restored.read("person:anna", "value").state is KnowledgeState.KNOWN
    assert restored.read("person:anna", "value").observation == teachings[0].observation
    assert restored.read("mission:99", "value").value["lifecycle"] == "COMPLETED"
    assert restored.snapshot().to_jsonable()["retention"] == snapshot["retention"]


def test_legacy_restore_is_lossless_and_reports_admission_pressure_without_cross_class_eviction():
    clock = Clock()
    original = PublicWorldModel(clock_ns=clock, clock_epoch="boot-a", max_facts=2)
    for index in range(2):
        observe(original, clock, f"person:{index}", "person_identity", {"name": str(index)})
    state = original.export_state()
    del state["retention"]
    restored = PublicWorldModel.from_state(state, clock_ns=clock, clock_epoch="boot-a")
    assert [fact.observation for fact in restored.snapshot().facts.values()] == [fact.observation for fact in original.snapshot().facts.values()]
    rejected = observe(restored, clock, "robot", "pose", {"x_m": 1})
    assert not rejected.accepted and rejected.reason == "RETENTION_CAPACITY"
    assert restored.read("robot", "value").state is KnowledgeState.UNKNOWN
    assert restored.read("person:0", "value").value["name"] == "0"
    assert restored.snapshot().to_jsonable()["retention"]["transient"]["observations_rejected"] == 1


def test_same_boot_restore_requires_new_binding_measurement_and_preserves_provenance():
    clock = Clock()
    world = PublicWorldModel(clock_ns=clock, clock_epoch="boot-a")
    scope = ValidityScope(frame_id="local", runtime_pid=123)
    identity = observe(world, clock, "person:anna", "person_identity", {"name": "Anna"}, attribute="identity")
    binding = observe(world, clock, "person:anna", "person_binding", {"track_id": "track-a"},
                      attribute="person_binding", sequence=7, validity_scope=scope)
    restored = PublicWorldModel.from_state(world.export_state(), clock_ns=clock, clock_epoch="boot-a")
    assert restored.read("person:anna", "identity").observation == identity.observation
    fact = restored.read("person:anna", "person_binding")
    assert fact.state is KnowledgeState.STALE and fact.freshness == "RESTORED_UNVALIDATED"
    assert fact.observation == binding.observation
    query = WorldQuery(entity_id="person:anna", domain="person_binding", scope=scope, require_current=True)
    assert not restored.query(query).facts
    repeated = observe(restored, clock, "person:anna", "person_binding", {"track_id": "track-a"},
                       attribute="person_binding", sequence=8, validity_scope=scope)
    assert not repeated.accepted
    assert not restored.query(query).facts
    clock.now += 1
    fresh = observe(restored, clock, "person:anna", "person_binding", {"track_id": "track-a"},
                    attribute="person_binding", sequence=9, validity_scope=scope)
    assert restored.query(query).facts[0].observation == fresh.observation


def test_spatial_geometry_churn_keeps_taught_place_and_restores_semantic_name():
    clock = Clock()
    world = PublicWorldModel(clock_ns=clock, clock_epoch="boot-a")
    spatial = SpatialService(world, clock_ns=clock, max_facts=4)
    taught = observe(world, clock, "room:kitchen", "room_topology", "konyha", attribute="name")
    spatial.sync_world()
    for index in range(30):
        clock.now += 1
        observe(world, clock, f"geometry:{index}", "map", {"kind": "visited_area"}, attribute="location")
        spatial.sync_world()
    memory = spatial.query(SpatialQuery(entity_id="room:kitchen")).entities[0].facts[0]
    assert memory.observation == taught.observation
    assert spatial.snapshot()["retention"]["knowledge"]["facts_evicted"] == 0
    assert spatial.snapshot()["retention"]["transient"]["facts_evicted"] > 0
    saved_world, saved_spatial = world.export_state(), spatial.export_state()
    restored_world = PublicWorldModel.from_state(saved_world, clock_ns=clock, clock_epoch="boot-b")
    restored = SpatialService(restored_world, clock_ns=clock, max_facts=4)
    restored.restore(saved_spatial)
    name = restored.query(SpatialQuery(entity_id="room:kitchen", require_current=True)).entities[0].facts[0]
    assert name.observation == taught.observation
    assert name.state is KnowledgeState.KNOWN
    assert not restored.query(SpatialQuery(entity_id="geometry:29", require_current=True)).entities
