"""Derived persistent spatial knowledge never renews or revives old coordinates."""
import json

import pytest

from r2b4_orchestration.robot_runtime import PublicRobotInterfaceAdapter, PublicRobotRuntime
from r2b4_orchestration.spatial_service import SpatialQuery, SpatialQueryResult, SpatialService
from r2b4_orchestration.world_model import KnowledgeState, PublicWorldModel, ValidityScope
from v3.robot_interface import RobotInterface


class Clock:
    now = 20_000_000_000

    def __call__(self):
        return self.now


def context(clock, *, generation=1, tick=1, frame="local"):
    return {"monotonic_ns": clock.now - 10, "tick_id": tick,
            "estimate": {"frame_id": frame, "localization_quality": {"generation": generation}},
            "world": {"frame_id": frame, "map_revision": 7}}


def place(world, clock, *, sequence=1, x_m=1.0, pid=123):
    return world.observe("room:kitchen", "location", {"frame_id": "local", "x_m": x_m, "y_m": 2.0},
                         domain="room_topology", measurement_time_ns=clock.now - 100,
                         source="semantic_projector", confidence=1.0, sequence=sequence,
                         revision=7, lineage=("runtime:123", "L4:map:7", "camera:8"),
                         validity_scope=ValidityScope(frame_id="local", runtime_pid=pid, map_revision=7))


def setup():
    clock = Clock()
    world = PublicWorldModel(clock_ns=clock, clock_epoch="boot-a")
    spatial = SpatialService(world, clock_ns=clock)
    spatial.completed_status(context(clock), runtime_pid=123)
    return clock, world, spatial


def test_spatial_entities_and_relations_derive_only_original_public_evidence():
    clock, world, spatial = setup()
    source = place(world, clock)
    world.observe("room:kitchen", "name", "konyha", domain="room_topology",
                  measurement_time_ns=clock.now, source="user", confidence=1.0)
    world.observe("room:kitchen", "topology", {"neighbors": ["room:hall"]}, domain="room_topology",
                  measurement_time_ns=clock.now, source="semantic_projector", confidence=1.0)
    world.observe("person:laci", "location", {"place_id": "room:kitchen"}, domain="person_position",
                  measurement_time_ns=clock.now - 100, source="semantic_vision", confidence=0.9)
    entities = spatial.query(SpatialQuery(require_current=True)).entities
    kitchen = next(item for item in entities if item.entity_id == "room:kitchen")
    assert kitchen.kind == "place"
    fact = next(fact for fact in kitchen.facts if fact.attribute == "location")
    assert fact.observation == source.observation
    assert fact.world_revision == source.world_revision
    assert fact.state is KnowledgeState.KNOWN
    relations = spatial.query(SpatialQuery(kind="relations")).relations
    assert {(item.entity_id, item.relation, item.target_entity_id) for item in relations} == {
        ("room:kitchen", "connected_to", "room:hall"),
        ("person:laci", "located_in", "room:kitchen"),
    }
    with pytest.raises(TypeError):
        fact.value["x_m"] = 100
    assert SpatialQueryResult.from_jsonable(spatial.query(SpatialQuery()).to_jsonable()).entities == entities
    clock.now += 400_000_000
    spatial.completed_status(context(clock, tick=2), runtime_pid=123)
    again = spatial.query(SpatialQuery(entity_id="room:kitchen")).entities[0].facts
    same = next(fact for fact in again if fact.attribute == "location")
    assert same.observation.measurement_time_ns == source.observation.measurement_time_ns
    assert same.age_ns == clock.now - source.observation.measurement_time_ns


def test_frame_generation_restart_and_stale_context_invalidate_current_coordinates():
    clock, world, spatial = setup()
    place(world, clock)
    query = SpatialQuery(entity_id="room:kitchen", require_current=True)
    assert spatial.query(query).entities
    clock.now += 1_000
    spatial.completed_status(context(clock, generation=2, tick=2), runtime_pid=123)
    assert not spatial.query(query).entities
    # Returning to the same frame/generation cannot revive earlier evidence.
    clock.now += 1_000
    spatial.completed_status(context(clock, generation=1, tick=3), runtime_pid=123)
    assert not spatial.query(query).entities
    clock.now += 1_000
    place(world, clock, sequence=2, x_m=3.0)
    assert spatial.query(query).entities[0].facts[0].value["x_m"] == 3.0
    clock.now += 1_000
    spatial.completed_status(context(clock, tick=4), runtime_pid=456)
    assert not spatial.query(query).entities
    assert spatial.query(SpatialQuery(entity_id="room:kitchen")).entities[0].facts[0].freshness == "SCOPE_MISMATCH"
    clock.now += 1_000
    place(world, clock, sequence=3, pid=456)
    assert spatial.query(query).entities
    clock.now += 500_000_000
    assert not spatial.query(query).entities
    assert spatial.query(SpatialQuery(entity_id="room:kitchen")).entities[0].facts[0].freshness == "CONTEXT_STALE"


def test_delayed_old_measurement_cannot_claim_new_localization_generation():
    clock, world, spatial = setup()
    place(world, clock)
    spatial.sync_world()
    measured_before_change = clock.now + 100
    clock.now += 10_000
    spatial.completed_status(context(clock, generation=2, tick=2), runtime_pid=123)
    delayed = world.observe("room:kitchen", "location", {"frame_id": "local", "x_m": 4.0, "y_m": 2.0},
                            domain="room_topology", measurement_time_ns=measured_before_change,
                            source="semantic_projector", confidence=1.0, sequence=2,
                            validity_scope=ValidityScope(frame_id="local", runtime_pid=123, map_revision=7))
    assert delayed.accepted
    assert not spatial.query(SpatialQuery(entity_id="room:kitchen", require_current=True)).entities
    assert spatial.query(SpatialQuery()).entities[0].facts[0].freshness == "MEASUREMENT_BEFORE_CONTEXT_CHANGE"


def test_missing_localization_generation_does_not_qualify_coordinates():
    clock = Clock()
    world = PublicWorldModel(clock_ns=clock, clock_epoch="boot-a")
    spatial = SpatialService(world, clock_ns=clock)
    status = context(clock)
    del status["estimate"]["localization_quality"]["generation"]
    spatial.completed_status(status, runtime_pid=123)
    place(world, clock)
    assert not spatial.query(SpatialQuery(require_current=True)).entities
    assert spatial.query(SpatialQuery()).entities[0].facts[0].freshness == "LOCALIZATION_GENERATION_UNAVAILABLE"


def test_restore_retains_memory_but_requires_new_source_measurement_even_same_boot():
    clock, world, spatial = setup()
    place(world, clock)
    saved_world, saved_spatial = world.export_state(), json.loads(json.dumps(spatial.export_state()))
    restored_world = PublicWorldModel.from_state(saved_world, clock_ns=clock, clock_epoch="boot-a")
    restored = SpatialService(restored_world, clock_ns=clock)
    restored.restore(saved_spatial)
    restored.completed_status(context(clock), runtime_pid=123)
    query = SpatialQuery(entity_id="room:kitchen", require_current=True)
    assert not restored.query(query).entities
    memory = restored.query(SpatialQuery(entity_id="room:kitchen")).entities[0].facts[0]
    assert memory.freshness == "RESTORED_UNVALIDATED"
    assert memory.observation.lineage == ("runtime:123", "L4:map:7", "camera:8")
    clock.now += 1_000
    place(restored_world, clock, sequence=2)
    assert restored.query(query).entities
    rebooted_world = PublicWorldModel.from_state(saved_world, clock_ns=clock, clock_epoch="boot-b")
    rebooted = SpatialService(rebooted_world, clock_ns=clock)
    rebooted.restore(saved_spatial)
    old = rebooted.query(SpatialQuery()).entities[0].facts[0]
    assert old.state is KnowledgeState.STALE and old.freshness == "CLOCK_MISMATCH"
    assert old.age_ns is None


def test_spatial_storage_is_bounded_and_retains_evicted_source_as_stale_memory():
    clock = Clock()
    world = PublicWorldModel(clock_ns=clock, clock_epoch="boot-a", max_facts=1)
    spatial = SpatialService(world, clock_ns=clock, max_facts=2)
    for index in range(3):
        world.observe(f"room:{index}", "name", f"room {index}", domain="room_topology",
                      measurement_time_ns=clock.now, confidence=1.0, source="user")
        spatial.sync_world()
    result = spatial.query(SpatialQuery())
    assert [item.entity_id for item in result.entities] == ["room:1", "room:2"]
    assert result.facts_evicted == 1
    assert result.entities[0].facts[0].freshness == "SOURCE_UNAVAILABLE"
    assert [item.entity_id for item in spatial.query(SpatialQuery(require_current=True)).entities] == ["room:2"]


def test_host_runtime_exposes_and_persists_spatial_subsystem_through_robot_interface(tmp_path):
    clock = Clock()

    class Backend:
        def read(self, resource):
            return {"runtime_pid": 123} if resource == "operator.status" else context(clock)

        def stop(self):
            raise AssertionError("read-only spatial work requested physical STOP")

    backend = Backend()
    world = PublicWorldModel(clock_ns=clock, clock_epoch="boot-a")
    runtime = PublicRobotRuntime(backend, root=tmp_path, world=world, clock_ns=clock)
    runtime.ingest_status(context(clock), runtime_pid=123)
    place(world, clock)

    class Client:
        def request(self, operation, **arguments):
            if operation == "read":
                return runtime.read(arguments["resource"])
            assert operation == "spatial_query"
            return runtime.spatial_query(SpatialQuery.from_jsonable(arguments["query"])).to_jsonable()

    interface = RobotInterface(controller=backend, adapters=(PublicRobotInterfaceAdapter(Client()),), upper_runtime=False)
    assert interface.capabilities()["capabilities"]["spatial.query"]["kind"] == "read"
    assert interface.read("spatial.snapshot")["motion_authority"] is False
    assert interface.spatial_query(SpatialQuery(entity_id="room:kitchen")).entities[0].entity_id == "room:kitchen"
    runtime._persist(force=True)
    saved = json.loads((tmp_path / "runtime/public_world/state.json").read_text())
    assert saved["spatial"]["facts"][0]["fact"]["measurement_time_ns"] == clock.now - 100
    restored = PublicRobotRuntime(backend, root=tmp_path, world=PublicWorldModel(clock_ns=clock, clock_epoch="boot-a"), clock_ns=clock)
    assert restored.read("spatial.snapshot")["entities"][0]["facts"][0]["freshness"] == "RESTORED_UNVALIDATED"
