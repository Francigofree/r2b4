"""Derived persistent spatial knowledge never renews or revives old coordinates."""
import json
import time
from types import SimpleNamespace

import pytest

from r2b4_orchestration.robot_runtime import PublicRobotInterfaceAdapter, PublicRobotRuntime
from r2b4_orchestration.spatial_service import SpatialQuery, SpatialQueryResult, SpatialService
from r2b4_orchestration.world_model import KnowledgeState, PublicWorldModel, ValidityScope
from v3.robot_interface import RobotInterface


def geometry_tick(clock, *, tick=1, sequence=8, measured=None, x_m=1.25,
                  frame="R2B4_ODOM_LOCAL", generation=1, cells=1):
    from rig import healthy_localization
    from v3.contracts import (CostmapCell, FinalActuation, RobotEstimate,
                             RollingLocalCostmap, SafetyDecision, TickContext, WorldSnapshot)
    from v3.engine import LayerRecord, TickResult, TickTrace
    stamp = clock.now - 10
    measured = stamp - 10_000_000 if measured is None else measured
    ctx = TickContext(tick, stamp)
    estimate = RobotEstimate(ctx, frame, x_m, 2.25, .2, 0., 0., (0.,) * 25,
                             localization_quality=healthy_localization(generation=generation),
                             transform_revision=4)
    costmap = RollingLocalCostmap(frame, 7, .1, 4.,
        tuple(CostmapCell(index, 0, 1) for index in range(cells)), sequence, stamp - measured)
    world = WorldSnapshot(ctx, frame, 9, (), stamp - measured, local_costmap=costmap)
    actuation = FinalActuation(ctx, 0., 0., False, SafetyDecision.STOP, "CLEAR", "NOT_ACTIVE")
    return TickResult(actuation, TickTrace(ctx, (
        LayerRecord("L3", estimate), LayerRecord("L4", world))))


def completed_geometry(clock, **values):
    """Exercise the real completed V3 -> compact status -> host projection."""
    from v3.resident_status import _tick_status
    return _tick_status(geometry_tick(clock, **values))


def test_completed_geometry_populates_persistent_anonymous_spatial_memory(tmp_path):
    clock = Clock()

    class Backend:
        def stop(self):
            raise AssertionError("spatial population must not actuate")

    def runtime():
        return PublicRobotRuntime(Backend(), root=tmp_path,
            world=PublicWorldModel(clock_ns=clock, clock_epoch="boot-a"), clock_ns=clock)

    host = runtime()
    status = completed_geometry(clock)
    host.ingest_status(status, runtime_pid=123)
    entities = host.spatial_query(SpatialQuery(require_current=True)).entities
    assert len(entities) == 1 and entities[0].kind == "geometric_area"
    area, fact = entities[0], entities[0].facts[0]
    measured = status["world"]["local_costmap"]["measurement_monotonic_ns"]
    assert fact.observation.measurement_time_ns == measured
    assert fact.observation.sequence == 8 and fact.observation.revision == 7
    assert fact.value["pose_reference_time_ns"] == status["monotonic_ns"]
    assert fact.value["localization_generation"] == 1
    assert fact.value["geometry"]["measurement_time_ns"] == measured
    assert fact.observation.validity_scope.map_revision == 9
    assert {"L3:estimate_reference_time", "L3:generation:1", "L3:transform:4",
            "L4:map:9", "L4:costmap:7", "L4:source_sequence:8"} <= set(fact.observation.lineage)
    assert fact.state is KnowledgeState.LIKELY and fact.value["motion_authority"] is False
    assert not host.spatial_query(SpatialQuery(kind="relations")).relations
    host._persist(force=True)
    restored = runtime()
    restored.ingest_status(status, runtime_pid=123)
    assert not restored.spatial_query(SpatialQuery(require_current=True)).entities
    remembered = restored.spatial_query(SpatialQuery(entity_id=area.entity_id)).entities[0].facts[0]
    assert remembered.freshness == "RESTORED_UNVALIDATED"
    assert remembered.observation == fact.observation
    clock.now += 20_000_000
    restored.ingest_status(completed_geometry(clock, tick=2, sequence=9), runtime_pid=123)
    refreshed = restored.spatial_query(SpatialQuery(require_current=True)).entities[0].facts[0]
    assert refreshed.observation.measurement_time_ns > measured


def test_predicted_pose_and_costmap_maintenance_never_renew_a_scan_or_create_areas():
    clock = Clock()
    world = PublicWorldModel(clock_ns=clock, clock_epoch="boot-a")
    host = PublicRobotRuntime(object(), world=world, clock_ns=clock)
    status = completed_geometry(clock)
    host.ingest_status(status, runtime_pid=123)
    original = host.spatial_query(SpatialQuery()).entities[0].facts[0]
    measured = original.observation.measurement_time_ns
    clock.now += 20_000_000
    predicted = completed_geometry(clock, tick=2, measured=measured, x_m=3.25)
    predicted["world"]["map_revision"] = 10
    predicted["world"]["local_costmap"]["revision"] = 8
    host.ingest_status(predicted, runtime_pid=123)
    entities = host.spatial_query(SpatialQuery()).entities
    assert len(entities) == 1
    assert entities[0].facts[0].observation == original.observation
    assert entities[0].facts[0].freshness == "SCOPE_MISMATCH"
    assert not host.spatial_query(SpatialQuery(require_current=True)).entities
    clock.now += 200_000_000
    host.ingest_status(completed_geometry(clock, tick=3, sequence=9, measured=measured, x_m=3.25), runtime_pid=123)
    assert len(host.spatial_query(SpatialQuery()).entities) == 1


@pytest.mark.parametrize("invalid", ("missing_sequence", "missing_generation", "lost_pose", "discontinuous", "future_measurement"))
def test_incomplete_or_unaligned_geometry_cannot_populate_spatial_coordinates(invalid):
    clock = Clock()
    status = completed_geometry(clock)
    if invalid == "missing_sequence":
        del status["world"]["local_costmap"]["source_sequence"]
    elif invalid == "missing_generation":
        del status["estimate"]["localization_quality"]["generation"]
    elif invalid == "lost_pose":
        status["estimate"]["localization_quality"]["local_translation"] = "LOST"
    elif invalid == "discontinuous":
        status["estimate"]["localization_quality"]["pose_discontinuity"] = True
    else:
        status["world"]["local_costmap"]["measurement_monotonic_ns"] = clock.now + 1
    host = PublicRobotRuntime(object(), world=PublicWorldModel(clock_ns=clock, clock_epoch="boot-a"), clock_ns=clock)
    host.ingest_status(status, runtime_pid=123)
    assert not host.spatial_query(SpatialQuery()).entities


def test_geometry_retains_original_generation_and_fails_closed_after_pose_loss():
    clock = Clock()
    host = PublicRobotRuntime(object(), world=PublicWorldModel(clock_ns=clock, clock_epoch="boot-a"), clock_ns=clock)
    status = completed_geometry(clock)
    host.ingest_status(status, runtime_pid=123)
    assert host.spatial_query(SpatialQuery(require_current=True)).entities
    clock.now += 20_000_000
    degraded = completed_geometry(clock, tick=2)
    degraded["estimate"]["localization_quality"]["heading"] = "LOST"
    host.ingest_status(degraded, runtime_pid=123)
    memory = host.spatial_query(SpatialQuery()).entities[0].facts[0]
    assert memory.freshness == "LOCALIZATION_UNUSABLE"
    assert not host.spatial_query(SpatialQuery(require_current=True)).entities
    clock.now += 20_000_000
    regained = completed_geometry(clock, tick=3, measured=memory.observation.measurement_time_ns)
    host.ingest_status(regained, runtime_pid=123)
    assert not host.spatial_query(SpatialQuery(require_current=True)).entities
    # A delayed public observation cannot borrow the newer context's generation.
    clock.now += 20_000_000
    host.spatial.completed_status(completed_geometry(clock, tick=4, generation=2), runtime_pid=123)
    old = memory.observation
    host.world.observe(old.entity_id, old.attribute, old.value, domain=old.domain,
        measurement_time_ns=clock.now - 10, confidence=old.confidence,
        source=old.source, sequence=9, revision=old.revision,
        validity_scope=old.validity_scope, lineage=old.lineage)
    assert not host.spatial_query(SpatialQuery(require_current=True)).entities


def test_completed_status_geometry_transport_remains_compact_and_roundtrips_lineage():
    clock = Clock()
    small, larger = completed_geometry(clock), completed_geometry(clock, cells=4000)
    assert "occupied_cells" not in larger["world"]["local_costmap"]
    assert len(json.dumps(larger)) - len(json.dumps(small)) < 16
    worlds = []
    for status in (larger, json.loads(json.dumps(larger))):
        host = PublicRobotRuntime(object(), world=PublicWorldModel(clock_ns=clock, clock_epoch="boot-a"), clock_ns=clock)
        host.ingest_status(status, runtime_pid=123)
        worlds.append(host.spatial_query(SpatialQuery()).entities)
    assert worlds[0] == worlds[1]


def test_status_process_preserves_the_same_spatial_measurement_as_direct_projection(tmp_path):
    from v3.process_sidecars import ProcessResidentStatusPublisher
    from v3.resident_status import _tick_status
    clock = Clock()
    tick = geometry_tick(clock, cells=4000)
    path = tmp_path / "status.json"
    publisher = ProcessResidentStatusPublisher(SimpleNamespace(path=path, file_mode=0o600))
    publisher.start()
    try:
        publisher.publish_tick(tick)
        deadline = time.monotonic() + 2
        received = None
        while time.monotonic() < deadline:
            if path.exists():
                received = json.loads(path.read_text())
                if received.get("tick_id") == tick.trace.context.tick_id:
                    break
            time.sleep(.005)
        assert received == _tick_status(tick)
        models = []
        for status in (_tick_status(tick), received):
            host = PublicRobotRuntime(object(), world=PublicWorldModel(clock_ns=clock, clock_epoch="boot-a"), clock_ns=clock)
            host.ingest_status(status, runtime_pid=123)
            models.append(host.spatial_query(SpatialQuery()).entities)
        assert models[0] == models[1]
        assert not publisher.failed and publisher.drop_count == 0
    finally:
        publisher.finish()


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
