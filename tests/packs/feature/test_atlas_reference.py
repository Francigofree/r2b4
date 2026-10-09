"""Synthetic offline atlas evidence remains distinct from current localization."""
import importlib.util
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from r2b4_orchestration.atlas_reference import load_atlas_reference
from r2b4_orchestration.spatial_service import SpatialQuery, SpatialQueryResult, SpatialService
from r2b4_orchestration.world_model import PublicWorldModel


@pytest.fixture
def builder():
    spec = importlib.util.spec_from_file_location("atlas_test_builder", Path(__file__).resolve().parents[3] / "tools/mcap50-to-map.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def exported(builder, tmp_path, *, count=2):
    points = np.array([[1., 0.], [1., 1.], [0., 1.]])
    ticks = ({"tick_id": 20, "sequence": 20, "reference_time_ns": 100},
             {"tick_id": 21, "sequence": 21, "reference_time_ns": 120})
    scans = ({"revision": 8, "sequence": 9, "measurement_time_ns": 110,
              "start_time_ns": 105, "end_time_ns": 115, "clock_epoch": "source-epoch",
              "pose_reference_ticks": list(ticks)},)
    keyframes = [builder.Keyframe(index, 110 + index, 50, 180, 3, (8. + index, 9., .2),
                 points, 4, 4, 3, 2, .01, .02, .8, ticks, scans, "R2B4_ODOM_LOCAL", "source-epoch")
                 for index in range(count)]
    poses = [(float(index), .1 * index, .01 * index) for index in range(count)]
    occupancy = {"image": np.array([[0, 254], [205, 205]], dtype=np.uint8),
                 "probability": np.ones((2, 2)) * .5, "logodds": np.zeros((2, 2)),
                 "observed": np.array([[1, 1], [0, 0]]), "origin": (0., 0., 0.), "resolution": .05}
    class Reader:
        def iter_json_messages(self, *, topics):
            assert topics == (builder.RUNTIME_TOPIC,)
            yield None, {"configuration": {"snapshot_id": "capture-config", "resolved_robot": {"lidar": "captured"}},
                         "metadata": {"runtime": "captured-runtime", "runtime_pid": 42, "session_id": "source-session"}}

    report = {"schema": "R2B4_GLOBAL_MAP_BUILD_V2", "input": {"sha256": "a" * 64,
              "capture_id": "source-capture", "clock_epoch": "source-epoch"},
              "build_identity": builder._build_identity(Reader())}
    output = tmp_path / "synthetic-atlas"
    builder._write_outputs(output, tmp_path / "synthetic.mcap", keyframes, poses, [], occupancy, report)
    return output


def test_export_closes_original_gauge_build_identity_and_measurement_references(builder, tmp_path):
    output = exported(builder, tmp_path)
    source = json.loads((output / "atlas_reference.json").read_text())
    assert source["source_gauge"]["original_pose"] == [8., 9., .2]
    assert source["source_gauge"]["trajectory_relationship"] == "PER_KEYFRAME_OPTIMIZED_NONRIGID"
    assert source["keyframes"][0]["original_pose"] == [8., 9., .2]
    assert source["keyframes"][0]["normalized_seed_pose"] == pytest.approx([0., 0., 0.])
    assert source["keyframes"][1]["optimized_pose"] == [1., .1, .01]
    scan = source["keyframes"][0]["source_scans"][0]
    assert (scan["revision"], scan["sequence"], scan["measurement_time_ns"]) == (8, 9, 110)
    assert [row["tick_id"] for row in scan["pose_reference_ticks"]] == [20, 21]
    assert source["build"]["tool"]["sha256"] == builder._sha256(Path(builder.__file__))
    assert source["build"]["capture_configuration_snapshot_id"] == "capture-config"
    assert source["build"]["runtime_pid"] == 42 and source["build"]["session_id"] == "source-session"
    assert source["build"]["numeric_options"]["MAP_RESOLUTION_M"] == .05
    assert source["build"]["calibration"]["extrinsic_measurement_verified"] is False
    assert source["motion_authority"] is False and source["current_alignment"] is None
    reference = load_atlas_reference(output)
    assert reference.map_id == source["map_id"]
    assert reference.viewpoints[0].source_frame_id == "R2B4_ODOM_LOCAL"
    assert reference.viewpoints[0].source_clock_epoch == "source-epoch"
    assert reference.viewpoints[0].source_scan_revision == 8
    assert reference.viewpoints[0].source_scan_sequence == 9
    assert reference.viewpoints[0].source_tick_ids == (20, 21)


def test_recorded_pose_and_scan_keep_original_clock_sequence_and_interpolation_sources(builder):
    class Reader:
        channels_by_topic = {topic: SimpleNamespace(metadata={"clock_epoch": "capture-epoch"})
                             for topic in (builder.RAW_TOPIC, builder.TICK_TOPIC)}
        def iter_json_messages(self, *, topics):
            if topics == (builder.TICK_TOPIC,):
                for tick, stamp in ((7, 100), (8, 120)):
                    yield SimpleNamespace(sequence=tick), {"tick_id": tick, "monotonic_ns": stamp,
                        "expected": {"layers": {"L3": {"local_pose": {
                            "frame_id": "original-local", "x_m": 5., "y_m": 6., "yaw_rad": .4},
                            "localization_quality": {"generation": 3}}}}}
            else:
                yield SimpleNamespace(sequence=22), {"revision": 19, "scan_start_monotonic_ns": 105,
                    "measurement_monotonic_ns": 110, "scan_end_monotonic_ns": 115,
                    "source_point_count": 10, "points": [[angle, 1., 20.] for angle in range(10)]}
    poses = builder._extract_pose_samples(Reader())
    interpolated = builder._interp_pose(poses, [item.t_ns for item in poses], 110)
    assert interpolated.frame_id == "original-local" and interpolated.clock_epoch == "capture-epoch"
    assert interpolated.generation_recorded
    assert tuple(item["tick_id"] for item in interpolated.source_ticks) == (7, 8)
    scan = builder._extract_raw_scans(Reader())[0]
    assert (scan.t_ns, scan.sequence, scan.revision, scan.clock_epoch) == (110, 22, 19, "capture-epoch")


def test_import_bounded_viewpoints_public_teaching_and_restore_have_no_current_motion(builder, tmp_path, monkeypatch):
    output = exported(builder, tmp_path, count=70)
    monkeypatch.setattr(np, "load", lambda *args, **kwargs: pytest.fail("host must not decode NPZ"))
    world = PublicWorldModel(clock_ns=lambda: 1_000, clock_epoch="current-epoch")
    spatial = SpatialService(world, clock_ns=lambda: 1_000)
    original_world = world.snapshot().to_jsonable()
    atlas = spatial.load_atlas_reference(output)
    assert world.snapshot().to_jsonable() == original_world
    assert len(atlas.viewpoints) == 64 and atlas.viewpoint_count == 70
    assert not spatial.query(SpatialQuery(kind="relations")).relations
    assert not spatial.query(SpatialQuery(kind="entities")).entities
    assert not spatial.query(SpatialQuery(kind="viewpoints", require_current=True)).viewpoints
    world.observe("room:kitchen", "atlas_viewpoint", {"map_id": atlas.map_id, "viewpoint_id": "keyframe:1"},
                  domain="room_topology", measurement_time_ns=900, source="user", confidence=1.)
    points = spatial.query(SpatialQuery(kind="viewpoints", entity_id="room:kitchen"))
    assert len(points.viewpoints) == 1 and points.viewpoints[0].place_ids == ("room:kitchen",)
    assert points.viewpoints[0].measurement_time_ns == 111
    assert SpatialQueryResult.from_jsonable(points.to_jsonable()).to_jsonable() == points.to_jsonable()
    atlases = spatial.query(SpatialQuery(kind="atlases"))
    assert SpatialQueryResult.from_jsonable(atlases.to_jsonable()).to_jsonable() == atlases.to_jsonable()
    restored = SpatialService(world, clock_ns=lambda: 1_000)
    restored.restore(spatial.export_state())
    assert restored.query(SpatialQuery(kind="viewpoints", entity_id="room:kitchen")).viewpoints == points.viewpoints
    assert not restored.query(SpatialQuery(kind="viewpoints", require_current=True)).viewpoints
    for field, invalid in (("frame_id", "R2B4_BOOT_ROBOT_MAP"), ("viewpoint_id", "keyframe:999")):
        corrupted = json.loads(json.dumps(spatial.export_state()))
        corrupted["atlas_references"][0]["viewpoints"][0][field] = invalid
        if field == "frame_id":
            corrupted["atlas_references"][0]["reference"][field] = invalid
            for point in corrupted["atlas_references"][0]["viewpoints"]:
                point[field] = invalid
        with pytest.raises(ValueError, match="identity mismatch"):
            restored.restore(corrupted)
        assert restored.query(SpatialQuery(kind="viewpoints", entity_id="room:kitchen")).viewpoints == points.viewpoints
    payload = json.dumps(restored.snapshot())
    assert "probability" not in payload and "logodds" not in payload and "source_scans" not in payload
    assert "map_data.npz" in payload and len(payload.encode()) < 30_000


def test_changed_manifest_cannot_replace_existing_reference(builder, tmp_path):
    output = exported(builder, tmp_path)
    service = SpatialService(PublicWorldModel())
    atlas = service.load_atlas_reference(output)
    path = output / "atlas_reference.json"
    content = json.loads(path.read_text())
    content["keyframes"][0]["optimized_pose"][0] += 100
    path.write_text(json.dumps(content))
    with pytest.raises(ValueError, match="identity"):
        service.load_atlas_reference(output)
    assert service.query(SpatialQuery(kind="atlases")).atlas_references == (atlas,)


def test_same_size_asset_corruption_fails_explicit_import(builder, tmp_path):
    output = exported(builder, tmp_path)
    asset = output / "map_data.npz"
    data = bytearray(asset.read_bytes())
    data[-1] ^= 1
    asset.write_bytes(data)
    with pytest.raises(ValueError, match="sha256 mismatch"):
        load_atlas_reference(output)


def test_legacy_map_is_historical_geometry_without_invented_source_gauge(tmp_path):
    (tmp_path / "report.json").write_text(json.dumps({"schema": "R2B4_GLOBAL_MAP_BUILD_V1", "input": {"sha256": "b" * 64}}))
    (tmp_path / "keyframes.csv").write_text("index,ref_ns,generation,opt_x,opt_y,opt_yaw\n0,100,3,0,0,0\n")
    before = (tmp_path / "report.json").read_bytes()
    reference = load_atlas_reference(tmp_path)
    assert reference.provenance == "LEGACY_SOURCE_GAUGE_UNAVAILABLE"
    assert reference.viewpoints[0].original_pose is None
    assert reference.viewpoints[0].source_frame_id is None
    assert reference.to_jsonable()["current_alignment"] is None
    assert (tmp_path / "report.json").read_bytes() == before


def test_host_import_and_place_teaching_persist_without_motion_or_frame_relabelling(builder, tmp_path):
    from r2b4_orchestration.local_task_planner import LocalTaskPlanner
    from r2b4_orchestration.robot_runtime import PublicRobotRuntime
    class NoHardware:
        def __init__(self):
            self.actions = []
        def capabilities(self):
            return {"capabilities": {"v3.command.navigate": {
                "supported": True, "available": True, "ready": True}}}
        def read(self, resource):
            raise RuntimeError("runtime unavailable")
        def execute(self, action, **parameters):
            self.actions.append(action)
            raise AssertionError("historical atlas cannot execute motion")
        def stop(self):
            pass
    output = exported(builder, tmp_path)
    backend = NoHardware()
    owner = PublicRobotRuntime(backend, root=tmp_path)
    loaded = owner.execute("spatial.load_atlas", {"path": str(output)})
    assert loaded["status"] == "LOADED" and loaded["durability"] == "SAVED"
    teaching = {"entity_id": "room:lounge", "name": "nappali", "map_id": loaded["map_id"],
                "viewpoint_id": "keyframe:1", "request_id": "explicit-place-teaching"}
    taught = owner.execute("spatial.teach_place", teaching)
    assert taught["status"] == "TAUGHT" and taught["motion_authority"] is False
    assert taught["durability"] == "SAVED"
    query = SpatialQuery(kind="viewpoints", entity_id="room:lounge")
    point = owner.spatial_query(query).viewpoints[0]
    assert point.frame_id.startswith("R2B4_ATLAS:") and point.place_ids == ("room:lounge",)
    assert owner.world.read("room:lounge", "location").observation is None
    restored = PublicRobotRuntime(backend, root=tmp_path)
    assert restored.spatial_query(query).viewpoints == (point,)
    proposal = LocalTaskPlanner().resolve("Menj a nappaliba", restored).plan
    assert proposal is not None
    pending = restored.brain.submit("Menj a nappaliba")
    result = restored.brain.adopt(pending["goal_id"], proposal)
    assert result["lifecycle"] == "FAILED"
    assert backend.actions == []
    with pytest.raises(ValueError, match="REQUEST_CONFLICT"):
        owner.execute("spatial.teach_place", {**teaching, "name": "konyha"})
    newer = {**teaching, "viewpoint_id": "keyframe:0", "request_id": "later-place-teaching"}
    owner.execute("spatial.teach_place", newer)
    restored = PublicRobotRuntime(backend, root=tmp_path)
    before = restored.world.read("room:lounge", "atlas_place")
    revision = restored.world.revision
    replayed = restored.execute("spatial.teach_place", teaching)
    assert replayed["duplicate"] is True and replayed["superseded"] is True
    after = restored.world.read("room:lounge", "atlas_place")
    assert after.value["viewpoint_id"] == "keyframe:0"
    assert after.observation == before.observation and restored.world.revision == revision
    with pytest.raises(ValueError, match="REQUEST_CONFLICT"):
        restored.execute("spatial.teach_place", {**teaching, "entity_id": "room:kitchen"})
    assert backend.actions == []
