"""Offline map views preserve measurement-time pose and missing evidence."""
from copy import deepcopy
import csv
import json
import math
import struct
from xml.etree import ElementTree

import pytest

from tools import atlas2d
from tools.mcap_evidence.compiler import compile_evidence
from tools.mcap_evidence.reader import EvidenceBundle, EvidenceViewRow
from tools.mcap_evidence.verify import verify
from v3.mcap_writer import StdlibMcapWriter
from v3.contracts import LocalizationQuality, QualityState, TickContext
from v3.local_costmap_capture import PackedLocalCostmap, unpack_local_costmap


def _tick(tick, stamp, x=0., y=0., yaw=0., generation=0, frame="R2B4_ODOM_LOCAL"):
    context = {"tick_id": tick, "monotonic_ns": stamp}
    return {"tick_id": tick, "monotonic_ns": stamp, "inputs": {"context": context},
            "expected": {"layers": {"L3": {"context": {"$ref": "input_context"},
                "local_pose": {"frame_id": frame, "x_m": x, "y_m": y, "yaw_rad": yaw},
                "localization_quality": {"local_translation": "GOOD", "heading": "GOOD",
                    "local_pose_continuous": True, "pose_discontinuity": False, "generation": generation}}}}}


def _scan(stamp=1_100_000_000, sequence=2, points=None):
    return {"source_topic": "v3.navigation_map", "payload": {
        "schema": "R2B4_NAVIGATION_MAP_V1", "event_type": "navigation_map_observation",
        "measurement_monotonic_ns": stamp, "scan_start_monotonic_ns": stamp - 40_000_000,
        "scan_end_monotonic_ns": stamp + 40_000_000, "source_sequence": sequence,
        "frame_id": "ROBOT_BASE", "points_xy_m": [[1., 0.]] if points is None else points,
        "source_point_count": 100, "minimum_range_m": .1, "maximum_range_m": 12.,
        "lidar_x_offset_m": -.005, "lidar_y_offset_m": 0., "sensor_health": "OK", "motion_authority": False}}


def _grid(tick=0, stamp=1_000_000_000, cells=((-3, 4, 2), (1, -2, 5))):
    quality = LocalizationQuality(local_translation=QualityState.GOOD, heading=QualityState.GOOD,
                                  global_position=QualityState.GOOD, local_pose_continuous=True)
    packed = PackedLocalCostmap(TickContext(tick, stamp), True, None, "R2B4_ODOM_LOCAL", 7, 10,
        stamp - 100_000_000, 100_000_000, .1, 6., 0, quality, len(cells),
        struct.pack(f"<{len(cells) * 3}i", *(n for cell in cells for n in cell)))
    return {"source_topic": "v3.local_costmap", "payload": unpack_local_costmap(packed)}


class Bundle:
    def __init__(self, tmp_path, ticks, events=()):
        self.root = tmp_path / "sealed.evidence"
        self.verification = {"status": "PASS", "source_checked": True}
        self.manifest = {"source": {"path": "/evidence/input.mcap", "sha256": "a" * 64},
                         "compiler_status": "COMPLETE", "source_integrity": "COMPLETE"}
        self.rows = {"ticks": [EvidenceViewRow("ticks", "tick" + str(i), "", "/r2b4/tick",
                tick["monotonic_ns"], tick["tick_id"], tick) for i, tick in enumerate(ticks)],
            "events": [EvidenceViewRow("events", "event" + str(i), "", "/r2b4/event",
                event["payload"].get("measurement_monotonic_ns", event["payload"].get("monotonic_ns")),
                None, event) for i, event in enumerate(events)]}

    def iter_view(self, view):
        return iter(self.rows.get(view, ()))

    def has_view(self, view):
        return bool(self.rows.get(view))


def test_measurement_time_projection_interpolates_yaw_and_does_not_reapply_extrinsic(tmp_path):
    ticks = [_tick(0, 1_000_000_000, 0., 0., math.radians(170)),
             _tick(10, 1_200_000_000, 2., 0., math.radians(-170))]
    panels, lineage, quality, _ = atlas2d.collect(Bundle(tmp_path, ticks, [_scan()]))
    panel = panels[("R2B4_ODOM_LOCAL", 0)]
    # Halfway pose is (1, 0, pi), so the already axle-relative point (1, 0)
    # becomes (0, 0). Applying the LiDAR offset again would move this return.
    assert panel["returns"][0] == pytest.approx((0., 0.), abs=1e-12)
    assert lineage[0]["bracket_tick_ids"] == [0, 10]
    assert lineage[0]["bracket_message_ids"] == ["tick0", "tick1"]
    assert lineage[0]["weight"] == .5
    assert quality["projected_return_points"] == 1


@pytest.mark.parametrize("change,reason", [
    ("generation", "pose_continuity_boundary"),
    ("frame", "pose_continuity_boundary"),
    ("lost", "unqualified_pose_bracket"),
    ("discontinuous", "unqualified_pose_bracket"),
    ("gap", "pose_continuity_boundary"),
])
def test_map_never_crosses_unqualified_pose_or_continuity_boundary(tmp_path, change, reason):
    ticks = [_tick(0, 1_000_000_000), _tick(10, 1_200_000_000)]
    l3 = ticks[1]["expected"]["layers"]["L3"]
    if change == "generation":
        l3["localization_quality"]["generation"] = 1
    elif change == "frame":
        l3["local_pose"]["frame_id"] = "OTHER_LOCAL_FRAME"
    elif change == "lost":
        l3["localization_quality"]["heading"] = "LOST"
    elif change == "discontinuous":
        l3["localization_quality"]["pose_discontinuity"] = True
    else:
        ticks[1] = _tick(10, 1_400_000_000)
    panels, lineage, quality, _ = atlas2d.collect(Bundle(tmp_path, ticks, [_scan()]))
    assert not any(panel["returns"] for panel in panels.values())
    assert lineage[0]["status"] == reason
    assert quality[reason] == 1


def test_invalid_pose_row_breaks_path_and_map_bracket_instead_of_being_skipped_over(tmp_path):
    ticks = [_tick(0, 1_000_000_000), _tick(1, 1_100_000_000), _tick(2, 1_200_000_000)]
    ticks[1]["expected"]["layers"]["L3"]["context"] = {"tick_id": 100, "monotonic_ns": 5}
    panels, lineage, quality, _ = atlas2d.collect(Bundle(tmp_path, ticks, [_scan()]))
    poses = panels[("R2B4_ODOM_LOCAL", 0)]["poses"]
    assert len(poses) == 2 and poses[0].segment != poses[1].segment
    assert quality["invalid_pose_rows"] == 1
    assert lineage[0]["status"] == "pose_continuity_boundary"


def test_no_nearest_pose_fallback_and_no_future_or_global_alignment(tmp_path):
    ticks = [_tick(0, 1_000_000_000), _tick(1, 1_200_000_000)]
    scans = [_scan(900_000_000), _scan(1_300_000_000)]
    panels, lineage, quality, _ = atlas2d.collect(Bundle(tmp_path, ticks, scans))
    assert quality["outside_recorded_pose_interval"] == 2
    assert not any(panel["returns"] for panel in panels.values())
    assert all(record["status"] == "outside_recorded_pose_interval" for record in lineage)


@pytest.mark.parametrize("field,value", [
    ("frame_id", "R2B4_BOOT_ROBOT_MAP"), ("motion_authority", True),
    ("points_xy_m", [[float("nan"), 0.]]), ("points_xy_m", [[1., 0.]] * 97),
    ("scan_start_monotonic_ns", 2_000_000_000), ("schema", "unknown"),
])
def test_invalid_or_oversized_map_observation_is_explicit(tmp_path, field, value):
    scan = _scan()
    scan["payload"][field] = value
    _, lineage, quality, _ = atlas2d.collect(Bundle(tmp_path,
        [_tick(0, 1_000_000_000), _tick(1, 1_200_000_000)], [scan]))
    assert lineage[0]["status"] == "invalid_map_observation"
    assert quality["invalid_map_observation"] == 1


@pytest.mark.parametrize("health", ("STALE", "ERROR"))
def test_unhealthy_returns_are_not_projected(tmp_path, health):
    scan = _scan()
    scan["payload"]["sensor_health"] = health
    _, lineage, quality, _ = atlas2d.collect(Bundle(tmp_path,
        [_tick(0, 1_000_000_000), _tick(1, 1_200_000_000)], [scan]))
    assert lineage[0]["status"] == "sensor_not_ok"
    assert quality["sensor_not_ok"] == 1


def test_latest_scan_uses_measurement_time_and_duplicate_is_not_counted_twice(tmp_path):
    scans = [_scan(1_150_000_000, 3, [[2., 0.]]), _scan(), deepcopy(_scan())]
    panels, _, quality, _ = atlas2d.collect(Bundle(tmp_path,
        [_tick(0, 1_000_000_000), _tick(1, 1_200_000_000)], scans))
    panel = panels[("R2B4_ODOM_LOCAL", 0)]
    assert panel["latest_returns"] == [(2., 0.)]
    assert panel["map_samples"] == 2 and len(panel["returns"]) == 2
    assert quality["duplicate_map_observation"] == 1


def test_real_mcap_compiler_verified_bundle_and_portable_artifacts(tmp_path):
    capture = tmp_path / "behavior5hz.mcap"
    with capture.open("wb") as stream:
        writer = StdlibMcapWriter(stream)
        writer.start()
        tick_channel = writer.register_channel("/r2b4/tick")
        event_channel = writer.register_channel("/r2b4/event")
        for tick, stamp in ((0, 1_000_000_000), (10, 1_200_000_000)):
            writer.add_message(tick_channel, log_time_ns=stamp, sequence=tick,
                               data=json.dumps(_tick(tick, stamp, x=(tick / 10))).encode())
        writer.add_message(event_channel, log_time_ns=1_140_000_000, sequence=1,
                           data=json.dumps(_scan()).encode())
        finalized = {"event_type": "capture_finalized", "capture_hz": 5, "status": "PASS",
                     "integrity": {"sample_complete": True, "raw_evidence_requested": False,
                        "replay_complete": False, "navigation_map_requested": True,
                        "navigation_map_hz": 5, "navigation_map_complete": True,
                        "navigation_map_integrity_reasons": [], "navigation_map_transport": {
                            "received_count": 1, "produced_count": 1, "superseded_count": 0, "end": True}}}
        writer.add_message(event_channel, log_time_ns=1_300_000_000, sequence=2,
                           data=json.dumps(finalized).encode())
        writer.finish()
    compiled = compile_evidence(capture, tmp_path / "capture.evidence", workers=1)
    assert verify(compiled["output"], source=capture)["status"] == "PASS"
    capture.unlink()  # Renderer has no reason to reopen MCAP or load devices.
    output = tmp_path / "view"
    result = atlas2d.render(EvidenceBundle.open(compiled["output"]), output)
    assert result["status"] == "PASS"
    report = json.loads((output / "report.json").read_text())
    assert report["quality"]["projected_map_samples"] == 1
    assert report["map_lineage"][0]["weight"] == .5
    assert report["motion_authority"] is False
    assert report["planner_occupancy"] == "NOT_RECONSTRUCTED"
    assert report["source"]["sha256"]
    assert report["map_lineage"][0]["bracket_tick_ids"] == [0, 10]
    assert report["capture_scope"]["navigation_map_complete"] is True
    assert report["capture_scope"]["raw_evidence_requested"] is False
    assert report["capture_scope"]["replay_complete"] is False
    assert report["capture_scope"]["navigation_map_transport"]["received_count"] == 1
    ElementTree.parse(output / "atlas2d.svg")
    with (output / "path.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert [row["tick_id"] for row in rows] == ["0", "10"]
    assert [float(row["x_m"]) for row in rows] == [0., 1.]


def test_old_behavior_capture_has_path_and_explicit_obstacle_absence(tmp_path):
    bundle = Bundle(tmp_path, [_tick(0, 1_000_000_000, frame="local<&"),
                                _tick(1, 1_200_000_000, x=.1, frame="local<&")])
    output = tmp_path / "view"
    result = atlas2d.render(bundle, output)
    assert result["warnings"] == ["OBSTACLE_GEOMETRY_NOT_RECORDED_OR_NOT_PROJECTABLE"]
    svg = (output / "atlas2d.svg").read_text()
    assert "Obstacle geometry was not recorded" in svg and "local&lt;&amp;" in svg
    assert "polyline" in svg
    ElementTree.parse(output / "atlas2d.svg")
    with pytest.raises(FileExistsError):
        atlas2d.render(bundle, output)


def test_cli_verification_failure_creates_no_output(monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(atlas2d.EvidenceBundle, "open", lambda *a: (_ for _ in ()).throw(ValueError("bad hash")))
    assert atlas2d.main(["input.evidence", "--output-dir", str(tmp_path / "view")]) == 1
    assert not (tmp_path / "view").exists()
    assert "bad hash" in capsys.readouterr().err


def test_output_cannot_modify_sealed_bundle_through_parent_symlink(tmp_path):
    bundle = Bundle(tmp_path, [_tick(0, 1_000_000_000)])
    bundle.root.mkdir()
    alias = tmp_path / "alias"
    alias.symlink_to(bundle.root, target_is_directory=True)
    with pytest.raises(ValueError, match="outside the sealed"):
        atlas2d.render(bundle, alias / "new-view")
    assert not (bundle.root / "new-view").exists()


def test_planner_hold_markers_describe_sampled_runs_without_inventing_physical_stop(tmp_path):
    ticks = [_tick(i, 1_000_000_000 + i * 200_000_000, x=.1 * i) for i in range(6)]
    for i, tick in enumerate(ticks):
        layers = tick["expected"]["layers"]
        layers["L5"] = {"lifecycle": "ACTIVE"}
        layers["L6"] = {"status": "IDLE" if i in (1, 2, 3, 5) else "ACTIVE",
                        "reason": "PLANNER_EXECUTION_HOLD" if i in (1, 2, 3, 5) else None}
    output = tmp_path / "view"
    atlas2d.render(Bundle(tmp_path, ticks), output)
    report = json.loads((output / "report.json").read_text())
    holds = report["sampled_planner_holds"]
    assert len(holds) == 1  # An active sample splits the run; one sample has no span.
    assert holds[0]["tick_id"] == 1 and holds[0]["end_tick_id"] == 3
    assert holds[0]["sample_count"] == 3 and holds[0]["sampled_span_s"] == .4
    assert holds[0]["capture_elapsed_s"] == .2
    assert holds[0]["physical_stop_inferred"] is False
    assert "intervening unsampled motion remains possible" in (output / "atlas2d.svg").read_text()
    ElementTree.parse(output / "atlas2d.svg")


def _html_data(output):
    html = (output / "atlas2d.html").read_text()
    return json.loads(html.split('<script type="application/json" id="data">', 1)[1].split('</script>', 1)[0])


def test_actual_grid_timeline_keeps_exact_cells_and_matches_same_tick_state(tmp_path):
    ticks = [_tick(0, 1_000_000_000), _tick(10, 1_200_000_000, x=.2)]
    layers = ticks[1]["expected"]["layers"]
    layers.update(L5={"lifecycle": "ACTIVE"}, L6={"status": "IDLE", "reason": "PLANNER_EXECUTION_HOLD"},
                  L8={"requested_v_mps": .19, "requested_omega_rad_s": .3},
                  L9={"allowed_v_mps": 0., "allowed_omega_rad_s": 0.})
    layers["L3"]["localization_quality"].update(accumulated_translation_bias_warning=True,
                                               encoder_lidar_consistency_m=.21)
    grid0, grid1 = _grid(), _grid(10, 1_200_000_000)
    grid1["payload"]["revision"] = 8
    output = tmp_path / "view"
    atlas2d.render(Bundle(tmp_path, ticks, [grid0, grid1]), output)
    data = _html_data(output)
    assert data["actual_grid_recorded"] is True
    assert data["cell_sets"] == [[[-3, 4, 2], [1, -2, 5]]]
    assert data["counts"]["aligned_snapshots"] == 2
    assert data["counts"]["unique_cell_sets"] == 1
    assert [s["revision"] for s in data["snapshots"]] == [7, 8]
    selected = data["states"][data["snapshots"][1]["state_index"]]
    assert selected["tick_id"] == 10 and selected["monotonic_ns"] == 1_200_000_000
    assert selected["pose"] == [.2, 0., 0.]
    assert selected["requested"] == [.19, .3] and selected["allowed"] == [0., 0.]
    assert selected["plan_reason"] == "PLANNER_EXECUTION_HOLD"
    assert selected["localization_quality"]["accumulated_translation_bias_warning"] is True
    report = json.loads((output / "report.json").read_text())
    assert report["planner_occupancy"] == "RECORDED_L4_SNAPSHOTS"
    assert report["local_costmap_timeline"]["recorded_snapshots"] == 2


@pytest.mark.parametrize("mismatch", ("tick", "time", "frame", "generation"))
def test_grid_without_exact_state_does_not_borrow_nearby_pose_or_reason(tmp_path, mismatch):
    grid = _grid(tick=1 if mismatch == "tick" else 0,
                 stamp=1_010_000_000 if mismatch == "time" else 1_000_000_000)
    if mismatch == "frame":
        grid["payload"]["frame_id"] = "OTHER_LOCAL_FRAME"
    if mismatch == "generation":
        grid["payload"]["generation"] = 1
        grid["payload"]["localization_quality"]["generation"] = 1
    bundle = Bundle(tmp_path, [_tick(0, 1_000_000_000), _tick(10, 1_200_000_000)], [grid])
    panels, _, _, _ = atlas2d.collect(bundle)
    timeline = atlas2d._timeline(bundle, panels)
    assert timeline["snapshots"][0]["state_index"] is None
    assert timeline["snapshots"][0]["available"] is True  # Actual grid remains visible.
    assert timeline["counts"]["unaligned_snapshots"] == 1


def test_old_capture_html_explicitly_lacks_grid_and_embedded_labels_cannot_execute(tmp_path):
    bundle = Bundle(tmp_path, [_tick(0, 1_000_000_000), _tick(10, 1_200_000_000)])
    bundle.manifest["source"]["path"] = '</script><script>alert("bad")</script>'
    output = tmp_path / "view"
    atlas2d.render(bundle, output)
    data = _html_data(output)
    assert data["actual_grid_recorded"] is False and data["cell_sets"] == []
    assert all(s["availability_reason"] == "NOT_RECORDED" for s in data["snapshots"])
    html = (output / "atlas2d.html").read_text()
    assert 'Actual L4 grid was not recorded' in html
    assert '</script><script>alert' not in html
    assert data["source"]["path"] == bundle.manifest["source"]["path"]


def test_native_mcap_evi_actual_grid_is_portable_in_standalone_html(tmp_path):
    capture = tmp_path / "actual-grid.mcap"
    with capture.open("wb") as stream:
        writer = StdlibMcapWriter(stream)
        writer.start()
        tick_channel = writer.register_channel("/r2b4/tick")
        event_channel = writer.register_channel("/r2b4/event")
        for tick, stamp in ((0, 1_000_000_000), (10, 1_200_000_000)):
            writer.add_message(tick_channel, log_time_ns=stamp, sequence=tick,
                               data=json.dumps(_tick(tick, stamp)).encode())
            writer.add_message(event_channel, log_time_ns=stamp, sequence=tick,
                               data=json.dumps(_grid(tick, stamp)).encode())
        writer.finish()
    compiled = compile_evidence(capture, tmp_path / "actual-grid.evidence", workers=1)
    assert verify(compiled["output"], source=capture)["status"] == "PASS"
    capture.unlink()
    output = tmp_path / "view"
    atlas2d.render(EvidenceBundle.open(compiled["output"]), output)
    data = _html_data(output)
    assert [s["tick_id"] for s in data["snapshots"]] == [0, 10]
    assert data["cell_sets"] == [[[-3, 4, 2], [1, -2, 5]]]
    assert all(data["states"][s["state_index"]]["tick_id"] == s["tick_id"] for s in data["snapshots"])
