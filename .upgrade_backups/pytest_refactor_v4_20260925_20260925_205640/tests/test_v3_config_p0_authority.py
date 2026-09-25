from __future__ import annotations

from copy import deepcopy
import ast
import json
from pathlib import Path

import pytest

from v3.config import ConfigResolver
from v3.contracts import TrajectoryEvaluation, TrajectoryPose
from v3.layers import l7_motion_selection as l7
from v3.replay import _decode_production_value, _migrate_legacy_resolved_config_snapshot


ROOT = (Path(__import__("os").environ["R2B4_ROOT"]).resolve() if __import__("os").environ.get("R2B4_ROOT") else next((p for p in Path(__file__).resolve().parents if (p / "conf" / "hardver.json").is_file() and (p / "v3").is_dir()), Path.cwd()))


def _documents():
    conf = ROOT / "conf"
    return tuple(
        json.loads((conf / name).read_text(encoding="utf-8"))
        for name in ("hardver.json", "fizika.json", "speed_map.json", "vezerles.json")
    )


def _candidate(candidate_id: str, omega_rad_s: float) -> TrajectoryEvaluation:
    horizon_ns = 100_000_000
    return TrajectoryEvaluation(
        candidate_id=candidate_id,
        v_mps=0.20,
        omega_rad_s=omega_rad_s,
        horizon_ns=horizon_ns,
        samples=(TrajectoryPose(0.02, 0.0, 0.0, horizon_ns),),
        collision=False,
        min_clearance_m=0.50,
        progress_score=0.50,
        smoothness_score=0.50,
        novelty_score=0.50,
        total_score=1.0,
    )


def test_camera_freshness_is_explicit_resolved_authority():
    hardware, physics, speed_map, control = _documents()
    expected = control["sensor_policy"]["camera_maximum_frame_age_ns"]
    resolved = ConfigResolver.from_documents(hardware, physics, speed_map, control)
    camera = resolved.runtime.sensor_inputs.inputs.camera_source
    assert camera is not None
    assert camera.maximum_frame_age_ns == expected == 250_000_000


def test_camera_freshness_missing_fails_closed():
    hardware, physics, speed_map, control = _documents()
    control = deepcopy(control)
    control["sensor_policy"].pop("camera_maximum_frame_age_ns")
    with pytest.raises(ValueError, match="camera_maximum_frame_age_ns"):
        ConfigResolver.from_documents(hardware, physics, speed_map, control)


def test_l7_reversal_threshold_is_configured_and_behavioral():
    hardware, physics, speed_map, control = _documents()
    resolved = ConfigResolver.from_documents(hardware, physics, speed_map, control)
    config = resolved.runtime.composition.live_control.control.motion_selection
    assert config.reversal_min_omega_rad_s == 0.05

    state = l7.MotionSelectionStateCheckpoint(
        last_mission_id="m",
        last_candidate_id="previous",
        last_v_mps=0.20,
        last_omega_rad_s=0.10,
    )
    reverse = _candidate("reverse", -0.08)
    assert l7._command_continuity_key(state, reverse, 0.05)[0] == 1
    assert l7._command_continuity_key(state, reverse, 0.20)[0] == 0


def test_l7_reversal_threshold_conflict_fails_closed():
    hardware, physics, speed_map, control = _documents()
    control = deepcopy(control)
    control["layers"]["motion_selection"]["reversal_min_omega_rad_s"] = (
        control["layers"]["operational_constraints"]["max_omega_rad_s"] + 0.1
    )
    with pytest.raises(ValueError, match="reversal threshold exceeds operational omega limit"):
        ConfigResolver.from_documents(hardware, physics, speed_map, control)


def test_l7_old_capture_field_shape_decodes_with_historical_value():
    encoded = {
        "__type__": "MotionSelectionConfig",
        "continuity_score_band": 0.005,
    }
    migrated = _migrate_legacy_resolved_config_snapshot(encoded)
    decoded = _decode_production_value(migrated, l7.MotionSelectionConfig, "legacy.motion_selection")
    assert decoded.reversal_min_omega_rad_s == 0.05


def test_measurement_tool_has_one_resolved_config_authority():
    path = ROOT / "tools" / "v3_sensor_measurement.py"
    source = path.read_text(encoding="utf-8")
    tree = ast.parse(source, filename=str(path))
    imported = set()
    function_names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module)
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            function_names.add(node.name)
    assert "v3.config" in imported
    assert "v3_bounded_config" not in imported
    assert "load_native_lidar_port_config" not in source
    assert "load_bounded_physical_runtime_config" not in source
    assert "native_sensor_policy" not in function_names
    assert "ConfigResolver.for_project(PROJECT_ROOT).resolve()" in source
    assert "_open_lidar_port(resolved.lidar, pose_provider)" in source
    assert 'summary["config_snapshot_id"] = resolved.snapshot_id' in source


def test_replay_hashes_captured_snapshot_before_schema_migration():
    source = (ROOT / "v3" / "replay.py").read_text(encoding="utf-8")
    assert "json.dumps(\n                robot_value," in source
    assert 'configuration.get("snapshot_id") != encoded_snapshot_id' in source
    assert 'decode_value = _migrate_legacy_resolved_config_snapshot(robot_value)' in source
    assert 'configuration.get("snapshot_id") != robot.snapshot_id' not in source
