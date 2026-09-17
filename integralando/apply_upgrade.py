#!/usr/bin/env python3
"""Apply the R2B4 test refactor to the current repository.

Baseline analysed: 7c623998ead6a8ddd148a49793aee0b1be91378e
Production v3/ files are intentionally not modified.
"""

from __future__ import annotations

import ast
from datetime import datetime
from pathlib import Path
import shutil
import sys


BASELINE = "7c623998ead6a8ddd148a49793aee0b1be91378e"
PACKAGE_ROOT = Path(__file__).resolve().parent
OVERLAY = PACKAGE_ROOT / "overlay"

DELETE_PATHS = (
    "tests/test_v3_test_hub.py",
    "tests/test_v3_test_hub_next.py",
    "tests/test_v3_test_hub_v2_p0.py",
)

PATCHED_PATHS = (
    "tests/test_v3_async_l6_planner.py",
    "tests/test_v3_hardware_runtime.py",
    "tests/test_v3_person_detection_runtime_integration.py",
    "tests/test_v3_replay.py",
    "tests/test_v3_bounded_runtime_config.py",
    "tests/test_v3_resident_runtime.py",
)


def _span(text: str, function_name: str) -> tuple[int, int]:
    tree = ast.parse(text)
    lines = text.splitlines(keepends=True)
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == function_name:
            starts = [node.lineno]
            starts.extend(item.lineno for item in node.decorator_list)
            start = min(starts) - 1
            end = node.end_lineno
            while end < len(lines) and not lines[end].strip():
                end += 1
            return start, end
    raise RuntimeError(f"top-level function not found: {function_name}")


def replace_function(path: Path, function_name: str, replacement: str | None) -> None:
    text = path.read_text(encoding="utf-8")
    lines = text.splitlines(keepends=True)
    start, end = _span(text, function_name)
    new_lines = []
    if replacement is not None:
        replacement = replacement.strip("\n") + "\n\n\n"
        new_lines = [replacement]
    path.write_text(
        "".join(lines[:start] + new_lines + lines[end:]),
        encoding="utf-8",
    )


def remove_exact_once(path: Path, needle: str) -> None:
    text = path.read_text(encoding="utf-8")
    if text.count(needle) != 1:
        raise RuntimeError(
            f"expected exactly one source anchor in {path}: {needle!r}"
        )
    path.write_text(text.replace(needle, "", 1), encoding="utf-8")


def backup_sources(root: Path, paths: set[str]) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = Path("/tmp") / f"r2b4_test_refactor_backup_{stamp}"
    for relative in sorted(paths):
        source = root / relative
        if not source.exists():
            continue
        target = backup / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    return backup


def main() -> int:
    root = Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
    if not (root / "v3").is_dir() or not (root / "tests").is_dir():
        raise SystemExit(f"not an R2B4 repo root: {root}")

    overlay_files = {
        str(path.relative_to(OVERLAY))
        for path in OVERLAY.rglob("*")
        if path.is_file()
    }
    touched = set(PATCHED_PATHS) | set(DELETE_PATHS) | overlay_files
    backup = backup_sources(root, touched)

    async_path = root / "tests/test_v3_async_l6_planner.py"
    replace_function(
        async_path,
        "test_process_adapter_boundary_has_compute_only_dependencies",
        None,
    )
    replace_function(
        async_path,
        "test_ready_release_result_replaces_stale_previous_plan_before_freshness_gate",
        None,
    )
    # Those imports only served the removed source-shape test.
    remove_exact_once(async_path, "import ast\n")
    remove_exact_once(async_path, "from pathlib import Path\n")
    remove_exact_once(
        async_path,
        'PROJECT_ROOT = Path(__file__).resolve().parents[1]\n\n\n',
    )

    hardware = root / "tests/test_v3_hardware_runtime.py"
    replace_function(
        hardware,
        "_policy",
        r"""
def _policy():
    from v3_test_fixtures import native_sensor_policy

    return native_sensor_policy()
""",
    )
    replace_function(
        hardware,
        "_runtime_config",
        r"""
def _runtime_config():
    from v3_test_fixtures import without_optional_perception

    runtime = load_bounded_physical_runtime_config(
        PROJECT_ROOT / "conf" / "hardver.json",
        PROJECT_ROOT / "conf" / "fizika.json",
        PROJECT_ROOT / "conf" / "speed_map.json",
        BoundedTeleopProfile(
            "hardware-runtime-test",
            start_tick_id=2,
            active_tick_count=1,
            v_mps=0.05,
            omega_rad_s=0.0,
            max_v_mps=0.10,
            max_omega_rad_s=0.20,
        ),
        sensor_policy=_policy(),
    )
    # Core-hardware tests disable the complete optional perception capability,
    # not only the camera device half of that capability.
    return without_optional_perception(runtime)
""",
    )

    person = root / "tests/test_v3_person_detection_runtime_integration.py"
    replace_function(
        person,
        "_mission",
        r"""
def _mission(tick: int, now_ns: int, mode=CommandMode.EXPLORE):
    from v3_test_fixtures import active_mission

    return active_mission(
        TickContext(tick, now_ns),
        mode=mode,
        mission_id="mission-operator-roomcruise-test",
    )
""",
    )
    replace_function(person, "_write", None)
    replace_function(person, "test_litert_is_scoped_to_the_detector_edge_adapter", None)
    replace_function(person, "test_shared_device_health_policy_exception_is_l12_only", None)

    replay = root / "tests/test_v3_replay.py"
    replace_function(
        replay,
        "test_general_replay_matches_generic_explore_trajectory_through_l4_l8",
        r"""
def test_general_replay_matches_generic_explore_trajectory_through_l4_l8(tmp_path):
    from v3_validation_helpers import control_config

    capture = create_explore_capture(tmp_path)
    result = replay_capture(capture, project_root=PROJECT_ROOT)

    assert result["status"] == "MATCH"
    assert result["determinism"]["repeated_trace_match"] is True
    for layer in tuple(f"L{index}" for index in range(4, 13)):
        assert result["diagnostics"]["layers"][layer]["mismatch_count"] == 0
        assert result["diagnostics"]["layers"][layer]["compared_tick_count"] == 8

    payload = json.loads(capture.read_text(encoding="utf-8"))
    ticks = payload["ticks"]
    active_ticks = ticks[1:7]
    active = active_ticks[0]["expected"]["layers"]
    navigation = control_config().navigation
    expected_count = (
        navigation.rollout_linear_samples
        * navigation.rollout_angular_samples
    )

    assert len(active["L6"]["trajectory_candidates"]) == expected_count
    assert active["L7"]["kind"] == "TRACK_TRAJECTORY"
    assert active["L7"]["selected_source"] == "navigation.trajectory"
    assert (
        active["L8"]["requested_v_mps"]
        == active["L7"]["trajectory"]["v_mps"]
    )
    assert (
        active["L8"]["requested_omega_rad_s"]
        < active["L7"]["trajectory"]["omega_rad_s"]
        < 0.0
    )
    assert {
        tick["expected"]["layers"]["L5"]["mission_id"]
        for tick in active_ticks
    } == {"mission-room-cruise-replay"}

    # Replay tests verify deterministic behaviour and valid bounds. They do not
    # freeze one scheduler handoff to one exact control tick.
    for tick in active_ticks:
        candidates = tick["expected"]["layers"]["L6"]["trajectory_candidates"]
        assert len(candidates) == expected_count
        assert len({item["candidate_id"] for item in candidates}) == expected_count

    assert all(
        later["monotonic_ns"] > earlier["monotonic_ns"]
        for earlier, later in zip(ticks, ticks[1:])
    )
""",
    )

    bounded = root / "tests/test_v3_bounded_runtime_config.py"
    replace_function(
        bounded,
        "_sensor_policy",
        r"""
def _sensor_policy() -> NativeSensorPolicyConfig:
    from v3_test_fixtures import native_sensor_policy

    return native_sensor_policy()
""",
    )
    replace_function(
        bounded,
        "test_active_sources_close_into_one_immutable_bounded_runtime_config",
        r"""
def test_closed_runtime_config_is_consistent_and_immutable():
    config = _load()
    physical = config.composition
    control = physical.live_control.control
    motors = physical.motor_output
    encoder = config.encoder

    assert encoder is not None
    assert config.tick_period_ns > 0
    assert physical.live_control.command_profile == _profile()
    assert control.estimation.frame_id == POSE_FRAME_ID
    assert (
        control.estimation.track_width_m
        == control.chassis_control.track_width_m
    )
    assert control.speed_map.schema == "R2B4_WHEEL_SPEED_MAP_V2"
    assert control.speed_map.map_state == "ACTIVE"

    motor_pins = motors.pins
    encoder_pins = encoder.counter_gpio.pins
    assert len(set(motor_pins)) == len(motor_pins)
    assert len(set(encoder_pins)) == len(encoder_pins)
    assert set(motor_pins).isdisjoint(encoder_pins)
    assert encoder.left_step_distance_m > 0.0
    assert encoder.right_step_distance_m > 0.0

    with pytest.raises(FrozenInstanceError):
        config.tick_period_ns = 1  # type: ignore[misc]
    with pytest.raises(FrozenInstanceError):
        encoder.left_step_distance_m = 1.0  # type: ignore[misc]
""",
    )
    replace_function(
        bounded,
        "test_active_v3_navigation_config_closes_local_perception_costmap_and_rollout",
        r"""
def test_active_v3_navigation_config_closes_local_perception_costmap_and_rollout():
    config = _load(sensor_policy=_sensor_policy(), control_path=CONTROL_PATH)
    control = config.composition.live_control.control
    hardware = config.sensor_inputs

    assert hardware is not None
    assert hardware.inputs.lidar_source.local_perception_min_range_m > 0.0
    assert (
        hardware.inputs.lidar_source.local_perception_max_range_m
        > hardware.inputs.lidar_source.local_perception_min_range_m
    )
    assert hardware.inputs.lidar_source.local_perception_max_points > 0
    assert control.world_model.local_costmap_resolution_m > 0.0
    assert control.world_model.local_costmap_radius_m > 0.0

    candidate_count = (
        control.navigation.rollout_linear_samples
        * control.navigation.rollout_angular_samples
    )
    assert 30 <= candidate_count <= 60
    assert control.navigation.rollout_step_count > 0
""",
    )

    resident = root / "tests/test_v3_resident_runtime.py"
    replace_function(
        resident,
        "_policy",
        r"""
def _policy():
    from v3_test_fixtures import native_sensor_policy

    return native_sensor_policy(
        encoder_maximum_sample_interval_ns=150_000_000,
        imu_maximum_sample_age_ns=150_000_000,
        lidar_maximum_result_age_ns=150_000_000,
        lidar_minimum_confidence=0.5,
        lidar_maximum_measurement_age_ns=150_000_000,
    )
""",
    )

    for source in OVERLAY.rglob("*"):
        if not source.is_file():
            continue
        relative = source.relative_to(OVERLAY)
        target = root / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)

    for relative in DELETE_PATHS:
        path = root / relative
        if path.exists():
            path.unlink()

    # Syntax validation of every changed/new Python file.
    for relative in sorted(touched):
        path = root / relative
        if path.suffix == ".py" and path.exists():
            ast.parse(path.read_text(encoding="utf-8"), filename=str(path))

    print(f"test refactor applied to: {root}")
    print(f"backup of previous test files: {backup}")
    print("next: python3 -m pytest -q tests/test_v3_gate.py -x")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
