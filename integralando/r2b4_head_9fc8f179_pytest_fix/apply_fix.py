#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import subprocess
import tempfile
from pathlib import Path

EXPECTED_HEAD = "9fc8f179b5ed5ed5d0bf28eb0c4305fffe8edaea"

AFFECTED_FILES = (
    "v3/import_guard.py",
    "tests/test_v3_architecture_boundaries.py",
    "tests/test_v3_resident_runtime.py",
    "tests/test_v3_differential_drive_arcs.py",
    "tests/test_v3_sensor_measurement_tool.py",
)


def fail(message: str):
    raise SystemExit(f"ERROR: {message}")


def git(repo: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(repo), *args],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if result.returncode != 0:
        fail(result.stderr.strip() or f"git {' '.join(args)} failed")
    return result.stdout.strip()


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        fail(f"{label}: expected exactly one source anchor, found {count}")
    return text.replace(old, new, 1)


def function_block(text: str, name: str) -> tuple[int, int, str]:
    marker = f"def {name}("
    start = text.find(marker)
    if start < 0:
        fail(f"function not found: {name}")
    next_def = text.find("\ndef ", start + len(marker))
    next_async = text.find("\nasync def ", start + len(marker))
    candidates = [p for p in (next_def, next_async) if p >= 0]
    end = min(candidates) + 1 if candidates else len(text)
    return start, end, text[start:end]


def replace_in_function(
    text: str, name: str, old: str, new: str, label: str
) -> str:
    start, end, block = function_block(text, name)
    changed = replace_once(block, old, new, label)
    return text[:start] + changed + text[end:]


def write_atomic(path: Path, text: str) -> None:
    mode = path.stat().st_mode
    fd, temp_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="") as handle:
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(temp_name, mode)
        os.replace(temp_name, path)
    finally:
        if os.path.exists(temp_name):
            os.unlink(temp_name)


def patch_import_guard(text: str) -> str:
    text = replace_once(
        text,
        '''APPROVED_THIRD_PARTY_ROOTS: frozenset[str] = frozenset({"numpy", "scipy"})
STDLIB_ROOTS = frozenset(sys.stdlib_module_names) | frozenset({"__future__"})
LAYER_PREFIX = "v3.layers."
''',
        '''APPROVED_THIRD_PARTY_ROOTS: frozenset[str] = frozenset({"numpy", "scipy"})
STDLIB_ROOTS = frozenset(sys.stdlib_module_names) | frozenset({"__future__"})
LAYER_PREFIX = "v3.layers."

# Concrete hardware/media libraries are not deterministic production
# dependencies. They are allowed only at the named camera edge adapters;
# the same imports remain forbidden everywhere else in V3.
EDGE_THIRD_PARTY_ROOTS: dict[str, frozenset[str]] = {
    "v3.adapters.picamera2_camera": frozenset({"libcamera", "picamera2"}),
    "v3.adapters.camera_media": frozenset({"picamera2"}),
}
''',
        "import guard camera edge dependency policy",
    )
    text = replace_once(
        text,
        '''    if root not in {"v3"} | set(STDLIB_ROOTS) | set(approved_third_party):
        violations.append(
''',
        '''    authorized_edge_roots = EDGE_THIRD_PARTY_ROOTS.get(importer, frozenset())
    if (
        root not in {"v3"} | set(STDLIB_ROOTS) | set(approved_third_party)
        and root not in authorized_edge_roots
    ):
        violations.append(
''',
        "import guard scoped edge allowance",
    )
    return text


def patch_architecture_tests(text: str) -> str:
    text = replace_once(
        text,
        '''def test_v3_only_import_policy_declares_its_math_dependencies():
    assert APPROVED_THIRD_PARTY_ROOTS == frozenset({"numpy", "scipy"})
''',
        '''def test_v3_only_import_policy_declares_its_math_dependencies():
    assert APPROVED_THIRD_PARTY_ROOTS == frozenset({"numpy", "scipy"})


def test_camera_hardware_dependencies_are_scoped_to_camera_edge_adapters(tmp_path):
    _write(
        tmp_path,
        "v3/adapters/picamera2_camera.py",
        "from picamera2 import Picamera2\\nfrom libcamera import controls\\n",
    )
    _write(
        tmp_path,
        "v3/adapters/camera_media.py",
        "from picamera2.encoders import H264Encoder\\n"
        "from picamera2.outputs import FileOutput\\n",
    )

    assert validate_v3_imports(tmp_path) == ()

    _write(
        tmp_path,
        "v3/layers/l4_world_model/illegal_camera_dependency.py",
        "from picamera2 import Picamera2\\n",
    )
    violations = validate_v3_imports(tmp_path)

    assert any(
        item.path == "v3/layers/l4_world_model/illegal_camera_dependency.py"
        and item.code == "PROJECT_OR_THIRD_PARTY_IMPORT_NOT_ALLOWED"
        and item.imported_module == "picamera2.Picamera2"
        for item in violations
    )
''',
        "architecture test for scoped camera edge dependencies",
    )

    text = replace_in_function(
        text,
        "test_bounded_live_control_has_no_physical_output_or_runtime_authority",
        '''        "v3.contracts",
        "v3.engine",
''',
        '''        "v3.contracts",
        "v3.device_health_policy",
        "v3.engine",
''',
        "bounded live control import whitelist",
    )

    text = replace_in_function(
        text,
        "test_bounded_physical_control_has_no_concrete_gpio_or_runtime_authority",
        '''        "v3.adapters.live_lidar",
        "v3.contracts",
''',
        '''        "v3.adapters.live_lidar",
        "v3.adapters.live_inputs",
        "v3.contracts",
''',
        "bounded physical control import whitelist",
    )

    text = replace_in_function(
        text,
        "test_bounded_physical_runtime_has_no_entrypoint_or_external_io_authority",
        '''        "v3.adapters.live_lidar",
        "v3.composition.bounded_physical_control",
''',
        '''        "v3.adapters.live_lidar",
        "v3.adapters.live_inputs",
        "v3.composition.bounded_physical_control",
''',
        "bounded runtime import whitelist",
    )

    text = replace_in_function(
        text,
        "test_bounded_runtime_config_loader_has_no_hardware_authority",
        '''        "v3.adapters.live_lidar",
        "v3.adapters.motor_pwm",
''',
        '''        "v3.adapters.live_lidar",
        "v3.adapters.live_camera",
        "v3.adapters.picamera2_camera",
        "v3.adapters.motor_pwm",
''',
        "bounded config camera import whitelist",
    )
    text = replace_in_function(
        text,
        "test_bounded_runtime_config_loader_has_no_hardware_authority",
        '''        "v3.composition.native_sensor_inputs",
        "v3.layers.l3_state_estimation",
''',
        '''        "v3.composition.native_sensor_inputs",
        "v3.device_health_policy",
        "v3.layers.l3_state_estimation",
''',
        "bounded config health-policy import whitelist",
    )
    return text


def patch_resident_runtime_tests(text: str) -> str:
    text = replace_in_function(
        text,
        "_sources",
        'NativeEncoderConfig("encoder", 0.5)',
        'NativeEncoderConfig("WHEEL_ENCODERS", 0.5)',
        "resident test encoder production identity",
    )
    text = replace_in_function(
        text,
        "_sources",
        'NativeImuConfig("imu", 0.5, 2)',
        'NativeImuConfig("BNO055_IMU", 0.5, 2)',
        "resident test imu production identity",
    )
    return text


def patch_arc_replay_test(text: str) -> str:
    text = replace_once(
        text,
        '''from v3.engine import TickInputs
from v3.execution import ExecutionRecord
''',
        '''from v3.device_health_policy import PRODUCTION_CRITICAL_DEVICE_IDS
from v3.engine import TickInputs
from v3.execution import ExecutionRecord
''',
        "arc replay production critical-device import",
    )
    text = replace_in_function(
        text,
        "_control_config",
        '''        lidar_safety=LidarSafetyConfig(
            "RPLIDAR_C1",
            float(hardware["lidar"]["biztonsagi_zona_m"]),
        ),
    )
''',
        '''        lidar_safety=LidarSafetyConfig(
            "RPLIDAR_C1",
            float(hardware["lidar"]["biztonsagi_zona_m"]),
        ),
        critical_device_ids=PRODUCTION_CRITICAL_DEVICE_IDS,
    )
''',
        "arc replay production critical-device policy",
    )
    text = replace_in_function(
        text,
        "_raw",
        '''            "LIDAR_LOCALIZATION",
            "lidar_health",
''',
        '''            "RPLIDAR_C1",
            "lidar_health",
''',
        "arc replay lidar sample production identity",
    )
    text = replace_in_function(
        text,
        "_raw",
        '''            DeviceHealth("LIDAR_LOCALIZATION", DeviceHealthState.OK),
''',
        '''            DeviceHealth("RPLIDAR_C1", DeviceHealthState.OK),
''',
        "arc replay lidar health production identity",
    )
    return text


def patch_sensor_measurement_test(text: str) -> str:
    text = replace_in_function(
        text,
        "test_summary_exposes_health_ranges_estimate_and_zero_commit",
        'NativeEncoderConfig("encoder", 0.5)',
        'NativeEncoderConfig("WHEEL_ENCODERS", 0.5)',
        "measurement test encoder production identity",
    )
    text = replace_in_function(
        text,
        "test_summary_exposes_health_ranges_estimate_and_zero_commit",
        'NativeImuConfig("imu", 0.5, 2)',
        'NativeImuConfig("BNO055_IMU", 0.5, 2)',
        "measurement test imu production identity",
    )
    text = replace_in_function(
        text,
        "test_summary_exposes_health_ranges_estimate_and_zero_commit",
        'NativeLidarConfig("lidar", 0.2, 100)',
        'NativeLidarConfig("RPLIDAR_C1", 0.2, 100)',
        "measurement test lidar production identity",
    )
    text = replace_in_function(
        text,
        "test_summary_exposes_health_ranges_estimate_and_zero_commit",
        '''    assert summary["source_state_counts"] == {
        "encoder": {"OK": 1},
        "imu": {"OK": 1},
        "lidar": {"OK": 1},
    }
''',
        '''    assert summary["source_state_counts"] == {
        "WHEEL_ENCODERS": {"OK": 1},
        "BNO055_IMU": {"OK": 1},
        "RPLIDAR_C1": {"OK": 1},
    }
''',
        "measurement test expected production identities",
    )
    return text


PATCHERS = {
    "v3/import_guard.py": patch_import_guard,
    "tests/test_v3_architecture_boundaries.py": patch_architecture_tests,
    "tests/test_v3_resident_runtime.py": patch_resident_runtime_tests,
    "tests/test_v3_differential_drive_arcs.py": patch_arc_replay_test,
    "tests/test_v3_sensor_measurement_tool.py": patch_sensor_measurement_test,
}


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Apply the source-first R2B4 pytest synchronization fix."
    )
    parser.add_argument(
        "repo",
        nargs="?",
        default=".",
        help="R2B4 repository root (default: current directory)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="validate every anchor but do not write files",
    )
    args = parser.parse_args()

    repo = Path(args.repo).expanduser().resolve()
    if not (repo / ".git").exists():
        fail(f"not a Git working tree: {repo}")

    head = git(repo, "rev-parse", "HEAD")
    if head != EXPECTED_HEAD:
        fail(
            f"HEAD mismatch: expected {EXPECTED_HEAD}, got {head}. "
            "Re-audit before applying this package."
        )

    dirty = {
        line[3:]
        for line in git(repo, "status", "--short", "--untracked-files=all").splitlines()
        if len(line) >= 4
    }
    overlap = sorted(set(AFFECTED_FILES) & dirty)
    if overlap:
        fail(
            "affected files have local changes; refusing to overwrite: "
            + ", ".join(overlap)
        )

    originals = {}
    patched = {}
    for relative, patcher in PATCHERS.items():
        path = repo / relative
        if not path.is_file():
            fail(f"missing expected file: {relative}")
        original = path.read_text(encoding="utf-8")
        changed = patcher(original)
        if changed == original:
            fail(f"no change produced for {relative}")
        originals[relative] = original
        patched[relative] = changed

    if args.dry_run:
        print("DRY-RUN PASS")
        for relative in AFFECTED_FILES:
            print(relative)
        return 0

    written = []
    try:
        for relative in AFFECTED_FILES:
            write_atomic(repo / relative, patched[relative])
            written.append(relative)
    except BaseException:
        for relative in reversed(written):
            try:
                write_atomic(repo / relative, originals[relative])
            except Exception:
                pass
        raise

    print("APPLIED")
    print(f"base_head={EXPECTED_HEAD}")
    for relative in AFFECTED_FILES:
        print(relative)
    print()
    print("Recommended validation:")
    print(
        "python3 -m pytest -q "
        "tests/test_v3_architecture_boundaries.py "
        "tests/test_v3_resident_runtime.py "
        "tests/test_v3_differential_drive_arcs.py "
        "tests/test_v3_sensor_measurement_tool.py"
    )
    print("python3 -m pytest -q")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
