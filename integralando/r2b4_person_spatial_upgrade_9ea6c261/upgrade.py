#!/usr/bin/env python3
"""Apply the R2B4 person spatial tracking upgrade for main@1ba63c27.

This package is intentionally narrow. It preserves the newer production
person-detector wiring and roomcruise photo-evidence path already present in
this baseline, and adds only the spatial camera+LiDAR tracking capability.
"""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
from pathlib import Path


BASELINE = "1ba63c27f44b787386ee96cc7ed6c70299b9524c"
PACKAGE_ROOT = Path(__file__).resolve().parent
PAYLOAD = PACKAGE_ROOT / "payload"


def _require_anchor(path: Path, anchor: str) -> None:
    text = path.read_text(encoding="utf-8")
    if anchor not in text:
        raise RuntimeError(f"upgrade baseline mismatch: expected source anchor missing in {path}")


def _replace_once(path: Path, old: str, new: str) -> None:
    text = path.read_text(encoding="utf-8")
    count = text.count(old)
    if count != 1:
        raise RuntimeError(
            f"upgrade baseline mismatch: expected exactly one source anchor in {path}, got {count}"
        )
    path.write_text(text.replace(old, new, 1), encoding="utf-8")


def _copy_payload(repo: Path, relative: str) -> None:
    source = PAYLOAD / relative
    destination = repo / relative
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)


def _git_head(repo: Path) -> str | None:
    try:
        result = subprocess.run(
            ["git", "-C", str(repo), "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return None
    return result.stdout.strip()


def main() -> int:
    repo = Path(sys.argv[1] if len(sys.argv) > 1 else ".").expanduser().resolve()

    # Fail before touching source when this is clearly a different committed baseline.
    head = _git_head(repo)
    if head is not None and head != BASELINE:
        raise RuntimeError(
            "upgrade was built for R2B4 main@1ba63c27; "
            f"current HEAD is {head[:12]}"
        )

    live_person = repo / "v3/adapters/live_person_detection.py"
    world_model = repo / "v3/layers/l4_world_model.py"
    native_control = repo / "v3/composition/native_control.py"
    control_path = repo / "conf/vezerles.json"

    # These checks also protect non-git copies and uncommitted source edits.
    _require_anchor(live_person, "Full multi-person boxes remain available on PersonDetectionPort")
    _require_anchor(world_model, '"""L4 deterministic shadow world state and the empty STOP-only path."""')
    _require_anchor(
        native_control,
        '    rolling = _mapping(root.get("rolling_costmap"), "v3_navigation.rolling_costmap")\n'
        '    exploration = _mapping(root.get("exploration"), "v3_navigation.exploration")\n',
    )

    control = json.loads(control_path.read_text(encoding="utf-8"))
    navigation = control.get("v3_navigation")
    if not isinstance(navigation, dict):
        raise RuntimeError("conf/vezerles.json has no v3_navigation object")
    if "person_tracking" in navigation:
        raise RuntimeError("v3_navigation.person_tracking already exists; refusing to overwrite it")

    backup = repo.parent / f"{repo.name}_backup_person_spatial_9ea6c261"
    if backup.exists():
        shutil.rmtree(backup)
    backup.mkdir(parents=True, exist_ok=True)

    touched = (
        "v3/adapters/live_person_detection.py",
        "v3/layers/l4_world_model.py",
        "v3/composition/native_control.py",
        "conf/vezerles.json",
    )
    for relative in touched:
        source = repo / relative
        target = backup / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)

    _copy_payload(repo, "v3/adapters/live_person_detection.py")
    _copy_payload(repo, "v3/layers/l4_world_model.py")

    _replace_once(
        native_control,
        '    rolling = _mapping(root.get("rolling_costmap"), "v3_navigation.rolling_costmap")\n'
        '    exploration = _mapping(root.get("exploration"), "v3_navigation.exploration")\n',
        '    rolling = _mapping(root.get("rolling_costmap"), "v3_navigation.rolling_costmap")\n'
        '    person_tracking_value = root.get("person_tracking")\n'
        '    person_tracking = (\n'
        '        {}\n'
        '        if person_tracking_value is None\n'
        '        else _mapping(person_tracking_value, "v3_navigation.person_tracking")\n'
        '    )\n'
        '    person_tracking_enabled = person_tracking.get("enabled", True)\n'
        '    if type(person_tracking_enabled) is not bool:\n'
        '        raise ValueError("v3_navigation.person_tracking.enabled must be bool")\n'
        '    exploration = _mapping(root.get("exploration"), "v3_navigation.exploration")\n',
    )
    _replace_once(
        native_control,
        '        local_costmap_max_points_per_scan=local_max_points,\n'
        '    )\n'
        '    navigation = NavigationConfig(\n',
        '        local_costmap_max_points_per_scan=local_max_points,\n'
        '        person_tracking_enabled=person_tracking_enabled,\n'
        '        person_camera_horizontal_fov_rad=_positive_float(\n'
        '            person_tracking.get("camera_horizontal_fov_rad", 1.1519173063162575),\n'
        '            "v3_navigation.person_tracking.camera_horizontal_fov_rad",\n'
        '        ),\n'
        '        person_camera_yaw_offset_rad=_finite_float(\n'
        '            person_tracking.get("camera_yaw_offset_rad", 0.0),\n'
        '            "v3_navigation.person_tracking.camera_yaw_offset_rad",\n'
        '        ),\n'
        '        person_lidar_max_skew_ns=_positive_int(\n'
        '            person_tracking.get("lidar_max_skew_ns", 150_000_000),\n'
        '            "v3_navigation.person_tracking.lidar_max_skew_ns",\n'
        '        ),\n'
        '        person_lidar_angular_margin_rad=_finite_float(\n'
        '            person_tracking.get("lidar_angular_margin_rad", 0.04),\n'
        '            "v3_navigation.person_tracking.lidar_angular_margin_rad",\n'
        '        ),\n'
        '        person_lidar_cluster_depth_m=_positive_float(\n'
        '            person_tracking.get("lidar_cluster_depth_m", 0.30),\n'
        '            "v3_navigation.person_tracking.lidar_cluster_depth_m",\n'
        '        ),\n'
        '        person_lidar_min_points=_positive_int(\n'
        '            person_tracking.get("lidar_min_points", 1),\n'
        '            "v3_navigation.person_tracking.lidar_min_points",\n'
        '        ),\n'
        '        person_track_max_association_distance_m=_positive_float(\n'
        '            person_tracking.get("max_association_distance_m", 0.75),\n'
        '            "v3_navigation.person_tracking.max_association_distance_m",\n'
        '        ),\n'
        '        person_track_max_speed_mps=_positive_float(\n'
        '            person_tracking.get("max_speed_mps", 6.0),\n'
        '            "v3_navigation.person_tracking.max_speed_mps",\n'
        '        ),\n'
        '        person_track_radius_m=_positive_float(\n'
        '            person_tracking.get("track_radius_m", 0.30),\n'
        '            "v3_navigation.person_tracking.track_radius_m",\n'
        '        ),\n'
        '    )\n'
        '    navigation = NavigationConfig(\n',
    )

    navigation["person_tracking"] = {
        "enabled": True,
        "camera_horizontal_fov_rad": 1.1519173063162575,
        "camera_yaw_offset_rad": 0.0,
        "lidar_max_skew_ns": 150000000,
        "lidar_angular_margin_rad": 0.04,
        "lidar_cluster_depth_m": 0.30,
        "lidar_min_points": 1,
        "max_association_distance_m": 0.75,
        "max_speed_mps": 6.0,
        "track_radius_m": 0.30,
    }
    control_path.write_text(
        json.dumps(control, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    test_source = PACKAGE_ROOT / "tests/test_v3_person_spatial_tracking.py"
    test_target = repo / "tests/test_v3_person_spatial_tracking.py"
    test_target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(test_source, test_target)

    print("R2B4 person spatial tracking upgrade applied.")
    print(f"baseline: {BASELINE}")
    print(f"backup: {backup}")
    print("preserved: native sensor wiring, LiteRT runtime config, photo evidence")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
