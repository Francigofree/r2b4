#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import py_compile
import shutil
from datetime import datetime
from pathlib import Path

PATCH_ID = "R2B4_P0_DUAL_FRAME_TEST_HOTFIX_20260928"
REL = Path("tests/feature/test_er2_navigation_motion.py")
EXPECTED_GIT_BLOB_SHA = "093846400b365f79c46846be711bd3bf459d15ef"

IMPORT_OLD = (
    "from v3.contracts import CommandMode, CommandRequest, DataField, NavigationStatus, "
    "RobotEstimate, RollingLocalCostmap, TickContext, WorldSnapshot"
)
IMPORT_NEW = (
    "from v3.contracts import (CommandMode, CommandRequest, DataField, GLOBAL_FRAME_ID, "
    "LOCAL_FRAME_ID, NavigationStatus, Pose2D, RobotEstimate, RollingLocalCostmap, "
    "TickContext, WorldSnapshot)"
)

ANCHOR = "from v3.layers.l5_command_mission import MissionManager\\n\\n\\n"
HELPER = '''def _dual_frame_estimate(
    context: TickContext,
    x_m: float,
    y_m: float,
    yaw_rad: float,
    v_mps: float = 0.0,
    omega_rad_s: float = 0.0,
) -> RobotEstimate:
    # Synthetic downstream-of-L3 estimate using the current dual-frame contract.
    return RobotEstimate(
        context,
        GLOBAL_FRAME_ID,
        x_m,
        y_m,
        yaw_rad,
        v_mps,
        omega_rad_s,
        (0.0,) * 25,
        local_pose=Pose2D(LOCAL_FRAME_ID, x_m, y_m, yaw_rad),
        global_pose=Pose2D(GLOBAL_FRAME_ID, x_m, y_m, yaw_rad),
        map_to_odom=Pose2D(GLOBAL_FRAME_ID, 0.0, 0.0, 0.0),
        localization_quality=healthy_localization(),
        local_v_mps=v_mps,
        local_omega_rad_s=omega_rad_s,
    )


'''

REPLACEMENTS = (
    (
        '            estimate = RobotEstimate(context, "odom", x, y, yaw, v, omega, (0.0,) * 25, localization_quality=healthy_localization())\\n'
        '            costmap = RollingLocalCostmap("odom", tick, 0.05, 2.5, (), tick, 0)\\n'
        '            world = WorldSnapshot(context, "odom", tick, (), 0, costmap)\\n',
        '            estimate = _dual_frame_estimate(context, x, y, yaw, v, omega)\\n'
        '            costmap = RollingLocalCostmap(LOCAL_FRAME_ID, tick, 0.05, 2.5, (), tick, 0)\\n'
        '            world = WorldSnapshot(context, LOCAL_FRAME_ID, tick, (), 0, costmap)\\n',
    ),
    (
        '    estimate = RobotEstimate(context, "odom", 0, 0, 0, 0, 0, (0.0,) * 25, localization_quality=healthy_localization())\\n'
        '    world = WorldSnapshot(context, "odom", 1, (), 0, RollingLocalCostmap("odom", 1, 0.05, 2.5, (), 1, 0))\\n',
        '    estimate = _dual_frame_estimate(context, 0, 0, 0)\\n'
        '    world = WorldSnapshot(context, LOCAL_FRAME_ID, 1, (), 0, RollingLocalCostmap(LOCAL_FRAME_ID, 1, 0.05, 2.5, (), 1, 0))\\n',
    ),
    (
        '    estimate = replace(estimate, context=context, x_m=0.4)\\n',
        '    estimate = _dual_frame_estimate(context, 0.4, 0.0, 0.0)\\n',
    ),
)

def git_blob_sha(data: bytes) -> str:
    header = f"blob {len(data)}\\0".encode("ascii")
    return hashlib.sha1(header + data).hexdigest()

def patched_text(text: str) -> str:
    if IMPORT_NEW in text and HELPER.strip() in text:
        raise RuntimeError("hotfix already appears to be applied")
    if IMPORT_OLD not in text:
        raise RuntimeError("expected import line not found")
    text = text.replace(IMPORT_OLD, IMPORT_NEW, 1)
    if ANCHOR not in text:
        raise RuntimeError("MissionManager import anchor not found")
    text = text.replace(ANCHOR, ANCHOR + HELPER, 1)
    for old, new in REPLACEMENTS:
        count = text.count(old)
        if count != 1:
            raise RuntimeError(
                f"expected exactly one patch target, found {count}: {old.splitlines()[0]!r}"
            )
        text = text.replace(old, new, 1)
    return text

def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("repo", nargs="?", default="/home/alba/project_r2b4")
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    repo = Path(args.repo).expanduser().resolve()
    path = repo / REL
    if not path.is_file():
        raise SystemExit(f"ERROR: missing {path}")

    original = path.read_bytes()
    actual = git_blob_sha(original)
    if actual != EXPECTED_GIT_BLOB_SHA:
        raise SystemExit(
            "ERROR: source guard failed for "
            f"{REL}\\nexpected={EXPECTED_GIT_BLOB_SHA}\\nactual={actual}\\n"
            "No file was modified."
        )

    new_text = patched_text(original.decode("utf-8"))

    if args.check:
        print(f"{PATCH_ID}: PRECHECK PASS")
        print(f"OK {REL}")
        return 0

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = repo / ".upgrade_backups" / f"p0_dual_frame_test_hotfix_20260928_{stamp}"
    backup_file = backup / REL
    backup_file.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(path, backup_file)

    try:
        path.write_text(new_text, encoding="utf-8")
        py_compile.compile(str(path), doraise=True)
    except Exception:
        shutil.copy2(backup_file, path)
        raise

    print(f"{PATCH_ID}: APPLIED")
    print(f"backup={backup}")
    print(f"CHANGED {REL}")
    print("Next: run verify_hotfix.sh, then `r test full`.")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
