#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shutil
import subprocess
from pathlib import Path

EXPECTED_BASE_L6_BLOB = "2cef8fc14895b49511c481b3f0db4ad88f0d5e03"


def git_blob(path: Path) -> str:
    return subprocess.check_output(["git", "hash-object", str(path)], text=True).strip()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Apply the R2B4 L6 P0 planning-scene optimization package."
    )
    parser.add_argument(
        "repo",
        nargs="?",
        default="/home/alba/project_r2b4",
        help="R2B4 repository root",
    )
    args = parser.parse_args()

    package_root = Path(__file__).resolve().parent
    repo = Path(args.repo).resolve()
    target_l6 = repo / "v3/layers/l6_navigation.py"
    target_test = repo / "tests/test_v3_l6_planning_scene.py"
    source_l6 = package_root / "v3/layers/l6_navigation.py"
    source_test = package_root / "tests/test_v3_l6_planning_scene.py"

    if not (repo / ".git").exists():
        raise SystemExit(f"not a git repository: {repo}")
    if not target_l6.exists():
        raise SystemExit(f"missing target: {target_l6}")

    current_blob = git_blob(target_l6)
    package_blob = git_blob(source_l6)
    if current_blob == package_blob:
        print("L6 source already matches this package.")
    elif current_blob != EXPECTED_BASE_L6_BLOB:
        raise SystemExit(
            "REFUSED: v3/layers/l6_navigation.py differs from the inspected baseline.\n"
            f"current blob:  {current_blob}\n"
            f"expected blob: {EXPECTED_BASE_L6_BLOB}\n"
            "Review/preserve the local changes first, then apply manually if appropriate."
        )
    else:
        shutil.copy2(source_l6, target_l6)
        print("updated: v3/layers/l6_navigation.py")

    if target_test.exists() and git_blob(target_test) != git_blob(source_test):
        raise SystemExit(
            "REFUSED: tests/test_v3_l6_planning_scene.py already exists with different content."
        )
    if not target_test.exists():
        target_test.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source_test, target_test)
        print("added: tests/test_v3_l6_planning_scene.py")

    print("\nRecommended validation:")
    print(
        "python -m pytest -q "
        "tests/test_v3_navigation_trajectory.py "
        "tests/test_v3_l6_planning_scene.py "
        "tests/test_v3_l5_l9_mission_navigation.py"
    )
    print("\nReview:")
    print(
        "git diff -- v3/layers/l6_navigation.py "
        "tests/test_v3_l6_planning_scene.py"
    )
    print("No commit or push was performed by this installer.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
