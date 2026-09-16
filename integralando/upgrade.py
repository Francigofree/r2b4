#!/usr/bin/env python3
"""Install the R2B4 async-L6 P0 handoff freshness fix.

Built against fe197fb6a9a5e0f63960871b9c9529231f76f12e.
Only two files are touched:
- v3/layers/l6_navigation.py
- tests/test_v3_async_l6_planner.py
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

BASE_COMMIT = 'fe197fb6a9a5e0f63960871b9c9529231f76f12e'
SOURCE_REL = Path("v3/layers/l6_navigation.py")
TEST_REL = Path("tests/test_v3_async_l6_planner.py")
MARKER = "test_ready_release_result_replaces_stale_previous_plan_before_freshness_gate"

OLD_BLOCK = '        if (\n            self._last_replan_ns is not None\n            and context.monotonic_ns - self._last_replan_ns > self._max_plan_age_ns\n        ):\n            raise RuntimeError("ASYNC_L6_PLAN_STALE")\n        if context.tick_id < release_tick_id:\n            return False\n        if context.tick_id > release_tick_id:\n            raise RuntimeError("ASYNC_L6_RELEASE_TICK_MISSED")\n        result = backend.take(request_id)\n        if result is None:\n            raise RuntimeError("ASYNC_L6_DEADLINE_MISSED")\n        if result.source_context != source_context:\n            raise RuntimeError("ASYNC_L6_SOURCE_CONTEXT_MISMATCH")\n'
NEW_BLOCK = '        if context.tick_id < release_tick_id:\n            # Before the deterministic handoff tick, the previously accepted\n            # plan is still authoritative, so its age remains safety-critical.\n            if (\n                self._last_replan_ns is not None\n                and context.monotonic_ns - self._last_replan_ns > self._max_plan_age_ns\n            ):\n                raise RuntimeError("ASYNC_L6_PLAN_STALE")\n            return False\n        if context.tick_id > release_tick_id:\n            raise RuntimeError("ASYNC_L6_RELEASE_TICK_MISSED")\n\n        # At the handoff tick, inspect the new result before judging plan age.\n        # The previous plan may have crossed max_plan_age_ns while the new\n        # rollout is already ready and fresh enough to replace it.\n        result = backend.take(request_id)\n        if result is None:\n            raise RuntimeError("ASYNC_L6_DEADLINE_MISSED")\n        if result.source_context != source_context:\n            raise RuntimeError("ASYNC_L6_SOURCE_CONTEXT_MISMATCH")\n        if (\n            context.monotonic_ns - result.source_context.monotonic_ns\n            > self._max_plan_age_ns\n        ):\n            raise RuntimeError("ASYNC_L6_PLAN_STALE")\n'
TEST_APPEND = '\n\ndef test_ready_release_result_replaces_stale_previous_plan_before_freshness_gate():\n    """Regression for the 2026-09-16 live tick-115 false PLAN_STALE fault."""\n\n    config = NavigationConfig()\n    backend = InlineTrajectoryRolloutBackend(config)\n    navigator = TrajectoryNavigator(\n        config,\n        rollout_backend=backend,\n        rollout_release_tick_gap=5,\n        # Tick 10 is 200 ms after the synchronous seed at tick 0, but only\n        # 100 ms after the pending request snapshot at tick 5.\n        max_plan_age_ns=150_000_000,\n    )\n    manager = MissionManager()\n\n    plans = [_evaluate(navigator, manager, tick_id) for tick_id in range(10)]\n    previous_candidates = plans[-1].trajectory_candidates\n\n    # The old accepted plan is stale here, but the ready result sourced at\n    # tick 5 is still fresh. The handoff must accept the new result first.\n    released = _evaluate(navigator, manager, 10)\n\n    assert released.trajectory_candidates != previous_candidates\n    checkpoint = navigator.checkpoint()\n    assert checkpoint.last_replan_tick_id == 5\n    assert checkpoint.pending_rollout_request is not None\n    assert checkpoint.pending_rollout_request.context.tick_id == 10\n    assert checkpoint.pending_release_tick_id == 15\n\n    backend.close()\n\n\ndef test_previous_plan_staleness_still_fails_closed_before_release_tick():\n    """The P0 fix must not weaken pre-release fail-closed freshness."""\n\n    config = NavigationConfig()\n    backend = InlineTrajectoryRolloutBackend(config)\n    navigator = TrajectoryNavigator(\n        config,\n        rollout_backend=backend,\n        rollout_release_tick_gap=5,\n        max_plan_age_ns=100_000_000,\n    )\n    manager = MissionManager()\n\n    for tick_id in range(6):\n        _evaluate(navigator, manager, tick_id)\n\n    # Pending result releases at tick 10. At tick 6 the old plan is already\n    # 120 ms old, so continuing to drive on it must still fail closed.\n    with pytest.raises(RuntimeError, match="ASYNC_L6_PLAN_STALE"):\n        _evaluate(navigator, manager, 6)\n\n    backend.close()\n'


def _run(repo: Path, *args: str) -> str:
    return subprocess.check_output(
        list(args), cwd=repo, text=True, stderr=subprocess.STDOUT
    ).strip()


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _runtime_pid(repo: Path) -> int | None:
    path = repo / "runtime/.r2b4_runtime_pid"
    try:
        value = path.read_text(encoding="utf-8").strip()
    except FileNotFoundError:
        return None
    try:
        pid = int(value)
    except ValueError:
        return None
    if pid <= 0:
        return None
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return None
    except PermissionError:
        return pid
    return pid


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("repo", nargs="?", default="/home/alba/project_r2b4")
    args = parser.parse_args()
    repo = Path(args.repo).resolve()
    package = Path(__file__).resolve().parent

    if not (repo / ".git").exists():
        raise SystemExit(f"ERROR: not a git repo: {repo}")

    running_pid = _runtime_pid(repo)
    if running_pid is not None:
        raise SystemExit(
            f"ERROR: resident runtime appears active (pid {running_pid}). "
            "Run ./r2b4 shutdown first."
        )

    head = _run(repo, "git", "rev-parse", "HEAD")
    try:
        _run(repo, "git", "merge-base", "--is-ancestor", BASE_COMMIT, head)
    except subprocess.CalledProcessError:
        raise SystemExit(
            "ERROR: repo HEAD is not based on the inspected async-L6 upgrade "
            f"commit {BASE_COMMIT} (HEAD={head})."
        )

    dirty = _run(
        repo,
        "git",
        "status",
        "--porcelain",
        "--",
        str(SOURCE_REL),
        str(TEST_REL),
    )
    if dirty:
        raise SystemExit(
            "ERROR: target source/test files have uncommitted changes. "
            "Commit/stash them before applying this P0 patch:\n" + dirty
        )

    source_path = repo / SOURCE_REL
    test_path = repo / TEST_REL
    source = source_path.read_text(encoding="utf-8")
    tests = test_path.read_text(encoding="utf-8")

    if NEW_BLOCK in source and MARKER in tests:
        print("PASS: P0 handoff fix is already installed.")
        return 0

    if source.count(OLD_BLOCK) != 1:
        raise SystemExit(
            "ERROR: expected _accept_pending_rollout anchor not found exactly once; "
            "repo source has drifted. No files changed."
        )
    if MARKER in tests:
        raise SystemExit(
            "ERROR: regression-test marker already exists but source fix is absent; "
            "partial/unknown state. No files changed."
        )

    backup = package / "backup"
    backup.mkdir(exist_ok=True)
    source_backup = backup / "l6_navigation.py"
    test_backup = backup / "test_v3_async_l6_planner.py"
    shutil.copy2(source_path, source_backup)
    shutil.copy2(test_path, test_backup)

    source_new = source.replace(OLD_BLOCK, NEW_BLOCK, 1)
    tests_new = tests.rstrip() + TEST_APPEND + "\n"

    source_path.write_text(source_new, encoding="utf-8")
    test_path.write_text(tests_new, encoding="utf-8")

    manifest = {
        "schema": "R2B4_ASYNC_L6_P0_HANDOFF_FIX_BACKUP_V1",
        "base_commit": BASE_COMMIT,
        "install_head": head,
        "files": {
            str(SOURCE_REL): {
                "backup": source_backup.name,
                "before_sha256": _sha256(source.encode()),
                "after_sha256": _sha256(source_new.encode()),
            },
            str(TEST_REL): {
                "backup": test_backup.name,
                "before_sha256": _sha256(tests.encode()),
                "after_sha256": _sha256(tests_new.encode()),
            },
        },
    }
    (backup / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    print("INSTALLED: async-L6 P0 handoff freshness fix")
    print(f"repo: {repo}")
    print("changed:")
    print(f"  {SOURCE_REL}")
    print(f"  {TEST_REL}")
    print("next:")
    print(f"  bash {package / 'validate_upgrade.sh'} {repo}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
