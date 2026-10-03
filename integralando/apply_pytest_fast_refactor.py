#!/usr/bin/env python3
from __future__ import annotations

import argparse
import datetime as dt
import json
import shutil
import subprocess
import sys
from pathlib import Path

UPGRADE_ID = "pytest_fast_refactor_20261003"
EXPECTED_HEAD = "4f12c19d5aae047bcb581d03dcafba8188daec42"
EXPECTED_BLOBS = {
    "v3/test_runner.py": "e1eab1208642d422dd7d522e3b17630a745100d4",
    "v3/host_cli.py": "63173482620fe15b7fbfea0d38e64c6fe5b39135",
    "v3/launcher_cli.py": "4d56991bf60a2893df696bf9183804639616ab44",
    "tests/README.md": "7b495d55adc22d08d16814984827d67022f58f7c",
    "docs/PYTEST_POLICY.md": "d082ee62824920f99df58edaa87bad6a2c082828",
}

NEW_TEST_RUNNER = '''from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TESTS = ROOT / "tests"

# Default developer loop: deliberately small, high-signal and hardware-free.
# Full layer coverage remains available through `r test core` and `r test full`.
QUICK_TARGETS = (
    "tests/core/test_v3_temporal_contract.py",
    "tests/core/test_v3_motor_pwm_contract.py",
    "tests/core/test_v3_stop_only_composition.py::test_fault_latch_keeps_later_healthy_request_at_zero",
    "tests/core/test_v3_stop_only_composition.py::test_writer_failure_is_single_zero_attempt_and_faults_lifecycle",
    "tests/core/test_v3_multirate_inputs.py::test_worker_initializer_failure_aborts_before_runtime_use",
    "tests/core/test_v3_bounded_runtime_config.py::test_loader_rejects_symlink_before_reading_config",
    "tests/core/test_v3_config_p0_authority.py::test_camera_freshness_is_explicit_resolved_authority",
    "tests/core/test_operator_sequence_stop.py::test_command_producer_owner_exit_and_watchdog_stop_before_next_heartbeat",
)

# Focused modes list files directly. This avoids importing/collecting an entire
# FEATURE/DEEP layer only to deselect most of it with `-k` afterwards.
FOCUSED = {
    "follow": (
        "tests/feature/test_v3_canonical_motion_harmony.py",
        "tests/feature/test_v3_person_detection_runtime_integration.py",
    ),
    "roomcruise": (
        "tests/feature/test_v3_roomcruise_localization_motion.py",
        "tests/feature/test_v3_roomcruise_tuner.py",
        "tests/feature/test_v3_canonical_motion_harmony.py",
    ),
    "localization": (
        "tests/feature/test_v3_dual_frame_localization.py",
        "tests/feature/test_v3_stationary_covariance.py",
        "tests/feature/test_v3_stationary_relocalization_repair.py",
        "tests/feature/test_v3_rate_only_heading_authority.py",
        "tests/feature/test_v3_roomcruise_localization_motion.py",
    ),
    "perception": (
        "tests/feature/test_camera_observation.py",
        "tests/feature/test_v3_person_detection.py",
        "tests/feature/test_v3_person_geometry_projection.py",
        "tests/feature/test_v3_latest_lidar_backend.py",
        "tests/feature/test_v3_live_lidar.py",
        "tests/feature/test_v3_lidar_world_model.py",
    ),
    "motion": (
        "tests/feature/test_v3_canonical_motion_harmony.py",
        "tests/feature/test_v3_finite_motion_frame_transaction.py",
        "tests/feature/test_v3_l11_reversal_reacquisition.py",
        "tests/feature/test_v3_motion_feedback_quality.py",
        "tests/feature/test_v3_roomcruise_localization_motion.py",
    ),
    "async": (
        "tests/deep/test_v3_async_peripheral_isolation.py",
    ),
    "process": (
        "tests/deep/test_v3_async_peripheral_isolation.py",
        "tests/deep/test_v3_localization_process.py",
        "tests/deep/test_v3_process_affinity.py",
    ),
    "replay": (
        "tests/deep/test_v3_lidar_world_replay.py",
        "tests/deep/test_v3_replay_claims.py",
    ),
}

FOCUSED_K = {
    "follow": "follow",
    "roomcruise": "room_cruise or roomcruise or cruise or explore",
}
ALIASES = {"room_cruise": "roomcruise"}


def _run(targets: tuple[str, ...] | list[str], *, k: str | None = None, fail_fast: bool = False) -> int:
    cmd = [sys.executable, "-m", "pytest", "-q", "--tb=short", *targets]
    if fail_fast:
        cmd += ["--maxfail=1"]
    if k:
        cmd += ["-k", k]
    return subprocess.call(cmd, cwd=ROOT)


def main(argv=None) -> int:
    from v3.runtime_performance import apply_host_affinity

    apply_host_affinity(ROOT, "diagnostics")
    args = list(sys.argv[1:] if argv is None else argv)
    mode = args.pop(0) if args else "quick"
    mode = ALIASES.get(mode, mode)
    if args:
        print(
            "Unexpected extra arguments. Use: ./r test [quick|core|full|follow|roomcruise|localization|perception|motion|async|process|replay]",
            file=sys.stderr,
        )
        return 2

    if mode == "quick":
        return _run(list(QUICK_TARGETS), fail_fast=True)
    if mode == "core":
        return _run([str(TESTS / "core")])
    if mode == "full":
        return _run([str(TESTS / "core"), str(TESTS / "feature"), str(TESTS / "deep")])
    if mode in FOCUSED:
        return _run(list(FOCUSED[mode]), k=FOCUSED_K.get(mode))
    if mode in {"list", "help", "-h", "--help"}:
        print("./r test            QUICK (11 high-signal CORE cases, fail-fast)")
        print("./r test core       complete CORE layer")
        print("./r test follow     focused Follow slice")
        print("./r test full       CORE + FEATURE + DEEP")
        print("Other focused modes: roomcruise localization perception motion async process replay")
        return 0

    print(f"Unknown test mode: {mode}", file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
'''

PATCHES = {
    "v3/host_cli.py": [
        (
            '"test": ("[MODE]", "Célzott tesztek; alapértelmezés: core. Módok: r test list."),',
            '"test": ("[MODE]", "Célzott tesztek; alapértelmezés: quick (11 high-signal CORE eset). Módok: r test list."),',
        ),
    ],
    "v3/launcher_cli.py": [
        (
            '        "  r test [MODE]              Tesztek; alap: core; módok: r test list\\n"',
            '        "  r test [MODE]              Tesztek; alap: quick; teljes CORE: r test core\\n"',
        ),
    ],
    "tests/README.md": [
        (
            "./r test\n./r test follow",
            "./r test\n./r test core\n./r test follow",
        ),
        (
            "A `./r test` CORE-t futtat. A fókuszált módok a megfelelő FEATURE vagy DEEP szeletet futtatják. A `./r test full` a teljes CORE + FEATURE + DEEP suite.",
            "A `./r test` egy 11 esetes, fail-fast QUICK kapu. A `./r test core` futtatja a teljes CORE réteget. A fókuszált módok csak a releváns FEATURE/DEEP fájlokat gyűjtik, így nem importálják a teljes réteget. A `./r test full` továbbra is a teljes CORE + FEATURE + DEEP suite.",
        ),
    ],
    "docs/PYTEST_POLICY.md": [
        (
            "- `./r test` — CORE after normal agent changes.\n- `./r test <feature>` — relevant scenario slice, e.g. `follow`.\n- `./r test full` — complete 100-200 case suite for shared boundaries / release acceptance.",
            "- `./r test` — QUICK: 11 high-signal, hardware-free CORE cases for the normal edit loop.\n- `./r test core` — complete CORE layer.\n- `./r test <feature>` — relevant scenario slice, e.g. `follow`; collect only the files owned by that slice.\n- `./r test full` — complete 100-200 case suite for shared boundaries / release acceptance.",
        ),
    ],
}


def run(cmd: list[str], *, cwd: Path, capture: bool = False) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, cwd=cwd, check=False, text=True, capture_output=capture)


def git_blob(root: Path, rel: str) -> str:
    cp = run(["git", "hash-object", rel], cwd=root, capture=True)
    if cp.returncode != 0:
        raise RuntimeError(cp.stderr.strip() or f"git hash-object failed: {rel}")
    return cp.stdout.strip()


def replace_once(text: str, old: str, new: str, rel: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{rel}: expected patch anchor exactly once, found {count}")
    return text.replace(old, new, 1)


def preflight(root: Path) -> None:
    if not (root / ".git").exists() or not (root / "v3").is_dir():
        raise RuntimeError(f"not an R2B4 git working tree: {root}")
    cp = run(["git", "rev-parse", "HEAD"], cwd=root, capture=True)
    head = cp.stdout.strip() if cp.returncode == 0 else ""
    if head != EXPECTED_HEAD:
        raise RuntimeError(
            f"HEAD mismatch: expected {EXPECTED_HEAD}, got {head or 'UNKNOWN'}. "
            "This upgrade is intentionally source-locked; regenerate it for the newer repo."
        )
    for rel, expected in EXPECTED_BLOBS.items():
        path = root / rel
        if not path.is_file():
            raise RuntimeError(f"missing file: {rel}")
        actual = git_blob(root, rel)
        if actual != expected:
            raise RuntimeError(f"local file differs from source snapshot: {rel} ({actual} != {expected})")


def apply(root: Path, *, dry_run: bool) -> Path | None:
    preflight(root)
    originals = {rel: (root / rel).read_text(encoding="utf-8") for rel in EXPECTED_BLOBS}
    updated = dict(originals)
    updated["v3/test_runner.py"] = NEW_TEST_RUNNER
    for rel, edits in PATCHES.items():
        value = updated[rel]
        for old, new in edits:
            value = replace_once(value, old, new, rel)
        updated[rel] = value

    if dry_run:
        for rel in EXPECTED_BLOBS:
            changed = originals[rel] != updated[rel]
            print(f"{'CHANGE' if changed else 'SAME  '} {rel}")
        return None

    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = root / ".upgrade_backups" / f"{UPGRADE_ID}_{stamp}"
    for rel, content in originals.items():
        dst = backup / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        dst.write_text(content, encoding="utf-8")
    for rel, content in updated.items():
        dst = root / rel
        dst.write_text(content, encoding="utf-8")

    checks = []
    cp = run([sys.executable, "-m", "py_compile", "v3/test_runner.py", "v3/host_cli.py", "v3/launcher_cli.py"], cwd=root, capture=True)
    checks.append({"name": "py_compile", "returncode": cp.returncode, "stdout": cp.stdout, "stderr": cp.stderr})
    if cp.returncode != 0:
        raise RuntimeError(f"post-check py_compile failed; backup: {backup}\n{cp.stderr}")

    cp = run([sys.executable, "-m", "v3.test_runner", "list"], cwd=root, capture=True)
    checks.append({"name": "test_runner_list", "returncode": cp.returncode, "stdout": cp.stdout, "stderr": cp.stderr})
    if cp.returncode != 0:
        raise RuntimeError(f"post-check test_runner list failed; backup: {backup}\n{cp.stderr}")

    # Collection-only validates all quick nodeids without executing robot logic.
    quick_cmd = [sys.executable, "-m", "pytest", "--collect-only", "-q", *QUICK_TARGETS]
    cp = run(quick_cmd, cwd=root, capture=True)
    checks.append({"name": "quick_collect", "returncode": cp.returncode, "stdout": cp.stdout, "stderr": cp.stderr})
    if cp.returncode != 0:
        raise RuntimeError(f"post-check quick collection failed; backup: {backup}\n{cp.stdout}\n{cp.stderr}")

    result = {
        "schema": "R2B4_PYTEST_FAST_REFACTOR_V1",
        "upgrade_id": UPGRADE_ID,
        "source_head": EXPECTED_HEAD,
        "backup": str(backup),
        "quick_target_specs": len(QUICK_TARGETS),
        "quick_expected_cases": 11,
        "focused_modes": sorted(FOCUSED),
        "checks": checks,
    }
    (root / "PYTEST_FAST_REFACTOR_RESULT.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(f"APPLIED: {UPGRADE_ID}")
    print(f"backup: {backup}")
    print("next: ./r test")
    print("full core: ./r test core")
    print("full suite: ./r test full")
    return backup


def rollback(root: Path, backup_value: str | None) -> None:
    base = root / ".upgrade_backups"
    if backup_value:
        backup = Path(backup_value).expanduser().resolve()
    else:
        candidates = sorted(base.glob(f"{UPGRADE_ID}_*"), reverse=True)
        if not candidates:
            raise RuntimeError(f"no backup found under {base}")
        backup = candidates[0]
    for rel in EXPECTED_BLOBS:
        src = backup / rel
        if not src.is_file():
            raise RuntimeError(f"backup is incomplete: {src}")
    for rel in EXPECTED_BLOBS:
        dst = root / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(backup / rel, dst)
    result = root / "PYTEST_FAST_REFACTOR_RESULT.json"
    if result.exists():
        result.unlink()
    print(f"ROLLED BACK from: {backup}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Apply the R2B4 fast pytest refactor.")
    parser.add_argument("--root", default=".", help="R2B4 repository root (default: current directory)")
    parser.add_argument("--dry-run", action="store_true", help="validate and show changed files without writing")
    parser.add_argument("--rollback", action="store_true", help="restore the most recent upgrade backup")
    parser.add_argument("--backup", help="explicit backup directory for --rollback")
    args = parser.parse_args()
    root = Path(args.root).expanduser().resolve()
    try:
        if args.rollback:
            rollback(root, args.backup)
        else:
            apply(root, dry_run=args.dry_run)
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
