#!/usr/bin/env python3
"""Apply R2B4 Gemini structured-output transport fix v2.

Verified base: e06f443d91ece6c34de3bcef0cd9ff976fe4e8e6
Provider-local change only. Supports clean baseline and known v1 partial state.
"""
from __future__ import annotations

import argparse
import datetime as dt
from pathlib import Path
import shutil
import subprocess
import sys

BASE_COMMIT = "e06f443d91ece6c34de3bcef0cd9ff976fe4e8e6"
TARGET = "r2b4_voice/gemini_llm.py"
BASE_BLOB = "f87c9a9336e4b9619f8b0813b12d52acd36b3a2f"
FIXED_BLOB = "5ce67026f1a2b5ca7895aabbb24d301d1624c7b4"
NEW_TEST = "tests/core/test_gemini_structured_transport.py"
BROKEN_V1_TEST_BLOB = "9cfe4e89d0a080644a33246a796b19d0c0e2a281"
FIXED_TEST_BLOB = "66318ce09428f025e3dac729cad59fcc6992cda5"


def run(cmd: list[str], *, cwd: Path, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, cwd=cwd, text=True, capture_output=True, check=check)


def blob(root: Path, relative: str) -> str:
    return run(["git", "hash-object", relative], cwd=root).stdout.strip()


def preflight(root: Path, allow_head_mismatch: bool) -> tuple[bool, bool]:
    if not (root / ".git").exists() or not (root / "v3").is_dir():
        raise RuntimeError(f"not an R2B4 git worktree: {root}")
    head = run(["git", "rev-parse", "HEAD"], cwd=root).stdout.strip()
    if head != BASE_COMMIT and not allow_head_mismatch:
        raise RuntimeError(
            f"HEAD mismatch: expected {BASE_COMMIT}, got {head}; "
            "use --allow-head-mismatch only after reviewing the target blob"
        )

    target = root / TARGET
    if not target.is_file():
        raise RuntimeError(f"missing target file: {TARGET}")
    target_blob = blob(root, TARGET)
    if target_blob not in {BASE_BLOB, FIXED_BLOB}:
        raise RuntimeError(
            f"target modified or unexpected version: {TARGET}: got blob {target_blob}; "
            f"expected baseline {BASE_BLOB} or known fixed {FIXED_BLOB}"
        )

    test = root / NEW_TEST
    test_exists = test.exists()
    if test_exists:
        if not test.is_file():
            raise RuntimeError(f"test target is not a regular file: {NEW_TEST}")
        test_blob = blob(root, NEW_TEST)
        if test_blob not in {BROKEN_V1_TEST_BLOB, FIXED_TEST_BLOB}:
            raise RuntimeError(
                f"existing test has unknown local modifications: {NEW_TEST}: blob {test_blob}"
            )
    return target_blob == FIXED_BLOB, test_exists


def validate(root: Path, run_tests: bool) -> None:
    run([sys.executable, "-m", "py_compile", TARGET, NEW_TEST], cwd=root)
    if run_tests:
        cp = run([
            sys.executable, "-m", "pytest", "-q",
            NEW_TEST,
            "tests/feature/test_agent_core.py",
        ], cwd=root, check=False)
        sys.stdout.write(cp.stdout)
        sys.stderr.write(cp.stderr)
        if cp.returncode:
            raise RuntimeError(f"targeted pytest failed with rc={cp.returncode}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default=".")
    parser.add_argument("--allow-head-mismatch", action="store_true")
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument("--run-tests", action="store_true")
    args = parser.parse_args(argv)

    root = Path(args.root).expanduser().resolve()
    package_root = Path(__file__).resolve().parent
    already_fixed, test_existed = preflight(root, args.allow_head_mismatch)
    if args.check_only:
        state = "KNOWN_FIXED" if already_fixed else "BASELINE"
        print(f"preflight: PASS ({state})")
        return 0

    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_root = root / ".upgrade_backups" / f"gemini_structured_fix_v2_{stamp}"
    backup_target = backup_root / TARGET
    backup_target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(root / TARGET, backup_target)
    backup_test = backup_root / NEW_TEST
    if test_existed:
        backup_test.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(root / NEW_TEST, backup_test)

    try:
        shutil.copy2(package_root / "payload" / TARGET, root / TARGET)
        test_target = root / NEW_TEST
        test_target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(package_root / "payload" / NEW_TEST, test_target)
        validate(root, args.run_tests)
    except Exception:
        shutil.copy2(backup_target, root / TARGET)
        if test_existed:
            shutil.copy2(backup_test, root / NEW_TEST)
        else:
            try:
                (root / NEW_TEST).unlink()
            except FileNotFoundError:
                pass
        raise

    print("upgrade: APPLIED")
    print(f"backup: {backup_root}")
    print("validation: py_compile PASS")
    if not args.run_tests:
        print(f"targeted: python3 -m pytest -q {NEW_TEST} tests/feature/test_agent_core.py")
    print('live smoke: r "Miért kék az ég? Egy rövid mondatban válaszolj."')
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
