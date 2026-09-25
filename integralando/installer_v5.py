#!/usr/bin/env python3
"""R2B4 pytest-refactor V5: staging/project-root fix.

Fixes a source-first bug exposed by moving curated tests from tests/test_*.py to
nested tests/core|feature|deep/test_*.py. Legacy tests that derive PROJECT_ROOT
from Path(__file__).parents[1] then resolve to tests/ instead of the repository
root. During V2/V4 staging this becomes /tmp/.../tests, causing config reads from
/tmp/.../tests/conf/*.json.

V5 temporarily normalizes those brittle root expressions before invoking the
existing V4 installer. --check restores the original tests tree. A real install
keeps only the curated V4 suite and validates it once more with R2B4_ROOT unset,
proving that the installed nested suite can find the real repository root on its
own.
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import time

UPGRADE_ID = "pytest_refactor_v5_20260925"

# One expression, no helper/import dependency: V2's AST curation can move/copy
# the containing assignment/function without having to discover an extra helper.
ROOT_EXPR = (
    '(Path(__import__("os").environ["R2B4_ROOT"]).resolve() '
    'if __import__("os").environ.get("R2B4_ROOT") else '
    'next((p for p in Path(__file__).resolve().parents '
    'if (p / "conf" / "hardver.json").is_file() and (p / "v3").is_dir()), Path.cwd()))'
)

# Patterns seen in older flat tests. They were valid while files lived directly
# under tests/, but become wrong after moving the same file into tests/core etc.
ROOT_PATTERNS = [
    re.compile(r'Path\(__file__\)\.resolve\(\)\.parents\[1\]'),
    re.compile(r'Path\(__file__\)\.resolve\(\)\.parent\.parent'),
    re.compile(r'Path\(__file__\)\.parents\[1\]'),
    re.compile(r'Path\(__file__\)\.parent\.parent'),
]


def find_repo(start: Path) -> Path:
    env = os.environ.get("R2B4_ROOT")
    if env:
        p = Path(env).expanduser().resolve()
        if (p / "tests").is_dir() and (p / "v3").is_dir() and (p / "conf").is_dir():
            return p
        raise SystemExit(f"R2B4_ROOT is not a repo root: {p}")
    candidates = [start, *start.parents, Path("/home/alba/project_r2b4")]
    seen: set[Path] = set()
    for p in candidates:
        try:
            p = p.resolve()
        except Exception:
            continue
        if p in seen:
            continue
        seen.add(p)
        if (p / "tests").is_dir() and (p / "v3").is_dir() and (p / "conf").is_dir():
            return p
    raise SystemExit("R2B4 repo root not found. Set R2B4_ROOT if needed.")


def find_v4(script_dir: Path, repo: Path) -> Path:
    override = os.environ.get("R2B4_PYTEST_V4_INSTALLER")
    candidates = []
    if override:
        candidates.append(Path(override))
    candidates += [
        script_dir / "installer_v4.py",
        script_dir.parent / "installer_v4.py",
        repo / "integralando" / "installer_v4.py",
    ]
    me = Path(__file__).resolve()
    for p in candidates:
        try:
            p = p.expanduser().resolve()
        except Exception:
            continue
        if p.is_file() and p != me:
            text = p.read_text(encoding="utf-8", errors="ignore")
            if "pytest_refactor" in text and "--check" in text:
                return p
    raise SystemExit(
        "installer_v4.py not found. Keep installer_v5.py under integralando next "
        "to the existing installer_v4.py, or set R2B4_PYTEST_V4_INSTALLER."
    )


def config_preflight(repo: Path) -> None:
    required = ["hardver.json", "fizika.json", "speed_map.json", "vezerles.json"]
    missing = []
    for name in required:
        p = repo / "conf" / name
        if p.is_symlink() or not p.is_file():
            missing.append(str(p))
    if missing:
        raise RuntimeError("canonical production config missing/non-regular: " + ", ".join(missing))
    print("Production config preflight: PASS (canonical 4 documents present as regular files)")


def patch_text(text: str) -> tuple[str, int]:
    # Avoid repatching a file already normalized by an interrupted/manual run.
    if 'environ["R2B4_ROOT"]' in text and 'conf" / "hardver.json' in text:
        return text, 0
    total = 0
    for pattern in ROOT_PATTERNS:
        text, n = pattern.subn(lambda _m: ROOT_EXPR, text)
        total += n
    return text, total


def patch_tests_in_place(tests: Path) -> tuple[dict[Path, bytes], list[tuple[Path, int]]]:
    originals: dict[Path, bytes] = {}
    changed: list[tuple[Path, int]] = []
    for p in sorted(tests.rglob("*.py")):
        if "__pycache__" in p.parts:
            continue
        try:
            raw = p.read_bytes()
            text = raw.decode("utf-8")
        except (OSError, UnicodeDecodeError):
            continue
        new_text, n = patch_text(text)
        if not n:
            continue
        originals[p] = raw
        p.write_text(new_text, encoding="utf-8")
        changed.append((p, n))
    return originals, changed


def restore_files(originals: dict[Path, bytes]) -> None:
    for p, raw in originals.items():
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(raw)


def full_tests_backup(repo: Path) -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    dst = repo / ".upgrade_backups" / f"{UPGRADE_ID}_{stamp}" / "tests"
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(repo / "tests", dst)
    return dst


def restore_tree(repo: Path, backup: Path) -> None:
    tests = repo / "tests"
    if tests.exists():
        shutil.rmtree(tests)
    shutil.copytree(backup, tests)


def run_v4(v4: Path, repo: Path, check: bool) -> int:
    cmd = [sys.executable, str(v4)]
    if check:
        cmd.append("--check")
    print("+", " ".join(cmd), flush=True)
    env = os.environ.copy()
    # The patched legacy PROJECT_ROOT expressions use this only during staging.
    env["R2B4_ROOT"] = str(repo)
    return subprocess.call(cmd, cwd=str(v4.parent), env=env)


def run_final_validation(repo: Path) -> int:
    targets = [repo / "tests" / x for x in ("core", "feature", "deep") if (repo / "tests" / x).is_dir()]
    if not targets:
        print("ERROR: curated core/feature/deep test directories not found after install")
        return 2
    cmd = [sys.executable, "-m", "pytest", "-q", *map(str, targets)]
    print("Final installed-suite validation (R2B4_ROOT deliberately unset):")
    print("+", " ".join(cmd), flush=True)
    env = os.environ.copy()
    env.pop("R2B4_ROOT", None)
    return subprocess.call(cmd, cwd=str(repo), env=env)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="dry-run; original tests are restored")
    ap.add_argument("--report", action="store_true", help="only report root-coupled test files")
    args = ap.parse_args()

    repo = find_repo(Path.cwd())
    v4 = find_v4(Path(__file__).resolve().parent, repo)
    try:
        config_preflight(repo)
    except Exception as exc:
        print("ERROR:", exc)
        return 2

    # Independent backup before any source normalization. On --check we only need
    # byte-level restoration of touched files; install additionally protects the
    # entire tests tree because V4 replaces it.
    tree_backup = None if args.check or args.report else full_tests_backup(repo)
    if tree_backup:
        print("Full pre-V5 tests backup:", tree_backup)

    originals, changed = patch_tests_in_place(repo / "tests")
    print(f"Root-coupled test normalization: {len(changed)} files, {sum(n for _, n in changed)} expressions")
    for p, n in changed:
        print(f" - {p.relative_to(repo)} ({n})")

    if args.report:
        restore_files(originals)
        print("REPORT restore: original test files restored.")
        return 0

    rc = 99
    try:
        rc = run_v4(v4, repo, args.check)
        if rc == 0 and not args.check:
            rc = run_final_validation(repo)
            if rc != 0:
                print("ERROR: installed curated suite failed without staging root override; rolling back.")
    finally:
        if args.check:
            restore_files(originals)
            print("V5 CHECK restore: original root-coupled tests restored.")
        elif rc != 0:
            assert tree_backup is not None
            restore_tree(repo, tree_backup)
            print("V5 INSTALL rollback: full original tests tree restored.")

    if rc == 0:
        print("V5 CHECK PASS" if args.check else "V5 INSTALL PASS")
        return 0
    print(f"V5 {'CHECK' if args.check else 'INSTALL'} FAILED (exit={rc}); no partial tests-tree install retained.")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
