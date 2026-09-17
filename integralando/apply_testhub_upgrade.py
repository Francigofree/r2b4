#!/usr/bin/env python3
"""Apply the compact R2B4 Test Hub portable-evidence upgrade.

Usage:
    cd /home/alba/project_r2b4
    python3 /path/to/apply_testhub_upgrade.py

The script performs a source-shape preflight, then atomically writes only the
small Test Hub surface. It creates no backup/admin tree; Git remains the backup.
"""

from __future__ import annotations

import argparse
import os
import py_compile
import shutil
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent

NEW_FILES = (
    "v3/mcap_replay_bridge.py",
    "v3/test_hub_next.py",
    "v3/test_hub_portable.py",
    "v3/test_hub_runtime.py",
    "tests/test_v3_test_hub_portable.py",
)

OLD_PUBLIC_ROUTE = 'if not arguments or arguments[0] in {"run", "view", "compare", "-h", "--help"}:'
NEW_PUBLIC_ROUTE = 'if not arguments or arguments[0] in {"run", "batch", "view", "compare", "test", "-h", "--help"}:'

OLD_RUNTIME_IMPORT = "from v3.test_hub_v2 import diagnose_run\n"
GITIGNORE_BLOCK = """# Local authoritative robot captures; portable Test Hub evidence remains trackable
/runtime/captures/*.mcap
/runtime/captures/.*.mcap.*.partial
"""
OLD_RUNTIME_BLOCK = '''        # finish has drained, checked RELIABLE integrity, fsynced and published MCAP.\n        self.evidence = diagnose_run(result.path, result.path.with_suffix(".evidence"),\n                                     replay_mode="incident", project_root=PROJECT_ROOT)\n        return result.path\n'''
NEW_RUNTIME_BLOCK = '''        # Hardware ownership has ended and the MCAP is fully fsynced/published.\n        # Analysis is process-isolated behind a tiny stdlib-only handoff.\n        from v3.test_hub_runtime import postprocess_capture\n        self.evidence = postprocess_capture(result.path, project_root=PROJECT_ROOT)\n        return result.path\n'''


def _atomic_write(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.testhub-upgrade.{os.getpid()}.tmp")
    try:
        temporary.write_bytes(data)
        os.replace(temporary, path)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def _patched_public_entrypoint(path: Path) -> str:
    text = path.read_text(encoding="utf-8")
    if NEW_PUBLIC_ROUTE in text:
        return text
    if OLD_PUBLIC_ROUTE not in text:
        raise RuntimeError("v3/test_hub.py public routing shape is not the expected current version")
    return text.replace(OLD_PUBLIC_ROUTE, NEW_PUBLIC_ROUTE, 1)


def _patched_runtime(path: Path) -> str:
    text = path.read_text(encoding="utf-8")
    already = (
        "from v3.test_hub_runtime import postprocess_capture" in text
        and "self.evidence = postprocess_capture(" in text
    )
    if already:
        return text
    if OLD_RUNTIME_IMPORT not in text:
        raise RuntimeError("v3_process_runtime.py no longer has the expected Test Hub import")
    if OLD_RUNTIME_BLOCK not in text:
        raise RuntimeError("v3_process_runtime.py capture-finalize shape is not the expected current version")
    text = text.replace(OLD_RUNTIME_IMPORT, "", 1)
    return text.replace(OLD_RUNTIME_BLOCK, NEW_RUNTIME_BLOCK, 1)


def _patched_gitignore(path: Path) -> str:
    text = path.read_text(encoding="utf-8") if path.is_file() else ""
    required = ("/runtime/captures/*.mcap", "/runtime/captures/.*.mcap.*.partial")
    if all(item in text.splitlines() for item in required):
        return text
    if text and not text.endswith("\n"):
        text += "\n"
    if text and not text.endswith("\n\n"):
        text += "\n"
    return text + GITIGNORE_BLOCK


def apply(repo_root: Path) -> None:
    root = repo_root.resolve()
    if not (root / "v3").is_dir() or not (root / "tests").is_dir():
        raise RuntimeError(f"not an R2B4 repo root: {root}")

    public_path = root / "v3" / "test_hub.py"
    runtime_path = root / "v3_process_runtime.py"
    gitignore_path = root / ".gitignore"
    if not public_path.is_file() or not runtime_path.is_file():
        raise RuntimeError("required current Test Hub/runtime files are missing")

    # Preflight all source shapes and package inputs before writing anything.
    public_text = _patched_public_entrypoint(public_path)
    runtime_text = _patched_runtime(runtime_path)
    gitignore_text = _patched_gitignore(gitignore_path)
    for relative in NEW_FILES:
        source = PACKAGE_ROOT / relative
        if not source.is_file():
            raise RuntimeError(f"upgrade package is incomplete: {relative}")

    writes: dict[Path, bytes] = {
        public_path: public_text.encode("utf-8"),
        runtime_path: runtime_text.encode("utf-8"),
        gitignore_path: gitignore_text.encode("utf-8"),
    }
    for relative in NEW_FILES:
        writes[root / relative] = (PACKAGE_ROOT / relative).read_bytes()

    for path, data in writes.items():
        _atomic_write(path, data)

    compile_targets = [
        root / "v3" / "test_hub.py",
        root / "v3" / "mcap_replay_bridge.py",
        root / "v3" / "test_hub_next.py",
        root / "v3" / "test_hub_portable.py",
        root / "v3" / "test_hub_runtime.py",
        root / "v3_process_runtime.py",
        root / "tests" / "test_v3_test_hub_portable.py",
    ]
    for target in compile_targets:
        py_compile.compile(str(target), doraise=True)

    print("Test Hub upgrade applied.")
    print("Changed/added:")
    print(" - .gitignore")
    for target in compile_targets:
        print(" -", target.relative_to(root))
    print()
    print("Targeted validation:")
    print(" python3 -m pytest -q tests/test_v3_test_hub_analysis.py tests/test_v3_test_hub_cli.py tests/test_v3_test_hub_evidence.py tests/test_v3_test_hub_portable.py tests/test_v3_mcap_e2e.py tests/test_v3_process_runtime.py")
    print("Full validation:")
    print(" python3 -m v3.test_hub test --scope full")
    print("Convert newest MCAP:")
    print(" python3 -m v3.test_hub")
    print("Convert any old/unprocessed MCAPs:")
    print(" python3 -m v3.test_hub batch")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("repo_root", nargs="?", default=".")
    args = parser.parse_args(argv)
    try:
        apply(Path(args.repo_root))
        return 0
    except (OSError, RuntimeError, py_compile.PyCompileError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
