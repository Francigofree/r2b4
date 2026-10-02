#!/usr/bin/env python3
"""Apply R2B4 Agent Core refactor to Francigofree/r2b4.

Base: f045f80ada7b76ff804b39e818371bcb7cc21418
The script refuses to overwrite target files that differ from the verified base
blob unless --allow-head-mismatch is used; even then target blob checks remain
strict, so unrelated local edits are preserved rather than overwritten.
"""
from __future__ import annotations

import argparse
import datetime as dt
import os
from pathlib import Path
import shutil
import subprocess
import sys

BASE_COMMIT = "f045f80ada7b76ff804b39e818371bcb7cc21418"
EXPECTED_BLOBS = {
    "r2b4_voice/gemini_llm.py": "f424f53ee8ec5f600b314fa9e97541eeba7605ab",
    "r2b4_voice/groq_llm.py": "b64ace055e395d4c7fc1ce63ea83a744689731e2",
    "r2b4_voice/conversation_service.py": "c8e1db25eac6059f320b9d8a65c42a536e05a4c9",
    "r2b4_voice/prompting.py": "80304c92f2579159a677b501498979756ddf6a6c",
    "r2b4_voice/conversation_interface.py": "cbdaa4a7eced2b1786a8dc381f0764b4705a8cff",
    "r2b4_orchestration/execution_mode.py": "7f1fcb277843dbc370dbc5697dfa6a81594cba4a",
    "r2b4_orchestration/executor.py": "e9cc8a80e4c33a7a2ae6c97bcffd3c6b6d7fecd5",
    "tests/feature/test_execution_mode_selector.py": "38584c1afc482b92cd88ef7e53d79baca88598ec",
    "tests/feature/test_execution_mode_executor.py": "0b285ce4db85a92f6ee81f4f0d41fbc6abadb9c8",
    "v3/launcher_cli.py": "1d3c468bc3cf58a8c8d1ba6d42e57e97d2aa3bcb",
    "r2b4_voice/conversation_cli.py": "8f276aac15a6a7a82e91d7d50ede23b3a34e60ae",
    "r2b4_voice/voice_service.py": "e6916f82eed7e76c9f9d6f0dc5079cdf36e1433c",
}

NEW_FILES = {
    "r2b4_orchestration/agent_contracts.py",
    "r2b4_orchestration/agent_core.py",
    "r2b4_orchestration/agent_tools.py",
    "r2b4_orchestration/agent_source_tools.py",
    "r2b4_orchestration/agent_evidence_tools.py",
    "r2b4_orchestration/agent_config_tools.py",
    "r2b4_orchestration/agent_runner.py",
    "conf/r2b4_agent_system.md",
    "docs/R2B4_AGENT_CORE.md",
    "tests/feature/test_agent_core.py",
    "tests/core/test_agent_config_tools.py",
}

REPLACEMENT_FILES = {
    "r2b4_voice/gemini_llm.py",
    "r2b4_voice/groq_llm.py",
    "r2b4_voice/conversation_service.py",
    "r2b4_voice/prompting.py",
    "r2b4_voice/conversation_interface.py",
    "r2b4_orchestration/execution_mode.py",
    "r2b4_orchestration/executor.py",
    "tests/feature/test_execution_mode_selector.py",
    "tests/feature/test_execution_mode_executor.py",
}

PATCHES = {
    "v3/launcher_cli.py": [
        (
            '"modes": [\n                "GEMINI_CHAT", "HOST_READ", "OBSERVATION",\n                "DIRECT_V3", "ER2_PREVIEW", "ER2_STREAM",\n            ],',
            '"modes": ["AGENT", "DIRECT_V3"],',
        ),
        (
            r'  r \"KÉRÉS\"                 Automatikus végrehajtási mód választás\n',
            r'  r \"KÉRÉS\"                 Agent Core: LLM + R2B4 toolok; exact STOP lokális\n',
        ),
        (
            r'  r route \"KÉRÉS\" --json    Módválasztás megmutatása végrehajtás nélkül\n',
            r'  r route \"KÉRÉS\" --json    Belépési route: STOP vagy AGENT\n',
        ),
        (
            "print('Auto route: r \"REQUEST\"; dry-run: r route \"REQUEST\" --json')",
            "print('Agent route: r \"REQUEST\"; dry-run: r route \"REQUEST\" --json')",
        ),
    ],
    "r2b4_voice/conversation_cli.py": [
        ('prompt = root / "conf" / "voice_llm_system.md"', 'prompt = root / "conf" / "r2b4_agent_system.md"'),
        ('"action_mode": "SHADOW",', '"action_mode": "AGENT_PROPOSAL_ONLY",'),
    ],
    "r2b4_voice/voice_service.py": [
        ("llm_timeout_s: float = 25.0", "llm_timeout_s: float = 90.0"),
    ],
}


def run(cmd: list[str], *, cwd: Path, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, cwd=cwd, text=True, capture_output=True, check=check)


def git_blob(root: Path, relative: str) -> str:
    cp = run(["git", "hash-object", relative], cwd=root)
    return cp.stdout.strip()


def preflight(root: Path, allow_head_mismatch: bool) -> None:
    if not (root / ".git").exists() or not (root / "v3").is_dir():
        raise RuntimeError(f"not an R2B4 git worktree: {root}")
    head = run(["git", "rev-parse", "HEAD"], cwd=root).stdout.strip()
    if head != BASE_COMMIT and not allow_head_mismatch:
        raise RuntimeError(f"HEAD mismatch: expected {BASE_COMMIT}, got {head}; use --allow-head-mismatch only after review")
    for relative, expected in EXPECTED_BLOBS.items():
        path = root / relative
        if not path.is_file():
            raise RuntimeError(f"missing target file: {relative}")
        actual = git_blob(root, relative)
        if actual != expected:
            raise RuntimeError(f"target modified or unexpected version: {relative}: expected blob {expected}, got {actual}")
    for relative in NEW_FILES:
        if (root / relative).exists():
            raise RuntimeError(f"new target already exists; refusing overwrite: {relative}")
    for relative, pairs in PATCHES.items():
        text = (root / relative).read_text(encoding="utf-8")
        for old, _new in pairs:
            if text.count(old) != 1:
                raise RuntimeError(f"patch anchor not unique in {relative}: {old[:80]!r}")


def backup(root: Path, relatives: set[str]) -> Path:
    dest = root / ".upgrade_backups" / f"agent_core_{dt.datetime.now():%Y%m%d_%H%M%S}"
    for relative in sorted(relatives):
        source = root / relative
        if source.exists():
            target = dest / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
    return dest


def apply_payload(root: Path, package_root: Path) -> None:
    payload = package_root / "payload"
    for relative in sorted(NEW_FILES | REPLACEMENT_FILES):
        source = payload / relative
        if not source.is_file():
            raise RuntimeError(f"package payload missing: {relative}")
        target = root / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    for relative, pairs in PATCHES.items():
        path = root / relative
        text = path.read_text(encoding="utf-8")
        for old, new in pairs:
            text = text.replace(old, new, 1)
        path.write_text(text, encoding="utf-8")


def validate(root: Path, run_tests: bool) -> None:
    changed_py = [
        *[str(p) for p in sorted(NEW_FILES | REPLACEMENT_FILES) if p.endswith(".py")],
        "v3/launcher_cli.py", "r2b4_voice/conversation_cli.py", "r2b4_voice/voice_service.py",
    ]
    run([sys.executable, "-m", "py_compile", *changed_py], cwd=root)
    if run_tests:
        cp = run([
            sys.executable, "-m", "pytest", "-q",
            "tests/feature/test_agent_core.py",
            "tests/feature/test_execution_mode_selector.py",
            "tests/feature/test_execution_mode_executor.py",
            "tests/core/test_agent_config_tools.py",
            "tests/feature/test_tools_diag_evidence.py",
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
    preflight(root, args.allow_head_mismatch)
    if args.check_only:
        print("preflight: PASS")
        return 0
    touched = set(EXPECTED_BLOBS) | NEW_FILES
    backup_dir = backup(root, touched)
    try:
        apply_payload(root, package_root)
        validate(root, args.run_tests)
    except Exception:
        # Restore all original files and remove newly created paths.
        for relative in sorted(NEW_FILES):
            try:
                (root / relative).unlink()
            except FileNotFoundError:
                pass
        for relative in sorted(set(EXPECTED_BLOBS)):
            source = backup_dir / relative
            if source.is_file():
                target = root / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source, target)
        raise
    print(f"upgrade: APPLIED\nbackup: {backup_dir}")
    print("validation: py_compile PASS")
    if not args.run_tests:
        print("recommended: ./r test && ./r test full (when shared-boundary validation is desired)")
        print("targeted: python3 -m pytest -q tests/feature/test_agent_core.py tests/feature/test_execution_mode_selector.py tests/feature/test_execution_mode_executor.py tests/core/test_agent_config_tools.py tests/feature/test_tools_diag_evidence.py")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
