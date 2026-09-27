#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

UPGRADE_NAME = "system_behavior_contract_p0_v1"
MARKER = "R2B4_SYSTEM_BEHAVIOR_P0_V1"
EXPECTED_BLOBS = {
    "r2b4_voice/voice_service.py": "401912c790ddf03436ae3bd3de0ecbc557aa8e86",
    "r2b4_voice/conversation_service.py": "c8e1db25eac6059f320b9d8a65c42a536e05a4c9",
    "r2b4_voice/conversation_contracts.py": "6c3b6e4372aad2fb4dadb7bd58bc3c9ea2a953bc",
    "r2b4_voice/llm_decision.py": "24a7aed66aaab6442bd3b7970d49925aefed7358",
    "r2b4_voice/gemini_llm.py": "1746a92685ed3355a2a80d0ee1b002e1bef57e30",
    "r2b4_voice/groq_llm.py": "b64ace055e395d4c7fc1ce63ea83a744689731e2",
    "r2b4_voice/robot_context.py": "34d664fdc5caca87d9bc13cf92f81a6fa0d77ff1",
    "r2b4_voice/prompting.py": "80304c92f2579159a677b501498979756ddf6a6c",
    "conf/voice_llm_system.md": "c8197cb003d70c35b78938957ede99054a52f97b",
    "r2b4_voice/action_executor.py": "4330df88a21f143d4912e1bbd3f72b6c32cfe870",
    "r2b4_voice/wake_core.py": "d49fcc5e58a3fa0e0902864aa088614e6cf81d39",
    "v3/adapters/camera.py": "641b7074a9bb3224ccd98f2f6ad331aef569895d",
    "v3/adapters/camera_media.py": "44b11c2b8c80c297052d75cc6b64d7c3d70fc113"
}
NEW_FILES = (
    "r2b4_voice/execution_mode.py",
    "r2b4_voice/observation_executor.py",
    "tests/feature/test_system_behavior_contract_p0.py",
)


def run(repo: Path, *args: str) -> None:
    subprocess.run(args, cwd=repo, check=True)


def output(repo: Path, *args: str) -> str:
    return subprocess.check_output(args, cwd=repo, text=True).strip()


def verify(repo: Path) -> None:
    if not (repo / "R2B4_SYSTEM_BEHAVIOR_CONTRACT.md").is_file():
        raise RuntimeError("not an R2B4 repository root")
    for rel, expected in EXPECTED_BLOBS.items():
        path = repo / rel
        if not path.is_file():
            raise RuntimeError(f"missing target: {rel}")
        actual = output(repo, "git", "hash-object", rel)
        if actual != expected:
            raise RuntimeError(
                f"{rel}: source precondition mismatch; expected {expected}, got {actual}. "
                "Do not force this upgrade onto a newer/different source tree."
            )
    for rel in NEW_FILES:
        if (repo / rel).exists():
            raise RuntimeError(f"new P0 file already exists: {rel}")


def backup(repo: Path) -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    root = repo / ".upgrade_backups" / f"{UPGRADE_NAME}_{stamp}"
    for rel in EXPECTED_BLOBS:
        src = repo / rel
        dst = root / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
    return root


def restore(repo: Path, backup_root: Path) -> None:
    for rel in EXPECTED_BLOBS:
        src = backup_root / rel
        if src.is_file():
            dst = repo / rel
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)
    for rel in NEW_FILES:
        try:
            (repo / rel).unlink()
        except FileNotFoundError:
            pass


def apply_operation(text: str, op: dict[str, object], rel: str) -> str:
    kind = op["type"]
    label = str(op.get("label") or kind)
    if kind == "replace":
        old = str(op["old"])
        new = str(op["new"])
        count = text.count(old)
        if count != 1:
            raise RuntimeError(f"{rel}:{label}: expected one match, found {count}")
        return text.replace(old, new, 1)
    if kind == "between":
        start = str(op["start"])
        end = str(op["end"])
        replacement = str(op["replacement"])
        first = text.find(start)
        if first < 0:
            raise RuntimeError(f"{rel}:{label}: start marker not found")
        second = text.find(end, first + len(start))
        if second < 0:
            raise RuntimeError(f"{rel}:{label}: end marker not found")
        return text[:first] + replacement + text[second:]
    if kind == "full":
        return str(op["content"])
    raise RuntimeError(f"{rel}:{label}: unsupported patch type {kind!r}")


def apply_patches(repo: Path, package_root: Path) -> None:
    patches = json.loads((package_root / "patches.json").read_text(encoding="utf-8"))
    if set(patches) != set(EXPECTED_BLOBS):
        raise RuntimeError("patch manifest does not match expected target set")
    for rel, operations in patches.items():
        path = repo / rel
        text = path.read_text(encoding="utf-8")
        for op in operations:
            text = apply_operation(text, op, rel)
        path.write_text(text, encoding="utf-8")
        print(f"patched: {rel}")

    for rel in NEW_FILES:
        src = package_root / "payload" / rel
        if not src.is_file():
            raise RuntimeError(f"payload missing: {rel}")
        dst = repo / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        print(f"added: {rel}")


def validate(repo: Path, *, run_tests: bool) -> None:
    changed_python = [
        rel for rel in (*EXPECTED_BLOBS.keys(), *NEW_FILES)
        if rel.endswith(".py")
    ]
    run(repo, sys.executable, "-m", "py_compile", *changed_python)
    print("compile: PASS")
    if not run_tests:
        print("tests: SKIPPED by --no-tests")
        return
    run(
        repo,
        sys.executable,
        "-m",
        "pytest",
        "-q",
        "tests/feature/test_system_behavior_contract_p0.py",
    )
    print("P0 targeted tests: PASS")
    launcher = repo / "r"
    if launcher.is_file():
        run(repo, str(launcher), "test")
        print("CORE tests: PASS")
        run(repo, str(launcher), "test", "feature")
        print("FEATURE tests: PASS")


def main() -> int:
    parser = argparse.ArgumentParser(description="Install R2B4 system behavior P0 upgrade")
    parser.add_argument(
        "--repo",
        type=Path,
        default=Path("/home/alba/project_r2b4"),
        help="R2B4 repository root",
    )
    parser.add_argument(
        "--no-tests",
        action="store_true",
        help="compile only; skip targeted/CORE tests (not recommended)",
    )
    args = parser.parse_args()
    repo = args.repo.expanduser().resolve()
    package_root = Path(__file__).resolve().parent

    verify(repo)
    backup_root = backup(repo)
    print(f"backup: {backup_root}")
    try:
        apply_patches(repo, package_root)
        if MARKER not in (repo / "r2b4_voice/voice_service.py").read_text(encoding="utf-8"):
            raise RuntimeError("post-install P0 marker missing")
        validate(repo, run_tests=not args.no_tests)
    except Exception:
        print("upgrade failed; restoring backup", file=sys.stderr)
        restore(repo, backup_root)
        raise

    receipt = {
        "upgrade": UPGRADE_NAME,
        "installed_at_unix": time.time(),
        "backup": str(backup_root),
        "tests_skipped": bool(args.no_tests),
        "full_repo_sha_gate": False,
        "source_preconditions": "per-file git blob SHA",
        "changed": list(EXPECTED_BLOBS),
        "added": list(NEW_FILES),
    }
    receipt_path = repo / "runtime" / f"{UPGRADE_NAME}_receipt.json"
    receipt_path.parent.mkdir(parents=True, exist_ok=True)
    receipt_path.write_text(
        json.dumps(receipt, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"receipt: {receipt_path}")
    print("R2B4 system behavior P0 installed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
