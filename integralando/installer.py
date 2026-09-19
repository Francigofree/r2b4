#!/usr/bin/env python3
"""Install the R2B4 LLM-P0/P1 conversation upgrade into the current repo."""

from __future__ import annotations

import shutil
import subprocess
import sys
import time
from pathlib import Path


TARGETED_TESTS = [
    "tests/test_r2b4_conversation_contracts.py",
    "tests/test_r2b4_robot_context.py",
    "tests/test_r2b4_groq_llm.py",
    "tests/test_r2b4_conversation_service.py",
    "tests/test_r2b4_conversation_interface.py",
    "tests/test_r2b4_prompting.py",
]


def _repo_root(installer: Path) -> Path:
    here = installer.resolve().parent
    candidates = [here.parent, Path("/home/alba/project_r2b4")]
    for candidate in candidates:
        if (candidate / "STRUKTURALIS_RETEGEK_V3.md").is_file() and (candidate / "v3" / "robot_interface.py").is_file():
            return candidate.resolve()
    raise SystemExit("ERROR: R2B4 repo root not found")


def _preflight(root: Path) -> None:
    required = [
        root / "v3" / "robot_interface.py",
        root / "v3" / "interface_adapters.py",
        root / "v3" / "operator_controller.py",
        root / "r2b4_voice" / "groq_stt.py",
        root / "conf",
    ]
    missing = [str(path) for path in required if not path.exists()]
    if missing:
        raise SystemExit("ERROR: required current-repo files missing:\n  " + "\n  ".join(missing))


def _install_payload(root: Path, payload: Path) -> tuple[int, Path | None]:
    files = sorted(path for path in payload.rglob("*") if path.is_file())
    backup_root: Path | None = None
    changed = 0
    stamp = time.strftime("%Y%m%d_%H%M%S")
    for src in files:
        rel = src.relative_to(payload)
        dst = root / rel
        if dst.exists() and dst.read_bytes() == src.read_bytes():
            print(f"unchanged: {rel}")
            continue
        if dst.exists():
            if backup_root is None:
                backup_root = root / "runtime" / "upgrade_backups" / f"llm_p01_{stamp}"
            backup = backup_root / rel
            backup.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(dst, backup)
            print(f"backup:    {rel}")
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        print(f"installed: {rel}")
        changed += 1
    return changed, backup_root


def _run_tests(root: Path) -> None:
    cmd = [sys.executable, "-m", "pytest", "-q", *TARGETED_TESTS]
    print("running targeted pytest...")
    subprocess.run(cmd, cwd=root, check=True)


def main() -> int:
    installer = Path(__file__).resolve()
    payload = installer.parent / "payload"
    if not payload.is_dir():
        raise SystemExit("ERROR: payload directory missing")
    root = _repo_root(installer)
    print(f"repo: {root}")
    _preflight(root)
    changed, backup = _install_payload(root, payload)
    _run_tests(root)

    print()
    print("PASS: R2B4 LLM-P0/P1 conversation upgrade installed")
    print(f"files changed/installed: {changed}")
    if backup is not None:
        print(f"backup: {backup}")
    print("action mode: SHADOW (LLM robot intents are validated+journaled, NEVER executed)")
    print("TTS: not wired in this slice")
    print()
    print("motor-free local check:")
    print(f"  cd {root}")
    print("  python3 -m r2b4_voice.conversation_cli --check")
    print("motor-free live LLM text test (contacts Groq; intents remain SHADOW):")
    print(f"  cd {root}")
    print('  python3 -m r2b4_voice.conversation_cli --text "Szia Alba. Milyen állapotban vagy?" --wait 25')
    print()
    print("futtasd a full pytest-et:")
    print(f"  cd {root} && python3 -m pytest -q")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
