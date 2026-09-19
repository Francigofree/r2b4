#!/usr/bin/env python3
"""Install R2B4 Gemini P2 conversation upgrade.

Safety policy:
- only files modified by this upgrade have source precondition checks;
- there is no full-repository SHA gate;
- no motor command or live motion test is executed;
- conf/.wake.env is never modified or printed.
"""
from __future__ import annotations

import hashlib
import shutil
import subprocess
import sys
import time
from pathlib import Path

BASE_COMMIT = "b5d39c46fd987ec8585567fb6ba7d76b1164140d"

# Git blob SHA from the source-first main snapshot. Only files this upgrade
# modifies are checked. New files are handled separately below.
MODIFIED_BASE_BLOBS = {
    "r2b4_voice/conversation_contracts.py": "c8423454db9a55bf2d2fceb79ef902862b29f742",
    "r2b4_voice/robot_context.py": "3c624984c0a2156da7cd5d579820067238a0d38f",
    "r2b4_voice/prompting.py": "cccea82dd04ebe30c387c51da6451448f685c929",
    "r2b4_voice/groq_llm.py": "8a3ddc65313113a3dc097526574f31cf1877897a",
    "r2b4_voice/conversation_interface.py": "1330ab2ca97c034a83b844e78ec66213d5747dc3",
    "r2b4_voice/conversation_cli.py": "968bcb33c5cb48f6540c6a6f9a608e55707d13de",
    "conf/voice_llm_system.md": "aa9f602af24b3d6af46f967e4a908db0d2854a20",
}

NEW_FILES = (
    "r2b4_voice/llm_decision.py",
    "r2b4_voice/gemini_llm.py",
    "r2b4_voice/llm_provider.py",
    "tests/test_r2b4_gemini_llm.py",
    "tests/test_r2b4_llm_provider.py",
    "tests/test_r2b4_robot_context_v2.py",
    "tests/test_r2b4_prompting_v2.py",
)

TARGETED_TESTS = (
    "tests/test_r2b4_conversation_contracts.py",
    "tests/test_r2b4_conversation_interface.py",
    "tests/test_r2b4_conversation_service.py",
    "tests/test_r2b4_groq_llm.py",
    "tests/test_r2b4_prompting.py",
    "tests/test_r2b4_robot_context.py",
    "tests/test_r2b4_gemini_llm.py",
    "tests/test_r2b4_llm_provider.py",
    "tests/test_r2b4_robot_context_v2.py",
    "tests/test_r2b4_prompting_v2.py",
)


def git_blob_sha(path: Path) -> str:
    data = path.read_bytes()
    header = f"blob {len(data)}\0".encode("ascii")
    return hashlib.sha1(header + data).hexdigest()


def fail(message: str) -> "NoReturn":
    print(f"ERROR: {message}", file=sys.stderr)
    raise SystemExit(2)


def main() -> int:
    package_root = Path(__file__).resolve().parent
    payload = package_root / "payload"
    repo = package_root.parent
    if not (repo / "v3").is_dir() or not (repo / "r2b4_voice").is_dir():
        fail(f"repo root not detected at {repo}")
    if not payload.is_dir():
        fail(f"payload missing: {payload}")

    print(f"repo: {repo}")
    print(f"source base: {BASE_COMMIT}")
    print("precondition: modified files only (no full-repo SHA check)")

    # Verify every modified target is either exactly the source-first base or
    # already equal to this payload (idempotent reinstall).
    for rel, expected_base in MODIFIED_BASE_BLOBS.items():
        src = payload / rel
        dst = repo / rel
        if not src.is_file():
            fail(f"payload file missing: {rel}")
        if not dst.is_file():
            fail(f"target file missing: {rel}")
        current = git_blob_sha(dst)
        desired = git_blob_sha(src)
        if current not in {expected_base, desired}:
            fail(
                f"precondition mismatch for {rel}\n"
                f"  expected base: {expected_base}\n"
                f"  current:       {current}\n"
                "Refusing to overwrite a locally changed file."
            )

    # New files may be absent or already equal to the payload. Never silently
    # replace an unrelated local implementation.
    for rel in NEW_FILES:
        src = payload / rel
        dst = repo / rel
        if not src.is_file():
            fail(f"payload file missing: {rel}")
        if dst.exists() and (not dst.is_file() or git_blob_sha(dst) != git_blob_sha(src)):
            fail(f"new-file collision: {rel} already exists with different content")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    backup_root = repo / "runtime" / "upgrade_backups" / f"gemini_p2_{stamp}"
    backup_root.mkdir(parents=True, exist_ok=True)

    installed: list[str] = []
    for rel in (*MODIFIED_BASE_BLOBS.keys(), *NEW_FILES):
        src = payload / rel
        dst = repo / rel
        if rel in MODIFIED_BASE_BLOBS and dst.exists() and git_blob_sha(dst) != git_blob_sha(src):
            backup = backup_root / rel
            backup.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(dst, backup)
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        installed.append(rel)
        print(f"installed: {rel}")

    print("running targeted pytest...")
    command = [sys.executable, "-m", "pytest", "-q", *TARGETED_TESTS]
    result = subprocess.run(command, cwd=repo)
    if result.returncode != 0:
        print(f"FAIL: targeted pytest returned {result.returncode}", file=sys.stderr)
        print(f"backup: {backup_root}", file=sys.stderr)
        return result.returncode

    print()
    print("PASS: Gemini P2 conversation slice installed")
    print("- default LLM provider: gemini")
    print("- default model: gemini-3.8-flash")
    print("- Gemini Interactions API: stateless store=false")
    print("- robot intents remain SHADOW-only; installer executed no motor action")
    print("- Groq STT and Groq LLM fallback remain available")
    print()
    print("Gemini secret: add GEMINI_API_KEY=... to /home/alba/project_r2b4/conf/.wake.env (keep chmod 600).")
    print("Optional fallback: --provider groq")
    print()
    print("motor-free check:")
    print("  cd /home/alba/project_r2b4 && python3 -m r2b4_voice.conversation_cli --check")
    print("motor-free Gemini test:")
    print('  cd /home/alba/project_r2b4 && python3 -m r2b4_voice.conversation_cli --text "Szia Alba. Milyen állapotban vagy?" --wait 25')
    print("futtasd a full pytest-et:")
    print("  cd /home/alba/project_r2b4 && python3 -m pytest -q")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
