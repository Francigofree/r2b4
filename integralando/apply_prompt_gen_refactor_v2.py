#!/usr/bin/env python3
"""Apply R2B4 PROMPT-GEN refactor V2 from this bundle.

Scope is intentionally limited to:
- conf/r2b4_agent_system.md
- r2b4_voice/prompting.py
- tests/core/test_agent_prompt_hierarchy.py
- docs/R2B4_AGENT_CORE.md

No Gemini transport, ER2, V3 action, safety or camera source is modified.
"""
from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import time
from pathlib import Path

FILES = (
    "conf/r2b4_agent_system.md",
    "r2b4_voice/prompting.py",
    "tests/core/test_agent_prompt_hierarchy.py",
    "docs/R2B4_AGENT_CORE.md",
)

def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--root", default=".", help="R2B4 repository root")
    p.add_argument("--skip-tests", action="store_true")
    args = p.parse_args()

    root = Path(args.root).expanduser().resolve()
    bundle = Path(__file__).resolve().parent
    marker = root / "r2b4_voice" / "prompting.py"
    if not marker.is_file():
        print(f"ERROR: not an R2B4 repo root: {root}", file=sys.stderr)
        return 2

    current_prompt = root / "conf" / "r2b4_agent_system.md"
    if current_prompt.is_file():
        head = current_prompt.read_text(encoding="utf-8").splitlines()[0].strip()
        if head not in {"R2B4_AGENT_SYSTEM_V1", "R2B4_AGENT_SYSTEM_V2"}:
            print(f"ERROR: unexpected system prompt header: {head}", file=sys.stderr)
            return 3

    stamp = time.strftime("%Y%m%d_%H%M%S")
    backup = root / ".upgrade_backups" / f"prompt_gen_v2_{stamp}"
    for rel in FILES:
        src = bundle / rel
        if not src.is_file():
            print(f"ERROR: bundle missing {rel}", file=sys.stderr)
            return 4
        dst = root / rel
        if dst.exists():
            b = backup / rel
            b.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(dst, b)

    for rel in FILES:
        src = bundle / rel
        dst = root / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        print(f"UPDATED {rel}")

    print(f"BACKUP {backup}")

    if args.skip_tests:
        return 0

    cmd = [sys.executable, "-m", "pytest", "-q",
           "tests/core/test_agent_prompt_hierarchy.py",
           "tests/core/test_gemini_structured_transport.py"]
    print("RUN", " ".join(cmd))
    return subprocess.run(cmd, cwd=root).returncode

if __name__ == "__main__":
    raise SystemExit(main())
