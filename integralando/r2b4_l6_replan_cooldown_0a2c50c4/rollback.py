#!/usr/bin/env python3
from __future__ import annotations

import os
import sys
from pathlib import Path

repo = Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
target = repo / "v3/layers/l6_navigation.py"
backup = target.with_suffix(target.suffix + ".pre_replan_cooldown")

if not backup.is_file():
    raise SystemExit(f"backup not found: {backup}")

tmp = target.with_suffix(target.suffix + f".rollback.{os.getpid()}")
tmp.write_bytes(backup.read_bytes())
os.replace(tmp, target)
print("rollback complete: v3/layers/l6_navigation.py")
