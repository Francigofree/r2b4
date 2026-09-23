#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(sys.argv[1] if len(sys.argv) > 1 else "/home/alba/project_r2b4").resolve()

def patch_once(path: Path, old: str, new: str) -> None:
    text = path.read_text(encoding="utf-8")
    if new in text:
        print(f"OK already fixed: {path}")
        return
    if old not in text:
        raise RuntimeError(f"anchor not found: {path}")
    path.write_text(text.replace(old, new, 1), encoding="utf-8")
    print(f"FIXED: {path}")

runtime = ROOT / "v3_runtime.py"
patch_once(
    runtime,
    '''    record_observer: Callable[[CaptureRecord], None] | None = None,
    timing_enabled: bool = False,
    trajectory_rollout_backend: object | None = None,
    enable_multirate_inputs: bool = True,
''',
    '''    record_observer: Callable[[CaptureRecord], None] | None = None,
    record_observer_hz: int = CONTROL_CAPTURE_HZ,
    timing_enabled: bool = False,
    trajectory_rollout_backend: object | None = None,
    enable_multirate_inputs: bool = True,
''',
)

operator = ROOT / "v3" / "operator_controller.py"
patch_once(
    operator,
    '''        if not self._wait_fresh_ready(pid, baseline, timeout=8.0):
''',
    '''        if not self._wait_fresh_ready(pid, baseline, timeout=20.0):
''',
)

for path in (runtime, operator):
    compile(path.read_text(encoding="utf-8"), str(path), "exec")

print("PASS: syntax")
print("Fixed:")
print("  - v3_runtime.py: owned runtime accepts record_observer_hz")
print("  - v3/operator_controller.py: cold-start ready timeout 8s -> 20s")
print("Next live check: r rc")
