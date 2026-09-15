#!/usr/bin/env python3
from __future__ import annotations

import os
import shutil
import sys
from pathlib import Path

target = Path(sys.argv[1] if len(sys.argv) > 1 else "r2b4")

old_1 = r'''
    previous_yaw = yaw(initial) if angle else 0.0
'''
new_1 = r'''
    v, omega = target(0.0)
    previous_yaw = yaw(initial) if angle else 0.0
'''

old_2 = r'''
            # Slow down near the requested angle; wheel ratios stay unchanged.
            scale = min(1.0, max(0.15, remaining / math.radians(25)))
'''

old_3 = r'''
            scale = 1.0
'''

old_4 = r'''
        v, omega = target(min(1.0, elapsed / 4.9))
        client.publish_teleop(f"{command_id}-{index}", v_mps=v * scale,
                             omega_rad_s=omega * scale, max_v_mps=0.25,
'''
new_4 = r'''
        client.publish_teleop(f"{command_id}-{index}", v_mps=v,
                             omega_rad_s=omega, max_v_mps=0.25,
'''

if not target.is_file():
    raise SystemExit(f"ERROR: launcher not found: {target}")

text = target.read_text(encoding="utf-8")

already_fixed = (
    "    v, omega = target(0.0)\n" in text
    and 'client.publish_teleop(f"{command_id}-{index}", v_mps=v,\n' in text
    and "v_mps=v * scale" not in text
)

if already_fixed:
    print(f"OK: {target} already contains the proba fix.")
    raise SystemExit(0)

checks = {
    "phase anchor": old_1,
    "turn slowdown": old_2,
    "non-turn scale": old_3,
    "dynamic target publish": old_4,
}
missing = [name for name, needle in checks.items() if text.count(needle) != 1]
if missing:
    print("ERROR: launcher differs from the expected version; nothing was changed.", file=sys.stderr)
    print("Unmatched/non-unique blocks: " + ", ".join(missing), file=sys.stderr)
    raise SystemExit(2)

backup = target.with_name(target.name + ".before_proba_fix")
if backup.exists():
    print(f"ERROR: backup already exists: {backup}", file=sys.stderr)
    print("Rename/remove that backup first, then run again.", file=sys.stderr)
    raise SystemExit(3)

shutil.copy2(target, backup)

fixed = text.replace(old_1, new_1, 1)
fixed = fixed.replace(old_2, "", 1)
fixed = fixed.replace(old_3, "", 1)
fixed = fixed.replace(old_4, new_4, 1)

target.write_text(fixed, encoding="utf-8")
os.chmod(target, os.stat(target).st_mode | 0o111)

print(f"OK: patched {target}")
print(f"backup: {backup}")
print("The proba phase now keeps one fixed (v, omega) target per command_id.")
