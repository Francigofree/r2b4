#!/usr/bin/env python3
from __future__ import annotations

import shutil
import sys
from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parent
BACKUP_DIR = PACKAGE_DIR / "backup"
TARGETS = (
    "v3/layers/l6_navigation.py",
    "v3/composition/native_control.py",
    "v3/composition/resident_live_control.py",
    "v3/composition/resident_physical_control.py",
    "v3_runtime.py",
    "v3_hardware_runtime.py",
    "v3_bounded_config.py",
    "v3/replay.py",
    "tests/v3_validation_helpers.py",
    "conf/vezerles.json",
    "STRUKTURALIS_RETEGEK_V3.md",
    "v3/adapters/l6_planner_process.py",
    "tests/test_v3_async_l6_planner.py",
)


def main() -> int:
    repo = Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
    if not BACKUP_DIR.is_dir():
        raise RuntimeError(
            f"backup directory does not exist: {BACKUP_DIR}; "
            "rollback is available only after upgrade.py created its backup"
        )
    restored = 0
    removed = 0
    for relative in TARGETS:
        source = BACKUP_DIR / relative
        absent = source.with_suffix(source.suffix + ".ABSENT")
        target = repo / relative
        if source.is_file():
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
            restored += 1
        elif absent.is_file():
            if target.exists():
                target.unlink()
                removed += 1
        else:
            raise RuntimeError(f"missing backup marker for {relative}")
    print(f"rollback complete: restored={restored}, removed_new={removed}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
