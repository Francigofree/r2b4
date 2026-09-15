#!/usr/bin/env python3
"""Apply R2B4 Camera Integration Slice 1 to a current V3 checkout.

Expected source baseline: main around commit 0968943d (2026-09-15). The patcher
uses structural text anchors and refuses partial application if an expected
anchor is missing.
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent
OVERLAY = PACKAGE_ROOT / "overlay"


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly one anchor, found {count}")
    return text.replace(old, new, 1)


def patch_l12(text: str) -> str:
    if "critical_device_ids: frozenset[str] | None = None" in text:
        return text
    old = '''    def __init__(\n        self,\n        writer: MotorWriter,\n        lidar: LidarSafetyConfig | None = None,\n    ) -> None:\n        if lidar is not None and not isinstance(lidar, LidarSafetyConfig):\n            raise TypeError("lidar must be LidarSafetyConfig or None")\n        self._writer = writer\n        self._lidar = lidar\n        self._fault_latched = False\n'''
    new = '''    def __init__(\n        self,\n        writer: MotorWriter,\n        lidar: LidarSafetyConfig | None = None,\n        critical_device_ids: frozenset[str] | None = None,\n    ) -> None:\n        if lidar is not None and not isinstance(lidar, LidarSafetyConfig):\n            raise TypeError("lidar must be LidarSafetyConfig or None")\n        if critical_device_ids is not None:\n            if not isinstance(critical_device_ids, frozenset):\n                raise TypeError("critical_device_ids must be frozenset[str] or None")\n            if not critical_device_ids:\n                raise ValueError("critical_device_ids cannot be empty")\n            if any(\n                not isinstance(device_id, str) or not device_id.strip()\n                for device_id in critical_device_ids\n            ):\n                raise ValueError("critical_device_ids must contain non-empty strings")\n        self._writer = writer\n        self._lidar = lidar\n        self._critical_device_ids = critical_device_ids\n        self._fault_latched = False\n'''
    text = replace_once(text, old, new, "l12 constructor")
    old2 = '''        failed_device = next(\n            (item for item in critical_health if item.state is DeviceHealthState.FAILED),\n            None,\n        )\n'''
    new2 = '''        if self._critical_device_ids is not None:\n            health_by_id = {item.device_id: item for item in critical_health}\n            if any(\n                device_id not in health_by_id\n                for device_id in self._critical_device_ids\n            ):\n                command = self._stop(\n                    context,\n                    SafetyDecision.STOP,\n                    "CRITICAL_DEVICE_HEALTH_MISSING",\n                )\n                try:\n                    self._writer.write(command)\n                except Exception as exc:\n                    self._fault_latched = True\n                    raise MotorWriteError(\n                        "the single final motor write failed",\n                        command,\n                    ) from exc\n                return command\n            critical_health = tuple(\n                item\n                for item in critical_health\n                if item.device_id in self._critical_device_ids\n            )\n\n        failed_device = next(\n            (item for item in critical_health if item.state is DeviceHealthState.FAILED),\n            None,\n        )\n'''
    return replace_once(text, old2, new2, "l12 critical-health filter")


def patch_native_control(text: str) -> str:
    if "critical_device_ids: frozenset[str] | None = None" not in text:
        old = '''    lidar_safety: LidarSafetyConfig | None = None\n\n    def __post_init__(self) -> None:\n'''
        new = '''    lidar_safety: LidarSafetyConfig | None = None\n    critical_device_ids: frozenset[str] | None = None\n\n    def __post_init__(self) -> None:\n'''
        text = replace_once(text, old, new, "native control config field")
        old2 = '''        if self.lidar_safety is not None and not isinstance(\n            self.lidar_safety,\n            LidarSafetyConfig,\n        ):\n            raise TypeError("lidar_safety must be LidarSafetyConfig or None")\n'''
        new2 = '''        if self.lidar_safety is not None and not isinstance(\n            self.lidar_safety,\n            LidarSafetyConfig,\n        ):\n            raise TypeError("lidar_safety must be LidarSafetyConfig or None")\n        if self.critical_device_ids is not None:\n            if not isinstance(self.critical_device_ids, frozenset):\n                raise TypeError("critical_device_ids must be frozenset[str] or None")\n            if not self.critical_device_ids:\n                raise ValueError("critical_device_ids cannot be empty")\n            if any(\n                not isinstance(device_id, str) or not device_id.strip()\n                for device_id in self.critical_device_ids\n            ):\n                raise ValueError("critical_device_ids must contain non-empty strings")\n'''
        text = replace_once(text, old2, new2, "native control config validation")
    old3 = "        final_safety = FinalSafetyGate(motor_writer, config.lidar_safety)\n"
    new3 = '''        final_safety = FinalSafetyGate(\n            motor_writer,\n            config.lidar_safety,\n            critical_device_ids=config.critical_device_ids,\n        )\n'''
    if old3 in text:
        text = replace_once(text, old3, new3, "native control final safety wiring")
    elif "critical_device_ids=config.critical_device_ids" not in text:
        raise RuntimeError("native control final safety wiring: anchor missing")
    return text


def patch_bounded_config(text: str) -> str:
    marker = 'critical_device_ids=frozenset({"WHEEL_ENCODERS", "BNO055_IMU", "RPLIDAR_C1"}),'
    if marker in text:
        return text
    old = '''        chassis_control=ChassisControlConfig(track_width_m=track_width_m),\n        lidar_safety=lidar_safety,\n        **navigation_kwargs,\n'''
    new = '''        chassis_control=ChassisControlConfig(track_width_m=track_width_m),\n        lidar_safety=lidar_safety,\n        critical_device_ids=frozenset(\n            {"WHEEL_ENCODERS", "BNO055_IMU", "RPLIDAR_C1"}\n        ),\n        **navigation_kwargs,\n'''
    return replace_once(text, old, new, "bounded production critical-device policy")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("repo", nargs="?", default=".")
    args = parser.parse_args()
    repo = Path(args.repo).resolve()
    if not (repo / "v3").is_dir():
        raise SystemExit(f"not an R2B4 checkout: {repo}")

    patches = {
        repo / "v3/layers/l12_safety_final.py": patch_l12,
        repo / "v3/composition/native_control.py": patch_native_control,
        repo / "v3_bounded_config.py": patch_bounded_config,
    }
    prepared: dict[Path, str] = {}
    for path, transform in patches.items():
        prepared[path] = transform(path.read_text(encoding="utf-8"))

    # Only write after every source anchor has validated.
    for path, content in prepared.items():
        path.write_text(content, encoding="utf-8")

    for relative in (
        Path("v3/adapters/picamera2_camera.py"),
        Path("v3/adapters/live_camera.py"),
        Path("tests/test_v3_camera_foundation.py"),
    ):
        source = OVERLAY / relative
        target = repo / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)

    print("R2B4 camera slice 1 applied")
    print("Run: python3 -m pytest -q tests/test_v3_camera_foundation.py")
    print("Then run the normal targeted V3 regression gates before live use.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
