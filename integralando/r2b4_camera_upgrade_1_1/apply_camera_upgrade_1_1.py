#!/usr/bin/env python3
"""Apply R2B4 camera foundation hardening upgrade 1.1.

Inspected baseline: Francigofree/r2b4 main 4945efa26646ab9d017cf357b857584177a5ff1c.
The patcher is structural and aborts before writing when an expected anchor is
missing.  It does not start hardware, camera or motors.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent
OVERLAY = PACKAGE_ROOT / "overlay"
EXPECTED_BASE = "4945efa26646ab9d017cf357b857584177a5ff1c"


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly one anchor, found {count}")
    return text.replace(old, new, 1)


def add_import_before(text: str, anchor: str, import_text: str, label: str) -> str:
    if import_text.strip() in text:
        return text
    return replace_once(text, anchor, import_text + anchor, label)


def patch_l12(text: str) -> str:
    text = add_import_before(
        text,
        "from v3.ports import MotorWriter\n",
        "from v3.device_health_policy import critical_device_health_view\n",
        "l12 health-policy import",
    )
    old = '''        if self._critical_device_ids is not None:\n            health_by_id = {item.device_id: item for item in critical_health}\n            if any(\n                device_id not in health_by_id\n                for device_id in self._critical_device_ids\n            ):\n                command = self._stop(\n                    context,\n                    SafetyDecision.STOP,\n                    "CRITICAL_DEVICE_HEALTH_MISSING",\n                )\n                try:\n                    self._writer.write(command)\n                except Exception as exc:\n                    self._fault_latched = True\n                    raise MotorWriteError(\n                        "the single final motor write failed",\n                        command,\n                    ) from exc\n                return command\n            critical_health = tuple(\n                item\n                for item in critical_health\n                if item.device_id in self._critical_device_ids\n            )\n\n        failed_device = next(\n'''
    new = '''        health_view = critical_device_health_view(\n            critical_health,\n            self._critical_device_ids,\n        )\n        critical_health = health_view.critical_health\n        missing_critical_health = bool(health_view.missing_device_ids)\n\n        failed_device = next(\n'''
    if old in text:
        text = replace_once(text, old, new, "l12 remove early missing-health return")
    elif "missing_critical_health = bool(health_view.missing_device_ids)" not in text:
        raise RuntimeError("l12 early-return anchor missing")
    old2 = '''        elif self._fault_latched:\n            command = self._stop(context, SafetyDecision.FAULT, "FAULT_LATCHED")\n        elif unknown_device is not None:\n'''
    new2 = '''        elif self._fault_latched:\n            command = self._stop(context, SafetyDecision.FAULT, "FAULT_LATCHED")\n        elif missing_critical_health:\n            command = self._stop(\n                context,\n                SafetyDecision.STOP,\n                "CRITICAL_DEVICE_HEALTH_MISSING",\n            )\n        elif unknown_device is not None:\n'''
    if old2 in text:
        text = replace_once(text, old2, new2, "l12 fault-priority ordering")
    elif 'elif missing_critical_health:' not in text:
        raise RuntimeError("l12 priority anchor missing")
    return text


def patch_bounded(text: str) -> str:
    text = add_import_before(
        text,
        "from v3.engine import TickExecutionError, TickInputs, TickResult\n",
        "from v3.device_health_policy import (\n"
        "    blocking_degraded_sources,\n"
        "    critical_devices_ready,\n"
        ")\n",
        "bounded health-policy import",
    )
    old = '''    @staticmethod\n    def _is_healthy_preflight(\n        batch_health: tuple[DeviceHealth, ...],\n        result: TickResult,\n    ) -> bool:\n'''
    new = '''    @staticmethod\n    def _is_healthy_preflight(\n        batch_health: tuple[DeviceHealth, ...],\n        result: TickResult,\n        critical_device_ids: frozenset[str] | None,\n    ) -> bool:\n'''
    if old in text:
        text = replace_once(text, old, new, "bounded preflight signature")
    old2 = '''            bool(batch_health)\n            and all(item.state is DeviceHealthState.OK for item in batch_health)\n            and isinstance(admission, AdmittedFrame)\n            and not admission.degraded_sources\n'''
    new2 = '''            critical_devices_ready(batch_health, critical_device_ids)\n            and isinstance(admission, AdmittedFrame)\n            and not blocking_degraded_sources(\n                admission.degraded_sources,\n                critical_device_ids,\n            )\n'''
    if old2 in text:
        text = replace_once(text, old2, new2, "bounded preflight policy")
    old3 = '''                and self._is_healthy_preflight(batch.device_health, result)\n'''
    new3 = '''                and self._is_healthy_preflight(\n                    batch.device_health,\n                    result,\n                    self._config.control.critical_device_ids,\n                )\n'''
    if old3 in text:
        text = replace_once(text, old3, new3, "bounded preflight call")
    if "critical_devices_ready(batch_health, critical_device_ids)" not in text:
        raise RuntimeError("bounded preflight policy was not applied")
    return text


def patch_resident(text: str) -> str:
    text = add_import_before(
        text,
        "from v3.engine import TickExecutionError, TickInputs, TickResult\n",
        "from v3.device_health_policy import critical_devices_ready\n",
        "resident health-policy import",
    )
    old = '''    @staticmethod\n    def _is_healthy_idle(\n        batch_health: tuple[DeviceHealth, ...],\n        result: TickResult,\n    ) -> bool:\n'''
    new = '''    @staticmethod\n    def _is_healthy_idle(\n        batch_health: tuple[DeviceHealth, ...],\n        result: TickResult,\n        critical_device_ids: frozenset[str] | None,\n    ) -> bool:\n'''
    if old in text:
        text = replace_once(text, old, new, "resident preflight signature")
    old2 = '''            bool(batch_health)\n            and all(item.state is DeviceHealthState.OK for item in batch_health)\n            and result.trace.fault_layer is None\n'''
    new2 = '''            critical_devices_ready(batch_health, critical_device_ids)\n            and result.trace.fault_layer is None\n'''
    if old2 in text:
        text = replace_once(text, old2, new2, "resident preflight policy")
    old3 = '''            if not active and self._is_healthy_idle(batch.device_health, result):\n'''
    new3 = '''            if not active and self._is_healthy_idle(\n                batch.device_health,\n                result,\n                self._config.control.critical_device_ids,\n            ):\n'''
    if old3 in text:
        text = replace_once(text, old3, new3, "resident preflight call")
    if "critical_devices_ready(batch_health, critical_device_ids)" not in text:
        raise RuntimeError("resident preflight policy was not applied")
    return text


def patch_replay(text: str) -> str:
    text = add_import_before(
        text,
        "from .engine import LayerValue, TickEngine, TickInputs, TickResult, TickTrace\n",
        "from .device_health_policy import PRODUCTION_CRITICAL_DEVICE_IDS\n",
        "replay health-policy import",
    )
    old = '''        chassis_control=ChassisControlConfig(track_width),\n        lidar_safety=lidar_safety,\n        **navigation_kwargs,\n'''
    new = '''        chassis_control=ChassisControlConfig(track_width),\n        lidar_safety=lidar_safety,\n        critical_device_ids=PRODUCTION_CRITICAL_DEVICE_IDS,\n        **navigation_kwargs,\n'''
    if old in text:
        text = replace_once(text, old, new, "replay compatibility critical policy")
    elif "critical_device_ids=PRODUCTION_CRITICAL_DEVICE_IDS" not in text:
        raise RuntimeError("replay config anchor missing")
    return text


def patch_bounded_config(text: str) -> str:
    text = add_import_before(
        text,
        "from v3.layers.l3_state_estimation import NativeStateEstimatorConfig\n",
        "from v3.device_health_policy import PRODUCTION_CRITICAL_DEVICE_IDS\n",
        "bounded config health-policy import",
    )
    old = '''        critical_device_ids=frozenset(\n            {"WHEEL_ENCODERS", "BNO055_IMU", "RPLIDAR_C1"}\n        ),\n'''
    new = '''        critical_device_ids=PRODUCTION_CRITICAL_DEVICE_IDS,\n'''
    if old in text:
        text = replace_once(text, old, new, "production critical-device SSOT")
    elif "critical_device_ids=PRODUCTION_CRITICAL_DEVICE_IDS" not in text:
        raise RuntimeError("bounded config critical-device anchor missing")
    return text


def patch_camera_owner(text: str) -> str:
    old_start = '''    def start(self) -> bool:\n        with self._lock:\n            if self._running:\n                return True\n            self._last_error = None\n            self._stop_event.clear()\n        camera: Picamera2Device | None = None\n'''
    new_start = '''    def start(self) -> bool:\n        with self._lock:\n            if self._running:\n                return True\n            stale_owner = self._picamera is not None or self._thread is not None\n        if stale_owner:\n            # A previous acquisition thread may have ended after an exception.\n            # Retire its physical handle before a new Picamera2 instance can\n            # replace the reference.\n            self.stop()\n        with self._lock:\n            self._last_error = None\n            self._stop_event.clear()\n        camera: Picamera2Device | None = None\n'''
    if old_start in text:
        text = replace_once(text, old_start, new_start, "camera restart cleanup")
    elif "stale_owner = self._picamera is not None or self._thread is not None" not in text:
        raise RuntimeError("camera start anchor missing")

    old_time = '''        # SensorTimestamp is the first-pixel readout time. Approximate the\n        # first-row exposure midpoint before mapping to the V3 monotonic domain.\n        sensor_measurement_ns = max(0, sensor_timestamp_ns - exposure_time_ns // 2)\n        measurement_monotonic_ns = self._timestamp_mapper(sensor_measurement_ns)\n'''
    new_time = '''        # Keep the documented sensor start-of-frame timestamp as the current\n        # physical reference.  Do not invent an exposure/rolling-shutter shift\n        # before an R2B4 camera-vs-LiDAR timing calibration has validated one.\n        measurement_monotonic_ns = self._timestamp_mapper(sensor_timestamp_ns)\n'''
    if old_time in text:
        text = replace_once(text, old_time, new_time, "camera timestamp semantics")
    elif "Do not invent an exposure/rolling-shutter shift" not in text:
        raise RuntimeError("camera timestamp anchor missing")
    return text


def patch_camera_test(text: str) -> str:
    # The replacement test is copied from overlay; no transform needed.
    return text


def current_head(repo: Path) -> str | None:
    try:
        result = subprocess.run(
            ["git", "-C", str(repo), "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return None
    return result.stdout.strip()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("repo", nargs="?", default=".")
    args = parser.parse_args()
    repo = Path(args.repo).resolve()
    if not (repo / "v3").is_dir():
        raise SystemExit(f"not an R2B4 checkout: {repo}")

    head = current_head(repo)
    if head and head != EXPECTED_BASE:
        print(f"NOTE: HEAD is {head[:12]}, inspected baseline was {EXPECTED_BASE[:12]}; structural anchors will decide compatibility.")

    transforms = {
        repo / "v3/layers/l12_safety_final.py": patch_l12,
        repo / "v3/composition/bounded_live_control.py": patch_bounded,
        repo / "v3/composition/resident_live_control.py": patch_resident,
        repo / "v3/replay.py": patch_replay,
        repo / "v3_bounded_config.py": patch_bounded_config,
        repo / "v3/adapters/picamera2_camera.py": patch_camera_owner,
    }

    prepared: dict[Path, str] = {}
    for path, transform in transforms.items():
        prepared[path] = transform(path.read_text(encoding="utf-8"))

    # All anchors validated; writes are now safe to begin.
    for path, content in prepared.items():
        path.write_text(content, encoding="utf-8")

    for relative in (
        Path("v3/device_health_policy.py"),
        Path("tests/test_v3_camera_foundation.py"),
        Path("tests/test_v3_device_health_policy.py"),
    ):
        source = OVERLAY / relative
        target = repo / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)

    print("R2B4 camera foundation upgrade 1.1 applied")
    print("No camera or motor was started.")
    print("Run validation_camera_upgrade_1_1.sh from this package.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
