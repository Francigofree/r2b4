"""Read-only Raspberry Pi/Linux host information for external R2B4 clients."""

from __future__ import annotations

import os
import platform
import shutil
import time
from collections.abc import Mapping
from pathlib import Path


class SystemInterfaceAdapter:
    name = "system"
    capability_names = frozenset({"system.status"})

    def __init__(self, root: Path) -> None:
        self.root = root

    def capabilities(self) -> Mapping[str, Mapping[str, object]]:
        return {
            "system.status": {
                "kind": "read",
                "supported": True,
                "available": True,
                "ready": True,
            }
        }

    def read(self, resource: str) -> object:
        if resource != "system.status":
            raise KeyError(resource)
        disk = shutil.disk_usage(self.root)
        uptime_s = self._uptime_s()
        return {
            "hostname": platform.node(),
            "platform": platform.platform(),
            "python": platform.python_version(),
            "pid": os.getpid(),
            "loadavg": list(os.getloadavg()) if hasattr(os, "getloadavg") else None,
            "uptime_s": uptime_s,
            "cpu_temperature_c": self._cpu_temperature_c(),
            "disk": {
                "total_bytes": disk.total,
                "used_bytes": disk.used,
                "free_bytes": disk.free,
            },
            "sampled_at_unix_s": time.time(),
        }

    def execute(self, action: str, **parameters: object) -> object:
        raise KeyError(action)

    @staticmethod
    def _uptime_s() -> float | None:
        try:
            return float(Path("/proc/uptime").read_text(encoding="utf-8").split()[0])
        except (OSError, ValueError, IndexError):
            return None

    @staticmethod
    def _cpu_temperature_c() -> float | None:
        path = Path("/sys/class/thermal/thermal_zone0/temp")
        try:
            raw = float(path.read_text(encoding="utf-8").strip())
        except (OSError, ValueError):
            return None
        return raw / 1000.0 if raw > 1000.0 else raw


__all__ = ["SystemInterfaceAdapter"]
