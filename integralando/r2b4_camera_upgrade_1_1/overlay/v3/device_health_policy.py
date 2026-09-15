"""Shared critical-device health policy for V3 safety and preflight gates.

The physical health of every device remains observable.  This module only
answers which device health values are allowed to control activation/safety.
"""

from __future__ import annotations

from dataclasses import dataclass

from v3.contracts import DeviceHealth, DeviceHealthState


PRODUCTION_CRITICAL_DEVICE_IDS = frozenset(
    {"WHEEL_ENCODERS", "BNO055_IMU", "RPLIDAR_C1"}
)


@dataclass(frozen=True, slots=True)
class CriticalDeviceHealthView:
    critical_health: tuple[DeviceHealth, ...]
    missing_device_ids: tuple[str, ...]


def critical_device_health_view(
    device_health: tuple[DeviceHealth, ...],
    critical_device_ids: frozenset[str] | None,
) -> CriticalDeviceHealthView:
    """Return the health values that own activation/safety authority.

    ``None`` deliberately preserves the legacy behaviour where every supplied
    device-health value is critical.  Production uses an explicit non-empty set.
    """

    if not isinstance(device_health, tuple) or any(
        not isinstance(item, DeviceHealth) for item in device_health
    ):
        raise TypeError("device_health must be tuple[DeviceHealth, ...]")
    if critical_device_ids is None:
        return CriticalDeviceHealthView(device_health, ())
    if not isinstance(critical_device_ids, frozenset) or not critical_device_ids:
        raise ValueError("critical_device_ids must be a non-empty frozenset or None")
    if any(
        not isinstance(device_id, str) or not device_id.strip()
        for device_id in critical_device_ids
    ):
        raise ValueError("critical_device_ids must contain non-empty strings")

    health_by_id = {item.device_id: item for item in device_health}
    critical = tuple(
        item for item in device_health if item.device_id in critical_device_ids
    )
    missing = tuple(sorted(critical_device_ids - set(health_by_id)))
    return CriticalDeviceHealthView(critical, missing)


def critical_devices_ready(
    device_health: tuple[DeviceHealth, ...],
    critical_device_ids: frozenset[str] | None,
) -> bool:
    """True only when every required critical device is present and OK."""

    view = critical_device_health_view(device_health, critical_device_ids)
    return bool(view.critical_health) and not view.missing_device_ids and all(
        item.state is DeviceHealthState.OK for item in view.critical_health
    )


def blocking_degraded_sources(
    degraded_sources: tuple[str, ...],
    critical_device_ids: frozenset[str] | None,
) -> tuple[str, ...]:
    """Filter L2 degraded source ids to only activation-critical sources."""

    if not isinstance(degraded_sources, tuple) or any(
        not isinstance(source, str) or not source for source in degraded_sources
    ):
        raise TypeError("degraded_sources must be tuple[str, ...]")
    if critical_device_ids is None:
        return degraded_sources
    return tuple(source for source in degraded_sources if source in critical_device_ids)


__all__ = [
    "CriticalDeviceHealthView",
    "PRODUCTION_CRITICAL_DEVICE_IDS",
    "blocking_degraded_sources",
    "critical_device_health_view",
    "critical_devices_ready",
]
