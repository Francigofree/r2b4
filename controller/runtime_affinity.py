"""Linux CPU-affinity contract for the robot runtime.

The Raspberry Pi 5 production layout keeps ordinary service work on CPUs
0-2 and the 50 Hz control loop on CPU 3.  No realtime scheduler or priority
change is made here: this module only narrows the current thread's CPU mask.

Linux represents threads as schedulable tasks, therefore
``os.sched_setaffinity(0, mask)`` affects the calling thread.  Threads created
after the service mask is applied inherit that mask; the controller main
thread is narrowed to the control mask only after startup has created the
logger, sensor and GUI workers.
"""

from __future__ import annotations

import os
import threading
from dataclasses import dataclass
from typing import Any, Callable, Dict, Iterable, Mapping, Optional, Tuple


class RuntimeAffinityError(RuntimeError):
    """Raised when a required affinity contract cannot be applied."""


@dataclass(frozen=True)
class RuntimeAffinityConfig:
    enabled: bool
    required: bool
    service_cpus: Tuple[int, ...]
    control_cpus: Tuple[int, ...]


_STATUS_BY_ROLE: Dict[str, Dict[str, Any]] = {}


def _cpu_tuple(values: Iterable[Any], *, name: str) -> Tuple[int, ...]:
    try:
        cpus = tuple(sorted({int(value) for value in values}))
    except Exception as exc:
        raise RuntimeAffinityError(f"runtime_affinity_invalid_{name}") from exc
    if not cpus or any(cpu < 0 for cpu in cpus):
        raise RuntimeAffinityError(f"runtime_affinity_invalid_{name}")
    return cpus


def config_from_root(root_config: Mapping[str, Any]) -> RuntimeAffinityConfig:
    vezerles = dict((root_config or {}).get("vezerles") or {})
    timing = dict(vezerles.get("idozites") or {})
    raw = dict(timing.get("runtime_cpu_affinity") or {})
    enabled = bool(raw.get("enabled", False))
    required = bool(raw.get("required", False))
    service_cpus = _cpu_tuple(raw.get("service_cpus", (0, 1, 2)), name="service_cpus")
    control_cpus = _cpu_tuple(raw.get("control_cpus", (3,)), name="control_cpus")
    if set(service_cpus).intersection(control_cpus):
        raise RuntimeAffinityError("runtime_affinity_cpu_sets_overlap")
    return RuntimeAffinityConfig(
        enabled=enabled,
        required=required,
        service_cpus=service_cpus,
        control_cpus=control_cpus,
    )


def _status_snapshot() -> Dict[str, Any]:
    return {
        "schema": "RUNTIME_CPU_AFFINITY_V1",
        "roles": {key: dict(value) for key, value in sorted(_STATUS_BY_ROLE.items())},
    }


def get_runtime_affinity_status() -> Dict[str, Any]:
    return _status_snapshot()


def apply_runtime_affinity(
    config: RuntimeAffinityConfig,
    *,
    role: str,
    setter: Optional[Callable[[int, set[int]], None]] = None,
    getter: Optional[Callable[[int], set[int]]] = None,
    cpu_count: Optional[int] = None,
) -> Dict[str, Any]:
    """Apply and verify the CPU mask for the calling thread.

    ``role`` is either ``service``/``hub`` (service mask) or ``control``.
    Test-only setter/getter injection keeps the contract deterministic without
    mutating the test runner's affinity.
    """

    role_name = str(role or "").strip().lower()
    if role_name not in {"service", "hub", "control", "status_writer"}:
        raise RuntimeAffinityError(f"runtime_affinity_unknown_role:{role_name or 'missing'}")
    requested = config.control_cpus if role_name == "control" else config.service_cpus
    status: Dict[str, Any] = {
        "role": role_name,
        "enabled": bool(config.enabled),
        "required": bool(config.required),
        "requested_cpus": list(requested),
        "applied": False,
        "verified": False,
        "effective_cpus": [],
        "pid": int(os.getpid()),
        "native_thread_id": int(threading.get_native_id()),
        "scheduler_policy_changed": False,
        "error": "",
    }
    if not config.enabled:
        _STATUS_BY_ROLE[role_name] = status
        return _status_snapshot()

    available_count = int(os.cpu_count() or 0) if cpu_count is None else int(cpu_count)
    if available_count <= max(requested):
        status["error"] = (
            f"runtime_affinity_cpu_unavailable:requested={list(requested)}:cpu_count={available_count}"
        )
        _STATUS_BY_ROLE[role_name] = status
        if config.required:
            raise RuntimeAffinityError(status["error"])
        return _status_snapshot()

    set_fn = setter if setter is not None else getattr(os, "sched_setaffinity", None)
    get_fn = getter if getter is not None else getattr(os, "sched_getaffinity", None)
    if not callable(set_fn) or not callable(get_fn):
        status["error"] = "runtime_affinity_api_unavailable"
        _STATUS_BY_ROLE[role_name] = status
        if config.required:
            raise RuntimeAffinityError(status["error"])
        return _status_snapshot()

    try:
        set_fn(0, set(requested))
        effective = tuple(sorted(int(cpu) for cpu in get_fn(0)))
        status["applied"] = True
        status["effective_cpus"] = list(effective)
        status["verified"] = effective == requested
        if not status["verified"]:
            status["error"] = (
                f"runtime_affinity_verify_failed:requested={list(requested)}:effective={list(effective)}"
            )
    except Exception as exc:
        status["error"] = f"runtime_affinity_apply_failed:{type(exc).__name__}:{exc}"

    _STATUS_BY_ROLE[role_name] = status
    if config.required and not status["verified"]:
        raise RuntimeAffinityError(status["error"] or "runtime_affinity_required_not_verified")
    return _status_snapshot()


def apply_active_service_thread_affinity(*, role: str) -> Dict[str, Any]:
    """Apply the active service mask at a late-created worker entry point."""

    # Lazy import avoids making this low-level module a config-loader
    # dependency during import and unit tests.
    from config_manager import config as global_config

    return apply_runtime_affinity(config_from_root(global_config.data), role=role)
