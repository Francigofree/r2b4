#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Peripheral usage SSOT (Single Source of Truth).

Canonical runtime file: runtime/peripherals_enabled.json
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Dict, Mapping

from log.runtime_debug import write_json_atomic

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RUNTIME_DIR = PROJECT_ROOT / "runtime"
SSOT_FILENAME = "peripherals_enabled.json"

# Only independently controllable runtime peripherals belong here.  BNO055 is
# one atomic IMU; the removed multi-chip accelerometer/gyro/magnetometer flags
# must never be recreated from an old state file.
PERIPHERAL_DEFAULTS: Dict[str, bool] = {
    "camera": False,
    "lidar": True,
    "encoder": True,
    "imu": True,
    "microphone": False,
}

_TRUE_VALUES = {"1", "true", "yes", "on"}
_CACHE: Dict[str, Dict[str, object]] = {}
_CACHE_TTL_SEC = 0.12


def _normalize_runtime_dir(runtime_dir: str | Path | None = None, status_path: str | Path | None = None) -> Path:
    if runtime_dir:
        return Path(runtime_dir)
    if status_path:
        return Path(status_path).resolve().parent
    return DEFAULT_RUNTIME_DIR


def _cache_key(runtime_dir: Path) -> str:
    try:
        return str(runtime_dir.resolve())
    except Exception:
        return str(runtime_dir)


def _normalize_runtime_dir_fast(runtime_dir: str | Path | None = None, status_path: str | Path | None = None) -> Path:
    if runtime_dir:
        path = Path(runtime_dir)
    elif status_path:
        path = Path(status_path).parent
    else:
        path = DEFAULT_RUNTIME_DIR
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    return path


def _cache_key_fast(runtime_dir: Path) -> str:
    return str(runtime_dir)


def _coerce_bool(value, default: bool) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        return value.strip().lower() in _TRUE_VALUES
    return bool(default)


def _normalize_state(raw) -> Dict[str, bool]:
    state: Dict[str, bool] = dict(PERIPHERAL_DEFAULTS)
    if isinstance(raw, Mapping):
        for key, value in raw.items():
            k = str(key).strip().lower()
            if k not in PERIPHERAL_DEFAULTS:
                continue
            default = state[k]
            state[k] = _coerce_bool(value, default)
    return state


def _read_state_uncached(runtime_dir: Path) -> tuple[Dict[str, bool], bool]:
    state_path = runtime_dir / SSOT_FILENAME
    raw = None
    parse_failed = False
    if state_path.exists():
        try:
            raw = json.loads(state_path.read_text(encoding="utf-8"))
        except Exception:
            raw = None
            parse_failed = True

    state = _normalize_state(raw)
    raw_keys = {
        str(key).strip().lower()
        for key in raw.keys()
    } if isinstance(raw, Mapping) else set()
    needs_sync = (
        (not state_path.exists())
        or parse_failed
        or raw_keys != set(PERIPHERAL_DEFAULTS)
    )

    return state, needs_sync


def _write_state(runtime_dir: Path, state: Mapping[str, bool]) -> Dict[str, bool]:
    runtime_dir.mkdir(parents=True, exist_ok=True)
    normalized = _normalize_state(state)
    state_path = runtime_dir / SSOT_FILENAME
    write_json_atomic(state_path, normalized, indent=2)

    _CACHE[_cache_key(runtime_dir)] = {
        "ts": time.monotonic(),
        "state": dict(normalized),
    }
    return normalized


def ensure_peripheral_ssot(runtime_dir: str | Path | None = None, *, status_path: str | Path | None = None) -> Dict[str, bool]:
    resolved = _normalize_runtime_dir(runtime_dir=runtime_dir, status_path=status_path)
    state, needs_sync = _read_state_uncached(resolved)
    if needs_sync:
        return _write_state(resolved, state)
    _CACHE[_cache_key(resolved)] = {
        "ts": time.monotonic(),
        "state": dict(state),
    }
    return state


def read_peripherals(
    runtime_dir: str | Path | None = None,
    *,
    status_path: str | Path | None = None,
    use_cache: bool = True,
    cache_ttl_s: float = _CACHE_TTL_SEC,
) -> Dict[str, bool]:
    resolved = _normalize_runtime_dir(runtime_dir=runtime_dir, status_path=status_path)
    key = _cache_key(resolved)
    now = time.monotonic()

    if use_cache:
        cached = _CACHE.get(key)
        if isinstance(cached, dict):
            age = now - float(cached.get("ts", 0.0) or 0.0)
            if age <= max(0.0, float(cache_ttl_s)):
                return dict(cached.get("state") or {})

    state, needs_sync = _read_state_uncached(resolved)
    if needs_sync:
        return _write_state(resolved, state)

    _CACHE[key] = {
        "ts": now,
        "state": dict(state),
    }
    return state


def get_cached_peripherals(
    runtime_dir: str | Path | None = None,
    *,
    status_path: str | Path | None = None,
) -> Dict[str, bool]:
    """Return the last RAM-cached peripheral state without filesystem I/O."""
    resolved = _normalize_runtime_dir_fast(runtime_dir=runtime_dir, status_path=status_path)
    cached = _CACHE.get(_cache_key_fast(resolved))
    if not isinstance(cached, dict):
        cached = _CACHE.get(str(resolved))
    if isinstance(cached, dict):
        state = cached.get("state")
        if isinstance(state, Mapping):
            return _normalize_state(state)
    return dict(PERIPHERAL_DEFAULTS)


def is_peripheral_enabled(
    name: str,
    runtime_dir: str | Path | None = None,
    *,
    status_path: str | Path | None = None,
    default: bool | None = None,
    use_cache: bool = True,
    cache_ttl_s: float = _CACHE_TTL_SEC,
) -> bool:
    key = str(name or "").strip().lower()
    if not key:
        return bool(default) if default is not None else False
    state = read_peripherals(
        runtime_dir=runtime_dir,
        status_path=status_path,
        use_cache=use_cache,
        cache_ttl_s=cache_ttl_s,
    )
    if key in state:
        return bool(state[key])
    if default is not None:
        return bool(default)
    return bool(PERIPHERAL_DEFAULTS.get(key, False))


def set_peripheral_enabled(
    name: str,
    enabled: bool,
    runtime_dir: str | Path | None = None,
    *,
    status_path: str | Path | None = None,
) -> Dict[str, bool]:
    key = str(name or "").strip().lower()
    if not key:
        raise ValueError("peripheral_name_missing")
    if key not in PERIPHERAL_DEFAULTS:
        raise ValueError(f"unsupported_peripheral:{key}")

    resolved = _normalize_runtime_dir(runtime_dir=runtime_dir, status_path=status_path)
    current = read_peripherals(runtime_dir=resolved, use_cache=False)
    current[key] = bool(enabled)
    return _write_state(resolved, current)


def set_peripherals(
    updates: Mapping[str, object],
    runtime_dir: str | Path | None = None,
    *,
    status_path: str | Path | None = None,
) -> Dict[str, bool]:
    resolved = _normalize_runtime_dir(runtime_dir=runtime_dir, status_path=status_path)
    current = read_peripherals(runtime_dir=resolved, use_cache=False)
    if isinstance(updates, Mapping):
        for key, value in updates.items():
            k = str(key or "").strip().lower()
            if k not in PERIPHERAL_DEFAULTS:
                raise ValueError(f"unsupported_peripheral:{k or 'MISSING'}")
            default = current[k]
            current[k] = _coerce_bool(value, bool(default))
    return _write_state(resolved, current)
