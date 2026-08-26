#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""DFRobot SEN0253 / BNO055 construction and presence checks."""

from __future__ import annotations

from typing import Any, Dict, Iterable, Mapping, Optional, Tuple


BNO055_DEFAULT_ADDR = 0x28
SEN0253_BMP280_ADDRS = (0x76, 0x77)


def _as_dict(value: Any) -> Dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


def hardver_config(config: Optional[Mapping[str, Any]]) -> Dict[str, Any]:
    cfg = _as_dict(config)
    return _as_dict(cfg.get("hardver")) if isinstance(cfg.get("hardver"), Mapping) else cfg


def imu_config(config: Optional[Mapping[str, Any]]) -> Dict[str, Any]:
    hw = hardver_config(config)
    return _as_dict(hw.get("imu"))


def imu_provider_from_config(config: Optional[Mapping[str, Any]]) -> str:
    provider = str(imu_config(config).get("provider", "") or "").strip().lower()
    if provider != "bno055":
        raise ValueError(f"unsupported_imu_provider:{provider or 'MISSING'}")
    return "bno055"


def _require_bno055_provider(
    provider: Optional[str],
    config: Optional[Mapping[str, Any]],
) -> str:
    selected = str(provider or "").strip().lower()
    if not selected:
        return imu_provider_from_config(config)
    if selected != "bno055":
        raise ValueError(f"unsupported_imu_provider:{selected}")
    return "bno055"


def _parse_int(value: Any, default: int) -> int:
    if isinstance(value, int):
        return int(value)
    if isinstance(value, str):
        text = value.strip().lower()
        try:
            return int(text, 16) if text.startswith("0x") else int(text)
        except ValueError:
            return int(default)
    return int(default)


def bno055_config(config: Optional[Mapping[str, Any]]) -> Dict[str, Any]:
    imu_cfg = imu_config(config)
    return _as_dict(imu_cfg.get("bno055"))


def bno055_address_from_config(config: Optional[Mapping[str, Any]]) -> int:
    bno_cfg = bno055_config(config)
    return _parse_int(bno_cfg.get("address", BNO055_DEFAULT_ADDR), BNO055_DEFAULT_ADDR)


def _normalized_i2c_addresses(addrs: Iterable[Any]) -> set[int]:
    out: set[int] = set()
    for item in addrs or []:
        try:
            if isinstance(item, str):
                out.add(int(item.strip().lower(), 16))
            else:
                out.add(int(item))
        except Exception:
            continue
    return out


def imu_presence_from_i2c(
    addrs: Iterable[Any],
    config: Optional[Mapping[str, Any]] = None,
    *,
    provider: Optional[str] = None,
    bno055_addr: Optional[int] = None,
) -> Tuple[bool, str, Dict[str, Any]]:
    """Return BNO055 presence without probing removed IMU providers."""
    configured = _require_bno055_provider(provider, config)
    bno_addr = int(bno055_addr if bno055_addr is not None else bno055_address_from_config(config))
    found = _normalized_i2c_addresses(addrs)

    has_bno = bno_addr in found
    has_bmp280 = any(addr in found for addr in SEN0253_BMP280_ADDRS)

    details = {
        "configured_provider": configured,
        "bno055": has_bno,
        "bno055_addr": hex(bno_addr),
        "sen0253_bmp280": has_bmp280,
    }
    return bool(has_bno), "bno055", details


def imu_probe_targets(
    config: Optional[Mapping[str, Any]] = None,
    *,
    provider: Optional[str] = None,
    bno055_addr: Optional[int] = None,
) -> list[tuple[int, int]]:
    _require_bno055_provider(provider, config)
    bno_addr = int(bno055_addr if bno055_addr is not None else bno055_address_from_config(config))
    return [(bno_addr, 0x00)]


def _tuple_ints(value: Any, default: tuple[int, int, int]) -> tuple[int, int, int]:
    if isinstance(value, (list, tuple)) and len(value) == 3:
        try:
            return int(value[0]), int(value[1]), int(value[2])
        except Exception:
            return default
    return default


def build_imu_devices(
    config: Optional[Mapping[str, Any]] = None,
    *,
    provider: Optional[str] = None,
    initialize: bool = True,
) -> Dict[str, Any]:
    """Create the only supported IMU driver."""
    _require_bno055_provider(provider, config)
    from driver.bno055 import BNO055IMU

    bno_cfg = bno055_config(config)
    imu = BNO055IMU(
        bus_num=int(bno_cfg.get("bus", 1)),
        address=bno055_address_from_config(config),
        operation_mode=str(bno_cfg.get("operation_mode", "NDOF")),
        axis_order=_tuple_ints(bno_cfg.get("axis_order"), (0, 1, 2)),
        axis_sign=_tuple_ints(bno_cfg.get("axis_sign"), (1, 1, 1)),
        update_rate_hz=float(bno_cfg.get("update_rate_hz", 50.0)),
        use_external_crystal=bool(bno_cfg.get("use_external_crystal", False)),
    )
    ok = imu.initialize() if initialize else True
    if not ok:
        raise RuntimeError("BNO055 init failed")
    return {
        "provider": "bno055",
        "driver": imu,
    }
