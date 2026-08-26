#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Sensor health ellenőrzések – indítás után validálás és futás közbeni monitorozás.
"""

from dataclasses import dataclass
import math
from typing import Optional, Any


@dataclass
class HealthResult:
    """Egyetlen szenzor health eredménye."""
    ok: bool
    message: str = ""
    details: dict = None

    def __post_init__(self):
        if self.details is None:
            self.details = {}


def check_imu(imu_driver=None, imu_service=None) -> HealthResult:
    """Validate the single BNO055 driver and its service snapshot."""
    try:
        if str(getattr(imu_driver, "provider", "") or "").strip().lower() != "bno055":
            return HealthResult(ok=False, message="BNO055 driver nincs")
        if not bool(getattr(imu_driver, "initialized", False)):
            return HealthResult(ok=False, message="BNO055 nincs inicializálva")
        snapshot = imu_service.get_snapshot() if imu_service is not None else None
        if snapshot is None:
            return HealthResult(ok=False, message="BNO055 snapshot nincs")
        health = str(getattr(snapshot, "health", "ERROR") or "ERROR").upper()
        gyro = tuple(getattr(snapshot, "gyro", ()) or ())
        accel = tuple(getattr(snapshot, "accel", ()) or ())
        if health not in ("OK", "DEGRADED"):
            return HealthResult(ok=False, message=f"BNO055 snapshot {health}")
        if len(gyro) < 3 or len(accel) < 3 or not all(
            isinstance(v, (int, float)) and math.isfinite(float(v))
            for v in (*gyro[:3], *accel[:3])
        ):
            return HealthResult(ok=False, message="BNO055 adat invalid")
        return HealthResult(ok=True, message="OK")
    except Exception as e:
        return HealthResult(ok=False, message=str(e))


def check_motor(motor_l=None, motor_r=None) -> HealthResult:
    """Motor driver health (PWM 0 állapot)."""
    try:
        if motor_l:
            motor_l.stop()
        if motor_r:
            motor_r.stop()
        return HealthResult(ok=True, message="OK")
    except Exception as e:
        return HealthResult(ok=False, message=str(e))


def check_encoder(enc_l=None, enc_r=None) -> HealthResult:
    """KIT0085 quadrature driver and counters are accessible."""
    try:
        if enc_l is not None and enc_r is not None:
            _ = enc_l.pulse_count, enc_r.pulse_count
            health_l = str(getattr(enc_l, "health", "ERROR") or "ERROR").upper()
            health_r = str(getattr(enc_r, "health", "ERROR") or "ERROR").upper()
            ok = health_l in ("OK", "DEGRADED") and health_r in ("OK", "DEGRADED")
            return HealthResult(
                ok=ok,
                message="OK" if ok else f"L:{health_l} R:{health_r}",
                details={
                    "model_left": str(getattr(enc_l, "model", "")),
                    "model_right": str(getattr(enc_r, "model", "")),
                    "pins_left": [getattr(enc_l, "pin_a", None), getattr(enc_l, "pin_b", None)],
                    "pins_right": [getattr(enc_r, "pin_a", None), getattr(enc_r, "pin_b", None)],
                },
            )
        return HealthResult(ok=False, message="Encoder nincs")
    except Exception as e:
        return HealthResult(ok=False, message=str(e))


def check_lidar(lidar_service=None) -> HealthResult:
    """LIDAR szolgáltatás elérhetőség."""
    if lidar_service is None:
        return HealthResult(ok=False, message="LIDAR nincs (DEGRADED)")
    try:
        snap = lidar_service.get_snapshot()
        if snap is None:
            return HealthResult(ok=False, message="LIDAR nincs adat")
        health = getattr(snap, "health", "OK")
        return HealthResult(ok=(health == "OK"), message=health)
    except Exception as e:
        return HealthResult(ok=False, message=str(e))


def run_all_checks(ctrl) -> dict:
    """Összes szenzor ellenőrzése – health report a supervisor számára."""
    def _r(h: HealthResult) -> dict:
        return {"ok": h.ok, "message": h.message}
    report = {}
    report["imu"] = _r(check_imu(
        getattr(ctrl, "imu_driver", None),
        getattr(ctrl, "imu_service", None),
    ))
    report["motor"] = _r(check_motor(
        getattr(ctrl, "motor_l", None),
        getattr(ctrl, "motor_r", None),
    ))
    report["encoder"] = _r(check_encoder(
        getattr(ctrl, "enc_l", None),
        getattr(ctrl, "enc_r", None),
    ))
    report["lidar"] = _r(check_lidar(getattr(ctrl, "lidar_service", None)))
    return report
