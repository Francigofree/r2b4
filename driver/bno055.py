#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
BNO055 fused IMU driver for DFRobot SEN0253 Gravity 10DOF.

The SEN0253 exposes the Bosch BNO055 at 0x28 and a BMP280 at 0x76.  This
driver uses the BNO055 fusion output directly: gyro, fused Euler heading,
quaternion and linear acceleration are read in one contiguous burst.
"""

from __future__ import annotations

import math
import time
from statistics import median
from typing import Any, Dict, Iterable, Optional, Tuple

import smbus2


class BNO055IMU:
    """BNO055 fused IMU used through its atomic ``read_sample`` API."""

    DEFAULT_ADDR = 0x28
    CHIP_ID = 0xA0

    REG_CHIP_ID = 0x00
    REG_PAGE_ID = 0x07
    REG_GYRO_DATA_X_LSB = 0x14
    REG_CALIB_STAT = 0x35
    REG_SYS_STATUS = 0x39
    REG_SYS_ERR = 0x3A
    REG_UNIT_SEL = 0x3B
    REG_OPR_MODE = 0x3D
    REG_PWR_MODE = 0x3E
    REG_SYS_TRIGGER = 0x3F

    MODE_CONFIG = 0x00
    MODE_IMU = 0x08
    MODE_NDOF_FMC_OFF = 0x0B
    MODE_NDOF = 0x0C

    OPERATION_MODES = {
        "CONFIG": MODE_CONFIG,
        "IMU": MODE_IMU,
        "NDOF_FMC_OFF": MODE_NDOF_FMC_OFF,
        "NDOF": MODE_NDOF,
    }

    def __init__(
        self,
        bus_num: int = 1,
        address: int = DEFAULT_ADDR,
        *,
        bus: Optional[Any] = None,
        operation_mode: str = "NDOF",
        axis_order: Optional[Iterable[int]] = None,
        axis_sign: Optional[Iterable[int]] = None,
        update_rate_hz: float = 50.0,
        use_external_crystal: bool = False,
    ):
        self.address = int(address)
        self.bus_num = int(bus_num)
        self.bus = bus if bus is not None else smbus2.SMBus(self.bus_num)
        self._owns_bus = bus is None
        self.operation_mode_name = str(operation_mode or "NDOF").strip().upper()
        self.operation_mode = self.OPERATION_MODES.get(self.operation_mode_name, self.MODE_NDOF)
        self.axis_order = tuple(int(v) for v in (axis_order or (0, 1, 2)))
        self.axis_sign = tuple(1 if int(v) >= 0 else -1 for v in (axis_sign or (1, 1, 1)))
        if len(self.axis_order) != 3 or sorted(self.axis_order) != [0, 1, 2]:
            self.axis_order = (0, 1, 2)
        if len(self.axis_sign) != 3:
            self.axis_sign = (1, 1, 1)
        self.update_rate_hz = max(1.0, float(update_rate_hz))
        self._cache_max_age_s = min(0.02, 0.5 / self.update_rate_hz)
        self.use_external_crystal = bool(use_external_crystal)

        self.initialized = False
        self.sensor_ok = False
        self.calibration_managed_by_device = True
        self.model = "DFRobot SEN0253 / Bosch BNO055"
        self.provider = "bno055"
        self.offsets: Dict[str, float] = {}
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_z = 0.0

        self._last_sample: Optional[Dict[str, Any]] = None
        self._last_sample_ts = 0.0
        self._last_diag: Dict[str, Any] = {}
        self._last_diag_ts = 0.0

    @staticmethod
    def _s16(lsb: int, msb: int) -> int:
        val = (int(msb) << 8) | int(lsb)
        if val & 0x8000:
            val -= 0x10000
        return val

    @classmethod
    def _vec3(cls, data: list[int], offset: int, scale: float) -> tuple[float, float, float]:
        return (
            cls._s16(data[offset], data[offset + 1]) * scale,
            cls._s16(data[offset + 2], data[offset + 3]) * scale,
            cls._s16(data[offset + 4], data[offset + 5]) * scale,
        )

    @classmethod
    def _quat(cls, data: list[int], offset: int) -> tuple[float, float, float, float]:
        scale = 1.0 / 16384.0
        return (
            cls._s16(data[offset], data[offset + 1]) * scale,
            cls._s16(data[offset + 2], data[offset + 3]) * scale,
            cls._s16(data[offset + 4], data[offset + 5]) * scale,
            cls._s16(data[offset + 6], data[offset + 7]) * scale,
        )

    def _map_vec(self, vec: tuple[float, float, float]) -> tuple[float, float, float]:
        return (
            float(vec[self.axis_order[0]]) * float(self.axis_sign[0]),
            float(vec[self.axis_order[1]]) * float(self.axis_sign[1]),
            float(vec[self.axis_order[2]]) * float(self.axis_sign[2]),
        )

    def _set_mode(self, mode: int) -> None:
        self.bus.write_byte_data(self.address, self.REG_OPR_MODE, int(mode) & 0x0F)
        time.sleep(0.03 if mode == self.MODE_CONFIG else 0.08)

    def _read_chip_id(self) -> int:
        return int(self.bus.read_byte_data(self.address, self.REG_CHIP_ID))

    def initialize(self) -> bool:
        """Initialize the BNO055 and switch to the configured fusion mode."""
        try:
            chip_id = None
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                try:
                    chip_id = self._read_chip_id()
                    if chip_id == self.CHIP_ID:
                        break
                except Exception:
                    pass
                time.sleep(0.05)
            if chip_id != self.CHIP_ID:
                print(f"[BNO055 HIBA] Hibás CHIP_ID: {hex(chip_id or 0)} (elvárt: {hex(self.CHIP_ID)})")
                return False

            self._set_mode(self.MODE_CONFIG)
            self.bus.write_byte_data(self.address, self.REG_PAGE_ID, 0x00)
            self.bus.write_byte_data(self.address, self.REG_PWR_MODE, 0x00)
            time.sleep(0.01)
            # Units: m/s2, dps, degrees, Celsius, Android orientation.
            self.bus.write_byte_data(self.address, self.REG_UNIT_SEL, 0x00)
            if self.use_external_crystal:
                self.bus.write_byte_data(self.address, self.REG_SYS_TRIGGER, 0x80)
                time.sleep(0.01)
            self._set_mode(self.operation_mode)

            self.initialized = True
            self.sensor_ok = True
            self.read_sample(force=True)
            print(f"BNO055 inicializálva {self.operation_mode_name} módban @ {hex(self.address)}.")
            return True
        except Exception as e:
            print(f"BNO055 inicializálási hiba: {e}")
            return False

    def _read_diag_if_due(self, now: float) -> Dict[str, Any]:
        if self._last_diag and (now - self._last_diag_ts) < 0.20:
            return dict(self._last_diag)
        try:
            calib = int(self.bus.read_byte_data(self.address, self.REG_CALIB_STAT))
            sys_status = int(self.bus.read_byte_data(self.address, self.REG_SYS_STATUS))
            sys_err_raw = int(self.bus.read_byte_data(self.address, self.REG_SYS_ERR))
            sys_err = sys_err_raw if sys_status == 1 else 0
            out = {
                "calibration": {
                    "sys": (calib >> 6) & 0x03,
                    "gyro": (calib >> 4) & 0x03,
                    "accel": (calib >> 2) & 0x03,
                    "mag": calib & 0x03,
                },
                "sys_status": sys_status,
                "sys_error": sys_err,
                "sys_error_raw": sys_err_raw,
            }
            self._last_diag = dict(out)
            self._last_diag_ts = float(now)
            return out
        except Exception:
            return dict(self._last_diag or {})

    def read_sample(self, *, force: bool = False) -> Dict[str, Any]:
        """
        Read one fused BNO055 sample.

        Register span 0x14..0x33:
        gyro, Euler, quaternion, linear acceleration and gravity.
        """
        now = time.perf_counter()
        if (
            not force
            and self._last_sample is not None
            and (now - self._last_sample_ts) <= self._cache_max_age_s
        ):
            return dict(self._last_sample)

        data = list(self.bus.read_i2c_block_data(self.address, self.REG_GYRO_DATA_X_LSB, 32))
        gyro_raw = (
            self._s16(data[0], data[1]),
            self._s16(data[2], data[3]),
            self._s16(data[4], data[5]),
        )
        gyro_dps = self._map_vec(tuple(float(v) / 16.0 for v in gyro_raw))
        euler = {
            "heading_deg": self._s16(data[6], data[7]) / 16.0,
            "roll_deg": self._s16(data[8], data[9]) / 16.0,
            "pitch_deg": self._s16(data[10], data[11]) / 16.0,
        }
        quat = self._quat(data, 12)
        linear_mps2 = self._map_vec(self._vec3(data, 20, 1.0 / 100.0))
        gravity_mps2 = self._map_vec(self._vec3(data, 26, 1.0 / 100.0))
        accel_g = tuple(float(v) / 9.80665 for v in linear_mps2)
        diag = self._read_diag_if_due(now)

        sample = {
            "timestamp": now,
            "source": self.provider,
            "accel_g": accel_g,
            "gyro_dps": gyro_dps,
            "heading_deg": float(euler["heading_deg"] % 360.0),
            "euler": euler,
            "quaternion": quat,
            "linear_accel_mps2": linear_mps2,
            "gravity_mps2": gravity_mps2,
            "raw_gyro": gyro_raw,
            **diag,
        }
        self._last_sample = dict(sample)
        self._last_sample_ts = float(now)
        return sample

    def calibrate(self, samples: int = 80) -> Dict[str, Any]:
        """
        BNO055 performs calibration internally.  For startup gating we collect a
        short stationary window and expose current device calibration bits.
        """
        count = max(10, min(int(samples), 120))
        for _ in range(count):
            self.read_sample(force=True)
            time.sleep(1.0 / self.update_rate_hz)
        sample = self.read_sample(force=True)
        return {
            "provider": self.provider,
            "device_managed": True,
            "calibration": dict(sample.get("calibration") or {}),
            "sys_status": int(sample.get("sys_status", 0) or 0),
            "sys_error": int(sample.get("sys_error", 0) or 0),
        }

    def measure_stationary_error(self, samples: int = 80) -> Dict[str, float]:
        xs, ys, zs = [], [], []
        axs, ays, azs = [], [], []
        count = max(10, int(samples))
        for _ in range(count):
            sample = self.read_sample(force=True)
            gx, gy, gz = tuple(sample.get("gyro_dps", (0.0, 0.0, 0.0)))
            ax, ay, az = tuple(sample.get("accel_g", (0.0, 0.0, 0.0)))
            xs.append(float(gx))
            ys.append(float(gy))
            zs.append(float(gz))
            axs.append(float(ax))
            ays.append(float(ay))
            azs.append(float(az))
            time.sleep(1.0 / self.update_rate_hz)

        def _mean(vals):
            return sum(vals) / float(len(vals)) if vals else 0.0

        def _std(vals, mean):
            if not vals:
                return 0.0
            return math.sqrt(sum((v - mean) * (v - mean) for v in vals) / float(len(vals)))

        mx, my, mz = _mean(xs), _mean(ys), _mean(zs)
        axm, aym, azm = _mean(axs), _mean(ays), _mean(azs)
        norm_mean = _mean([math.sqrt(x * x + y * y + z * z) for x, y, z in zip(axs, ays, azs)])
        return {
            "x_mean_dps": mx,
            "y_mean_dps": my,
            "z_mean_dps": mz,
            "x_std_dps": _std(xs, mx),
            "y_std_dps": _std(ys, my),
            "z_std_dps": _std(zs, mz),
            "x_mean_g": axm,
            "y_mean_g": aym,
            "z_mean_g": azm,
            "x_std_g": _std(axs, axm),
            "y_std_g": _std(ays, aym),
            "z_std_g": _std(azs, azm),
            "norm_mean_g": norm_mean,
            "z_median_dps": median(zs) if zs else 0.0,
        }

    def close(self) -> None:
        if self._owns_bus and getattr(self, "bus", None) is not None:
            try:
                self.bus.close()
            except Exception:
                pass
        self.bus = None
