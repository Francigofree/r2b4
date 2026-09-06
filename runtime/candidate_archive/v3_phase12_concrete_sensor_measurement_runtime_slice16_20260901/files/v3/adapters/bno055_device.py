"""Owned native BNO055 register device with one monotonic time domain."""

from __future__ import annotations

import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Protocol


class Bno055RegisterBus(Protocol):
    """Capability-limited SMBus surface required by the native device."""

    def read_byte_data(self, address: int, register: int) -> int: ...

    def write_byte_data(self, address: int, register: int, value: int) -> object: ...

    def read_i2c_block_data(
        self,
        address: int,
        register: int,
        length: int,
    ) -> object: ...

    def close(self) -> object: ...


def _positive_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _nonnegative_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


@dataclass(frozen=True, slots=True)
class NativeBno055DeviceConfig:
    """Immutable bus, fusion-mode and physical-axis configuration."""

    bus_number: int = 1
    address: int = 0x28
    operation_mode: str = "NDOF"
    axis_order: tuple[int, int, int] = (0, 1, 2)
    axis_sign: tuple[int, int, int] = (1, 1, 1)
    use_external_crystal: bool = False
    startup_timeout_ns: int = 1_000_000_000
    startup_poll_interval_ns: int = 50_000_000

    def __post_init__(self) -> None:
        _nonnegative_int(self.bus_number, "bus_number")
        if (
            not isinstance(self.address, int)
            or isinstance(self.address, bool)
            or not 0x08 <= self.address <= 0x77
        ):
            raise ValueError("address must be a valid seven-bit I2C address")
        if self.operation_mode not in NativeBno055Device.OPERATION_MODES:
            raise ValueError("operation_mode must be IMU, NDOF_FMC_OFF or NDOF")
        if (
            not isinstance(self.axis_order, tuple)
            or len(self.axis_order) != 3
            or sorted(self.axis_order) != [0, 1, 2]
        ):
            raise ValueError("axis_order must be a permutation of (0, 1, 2)")
        if (
            not isinstance(self.axis_sign, tuple)
            or len(self.axis_sign) != 3
            or any(value not in (-1, 1) for value in self.axis_sign)
        ):
            raise ValueError("axis_sign must contain exactly three -1 or 1 values")
        if type(self.use_external_crystal) is not bool:
            raise ValueError("use_external_crystal must be bool")
        _positive_int(self.startup_timeout_ns, "startup_timeout_ns")
        poll_ns = _positive_int(
            self.startup_poll_interval_ns,
            "startup_poll_interval_ns",
        )
        if poll_ns > self.startup_timeout_ns:
            raise ValueError("startup_poll_interval_ns cannot exceed startup_timeout_ns")


class NativeBno055Device:
    """Own one already-open bus and expose forced fused samples.

    The injected ``monotonic_ns`` callable must be the same clock domain used by
    the V3 owner loop.  A sample is timestamped immediately after the fused
    register burst, so freshness comparison never crosses clock domains.
    """

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
    OPERATION_MODES = {
        "IMU": 0x08,
        "NDOF_FMC_OFF": 0x0B,
        "NDOF": 0x0C,
    }

    __slots__ = (
        "_bus",
        "_closed",
        "_config",
        "_monotonic_ns",
        "_sleep",
        "initialized",
        "sensor_ok",
    )

    def __init__(
        self,
        bus: Bno055RegisterBus,
        config: NativeBno055DeviceConfig,
        *,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        if not isinstance(config, NativeBno055DeviceConfig):
            raise TypeError("config must be NativeBno055DeviceConfig")
        for method_name in (
            "read_byte_data",
            "write_byte_data",
            "read_i2c_block_data",
            "close",
        ):
            if not callable(getattr(bus, method_name, None)):
                raise TypeError(f"bus must provide a callable {method_name} method")
        for callback, name in ((monotonic_ns, "monotonic_ns"), (sleep, "sleep")):
            if not callable(callback):
                raise TypeError(f"{name} must be callable")
        self._bus = bus
        self._config = config
        self._monotonic_ns = monotonic_ns
        self._sleep = sleep
        self._closed = False
        self.initialized = False
        self.sensor_ok = False

    @staticmethod
    def _signed_16(lsb: int, msb: int) -> int:
        value = (int(msb) << 8) | int(lsb)
        return value - 0x10000 if value & 0x8000 else value

    def _clock_ns(self) -> int:
        value = self._monotonic_ns()
        if not isinstance(value, int) or isinstance(value, bool) or value < 0:
            raise ValueError("monotonic_ns must return a non-negative integer")
        return value

    def _set_mode(self, mode: int) -> None:
        self._bus.write_byte_data(
            self._config.address,
            self.REG_OPR_MODE,
            mode & 0x0F,
        )
        self._sleep(0.03 if mode == self.MODE_CONFIG else 0.08)

    def initialize(self) -> None:
        """Initialize fusion mode or close the owned bus on any failure."""

        if self._closed:
            raise RuntimeError("BNO055 device is closed")
        if self.initialized:
            return
        try:
            started_ns = self._clock_ns()
            chip_id: int | None = None
            while True:
                try:
                    chip_id = int(
                        self._bus.read_byte_data(
                            self._config.address,
                            self.REG_CHIP_ID,
                        )
                    )
                except Exception:
                    chip_id = None
                if chip_id == self.CHIP_ID:
                    break
                now_ns = self._clock_ns()
                if now_ns < started_ns:
                    raise RuntimeError("monotonic clock moved backwards")
                if now_ns - started_ns >= self._config.startup_timeout_ns:
                    raise OSError("BNO055 chip ID did not become available")
                self._sleep(self._config.startup_poll_interval_ns / 1_000_000_000.0)

            self._set_mode(self.MODE_CONFIG)
            self._bus.write_byte_data(self._config.address, self.REG_PAGE_ID, 0x00)
            self._bus.write_byte_data(self._config.address, self.REG_PWR_MODE, 0x00)
            self._sleep(0.01)
            self._bus.write_byte_data(self._config.address, self.REG_UNIT_SEL, 0x00)
            if self._config.use_external_crystal:
                self._bus.write_byte_data(
                    self._config.address,
                    self.REG_SYS_TRIGGER,
                    0x80,
                )
                self._sleep(0.01)
            self._set_mode(self.OPERATION_MODES[self._config.operation_mode])
            self.initialized = True
            self.sensor_ok = True
            self.read_sample(force=True)
        except Exception:
            self.sensor_ok = False
            self.initialized = False
            self.close()
            raise

    def _mapped_gyro(self, data: list[int]) -> tuple[float, float, float]:
        raw = (
            self._signed_16(data[0], data[1]) / 16.0,
            self._signed_16(data[2], data[3]) / 16.0,
            self._signed_16(data[4], data[5]) / 16.0,
        )
        order = self._config.axis_order
        sign = self._config.axis_sign
        return (
            float(raw[order[0]]) * float(sign[0]),
            float(raw[order[1]]) * float(sign[1]),
            float(raw[order[2]]) * float(sign[2]),
        )

    def _read_sample_at(self, captured_ns: int) -> Mapping[str, object]:
        if self._closed or not self.initialized:
            raise RuntimeError("BNO055 device is not initialized")
        try:
            raw = self._bus.read_i2c_block_data(
                self._config.address,
                self.REG_GYRO_DATA_X_LSB,
                32,
            )
            if not isinstance(raw, (list, tuple)) or len(raw) != 32:
                raise OSError("BNO055 fused burst must contain exactly 32 bytes")
            data = [int(value) for value in raw]
            if any(value < 0 or value > 255 for value in data):
                raise OSError("BNO055 fused burst contains an invalid byte")
            calibration = int(
                self._bus.read_byte_data(
                    self._config.address,
                    self.REG_CALIB_STAT,
                )
            )
            system_status = int(
                self._bus.read_byte_data(
                    self._config.address,
                    self.REG_SYS_STATUS,
                )
            )
            system_error = int(
                self._bus.read_byte_data(
                    self._config.address,
                    self.REG_SYS_ERR,
                )
            )
            if not 0 <= calibration <= 255:
                raise OSError("BNO055 calibration register is invalid")
            if not 0 <= system_status <= 255 or not 0 <= system_error <= 255:
                raise OSError("BNO055 system status register is invalid")
        except Exception:
            self.sensor_ok = False
            raise

        self.sensor_ok = system_error == 0
        return {
            "timestamp": captured_ns / 1_000_000_000.0,
            "heading_deg": float(
                self._signed_16(data[6], data[7]) / 16.0 % 360.0
            ),
            "gyro_dps": self._mapped_gyro(data),
            "calibration": {
                "sys": (calibration >> 6) & 0x03,
                "gyro": (calibration >> 4) & 0x03,
                "accel": (calibration >> 2) & 0x03,
                "mag": calibration & 0x03,
            },
            "sys_status": system_status,
            "sys_error": system_error,
        }

    def read_sample_at(self, captured_monotonic_ns: int) -> Mapping[str, object]:
        """Bind one forced burst to the caller's already-closed tick time."""

        captured_ns = _nonnegative_int(
            captured_monotonic_ns,
            "captured_monotonic_ns",
        )
        return self._read_sample_at(captured_ns)

    def read_sample(self, *, force: bool = False) -> Mapping[str, object]:
        """Read exactly one fresh fused register burst; caching is unsupported."""

        if type(force) is not bool:
            raise TypeError("force must be bool")
        return self._read_sample_at(self._clock_ns())

    def close(self) -> None:
        """Release the bus once and make every later read fail closed."""

        if self._closed:
            return
        self._closed = True
        self.initialized = False
        self.sensor_ok = False
        self._bus.close()


__all__ = [
    "Bno055RegisterBus",
    "NativeBno055Device",
    "NativeBno055DeviceConfig",
]
