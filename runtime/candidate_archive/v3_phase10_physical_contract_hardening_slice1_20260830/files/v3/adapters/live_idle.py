"""Physical zero-only edge adapters for the explicit V3 IDLE cutover."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Protocol

from v3.contracts import (
    CommandMode,
    CommandRequest,
    DeviceHealth,
    DeviceHealthState,
    FinalActuation,
    RawDeviceBatch,
    SafetyDecision,
    TickContext,
)


class GpioBackend(Protocol):
    """The small injected subset of ``lgpio`` used by the live edge."""

    def gpiochip_open(self, chip: int) -> int: ...

    def gpio_claim_output(self, handle: int, pin: int) -> object: ...

    def tx_pwm(
        self,
        handle: int,
        pin: int,
        frequency_hz: int,
        duty_cycle: float,
    ) -> object: ...

    def gpiochip_close(self, handle: int) -> object: ...


def _require_nonnegative_int(value: int, name: str) -> None:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")


class PwmDecayMode(str, Enum):
    """Native V3 DRV8871 PWM decay semantics."""

    COAST = "coast"
    BRAKE = "brake"


@dataclass(frozen=True, slots=True)
class MotorChannelPhysicalConfig:
    """Closed physical contract for one native V3 motor channel."""

    in1: int
    in2: int
    invert: bool = False
    pwm_decay_mode: PwmDecayMode = PwmDecayMode.COAST

    def __post_init__(self) -> None:
        _require_nonnegative_int(self.in1, "MotorChannelPhysicalConfig.in1")
        _require_nonnegative_int(self.in2, "MotorChannelPhysicalConfig.in2")
        if self.in1 == self.in2:
            raise ValueError("one motor channel requires two distinct pins")
        if type(self.invert) is not bool:
            raise ValueError("MotorChannelPhysicalConfig.invert must be bool")
        if not isinstance(self.pwm_decay_mode, PwmDecayMode):
            raise ValueError(
                "MotorChannelPhysicalConfig.pwm_decay_mode must be PwmDecayMode"
            )


@dataclass(frozen=True, slots=True)
class GpioZeroWriterConfig:
    """Closed hardware configuration for the zero-only physical writer."""

    left: MotorChannelPhysicalConfig
    right: MotorChannelPhysicalConfig
    gpio_chip: int = 0
    pwm_frequency_hz: int = 8_000

    def __post_init__(self) -> None:
        _require_nonnegative_int(self.gpio_chip, "GpioZeroWriterConfig.gpio_chip")
        _require_nonnegative_int(
            self.pwm_frequency_hz,
            "GpioZeroWriterConfig.pwm_frequency_hz",
        )
        if self.pwm_frequency_hz == 0:
            raise ValueError("pwm_frequency_hz must be positive")
        pins = (self.left.in1, self.left.in2, self.right.in1, self.right.in2)
        if len(set(pins)) != len(pins):
            raise ValueError("left and right motor GPIO pins must be unique")


class LiveIdleWriteRejected(RuntimeError):
    """A caller attempted to use the IDLE cutover edge for motion."""


class GpioZeroMotorWriter:
    """Apply only zero STOP/FAULT commands to one claimed GPIO chip.

    The object owns the physical GPIO handle.  It deliberately has no code path
    that converts a non-zero V3 command into a hardware duty cycle.
    """

    __slots__ = ("_backend", "_closed", "_config", "_handle", "_write_failed")

    def __init__(self, backend: GpioBackend, config: GpioZeroWriterConfig) -> None:
        self._backend = backend
        self._config = config
        self._closed = False
        self._write_failed = False
        self._handle = self._open_handle()
        try:
            for pin in self._pins:
                self._checked_call(
                    self._backend.gpio_claim_output(self._handle, pin),
                    f"claim GPIO {pin}",
                )
                self._apply_zero_pin(pin)
        except Exception:
            try:
                self._backend.gpiochip_close(self._handle)
            finally:
                self._closed = True
            raise

    @property
    def _pins(self) -> tuple[int, int, int, int]:
        return (
            self._config.left.in1,
            self._config.left.in2,
            self._config.right.in1,
            self._config.right.in2,
        )

    @property
    def closed(self) -> bool:
        return self._closed

    def _open_handle(self) -> int:
        handle = self._backend.gpiochip_open(self._config.gpio_chip)
        if not isinstance(handle, int) or isinstance(handle, bool) or handle < 0:
            raise OSError("opening the configured GPIO chip failed")
        return handle

    @staticmethod
    def _checked_call(result: object, operation: str) -> None:
        if isinstance(result, int) and not isinstance(result, bool) and result < 0:
            raise OSError(f"{operation} failed with status {result}")

    def _apply_zero(self) -> None:
        for pin in self._pins:
            self._apply_zero_pin(pin)

    def _apply_zero_pin(self, pin: int) -> None:
        self._checked_call(
            self._backend.tx_pwm(
                self._handle,
                pin,
                self._config.pwm_frequency_hz,
                0.0,
            ),
            f"zero PWM on GPIO {pin}",
        )

    def write(self, command: FinalActuation) -> None:
        if self._closed:
            raise OSError("the live IDLE motor writer is closed")
        if not isinstance(command, FinalActuation):
            self._write_failed = True
            raise TypeError("the live IDLE writer requires FinalActuation")
        if (
            command.enabled
            or command.safety_decision is SafetyDecision.ALLOW
            or command.left_output != 0.0
            or command.right_output != 0.0
        ):
            self._write_failed = True
            raise LiveIdleWriteRejected(
                "the live IDLE cutover writer rejects every motion-capable command"
            )
        try:
            self._apply_zero()
        except Exception:
            self._write_failed = True
            raise

    def close(self) -> None:
        if self._closed:
            return
        try:
            if not self._write_failed:
                try:
                    self._apply_zero()
                except Exception:
                    self._write_failed = True
                    raise
        finally:
            try:
                self._checked_call(
                    self._backend.gpiochip_close(self._handle),
                    "close GPIO chip",
                )
            finally:
                self._closed = True


@dataclass(frozen=True, slots=True)
class LiveIdleDeviceReader:
    """Close the minimal L0 health snapshot required by the IDLE slice."""

    device_id: str = "v3-live-motor-driver"

    def read(self, context: TickContext) -> RawDeviceBatch:
        return RawDeviceBatch(
            context=context,
            samples=(),
            device_health=(DeviceHealth(self.device_id, DeviceHealthState.OK),),
        )


@dataclass(frozen=True, slots=True)
class LockedStopCommandGateway:
    """Expose no external command authority during the IDLE cutover."""

    def snapshot(self, context: TickContext) -> CommandRequest:
        return CommandRequest(
            context=context,
            command_id=f"live-idle-stop-{context.tick_id}",
            mode=CommandMode.STOP,
            goal=(),
            expiry_tick=context.tick_id,
        )


__all__ = [
    "GpioBackend",
    "GpioZeroMotorWriter",
    "GpioZeroWriterConfig",
    "LiveIdleDeviceReader",
    "LiveIdleWriteRejected",
    "LockedStopCommandGateway",
    "MotorChannelPhysicalConfig",
    "PwmDecayMode",
]
