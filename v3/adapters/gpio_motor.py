"""Unwired native V3 GPIO sink for one paired DRV8871 motor frame."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from v3.contracts import SafetyDecision

from .motor_pwm import Drv8871MotorFrame, MotorChannelPhysicalConfig


class PwmGpioBackend(Protocol):
    """Injected GPIO operations required by the paired physical sink."""

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


@dataclass(frozen=True, slots=True)
class GpioMotorFrameSinkConfig:
    """Immutable pin ownership and PWM frequency for one paired motor sink."""

    left: MotorChannelPhysicalConfig
    right: MotorChannelPhysicalConfig
    gpio_chip: int = 0
    pwm_frequency_hz: int = 8_000

    def __post_init__(self) -> None:
        if not isinstance(self.left, MotorChannelPhysicalConfig):
            raise TypeError("left must be MotorChannelPhysicalConfig")
        if not isinstance(self.right, MotorChannelPhysicalConfig):
            raise TypeError("right must be MotorChannelPhysicalConfig")
        _require_nonnegative_int(self.gpio_chip, "gpio_chip")
        _require_nonnegative_int(self.pwm_frequency_hz, "pwm_frequency_hz")
        if self.pwm_frequency_hz == 0:
            raise ValueError("pwm_frequency_hz must be positive")
        if len(set(self.pins)) != len(self.pins):
            raise ValueError("left and right motor GPIO pins must be unique")

    @property
    def pins(self) -> tuple[int, int, int, int]:
        return (self.left.in1, self.left.in2, self.right.in1, self.right.in2)


class GpioMotorFrameSink:
    """Own one GPIO handle and apply paired frames fail-closed.

    Each normal write first drives all four pins to zero. ALLOW duties are
    applied only after that break-before-make boundary. A physical write error
    permanently closes this capability after best-effort emergency zeroing.
    """

    __slots__ = ("_backend", "_closed", "_config", "_failed", "_handle")

    def __init__(
        self,
        backend: PwmGpioBackend,
        config: GpioMotorFrameSinkConfig,
    ) -> None:
        if not isinstance(config, GpioMotorFrameSinkConfig):
            raise TypeError("config must be GpioMotorFrameSinkConfig")
        for method_name in (
            "gpiochip_open",
            "gpio_claim_output",
            "tx_pwm",
            "gpiochip_close",
        ):
            if not callable(getattr(backend, method_name, None)):
                raise TypeError(
                    f"backend must provide a callable {method_name} method"
                )

        self._backend = backend
        self._config = config
        self._closed = False
        self._failed = False
        self._handle = self._open_handle()
        try:
            for pin in self._config.pins:
                self._checked_call(
                    self._backend.gpio_claim_output(self._handle, pin),
                    f"claim GPIO {pin}",
                )
                self._apply_pin(pin, 0.0)
        except Exception:
            try:
                self._backend.gpiochip_close(self._handle)
            finally:
                self._closed = True
                self._failed = True
            raise

    @property
    def closed(self) -> bool:
        return self._closed

    @property
    def failed(self) -> bool:
        return self._failed

    def _open_handle(self) -> int:
        handle = self._backend.gpiochip_open(self._config.gpio_chip)
        if not isinstance(handle, int) or isinstance(handle, bool) or handle < 0:
            raise OSError("opening the configured GPIO chip failed")
        return handle

    @staticmethod
    def _checked_call(result: object, operation: str) -> None:
        if isinstance(result, int) and not isinstance(result, bool) and result < 0:
            raise OSError(f"{operation} failed with status {result}")

    def _apply_pin(self, pin: int, duty_cycle: float) -> None:
        self._checked_call(
            self._backend.tx_pwm(
                self._handle,
                pin,
                self._config.pwm_frequency_hz,
                duty_cycle,
            ),
            f"PWM write on GPIO {pin}",
        )

    def _apply_zero(self) -> None:
        for pin in self._config.pins:
            self._apply_pin(pin, 0.0)

    def _target_duties(
        self,
        frame: Drv8871MotorFrame,
    ) -> tuple[tuple[int, float], ...]:
        return (
            (self._config.left.in1, frame.left.in1_duty_cycle),
            (self._config.left.in2, frame.left.in2_duty_cycle),
            (self._config.right.in1, frame.right.in1_duty_cycle),
            (self._config.right.in2, frame.right.in2_duty_cycle),
        )

    def _emergency_zero_and_close(self) -> None:
        self._failed = True
        for pin in self._config.pins:
            try:
                self._apply_pin(pin, 0.0)
            except Exception:
                pass
        try:
            self._backend.gpiochip_close(self._handle)
        except Exception:
            pass
        self._closed = True

    def write(self, frame: Drv8871MotorFrame) -> None:
        """Apply one frame or permanently close the sink on physical failure."""

        if self._closed:
            raise OSError("the GPIO motor frame sink is closed")
        if not isinstance(frame, Drv8871MotorFrame):
            raise TypeError("frame must be Drv8871MotorFrame")
        try:
            self._apply_zero()
            if frame.safety_decision is SafetyDecision.ALLOW:
                for pin, duty_cycle in self._target_duties(frame):
                    if duty_cycle != 0.0:
                        self._apply_pin(pin, duty_cycle)
        except Exception:
            self._emergency_zero_and_close()
            raise

    def close(self) -> None:
        """Zero the paired output and release the handle exactly once."""

        if self._closed:
            return
        try:
            self._apply_zero()
        except Exception:
            self._emergency_zero_and_close()
            raise
        try:
            self._checked_call(
                self._backend.gpiochip_close(self._handle),
                "close GPIO chip",
            )
        finally:
            self._closed = True


__all__ = [
    "GpioMotorFrameSink",
    "GpioMotorFrameSinkConfig",
    "PwmGpioBackend",
]
