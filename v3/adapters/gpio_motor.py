"""Native V3 GPIO sink for one paired DRV8871 motor frame."""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol

from v3.contracts import SafetyDecision

from .motor_pwm import (
    Drv8871MotorFrame,
    Drv8871PwmPlan,
    MotorChannelPhysicalConfig,
    PwmDecayMode,
)


class PwmGpioBackend(Protocol):
    """Injected GPIO operations required by the paired physical sink."""

    def gpiochip_open(self, chip: int) -> int: ...

    def gpio_claim_output(
        self,
        handle: int,
        pin: int,
        initial_level: int,
    ) -> object: ...

    def gpio_write(self, handle: int, pin: int, level: int) -> object: ...

    def gpio_read(self, handle: int, pin: int) -> object: ...

    def gpio_free(self, handle: int, pin: int) -> object: ...

    def tx_busy(self, handle: int, pin: int, kind: int) -> object: ...

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

    The first ALLOW and every direction change use a verified
    break-before-make boundary. Same-direction ALLOW updates keep the existing
    PWM transmitters active and change only their duties. STOP, FAULT and close
    retain the verified LOW state long enough for the DRV8871 to enter sleep
    before the GPIO handle is released. A physical error permanently closes
    this capability after best-effort emergency hard-low handling.
    """

    _DRV8871_SLEEP_HOLD_S = 0.002

    __slots__ = (
        "_backend",
        "_claimed_pins",
        "_closed",
        "_config",
        "_failed",
        "_handle",
        "_last_allow_directions",
        "_sleep",
    )

    def __init__(
        self,
        backend: PwmGpioBackend,
        config: GpioMotorFrameSinkConfig,
        *,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        if not isinstance(config, GpioMotorFrameSinkConfig):
            raise TypeError("config must be GpioMotorFrameSinkConfig")
        if not callable(sleep):
            raise TypeError("sleep must be callable")
        for method_name in (
            "gpiochip_open",
            "gpio_claim_output",
            "gpio_write",
            "gpio_read",
            "gpio_free",
            "tx_busy",
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
        self._last_allow_directions: tuple[int, int] | None = None
        self._sleep = sleep
        self._claimed_pins: list[int] = []
        self._handle = self._open_handle()
        try:
            initialization_error: Exception | None = None
            for pin in self._config.pins:
                # Include the attempted pin in emergency handling: a backend
                # may raise after the kernel has already accepted the claim.
                self._claimed_pins.append(pin)
                try:
                    self._checked_call(
                        self._backend.gpio_claim_output(self._handle, pin, 0),
                        f"claim GPIO {pin}",
                    )
                    self._cancel_pwm(pin)
                    self._write_low(pin)
                    self._verify_low(pin)
                except Exception as exc:
                    if initialization_error is None:
                        initialization_error = exc
            if initialization_error is not None:
                raise initialization_error
            self._hold_and_reverify_low()
        except Exception:
            self._emergency_hard_low_and_close()
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

    def _cancel_pwm(self, pin: int) -> None:
        # lgpio rejects the nominal 0 Hz cancellation call when no
        # transmitter record exists. Querying the single-writer-owned record
        # first makes cancellation both explicit and valid on that backend.
        busy = self._backend.tx_busy(self._handle, pin, 0)
        if (
            not isinstance(busy, int)
            or isinstance(busy, bool)
            or busy not in (0, 1)
        ):
            raise OSError(f"GPIO {pin} PWM busy query returned {busy!r}")
        if busy:
            self._checked_call(
                self._backend.tx_pwm(self._handle, pin, 0, 0.0),
                f"PWM cancel on GPIO {pin}",
            )

    def _write_low(self, pin: int) -> None:
        self._checked_call(
            self._backend.gpio_write(self._handle, pin, 0),
            f"LOW write on GPIO {pin}",
        )

    def _verify_low(self, pin: int) -> None:
        level = self._backend.gpio_read(self._handle, pin)
        if (
            not isinstance(level, int)
            or isinstance(level, bool)
            or level not in (0, 1)
        ):
            raise OSError(f"GPIO {pin} readback returned invalid level {level!r}")
        if level != 0:
            raise OSError(f"GPIO {pin} remained HIGH after the LOW write")

    def _hold_and_reverify_low(self) -> None:
        self._sleep(self._DRV8871_SLEEP_HOLD_S)
        for pin in self._claimed_pins:
            self._verify_low(pin)

    def _hard_low(self, *, enter_sleep: bool) -> None:
        for pin in self._claimed_pins:
            self._cancel_pwm(pin)
        for pin in self._claimed_pins:
            self._write_low(pin)
        for pin in self._claimed_pins:
            self._verify_low(pin)
        if enter_sleep:
            self._hold_and_reverify_low()

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

    @staticmethod
    def _channel_direction(
        plan: Drv8871PwmPlan,
        config: MotorChannelPhysicalConfig,
    ) -> int:
        """Return physical pin polarity without reconstructing logical motion."""

        in1 = plan.in1_duty_cycle
        in2 = plan.in2_duty_cycle
        if in1 == 0.0 and in2 == 0.0:
            return 0
        if config.pwm_decay_mode is PwmDecayMode.COAST:
            if in1 > 0.0 and in2 == 0.0:
                return 1
            if in1 == 0.0 and in2 > 0.0:
                return -1
        else:
            if in1 == 100.0 and 0.0 <= in2 < 100.0:
                return 1
            if in2 == 100.0 and 0.0 <= in1 < 100.0:
                return -1
        raise ValueError("ALLOW motor frame has invalid DRV8871 direction duties")

    def _allow_directions(self, frame: Drv8871MotorFrame) -> tuple[int, int]:
        return (
            self._channel_direction(frame.left, self._config.left),
            self._channel_direction(frame.right, self._config.right),
        )

    def _apply_allow_duties(self, frame: Drv8871MotorFrame) -> None:
        for pin, duty_cycle in self._target_duties(frame):
            if duty_cycle != 0.0:
                self._apply_pin(pin, duty_cycle)

    def _emergency_hard_low_and_close(self) -> None:
        self._failed = True
        self._last_allow_directions = None
        for pin in self._claimed_pins:
            try:
                self._cancel_pwm(pin)
            except Exception:
                # gpio_free synchronously invalidates lgpio's transmitter
                # record. Reclaim at LOW so a broken busy/cancel path cannot
                # leave a software PWM worker with authority over the pin.
                try:
                    self._backend.gpio_free(self._handle, pin)
                except Exception:
                    pass
                try:
                    self._backend.gpio_claim_output(self._handle, pin, 0)
                except Exception:
                    pass
        for pin in self._claimed_pins:
            try:
                self._write_low(pin)
            except Exception:
                pass
        for pin in self._claimed_pins:
            try:
                self._verify_low(pin)
            except Exception:
                pass
        try:
            self._sleep(self._DRV8871_SLEEP_HOLD_S)
        except Exception:
            pass
        for pin in self._claimed_pins:
            try:
                self._verify_low(pin)
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
            if frame.safety_decision is SafetyDecision.ALLOW:
                directions = self._allow_directions(frame)
                if directions != self._last_allow_directions:
                    self._hard_low(enter_sleep=False)
                self._apply_allow_duties(frame)
                self._last_allow_directions = directions
            else:
                self._hard_low(enter_sleep=True)
                self._last_allow_directions = None
        except Exception:
            self._emergency_hard_low_and_close()
            raise

    def close(self) -> None:
        """Cancel PWM, hold verified LOW, and release the handle once."""

        if self._closed:
            return
        try:
            self._hard_low(enter_sleep=True)
            self._last_allow_directions = None
        except Exception:
            self._emergency_hard_low_and_close()
            raise
        try:
            self._checked_call(
                self._backend.gpiochip_close(self._handle),
                "close GPIO chip",
            )
        except Exception:
            self._emergency_hard_low_and_close()
            raise
        finally:
            self._closed = True


__all__ = [
    "GpioMotorFrameSink",
    "GpioMotorFrameSinkConfig",
    "PwmGpioBackend",
]
