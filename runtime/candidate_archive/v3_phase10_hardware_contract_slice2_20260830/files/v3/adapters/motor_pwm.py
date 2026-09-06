"""Pure native V3 DRV8871 physical-output planning contracts."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum


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
class Drv8871PwmPlan:
    """I/O-free duty plan for the two DRV8871 input pins."""

    in1_duty_cycle: float
    in2_duty_cycle: float

    def __post_init__(self) -> None:
        for name, duty_cycle in (
            ("in1_duty_cycle", self.in1_duty_cycle),
            ("in2_duty_cycle", self.in2_duty_cycle),
        ):
            if (
                isinstance(duty_cycle, bool)
                or not isinstance(duty_cycle, (int, float))
                or not math.isfinite(duty_cycle)
                or not 0.0 <= duty_cycle <= 100.0
            ):
                raise ValueError(f"{name} must be finite and within [0, 100]")


def plan_drv8871_pwm(
    config: MotorChannelPhysicalConfig,
    normalized_output: float,
) -> Drv8871PwmPlan:
    """Map one normalized wheel output to native DRV8871 pin duties.

    This function deliberately has no GPIO/backend capability.  It closes the
    physical polarity and coast/brake semantics before a later ACTIVE-ready
    writer is considered.
    """

    if not isinstance(config, MotorChannelPhysicalConfig):
        raise TypeError("config must be MotorChannelPhysicalConfig")
    if (
        isinstance(normalized_output, bool)
        or not isinstance(normalized_output, (int, float))
        or not math.isfinite(normalized_output)
        or not -1.0 <= normalized_output <= 1.0
    ):
        raise ValueError("normalized_output must be finite and within [-1, 1]")

    output = float(normalized_output)
    if config.invert:
        output = -output
    if output == 0.0:
        return Drv8871PwmPlan(0.0, 0.0)

    duty_cycle = abs(output) * 100.0
    if config.pwm_decay_mode is PwmDecayMode.COAST:
        if output > 0.0:
            return Drv8871PwmPlan(duty_cycle, 0.0)
        return Drv8871PwmPlan(0.0, duty_cycle)

    if output > 0.0:
        return Drv8871PwmPlan(100.0, 100.0 - duty_cycle)
    return Drv8871PwmPlan(100.0 - duty_cycle, 100.0)


__all__ = [
    "Drv8871PwmPlan",
    "MotorChannelPhysicalConfig",
    "PwmDecayMode",
    "plan_drv8871_pwm",
]
