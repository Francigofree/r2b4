from unittest.mock import patch

import pytest

from driver.motor import AlbaMotor, MotorChannelConfig
from v3.adapters.motor_pwm import (
    Drv8871PwmPlan,
    MotorChannelPhysicalConfig,
    PwmDecayMode,
    plan_drv8871_pwm,
)


class _FakeLgpio:
    def __init__(self) -> None:
        self.duties: dict[int, float] = {}

    def gpiochip_open(self, chip: int) -> int:
        assert chip == 0
        return 1

    def gpio_claim_output(self, handle: int, pin: int) -> None:
        assert handle == 1
        self.duties.setdefault(pin, 0.0)

    def gpiochip_close(self, handle: int) -> None:
        assert handle == 1

    def tx_pwm(
        self,
        handle: int,
        pin: int,
        frequency_hz: int,
        duty_cycle: float,
    ) -> None:
        assert handle == 1
        assert frequency_hz == 8_000
        self.duties[pin] = float(duty_cycle)


def _legacy_plan(
    *,
    decay_mode: PwmDecayMode,
    invert: bool,
    normalized_output: float,
) -> Drv8871PwmPlan:
    fake = _FakeLgpio()
    config = MotorChannelConfig(
        side_key="parity-test",
        gpio_in1=12,
        gpio_in2=13,
        invert=invert,
        pwm_decay_mode=decay_mode.value,
    )
    with patch("driver.motor.lgpio", fake):
        motor = AlbaMotor(config)
        motor.set_pwm(normalized_output)
        observed = Drv8871PwmPlan(fake.duties[12], fake.duties[13])
        motor.close()
    return observed


@pytest.mark.parametrize("decay_mode", tuple(PwmDecayMode))
@pytest.mark.parametrize("invert", (False, True))
@pytest.mark.parametrize(
    "normalized_output",
    (-1.0, -0.60, -0.25, 0.0, 0.25, 0.60, 1.0),
)
def test_native_planner_matches_canonical_legacy_drv8871_semantics(
    decay_mode: PwmDecayMode,
    invert: bool,
    normalized_output: float,
):
    native = plan_drv8871_pwm(
        MotorChannelPhysicalConfig(
            12,
            13,
            invert=invert,
            pwm_decay_mode=decay_mode,
        ),
        normalized_output,
    )

    assert native == _legacy_plan(
        decay_mode=decay_mode,
        invert=invert,
        normalized_output=normalized_output,
    )
