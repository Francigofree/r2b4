import pytest

from v3.adapters.motor_pwm import (
    Drv8871PwmPlan,
    MotorChannelPhysicalConfig,
    PwmDecayMode,
    plan_drv8871_pwm,
)


@pytest.mark.parametrize(
    ("decay_mode", "invert", "normalized_output", "expected"),
    (
        (PwmDecayMode.COAST, False, 0.60, Drv8871PwmPlan(60.0, 0.0)),
        (PwmDecayMode.COAST, False, -0.25, Drv8871PwmPlan(0.0, 25.0)),
        (PwmDecayMode.COAST, True, 0.60, Drv8871PwmPlan(0.0, 60.0)),
        (PwmDecayMode.COAST, True, -0.25, Drv8871PwmPlan(25.0, 0.0)),
        (PwmDecayMode.BRAKE, False, 0.60, Drv8871PwmPlan(100.0, 40.0)),
        (PwmDecayMode.BRAKE, False, -0.25, Drv8871PwmPlan(75.0, 100.0)),
        (PwmDecayMode.BRAKE, True, 0.60, Drv8871PwmPlan(40.0, 100.0)),
        (PwmDecayMode.BRAKE, True, -0.25, Drv8871PwmPlan(100.0, 75.0)),
        (PwmDecayMode.BRAKE, False, 0.0, Drv8871PwmPlan(0.0, 0.0)),
    ),
)
def test_native_drv8871_polarity_and_decay_characterization(
    decay_mode: PwmDecayMode,
    invert: bool,
    normalized_output: float,
    expected: Drv8871PwmPlan,
):
    actual = plan_drv8871_pwm(
        MotorChannelPhysicalConfig(
            12,
            13,
            invert=invert,
            pwm_decay_mode=decay_mode,
        ),
        normalized_output,
    )

    assert actual == expected
