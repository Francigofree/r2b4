import pytest

from v3.adapters.gpio_motor import GpioMotorFrameSinkConfig
from v3.adapters.motor_pwm import MotorChannelPhysicalConfig, PwmDecayMode
from v3.composition.motor_output import NativeMotorOutputComposition
from v3.contracts import FinalActuation, SafetyDecision, TickContext


class FakePwmGpio:
    def __init__(self) -> None:
        self.calls: list[tuple[object, ...]] = []
        self.fail_pwm_call: int | None = None
        self._pwm_calls = 0

    @property
    def pwm_calls(self) -> int:
        return self._pwm_calls

    def gpiochip_open(self, chip: int) -> int:
        self.calls.append(("open", chip))
        return 7

    def gpio_claim_output(self, handle: int, pin: int) -> int:
        self.calls.append(("claim", handle, pin))
        return 0

    def tx_pwm(
        self,
        handle: int,
        pin: int,
        frequency_hz: int,
        duty_cycle: float,
    ) -> int:
        self._pwm_calls += 1
        self.calls.append(("pwm", handle, pin, frequency_hz, duty_cycle))
        if self.fail_pwm_call == self._pwm_calls:
            raise OSError("injected GPIO PWM failure")
        return 0

    def gpiochip_close(self, handle: int) -> int:
        self.calls.append(("close", handle))
        return 0


def _config() -> GpioMotorFrameSinkConfig:
    return GpioMotorFrameSinkConfig(
        left=MotorChannelPhysicalConfig(12, 13),
        right=MotorChannelPhysicalConfig(
            18,
            19,
            invert=True,
            pwm_decay_mode=PwmDecayMode.BRAKE,
        ),
    )


def _command(
    decision: SafetyDecision,
    *,
    left_output: float = 0.0,
    right_output: float = 0.0,
    enabled: bool = False,
) -> FinalActuation:
    return FinalActuation(
        context=TickContext(9, 15_000),
        left_output=left_output,
        right_output=right_output,
        enabled=enabled,
        safety_decision=decision,
        latch_state="CLEAR",
        reason=None if decision is SafetyDecision.ALLOW else "STOPPED",
    )


def test_one_config_closes_final_actuation_to_the_owned_gpio_pin_mapping():
    gpio = FakePwmGpio()
    output = NativeMotorOutputComposition(gpio, _config())
    gpio.calls.clear()

    result = output.write(
        _command(
            SafetyDecision.ALLOW,
            left_output=0.25,
            right_output=-0.5,
            enabled=True,
        )
    )

    assert result is None
    assert gpio.calls == [
        ("pwm", 7, 12, 8_000, 0.0),
        ("pwm", 7, 13, 8_000, 0.0),
        ("pwm", 7, 18, 8_000, 0.0),
        ("pwm", 7, 19, 8_000, 0.0),
        ("pwm", 7, 12, 8_000, 25.0),
        ("pwm", 7, 18, 8_000, 100.0),
        ("pwm", 7, 19, 8_000, 50.0),
    ]
    assert not output.closed
    assert not output.failed


@pytest.mark.parametrize("decision", [SafetyDecision.STOP, SafetyDecision.FAULT])
def test_stop_and_fault_remain_one_all_zero_frame_sink_call(
    decision: SafetyDecision,
):
    gpio = FakePwmGpio()
    output = NativeMotorOutputComposition(gpio, _config())
    gpio.calls.clear()

    output.write(_command(decision))

    assert gpio.calls == [
        ("pwm", 7, 12, 8_000, 0.0),
        ("pwm", 7, 13, 8_000, 0.0),
        ("pwm", 7, 18, 8_000, 0.0),
        ("pwm", 7, 19, 8_000, 0.0),
    ]


def test_physical_error_propagates_and_closes_the_only_output_capability():
    gpio = FakePwmGpio()
    output = NativeMotorOutputComposition(gpio, _config())
    gpio.fail_pwm_call = gpio.pwm_calls + 1
    gpio.calls.clear()

    with pytest.raises(OSError, match="injected GPIO PWM failure"):
        output.write(
            _command(
                SafetyDecision.ALLOW,
                left_output=0.2,
                right_output=0.2,
                enabled=True,
            )
        )

    assert output.closed
    assert output.failed
    assert gpio.calls[-1] == ("close", 7)
    before = tuple(gpio.calls)
    with pytest.raises(OSError, match="closed"):
        output.write(_command(SafetyDecision.STOP))
    assert tuple(gpio.calls) == before


def test_invalid_command_never_reaches_gpio_and_does_not_close_the_owner():
    gpio = FakePwmGpio()
    output = NativeMotorOutputComposition(gpio, _config())
    gpio.calls.clear()

    with pytest.raises(TypeError, match="command must be FinalActuation"):
        output.write(object())  # type: ignore[arg-type]

    assert gpio.calls == []
    assert not output.closed


def test_close_zeros_releases_once_and_no_runtime_authority_is_exposed():
    gpio = FakePwmGpio()
    output = NativeMotorOutputComposition(gpio, _config())
    gpio.calls.clear()

    output.close()
    output.close()

    assert gpio.calls == [
        ("pwm", 7, 12, 8_000, 0.0),
        ("pwm", 7, 13, 8_000, 0.0),
        ("pwm", 7, 18, 8_000, 0.0),
        ("pwm", 7, 19, 8_000, 0.0),
        ("close", 7),
    ]
    assert output.closed
    assert not hasattr(output, "activate")
    assert not hasattr(output, "tick")
    assert not hasattr(output, "sink")
    assert not hasattr(output, "writer")
