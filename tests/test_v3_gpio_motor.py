import pytest

from v3.adapters.gpio_motor import GpioMotorFrameSink, GpioMotorFrameSinkConfig
from v3.adapters.motor_pwm import (
    MotorChannelPhysicalConfig,
    PwmDecayMode,
    plan_final_actuation,
)
from v3.contracts import FinalActuation, SafetyDecision, TickContext


class FakePwmGpio:
    def __init__(self) -> None:
        self.calls: list[tuple[object, ...]] = []
        self.fail_claim_call: int | None = None
        self.fail_pwm_call: int | None = None
        self._claim_calls = 0
        self._pwm_calls = 0

    @property
    def pwm_calls(self) -> int:
        return self._pwm_calls

    def gpiochip_open(self, chip: int) -> int:
        self.calls.append(("open", chip))
        return 7

    def gpio_claim_output(self, handle: int, pin: int) -> int:
        self._claim_calls += 1
        self.calls.append(("claim", handle, pin))
        if self.fail_claim_call == self._claim_calls:
            raise OSError("injected GPIO claim failure")
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


def _frame(
    decision: SafetyDecision,
    *,
    left_output: float = 0.0,
    right_output: float = 0.0,
    enabled: bool = False,
):
    config = _config()
    command = FinalActuation(
        context=TickContext(3, 5_000),
        left_output=left_output,
        right_output=right_output,
        enabled=enabled,
        safety_decision=decision,
        latch_state="CLEAR",
        reason=None if decision is SafetyDecision.ALLOW else "STOPPED",
    )
    return plan_final_actuation(config.left, config.right, command)


def test_construction_claims_one_handle_and_zeros_each_pin_immediately():
    gpio = FakePwmGpio()

    sink = GpioMotorFrameSink(gpio, _config())

    assert gpio.calls == [
        ("open", 0),
        ("claim", 7, 12),
        ("pwm", 7, 12, 8_000, 0.0),
        ("claim", 7, 13),
        ("pwm", 7, 13, 8_000, 0.0),
        ("claim", 7, 18),
        ("pwm", 7, 18, 8_000, 0.0),
        ("claim", 7, 19),
        ("pwm", 7, 19, 8_000, 0.0),
    ]
    assert not sink.closed
    assert not sink.failed


def test_allow_frame_is_applied_only_after_break_before_make_zeroing():
    gpio = FakePwmGpio()
    sink = GpioMotorFrameSink(gpio, _config())
    gpio.calls.clear()

    sink.write(
        _frame(
            SafetyDecision.ALLOW,
            left_output=0.25,
            right_output=-0.5,
            enabled=True,
        )
    )

    assert gpio.calls == [
        ("pwm", 7, 12, 8_000, 0.0),
        ("pwm", 7, 13, 8_000, 0.0),
        ("pwm", 7, 18, 8_000, 0.0),
        ("pwm", 7, 19, 8_000, 0.0),
        ("pwm", 7, 12, 8_000, 25.0),
        ("pwm", 7, 18, 8_000, 100.0),
        ("pwm", 7, 19, 8_000, 50.0),
    ]
    assert not sink.closed


@pytest.mark.parametrize("decision", [SafetyDecision.STOP, SafetyDecision.FAULT])
def test_stop_and_fault_frames_write_only_all_zero_duties(
    decision: SafetyDecision,
):
    gpio = FakePwmGpio()
    sink = GpioMotorFrameSink(gpio, _config())
    gpio.calls.clear()

    sink.write(_frame(decision))

    assert gpio.calls == [
        ("pwm", 7, 12, 8_000, 0.0),
        ("pwm", 7, 13, 8_000, 0.0),
        ("pwm", 7, 18, 8_000, 0.0),
        ("pwm", 7, 19, 8_000, 0.0),
    ]


def test_physical_write_failure_emergency_zeros_closes_and_never_retries():
    gpio = FakePwmGpio()
    sink = GpioMotorFrameSink(gpio, _config())
    gpio.fail_pwm_call = gpio.pwm_calls + 2
    gpio.calls.clear()

    with pytest.raises(OSError, match="injected GPIO PWM failure"):
        sink.write(
            _frame(
                SafetyDecision.ALLOW,
                left_output=0.25,
                right_output=0.25,
                enabled=True,
            )
        )

    assert gpio.calls == [
        ("pwm", 7, 12, 8_000, 0.0),
        ("pwm", 7, 13, 8_000, 0.0),
        ("pwm", 7, 12, 8_000, 0.0),
        ("pwm", 7, 13, 8_000, 0.0),
        ("pwm", 7, 18, 8_000, 0.0),
        ("pwm", 7, 19, 8_000, 0.0),
        ("close", 7),
    ]
    assert sink.closed
    assert sink.failed
    before = tuple(gpio.calls)
    with pytest.raises(OSError, match="closed"):
        sink.write(_frame(SafetyDecision.STOP))
    assert tuple(gpio.calls) == before


def test_initialization_failure_closes_the_only_open_handle():
    gpio = FakePwmGpio()
    gpio.fail_claim_call = 3

    with pytest.raises(OSError, match="claim failure"):
        GpioMotorFrameSink(gpio, _config())

    assert gpio.calls == [
        ("open", 0),
        ("claim", 7, 12),
        ("pwm", 7, 12, 8_000, 0.0),
        ("claim", 7, 13),
        ("pwm", 7, 13, 8_000, 0.0),
        ("claim", 7, 18),
        ("close", 7),
    ]


def test_close_zeros_then_releases_the_handle_and_is_idempotent():
    gpio = FakePwmGpio()
    sink = GpioMotorFrameSink(gpio, _config())
    gpio.calls.clear()

    sink.close()
    sink.close()

    assert gpio.calls == [
        ("pwm", 7, 12, 8_000, 0.0),
        ("pwm", 7, 13, 8_000, 0.0),
        ("pwm", 7, 18, 8_000, 0.0),
        ("pwm", 7, 19, 8_000, 0.0),
        ("close", 7),
    ]
    assert sink.closed
    assert not sink.failed


def test_invalid_config_and_backend_fail_before_gpio_is_opened():
    with pytest.raises(ValueError, match="GPIO pins must be unique"):
        GpioMotorFrameSinkConfig(
            MotorChannelPhysicalConfig(1, 2),
            MotorChannelPhysicalConfig(2, 3),
        )

    with pytest.raises(TypeError, match="gpiochip_open"):
        GpioMotorFrameSink(object(), _config())  # type: ignore[arg-type]
