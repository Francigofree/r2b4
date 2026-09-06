import pytest

from v3.adapters.gpio_motor import GpioMotorFrameSink, GpioMotorFrameSinkConfig
from v3.adapters.motor_pwm import (
    MotorChannelPhysicalConfig,
    PwmDecayMode,
    plan_final_actuation,
)
from v3.contracts import FinalActuation, SafetyDecision, TickContext


PINS = (12, 13, 18, 19)


class FakePwmGpio:
    def __init__(self) -> None:
        self.calls: list[tuple[object, ...]] = []
        self.fail_claim_call: int | None = None
        self.fail_busy_pin: int | None = None
        self.fail_pwm_call: int | None = None
        self.fail_write_call: int | None = None
        self.high_read_call: int | None = None
        self._claim_calls = 0
        self._pwm_calls = 0
        self._write_calls = 0
        self._read_calls = 0
        self._levels: dict[int, int] = {}
        self._pwm_busy: set[int] = set()

    @property
    def pwm_calls(self) -> int:
        return self._pwm_calls

    @property
    def read_calls(self) -> int:
        return self._read_calls

    def gpiochip_open(self, chip: int) -> int:
        self.calls.append(("open", chip))
        return 7

    def gpio_claim_output(self, handle: int, pin: int, initial_level: int) -> int:
        self._claim_calls += 1
        self.calls.append(("claim", handle, pin, initial_level))
        # Model a backend that can report an error after the kernel accepted
        # the requested safe initial level.
        self._levels[pin] = initial_level
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
        if frequency_hz == 0:
            self._pwm_busy.discard(pin)
        elif duty_cycle != 0.0:
            self._pwm_busy.add(pin)
        return 0

    def tx_busy(self, handle: int, pin: int, kind: int) -> int:
        self.calls.append(("busy", handle, pin, kind))
        if self.fail_busy_pin == pin:
            raise OSError("injected GPIO PWM busy failure")
        return int(pin in self._pwm_busy)

    def gpio_write(self, handle: int, pin: int, level: int) -> int:
        self._write_calls += 1
        self.calls.append(("write", handle, pin, level))
        if self.fail_write_call == self._write_calls:
            raise OSError("injected GPIO write failure")
        self._levels[pin] = level
        return 0

    def gpio_read(self, handle: int, pin: int) -> int:
        self._read_calls += 1
        self.calls.append(("read", handle, pin))
        if self.high_read_call == self._read_calls:
            return 1
        return self._levels[pin]

    def gpio_free(self, handle: int, pin: int) -> int:
        self.calls.append(("free", handle, pin))
        self._pwm_busy.discard(pin)
        return 0

    def gpiochip_close(self, handle: int) -> int:
        self.calls.append(("close", handle))
        return 0


class FakeSleep:
    def __init__(self, calls: list[tuple[object, ...]]) -> None:
        self._calls = calls

    def __call__(self, duration_s: float) -> None:
        self._calls.append(("sleep", duration_s))


def _sink(gpio: FakePwmGpio) -> GpioMotorFrameSink:
    return GpioMotorFrameSink(gpio, _config(), sleep=FakeSleep(gpio.calls))


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


def _hard_low_calls(
    *,
    active_pins: tuple[int, ...] = (),
    sleep: bool,
) -> list[tuple[object, ...]]:
    calls: list[tuple[object, ...]] = []
    for pin in PINS:
        calls.append(("busy", 7, pin, 0))
        if pin in active_pins:
            calls.append(("pwm", 7, pin, 0, 0.0))
    calls.extend(("write", 7, pin, 0) for pin in PINS)
    calls.extend(("read", 7, pin) for pin in PINS)
    if sleep:
        calls.append(("sleep", 0.002))
        calls.extend(("read", 7, pin) for pin in PINS)
    return calls


def test_construction_claims_low_then_cancels_writes_and_verifies_each_pin():
    gpio = FakePwmGpio()

    sink = _sink(gpio)

    expected: list[tuple[object, ...]] = [("open", 0)]
    for pin in PINS:
        expected.extend(
            (
                ("claim", 7, pin, 0),
                ("busy", 7, pin, 0),
                ("write", 7, pin, 0),
                ("read", 7, pin),
            )
        )
    expected.append(("sleep", 0.002))
    expected.extend(("read", 7, pin) for pin in PINS)
    assert gpio.calls == expected
    assert not sink.closed
    assert not sink.failed


def test_allow_frame_is_applied_only_after_verified_break_before_make_low():
    gpio = FakePwmGpio()
    sink = _sink(gpio)
    gpio.calls.clear()

    sink.write(
        _frame(
            SafetyDecision.ALLOW,
            left_output=0.25,
            right_output=-0.5,
            enabled=True,
        )
    )

    assert gpio.calls == _hard_low_calls(sleep=False) + [
        ("pwm", 7, 12, 8_000, 25.0),
        ("pwm", 7, 18, 8_000, 100.0),
        ("pwm", 7, 19, 8_000, 50.0),
    ]
    assert not sink.closed


@pytest.mark.parametrize("decision", [SafetyDecision.STOP, SafetyDecision.FAULT])
def test_stop_and_fault_cancel_pwm_hold_verified_low_and_keep_ownership(
    decision: SafetyDecision,
):
    gpio = FakePwmGpio()
    sink = _sink(gpio)
    sink.write(
        _frame(
            SafetyDecision.ALLOW,
            left_output=0.25,
            right_output=0.25,
            enabled=True,
        )
    )
    gpio.calls.clear()

    sink.write(_frame(decision))

    assert gpio.calls == _hard_low_calls(
        active_pins=(12, 18, 19),
        sleep=True,
    )
    assert not sink.closed
    assert not sink.failed


def test_physical_write_failure_emergency_hard_lows_closes_and_never_retries():
    gpio = FakePwmGpio()
    sink = _sink(gpio)
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
        *_hard_low_calls(sleep=False),
        ("pwm", 7, 12, 8_000, 25.0),
        ("pwm", 7, 18, 8_000, 75.0),
        *_hard_low_calls(active_pins=(12,), sleep=True),
        ("close", 7),
    ]
    assert sink.closed
    assert sink.failed
    before = tuple(gpio.calls)
    with pytest.raises(OSError, match="closed"):
        sink.write(_frame(SafetyDecision.STOP))
    assert tuple(gpio.calls) == before


def test_high_readback_is_a_capability_failure_before_any_allow_pwm():
    gpio = FakePwmGpio()
    sink = _sink(gpio)
    gpio.high_read_call = gpio.read_calls + 1
    gpio.calls.clear()

    with pytest.raises(OSError, match="remained HIGH"):
        sink.write(
            _frame(
                SafetyDecision.ALLOW,
                left_output=0.25,
                right_output=0.25,
                enabled=True,
            )
        )

    assert not any(call[0] == "pwm" and call[3] != 0 for call in gpio.calls)
    assert gpio.calls[-1] == ("close", 7)
    assert sink.closed
    assert sink.failed


def test_initialization_failure_hard_lows_attempted_pins_before_close():
    gpio = FakePwmGpio()
    gpio.fail_claim_call = 3

    with pytest.raises(OSError, match="claim failure"):
        _sink(gpio)

    assert ("claim", 7, 19, 0) in gpio.calls
    assert gpio.calls[-18:] == [
        ("busy", 7, 12, 0),
        ("busy", 7, 13, 0),
        ("busy", 7, 18, 0),
        ("busy", 7, 19, 0),
        ("write", 7, 12, 0),
        ("write", 7, 13, 0),
        ("write", 7, 18, 0),
        ("write", 7, 19, 0),
        ("read", 7, 12),
        ("read", 7, 13),
        ("read", 7, 18),
        ("read", 7, 19),
        ("sleep", 0.002),
        ("read", 7, 12),
        ("read", 7, 13),
        ("read", 7, 18),
        ("read", 7, 19),
        ("close", 7),
    ]


def test_close_holds_verified_low_then_releases_handle_and_is_idempotent():
    gpio = FakePwmGpio()
    sink = _sink(gpio)
    sink.write(
        _frame(
            SafetyDecision.ALLOW,
            left_output=0.25,
            right_output=0.25,
            enabled=True,
        )
    )
    gpio.calls.clear()

    sink.close()
    sink.close()

    assert gpio.calls == _hard_low_calls(
        active_pins=(12, 18, 19),
        sleep=True,
    ) + [("close", 7)]
    assert sink.closed
    assert not sink.failed


def test_broken_busy_query_uses_free_reclaim_emergency_cancellation():
    gpio = FakePwmGpio()
    sink = _sink(gpio)
    sink.write(
        _frame(
            SafetyDecision.ALLOW,
            left_output=0.25,
            right_output=0.25,
            enabled=True,
        )
    )
    gpio.fail_busy_pin = 12
    gpio.calls.clear()

    with pytest.raises(OSError, match="busy failure"):
        sink.write(_frame(SafetyDecision.STOP))

    assert ("free", 7, 12) in gpio.calls
    free_index = gpio.calls.index(("free", 7, 12))
    assert gpio.calls[free_index + 1] == ("claim", 7, 12, 0)
    assert gpio.calls[-1] == ("close", 7)
    assert sink.closed
    assert sink.failed


def test_invalid_config_and_backend_fail_before_gpio_is_opened():
    with pytest.raises(ValueError, match="GPIO pins must be unique"):
        GpioMotorFrameSinkConfig(
            MotorChannelPhysicalConfig(1, 2),
            MotorChannelPhysicalConfig(2, 3),
        )

    with pytest.raises(TypeError, match="gpiochip_open"):
        GpioMotorFrameSink(object(), _config())  # type: ignore[arg-type]

    class IncompleteBackend:
        gpiochip_open = lambda self, _chip: 7
        gpio_claim_output = lambda self, _handle, _pin, _level: 0

    with pytest.raises(TypeError, match="gpio_write"):
        GpioMotorFrameSink(IncompleteBackend(), _config())  # type: ignore[arg-type]
