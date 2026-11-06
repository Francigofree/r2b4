import pytest
from v3.adapters.gpio_motor import GpioMotorFrameSink, GpioMotorFrameSinkConfig
from v3.adapters.motor_pwm import MotorChannelPhysicalConfig, PwmDecayMode, plan_final_actuation
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
        self.calls.append(('open', chip))
        return 7

    def gpio_claim_output(self, handle: int, pin: int, initial_level: int) -> int:
        self._claim_calls += 1
        self.calls.append(('claim', handle, pin, initial_level))
        self._levels[pin] = initial_level
        if self.fail_claim_call == self._claim_calls:
            raise OSError('injected GPIO claim failure')
        return 0

    def tx_pwm(self, handle: int, pin: int, frequency_hz: int, duty_cycle: float) -> int:
        self._pwm_calls += 1
        self.calls.append(('pwm', handle, pin, frequency_hz, duty_cycle))
        if self.fail_pwm_call == self._pwm_calls:
            raise OSError('injected GPIO PWM failure')
        if frequency_hz == 0:
            self._pwm_busy.discard(pin)
        elif duty_cycle != 0.0:
            self._pwm_busy.add(pin)
        return 0

    def tx_busy(self, handle: int, pin: int, kind: int) -> int:
        self.calls.append(('busy', handle, pin, kind))
        if self.fail_busy_pin == pin:
            raise OSError('injected GPIO PWM busy failure')
        return int(pin in self._pwm_busy)

    def gpio_write(self, handle: int, pin: int, level: int) -> int:
        self._write_calls += 1
        self.calls.append(('write', handle, pin, level))
        if self.fail_write_call == self._write_calls:
            raise OSError('injected GPIO write failure')
        self._levels[pin] = level
        return 0

    def gpio_read(self, handle: int, pin: int) -> int:
        self._read_calls += 1
        self.calls.append(('read', handle, pin))
        if self.high_read_call == self._read_calls:
            return 1
        return self._levels[pin]

    def gpio_free(self, handle: int, pin: int) -> int:
        self.calls.append(('free', handle, pin))
        self._pwm_busy.discard(pin)
        return 0

    def gpiochip_close(self, handle: int) -> int:
        self.calls.append(('close', handle))
        return 0

class FakeSleep:

    def __init__(self, calls: list[tuple[object, ...]]) -> None:
        self._calls = calls

    def __call__(self, duration_s: float) -> None:
        self._calls.append(('sleep', duration_s))

def _sink(gpio: FakePwmGpio) -> GpioMotorFrameSink:
    return GpioMotorFrameSink(gpio, _config(), sleep=FakeSleep(gpio.calls))

def _config() -> GpioMotorFrameSinkConfig:
    return GpioMotorFrameSinkConfig(left=MotorChannelPhysicalConfig(12, 13), right=MotorChannelPhysicalConfig(18, 19, invert=True, pwm_decay_mode=PwmDecayMode.BRAKE))

def _frame(decision: SafetyDecision, *, left_output: float=0.0, right_output: float=0.0, enabled: bool=False):
    config = _config()
    command = FinalActuation(context=TickContext(3, 5000), left_output=left_output, right_output=right_output, enabled=enabled, safety_decision=decision, latch_state='CLEAR', reason=None if decision is SafetyDecision.ALLOW else 'STOPPED')
    return plan_final_actuation(config.left, config.right, command)

def _hard_low_calls(*, active_pins: tuple[int, ...]=(), sleep: bool) -> list[tuple[object, ...]]:
    calls: list[tuple[object, ...]] = []
    for pin in PINS:
        calls.append(('busy', 7, pin, 0))
        if pin in active_pins:
            calls.append(('pwm', 7, pin, 0, 0.0))
    calls.extend((('write', 7, pin, 0) for pin in PINS))
    calls.extend((('read', 7, pin) for pin in PINS))
    if sleep:
        calls.append(('sleep', 0.002))
        calls.extend((('read', 7, pin) for pin in PINS))
    return calls

@pytest.mark.parametrize('decision', [SafetyDecision.STOP, SafetyDecision.FAULT])
def test_stop_and_fault_cancel_pwm_hold_verified_low_and_keep_ownership(decision: SafetyDecision):
    gpio = FakePwmGpio()
    sink = _sink(gpio)
    sink.write(_frame(SafetyDecision.ALLOW, left_output=0.25, right_output=0.25, enabled=True))
    gpio.calls.clear()
    sink.write(_frame(decision))
    assert gpio.calls == _hard_low_calls(active_pins=(12, 18, 19), sleep=True)
    assert not sink.closed
    assert not sink.failed
