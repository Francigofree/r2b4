import json
from dataclasses import replace
from pathlib import Path

import pytest

from v3.adapters.live_idle import (
    GpioZeroMotorWriter,
    GpioZeroWriterConfig,
    LiveIdleWriteRejected,
    MotorChannelPhysicalConfig,
    PwmDecayMode,
)
from v3.composition.live_idle import LiveIdleComposition, LiveIdleConfig
from v3.contracts import FinalActuation, LifecycleState, SafetyDecision, TickContext
from v3.engine import TickExecutionError
from v3_idle_runtime import load_live_idle_runtime_config, run_live_idle


class FakeGpio:
    def __init__(self) -> None:
        self.calls: list[tuple[object, ...]] = []
        self.fail_pwm_call: int | None = None
        self.fail_claim_call: int | None = None
        self._claim_calls = 0
        self._pwm_calls = 0

    def gpiochip_open(self, chip: int) -> int:
        self.calls.append(("open", chip))
        return 7

    def gpio_claim_output(self, handle: int, pin: int) -> int:
        self._claim_calls += 1
        self.calls.append(("claim", handle, pin))
        if self.fail_claim_call == self._claim_calls:
            raise OSError("injected GPIO claim failure")
        return 0

    def tx_pwm(self, handle: int, pin: int, frequency_hz: int, duty: float) -> int:
        self._pwm_calls += 1
        self.calls.append(("pwm", handle, pin, frequency_hz, duty))
        if self.fail_pwm_call == self._pwm_calls:
            raise OSError("injected GPIO PWM failure")
        return 0

    def gpiochip_close(self, handle: int) -> int:
        self.calls.append(("close", handle))
        return 0


def _config() -> LiveIdleConfig:
    return LiveIdleConfig(
        GpioZeroWriterConfig(
            left=MotorChannelPhysicalConfig(
                12,
                13,
                pwm_decay_mode=PwmDecayMode.BRAKE,
            ),
            right=MotorChannelPhysicalConfig(
                18,
                19,
                invert=True,
                pwm_decay_mode=PwmDecayMode.BRAKE,
            ),
        )
    )


def _pwm_calls(gpio: FakeGpio) -> list[tuple[object, ...]]:
    return [call for call in gpio.calls if call[0] == "pwm"]


def _assert_zero(result) -> None:
    assert result.final_actuation.enabled is False
    assert result.final_actuation.safety_decision in (
        SafetyDecision.STOP,
        SafetyDecision.FAULT,
    )
    assert result.final_actuation.left_output == 0.0
    assert result.final_actuation.right_output == 0.0


def test_gpio_writer_claims_one_chip_and_initializes_every_pin_to_zero():
    gpio = FakeGpio()

    writer = GpioZeroMotorWriter(gpio, _config().motors)

    assert gpio.calls[:9] == [
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
    assert _pwm_calls(gpio) == [
        ("pwm", 7, 12, 8_000, 0.0),
        ("pwm", 7, 13, 8_000, 0.0),
        ("pwm", 7, 18, 8_000, 0.0),
        ("pwm", 7, 19, 8_000, 0.0),
    ]
    writer.close()
    assert gpio.calls[-1] == ("close", 7)


def test_gpio_writer_zeros_each_claimed_pin_before_claiming_the_next():
    gpio = FakeGpio()
    gpio.fail_claim_call = 3

    with pytest.raises(OSError, match="claim failure"):
        GpioZeroMotorWriter(gpio, _config().motors)

    assert gpio.calls == [
        ("open", 0),
        ("claim", 7, 12),
        ("pwm", 7, 12, 8_000, 0.0),
        ("claim", 7, 13),
        ("pwm", 7, 13, 8_000, 0.0),
        ("claim", 7, 18),
        ("close", 7),
    ]


def test_gpio_writer_rejects_allow_or_nonzero_without_touching_hardware():
    gpio = FakeGpio()
    writer = GpioZeroMotorWriter(gpio, _config().motors)
    before = tuple(gpio.calls)
    context = TickContext(0, 1)
    allowed = FinalActuation(
        context=context,
        left_output=0.2,
        right_output=0.2,
        enabled=True,
        safety_decision=SafetyDecision.ALLOW,
        latch_state="CLEAR",
    )

    with pytest.raises(LiveIdleWriteRejected):
        writer.write(allowed)

    assert tuple(gpio.calls) == before
    writer.close()


def test_live_composition_reaches_only_idle_and_shutdown_with_zero_commits():
    gpio = FakeGpio()
    runtime = LiveIdleComposition(gpio, _config())

    assert runtime.lifecycle is LifecycleState.BOOTING
    assert not hasattr(runtime, "activate")
    runtime.enter_idle()
    idle = runtime.tick(1_000)
    shutdown = runtime.close(2_000)

    assert runtime.lifecycle is LifecycleState.SHUTDOWN
    assert runtime.closed
    _assert_zero(idle)
    assert shutdown is not None
    _assert_zero(shutdown)
    assert len(_pwm_calls(gpio)) == 16
    assert gpio.calls[-1] == ("close", 7)


def test_live_writer_failure_is_one_l12_attempt_and_faults_without_retry():
    gpio = FakeGpio()
    runtime = LiveIdleComposition(gpio, _config())
    runtime.enter_idle()
    gpio.fail_pwm_call = 5

    with pytest.raises(TickExecutionError, match="L12"):
        runtime.tick(1_000)

    assert runtime.lifecycle is LifecycleState.FAULT
    assert len(_pwm_calls(gpio)) == 5
    runtime.close(2_000)
    assert runtime.closed
    assert len(_pwm_calls(gpio)) == 5
    assert gpio.calls[-1] == ("close", 7)


def test_hardware_config_is_loaded_once_into_immutable_v3_values(tmp_path: Path):
    path = tmp_path / "hardver.json"
    path.write_text(
        json.dumps(
            {
                "motorok": {
                    "pwm_decay_mode": "brake",
                    "bal_oldal": {
                        "gpio_in1": 12,
                        "gpio_in2": 13,
                        "invert": False,
                    },
                    "jobb_oldal": {
                        "gpio_in1": 18,
                        "gpio_in2": 19,
                        "invert": True,
                    },
                }
            }
        ),
        encoding="utf-8",
    )

    config = load_live_idle_runtime_config(path)

    assert config.composition == _config()
    with pytest.raises(ValueError, match="unique"):
        replace(
            config.composition.motors,
            right=MotorChannelPhysicalConfig(13, 19),
        )


def test_hardware_config_rejects_unknown_physical_motor_semantics(tmp_path: Path):
    path = tmp_path / "hardver.json"
    path.write_text(
        json.dumps(
            {
                "motorok": {
                    "pwm_decay_mode": "unknown",
                    "bal_oldal": {"gpio_in1": 12, "gpio_in2": 13},
                    "jobb_oldal": {"gpio_in1": 18, "gpio_in2": 19},
                },
            }
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="pwm_decay_mode is invalid"):
        load_live_idle_runtime_config(path)


def test_headless_owner_loop_has_no_active_transition_and_finishes_zero():
    gpio = FakeGpio()
    times = iter((100, 100, 120, 121))

    result = run_live_idle(
        gpio,
        load_live_idle_runtime_config(
            Path(__file__).resolve().parents[1] / "conf" / "hardver.json"
        ),
        stop_requested=lambda: False,
        monotonic_ns=lambda: next(times),
        sleep=lambda _seconds: None,
        max_ticks=1,
    )

    assert result == 0
    assert all(call[-1] == 0.0 for call in _pwm_calls(gpio))
    assert gpio.calls[-1] == ("close", 7)


def test_owner_loop_releases_gpio_if_the_injected_clock_fails_before_idle():
    gpio = FakeGpio()

    def failed_clock() -> int:
        raise OSError("injected monotonic clock failure")

    with pytest.raises(OSError, match="clock failure"):
        run_live_idle(
            gpio,
            load_live_idle_runtime_config(
                Path(__file__).resolve().parents[1] / "conf" / "hardver.json"
            ),
            stop_requested=lambda: False,
            monotonic_ns=failed_clock,
            sleep=lambda _seconds: None,
            max_ticks=1,
        )

    assert all(call[-1] == 0.0 for call in _pwm_calls(gpio))
    assert gpio.calls[-1] == ("close", 7)
