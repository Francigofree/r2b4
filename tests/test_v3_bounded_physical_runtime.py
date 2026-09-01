import pytest

from v3.adapters.bounded_command import BoundedTeleopProfile
from v3.adapters.gpio_motor import GpioMotorFrameSinkConfig
from v3.adapters.live_encoder import (
    EncoderVelocityReading,
    NativeEncoderConfig,
    NativeEncoderSource,
)
from v3.adapters.live_imu import ImuHeadingReading, NativeImuConfig, NativeImuSource
from v3.adapters.live_lidar import (
    LidarHealthReading,
    NativeLidarConfig,
    NativeLidarSource,
)
from v3.adapters.motor_pwm import MotorChannelPhysicalConfig
from v3.composition.bounded_live_control import BoundedLiveControlConfig
from v3.composition.bounded_physical_control import BoundedPhysicalControlConfig
from v3.composition.native_control import NativeControlCompositionConfig
from v3.contracts import TickContext
from v3.engine import TickExecutionError
from v3.layers.l11_actuator_control import (
    SpeedMapPoint,
    WheelSpeedCurve,
    WheelSpeedMap,
)
from v3_bounded_runtime import (
    BoundedPhysicalRuntimeConfig,
    RUN_FAULT,
    RUN_OK,
    run_bounded_physical_control,
)


class EncoderBackend:
    def __init__(self, fail_tick: int | None = None) -> None:
        self.fail_tick = fail_tick
        self.calls: list[TickContext] = []

    def read(self, context: TickContext) -> EncoderVelocityReading:
        self.calls.append(context)
        if context.tick_id == self.fail_tick:
            raise OSError("injected encoder failure")
        return EncoderVelocityReading(
            context.tick_id,
            context.monotonic_ns,
            left_mps=0.0,
            right_mps=0.0,
            trust=1.0,
            stale=False,
            timing_valid=True,
        )


class ImuBackend:
    def read(self, context: TickContext) -> ImuHeadingReading:
        return ImuHeadingReading(
            context.tick_id,
            context.monotonic_ns,
            yaw_rad=0.0,
            omega_rad_s=0.0,
            confidence=1.0,
            calibration=3,
            stale=False,
            timing_valid=True,
        )


class LidarBackend:
    def read(self, context: TickContext) -> LidarHealthReading:
        return LidarHealthReading(
            context.tick_id,
            context.monotonic_ns,
            measurement_age_ns=0,
            confidence=1.0,
            stale=False,
            timing_valid=True,
        )


class FakePwmGpio:
    def __init__(self, *, fail_nonzero: bool = False) -> None:
        self.fail_nonzero = fail_nonzero
        self.calls: list[tuple[object, ...]] = []

    def gpiochip_open(self, chip: int) -> int:
        self.calls.append(("open", chip))
        return 11

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
        self.calls.append(("pwm", handle, pin, frequency_hz, duty_cycle))
        if self.fail_nonzero and duty_cycle != 0.0:
            raise OSError("injected GPIO PWM failure")
        return 0

    def gpiochip_close(self, handle: int) -> int:
        self.calls.append(("close", handle))
        return 0


class StepClock:
    def __init__(self, start_ns: int = 1_000_000_000, step_ns: int = 100_000_000):
        self.next_ns = start_ns
        self.step_ns = step_ns

    def __call__(self) -> int:
        value = self.next_ns
        self.next_ns += self.step_ns
        return value


class SequenceClock:
    def __init__(self, values: tuple[int, ...]) -> None:
        self._values = iter(values)

    def __call__(self) -> int:
        return next(self._values)


def _speed_map() -> WheelSpeedMap:
    return WheelSpeedMap(
        schema="R2B4_WHEEL_SPEED_MAP_V2",
        map_state="ACTIVE",
        curves=tuple(
            WheelSpeedCurve(
                name,
                (SpeedMapPoint(0.02, 0.15), SpeedMapPoint(0.50, 0.85)),
                maintenance_output=0.12,
                startup_output=0.15,
            )
            for name in (
                "left_forward",
                "left_reverse",
                "right_forward",
                "right_reverse",
            )
        ),
    )


def _runtime_config() -> BoundedPhysicalRuntimeConfig:
    return BoundedPhysicalRuntimeConfig(
        composition=BoundedPhysicalControlConfig(
            live_control=BoundedLiveControlConfig(
                command_profile=BoundedTeleopProfile(
                    command_id="phase12-owner-loop",
                    start_tick_id=1,
                    active_tick_count=3,
                    v_mps=0.08,
                    omega_rad_s=0.0,
                    max_v_mps=0.10,
                    max_omega_rad_s=0.20,
                ),
                control=NativeControlCompositionConfig(speed_map=_speed_map()),
                max_preflight_age_ns=150_000_000,
            ),
            motor_output=GpioMotorFrameSinkConfig(
                left=MotorChannelPhysicalConfig(12, 13),
                right=MotorChannelPhysicalConfig(18, 19, invert=True),
            ),
        ),
        tick_period_ns=100_000_000,
    )


def _sources(encoder: EncoderBackend):
    return (
        NativeEncoderSource(encoder, NativeEncoderConfig("encoder", 0.5)),
        NativeImuSource(ImuBackend(), NativeImuConfig("imu", 0.5, 2)),
        NativeLidarSource(
            LidarBackend(),
            NativeLidarConfig("lidar", 0.5, 100_000_000),
        ),
    )


def _run(
    gpio: FakePwmGpio,
    encoder: EncoderBackend,
    *,
    stop_requested=lambda: False,
    monotonic_ns=None,
) -> int:
    clock = StepClock() if monotonic_ns is None else monotonic_ns
    return run_bounded_physical_control(
        *_sources(encoder),
        gpio,
        _runtime_config(),
        stop_requested=stop_requested,
        monotonic_ns=clock,
        sleep=lambda _seconds: None,
    )


def _pwm_calls(gpio: FakePwmGpio):
    return [call for call in gpio.calls if call[0] == "pwm"]


def test_owner_loop_runs_preflight_bounded_active_post_window_and_final_zero():
    gpio = FakePwmGpio()
    encoder = EncoderBackend()

    status = _run(gpio, encoder)

    assert status == RUN_OK
    assert [context.tick_id for context in encoder.calls] == [0, 1, 2, 3, 4]
    assert any(call[-1] != 0.0 for call in _pwm_calls(gpio))
    assert all(call[-1] == 0.0 for call in _pwm_calls(gpio)[-8:])
    assert gpio.calls[-1] == ("close", 11)
    assert gpio.calls.count(("close", 11)) == 1


def test_stop_after_preflight_closes_without_entering_active_output():
    gpio = FakePwmGpio()
    encoder = EncoderBackend()
    requests = iter((False, True))

    status = _run(gpio, encoder, stop_requested=lambda: next(requests))

    assert status == RUN_OK
    assert [context.tick_id for context in encoder.calls] == [0]
    assert all(call[-1] == 0.0 for call in _pwm_calls(gpio))
    assert gpio.calls[-1] == ("close", 11)


def test_initial_stop_request_does_not_claim_gpio_or_poll_sources():
    gpio = FakePwmGpio()
    encoder = EncoderBackend()

    status = _run(gpio, encoder, stop_requested=lambda: True)

    assert status == RUN_OK
    assert gpio.calls == []
    assert encoder.calls == []


def test_source_fault_returns_fault_after_one_zero_commit_and_no_retry():
    gpio = FakePwmGpio()
    encoder = EncoderBackend(fail_tick=1)

    status = _run(gpio, encoder)

    assert status == RUN_FAULT
    assert [context.tick_id for context in encoder.calls] == [0, 1]
    assert all(call[-1] == 0.0 for call in _pwm_calls(gpio))
    assert gpio.calls[-1] == ("close", 11)


def test_writer_failure_propagates_after_emergency_zero_without_retry():
    gpio = FakePwmGpio(fail_nonzero=True)
    encoder = EncoderBackend()

    with pytest.raises(TickExecutionError, match="L12"):
        _run(gpio, encoder)

    assert [context.tick_id for context in encoder.calls] == [0, 1]
    assert gpio.calls[-1] == ("close", 11)
    assert gpio.calls.count(("close", 11)) == 1


def test_clock_regression_raises_and_closes_before_an_active_tick():
    gpio = FakePwmGpio()
    encoder = EncoderBackend()
    clock = SequenceClock((1_000, 1_000, 999))

    with pytest.raises(RuntimeError, match="moved backwards"):
        _run(gpio, encoder, monotonic_ns=clock)

    assert [context.tick_id for context in encoder.calls] == [0]
    assert all(call[-1] == 0.0 for call in _pwm_calls(gpio))
    assert gpio.calls[-1] == ("close", 11)


def test_invalid_callback_is_rejected_before_gpio_claim():
    gpio = FakePwmGpio()
    encoder = EncoderBackend()

    with pytest.raises(TypeError, match="stop_requested"):
        run_bounded_physical_control(
            *_sources(encoder),
            gpio,
            _runtime_config(),
            stop_requested=None,  # type: ignore[arg-type]
        )

    assert gpio.calls == []
    assert encoder.calls == []


def test_invalid_stop_result_is_rejected_before_gpio_claim():
    gpio = FakePwmGpio()
    encoder = EncoderBackend()

    with pytest.raises(TypeError, match="must return bool"):
        run_bounded_physical_control(
            *_sources(encoder),
            gpio,
            _runtime_config(),
            stop_requested=lambda: 0,  # type: ignore[return-value]
        )

    assert gpio.calls == []
    assert encoder.calls == []


def test_schedule_cannot_be_slower_than_preflight_freshness_bound():
    config = _runtime_config()

    with pytest.raises(ValueError, match="preflight freshness"):
        BoundedPhysicalRuntimeConfig(
            config.composition,
            tick_period_ns=150_000_001,
        )
