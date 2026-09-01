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
from v3.adapters.motor_pwm import MotorChannelPhysicalConfig, PwmDecayMode
from v3.composition.bounded_live_control import BoundedLiveControlConfig
from v3.composition.bounded_physical_control import (
    BoundedPhysicalControlComposition,
    BoundedPhysicalControlConfig,
)
from v3.composition.native_control import NativeControlCompositionConfig
from v3.contracts import LifecycleState, SafetyDecision, TickContext
from v3.engine import TickExecutionError
from v3.layers.l11_actuator_control import (
    SpeedMapPoint,
    WheelSpeedCurve,
    WheelSpeedMap,
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
    def __init__(self) -> None:
        self.calls: list[tuple[object, ...]] = []
        self.fail_pwm_call: int | None = None
        self.pwm_calls = 0

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
        self.pwm_calls += 1
        self.calls.append(("pwm", handle, pin, frequency_hz, duty_cycle))
        if self.pwm_calls == self.fail_pwm_call:
            raise OSError("injected GPIO PWM failure")
        return 0

    def gpiochip_close(self, handle: int) -> int:
        self.calls.append(("close", handle))
        return 0


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


def _config() -> BoundedPhysicalControlConfig:
    return BoundedPhysicalControlConfig(
        live_control=BoundedLiveControlConfig(
            command_profile=BoundedTeleopProfile(
                command_id="phase12-bounded-physical",
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
            right=MotorChannelPhysicalConfig(
                18,
                19,
                invert=True,
                pwm_decay_mode=PwmDecayMode.BRAKE,
            ),
        ),
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


def _context(tick_id: int) -> TickContext:
    return TickContext(tick_id, 1_000_000_000 + tick_id * 100_000_000)


def _root(gpio: FakePwmGpio, encoder: EncoderBackend | None = None):
    encoder_backend = EncoderBackend() if encoder is None else encoder
    return (
        BoundedPhysicalControlComposition(
            *_sources(encoder_backend),
            gpio,
            _config(),
        ),
        encoder_backend,
    )


def _pwm_calls(gpio: FakePwmGpio):
    return [call for call in gpio.calls if call[0] == "pwm"]


def test_complete_fake_physical_path_is_bounded_and_finishes_zero_closed():
    gpio = FakePwmGpio()
    root, encoder = _root(gpio)

    results = tuple(root.tick(_context(tick_id)) for tick_id in range(4))
    end_of_active = len(gpio.calls)
    final_idle = root.tick(_context(4))
    root.close()

    assert results[0].final_actuation.safety_decision is SafetyDecision.STOP
    assert all(
        result.final_actuation.safety_decision is SafetyDecision.ALLOW
        for result in results[1:]
    )
    assert any(call[-1] != 0.0 for call in _pwm_calls(gpio))
    assert final_idle.final_actuation.safety_decision is SafetyDecision.STOP
    assert all(
        call[0] != "pwm" or call[-1] == 0.0
        for call in gpio.calls[end_of_active:]
    )
    assert gpio.calls[-1] == ("close", 7)
    assert root.closed
    assert root.lifecycle is LifecycleState.SHUTDOWN
    assert [context.tick_id for context in encoder.calls] == [0, 1, 2, 3, 4]


def test_missing_preflight_can_only_reach_fault_zero_on_gpio():
    gpio = FakePwmGpio()
    root, _ = _root(gpio)

    result = root.tick(_context(1))
    root.close()

    assert result.final_actuation.safety_decision is SafetyDecision.FAULT
    assert all(call[-1] == 0.0 for call in _pwm_calls(gpio))
    assert gpio.calls[-1] == ("close", 7)


def test_source_failure_after_preflight_commits_fault_zero_then_closes_zero():
    gpio = FakePwmGpio()
    root, _ = _root(gpio, EncoderBackend(fail_tick=1))
    root.tick(_context(0))

    result = root.tick(_context(1))
    root.close()

    assert result.final_actuation.safety_decision is SafetyDecision.FAULT
    assert result.trace.fault_layer == "L0"
    assert all(call[-1] == 0.0 for call in _pwm_calls(gpio))
    assert gpio.calls[-1] == ("close", 7)


def test_physical_write_failure_emergency_closes_and_never_retries():
    gpio = FakePwmGpio()
    root, encoder = _root(gpio)
    root.tick(_context(0))
    gpio.fail_pwm_call = gpio.pwm_calls + 1

    with pytest.raises(TickExecutionError, match="L12"):
        root.tick(_context(1))

    assert root.closed
    assert root.lifecycle is LifecycleState.FAULT
    assert gpio.calls[-1] == ("close", 7)
    before = (tuple(gpio.calls), tuple(encoder.calls))
    with pytest.raises(RuntimeError, match="retry is forbidden"):
        root.tick(_context(2))
    assert before == (tuple(gpio.calls), tuple(encoder.calls))
    root.close()
    assert gpio.calls[-1] == ("close", 7)
    assert root.lifecycle is LifecycleState.SHUTDOWN


def test_invalid_source_is_rejected_before_gpio_handle_is_opened():
    gpio = FakePwmGpio()
    _, imu, lidar = _sources(EncoderBackend())

    with pytest.raises(TypeError, match="encoder_source"):
        BoundedPhysicalControlComposition(object(), imu, lidar, gpio, _config())  # type: ignore[arg-type]

    assert gpio.calls == []
