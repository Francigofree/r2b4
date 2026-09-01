from dataclasses import replace

import pytest

from v3.adapters.bounded_command import BoundedTeleopProfile
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
from v3.composition.bounded_live_control import (
    BoundedLiveControlComposition,
    BoundedLiveControlConfig,
)
from v3.composition.native_control import NativeControlCompositionConfig
from v3.contracts import FinalActuation, LifecycleState, SafetyDecision, TickContext
from v3.engine import TickExecutionError
from v3.layers.l11_actuator_control import (
    SpeedMapPoint,
    WheelSpeedCurve,
    WheelSpeedMap,
)


class EncoderBackend:
    def __init__(self, *, fail_tick: int | None = None, stale_tick: int | None = None):
        self.calls: list[TickContext] = []
        self.fail_tick = fail_tick
        self.stale_tick = stale_tick

    def read(self, context: TickContext) -> EncoderVelocityReading:
        self.calls.append(context)
        if context.tick_id == self.fail_tick:
            raise OSError("injected encoder failure")
        return EncoderVelocityReading(
            sequence=context.tick_id,
            captured_monotonic_ns=context.monotonic_ns,
            left_mps=0.0,
            right_mps=0.0,
            trust=1.0,
            stale=context.tick_id == self.stale_tick,
            timing_valid=True,
        )


class ImuBackend:
    def __init__(self) -> None:
        self.calls: list[TickContext] = []

    def read(self, context: TickContext) -> ImuHeadingReading:
        self.calls.append(context)
        return ImuHeadingReading(
            sequence=context.tick_id,
            captured_monotonic_ns=context.monotonic_ns,
            yaw_rad=0.0,
            omega_rad_s=0.0,
            confidence=1.0,
            calibration=3,
            stale=False,
            timing_valid=True,
        )


class LidarBackend:
    def __init__(self) -> None:
        self.calls: list[TickContext] = []

    def read(self, context: TickContext) -> LidarHealthReading:
        self.calls.append(context)
        return LidarHealthReading(
            revision=context.tick_id,
            captured_monotonic_ns=context.monotonic_ns,
            measurement_age_ns=0,
            confidence=1.0,
            stale=False,
            timing_valid=True,
        )


class RecordingMotorWriter:
    def __init__(self, fail_tick: int | None = None) -> None:
        self.commands: list[FinalActuation] = []
        self.fail_tick = fail_tick

    def write(self, command: FinalActuation) -> None:
        self.commands.append(command)
        if command.context.tick_id == self.fail_tick:
            raise OSError("injected writer failure")


def _speed_map() -> WheelSpeedMap:
    return WheelSpeedMap(
        schema="R2B4_WHEEL_SPEED_MAP_V2",
        map_state="ACTIVE",
        curves=tuple(
            WheelSpeedCurve(
                name=name,
                points=(SpeedMapPoint(0.02, 0.15), SpeedMapPoint(0.50, 0.85)),
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


def _config() -> BoundedLiveControlConfig:
    return BoundedLiveControlConfig(
        command_profile=BoundedTeleopProfile(
            command_id="phase12-bounded-live",
            start_tick_id=1,
            active_tick_count=3,
            v_mps=0.08,
            omega_rad_s=0.0,
            max_v_mps=0.10,
            max_omega_rad_s=0.20,
        ),
        control=NativeControlCompositionConfig(speed_map=_speed_map()),
        max_preflight_age_ns=150_000_000,
    )


def _sources(
    encoder_backend: EncoderBackend,
    imu_backend: ImuBackend,
    lidar_backend: LidarBackend,
):
    return (
        NativeEncoderSource(
            encoder_backend,
            NativeEncoderConfig("encoder", minimum_trust=0.5),
        ),
        NativeImuSource(
            imu_backend,
            NativeImuConfig("imu", minimum_confidence=0.5, minimum_calibration=2),
        ),
        NativeLidarSource(
            lidar_backend,
            NativeLidarConfig(
                "lidar",
                minimum_confidence=0.5,
                maximum_measurement_age_ns=100_000_000,
            ),
        ),
    )


def _composition(
    writer: RecordingMotorWriter,
    *,
    encoder: EncoderBackend | None = None,
):
    encoder_backend = EncoderBackend() if encoder is None else encoder
    imu_backend = ImuBackend()
    lidar_backend = LidarBackend()
    sources = _sources(encoder_backend, imu_backend, lidar_backend)
    composition = BoundedLiveControlComposition(
        *sources,
        writer,
        _config(),
    )
    return composition, encoder_backend, imu_backend, lidar_backend


def _context(tick_id: int, *, elapsed_ns: int | None = None) -> TickContext:
    monotonic_ns = (
        1_000_000_000 + tick_id * 100_000_000
        if elapsed_ns is None
        else 1_000_000_000 + elapsed_ns
    )
    return TickContext(tick_id, monotonic_ns)


def test_healthy_preflight_opens_only_the_finite_window_then_returns_idle_zero():
    writer = RecordingMotorWriter()
    composition, encoder, imu, lidar = _composition(writer)

    results = tuple(composition.tick(_context(tick_id)) for tick_id in range(5))

    assert composition.lifecycle is LifecycleState.IDLE
    assert composition.preflight_complete
    assert len(writer.commands) == 5
    assert tuple(writer.commands) == tuple(item.final_actuation for item in results)
    assert results[0].final_actuation.safety_decision is SafetyDecision.STOP
    active = tuple(item.final_actuation for item in results[1:4])
    assert all(item.safety_decision is SafetyDecision.ALLOW for item in active)
    assert any(item.left_output != 0.0 or item.right_output != 0.0 for item in active)
    assert results[4].final_actuation.safety_decision is SafetyDecision.STOP
    assert results[4].final_actuation.left_output == 0.0
    assert results[4].final_actuation.right_output == 0.0
    expected_contexts = [_context(tick_id) for tick_id in range(5)]
    assert encoder.calls == imu.calls == lidar.calls == expected_contexts


@pytest.mark.parametrize("elapsed_ns", [200_000_000, 0])
def test_missing_or_stale_direct_preflight_faults_before_any_active_command(
    elapsed_ns: int,
):
    writer = RecordingMotorWriter()
    composition, _, _, _ = _composition(writer)
    if elapsed_ns:
        preflight = composition.tick(_context(0))
        assert preflight.final_actuation.safety_decision is SafetyDecision.STOP

    result = composition.tick(_context(1, elapsed_ns=elapsed_ns))

    assert composition.lifecycle is LifecycleState.FAULT
    assert result.final_actuation.safety_decision is SafetyDecision.FAULT
    assert result.final_actuation.reason in {"PREFLIGHT_REQUIRED", "FAULT_LATCHED"}
    assert result.final_actuation.left_output == 0.0
    assert result.final_actuation.right_output == 0.0


def test_degraded_idle_tick_does_not_qualify_as_preflight():
    writer = RecordingMotorWriter()
    encoder = EncoderBackend(stale_tick=0)
    composition, _, _, _ = _composition(writer, encoder=encoder)

    idle = composition.tick(_context(0))
    active = composition.tick(_context(1))

    assert idle.final_actuation.safety_decision is SafetyDecision.STOP
    assert not composition.preflight_complete
    assert active.final_actuation.safety_decision is SafetyDecision.FAULT
    assert composition.lifecycle is LifecycleState.FAULT


def test_source_failure_closes_one_fault_commit_and_latches_session():
    writer = RecordingMotorWriter()
    encoder = EncoderBackend(fail_tick=1)
    composition, encoder, imu, lidar = _composition(writer, encoder=encoder)
    composition.tick(_context(0))

    result = composition.tick(_context(1))

    assert result.final_actuation.safety_decision is SafetyDecision.FAULT
    assert result.trace.fault_layer == "L0"
    assert composition.lifecycle is LifecycleState.FAULT
    assert len(writer.commands) == 2
    assert [item.tick_id for item in encoder.calls] == [0, 1]
    assert [item.tick_id for item in imu.calls] == [0]
    assert [item.tick_id for item in lidar.calls] == [0]


def test_active_sensor_degradation_faults_and_cannot_resume_source_polling():
    writer = RecordingMotorWriter()
    encoder = EncoderBackend(stale_tick=2)
    composition, encoder, imu, lidar = _composition(writer, encoder=encoder)
    composition.tick(_context(0))
    composition.tick(_context(1))

    degraded = composition.tick(_context(2))
    faulted = composition.tick(_context(3))

    assert degraded.final_actuation.safety_decision is SafetyDecision.FAULT
    assert degraded.final_actuation.reason == "L11_ERROR"
    assert degraded.final_actuation.left_output == 0.0
    assert degraded.final_actuation.right_output == 0.0
    assert faulted.final_actuation.safety_decision is SafetyDecision.FAULT
    assert composition.lifecycle is LifecycleState.FAULT
    assert [item.tick_id for item in encoder.calls] == [0, 1, 2]
    assert [item.tick_id for item in imu.calls] == [0, 1, 2]
    assert [item.tick_id for item in lidar.calls] == [0, 1, 2]
    assert tuple(item.context.tick_id for item in writer.commands) == (0, 1, 2, 3)


def test_writer_failure_has_no_retry_and_blocks_every_later_tick():
    writer = RecordingMotorWriter(fail_tick=1)
    composition, encoder, imu, lidar = _composition(writer)
    composition.tick(_context(0))

    with pytest.raises(TickExecutionError, match="L12"):
        composition.tick(_context(1))

    calls_before = (
        tuple(writer.commands),
        tuple(encoder.calls),
        tuple(imu.calls),
        tuple(lidar.calls),
    )
    with pytest.raises(RuntimeError, match="retry is forbidden"):
        composition.tick(_context(2))
    assert calls_before == (
        tuple(writer.commands),
        tuple(encoder.calls),
        tuple(imu.calls),
        tuple(lidar.calls),
    )
    assert tuple(item.context.tick_id for item in writer.commands) == (0, 1)
    assert composition.lifecycle is LifecycleState.FAULT


def test_config_requires_space_for_preflight_and_no_runtime_api_is_exposed():
    with pytest.raises(ValueError, match="earlier preflight"):
        replace(
            _config(),
            command_profile=replace(_config().command_profile, start_tick_id=0),
        )

    composition, _, _, _ = _composition(RecordingMotorWriter())
    assert not hasattr(composition, "activate")
    assert not hasattr(composition, "clock")
    assert not hasattr(composition, "close")
    assert not hasattr(composition, "writer")
