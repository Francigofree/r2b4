from dataclasses import replace

import pytest

from v3.adapters.bounded_command import (
    BoundedTeleopCommandGateway,
    BoundedTeleopProfile,
)
from v3.composition.native_control import (
    NativeControlComposition,
    NativeControlCompositionConfig,
)
from v3.contracts import (
    DataField,
    DeviceHealth,
    DeviceHealthState,
    DeviceSample,
    FinalActuation,
    LifecycleState,
    RawDeviceBatch,
    SafetyDecision,
    TickContext,
)
from v3.engine import TickExecutionError, TickInputs
from v3.layers.l10_chassis_control import ChassisControlConfig
from v3.layers.l11_actuator_control import (
    SpeedMapPoint,
    WheelSpeedCurve,
    WheelSpeedMap,
)


class RecordingMotorWriter:
    def __init__(self, fail_ticks: frozenset[int] = frozenset()) -> None:
        self.commands: list[FinalActuation] = []
        self.fail_ticks = fail_ticks

    def write(self, command: FinalActuation) -> None:
        self.commands.append(command)
        if command.context.tick_id in self.fail_ticks:
            raise OSError("injected motor writer failure")


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


def _config() -> NativeControlCompositionConfig:
    return NativeControlCompositionConfig(speed_map=_speed_map())


def _context(tick_id: int) -> TickContext:
    return TickContext(tick_id, 1_000_000_000 + tick_id * 100_000_000)


def _raw_devices(context: TickContext) -> RawDeviceBatch:
    samples = (
        DeviceSample(
            device_id="native-wheel-odometry",
            kind="wheel_velocity",
            sequence=context.tick_id,
            captured_monotonic_ns=context.monotonic_ns,
            values=(
                DataField("left_mps", 0.0),
                DataField("right_mps", 0.0),
                DataField("trust", 1.0),
            ),
        ),
        DeviceSample(
            device_id="native-imu",
            kind="ekf_heading",
            sequence=context.tick_id,
            captured_monotonic_ns=context.monotonic_ns,
            values=(
                DataField("yaw_rad", 0.0),
                DataField("omega_rad_s", 0.0),
                DataField("confidence", 1.0),
            ),
        ),
        DeviceSample(
            device_id="native-lidar",
            kind="lidar_health",
            sequence=context.tick_id,
            captured_monotonic_ns=context.monotonic_ns,
            values=(DataField("age_ns", 0), DataField("confidence", 1.0)),
        ),
    )
    health = tuple(
        DeviceHealth(device_id, DeviceHealthState.OK)
        for device_id in (
            "native-wheel-odometry",
            "native-imu",
            "native-lidar",
            "native-motor-driver",
        )
    )
    return RawDeviceBatch(context, samples, health)


def _inputs(tick_id: int, gateway: BoundedTeleopCommandGateway) -> TickInputs:
    context = _context(tick_id)
    lifecycle = (
        LifecycleState.ACTIVE
        if 1 <= tick_id <= 4
        else LifecycleState.IDLE
    )
    return TickInputs(
        context=context,
        raw_devices=_raw_devices(context),
        command=gateway.snapshot(context),
        lifecycle=lifecycle,
    )


def _gateway() -> BoundedTeleopCommandGateway:
    return BoundedTeleopCommandGateway(
        BoundedTeleopProfile(
            command_id="phase12-control-core",
            start_tick_id=1,
            active_tick_count=3,
            v_mps=0.08,
            omega_rad_s=0.0,
            max_v_mps=0.10,
            max_omega_rad_s=0.20,
        )
    )


def test_bounded_active_window_runs_native_l1_l12_then_returns_to_zero():
    writer = RecordingMotorWriter()
    composition = NativeControlComposition(writer, _config())
    gateway = _gateway()

    results = tuple(composition.run_tick(_inputs(tick_id, gateway)) for tick_id in range(6))

    assert len(writer.commands) == 6
    assert tuple(writer.commands) == tuple(result.final_actuation for result in results)
    assert all(len(result.trace.layers) == 12 for result in results)
    assert results[0].final_actuation.safety_decision is SafetyDecision.STOP
    active = tuple(result.final_actuation for result in results[1:4])
    assert all(command.safety_decision is SafetyDecision.ALLOW for command in active)
    assert any(
        command.left_output != 0.0 or command.right_output != 0.0
        for command in active
    )
    assert results[4].final_actuation.left_output == 0.0
    assert results[4].final_actuation.right_output == 0.0
    assert results[5].final_actuation.safety_decision is SafetyDecision.STOP
    assert results[5].final_actuation.left_output == 0.0
    assert results[5].final_actuation.right_output == 0.0


def test_writer_failure_is_one_l12_attempt_without_retry():
    writer = RecordingMotorWriter(fail_ticks=frozenset({2}))
    composition = NativeControlComposition(writer, _config())
    gateway = _gateway()
    composition.run_tick(_inputs(0, gateway))
    composition.run_tick(_inputs(1, gateway))

    with pytest.raises(TickExecutionError, match="L12"):
        composition.run_tick(_inputs(2, gateway))

    assert tuple(command.context.tick_id for command in writer.commands) == (0, 1, 2)


def test_explicit_input_fault_closes_one_zero_fault_commit():
    writer = RecordingMotorWriter()
    composition = NativeControlComposition(writer, _config())
    context = _context(0)

    result = composition.run_fault_tick(
        context,
        LifecycleState.ACTIVE,
        "L0_ERROR",
        "L0",
    )

    assert writer.commands == [result.final_actuation]
    assert result.final_actuation.safety_decision is SafetyDecision.FAULT
    assert result.final_actuation.left_output == 0.0
    assert result.final_actuation.right_output == 0.0
    assert result.trace.fault_layer == "L0"


def test_config_rejects_geometry_drift_and_writer_is_never_exposed():
    with pytest.raises(ValueError, match="L3 and L10"):
        replace(_config(), chassis_control=ChassisControlConfig(track_width_m=0.42))

    with pytest.raises(TypeError, match="callable write"):
        NativeControlComposition(object(), _config())

    composition = NativeControlComposition(RecordingMotorWriter(), _config())
    assert not hasattr(composition, "activate")
    assert not hasattr(composition, "reader")
    assert not hasattr(composition, "writer")
