from pathlib import Path

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
    LidarScanReading,
    NativeLidarConfig,
    NativeLidarSource,
)
from v3.contracts import (
    CommandMode,
    CommandRequest,
    DataField,
    LifecycleState,
    SafetyDecision,
    TickContext,
)
from v3_bounded_config import (
    NativeSensorPolicyConfig,
    load_bounded_physical_runtime_config,
)
from v3_bounded_runtime import RUN_FAULT, RUN_OK
from v3_runtime import ResidentPhysicalRuntimeConfig, run_resident_physical_control


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class EncoderBackend:
    def __init__(self) -> None:
        self.calls = []

    def read(self, context):
        self.calls.append(context)
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
    def read(self, context):
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
    def __init__(self, revisions=None):
        self.revisions = dict(revisions or {})

    def read(self, context):
        revision = self.revisions.get(context.tick_id, context.tick_id + 1)
        return LidarHealthReading(
            revision,
            context.monotonic_ns,
            measurement_age_ns=0,
            confidence=1.0,
            stale=False,
            timing_valid=True,
            scan=LidarScanReading(
                revision=revision,
                captured_monotonic_ns=context.monotonic_ns,
                measurement_age_ns=0,
                health="OK",
                stale=False,
                timing_valid=True,
                point_count=80,
                front_clearance_m=1.0,
                rear_clearance_m=1.0,
                left_clearance_m=1.0,
                right_clearance_m=1.0,
                front_observation_count=20,
                rear_observation_count=20,
                left_observation_count=20,
                right_observation_count=20,
            ),
        )


class MotorGpio:
    def __init__(self):
        self.calls = []
        self.levels = {}
        self.busy = set()

    def gpiochip_open(self, chip):
        self.calls.append(("open", chip))
        return 4

    def gpio_claim_output(self, handle, pin, initial_level):
        self.calls.append(("claim", pin, initial_level))
        self.levels[pin] = initial_level
        return 0

    def gpio_write(self, handle, pin, level):
        self.calls.append(("write", pin, level))
        self.levels[pin] = level
        return 0

    def gpio_read(self, handle, pin):
        self.calls.append(("read", pin))
        return self.levels[pin]

    def gpio_free(self, handle, pin):
        self.calls.append(("free", pin))
        self.busy.discard(pin)
        return 0

    def tx_busy(self, handle, pin, kind):
        self.calls.append(("busy", pin, kind))
        return int(pin in self.busy)

    def tx_pwm(self, handle, pin, frequency_hz, duty_cycle):
        self.calls.append(("pwm", pin, frequency_hz, duty_cycle))
        if frequency_hz == 0 and duty_cycle == 0.0:
            self.busy.discard(pin)
        elif duty_cycle != 0.0:
            self.busy.add(pin)
        return 0

    def gpiochip_close(self, handle):
        self.calls.append(("close", handle))
        return 0


class StepClock:
    def __init__(self, start=1_000_000_000, step=100_000_000):
        self.value = start
        self.step = step

    def __call__(self):
        value = self.value
        self.value += self.step
        return value


class StopAfterCall:
    def __init__(self, stop_on_call):
        self.stop_on_call = stop_on_call
        self.calls = 0

    def __call__(self):
        self.calls += 1
        return self.calls >= self.stop_on_call


class SequenceGateway:
    def __init__(self, active_ticks=(), fail_tick=None, invalid_context_tick=None):
        self.active_ticks = set(active_ticks)
        self.fail_tick = fail_tick
        self.invalid_context_tick = invalid_context_tick
        self.calls = []

    def snapshot(self, context):
        self.calls.append(context.tick_id)
        if context.tick_id == self.fail_tick:
            raise OSError("injected command edge failure")
        if context.tick_id == self.invalid_context_tick:
            foreign_context = TickContext(
                context.tick_id + 1,
                context.monotonic_ns,
            )
            return CommandRequest(
                foreign_context,
                "resident.invalid-context",
                CommandMode.STOP,
                (),
                foreign_context.tick_id,
            )
        if context.tick_id in self.active_ticks:
            return CommandRequest(
                context,
                f"resident.active.{context.tick_id}",
                CommandMode.TELEOP,
                (
                    DataField("v_mps", 0.08),
                    DataField("omega_rad_s", 0.0),
                    DataField("max_v_mps", 0.10),
                    DataField("max_omega_rad_s", 0.20),
                ),
                context.tick_id,
            )
        return CommandRequest(
            context,
            f"resident.stop.{context.tick_id}",
            CommandMode.STOP,
            (),
            context.tick_id,
        )


def _policy():
    return NativeSensorPolicyConfig(
        encoder_maximum_sample_interval_ns=150_000_000,
        encoder_maximum_abs_velocity_mps=1.5,
        encoder_minimum_trust=0.5,
        imu_maximum_sample_age_ns=150_000_000,
        imu_heading_clockwise_positive=True,
        imu_yaw_rate_axis=2,
        imu_yaw_rate_clockwise_positive=False,
        imu_yaw_offset_rad=0.0,
        imu_minimum_confidence=0.5,
        imu_minimum_calibration=2,
        imu_allow_rate_only=True,
        lidar_maximum_result_age_ns=150_000_000,
        lidar_maximum_future_skew_ns=10_000_000,
        lidar_pose_r_scale=1.0,
        lidar_minimum_confidence=0.5,
        lidar_maximum_measurement_age_ns=150_000_000,
    )


def _runtime_config(*, required_lidar_preflight_revisions=1):
    bounded = load_bounded_physical_runtime_config(
        PROJECT_ROOT / "conf" / "hardver.json",
        PROJECT_ROOT / "conf" / "fizika.json",
        PROJECT_ROOT / "conf" / "speed_map.json",
        BoundedTeleopProfile(
            "resident-config-source",
            start_tick_id=1,
            active_tick_count=1,
            v_mps=0.08,
            omega_rad_s=0.0,
            max_v_mps=0.10,
            max_omega_rad_s=0.20,
        ),
        sensor_policy=_policy(),
    )
    return ResidentPhysicalRuntimeConfig.from_bounded(
        bounded,
        required_lidar_preflight_revisions=required_lidar_preflight_revisions,
    )


def _sources(encoder, lidar_backend=None):
    return (
        NativeEncoderSource(encoder, NativeEncoderConfig("encoder", 0.5)),
        NativeImuSource(ImuBackend(), NativeImuConfig("imu", 0.5, 2)),
        NativeLidarSource(
            lidar_backend or LidarBackend(),
            NativeLidarConfig("RPLIDAR_C1", 0.5, 100_000_000),
        ),
    )


def test_resident_runtime_rearms_then_runs_active_and_signal_shutdown_tick():
    encoder = EncoderBackend()
    gateway = SequenceGateway(active_ticks=(1, 2))
    motor = MotorGpio()
    observed = []

    report = run_resident_physical_control(
        *_sources(encoder),
        gateway,
        motor,
        _runtime_config(),
        stop_requested=StopAfterCall(5),
        monotonic_ns=StepClock(),
        sleep=lambda _seconds: None,
        tick_observer=observed.append,
    )

    assert report.status == RUN_OK
    assert report.exit_reason == "STOP_REQUESTED"
    assert report.normal_tick_count == 3
    assert report.tick_count == 4
    assert report.last_tick_id == 3
    assert report.final_lifecycle is LifecycleState.SHUTDOWN
    assert report.final_safety_decision is SafetyDecision.STOP
    assert report.final_reason == "NOT_ACTIVE"
    assert report.operator_stopped is True
    assert report.as_dict() == {
        "schema": "R2B4_V3_RESIDENT_RUNTIME_REPORT_V2",
        "status": "PASS",
        "run_status": RUN_OK,
        "exit_reason": "STOP_REQUESTED",
        "tick_count": 4,
        "normal_tick_count": 3,
        "last_tick_id": 3,
        "final_lifecycle": "SHUTDOWN",
        "final_safety_decision": "STOP",
        "final_reason": "NOT_ACTIVE",
        "fault_layer": None,
        "operator_stopped": True,
        "termination_class": "SHUTDOWN_SAFE_LOW",
    }
    assert gateway.calls == [0, 1, 2]
    assert [item.trace.context.tick_id for item in observed] == [0, 1, 2, 3]
    assert any(call[0] == "pwm" and call[-1] != 0.0 for call in motor.calls)
    assert motor.busy == set()
    assert motor.levels == {12: 0, 13: 0, 18: 0, 19: 0}
    assert motor.calls[-1] == ("close", 4)


def test_resident_runtime_rejects_active_first_tick_and_latches_fault_zero():
    motor = MotorGpio()

    report = run_resident_physical_control(
        *_sources(EncoderBackend()),
        SequenceGateway(active_ticks=(0,)),
        motor,
        _runtime_config(),
        stop_requested=lambda: False,
        monotonic_ns=StepClock(),
        sleep=lambda _seconds: None,
    )

    assert report.status == RUN_FAULT
    assert report.exit_reason == "RUNTIME_FAULT"
    assert report.normal_tick_count == 1
    assert report.final_lifecycle is LifecycleState.FAULT
    assert report.final_safety_decision is SafetyDecision.FAULT
    assert report.final_reason == "PREFLIGHT_REQUIRED"
    assert report.fault_layer == "ResidentLiveControl"
    assert report.termination_class == "FAULT_SAFE_LOW"
    assert report.as_dict()["termination_class"] == "FAULT_SAFE_LOW"
    assert not any(call[0] == "pwm" and call[-1] != 0.0 for call in motor.calls)
    assert motor.calls[-1] == ("close", 4)


def test_resident_command_edge_exception_is_one_fault_commit_and_close():
    motor = MotorGpio()

    report = run_resident_physical_control(
        *_sources(EncoderBackend()),
        SequenceGateway(fail_tick=0),
        motor,
        _runtime_config(),
        stop_requested=lambda: False,
        monotonic_ns=StepClock(),
        sleep=lambda _seconds: None,
    )

    assert report.status == RUN_FAULT
    assert report.final_reason == "COMMAND_GATEWAY_ERROR"
    assert report.fault_layer == "CommandGateway"
    assert not any(call[0] == "pwm" and call[-1] != 0.0 for call in motor.calls)
    assert motor.calls[-1] == ("close", 4)


def test_resident_runtime_requires_a_new_idle_preflight_after_each_stop():
    gateway = SequenceGateway(active_ticks=(1, 3))
    observed = []

    report = run_resident_physical_control(
        *_sources(EncoderBackend()),
        gateway,
        MotorGpio(),
        _runtime_config(),
        stop_requested=StopAfterCall(6),
        monotonic_ns=StepClock(),
        sleep=lambda _seconds: None,
        tick_observer=observed.append,
    )

    assert report.status == RUN_OK
    assert report.normal_tick_count == 4
    assert gateway.calls == [0, 1, 2, 3]
    assert [
        item.trace.context.tick_id
        for item in observed
        if item.final_actuation.safety_decision is SafetyDecision.ALLOW
    ] == [1, 3]


def test_resident_command_context_mismatch_faults_before_any_nonzero_commit():
    motor = MotorGpio()

    report = run_resident_physical_control(
        *_sources(EncoderBackend()),
        SequenceGateway(invalid_context_tick=0),
        motor,
        _runtime_config(),
        stop_requested=lambda: False,
        monotonic_ns=StepClock(),
        sleep=lambda _seconds: None,
    )

    assert report.status == RUN_FAULT
    assert report.final_reason == "COMMAND_GATEWAY_INVALID"
    assert report.fault_layer == "CommandGateway"
    assert not any(call[0] == "pwm" and call[-1] != 0.0 for call in motor.calls)


def test_initial_stop_and_invalid_gateway_claim_no_motor_capability():
    motor = MotorGpio()
    report = run_resident_physical_control(
        *_sources(EncoderBackend()),
        SequenceGateway(),
        motor,
        _runtime_config(),
        stop_requested=lambda: True,
    )
    assert report.status == RUN_OK
    assert report.exit_reason == "STOP_REQUESTED_BEFORE_START"
    assert report.tick_count == 0
    assert report.termination_class == "OUTPUT_NOT_OPENED"
    assert motor.calls == []

    with pytest.raises(TypeError, match="command_gateway"):
        run_resident_physical_control(
            *_sources(EncoderBackend()),
            object(),
            motor,
            _runtime_config(),
            stop_requested=lambda: False,
        )
    assert motor.calls == []


def test_resident_activation_requires_three_distinct_healthy_lidar_revisions():
    successful = run_resident_physical_control(
        *_sources(EncoderBackend()),
        SequenceGateway(active_ticks=(3,)),
        MotorGpio(),
        _runtime_config(required_lidar_preflight_revisions=3),
        stop_requested=StopAfterCall(6),
        monotonic_ns=StepClock(),
        sleep=lambda _seconds: None,
    )

    repeated_revision = run_resident_physical_control(
        *_sources(
            EncoderBackend(),
            LidarBackend({0: 1, 1: 1, 2: 2, 3: 3}),
        ),
        SequenceGateway(active_ticks=(3,)),
        MotorGpio(),
        _runtime_config(required_lidar_preflight_revisions=3),
        stop_requested=lambda: False,
        monotonic_ns=StepClock(),
        sleep=lambda _seconds: None,
    )

    assert successful.status == RUN_OK
    assert repeated_revision.status == RUN_FAULT
    assert repeated_revision.final_reason == "PREFLIGHT_REQUIRED"
    assert repeated_revision.fault_layer == "ResidentLiveControl"
