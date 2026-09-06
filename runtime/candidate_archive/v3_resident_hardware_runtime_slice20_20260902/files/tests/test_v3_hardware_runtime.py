from pathlib import Path

import pytest

from v3.adapters.bno055_device import NativeBno055Device
from v3.adapters.bounded_command import BoundedTeleopProfile
from v3.contracts import (
    CommandMode,
    CommandRequest,
    DataField,
    LifecycleState,
    SafetyDecision,
)
from v3_bounded_config import NativeSensorPolicyConfig, load_bounded_physical_runtime_config
from v3_hardware_runtime import (
    FiniteSensorMeasurementConfig,
    PHYSICAL_RUN_APPROVAL,
    RESIDENT_PHYSICAL_RUN_APPROVAL,
    run_finite_sensor_measurement,
    run_native_hardware_bounded_physical_control,
    run_native_hardware_resident_control,
)
from v3_runtime import ResidentPhysicalRuntimeConfig


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class Callback:
    def __init__(self) -> None:
        self.cancel_calls = 0

    def cancel(self):
        self.cancel_calls += 1
        return 0


class CounterGpio:
    RISING_EDGE = 1
    BOTH_EDGES = 2
    SET_PULL_UP = 4

    def __init__(self, events) -> None:
        self.events = events
        self.callbacks = []
        self.open_calls = 0
        self.close_calls = 0

    def gpiochip_open(self, chip):
        self.events.append("counter-open")
        self.open_calls += 1
        return 10

    def gpio_claim_alert(self, handle, pin, edge, flags):
        return 0

    def gpio_set_debounce_micros(self, handle, pin, micros):
        return 0

    def gpio_read(self, handle, pin):
        return 1

    def callback(self, handle, pin, edge, function):
        callback = Callback()
        self.callbacks.append(callback)
        return callback

    def gpio_free(self, handle, pin):
        return 0

    def gpiochip_close(self, handle):
        self.close_calls += 1
        return 0


class ImuBus:
    def __init__(self) -> None:
        self.close_calls = 0
        self.burst_calls = 0

    def read_byte_data(self, address, register):
        if register == NativeBno055Device.REG_CHIP_ID:
            return NativeBno055Device.CHIP_ID
        if register == NativeBno055Device.REG_CALIB_STAT:
            return 0xFF
        if register == NativeBno055Device.REG_SYS_STATUS:
            return 5
        if register == NativeBno055Device.REG_SYS_ERR:
            return 0
        raise AssertionError(register)

    def write_byte_data(self, address, register, value):
        return 0

    def read_i2c_block_data(self, address, register, length):
        self.burst_calls += 1
        return [0] * 32

    def close(self):
        self.close_calls += 1


class MatcherResult:
    def __init__(self, revision, timestamp) -> None:
        self.matcher_result_id = revision
        self.candidate_id = revision
        self.source_raw_scan_id = revision
        self.source_raw_scan_timestamp = timestamp
        self.timestamp = timestamp
        self.summary = {
            "matcher_contract_id": "R2B4_SCAN_MATCHER_PROCESS_LATEST_ONLY_V1",
            "matcher_confidence_model": "R2B4_SCAN_MATCH_CONFIDENCE_V2",
            "matcher_transport": "process_latest_only",
            "map_frame_id": "R2B4_BOOT_ROBOT_MAP",
            "map_frame_owner": "EKF_POSE_ODOMETRY_SSOT",
            "yaw_convention": "CCW_POSITIVE_LEFT",
            "lidar_pose_x": 0.0,
            "lidar_pose_y": 0.0,
            "lidar_pose_theta": 0.0,
            "lidar_pose_confidence": 1.0,
        }


class LidarPort:
    def __init__(self, timestamps) -> None:
        self.timestamps = iter(timestamps)
        self.revision = 0
        self.result_calls = 0
        self.status_calls = 0
        self.stop_calls = 0

    def get_matcher_result(self):
        self.result_calls += 1
        self.revision += 1
        return MatcherResult(self.revision, next(self.timestamps))

    def get_runtime_status(self):
        self.status_calls += 1
        return {
            "matcher_contract_id": "R2B4_SCAN_MATCHER_PROCESS_LATEST_ONLY_V1",
            "matcher_confidence_model": "R2B4_SCAN_MATCH_CONFIDENCE_V2",
            "matcher_transport": "process_latest_only",
            "running": True,
            "matcher_process_alive": True,
            "health": "OK",
        }

    def stop(self):
        self.stop_calls += 1


class MotorGpio:
    def __init__(self) -> None:
        self.calls = []
        self.levels = {}
        self.pwm_busy = set()

    def gpiochip_open(self, chip):
        self.calls.append(("open", chip))
        return 20

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
        self.pwm_busy.discard(pin)
        return 0

    def tx_busy(self, handle, pin, kind):
        self.calls.append(("busy", pin, kind))
        return int(pin in self.pwm_busy)

    def tx_pwm(self, handle, pin, frequency_hz, duty_cycle):
        self.calls.append(("pwm", pin, duty_cycle))
        if frequency_hz == 0:
            self.pwm_busy.discard(pin)
        elif duty_cycle != 0.0:
            self.pwm_busy.add(pin)
        return 0

    def gpiochip_close(self, handle):
        self.calls.append(("close", handle))
        return 0


class StepClock:
    def __init__(self, start=1_000_000_000, step=20_000_000) -> None:
        self.value = start
        self.step = step

    def __call__(self):
        value = self.value
        self.value += self.step
        return value


class StopAfterCall:
    def __init__(self, stop_on_call) -> None:
        self.stop_on_call = stop_on_call
        self.calls = 0

    def __call__(self):
        self.calls += 1
        return self.calls >= self.stop_on_call


class ResidentGateway:
    def __init__(self) -> None:
        self.calls = []

    def snapshot(self, context):
        self.calls.append(context.tick_id)
        if context.tick_id == 2:
            return CommandRequest(
                context,
                "resident-hardware-active",
                CommandMode.TELEOP,
                (
                    DataField("v_mps", 0.05),
                    DataField("omega_rad_s", 0.0),
                    DataField("max_v_mps", 0.10),
                    DataField("max_omega_rad_s", 0.20),
                ),
                context.tick_id,
            )
        return CommandRequest(
            context,
            f"resident-hardware-stop-{context.tick_id}",
            CommandMode.STOP,
            (),
            context.tick_id,
        )


def _policy() -> NativeSensorPolicyConfig:
    return NativeSensorPolicyConfig(
        encoder_maximum_sample_interval_ns=100_000_000,
        encoder_maximum_abs_velocity_mps=1.5,
        encoder_minimum_trust=0.5,
        imu_maximum_sample_age_ns=100_000_000,
        imu_heading_clockwise_positive=True,
        imu_yaw_rate_axis=2,
        imu_yaw_rate_clockwise_positive=False,
        imu_yaw_offset_rad=0.0,
        imu_minimum_confidence=0.5,
        imu_minimum_calibration=2,
        imu_allow_rate_only=True,
        lidar_maximum_result_age_ns=250_000_000,
        lidar_maximum_future_skew_ns=10_000_000,
        lidar_pose_r_scale=1.0,
        lidar_minimum_confidence=0.2,
        lidar_maximum_measurement_age_ns=250_000_000,
    )


def _runtime_config():
    return load_bounded_physical_runtime_config(
        PROJECT_ROOT / "conf" / "hardver.json",
        PROJECT_ROOT / "conf" / "fizika.json",
        PROJECT_ROOT / "conf" / "speed_map.json",
        BoundedTeleopProfile(
            "hardware-runtime-test",
            start_tick_id=2,
            active_tick_count=1,
            v_mps=0.05,
            omega_rad_s=0.0,
            max_v_mps=0.10,
            max_omega_rad_s=0.20,
        ),
        sensor_policy=_policy(),
    )


def _ports(timestamps):
    events = []
    counter = CounterGpio(events)
    imu = ImuBus()
    lidar = LidarPort(timestamps)

    def open_imu(bus_number):
        assert bus_number == 1
        events.append("imu-open")
        return imu

    pose_providers = []

    def open_lidar(pose_provider):
        events.append("lidar-open")
        pose_providers.append(pose_provider)
        return lidar

    return events, counter, imu, lidar, open_imu, open_lidar, pose_providers


def test_finite_measurement_runs_real_l1_l3_path_and_closes_every_owner():
    runtime = _runtime_config()
    measurement = FiniteSensorMeasurementConfig.from_runtime(runtime, tick_count=3)
    events, counter, imu, lidar, open_imu, open_lidar, pose_providers = _ports(
        (1.06, 1.08, 1.10)
    )

    report = run_finite_sensor_measurement(
        counter,
        open_imu,
        open_lidar,
        measurement,
        stop_requested=lambda: False,
        monotonic_ns=StepClock(),
        sleep=lambda _: None,
    )

    assert events == ["imu-open", "lidar-open", "counter-open"]
    assert len(report.ticks) == 3
    assert report.operator_stopped is False
    assert report.healthy_tick_count == 2  # Tick 0 is the encoder baseline.
    assert len(report.l3_estimates) == 3
    assert report.fault_tick_count == 0
    assert report.all_commits_zero is True
    assert all(
        item.final_actuation.safety_decision
        in (SafetyDecision.STOP, SafetyDecision.FAULT)
        for item in report.ticks
    )
    assert imu.burst_calls == 4  # One init proof plus one read per V3 tick.
    assert lidar.result_calls == 3
    assert lidar.status_calls == 3
    assert imu.close_calls == 1
    assert lidar.stop_calls == 1
    assert counter.close_calls == 1
    assert all(item.cancel_calls == 1 for item in counter.callbacks)
    assert len(pose_providers) == 1
    assert pose_providers[0]() == (
        report.l3_estimates[-1].x_m,
        report.l3_estimates[-1].y_m,
        report.l3_estimates[-1].yaw_rad,
    )


def test_initial_stop_and_missing_approval_open_no_hardware():
    runtime = _runtime_config()
    measurement = FiniteSensorMeasurementConfig.from_runtime(runtime, tick_count=3)
    events, counter, imu, lidar, open_imu, open_lidar, _ = _ports(())

    report = run_finite_sensor_measurement(
        counter,
        open_imu,
        open_lidar,
        measurement,
        stop_requested=lambda: True,
    )
    assert report.ticks == ()
    assert report.operator_stopped is True
    assert events == []

    with pytest.raises(PermissionError, match="approval"):
        run_native_hardware_bounded_physical_control(
            counter,
            open_imu,
            open_lidar,
            MotorGpio(),
            runtime,
            approval="wrong",
            stop_requested=lambda: False,
        )
    assert events == []
    assert imu.close_calls == 0
    assert lidar.stop_calls == 0


def test_bounded_physical_surface_uses_existing_single_writer_and_final_zero():
    runtime = _runtime_config()
    events, counter, imu, lidar, open_imu, open_lidar, pose_providers = _ports(
        (1.06, 1.08, 1.10, 1.12)
    )
    motor = MotorGpio()

    status = run_native_hardware_bounded_physical_control(
        counter,
        open_imu,
        open_lidar,
        motor,
        runtime,
        approval=PHYSICAL_RUN_APPROVAL,
        stop_requested=lambda: False,
        monotonic_ns=StepClock(),
        sleep=lambda _: None,
    )

    assert status == 0
    assert len([call for call in motor.calls if call[0] == "open"]) == 1
    nonzero = [call for call in motor.calls if call[0] == "pwm" and call[2] != 0.0]
    assert nonzero
    last_pwm_by_pin = {
        call[1]: call[2] for call in motor.calls if call[0] == "pwm"
    }
    assert last_pwm_by_pin == {12: 0.0, 13: 0.0, 18: 0.0, 19: 0.0}
    assert motor.calls[-1] == ("close", 20)
    assert imu.close_calls == 1
    assert lidar.stop_calls == 1
    assert counter.close_calls == 1
    assert len(pose_providers) == 1


def test_invalid_measurement_config_fails_before_any_factory_call():
    runtime = _runtime_config()
    assert runtime.sensor_inputs is not None
    with pytest.raises(ValueError, match="admission freshness"):
        FiniteSensorMeasurementConfig(
            runtime.sensor_inputs,
            FiniteSensorMeasurementConfig.from_runtime(
                runtime,
                tick_count=1,
            ).live_inputs,
            tick_count=1,
            tick_period_ns=300_000_000,
        )


def test_resident_hardware_surface_owns_all_edges_and_signal_shutdowns_zero():
    resident = ResidentPhysicalRuntimeConfig.from_bounded(_runtime_config())
    events, counter, imu, lidar, open_imu, open_lidar, pose_providers = _ports(
        (1.06, 1.08, 1.10, 1.12)
    )
    motor = MotorGpio()
    gateway = ResidentGateway()
    observed = []

    report = run_native_hardware_resident_control(
        counter,
        open_imu,
        open_lidar,
        gateway,
        motor,
        resident,
        approval=RESIDENT_PHYSICAL_RUN_APPROVAL,
        stop_requested=StopAfterCall(6),
        monotonic_ns=StepClock(),
        sleep=lambda _seconds: None,
        tick_observer=observed.append,
    )

    assert report.status == 0
    assert report.exit_reason == "STOP_REQUESTED"
    assert report.normal_tick_count == 3
    assert report.tick_count == 4
    assert report.final_lifecycle is LifecycleState.SHUTDOWN
    assert report.final_safety_decision is SafetyDecision.STOP
    assert gateway.calls == [0, 1, 2]
    assert [item.trace.context.tick_id for item in observed] == [0, 1, 2, 3]
    assert any(call[0] == "pwm" and call[2] != 0.0 for call in motor.calls)
    assert motor.pwm_busy == set()
    assert motor.calls[-1] == ("close", 20)
    assert events == ["imu-open", "lidar-open", "counter-open"]
    assert imu.close_calls == 1
    assert lidar.stop_calls == 1
    assert counter.close_calls == 1
    assert len(pose_providers) == 1


def test_resident_hardware_approval_and_initial_stop_open_no_device():
    resident = ResidentPhysicalRuntimeConfig.from_bounded(_runtime_config())
    events, counter, imu, lidar, open_imu, open_lidar, _ = _ports(())
    gateway = ResidentGateway()

    with pytest.raises(PermissionError, match="resident V3 approval"):
        run_native_hardware_resident_control(
            counter,
            open_imu,
            open_lidar,
            gateway,
            MotorGpio(),
            resident,
            approval="wrong",
            stop_requested=lambda: False,
        )
    assert events == []

    report = run_native_hardware_resident_control(
        counter,
        open_imu,
        open_lidar,
        gateway,
        MotorGpio(),
        resident,
        approval=RESIDENT_PHYSICAL_RUN_APPROVAL,
        stop_requested=lambda: True,
    )
    assert report.exit_reason == "STOP_REQUESTED_BEFORE_START"
    assert report.tick_count == 0
    assert events == []
    assert imu.close_calls == 0
    assert lidar.stop_calls == 0
