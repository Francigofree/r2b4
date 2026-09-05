from dataclasses import replace
from types import SimpleNamespace

import pytest

from v3.adapters.bno055_imu import Bno055ImuBackendConfig
from v3.adapters.bounded_command import BoundedTeleopProfile
from v3.adapters.counter_encoder import CounterEncoderBackendConfig
from v3.adapters.gpio_counter import (
    GpioCounterChannelConfig,
    GpioCounterPairConfig,
)
from v3.adapters.gpio_motor import GpioMotorFrameSinkConfig
from v3.adapters.latest_lidar import LatestLidarBackendConfig
from v3.adapters.live_encoder import NativeEncoderConfig
from v3.adapters.live_imu import NativeImuConfig
from v3.adapters.live_inputs import NativeLiveInputReader
from v3.adapters.live_lidar import NativeLidarConfig
from v3.adapters.motor_pwm import MotorChannelPhysicalConfig
from v3.composition.bounded_live_control import BoundedLiveControlConfig
from v3.composition.bounded_physical_control import BoundedPhysicalControlConfig
from v3.composition.native_control import NativeControlCompositionConfig
from v3.composition.native_sensor_inputs import (
    NativeSensorInputConfig,
    NativeSensorInputOwner,
)
from v3.contracts import TickContext
from v3.layers.l11_actuator_control import (
    SpeedMapPoint,
    WheelSpeedCurve,
    WheelSpeedMap,
)
from v3_bounded_runtime import (
    BoundedPhysicalRuntimeConfig,
    RUN_OK,
    run_owned_bounded_physical_control,
)


class Callback:
    def __init__(self, function) -> None:
        self.function = function
        self.cancel_calls = 0

    def cancel(self) -> int:
        self.cancel_calls += 1
        return 0


class CounterGpio:
    RISING_EDGE = 1
    BOTH_EDGES = 2
    SET_PULL_UP = 4

    def __init__(self) -> None:
        self.open_calls = 0
        self.close_calls = 0
        self.callbacks = []

    def gpiochip_open(self, chip):
        self.open_calls += 1
        return 10

    def gpio_claim_alert(self, handle, pin, edge, flags):
        return 0

    def gpio_set_debounce_micros(self, handle, pin, debounce_micros):
        return 0

    def gpio_read(self, handle, pin):
        return 1

    def callback(self, handle, pin, edge, function):
        callback = Callback(function)
        self.callbacks.append(callback)
        return callback

    def gpio_free(self, handle, pin):
        return 0

    def gpiochip_close(self, handle):
        self.close_calls += 1
        return 0


class ImuDevice:
    initialized = True
    sensor_ok = True

    def __init__(self) -> None:
        self.read_calls = 0
        self.close_calls = 0

    def read_sample(self, *, force=False):
        assert force is True
        self.read_calls += 1
        return {
            "timestamp": 1.0,
            "heading_deg": 0.0,
            "gyro_dps": (0.0, 0.0, 0.0),
            "calibration": {"sys": 3, "gyro": 3, "accel": 3, "mag": 3},
            "sys_error": 0,
        }

    def close(self):
        self.close_calls += 1


class MatcherResult:
    matcher_result_id = 1
    candidate_id = 1
    source_raw_scan_id = 1
    source_raw_scan_timestamp = 1.0
    timestamp = 1.0
    summary = {
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
    def __init__(self) -> None:
        self.result_calls = 0
        self.status_calls = 0
        self.stop_calls = 0

    def get_matcher_result(self):
        self.result_calls += 1
        return MatcherResult()

    def get_runtime_status(self):
        self.status_calls += 1
        return {
            "matcher_contract_id": "R2B4_SCAN_MATCHER_PROCESS_LATEST_ONLY_V1",
            "matcher_confidence_model": "R2B4_SCAN_MATCH_CONFIDENCE_V2",
            "matcher_transport": "process_latest_only",
            "running": True,
            "matcher_process_alive": True,
            "driver_connected": True,
            "health": "OK",
        }

    def get_raw_scan_snapshot(self):
        return SimpleNamespace(
            raw_scan_id=1,
            raw_scan_timestamp=1.0,
            health="OK",
            raw_scan=(),
            summary={
                "raw_safety_valid_point_count": 80,
                "front_clearance_m": 1.0,
                "rear_clearance_m": 1.0,
                "left_clearance_m": 1.0,
                "right_clearance_m": 1.0,
                "front_observation_count": 20,
                "rear_observation_count": 20,
                "left_observation_count": 20,
                "right_observation_count": 20,
            },
        )

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


def _input_config() -> NativeSensorInputConfig:
    return NativeSensorInputConfig(
        encoder_counter=GpioCounterPairConfig(
            GpioCounterChannelConfig(1, 2),
            GpioCounterChannelConfig(3, 4),
        ),
        encoder_backend=CounterEncoderBackendConfig(0.001, 0.001),
        encoder_source=NativeEncoderConfig("encoder", 0.5),
        imu_backend=Bno055ImuBackendConfig(100_000_000, True, 2, False),
        imu_source=NativeImuConfig("imu", 0.5, 2),
        lidar_backend=LatestLidarBackendConfig(100_000_000),
        lidar_source=NativeLidarConfig("lidar", 0.5, 100_000_000),
    )


def test_rate_only_imu_requires_a_positive_lidar_first_confidence_gate():
    config = _input_config()

    with pytest.raises(ValueError, match="positive LIDAR_FIRST confidence"):
        replace(
            config,
            imu_source=replace(config.imu_source, allow_rate_only=True),
            lidar_source=replace(config.lidar_source, minimum_confidence=0.0),
        )


def _owner():
    gpio = CounterGpio()
    imu = ImuDevice()
    lidar = LidarPort()
    owner = NativeSensorInputOwner(gpio, imu, lidar, _input_config())
    return owner, gpio, imu, lidar


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
        BoundedPhysicalControlConfig(
            BoundedLiveControlConfig(
                BoundedTeleopProfile(
                    "owned-inputs",
                    start_tick_id=2,
                    active_tick_count=1,
                    v_mps=0.05,
                    omega_rad_s=0.0,
                    max_v_mps=0.10,
                    max_omega_rad_s=0.20,
                ),
                NativeControlCompositionConfig(speed_map=_speed_map()),
            ),
            GpioMotorFrameSinkConfig(
                MotorChannelPhysicalConfig(12, 13),
                MotorChannelPhysicalConfig(18, 19),
            ),
        )
    )


def test_owner_closes_three_native_sources_into_one_ordered_tick_batch():
    owner, gpio, imu, lidar = _owner()

    batch = NativeLiveInputReader(owner.sources).read(
        TickContext(0, 1_000_000_000)
    )

    assert tuple(item.device_id for item in batch.device_health) == (
        "encoder",
        "imu",
        "lidar",
    )
    assert tuple(sample.kind for sample in batch.samples) == (
        "wheel_velocity",
        "ekf_heading",
        "lidar_health",
        "lidar_safety_clearance",
        "lidar_localization_health",
        "lidar_matcher_diagnostics",
        "lidar_pose",
    )
    assert imu.read_calls == 1
    assert lidar.result_calls == 1
    assert lidar.status_calls == 1

    owner.close()
    owner.close()

    assert owner.closed is True
    assert gpio.close_calls == 1
    assert imu.close_calls == 1
    assert lidar.stop_calls == 1
    assert all(callback.cancel_calls == 1 for callback in gpio.callbacks)


def test_invalid_external_port_is_rejected_before_encoder_gpio_open():
    gpio = CounterGpio()

    with pytest.raises(TypeError, match="get_matcher_result"):
        NativeSensorInputOwner(
            gpio,
            ImuDevice(),
            object(),  # type: ignore[arg-type]
            _input_config(),
        )

    assert gpio.open_calls == 0


def test_failure_after_gpio_open_closes_every_transferred_capability(monkeypatch):
    gpio = CounterGpio()
    imu = ImuDevice()
    lidar = LidarPort()

    def fail_source(*_args, **_kwargs):
        raise RuntimeError("injected source construction failure")

    monkeypatch.setattr(
        "v3.composition.native_sensor_inputs.NativeImuSource",
        fail_source,
    )

    with pytest.raises(RuntimeError, match="source construction failure"):
        NativeSensorInputOwner(gpio, imu, lidar, _input_config())

    assert gpio.open_calls == 1
    assert gpio.close_calls == 1
    assert imu.close_calls == 1
    assert lidar.stop_calls == 1


def test_config_rejects_duplicate_device_identity_and_frame_drift():
    config = _input_config()
    with pytest.raises(ValueError, match="device IDs"):
        replace(config, imu_source=NativeImuConfig("encoder", 0.5, 2))
    with pytest.raises(ValueError, match="pose frame IDs"):
        replace(
            config,
            lidar_source=NativeLidarConfig(
                "lidar",
                0.5,
                100_000_000,
                pose_frame_id="wrong",
            ),
        )


def test_owned_runtime_initial_stop_closes_inputs_without_motor_or_sensor_poll():
    owner, gpio, imu, lidar = _owner()
    motor = MotorGpio()

    status = run_owned_bounded_physical_control(
        owner,
        motor,
        _runtime_config(),
        stop_requested=lambda: True,
    )

    assert status == RUN_OK
    assert owner.closed is True
    assert gpio.close_calls == 1
    assert imu.read_calls == 0
    assert lidar.result_calls == 0
    assert lidar.status_calls == 0
    assert imu.close_calls == 1
    assert lidar.stop_calls == 1
    assert motor.calls == []
