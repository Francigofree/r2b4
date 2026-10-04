import math
import pytest
from v3.adapters.bno055_imu import Bno055ImuBackendConfig, NativeBno055ImuBackend
from v3.adapters.live_imu import NativeImuConfig, NativeImuSource
from v3.contracts import DeviceHealthState, TickContext

class Device:

    def __init__(self, sample: object) -> None:
        self.sample = sample
        self.initialized = True
        self.sensor_ok = True
        self.read_calls: list[bool] = []
        self.close_calls = 0

    def read_sample(self, *, force: bool=False):
        self.read_calls.append(force)
        return self.sample

    def close(self) -> None:
        self.close_calls += 1

def _sample(**changes):
    value = {'timestamp': 0.99, 'heading_deg': 90.0, 'gyro_dps': (0.0, 0.0, 30.0), 'calibration': {'sys': 3, 'gyro': 3, 'accel': 2, 'mag': 3}, 'sys_error': 0}
    value.update(changes)
    return value

def _config(**changes) -> Bno055ImuBackendConfig:
    values = {'maximum_sample_age_ns': 20000000, 'heading_clockwise_positive': True, 'yaw_rate_axis': 2, 'yaw_rate_clockwise_positive': False}
    values.update(changes)
    return Bno055ImuBackendConfig(**values)

def test_stale_and_device_failure_map_through_existing_native_source_health():
    stale_device = Device(_sample(timestamp=0.97))
    stale = NativeImuSource(NativeBno055ImuBackend(stale_device, _config()), NativeImuConfig('imu', 0.5, 2)).read(TickContext(1, 1000000000))
    failed_device = Device(_sample())
    failed_device.sensor_ok = False
    failed = NativeImuSource(NativeBno055ImuBackend(failed_device, _config()), NativeImuConfig('imu', 0.5, 2)).read(TickContext(1, 1000000000))
    assert stale_device.read_calls == [True]
    assert stale.health.state is DeviceHealthState.DEGRADED
    assert stale.health.reason == 'IMU_STALE'
    assert failed_device.read_calls == [True]
    assert failed.health.state is DeviceHealthState.FAILED
    assert failed.health.reason == 'IMU_TIMING_INVALID'

@pytest.mark.parametrize('sample', (object(), _sample(timestamp=float('nan')), _sample(gyro_dps=(0.0, 1.0)), _sample(calibration={'sys': 4, 'gyro': 3, 'accel': 3, 'mag': 3})))
def test_malformed_atomic_sample_fails_closed_without_retry(sample):
    device = Device(sample)
    backend = NativeBno055ImuBackend(device, _config())
    with pytest.raises((TypeError, ValueError)):
        backend.read(TickContext(1, 1000000000))
    assert device.read_calls == [True]
