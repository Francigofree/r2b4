from types import MappingProxyType

from tools.v3_sensor_measurement import _json_value, summarize_report
from v3.adapters.live_encoder import EncoderVelocityReading, NativeEncoderConfig, NativeEncoderSource
from v3.adapters.live_imu import ImuHeadingReading, NativeImuConfig, NativeImuSource
from v3.adapters.live_lidar import LidarHealthReading, NativeLidarConfig, NativeLidarSource
from v3.composition.live_inputs import LiveInputComposition
from v3.contracts import TickContext
from v3_hardware_runtime import SensorMeasurementReport


class Backend:
    def __init__(self, values):
        self.values = iter(values)

    def read(self, _context):
        return next(self.values)


def test_native_lidar_diagnostics_mapping_is_recursively_json_compatible():
    value = MappingProxyType(
        {"quality": MappingProxyType({"reasons": ("a", "b")})}
    )

    assert _json_value(value) == {"quality": {"reasons": ["a", "b"]}}


def test_summary_exposes_health_ranges_estimate_and_zero_commit():
    context = TickContext(0, 1_000)
    encoder = NativeEncoderSource(
        Backend((EncoderVelocityReading(0, 1_000, 0.1, 0.2, 1.0, False, True),)),
        NativeEncoderConfig("encoder", 0.5),
    )
    imu = NativeImuSource(
        Backend((ImuHeadingReading(0, 1_000, 0.3, 0.4, 1.0, 3, False, True),)),
        NativeImuConfig("imu", 0.5, 2),
    )
    lidar = NativeLidarSource(
        Backend((LidarHealthReading(1, 1_000, 0, 1.0, False, True),)),
        NativeLidarConfig("lidar", 0.2, 100),
    )
    result = LiveInputComposition(encoder, imu, lidar).tick(context)

    summary = summarize_report(SensorMeasurementReport((result,), False))

    assert summary["status"] == "PASS"
    assert summary["healthy_tick_count"] == 1
    assert summary["l3_estimate_count"] == 1
    assert summary["all_commits_zero"] is True
    assert summary["source_state_counts"] == {
        "encoder": {"OK": 1},
        "imu": {"OK": 1},
        "lidar": {"OK": 1},
    }
    assert summary["sample_ranges"]["wheel_velocity"]["left_mps"] == [0.1, 0.1]
    assert summary["sample_ranges"]["ekf_heading"]["yaw_rad"] == [0.3, 0.3]
    assert summary["last_estimate"]["yaw_rad"] != 0.0
