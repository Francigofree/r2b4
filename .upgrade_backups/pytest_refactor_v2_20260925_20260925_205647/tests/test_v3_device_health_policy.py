from v3.contracts import DeviceHealth, DeviceHealthState
from v3.device_health_policy import (
    PRODUCTION_CRITICAL_DEVICE_IDS,
    blocking_degraded_sources,
    critical_device_health_view,
    critical_devices_ready,
)


def _health(*, camera_state=DeviceHealthState.FAILED):
    return (
        DeviceHealth("WHEEL_ENCODERS", DeviceHealthState.OK),
        DeviceHealth("BNO055_IMU", DeviceHealthState.OK),
        DeviceHealth("RPLIDAR_C1", DeviceHealthState.OK),
        DeviceHealth("CAMERA_FRONT", camera_state, "CAMERA_TEST"),
    )


def test_noncritical_camera_health_does_not_block_preflight_policy():
    health = _health()
    assert critical_devices_ready(health, PRODUCTION_CRITICAL_DEVICE_IDS)
    assert blocking_degraded_sources(
        ("CAMERA_FRONT",), PRODUCTION_CRITICAL_DEVICE_IDS
    ) == ()


def test_missing_critical_device_is_not_ready():
    health = _health()[:-2] + (_health()[-1],)
    view = critical_device_health_view(health, PRODUCTION_CRITICAL_DEVICE_IDS)
    assert "RPLIDAR_C1" in view.missing_device_ids
    assert not critical_devices_ready(health, PRODUCTION_CRITICAL_DEVICE_IDS)


def test_legacy_none_policy_preserves_all_device_semantics():
    health = _health()
    assert not critical_devices_ready(health, None)
    assert blocking_degraded_sources(("CAMERA_FRONT",), None) == ("CAMERA_FRONT",)
