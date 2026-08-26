import pytest

from driver.imu_factory import (
    imu_presence_from_i2c,
    imu_probe_targets,
    imu_provider_from_config,
)


def _config(provider="bno055"):
    return {
        "imu": {
            "provider": provider,
            "bno055": {"address": "0x28"},
        }
    }


def test_only_bno055_provider_is_accepted():
    assert imu_provider_from_config(_config()) == "bno055"
    for removed in ("legacy", "auto", ""):
        with pytest.raises(ValueError, match="unsupported_imu_provider"):
            imu_provider_from_config(_config(removed))


def test_presence_and_probe_ignore_removed_sensor_addresses():
    ok, provider, details = imu_presence_from_i2c(
        [0x53, 0x68, 0x0C],
        config=_config(),
    )
    assert ok is False
    assert provider == "bno055"
    assert details["bno055"] is False
    assert not any(key.startswith("legacy_") for key in details)
    assert imu_probe_targets(config=_config()) == [(0x28, 0x00)]


def test_explicit_removed_provider_fails_closed():
    with pytest.raises(ValueError, match="unsupported_imu_provider:legacy"):
        imu_presence_from_i2c([0x28], config=_config(), provider="legacy")
