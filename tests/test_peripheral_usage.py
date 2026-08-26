import json
from pathlib import Path

import pytest

from middleware.peripheral_usage import (
    ensure_peripheral_ssot,
    get_cached_peripherals,
    read_peripherals,
    set_peripheral_enabled,
)


def test_json_ssot_cannot_be_overridden_by_removed_text_mirror(tmp_path):
    (tmp_path / "peripherals_enabled.json").write_text(
        json.dumps({"camera": False, "lidar": True, "encoder": True}),
        encoding="utf-8",
    )
    (tmp_path / "camera_enabled.txt").write_text("1", encoding="utf-8")

    state = read_peripherals(runtime_dir=tmp_path, use_cache=False)

    assert state["camera"] is False


def test_removed_multi_chip_imu_flags_are_migrated_out(tmp_path):
    (tmp_path / "peripherals_enabled.json").write_text(
        json.dumps(
            {
                "camera": False,
                "lidar": True,
                "encoder": True,
                "imu": True,
                "accelerometer": False,
                "gyroscope": False,
                "magnetometer": False,
                "microphone": False,
            }
        ),
        encoding="utf-8",
    )

    state = read_peripherals(runtime_dir=tmp_path, use_cache=False)

    assert set(state) == {"camera", "lidar", "encoder", "imu", "microphone"}
    assert state["imu"] is True
    persisted = json.loads((tmp_path / "peripherals_enabled.json").read_text(encoding="utf-8"))
    assert persisted == state


def test_removed_imu_axis_cannot_be_reintroduced(tmp_path):
    ensure_peripheral_ssot(runtime_dir=tmp_path)

    with pytest.raises(ValueError, match="unsupported_peripheral:gyroscope"):
        set_peripheral_enabled("gyroscope", False, runtime_dir=tmp_path)


def test_peripheral_writes_only_canonical_json(tmp_path):
    ensure_peripheral_ssot(runtime_dir=tmp_path)
    state = set_peripheral_enabled("camera", True, runtime_dir=tmp_path)

    assert state["camera"] is True
    persisted = json.loads((tmp_path / "peripherals_enabled.json").read_text(encoding="utf-8"))
    assert persisted["camera"] is True
    assert not (tmp_path / "camera_enabled.txt").exists()
    assert not (tmp_path / "lidar_enabled.txt").exists()
    assert not (tmp_path / "encoder_enabled.txt").exists()


def test_cached_peripherals_tracks_latest_ssot_state(tmp_path):
    ensure_peripheral_ssot(runtime_dir=tmp_path)

    initial = get_cached_peripherals(runtime_dir=tmp_path)
    assert initial["camera"] is False

    set_peripheral_enabled("camera", True, runtime_dir=tmp_path)
    cached = get_cached_peripherals(runtime_dir=tmp_path)

    assert cached["camera"] is True
    assert cached["lidar"] is True


def test_cached_peripherals_status_path_fast_path_has_no_filesystem_io(tmp_path, monkeypatch):
    ensure_peripheral_ssot(runtime_dir=tmp_path)
    set_peripheral_enabled("camera", True, runtime_dir=tmp_path)

    def forbidden(*_args, **_kwargs):
        raise AssertionError("filesystem_call_forbidden_on_cache_only_path")

    monkeypatch.setattr(Path, "resolve", forbidden)
    monkeypatch.setattr(Path, "read_text", forbidden)

    cached = get_cached_peripherals(status_path=tmp_path / "status.json")

    assert cached["camera"] is True
    assert cached["encoder"] is True
