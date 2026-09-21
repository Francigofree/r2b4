from __future__ import annotations

import inspect
from types import SimpleNamespace

from v3.adapters.process_lidar_port import _unwire_raw, _wire_control_raw, _wire_raw
from v3.adapters.process_vision_port import CameraFrameControlSnapshot, _wire_camera
from v3.adapters.picamera2_camera import CameraEdgeSnapshot, CameraRuntimeStatus
from v3.adapters.rplidar_c1 import RplidarPoint


def _raw_snapshot(point_count: int = 360):
    points = tuple(
        RplidarPoint(float(index % 360), 0.5 + (index % 20) * 0.01, 15)
        for index in range(point_count)
    )
    return SimpleNamespace(
        raw_scan_id=7,
        raw_scan_timestamp=1.1,
        scan_start_monotonic_ns=1_000_000_000,
        scan_end_monotonic_ns=1_100_000_000,
        measurement_monotonic_ns=1_050_000_000,
        health="OK",
        raw_scan=points,
        summary={"raw_safety_valid_point_count": point_count},
        observed_monotonic_ns=1_100_000_000,
    )


def test_lidar_control_wire_is_bounded_but_capture_wire_keeps_full_scan():
    raw = _raw_snapshot(360)
    compact = _wire_control_raw(
        raw,
        minimum_range_m=0.08,
        maximum_range_m=2.5,
        maximum_points=96,
    )
    full = _wire_raw(raw)
    assert compact is not None and full is not None
    assert len(compact[6]) <= 96
    assert len(full[6]) == 360
    restored = _unwire_raw(compact)
    assert restored is not None
    assert restored.raw_scan_id == 7
    assert len(restored.raw_scan) <= 96
    assert restored.summary["raw_safety_valid_point_count"] == 360


def test_vision_wire_contains_metadata_not_image_bytes():
    status = CameraRuntimeStatus(True, 4, 0, None, "imx708", 2_000_000)
    frame = SimpleNamespace(
        sequence=4,
        sensor_timestamp_ns=100,
        measurement_monotonic_ns=200,
        completed_monotonic_ns=300,
        exposure_time_ns=10,
        frame_duration_ns=50,
        width=640,
        height=360,
        pixel_format="RGB888",
        stride_bytes=1920,
        frame_size_bytes=691200,
        focus_state="focused",
        lens_position=1.0,
        image_bytes=b"x" * 16,
    )
    wire = _wire_camera(CameraEdgeSnapshot(status, frame))
    assert b"x" * 16 not in repr(wire).encode()
    assert "image_bytes" not in inspect.getsource(_wire_camera)


def test_camera_control_snapshot_has_no_raw_frame_payload():
    fields = CameraFrameControlSnapshot.__dataclass_fields__
    assert "image_bytes" not in fields
    assert "frame_size_bytes" in fields


def test_encoder_owner_imports_lgpio_only_in_child_entry():
    from v3.adapters import process_encoder_backend as module

    source = inspect.getsource(module)
    child = inspect.getsource(module._encoder_process_main)
    assert "import lgpio" in child
    assert "NativeGpioSignedCounterPair" in child
    # No module-level lgpio import: importing the control side must not open/own GPIO.
    prefix = source[: source.index("def _encoder_process_main")]
    assert "import lgpio" not in prefix
