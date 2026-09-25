from __future__ import annotations

import inspect
import queue
import threading
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


def test_direct_and_compact_lidar_paths_close_identical_typed_samples():
    from v3.adapters.latest_lidar import LatestLidarBackendConfig, NativeLatestLidarBackend
    from v3.adapters.live_lidar import NativeLidarConfig, NativeLidarSource
    from v3.contracts import TickContext
    from test_v3_latest_lidar_backend import Port, RawSnapshot

    raw = RawSnapshot(raw_scan=_raw_snapshot().raw_scan)
    compact = _unwire_raw(_wire_control_raw(
        raw, minimum_range_m=.08, maximum_range_m=2.5, maximum_points=96,
    ))
    config = NativeLidarConfig(
        "RPLIDAR_C1", minimum_confidence=.3, maximum_measurement_age_ns=500_000_000,
        local_perception_min_range_m=.08, local_perception_max_range_m=2.5,
        local_perception_max_points=96,
    )
    sources = [NativeLidarSource(
        NativeLatestLidarBackend(Port(raw=snapshot), LatestLidarBackendConfig(500_000_000)),
        config,
    ) for snapshot in (raw, compact)]
    for context in (TickContext(7, 1_000_000_000), TickContext(8, 2_000_000_000)):
        assert sources[0].read(context) == sources[1].read(context)


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


def test_lidar_matcher_update_supersedes_scan_update_without_losing_scan(monkeypatch):
    import sys
    from v3.adapters import process_lidar_port as module

    raw0 = _raw_snapshot()
    raw1 = _raw_snapshot()
    raw1.raw_scan_id = 8
    raw1.raw_scan_timestamp = 1.2
    raw1.scan_start_monotonic_ns += 100_000_000
    raw1.scan_end_monotonic_ns += 100_000_000
    raw1.measurement_monotonic_ns += 100_000_000
    raw1.observed_monotonic_ns += 100_000_000
    scans = iter((raw0, raw1, raw1))
    matcher = SimpleNamespace(
        matcher_result_id=8, candidate_id=8, source_raw_scan_id=8,
        source_raw_scan_timestamp=raw1.raw_scan_timestamp,
        scan_start_monotonic_ns=raw1.scan_start_monotonic_ns,
        scan_end_monotonic_ns=raw1.scan_end_monotonic_ns,
        measurement_monotonic_ns=raw1.measurement_monotonic_ns,
        pose_reference_monotonic_ns=raw1.measurement_monotonic_ns,
        timestamp=1.3, summary={},
    )
    matches = iter((None, None, matcher))
    source = SimpleNamespace(
        get_raw_scan_snapshot=lambda: next(scans), get_matcher_result=lambda: next(matches),
        get_runtime_status=lambda: {"running": True}, stop=lambda: None,
    )
    class TwoIterations:
        count = 0
        def is_set(self):
            return self.count == 2
        def wait(self, timeout):
            self.count += 1

    monkeypatch.setitem(sys.modules, "serial", SimpleNamespace(Serial=object))
    monkeypatch.setattr(module, "open_native_lidar_port", lambda *args: source)
    states, raw_queue = queue.Queue(maxsize=2), queue.Queue(maxsize=2)
    module._lidar_owner_process_main(
        None, None, None, None, None, states, raw_queue,
        threading.Event(), TwoIterations(), None, False, 0.08, 2.5, 96,
    )
    port = module.ProcessLidarPort.__new__(module.ProcessLidarPort)
    port._raw_snapshot = raw0
    port._state_queue = states
    port._process = SimpleNamespace(is_alive=lambda: True)
    port._fatal_error = ""
    port._stopped = False
    port._drain_state()
    assert port._raw_snapshot.raw_scan_id == 8
    assert port._raw_snapshot.scan_end_monotonic_ns == raw1.scan_end_monotonic_ns
    assert port._raw_snapshot.measurement_monotonic_ns == raw1.measurement_monotonic_ns
    assert len(port._raw_snapshot.raw_scan) <= 96
    assert port._matcher_result.source_raw_scan_id == 8
    # Repeated heartbeat does not rebuild even the bounded geometry in control.
    snapshot = port._raw_snapshot
    states.put(("state", module._wire_control_raw(raw1, minimum_range_m=.08,
                 maximum_range_m=2.5, maximum_points=96), module._wire_matcher(matcher), {}))
    port._drain_state()
    assert port._raw_snapshot is snapshot


def test_lidar_transport_error_is_not_hidden_by_a_later_state():
    from v3.adapters.process_lidar_port import ProcessLidarPort
    port = ProcessLidarPort.__new__(ProcessLidarPort)
    port._raw_snapshot = None
    port._state_queue = queue.Queue(maxsize=2)
    port._state_queue.put(("error", "OSError", "disconnected"))
    port._state_queue.put(("state", None, None, {"running": True}))
    port._process = SimpleNamespace(is_alive=lambda: True)
    port._fatal_error = ""
    port._stopped = False
    port._capture_raw_revision = 0
    port._drain_state()
    assert port.get_runtime_status()["running"] is False
    assert port.get_runtime_status()["fatal_error"] == "OSError:disconnected"


def test_external_lidar_evidence_queue_is_never_read_by_control():
    from v3.adapters.process_lidar_port import ProcessLidarPort
    port = ProcessLidarPort.__new__(ProcessLidarPort)
    port._external_raw_queue = True
    # No queue exists on this proxy: even attempting to drain would fail.
    assert port.get_capture_raw_scan_snapshot() is None


def test_lidar_owner_death_is_fail_closed_even_with_a_cached_scan():
    from v3.adapters.process_lidar_port import ProcessLidarPort
    port = ProcessLidarPort.__new__(ProcessLidarPort)
    port._state_queue = queue.Queue(maxsize=2)
    port._process = SimpleNamespace(is_alive=lambda: False)
    port._fatal_error = ""
    port._stopped = False
    port._status = {"running": True}
    port._capture_raw_revision = 0
    port._raw_snapshot = _raw_snapshot()
    port._drain_state()
    status = port.get_runtime_status()
    assert status["running"] is False
    assert status["driver_connected"] is False
    assert status["health"] == "ERROR"
    assert status["fatal_error"] == "LIDAR_OWNER_PROCESS_EXITED"


def test_lidar_shutdown_keeps_collector_alive_until_producer_has_flushed():
    from v3.adapters.process_lidar_port import ProcessLidarPort
    port = ProcessLidarPort.__new__(ProcessLidarPort)
    port._stopped = False
    port._stop_event = threading.Event()
    port._collector_stop = threading.Event()
    port._collector = None
    port._external_raw_queue = True
    closed = []
    port._state_queue = SimpleNamespace(close=lambda: closed.append("state"))

    def join(timeout):
        assert port._stop_event.is_set()
        assert not port._collector_stop.is_set()

    port._process = SimpleNamespace(join=join, is_alive=lambda: False)
    port.stop()
    assert port._collector_stop.is_set()
    assert closed == ["state"]
