from __future__ import annotations

import multiprocessing
from pathlib import Path

import v3_process_runtime as process
from v3.adapters.native_lidar_port import TimedPoseReference
from v3.adapters.process_lidar_port import _SharedPoseHistory
from v3.process_sidecars import ProcessMcapCaptureSession, ProcessResidentStatusPublisher


def test_shared_pose_history_preserves_exact_and_interpolated_scan_time_pose():
    context = multiprocessing.get_context("spawn")
    lock = context.Lock()
    sequence = context.RawValue("Q", 0)
    times = context.RawArray("q", 4)
    values = context.RawArray("d", 12)
    history = _SharedPoseHistory(4, lock, sequence, times, values)
    history.publish(TimedPoseReference(100, 0.0, 0.0, 0.0))
    history.publish(TimedPoseReference(200, 2.0, 4.0, 0.4))

    exact = history.lookup(200)
    assert exact == TimedPoseReference(200, 2.0, 4.0, 0.4)
    middle = history.lookup(150)
    assert middle is not None
    assert middle.monotonic_ns == 150
    assert middle.x_m == 1.0
    assert middle.y_m == 2.0
    assert abs(middle.yaw_rad - 0.2) < 1e-12
    assert history.lookup(99) is None
    assert history.lookup(201) is None


def test_production_main_is_wired_to_process_isolated_sidecars_and_lidar():
    source = Path(process.__file__).read_text(encoding="utf-8")
    assert "ProcessResidentStatusPublisher(" in source
    assert "ProcessMcapCaptureSession(" in source
    assert "open_lidar = process_lidar_factory(" in source
    assert "import serial\n" not in source[source.index("def main(") :]


def test_process_sidecar_contracts_are_passive_surfaces():
    assert callable(ProcessMcapCaptureSession.observe)
    assert callable(ProcessMcapCaptureSession.observe_raw_lidar)
    assert callable(ProcessMcapCaptureSession.trigger)
    assert callable(ProcessResidentStatusPublisher.publish_tick)
    assert not hasattr(ProcessMcapCaptureSession, "command")
    assert not hasattr(ProcessResidentStatusPublisher, "command")
