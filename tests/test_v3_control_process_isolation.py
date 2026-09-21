from __future__ import annotations

import multiprocessing
import os
import threading
import time
from pathlib import Path

import pytest

import v3_process_runtime as process
from v3.adapters.native_lidar_port import TimedPoseReference
from v3.adapters.process_lidar_port import _SharedPoseHistory
from v3.adapters.process_lidar_port import ProcessLidarPort
from v3.process_sidecars import ProcessMcapCaptureSession, ProcessResidentStatusPublisher


def _raw_evidence_producer(target, times):
    from v3.adapters.process_lidar_port import _wire_raw
    from test_v3_mcap_e2e import raw
    for revision, monotonic_ns in enumerate(times, 1):
        target.put(("raw", _wire_raw(raw(revision, monotonic_ns, points=360))), timeout=2.0)
    target.close()
    target.join_thread()


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


def test_control_lidar_observer_never_receives_ipc_or_waits_for_pose_lock(monkeypatch):
    from collections import deque
    port = ProcessLidarPort.__new__(ProcessLidarPort)
    port._stopped = False
    port._pending_poses = deque(maxlen=64)
    port._raw_snapshot = object()
    port._matcher_result = object()
    port._collector_stop = threading.Event()
    port._fatal_error = ""
    entered, release = threading.Event(), threading.Event()

    def blocked_receive(self):
        entered.set()
        assert release.wait(2.0)

    monkeypatch.setattr(ProcessLidarPort, "_drain_state", blocked_receive)
    collector = threading.Thread(target=port._collect_state)
    collector.start()
    try:
        assert entered.wait(1.0)
        started = time.monotonic()
        assert port.get_raw_scan_snapshot() is port._raw_snapshot
        assert port.get_matcher_result() is port._matcher_result
        pose = TimedPoseReference(100, 0., 0., 0.)
        port.publish_pose_reference(pose)
        assert tuple(port._pending_poses) == (pose,)
        assert time.monotonic() - started < .05
    finally:
        port._collector_stop.set()
        release.set()
        collector.join(1.0)
    assert not collector.is_alive()


def test_real_sidecars_warm_feeders_off_control_cpu_and_drain_to_replay(tmp_path):
    from types import SimpleNamespace
    from v3.mcap_capture import McapCaptureConfig
    from v3.mcap_reader import McapReader
    from v3.mcap_replay_bridge import replay_mcap
    from test_v3_mcap_e2e import records

    cpus = sorted(os.sched_getaffinity(0))
    if len(cpus) < 2:
        pytest.skip("two allowed CPUs needed for isolation test")
    original = set(cpus)
    io_cpu, control_cpu = cpus[0], cpus[-1]
    config, values = records(6)
    capture = ProcessMcapCaptureSession(
        "isolated", tmp_path / "isolated.mcap", configuration={"resolved_control": config},
        config=McapCaptureConfig(mode="append_only"), project_root=Path(process.__file__).parent,
        worker_cpu=io_cpu, strict_affinity=True,
    )
    status = ProcessResidentStatusPublisher(
        process.ResidentStatusConfig(tmp_path / "status.json"), worker_cpu=io_cpu,
        strict_affinity=True,
    )
    try:
        os.sched_setaffinity(0, {control_cpu})
        capture.start()
        status.start()
        for feeder in (capture._data_queue._thread, status._tick_queue._thread):
            assert os.sched_getaffinity(feeder.native_id) == {io_cpu}
        producer = multiprocessing.get_context("spawn").Process(
            target=_raw_evidence_producer,
            args=(capture.raw_lidar_queue, tuple(r.inputs.context.monotonic_ns for r in values)),
        )
        producer.start()
        for record in values:
            capture.observe(record)
            status.publish_tick(record.result)
        producer.join(5.0)
        assert producer.exitcode == 0
        status.finish()
        path = capture.finalize(SimpleNamespace(status=0))
        assert capture.transport_drop_count == 0
        assert McapReader(path).capture_integrity()["captured_tick_count"] == 6
        raw_rows = [row for _, row in McapReader(path).iter_json_messages(topics=("/r2b4/raw_lidar",))]
        assert [row["revision"] for row in raw_rows] == list(range(1, 7))
        assert all(row["source_point_count"] == 360 for row in raw_rows)
        assert [row["measurement_monotonic_ns"] for row in raw_rows] == [
            record.inputs.context.monotonic_ns for record in values
        ]
        assert replay_mcap(path, project_root=Path(process.__file__).parent)["status"] == "MATCH"
    finally:
        os.sched_setaffinity(0, original)
        for sidecar in (status, capture):
            if sidecar._process.is_alive():
                sidecar._process.terminate()
                sidecar._process.join(2.0)


def test_status_ipc_contains_compact_projection_and_remains_bounded(tmp_path):
    import pickle
    import queue
    from test_v3_mcap_e2e import records
    from v3.resident_status import _tick_status

    _, values = records(1)
    result = values[0].result
    publisher = ProcessResidentStatusPublisher(process.ResidentStatusConfig(tmp_path / "status.json"))
    publisher._tick_queue.close()
    publisher._tick_queue = queue.Queue(maxsize=1)
    publisher._started = True
    publisher.publish_tick(result, True)
    publisher.publish_tick(result, False)
    assert publisher.drop_count == 1
    snapshot = publisher._tick_queue.get_nowait()
    assert snapshot == _tick_status(result, True)
    wire = pickle.dumps(snapshot)
    assert b"TickResult" not in wire
    assert b"DataField" not in wire
    assert b"CostmapCell" not in wire
    assert pickle.loads(wire)["tick_id"] == result.trace.context.tick_id
