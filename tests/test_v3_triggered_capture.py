import json
import threading
import time
from dataclasses import replace
from pathlib import Path

from v3.adapters.native_lidar_port import NativeRawLidarSnapshot
from v3.adapters.rplidar_c1 import RplidarPoint
from v3.capture import (
    CaptureSink,
    CaptureWindowConfig,
    TriggeredCaptureWorker,
    load_capture,
)
from v3.composition.native_control import NativeControlComposition
from v3.contracts import DataField, DeviceSample, LifecycleState, TickContext
from v3.execution import EdgeFaultRecord, ExecutionBoundary, IterableInputSource, MemoryOutputSink
from v3.layers.l2_admission import AdmissionConfig
from v3.replay import replay_capture
from v3_validation_helpers import RecordingMotorSink, control_config, tick_inputs


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _records(count=6, config=None, inputs=None):
    settings = config or control_config()
    output = MemoryOutputSink()
    writer = RecordingMotorSink()
    ExecutionBoundary(NativeControlComposition(writer, settings)).run(
        IterableInputSource(inputs or tick_inputs(count)),
        output,
    )
    return output.records, writer


def _worker(tmp_path, *, config=None, capture_config=None):
    path = tmp_path / "triggered.json"
    worker = TriggeredCaptureWorker(
        CaptureSink(
            "triggered",
            configuration={"resolved_control": config or control_config()},
        ),
        path,
        capture_config,
    )
    worker.start()
    return worker, path


def test_manual_trigger_selects_exact_pre_post_tick_window(tmp_path):
    records, _writer = _records()
    worker, path = _worker(
        tmp_path,
        capture_config=CaptureWindowConfig(
            pre_event_ns=40_000_000,
            post_event_ns=40_000_000,
            max_tick_count=16,
        ),
    )

    for record in records[:4]:
        worker.observe(record)
    worker.trigger("MANUAL", records[3].inputs.context.monotonic_ns)
    assert not path.exists()
    for record in records[4:]:
        worker.observe(record)
    worker.finish("PASS", terminal=False)

    payload = load_capture(path)
    assert [tick["tick_id"] for tick in payload["ticks"]] == [1, 2, 3, 4, 5]
    assert payload["capture_window"]["post_window_complete"] is True
    assert payload["capture_window"]["terminal_short_post_window"] is False
    assert payload["capture_integrity"]["complete"] is True


def test_terminal_fault_auto_triggers_and_marks_short_post_window(tmp_path):
    config = control_config()
    composition = NativeControlComposition(RecordingMotorSink(), config)
    context = TickContext(0, 1_000_000_000)
    result = composition.run_fault_tick(
        context,
        LifecycleState.FAULT,
        "L0_ERROR",
        "L0",
    )
    worker, path = _worker(
        tmp_path,
        config=config,
        capture_config=CaptureWindowConfig(
            pre_event_ns=20_000_000,
            post_event_ns=100_000_000,
        ),
    )

    worker.observe(
        EdgeFaultRecord(
            context,
            LifecycleState.FAULT,
            "L0_ERROR",
            "L0",
            (),
            result,
        )
    )
    worker.finish("FAULT")

    payload = load_capture(path)
    assert payload["ticks"][0]["record_type"] == "edge_fault_tick"
    assert payload["capture_window"]["trigger_reason"] == "L0_ERROR"
    assert payload["capture_window"]["post_window_complete"] is False
    assert payload["capture_window"]["terminal_short_post_window"] is True
    assert replay_capture(path, project_root=PROJECT_ROOT)["status"] in {
        "MATCH",
        "MISMATCH",
    }


def test_blocked_encoder_never_blocks_production_observer(tmp_path, monkeypatch):
    records, _writer = _records(3)
    worker, _path = _worker(tmp_path)
    entered = threading.Event()
    release = threading.Event()
    from v3 import capture as capture_module

    real_encode = capture_module.encode_capture_record

    def blocked_encode(record):
        entered.set()
        assert release.wait(timeout=2.0)
        return real_encode(record)

    monkeypatch.setattr(capture_module, "encode_capture_record", blocked_encode)
    worker.observe(records[0])
    assert entered.wait(timeout=1.0)
    started = time.monotonic()
    worker.observe(records[1])
    elapsed = time.monotonic() - started
    release.set()
    worker.trigger("MANUAL", records[0].inputs.context.monotonic_ns)
    worker.finish("PASS")

    assert elapsed < 0.05


def test_queue_overflow_preserves_motor_output_and_invalidates_match(tmp_path, monkeypatch):
    records, writer = _records(4)
    worker, path = _worker(
        tmp_path,
        capture_config=CaptureWindowConfig(
            pre_event_ns=1_000_000_000,
            post_event_ns=0,
            ingress_queue_capacity=1,
            max_tick_count=16,
        ),
    )
    entered = threading.Event()
    release = threading.Event()
    from v3 import capture as capture_module

    real_encode = capture_module.encode_capture_record
    calls = 0

    def blocked_first(record):
        nonlocal calls
        calls += 1
        if calls == 1:
            entered.set()
            assert release.wait(timeout=2.0)
        return real_encode(record)

    monkeypatch.setattr(capture_module, "encode_capture_record", blocked_first)
    worker.observe(records[0])
    assert entered.wait(timeout=1.0)
    worker.observe(records[1])
    worker.observe(records[2])
    worker.trigger("MANUAL", records[1].inputs.context.monotonic_ns)
    release.set()
    worker.finish("PASS")

    payload = load_capture(path)
    assert len(writer.commands) == 4
    assert payload["capture_integrity"]["dropped_ingress_count"] > 0
    assert payload["capture_integrity"]["replay_match_eligible"] is False
    assert replay_capture(path, project_root=PROJECT_ROOT)["status"] == "MISMATCH"


def _raw_snapshot(revision):
    return NativeRawLidarSnapshot(
        raw_scan_id=revision,
        raw_scan_timestamp=1.0 + revision * 0.02,
        health="OK",
        raw_scan=(
            RplidarPoint(0.0, 1.0, 20),
            RplidarPoint(90.0, 1.5, 21),
        ),
        summary={"revision": revision},
    )


def test_raw_lidar_is_persisted_once_and_only_when_selected_ticks_reference_it(tmp_path):
    inputs = list(tick_inputs(4))
    matcher = DeviceSample(
        "RPLIDAR_C1",
        "lidar_matcher_diagnostics",
        200,
        inputs[2].context.monotonic_ns,
        (
            DataField("candidate_id", 200),
            DataField("source_raw_scan_id", 99),
        ),
    )
    inputs[2] = replace(
        inputs[2],
        raw_devices=replace(
            inputs[2].raw_devices,
            samples=inputs[2].raw_devices.samples + (matcher,),
        ),
    )
    records, _writer = _records(4, inputs=tuple(inputs))
    worker, path = _worker(
        tmp_path,
        capture_config=CaptureWindowConfig(
            pre_event_ns=20_000_000,
            post_event_ns=0,
            max_raw_lidar_scans=16,
            max_raw_lidar_points_per_scan=1,
        ),
    )
    for revision in (1, 2, 2, 3, 4, 77, 99):
        worker.observe_raw_lidar(_raw_snapshot(revision))
    for record in records[:3]:
        worker.observe(record)
    worker.trigger("MANUAL", records[2].inputs.context.monotonic_ns)
    assert not path.exists()
    worker.finish("PASS")

    payload = load_capture(path)
    revisions = [scan["revision"] for scan in payload["raw_lidar_scans"]]
    assert revisions == [1, 2, 3, 99]
    assert all(scan["points_truncated"] is True for scan in payload["raw_lidar_scans"])
    assert all(len(scan["points"]) == 1 for scan in payload["raw_lidar_scans"])
    assert payload["raw_lidar_evidence"]["missing_revisions"] == []


def test_missing_referenced_raw_lidar_is_explicit_and_not_proven(tmp_path):
    records, _writer = _records(3)
    worker, path = _worker(
        tmp_path,
        capture_config=CaptureWindowConfig(
            pre_event_ns=40_000_000,
            post_event_ns=0,
            max_raw_lidar_scans=1,
        ),
    )
    worker.observe_raw_lidar(_raw_snapshot(1))
    worker.observe_raw_lidar(_raw_snapshot(2))
    for record in records:
        worker.observe(record)
    worker.trigger("MANUAL", records[-1].inputs.context.monotonic_ns)
    worker.finish("PASS")

    payload = load_capture(path)
    assert payload["raw_lidar_evidence"]["missing_revisions"] == [1, 3]
    assert payload["raw_lidar_evidence"]["physical_diagnosis"] == "NOT_PROVEN"


def test_complete_nondefault_resolved_config_round_trips_without_legacy_authority(tmp_path):
    base = control_config()
    config = replace(
        base,
        admission=AdmissionConfig(max_sample_age_ns=333_000_000, max_future_skew_ns=7),
        estimation=replace(
            base.estimation,
            max_dt_ns=222_000_000,
            velocity_nis_max=12.5,
            yaw_nis_max=19.5,
        ),
        world_model=replace(base.world_model, max_track_age_ns=610_000_000),
        mission=replace(
            base.mission,
            default_constraints=replace(
                base.mission.default_constraints,
                max_v_mps=0.31,
            ),
        ),
        navigation=replace(base.navigation, rollout_step_count=7),
        motion_realization=replace(base.motion_realization, distance_gain=1.2),
        operational_constraints=replace(
            base.operational_constraints,
            max_acceleration_mps2=0.51,
        ),
        wheel_pi=replace(base.wheel_pi, kp=0.33, ki=0.07),
        lidar_safety=replace(base.lidar_safety, maximum_sample_age_ns=321_000_000),
    )
    records, _writer = _records(4, config)
    path = tmp_path / "config.json"
    sink = CaptureSink(
        "config",
        configuration={
            "resolved_control": config,
            "legacy_documents": {
                "physics": {},
                "speed_map": {},
                "hardware": {},
                "control": {},
            },
        },
    )
    for record in records:
        sink.write(record)
    sink.finalize("PASS", path)

    assert replay_capture(path, project_root=PROJECT_ROOT)["status"] == "MATCH"
    encoded = json.loads(path.read_text())["configuration"]["resolved_control"]
    assert encoded["admission"]["max_sample_age_ns"] == 333_000_000
    assert encoded["wheel_pi"]["kp"] == 0.33
