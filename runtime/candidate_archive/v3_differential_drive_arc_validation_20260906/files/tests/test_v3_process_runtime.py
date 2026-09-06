import json
import signal
import threading
import time
from pathlib import Path
from unittest import mock

import pytest

import v3_process_runtime as process
from v3.adapters.fake_edges import FakeCommandGateway, FakeHal
from v3.adapters.resident_command import (
    AtomicResidentCommandGateway,
    ResidentCommandMailboxConfig,
)
from v3.composition.stop_only import StopOnlyComposition
from v3.capture import CaptureSink, inspect_capture
from v3.contracts import LifecycleState, SafetyDecision
from v3.execution import ExecutionBoundary, IterableInputSource, MemoryOutputSink
from v3.composition.native_control import NativeControlComposition
from v3.test_hub import validate_run, verify_evidence
from v3_bounded_runtime import RUN_OK
from v3_runtime import ResidentRuntimeReport
from v3_validation_helpers import (
    RecordingMotorSink,
    configuration_documents,
    control_config,
    tick_inputs,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _tick_result():
    hal = FakeHal()
    runtime = StopOnlyComposition(hal, FakeCommandGateway(), hal)
    runtime.enter_idle()
    return runtime.tick(1_000_000_000)


def _report():
    return ResidentRuntimeReport(
        status=RUN_OK,
        exit_reason="STOP_REQUESTED",
        tick_count=2,
        normal_tick_count=1,
        last_tick_id=1,
        final_lifecycle=LifecycleState.SHUTDOWN,
        final_safety_decision=SafetyDecision.STOP,
        final_reason="NOT_ACTIVE",
        fault_layer=None,
        operator_stopped=True,
    )


def _execution_record():
    output = MemoryOutputSink()
    ExecutionBoundary(
        NativeControlComposition(RecordingMotorSink(), control_config())
    ).run(IterableInputSource(tick_inputs(1)), output)
    return output.records[0]


def test_process_uses_one_canonical_hardware_runner_and_publishes_final_status(tmp_path):
    command = tmp_path / "command.json"
    status = tmp_path / "status.json"
    gateway = AtomicResidentCommandGateway(
        ResidentCommandMailboxConfig(command)
    )
    publisher = process.AsyncResidentStatusPublisher(
        process.ResidentStatusConfig(status)
    )
    runtime_config = process.load_resident_runtime_config(PROJECT_ROOT)
    calls = []

    def fake_run(*args, **kwargs):
        calls.append((args, kwargs))
        assert args[3] is gateway
        assert args[5] is runtime_config
        assert kwargs["approval"] == "native-resident-v3"
        assert kwargs["stop_requested"]() is False
        kwargs["tick_observer"](_tick_result())
        return _report()

    report = process.run_v3_resident_process(
        object(),
        lambda _bus: object(),
        lambda _pose: object(),
        object(),
        gateway,
        runtime_config,
        publisher,
        approval="native-resident-v3",
        stop_requested=lambda: False,
        run_hardware=fake_run,
    )

    assert report == _report()
    assert len(calls) == 1
    payload = json.loads(status.read_text(encoding="utf-8"))
    assert payload["schema"] == process.RESIDENT_PROCESS_STATUS_SCHEMA
    assert payload["state"] == "STOPPED"
    assert payload["report"] == report.as_dict()
    assert status.stat().st_mode & 0o777 == 0o600


def test_tick_status_publication_is_latest_only_and_does_not_wait_for_disk(tmp_path):
    status = tmp_path / "status.json"
    publisher = process.AsyncResidentStatusPublisher(
        process.ResidentStatusConfig(status)
    )
    writer_entered = threading.Event()
    writer_release = threading.Event()
    real_writer = process._atomic_private_json
    calls = 0

    def blocked_writer(path, payload, mode):
        nonlocal calls
        calls += 1
        if calls > 1:
            writer_entered.set()
            assert writer_release.wait(timeout=2.0)
        real_writer(path, payload, mode)

    with mock.patch.object(process, "_atomic_private_json", side_effect=blocked_writer):
        publisher.start()
        started = time.monotonic()
        publisher.publish_tick(_tick_result())
        elapsed = time.monotonic() - started
        assert writer_entered.wait(timeout=1.0)
        publisher.publish_tick(_tick_result())
        assert elapsed < 0.05
        writer_release.set()
        publisher.finish(report=_report())

    payload = json.loads(status.read_text(encoding="utf-8"))
    assert payload["state"] == "STOPPED"


def test_status_failure_requests_shutdown_and_is_reported_after_hardware_close(tmp_path):
    command = tmp_path / "command.json"
    status = tmp_path / "status.json"
    gateway = AtomicResidentCommandGateway(
        ResidentCommandMailboxConfig(command)
    )
    publisher = process.AsyncResidentStatusPublisher(
        process.ResidentStatusConfig(status)
    )
    runtime_config = process.load_resident_runtime_config(PROJECT_ROOT)
    real_writer = process._atomic_private_json
    calls = 0

    def failing_writer(path, payload, mode):
        nonlocal calls
        calls += 1
        if calls > 1:
            raise OSError("injected status failure")
        real_writer(path, payload, mode)

    def fake_run(*_args, **kwargs):
        kwargs["tick_observer"](_tick_result())
        deadline = time.monotonic() + 1.0
        while not kwargs["stop_requested"]() and time.monotonic() < deadline:
            time.sleep(0.001)
        assert kwargs["stop_requested"]() is True
        return _report()

    with mock.patch.object(process, "_atomic_private_json", side_effect=failing_writer):
        with pytest.raises(RuntimeError, match="status publication failed"):
            process.run_v3_resident_process(
                object(),
                lambda _bus: object(),
                lambda _pose: object(),
                object(),
                gateway,
                runtime_config,
                publisher,
                approval="native-resident-v3",
                stop_requested=lambda: False,
                run_hardware=fake_run,
            )


def test_process_passively_finalizes_existing_v3_capture_sink(tmp_path):
    command = tmp_path / "command.json"
    status = tmp_path / "status.json"
    capture = tmp_path / "capture.json"
    gateway = AtomicResidentCommandGateway(ResidentCommandMailboxConfig(command))
    publisher = process.AsyncResidentStatusPublisher(process.ResidentStatusConfig(status))
    session = process._PassiveCaptureSession(
        CaptureSink("resident-process", configuration=configuration_documents()),
        capture,
    )
    record = _execution_record()

    def fake_run(*_args, **kwargs):
        kwargs["record_observer"](record)
        kwargs["tick_observer"](record.result)
        return _report()

    process.run_v3_resident_process(
        object(),
        lambda _bus: object(),
        lambda _pose: object(),
        object(),
        gateway,
        process.load_resident_runtime_config(PROJECT_ROOT),
        publisher,
        approval="native-resident-v3",
        stop_requested=lambda: False,
        capture_session=session,
        run_hardware=fake_run,
    )

    assert inspect_capture(capture)["tick_count"] == 1
    assert capture.stat().st_mode & 0o777 == 0o600
    evidence = validate_run(capture, tmp_path / "evidence", project_root=PROJECT_ROOT)
    assert evidence["replay_status"] == "MATCH"
    assert verify_evidence(evidence["evidence_index"])["status"] == "PASS"


def test_capture_failure_is_reported_only_after_hardware_returns_safe(tmp_path):
    command = tmp_path / "command.json"
    status = tmp_path / "status.json"
    capture = tmp_path / "capture.json"
    gateway = AtomicResidentCommandGateway(ResidentCommandMailboxConfig(command))
    publisher = process.AsyncResidentStatusPublisher(process.ResidentStatusConfig(status))
    session = process._PassiveCaptureSession(
        CaptureSink("resident-process", configuration=configuration_documents()),
        capture,
    )
    record = _execution_record()
    hardware_returned = False

    def fake_run(*_args, **kwargs):
        nonlocal hardware_returned
        kwargs["record_observer"](record)
        assert kwargs["stop_requested"]() is False
        hardware_returned = True
        return _report()

    with mock.patch.object(CaptureSink, "write", side_effect=OSError("capture failed")):
        with pytest.raises(RuntimeError, match="production V3 capture failed"):
            process.run_v3_resident_process(
                object(),
                lambda _bus: object(),
                lambda _pose: object(),
                object(),
                gateway,
                process.load_resident_runtime_config(PROJECT_ROOT),
                publisher,
                approval="native-resident-v3",
                stop_requested=lambda: False,
                capture_session=session,
                run_hardware=fake_run,
            )

    assert hardware_returned is True
    assert json.loads(status.read_text())["state"] == "STOPPED"


def test_signal_latch_and_cli_approval_fail_before_hardware_import(capsys):
    stop = process.SignalStop()
    stop.handle(signal.SIGTERM, None)
    assert stop() is True
    assert stop.signum == signal.SIGTERM

    assert process.main(["--approval", "wrong"]) == 2
    payload = json.loads(capsys.readouterr().out)
    assert payload["status"] == "ERROR"
    assert "approval" in payload["error"]
    assert process._tick_status(_tick_result())["ready_for_active"] is False


def test_process_paths_cannot_escape_runtime_and_config_closes_native_sensors():
    config = process.load_resident_runtime_config(PROJECT_ROOT)
    assert config.composition.live_control.required_lidar_preflight_revisions == 3
    assert config.sensor_inputs.inputs.lidar_source.pose_frame_id == "R2B4_BOOT_ROBOT_MAP"
    assert process._runtime_owned_path(
        "runtime/v3_command.json",
        PROJECT_ROOT,
    ) == PROJECT_ROOT / "runtime" / "v3_command.json"
    with pytest.raises(ValueError, match="runtime directory"):
        process._runtime_owned_path("/tmp/v3-command.json", PROJECT_ROOT)


def test_process_native_lidar_factory_closes_config_once_without_legacy_service(monkeypatch):
    runtime = process.load_resident_runtime_config(PROJECT_ROOT)
    assert runtime.sensor_inputs is not None
    config_token = object()
    port_token = object()
    loads = []
    opens = []

    def fake_load(hardware_path, control_path, *, danger_zone_m):
        loads.append((hardware_path, control_path, danger_zone_m))
        return config_token

    def fake_open(config, pose_provider, serial_factory):
        opens.append((config, pose_provider, serial_factory))
        return port_token

    monkeypatch.setattr(process, "load_native_lidar_port_config", fake_load)
    monkeypatch.setattr(process, "open_native_lidar_port", fake_open)
    serial_factory = mock.Mock()
    factory = process.native_lidar_factory(
        runtime.sensor_inputs,
        serial_factory,
        PROJECT_ROOT,
    )
    pose_provider = lambda: (0.0, 0.0, 0.0)

    assert factory(pose_provider) is port_token
    assert loads == [
        (
            PROJECT_ROOT / "conf" / "hardver.json",
            PROJECT_ROOT / "conf" / "vezerles.json",
                0.4,
        )
    ]
    assert opens == [(config_token, pose_provider, serial_factory)]
    assert "sensors.lidar_service" not in Path(process.__file__).read_text(
        encoding="utf-8"
    )
