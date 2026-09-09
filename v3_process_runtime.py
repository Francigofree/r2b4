"""Production process boundary for the native resident V3 hardware runtime."""

from __future__ import annotations

import argparse
import json
import os
import signal
import stat
import threading
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from v3.adapters.bounded_command import BoundedTeleopProfile
from v3.adapters.resident_command import (
    AtomicResidentCommandGateway,
    ResidentCommandMailboxConfig,
)
from v3.capture import CaptureSink, CaptureWindowConfig, TriggeredCaptureWorker
from v3.adapters.native_lidar_port import (
    load_native_lidar_port_config,
    open_native_lidar_port,
)
from v3.contracts import AcquisitionFrame, RobotEstimate
from v3.composition.native_sensor_inputs import NativeSensorHardwareConfig
from v3.engine import TickResult
from v3.execution import CaptureRecord
from v3_bounded_config import (
    NativeSensorPolicyConfig,
    load_bounded_physical_runtime_config,
)
from v3_hardware_runtime import (
    RESIDENT_PHYSICAL_RUN_APPROVAL,
    ResidentRuntimeReport,
    run_native_hardware_resident_control,
)
from v3_runtime import ResidentPhysicalRuntimeConfig


PROJECT_ROOT = Path(__file__).resolve().parent
RESIDENT_PROCESS_STATUS_SCHEMA = "R2B4_V3_RESIDENT_PROCESS_STATUS_V1"


@dataclass(frozen=True, slots=True)
class ResidentStatusConfig:
    """One private atomic status target outside the motor-control thread."""

    path: Path
    file_mode: int = 0o600

    def __post_init__(self) -> None:
        path = Path(self.path)
        if not path.is_absolute():
            raise ValueError("status path must be absolute")
        object.__setattr__(self, "path", path)
        if (
            not isinstance(self.file_mode, int)
            or isinstance(self.file_mode, bool)
            or self.file_mode != 0o600
        ):
            raise ValueError("status file_mode must be 0o600")


class SignalStop:
    """Minimal SIGINT/SIGTERM latch consumed by the resident owner loop."""

    __slots__ = ("requested", "signum")

    def __init__(self) -> None:
        self.requested = False
        self.signum: int | None = None

    def handle(self, signum: int, _frame: Any) -> None:
        self.signum = int(signum)
        self.requested = True

    def __call__(self) -> bool:
        return self.requested


class SignalCaptureTrigger:
    """Passive SIGUSR1 latch; it grants no command or runtime authority."""

    __slots__ = ("requested",)

    def __init__(self) -> None:
        self.requested = False

    def handle(self, _signum: int, _frame: Any) -> None:
        self.requested = True

    def consume(self) -> bool:
        value = self.requested
        self.requested = False
        return value


class _PassiveCaptureSession:
    """Compatibility wrapper for explicit whole-session capture in older callers."""

    __slots__ = ("_triggered", "_worker")

    def __init__(self, sink: CaptureSink, output_path: Path) -> None:
        if not isinstance(sink, CaptureSink):
            raise TypeError("sink must be CaptureSink")
        self._worker = TriggeredCaptureWorker(
            sink,
            output_path,
            CaptureWindowConfig(
                pre_event_ns=0,
                post_event_ns=2**63 - 1,
            ),
        )
        self._triggered = False
        self._worker.start()

    @property
    def failed(self) -> bool:
        return self._worker.failed

    def start(self) -> None:
        self._worker.start()

    def observe(self, record: CaptureRecord) -> None:
        if not self._triggered:
            context = record.inputs.context if hasattr(record, "inputs") and record.inputs is not None else record.context
            self._worker.trigger("EXPLICIT_WHOLE_SESSION", context.monotonic_ns)
            self._triggered = True
        self._worker.observe(record)

    def observe_raw_lidar(self, snapshot: object | None) -> None:
        self._worker.observe_raw_lidar(snapshot)

    def trigger(self, reason: str = "MANUAL", monotonic_ns: int | None = None) -> None:
        self._worker.trigger(reason, monotonic_ns)

    def finalize(
        self,
        report: ResidentRuntimeReport | None,
        *,
        error: BaseException | None = None,
    ) -> Path | None:
        status = "PASS" if error is None and report is not None and report.status == 0 else "FAULT"
        return self._worker.finish(status, terminal=True)


class TriggeredCaptureSession:
    """Default process capture: bounded pre/post ring with fault/manual trigger."""

    __slots__ = ("_worker",)

    def __init__(
        self,
        sink: CaptureSink,
        output_path: Path,
        config: CaptureWindowConfig | None = None,
    ) -> None:
        self._worker = TriggeredCaptureWorker(sink, output_path, config)

    @property
    def failed(self) -> bool:
        return self._worker.failed

    def start(self) -> None:
        self._worker.start()

    def observe(self, record: CaptureRecord) -> None:
        self._worker.observe(record)

    def observe_raw_lidar(self, snapshot: object | None) -> None:
        self._worker.observe_raw_lidar(snapshot)

    def trigger(self, reason: str = "MANUAL", monotonic_ns: int | None = None) -> None:
        self._worker.trigger(reason, monotonic_ns)

    def finalize(
        self,
        report: ResidentRuntimeReport | None,
        *,
        error: BaseException | None = None,
    ) -> Path | None:
        status = "PASS" if error is None and report is not None and report.status == 0 else "FAULT"
        return self._worker.finish(status, terminal=True)


def _atomic_private_json(path: Path, payload: Mapping[str, object], mode: int) -> None:
    parent = path.parent
    parent.mkdir(parents=True, exist_ok=True)
    if parent.is_symlink() or not parent.is_dir():
        raise ValueError("status parent must be a regular directory")
    rendered = (json.dumps(dict(payload), sort_keys=True) + "\n").encode("utf-8")
    temporary = parent / (
        f".{path.name}.tmp.{os.getpid()}.{threading.get_ident()}.{time.monotonic_ns()}"
    )
    descriptor: int | None = None
    try:
        descriptor = os.open(
            temporary,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_CLOEXEC", 0),
            mode,
        )
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode):
            raise ValueError("temporary status target must be a regular file")
        view = memoryview(rendered)
        while view:
            written = os.write(descriptor, view)
            if written <= 0:
                raise OSError("status write made no progress")
            view = view[written:]
        os.fsync(descriptor)
        os.close(descriptor)
        descriptor = None
        os.replace(temporary, path)
    finally:
        if descriptor is not None:
            os.close(descriptor)
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def _layer(result: TickResult, name: str) -> object | None:
    for record in result.trace.layers:
        if record.layer == name:
            return record.output
    return None


def _tick_status(result: TickResult) -> dict[str, object]:
    acquisition = _layer(result, "L1")
    estimate = _layer(result, "L3")
    health: list[dict[str, object]] = []
    if isinstance(acquisition, AcquisitionFrame):
        health = [
            {
                "device_id": item.device_id,
                "state": item.state.value,
                "reason": item.reason,
            }
            for item in acquisition.io_health
        ]
    estimate_payload: dict[str, object] | None = None
    if isinstance(estimate, RobotEstimate):
        estimate_payload = {
            "frame_id": estimate.frame_id,
            "x_m": estimate.x_m,
            "y_m": estimate.y_m,
            "yaw_rad": estimate.yaw_rad,
            "v_mps": estimate.v_mps,
            "omega_rad_s": estimate.omega_rad_s,
        }
    final = result.final_actuation
    ready_for_active = bool(
        result.trace.fault_layer is None
        and final.safety_decision.value == "STOP"
        and final.reason == "NOT_ACTIVE"
        and not final.enabled
        and final.left_output == 0.0
        and final.right_output == 0.0
        and len(health) == 3
        and len({str(item["device_id"]) for item in health}) == 3
        and all(item["state"] == "OK" for item in health)
    )
    return {
        "schema": RESIDENT_PROCESS_STATUS_SCHEMA,
        "state": "RUNNING",
        "tick_id": result.trace.context.tick_id,
        "monotonic_ns": result.trace.context.monotonic_ns,
        "fault_layer": result.trace.fault_layer,
        "safety_decision": final.safety_decision.value,
        "safety_reason": final.reason,
        "enabled": final.enabled,
        "left_output": final.left_output,
        "right_output": final.right_output,
        "ready_for_active": ready_for_active,
        "source_health": health,
        "estimate": estimate_payload,
    }


class AsyncResidentStatusPublisher:
    """Latest-only background writer; tick publication never performs file I/O."""

    __slots__ = (
        "_condition",
        "_config",
        "_error",
        "_finished",
        "_pending",
        "_thread",
    )

    def __init__(self, config: ResidentStatusConfig) -> None:
        if not isinstance(config, ResidentStatusConfig):
            raise TypeError("config must be ResidentStatusConfig")
        self._config = config
        self._condition = threading.Condition()
        self._pending: dict[str, object] | None = None
        self._error: BaseException | None = None
        self._finished = False
        self._thread: threading.Thread | None = None

    @property
    def failed(self) -> bool:
        with self._condition:
            return self._error is not None

    @property
    def error(self) -> BaseException | None:
        with self._condition:
            return self._error

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("status publisher is already started")
        initial = {
            "schema": RESIDENT_PROCESS_STATUS_SCHEMA,
            "state": "BOOTING",
            "monotonic_ns": time.monotonic_ns(),
        }
        _atomic_private_json(self._config.path, initial, self._config.file_mode)
        self._thread = threading.Thread(
            target=self._run,
            name="v3-resident-status",
            daemon=False,
        )
        self._thread.start()

    def publish_tick(self, result: TickResult) -> None:
        if not isinstance(result, TickResult):
            raise TypeError("result must be TickResult")
        payload = _tick_status(result)
        with self._condition:
            if self._finished:
                if self._error is not None:
                    return
                raise RuntimeError("status publisher is finished")
            self._pending = payload
            self._condition.notify()

    def finish(
        self,
        *,
        report: ResidentRuntimeReport | None = None,
        error: BaseException | None = None,
    ) -> None:
        if self._thread is None:
            return
        if report is not None and error is not None:
            raise ValueError("finish accepts report or error, not both")
        payload: dict[str, object] = {
            "schema": RESIDENT_PROCESS_STATUS_SCHEMA,
            "state": "STOPPED" if error is None else "ERROR",
            "monotonic_ns": time.monotonic_ns(),
            "report": report.as_dict() if report is not None else None,
            "error_type": type(error).__name__ if error is not None else None,
            "error": str(error) if error is not None else None,
        }
        with self._condition:
            self._pending = payload
            self._finished = True
            self._condition.notify()
        self._thread.join(timeout=5.0)
        if self._thread.is_alive():
            raise RuntimeError("status publisher did not stop")

    def _run(self) -> None:
        while True:
            with self._condition:
                while self._pending is None and not self._finished:
                    self._condition.wait()
                payload = self._pending
                self._pending = None
                should_finish = self._finished
            if payload is not None:
                try:
                    _atomic_private_json(
                        self._config.path,
                        payload,
                        self._config.file_mode,
                    )
                except BaseException as exc:
                    with self._condition:
                        self._error = exc
                        self._finished = True
                    return
            if should_finish:
                return


def run_v3_resident_process(
    counter_gpio_backend: object,
    open_imu_bus: Callable[[int], object],
    open_lidar_port: Callable[[Callable[[], tuple[float, float, float]]], object],
    motor_gpio_backend: object,
    command_gateway: AtomicResidentCommandGateway,
    runtime_config: ResidentPhysicalRuntimeConfig,
    status_publisher: AsyncResidentStatusPublisher,
    *,
    approval: str,
    stop_requested: Callable[[], bool],
    capture_session: _PassiveCaptureSession | TriggeredCaptureSession | None = None,
    capture_trigger_requested: Callable[[], bool] | None = None,
    run_hardware: Callable[..., ResidentRuntimeReport] = run_native_hardware_resident_control,
) -> ResidentRuntimeReport:
    """Run one process ownership session and stop if status publication fails."""

    if not isinstance(command_gateway, AtomicResidentCommandGateway):
        raise TypeError("command_gateway must be AtomicResidentCommandGateway")
    if not isinstance(runtime_config, ResidentPhysicalRuntimeConfig):
        raise TypeError("runtime_config must be ResidentPhysicalRuntimeConfig")
    if not isinstance(status_publisher, AsyncResidentStatusPublisher):
        raise TypeError("status_publisher must be AsyncResidentStatusPublisher")
    if not callable(stop_requested):
        raise TypeError("stop_requested must be callable")
    if not callable(run_hardware):
        raise TypeError("run_hardware must be callable")
    if capture_session is not None and not isinstance(
        capture_session,
        (_PassiveCaptureSession, TriggeredCaptureSession),
    ):
        raise TypeError("capture_session must be a process capture session or None")
    if capture_trigger_requested is not None and not callable(capture_trigger_requested):
        raise TypeError("capture_trigger_requested must be callable or None")

    if capture_session is not None:
        capture_session.start()
    status_publisher.start()
    report: ResidentRuntimeReport | None = None
    caught: BaseException | None = None
    capture_error: BaseException | None = None

    def combined_stop() -> bool:
        if (
            capture_session is not None
            and capture_trigger_requested is not None
            and capture_trigger_requested()
        ):
            capture_session.trigger("SIGUSR1")
        value = stop_requested()
        if type(value) is not bool:
            raise TypeError("stop_requested must return bool")
        return value or status_publisher.failed

    try:
        hardware_kwargs: dict[str, object] = {
            "approval": approval,
            "stop_requested": combined_stop,
            "tick_observer": status_publisher.publish_tick,
        }
        if capture_session is not None:
            hardware_kwargs["record_observer"] = capture_session.observe
            hardware_kwargs["raw_lidar_observer"] = capture_session.observe_raw_lidar
        report = run_hardware(
            counter_gpio_backend,
            open_imu_bus,
            open_lidar_port,
            command_gateway,
            motor_gpio_backend,
            runtime_config,
            **hardware_kwargs,
        )
    except BaseException as exc:
        caught = exc
    finally:
        status_publisher.finish(report=report, error=caught)
        if capture_session is not None:
            try:
                capture_session.finalize(report, error=caught)
            except BaseException as exc:
                capture_error = exc

    status_error = status_publisher.error
    if status_error is not None:
        raise RuntimeError("resident status publication failed") from status_error
    if capture_error is not None:
        raise RuntimeError("production V3 capture failed") from capture_error
    if caught is not None:
        raise caught
    assert report is not None
    return report


def native_sensor_policy() -> NativeSensorPolicyConfig:
    """Return the policy proven by the live resident Test Hub session."""

    return NativeSensorPolicyConfig(
        encoder_maximum_sample_interval_ns=100_000_000,
        encoder_maximum_abs_velocity_mps=1.5,
        encoder_minimum_trust=0.5,
        imu_maximum_sample_age_ns=100_000_000,
        imu_heading_clockwise_positive=True,
        imu_yaw_rate_axis=2,
        imu_yaw_rate_clockwise_positive=False,
        imu_yaw_offset_rad=0.0,
        imu_minimum_confidence=0.5,
        imu_minimum_calibration=2,
        imu_allow_rate_only=True,
        lidar_maximum_result_age_ns=250_000_000,
        lidar_maximum_future_skew_ns=10_000_000,
        lidar_pose_r_scale=1.0,
        lidar_minimum_confidence=0.2,
        lidar_maximum_measurement_age_ns=250_000_000,
        encoder_minimum_estimation_pulses=4,
        encoder_minimum_estimation_window_ns=40_000_000,
        encoder_maximum_estimation_window_ns=160_000_000,
    )


def load_resident_runtime_config(project_root: Path = PROJECT_ROOT) -> ResidentPhysicalRuntimeConfig:
    """Close canonical static config and discard the bounded-only command profile."""

    bounded = load_bounded_physical_runtime_config(
        project_root / "conf" / "hardver.json",
        project_root / "conf" / "fizika.json",
        project_root / "conf" / "speed_map.json",
        BoundedTeleopProfile(
            command_id="resident-config-loader-not-executed",
            start_tick_id=1,
            active_tick_count=1,
            v_mps=0.01,
            omega_rad_s=0.0,
            max_v_mps=0.01,
            max_omega_rad_s=0.01,
        ),
        sensor_policy=native_sensor_policy(),
        control_path=project_root / "conf" / "vezerles.json",
    )
    return ResidentPhysicalRuntimeConfig.from_bounded(bounded)


def native_lidar_factory(
    sensors: NativeSensorHardwareConfig,
    serial_factory: Callable[..., object],
    project_root: Path = PROJECT_ROOT,
) -> Callable[[Callable[[], tuple[float, float, float]]], object]:
    """Close active LiDAR config once and return the sole production opener."""

    if not isinstance(sensors, NativeSensorHardwareConfig):
        raise TypeError("sensors must be NativeSensorHardwareConfig")
    if not callable(serial_factory):
        raise TypeError("serial_factory must be callable")
    lidar_config = load_native_lidar_port_config(
        project_root / "conf" / "hardver.json",
        project_root / "conf" / "vezerles.json",
        danger_zone_m=sensors.lidar_danger_zone_m,
    )

    def open_lidar(
        pose_provider: Callable[[], tuple[float, float, float]],
    ) -> object:
        return open_native_lidar_port(lidar_config, pose_provider, serial_factory)

    return open_lidar


def _runtime_owned_path(value: str, project_root: Path) -> Path:
    path = Path(value)
    if not path.is_absolute():
        path = project_root / path
    path = path.resolve(strict=False)
    try:
        path.relative_to((project_root / "runtime").resolve())
    except ValueError as exc:
        raise ValueError("process paths must stay below the canonical runtime directory") from exc
    return path


def _capture_configuration(
    project_root: Path,
    runtime_config: ResidentPhysicalRuntimeConfig | None = None,
) -> dict[str, object]:
    documents: dict[str, object] = {}
    for name, relative in (
        ("physics", "conf/fizika.json"),
        ("speed_map", "conf/speed_map.json"),
        ("hardware", "conf/hardver.json"),
        ("control", "conf/vezerles.json"),
    ):
        payload = json.loads((project_root / relative).read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            raise ValueError(f"{relative} must contain a JSON object")
        documents[name] = payload
    if runtime_config is None:
        return documents
    live = runtime_config.composition.live_control
    return {
        "resolved_control": live.control,
        "resolved_sensor_inputs": runtime_config.sensor_inputs,
        "resolved_resident_policy": {
            "max_preflight_age_ns": live.max_preflight_age_ns,
            "required_lidar_preflight_revisions": (
                live.required_lidar_preflight_revisions
            ),
            "tick_period_ns": runtime_config.tick_period_ns,
        },
        "resolved_motor_edge": runtime_config.composition.motor_output,
        "legacy_documents": documents,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the native resident V3 robot process")
    parser.add_argument("--approval", required=True)
    parser.add_argument("--command-path", default="runtime/v3_command.json")
    parser.add_argument("--status-path", default="runtime/v3_status.json")
    parser.add_argument("--capture-path")
    parser.add_argument("--capture-pre-event-ns", type=int, default=5_000_000_000)
    parser.add_argument("--capture-post-event-ns", type=int, default=2_000_000_000)
    parser.add_argument("--capture-ingress-capacity", type=int, default=256)
    parser.add_argument("--capture-max-ticks", type=int, default=512)
    parser.add_argument("--capture-max-bytes", type=int, default=16 * 1024 * 1024)
    parser.add_argument("--capture-max-raw-scans", type=int, default=64)
    parser.add_argument("--capture-max-raw-points", type=int, default=4_096)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.approval != RESIDENT_PHYSICAL_RUN_APPROVAL:
        print(
            json.dumps(
                {
                    "schema": RESIDENT_PROCESS_STATUS_SCHEMA,
                    "status": "ERROR",
                    "error": "explicit native resident V3 approval is required",
                },
                sort_keys=True,
            )
        )
        return 2

    stop = SignalStop()
    capture_trigger = SignalCaptureTrigger()
    old_handlers = {
        signum: signal.signal(signum, stop.handle)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    if args.capture_path is not None:
        old_handlers[signal.SIGUSR1] = signal.signal(
            signal.SIGUSR1,
            capture_trigger.handle,
        )
    try:
        command_path = _runtime_owned_path(args.command_path, PROJECT_ROOT)
        status_path = _runtime_owned_path(args.status_path, PROJECT_ROOT)
        capture_path = (
            _runtime_owned_path(args.capture_path, PROJECT_ROOT)
            if args.capture_path is not None
            else None
        )
        process_paths = [command_path, status_path]
        if capture_path is not None:
            process_paths.append(capture_path)
        if len(set(process_paths)) != len(process_paths):
            raise ValueError("command, status and capture paths must differ")
        runtime_config = load_resident_runtime_config()
        command_gateway = AtomicResidentCommandGateway(
            ResidentCommandMailboxConfig(path=command_path)
        )
        status_publisher = AsyncResidentStatusPublisher(
            ResidentStatusConfig(path=status_path)
        )
        capture_session = (
            TriggeredCaptureSession(
                CaptureSink(
                    capture_path.stem,
                    configuration=_capture_configuration(PROJECT_ROOT, runtime_config),
                    metadata={
                        "runtime": "v3_process_runtime",
                        "command_gateway": "AtomicResidentCommandGateway",
                    },
                ),
                capture_path,
                CaptureWindowConfig(
                    pre_event_ns=args.capture_pre_event_ns,
                    post_event_ns=args.capture_post_event_ns,
                    ingress_queue_capacity=args.capture_ingress_capacity,
                    max_tick_count=args.capture_max_ticks,
                    max_byte_capacity=args.capture_max_bytes,
                    max_raw_lidar_scans=args.capture_max_raw_scans,
                    max_raw_lidar_points_per_scan=args.capture_max_raw_points,
                ),
            )
            if capture_path is not None
            else None
        )

        import lgpio
        import serial
        import smbus2

        if runtime_config.sensor_inputs is None:
            raise ValueError("resident runtime did not close native sensor inputs")
        open_lidar = native_lidar_factory(
            runtime_config.sensor_inputs,
            serial.Serial,
        )

        report = run_v3_resident_process(
            lgpio,
            smbus2.SMBus,
            open_lidar,
            lgpio,
            command_gateway,
            runtime_config,
            status_publisher,
            approval=args.approval,
            stop_requested=stop,
            capture_session=capture_session,
            capture_trigger_requested=(
                capture_trigger.consume if capture_session is not None else None
            ),
        )
        print(json.dumps(report.as_dict(), sort_keys=True))
        return report.status
    except Exception as exc:
        print(
            json.dumps(
                {
                    "schema": RESIDENT_PROCESS_STATUS_SCHEMA,
                    "status": "ERROR",
                    "error_type": type(exc).__name__,
                    "error": str(exc),
                },
                sort_keys=True,
            )
        )
        return 1
    finally:
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "AsyncResidentStatusPublisher",
    "PROJECT_ROOT",
    "RESIDENT_PROCESS_STATUS_SCHEMA",
    "ResidentStatusConfig",
    "SignalCaptureTrigger",
    "SignalStop",
    "TriggeredCaptureSession",
    "_PassiveCaptureSession",
    "_capture_configuration",
    "load_resident_runtime_config",
    "main",
    "native_lidar_factory",
    "native_sensor_policy",
    "run_v3_resident_process",
]
