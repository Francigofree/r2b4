#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import json
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path

BASELINE = "cb90eef504867b6dc2a468a1c55bcb406c585464"
PACKAGE_ROOT = Path(__file__).resolve().parent
PAYLOAD = PACKAGE_ROOT / "payload"


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly one source match, got {count}")
    return text.replace(old, new, 1)


def patch_v3_runtime(text: str) -> str:
    text = replace_once(
        text,
        "from v3.ports import CommandGateway\n",
        "from v3.ports import CommandGateway\n"
        "from v3.runtime_performance import (\n"
        "    RuntimeTimingAccumulator,\n"
        "    RuntimeTimingEvidence,\n"
        ")\n",
        "v3_runtime import timing",
    )
    text = replace_once(
        text,
        "    operator_stopped: bool\n\n    def __post_init__(self) -> None:\n",
        "    operator_stopped: bool\n"
        "    timing: RuntimeTimingEvidence | None = None\n\n"
        "    def __post_init__(self) -> None:\n",
        "v3_runtime report timing field",
    )
    text = replace_once(
        text,
        "        if type(self.operator_stopped) is not bool:\n"
        "            raise TypeError(\"operator_stopped must be bool\")\n\n"
        "    def as_dict(self) -> dict[str, object]:\n"
        "        \"\"\"Return the bounded status surface used by a later process entrypoint.\"\"\"\n\n"
        "        return {\n"
        "            \"schema\": \"R2B4_V3_RESIDENT_RUNTIME_REPORT_V2\",\n"
        "            \"status\": \"PASS\" if self.status == RUN_OK else \"FAULT\",\n"
        "            \"run_status\": self.status,\n"
        "            \"exit_reason\": self.exit_reason,\n"
        "            \"tick_count\": self.tick_count,\n"
        "            \"normal_tick_count\": self.normal_tick_count,\n"
        "            \"last_tick_id\": self.last_tick_id,\n"
        "            \"final_lifecycle\": self.final_lifecycle.value,\n"
        "            \"final_safety_decision\": (\n"
        "                self.final_safety_decision.value\n"
        "                if self.final_safety_decision is not None\n"
        "                else None\n"
        "            ),\n"
        "            \"final_reason\": self.final_reason,\n"
        "            \"fault_layer\": self.fault_layer,\n"
        "            \"operator_stopped\": self.operator_stopped,\n"
        "            \"termination_class\": self.termination_class,\n"
        "        }\n",
        "        if type(self.operator_stopped) is not bool:\n"
        "            raise TypeError(\"operator_stopped must be bool\")\n"
        "        if self.timing is not None and not isinstance(\n"
        "            self.timing, RuntimeTimingEvidence\n"
        "        ):\n"
        "            raise TypeError(\"timing must be RuntimeTimingEvidence or None\")\n\n"
        "    def as_dict(self) -> dict[str, object]:\n"
        "        \"\"\"Return terminal status plus passive scheduling evidence.\"\"\"\n\n"
        "        payload: dict[str, object] = {\n"
        "            \"schema\": \"R2B4_V3_RESIDENT_RUNTIME_REPORT_V2\",\n"
        "            \"status\": \"PASS\" if self.status == RUN_OK else \"FAULT\",\n"
        "            \"run_status\": self.status,\n"
        "            \"exit_reason\": self.exit_reason,\n"
        "            \"tick_count\": self.tick_count,\n"
        "            \"normal_tick_count\": self.normal_tick_count,\n"
        "            \"last_tick_id\": self.last_tick_id,\n"
        "            \"final_lifecycle\": self.final_lifecycle.value,\n"
        "            \"final_safety_decision\": (\n"
        "                self.final_safety_decision.value\n"
        "                if self.final_safety_decision is not None\n"
        "                else None\n"
        "            ),\n"
        "            \"final_reason\": self.final_reason,\n"
        "            \"fault_layer\": self.fault_layer,\n"
        "            \"operator_stopped\": self.operator_stopped,\n"
        "            \"termination_class\": self.termination_class,\n"
        "        }\n"
        "        if self.timing is not None:\n"
        "            payload[\"timing\"] = self.timing.as_dict()\n"
        "        return payload\n",
        "v3_runtime report serialization",
    )
    text = replace_once(
        text,
        "    operator_stopped: bool,\n) -> ResidentRuntimeReport:\n",
        "    operator_stopped: bool,\n"
        "    timing: RuntimeTimingEvidence | None = None,\n"
        ") -> ResidentRuntimeReport:\n",
        "v3_runtime _report signature",
    )
    text = replace_once(
        text,
        "        operator_stopped=operator_stopped,\n    )\n\n\ndef run_resident_physical_control",
        "        operator_stopped=operator_stopped,\n"
        "        timing=timing,\n"
        "    )\n\n\ndef run_resident_physical_control",
        "v3_runtime _report construction",
    )
    text = replace_once(
        text,
        "    record_observer: Callable[[CaptureRecord], None] | None = None,\n"
        ") -> ResidentRuntimeReport:\n"
        "    \"\"\"Run until signal/stop or fault, then release every physical capability.\"\"\"\n",
        "    record_observer: Callable[[CaptureRecord], None] | None = None,\n"
        "    timing_enabled: bool = False,\n"
        ") -> ResidentRuntimeReport:\n"
        "    \"\"\"Run until signal/stop or fault, then release every physical capability.\"\"\"\n",
        "v3_runtime timing flag signature",
    )
    text = replace_once(
        text,
        "    if record_observer is not None and not callable(record_observer):\n"
        "        raise TypeError(\"record_observer must be callable or None\")\n"
        "    if _stop_is_requested(stop_requested):\n",
        "    if record_observer is not None and not callable(record_observer):\n"
        "        raise TypeError(\"record_observer must be callable or None\")\n"
        "    if type(timing_enabled) is not bool:\n"
        "        raise TypeError(\"timing_enabled must be bool\")\n"
        "    if _stop_is_requested(stop_requested):\n",
        "v3_runtime timing flag validation",
    )
    text = replace_once(
        text,
        "    if _stop_is_requested(stop_requested):\n"
        "        return _report(\n",
        "    timing = (\n"
        "        RuntimeTimingAccumulator(config.tick_period_ns)\n"
        "        if timing_enabled\n"
        "        else None\n"
        "    )\n"
        "    if _stop_is_requested(stop_requested):\n"
        "        return _report(\n",
        "v3_runtime timing accumulator",
    )
    text = replace_once(
        text,
        "            context = TickContext(tick_id, now_ns)\n"
        "            if shutdown_requested:\n",
        "            context = TickContext(tick_id, now_ns)\n"
        "            if shutdown_requested:\n",
        "v3_runtime context anchor",
    )
    text = replace_once(
        text,
        "                    normal_tick_count=normal_tick_count,\n"
        "                    operator_stopped=True,\n"
        "                )\n\n"
        "            try:\n"
        "                last_result, record = runtime.tick_execution(context)\n",
        "                    normal_tick_count=normal_tick_count,\n"
        "                    operator_stopped=True,\n"
        "                    timing=(timing.snapshot() if timing is not None else None),\n"
        "                )\n\n"
        "            if timing is not None:\n"
        "                timing.observe_tick_start(now_ns, next_deadline_ns)\n"
        "            work_started_ns = time.perf_counter_ns()\n"
        "            control_started_ns = work_started_ns\n"
        "            try:\n"
        "                last_result, record = runtime.tick_execution(context)\n",
        "v3_runtime timing start",
    )
    text = replace_once(
        text,
        "            if record_observer is not None:\n"
        "                if (\n",
        "            control_completed_ns = time.perf_counter_ns()\n"
        "            if timing is not None:\n"
        "                timing.observe_control(control_completed_ns - control_started_ns)\n"
        "            observer_started_ns = control_completed_ns\n"
        "            if record_observer is not None:\n"
        "                if (\n",
        "v3_runtime control duration",
    )
    text = replace_once(
        text,
        "            if readiness_observer is not None:\n"
        "                readiness_observer(last_result, runtime.ready_for_active)\n"
        "            normal_tick_count += 1\n",
        "            if readiness_observer is not None:\n"
        "                readiness_observer(last_result, runtime.ready_for_active)\n"
        "            observers_completed_ns = time.perf_counter_ns()\n"
        "            if timing is not None:\n"
        "                timing.observe_observer(observers_completed_ns - observer_started_ns)\n"
        "                timing.observe_work(observers_completed_ns - work_started_ns)\n"
        "            normal_tick_count += 1\n",
        "v3_runtime observer timing",
    )
    text = replace_once(
        text,
        "                    normal_tick_count=normal_tick_count,\n"
        "                    operator_stopped=False,\n"
        "                )\n",
        "                    normal_tick_count=normal_tick_count,\n"
        "                    operator_stopped=False,\n"
        "                    timing=(timing.snapshot() if timing is not None else None),\n"
        "                )\n",
        "v3_runtime fault timing",
    )
    text = replace_once(
        text,
        "    record_observer: Callable[[CaptureRecord], None] | None = None,\n"
        ") -> ResidentRuntimeReport:\n"
        "    \"\"\"Run the resident path and always close the sole concrete input owner.\"\"\"\n",
        "    record_observer: Callable[[CaptureRecord], None] | None = None,\n"
        "    timing_enabled: bool = False,\n"
        ") -> ResidentRuntimeReport:\n"
        "    \"\"\"Run the resident path and always close the sole concrete input owner.\"\"\"\n",
        "v3_runtime owned timing signature",
    )
    text = replace_once(
        text,
        "            record_observer=record_observer,\n"
        "        )\n",
        "            record_observer=record_observer,\n"
        "            timing_enabled=timing_enabled,\n"
        "        )\n",
        "v3_runtime owned timing forward",
    )
    return text


def patch_process_runtime(text: str) -> str:
    text = replace_once(
        text,
        "from v3.execution import CaptureRecord\n",
        "from v3.execution import CaptureRecord\n"
        "from v3.runtime_performance import (\n"
        "    RuntimeAffinityConfig,\n"
        "    apply_process_affinity_layout,\n"
        "    load_runtime_affinity_config,\n"
        "    temporary_current_affinity,\n"
        ")\n",
        "process runtime affinity import",
    )
    text = replace_once(
        text,
        "    capture_trigger_requested: Callable[[], bool] | None = None,\n"
        "    run_hardware: Callable[..., ResidentRuntimeReport] = run_native_hardware_resident_control,\n",
        "    capture_trigger_requested: Callable[[], bool] | None = None,\n"
        "    affinity_config: RuntimeAffinityConfig | None = None,\n"
        "    run_hardware: Callable[..., ResidentRuntimeReport] = run_native_hardware_resident_control,\n",
        "process runtime signature",
    )
    text = replace_once(
        text,
        "    if capture_trigger_requested is not None and not callable(capture_trigger_requested):\n"
        "        raise TypeError(\"capture_trigger_requested must be callable or None\")\n\n"
        "    if capture_session is not None:\n"
        "        capture_session.start()\n",
        "    if capture_trigger_requested is not None and not callable(capture_trigger_requested):\n"
        "        raise TypeError(\"capture_trigger_requested must be callable or None\")\n"
        "    if affinity_config is not None and not isinstance(\n"
        "        affinity_config, RuntimeAffinityConfig\n"
        "    ):\n"
        "        raise TypeError(\"affinity_config must be RuntimeAffinityConfig or None\")\n\n"
        "    affinity = affinity_config or RuntimeAffinityConfig(enabled=False)\n"
        "    if affinity.enabled:\n"
        "        apply_process_affinity_layout(affinity)\n"
        "    if capture_session is not None:\n"
        "        with temporary_current_affinity(\n"
        "            affinity.io_cpu if affinity.enabled else None,\n"
        "            role=\"io-start\",\n"
        "            strict=affinity.strict,\n"
        "        ):\n"
        "            capture_session.start()\n",
        "process runtime start affinity",
    )
    text = replace_once(
        text,
        "    try:\n"
        "        status_publisher.start()\n"
        "        hardware_kwargs: dict[str, object] = {\n",
        "    try:\n"
        "        with temporary_current_affinity(\n"
        "            affinity.io_cpu if affinity.enabled else None,\n"
        "            role=\"io-start\",\n"
        "            strict=affinity.strict,\n"
        "        ):\n"
        "            status_publisher.start()\n"
        "        hardware_kwargs: dict[str, object] = {\n",
        "process status affinity",
    )
    text = replace_once(
        text,
        "            \"readiness_observer\": status_publisher.publish_tick,\n"
        "        }\n",
        "            \"readiness_observer\": status_publisher.publish_tick,\n"
        "        }\n"
        "        if affinity_config is not None:\n"
        "            hardware_kwargs[\"affinity_config\"] = affinity_config\n",
        "process hardware affinity kwarg",
    )
    text = replace_once(
        text,
        "def native_lidar_factory(\n"
        "    sensors: NativeSensorHardwareConfig,\n"
        "    serial_factory: Callable[..., object],\n"
        "    project_root: Path = PROJECT_ROOT,\n"
        ") -> Callable[[Callable[[int], TimedPoseReference | None]], object]:\n",
        "def native_lidar_factory(\n"
        "    sensors: NativeSensorHardwareConfig,\n"
        "    serial_factory: Callable[..., object],\n"
        "    project_root: Path = PROJECT_ROOT,\n"
        "    affinity_config: RuntimeAffinityConfig | None = None,\n"
        ") -> Callable[[Callable[[int], TimedPoseReference | None]], object]:\n",
        "lidar factory affinity signature",
    )
    text = replace_once(
        text,
        "    if not callable(serial_factory):\n"
        "        raise TypeError(\"serial_factory must be callable\")\n"
        "    lidar_config = load_native_lidar_port_config(\n",
        "    if not callable(serial_factory):\n"
        "        raise TypeError(\"serial_factory must be callable\")\n"
        "    if affinity_config is not None and not isinstance(\n"
        "        affinity_config, RuntimeAffinityConfig\n"
        "    ):\n"
        "        raise TypeError(\"affinity_config must be RuntimeAffinityConfig or None\")\n"
        "    affinity = affinity_config or RuntimeAffinityConfig(enabled=False)\n"
        "    lidar_config = load_native_lidar_port_config(\n",
        "lidar factory affinity validation",
    )
    text = replace_once(
        text,
        "    def open_lidar(\n"
        "        pose_provider: Callable[[int], TimedPoseReference | None],\n"
        "    ) -> object:\n"
        "        return open_native_lidar_port(lidar_config, pose_provider, serial_factory)\n",
        "    def open_lidar(\n"
        "        pose_provider: Callable[[int], TimedPoseReference | None],\n"
        "    ) -> object:\n"
        "        with temporary_current_affinity(\n"
        "            affinity.lidar_cpu if affinity.enabled else None,\n"
        "            role=\"lidar\",\n"
        "            strict=affinity.strict,\n"
        "        ):\n"
        "            return open_native_lidar_port(lidar_config, pose_provider, serial_factory)\n",
        "lidar factory affinity start",
    )
    text = replace_once(
        text,
        "        runtime_config = load_resident_runtime_config()\n"
        "        command_gateway = AtomicResidentCommandGateway(\n",
        "        runtime_config = load_resident_runtime_config()\n"
        "        affinity_config = load_runtime_affinity_config(\n"
        "            PROJECT_ROOT / \"conf\" / \"vezerles.json\"\n"
        "        )\n"
        "        command_gateway = AtomicResidentCommandGateway(\n",
        "process main load affinity",
    )
    text = replace_once(
        text,
        "                metadata={\"runtime\": \"v3_process_runtime\", \"command_gateway\": \"AtomicResidentCommandGateway\"},\n",
        "                metadata={\n"
        "                    \"runtime\": \"v3_process_runtime\",\n"
        "                    \"command_gateway\": \"AtomicResidentCommandGateway\",\n"
        "                    \"runtime_affinity\": affinity_config.as_dict(),\n"
        "                },\n",
        "capture affinity metadata",
    )
    text = replace_once(
        text,
        "        open_lidar = native_lidar_factory(\n"
        "            runtime_config.sensor_inputs,\n"
        "            serial.Serial,\n"
        "        )\n",
        "        open_lidar = native_lidar_factory(\n"
        "            runtime_config.sensor_inputs,\n"
        "            serial.Serial,\n"
        "            affinity_config=affinity_config,\n"
        "        )\n",
        "process main lidar affinity",
    )
    text = replace_once(
        text,
        "            capture_trigger_requested=(\n"
        "                capture_trigger.consume if capture_session is not None else None\n"
        "            ),\n"
        "        )\n",
        "            capture_trigger_requested=(\n"
        "                capture_trigger.consume if capture_session is not None else None\n"
        "            ),\n"
        "            affinity_config=affinity_config,\n"
        "        )\n",
        "process main runtime affinity",
    )
    return text


def patch_hardware_runtime(text: str) -> str:
    text = replace_once(
        text,
        "from v3.ports import CommandGateway\n",
        "from v3.ports import CommandGateway\n"
        "from v3.runtime_performance import (\n"
        "    RuntimeAffinityConfig,\n"
        "    temporary_current_affinity,\n"
        ")\n",
        "hardware runtime affinity import",
    )
    text = replace_once(
        text,
        "        open_camera: Picamera2Factory = default_picamera2_factory,\n"
        "        monotonic_ns: Callable[[], int] = time.monotonic_ns,\n",
        "        open_camera: Picamera2Factory = default_picamera2_factory,\n"
        "        affinity_config: RuntimeAffinityConfig | None = None,\n"
        "        monotonic_ns: Callable[[], int] = time.monotonic_ns,\n",
        "hardware owner signature",
    )
    text = replace_once(
        text,
        "        if not isinstance(config, NativeSensorHardwareConfig):\n"
        "            raise TypeError(\"config must be NativeSensorHardwareConfig\")\n"
        "        for callback, name in (\n",
        "        if not isinstance(config, NativeSensorHardwareConfig):\n"
        "            raise TypeError(\"config must be NativeSensorHardwareConfig\")\n"
        "        if affinity_config is not None and not isinstance(\n"
        "            affinity_config, RuntimeAffinityConfig\n"
        "        ):\n"
        "            raise TypeError(\"affinity_config must be RuntimeAffinityConfig or None\")\n"
        "        affinity = affinity_config or RuntimeAffinityConfig(enabled=False)\n"
        "        for callback, name in (\n",
        "hardware owner affinity validation",
    )
    old_camera = '''            if config.camera_device is not None:\n                camera = NativePicamera2Camera(\n                    config.camera_device,\n                    picamera_factory=open_camera,\n                    sensor_timestamp_mapper=(\n                        raspberry_pi_sensor_timestamp_to_monotonic_ns\n                    ),\n                    monotonic_ns=monotonic_ns,\n                )\n                # Camera is explicitly non-critical. A start failure remains\n                # visible as CAMERA_FRONT FAILED but must not abort core input\n                # ownership or the motor-control runtime.\n                camera.start()\n\n'''
    new_camera = '''            if config.camera_device is not None:\n                with temporary_current_affinity(\n                    affinity.vision_cpu if affinity.enabled else None,\n                    role="vision",\n                    strict=affinity.strict,\n                ):\n                    camera = NativePicamera2Camera(\n                        config.camera_device,\n                        picamera_factory=open_camera,\n                        sensor_timestamp_mapper=(\n                            raspberry_pi_sensor_timestamp_to_monotonic_ns\n                        ),\n                        monotonic_ns=monotonic_ns,\n                    )\n                    # Camera/libcamera workers inherit the dedicated vision CPU.\n                    # Camera remains non-critical for motor safety authority.\n                    camera.start()\n\n'''
    text = replace_once(text, old_camera, new_camera, "hardware camera affinity")
    old_detector = '''                else:\n                    detector: NativePersonDetector | None = None\n                    try:\n                        backend = LiteRtSsdPersonDetector(\n                            config.person_detection_backend\n                        )\n                        detector = NativePersonDetector(camera, backend)\n                        if not detector.start():\n                            raise RuntimeError("person detector worker did not start")\n                        person_detection_port = detector\n                    except Exception as exc:\n                        if detector is not None:\n                            try:\n                                detector.stop()\n                            except Exception:\n                                pass\n                        # Detector/model/runtime failures are capability-local.\n                        # TELEOP/EXPLORE motor availability is unchanged because\n                        # PERSON_DETECTOR_FRONT is not production-critical.\n                        person_detection_port = UnavailablePersonDetectionPort(\n                            f"{type(exc).__name__}:{exc}"\n                        )\n\n'''
    new_detector = '''                else:\n                    detector: NativePersonDetector | None = None\n                    with temporary_current_affinity(\n                        affinity.vision_cpu if affinity.enabled else None,\n                        role="vision",\n                        strict=affinity.strict,\n                    ):\n                        try:\n                            backend = LiteRtSsdPersonDetector(\n                                config.person_detection_backend\n                            )\n                            detector = NativePersonDetector(camera, backend)\n                            if not detector.start():\n                                raise RuntimeError("person detector worker did not start")\n                            person_detection_port = detector\n                        except Exception as exc:\n                            if detector is not None:\n                                try:\n                                    detector.stop()\n                                except Exception:\n                                    pass\n                            # Detector/model/runtime failures are capability-local.\n                            # TELEOP/EXPLORE motor availability is unchanged because\n                            # PERSON_DETECTOR_FRONT is not production-critical.\n                            person_detection_port = UnavailablePersonDetectionPort(\n                                f"{type(exc).__name__}:{exc}"\n                            )\n\n'''
    text = replace_once(text, old_detector, new_detector, "hardware detector affinity")
    text = replace_once(
        text,
        "    raw_lidar_observer: Callable[[object | None], None] | None = None,\n"
        ") -> ResidentRuntimeReport:\n",
        "    raw_lidar_observer: Callable[[object | None], None] | None = None,\n"
        "    affinity_config: RuntimeAffinityConfig | None = None,\n"
        ") -> ResidentRuntimeReport:\n",
        "resident hardware affinity signature",
    )
    text = replace_once(
        text,
        "    if raw_lidar_observer is not None and not callable(raw_lidar_observer):\n"
        "        raise TypeError(\"raw_lidar_observer must be callable or None\")\n"
        "    if _stop_value(stop_requested):\n",
        "    if raw_lidar_observer is not None and not callable(raw_lidar_observer):\n"
        "        raise TypeError(\"raw_lidar_observer must be callable or None\")\n"
        "    if affinity_config is not None and not isinstance(\n"
        "        affinity_config, RuntimeAffinityConfig\n"
        "    ):\n"
        "        raise TypeError(\"affinity_config must be RuntimeAffinityConfig or None\")\n"
        "    if _stop_value(stop_requested):\n",
        "resident hardware affinity validation",
    )
    # Only the resident owner gets affinity; finite/bounded test surfaces retain old defaults.
    marker = '''    owner = NativeHardwareSensorOwner(\n        counter_gpio_backend,\n        open_imu_bus,\n        open_lidar_port,\n        config.sensor_inputs,\n        monotonic_ns=monotonic_ns,\n        sleep=sleep,\n    )\n    try:\n        def observe(result: TickResult) -> None:\n'''
    replacement = '''    owner = NativeHardwareSensorOwner(\n        counter_gpio_backend,\n        open_imu_bus,\n        open_lidar_port,\n        config.sensor_inputs,\n        affinity_config=affinity_config,\n        monotonic_ns=monotonic_ns,\n        sleep=sleep,\n    )\n    try:\n        def observe(result: TickResult) -> None:\n'''
    text = replace_once(text, marker, replacement, "resident owner affinity")
    text = replace_once(
        text,
        "            record_observer=record_observer,\n"
        "        )\n"
        "    finally:\n"
        "        owner.close()\n\n\n__all__ = [",
        "            record_observer=record_observer,\n"
        "            timing_enabled=bool(\n"
        "                affinity_config is not None and affinity_config.enabled\n"
        "            ),\n"
        "        )\n"
        "    finally:\n"
        "        owner.close()\n\n\n__all__ = [",
        "resident hardware timing enable",
    )
    return text


def patch_configs(root: Path) -> dict[Path, str]:
    control_path = root / "conf" / "vezerles.json"
    hardware_path = root / "conf" / "hardver.json"
    control = json.loads(control_path.read_text(encoding="utf-8"))
    hardware = json.loads(hardware_path.read_text(encoding="utf-8"))
    control["runtime_affinity"] = {
        "enabled": True,
        "strict": True,
        "runtime_cpu": 3,
        "lidar_cpu": 2,
        "vision_cpu": 1,
        "io_cpu": 0,
    }
    person = hardware.get("person_detection")
    if not isinstance(person, dict):
        raise RuntimeError("conf/hardver.json person_detection is missing")
    person["num_threads"] = 1
    return {
        control_path: json.dumps(control, indent=2, ensure_ascii=False) + "\n",
        hardware_path: json.dumps(hardware, indent=2, ensure_ascii=False) + "\n",
    }


def main() -> int:
    if len(sys.argv) != 2:
        raise SystemExit("usage: python3 upgrade.py /home/alba/project_r2b4")
    root = Path(sys.argv[1]).resolve()
    head = subprocess.check_output(
        ["git", "-C", str(root), "rev-parse", "HEAD"], text=True
    ).strip()
    if head != BASELINE:
        raise RuntimeError(f"upgrade baseline is {BASELINE[:12]}, current HEAD is {head[:12]}")

    targets: dict[Path, str] = {}
    source_transforms = {
        root / "v3_runtime.py": patch_v3_runtime,
        root / "v3_process_runtime.py": patch_process_runtime,
        root / "v3_hardware_runtime.py": patch_hardware_runtime,
    }
    for path, transform in source_transforms.items():
        targets[path] = transform(path.read_text(encoding="utf-8"))
    targets.update(patch_configs(root))
    for relative in ("v3/runtime_performance.py", "tools/v3_performance_audit.py"):
        targets[root / relative] = (PAYLOAD / relative).read_text(encoding="utf-8")

    backup = root.parent / f"{root.name}_backup_cpu_affinity_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    backup.mkdir(parents=True, exist_ok=False)
    for path in targets:
        if path.exists():
            destination = backup / path.relative_to(root)
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, destination)

    for path, content in targets.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
        if path.name == "v3_performance_audit.py":
            path.chmod(0o755)

    print("R2B4 CPU-affinity + timing-evidence upgrade applied")
    print(f"baseline={BASELINE}")
    print(f"backup={backup}")
    for path in sorted(targets):
        digest = hashlib.sha256(path.read_bytes()).hexdigest()[:16]
        print(f"updated {path.relative_to(root)} sha256={digest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
