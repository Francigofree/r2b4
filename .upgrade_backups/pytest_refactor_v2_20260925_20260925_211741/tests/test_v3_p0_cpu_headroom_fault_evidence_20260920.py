# upgrade: p0_cpu_headroom_fault_evidence_20260920
from __future__ import annotations

import importlib.util
import sys
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

from v3.capture_encoding import encode_capture_record
from v3.composition.native_control import NativeControlComposition
from v3.engine import LayerFaultEvidence, TickEngine
from v3.execution import ExecutionRecord


ROOT = (Path(__import__("os").environ["R2B4_ROOT"]).resolve() if __import__("os").environ.get("R2B4_ROOT") else next((p for p in Path(__file__).resolve().parents if (p / "conf" / "hardver.json").is_file() and (p / "v3").is_dir()), Path.cwd()))


def _load_tick_engine_helpers():
    path = ROOT / "tests" / "test_v3_tick_engine.py"
    name = "_r2b4_tick_engine_test_helpers_p0_cpu_headroom"
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def test_vision_workers_are_process_isolated():
    source = (ROOT / "v3_hardware_runtime.py").read_text(encoding="utf-8")
    start = source.index("            # Camera + detector share one child")
    end = source.index("            inputs = NativeSensorInputOwner(", start)
    vision_region = source[start:end]

    assert "ProcessVisionPort(" in vision_region
    assert "affinity.vision_cpu if affinity.enabled else None" in vision_region
    assert "affinity.io_cpu if affinity.enabled else None" not in vision_region
    assert "person_detection_port = camera" in vision_region


def test_encoder_imu_lane_stays_on_io_cpu_and_lidar_stays_on_lidar_cpu():
    source = (ROOT / "v3_hardware_runtime.py").read_text(encoding="utf-8")
    start = source.index("        return run_owned_resident_physical_control(")
    region = source[start:]

    assert "input_worker_cpu=(affinity.io_cpu if affinity.enabled else None)" in region
    assert "affinity.lidar_cpu if affinity.enabled else None" in region
    assert "Critical encoder/IMU acquisition stays on the dedicated I/O CPU" in region


def test_tick_engine_keeps_bounded_exception_type_and_message_as_evidence():
    helpers = _load_tick_engine_helpers()
    writer = helpers.RecordingWriter([])
    layers = helpers._layers(writer)

    def fail_l11(wheels, frame):
        raise ValueError("diagnostic-l11-failure")

    engine = TickEngine(replace(layers, actuator_control=fail_l11))
    inputs = helpers._inputs()
    result = engine.run_tick(inputs)

    assert result.trace.fault_layer == "L11"
    assert result.final_actuation.reason == "L11_ERROR"
    assert result.final_actuation.enabled is False

    evidence = engine.fault_evidence
    assert len(evidence) == 1
    fault = evidence[0]
    assert isinstance(fault, LayerFaultEvidence)
    assert fault.context == inputs.context
    assert fault.layer == "L11"
    assert fault.error_type == "ValueError"
    assert fault.message == "diagnostic-l11-failure"

    encoded = encode_capture_record(ExecutionRecord(inputs, result, evidence))
    item = encoded["tick_evidence"][0]
    assert item["__type__"] == "LayerFaultEvidence"
    assert item["layer"] == "L11"
    assert item["error_type"] == "ValueError"
    assert item["message"] == "diagnostic-l11-failure"


def test_native_control_tick_evidence_includes_ekf_fault_and_follow_evidence():
    composition = object.__new__(NativeControlComposition)
    composition._estimator = SimpleNamespace(last_update_evidence=("ekf",))
    composition._engine = SimpleNamespace(fault_evidence=("fault",))
    composition._navigation = SimpleNamespace(follow_person_evidence="follow")
    assert composition.tick_evidence == ("ekf", "fault", "follow")
