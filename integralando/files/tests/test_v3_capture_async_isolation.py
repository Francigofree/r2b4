from __future__ import annotations

import ast
import inspect
import pickle
import queue
from pathlib import Path

import v3.process_sidecars as sidecars
import v3_runtime
from v3.execution import CaptureRecord
from v3.process_sidecars import ProcessMcapCaptureSession


FORBIDDEN_CONTROL_CAPTURE_CALLS = {
    "encode_capture_record",
    "encode_value",
    "_measure_projection",
    "json.dumps",
    "pickle.dumps",
    "deepcopy",
    "postprocess_capture",
}


def _call_names(source: str) -> set[str]:
    tree = ast.parse(source)
    names: set[str] = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        target = node.func
        if isinstance(target, ast.Name):
            names.add(target.id)
        elif isinstance(target, ast.Attribute):
            chain = [target.attr]
            value = target.value
            while isinstance(value, ast.Attribute):
                chain.append(value.attr)
                value = value.value
            if isinstance(value, ast.Name):
                chain.append(value.id)
            names.add(".".join(reversed(chain)))
    return names


def test_process_capture_observe_contains_no_recursive_encoding_or_file_io():
    source = inspect.getsource(ProcessMcapCaptureSession.observe)
    calls = _call_names(source)
    assert not (calls & FORBIDDEN_CONTROL_CAPTURE_CALLS)
    assert "_enqueue" in calls or "self._enqueue" in calls


def test_process_capture_observe_preserves_exact_immutable_record(tmp_path):
    from test_v3_mcap_e2e import records

    _, values = records(2)
    session = ProcessMcapCaptureSession(
        "exact-record",
        tmp_path / "exact-record.mcap",
        configuration={"x": 1},
        project_root=Path(sidecars.__file__).parents[1],
    )
    session._data_queue.close()
    session._data_queue = queue.Queue(maxsize=8)
    session._started = True

    expected_pickle = pickle.dumps(values[0], protocol=pickle.HIGHEST_PROTOCOL)
    session.observe(values[0])
    kind, payload = session._data_queue.get_nowait()

    assert kind == "record"
    assert isinstance(payload, CaptureRecord)
    assert pickle.dumps(payload, protocol=pickle.HIGHEST_PROTOCOL) == expected_pickle


def test_recursive_capture_projector_is_not_production_ingress():
    source = Path(sidecars.__file__).read_text(encoding="utf-8")
    session_source = source.split("class ProcessMcapCaptureSession:", 1)[1]
    assert "CaptureCoreProjector" not in session_source
    assert ".project(record)" not in session_source


def test_runtime_exposes_separate_capture_checkpoint_and_tap_timing():
    source = Path(v3_runtime.__file__).read_text(encoding="utf-8")
    assert '"CAPTURE_CHECKPOINT"' in source
    assert '"CAPTURE_TAP"' in source
    assert source.index('"CAPTURE_CHECKPOINT"') < source.index('"CAPTURE_TAP"')


def test_sidecar_still_owns_canonical_mcap_encoding_and_postprocess():
    source = Path(sidecars.__file__).read_text(encoding="utf-8")
    assert 'hub.publish(payload, topic="v3.capture_record")' in source
    assert "McapCaptureConsumer" in source
    assert "postprocess_capture" in source


def test_raw_lidar_remains_direct_producer_to_sidecar_lane():
    project_root = Path(sidecars.__file__).parents[1]
    source = (project_root / "v3_process_runtime.py").read_text(encoding="utf-8")
    assert "capture_session.raw_lidar_queue" in source
    assert "capture_raw_queue=" in source
