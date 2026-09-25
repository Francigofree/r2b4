from __future__ import annotations
import threading
import time
from dataclasses import dataclass
from pathlib import Path
import numpy as np
from v3.adapters.litert_person_detector import LiteRtPersonDetectorConfig, LiteRtSsdPersonDetector
from v3.adapters.person_detection import NativePersonDetector, PersonBox, PersonDetection

@dataclass(frozen=True)
class _Frame:
    sequence: int
    measurement_monotonic_ns: int
    width: int = 2
    height: int = 2
    pixel_format: str = 'RGB888'
    stride_bytes: int = 8
    image_bytes: bytes = bytes(range(16))

@dataclass(frozen=True)
class _Status:
    running: bool = True
    last_error: str | None = None

@dataclass(frozen=True)
class _Edge:
    status: _Status
    frame: _Frame | None

class _Camera:

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._frame: _Frame | None = None

    def publish(self, frame: _Frame) -> None:
        with self._condition:
            self._frame = frame
            self._condition.notify_all()

    def get_edge_snapshot(self):
        with self._condition:
            return _Edge(_Status(), self._frame)

    def wait_for_new_frame(self, after_sequence=0, timeout_s=1.0):
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while self._frame is None or self._frame.sequence <= after_sequence:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return _Edge(_Status(), self._frame)
                self._condition.wait(remaining)
            return _Edge(_Status(), self._frame)

class _Backend:

    def __init__(self) -> None:
        self.frames = []

    def detect(self, frame):
        self.frames.append(frame.sequence)
        return (PersonDetection(0.8, PersonBox(0.1, 0.2, 0.9, 0.7)), PersonDetection(0.6, PersonBox(0.2, 0.1, 0.8, 0.5)))

def test_person_detector_publishes_frame_lineage_and_primary():
    camera = _Camera()
    backend = _Backend()
    detector = NativePersonDetector(camera, backend)
    assert detector.start()
    camera.publish(_Frame(11, 1000))
    result = detector.wait_for_new_detection(0, timeout_s=1.0)
    detector.stop()
    assert result is not None
    assert result.source_frame_sequence == 11
    assert result.measurement_monotonic_ns == 1000
    assert result.primary is not None
    assert result.primary.confidence == 0.8
    assert result.primary.box.center_x == pytest.approx(0.45)
    assert backend.frames == [11]

class _FakeInterpreter:

    def __init__(self, **kwargs) -> None:
        self.kwargs = kwargs
        self.input = None

    def allocate_tensors(self):
        pass

    def get_input_details(self):
        return [{'index': 1, 'shape': np.array([1, 2, 2, 3]), 'dtype': np.uint8, 'quantization': (0.0, 0), 'name': 'input'}]

    def get_output_details(self):
        return [{'index': 2, 'name': 'boxes'}, {'index': 3, 'name': 'classes'}, {'index': 4, 'name': 'scores'}, {'index': 5, 'name': 'count'}]

    def set_tensor(self, index, value):
        assert index == 1
        self.input = value

    def invoke(self):
        pass

    def get_tensor(self, index):
        values = {2: np.array([[[0.1, 0.2, 0.9, 0.8], [0.0, 0.0, 0.5, 0.5], [0.2, 0.3, 0.6, 0.7]]], dtype=np.float32), 3: np.array([[0.0, 3.0, 0.0]], dtype=np.float32), 4: np.array([[0.91, 0.99, 0.2]], dtype=np.float32), 5: np.array([3.0], dtype=np.float32)}
        return values[index]

def test_litert_backend_filters_person_class_and_threshold(tmp_path: Path):
    model = tmp_path / 'model.tflite'
    model.write_bytes(b'fake')
    detector = LiteRtSsdPersonDetector(LiteRtPersonDetectorConfig(model_path=str(model), person_class_id=0, score_threshold=0.45, max_detections=5), interpreter_factory=_FakeInterpreter)
    frame = _Frame(1, 10)
    result = detector.detect(frame)
    assert len(result) == 1
    assert result[0].confidence == pytest.approx(0.91)
    assert result[0].box.xmin == pytest.approx(0.2)
import pytest

def test_person_box_rejects_zero_area():
    with pytest.raises(ValueError):
        PersonBox(0.1, 0.2, 0.1, 0.8)

class _BrokenBackend:

    def detect(self, frame):
        raise RuntimeError('inference failed')

def test_backend_failure_isolated_in_detector_status():
    camera = _Camera()
    detector = NativePersonDetector(camera, _BrokenBackend())
    detector.start()
    camera.publish(_Frame(3, 1000))
    deadline = time.monotonic() + 1.0
    status = detector.get_detection_status()
    while status.running and time.monotonic() < deadline:
        time.sleep(0.005)
        status = detector.get_detection_status()
    detector.stop()
    assert status.last_error is not None
    assert 'inference failed' in status.last_error
    assert detector.get_detection_snapshot() is None

def test_picamera2_rgb888_memory_is_reordered_to_model_rgb(tmp_path: Path):
    model = tmp_path / 'model.tflite'
    model.write_bytes(b'fake')
    created = []

    class CapturingInterpreter(_FakeInterpreter):

        def __init__(self, **kwargs) -> None:
            super().__init__(**kwargs)
            created.append(self)
    detector = LiteRtSsdPersonDetector(LiteRtPersonDetectorConfig(model_path=str(model)), interpreter_factory=CapturingInterpreter)
    payload = bytes([10, 20, 30, 40, 50, 60, 0, 0, 70, 80, 90, 100, 110, 120, 0, 0])
    detector.detect(_Frame(1, 10, image_bytes=payload))
    assert created[0].input[0, 0, 0].tolist() == [30, 20, 10]
    assert created[0].input[0, 0, 1].tolist() == [60, 50, 40]
