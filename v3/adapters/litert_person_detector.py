"""LiteRT SSD/EfficientDet person-detection backend for Raspberry Pi 5."""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from v3.adapters.person_detection import PersonBox, PersonDetection


@dataclass(frozen=True, slots=True)
class LiteRtPersonDetectorConfig:
    model_path: str = "models/efficientdet_lite0.tflite"
    person_class_id: int = 0
    score_threshold: float = 0.45
    max_detections: int = 5
    num_threads: int = 2

    def __post_init__(self) -> None:
        if not isinstance(self.model_path, str) or not self.model_path.strip():
            raise ValueError("model_path must be a non-empty string")
        if not isinstance(self.person_class_id, int) or isinstance(self.person_class_id, bool) or self.person_class_id < 0:
            raise ValueError("person_class_id must be a non-negative integer")
        if (
            isinstance(self.score_threshold, bool)
            or not isinstance(self.score_threshold, (int, float))
            or not math.isfinite(self.score_threshold)
            or not 0.0 <= float(self.score_threshold) <= 1.0
        ):
            raise ValueError("score_threshold must be finite and in [0, 1]")
        for value, name in ((self.max_detections, "max_detections"), (self.num_threads, "num_threads")):
            if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")


def litert_person_detector_config_from_mapping(value: Mapping[str, object]) -> LiteRtPersonDetectorConfig:
    if not isinstance(value, Mapping):
        raise TypeError("person detection config must be a mapping")
    allowed = {
        "enabled",
        "provider",
        "model_path",
        "person_class_id",
        "score_threshold",
        "max_detections",
        "num_threads",
    }
    unknown = sorted(set(value) - allowed)
    if unknown:
        raise ValueError("unknown person detection config keys: " + ", ".join(unknown))
    provider = value.get("provider", "litert_ssd")
    if provider != "litert_ssd":
        raise ValueError("person_detection.provider must be litert_ssd")
    return LiteRtPersonDetectorConfig(
        model_path=str(value.get("model_path", "models/efficientdet_lite0.tflite")),
        person_class_id=int(value.get("person_class_id", 0)),
        score_threshold=float(value.get("score_threshold", 0.45)),
        max_detections=int(value.get("max_detections", 5)),
        num_threads=int(value.get("num_threads", 2)),
    )


InterpreterFactory = Callable[..., Any]


def _default_interpreter_factory(**kwargs: object) -> object:
    try:
        from ai_edge_litert.interpreter import Interpreter  # type: ignore[import-not-found]
    except ImportError as exc:  # pragma: no cover - target hardware dependency
        raise RuntimeError(
            "ai-edge-litert is unavailable; install the Raspberry Pi LiteRT runtime first"
        ) from exc
    return Interpreter(**kwargs)


def _repo_relative_path(path_value: str) -> Path:
    path = Path(path_value).expanduser()
    if not path.is_absolute():
        path = Path(__file__).resolve().parents[2] / path
    return path.resolve()


def _nearest_resize_rgb(image: np.ndarray, out_height: int, out_width: int) -> np.ndarray:
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("person detector expects HxWx3 RGB input")
    in_height, in_width, _ = image.shape
    if in_height == out_height and in_width == out_width:
        return np.ascontiguousarray(image)
    y = np.minimum((np.arange(out_height) * in_height // out_height), in_height - 1)
    x = np.minimum((np.arange(out_width) * in_width // out_width), in_width - 1)
    return np.ascontiguousarray(image[y[:, None], x[None, :], :])


def _frame_rgb(frame: object) -> np.ndarray:
    pixel_format = getattr(frame, "pixel_format", None)
    if pixel_format not in {"RGB888", "BGR888"}:
        raise ValueError(f"unsupported person-detector pixel format: {pixel_format!r}")
    width = int(getattr(frame, "width"))
    height = int(getattr(frame, "height"))
    stride_bytes = int(getattr(frame, "stride_bytes"))
    payload = getattr(frame, "image_bytes")
    if not isinstance(payload, bytes):
        raise TypeError("camera image_bytes must be bytes")
    row_bytes = width * 3
    if stride_bytes < row_bytes:
        raise ValueError("camera stride is smaller than width * 3")
    expected = stride_bytes * height
    if len(payload) < expected:
        raise ValueError("camera payload is shorter than stride * height")
    plane = np.frombuffer(payload, dtype=np.uint8, count=expected).reshape(height, stride_bytes)
    image = plane[:, :row_bytes].reshape(height, width, 3)
    # Picamera2/libcamera's packed format names follow DRM/libcamera naming:
    # RGB888 is B,G,R byte order in memory, while BGR888 is R,G,B.  LiteRT
    # EfficientDet expects actual R,G,B tensor channels.
    if pixel_format == "RGB888":
        image = image[:, :, ::-1]
    return np.ascontiguousarray(image)


def _output_name(detail: Mapping[str, object]) -> str:
    return str(detail.get("name", "")).casefold()


class LiteRtSsdPersonDetector:
    """Run one standard TFLite Detection_PostProcess-style detector."""

    __slots__ = (
        "_config",
        "_input_detail",
        "_input_height",
        "_input_width",
        "_interpreter",
        "_output_details",
    )

    def __init__(
        self,
        config: LiteRtPersonDetectorConfig,
        *,
        interpreter_factory: InterpreterFactory = _default_interpreter_factory,
    ) -> None:
        if not isinstance(config, LiteRtPersonDetectorConfig):
            raise TypeError("config must be LiteRtPersonDetectorConfig")
        if not callable(interpreter_factory):
            raise TypeError("interpreter_factory must be callable")
        model_path = _repo_relative_path(config.model_path)
        if model_path.is_symlink() or not model_path.is_file():
            raise RuntimeError(f"person detector model is missing: {model_path}")
        interpreter = interpreter_factory(model_path=str(model_path), num_threads=config.num_threads)
        interpreter.allocate_tensors()
        input_details = tuple(interpreter.get_input_details())
        output_details = tuple(interpreter.get_output_details())
        if len(input_details) != 1:
            raise RuntimeError("person detector model must have exactly one input tensor")
        input_detail = input_details[0]
        shape = tuple(int(item) for item in np.asarray(input_detail["shape"]).tolist())
        if len(shape) != 4 or shape[0] != 1 or shape[3] != 3:
            raise RuntimeError(f"unsupported person detector input shape: {shape}")
        self._config = config
        self._interpreter = interpreter
        self._input_detail = input_detail
        self._output_details = output_details
        self._input_height = shape[1]
        self._input_width = shape[2]

    @property
    def config(self) -> LiteRtPersonDetectorConfig:
        return self._config

    def detect(self, frame: object) -> tuple[PersonDetection, ...]:
        image = _frame_rgb(frame)
        resized = _nearest_resize_rgb(image, self._input_height, self._input_width)
        tensor = self._prepare_input(resized)
        self._interpreter.set_tensor(int(self._input_detail["index"]), tensor)
        self._interpreter.invoke()
        outputs = [
            np.asarray(self._interpreter.get_tensor(int(detail["index"])))
            for detail in self._output_details
        ]
        boxes, classes, scores, count = self._decode_outputs(outputs)
        limit = min(count, len(boxes), len(classes), len(scores))
        detections: list[PersonDetection] = []
        for index in range(limit):
            class_id = int(round(float(classes[index])))
            confidence = float(scores[index])
            if class_id != self._config.person_class_id or confidence < self._config.score_threshold:
                continue
            ymin, xmin, ymax, xmax = (float(item) for item in boxes[index])
            ymin = min(1.0, max(0.0, ymin))
            xmin = min(1.0, max(0.0, xmin))
            ymax = min(1.0, max(0.0, ymax))
            xmax = min(1.0, max(0.0, xmax))
            if ymax <= ymin or xmax <= xmin:
                continue
            detections.append(
                PersonDetection(
                    confidence=confidence,
                    box=PersonBox(ymin=ymin, xmin=xmin, ymax=ymax, xmax=xmax),
                )
            )
        detections.sort(key=lambda item: item.confidence, reverse=True)
        return tuple(detections[: self._config.max_detections])

    def _prepare_input(self, image: np.ndarray) -> np.ndarray:
        dtype = np.dtype(self._input_detail["dtype"])
        if dtype == np.uint8:
            value = image
        elif dtype == np.int8:
            scale, zero_point = self._input_detail.get("quantization", (0.0, 0))
            if not scale:
                raise RuntimeError("int8 person detector input lacks quantization scale")
            value = np.rint(image.astype(np.float32) / float(scale) + int(zero_point))
            value = np.clip(value, -128, 127).astype(np.int8)
        elif dtype in {np.dtype(np.float32), np.dtype(np.float64)}:
            value = image.astype(dtype) / 255.0
        else:
            raise RuntimeError(f"unsupported person detector input dtype: {dtype}")
        return np.expand_dims(np.ascontiguousarray(value), axis=0)

    def _decode_outputs(
        self, outputs: list[np.ndarray]
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, int]:
        flattened = [np.squeeze(item) for item in outputs]
        boxes_index = next(
            (i for i, item in enumerate(flattened) if item.ndim == 2 and item.shape[-1] == 4),
            None,
        )
        if boxes_index is None:
            raise RuntimeError("person detector output does not contain [N,4] boxes")
        boxes = np.asarray(flattened[boxes_index], dtype=np.float32)

        count = len(boxes)
        for detail, item in zip(self._output_details, flattened):
            if item.size == 1 and ("count" in _output_name(detail) or "num" in _output_name(detail)):
                count = max(0, int(round(float(item.reshape(-1)[0]))))
                break

        one_dim: list[tuple[int, np.ndarray]] = []
        for i, item in enumerate(flattened):
            if i == boxes_index or item.size == 1:
                continue
            vector = np.asarray(item).reshape(-1)
            if vector.size >= len(boxes):
                one_dim.append((i, vector))
        if len(one_dim) < 2:
            raise RuntimeError("person detector output lacks class/score vectors")

        score_index = next(
            (i for i, _ in one_dim if "score" in _output_name(self._output_details[i])),
            None,
        )
        class_index = next(
            (i for i, _ in one_dim if "class" in _output_name(self._output_details[i])),
            None,
        )
        if score_index is None:
            score_index = next(
                (
                    i
                    for i, vector in one_dim
                    if np.all(np.isfinite(vector[: len(boxes)]))
                    and np.all((vector[: len(boxes)] >= -1e-6) & (vector[: len(boxes)] <= 1.000001))
                    and not np.allclose(vector[: len(boxes)], np.rint(vector[: len(boxes)]), atol=1e-6)
                ),
                None,
            )
        if class_index is None:
            class_index = next(
                (
                    i
                    for i, vector in one_dim
                    if i != score_index
                    and np.allclose(vector[: len(boxes)], np.rint(vector[: len(boxes)]), atol=1e-4)
                ),
                None,
            )
        if score_index is None or class_index is None or score_index == class_index:
            raise RuntimeError("cannot identify person detector class/score outputs")
        scores = np.asarray(flattened[score_index]).reshape(-1).astype(np.float32)
        classes = np.asarray(flattened[class_index]).reshape(-1).astype(np.float32)
        return boxes, classes, scores, count


__all__ = [
    "LiteRtPersonDetectorConfig",
    "LiteRtSsdPersonDetector",
    "litert_person_detector_config_from_mapping",
]
