"""Canonical calibrated JPEG and the identity of the frame that produced it."""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
import math

MAX_JPEG_BYTES = 4 * 1024 * 1024
MAX_METADATA_BYTES = 4096


@dataclass(frozen=True, slots=True)
class CameraJpegMetadata:
    source_sequence: int
    sensor_timestamp_ns: int
    measurement_monotonic_ns: int
    completed_monotonic_ns: int
    calibration_id: str
    stream: str
    width: int
    height: int
    rectified_K: tuple[tuple[float, float, float], ...]
    owner_generation: str
    calibration_state: str = "CALIBRATED"

    def __post_init__(self) -> None:
        for name in ("source_sequence", "width", "height"):
            if type(getattr(self, name)) is not int or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be a positive integer")
        for name in ("sensor_timestamp_ns", "measurement_monotonic_ns", "completed_monotonic_ns"):
            if type(getattr(self, name)) is not int or getattr(self, name) < 0:
                raise ValueError(f"{name} must be a nonnegative integer")
        if self.completed_monotonic_ns < self.measurement_monotonic_ns:
            raise ValueError("JPEG completion precedes measurement")
        if self.calibration_state != "CALIBRATED" or not isinstance(self.calibration_id, str) or not self.calibration_id:
            raise ValueError("JPEG must have canonical calibration identity")
        if self.stream not in {"lores", "main"}:
            raise ValueError("stream must be lores or main")
        if not isinstance(self.owner_generation, str) or not self.owner_generation:
            raise ValueError("JPEG owner generation is missing")
        matrix = self.rectified_K
        if (not isinstance(matrix, tuple) or len(matrix) != 3
                or any(not isinstance(row, tuple) or len(row) != 3 for row in matrix)
                or any(type(v) not in (int, float) or not math.isfinite(v) for row in matrix for v in row)
                or matrix[0][0] <= 0 or matrix[1][1] <= 0
                or tuple(matrix[2]) != (0, 0, 1)):
            raise ValueError("JPEG rectified_K must be valid immutable camera intrinsics")

    def to_jsonable(self) -> dict[str, object]:
        return asdict(self)

    @classmethod
    def from_jsonable(cls, value: dict) -> CameraJpegMetadata:
        value = dict(value)
        value["rectified_K"] = tuple(tuple(row) for row in value["rectified_K"])
        return cls(**value)

    def require_fresh(self, now_ns: int, *, generation: str, maximum_age_ns: int) -> None:
        if self.owner_generation != generation:
            raise RuntimeError("VISION_GENERATION_MISMATCH")
        if not 0 <= now_ns - self.measurement_monotonic_ns <= maximum_age_ns:
            raise RuntimeError("VISION_FRAME_STALE")
        if self.completed_monotonic_ns > now_ns:
            raise RuntimeError("VISION_LINEAGE_INVALID")


@dataclass(frozen=True, slots=True)
class VisionJpeg:
    image_bytes: bytes = field(repr=False)
    metadata: CameraJpegMetadata

    def __post_init__(self) -> None:
        if not isinstance(self.metadata, CameraJpegMetadata):
            raise TypeError("metadata must be CameraJpegMetadata")
        validate_jpeg(self.image_bytes)


def validate_jpeg(payload: bytes) -> None:
    if not isinstance(payload, bytes) or not 1 <= len(payload) <= MAX_JPEG_BYTES:
        raise ValueError("JPEG outside bounded size")
    if not payload.startswith(b"\xff\xd8") or not payload.endswith(b"\xff\xd9"):
        raise ValueError("payload is not a complete JPEG")
