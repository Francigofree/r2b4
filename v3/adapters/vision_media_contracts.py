"""Canonical calibrated JPEG and the identity of the frame that produced it."""
from __future__ import annotations

from dataclasses import asdict, dataclass, field

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

    def to_jsonable(self) -> dict[str, object]:
        return asdict(self)


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
