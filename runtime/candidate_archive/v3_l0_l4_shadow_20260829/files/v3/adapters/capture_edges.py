"""Read-only L0 adapter for canonical Replayer V1 sensor-feedback captures."""

from __future__ import annotations

import json
import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

from v3.contracts import (
    DataField,
    DeviceHealth,
    DeviceHealthState,
    DeviceSample,
    RawDeviceBatch,
    TickContext,
)


REPLAYER_FRAME_SCHEMA_V1 = "R2B4_REPLAYER_FRAME_V1"
ENCODER_DEVICE_ID = "KIT0085_ENCODER"
ESTIMATE_DEVICE_ID = "EKF_POSE_ODOMETRY_SSOT"
LIDAR_DEVICE_ID = "LIDAR_LOCALIZATION"


def _mapping(value: object, field: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{field} must be an object")
    return value


def _finite(value: object, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{field} must be finite")
    return result


def _optional_finite(value: object, field: str) -> float | None:
    if value is None:
        return None
    return _finite(value, field)


def _integer(value: object, field: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError(f"{field} must be a non-negative integer")
    return value


def _boolean(value: object, field: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{field} must be boolean")
    return value


def _optional_boolean(value: object, field: str) -> bool | None:
    if value is None:
        return None
    return _boolean(value, field)


@dataclass(frozen=True, slots=True)
class ReplayerV1CaptureConfig:
    """Immutable edge thresholds; no layer reads live configuration."""

    minimum_encoder_trust: float = 0.3
    minimum_lidar_confidence: float = 0.3
    maximum_lidar_age_ns: int = 250_000_000

    def __post_init__(self) -> None:
        for name in ("minimum_encoder_trust", "minimum_lidar_confidence"):
            value = _finite(getattr(self, name), name)
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be in [0, 1]")
        if (
            not isinstance(self.maximum_lidar_age_ns, int)
            or isinstance(self.maximum_lidar_age_ns, bool)
            or self.maximum_lidar_age_ns < 0
        ):
            raise ValueError("maximum_lidar_age_ns must be a non-negative integer")


class ReplayerV1InputAdapter:
    """Close persisted runtime feedback into immutable L0 batches.

    The adapter understands only the small sensor-feedback projection already
    present in canonical Replayer V1 captures.  It never imports or rehydrates
    a legacy controller, estimator, HAL, or writer.
    """

    __slots__ = ("_config",)

    def __init__(self, config: ReplayerV1CaptureConfig = ReplayerV1CaptureConfig()) -> None:
        self._config = config

    def load(self, frames_path: str | Path) -> tuple[RawDeviceBatch, ...]:
        path = Path(frames_path)
        if path.is_symlink() or not path.is_file():
            raise ValueError("capture frames path must be a regular non-symlink file")

        batches: list[RawDeviceBatch] = []
        previous_capture_seq: int | None = None
        previous_monotonic_ns: int | None = None
        previous_lidar_age_ns: int | None = None
        lidar_sequence = -1

        with path.open("r", encoding="utf-8") as stream:
            for line_no, line in enumerate(stream, start=1):
                if not line.strip():
                    raise ValueError(f"capture line {line_no} is empty")
                try:
                    raw = json.loads(line)
                except json.JSONDecodeError as exc:
                    raise ValueError(f"capture line {line_no} is invalid JSON") from exc
                record = _mapping(raw, f"capture line {line_no}")
                if record.get("schema") != REPLAYER_FRAME_SCHEMA_V1:
                    raise ValueError(f"capture line {line_no} has an unsupported schema")
                capture_seq = _integer(record.get("capture_seq"), "capture_seq")
                monotonic_ns = _integer(record.get("monotonic_ns"), "monotonic_ns")
                if previous_capture_seq is not None and capture_seq != previous_capture_seq + 1:
                    raise ValueError("capture sequence must be contiguous")
                if previous_monotonic_ns is not None and monotonic_ns <= previous_monotonic_ns:
                    raise ValueError("capture monotonic time must increase")

                executor_call = _mapping(record.get("executor_call"), "executor_call")
                kwargs = _mapping(executor_call.get("kwargs"), "executor_call.kwargs")
                feedback_value = kwargs.get("sensor_feedback")
                feedback = (
                    {}
                    if feedback_value is None
                    else _mapping(feedback_value, "sensor_feedback")
                )
                lidar_age_s = _optional_finite(
                    feedback.get("lidar_latest_age_s"),
                    "lidar_latest_age_s",
                )
                if lidar_age_s is not None and lidar_age_s < 0.0:
                    raise ValueError("lidar_latest_age_s cannot be negative")
                lidar_age_ns = (
                    None
                    if lidar_age_s is None
                    else int(round(lidar_age_s * 1_000_000_000.0))
                )
                if lidar_age_ns is not None and (
                    previous_lidar_age_ns is None or lidar_age_ns < previous_lidar_age_ns
                ):
                    lidar_sequence += 1

                context = TickContext(capture_seq, monotonic_ns)
                batches.append(
                    self._batch(
                        context,
                        feedback,
                        lidar_sequence=lidar_sequence,
                        lidar_age_ns=lidar_age_ns,
                    )
                )
                previous_capture_seq = capture_seq
                previous_monotonic_ns = monotonic_ns
                if lidar_age_ns is not None:
                    previous_lidar_age_ns = lidar_age_ns

        if not batches:
            raise ValueError("capture contains no frames")
        return tuple(batches)

    def _batch(
        self,
        context: TickContext,
        feedback: Mapping[str, object],
        *,
        lidar_sequence: int,
        lidar_age_ns: int | None,
    ) -> RawDeviceBatch:
        left_mps = _optional_finite(feedback.get("v_l"), "v_l")
        right_mps = _optional_finite(feedback.get("v_r"), "v_r")
        encoder_trust = _optional_finite(
            feedback.get("encoder_combined_trust"),
            "encoder_combined_trust",
        )
        if encoder_trust is not None and not 0.0 <= encoder_trust <= 1.0:
            raise ValueError("encoder_combined_trust must be in [0, 1]")
        encoder_stale = _optional_boolean(
            feedback.get("encoder_snapshot_stale"),
            "encoder_snapshot_stale",
        )
        encoder_timing_valid = _optional_boolean(
            feedback.get("encoder_timing_valid"),
            "encoder_timing_valid",
        )
        yaw_deg = _optional_finite(feedback.get("current_yaw"), "current_yaw")
        omega_rad_s = _optional_finite(
            feedback.get("ekf_omega_rad_s"),
            "ekf_omega_rad_s",
        )
        lidar_confidence = _optional_finite(
            feedback.get("lidar_latest_confidence"),
            "lidar_latest_confidence",
        )
        if lidar_confidence is not None and not 0.0 <= lidar_confidence <= 1.0:
            raise ValueError("lidar_latest_confidence must be in [0, 1]")

        encoder_complete = all(
            value is not None
            for value in (
                left_mps,
                right_mps,
                encoder_trust,
                encoder_stale,
                encoder_timing_valid,
            )
        )
        if not encoder_complete:
            encoder_health = DeviceHealth(
                ENCODER_DEVICE_ID,
                DeviceHealthState.UNKNOWN,
                "ENCODER_SAMPLE_MISSING",
            )
        elif not encoder_timing_valid:
            encoder_health = DeviceHealth(
                ENCODER_DEVICE_ID,
                DeviceHealthState.FAILED,
                "ENCODER_TIMING_INVALID",
            )
        elif encoder_stale:
            encoder_health = DeviceHealth(
                ENCODER_DEVICE_ID,
                DeviceHealthState.DEGRADED,
                "ENCODER_STALE",
            )
        elif encoder_trust < self._config.minimum_encoder_trust:
            encoder_health = DeviceHealth(
                ENCODER_DEVICE_ID,
                DeviceHealthState.DEGRADED,
                "ENCODER_LOW_TRUST",
            )
        else:
            encoder_health = DeviceHealth(ENCODER_DEVICE_ID, DeviceHealthState.OK)

        if lidar_age_ns is None or lidar_confidence is None:
            lidar_health = DeviceHealth(
                LIDAR_DEVICE_ID,
                DeviceHealthState.UNKNOWN,
                "LIDAR_SAMPLE_MISSING",
            )
        elif lidar_age_ns > self._config.maximum_lidar_age_ns:
            lidar_health = DeviceHealth(
                LIDAR_DEVICE_ID,
                DeviceHealthState.DEGRADED,
                "LIDAR_STALE",
            )
        elif lidar_confidence < self._config.minimum_lidar_confidence:
            lidar_health = DeviceHealth(
                LIDAR_DEVICE_ID,
                DeviceHealthState.DEGRADED,
                "LIDAR_LOW_CONFIDENCE",
            )
        else:
            lidar_health = DeviceHealth(LIDAR_DEVICE_ID, DeviceHealthState.OK)

        heading_complete = yaw_deg is not None and omega_rad_s is not None
        estimate_health = (
            DeviceHealth(ESTIMATE_DEVICE_ID, DeviceHealthState.OK)
            if heading_complete
            else DeviceHealth(
                ESTIMATE_DEVICE_ID,
                DeviceHealthState.UNKNOWN,
                "EKF_HEADING_MISSING",
            )
        )
        samples: list[DeviceSample] = []
        if encoder_complete:
            samples.append(
                DeviceSample(
                    ENCODER_DEVICE_ID,
                    "wheel_velocity",
                    context.tick_id,
                    context.monotonic_ns,
                    (
                        DataField("left_mps", left_mps),
                        DataField("right_mps", right_mps),
                        DataField("trust", encoder_trust),
                    ),
                )
            )
        if heading_complete:
            samples.append(
                DeviceSample(
                    ESTIMATE_DEVICE_ID,
                    "ekf_heading",
                    context.tick_id,
                    context.monotonic_ns,
                    (
                        DataField("yaw_rad", math.radians(yaw_deg)),
                        DataField("omega_rad_s", omega_rad_s),
                        DataField(
                            "confidence",
                            0.0 if lidar_confidence is None else lidar_confidence,
                        ),
                    ),
                )
            )
        if lidar_age_ns is not None and lidar_confidence is not None:
            samples.append(
                DeviceSample(
                    LIDAR_DEVICE_ID,
                    "lidar_health",
                    lidar_sequence,
                    context.monotonic_ns,
                    (
                        DataField("age_ns", lidar_age_ns),
                        DataField("confidence", lidar_confidence),
                    ),
                )
            )
        return RawDeviceBatch(
            context,
            tuple(samples),
            (
                encoder_health,
                estimate_health,
                lidar_health,
            ),
        )


__all__ = [
    "ENCODER_DEVICE_ID",
    "ESTIMATE_DEVICE_ID",
    "LIDAR_DEVICE_ID",
    "REPLAYER_FRAME_SCHEMA_V1",
    "ReplayerV1CaptureConfig",
    "ReplayerV1InputAdapter",
]
