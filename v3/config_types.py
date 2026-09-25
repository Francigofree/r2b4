"""Immutable runtime-edge policy values; file loading belongs to ConfigResolver."""
from __future__ import annotations

from dataclasses import dataclass, fields
import math



def _positive_fields(value: object) -> None:
    for field in fields(value):
        number = getattr(value, field.name)
        if isinstance(number, bool) or not isinstance(number, (int, float)) or not math.isfinite(number) or number <= 0:
            raise ValueError(f"{type(value).__name__}.{field.name} must be finite and positive")


@dataclass(frozen=True, slots=True)
class EncoderProcessConfig:
    sample_period_ns: int
    queue_capacity: int
    ready_timeout_s: float
    stop_timeout_s: float

    def __post_init__(self) -> None:
        _positive_fields(self)


@dataclass(frozen=True, slots=True)
class ImuProcessConfig:
    sample_period_ns: int
    history_size: int
    ready_timeout_s: float
    stop_timeout_s: float

    def __post_init__(self) -> None:
        _positive_fields(self)


@dataclass(frozen=True, slots=True)
class LidarProcessConfig:
    pose_history_capacity: int
    state_queue_capacity: int
    raw_queue_capacity: int
    state_heartbeat_ns: int
    ready_timeout_s: float
    stop_timeout_s: float
    pose_lock_timeout_s: float
    raw_end_timeout_s: float

    def __post_init__(self) -> None:
        _positive_fields(self)


@dataclass(frozen=True, slots=True)
class PlannerProcessConfig:
    result_buffer_capacity: int
    collector_poll_s: float
    stop_enqueue_timeout_s: float
    stop_timeout_s: float

    def __post_init__(self) -> None:
        _positive_fields(self)



@dataclass(frozen=True, slots=True)
class CommandIngressPolicy:
    maximum_ttl_ns: int
    maximum_future_skew_ns: int
    maximum_linear_speed_mps: float
    maximum_angular_speed_rad_s: float
    maximum_file_bytes: int
    reader_poll_s: float
    reader_stop_timeout_s: float

    def __post_init__(self) -> None:
        _positive_fields(self)
