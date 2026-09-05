"""Native V3 physical-scan, safety and optional localization source."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol

from v3.contracts import (
    DataField,
    DeviceHealth,
    DeviceHealthState,
    DeviceSample,
    TickContext,
)

from .live_inputs import LiveDeviceSnapshot


def _finite(value: object, name: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
    ):
        raise ValueError(f"{name} must be finite numeric")
    return float(value)


def _nonnegative_integer(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


def _optional_finite(value: object, name: str) -> float | None:
    if value is None:
        return None
    return _finite(value, name)


@dataclass(frozen=True, slots=True)
class NativeLidarConfig:
    """Immutable identity, pose frame and gates for one lidar matcher source."""

    device_id: str
    minimum_confidence: float
    maximum_measurement_age_ns: int
    pose_frame_id: str = "R2B4_BOOT_ROBOT_MAP"

    def __post_init__(self) -> None:
        if not isinstance(self.device_id, str) or not self.device_id.strip():
            raise ValueError("device_id must be a non-empty string")
        confidence = _finite(self.minimum_confidence, "minimum_confidence")
        if not 0.0 <= confidence <= 1.0:
            raise ValueError("minimum_confidence must be within [0, 1]")
        _nonnegative_integer(
            self.maximum_measurement_age_ns,
            "maximum_measurement_age_ns",
        )
        if not isinstance(self.pose_frame_id, str) or not self.pose_frame_id.strip():
            raise ValueError("pose_frame_id must be a non-empty string")


@dataclass(frozen=True, slots=True)
class LidarPoseReading:
    """One matcher-owned absolute pose in the configured V3 pose frame."""

    x_m: float
    y_m: float
    yaw_rad: float
    r_scale: float = 1.0

    def __post_init__(self) -> None:
        _finite(self.x_m, "x_m")
        _finite(self.y_m, "y_m")
        _finite(self.yaw_rad, "yaw_rad")
        r_scale = _finite(self.r_scale, "r_scale")
        if not 0.05 <= r_scale <= 20.0:
            raise ValueError("r_scale must be within [0.05, 20]")


@dataclass(frozen=True, slots=True)
class LidarMatcherDiagnostics:
    """Passive matcher identity and quality evidence for one result revision."""

    candidate_id: int
    source_raw_scan_id: int
    source_raw_scan_timestamp_ns: int
    matcher_reason: str = ""
    tracking_ready: bool | None = None
    matcher_timed_out: bool | None = None
    matcher_degenerate: bool | None = None
    degeneracy_reasons: tuple[str, ...] = ()
    matcher_runtime_ms: float | None = None
    matcher_queue_delay_ms: float | None = None
    robust_rmse_m: float | None = None
    sector_coverage: float | None = None
    observability_score: float | None = None
    ambiguity_margin: float | None = None

    def __post_init__(self) -> None:
        _nonnegative_integer(self.candidate_id, "candidate_id")
        if self.candidate_id == 0:
            raise ValueError("candidate_id must be positive")
        _nonnegative_integer(self.source_raw_scan_id, "source_raw_scan_id")
        if self.source_raw_scan_id == 0:
            raise ValueError("source_raw_scan_id must be positive")
        _nonnegative_integer(
            self.source_raw_scan_timestamp_ns,
            "source_raw_scan_timestamp_ns",
        )
        if not isinstance(self.matcher_reason, str):
            raise ValueError("matcher_reason must be a string")
        for value, name in (
            (self.tracking_ready, "tracking_ready"),
            (self.matcher_timed_out, "matcher_timed_out"),
            (self.matcher_degenerate, "matcher_degenerate"),
        ):
            if value is not None and type(value) is not bool:
                raise ValueError(f"{name} must be bool or None")
        if not isinstance(self.degeneracy_reasons, tuple) or any(
            not isinstance(item, str) for item in self.degeneracy_reasons
        ):
            raise ValueError("degeneracy_reasons must be a tuple of strings")
        for value, name in (
            (self.matcher_runtime_ms, "matcher_runtime_ms"),
            (self.matcher_queue_delay_ms, "matcher_queue_delay_ms"),
            (self.robust_rmse_m, "robust_rmse_m"),
        ):
            parsed = _optional_finite(value, name)
            if parsed is not None and parsed < 0.0:
                raise ValueError(f"{name} must be non-negative or None")
        for value, name in (
            (self.sector_coverage, "sector_coverage"),
            (self.observability_score, "observability_score"),
            (self.ambiguity_margin, "ambiguity_margin"),
        ):
            parsed = _optional_finite(value, name)
            if parsed is not None and not 0.0 <= parsed <= 1.0:
                raise ValueError(f"{name} must be within [0, 1] or None")


@dataclass(frozen=True, slots=True)
class LidarScanReading:
    """One physical scan and its matcher-independent sector safety result."""

    revision: int
    captured_monotonic_ns: int
    measurement_age_ns: int
    health: str
    stale: bool
    timing_valid: bool
    point_count: int
    front_clearance_m: float
    rear_clearance_m: float
    left_clearance_m: float
    right_clearance_m: float
    front_observation_count: int
    rear_observation_count: int
    left_observation_count: int
    right_observation_count: int

    def __post_init__(self) -> None:
        _nonnegative_integer(self.revision, "revision")
        _nonnegative_integer(self.captured_monotonic_ns, "captured_monotonic_ns")
        _nonnegative_integer(self.measurement_age_ns, "measurement_age_ns")
        if self.health not in {"OK", "STALE", "ERROR"}:
            raise ValueError("health must be OK, STALE or ERROR")
        if type(self.stale) is not bool or type(self.timing_valid) is not bool:
            raise ValueError("stale and timing_valid must be bool")
        for value, name in (
            (self.point_count, "point_count"),
            (self.front_observation_count, "front_observation_count"),
            (self.rear_observation_count, "rear_observation_count"),
            (self.left_observation_count, "left_observation_count"),
            (self.right_observation_count, "right_observation_count"),
        ):
            _nonnegative_integer(value, name)
        for value, name in (
            (self.front_clearance_m, "front_clearance_m"),
            (self.rear_clearance_m, "rear_clearance_m"),
            (self.left_clearance_m, "left_clearance_m"),
            (self.right_clearance_m, "right_clearance_m"),
        ):
            parsed = _finite(value, name)
            if parsed < 0.0:
                raise ValueError(f"{name} must be non-negative")


@dataclass(frozen=True, slots=True)
class LidarHealthReading:
    """One physical scan plus the latest optional localization result."""

    revision: int
    captured_monotonic_ns: int
    measurement_age_ns: int
    confidence: float
    stale: bool
    timing_valid: bool
    pose: LidarPoseReading | None = None
    diagnostics: LidarMatcherDiagnostics | None = None
    scan: LidarScanReading | None = None

    def __post_init__(self) -> None:
        _nonnegative_integer(self.revision, "revision")
        _nonnegative_integer(self.captured_monotonic_ns, "captured_monotonic_ns")
        _nonnegative_integer(self.measurement_age_ns, "measurement_age_ns")
        confidence = _finite(self.confidence, "confidence")
        if not 0.0 <= confidence <= 1.0:
            raise ValueError("confidence must be within [0, 1]")
        if type(self.stale) is not bool:
            raise ValueError("stale must be bool")
        if type(self.timing_valid) is not bool:
            raise ValueError("timing_valid must be bool")
        if self.pose is not None and not isinstance(self.pose, LidarPoseReading):
            raise ValueError("pose must be LidarPoseReading or None")
        if self.diagnostics is not None and not isinstance(
            self.diagnostics,
            LidarMatcherDiagnostics,
        ):
            raise ValueError("diagnostics must be LidarMatcherDiagnostics or None")
        if (
            self.diagnostics is not None
            and self.diagnostics.candidate_id != self.revision
        ):
            raise ValueError("diagnostics candidate_id must match revision")
        if self.scan is not None and not isinstance(self.scan, LidarScanReading):
            raise ValueError("scan must be LidarScanReading or None")


class LidarHealthBackend(Protocol):
    """Injected edge backend; process, queue and hardware owners stay outside."""

    def read(self, context: TickContext) -> LidarHealthReading: ...


class NativeLidarSource:
    """Split one acquired scan into independent native V3 capabilities."""

    __slots__ = ("_backend", "_config")

    def __init__(
        self,
        backend: LidarHealthBackend,
        config: NativeLidarConfig,
    ) -> None:
        if not isinstance(config, NativeLidarConfig):
            raise TypeError("config must be NativeLidarConfig")
        self._backend = backend
        self._config = config

    @property
    def device_id(self) -> str:
        return self._config.device_id

    def read(self, context: TickContext) -> LiveDeviceSnapshot:
        if not isinstance(context, TickContext):
            raise TypeError("context must be TickContext")
        reading = self._backend.read(context)
        if not isinstance(reading, LidarHealthReading):
            raise TypeError("lidar backend must return LidarHealthReading")

        scan = reading.scan
        # The fallback exists only for isolated pre-native test edges. The
        # production backend always supplies a scan reading, including startup.
        if scan is None:
            physical_revision = reading.revision
            physical_captured_ns = reading.captured_monotonic_ns
            physical_age_ns = reading.measurement_age_ns
            physical_stale = reading.stale
            physical_timing_valid = reading.timing_valid
            physical_health = "OK"
        else:
            physical_revision = scan.revision
            physical_captured_ns = scan.captured_monotonic_ns
            physical_age_ns = scan.measurement_age_ns
            physical_stale = scan.stale
            physical_timing_valid = scan.timing_valid
            physical_health = scan.health

        if not physical_timing_valid or physical_health == "ERROR":
            health = DeviceHealth(
                self.device_id,
                DeviceHealthState.FAILED,
                "LIDAR_TIMING_INVALID",
            )
        elif (
            physical_stale
            or physical_health == "STALE"
            or physical_age_ns > self._config.maximum_measurement_age_ns
        ):
            health = DeviceHealth(
                self.device_id,
                DeviceHealthState.DEGRADED,
                "LIDAR_STALE",
            )
        else:
            health = DeviceHealth(self.device_id, DeviceHealthState.OK)

        health_sample = DeviceSample(
            device_id=self.device_id,
            kind="lidar_health",
            sequence=physical_revision,
            captured_monotonic_ns=physical_captured_ns,
            values=(
                (
                    DataField("age_ns", physical_age_ns),
                    DataField("point_count", scan.point_count),
                )
                if scan is not None
                else (
                    DataField("age_ns", physical_age_ns),
                    DataField("confidence", reading.confidence),
                )
            ),
        )
        samples = [health_sample]
        if scan is not None and scan.revision > 0:
            samples.append(
                DeviceSample(
                    device_id=self.device_id,
                    kind="lidar_safety_clearance",
                    sequence=scan.revision,
                    captured_monotonic_ns=scan.captured_monotonic_ns,
                    values=(
                        DataField("age_ns", scan.measurement_age_ns),
                        DataField("front_clearance_m", scan.front_clearance_m),
                        DataField("rear_clearance_m", scan.rear_clearance_m),
                        DataField("left_clearance_m", scan.left_clearance_m),
                        DataField("right_clearance_m", scan.right_clearance_m),
                        DataField(
                            "front_observation_count",
                            scan.front_observation_count,
                        ),
                        DataField(
                            "rear_observation_count",
                            scan.rear_observation_count,
                        ),
                        DataField(
                            "left_observation_count",
                            scan.left_observation_count,
                        ),
                        DataField(
                            "right_observation_count",
                            scan.right_observation_count,
                        ),
                    ),
                )
            )
        localization_usable = bool(
            reading.pose is not None
            and reading.timing_valid
            and not reading.stale
            and reading.measurement_age_ns
            <= self._config.maximum_measurement_age_ns
            and reading.confidence >= self._config.minimum_confidence
        )
        has_localization = bool(
            reading.revision > 0
            or reading.pose is not None
            or reading.diagnostics is not None
        )
        if has_localization:
            samples.append(
                DeviceSample(
                    device_id=self.device_id,
                    kind="lidar_localization_health",
                    sequence=reading.revision,
                    captured_monotonic_ns=reading.captured_monotonic_ns,
                    values=(
                        DataField("age_ns", reading.measurement_age_ns),
                        DataField("confidence", reading.confidence),
                        DataField("timing_valid", reading.timing_valid),
                        DataField("stale", reading.stale),
                        DataField("usable", localization_usable),
                    ),
                )
            )
        if reading.diagnostics is not None:
            diagnostics = reading.diagnostics
            samples.append(
                DeviceSample(
                    device_id=self.device_id,
                    kind="lidar_matcher_diagnostics",
                    sequence=reading.revision,
                    captured_monotonic_ns=reading.captured_monotonic_ns,
                    values=(
                        DataField("candidate_id", diagnostics.candidate_id),
                        DataField(
                            "source_raw_scan_id",
                            diagnostics.source_raw_scan_id,
                        ),
                        DataField(
                            "source_raw_scan_timestamp_ns",
                            diagnostics.source_raw_scan_timestamp_ns,
                        ),
                        DataField("matcher_reason", diagnostics.matcher_reason),
                        DataField("tracking_ready", diagnostics.tracking_ready),
                        DataField(
                            "matcher_timed_out",
                            diagnostics.matcher_timed_out,
                        ),
                        DataField(
                            "matcher_degenerate",
                            diagnostics.matcher_degenerate,
                        ),
                        DataField(
                            "degeneracy_reasons",
                            "|".join(diagnostics.degeneracy_reasons),
                        ),
                        DataField(
                            "matcher_runtime_ms",
                            diagnostics.matcher_runtime_ms,
                        ),
                        DataField(
                            "matcher_queue_delay_ms",
                            diagnostics.matcher_queue_delay_ms,
                        ),
                        DataField("robust_rmse_m", diagnostics.robust_rmse_m),
                        DataField("sector_coverage", diagnostics.sector_coverage),
                        DataField(
                            "observability_score",
                            diagnostics.observability_score,
                        ),
                        DataField("ambiguity_margin", diagnostics.ambiguity_margin),
                    ),
                )
            )
        if localization_usable:
            assert reading.pose is not None
            samples.append(
                DeviceSample(
                    device_id=self.device_id,
                    kind="lidar_pose",
                    sequence=reading.revision,
                    captured_monotonic_ns=reading.captured_monotonic_ns,
                    values=(
                        DataField("frame_id", self._config.pose_frame_id),
                        DataField("x_m", reading.pose.x_m),
                        DataField("y_m", reading.pose.y_m),
                        DataField("yaw_rad", reading.pose.yaw_rad),
                        DataField("confidence", reading.confidence),
                        DataField("r_scale", reading.pose.r_scale),
                    ),
                )
            )
        return LiveDeviceSnapshot(context, health, tuple(samples))


__all__ = [
    "LidarHealthBackend",
    "LidarHealthReading",
    "LidarMatcherDiagnostics",
    "LidarPoseReading",
    "LidarScanReading",
    "NativeLidarConfig",
    "NativeLidarSource",
]
