"""Native adapter for the protected latest-only scan-matcher result port."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Protocol

from v3.contracts import TickContext

from .live_lidar import (
    LidarHealthReading,
    LidarMatcherDiagnostics,
    LidarPoseReading,
    LidarScanReading,
)


MATCHER_CONTRACT_ID = "R2B4_SCAN_MATCHER_PROCESS_LATEST_ONLY_V1"
MATCHER_CONFIDENCE_MODEL = "R2B4_SCAN_MATCH_CONFIDENCE_V2"
MATCHER_TRANSPORT = "process_latest_only"
ODOMETRY_MODE = "LIDAR_FIRST"
POSE_FRAME_ID = "R2B4_BOOT_ROBOT_MAP"
POSE_FRAME_OWNER = "EKF_POSE_ODOMETRY_SSOT"
POSE_FRAME_YAW = "CCW_POSITIVE_LEFT"


def _finite(value: object, name: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
    ):
        raise ValueError(f"{name} must be finite numeric")
    return float(value)


def _nonnegative_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


def _optional_finite(value: object, name: str) -> float | None:
    if value is None:
        return None
    return _finite(value, name)


def _optional_bool(value: object, name: str) -> bool | None:
    if value is None:
        return None
    if type(value) is not bool:
        raise ValueError(f"{name} must be bool or None")
    return value


def _string_tuple(value: object, name: str) -> tuple[str, ...]:
    if value is None:
        return ()
    if not isinstance(value, (list, tuple)) or any(
        not isinstance(item, str) for item in value
    ):
        raise ValueError(f"{name} must be a string sequence")
    return tuple(value)


def _wrapped_yaw(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


class LatestMatcherResultPort(Protocol):
    """Read-only latest-result surface of the protected matcher owner."""

    def get_matcher_result(self) -> object | None: ...

    def get_runtime_status(self) -> Mapping[str, object]: ...

    def get_raw_scan_snapshot(self) -> object | None: ...

    def stop(self) -> object: ...


@dataclass(frozen=True, slots=True)
class LatestLidarBackendConfig:
    """Immutable freshness and protected-identity contract."""

    maximum_result_age_ns: int
    pose_r_scale: float = 1.0
    odometry_mode: str = ODOMETRY_MODE
    matcher_contract_id: str = MATCHER_CONTRACT_ID
    matcher_confidence_model: str = MATCHER_CONFIDENCE_MODEL
    matcher_transport: str = MATCHER_TRANSPORT
    pose_frame_id: str = POSE_FRAME_ID
    pose_frame_owner: str = POSE_FRAME_OWNER
    pose_frame_yaw: str = POSE_FRAME_YAW
    maximum_future_skew_ns: int = 0

    def __post_init__(self) -> None:
        maximum_age = _nonnegative_int(
            self.maximum_result_age_ns,
            "maximum_result_age_ns",
        )
        if maximum_age == 0:
            raise ValueError("maximum_result_age_ns must be positive")
        future_skew = _nonnegative_int(
            self.maximum_future_skew_ns,
            "maximum_future_skew_ns",
        )
        if future_skew > maximum_age:
            raise ValueError(
                "maximum_future_skew_ns cannot exceed maximum_result_age_ns"
            )
        r_scale = _finite(self.pose_r_scale, "pose_r_scale")
        if not 0.05 <= r_scale <= 20.0:
            raise ValueError("pose_r_scale must be within [0.05, 20]")
        expected = {
            "odometry_mode": ODOMETRY_MODE,
            "matcher_contract_id": MATCHER_CONTRACT_ID,
            "matcher_confidence_model": MATCHER_CONFIDENCE_MODEL,
            "matcher_transport": MATCHER_TRANSPORT,
            "pose_frame_id": POSE_FRAME_ID,
            "pose_frame_owner": POSE_FRAME_OWNER,
            "pose_frame_yaw": POSE_FRAME_YAW,
        }
        for name, value in expected.items():
            if getattr(self, name) != value:
                raise ValueError(f"{name} must be {value}")


class NativeLatestLidarBackend:
    """Close one latest matcher result into the native lidar contract."""

    __slots__ = ("_config", "_port")

    def __init__(
        self,
        port: LatestMatcherResultPort,
        config: LatestLidarBackendConfig,
    ) -> None:
        if not isinstance(config, LatestLidarBackendConfig):
            raise TypeError("config must be LatestLidarBackendConfig")
        for method_name in ("get_matcher_result", "get_runtime_status", "stop"):
            if not callable(getattr(port, method_name, None)):
                raise TypeError(f"port must provide a callable {method_name} method")
        self._port = port
        self._config = config

    @property
    def port(self) -> LatestMatcherResultPort:
        return self._port

    def _runtime_contract_valid(self, status: Mapping[str, object]) -> bool:
        return (
            status.get("matcher_contract_id")
            == self._config.matcher_contract_id
            and status.get("matcher_confidence_model")
            == self._config.matcher_confidence_model
            and status.get("matcher_transport") == self._config.matcher_transport
            and type(status.get("running")) is bool
            and status.get("running") is True
            and type(status.get("matcher_process_alive")) is bool
            and status.get("matcher_process_alive") is True
        )

    def _read_raw_snapshot(
        self,
        context: TickContext,
        status: Mapping[str, object],
    ) -> LidarScanReading:
        reader = getattr(self._port, "get_raw_scan_snapshot", None)
        if not callable(reader):
            reader = getattr(self._port, "get_snapshot", None)
        snapshot = reader() if callable(reader) else None
        physical_runtime_valid = (
            type(status.get("running")) is bool
            and status.get("running") is True
            and type(status.get("driver_connected", True)) is bool
            and status.get("driver_connected", True) is True
        )
        if snapshot is None:
            return LidarScanReading(
                revision=0,
                captured_monotonic_ns=context.monotonic_ns,
                measurement_age_ns=0,
                health="STALE" if physical_runtime_valid else "ERROR",
                stale=physical_runtime_valid,
                timing_valid=physical_runtime_valid,
                point_count=0,
                front_clearance_m=0.0,
                rear_clearance_m=0.0,
                left_clearance_m=0.0,
                right_clearance_m=0.0,
                front_observation_count=0,
                rear_observation_count=0,
                left_observation_count=0,
                right_observation_count=0,
            )
        revision = _nonnegative_int(
            getattr(snapshot, "raw_scan_id", None),
            "raw_scan_id",
        )
        if revision == 0:
            raise ValueError("raw_scan_id must be positive")
        captured_s = _finite(
            getattr(snapshot, "raw_scan_timestamp", None),
            "raw_scan_timestamp",
        )
        captured_ns = int(round(captured_s * 1_000_000_000.0))
        measurement_age_ns = context.monotonic_ns - captured_ns
        snapshot_health = getattr(snapshot, "health", None)
        if snapshot_health not in {"OK", "STALE", "ERROR"}:
            raise ValueError("raw scan health must be OK, STALE or ERROR")
        summary = getattr(snapshot, "summary", None)
        if not isinstance(summary, Mapping):
            raise TypeError("raw scan summary must be a mapping")

        def clearance(primary: str, legacy: str) -> float:
            value = _finite(summary.get(primary, summary.get(legacy)), primary)
            if value < 0.0:
                raise ValueError(f"{primary} must be non-negative")
            return value

        def count(name: str, fallback: int = 0) -> int:
            return _nonnegative_int(summary.get(name, fallback), name)

        point_count = count(
            "raw_safety_valid_point_count",
            len(tuple(getattr(snapshot, "raw_scan", ()) or ())),
        )
        timing_valid = bool(
            physical_runtime_valid
            and measurement_age_ns >= -self._config.maximum_future_skew_ns
        )
        stale = bool(
            timing_valid
            and (
                snapshot_health != "OK"
                or measurement_age_ns > self._config.maximum_result_age_ns
            )
        )
        return LidarScanReading(
            revision=revision,
            captured_monotonic_ns=min(captured_ns, context.monotonic_ns),
            measurement_age_ns=max(0, measurement_age_ns),
            health=str(snapshot_health),
            stale=stale,
            timing_valid=timing_valid,
            point_count=point_count,
            front_clearance_m=clearance("front_clearance_m", "min_dist"),
            rear_clearance_m=clearance("rear_clearance_m", "min_back"),
            left_clearance_m=clearance("left_clearance_m", "avg_left"),
            right_clearance_m=clearance("right_clearance_m", "avg_right"),
            front_observation_count=count(
                "front_observation_count",
                point_count if summary.get("raw_safety_min_dist_point") else 0,
            ),
            rear_observation_count=count(
                "rear_observation_count",
                point_count if clearance("rear_clearance_m", "min_back") > 0.0 else 0,
            ),
            left_observation_count=count(
                "left_observation_count",
                point_count if clearance("left_clearance_m", "avg_left") > 0.0 else 0,
            ),
            right_observation_count=count(
                "right_observation_count",
                point_count if clearance("right_clearance_m", "avg_right") > 0.0 else 0,
            ),
        )

    def read(self, context: TickContext) -> LidarHealthReading:
        if not isinstance(context, TickContext):
            raise TypeError("context must be TickContext")
        result = self._port.get_matcher_result()
        status = self._port.get_runtime_status()
        if not isinstance(status, Mapping):
            raise TypeError("get_runtime_status must return a mapping")
        scan = self._read_raw_snapshot(context, status)
        runtime_contract_valid = self._runtime_contract_valid(status)

        if result is None:
            return LidarHealthReading(
                revision=0,
                captured_monotonic_ns=context.monotonic_ns,
                measurement_age_ns=0,
                confidence=0.0,
                stale=runtime_contract_valid,
                timing_valid=runtime_contract_valid,
                scan=scan,
            )

        revision = _nonnegative_int(
            getattr(result, "matcher_result_id", None),
            "matcher_result_id",
        )
        if revision == 0:
            raise ValueError("matcher_result_id must be positive")
        candidate_id = _nonnegative_int(
            getattr(result, "candidate_id", None),
            "candidate_id",
        )
        source_scan_id = _nonnegative_int(
            getattr(result, "source_raw_scan_id", None),
            "source_raw_scan_id",
        )
        if candidate_id != revision or source_scan_id == 0:
            raise ValueError("matcher result identity is inconsistent")

        captured_s = _finite(getattr(result, "timestamp", None), "timestamp")
        source_s = _finite(
            getattr(result, "source_raw_scan_timestamp", None),
            "source_raw_scan_timestamp",
        )
        if captured_s < 0.0 or source_s < 0.0:
            raise ValueError("matcher timestamps must be non-negative")
        captured_ns = int(round(captured_s * 1_000_000_000.0))
        source_ns = int(round(source_s * 1_000_000_000.0))
        result_age_ns = context.monotonic_ns - captured_ns
        measurement_age_ns = context.monotonic_ns - source_ns

        summary = getattr(result, "summary", None)
        if not isinstance(summary, Mapping):
            raise TypeError("matcher result summary must be a mapping")
        confidence = _finite(
            summary.get("lidar_pose_confidence"),
            "lidar_pose_confidence",
        )
        if not 0.0 <= confidence <= 1.0:
            raise ValueError("lidar_pose_confidence must be within [0, 1]")
        summary_contract_valid = (
            summary.get("matcher_contract_id")
            == self._config.matcher_contract_id
            and summary.get("matcher_confidence_model")
            == self._config.matcher_confidence_model
            and summary.get("matcher_transport") == self._config.matcher_transport
            and summary.get("map_frame_id") == self._config.pose_frame_id
            and summary.get("map_frame_owner") == self._config.pose_frame_owner
            and summary.get("yaw_convention") == self._config.pose_frame_yaw
        )
        timing_valid = (
            runtime_contract_valid
            and summary_contract_valid
            and result_age_ns >= -self._config.maximum_future_skew_ns
            and measurement_age_ns >= -self._config.maximum_future_skew_ns
            and status.get("health") != "ERROR"
        )
        stale = (
            timing_valid
            and (
                status.get("health") != "OK"
                or result_age_ns > self._config.maximum_result_age_ns
            )
        )
        pose = LidarPoseReading(
            x_m=_finite(summary.get("lidar_pose_x"), "lidar_pose_x"),
            y_m=_finite(summary.get("lidar_pose_y"), "lidar_pose_y"),
            yaw_rad=_wrapped_yaw(
                _finite(summary.get("lidar_pose_theta"), "lidar_pose_theta")
            ),
            r_scale=self._config.pose_r_scale,
        )
        quality_value = summary.get("matcher_quality")
        if quality_value is None:
            quality: Mapping[str, object] = {}
        elif isinstance(quality_value, Mapping):
            quality = quality_value
        else:
            raise TypeError("matcher_quality must be a mapping or None")
        reason = summary.get("matcher_reason", "")
        if not isinstance(reason, str):
            raise TypeError("matcher_reason must be a string")
        diagnostics = LidarMatcherDiagnostics(
            candidate_id=candidate_id,
            source_raw_scan_id=source_scan_id,
            source_raw_scan_timestamp_ns=source_ns,
            matcher_reason=reason,
            tracking_ready=_optional_bool(
                summary.get("tracking_ready"),
                "tracking_ready",
            ),
            matcher_timed_out=_optional_bool(
                summary.get("matcher_timed_out"),
                "matcher_timed_out",
            ),
            matcher_degenerate=_optional_bool(
                summary.get("matcher_degenerate"),
                "matcher_degenerate",
            ),
            degeneracy_reasons=_string_tuple(
                summary.get("matcher_degeneracy_reasons"),
                "matcher_degeneracy_reasons",
            ),
            matcher_runtime_ms=_optional_finite(
                summary.get("matcher_runtime_ms"),
                "matcher_runtime_ms",
            ),
            matcher_queue_delay_ms=_optional_finite(
                summary.get("matcher_queue_delay_ms"),
                "matcher_queue_delay_ms",
            ),
            robust_rmse_m=_optional_finite(
                quality.get("robust_rmse_m"),
                "robust_rmse_m",
            ),
            sector_coverage=_optional_finite(
                quality.get("sector_coverage"),
                "sector_coverage",
            ),
            observability_score=_optional_finite(
                quality.get("observability_score"),
                "observability_score",
            ),
            ambiguity_margin=_optional_finite(
                quality.get("ambiguity_margin"),
                "ambiguity_margin",
            ),
        )
        return LidarHealthReading(
            revision=revision,
            captured_monotonic_ns=min(captured_ns, context.monotonic_ns),
            measurement_age_ns=max(0, measurement_age_ns),
            confidence=confidence,
            stale=stale,
            timing_valid=timing_valid,
            pose=pose,
            diagnostics=diagnostics,
            scan=scan,
        )


__all__ = [
    "LatestLidarBackendConfig",
    "LatestMatcherResultPort",
    "NativeLatestLidarBackend",
]
