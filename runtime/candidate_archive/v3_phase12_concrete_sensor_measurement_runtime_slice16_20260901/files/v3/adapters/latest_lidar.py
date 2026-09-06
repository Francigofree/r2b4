"""Native adapter for the protected latest-only scan-matcher result port."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Protocol

from v3.contracts import TickContext

from .live_lidar import LidarHealthReading, LidarPoseReading


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


def _wrapped_yaw(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


class LatestMatcherResultPort(Protocol):
    """Read-only latest-result surface of the protected matcher owner."""

    def get_matcher_result(self) -> object | None: ...

    def get_runtime_status(self) -> Mapping[str, object]: ...

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

    def read(self, context: TickContext) -> LidarHealthReading:
        if not isinstance(context, TickContext):
            raise TypeError("context must be TickContext")
        result = self._port.get_matcher_result()
        status = self._port.get_runtime_status()
        if not isinstance(status, Mapping):
            raise TypeError("get_runtime_status must return a mapping")
        runtime_contract_valid = self._runtime_contract_valid(status)

        if result is None:
            return LidarHealthReading(
                revision=0,
                captured_monotonic_ns=context.monotonic_ns,
                measurement_age_ns=0,
                confidence=0.0,
                stale=runtime_contract_valid,
                timing_valid=runtime_contract_valid,
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
        return LidarHealthReading(
            revision=revision,
            captured_monotonic_ns=min(captured_ns, context.monotonic_ns),
            measurement_age_ns=max(0, measurement_age_ns),
            confidence=confidence,
            stale=stale,
            timing_valid=timing_valid,
            pose=pose,
        )


__all__ = [
    "LatestLidarBackendConfig",
    "LatestMatcherResultPort",
    "NativeLatestLidarBackend",
]
