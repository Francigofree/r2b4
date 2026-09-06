"""L4 deterministic shadow world state and the empty STOP-only path."""

from __future__ import annotations

import math
from dataclasses import dataclass

from v3.contracts import AdmittedFrame, ObstacleTrack, Observation, RobotEstimate, WorldSnapshot


def _values(observation: Observation) -> dict[str, object]:
    return {field.key: field.value for field in observation.values}


def _number(values: dict[str, object], key: str) -> float:
    value = values.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"obstacle/lidar field {key} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"obstacle/lidar field {key} must be finite")
    return result


@dataclass(frozen=True, slots=True)
class WorldModelConfig:
    max_track_age_ns: int = 500_000_000

    def __post_init__(self) -> None:
        if (
            not isinstance(self.max_track_age_ns, int)
            or isinstance(self.max_track_age_ns, bool)
            or self.max_track_age_ns <= 0
        ):
            raise ValueError("max_track_age_ns must be a positive integer")


class ShadowWorldModel:
    """Own lidar revision/freshness and typed obstacle-track history."""

    __slots__ = (
        "_config",
        "_last_lidar_measurement_ns",
        "_last_lidar_sequence",
        "_map_revision",
        "_tracks",
    )

    def __init__(self, config: WorldModelConfig = WorldModelConfig()) -> None:
        self._config = config
        self._last_lidar_measurement_ns: int | None = None
        self._last_lidar_sequence: int | None = None
        self._map_revision = 0
        self._tracks: dict[str, tuple[ObstacleTrack, int]] = {}

    def __call__(self, frame: AdmittedFrame, estimate: RobotEstimate) -> WorldSnapshot:
        if frame.context != estimate.context:
            raise ValueError("L4 inputs must use the same tick context")

        lidar = tuple(item for item in frame.accepted if item.kind == "lidar_health")
        if len(lidar) > 1:
            raise ValueError("L4 accepts at most one lidar_health observation per tick")
        if lidar:
            values = _values(lidar[0])
            age_ns = _number(values, "age_ns")
            if "point_count" in values:
                point_count = _number(values, "point_count")
                quality_valid = point_count >= 0.0
                measurement_ns = lidar[0].captured_monotonic_ns
            else:
                # Replay V1 captures predate the split physical/localization
                # samples. Preserve their closed interpretation without using
                # localization confidence as new device-health authority.
                confidence = _number(values, "confidence")
                quality_valid = 0.0 <= confidence <= 1.0
                measurement_ns = max(
                    0,
                    lidar[0].captured_monotonic_ns - int(round(age_ns)),
                )
            if age_ns < 0.0 or not quality_valid:
                raise ValueError("lidar health values are outside their physical range")
            if (
                self._last_lidar_sequence is not None
                and lidar[0].source_sequence < self._last_lidar_sequence
            ):
                raise ValueError("L4 lidar sequence must not move backwards")
            if (
                self._last_lidar_measurement_ns is not None
                and measurement_ns < self._last_lidar_measurement_ns
            ):
                raise ValueError("L4 lidar measurement time must not move backwards")
            if self._last_lidar_sequence is None or (
                lidar[0].source_sequence > self._last_lidar_sequence
            ):
                self._map_revision += 1
                self._last_lidar_sequence = lidar[0].source_sequence
                self._last_lidar_measurement_ns = measurement_ns

        changed_tracks = False
        for observation in frame.accepted:
            if observation.kind != "obstacle_track":
                continue
            values = _values(observation)
            track_id = values.get("track_id")
            if not isinstance(track_id, str) or not track_id:
                raise ValueError("obstacle_track.track_id must be a non-empty string")
            track = ObstacleTrack(
                track_id=track_id,
                x_m=_number(values, "x_m"),
                y_m=_number(values, "y_m"),
                radius_m=_number(values, "radius_m"),
                vx_mps=_number(values, "vx_mps"),
                vy_mps=_number(values, "vy_mps"),
                confidence=_number(values, "confidence"),
            )
            self._tracks[track_id] = (track, observation.captured_monotonic_ns)
            changed_tracks = True

        expired = tuple(
            track_id
            for track_id, (_, captured_ns) in self._tracks.items()
            if frame.context.monotonic_ns - captured_ns > self._config.max_track_age_ns
        )
        for track_id in expired:
            del self._tracks[track_id]
        if changed_tracks or expired:
            self._map_revision += 1

        if self._last_lidar_measurement_ns is None:
            raise ValueError("L4 requires an admitted lidar_health observation before output")
        freshness_ns = max(
            0,
            frame.context.monotonic_ns - self._last_lidar_measurement_ns,
        )
        tracks = tuple(self._tracks[key][0] for key in sorted(self._tracks))
        return WorldSnapshot(
            frame.context,
            frame_id=estimate.frame_id,
            map_revision=self._map_revision,
            obstacle_tracks=tracks,
            freshness_ns=freshness_ns,
        )


def build_empty_world(frame: AdmittedFrame, estimate: RobotEstimate) -> WorldSnapshot:
    return WorldSnapshot(
        frame.context,
        frame_id=estimate.frame_id,
        map_revision=0,
        obstacle_tracks=(),
        freshness_ns=0,
    )


__all__ = ["ShadowWorldModel", "WorldModelConfig", "build_empty_world"]
