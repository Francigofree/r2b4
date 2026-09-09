"""L4 deterministic shadow world state and the empty STOP-only path."""

from __future__ import annotations

import math
from dataclasses import dataclass

from v3.contracts import (
    AdmittedFrame,
    CostmapCell,
    DataField,
    ObstacleTrack,
    Observation,
    RobotEstimate,
    RollingLocalCostmap,
    WorldSnapshot,
)


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


def _integer(values: dict[str, object], key: str) -> int:
    value = values.get(key)
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError(f"obstacle/lidar field {key} must be a non-negative integer")
    return value


@dataclass(frozen=True, slots=True)
class WorldModelConfig:
    max_track_age_ns: int = 500_000_000
    local_costmap_resolution_m: float = 0.10
    local_costmap_radius_m: float = 2.50
    local_costmap_max_cell_age_ns: int = 750_000_000
    local_costmap_max_cells: int = 1_200
    local_costmap_max_points_per_scan: int = 96

    def __post_init__(self) -> None:
        for name in ("max_track_age_ns", "local_costmap_max_cell_age_ns"):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")


@dataclass(frozen=True, slots=True)
class WorldModelStateCheckpoint:
    last_lidar_measurement_ns: int | None
    last_lidar_sequence: int | None
    map_revision: int
    tracks: tuple[tuple[ObstacleTrack, int], ...]
    last_local_measurement_ns: int | None
    last_local_sequence: int | None
    last_local_values: tuple[DataField, ...] | None
    costmap_revision: int
    cells: tuple[tuple[int, int, int, int], ...]
        for name in ("local_costmap_resolution_m", "local_costmap_radius_m"):
            value = getattr(self, name)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value <= 0.0
            ):
                raise ValueError(f"{name} must be finite and positive")
        for name in ("local_costmap_max_cells", "local_costmap_max_points_per_scan"):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")


class ShadowWorldModel:
    """Own lidar revision, obstacle history and one rolling local costmap."""

    __slots__ = (
        "_config",
        "_costmap_revision",
        "_cells",
        "_last_lidar_measurement_ns",
        "_last_lidar_sequence",
        "_last_local_measurement_ns",
        "_last_local_sequence",
        "_last_local_values",
        "_map_revision",
        "_tracks",
    )

    def __init__(self, config: WorldModelConfig = WorldModelConfig()) -> None:
        self._config = config
        self._last_lidar_measurement_ns: int | None = None
        self._last_lidar_sequence: int | None = None
        self._map_revision = 0
        self._tracks: dict[str, tuple[ObstacleTrack, int]] = {}
        self._last_local_measurement_ns: int | None = None
        self._last_local_sequence: int | None = None
        self._last_local_values: tuple[object, ...] | None = None
        self._costmap_revision = 0
        self._cells: dict[tuple[int, int], tuple[int, int]] = {}

    def checkpoint(self) -> WorldModelStateCheckpoint:
        return WorldModelStateCheckpoint(
            self._last_lidar_measurement_ns,
            self._last_lidar_sequence,
            self._map_revision,
            tuple(self._tracks[key] for key in sorted(self._tracks)),
            self._last_local_measurement_ns,
            self._last_local_sequence,
            self._last_local_values,
            self._costmap_revision,
            tuple(
                (x_index, y_index, count, captured_ns)
                for (x_index, y_index), (count, captured_ns) in sorted(
                    self._cells.items()
                )
            ),
        )

    def restore(self, checkpoint: WorldModelStateCheckpoint) -> None:
        if not isinstance(checkpoint, WorldModelStateCheckpoint):
            raise TypeError("checkpoint must be WorldModelStateCheckpoint")
        self._last_lidar_measurement_ns = checkpoint.last_lidar_measurement_ns
        self._last_lidar_sequence = checkpoint.last_lidar_sequence
        self._map_revision = checkpoint.map_revision
        self._tracks = {
            track.track_id: (track, captured_ns)
            for track, captured_ns in checkpoint.tracks
        }
        self._last_local_measurement_ns = checkpoint.last_local_measurement_ns
        self._last_local_sequence = checkpoint.last_local_sequence
        self._last_local_values = checkpoint.last_local_values
        self._costmap_revision = checkpoint.costmap_revision
        self._cells = {
            (x_index, y_index): (count, captured_ns)
            for x_index, y_index, count, captured_ns in checkpoint.cells
        }

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

        local = tuple(
            item for item in frame.accepted if item.kind == "lidar_local_points"
        )
        if len(local) > 1:
            raise ValueError("L4 accepts at most one lidar_local_points observation per tick")
        costmap_changed = False
        if local:
            observation = local[0]
            values = _values(observation)
            if values.get("frame_id") != "ROBOT_BASE":
                raise ValueError("lidar local points must use the ROBOT_BASE frame")
            point_count = _integer(values, "point_count")
            if point_count > self._config.local_costmap_max_points_per_scan:
                raise ValueError("lidar local point_count exceeds the configured bound")
            expected_keys = {"frame_id", "point_count"}
            expected_keys.update(
                f"point_{index:03d}_{suffix}"
                for index in range(point_count)
                for suffix in ("x_m", "y_m", "quality")
            )
            if set(values) != expected_keys:
                raise ValueError("lidar local point fields do not match point_count")
            if (
                self._last_local_sequence is not None
                and observation.source_sequence < self._last_local_sequence
            ):
                raise ValueError("L4 local perception sequence must not move backwards")
            if (
                self._last_local_measurement_ns is not None
                and observation.captured_monotonic_ns
                < self._last_local_measurement_ns
            ):
                raise ValueError("L4 local perception time must not move backwards")
            repeated_sequence = observation.source_sequence == self._last_local_sequence
            if repeated_sequence and (
                observation.captured_monotonic_ns != self._last_local_measurement_ns
                or observation.values != self._last_local_values
            ):
                raise ValueError("L4 local perception sequence was rewritten")
            if not repeated_sequence:
                yaw_cos = math.cos(estimate.yaw_rad)
                yaw_sin = math.sin(estimate.yaw_rad)
                observed_cells: set[tuple[int, int]] = set()
                for index in range(point_count):
                    local_x = _number(values, f"point_{index:03d}_x_m")
                    local_y = _number(values, f"point_{index:03d}_y_m")
                    _integer(values, f"point_{index:03d}_quality")
                    world_x = estimate.x_m + yaw_cos * local_x - yaw_sin * local_y
                    world_y = estimate.y_m + yaw_sin * local_x + yaw_cos * local_y
                    if math.hypot(world_x - estimate.x_m, world_y - estimate.y_m) > (
                        self._config.local_costmap_radius_m
                    ):
                        continue
                    observed_cells.add(self._grid_key(world_x, world_y))
                for key in sorted(observed_cells):
                    previous_count = self._cells.get(key, (0, 0))[0]
                    self._cells[key] = (
                        previous_count + 1,
                        observation.captured_monotonic_ns,
                    )
                self._last_local_sequence = observation.source_sequence
                self._last_local_measurement_ns = observation.captured_monotonic_ns
                self._last_local_values = observation.values
                costmap_changed = True

        expired = tuple(
            track_id
            for track_id, (_, captured_ns) in self._tracks.items()
            if frame.context.monotonic_ns - captured_ns > self._config.max_track_age_ns
        )
        for track_id in expired:
            del self._tracks[track_id]
        if changed_tracks or expired:
            self._map_revision += 1

        expired_cells = tuple(
            key
            for key, (_, captured_ns) in self._cells.items()
            if frame.context.monotonic_ns - captured_ns
            > self._config.local_costmap_max_cell_age_ns
            or self._cell_distance_m(key, estimate.x_m, estimate.y_m)
            > self._config.local_costmap_radius_m
        )
        for key in expired_cells:
            del self._cells[key]
        if len(self._cells) > self._config.local_costmap_max_cells:
            keep = set(
                sorted(
                    self._cells,
                    key=lambda key: (
                        -self._cells[key][1],
                        self._cell_distance_m(key, estimate.x_m, estimate.y_m),
                        key,
                    ),
                )[: self._config.local_costmap_max_cells]
            )
            for key in tuple(self._cells):
                if key not in keep:
                    del self._cells[key]
                    costmap_changed = True
        if costmap_changed or expired_cells:
            self._costmap_revision += 1
            self._map_revision += 1

        if self._last_lidar_measurement_ns is None:
            raise ValueError("L4 requires an admitted lidar_health observation before output")
        freshness_ns = max(
            0,
            frame.context.monotonic_ns - self._last_lidar_measurement_ns,
        )
        tracks = tuple(self._tracks[key][0] for key in sorted(self._tracks))
        local_costmap = None
        if (
            self._last_local_measurement_ns is not None
            and self._last_local_sequence is not None
        ):
            local_costmap = RollingLocalCostmap(
                frame_id=estimate.frame_id,
                revision=self._costmap_revision,
                resolution_m=self._config.local_costmap_resolution_m,
                radius_m=self._config.local_costmap_radius_m,
                occupied_cells=tuple(
                    CostmapCell(key[0], key[1], self._cells[key][0])
                    for key in sorted(self._cells)
                ),
                source_sequence=self._last_local_sequence,
                freshness_ns=max(
                    0,
                    frame.context.monotonic_ns - self._last_local_measurement_ns,
                ),
            )
        return WorldSnapshot(
            frame.context,
            frame_id=estimate.frame_id,
            map_revision=self._map_revision,
            obstacle_tracks=tracks,
            freshness_ns=freshness_ns,
            local_costmap=local_costmap,
        )

    def _grid_key(self, x_m: float, y_m: float) -> tuple[int, int]:
        resolution = self._config.local_costmap_resolution_m
        return math.floor(x_m / resolution), math.floor(y_m / resolution)

    def _cell_distance_m(self, key: tuple[int, int], x_m: float, y_m: float) -> float:
        resolution = self._config.local_costmap_resolution_m
        center_x = (key[0] + 0.5) * resolution
        center_y = (key[1] + 0.5) * resolution
        return math.hypot(center_x - x_m, center_y - y_m)


def build_empty_world(frame: AdmittedFrame, estimate: RobotEstimate) -> WorldSnapshot:
    return WorldSnapshot(
        frame.context,
        frame_id=estimate.frame_id,
        map_revision=0,
        obstacle_tracks=(),
        freshness_ns=0,
    )


__all__ = [
    "ShadowWorldModel",
    "WorldModelConfig",
    "WorldModelStateCheckpoint",
    "build_empty_world",
]
