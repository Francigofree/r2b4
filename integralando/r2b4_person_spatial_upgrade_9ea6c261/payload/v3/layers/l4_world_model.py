"""L4 deterministic rolling world state with LiDAR-backed person tracking."""

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
        raise ValueError(f"world-model field {key} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"world-model field {key} must be finite")
    return result


def _integer(values: dict[str, object], key: str) -> int:
    value = values.get(key)
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError(f"world-model field {key} must be a non-negative integer")
    return value


def _boolean(values: dict[str, object], key: str) -> bool:
    value = values.get(key)
    if type(value) is not bool:
        raise ValueError(f"world-model field {key} must be bool")
    return value


def _unit_number(values: dict[str, object], key: str) -> float:
    result = _number(values, key)
    if not 0.0 <= result <= 1.0:
        raise ValueError(f"world-model field {key} must be in [0, 1]")
    return result


@dataclass(frozen=True, slots=True)
class WorldModelConfig:
    max_track_age_ns: int = 500_000_000
    local_costmap_resolution_m: float = 0.10
    local_costmap_radius_m: float = 2.50
    local_costmap_max_cell_age_ns: int = 750_000_000
    local_costmap_max_cells: int = 1_200
    local_costmap_max_points_per_scan: int = 96
    person_tracking_enabled: bool = True
    person_camera_horizontal_fov_rad: float = 1.1519173063162575  # 66 deg Camera Module 3
    person_camera_yaw_offset_rad: float = 0.0
    person_lidar_max_skew_ns: int = 150_000_000
    person_lidar_angular_margin_rad: float = 0.04
    person_lidar_cluster_depth_m: float = 0.30
    person_lidar_min_points: int = 1
    person_track_max_association_distance_m: float = 0.75
    person_track_max_speed_mps: float = 6.0
    person_track_radius_m: float = 0.30

    def __post_init__(self) -> None:
        for name in (
            "max_track_age_ns",
            "local_costmap_max_cell_age_ns",
            "person_lidar_max_skew_ns",
        ):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        for name in (
            "local_costmap_resolution_m",
            "local_costmap_radius_m",
            "person_lidar_cluster_depth_m",
            "person_track_max_association_distance_m",
            "person_track_max_speed_mps",
            "person_track_radius_m",
        ):
            value = getattr(self, name)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value <= 0.0
            ):
                raise ValueError(f"{name} must be finite and positive")
        for name in (
            "local_costmap_max_cells",
            "local_costmap_max_points_per_scan",
            "person_lidar_min_points",
        ):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if type(self.person_tracking_enabled) is not bool:
            raise TypeError("person_tracking_enabled must be bool")
        if (
            not math.isfinite(self.person_camera_horizontal_fov_rad)
            or not 0.0 < self.person_camera_horizontal_fov_rad < math.pi
        ):
            raise ValueError("person_camera_horizontal_fov_rad must be in (0, pi)")
        if not math.isfinite(self.person_camera_yaw_offset_rad):
            raise ValueError("person_camera_yaw_offset_rad must be finite")
        if (
            not math.isfinite(self.person_lidar_angular_margin_rad)
            or not 0.0 <= self.person_lidar_angular_margin_rad < math.pi / 2.0
        ):
            raise ValueError("person_lidar_angular_margin_rad is outside its valid range")


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


@dataclass(frozen=True, slots=True)
class _PersonImageDetection:
    confidence: float
    xmin: float
    xmax: float


@dataclass(frozen=True, slots=True)
class _PersonSpatialMeasurement:
    confidence: float
    x_m: float
    y_m: float


class ShadowWorldModel:
    """Own LiDAR revision, obstacle/person history and one rolling local costmap."""

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
        self._last_local_values: tuple[DataField, ...] | None = None
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

        if self._config.person_tracking_enabled:
            person_observations = tuple(
                item for item in frame.accepted if item.kind == "person_detection"
            )
            if len(person_observations) > 1:
                raise ValueError("L4 accepts at most one person_detection observation per tick")
            if person_observations:
                changed_tracks = (
                    self._update_person_tracks(person_observations[0], estimate)
                    or changed_tracks
                )

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

    def _update_person_tracks(
        self,
        observation: Observation,
        estimate: RobotEstimate,
    ) -> bool:
        if self._last_local_measurement_ns is None or self._last_local_values is None:
            return False
        if abs(observation.captured_monotonic_ns - self._last_local_measurement_ns) > (
            self._config.person_lidar_max_skew_ns
        ):
            return False
        image_detections = self._person_image_detections(observation)
        if not image_detections:
            return False
        spatial = self._localize_people(image_detections, estimate)
        if not spatial:
            return False
        return self._associate_person_tracks(spatial, observation.captured_monotonic_ns)

    def _person_image_detections(
        self,
        observation: Observation,
    ) -> tuple[_PersonImageDetection, ...]:
        values = _values(observation)
        detected = _boolean(values, "person_detected")
        if not detected:
            return ()
        emitted = values.get("emitted_person_count")
        if emitted is None:
            confidence = _unit_number(values, "primary_confidence")
            xmin = _unit_number(values, "primary_xmin")
            xmax = _unit_number(values, "primary_xmax")
            if xmax <= xmin:
                raise ValueError("person detection bounding box must have positive width")
            return (_PersonImageDetection(confidence, xmin, xmax),)
        count = _integer(values, "emitted_person_count")
        detections: list[_PersonImageDetection] = []
        for index in range(count):
            prefix = f"person_{index:03d}"
            confidence = _unit_number(values, f"{prefix}_confidence")
            xmin = _unit_number(values, f"{prefix}_xmin")
            xmax = _unit_number(values, f"{prefix}_xmax")
            if xmax <= xmin:
                raise ValueError("person detection bounding box must have positive width")
            detections.append(_PersonImageDetection(confidence, xmin, xmax))
        return tuple(detections)

    def _localize_people(
        self,
        detections: tuple[_PersonImageDetection, ...],
        estimate: RobotEstimate,
    ) -> tuple[_PersonSpatialMeasurement, ...]:
        assert self._last_local_values is not None
        values = {field.key: field.value for field in self._last_local_values}
        point_count = _integer(values, "point_count")
        lidar_points: list[tuple[int, float, float, float, float]] = []
        for index in range(point_count):
            x_m = _number(values, f"point_{index:03d}_x_m")
            y_m = _number(values, f"point_{index:03d}_y_m")
            _integer(values, f"point_{index:03d}_quality")
            range_m = math.hypot(x_m, y_m)
            if range_m <= 0.0:
                continue
            bearing = math.atan2(y_m, x_m)
            lidar_points.append((index, x_m, y_m, range_m, bearing))

        used_points: set[int] = set()
        result: list[_PersonSpatialMeasurement] = []
        yaw_cos = math.cos(estimate.yaw_rad)
        yaw_sin = math.sin(estimate.yaw_rad)
        for detection in detections:
            left = self._pixel_bearing(detection.xmin)
            right = self._pixel_bearing(detection.xmax)
            lower = min(left, right) - self._config.person_lidar_angular_margin_rad
            upper = max(left, right) + self._config.person_lidar_angular_margin_rad
            candidates = [
                point
                for point in lidar_points
                if point[0] not in used_points and lower <= point[4] <= upper
            ]
            if not candidates:
                continue
            candidates.sort(key=lambda item: (item[3], item[1], item[2], item[0]))
            nearest = candidates[0][3]
            cluster = tuple(
                point
                for point in candidates
                if point[3] <= nearest + self._config.person_lidar_cluster_depth_m
            )
            if len(cluster) < self._config.person_lidar_min_points:
                continue
            for point in cluster:
                used_points.add(point[0])
            local_x = sum(point[1] for point in cluster) / len(cluster)
            local_y = sum(point[2] for point in cluster) / len(cluster)
            world_x = estimate.x_m + yaw_cos * local_x - yaw_sin * local_y
            world_y = estimate.y_m + yaw_sin * local_x + yaw_cos * local_y
            result.append(
                _PersonSpatialMeasurement(detection.confidence, world_x, world_y)
            )
        return tuple(result)

    def _pixel_bearing(self, normalized_x: float) -> float:
        focal = 0.5 / math.tan(self._config.person_camera_horizontal_fov_rad * 0.5)
        return self._config.person_camera_yaw_offset_rad + math.atan2(
            0.5 - normalized_x,
            focal,
        )

    def _associate_person_tracks(
        self,
        measurements: tuple[_PersonSpatialMeasurement, ...],
        captured_ns: int,
    ) -> bool:
        available = {
            track_id: value
            for track_id, value in self._tracks.items()
            if track_id.startswith("person-")
        }
        used_track_ids: set[str] = set()
        changed = False
        for measurement in measurements:
            best: tuple[float, str, ObstacleTrack, int] | None = None
            for track_id in sorted(available):
                if track_id in used_track_ids:
                    continue
                previous, previous_ns = available[track_id]
                dt_ns = captured_ns - previous_ns
                if dt_ns <= 0:
                    continue
                distance_m = math.hypot(
                    measurement.x_m - previous.x_m,
                    measurement.y_m - previous.y_m,
                )
                if distance_m > self._config.person_track_max_association_distance_m:
                    continue
                speed_mps = distance_m / (dt_ns / 1e9)
                if speed_mps > self._config.person_track_max_speed_mps:
                    continue
                candidate = (distance_m, track_id, previous, previous_ns)
                if best is None or candidate[:2] < best[:2]:
                    best = candidate

            if best is None:
                track_id = self._allocate_person_track_id()
                vx_mps = 0.0
                vy_mps = 0.0
            else:
                _, track_id, previous, previous_ns = best
                used_track_ids.add(track_id)
                dt_s = (captured_ns - previous_ns) / 1e9
                vx_mps = (measurement.x_m - previous.x_m) / dt_s
                vy_mps = (measurement.y_m - previous.y_m) / dt_s

            self._tracks[track_id] = (
                ObstacleTrack(
                    track_id=track_id,
                    x_m=measurement.x_m,
                    y_m=measurement.y_m,
                    radius_m=self._config.person_track_radius_m,
                    vx_mps=vx_mps,
                    vy_mps=vy_mps,
                    confidence=measurement.confidence,
                ),
                captured_ns,
            )
            changed = True
        return changed

    def _allocate_person_track_id(self) -> str:
        used: set[int] = set()
        for track_id in self._tracks:
            if not track_id.startswith("person-"):
                continue
            suffix = track_id[len("person-") :]
            if suffix.isdigit():
                used.add(int(suffix))
        candidate = 1
        while candidate in used:
            candidate += 1
        return f"person-{candidate}"

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
