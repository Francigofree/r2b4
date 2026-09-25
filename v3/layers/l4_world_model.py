# R2B4_FOLLOW_PERSON_P0_V2_20260923
"""L4 deterministic temporal realtime world model.

Tracks expose measurement lineage and bounded prediction quality separately
from navigation behavior. Structural memory uses dynamic masking and
measurement-time quality.
Fresh measurement-time-aligned LiDAR evidence always outranks remembered
geometry; no navigation, persistence or I/O authority is added to L4.
"""

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
from v3.layers.l4_structural_memory import (
    StructuralMemoryCheckpoint,
    StructuralMemoryGrid,
)
from v3.layers.l4_temporal_history import (
    LidarScanSnapshot,
    PoseHistory,
    PoseHistoryCheckpoint,
    PoseSample,
    ScanHistory,
    ScanHistoryCheckpoint,
)
from v3.layers.l4_temporal_occupancy import (
    ScanCellEvidence,
    TemporalOccupancyCheckpoint,
    TemporalOccupancyGrid,
)
from v3.layers.l4_temporal_tracking import (
    PersonImageRegion,
    PersonMeasurement,
    TemporalTrackCheckpoint,
    TemporalTrackStore,
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


def _wrapped_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


@dataclass(frozen=True, slots=True)
class WorldModelConfig:
    # Existing fields stay first and in their original order for compatibility.
    max_track_age_ns: int
    local_costmap_resolution_m: float
    local_costmap_radius_m: float
    local_costmap_max_cell_age_ns: int
    local_costmap_max_cells: int
    local_costmap_max_points_per_scan: int
    person_tracking_enabled: bool
    person_camera_horizontal_fov_rad: float  # 66 deg Camera Module 3
    person_camera_yaw_offset_rad: float
    person_lidar_max_skew_ns: int
    person_lidar_angular_margin_rad: float
    person_lidar_cluster_depth_m: float
    person_lidar_min_points: int
    person_track_max_association_distance_m: float
    person_track_max_speed_mps: float
    person_track_radius_m: float
    person_track_max_age_ns: int
    person_track_reacquire_max_age_ns: int

    # TEMPORAL-1 private implementation bounds. No public contract changes.
    pose_history_max_age_ns: int
    pose_history_max_samples: int
    pose_lookup_max_skew_ns: int
    scan_history_max_age_ns: int
    scan_history_max_scans: int
    occupancy_hit_increment: int
    occupancy_free_decrement: int
    occupancy_max_score: int
    person_track_alpha: float
    person_track_beta: float
    person_track_prediction_max_age_ns: int

    # TEMPORAL-2 structural local-memory bounds. Still L4-private.
    structural_memory_enabled: bool
    structural_max_age_ns: int
    structural_max_cells: int
    structural_hit_increment: int
    structural_free_decrement: int
    structural_max_score: int
    structural_confirm_score: int
    structural_deconfirm_score: int | None
    structural_confirm_min_hits: int
    structural_confirm_min_span_ns: int
    structural_clear_score: int
    structural_dynamic_mask_margin_m: float
    structural_dynamic_mask_min_speed_mps: float
    structural_dynamic_mask_max_tracks: int
    structural_max_position_variance: float
    structural_max_yaw_variance: float
    continuity_translation_base_m: float
    continuity_translation_rate_mps: float
    continuity_yaw_base_rad: float
    continuity_yaw_rate_rad_s: float

    def __post_init__(self) -> None:
        for name in (
            "max_track_age_ns",
            "local_costmap_max_cell_age_ns",
            "person_lidar_max_skew_ns",
            "person_track_max_age_ns",
            "person_track_reacquire_max_age_ns",
            "pose_history_max_age_ns",
            "scan_history_max_age_ns",
            "structural_max_age_ns",
        ):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if self.person_track_reacquire_max_age_ns <= self.person_track_max_age_ns:
            raise ValueError("person_track_reacquire_max_age_ns must exceed active track max age")
        for name in ("pose_lookup_max_skew_ns", "person_track_prediction_max_age_ns", "structural_confirm_min_span_ns"):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                raise ValueError(f"{name} must be a non-negative integer")
        for name in (
            "local_costmap_resolution_m",
            "local_costmap_radius_m",
            "person_lidar_cluster_depth_m",
            "person_track_max_association_distance_m",
            "person_track_max_speed_mps",
            "person_track_radius_m",
            "structural_max_position_variance",
            "structural_max_yaw_variance",
            "structural_dynamic_mask_margin_m",
            "structural_dynamic_mask_min_speed_mps",
            "continuity_translation_base_m",
            "continuity_translation_rate_mps",
            "continuity_yaw_base_rad",
            "continuity_yaw_rate_rad_s",
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
            "pose_history_max_samples",
            "scan_history_max_scans",
            "occupancy_hit_increment",
            "occupancy_free_decrement",
            "occupancy_max_score",
            "structural_max_cells",
            "structural_hit_increment",
            "structural_free_decrement",
            "structural_max_score",
            "structural_confirm_score",
            "structural_confirm_min_hits",
            "structural_dynamic_mask_max_tracks",
        ):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if (
            not isinstance(self.structural_clear_score, int)
            or isinstance(self.structural_clear_score, bool)
            or self.structural_clear_score < 0
        ):
            raise ValueError("structural_clear_score must be a non-negative integer")
        if self.structural_confirm_score > self.structural_max_score:
            raise ValueError("structural_confirm_score cannot exceed structural_max_score")
        if self.structural_deconfirm_score is not None and (
            not isinstance(self.structural_deconfirm_score, int)
            or isinstance(self.structural_deconfirm_score, bool)
        ):
            raise ValueError("structural_deconfirm_score must be an integer or None")
        effective_deconfirm_score = (
            max(
                self.structural_clear_score,
                (self.structural_clear_score + self.structural_confirm_score) // 2,
            )
            if self.structural_deconfirm_score is None
            else self.structural_deconfirm_score
        )
        if not (
            self.structural_clear_score
            <= effective_deconfirm_score
            < self.structural_confirm_score
        ):
            raise ValueError(
                "structural scores must satisfy clear <= deconfirm < confirm"
            )
        if type(self.person_tracking_enabled) is not bool:
            raise TypeError("person_tracking_enabled must be bool")
        if type(self.structural_memory_enabled) is not bool:
            raise TypeError("structural_memory_enabled must be bool")
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
        for name in ("person_track_alpha", "person_track_beta"):
            value = getattr(self, name)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or not 0.0 < float(value) <= 1.0
            ):
                raise ValueError(f"{name} must be in (0, 1]")


@dataclass(frozen=True, slots=True)
class WorldModelStateCheckpoint:
    last_lidar_measurement_ns: int | None
    last_lidar_sequence: int | None
    map_revision: int
    tracks: TemporalTrackCheckpoint
    last_local_measurement_ns: int | None
    last_local_sequence: int | None
    last_local_values: tuple[DataField, ...] | None
    costmap_revision: int
    cells: TemporalOccupancyCheckpoint
    pose_history: PoseHistoryCheckpoint
    scan_history: ScanHistoryCheckpoint
    structural_memory: StructuralMemoryCheckpoint | None = None
    last_continuity_pose: PoseSample | None = None
    structural_recall_ready: bool = False


@dataclass(frozen=True, slots=True)
class _PersonImageDetection:
    confidence: float
    xmin: float
    xmax: float
    ymin: float | None = None
    ymax: float | None = None


@dataclass(frozen=True, slots=True)
class _PersonSpatialMeasurement:
    confidence: float
    x_m: float
    y_m: float
    image_region: PersonImageRegion | None = None


class ShadowWorldModel:
    """Own one bounded temporal world model and emit the unchanged WorldSnapshot."""

    __slots__ = (
        "_config",
        "_costmap_revision",
        "_last_continuity_pose",
        "_last_lidar_measurement_ns",
        "_last_lidar_sequence",
        "_last_local_measurement_ns",
        "_last_local_sequence",
        "_last_local_values",
        "_map_revision",
        "_occupancy",
        "_pose_history",
        "_scan_history",
        "_structural_memory",
        "_structural_recall_ready",
        "_track_store",
    )

    def __init__(self, config: WorldModelConfig) -> None:
        self._config = config
        self._last_lidar_measurement_ns: int | None = None
        self._last_lidar_sequence: int | None = None
        self._map_revision = 0
        self._last_local_measurement_ns: int | None = None
        self._last_local_sequence: int | None = None
        self._last_local_values: tuple[DataField, ...] | None = None
        self._costmap_revision = 0
        self._last_continuity_pose: PoseSample | None = None
        self._structural_recall_ready = False
        self._pose_history = PoseHistory(
            config.pose_history_max_age_ns,
            config.pose_history_max_samples,
            config.pose_lookup_max_skew_ns,
        )
        self._scan_history = ScanHistory(
            config.scan_history_max_age_ns,
            config.scan_history_max_scans,
        )
        self._occupancy = TemporalOccupancyGrid(
            resolution_m=config.local_costmap_resolution_m,
            radius_m=config.local_costmap_radius_m,
            max_age_ns=config.local_costmap_max_cell_age_ns,
            max_cells=config.local_costmap_max_cells,
            hit_increment=config.occupancy_hit_increment,
            free_decrement=config.occupancy_free_decrement,
            max_score=config.occupancy_max_score,
        )
        self._structural_memory = StructuralMemoryGrid(
            resolution_m=config.local_costmap_resolution_m,
            max_age_ns=config.structural_max_age_ns,
            max_cells=config.structural_max_cells,
            hit_increment=config.structural_hit_increment,
            free_decrement=config.structural_free_decrement,
            max_score=config.structural_max_score,
            confirm_score=config.structural_confirm_score,
            deconfirm_score=config.structural_deconfirm_score,
            confirm_min_hits=config.structural_confirm_min_hits,
            confirm_min_span_ns=config.structural_confirm_min_span_ns,
            clear_score=config.structural_clear_score,
        )
        self._track_store = TemporalTrackStore(
            alpha=config.person_track_alpha,
            beta=config.person_track_beta,
            prediction_max_age_ns=config.person_track_prediction_max_age_ns,
            max_speed_mps=config.person_track_max_speed_mps,
            person_reacquire_max_age_ns=config.person_track_reacquire_max_age_ns,
        )

    def checkpoint(self) -> WorldModelStateCheckpoint:
        return WorldModelStateCheckpoint(
            self._last_lidar_measurement_ns,
            self._last_lidar_sequence,
            self._map_revision,
            self._track_store.checkpoint(),
            self._last_local_measurement_ns,
            self._last_local_sequence,
            self._last_local_values,
            self._costmap_revision,
            self._occupancy.checkpoint(),
            self._pose_history.checkpoint(),
            self._scan_history.checkpoint(),
            self._structural_memory.checkpoint(),
            self._last_continuity_pose,
            self._structural_recall_ready,
        )

    def restore(self, checkpoint: WorldModelStateCheckpoint) -> None:
        if not isinstance(checkpoint, WorldModelStateCheckpoint):
            raise TypeError("checkpoint must be WorldModelStateCheckpoint")
        self._last_lidar_measurement_ns = checkpoint.last_lidar_measurement_ns
        self._last_lidar_sequence = checkpoint.last_lidar_sequence
        self._map_revision = checkpoint.map_revision
        self._track_store.restore(checkpoint.tracks)
        self._last_local_measurement_ns = checkpoint.last_local_measurement_ns
        self._last_local_sequence = checkpoint.last_local_sequence
        self._last_local_values = checkpoint.last_local_values
        self._costmap_revision = checkpoint.costmap_revision
        self._occupancy.restore(checkpoint.cells)
        self._pose_history.restore(checkpoint.pose_history)
        self._scan_history.restore(checkpoint.scan_history)
        if checkpoint.structural_memory is None:
            self._structural_memory.clear()
        else:
            self._structural_memory.restore(checkpoint.structural_memory)
        self._last_continuity_pose = checkpoint.last_continuity_pose
        self._structural_recall_ready = checkpoint.structural_recall_ready

    def __call__(self, frame: AdmittedFrame, estimate: RobotEstimate) -> WorldSnapshot:
        if frame.context != estimate.context:
            raise ValueError("L4 inputs must use the same tick context")

        continuity_broken = self._pose_discontinuity(estimate)
        if continuity_broken:
            self._pose_history.clear()
        frame_changed = self._pose_history.add(estimate)
        if frame_changed or continuity_broken:
            self._reset_spatial_state()
        self._last_continuity_pose = PoseSample(
            estimate.frame_id,
            estimate.context.monotonic_ns,
            estimate.x_m,
            estimate.y_m,
            estimate.yaw_rad,
            estimate.covariance_5x5[0],
            estimate.covariance_5x5[6],
            estimate.covariance_5x5[12],
        )

        self._update_lidar_health(frame)

        changed_tracks = False
        for observation in frame.accepted:
            if observation.kind != "obstacle_track":
                continue
            values = _values(observation)
            track_id = values.get("track_id")
            if not isinstance(track_id, str) or not track_id:
                raise ValueError("obstacle_track.track_id must be a non-empty string")
            changed_tracks = self._track_store.upsert_external(
                ObstacleTrack(
                    track_id=track_id,
                    x_m=_number(values, "x_m"),
                    y_m=_number(values, "y_m"),
                    radius_m=_number(values, "radius_m"),
                    vx_mps=_number(values, "vx_mps"),
                    vy_mps=_number(values, "vy_mps"),
                    confidence=_number(values, "confidence"),
                ),
                observation.captured_monotonic_ns,
                observed_ns=frame.context.monotonic_ns,
            ) or changed_tracks

        costmap_changed = self._update_local_scan(frame, estimate)

        if self._config.person_tracking_enabled:
            person_observations = tuple(
                item for item in frame.accepted if item.kind == "person_detection"
            )
            if len(person_observations) > 1:
                raise ValueError("L4 accepts at most one person_detection observation per tick")
            if person_observations:
                changed_tracks = self._update_person_tracks(person_observations[0], estimate) or changed_tracks

        expired_tracks = self._track_store.expire(
            frame.context.monotonic_ns,
            person_max_age_ns=self._config.person_track_max_age_ns,
            other_max_age_ns=self._config.max_track_age_ns,
        )
        if changed_tracks or expired_tracks:
            self._map_revision += 1

        if self._occupancy.prune(
            now_ns=frame.context.monotonic_ns,
            center_x_m=estimate.x_m,
            center_y_m=estimate.y_m,
        ):
            costmap_changed = True
        if costmap_changed:
            self._costmap_revision += 1
            self._map_revision += 1

        if self._last_lidar_measurement_ns is None:
            raise ValueError("L4 requires an admitted lidar_health observation before output")
        freshness_ns = max(0, frame.context.monotonic_ns - self._last_lidar_measurement_ns)
        tracks = self._track_store.projected_tracks(frame.context.monotonic_ns)
        local_costmap = None
        if self._last_local_measurement_ns is not None and self._last_local_sequence is not None:
            local_costmap = RollingLocalCostmap(
                frame_id=estimate.frame_id,
                revision=self._costmap_revision,
                resolution_m=self._config.local_costmap_resolution_m,
                radius_m=self._config.local_costmap_radius_m,
                occupied_cells=tuple(
                    CostmapCell(grid_x, grid_y, count)
                    for grid_x, grid_y, count in self._assembled_occupied_cells(estimate)
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

    def _reset_spatial_state(self) -> None:
        self._scan_history.clear()
        had_occupancy = self._occupancy.clear()
        had_structural = self._structural_memory.clear()
        had_tracks = self._track_store.clear()
        self._last_local_measurement_ns = None
        self._last_local_sequence = None
        self._last_local_values = None
        self._structural_recall_ready = False
        if had_occupancy or had_structural or had_tracks:
            self._costmap_revision += 1
        self._map_revision += 1

    def _pose_discontinuity(self, estimate: RobotEstimate) -> bool:
        previous = self._last_continuity_pose
        if previous is None or previous.frame_id != estimate.frame_id:
            return False
        elapsed_ns = estimate.context.monotonic_ns - previous.monotonic_ns
        if elapsed_ns <= 0:
            return False
        dt_s = elapsed_ns / 1_000_000_000.0
        translation_limit = (
            self._config.continuity_translation_base_m
            + self._config.continuity_translation_rate_mps * dt_s
        )
        yaw_limit = (
            self._config.continuity_yaw_base_rad
            + self._config.continuity_yaw_rate_rad_s * dt_s
        )
        translation = math.hypot(estimate.x_m - previous.x_m, estimate.y_m - previous.y_m)
        yaw_delta = abs(_wrapped_angle(estimate.yaw_rad - previous.yaw_rad))
        return translation > translation_limit or yaw_delta > yaw_limit

    def _structural_quality_ok(self, estimate: RobotEstimate) -> bool:
        covariance = estimate.covariance_5x5
        return self._structural_variance_ok(covariance[0], covariance[6], covariance[12])

    def _structural_pose_quality_ok(self, pose: PoseSample) -> bool:
        return self._structural_variance_ok(
            pose.position_variance_x,
            pose.position_variance_y,
            pose.yaw_variance,
        )

    def _structural_variance_ok(self, x_variance: float, y_variance: float, yaw_variance: float) -> bool:
        return (
            x_variance <= self._config.structural_max_position_variance
            and y_variance <= self._config.structural_max_position_variance
            and yaw_variance <= self._config.structural_max_yaw_variance
        )

    def _dynamic_structural_hit_keys(
        self,
        frame: AdmittedFrame,
        *,
        captured_ns: int,
        evidence: ScanCellEvidence,
    ) -> frozenset[tuple[int, int]]:
        if not evidence.hit_keys:
            return frozenset()

        projection_limit_ns = self._config.person_track_prediction_max_age_ns
        tracks: dict[str, ObstacleTrack] = {}
        for state in self._track_store.checkpoint().states:
            if (
                not state.track.track_id.startswith("person-")
                or state.captured_ns > captured_ns
            ):
                continue
            projected_ns = min(captured_ns - state.captured_ns, projection_limit_ns)
            dt_s = projected_ns / 1_000_000_000.0
            track = state.track
            tracks[track.track_id] = ObstacleTrack(
                track_id=track.track_id,
                x_m=track.x_m + track.vx_mps * dt_s,
                y_m=track.y_m + track.vy_mps * dt_s,
                radius_m=track.radius_m,
                vx_mps=track.vx_mps,
                vy_mps=track.vy_mps,
                confidence=track.confidence,
            )
        for observation in frame.accepted:
            if observation.kind != "obstacle_track" or observation.captured_monotonic_ns > captured_ns:
                continue
            values = _values(observation)
            track_id = values.get("track_id")
            if not isinstance(track_id, str) or not track_id:
                raise ValueError("obstacle_track.track_id must be a non-empty string")
            vx_mps = _number(values, "vx_mps")
            vy_mps = _number(values, "vy_mps")
            speed_mps = math.hypot(vx_mps, vy_mps)
            if (
                not track_id.startswith("person-")
                and speed_mps < self._config.structural_dynamic_mask_min_speed_mps
            ):
                continue
            dt_ns = min(captured_ns - observation.captured_monotonic_ns, projection_limit_ns)
            dt_s = dt_ns / 1_000_000_000.0
            tracks[track_id] = ObstacleTrack(
                track_id=track_id,
                x_m=_number(values, "x_m") + vx_mps * dt_s,
                y_m=_number(values, "y_m") + vy_mps * dt_s,
                radius_m=_number(values, "radius_m"),
                vx_mps=vx_mps,
                vy_mps=vy_mps,
                confidence=_number(values, "confidence"),
            )

        ordered_tracks = tuple(tracks[key] for key in sorted(tracks))[
            : self._config.structural_dynamic_mask_max_tracks
        ]
        if not ordered_tracks:
            return frozenset()

        resolution_m = self._config.local_costmap_resolution_m
        cell_radius_m = resolution_m / math.sqrt(2.0)
        ignored: set[tuple[int, int]] = set()
        for key in evidence.hit_keys:
            cell_x = (key[0] + 0.5) * resolution_m
            cell_y = (key[1] + 0.5) * resolution_m
            if any(
                math.hypot(cell_x - track.x_m, cell_y - track.y_m)
                <= track.radius_m + self._config.structural_dynamic_mask_margin_m + cell_radius_m
                for track in ordered_tracks
            ):
                ignored.add(key)
        return frozenset(ignored)

    def _assembled_occupied_cells(self, estimate: RobotEstimate) -> tuple[tuple[int, int, int], ...]:
        fast_all = self._occupancy.occupied_cells()
        structural_active = (
            self._config.structural_memory_enabled
            and self._structural_recall_ready
            and self._structural_quality_ok(estimate)
        )
        if not structural_active:
            return fast_all[: self._config.local_costmap_max_cells]

        # A trustworthy current ray that traverses a cell is stronger evidence
        # than either the short temporal accumulator or the long memory. Keep
        # the internal scores for hysteresis, but do not expose that cell as an
        # obstacle in the current planner snapshot.
        current_free = frozenset(self._structural_memory.current_free_keys)
        fast = tuple(
            cell for cell in fast_all if (cell[0], cell[1]) not in current_free
        )
        if len(fast) >= self._config.local_costmap_max_cells:
            return fast[: self._config.local_costmap_max_cells]

        result = list(fast)
        used = {(grid_x, grid_y) for grid_x, grid_y, _ in fast}
        excluded = frozenset((*used, *current_free))
        remembered = self._structural_memory.confirmed_cells(
            center_x_m=estimate.x_m,
            center_y_m=estimate.y_m,
            radius_m=self._config.local_costmap_radius_m,
            excluded_keys=excluded,
        )
        remaining = self._config.local_costmap_max_cells - len(result)
        result.extend(remembered[:remaining])
        return tuple(result)

    def _update_lidar_health(self, frame: AdmittedFrame) -> None:
        lidar = tuple(item for item in frame.accepted if item.kind == "lidar_health")
        if len(lidar) > 1:
            raise ValueError("L4 accepts at most one lidar_health observation per tick")
        if not lidar:
            return
        observation = lidar[0]
        values = _values(observation)
        age_ns = _number(values, "age_ns")
        if "point_count" in values:
            point_count = _number(values, "point_count")
            quality_valid = point_count >= 0.0
            measurement_ns = observation.captured_monotonic_ns
        else:
            confidence = _number(values, "confidence")
            quality_valid = 0.0 <= confidence <= 1.0
            measurement_ns = max(0, observation.captured_monotonic_ns - int(round(age_ns)))
        if age_ns < 0.0 or not quality_valid:
            raise ValueError("lidar health values are outside their physical range")
        if self._last_lidar_sequence is not None and observation.source_sequence < self._last_lidar_sequence:
            raise ValueError("L4 lidar sequence must not move backwards")
        if self._last_lidar_measurement_ns is not None and measurement_ns < self._last_lidar_measurement_ns:
            raise ValueError("L4 lidar measurement time must not move backwards")
        if self._last_lidar_sequence is None or observation.source_sequence > self._last_lidar_sequence:
            self._map_revision += 1
            self._last_lidar_sequence = observation.source_sequence
            self._last_lidar_measurement_ns = measurement_ns

    def _update_local_scan(self, frame: AdmittedFrame, estimate: RobotEstimate) -> bool:
        local = tuple(item for item in frame.accepted if item.kind == "lidar_local_points")
        if len(local) > 1:
            raise ValueError("L4 accepts at most one lidar_local_points observation per tick")
        if not local:
            return False
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
        if self._last_local_sequence is not None and observation.source_sequence < self._last_local_sequence:
            raise ValueError("L4 local perception sequence must not move backwards")
        if (
            self._last_local_measurement_ns is not None
            and observation.captured_monotonic_ns < self._last_local_measurement_ns
        ):
            raise ValueError("L4 local perception time must not move backwards")
        repeated_sequence = observation.source_sequence == self._last_local_sequence
        if repeated_sequence and (
            observation.captured_monotonic_ns != self._last_local_measurement_ns
            or observation.values != self._last_local_values
        ):
            raise ValueError("L4 local perception sequence was rewritten")
        if repeated_sequence:
            return False

        pose = self._pose_history.lookup(observation.captured_monotonic_ns, estimate.frame_id)
        if pose is None:
            raise ValueError("L4 cannot align local perception to pose history")
        yaw_cos = math.cos(pose.yaw_rad)
        yaw_sin = math.sin(pose.yaw_rad)
        endpoints: list[tuple[float, float]] = []
        for index in range(point_count):
            local_x = _number(values, f"point_{index:03d}_x_m")
            local_y = _number(values, f"point_{index:03d}_y_m")
            _integer(values, f"point_{index:03d}_quality")
            if math.hypot(local_x, local_y) > self._config.local_costmap_radius_m:
                continue
            endpoints.append(
                (
                    pose.x_m + yaw_cos * local_x - yaw_sin * local_y,
                    pose.y_m + yaw_sin * local_x + yaw_cos * local_y,
                )
            )
        evidence = self._occupancy.integrate_scan(
            origin_x_m=pose.x_m,
            origin_y_m=pose.y_m,
            endpoints=tuple(endpoints),
            captured_ns=observation.captured_monotonic_ns,
        )
        if self._config.structural_memory_enabled:
            if self._structural_pose_quality_ok(pose):
                ignored_hit_keys = self._dynamic_structural_hit_keys(
                    frame,
                    captured_ns=observation.captured_monotonic_ns,
                    evidence=evidence,
                )
                self._structural_memory.integrate(
                    evidence,
                    captured_ns=observation.captured_monotonic_ns,
                    ignored_hit_keys=ignored_hit_keys,
                )
                self._structural_memory.prune(now_ns=observation.captured_monotonic_ns)
                self._structural_recall_ready = True
            else:
                # Do not learn or clear long-lived geometry from a spatially
                # uncertain scan. Recall remains disabled until the next good
                # local scan closes a trustworthy structural observation.
                self._structural_recall_ready = False
        self._scan_history.add(
            LidarScanSnapshot(
                observation.source_sequence,
                observation.captured_monotonic_ns,
                estimate.frame_id,
                observation.values,
                pose,
            )
        )
        self._last_local_sequence = observation.source_sequence
        self._last_local_measurement_ns = observation.captured_monotonic_ns
        self._last_local_values = observation.values
        # Preserve the historic revision meaning: every accepted new local scan
        # advances the local costmap revision even when it contains zero hits.
        return True

    def _update_person_tracks(self, observation: Observation, estimate: RobotEstimate) -> bool:
        scan = self._scan_history.nearest(
            observation.captured_monotonic_ns,
            self._config.person_lidar_max_skew_ns,
            estimate.frame_id,
        )
        if scan is None:
            return False
        detection_pose = self._pose_history.lookup(
            observation.captured_monotonic_ns,
            estimate.frame_id,
        )
        if detection_pose is None:
            return False
        image_detections = self._person_image_detections(observation)
        if not image_detections:
            return False
        spatial = self._localize_people(image_detections, scan, detection_pose)
        if not spatial:
            return False
        return self._track_store.associate_people(
            tuple(PersonMeasurement(item.confidence, item.x_m, item.y_m, item.image_region)
                  for item in spatial),
            captured_ns=observation.captured_monotonic_ns,
            radius_m=self._config.person_track_radius_m,
            max_association_distance_m=self._config.person_track_max_association_distance_m,
            observed_ns=estimate.context.monotonic_ns,
        )

    def _person_image_detections(self, observation: Observation) -> tuple[_PersonImageDetection, ...]:
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
            ymin, ymax = self._person_vertical_extent(values, "primary")
            return (_PersonImageDetection(confidence, xmin, xmax, ymin, ymax),)
        count = _integer(values, "emitted_person_count")
        detections: list[_PersonImageDetection] = []
        for index in range(count):
            prefix = f"person_{index:03d}"
            confidence = _unit_number(values, f"{prefix}_confidence")
            xmin = _unit_number(values, f"{prefix}_xmin")
            xmax = _unit_number(values, f"{prefix}_xmax")
            if xmax <= xmin:
                raise ValueError("person detection bounding box must have positive width")
            ymin, ymax = self._person_vertical_extent(values, prefix)
            detections.append(_PersonImageDetection(confidence, xmin, xmax, ymin, ymax))
        return tuple(detections)

    @staticmethod
    def _person_vertical_extent(
        values: dict[str, object], prefix: str,
    ) -> tuple[float | None, float | None]:
        # Older observations only carried horizontal bounds.
        if f"{prefix}_ymin" not in values and f"{prefix}_ymax" not in values:
            return None, None
        ymin = _unit_number(values, f"{prefix}_ymin")
        ymax = _unit_number(values, f"{prefix}_ymax")
        if ymax <= ymin:
            raise ValueError("person detection bounding box must have positive height")
        return ymin, ymax

    def _localize_people(
        self,
        detections: tuple[_PersonImageDetection, ...],
        scan: LidarScanSnapshot,
        detection_pose: PoseSample,
    ) -> tuple[_PersonSpatialMeasurement, ...]:
        values = {field.key: field.value for field in scan.values}
        point_count = _integer(values, "point_count")
        scan_cos = math.cos(scan.pose.yaw_rad)
        scan_sin = math.sin(scan.pose.yaw_rad)
        lidar_points: list[tuple[int, float, float, float, float]] = []
        for index in range(point_count):
            local_x = _number(values, f"point_{index:03d}_x_m")
            local_y = _number(values, f"point_{index:03d}_y_m")
            _integer(values, f"point_{index:03d}_quality")
            world_x = scan.pose.x_m + scan_cos * local_x - scan_sin * local_y
            world_y = scan.pose.y_m + scan_sin * local_x + scan_cos * local_y
            dx = world_x - detection_pose.x_m
            dy = world_y - detection_pose.y_m
            range_m = math.hypot(dx, dy)
            if range_m <= 0.0:
                continue
            bearing = _wrapped_angle(math.atan2(dy, dx) - detection_pose.yaw_rad)
            lidar_points.append((index, world_x, world_y, range_m, bearing))

        used_points: set[int] = set()
        result: list[_PersonSpatialMeasurement] = []
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
            world_x = sum(point[1] for point in cluster) / len(cluster)
            world_y = sum(point[2] for point in cluster) / len(cluster)
            region = None
            if detection.ymin is not None and detection.ymax is not None:
                region = PersonImageRegion(
                    _wrapped_angle(detection_pose.yaw_rad + (left + right) / 2),
                    abs(left - right), detection.ymin, detection.ymax,
                )
            result.append(_PersonSpatialMeasurement(
                detection.confidence, world_x, world_y, region,
            ))
        return tuple(result)

    def _pixel_bearing(self, normalized_x: float) -> float:
        focal = 0.5 / math.tan(self._config.person_camera_horizontal_fov_rad * 0.5)
        return self._config.person_camera_yaw_offset_rad + math.atan2(0.5 - normalized_x, focal)

    # Kept as compatibility helpers for tests/tools that may inspect L4 directly.
    def _grid_key(self, x_m: float, y_m: float) -> tuple[int, int]:
        return self._occupancy.grid_key(x_m, y_m)

    def _cell_distance_m(self, key: tuple[int, int], x_m: float, y_m: float) -> float:
        return self._occupancy.cell_distance_m(key, x_m, y_m)


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
