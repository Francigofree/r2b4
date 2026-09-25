"""Typed LiDAR localization policy injected before worker startup."""
from __future__ import annotations
from dataclasses import dataclass, fields, asdict
import math

@dataclass(frozen=True, slots=True)
class LidarMatcherConfig:
    dx_range: tuple[float, float]
    dy_range: tuple[float, float]
    dtheta_range: tuple[float, float]
    dx_step: float
    dy_step: float
    dtheta_step: float
    max_points: int
    confidence_min: float
    min_filtered_points: int
    min_valid_distance_m: float
    max_valid_distance_m: float
    scan_to_map_enabled: bool
    local_map_enabled: bool
    local_map_radius_m: float
    local_map_max_keyframes: int
    local_map_min_points: int
    local_map_points_per_keyframe: int
    keyframe_translation_m: float
    keyframe_rotation_rad: float
    tracking_reacquire_consecutive_scans: int
    tracking_reacquire_max_delta_m: float
    tracking_reacquire_max_delta_rad: float
    tracking_direction_min_wheel_speed_mps: float
    tracking_direction_backtrack_tolerance_m: float
    matcher_seed_low_confidence_to_pose_ref: bool
    matcher_seed_translation_prior_weight: float
    matcher_seed_rotation_prior_weight: float
    adaptive_max_points_enabled: bool
    adaptive_max_points_min: int
    matcher_budget_ms: float
    slow_path_budget_ms: float
    robust_inlier_distance_m: float
    robust_trim_fraction: float
    confidence_residual_scale_m: float
    confidence_sector_count: int
    confidence_target_sector_coverage: float
    ambiguity_translation_m: float
    ambiguity_rotation_rad: float
    ambiguity_margin_scale: float
    ambiguity_residual_margin_scale_m: float
    ambiguity_basin_top_k: int
    ambiguity_basin_refine_iters: int
    ambiguity_basin_barrier_scale: float
    observability_translation_step_m: float
    observability_rotation_step_rad: float
    observability_cost_scale: float
    relocalization_enabled: bool
    relocalization_confidence_min: float
    relocalization_cooldown_s: float
    relocalization_dx_range: tuple[float, float]
    relocalization_dy_range: tuple[float, float]
    relocalization_dtheta_range: tuple[float, float]
    relocalization_dx_step: float
    relocalization_dy_step: float
    relocalization_dtheta_step: float
    relocalization_seed_keyframes: int
    relocalization_direct_apply_max_delta_m: float
    relocalization_direct_apply_max_delta_rad: float
    relocalization_step_limit_m: float
    relocalization_step_limit_rad: float
    loop_closure_enabled: bool
    loop_closure_radius_m: float
    loop_closure_angle_rad: float
    loop_closure_min_keyframes: int
    loop_closure_cooldown_s: float
    loop_closure_blend: float
    loop_closure_max_correction_m: float
    loop_closure_max_correction_rad: float
    loop_closure_direct_apply_max_delta_m: float
    loop_closure_direct_apply_max_delta_rad: float
    loop_closure_step_limit_m: float
    loop_closure_step_limit_rad: float
    enabled: bool

    adaptive_relocalization_point_factor: float
    adaptive_tracking_point_factor: float
    adaptive_map_point_floor: int
    adaptive_map_point_multiplier: int
    normal_neighbor_count: int
    normal_isotropy_ratio_max: float
    normal_rank_ratio_scale: float
    normal_support_scale: float
    confidence_inlier_offset: float
    confidence_inlier_scale: float
    confidence_ambiguity_floor: float
    confidence_observability_floor: float
    integrity_min_inlier_ratio: float
    integrity_min_sector_coverage: float
    integrity_min_uniqueness: float
    integrity_min_observability: float

    def __post_init__(self) -> None:
        for field in fields(self):
            value = getattr(self, field.name)
            if isinstance(value, tuple):
                if len(value) != 2 or not all(math.isfinite(v) for v in value) or value[0] >= value[1]:
                    raise ValueError(f"{field.name} must be finite increasing range")
            elif type(value) is not bool and (not math.isfinite(value) or value < 0):
                raise ValueError(f"{field.name} must be finite non-negative")
        if self.min_valid_distance_m >= self.max_valid_distance_m:
            raise ValueError("LiDAR minimum range must be smaller than maximum range")
        if not 0 < self.robust_trim_fraction <= 1:
            raise ValueError("robust_trim_fraction must be in (0, 1]")
        if self.adaptive_max_points_min > self.max_points:
            raise ValueError("adaptive_max_points_min exceeds max_points")
        for name in ("dx_step", "dy_step", "dtheta_step", "relocalization_dx_step", "relocalization_dy_step", "relocalization_dtheta_step"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")

    def as_mapping(self) -> dict[str, object]:
        return asdict(self)
