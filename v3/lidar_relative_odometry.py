"""Bounded scan-to-scan submap registration, exclusively in the LiDAR worker.

The preceding scan is a short lived submap. No global map, EKF pose or rolling
obstacle map enters the registration, so its residual can detect wheel slip.
"""
from __future__ import annotations

import math
import numpy as np
from scipy.spatial import cKDTree

from v3.scan_matching import scan_to_points
from v3.lidar_config import LidarMatcherConfig


class RelativeLidarOdometry:
    def __init__(self, config: LidarMatcherConfig) -> None:
        self._config = config
        self._previous = None
        self._previous_ns = None

    def process(self, scan: list[dict], measurement_ns: int) -> dict[str, float | int] | None:
        cfg = self._config
        points = scan_to_points(scan, min_dist_m=cfg.min_valid_distance_m,
                                max_dist_m=cfg.max_valid_distance_m)
        # Fixed deterministic work, including nearest-neighbour searches.
        if len(points) > cfg.relative_max_points:
            points = points[np.linspace(0, len(points)-1, cfg.relative_max_points, dtype=int)]
        reference, start_ns = self._previous, self._previous_ns
        self._previous, self._previous_ns = points, measurement_ns
        if (reference is None or start_ns is None or len(reference) < cfg.min_filtered_points
                or len(points) < cfg.min_filtered_points
                or not 0 < measurement_ns-start_ns <= cfg.relative_max_interval_ns):
            return None
        tree = cKDTree(reference)
        # LiDAR beams resample surfaces on every revolution. Point-to-point ICP
        # mistakes tangential beam spacing for robot motion and accumulates a
        # false translation/yaw discrepancy against wheel/gyro odometry. Fit
        # surface-normal residuals instead; no wheel/pose prior enters this
        # independent measurement.
        _, neighbours = tree.query(reference, k=min(6, len(reference)), workers=1)
        neighbourhood = reference[neighbours]
        centred = neighbourhood-neighbourhood.mean(axis=1, keepdims=True)
        surface_values, vectors = np.linalg.eigh(np.einsum("nki,nkj->nij", centred, centred))
        normals = vectors[:, :, 0]
        # Corners and scattered returns have no reliable single surface normal.
        surfaces = surface_values[:, 0] < .1*np.maximum(surface_values[:, 1], 1e-12)
        transformed = points.copy()
        rotation, translation = np.eye(2), np.zeros(2)
        for _ in range(cfg.relative_iterations):
            distances, indices = tree.query(transformed, workers=1)
            matched_normals = normals[indices]
            residuals = np.einsum("ni,ni->n", matched_normals, transformed-reference[indices])
            selected = (distances < cfg.robust_inlier_distance_m) & surfaces[indices]
            if np.count_nonzero(selected) < cfg.min_filtered_points:
                return None
            # Trim dynamic objects and unmatched scan edges.
            cutoff = float(np.quantile(np.abs(residuals[selected]), cfg.robust_trim_fraction))
            selected &= np.abs(residuals) <= cutoff
            if np.count_nonzero(selected) < cfg.min_filtered_points:
                return None
            a, n = transformed[selected], matched_normals[selected]
            jacobian = np.column_stack((n[:, 0], n[:, 1],
                                       -a[:, 1]*n[:, 0]+a[:, 0]*n[:, 1]))
            step, _, rank, _ = np.linalg.lstsq(jacobian, -residuals[selected], rcond=None)
            if rank < 3:
                return None
            c, s = math.cos(step[2]), math.sin(step[2])
            r, t = np.array([[c, -s], [s, c]]), step[:2]
            transformed = transformed@r.T+t
            rotation, translation = r@rotation, r@translation+t
        distances, indices = tree.query(transformed, workers=1)
        selected = (distances < cfg.robust_inlier_distance_m) & surfaces[indices]
        inliers = int(np.count_nonzero(selected))
        if inliers < cfg.min_filtered_points or inliers/len(points) < cfg.integrity_min_inlier_ratio:
            return None
        residuals = np.einsum("ni,ni->n", normals[indices[selected]],
                              transformed[selected]-reference[indices[selected]])
        rmse = float(np.sqrt(np.mean(residuals**2)))
        # Surface-normal rank identifies parallel-wall / feature-poor corridors.
        matched_normals = normals[indices[selected]]
        eigenvalues = np.linalg.eigvalsh(matched_normals.T@matched_normals/inliers)
        observability = float(min(1.0, max(0.0, 2*eigenvalues[0])))
        # Translation rank alone cannot detect unobservable rotation (e.g. a
        # circular wall). Normalize the angular column by the scan radius so
        # the full SE(2) information has comparable, dimensionless columns.
        a = transformed[selected]
        radius = max(float(np.sqrt(np.mean(np.sum(a*a, axis=1)))), 1e-9)
        angular = (-a[:, 1]*matched_normals[:, 0]+a[:, 0]*matched_normals[:, 1])/radius
        information = np.column_stack((matched_normals, angular))
        full_rank = np.linalg.eigvalsh(information.T@information/inliers)
        observability = min(observability, float(max(0.0, min(1.0, 3*full_rank[0]))))
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        interval_s = (measurement_ns-start_ns)/1e9
        if (rmse > cfg.relative_max_rmse_m
                or np.linalg.norm(translation) > cfg.relative_max_speed_mps*interval_s
                or abs(yaw) > cfg.relative_max_omega_rad_s*interval_s):
            return None
        return {"start_ns": start_ns, "dx_m": float(translation[0]),
                "dy_m": float(translation[1]), "dyaw_rad": yaw,
                "rmse_m": rmse, "observability": observability}
