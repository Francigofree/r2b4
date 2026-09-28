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
        transformed = points.copy()
        rotation, translation = np.eye(2), np.zeros(2)
        for _ in range(cfg.relative_iterations):
            distances, indices = tree.query(transformed, workers=1)
            selected = distances < cfg.robust_inlier_distance_m
            if np.count_nonzero(selected) < cfg.min_filtered_points:
                return None
            # Trim dynamic objects and unmatched scan edges.
            cutoff = min(cfg.robust_inlier_distance_m,
                         float(np.quantile(distances[selected], cfg.robust_trim_fraction)))
            selected &= distances <= cutoff
            if np.count_nonzero(selected) < 3:
                return None
            a, b = transformed[selected], reference[indices[selected]]
            ca, cb = a.mean(axis=0), b.mean(axis=0)
            u, _, vt = np.linalg.svd((a-ca).T @ (b-cb))
            r = vt.T @ u.T
            if np.linalg.det(r) < 0:
                vt[-1] *= -1
                r = vt.T @ u.T
            t = cb-r@ca
            transformed = transformed@r.T+t
            rotation, translation = r@rotation, r@translation+t
        distances, indices = tree.query(transformed, workers=1)
        selected = distances < cfg.robust_inlier_distance_m
        inliers = int(np.count_nonzero(selected))
        if inliers < cfg.min_filtered_points or inliers/len(points) < cfg.integrity_min_inlier_ratio:
            return None
        rmse = float(np.sqrt(np.mean(distances[selected]**2)))
        # Surface-normal rank identifies parallel-wall / feature-poor corridors.
        _, neighbours = tree.query(reference[indices[selected]], k=min(6, len(reference)), workers=1)
        neighbourhood = reference[neighbours]
        centred = neighbourhood-neighbourhood.mean(axis=1, keepdims=True)
        _, vectors = np.linalg.eigh(np.einsum("nki,nkj->nij", centred, centred))
        normals = vectors[:, :, 0]
        eigenvalues = np.linalg.eigvalsh(normals.T@normals/inliers)
        observability = float(min(1.0, max(0.0, 2*eigenvalues[0])))
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        interval_s = (measurement_ns-start_ns)/1e9
        if (rmse > cfg.relative_max_rmse_m
                or np.linalg.norm(translation) > cfg.relative_max_speed_mps*interval_s
                or abs(yaw) > cfg.relative_max_omega_rad_s*interval_s):
            return None
        return {"start_ns": start_ns, "dx_m": float(translation[0]),
                "dy_m": float(translation[1]), "dyaw_rad": yaw,
                "rmse_m": rmse, "observability": observability}
