import math
import time
from typing import Callable, Dict, List, Tuple, Optional, Any, Sequence

import numpy as np
from scipy.spatial import cKDTree

from middleware.scan_matcher_contract import (
    SCAN_MATCH_CONFIDENCE_MODEL,
    SCAN_MATCH_INTEGRITY_MODEL,
)


def _clamp_scalar(value: float, lower: float, upper: float) -> float:
    """Clamp one scalar without entering NumPy's array-dispatch hot path."""
    number = float(value)
    if math.isnan(number):
        return number
    return float(min(float(upper), max(float(lower), number)))


def _scan_to_xy(
    scan: List[dict],
    dist_m: bool = False,
    min_dist_m: float = 0.05,
    max_dist_m: Optional[float] = None,
) -> np.ndarray:
    """
    Scan pontok (angle_rad, dist) -> xy sík robot koordinátában [m].
    A LIDAR szögkonvenciója: 90 fok = robot jobb oldal, 270 fok = bal oldal.
    A robot pose-konvencióban viszont +y balra mutat, ezért a laterális tengely
    előjele tükrözött a nyers LIDAR szögekhez képest.
    dist: ha False, a scan 'dist' mm-ben van -> /1000.
    """
    if not scan:
        return np.zeros((0, 2))
    pts = []
    for p in scan:
        a = p.get("angle_rad", math.radians(p.get("angle", 0)))
        d = float(p.get("dist", 0))
        if not dist_m:
            d /= 1000.0
        if d < float(min_dist_m):  # minimum tartomány alatt zaj
            continue
        if max_dist_m is not None and max_dist_m > 0.0 and d > float(max_dist_m):
            continue
        x = d * math.cos(a)
        y = -d * math.sin(a)
        pts.append([x, y])
    return np.array(pts, dtype=float) if pts else np.zeros((0, 2))


def scan_to_points(
    scan: List[dict],
    *,
    dist_in_m: bool = False,
    min_dist_m: float = 0.05,
    max_dist_m: Optional[float] = None,
) -> np.ndarray:
    """
    Public helper for converting a scan to XY points in robot frame [m].
    """
    return _scan_to_xy(
        scan=scan,
        dist_m=bool(dist_in_m),
        min_dist_m=float(min_dist_m),
        max_dist_m=max_dist_m,
    )


def _subsample(pts: np.ndarray, max_n: int) -> np.ndarray:
    """Egyenletes vagy véletlen alámintavételezés, max_n pont."""
    if pts.shape[0] <= max_n:
        return pts
    idx = np.linspace(0, pts.shape[0] - 1, max_n, dtype=int)
    return pts[idx]


def _transform_points(
    cur_pts: np.ndarray,
    dx: float,
    dy: float,
    dtheta: float,
) -> np.ndarray:
    c, s = math.cos(dtheta), math.sin(dtheta)
    tx = c * cur_pts[:, 0] - s * cur_pts[:, 1] + dx
    ty = s * cur_pts[:, 0] + c * cur_pts[:, 1] + dy
    return np.column_stack((tx, ty))


def _robust_match_metrics(
    tree: cKDTree,
    cur_pts: np.ndarray,
    dx: float,
    dy: float,
    dtheta: float,
    *,
    inlier_distance_m: float,
    trim_fraction: float,
    sector_count: int,
    include_support: bool = True,
) -> Dict[str, Any]:
    """
    Deterministic robust point-to-map score.

    The objective trims only the worst bounded fraction and clips every
    retained residual.  Dynamic returns therefore cannot dominate the pose,
    while inlier ratio and angular support remain separately measurable for
    confidence calibration.
    """
    if cur_pts.size == 0:
        return {
            "cost": 1e6,
            "rmse_m": 1e3,
            "inlier_count": 0,
            "inlier_ratio": 0.0,
            "sector_coverage": 0.0,
        }

    trans = _transform_points(cur_pts, dx, dy, dtheta)
    dists, _ = tree.query(trans, k=1, workers=1)
    dists = np.asarray(dists, dtype=float)
    finite = np.isfinite(dists)
    cutoff = max(1e-4, float(inlier_distance_m))
    bounded = np.where(finite, np.minimum(dists, cutoff), cutoff)
    # This function runs once for every matcher candidate.  Using np.clip for
    # one scalar repeatedly consumed a measurable part of the 45 ms live
    # matcher budget; keep NumPy for the vector clamp below, but use the
    # result-identical scalar path here.
    fraction = _clamp_scalar(float(trim_fraction), 0.50, 1.0)
    keep_count = max(3, min(int(bounded.size), int(math.ceil(bounded.size * fraction))))
    kept = np.partition(bounded, keep_count - 1)[:keep_count]
    cost = float(np.mean(kept * kept)) if kept.size else 1e6

    inliers = finite & (dists <= cutoff)
    inlier_count = int(np.count_nonzero(inliers))
    inlier_ratio = float(inlier_count / max(1, int(dists.size)))
    sectors = max(4, int(sector_count))
    occupied = 0
    if include_support and inlier_count > 0:
        angles = np.arctan2(cur_pts[inliers, 1], cur_pts[inliers, 0])
        bins = np.floor((angles + math.pi) * sectors / (2.0 * math.pi)).astype(int)
        bins = np.clip(bins, 0, sectors - 1)
        occupied = int(np.unique(bins).size)
    return {
        "cost": float(cost),
        "rmse_m": float(math.sqrt(max(0.0, cost))),
        "inlier_count": int(inlier_count),
        "inlier_ratio": float(inlier_ratio),
        "sector_coverage": float(occupied / sectors),
    }


def _cost_kdtree(
    tree: cKDTree,
    cur_pts: np.ndarray,
    dx: float,
    dy: float,
    dtheta: float,
) -> float:
    """Compatibility cost used only when no calibrated objective is supplied."""
    metrics = _robust_match_metrics(
        tree,
        cur_pts,
        dx,
        dy,
        dtheta,
        inlier_distance_m=0.20,
        trim_fraction=0.80,
        sector_count=12,
    )
    return float(metrics["cost"])


def _normalize_angle_rad(angle: float) -> float:
    return float((float(angle) + math.pi) % (2.0 * math.pi) - math.pi)


def _refine_local_pose(
    tree: cKDTree,
    cur_pts: np.ndarray,
    *,
    x0: float,
    y0: float,
    t0: float,
    dx_step: float,
    dy_step: float,
    dtheta_step: float,
    max_iters: int = 6,
    deadline_monotonic: Optional[float] = None,
    objective_fn: Optional[Callable[[float, float, float], float]] = None,
) -> Tuple[float, float, float, float]:
    """
    Local deterministic refinement around grid optimum.
    Uses shrinking coordinate-neighborhood search to reduce quantization jumps.
    """
    best_x = float(x0)
    best_y = float(y0)
    best_t = float(t0)
    def objective(x: float, y: float, theta: float) -> float:
        if objective_fn is not None:
            return float(objective_fn(float(x), float(y), float(theta)))
        return _cost_kdtree(tree, cur_pts, float(x), float(y), float(theta))

    best_cost = objective(best_x, best_y, best_t)

    sx = max(1e-4, abs(float(dx_step)) * 0.5)
    sy = max(1e-4, abs(float(dy_step)) * 0.5)
    st = max(1e-4, abs(float(dtheta_step)) * 0.5)
    min_sx = max(1e-5, abs(float(dx_step)) * 0.05)
    min_sy = max(1e-5, abs(float(dy_step)) * 0.05)
    min_st = max(1e-5, abs(float(dtheta_step)) * 0.05)
    eval_count = 0

    for _ in range(max(1, int(max_iters))):
        if deadline_monotonic is not None and time.monotonic() >= float(deadline_monotonic):
            break
        improved = False
        candidate_cost = best_cost
        candidate_x, candidate_y, candidate_t = best_x, best_y, best_t
        for ddx in (0.0, -sx, sx):
            for ddy in (0.0, -sy, sy):
                for ddt in (0.0, -st, st):
                    eval_count += 1
                    if (
                        deadline_monotonic is not None
                        and (eval_count & 0x1F) == 0
                        and time.monotonic() >= float(deadline_monotonic)
                    ):
                        return float(best_cost), float(best_x), float(best_y), float(
                            _normalize_angle_rad(best_t)
                        )
                    if ddx == 0.0 and ddy == 0.0 and ddt == 0.0:
                        continue
                    tx = float(best_x + ddx)
                    ty = float(best_y + ddy)
                    tt = _normalize_angle_rad(float(best_t + ddt))
                    cost = objective(tx, ty, tt)
                    if cost < candidate_cost:
                        candidate_cost = float(cost)
                        candidate_x = float(tx)
                        candidate_y = float(ty)
                        candidate_t = float(tt)
                        improved = True
        if improved:
            best_cost = float(candidate_cost)
            best_x = float(candidate_x)
            best_y = float(candidate_y)
            best_t = float(candidate_t)
        else:
            sx *= 0.5
            sy *= 0.5
            st *= 0.5
            if sx <= min_sx and sy <= min_sy and st <= min_st:
                break

    return float(best_cost), float(best_x), float(best_y), float(_normalize_angle_rad(best_t))


def _axis_observability_score(
    base_cost: float,
    negative_cost: float,
    positive_cost: float,
    *,
    scale: float,
) -> float:
    rise = max(
        0.0,
        0.5 * (float(negative_cost) + float(positive_cost)) - float(base_cost),
    )
    return float(1.0 - math.exp(-rise / max(1e-9, float(scale))))


def _normal_geometry_observability(
    tree: cKDTree,
    transformed_points: np.ndarray,
    *,
    pose_x: float,
    pose_y: float,
    inlier_distance_m: float,
    translation_step_m: float,
    rotation_step_rad: float,
) -> Optional[float]:
    """Estimate SE(2) rank from local map normals around correspondences."""
    map_points = np.asarray(getattr(tree, "data", np.zeros((0, 2))), dtype=float)
    if map_points.shape[0] < 6 or transformed_points.shape[0] < 6:
        return None
    neighbor_count = min(6, int(map_points.shape[0]))
    distances, indices = tree.query(
        transformed_points,
        k=neighbor_count,
        workers=1,
    )
    distances = np.asarray(distances, dtype=float)
    indices = np.asarray(indices, dtype=int)
    if distances.ndim != 2 or indices.ndim != 2:
        return None

    rows = []
    cutoff = max(0.01, float(inlier_distance_m))
    for point, point_distances, point_indices in zip(
        transformed_points,
        distances,
        indices,
    ):
        if not np.isfinite(point_distances[0]) or float(point_distances[0]) > cutoff:
            continue
        neighbors = map_points[point_indices]
        centered = neighbors - np.mean(neighbors, axis=0)
        covariance = centered.T @ centered / max(1, int(neighbors.shape[0]))
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        if not np.isfinite(eigenvalues).all() or float(eigenvalues[-1]) <= 1e-10:
            continue
        # A local line has a stable surface normal. Isotropic clusters do not.
        if float(eigenvalues[0] / eigenvalues[-1]) > 0.35:
            continue
        normal = eigenvectors[:, 0]
        rel_x = float(point[0]) - float(pose_x)
        rel_y = float(point[1]) - float(pose_y)
        yaw_derivative = float(normal[0]) * (-rel_y) + float(normal[1]) * rel_x
        rows.append(
            (
                float(normal[0]) * float(translation_step_m),
                float(normal[1]) * float(translation_step_m),
                yaw_derivative * float(rotation_step_rad),
            )
        )
    if len(rows) < 6:
        return None
    jacobian = np.asarray(rows, dtype=float)
    information = jacobian.T @ jacobian / max(1, int(jacobian.shape[0]))
    eigenvalues = np.linalg.eigvalsh(information)
    if not np.isfinite(eigenvalues).all() or float(eigenvalues[-1]) <= 1e-12:
        return 0.0
    rank_ratio = max(0.0, float(eigenvalues[0] / eigenvalues[-1]))
    rank_score = float(np.clip(rank_ratio / 0.03, 0.0, 1.0))
    support_score = float(np.clip(len(rows) / 12.0, 0.0, 1.0))
    return float(rank_score * support_score)


def _as_xy_array(points: Any) -> np.ndarray:
    """
    Convert arbitrary XY point container to an (N,2) float ndarray.
    """
    if points is None:
        return np.zeros((0, 2), dtype=float)
    arr = np.asarray(points, dtype=float)
    if arr.ndim != 2 or arr.shape[1] != 2:
        return np.zeros((0, 2), dtype=float)
    if arr.size == 0:
        return np.zeros((0, 2), dtype=float)
    finite = np.isfinite(arr).all(axis=1)
    if not np.any(finite):
        return np.zeros((0, 2), dtype=float)
    return np.asarray(arr[finite], dtype=float)


def match_scan_to_map(
    map_points_xy: Sequence[Sequence[float]] | np.ndarray,
    current_scan: List[dict],
    *,
    seed_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    dx_range: Tuple[float, float] = (-0.25, 0.25),
    dy_range: Tuple[float, float] = (-0.25, 0.25),
    dtheta_range: Tuple[float, float] = (-0.15, 0.15),
    dx_step: float = 0.03,
    dy_step: float = 0.03,
    dtheta_step: float = 0.04,
    max_points: int = 80,
    dist_in_m: bool = False,
    min_dist_m: float = 0.05,
    max_dist_m: Optional[float] = None,
    min_points: int = 10,
    deadline_monotonic: Optional[float] = None,
    stats: Optional[Dict[str, Any]] = None,
    seed_translation_prior_weight: float = 0.0,
    seed_rotation_prior_weight: float = 0.0,
    robust_inlier_distance_m: float = 0.18,
    robust_trim_fraction: float = 0.80,
    confidence_residual_scale_m: float = 0.15,
    confidence_sector_count: int = 12,
    confidence_target_sector_coverage: float = 0.50,
    ambiguity_translation_m: float = 0.08,
    ambiguity_rotation_rad: float = 0.12,
    ambiguity_margin_scale: float = 0.20,
    ambiguity_residual_margin_scale_m: float = 0.04,
    ambiguity_basin_top_k: int = 3,
    ambiguity_basin_refine_iters: int = 2,
    ambiguity_basin_barrier_scale: Optional[float] = None,
    observability_translation_step_m: Optional[float] = None,
    observability_rotation_step_rad: Optional[float] = None,
    observability_cost_scale: float = 0.0004,
) -> Tuple[float, float, float, float]:
    """
    Match current scan to a global/local map point cloud (scan-to-map).

    Returns absolute pose estimate (x, y, theta) and confidence.
    """
    map_pts = _as_xy_array(map_points_xy)
    cur_pts = _scan_to_xy(
        current_scan,
        dist_m=dist_in_m,
        min_dist_m=min_dist_m,
        max_dist_m=max_dist_m,
    )

    min_pts = max(3, int(min_points))
    if map_pts.shape[0] < min_pts or cur_pts.shape[0] < min_pts:
        sx, sy, sth = seed_pose
        return float(sx), float(sy), float(sth), 0.0

    map_tree = cKDTree(map_pts, compact_nodes=True, balanced_tree=True)
    cur_pts = _subsample(cur_pts, max_points)

    try:
        sx = float(seed_pose[0])
        sy = float(seed_pose[1])
        sth = float(seed_pose[2])
    except Exception:
        sx, sy, sth = 0.0, 0.0, 0.0

    deadline = float(deadline_monotonic) if deadline_monotonic is not None else None
    timed_out = False
    deadline_stage: Optional[str] = None
    evaluated_candidates = 0
    translation_prior_weight = max(0.0, float(seed_translation_prior_weight))
    rotation_prior_weight = max(0.0, float(seed_rotation_prior_weight))
    inlier_distance_m = max(0.01, float(robust_inlier_distance_m))
    trim_fraction = _clamp_scalar(float(robust_trim_fraction), 0.50, 1.0)
    sector_count = max(4, int(confidence_sector_count))
    candidate_samples: List[Dict[str, Any]] = []

    def deadline_reached(stage: str) -> bool:
        nonlocal timed_out, deadline_stage
        if deadline is None or time.monotonic() < deadline:
            return False
        timed_out = True
        if deadline_stage is None:
            deadline_stage = str(stage)
        return True

    def seed_prior_cost(x: float, y: float, theta: float) -> float:
        dx = float(x) - sx
        dy = float(y) - sy
        dtheta = _normalize_angle_rad(float(theta) - sth)
        return float(
            translation_prior_weight * (dx * dx + dy * dy)
            + rotation_prior_weight * dtheta * dtheta
        )

    def objective(
        x: float,
        y: float,
        theta: float,
        *,
        stage: str = "search",
    ) -> float:
        nonlocal evaluated_candidates
        metrics = _robust_match_metrics(
            map_tree,
            cur_pts,
            float(x),
            float(y),
            float(theta),
            inlier_distance_m=inlier_distance_m,
            trim_fraction=trim_fraction,
            sector_count=sector_count,
            include_support=False,
        )
        scan_cost = float(metrics["cost"])
        prior_cost = seed_prior_cost(float(x), float(y), float(theta))
        candidate_samples.append(
            {
                "scan_cost": float(scan_cost),
                "prior_cost": float(prior_cost),
                "objective_cost": float(scan_cost + prior_cost),
                "x": float(x),
                "y": float(y),
                "theta": _normalize_angle_rad(float(theta)),
                "stage": str(stage),
            }
        )
        evaluated_candidates += 1
        return float(scan_cost + prior_cost)

    def search_grid(
        x0: float,
        x1: float,
        xs: float,
        y0: float,
        y1: float,
        ys: float,
        t0: float,
        t1: float,
        ts: float,
        seed: Tuple[float, float, float],
        stage: str,
    ) -> Tuple[float, float, float, float]:
        if deadline_reached(stage):
            seed_x, seed_y, seed_t = seed
            seed_cost = objective(
                float(seed_x),
                float(seed_y),
                float(seed_t),
                stage=f"{stage}_seed",
            )
            return seed_cost, float(seed_x), float(seed_y), float(seed_t)
        xs = max(1e-6, float(xs))
        ys = max(1e-6, float(ys))
        ts = max(1e-6, float(ts))
        x_vals = np.arange(float(x0), float(x1) + 1e-9, xs)
        y_vals = np.arange(float(y0), float(y1) + 1e-9, ys)
        t_vals = np.arange(float(t0), float(t1) + 1e-9, ts)

        best_x, best_y, best_t = seed
        best_cost = objective(
            float(best_x),
            float(best_y),
            float(best_t),
            stage=f"{stage}_seed",
        )
        for x in x_vals:
            for y in y_vals:
                for theta in t_vals:
                    if deadline_reached(stage):
                        return best_cost, best_x, best_y, best_t
                    cost = objective(
                        float(x),
                        float(y),
                        float(theta),
                        stage=stage,
                    )
                    if cost < best_cost:
                        best_cost = float(cost)
                        best_x = float(x)
                        best_y = float(y)
                        best_t = float(theta)
        return best_cost, best_x, best_y, best_t

    best_cost, best_x, best_y, best_t = search_grid(
        sx + dx_range[0],
        sx + dx_range[1],
        dx_step * 4.0,
        sy + dy_range[0],
        sy + dy_range[1],
        dy_step * 4.0,
        sth + dtheta_range[0],
        sth + dtheta_range[1],
        dtheta_step * 4.0,
        (sx, sy, sth),
        "coarse",
    )
    if not timed_out:
        best_cost, best_x, best_y, best_t = search_grid(
            best_x - dx_step * 2.0,
            best_x + dx_step * 2.0,
            dx_step * 2.0,
            best_y - dy_step * 2.0,
            best_y + dy_step * 2.0,
            dy_step * 2.0,
            best_t - dtheta_step * 2.0,
            best_t + dtheta_step * 2.0,
            dtheta_step * 2.0,
            (best_x, best_y, best_t),
            "medium",
        )
    if not timed_out:
        best_cost, best_x, best_y, best_t = search_grid(
            best_x - dx_step,
            best_x + dx_step,
            dx_step,
            best_y - dy_step,
            best_y + dy_step,
            dy_step,
            best_t - dtheta_step,
            best_t + dtheta_step,
            dtheta_step,
            (best_x, best_y, best_t),
            "fine",
        )
    if not timed_out:
        best_cost, best_x, best_y, best_t = _refine_local_pose(
            map_tree,
            cur_pts,
            x0=best_x,
            y0=best_y,
            t0=best_t,
            dx_step=dx_step,
            dy_step=dy_step,
            dtheta_step=dtheta_step,
            max_iters=3,
            deadline_monotonic=deadline,
            objective_fn=lambda x, y, theta: objective(
                x,
                y,
                theta,
                stage="winner_refine",
            ),
        )
        deadline_reached("winner_refine")

    best_t = (float(best_t) + math.pi) % (2.0 * math.pi) - math.pi
    best_metrics = _robust_match_metrics(
        map_tree,
        cur_pts,
        best_x,
        best_y,
        best_t,
        inlier_distance_m=inlier_distance_m,
        trim_fraction=trim_fraction,
        sector_count=sector_count,
    )
    best_scan_cost = float(best_metrics["cost"])
    best_prior_cost = seed_prior_cost(best_x, best_y, best_t)

    def scan_cost_only(x: float, y: float, theta: float) -> float:
        return float(
            _robust_match_metrics(
                map_tree,
                cur_pts,
                x,
                y,
                theta,
                inlier_distance_m=inlier_distance_m,
                trim_fraction=trim_fraction,
                sector_count=sector_count,
                include_support=False,
            )["cost"]
        )

    def evaluated_scan_candidate(
        x: float,
        y: float,
        theta: float,
        *,
        stage: str,
    ) -> Dict[str, Any]:
        objective(float(x), float(y), float(theta), stage=stage)
        return dict(candidate_samples[-1])

    translation_separation = max(1e-4, float(ambiguity_translation_m))
    rotation_separation = max(1e-4, float(ambiguity_rotation_rad))
    barrier_scale = max(
        1e-9,
        float(
            ambiguity_basin_barrier_scale
            if ambiguity_basin_barrier_scale is not None
            else observability_cost_scale
        ),
    )
    basin_top_k = max(1, min(8, int(ambiguity_basin_top_k)))
    basin_refine_iters = max(1, min(6, int(ambiguity_basin_refine_iters)))
    x_bounds = (sx + float(dx_range[0]), sx + float(dx_range[1]))
    y_bounds = (sy + float(dy_range[0]), sy + float(dy_range[1]))
    t_bounds = (sth + float(dtheta_range[0]), sth + float(dtheta_range[1]))

    def pose_separation(
        candidate: Dict[str, Any],
        reference: Dict[str, Any],
    ) -> Tuple[float, float]:
        translation_delta = math.hypot(
            float(candidate["x"]) - float(reference["x"]),
            float(candidate["y"]) - float(reference["y"]),
        )
        rotation_delta = abs(
            _normalize_angle_rad(
                float(candidate["theta"]) - float(reference["theta"])
            )
        )
        return float(translation_delta), float(rotation_delta)

    def refine_scan_basin(seed: Dict[str, Any]) -> Dict[str, Any]:
        best = evaluated_scan_candidate(
            float(seed["x"]),
            float(seed["y"]),
            float(seed["theta"]),
            stage="basin_refine_seed",
        )
        step_x = max(0.005, abs(float(dx_step)))
        step_y = max(0.005, abs(float(dy_step)))
        step_t = max(0.005, abs(float(dtheta_step)))
        refine_iterations = (
            min(6, basin_refine_iters + 2)
            if str(seed.get("stage")) == "basin_offset_diagonal"
            else max(1, basin_refine_iters - 1)
        )
        for _ in range(refine_iterations):
            if deadline_reached("basin_refine"):
                break
            improved = False
            for ddx, ddy, ddt in (
                (-step_x, 0.0, 0.0),
                (step_x, 0.0, 0.0),
                (0.0, -step_y, 0.0),
                (0.0, step_y, 0.0),
                (0.0, 0.0, -step_t),
                (0.0, 0.0, step_t),
            ):
                if deadline_reached("basin_refine"):
                    break
                candidate_x = _clamp_scalar(
                    float(best["x"]) + ddx,
                    *x_bounds,
                )
                candidate_y = _clamp_scalar(
                    float(best["y"]) + ddy,
                    *y_bounds,
                )
                raw_theta = float(best["theta"]) + ddt
                candidate_theta = _normalize_angle_rad(
                    _clamp_scalar(raw_theta, *t_bounds)
                )
                candidate = evaluated_scan_candidate(
                    candidate_x,
                    candidate_y,
                    candidate_theta,
                    stage="basin_refine",
                )
                if float(candidate["scan_cost"]) + 1e-15 < float(best["scan_cost"]):
                    best = candidate
                    improved = True
            if not improved:
                step_x *= 0.5
                step_y *= 0.5
                step_t *= 0.5
        return dict(best)

    best_candidate = {
        "scan_cost": float(best_scan_cost),
        "prior_cost": float(best_prior_cost),
        "objective_cost": float(best_scan_cost + best_prior_cost),
        "x": float(best_x),
        "y": float(best_y),
        "theta": float(best_t),
        "stage": "winner",
    }

    # Coarse samples span the full configured SE(2) window. Select separated
    # low-cost seeds, refine each with scan-only cost, then require an actual
    # saddle/barrier between the refined pose and the winner. A point elsewhere
    # on one broad monotonic valley is therefore not a global competitor.
    if not timed_out:
        for offset_x in (-translation_separation, 0.0, translation_separation):
            for offset_y in (-translation_separation, 0.0, translation_separation):
                if offset_x == 0.0 and offset_y == 0.0:
                    continue
                if deadline_reached("basin_offset_grid"):
                    break
                candidate_x = float(best_x + offset_x)
                candidate_y = float(best_y + offset_y)
                if not (
                    x_bounds[0] <= candidate_x <= x_bounds[1]
                    and y_bounds[0] <= candidate_y <= y_bounds[1]
                ):
                    continue
                evaluated_scan_candidate(
                    candidate_x,
                    candidate_y,
                    best_t,
                    stage=(
                        "basin_offset_diagonal"
                        if offset_x != 0.0 and offset_y != 0.0
                        else "basin_offset_axis"
                    ),
                )
            if timed_out:
                break
    if not timed_out:
        for offset_t in (-rotation_separation, rotation_separation):
            candidate_t = float(best_t + offset_t)
            if not t_bounds[0] <= candidate_t <= t_bounds[1]:
                continue
            evaluated_scan_candidate(
                best_x,
                best_y,
                candidate_t,
                stage="basin_offset_yaw",
            )

    seed_candidate_stages = {
        "coarse",
        "basin_offset_axis",
        "basin_offset_diagonal",
        "basin_offset_yaw",
    }
    all_seed_candidates = [
        dict(item)
        for item in candidate_samples
        if str(item.get("stage")) in seed_candidate_stages
    ]
    coarse_candidates = sorted(
        all_seed_candidates,
        key=lambda item: float(item["scan_cost"]),
    )
    diagonal_candidates = sorted(
        (
            item
            for item in all_seed_candidates
            if str(item.get("stage")) == "basin_offset_diagonal"
        ),
        key=lambda item: float(item["scan_cost"]),
    )
    yaw_candidates = sorted(
        (
            item
            for item in all_seed_candidates
            if str(item.get("stage")) == "basin_offset_yaw"
        ),
        key=lambda item: float(item["scan_cost"]),
    )
    reserved_candidates = []
    if diagonal_candidates:
        reserved_candidates.append(diagonal_candidates[0])
    if yaw_candidates:
        reserved_candidates.append(yaw_candidates[0])
    reserved_ids = {id(item) for item in reserved_candidates}
    coarse_candidates = reserved_candidates + [
        item for item in coarse_candidates if id(item) not in reserved_ids
    ]
    basin_seeds: List[Dict[str, Any]] = []
    for candidate in coarse_candidates:
        candidate_delta_m, candidate_delta_rad = pose_separation(
            candidate,
            best_candidate,
        )
        if (
            candidate_delta_m < translation_separation
            and candidate_delta_rad < rotation_separation
        ):
            continue
        if any(
            pose_separation(candidate, selected)[0] < translation_separation
            and pose_separation(candidate, selected)[1] < rotation_separation
            for selected in basin_seeds
        ):
            continue
        basin_seeds.append(candidate)
        if len(basin_seeds) >= basin_top_k:
            break

    distinct_basins: List[Dict[str, Any]] = []
    basin_candidate_summaries: List[Dict[str, Any]] = []
    for basin_index, seed in enumerate(basin_seeds, start=1):
        if deadline_reached("basin_refine"):
            break
        refined = refine_scan_basin(seed)
        translation_delta, rotation_delta = pose_separation(refined, best_candidate)
        barrier_cost = max(float(best_scan_cost), float(refined["scan_cost"]))
        barrier_samples = 0
        for alpha in (0.25, 0.50, 0.75):
            if deadline_reached("basin_barrier"):
                break
            interp_x = float(best_x + alpha * (float(refined["x"]) - best_x))
            interp_y = float(best_y + alpha * (float(refined["y"]) - best_y))
            interp_theta = _normalize_angle_rad(
                best_t
                + alpha
                * _normalize_angle_rad(float(refined["theta"]) - float(best_t))
            )
            path_candidate = evaluated_scan_candidate(
                interp_x,
                interp_y,
                interp_theta,
                stage="basin_barrier",
            )
            barrier_cost = max(barrier_cost, float(path_candidate["scan_cost"]))
            barrier_samples += 1
        endpoint_cost = max(float(best_scan_cost), float(refined["scan_cost"]))
        barrier_rise = max(0.0, float(barrier_cost) - endpoint_cost)
        barrier_score = float(np.clip(barrier_rise / barrier_scale, 0.0, 1.0))
        separated = bool(
            translation_delta >= translation_separation
            or rotation_delta >= rotation_separation
        )
        distinct = bool(
            separated
            and barrier_samples == 3
            and barrier_rise >= barrier_scale
            and not timed_out
        )
        summary = {
            "basin_id": int(basin_index),
            "seed_pose": {
                "x": float(seed["x"]),
                "y": float(seed["y"]),
                "theta": float(seed["theta"]),
            },
            "pose": {
                "x": float(refined["x"]),
                "y": float(refined["y"]),
                "theta": float(refined["theta"]),
            },
            "scan_cost": float(refined["scan_cost"]),
            "prior_cost": float(refined["prior_cost"]),
            "objective_cost": float(refined["objective_cost"]),
            "translation_delta_m": float(translation_delta),
            "rotation_delta_rad": float(rotation_delta),
            "barrier_rise": float(barrier_rise),
            "barrier_score": float(barrier_score),
            "distinct": bool(distinct),
        }
        basin_candidate_summaries.append(summary)
        if distinct:
            distinct_basins.append({**refined, **summary})

    competitor = (
        min(distinct_basins, key=lambda item: float(item["scan_cost"]))
        if distinct_basins
        else None
    )
    competitor_cost = None if competitor is None else float(competitor["scan_cost"])
    competitor_prior_cost = (
        None if competitor is None else float(competitor["prior_cost"])
    )
    competitor_objective_cost = (
        None if competitor is None else float(competitor["objective_cost"])
    )

    if competitor_cost is None:
        ambiguity_margin = 1.0
        ambiguity_residual_margin_m = None
        uniqueness_score = 1.0
        posterior_ambiguity_margin = 1.0
        posterior_residual_margin_m = None
        posterior_uniqueness_score = 1.0
    else:
        ambiguity_margin = max(
            0.0,
            (float(competitor_cost) - float(best_scan_cost))
            / max(1e-9, float(competitor_cost) + float(best_scan_cost)),
        )
        ambiguity_residual_margin_m = max(
            0.0,
            math.sqrt(max(0.0, float(competitor_cost)))
            - math.sqrt(max(0.0, float(best_scan_cost))),
        )
        uniqueness_score = float(
            min(
                np.clip(
                    ambiguity_margin / max(1e-6, float(ambiguity_margin_scale)),
                    0.0,
                    1.0,
                ),
                np.clip(
                    ambiguity_residual_margin_m
                    / max(1e-6, float(ambiguity_residual_margin_scale_m)),
                    0.0,
                    1.0,
                ),
            )
        )

        best_objective_cost = float(best_scan_cost + best_prior_cost)
        posterior_ambiguity_margin = max(
            0.0,
            (float(competitor_objective_cost) - best_objective_cost)
            / max(1e-9, float(competitor_objective_cost) + best_objective_cost),
        )
        posterior_residual_margin_m = max(
            0.0,
            math.sqrt(max(0.0, float(competitor_objective_cost)))
            - math.sqrt(max(0.0, best_objective_cost)),
        )
        posterior_uniqueness_score = float(
            min(
                np.clip(
                    posterior_ambiguity_margin
                    / max(1e-6, float(ambiguity_margin_scale)),
                    0.0,
                    1.0,
                ),
                np.clip(
                    posterior_residual_margin_m
                    / max(1e-6, float(ambiguity_residual_margin_scale_m)),
                    0.0,
                    1.0,
                ),
            )
        )

    obs_translation_step = max(
        0.005,
        float(
            observability_translation_step_m
            if observability_translation_step_m is not None
            else max(abs(float(dx_step)), abs(float(dy_step)))
        ),
    )
    obs_rotation_step = max(
        0.005,
        float(
            observability_rotation_step_rad
            if observability_rotation_step_rad is not None
            else abs(float(dtheta_step))
        ),
    )

    def auxiliary_scan_cost(x: float, y: float, theta: float, stage: str) -> float:
        if deadline_reached(stage):
            return float(best_scan_cost)
        return float(
            evaluated_scan_candidate(
                float(x),
                float(y),
                float(theta),
                stage=stage,
            )["scan_cost"]
        )

    obs_scale = max(1e-9, float(observability_cost_scale))
    observability_x = _axis_observability_score(
        best_scan_cost,
        auxiliary_scan_cost(
            best_x - obs_translation_step, best_y, best_t, "observability"
        ),
        auxiliary_scan_cost(
            best_x + obs_translation_step, best_y, best_t, "observability"
        ),
        scale=obs_scale,
    )
    observability_y = _axis_observability_score(
        best_scan_cost,
        auxiliary_scan_cost(
            best_x, best_y - obs_translation_step, best_t, "observability"
        ),
        auxiliary_scan_cost(
            best_x, best_y + obs_translation_step, best_t, "observability"
        ),
        scale=obs_scale,
    )
    observability_yaw = _axis_observability_score(
        best_scan_cost,
        auxiliary_scan_cost(
            best_x, best_y, best_t - obs_rotation_step, "observability"
        ),
        auxiliary_scan_cost(
            best_x, best_y, best_t + obs_rotation_step, "observability"
        ),
        scale=obs_scale,
    )
    observability_score = float(
        (max(0.0, observability_x)
         * max(0.0, observability_y)
         * max(0.0, observability_yaw))
        ** (1.0 / 3.0)
    )
    normal_observability = None
    if not deadline_reached("normal_observability"):
        normal_observability = _normal_geometry_observability(
            map_tree,
            _transform_points(cur_pts, best_x, best_y, best_t),
            pose_x=best_x,
            pose_y=best_y,
            inlier_distance_m=inlier_distance_m,
            translation_step_m=obs_translation_step,
            rotation_step_rad=obs_rotation_step,
        )
        deadline_reached("normal_observability")
    if normal_observability is not None:
        observability_score = float(
            math.sqrt(max(0.0, observability_score * normal_observability))
        )

    residual_scale = max(1e-4, float(confidence_residual_scale_m))
    residual_score = float(
        1.0 / (1.0 + (float(best_metrics["rmse_m"]) / residual_scale) ** 2)
    )
    inlier_score = float(
        np.clip((float(best_metrics["inlier_ratio"]) - 0.20) / 0.75, 0.0, 1.0)
    )
    coverage_score = float(
        np.clip(
            float(best_metrics["sector_coverage"])
            / max(1e-6, float(confidence_target_sector_coverage)),
            0.0,
            1.0,
        )
    )
    base_quality = float(
        residual_score * math.sqrt(max(0.0, inlier_score * coverage_score))
    )
    ambiguity_factor = float(0.05 + 0.95 * uniqueness_score)
    observability_factor = float(0.20 + 0.80 * observability_score)
    measurement_confidence = float(
        np.clip(base_quality * observability_factor, 0.0, 1.0)
    )
    localization_integrity_score = float(
        np.clip(base_quality * ambiguity_factor, 0.0, 1.0)
    )
    confidence = float(measurement_confidence)
    combined_confidence = float(
        min(measurement_confidence, localization_integrity_score)
    )

    degeneracy_reasons = []
    if float(best_metrics["inlier_ratio"]) < 0.35:
        degeneracy_reasons.append("low_inlier_ratio")
    if float(best_metrics["sector_coverage"]) < 0.25:
        degeneracy_reasons.append("partial_angular_support")
    if uniqueness_score < 0.20:
        degeneracy_reasons.append("ambiguous_alternative")
    if observability_score < 0.12:
        degeneracy_reasons.append("weak_observability")
    if timed_out:
        integrity_state = "INCOMPLETE"
    elif float(best_metrics["inlier_ratio"]) < 0.35 or float(
        best_metrics["sector_coverage"]
    ) < 0.25:
        integrity_state = "INSUFFICIENT_SUPPORT"
    elif uniqueness_score < 0.20:
        integrity_state = "MULTIMODAL"
    elif observability_score < 0.12:
        integrity_state = "DEGRADED_OBSERVABILITY"
    else:
        integrity_state = "OK"
    if timed_out:
        degeneracy_reasons.append("budget_exceeded")
        confidence = 0.0
        measurement_confidence = 0.0
        localization_integrity_score = 0.0
        combined_confidence = 0.0

    stage_evaluations: Dict[str, int] = {}
    for sample in candidate_samples:
        sample_stage = str(sample.get("stage", "unknown"))
        stage_evaluations[sample_stage] = stage_evaluations.get(sample_stage, 0) + 1

    if isinstance(stats, dict):
        stats["confidence_model"] = SCAN_MATCH_CONFIDENCE_MODEL
        stats["integrity_model"] = SCAN_MATCH_INTEGRITY_MODEL
        stats["timed_out"] = bool(timed_out)
        stats["search_complete"] = bool(not timed_out)
        stats["deadline_stage"] = deadline_stage
        stats["evaluated_candidates"] = int(evaluated_candidates)
        stats["stage_evaluations"] = dict(stage_evaluations)
        stats["scan_cost"] = float(best_scan_cost)
        stats["robust_rmse_m"] = float(best_metrics["rmse_m"])
        stats["inlier_count"] = int(best_metrics["inlier_count"])
        stats["inlier_ratio"] = float(best_metrics["inlier_ratio"])
        stats["sector_coverage"] = float(best_metrics["sector_coverage"])
        stats["competitor_scan_cost"] = (
            None if competitor_cost is None else float(competitor_cost)
        )
        stats["competitor_prior_cost"] = competitor_prior_cost
        stats["competitor_objective_cost"] = competitor_objective_cost
        stats["competitor_pose"] = (
            None if competitor is None else dict(competitor["pose"])
        )
        stats["competitor_translation_delta_m"] = (
            None
            if competitor is None
            else float(competitor["translation_delta_m"])
        )
        stats["competitor_rotation_delta_rad"] = (
            None
            if competitor is None
            else float(competitor["rotation_delta_rad"])
        )
        stats["competitor_basin_id"] = (
            None if competitor is None else int(competitor["basin_id"])
        )
        stats["competitor_stage"] = (
            None if competitor is None else "basin_refine"
        )
        stats["competitor_barrier_rise"] = (
            None if competitor is None else float(competitor["barrier_rise"])
        )
        stats["competitor_barrier_score"] = (
            None if competitor is None else float(competitor["barrier_score"])
        )
        stats["basin_candidate_count"] = int(len(basin_candidate_summaries))
        stats["distinct_basin_count"] = int(len(distinct_basins))
        stats["basin_candidates"] = list(basin_candidate_summaries)
        stats["ambiguity_margin"] = float(ambiguity_margin)
        stats["ambiguity_residual_margin_m"] = (
            None
            if ambiguity_residual_margin_m is None
            else float(ambiguity_residual_margin_m)
        )
        stats["uniqueness_score"] = float(uniqueness_score)
        stats["measurement_uniqueness_score"] = float(uniqueness_score)
        stats["posterior_ambiguity_margin"] = float(posterior_ambiguity_margin)
        stats["posterior_residual_margin_m"] = (
            None
            if posterior_residual_margin_m is None
            else float(posterior_residual_margin_m)
        )
        stats["posterior_uniqueness_score"] = float(posterior_uniqueness_score)
        stats["global_multimodality_score"] = float(1.0 - uniqueness_score)
        stats["observability_x"] = float(observability_x)
        stats["observability_y"] = float(observability_y)
        stats["observability_yaw"] = float(observability_yaw)
        stats["observability_score"] = float(observability_score)
        stats["normal_observability_score"] = (
            None
            if normal_observability is None
            else float(normal_observability)
        )
        stats["residual_score"] = float(residual_score)
        stats["inlier_score"] = float(inlier_score)
        stats["coverage_score"] = float(coverage_score)
        stats["fit_quality"] = float(base_quality)
        stats["measurement_confidence"] = float(measurement_confidence)
        stats["localization_integrity_score"] = float(
            localization_integrity_score
        )
        stats["combined_confidence"] = float(combined_confidence)
        stats["integrity_state"] = str(integrity_state)
        stats["covariance_proxy"] = {
            "x_variance_m2": float(
                obs_translation_step * obs_translation_step
                / max(0.05, observability_x)
            ),
            "y_variance_m2": float(
                obs_translation_step * obs_translation_step
                / max(0.05, observability_y)
            ),
            "yaw_variance_rad2": float(
                obs_rotation_step * obs_rotation_step
                / max(0.05, observability_yaw)
            ),
        }
        stats["confidence"] = float(confidence)
        stats["degenerate"] = bool(degeneracy_reasons)
        stats["degeneracy_reasons"] = list(degeneracy_reasons)
        stats["seed_prior_cost"] = float(best_prior_cost)
        stats["objective_cost"] = float(best_scan_cost + best_prior_cost)
    return float(best_x), float(best_y), float(best_t), float(confidence)


def relative_to_absolute(
    px_prev: float,
    py_prev: float,
    theta_prev: float,
    dx: float,
    dy: float,
    dtheta: float,
) -> Tuple[float, float, float]:
    """
    Relatív (dx, dy, dtheta) a reference (prev) robot frame-ben
    -> abszolút (x, y, theta) világ koordinátában.
    """
    x = px_prev + dx * math.cos(theta_prev) - dy * math.sin(theta_prev)
    y = py_prev + dx * math.sin(theta_prev) + dy * math.cos(theta_prev)
    theta = (theta_prev + dtheta + math.pi) % (2.0 * math.pi) - math.pi
    return x, y, theta


def absolute_to_relative(
    px_prev: float,
    py_prev: float,
    theta_prev: float,
    px_cur: float,
    py_cur: float,
    theta_cur: float,
) -> Tuple[float, float, float]:
    """
    Abszolút (x, y, theta) világ koordináták -> relatív (dx, dy, dtheta) 
    a 'prev' robot frame-ben. (relative_to_absolute inverze).
    """
    dx_global = px_cur - px_prev
    dy_global = py_cur - py_prev
    
    # Forgatás a prev robot frame-be (theta_prev szöggel visszaforgatva)
    # R(-theta_prev) * [dx_g, dy_g]
    c = math.cos(theta_prev)
    s = math.sin(theta_prev)
    dx = dx_global * c + dy_global * s
    dy = -dx_global * s + dy_global * c
    
    dtheta = (theta_cur - theta_prev + math.pi) % (2.0 * math.pi) - math.pi
    return dx, dy, dtheta
