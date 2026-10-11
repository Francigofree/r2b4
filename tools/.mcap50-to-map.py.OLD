#!/usr/bin/env python3
"""Build a high-quality 2D global occupancy map from an R2B4 50 Hz MCAP.

Historical source-convention baseline (actual tool hash is exported per build):
  7a90206013d0723c1ad668231739eb19c3fc51ae

Usage:
  python3 mcap50-to-map.py xyz.mcap

Default strategy:
- require a finalized, integrity-valid 50 Hz capture with complete raw LiDAR;
- extract the complete /r2b4/raw_lidar stream and every-tick L3 local trajectory;
- detect stable stationary windows (the intended map-survey stop-and-scan mode);
- reject the settling prefix of each stop, then fuse repeated scans into a
  persistent/static keyframe cloud;
- register neighbouring keyframes with robust point-to-plane ICP;
- search conservative non-neighbour loop closures and verify them bidirectionally;
- optimize an SE(2) pose graph with SciPy robust least-squares;
- ray-trace the optimized static keyframes into a 5 cm occupancy map;
- write ROS-compatible PGM+YAML plus NPZ/JSON/CSV provenance artifacts.

This tool intentionally prefers correctness over forcing a map from bad data.
By default it rejects raw-LiDAR loss, truncation, queue supersede, discontinuous
localization, and captures that are not 50 Hz sensor-debug captures.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import json
import math
import os
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np
import scipy
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial import cKDTree

# R2B4 source constants / conventions used by this tool.
EXPECTED_CAPTURE_HZ = 50
RAW_TOPIC = "/r2b4/raw_lidar"
TICK_TOPIC = "/r2b4/tick"
RUNTIME_TOPIC = "/r2b4/runtime"
R2B4_SOURCE_MAIN = "7a90206013d0723c1ad668231739eb19c3fc51ae"

# Mapping defaults chosen for the stop-and-scan survey described in the design.
MAP_RESOLUTION_M = 0.05
MIN_RANGE_M = 0.08
MAX_RANGE_M = 12.0
STATIONARY_MAX_V_MPS = 0.03
STATIONARY_MAX_OMEGA_RAD_S = 0.05
STATIONARY_MIN_DURATION_S = 1.20
STATIONARY_SETTLE_S = 0.45
STATIONARY_END_GUARD_S = 0.10
MIN_SCANS_PER_KEYFRAME = 4
MAX_SCANS_PER_KEYFRAME = 28
FUSION_VOXEL_M = 0.04
FUSION_MIN_SCAN_FRACTION = 0.15
MAX_KEYFRAME_POINTS = 2600
ICP_MAX_POINTS = 1600
ICP_K_NEIGHBOURS = 6
ICP_MAX_ITERS = 24
ICP_MAX_CORRESPONDENCE_M = 0.28
SEQUENTIAL_MAX_RMSE_M = 0.065
SEQUENTIAL_MIN_INLIER_RATIO = 0.32
SEQUENTIAL_MIN_OBSERVABILITY = 0.045
LOOP_SEARCH_RADIUS_M = 2.0
LOOP_MIN_INDEX_SEPARATION = 3
LOOP_MAX_CANDIDATES_PER_NODE = 4
LOOP_MAX_RMSE_M = 0.050
LOOP_MIN_INLIER_RATIO = 0.45
LOOP_MIN_OBSERVABILITY = 0.09
LOOP_MAX_SEED_CORRECTION_M = 0.65
LOOP_MAX_SEED_CORRECTION_RAD = 0.65
LOOP_RECIPROCAL_MAX_M = 0.07
LOOP_RECIPROCAL_MAX_RAD = 0.10
LOG_ODDS_FREE = -0.42
LOG_ODDS_OCC = 0.90
LOG_ODDS_MIN = -4.0
LOG_ODDS_MAX = 4.0
OCCUPIED_PROB = 0.65
FREE_PROB = 0.35
MAP_MARGIN_M = 0.60


class MapBuildError(RuntimeError):
    pass


@dataclass(frozen=True)
class PoseSample:
    t_ns: int
    x: float
    y: float
    yaw: float
    v: float
    omega: float
    generation: int
    continuous: bool
    discontinuity: bool
    slip: bool
    local_sigma: float
    yaw_sigma: float
    observability: float
    local_translation: str
    heading: str
    source_ticks: tuple[dict[str, object], ...] = ()
    frame_id: str | None = None
    clock_epoch: str | None = None
    generation_recorded: bool = True


@dataclass(frozen=True)
class RawScan:
    revision: int
    start_ns: int
    t_ns: int
    end_ns: int
    points: np.ndarray  # columns: angle_deg, distance_m, quality
    sequence: int | None = None
    clock_epoch: str | None = None


@dataclass(frozen=True)
class StationaryWindow:
    start_ns: int
    end_ns: int
    usable_start_ns: int
    usable_end_ns: int
    generation: int
    sample_count: int


@dataclass
class Keyframe:
    index: int
    ref_ns: int
    window_start_ns: int
    window_end_ns: int
    generation: int
    seed_pose: tuple[float, float, float]
    cloud_local: np.ndarray
    raw_scan_count: int
    used_scan_count: int
    retained_voxel_count: int
    min_voxel_support: int
    local_sigma_m: float
    yaw_sigma_rad: float
    observability: float
    source_ticks: tuple[dict[str, object], ...] = ()
    source_scans: tuple[dict[str, object], ...] = ()
    source_frame_id: str | None = None
    source_clock_epoch: str | None = None
    generation_recorded: bool = True


@dataclass
class Edge:
    i: int
    j: int
    z: tuple[float, float, float]  # T_i^-1 * T_j
    sigma_t: float
    sigma_yaw: float
    kind: str
    metrics: dict[str, float]


def _find_repo_root() -> Path:
    starts = [Path.cwd(), Path(__file__).resolve().parent]
    seen: set[Path] = set()
    for start in starts:
        for candidate in (start, *start.parents):
            candidate = candidate.resolve()
            if candidate in seen:
                continue
            seen.add(candidate)
            if (candidate / "v3" / "mcap_reader.py").is_file():
                return candidate
    raise MapBuildError(
        "R2B4 repo not found. Put mcap50-to-map.py in the R2B4 repo root "
        "or run it from inside the repo."
    )


def _load_reader(path: Path):
    root = _find_repo_root()
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    try:
        from v3.mcap_reader import McapReader, McapReadError
    except Exception as exc:  # pragma: no cover - diagnostic boundary
        raise MapBuildError(f"cannot import R2B4 v3.mcap_reader: {exc}") from exc
    try:
        return root, McapReader(path), McapReadError
    except Exception as exc:
        raise MapBuildError(f"cannot open MCAP: {exc}") from exc


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def _json_sha256(value: object) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def _build_identity(reader: Any) -> dict[str, object]:
    """Record actual build inputs; never substitute the host's active config."""
    runtime = next((row for _, row in reader.iter_json_messages(topics=(RUNTIME_TOPIC,))
                    if isinstance(row, Mapping)), {})
    configuration = _as_mapping(runtime.get("configuration"))
    metadata = _as_mapping(runtime.get("metadata"))
    options = {name: value for name, value in globals().items()
               if name.isupper() and type(value) in (int, float)}
    options["optimizer"] = {"loss": "huber", "f_scale": 1.0, "max_nfev": 500,
                            "xtol": 1e-8, "ftol": 1e-8, "gtol": 1e-8}
    return {
        "tool": {"name": "mcap50-to-map.py", "sha256": _sha256(Path(__file__).resolve()),
                 "numpy_version": np.__version__, "scipy_version": scipy.__version__},
        "numeric_options": options,
        "numeric_options_sha256": _json_sha256(options),
        "capture_configuration_sha256": _json_sha256(configuration) if configuration else None,
        "capture_configuration_snapshot_id": configuration.get("snapshot_id"),
        "capture_runtime": metadata.get("runtime"),
        "runtime_pid": metadata.get("runtime_pid"),
        "session_id": metadata.get("session_id"),
        "calibration": {"source_configuration_sha256": _json_sha256(configuration) if configuration else None,
                        "extrinsic_identity": metadata.get("extrinsic_identity"),
                        "lidar_to_base_convention": "R2B4_RAW_LIDAR_MIRRORED_Y_V1",
                        "applied_transform": "x=d*cos(angle); y=-d*sin(angle)",
                        "extrinsic_measurement_verified": False},
    }


def _angle(a: float) -> float:
    return float((float(a) + math.pi) % (2.0 * math.pi) - math.pi)


def _pose_compose(a: Sequence[float], b: Sequence[float]) -> tuple[float, float, float]:
    ax, ay, at = map(float, a)
    bx, by, bt = map(float, b)
    c, s = math.cos(at), math.sin(at)
    return (ax + c * bx - s * by, ay + s * bx + c * by, _angle(at + bt))


def _pose_inverse(a: Sequence[float]) -> tuple[float, float, float]:
    x, y, t = map(float, a)
    c, s = math.cos(t), math.sin(t)
    return (-c * x - s * y, s * x - c * y, _angle(-t))


def _pose_relative(a: Sequence[float], b: Sequence[float]) -> tuple[float, float, float]:
    return _pose_compose(_pose_inverse(a), b)


def _pose_diff(a: Sequence[float], b: Sequence[float]) -> tuple[float, float, float]:
    d = _pose_relative(a, b)
    return (float(d[0]), float(d[1]), _angle(float(d[2])))


def _transform(points: np.ndarray, pose: Sequence[float]) -> np.ndarray:
    if points.size == 0:
        return np.zeros((0, 2), dtype=float)
    x, y, t = map(float, pose)
    c, s = math.cos(t), math.sin(t)
    out = np.empty_like(points, dtype=float)
    out[:, 0] = c * points[:, 0] - s * points[:, 1] + x
    out[:, 1] = s * points[:, 0] + c * points[:, 1] + y
    return out


def _even_subsample(points: np.ndarray, maximum: int) -> np.ndarray:
    if len(points) <= maximum:
        return points
    idx = np.linspace(0, len(points) - 1, maximum, dtype=int)
    return points[idx]


def _as_mapping(value: object) -> Mapping[str, object]:
    return value if isinstance(value, Mapping) else {}


def _quality_token(value: object) -> str:
    if isinstance(value, str):
        return value
    if isinstance(value, Mapping):
        # Defensive support for future enum encodings.
        for key in ("value", "name"):
            if isinstance(value.get(key), str):
                return str(value[key])
    return "UNKNOWN"


def _extract_pose_samples(reader: Any) -> list[PoseSample]:
    samples: list[PoseSample] = []
    channel = getattr(reader, "channels_by_topic", {}).get(TICK_TOPIC)
    clock_epoch = channel.metadata.get("clock_epoch") if channel else None
    for msg, payload in reader.iter_json_messages(topics=(TICK_TOPIC,)):
        if not isinstance(payload, Mapping):
            continue
        expected = _as_mapping(payload.get("expected"))
        layers = _as_mapping(expected.get("layers"))
        l3 = _as_mapping(layers.get("L3"))
        if not l3:
            continue
        t_ns = payload.get("monotonic_ns")
        if type(t_ns) is not int:
            continue
        local_pose = _as_mapping(l3.get("local_pose"))
        if local_pose:
            x, y, yaw = local_pose.get("x_m"), local_pose.get("y_m"), local_pose.get("yaw_rad")
        else:
            x, y, yaw = l3.get("x_m"), l3.get("y_m"), l3.get("yaw_rad")
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(float(v)) for v in (x, y, yaw)):
            continue
        v = l3.get("local_v_mps", l3.get("v_mps", 0.0))
        omega = l3.get("local_omega_rad_s", l3.get("omega_rad_s", 0.0))
        quality = _as_mapping(l3.get("localization_quality"))
        try:
            sample = PoseSample(
                t_ns=int(t_ns), x=float(x), y=float(y), yaw=_angle(float(yaw)),
                v=float(v or 0.0), omega=float(omega or 0.0),
                generation=int(quality.get("generation", 0)),
                continuous=bool(quality.get("local_pose_continuous", True)),
                discontinuity=bool(quality.get("pose_discontinuity", False)),
                slip=bool(quality.get("slip_suspected", False)),
                local_sigma=float(quality.get("local_sigma_m", 0.10)),
                yaw_sigma=float(quality.get("yaw_sigma_rad", 0.10)),
                observability=float(quality.get("observability", 0.0)),
                local_translation=_quality_token(quality.get("local_translation", "UNKNOWN")),
                heading=_quality_token(quality.get("heading", "UNKNOWN")),
                source_ticks=({"tick_id": payload.get("tick_id"),
                               "sequence": msg.sequence, "reference_time_ns": t_ns},),
                frame_id=local_pose.get("frame_id", l3.get("frame_id")),
                clock_epoch=clock_epoch,
                generation_recorded=type(quality.get("generation")) is int,
            )
        except (TypeError, ValueError, OverflowError):
            continue
        samples.append(sample)
    samples.sort(key=lambda p: p.t_ns)
    if len(samples) < 2:
        raise MapBuildError("capture does not contain enough L3 pose samples")
    return samples


def _extract_raw_scans(reader: Any) -> list[RawScan]:
    scans: list[RawScan] = []
    for msg, payload in reader.iter_json_messages(topics=(RAW_TOPIC,)):
        if not isinstance(payload, Mapping):
            continue
        if payload.get("points_truncated") is True:
            raise MapBuildError("raw LiDAR record is truncated; refusing map build")
        revision = payload.get("revision")
        start_ns = payload.get("scan_start_monotonic_ns")
        t_ns = payload.get("measurement_monotonic_ns")
        end_ns = payload.get("scan_end_monotonic_ns")
        raw = payload.get("points")
        if not (type(revision) is int and type(start_ns) is int and type(t_ns) is int and type(end_ns) is int):
            continue
        if not isinstance(raw, Sequence) or isinstance(raw, (str, bytes)):
            continue
        pts: list[tuple[float, float, float]] = []
        for row in raw:
            if not isinstance(row, Sequence) or isinstance(row, (str, bytes)) or len(row) < 3:
                continue
            try:
                a, d, q = float(row[0]), float(row[1]), float(row[2])
            except (TypeError, ValueError):
                continue
            if not all(math.isfinite(v) for v in (a, d, q)):
                continue
            if MIN_RANGE_M <= d <= MAX_RANGE_M:
                pts.append((a, d, q))
        if len(pts) < 10:
            continue
        source_count = payload.get("source_point_count")
        if type(source_count) is int and source_count != len(raw):
            raise MapBuildError(
                f"raw LiDAR source_count mismatch at revision {revision}: "
                f"{source_count} != {len(raw)}"
            )
        channel = getattr(reader, "channels_by_topic", {}).get(RAW_TOPIC)
        scans.append(RawScan(int(revision), int(start_ns), int(t_ns), int(end_ns), np.asarray(pts, dtype=float),
                             msg.sequence, channel.metadata.get("clock_epoch") if channel else None))
    scans.sort(key=lambda s: (s.t_ns, s.revision))
    if not scans:
        raise MapBuildError("capture contains no usable /r2b4/raw_lidar scans")
    return scans


def _interp_pose(samples: Sequence[PoseSample], times: Sequence[int], t_ns: int) -> PoseSample | None:
    pos = bisect.bisect_left(times, t_ns)
    if pos < len(samples) and samples[pos].t_ns == t_ns:
        return samples[pos]
    if pos <= 0 or pos >= len(samples):
        return None
    a, b = samples[pos - 1], samples[pos]
    if (a.generation != b.generation or a.discontinuity or b.discontinuity
            or a.frame_id != b.frame_id or a.clock_epoch != b.clock_epoch):
        return None
    dt = b.t_ns - a.t_ns
    if dt <= 0 or dt > 100_000_000:  # c50 should be much tighter; reject large gaps.
        return None
    f = (t_ns - a.t_ns) / dt
    dyaw = _angle(b.yaw - a.yaw)
    return PoseSample(
        t_ns=t_ns,
        x=a.x + f * (b.x - a.x),
        y=a.y + f * (b.y - a.y),
        yaw=_angle(a.yaw + f * dyaw),
        v=a.v + f * (b.v - a.v),
        omega=a.omega + f * (b.omega - a.omega),
        generation=a.generation,
        continuous=a.continuous and b.continuous,
        discontinuity=False,
        slip=a.slip or b.slip,
        local_sigma=max(a.local_sigma, b.local_sigma),
        yaw_sigma=max(a.yaw_sigma, b.yaw_sigma),
        observability=min(a.observability, b.observability),
        local_translation=a.local_translation if a.local_translation == b.local_translation else "DEGRADED",
        heading=a.heading if a.heading == b.heading else "DEGRADED",
        source_ticks=a.source_ticks + b.source_ticks,
        frame_id=a.frame_id,
        clock_epoch=a.clock_epoch,
        generation_recorded=a.generation_recorded and b.generation_recorded,
    )


def _stationary_ok(p: PoseSample) -> bool:
    return bool(
        abs(p.v) <= STATIONARY_MAX_V_MPS
        and abs(p.omega) <= STATIONARY_MAX_OMEGA_RAD_S
        and p.continuous
        and not p.discontinuity
        and not p.slip
        and p.local_translation != "LOST"
        and p.heading != "LOST"
    )


def _stationary_windows(samples: Sequence[PoseSample]) -> list[StationaryWindow]:
    windows: list[StationaryWindow] = []
    start: int | None = None
    last: PoseSample | None = None
    generation = -1
    count = 0

    def close(end_ns: int) -> None:
        nonlocal start, count, generation
        if start is None:
            return
        duration = (end_ns - start) / 1e9
        usable_start = start + int(STATIONARY_SETTLE_S * 1e9)
        usable_end = end_ns - int(STATIONARY_END_GUARD_S * 1e9)
        if duration >= STATIONARY_MIN_DURATION_S and usable_end > usable_start:
            windows.append(StationaryWindow(start, end_ns, usable_start, usable_end, generation, count))
        start = None
        count = 0
        generation = -1

    for p in samples:
        contiguous = last is None or p.t_ns - last.t_ns <= 60_000_000
        pose_stable = True
        if last is not None and contiguous:
            dt_s = max((p.t_ns - last.t_ns) / 1e9, 1e-9)
            pose_speed = math.hypot(p.x - last.x, p.y - last.y) / dt_s
            pose_yaw_rate = abs(_angle(p.yaw - last.yaw)) / dt_s
            pose_stable = pose_speed <= 0.05 and pose_yaw_rate <= 0.08
        ok = _stationary_ok(p) and pose_stable
        if ok and (start is None or (contiguous and p.generation == generation)):
            if start is None:
                start = p.t_ns
                generation = p.generation
                count = 1
            else:
                count += 1
        else:
            if start is not None and last is not None:
                close(last.t_ns)
            if ok:
                start = p.t_ns
                generation = p.generation
                count = 1
        last = p
    if start is not None and last is not None:
        close(last.t_ns)
    return windows


def _scan_xy(scan: RawScan) -> np.ndarray:
    a = np.deg2rad(scan.points[:, 0])
    d = scan.points[:, 1]
    # Source-first R2B4 convention from v3.scan_matching: +y robot-left,
    # raw LiDAR positive angle is mirrored laterally.
    return np.column_stack((d * np.cos(a), -d * np.sin(a)))


def _fuse_window(
    index: int,
    window: StationaryWindow,
    scans: Sequence[RawScan],
    poses: Sequence[PoseSample],
    pose_times: Sequence[int],
) -> Keyframe | None:
    inside = [s for s in scans if window.usable_start_ns <= s.t_ns <= window.usable_end_ns]
    if len(inside) < MIN_SCANS_PER_KEYFRAME:
        return None
    if len(inside) > MAX_SCANS_PER_KEYFRAME:
        pick = np.linspace(0, len(inside) - 1, MAX_SCANS_PER_KEYFRAME, dtype=int)
        inside = [inside[int(i)] for i in pick]
    ref_scan = inside[len(inside) // 2]
    ref = _interp_pose(poses, pose_times, ref_scan.t_ns)
    if ref is None or ref.generation != window.generation:
        return None
    ref_pose = (ref.x, ref.y, ref.yaw)

    # Per-voxel persistent support across independent revolutions. This removes
    # most people/transient returns while preserving walls/furniture repeatedly
    # observed during the stationary dwell.
    accum: dict[tuple[int, int], list[float]] = {}
    usable_scans = 0
    sigmas: list[float] = []
    yaw_sigmas: list[float] = []
    observability: list[float] = []
    source_scans: list[dict[str, object]] = []
    for scan_idx, scan in enumerate(inside):
        p = _interp_pose(poses, pose_times, scan.t_ns)
        if p is None or p.generation != window.generation or p.discontinuity:
            continue
        xy = _scan_xy(scan)
        if len(xy) < 10:
            continue
        rel = _pose_relative(ref_pose, (p.x, p.y, p.yaw))
        xy = _transform(xy, rel)
        keys = np.floor(xy / FUSION_VOXEL_M).astype(np.int32)
        per_scan: dict[tuple[int, int], list[float]] = {}
        for key_arr, pt in zip(keys, xy):
            key = (int(key_arr[0]), int(key_arr[1]))
            row = per_scan.get(key)
            if row is None:
                per_scan[key] = [float(pt[0]), float(pt[1]), 1.0]
            else:
                row[0] += float(pt[0]); row[1] += float(pt[1]); row[2] += 1.0
        for key, row in per_scan.items():
            mx, my = row[0] / row[2], row[1] / row[2]
            dst = accum.get(key)
            if dst is None:
                accum[key] = [mx, my, 1.0, 1.0]
            else:
                dst[0] += mx; dst[1] += my; dst[2] += 1.0; dst[3] += 1.0
        usable_scans += 1
        source_scans.append({"revision": scan.revision, "sequence": scan.sequence,
                             "measurement_time_ns": scan.t_ns, "start_time_ns": scan.start_ns,
                             "end_time_ns": scan.end_ns, "clock_epoch": scan.clock_epoch,
                             "pose_reference_ticks": list(p.source_ticks)})
        sigmas.append(p.local_sigma)
        yaw_sigmas.append(p.yaw_sigma)
        observability.append(p.observability)

    if usable_scans < MIN_SCANS_PER_KEYFRAME:
        return None
    min_support = max(2, int(math.ceil(usable_scans * FUSION_MIN_SCAN_FRACTION)))
    cloud = []
    for row in accum.values():
        scan_support = int(row[3])
        if scan_support >= min_support:
            cloud.append((row[0] / row[2], row[1] / row[2]))
    if len(cloud) < 24:
        return None
    cloud_arr = np.asarray(cloud, dtype=float)
    # Deterministic angular ordering makes later deterministic subsampling less biased.
    order = np.argsort(np.arctan2(cloud_arr[:, 1], cloud_arr[:, 0]))
    cloud_arr = _even_subsample(cloud_arr[order], MAX_KEYFRAME_POINTS)
    return Keyframe(
        index=index,
        ref_ns=ref_scan.t_ns,
        window_start_ns=window.start_ns,
        window_end_ns=window.end_ns,
        generation=window.generation,
        seed_pose=ref_pose,
        cloud_local=cloud_arr,
        raw_scan_count=len([s for s in scans if window.start_ns <= s.t_ns <= window.end_ns]),
        used_scan_count=usable_scans,
        retained_voxel_count=len(cloud_arr),
        min_voxel_support=min_support,
        local_sigma_m=float(np.median(sigmas)) if sigmas else 0.10,
        yaw_sigma_rad=float(np.median(yaw_sigmas)) if yaw_sigmas else 0.10,
        observability=float(np.median(observability)) if observability else 0.0,
        source_ticks=ref.source_ticks,
        source_scans=tuple(source_scans),
        source_frame_id=ref.frame_id,
        source_clock_epoch=ref.clock_epoch,
        generation_recorded=ref.generation_recorded,
    )


def _target_normals(target: np.ndarray) -> tuple[cKDTree, np.ndarray, np.ndarray]:
    tree = cKDTree(target)
    k = min(ICP_K_NEIGHBOURS, len(target))
    if k < 3:
        return tree, np.zeros_like(target), np.zeros(len(target), dtype=bool)
    _d, idx = tree.query(target, k=k, workers=1)
    neighbourhood = target[idx]
    centred = neighbourhood - neighbourhood.mean(axis=1, keepdims=True)
    cov = np.einsum("nki,nkj->nij", centred, centred)
    values, vectors = np.linalg.eigh(cov)
    normals = vectors[:, :, 0]
    surfaces = values[:, 0] < 0.20 * np.maximum(values[:, 1], 1e-12)
    return tree, normals, surfaces


def _icp(
    target_cloud: np.ndarray,
    source_cloud: np.ndarray,
    init: Sequence[float],
    *,
    max_corr: float = ICP_MAX_CORRESPONDENCE_M,
) -> tuple[tuple[float, float, float], dict[str, float]] | None:
    target = _even_subsample(np.asarray(target_cloud, dtype=float), ICP_MAX_POINTS)
    source = _even_subsample(np.asarray(source_cloud, dtype=float), ICP_MAX_POINTS)
    if len(target) < 24 or len(source) < 24:
        return None
    tree, normals, surfaces = _target_normals(target)
    if int(np.count_nonzero(surfaces)) < 12:
        return None

    pose = (float(init[0]), float(init[1]), _angle(float(init[2])))
    transformed = _transform(source, pose)
    for _ in range(ICP_MAX_ITERS):
        distances, indices = tree.query(transformed, k=1, workers=1)
        selected = (distances <= max_corr) & surfaces[indices]
        if int(np.count_nonzero(selected)) < 18:
            return None
        a = transformed[selected]
        q = target[indices[selected]]
        n = normals[indices[selected]]
        residual = np.einsum("ni,ni->n", n, a - q)
        med = float(np.median(residual))
        noise = max(0.003, 1.4826 * float(np.median(np.abs(residual - med))))
        huber = max(0.008, 1.5 * noise)
        weights = np.sqrt(np.minimum(1.0, huber / np.maximum(np.abs(residual), 1e-12)))
        jac = np.column_stack((n[:, 0], n[:, 1], -a[:, 1] * n[:, 0] + a[:, 0] * n[:, 1]))
        step, _resid, rank, _sv = np.linalg.lstsq(jac * weights[:, None], -residual * weights, rcond=None)
        if rank < 3 or not np.all(np.isfinite(step)):
            return None
        trans_norm = float(np.linalg.norm(step[:2]))
        if trans_norm > 0.12:
            step[:2] *= 0.12 / trans_norm
        step[2] = float(np.clip(step[2], -0.12, 0.12))
        delta = (float(step[0]), float(step[1]), float(step[2]))
        transformed = _transform(transformed, delta)
        pose = _pose_compose(delta, pose)
        if trans_norm < 0.0005 and abs(step[2]) < 0.0005:
            break

    distances, indices = tree.query(transformed, k=1, workers=1)
    selected = (distances <= max_corr) & surfaces[indices]
    count = int(np.count_nonzero(selected))
    if count < 18:
        return None
    a = transformed[selected]
    q = target[indices[selected]]
    n = normals[indices[selected]]
    normal_resid = np.einsum("ni,ni->n", n, a - q)
    rmse = float(np.sqrt(np.mean(normal_resid * normal_resid)))
    p2p_rmse = float(np.sqrt(np.mean(distances[selected] ** 2)))
    inlier_ratio = float(count / max(1, len(source)))
    matched_normals = n
    eigenvalues = np.linalg.eigvalsh(matched_normals.T @ matched_normals / count)
    observability = float(min(1.0, max(0.0, 2.0 * float(eigenvalues[0]))))
    radius = max(float(np.sqrt(np.mean(np.sum(a * a, axis=1)))), 1e-6)
    angular = (-a[:, 1] * matched_normals[:, 0] + a[:, 0] * matched_normals[:, 1]) / radius
    info = np.column_stack((matched_normals, angular))
    full_rank = np.linalg.eigvalsh(info.T @ info / count)
    observability = min(observability, float(max(0.0, min(1.0, 3.0 * full_rank[0]))))
    metrics = {
        "rmse_m": rmse,
        "p2p_rmse_m": p2p_rmse,
        "inlier_ratio": inlier_ratio,
        "observability": observability,
        "inlier_count": float(count),
    }
    return pose, metrics


def _scan_edge(i: int, j: int, keyframes: Sequence[Keyframe], *, loop: bool) -> Edge | None:
    a, b = keyframes[i], keyframes[j]
    seed = _pose_relative(a.seed_pose, b.seed_pose)
    forward = _icp(a.cloud_local, b.cloud_local, seed)
    if forward is None:
        return None
    z, metrics = forward
    if loop:
        correction = _pose_diff(seed, z)
        if math.hypot(correction[0], correction[1]) > LOOP_MAX_SEED_CORRECTION_M or abs(correction[2]) > LOOP_MAX_SEED_CORRECTION_RAD:
            return None
        if not (
            metrics["rmse_m"] <= LOOP_MAX_RMSE_M
            and metrics["inlier_ratio"] >= LOOP_MIN_INLIER_RATIO
            and metrics["observability"] >= LOOP_MIN_OBSERVABILITY
        ):
            return None
        reverse = _icp(b.cloud_local, a.cloud_local, _pose_inverse(seed))
        if reverse is None:
            return None
        z_rev, rev_metrics = reverse
        reciprocal = _pose_diff(z, _pose_inverse(z_rev))
        if math.hypot(reciprocal[0], reciprocal[1]) > LOOP_RECIPROCAL_MAX_M or abs(reciprocal[2]) > LOOP_RECIPROCAL_MAX_RAD:
            return None
        metrics = {
            **metrics,
            "reverse_rmse_m": rev_metrics["rmse_m"],
            "reverse_inlier_ratio": rev_metrics["inlier_ratio"],
            "reciprocal_translation_m": math.hypot(reciprocal[0], reciprocal[1]),
            "reciprocal_yaw_rad": abs(reciprocal[2]),
        }
        sigma_t = max(0.018, min(0.06, metrics["rmse_m"] * 1.8))
        sigma_y = max(0.022, min(0.08, sigma_t / 1.5))
        return Edge(i, j, z, sigma_t, sigma_y, "LOOP", metrics)
    if not (
        metrics["rmse_m"] <= SEQUENTIAL_MAX_RMSE_M
        and metrics["inlier_ratio"] >= SEQUENTIAL_MIN_INLIER_RATIO
        and metrics["observability"] >= SEQUENTIAL_MIN_OBSERVABILITY
    ):
        return None
    sigma_t = max(0.022, min(0.08, metrics["rmse_m"] * 2.0))
    sigma_y = max(0.028, min(0.10, sigma_t / 1.3))
    return Edge(i, j, z, sigma_t, sigma_y, "SCAN", metrics)


def _build_edges(keyframes: Sequence[Keyframe]) -> list[Edge]:
    edges: list[Edge] = []
    # Always preserve the L3 local trajectory as a weak odometry backbone when
    # the localization generation is continuous. Add independent scan edges on top.
    for i in range(len(keyframes) - 1):
        a, b = keyframes[i], keyframes[i + 1]
        if a.generation == b.generation:
            z = _pose_relative(a.seed_pose, b.seed_pose)
            sigma_t = max(0.045, min(0.25, a.local_sigma_m + b.local_sigma_m + 0.025))
            sigma_y = max(0.05, min(0.30, a.yaw_sigma_rad + b.yaw_sigma_rad + 0.025))
            edges.append(Edge(i, i + 1, z, sigma_t, sigma_y, "ODOM", {}))
        scan_edge = _scan_edge(i, i + 1, keyframes, loop=False)
        if scan_edge is not None:
            edges.append(scan_edge)

    # Conservative loop candidates: nearby under the current L3 seed but not
    # immediate neighbours. Verify each candidate with strict reciprocal ICP.
    used: set[tuple[int, int]] = set()
    for j in range(len(keyframes)):
        candidates: list[tuple[float, int]] = []
        for i in range(0, j - LOOP_MIN_INDEX_SEPARATION + 1):
            if j - i < LOOP_MIN_INDEX_SEPARATION:
                continue
            dx = keyframes[j].seed_pose[0] - keyframes[i].seed_pose[0]
            dy = keyframes[j].seed_pose[1] - keyframes[i].seed_pose[1]
            d = math.hypot(dx, dy)
            if d <= LOOP_SEARCH_RADIUS_M:
                candidates.append((d, i))
        candidates.sort()
        accepted = 0
        for _d, i in candidates:
            if accepted >= LOOP_MAX_CANDIDATES_PER_NODE:
                break
            pair = (i, j)
            if pair in used:
                continue
            used.add(pair)
            edge = _scan_edge(i, j, keyframes, loop=True)
            if edge is not None:
                edges.append(edge)
                accepted += 1
    return edges


def _normalize_seed_poses(keyframes: Sequence[Keyframe]) -> list[tuple[float, float, float]]:
    base = keyframes[0].seed_pose
    return [_pose_relative(base, k.seed_pose) for k in keyframes]


def _edge_error(edge: Edge, poses: Sequence[Sequence[float]]) -> tuple[float, float, float]:
    pred = _pose_relative(poses[edge.i], poses[edge.j])
    e = _pose_compose(_pose_inverse(edge.z), pred)
    return (e[0], e[1], _angle(e[2]))


def _optimize_once(seed: Sequence[Sequence[float]], edges: Sequence[Edge]):
    n = len(seed)
    if n == 1:
        return [tuple(seed[0])], None
    x0 = np.asarray([v for p in seed[1:] for v in p], dtype=float)

    def unpack(x: np.ndarray) -> list[tuple[float, float, float]]:
        out = [tuple(seed[0])]
        for k in range(n - 1):
            out.append((float(x[3 * k]), float(x[3 * k + 1]), _angle(float(x[3 * k + 2]))))
        return out

    def residual(x: np.ndarray) -> np.ndarray:
        poses = unpack(x)
        rows: list[float] = []
        for edge in edges:
            ex, ey, et = _edge_error(edge, poses)
            rows.extend((ex / edge.sigma_t, ey / edge.sigma_t, et / edge.sigma_yaw))
        return np.asarray(rows, dtype=float)

    sparsity = lil_matrix((3 * len(edges), 3 * (n - 1)), dtype=int)
    for row, edge in enumerate(edges):
        for node in (edge.i, edge.j):
            if node == 0:
                continue
            c = 3 * (node - 1)
            sparsity[3 * row : 3 * row + 3, c : c + 3] = 1
    result = least_squares(
        residual,
        x0,
        jac_sparsity=sparsity.tocsr(),
        loss="huber",
        f_scale=1.0,
        max_nfev=500,
        xtol=1e-8,
        ftol=1e-8,
        gtol=1e-8,
    )
    return unpack(result.x), result


def _optimize_graph(keyframes: Sequence[Keyframe], edges: list[Edge]):
    seed = _normalize_seed_poses(keyframes)
    if not edges:
        return seed, edges, {"status": "NO_EDGES"}
    poses, result = _optimize_once(seed, edges)

    # Remove only geometrically impossible loop closures after the first robust
    # solve, then solve once more. ODOM/SCAN continuity remains robust-loss gated.
    kept: list[Edge] = []
    rejected_loops = 0
    for edge in edges:
        if edge.kind != "LOOP":
            kept.append(edge)
            continue
        ex, ey, et = _edge_error(edge, poses)
        norm = math.sqrt((ex / edge.sigma_t) ** 2 + (ey / edge.sigma_t) ** 2 + (et / edge.sigma_yaw) ** 2)
        if norm <= 6.0:
            kept.append(edge)
        else:
            rejected_loops += 1
    if rejected_loops:
        poses, result = _optimize_once(seed, kept)
    info = {
        "status": "OK" if result is not None and result.success else "OPTIMIZER_WARNING",
        "success": bool(result.success) if result is not None else True,
        "message": str(result.message) if result is not None else "single node",
        "cost": float(result.cost) if result is not None else 0.0,
        "optimality": float(result.optimality) if result is not None else 0.0,
        "nfev": int(result.nfev) if result is not None else 0,
        "rejected_loop_edges": rejected_loops,
    }
    return poses, kept, info


def _graph_connected(n: int, edges: Sequence[Edge]) -> bool:
    if n <= 1:
        return True
    adj = [[] for _ in range(n)]
    for e in edges:
        adj[e.i].append(e.j); adj[e.j].append(e.i)
    seen = {0}
    stack = [0]
    while stack:
        i = stack.pop()
        for j in adj[i]:
            if j not in seen:
                seen.add(j); stack.append(j)
    return len(seen) == n


def _bresenham(x0: int, y0: int, x1: int, y1: int):
    dx = abs(x1 - x0); sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0); sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    while True:
        yield x, y
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy; x += sx
        if e2 <= dx:
            err += dx; y += sy


def _render_occupancy(keyframes: Sequence[Keyframe], poses: Sequence[Sequence[float]], resolution: float):
    world_clouds: list[np.ndarray] = []
    origins: list[tuple[float, float]] = []
    all_xy: list[np.ndarray] = []
    for kf, pose in zip(keyframes, poses):
        cloud = _transform(kf.cloud_local, pose)
        world_clouds.append(cloud)
        origins.append((float(pose[0]), float(pose[1])))
        all_xy.append(cloud)
    merged = np.vstack(all_xy)
    min_x = min(float(np.min(merged[:, 0])), min(x for x, _ in origins)) - MAP_MARGIN_M
    max_x = max(float(np.max(merged[:, 0])), max(x for x, _ in origins)) + MAP_MARGIN_M
    min_y = min(float(np.min(merged[:, 1])), min(y for _, y in origins)) - MAP_MARGIN_M
    max_y = max(float(np.max(merged[:, 1])), max(y for _, y in origins)) + MAP_MARGIN_M
    width = int(math.ceil((max_x - min_x) / resolution)) + 1
    height = int(math.ceil((max_y - min_y) / resolution)) + 1
    if width * height > 20_000_000:
        raise MapBuildError(f"map grid unexpectedly large: {width}x{height}")
    logodds = np.zeros((height, width), dtype=np.float32)
    observed = np.zeros((height, width), dtype=np.uint8)

    def cell(x: float, y: float) -> tuple[int, int]:
        return int(math.floor((x - min_x) / resolution)), int(math.floor((y - min_y) / resolution))

    for origin, cloud in zip(origins, world_clouds):
        ox, oy = cell(origin[0], origin[1])
        rays = _even_subsample(cloud, 2200)
        for pt in rays:
            ex, ey = cell(float(pt[0]), float(pt[1]))
            cells = list(_bresenham(ox, oy, ex, ey))
            if not cells:
                continue
            for cx, cy in cells[:-1]:
                if 0 <= cx < width and 0 <= cy < height:
                    logodds[cy, cx] = max(LOG_ODDS_MIN, float(logodds[cy, cx]) + LOG_ODDS_FREE)
                    observed[cy, cx] = 1
            cx, cy = cells[-1]
            if 0 <= cx < width and 0 <= cy < height:
                logodds[cy, cx] = min(LOG_ODDS_MAX, float(logodds[cy, cx]) + LOG_ODDS_OCC)
                observed[cy, cx] = 1

    prob = 1.0 / (1.0 + np.exp(-logodds.astype(np.float64)))
    prob[observed == 0] = np.nan
    image = np.full((height, width), 205, dtype=np.uint8)  # ROS unknown
    image[(observed == 1) & (prob < FREE_PROB)] = 254
    image[(observed == 1) & (prob > OCCUPIED_PROB)] = 0
    uncertain = (observed == 1) & ~(prob < FREE_PROB) & ~(prob > OCCUPIED_PROB)
    image[uncertain] = 205
    return {
        "logodds": logodds,
        "probability": prob,
        "observed": observed,
        "image": image,
        "origin": (min_x, min_y, 0.0),
        "resolution": resolution,
        "width": width,
        "height": height,
    }


def _write_pgm(path: Path, image: np.ndarray) -> None:
    # PGM is top-down; occupancy grid y grows upward, hence flipud.
    img = np.flipud(np.asarray(image, dtype=np.uint8))
    header = f"P5\n{img.shape[1]} {img.shape[0]}\n255\n".encode("ascii")
    with path.open("wb") as f:
        f.write(header)
        f.write(img.tobytes(order="C"))


def _write_yaml(path: Path, pgm_name: str, resolution: float, origin: Sequence[float]) -> None:
    text = (
        f"image: {pgm_name}\n"
        f"resolution: {resolution:.6f}\n"
        f"origin: [{origin[0]:.6f}, {origin[1]:.6f}, {origin[2]:.6f}]\n"
        "negate: 0\n"
        f"occupied_thresh: {OCCUPIED_PROB:.3f}\n"
        f"free_thresh: {FREE_PROB:.3f}\n"
    )
    path.write_text(text, encoding="utf-8")


def _jsonable_edge(edge: Edge) -> dict[str, object]:
    return {
        "i": edge.i, "j": edge.j, "kind": edge.kind,
        "measurement": [float(v) for v in edge.z],
        "sigma_t_m": edge.sigma_t, "sigma_yaw_rad": edge.sigma_yaw,
        "metrics": {k: float(v) for k, v in edge.metrics.items()},
    }


def _write_outputs(
    outdir: Path,
    input_path: Path,
    keyframes: Sequence[Keyframe],
    poses: Sequence[Sequence[float]],
    edges: Sequence[Edge],
    occupancy: Mapping[str, object],
    report: dict[str, object],
) -> None:
    outdir.mkdir(parents=True, exist_ok=False)
    pgm = outdir / "map.pgm"
    yaml = outdir / "map.yaml"
    _write_pgm(pgm, np.asarray(occupancy["image"]))
    _write_yaml(yaml, pgm.name, float(occupancy["resolution"]), occupancy["origin"])
    np.savez_compressed(
        outdir / "map_data.npz",
        probability=np.asarray(occupancy["probability"]),
        logodds=np.asarray(occupancy["logodds"]),
        observed=np.asarray(occupancy["observed"]),
        optimized_poses=np.asarray(poses, dtype=float),
        seed_poses=np.asarray([_normalize_seed_poses(keyframes)[i] for i in range(len(keyframes))], dtype=float),
        original_seed_poses=np.asarray([kf.seed_pose for kf in keyframes], dtype=float),
        origin=np.asarray(occupancy["origin"], dtype=float),
        resolution=np.asarray([occupancy["resolution"]], dtype=float),
    )
    with (outdir / "keyframes.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["index", "ref_ns", "generation", "seed_x", "seed_y", "seed_yaw",
                    "opt_x", "opt_y", "opt_yaw", "used_scans", "points", "local_sigma_m",
                    "yaw_sigma_rad", "observability"])
        normalized = _normalize_seed_poses(keyframes)
        for kf, seed, opt in zip(keyframes, normalized, poses):
            w.writerow([kf.index, kf.ref_ns, kf.generation, *seed, *opt, kf.used_scan_count,
                        len(kf.cloud_local), kf.local_sigma_m, kf.yaw_sigma_rad, kf.observability])
    (outdir / "pose_graph.json").write_text(
        json.dumps({"edges": [_jsonable_edge(e) for e in edges]}, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    normalized = _normalize_seed_poses(keyframes)
    reference = {
        "schema": "R2B4_ATLAS_REFERENCE_V1",
        "revision": 1,
        "input": report["input"],
        "build": report["build_identity"],
        "source_gauge": {
            "kind": "FIRST_KEYFRAME_LOCAL_POSE",
            "anchor_keyframe_index": keyframes[0].index,
            "original_pose": list(keyframes[0].seed_pose),
            "frame_id": keyframes[0].source_frame_id,
            "clock_epoch": keyframes[0].source_clock_epoch,
            "generation": keyframes[0].generation if keyframes[0].generation_recorded else None,
            "measurement_time_ns": keyframes[0].ref_ns,
            "pose_reference_ticks": list(keyframes[0].source_ticks),
            "trajectory_relationship": "PER_KEYFRAME_OPTIMIZED_NONRIGID",
        },
        "assets": [{"name": name, "sha256": _sha256(outdir / name),
                    "bytes": (outdir / name).stat().st_size}
                   for name in ("map.pgm", "map.yaml", "map_data.npz", "keyframes.csv", "pose_graph.json")],
        "keyframes": [{"index": kf.index, "measurement_time_ns": kf.ref_ns,
                       "original_pose": list(kf.seed_pose), "normalized_seed_pose": list(seed),
                       "optimized_pose": list(opt), "source_frame_id": kf.source_frame_id,
                       "source_clock_epoch": kf.source_clock_epoch,
                       "generation": kf.generation if kf.generation_recorded else None,
                       "source_ticks": list(kf.source_ticks), "source_scans": list(kf.source_scans),
                       "window_start_ns": kf.window_start_ns, "window_end_ns": kf.window_end_ns}
                      for kf, seed, opt in zip(keyframes, normalized, poses)],
        "motion_authority": False,
        "current_alignment": None,
    }
    digest = _json_sha256(reference)
    reference.update(map_id="atlas:" + digest, frame_id="R2B4_ATLAS:" + digest)
    (outdir / "atlas_reference.json").write_text(json.dumps(reference, indent=2, sort_keys=True, allow_nan=False),
                                                encoding="utf-8")
    report["atlas_reference"] = {"name": "atlas_reference.json", "map_id": reference["map_id"],
                                 "revision": 1, "frame_id": reference["frame_id"],
                                 "motion_authority": False, "current_alignment": None}
    (outdir / "report.json").write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")


def build(input_path: Path, output_dir: Path | None = None) -> Path:
    input_path = input_path.expanduser().resolve()
    if input_path.is_symlink() or not input_path.is_file():
        raise MapBuildError(f"input is not a regular MCAP file: {input_path}")
    root, reader, McapReadError = _load_reader(input_path)

    structure = reader.inspect(verify_chunks=True)
    if not structure.valid:
        raise MapBuildError("MCAP container/integrity structure invalid: " + "; ".join(structure.errors))
    meta = reader.latest_metadata("r2b4.capture") or {}
    try:
        hz = int(meta.get("tick_sample_hz", "0"))
    except ValueError as exc:
        raise MapBuildError("invalid capture tick_sample_hz metadata") from exc
    if hz != EXPECTED_CAPTURE_HZ:
        raise MapBuildError(f"requires a 50 Hz capture, got {hz} Hz")
    if meta.get("raw_evidence_requested", "false") != "true":
        raise MapBuildError("capture was not recorded with raw sensor evidence")
    try:
        final = reader.capture_integrity(require_raw_evidence=True)
    except McapReadError as exc:
        raise MapBuildError(f"capture integrity gate failed: {exc}") from exc
    integrity = _as_mapping(final.get("integrity"))

    # Stronger than the recorder's generic sparse-loss tolerance: map building
    # defaults to zero loss, zero truncation, zero producer supersede.
    strict_zero_fields = {
        "raw_lidar_missing_count": integrity.get("raw_lidar_missing_count", 0),
        "raw_lidar_truncated_count": integrity.get("raw_lidar_truncated_count", 0),
        "raw_transport_superseded_count": integrity.get("raw_transport_superseded_count", 0),
        "raw_capacity_eviction_count": integrity.get("raw_capacity_eviction_count", 0),
        "ingress_drop_count": integrity.get("ingress_drop_count", 0),
    }
    bad = {k: v for k, v in strict_zero_fields.items() if isinstance(v, int) and v != 0}
    if bad:
        raise MapBuildError("strict mapping evidence gate failed: " + json.dumps(bad, sort_keys=True))

    poses = _extract_pose_samples(reader)
    scans = _extract_raw_scans(reader)
    pose_times = [p.t_ns for p in poses]
    windows = _stationary_windows(poses)
    keyframes: list[Keyframe] = []
    for window in windows:
        kf = _fuse_window(len(keyframes), window, scans, poses, pose_times)
        if kf is not None:
            keyframes.append(kf)
    if len(keyframes) < 2:
        raise MapBuildError(
            f"only {len(keyframes)} usable stationary keyframe(s) found. "
            "Use the intended survey motion: move 0.5-1 m, stop about 3 s, repeat."
        )

    edges = _build_edges(keyframes)
    if not _graph_connected(len(keyframes), edges):
        raise MapBuildError(
            "pose graph is disconnected. This usually means a localization generation break "
            "without a geometrically verified scan connection."
        )
    optimized, edges, opt_info = _optimize_graph(keyframes, edges)
    occupancy = _render_occupancy(keyframes, optimized, MAP_RESOLUTION_M)

    if output_dir is None:
        output_dir = input_path.with_name(input_path.stem + "_global_map")
    else:
        output_dir = output_dir.expanduser().resolve()
    if output_dir.exists():
        raise MapBuildError(f"output already exists: {output_dir}")

    kinds: dict[str, int] = {}
    for e in edges:
        kinds[e.kind] = kinds.get(e.kind, 0) + 1
    total_stationary_s = sum((w.end_ns - w.start_ns) for w in windows) / 1e9
    observed = np.asarray(occupancy["observed"])
    probability = np.asarray(occupancy["probability"])
    observed_cells = int(np.count_nonzero(observed))
    occupied_cells = int(np.count_nonzero((observed == 1) & (probability > OCCUPIED_PROB)))
    free_cells = int(np.count_nonzero((observed == 1) & (probability < FREE_PROB)))
    report: dict[str, object] = {
        "schema": "R2B4_GLOBAL_MAP_BUILD_V2",
        "tool": "mcap50-to-map.py",
        "source_first_repo_main": R2B4_SOURCE_MAIN,
        "repo_root": str(root),
        "build_identity": _build_identity(reader),
        "input": {
            "path": str(input_path),
            "sha256": _sha256(input_path),
            "bytes": input_path.stat().st_size,
            "capture_hz": hz,
            "capture_id": meta.get("capture_id"),
            "clock_epoch": meta.get("clock_epoch"),
            "captured_tick_count": final.get("captured_tick_count"),
            "captured_raw_lidar_count": final.get("captured_raw_lidar_count"),
            "raw_evidence_complete": integrity.get("raw_evidence_complete"),
            "replay_complete": integrity.get("replay_complete"),
        },
        "evidence_gate": {"strict_zero_loss": True, **strict_zero_fields},
        "trajectory": {"l3_pose_samples": len(poses)},
        "raw_lidar": {"usable_scans": len(scans)},
        "stationary": {
            "detected_windows": len(windows),
            "usable_keyframes": len(keyframes),
            "total_stationary_s": total_stationary_s,
            "max_v_mps": STATIONARY_MAX_V_MPS,
            "max_omega_rad_s": STATIONARY_MAX_OMEGA_RAD_S,
            "settle_s": STATIONARY_SETTLE_S,
        },
        "pose_graph": {
            "edge_counts": kinds,
            "edge_count": len(edges),
            "connected": True,
            "optimization": opt_info,
        },
        "map": {
            "resolution_m": occupancy["resolution"],
            "origin": list(occupancy["origin"]),
            "width_cells": occupancy["width"],
            "height_cells": occupancy["height"],
            "observed_cells": observed_cells,
            "free_cells": free_cells,
            "occupied_cells": occupied_cells,
            "observed_area_m2": observed_cells * MAP_RESOLUTION_M * MAP_RESOLUTION_M,
            "free_area_m2": free_cells * MAP_RESOLUTION_M * MAP_RESOLUTION_M,
        },
        "limitations": [
            "raw LiDAR points do not yet carry per-point acquisition timestamps; this builder therefore uses stationary evidence for final occupancy",
            "persistent cross-session relocalization/topology/semantics are outside this tool",
            "LiDAR-to-base extrinsics are assumed identical to the current R2B4 production scan-matching convention",
            "the first source keyframe fixes the seed gauge; graph optimization is not a single rigid transform of the source odometry trajectory",
            "missing source runtime/session/calibration identity remains unknown; an imported atlas never establishes current alignment",
        ],
    }
    _write_outputs(output_dir, input_path, keyframes, optimized, edges, occupancy, report)
    return output_dir


def _parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Build an R2B4 global occupancy map from a complete 50 Hz MCAP capture."
    )
    p.add_argument("mcap", type=Path, help="50 Hz R2B4 MCAP capture")
    p.add_argument("--output", type=Path, default=None, help="output directory (default: <capture>_global_map)")
    return p


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        output = build(args.mcap, args.output)
    except (MapBuildError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    report = json.loads((output / "report.json").read_text(encoding="utf-8"))
    print(f"global map: {output / 'map.pgm'}")
    print(f"map yaml:   {output / 'map.yaml'}")
    print(f"map data:   {output / 'map_data.npz'}")
    print(f"report:     {output / 'report.json'}")
    print(
        "summary: "
        f"{report['stationary']['usable_keyframes']} keyframes, "
        f"{report['pose_graph']['edge_count']} graph edges, "
        f"{report['map']['observed_area_m2']:.2f} m^2 observed"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
