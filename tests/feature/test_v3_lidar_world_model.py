from __future__ import annotations

import pickle
from dataclasses import replace

import pytest

from rig import resolved_config, healthy_localization
from v3.contracts import (
    AdmittedFrame, CostmapCell, DataField, Observation, RobotEstimate,
    RollingLocalCostmap, TickContext, TrackEstimateStatus,
)
from v3.layers.l4_structural_memory import StructuralMemoryGrid
from v3.layers.l4_temporal_occupancy import TemporalOccupancyGrid
from v3.layers.l4_world_model import ShadowWorldModel


def _config():
    return resolved_config().runtime.composition.live_control.control.world_model


def _observation(kind, sequence, captured_ns, **values):
    return Observation(kind, "RPLIDAR_C1", sequence, captured_ns,
                       tuple(DataField(key, value) for key, value in values.items()))


def _scan(sequence, captured_ns, points=((1.05, 0.05),)):
    values = {"frame_id": "ROBOT_BASE", "point_count": len(points)}
    for index, (x, y) in enumerate(points):
        values.update({f"point_{index:03d}_x_m": x, f"point_{index:03d}_y_m": y,
                       f"point_{index:03d}_quality": 10})
    return (
        _observation("lidar_health", sequence, captured_ns, age_ns=0, point_count=len(points)),
        _observation("lidar_local_points", sequence, captured_ns, **values),
    )


def _tick(model, now_ns, observations=(), *, x=0.0, variance=0.0, frame_id="odom"):
    context = TickContext(now_ns, now_ns)
    covariance = tuple(variance if i in (0, 6, 12) else 0.0 for i in range(25))
    estimate = RobotEstimate(context, frame_id, x, 0.0, 0.0, 0.0, 0.0, covariance, localization_quality=healthy_localization())
    return model(AdmittedFrame(context, observations, ()), estimate)


def _confirmed_wall(model, config, point=(1.05, 0.05)):
    count = max(config.structural_confirm_min_hits, config.structural_confirm_score)
    step = max(20_000_000, (config.structural_confirm_min_span_ns + count - 2) // (count - 1))
    for sequence in range(1, count + 1):
        now = 1_000_000_000 + (sequence - 1) * step
        result = _tick(model, now, _scan(sequence, now, (point,)))
    return now, sequence, result


def _no_map_work(*args, **kwargs):
    pytest.fail("unchanged control tick traversed or rebuilt map geometry")


def test_lidar_multirate_cache_keeps_50hz_projection_freshness_and_lineage(monkeypatch):
    config = _config()
    model = ShadowWorldModel(config)
    previous = None
    start = 1_000_000_000
    for tick in range(50):
        now = start + tick * 20_000_000
        observations = _scan(tick // 10 + 1, now) if tick % 10 == 0 else ()
        if tick == 0:
            observations += (_observation(
                "obstacle_track", 1, now, track_id="cart", x_m=-1.0, y_m=1.0,
                radius_m=0.1, vx_mps=0.1, vy_mps=0.0, confidence=1.0,
            ),)
        if tick % 10:
            with monkeypatch.context() as patch:
                for cls, methods in (
                    (TemporalOccupancyGrid, ("integrate_scan", "prune", "occupied_cells", "window_margin_m")),
                    (StructuralMemoryGrid, ("integrate", "prune", "confirmed_cells", "window_margin_m")),
                    (RollingLocalCostmap, ("__post_init__",)),
                ):
                    for method in methods:
                        patch.setattr(cls, method, _no_map_work)
                result = _tick(model, now, observations, x=tick * 0.001)
            assert result.local_costmap.occupied_cells is previous.local_costmap.occupied_cells
            assert result.local_costmap.revision == previous.local_costmap.revision
        else:
            result = _tick(model, now, observations, x=tick * 0.001)
            if previous is not None:
                assert result.local_costmap.revision > previous.local_costmap.revision
        assert result.context.monotonic_ns == now
        assert result.freshness_ns == result.local_costmap.freshness_ns == tick % 10 * 20_000_000
        assert result.local_costmap.source_sequence == tick // 10 + 1
        age = now - start
        if age <= config.max_track_age_ns:
            track = result.obstacle_tracks[0]
            assert track.measurement_monotonic_ns == start
            assert track.x_m == pytest.approx(-1.0 + 0.1 * min(age, config.person_track_prediction_max_age_ns) / 1e9)
            assert track.usable_at(now) == (age <= config.person_track_prediction_max_age_ns)
            if age > config.person_track_prediction_max_age_ns:
                assert track.estimate_status is TrackEstimateStatus.DEGRADED
        else:
            assert result.obstacle_tracks == ()
        previous = result


def test_lidar_memory_quality_and_expiry_invalidate_cache_without_new_scans():
    config = _config()
    model = ShadowWorldModel(config)
    last_ns, sequence, initial = _confirmed_wall(model, config)
    temporal_expiry = last_ns + config.local_costmap_max_cell_age_ns + 1
    remembered = _tick(model, temporal_expiry)
    assert remembered.local_costmap.occupied_cells == initial.local_costmap.occupied_cells
    assert remembered.local_costmap.revision > initial.local_costmap.revision
    degraded = _tick(model, temporal_expiry + 20_000_000, variance=1.0)
    assert degraded.local_costmap.occupied_cells == ()
    assert degraded.local_costmap.revision > remembered.local_costmap.revision
    recovered = _tick(model, temporal_expiry + 40_000_000)
    assert recovered.local_costmap.occupied_cells == remembered.local_costmap.occupied_cells
    boundary = last_ns + config.structural_max_age_ns
    valid = _tick(model, boundary)
    assert valid.local_costmap.occupied_cells
    expired = _tick(model, boundary + 1)
    assert expired.local_costmap.occupied_cells == ()
    assert expired.local_costmap.revision > valid.local_costmap.revision
    assert expired.local_costmap.source_sequence == sequence
    assert expired.local_costmap.freshness_ns == config.structural_max_age_ns + 1


def test_lidar_rolling_window_recall_free_space_and_continuity():
    config = _config()
    model = ShadowWorldModel(config)
    last_ns, sequence, initial = _confirmed_wall(model, config, point=(2.35, 0.05))
    outside = _tick(model, last_ns + 20_000_000, x=-0.2)
    assert outside.local_costmap.occupied_cells == ()
    assert outside.local_costmap.revision > initial.local_costmap.revision
    recalled = _tick(model, last_ns + 40_000_000)
    assert recalled.local_costmap.occupied_cells == initial.local_costmap.occupied_cells
    assert recalled.local_costmap.revision > outside.local_costmap.revision
    now = last_ns + 60_000_000
    cleared = _tick(model, now, _scan(sequence + 1, now, ((2.45, 0.05),)))
    assert initial.local_costmap.occupied_cells[0] not in cleared.local_costmap.occupied_cells
    # A pose discontinuity is an immediate reset, even with no new sensor data.
    reset = _tick(model, now + 20_000_000, x=1.0)
    assert reset.local_costmap is None
    assert reset.map_revision > cleared.map_revision
    now += 40_000_000
    rebuilt = _tick(model, now, _scan(sequence + 2, now), x=1.0)
    assert rebuilt.local_costmap is not None
    reframed = _tick(model, now + 20_000_000, x=1.0, frame_id="relocated")
    assert reframed.frame_id == "relocated"
    assert reframed.local_costmap is None


def test_lidar_checkpoint_restore_preserves_cache_and_future_invalidation(monkeypatch):
    config = _config()
    original = ShadowWorldModel(config)
    last_ns, sequence, initial = _confirmed_wall(original, config)
    restored = ShadowWorldModel(config)
    restored.restore(pickle.loads(pickle.dumps(original.checkpoint())))
    with monkeypatch.context() as patch:
        patch.setattr(TemporalOccupancyGrid, "occupied_cells", _no_map_work)
        patch.setattr(StructuralMemoryGrid, "confirmed_cells", _no_map_work)
        assert _tick(original, last_ns + 20_000_000) == _tick(restored, last_ns + 20_000_000)
    for offset, variance in ((config.local_costmap_max_cell_age_ns + 1, 0.0),
                             (1_000_000_000, 1.0), (1_020_000_000, 0.0),
                             (config.structural_max_age_ns + 1, 0.0)):
        now = last_ns + offset
        assert _tick(original, now, variance=variance) == _tick(restored, now, variance=variance)
    now += 20_000_000
    observations = _scan(sequence + 1, now)
    assert _tick(original, now, observations) == _tick(restored, now, observations)


def test_lidar_duplicate_measurement_and_cached_costmap_remain_immutable():
    model = ShadowWorldModel(_config())
    now = 1_000_000_000
    observations = _scan(1, now)
    first = _tick(model, now, observations)
    duplicate = _tick(model, now + 20_000_000, observations)
    assert duplicate.local_costmap.occupied_cells is first.local_costmap.occupied_cells
    assert duplicate.local_costmap.revision == first.local_costmap.revision
    assert duplicate.local_costmap.freshness_ns == 20_000_000
    assert first.local_costmap.freshness_ns == 0
    with pytest.raises(ValueError, match="rewritten"):
        _tick(model, now + 40_000_000, _scan(1, now, ((1.5, 0.0),)))
    with pytest.raises(ValueError):
        first.local_costmap.with_freshness_ns(-1)
    with pytest.raises(ValueError, match="duplicates"):
        replace(first.local_costmap, occupied_cells=(CostmapCell(1, 1, 1),) * 2)
    with pytest.raises(ValueError, match="immutable tuple"):
        replace(first.local_costmap, occupied_cells=list(first.local_costmap.occupied_cells))
