from v3_config_fixtures import configured
import pytest

from v3.contracts import AdmittedFrame, DataField, Observation, RobotEstimate, TickContext
from v3.layers.l4_structural_memory import StructuralMemoryGrid
from v3.layers.l4_temporal_occupancy import ScanCellEvidence
from v3.layers.l4_world_model import ShadowWorldModel, WorldModelConfig


def _estimate(
    context: TickContext,
    *,
    position_variance: float = 0.01,
    yaw_variance: float = 0.01,
) -> RobotEstimate:
    covariance = [0.0] * 25
    covariance[0] = position_variance
    covariance[6] = position_variance
    covariance[12] = yaw_variance
    covariance[18] = 0.01
    covariance[24] = 0.01
    return RobotEstimate(
        context,
        "R2B4_BOOT_ROBOT_MAP",
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        tuple(covariance),
    )


def _lidar_values(points: tuple[tuple[float, float, int], ...]) -> tuple[DataField, ...]:
    values = [DataField("frame_id", "ROBOT_BASE"), DataField("point_count", len(points))]
    for index, (x_m, y_m, quality) in enumerate(points):
        values.extend(
            (
                DataField(f"point_{index:03d}_x_m", x_m),
                DataField(f"point_{index:03d}_y_m", y_m),
                DataField(f"point_{index:03d}_quality", quality),
            )
        )
    return tuple(values)


def _track_values(
    *,
    track_id: str,
    x_m: float = 1.0,
    y_m: float = 0.0,
    vx_mps: float = 0.0,
    vy_mps: float = 0.0,
) -> tuple[DataField, ...]:
    return (
        DataField("track_id", track_id),
        DataField("x_m", x_m),
        DataField("y_m", y_m),
        DataField("radius_m", 0.30),
        DataField("vx_mps", vx_mps),
        DataField("vy_mps", vy_mps),
        DataField("confidence", 1.0),
    )


def _frame(
    context: TickContext,
    *,
    sequence: int,
    local_captured_ns: int | None = None,
    points: tuple[tuple[float, float, int], ...] | None = ((1.0, 0.0, 10),),
    track: tuple[DataField, ...] | None = None,
    track_captured_ns: int | None = None,
) -> AdmittedFrame:
    observations: list[Observation] = [
        Observation(
            "lidar_health",
            "RPLIDAR_C1",
            sequence,
            context.monotonic_ns,
            (
                DataField("age_ns", 0),
                DataField("point_count", 0 if points is None else len(points)),
            ),
        )
    ]
    if points is not None:
        observations.append(
            Observation(
                "lidar_local_points",
                "RPLIDAR_C1",
                sequence,
                context.monotonic_ns if local_captured_ns is None else local_captured_ns,
                _lidar_values(points),
            )
        )
    if track is not None:
        observations.append(
            Observation(
                "obstacle_track",
                "TRACKER",
                sequence,
                context.monotonic_ns if track_captured_ns is None else track_captured_ns,
                track,
            )
        )
    return AdmittedFrame(context, tuple(observations), ())


def _model() -> ShadowWorldModel:
    return configured(ShadowWorldModel, 
        configured(WorldModelConfig, 
            structural_confirm_score=3,
            structural_deconfirm_score=2,
            structural_confirm_min_hits=3,
            structural_confirm_min_span_ns=200_000_000,
        )
    )


def test_moving_track_hit_is_not_learned_as_structural_geometry():
    model = _model()
    base = 1_000_000_000
    for index in range(4):
        context = TickContext(index, base + index * 100_000_000)
        world = model(
            _frame(
                context,
                sequence=index + 1,
                track=_track_values(track_id="moving-1", vx_mps=0.20),
            ),
            _estimate(context),
        )
        assert world.local_costmap is not None
        assert (10, 0) in {
            (cell.grid_x, cell.grid_y) for cell in world.local_costmap.occupied_cells
        }

    checkpoint = model.checkpoint()
    assert checkpoint.structural_memory is not None
    assert checkpoint.structural_memory.cells == ()


def test_stationary_person_track_is_masked_from_structural_learning():
    model = _model()
    base = 2_000_000_000
    for index in range(4):
        context = TickContext(index, base + index * 100_000_000)
        model(
            _frame(
                context,
                sequence=index + 1,
                track=_track_values(track_id="person-99", vx_mps=0.0),
            ),
            _estimate(context),
        )

    checkpoint = model.checkpoint()
    assert checkpoint.structural_memory is not None
    assert checkpoint.structural_memory.cells == ()


def test_delayed_scan_uses_measurement_time_covariance_for_structural_learning():
    model = _model()
    t0 = 3_000_000_000
    first = TickContext(0, t0)
    model(_frame(first, sequence=1, points=None), _estimate(first, position_variance=0.30))

    # The scan becomes visible later, when current localization is good. Structural
    # learning must still use the bad covariance attached to the pose at t0.
    second = TickContext(1, t0 + 100_000_000)
    model(
        _frame(second, sequence=2, local_captured_ns=t0),
        _estimate(second, position_variance=0.01),
    )

    checkpoint = model.checkpoint()
    assert checkpoint.structural_memory is not None
    assert checkpoint.structural_memory.cells == ()


def test_structural_confirmation_has_separate_deconfirm_hysteresis():
    grid = StructuralMemoryGrid(
        resolution_m=0.1,
        max_age_ns=60_000_000_000,
        max_cells=16,
        hit_increment=1,
        free_decrement=1,
        max_score=8,
        confirm_score=4,
        deconfirm_score=2,
        confirm_min_hits=1,
        confirm_min_span_ns=0,
        clear_score=0,
    )
    hit = ScanCellEvidence(((10, 0),), ())
    free = ScanCellEvidence((), ((10, 0),))

    for captured_ns in range(1, 5):
        grid.integrate(hit, captured_ns=captured_ns)
    checkpoint = grid.checkpoint()
    cell = checkpoint.cells[0][2]
    assert cell.score == 4
    assert cell.confirmed is True

    grid.integrate(free, captured_ns=5)
    cell = grid.checkpoint().cells[0][2]
    assert cell.score == 3
    assert cell.confirmed is True

    grid.integrate(free, captured_ns=6)
    cell = grid.checkpoint().cells[0][2]
    assert cell.score == 2
    assert cell.confirmed is False

    # The cell still exists as candidate evidence; deconfirm is intentionally
    # separate from physical deletion at clear_score.
    assert grid.confirmed_cells(center_x_m=0.0, center_y_m=0.0, radius_m=2.5) == ()


def test_ignored_dynamic_hits_do_not_destroy_current_free_veto_state():
    grid = StructuralMemoryGrid(
        resolution_m=0.1,
        max_age_ns=60_000_000_000,
        max_cells=16,
        hit_increment=1,
        free_decrement=1,
        max_score=8,
        confirm_score=4,
        deconfirm_score=2,
        confirm_min_hits=4,
        confirm_min_span_ns=0,
        clear_score=0,
    )
    evidence = ScanCellEvidence(((10, 0),), ((1, 0), (2, 0)))
    grid.integrate(evidence, captured_ns=1, ignored_hit_keys=frozenset({(10, 0)}))
    checkpoint = grid.checkpoint()
    assert checkpoint.cells == ()
    assert checkpoint.current_hit_keys == ((10, 0),)
    assert checkpoint.current_free_keys == ((1, 0), (2, 0))
