from v3_config_fixtures import configured
import pytest

from v3.contracts import AdmittedFrame, DataField, Observation, RobotEstimate, TickContext
from v3.layers.l4_structural_memory import StructuralMemoryGrid
from v3.layers.l4_temporal_occupancy import ScanCellEvidence
from v3.layers.l4_world_model import ShadowWorldModel, WorldModelConfig


def _estimate(
    context: TickContext,
    *,
    x_m: float = 0.0,
    y_m: float = 0.0,
    yaw_rad: float = 0.0,
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
        x_m,
        y_m,
        yaw_rad,
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


def _frame(
    context: TickContext,
    *,
    sequence: int,
    lidar_points: tuple[tuple[float, float, int], ...] | None,
) -> AdmittedFrame:
    observations = [
        Observation(
            "lidar_health",
            "RPLIDAR_C1",
            sequence,
            context.monotonic_ns,
            (
                DataField("age_ns", 0),
                DataField("point_count", 0 if lidar_points is None else len(lidar_points)),
            ),
        )
    ]
    if lidar_points is not None:
        observations.append(
            Observation(
                "lidar_local_points",
                "RPLIDAR_C1",
                sequence,
                context.monotonic_ns,
                _lidar_values(lidar_points),
            )
        )
    return AdmittedFrame(context, tuple(observations), ())



def _model() -> ShadowWorldModel:
    return configured(ShadowWorldModel, 
        configured(WorldModelConfig, 
            structural_confirm_score=4,
            structural_confirm_min_hits=4,
            structural_confirm_min_span_ns=300_000_000,
        )
    )


def _cell_keys(world) -> set[tuple[int, int]]:
    assert world.local_costmap is not None
    return {(cell.grid_x, cell.grid_y) for cell in world.local_costmap.occupied_cells}


def _confirm_wall(model: ShadowWorldModel, *, first_ns: int = 1_000_000_000) -> int:
    for index in range(4):
        context = TickContext(index, first_ns + index * 100_000_000)
        world = model(
            _frame(context, sequence=index + 1, lidar_points=((1.0, 0.0, 10),)),
            _estimate(context),
        )
    assert (10, 0) in _cell_keys(world)
    return first_ns + 300_000_000


def test_structural_memory_recalls_confirmed_geometry_after_fast_occupancy_expires():
    model = _model()
    last_scan_ns = _confirm_wall(model)

    context = TickContext(4, last_scan_ns + 900_000_000)
    world = model(
        _frame(context, sequence=5, lidar_points=None),
        _estimate(context),
    )

    assert (10, 0) in _cell_keys(world)
    assert world.local_costmap is not None
    # Structural recall must never pretend that the old local scan is fresh.
    assert world.local_costmap.freshness_ns == 900_000_000
    assert world.local_costmap.source_sequence == 4


def test_current_free_space_vetoes_confirmed_structural_cell_immediately():
    model = _model()
    last_scan_ns = _confirm_wall(model, first_ns=2_000_000_000)

    context = TickContext(4, last_scan_ns + 100_000_000)
    world = model(
        _frame(context, sequence=5, lidar_points=((1.5, 0.0, 10),)),
        _estimate(context),
    )

    keys = _cell_keys(world)
    assert (10, 0) not in keys
    assert (15, 0) in keys


def test_repeated_free_space_clears_structural_memory_not_only_output_veto():
    model = _model()
    last_scan_ns = _confirm_wall(model, first_ns=3_000_000_000)

    for offset in range(1, 5):
        context = TickContext(3 + offset, last_scan_ns + offset * 100_000_000)
        model(
            _frame(context, sequence=4 + offset, lidar_points=((1.5, 0.0, 10),)),
            _estimate(context),
        )

    # Move the fresh scan so grid 10 is no longer in the latest ray and let fast
    # occupancy age out. If structural clearing worked, grid 10 must not return.
    context = TickContext(8, last_scan_ns + 1_400_000_000)
    world = model(
        _frame(context, sequence=9, lidar_points=((0.0, 1.5, 10),)),
        _estimate(context),
    )
    assert (10, 0) not in _cell_keys(world)


def test_bad_localization_covariance_pauses_structural_learning():
    model = _model()
    first_ns = 4_000_000_000
    for index in range(4):
        context = TickContext(index, first_ns + index * 100_000_000)
        model(
            _frame(context, sequence=index + 1, lidar_points=((1.0, 0.0, 10),)),
            _estimate(context, position_variance=0.30),
        )

    # Let fast occupancy expire, then recover localization without a new local scan.
    context = TickContext(4, first_ns + 1_200_000_000)
    world = model(
        _frame(context, sequence=5, lidar_points=None),
        _estimate(context, position_variance=0.01),
    )
    assert (10, 0) not in _cell_keys(world)


def test_pose_discontinuity_clears_structural_geometry():
    model = _model()
    last_scan_ns = _confirm_wall(model, first_ns=5_000_000_000)

    context = TickContext(4, last_scan_ns + 100_000_000)
    world = model(
        _frame(context, sequence=5, lidar_points=None),
        _estimate(context, x_m=1.0),
    )
    assert world.local_costmap is None


def test_structural_checkpoint_restore_is_deterministic():
    model = _model()
    last_scan_ns = _confirm_wall(model, first_ns=6_000_000_000)
    checkpoint = model.checkpoint()

    context = TickContext(4, last_scan_ns + 100_000_000)
    frame = _frame(context, sequence=5, lidar_points=((1.5, 0.0, 10),))
    estimate = _estimate(context)
    expected = model(frame, estimate)

    restored = _model()
    restored.restore(checkpoint)
    actual = restored(frame, estimate)
    assert actual == expected


def test_structural_memory_bound_prefers_confirmed_strong_recent_cells():
    grid = StructuralMemoryGrid(
        resolution_m=0.1,
        max_age_ns=60_000_000_000,
        max_cells=2,
        hit_increment=1,
        free_decrement=1,
        max_score=8,
        confirm_score=2,
        confirm_min_hits=2,
        confirm_min_span_ns=1,
        clear_score=0,
    )
    grid.integrate(ScanCellEvidence(((1, 0), (2, 0), (3, 0)), ()), captured_ns=1)
    grid.integrate(ScanCellEvidence(((1, 0), (2, 0)), ()), captured_ns=2)

    checkpoint = grid.checkpoint()
    keys = {(x, y) for x, y, _ in checkpoint.cells}
    assert keys == {(1, 0), (2, 0)}
