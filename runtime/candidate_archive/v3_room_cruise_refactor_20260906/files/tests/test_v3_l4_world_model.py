import pytest

from v3.contracts import (
    AdmittedFrame,
    DataField,
    Observation,
    RobotEstimate,
    TickContext,
)
from v3.layers.l4_world_model import ShadowWorldModel, WorldModelConfig


def _estimate(context: TickContext, *, x_m: float = 1.0, y_m: float = 2.0):
    covariance = tuple(0.01 if index % 6 == 0 else 0.0 for index in range(25))
    return RobotEstimate(
        context,
        "R2B4_BOOT_ROBOT_MAP",
        x_m,
        y_m,
        0.0,
        0.0,
        0.0,
        covariance,
    )


def _frame(
    context: TickContext,
    *,
    sequence: int,
    local_points: tuple[tuple[float, float, int], ...] | None,
    captured_monotonic_ns: int | None = None,
) -> AdmittedFrame:
    captured_ns = (
        context.monotonic_ns
        if captured_monotonic_ns is None
        else captured_monotonic_ns
    )
    observations = [
        Observation(
            "lidar_health",
            "RPLIDAR_C1",
            sequence,
            captured_ns,
            (
                DataField("age_ns", context.monotonic_ns - captured_ns),
                DataField("point_count", 20),
            ),
        )
    ]
    if local_points is not None:
        values = [
            DataField("frame_id", "ROBOT_BASE"),
            DataField("point_count", len(local_points)),
        ]
        for index, (x_m, y_m, quality) in enumerate(local_points):
            values.extend(
                (
                    DataField(f"point_{index:03d}_x_m", x_m),
                    DataField(f"point_{index:03d}_y_m", y_m),
                    DataField(f"point_{index:03d}_quality", quality),
                )
            )
        observations.append(
            Observation(
                "lidar_local_points",
                "RPLIDAR_C1",
                sequence,
                captured_ns,
                tuple(values),
            )
        )
    return AdmittedFrame(context, tuple(observations), ())


def test_l4_builds_and_retains_a_bounded_map_frame_rolling_costmap():
    model = ShadowWorldModel(
        WorldModelConfig(
            local_costmap_resolution_m=0.1,
            local_costmap_radius_m=2.5,
            local_costmap_max_cell_age_ns=200,
            local_costmap_max_cells=2,
        )
    )
    first_context = TickContext(0, 1_000)

    first = model(
        _frame(
            first_context,
            sequence=1,
            local_points=((0.25, 0.05, 10), (0.26, 0.06, 8), (-0.2, 0.0, 5)),
        ),
        _estimate(first_context),
    )

    assert first.local_costmap is not None
    assert first.local_costmap.frame_id == first.frame_id
    assert tuple(
        (cell.grid_x, cell.grid_y, cell.observation_count)
        for cell in first.local_costmap.occupied_cells
    ) == ((8, 20, 1), (12, 20, 1))
    assert first.local_costmap.source_sequence == 1

    second_context = TickContext(1, 1_100)
    retained = model(
        _frame(second_context, sequence=2, local_points=None),
        _estimate(second_context),
    )
    assert retained.local_costmap is not None
    assert retained.local_costmap.occupied_cells == first.local_costmap.occupied_cells
    assert retained.local_costmap.freshness_ns == 100

    expired_context = TickContext(2, 1_201)
    expired = model(
        _frame(expired_context, sequence=3, local_points=None),
        _estimate(expired_context),
    )
    assert expired.local_costmap is not None
    assert expired.local_costmap.occupied_cells == ()


def test_l4_rejects_malformed_or_non_robot_frame_local_perception():
    context = TickContext(0, 1_000)
    frame = _frame(context, sequence=1, local_points=((0.2, 0.0, 4),))
    health, local = frame.accepted
    malformed = Observation(
        local.kind,
        local.source_device_id,
        local.source_sequence,
        local.captured_monotonic_ns,
        tuple(
            DataField(field.key, "wrong") if field.key == "frame_id" else field
            for field in local.values
        ),
    )

    with pytest.raises(ValueError, match="ROBOT_BASE"):
        ShadowWorldModel()(AdmittedFrame(context, (health, malformed), ()), _estimate(context))


def test_l4_accepts_an_exact_repeated_scan_without_counting_it_twice():
    model = ShadowWorldModel()
    first_context = TickContext(0, 1_000)
    points = ((0.25, 0.05, 10),)
    first = model(
        _frame(first_context, sequence=1, local_points=points),
        _estimate(first_context),
    )
    second_context = TickContext(1, 1_100)
    repeated = model(
        _frame(
            second_context,
            sequence=1,
            local_points=points,
            captured_monotonic_ns=first_context.monotonic_ns,
        ),
        _estimate(second_context),
    )

    assert repeated.local_costmap is not None
    assert first.local_costmap is not None
    assert repeated.local_costmap.revision == first.local_costmap.revision
    assert repeated.local_costmap.occupied_cells == first.local_costmap.occupied_cells
    assert repeated.local_costmap.freshness_ns == 100
