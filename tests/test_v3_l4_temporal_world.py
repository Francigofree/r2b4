import pytest

from v3.contracts import AdmittedFrame, DataField, Observation, RobotEstimate, TickContext
from v3.layers.l4_world_model import ShadowWorldModel, WorldModelConfig


def _estimate(context: TickContext, *, x_m: float = 0.0, y_m: float = 0.0, yaw_rad: float = 0.0):
    covariance = tuple(0.01 if index % 6 == 0 else 0.0 for index in range(25))
    return RobotEstimate(context, "R2B4_BOOT_ROBOT_MAP", x_m, y_m, yaw_rad, 0.0, 0.0, covariance)


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


def _person_values(*boxes: tuple[float, float, float]) -> tuple[DataField, ...]:
    values = [
        DataField("age_ns", 0),
        DataField("measurement_timing_valid", True),
        DataField("measurement_stale", False),
        DataField("source_frame_sequence", 1),
        DataField("inference_duration_ns", 10_000_000),
        DataField("person_count", len(boxes)),
        DataField("emitted_person_count", len(boxes)),
        DataField("person_detected", bool(boxes)),
    ]
    if boxes:
        confidence, xmin, xmax = boxes[0]
        values.extend(
            (
                DataField("primary_confidence", confidence),
                DataField("primary_xmin", xmin),
                DataField("primary_ymin", 0.1),
                DataField("primary_xmax", xmax),
                DataField("primary_ymax", 0.9),
                DataField("primary_center_x", (xmin + xmax) * 0.5),
                DataField("primary_center_y", 0.5),
                DataField("primary_area", (xmax - xmin) * 0.8),
            )
        )
    for index, (confidence, xmin, xmax) in enumerate(boxes):
        prefix = f"person_{index:03d}"
        values.extend(
            (
                DataField(f"{prefix}_confidence", confidence),
                DataField(f"{prefix}_xmin", xmin),
                DataField(f"{prefix}_ymin", 0.1),
                DataField(f"{prefix}_xmax", xmax),
                DataField(f"{prefix}_ymax", 0.9),
                DataField(f"{prefix}_center_x", (xmin + xmax) * 0.5),
                DataField(f"{prefix}_center_y", 0.5),
                DataField(f"{prefix}_area", (xmax - xmin) * 0.8),
            )
        )
    return tuple(values)


def _frame(
    context: TickContext,
    *,
    sequence: int,
    lidar_points: tuple[tuple[float, float, int], ...] | None,
    person_boxes: tuple[tuple[float, float, float], ...] | None = None,
    lidar_captured_ns: int | None = None,
    person_captured_ns: int | None = None,
) -> AdmittedFrame:
    lidar_ns = context.monotonic_ns if lidar_captured_ns is None else lidar_captured_ns
    observations = [
        Observation(
            "lidar_health",
            "RPLIDAR_C1",
            sequence,
            lidar_ns,
            (
                DataField("age_ns", context.monotonic_ns - lidar_ns),
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
                lidar_ns,
                _lidar_values(lidar_points),
            )
        )
    if person_boxes is not None:
        person_ns = context.monotonic_ns if person_captured_ns is None else person_captured_ns
        observations.append(
            Observation(
                "person_detection",
                "PERSON_DETECTOR_FRONT",
                sequence,
                person_ns,
                _person_values(*person_boxes),
            )
        )
    return AdmittedFrame(context, tuple(observations), ())


def test_temporal_l4_projects_delayed_lidar_with_measurement_time_pose():
    model = ShadowWorldModel()
    first_context = TickContext(0, 1_000_000_000)
    model(_frame(first_context, sequence=1, lidar_points=None), _estimate(first_context, x_m=0.0))

    second_context = TickContext(1, 1_100_000_000)
    world = model(
        _frame(
            second_context,
            sequence=2,
            lidar_points=((1.0, 0.0, 10),),
            lidar_captured_ns=first_context.monotonic_ns,
        ),
        _estimate(second_context, x_m=0.10),
    )

    assert world.local_costmap is not None
    assert tuple((cell.grid_x, cell.grid_y) for cell in world.local_costmap.occupied_cells) == ((10, 0),)


def test_temporal_l4_uses_historical_scan_for_delayed_person_result():
    model = ShadowWorldModel()
    first_context = TickContext(0, 2_000_000_000)
    model(
        _frame(first_context, sequence=1, lidar_points=((1.0, 0.0, 10),)),
        _estimate(first_context),
    )

    second_context = TickContext(1, 2_100_000_000)
    world = model(
        _frame(
            second_context,
            sequence=2,
            lidar_points=((2.0, 0.0, 10),),
            person_boxes=((0.9, 0.44, 0.56),),
            person_captured_ns=first_context.monotonic_ns,
        ),
        _estimate(second_context),
    )

    person = next(track for track in world.obstacle_tracks if track.track_id == "person-1")
    assert person.x_m == pytest.approx(1.0, abs=1e-6)
    assert person.y_m == pytest.approx(0.0, abs=1e-6)


def test_temporal_l4_free_space_clears_old_hit_on_same_ray():
    model = ShadowWorldModel(WorldModelConfig(local_costmap_resolution_m=0.1))
    first_context = TickContext(0, 3_000_000_000)
    first = model(
        _frame(first_context, sequence=1, lidar_points=((1.0, 0.0, 10),)),
        _estimate(first_context),
    )
    assert first.local_costmap is not None
    assert any(cell.grid_x == 10 for cell in first.local_costmap.occupied_cells)

    second_context = TickContext(1, 3_100_000_000)
    second = model(
        _frame(second_context, sequence=2, lidar_points=((1.5, 0.0, 10),)),
        _estimate(second_context),
    )
    assert second.local_costmap is not None
    x_indices = {cell.grid_x for cell in second.local_costmap.occupied_cells}
    assert 10 not in x_indices
    assert 15 in x_indices


def test_temporal_l4_projects_person_track_to_current_tick_and_restores_deterministically():
    model = ShadowWorldModel()
    first_context = TickContext(0, 4_000_000_000)
    model(
        _frame(
            first_context,
            sequence=1,
            lidar_points=((1.0, 0.0, 10),),
            person_boxes=((0.9, 0.44, 0.56),),
        ),
        _estimate(first_context),
    )
    second_context = TickContext(1, 4_100_000_000)
    model(
        _frame(
            second_context,
            sequence=2,
            lidar_points=((1.1, 0.0, 10),),
            person_boxes=((0.9, 0.44, 0.56),),
        ),
        _estimate(second_context),
    )

    third_context = TickContext(2, 4_200_000_000)
    world = model(
        _frame(third_context, sequence=3, lidar_points=((1.2, 0.0, 10),)),
        _estimate(third_context),
    )
    person = next(track for track in world.obstacle_tracks if track.track_id == "person-1")
    assert person.vx_mps == pytest.approx(1.0, abs=1e-6)
    assert person.x_m == pytest.approx(1.2, abs=1e-6)

    checkpoint = model.checkpoint()
    fourth_context = TickContext(3, 4_250_000_000)
    frame = _frame(fourth_context, sequence=4, lidar_points=((1.25, 0.0, 10),))
    expected = model(frame, _estimate(fourth_context))

    restored = ShadowWorldModel()
    restored.restore(checkpoint)
    actual = restored(frame, _estimate(fourth_context))
    assert actual == expected
