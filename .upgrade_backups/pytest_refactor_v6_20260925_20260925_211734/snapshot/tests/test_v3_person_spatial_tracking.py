from v3_config_fixtures import configured
import pytest

from v3.contracts import AdmittedFrame, DataField, Observation, RobotEstimate, TickContext
from v3.layers.l4_world_model import ShadowWorldModel, WorldModelConfig


def _estimate(context: TickContext) -> RobotEstimate:
    covariance = tuple(0.01 if index % 6 == 0 else 0.0 for index in range(25))
    return RobotEstimate(
        context,
        "R2B4_BOOT_ROBOT_MAP",
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        covariance,
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


def _person_values(*boxes: tuple[float, float, float]) -> tuple[DataField, ...]:
    # (confidence, xmin, xmax)
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
        center_x = (xmin + xmax) * 0.5
        values.extend(
            (
                DataField("primary_confidence", confidence),
                DataField("primary_xmin", xmin),
                DataField("primary_ymin", 0.1),
                DataField("primary_xmax", xmax),
                DataField("primary_ymax", 0.9),
                DataField("primary_center_x", center_x),
                DataField("primary_center_y", 0.5),
                DataField("primary_area", (xmax - xmin) * 0.8),
            )
        )
    for index, (confidence, xmin, xmax) in enumerate(boxes):
        prefix = f"person_{index:03d}"
        center_x = (xmin + xmax) * 0.5
        values.extend(
            (
                DataField(f"{prefix}_confidence", confidence),
                DataField(f"{prefix}_xmin", xmin),
                DataField(f"{prefix}_ymin", 0.1),
                DataField(f"{prefix}_xmax", xmax),
                DataField(f"{prefix}_ymax", 0.9),
                DataField(f"{prefix}_center_x", center_x),
                DataField(f"{prefix}_center_y", 0.5),
                DataField(f"{prefix}_area", (xmax - xmin) * 0.8),
            )
        )
    return tuple(values)


def _frame(
    context: TickContext,
    *,
    sequence: int,
    lidar_points: tuple[tuple[float, float, int], ...],
    person_boxes: tuple[tuple[float, float, float], ...],
    lidar_captured_ns: int | None = None,
    person_captured_ns: int | None = None,
) -> AdmittedFrame:
    lidar_ns = context.monotonic_ns if lidar_captured_ns is None else lidar_captured_ns
    person_ns = context.monotonic_ns if person_captured_ns is None else person_captured_ns
    accepted = (
        Observation(
            "lidar_health",
            "RPLIDAR_C1",
            sequence,
            lidar_ns,
            (
                DataField("age_ns", context.monotonic_ns - lidar_ns),
                DataField("point_count", len(lidar_points)),
            ),
        ),
        Observation(
            "lidar_local_points",
            "RPLIDAR_C1",
            sequence,
            lidar_ns,
            _lidar_values(lidar_points),
        ),
        Observation(
            "person_detection",
            "PERSON_DETECTOR_FRONT",
            sequence,
            person_ns,
            _person_values(*person_boxes),
        ),
    )
    return AdmittedFrame(context, accepted, ())


def test_l4_localizes_person_from_camera_bearing_and_lidar_depth():
    context = TickContext(0, 1_000_000_000)
    world = configured(ShadowWorldModel, )(
        _frame(
            context,
            sequence=1,
            lidar_points=((1.40, -0.02, 10), (1.42, 0.02, 9), (2.0, 1.0, 8)),
            person_boxes=((0.91, 0.43, 0.57),),
        ),
        _estimate(context),
    )

    people = tuple(track for track in world.obstacle_tracks if track.track_id.startswith("person-"))
    assert len(people) == 1
    assert people[0].track_id == "person-1"
    assert people[0].x_m == pytest.approx(1.41, abs=0.03)
    assert people[0].y_m == pytest.approx(0.0, abs=0.03)
    assert people[0].vx_mps == 0.0
    assert people[0].vy_mps == 0.0
    assert people[0].confidence == pytest.approx(0.91)


def test_l4_keeps_person_identity_and_derives_velocity():
    model = configured(ShadowWorldModel, configured(WorldModelConfig, person_track_max_speed_mps=6.0))
    first_context = TickContext(0, 1_000_000_000)
    model(
        _frame(
            first_context,
            sequence=1,
            lidar_points=((1.40, 0.0, 10),),
            person_boxes=((0.9, 0.44, 0.56),),
        ),
        _estimate(first_context),
    )

    second_context = TickContext(1, 1_100_000_000)
    world = model(
        _frame(
            second_context,
            sequence=2,
            lidar_points=((1.42, 0.0, 10),),
            person_boxes=((0.92, 0.44, 0.56),),
        ),
        _estimate(second_context),
    )

    person = next(track for track in world.obstacle_tracks if track.track_id == "person-1")
    assert person.x_m == pytest.approx(1.42)
    assert person.vx_mps == pytest.approx(0.2, abs=1e-6)
    assert person.vy_mps == pytest.approx(0.0, abs=1e-6)


def test_l4_does_not_fuse_person_with_time_misaligned_lidar():
    config = configured(WorldModelConfig, person_lidar_max_skew_ns=50_000_000)
    model = configured(ShadowWorldModel, config)
    context = TickContext(0, 1_000_000_000)
    world = model(
        _frame(
            context,
            sequence=1,
            lidar_points=((1.4, 0.0, 10),),
            person_boxes=((0.9, 0.44, 0.56),),
            lidar_captured_ns=900_000_000,
            person_captured_ns=1_000_000_000,
        ),
        _estimate(context),
    )
    assert not any(track.track_id.startswith("person-") for track in world.obstacle_tracks)


def test_l4_person_tracking_checkpoint_restore_is_deterministic():
    config = configured(WorldModelConfig, )
    model = configured(ShadowWorldModel, config)
    first_context = TickContext(0, 1_000_000_000)
    model(
        _frame(
            first_context,
            sequence=1,
            lidar_points=((1.4, 0.0, 10),),
            person_boxes=((0.9, 0.44, 0.56),),
        ),
        _estimate(first_context),
    )
    checkpoint = model.checkpoint()

    second_context = TickContext(1, 1_100_000_000)
    frame = _frame(
        second_context,
        sequence=2,
        lidar_points=((1.45, 0.0, 10),),
        person_boxes=((0.93, 0.44, 0.56),),
    )
    expected = model(frame, _estimate(second_context))

    restored = configured(ShadowWorldModel, config)
    restored.restore(checkpoint)
    actual = restored(frame, _estimate(second_context))
    assert actual == expected


def test_live_world_path_preserves_measurement_through_empty_detection_and_dropout():
    from dataclasses import replace
    from v3.contracts import TrackEstimateStatus

    model = configured(ShadowWorldModel, )
    context = TickContext(0, 1_000_000_000)
    frame = _frame(context, sequence=1, lidar_points=((2.0, 0.0, 10),),
                   person_boxes=((0.9, 0.43, 0.57),))
    observed = model(frame, _estimate(context)).obstacle_tracks[0]
    assert observed.estimate_status is TrackEstimateStatus.OBSERVED
    for tick, age in ((1, 100_000_000), (2, 200_000_000), (3, 350_000_001)):
        context = TickContext(tick, 1_000_000_000 + age)
        frame = _frame(context, sequence=tick + 1, lidar_points=((2.0, 0.0, 10),), person_boxes=())
        if tick == 2:
            frame = replace(frame, accepted=tuple(item for item in frame.accepted if item.kind != "person_detection"))
        track = model(frame, _estimate(context)).obstacle_tracks[0]
        assert track.track_id == observed.track_id
        assert track.measurement_monotonic_ns == observed.measurement_monotonic_ns
        assert track.prediction_valid_until_ns == observed.prediction_valid_until_ns
        assert track.estimate_status is (TrackEstimateStatus.DEGRADED if tick == 3 else TrackEstimateStatus.PREDICTED)
    restored = configured(ShadowWorldModel, )
    restored.restore(model.checkpoint())
    context = TickContext(4, 1_400_000_000)
    frame = _frame(context, sequence=5, lidar_points=((2.0, 0.0, 10),), person_boxes=((0.9, 0.43, 0.57),))
    assert model(frame, _estimate(context)) == restored(frame, _estimate(context))
    assert model.checkpoint() == restored.checkpoint()
