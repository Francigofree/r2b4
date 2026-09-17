from __future__ import annotations

import math
from pathlib import Path

import pytest

from v3.adapters.resident_command import (
    AtomicResidentCommandGateway,
    ResidentCommandClient,
    ResidentCommandMailboxConfig,
)
from v3.contracts import (
    CommandMode,
    CommandRequest,
    DataField,
    MissionLifecycle,
    NavigationStatus,
    ObstacleTrack,
    RobotEstimate,
    TickContext,
    WorldSnapshot,
)
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import NavigationConfig, TrajectoryNavigator
from v3.layers.l7_motion_selection import select_motion
from v3.layers.l8_motion_realization import MotionRealizer


def _estimate(context: TickContext, *, yaw: float = 0.0) -> RobotEstimate:
    covariance = tuple(0.01 if index % 6 == 0 else 0.0 for index in range(25))
    return RobotEstimate(
        context=context,
        frame_id="R2B4_BOOT_ROBOT_MAP",
        x_m=0.0,
        y_m=0.0,
        yaw_rad=yaw,
        v_mps=0.0,
        omega_rad_s=0.0,
        covariance_5x5=covariance,
    )


def _world(context: TickContext, *tracks: ObstacleTrack) -> WorldSnapshot:
    return WorldSnapshot(
        context=context,
        frame_id="R2B4_BOOT_ROBOT_MAP",
        map_revision=1,
        obstacle_tracks=tuple(tracks),
        freshness_ns=0,
        local_costmap=None,
    )


def _person(track_id: str, x: float, y: float, confidence: float = 0.9) -> ObstacleTrack:
    return ObstacleTrack(
        track_id=track_id,
        x_m=x,
        y_m=y,
        radius_m=0.30,
        vx_mps=0.0,
        vy_mps=0.0,
        confidence=confidence,
    )


def _mission(context: TickContext, *, command_id: str = "face-1"):
    return MissionManager().evaluate(
        CommandRequest(
            context=context,
            command_id=command_id,
            mode=CommandMode.FACE_PERSON,
            goal=(DataField("max_omega_rad_s", 0.50),),
            expiry_tick=context.tick_id,
        )
    )


def test_face_person_is_an_active_targetless_mission():
    context = TickContext(1, 1_000_000_000)
    mission = _mission(context)
    assert mission.mode is CommandMode.FACE_PERSON
    assert mission.lifecycle is MissionLifecycle.ACTIVE
    assert mission.target_pose is None
    assert mission.velocity_target is None
    assert mission.constraints.max_omega_rad_s == pytest.approx(0.50)


@pytest.mark.parametrize(("y_m", "sign"), [(1.0, 1.0), (-1.0, -1.0)])
def test_face_person_uses_existing_l7_l8_path_and_never_requests_translation(y_m, sign):
    context = TickContext(2, 1_020_000_000)
    estimate = _estimate(context)
    world = _world(context, _person("person-1", 1.0, y_m))
    plan = TrajectoryNavigator().evaluate(_mission(context), estimate, world)
    assert plan.status is NavigationStatus.ACTIVE
    assert len(plan.route) == 1
    assert plan.route[0].x_m == pytest.approx(estimate.x_m)
    assert plan.route[0].y_m == pytest.approx(estimate.y_m)

    motion = MotionRealizer().evaluate(select_motion(plan), estimate, world)
    assert motion.requested_v_mps == 0.0
    assert math.copysign(1.0, motion.requested_omega_rad_s) == sign


def test_face_person_alignment_hysteresis_prevents_chatter():
    navigator = TrajectoryNavigator(
        NavigationConfig(
            face_person_align_tolerance_rad=0.10,
            face_person_release_tolerance_rad=0.16,
        )
    )
    command_id = "face-hysteresis"

    c1 = TickContext(10, 1_000_000_000)
    p1 = navigator.evaluate(
        _mission(c1, command_id=command_id),
        _estimate(c1),
        _world(c1, _person("person-1", math.cos(0.05), math.sin(0.05))),
    )
    assert navigator.checkpoint().face_person_aligned is True
    assert p1.route[0].yaw_rad == pytest.approx(0.0)

    c2 = TickContext(11, 1_020_000_000)
    p2 = navigator.evaluate(
        _mission(c2, command_id=command_id),
        _estimate(c2),
        _world(c2, _person("person-1", math.cos(0.12), math.sin(0.12))),
    )
    assert navigator.checkpoint().face_person_aligned is True
    assert p2.route[0].yaw_rad == pytest.approx(0.0)

    c3 = TickContext(12, 1_040_000_000)
    p3 = navigator.evaluate(
        _mission(c3, command_id=command_id),
        _estimate(c3),
        _world(c3, _person("person-1", math.cos(0.20), math.sin(0.20))),
    )
    assert navigator.checkpoint().face_person_aligned is False
    assert p3.route[0].yaw_rad == pytest.approx(0.20)


def test_face_person_keeps_selected_track_until_it_is_lost():
    navigator = TrajectoryNavigator()
    command_id = "face-sticky"

    c1 = TickContext(20, 2_000_000_000)
    navigator.evaluate(
        _mission(c1, command_id=command_id),
        _estimate(c1),
        _world(
            c1,
            _person("person-1", 1.0, 0.1, 0.80),
            _person("person-2", 2.0, 0.8, 0.90),
        ),
    )
    assert navigator.checkpoint().face_person_track_id == "person-2"

    c2 = TickContext(21, 2_020_000_000)
    navigator.evaluate(
        _mission(c2, command_id=command_id),
        _estimate(c2),
        _world(
            c2,
            _person("person-1", 1.0, -0.1, 0.99),
            _person("person-2", 2.0, 0.7, 0.61),
        ),
    )
    assert navigator.checkpoint().face_person_track_id == "person-2"

    c3 = TickContext(22, 2_040_000_000)
    plan = navigator.evaluate(
        _mission(c3, command_id=command_id),
        _estimate(c3),
        _world(c3, _person("person-1", 1.0, -0.1, 0.99)),
    )
    assert plan.status is NavigationStatus.ACTIVE
    assert navigator.checkpoint().face_person_track_id == "person-1"


def test_face_person_target_loss_is_fail_closed_but_reacquirable():
    navigator = TrajectoryNavigator()
    command_id = "face-loss"

    c1 = TickContext(30, 3_000_000_000)
    missing = navigator.evaluate(
        _mission(c1, command_id=command_id),
        _estimate(c1),
        _world(c1),
    )
    assert missing.status is NavigationStatus.INVALIDATED
    assert missing.reason == "PERSON_TARGET_NOT_AVAILABLE"

    c2 = TickContext(31, 3_020_000_000)
    reacquired = navigator.evaluate(
        _mission(c2, command_id=command_id),
        _estimate(c2),
        _world(c2, _person("person-7", 1.0, 0.5)),
    )
    assert reacquired.status is NavigationStatus.ACTIVE
    assert navigator.checkpoint().face_person_track_id == "person-7"


def test_face_person_checkpoint_restores_target_identity_and_alignment():
    nav = TrajectoryNavigator()
    c1 = TickContext(40, 4_000_000_000)
    nav.evaluate(
        _mission(c1, command_id="face-checkpoint"),
        _estimate(c1),
        _world(c1, _person("person-9", 1.0, 0.01)),
    )
    checkpoint = nav.checkpoint()
    assert checkpoint.face_person_track_id == "person-9"
    assert checkpoint.face_person_aligned is True

    restored = TrajectoryNavigator()
    restored.restore(checkpoint)
    assert restored.checkpoint().face_person_track_id == "person-9"
    assert restored.checkpoint().face_person_aligned is True


def test_resident_face_person_command_roundtrip(tmp_path: Path):
    path = tmp_path / "command.json"
    config = ResidentCommandMailboxConfig(path=path)
    client = ResidentCommandClient(config, monotonic_ns=lambda: 10_000_000_000)
    client.publish_face_person(
        "face-mailbox",
        max_omega_rad_s=0.50,
        ttl_ns=200_000_000,
    )

    gateway = AtomicResidentCommandGateway(
        config,
        monotonic_ns=lambda: 10_000_000_000,
    )
    command = gateway.snapshot(TickContext(50, 10_000_000_000))
    assert command.mode is CommandMode.FACE_PERSON
    assert command.goal == (DataField("max_omega_rad_s", 0.50),)
