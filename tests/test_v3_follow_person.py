from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from v3 import control_cli, operator_cli
from v3.adapters.resident_command import (
    AtomicResidentCommandGateway,
    ResidentCommandClient,
    ResidentCommandMailboxConfig,
)
from v3.composition.native_control import v3_navigation_config_from_mapping
from v3.contracts import (
    CommandMode,
    CommandRequest,
    DataField,
    MissionLifecycle,
    MotionObjectiveKind,
    NavigationStatus,
    ObstacleTrack,
    RobotEstimate,
    RollingLocalCostmap,
    TickContext,
    WorldSnapshot,
)
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import TrajectoryNavigator
from v3.layers.l7_motion_selection import select_motion
from v3.layers.l8_motion_realization import MotionRealizer


PROJECT_ROOT = Path(__file__).resolve().parents[1]


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


def _person(
    track_id: str,
    x: float,
    y: float,
    confidence: float = 0.9,
) -> ObstacleTrack:
    return ObstacleTrack(
        track_id=track_id,
        x_m=x,
        y_m=y,
        radius_m=0.30,
        vx_mps=0.0,
        vy_mps=0.0,
        confidence=confidence,
    )


def _costmap() -> RollingLocalCostmap:
    return RollingLocalCostmap(
        frame_id="R2B4_BOOT_ROBOT_MAP",
        revision=1,
        resolution_m=0.1,
        radius_m=2.5,
        occupied_cells=(),
        source_sequence=1,
        freshness_ns=0,
    )


def _world(
    context: TickContext,
    *tracks: ObstacleTrack,
    with_costmap: bool = True,
) -> WorldSnapshot:
    return WorldSnapshot(
        context=context,
        frame_id="R2B4_BOOT_ROBOT_MAP",
        map_revision=1,
        obstacle_tracks=tuple(tracks),
        freshness_ns=0,
        local_costmap=_costmap() if with_costmap else None,
    )


def _mission(
    context: TickContext,
    *,
    command_id: str = "follow-1",
    max_v_mps: float = 0.15,
    max_omega_rad_s: float = 0.30,
):
    return MissionManager().evaluate(
        CommandRequest(
            context=context,
            command_id=command_id,
            mode=CommandMode.FOLLOW_PERSON,
            goal=(
                DataField("max_v_mps", max_v_mps),
                DataField("max_omega_rad_s", max_omega_rad_s),
            ),
            expiry_tick=context.tick_id,
        )
    )


def _production_navigation():
    control = json.loads(
        (PROJECT_ROOT / "conf" / "vezerles.json").read_text(encoding="utf-8")
    )
    return v3_navigation_config_from_mapping(control).navigation


def test_follow_person_is_active_targetless_mission():
    context = TickContext(1, 1_000_000_000)
    mission = _mission(context)
    assert mission.mode is CommandMode.FOLLOW_PERSON
    assert mission.lifecycle is MissionLifecycle.ACTIVE
    assert mission.target_pose is None
    assert mission.velocity_target is None
    assert mission.constraints.max_v_mps == pytest.approx(0.15)
    assert mission.constraints.max_omega_rad_s == pytest.approx(0.30)


def test_follow_person_aligns_before_translation():
    context = TickContext(2, 1_020_000_000)
    estimate = _estimate(context)
    navigator = TrajectoryNavigator(_production_navigation())
    plan = navigator.evaluate(
        _mission(context),
        estimate,
        _world(context, _person("person-1", 1.5, 1.0)),
    )

    assert plan.status is NavigationStatus.ACTIVE
    assert len(plan.route) == 1
    assert plan.trajectory_candidates == ()
    assert plan.route[0].x_m == pytest.approx(estimate.x_m)
    assert plan.route[0].y_m == pytest.approx(estimate.y_m)

    motion = MotionRealizer().evaluate(select_motion(plan), estimate, _world(
        context, _person("person-1", 1.5, 1.0)
    ))
    assert motion.requested_v_mps == 0.0
    assert motion.requested_omega_rad_s > 0.0


def test_follow_person_far_and_aligned_advances_with_bounded_safe_motion():
    context = TickContext(3, 1_040_000_000)
    estimate = _estimate(context)
    navigator = TrajectoryNavigator(_production_navigation())
    world = _world(context, _person("person-1", 2.0, 0.0))

    plan = navigator.evaluate(_mission(context), estimate, world)

    assert plan.status is NavigationStatus.ACTIVE
    assert plan.route == ()
    assert plan.local_goal is not None
    assert plan.local_goal.x_m > estimate.x_m
    assert plan.trajectory_candidates

    objective = select_motion(plan)
    assert objective.kind is MotionObjectiveKind.TRACK_TRAJECTORY
    assert objective.trajectory is not None
    assert objective.trajectory.collision is False

    motion = MotionRealizer().evaluate(objective, estimate, world)
    assert motion.requested_v_mps > 0.0
    assert motion.requested_v_mps <= 0.15 + 1e-12


def test_follow_person_holds_inside_standoff_band_and_never_reverses():
    context = TickContext(4, 1_060_000_000)
    estimate = _estimate(context)
    config = _production_navigation()
    navigator = TrajectoryNavigator(config)
    hold_enter = (
        config.follow_person_stand_off_m
        + config.follow_person_distance_deadband_m
    )
    hold_distance = 0.5 * (
        config.follow_person_min_safe_distance_m + hold_enter
    )
    world = _world(context, _person("person-1", hold_distance, 0.0))

    plan = navigator.evaluate(_mission(context), estimate, world)

    assert plan.status is NavigationStatus.ACTIVE
    motion = MotionRealizer().evaluate(select_motion(plan), estimate, world)
    assert motion.requested_v_mps == 0.0
    assert motion.requested_omega_rad_s == 0.0


def test_follow_person_too_close_is_fail_closed():
    context = TickContext(5, 1_080_000_000)
    config = _production_navigation()
    navigator = TrajectoryNavigator(config)
    too_close = config.follow_person_min_safe_distance_m * 0.9
    plan = navigator.evaluate(
        _mission(context),
        _estimate(context),
        _world(context, _person("person-1", too_close, 0.0)),
    )

    assert plan.status is NavigationStatus.INVALIDATED
    assert plan.reason == "PERSON_TOO_CLOSE"
    assert select_motion(plan).kind is MotionObjectiveKind.STOP


def test_follow_person_requires_costmap_only_when_translation_is_needed():
    context = TickContext(6, 1_100_000_000)
    navigator = TrajectoryNavigator(_production_navigation())

    aligned_far = navigator.evaluate(
        _mission(context, command_id="follow-costmap"),
        _estimate(context),
        _world(
            context,
            _person("person-1", 2.0, 0.0),
            with_costmap=False,
        ),
    )
    assert aligned_far.status is NavigationStatus.INVALIDATED
    assert aligned_far.reason == "LOCAL_COSTMAP_MISSING"


def test_follow_person_locks_target_and_never_silently_switches():
    config = _production_navigation()
    navigator = TrajectoryNavigator(config)
    command_id = "follow-sticky"
    distance = 1.8
    locked_angle = 0.70
    other_angle = -0.70

    locked = lambda confidence: _person(
        "person-2",
        distance * math.cos(locked_angle),
        distance * math.sin(locked_angle),
        confidence,
    )
    other = lambda confidence: _person(
        "person-1",
        distance * math.cos(other_angle),
        distance * math.sin(other_angle),
        confidence,
    )

    c1 = TickContext(10, 2_000_000_000)
    acquired_world = _world(c1, other(0.80), locked(0.90))
    acquired = navigator.evaluate(
        _mission(c1, command_id=command_id),
        _estimate(c1),
        acquired_world,
    )
    assert acquired.status is NavigationStatus.ACTIVE
    assert len(acquired.route) == 1
    assert acquired.route[0].yaw_rad > 0.0

    c2 = TickContext(11, 2_020_000_000)
    sticky_world = _world(c2, other(0.99), locked(0.61))
    sticky = navigator.evaluate(
        _mission(c2, command_id=command_id),
        _estimate(c2),
        sticky_world,
    )
    assert sticky.status is NavigationStatus.ACTIVE
    assert len(sticky.route) == 1
    assert sticky.route[0].yaw_rad > 0.0

    c3 = TickContext(12, 2_040_000_000)
    lost_world = _world(c3, other(0.99))
    hold = navigator.evaluate(
        _mission(c3, command_id=command_id),
        _estimate(c3),
        lost_world,
    )
    hold_motion = MotionRealizer().evaluate(
        select_motion(hold),
        _estimate(c3),
        lost_world,
    )
    assert hold.status is NavigationStatus.ACTIVE
    assert hold_motion.requested_v_mps == 0.0
    assert hold_motion.requested_omega_rad_s == 0.0

    c4 = TickContext(
        13,
        c3.monotonic_ns + config.follow_person_lost_hold_ns + 1,
    )
    recovery_world = _world(c4, other(0.99))
    recovery = navigator.evaluate(
        _mission(c4, command_id=command_id),
        _estimate(c4),
        recovery_world,
    )
    recovery_motion = MotionRealizer().evaluate(
        select_motion(recovery),
        _estimate(c4),
        recovery_world,
    )
    assert recovery.status is NavigationStatus.ACTIVE
    assert recovery_motion.requested_v_mps == 0.0
    assert recovery_motion.requested_omega_rad_s > 0.0

    c5 = TickContext(
        14,
        c3.monotonic_ns + config.follow_person_lost_hold_ns
        + config.follow_person_search_max_duration_ns + 1,
    )
    expired = navigator.evaluate(
        _mission(c5, command_id=command_id),
        _estimate(c5),
        _world(c5, other(0.99)),
    )
    assert expired.status is NavigationStatus.INVALIDATED
    assert expired.reason == "PERSON_TARGET_LOST"

    c6 = TickContext(15, c5.monotonic_ns + 20_000_000)
    reacquired = navigator.evaluate(
        _mission(c6, command_id=command_id),
        _estimate(c6),
        _world(c6, locked(0.70)),
    )
    assert reacquired.status is NavigationStatus.ACTIVE
    assert len(reacquired.route) == 1
    assert reacquired.route[0].yaw_rad > 0.0


def test_follow_person_target_loss_is_fail_closed_but_reacquirable():
    navigator = TrajectoryNavigator(_production_navigation())
    command_id = "follow-loss"

    c1 = TickContext(20, 3_000_000_000)
    missing = navigator.evaluate(
        _mission(c1, command_id=command_id),
        _estimate(c1),
        _world(c1),
    )
    assert missing.status is NavigationStatus.INVALIDATED
    assert missing.reason == "PERSON_TARGET_NOT_AVAILABLE"

    c2 = TickContext(21, 3_020_000_000)
    reacquired = navigator.evaluate(
        _mission(c2, command_id=command_id),
        _estimate(c2),
        _world(c2, _person("person-7", 1.7, 0.0)),
    )
    assert reacquired.status is NavigationStatus.ACTIVE
    assert navigator.checkpoint().follow_person_track_id == "person-7"


def test_resident_follow_person_command_roundtrip(tmp_path: Path):
    path = tmp_path / "command.json"
    config = ResidentCommandMailboxConfig(path=path)
    client = ResidentCommandClient(config, monotonic_ns=lambda: 10_000_000_000)
    client.publish_follow_person(
        "follow-mailbox",
        max_v_mps=0.15,
        max_omega_rad_s=0.30,
        ttl_ns=200_000_000,
    )

    gateway = AtomicResidentCommandGateway(
        config,
        monotonic_ns=lambda: 10_000_000_000,
    )
    command = gateway.snapshot(TickContext(30, 10_000_000_000))
    assert command.mode is CommandMode.FOLLOW_PERSON
    assert command.goal == (
        DataField("max_v_mps", 0.15),
        DataField("max_omega_rad_s", 0.30),
    )


def test_control_cli_follow_person_writes_expected_mailbox(tmp_path, monkeypatch, capsys):
    runtime = tmp_path / "runtime"
    runtime.mkdir()
    status_path = runtime / "v3_status.json"
    status_path.write_text(
        json.dumps(
            {
                "schema": control_cli.RESIDENT_PROCESS_STATUS_SCHEMA,
                "state": "RUNNING",
                "ready_for_active": True,
            }
        ),
        encoding="utf-8",
    )
    status_path.chmod(0o600)
    monkeypatch.setattr(control_cli, "PROJECT_ROOT", tmp_path)

    def publish_once(_client, publish, *, command_id, **_kwargs):
        publish(command_id)
        return 0

    monkeypatch.setattr(control_cli, "_run_active", publish_once)
    assert control_cli.main(
        ["followperson", "--command-id", "follow-cli"]
    ) == 0
    capsys.readouterr()

    payload = json.loads(
        (runtime / "v3_command.json").read_text(encoding="utf-8")
    )
    assert payload["mode"] == "FOLLOW_PERSON"
    assert payload["command_id"] == "follow-cli"
    assert payload["max_v_mps"] == pytest.approx(0.15)
    assert payload["max_omega_rad_s"] == pytest.approx(0.30)


def test_operator_cli_routes_followperson_without_hardware(monkeypatch):
    calls = []

    class FakeController:
        def followperson(self, **kwargs):
            calls.append(kwargs)

    monkeypatch.setattr(
        operator_cli,
        "OperatorController",
        lambda **_kwargs: FakeController(),
    )
    assert operator_cli.main(["followperson", "c", "full"]) == 0
    assert calls == [
        {
            "max_v_mps": 0.15,
            "max_omega_rad_s": 0.30,
            "capture": True,
            "capture_mode": "full",
        }
    ]
