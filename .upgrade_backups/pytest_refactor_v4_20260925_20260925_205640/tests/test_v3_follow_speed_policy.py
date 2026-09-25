# R2B4_FOLLOW_SPEED_SMOOTH_P0_20260923
from __future__ import annotations
from v3_config_fixtures import configured

import inspect
import json
from pathlib import Path

import pytest

from v3.action_catalog import (
    FOLLOW_PERSON_DEFAULT_MAX_OMEGA_RAD_S,
    FOLLOW_PERSON_DEFAULT_MAX_V_MPS,
    action_descriptor,
)
from v3.adapters.v3_control import V3ControlInterfaceAdapter
from v3_config_fixtures import navigation_from_control as v3_navigation_config_from_mapping
from v3.contracts import (
    CommandMode,
    CommandRequest,
    DataField,
    MotionObjectiveKind,
    ObstacleTrack,
    RobotEstimate,
    RollingLocalCostmap,
    TickContext,
    WorldSnapshot,
)
from v3.control_cli import _parser as control_parser
from v3.interface_cli import _parser as interface_parser
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import TrajectoryNavigator
from v3.layers.l7_motion_selection import MotionSelector
from v3.layers.l8_motion_realization import MotionRealizationConfig
from v3.layers.l9_operational_constraints import OperationalConstraintsConfig
from v3.operator_controller import OperatorController


PROJECT_ROOT = (Path(__import__("os").environ["R2B4_ROOT"]).resolve() if __import__("os").environ.get("R2B4_ROOT") else next((p for p in Path(__file__).resolve().parents if (p / "conf" / "hardver.json").is_file() and (p / "v3").is_dir()), Path.cwd()))


def _navigation_config():
    raw = json.loads((PROJECT_ROOT / "conf/vezerles.json").read_text(encoding="utf-8"))
    return v3_navigation_config_from_mapping(raw).navigation


def _estimate(context: TickContext) -> RobotEstimate:
    covariance = tuple(0.01 if index % 6 == 0 else 0.0 for index in range(25))
    return RobotEstimate(
        context=context,
        frame_id="R2B4_BOOT_ROBOT_MAP",
        x_m=0.0,
        y_m=0.0,
        yaw_rad=0.0,
        v_mps=0.0,
        omega_rad_s=0.0,
        covariance_5x5=covariance,
    )


def _person(x_m: float) -> ObstacleTrack:
    return ObstacleTrack(
        track_id="person-1",
        x_m=x_m,
        y_m=0.0,
        radius_m=0.30,
        vx_mps=0.0,
        vy_mps=0.0,
        confidence=0.90,
    )


def _world(context: TickContext, x_m: float) -> WorldSnapshot:
    costmap = RollingLocalCostmap(
        frame_id="R2B4_BOOT_ROBOT_MAP",
        revision=1,
        resolution_m=0.10,
        radius_m=2.5,
        occupied_cells=(),
        source_sequence=1,
        freshness_ns=0,
    )
    return WorldSnapshot(
        context=context,
        frame_id="R2B4_BOOT_ROBOT_MAP",
        map_revision=1,
        obstacle_tracks=(_person(x_m),),
        freshness_ns=0,
        local_costmap=costmap,
    )


def _mission(context: TickContext):
    return configured(MissionManager, ).evaluate(
        CommandRequest(
            context=context,
            command_id="follow-speed-policy",
            mode=CommandMode.FOLLOW_PERSON,
            goal=(
                DataField("max_v_mps", FOLLOW_PERSON_DEFAULT_MAX_V_MPS),
                DataField(
                    "max_omega_rad_s",
                    FOLLOW_PERSON_DEFAULT_MAX_OMEGA_RAD_S,
                ),
            ),
            expiry_tick=context.tick_id,
        )
    )


def test_follow_default_has_one_canonical_value():
    assert FOLLOW_PERSON_DEFAULT_MAX_V_MPS == pytest.approx(0.25)
    assert FOLLOW_PERSON_DEFAULT_MAX_OMEGA_RAD_S == pytest.approx(0.30)

    descriptor = action_descriptor("v3.command.follow_person")
    assert descriptor is not None
    assert descriptor.parameter("max_v_mps").default == pytest.approx(0.25)
    assert descriptor.parameter("max_omega_rad_s").default == pytest.approx(0.30)


def test_all_human_and_machine_ingress_defaults_use_canonical_follow_speed():
    machine = control_parser().parse_args(["followperson"])
    human = interface_parser().parse_args(["followperson"])

    assert machine.max_v_mps == pytest.approx(FOLLOW_PERSON_DEFAULT_MAX_V_MPS)
    assert human.max_v == pytest.approx(FOLLOW_PERSON_DEFAULT_MAX_V_MPS)

    signature = inspect.signature(OperatorController.followperson)
    assert (
        signature.parameters["max_v_mps"].default
        == FOLLOW_PERSON_DEFAULT_MAX_V_MPS
    )


class _FakeController:
    def status(self):
        return {"runtime_running": False}

    def live_runtime_status(self):
        return None

    def followperson(self, **kwargs):
        return kwargs


def test_robot_interface_adapter_default_cannot_fall_back_to_old_015():
    adapter = V3ControlInterfaceAdapter(_FakeController())
    result = adapter.execute("v3.command.follow_person", capture=False)
    assert result["max_v_mps"] == pytest.approx(FOLLOW_PERSON_DEFAULT_MAX_V_MPS)
    assert result["max_omega_rad_s"] == pytest.approx(
        FOLLOW_PERSON_DEFAULT_MAX_OMEGA_RAD_S
    )


def test_nominal_downstream_static_caps_are_above_follow_default():
    # This guards accidental future static caps. Dynamic distance/heading,
    # localization, acceleration, obstacle and L12 safety limits remain valid.
    assert configured(MotionRealizationConfig, ).cruise_v_mps >= FOLLOW_PERSON_DEFAULT_MAX_V_MPS
    assert configured(OperationalConstraintsConfig, ).max_v_mps >= FOLLOW_PERSON_DEFAULT_MAX_V_MPS


def test_far_aligned_follow_exposes_full_025_planning_envelope():
    context = TickContext(1, 1_000_000_000)
    navigator = configured(TrajectoryNavigator, _navigation_config())
    plan = navigator.evaluate(
        _mission(context),
        _estimate(context),
        _world(context, 2.0),
    )

    assert plan.constraints.max_v_mps == pytest.approx(
        FOLLOW_PERSON_DEFAULT_MAX_V_MPS
    )
    if plan.trajectory_candidates:
        assert max(item.v_mps for item in plan.trajectory_candidates) == pytest.approx(
            FOLLOW_PERSON_DEFAULT_MAX_V_MPS
        )


def test_normal_distance_hold_is_active_arrival_not_stop_revocation():
    config = _navigation_config()
    context = TickContext(1, 2_000_000_000)
    navigator = configured(TrajectoryNavigator, config)
    plan = navigator.evaluate(
        _mission(context),
        _estimate(context),
        _world(context, config.follow_person_stand_off_m),
    )

    assert plan.status.value == "ACTIVE"
    assert plan.reason == "PERSON_DISTANCE_HOLD"
    assert plan.route
    objective = configured(MotionSelector, ).evaluate(plan)
    assert objective.kind is MotionObjectiveKind.TRACK_PLAN
