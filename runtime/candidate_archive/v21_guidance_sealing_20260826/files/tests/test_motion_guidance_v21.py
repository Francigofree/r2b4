"""T011/T012 tests for the sealed resolver-to-physical L7A boundary."""

from __future__ import annotations

import dataclasses

import pytest

from controller.motion_guidance import MotionGuidance
from controller.motion_guidance_contract import (
    GUIDANCE_HEADING_HOLD,
    GUIDANCE_TURN_TO_HEADING,
    GuidanceRequest,
    MOTION_INTENT_CONTRACT_ID,
    MotionGuidanceInput,
    PoseSnapshot,
    ResolvedMotionIntent,
    WorldModelSnapshot,
)
from controller.heading_turn_controller import HeadingTurnController
from controller.motion_platform_contract import (
    PHYSICAL_MODE_BODY_TWIST,
    PHYSICAL_MODE_STOP,
    PHYSICAL_MODE_WHEEL_VELOCITY,
    CycleContext,
    DriveCapabilities,
)
from controller.motion_semantics_engine import MotionSemanticsEngine


def _input(*, cycle_id: str = "guidance-17", now: float = 1.0) -> MotionGuidanceInput:
    cycle = CycleContext(
        cycle_id=cycle_id,
        monotonic_time=now,
        dt_observed_s=0.02,
        dt_control_s=0.02,
        timing_valid=True,
    )
    resolved = ResolvedMotionIntent(
        contract_id=MOTION_INTENT_CONTRACT_ID,
        resolved_id=f"resolved:{cycle_id}",
        cycle_id=cycle_id,
        selected_proposal_id=f"proposal:{cycle_id}",
        valid_until_monotonic=now + 0.10,
        nominal_mode=PHYSICAL_MODE_BODY_TWIST,
        v_mps=0.20,
        omega_rad_s=0.0,
        guidance_type=GUIDANCE_HEADING_HOLD,
    )
    return MotionGuidanceInput(
        resolved_intent=resolved,
        pose=PoseSnapshot(
            frame_id="R2B4_BOOT_ROBOT_MAP",
            pose_id=f"pose:{cycle_id}",
            source_timestamp=now,
            x_m=0.0,
            y_m=0.0,
            yaw_rad=0.0,
            v_mps=0.0,
            omega_rad_s=0.0,
            validity="VALID",
        ),
        world=WorldModelSnapshot(
            world_id=f"world:{cycle_id}",
            source_timestamp=now,
            validity="VALID",
            lidar_summary={"front_clearance_m": 2.0, "blocked_front": False},
        ),
        cycle_context=cycle,
        drive_capabilities=DriveCapabilities(
            track_width_m=0.20,
            calibrated_wheel_min_mps=0.15,
            calibrated_wheel_max_mps=0.58,
            max_wheel_accel_mps2=0.35,
            max_wheel_decel_mps2=0.80,
            capability_version="test-v21",
        ),
        actual_linear_mps=0.0,
        actual_angular_dps=0.0,
    )


def _guidance() -> MotionGuidance:
    return MotionGuidance(
        semantics=MotionSemanticsEngine(
            {"forward_heading_hold_enable": True}
        ),
        policy_config={"enabled": True},
    )


def test_t011_selected_intent_is_reduced_to_one_physical_command():
    guidance_input = _input()

    physical = _guidance().compute(guidance_input)

    assert physical.physical_mode == PHYSICAL_MODE_BODY_TWIST
    assert physical.cycle_id == guidance_input.cycle_context.cycle_id
    assert physical.resolved_id == guidance_input.resolved_intent.resolved_id
    assert physical.physical_command_id == f"physical:{physical.cycle_id}"
    assert dict(physical.trace_metadata) == {
        "selected_proposal_id": guidance_input.resolved_intent.selected_proposal_id,
        "guidance_type": GUIDANCE_HEADING_HOLD,
        "pose_id": guidance_input.pose.pose_id,
        "world_id": guidance_input.world.world_id,
    }


@pytest.mark.parametrize(
    "replacement",
    [
        lambda value: dataclasses.replace(
            value,
            pose=dataclasses.replace(value.pose, validity="INVALID"),
        ),
        lambda value: dataclasses.replace(
            value,
            cycle_context=dataclasses.replace(value.cycle_context, timing_valid=False),
        ),
        lambda value: dataclasses.replace(
            value,
            resolved_intent=dataclasses.replace(
                value.resolved_intent,
                valid_until_monotonic=value.cycle_context.monotonic_time - 0.01,
            ),
        ),
    ],
)
def test_t011_invalid_selected_intent_or_feedback_fails_closed(replacement):
    physical = _guidance().compute(replacement(_input()))

    assert physical.physical_mode == PHYSICAL_MODE_STOP
    assert (physical.v_mps, physical.omega_rad_s) == (0.0, 0.0)
    assert (physical.left_mps, physical.right_mps) == (0.0, 0.0)


def test_t012_lineage_stays_intact_into_the_physical_contract():
    guidance_input = _input(cycle_id="lineage")
    physical = _guidance().compute(guidance_input)

    assert physical.resolved_id == "resolved:lineage"
    assert physical.physical_command_id == "physical:lineage"
    assert physical.trace_metadata["selected_proposal_id"] == "proposal:lineage"


def test_t011_heading_turn_is_owned_and_reduced_inside_motion_guidance():
    base = _input(cycle_id="turn", now=4.0)
    turn_input = dataclasses.replace(
        base,
        resolved_intent=dataclasses.replace(
            base.resolved_intent,
            v_mps=0.0,
            guidance_type=GUIDANCE_TURN_TO_HEADING,
            guidance_request=GuidanceRequest(
                guidance_type=GUIDANCE_TURN_TO_HEADING,
                request_id="turn-request-1",
                target_heading_deg=90.0,
                speed_level=1,
            ),
        ),
        measured_left_mps=0.0,
        measured_right_mps=0.0,
        gyro_z_rad_s=0.0,
    )
    guidance = MotionGuidance(
        semantics=MotionSemanticsEngine(),
        heading_controller=HeadingTurnController(
            0.20,
            {"runtime_rotate_levels_autoload": False},
        ),
        policy_config={"enabled": True},
    )

    physical = guidance.compute(turn_input)

    assert physical.physical_mode == PHYSICAL_MODE_WHEEL_VELOCITY
    assert physical.resolved_id == "resolved:turn"
    assert guidance.heading_status()["owner"] == "MOTION_GUIDANCE_L7A"
    assert guidance.diagnostics().reason == "TURN_TO_HEADING_ACTIVE"


def test_t011_turn_request_missing_target_fails_closed():
    base = _input(cycle_id="turn-invalid", now=4.0)
    turn_input = dataclasses.replace(
        base,
        resolved_intent=dataclasses.replace(
            base.resolved_intent,
            v_mps=0.0,
            guidance_type=GUIDANCE_TURN_TO_HEADING,
            guidance_request=GuidanceRequest(
                guidance_type=GUIDANCE_TURN_TO_HEADING,
                request_id="turn-request-invalid",
            ),
        ),
    )

    guidance = _guidance()
    physical = guidance.compute(turn_input)

    assert physical.physical_mode == PHYSICAL_MODE_STOP
    assert guidance.diagnostics().reason == "TURN_GUIDANCE_TARGET_MISSING"
