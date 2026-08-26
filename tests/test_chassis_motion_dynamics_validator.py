import copy

import pytest

from tools import live_motion_measurement_validator as m1
from tools.chassis_motion_dynamics_validator import (
    CONTRACT_ID,
    PROFILE_NAME,
    analyze_m1_result,
)


def _case_row(case_name):
    metrics = {
        "case": case_name,
        "command": {
            "duration_s": 2.4,
            "chassis_dynamics_verdict": False,
            "caster_pair": "",
            "caster_orientation": "",
        },
        "encoder": {
            "average_delta_m": 0.50,
            "differential_delta_m": 0.10,
        },
        "ekf": {
            "forward_delta_m": 0.45,
            "yaw_delta_deg": 0.0,
        },
        "lidar": {
            "pose_chord_m": 0.43,
            "yaw_delta_deg": 0.0,
        },
        "imu": {"yaw_delta_deg": 0.0},
        "phase_tracking": {},
        "command_fidelity": {
            "errors": {},
            "transient": {},
        },
    }
    if case_name in ("forward", "backward"):
        metrics["imu"]["yaw_delta_deg"] = (
            1.5 if case_name == "forward" else -2.0
        )
    elif case_name in ("arc_left", "arc_right"):
        sign = 1.0 if case_name == "arc_left" else -1.0
        metrics["command"].update(
            {
                "caster_pair": f"pair_{case_name}",
                "caster_orientation": "uncontrolled_case_start",
            }
        )
        metrics["imu"]["yaw_delta_deg"] = sign * 25.0
        metrics["command_fidelity"]["errors"].update(
            {
                "imu_angular_speed_ratio_vs_executed": 0.90,
                "linear_speed_ratio_vs_executed": 1.00,
            }
        )
        metrics["phase_tracking"]["caster_influence"] = {
            "allowance_used": True,
            "settled_wheel_mae_mps": 0.020,
            "post_caster_wheel_mae_mps": 0.018,
            "case_limit_mps": 0.045,
        }
    elif case_name in ("rotate_left", "rotate_right"):
        sign = 1.0 if case_name == "rotate_left" else -1.0
        metrics["imu"]["yaw_delta_deg"] = sign * 44.0
        metrics["command_fidelity"]["errors"][
            "imu_angle_error_vs_requested_deg"
        ] = -1.0 * sign
        metrics["command_fidelity"]["transient"].update(
            {
                "pivot_settling_time_s": 1.0,
                "pivot_overshoot_deg": 1.0,
            }
        )
        metrics["command"]["duration_s"] = 8.0
    return {
        "case": case_name,
        "success": True,
        "failures": [],
        "metrics": metrics,
    }


def _source_m1():
    case_names = [case.name for case in m1.M1_CASES]
    return {
        "phase": "M1",
        "status": "PASS",
        "success": True,
        "cases_requested": case_names,
        "cases": [_case_row(case_name) for case_name in case_names],
        "m0_mini": {"ok": True},
        "validation_motion_contract": {"track_width_m": 0.3557},
        "m1_speed_map_execution_contract": {
            "contract_id": m1.M1_SPEED_MAP_EXECUTION_CONTRACT_ID,
            "required": True,
            "promotion_blocking": True,
            "chassis_dynamics_verdict": False,
            "delegated_validator": PROFILE_NAME,
        },
    }


def test_m2_passes_bounded_chassis_dynamics_and_is_nonblocking():
    result = analyze_m1_result(_source_m1())

    assert result["contract_id"] == CONTRACT_ID
    assert result["status"] == "PASS"
    assert result["speed_map_promotion_blocking"] is False
    assert result["promotion_contract"] == {
        "included_in_speed_map_decision": False,
        "may_block_speed_map_acceptance": False,
        "may_block_speed_map_promotion": False,
    }
    assert result["symmetry"]["status"] == "PASS"


def test_m2_fails_physical_understeer_without_blocking_speed_map_promotion():
    raw = _source_m1()
    arc_right = next(
        item for item in raw["cases"] if item["case"] == "arc_right"
    )
    arc_right["metrics"]["command_fidelity"]["errors"][
        "imu_angular_speed_ratio_vs_executed"
    ] = 0.7344
    arc_right["metrics"]["command_fidelity"]["errors"][
        "linear_speed_ratio_vs_executed"
    ] = 1.12

    result = analyze_m1_result(raw)

    assert result["status"] == "FAIL"
    assert (
        "arc_right:physical_angular_response_out_of_range"
        in result["failures"]
    )
    assert (
        "arc_right:physical_curvature_response_out_of_range"
        in result["failures"]
    )
    assert result["speed_map_promotion_blocking"] is False


@pytest.mark.parametrize(
    "mutation,expected_failure",
    [
        (
            lambda raw: raw["m1_speed_map_execution_contract"].update(
                contract_id="OLD"
            ),
            "source_m1_contract_invalid",
        ),
        (
            lambda raw: raw.update(status="FAIL", success=False),
            "source_m1_execution_not_pass",
        ),
        (
            lambda raw: raw["cases"].pop(),
            "source_m1_cases_incomplete",
        ),
    ],
)
def test_m2_fails_closed_for_invalid_source_m1(
    mutation,
    expected_failure,
):
    raw = copy.deepcopy(_source_m1())
    mutation(raw)

    result = analyze_m1_result(raw)

    assert result["status"] == "FAIL"
    assert expected_failure in result["failures"]
    assert result["speed_map_promotion_blocking"] is False


def test_m2_detects_ground_motion_and_pivot_symmetry_failures():
    raw = _source_m1()
    arc_left = next(
        item for item in raw["cases"] if item["case"] == "arc_left"
    )
    arc_left["metrics"]["ekf"]["forward_delta_m"] = 0.20
    rotate_right = next(
        item for item in raw["cases"] if item["case"] == "rotate_right"
    )
    rotate_right["metrics"]["imu"]["yaw_delta_deg"] = -32.0

    result = analyze_m1_result(raw)

    assert (
        "arc_left:ground_motion_ekf_vs_encoder_out_of_range"
        in result["failures"]
    )
    assert "pivot_left_right_difference_high" in result["failures"]


def test_m2_ground_motion_uses_shared_source_time_window():
    raw = _source_m1()
    arc_left = next(
        item for item in raw["cases"] if item["case"] == "arc_left"
    )
    metrics = arc_left["metrics"]
    metrics["encoder"]["average_delta_m"] = 0.50
    metrics["ekf"]["forward_delta_m"] = 0.20
    metrics["lidar"]["pose_chord_m"] = 0.18
    metrics["sensor_endpoint_shared_window"] = {
        "contract_id": m1.SENSOR_ENDPOINT_SHARED_WINDOW_CONTRACT_ID,
        "available": True,
        "encoder": {"average_delta_m": 0.25},
        "ekf_control": {"forward_delta_m": 0.225},
        "lidar": {"pose_chord_m": 0.21},
    }

    result = analyze_m1_result(raw)

    arc = next(item for item in result["arcs"] if item["case"] == "arc_left")
    assert arc["ground_motion_ratios"]["ekf_vs_encoder"] == pytest.approx(0.90)
    assert arc["ground_motion_ratios"]["lidar_chord_vs_encoder"] == pytest.approx(
        0.84
    )
    assert (
        "arc_left:ground_motion_ekf_vs_encoder_out_of_range"
        not in result["failures"]
    )
    assert (
        "arc_left:ground_motion_lidar_chord_vs_encoder_out_of_range"
        not in result["failures"]
    )


def test_m2_ground_motion_fails_closed_when_shared_window_is_unavailable():
    raw = _source_m1()
    arc_left = next(
        item for item in raw["cases"] if item["case"] == "arc_left"
    )
    arc_left["metrics"]["sensor_endpoint_shared_window"] = {
        "contract_id": m1.SENSOR_ENDPOINT_SHARED_WINDOW_CONTRACT_ID,
        "available": False,
        "failure_reason": "shared_lidar_encoder_interval_insufficient",
    }

    result = analyze_m1_result(raw)

    assert (
        "arc_left:ground_motion_ekf_vs_encoder_out_of_range"
        in result["failures"]
    )
    assert (
        "arc_left:ground_motion_lidar_chord_vs_encoder_out_of_range"
        in result["failures"]
    )
