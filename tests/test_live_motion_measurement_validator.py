import math
import unittest

from tools.live_motion_measurement_validator import (
    M0_CASES,
    M0_MINI_CASE_NAME,
    M0_MINI_CONTRACT_ID,
    M1_CASTER_ARC_ANGULAR_RATIO_MIN,
    M1_CASTER_CASE_CONTRACTS,
    M1_CASTER_INFLUENCE_CONTRACT_ID,
    M1_CASTER_ORIENTATION,
    M1_CASES,
    M1_CHASSIS_DYNAMICS_VALIDATOR,
    M1_SPEED_MAP_EXECUTION_CONTRACT_ID,
    SENSOR_ENDPOINT_SHARED_WINDOW_CONTRACT_ID,
    VALIDATION_SPEED_LEVEL,
    MeasurementCase,
    _canonical_encoder_count_window_identity,
    _case_failures,
    _case_retry_reason,
    _case_warnings,
    _direction_changes,
    _extract_lidar_status,
    _extract_runtime_diagnostics,
    _manual_reposition_suspected,
    _m1_caster_wheel_mae_limit_mps,
    _m0_mini_allows_continuation,
    _m0_mini_result,
    _measurement_ready_snapshot,
    _measured_linear_speed_abs,
    _repeatability_pose_angle_abs,
    _runtime_limit_state_matches,
    _sample_status,
    _shared_sensor_endpoint_window,
    _summarize_samples,
    _timing_gap_warning_summary,
    _total_variation,
    _integrated_pose_forward_delta_m,
    _unwrapped_yaw_delta_deg,
    _validation_motion_contract,
    build_parser,
)


class LiveMotionMeasurementValidatorTests(unittest.TestCase):
    def test_validation_level_exposes_full_active_wheel_range(self):
        self.assertEqual(VALIDATION_SPEED_LEVEL, 9)
        self.assertAlmostEqual(
            VALIDATION_SPEED_LEVEL / 9.0,
            1.0,
        )

    def test_runtime_limit_wait_requires_the_commanded_state_not_any_version(self):
        stale = {
            "state": "IDLE",
            "speed_level": 0,
            "v_target": 0.0,
            "omega_target": 0.0,
            "pwm": {"left": 0.0, "right": 0.0},
            "motion_state": {
                "gear_level": 0,
                "gear_ratio": 0.15,
                "v_max_active": 0.15,
            },
        }
        applied = {
            **stale,
            "speed_level": 9,
            "motion_state": {
                "gear_level": 9,
                "gear_ratio": 1.0,
                "v_max_active": 0.30,
            },
        }

        self.assertFalse(
            _runtime_limit_state_matches(
                stale,
                expected_level=9,
                expected_gear_ratio=1.0,
                expected_v_max_mps=0.30,
            )
        )
        self.assertTrue(
            _runtime_limit_state_matches(
                applied,
                expected_level=9,
                expected_gear_ratio=1.0,
                expected_v_max_mps=0.30,
            )
        )

    def test_lidar_sample_preserves_direction_gate_diagnostics_without_changing_gates(self):
        lidar = _extract_lidar_status(
            {
                "lidar_health": "OK",
                "lidar_enabled": True,
                "lidar_odom_status": {
                    "tracking_direction_checked": True,
                    "tracking_direction_consistent": False,
                    "tracking_direction_rejected": True,
                    "tracking_direction_rejected_total": 4,
                    "tracking_reference_delta_m": 0.04,
                    "tracking_reference_linear_mps": 0.22,
                    "tracking_candidate_projection_m": -0.05,
                    "tracking_backtrack_debt_m": 0.05,
                    "tracking_direction_reference_source": "encoder_canonical",
                    "tracking_direction_backtrack_tolerance_m": 0.03,
                },
            }
        )

        self.assertEqual(
            lidar["tracking_direction"],
            {
                "checked": True,
                "consistent": False,
                "rejected": True,
                "rejected_total": 4,
                "reference_delta_m": 0.04,
                "reference_linear_mps": 0.22,
                "candidate_projection_m": -0.05,
                "backtrack_debt_m": 0.05,
                "reference_source": "encoder_canonical",
                "backtrack_tolerance_m": 0.03,
            },
        )

    def test_lidar_sample_preserves_explicit_observation_lineage(self):
        lidar = _extract_lidar_status(
            {
                "lidar_health": "OK",
                "lidar_enabled": True,
                "lidar_odom_status": {
                    "raw_scan_id": 91,
                    "matcher_result_id": 101,
                    "candidate_id": 101,
                    "candidate_source_raw_scan_id": 91,
                    "lidar_odometry_measurement_id": 111,
                    "lidar_odometry_measurement_timestamp": 45.7,
                    "measurement_source_matcher_result_id": 101,
                    "measurement_source_raw_scan_id": 91,
                    "measurement_source_raw_scan_timestamp": 45.6,
                    "ekf_input_lidar_odometry_measurement_id": 111,
                    "ekf_last_processed_lidar_odometry_measurement_id": 111,
                    "ekf_last_applied_lidar_odometry_measurement_id": 111,
                    "latest_confidence": 0.8,
                    "applied": True,
                },
            }
        )

        observation = lidar["observation"]
        self.assertEqual(observation["raw_scan_id"], 91)
        self.assertEqual(observation["matcher_result_id"], 101)
        self.assertEqual(observation["lidar_odometry_measurement_id"], 111)
        self.assertAlmostEqual(
            observation["lidar_odometry_measurement_timestamp_s"], 45.7
        )
        self.assertEqual(observation["measurement_source_matcher_result_id"], 101)
        self.assertEqual(observation["measurement_source_raw_scan_id"], 91)
        self.assertAlmostEqual(
            observation["measurement_source_raw_scan_timestamp_s"], 45.6
        )
        self.assertEqual(observation["lineage_errors"], [])

    def test_shared_endpoint_uses_lidar_source_time_not_poll_endpoints(self):
        def sample(
            timestamp_s,
            pulses,
            measurement_id,
            source_timestamp_s,
            lidar_x,
        ):
            return {
                "ts": 1_700_000_000.0 + timestamp_s,
                "status_version": int(timestamp_s * 10),
                "state": "FORWARD",
                "pose": {
                    "x": float(pulses) * 0.0009,
                    "y": 0.0,
                    "theta": 0.0,
                    "theta_deg": 0.0,
                },
                "encoder": {
                    "measurement_timestamp_s": float(timestamp_s),
                    "left_pulses": int(pulses),
                    "right_pulses": int(pulses),
                    "left_distance_m": float(pulses) * 0.001,
                    "right_distance_m": float(pulses) * 0.001,
                    "step_distance_m": {"left": 0.001, "right": 0.001},
                },
                "imu": {},
                "lidar": {
                    "enabled": True,
                    "health": "OK",
                    "latest_age_s": 0.05,
                    "latest_confidence": 0.8,
                    "accepted": int(measurement_id),
                    "observation": {
                        "lidar_odometry_measurement_id": int(measurement_id),
                        "lidar_odometry_measurement_timestamp_s": (
                            float(source_timestamp_s) + 0.08
                        ),
                        "measurement_source_raw_scan_id": (
                            int(measurement_id) + 100
                        ),
                        "measurement_source_raw_scan_timestamp_s": float(
                            source_timestamp_s
                        ),
                        "measurement_source_matcher_result_id": (
                            int(measurement_id) + 200
                        ),
                        "lineage_errors": [],
                    },
                    "pose": {
                        "available": True,
                        "source": "lidar_odom_status.last_lidar_pose",
                        "x": float(lidar_x),
                        "y": 0.0,
                        "theta": 0.0,
                        "theta_deg": 0.0,
                    },
                },
                "pwm": {"left": 0.2, "right": 0.2},
                "safety": {"allow": True},
            }

        samples = [
            sample(100.0, 0, 9, 99.5, -0.08),
            sample(100.2, 100, 9, 99.5, -0.08),
            sample(100.4, 200, 10, 100.2, 0.0),
            sample(100.6, 300, 10, 100.2, 0.0),
            sample(100.8, 400, 11, 100.6, 0.16),
            sample(101.0, 500, 11, 100.6, 0.16),
            sample(101.2, 600, 11, 100.6, 0.16),
        ]
        case = MeasurementCase(
            "forward",
            "forward",
            0.15,
            0.0,
            3.0,
            expected_linear_sign=1,
        )

        metrics = _summarize_samples(
            case,
            samples,
            {"normal_stop_confirmed": True, "status": {}},
        )
        shared = metrics["sensor_endpoint_shared_window"]

        self.assertEqual(
            shared["contract_id"],
            SENSOR_ENDPOINT_SHARED_WINDOW_CONTRACT_ID,
        )
        self.assertTrue(shared["available"])
        self.assertAlmostEqual(shared["start_timestamp_s"], 100.2)
        self.assertAlmostEqual(shared["end_timestamp_s"], 100.6)
        self.assertEqual(
            shared["lidar"]["accepted_measurement_id_start"],
            10,
        )
        self.assertEqual(
            shared["lidar"]["accepted_measurement_id_end"],
            11,
        )
        self.assertAlmostEqual(
            shared["encoder"]["left_signed_pulse_delta"],
            200.0,
        )
        self.assertAlmostEqual(
            shared["encoder"]["average_delta_m"],
            0.20,
        )
        self.assertAlmostEqual(shared["lidar"]["pose_chord_m"], 0.16)
        self.assertAlmostEqual(
            shared["ekf_control"]["forward_delta_m"],
            0.18,
        )
        self.assertAlmostEqual(
            shared["ratios"]["lidar_chord_vs_encoder"],
            0.80,
        )
        self.assertAlmostEqual(metrics["encoder"]["average_delta_m"], 0.60)
        self.assertAlmostEqual(metrics["lidar"]["pose_chord_m"], 0.24)
        failures = _case_failures(metrics)
        self.assertNotIn(
            "encoder_lidar_endpoint_distance_mismatch",
            failures,
        )
        self.assertNotIn(
            "encoder_ekf_endpoint_distance_mismatch",
            failures,
        )

    def test_shared_endpoint_fails_closed_without_source_timestamps(self):
        samples = [
            {
                "encoder": {
                    "measurement_timestamp_s": float(timestamp_s),
                    "left_pulses": pulses,
                    "right_pulses": pulses,
                    "left_distance_m": pulses * 0.001,
                    "right_distance_m": pulses * 0.001,
                    "step_distance_m": {"left": 0.001, "right": 0.001},
                },
                "pose": {
                    "x": pulses * 0.001,
                    "y": 0.0,
                    "theta_deg": 0.0,
                },
                "lidar": {
                    "pose": {
                        "available": True,
                        "x": pulses * 0.001,
                        "y": 0.0,
                    },
                    "observation": {
                        "lidar_odometry_measurement_id": index + 1,
                    },
                },
            }
            for index, (timestamp_s, pulses) in enumerate(
                ((10.0, 0), (10.1, 10), (10.2, 20))
            )
        ]

        shared = _shared_sensor_endpoint_window(samples)

        self.assertFalse(shared["available"])
        self.assertEqual(
            shared["failure_reason"],
            "shared_lidar_encoder_interval_insufficient",
        )

    def test_lidar_summary_counts_unique_measurements_not_poll_repeats(self):
        case = MeasurementCase("idle", "idle", 0.0, 0.0, 1.0)

        def sample(measurement_id, confidence, raw_scan_id, matcher_result_id):
            return _sample_status(
                "idle",
                {
                    "status_version": raw_scan_id,
                    "lidar_health": "OK",
                    "lidar_enabled": True,
                    "lidar_odom_status": {
                        "raw_scan_id": raw_scan_id,
                        "matcher_result_id": matcher_result_id,
                        "candidate_id": matcher_result_id,
                        "candidate_source_raw_scan_id": raw_scan_id,
                        "lidar_odometry_measurement_id": measurement_id,
                        "measurement_source_matcher_result_id": matcher_result_id,
                        "measurement_source_raw_scan_id": raw_scan_id,
                        "ekf_input_lidar_odometry_measurement_id": measurement_id,
                        "ekf_last_processed_lidar_odometry_measurement_id": measurement_id,
                        "ekf_last_applied_lidar_odometry_measurement_id": measurement_id,
                        "latest_age_s": 0.05,
                        "latest_confidence": confidence,
                        "accepted": measurement_id,
                        "applied": True,
                    },
                },
            )

        repeated = sample(11, 0.9, 91, 101)
        samples = [repeated, dict(repeated), dict(repeated), sample(12, 0.3, 92, 102)]
        metrics = _summarize_samples(
            case,
            samples,
            {"normal_stop_confirmed": True, "status": {}},
        )

        lidar = metrics["lidar"]
        self.assertEqual(lidar["applied_status_samples"], 4)
        self.assertEqual(lidar["applied_samples"], 2)
        self.assertEqual(lidar["unique_lidar_odometry_measurements"], 2)
        self.assertEqual(lidar["unique_raw_scan_observations"], 2)
        self.assertAlmostEqual(lidar["latest_confidence_poll_median"], 0.9)
        self.assertAlmostEqual(lidar["latest_confidence_median"], 0.6)
        self.assertEqual(lidar["observation_contract_errors"], [])

    def test_lidar_observation_contract_violation_is_a_validator_failure(self):
        metrics = self._base_metrics()
        metrics["lidar"]["observation_contract_errors"] = [
            "applied_measurement_id_missing"
        ]
        metrics["lidar"]["applied_missing_measurement_id_samples"] = 1

        self.assertIn("lidar_observation_contract_violation", _case_failures(metrics))

    def test_live_profile_defaults_to_unified_mode(self):
        args = build_parser().parse_args([])
        self.assertEqual(args.control_mode, "UNIFIED")

    def test_repeatability_angle_uses_ekf_pose_ssot_not_gyro_window(self):
        metrics = {
            "ekf": {"yaw_delta_deg": -43.5},
            "imu": {"yaw_delta_deg": -51.0},
        }

        self.assertAlmostEqual(_repeatability_pose_angle_abs(metrics), 43.5)

    def test_live_measurement_cases_raise_speed_without_increasing_path_length(self):
        m0 = {case.name: case for case in M0_CASES}
        m1 = {case.name: case for case in M1_CASES}

        self.assertAlmostEqual(m0["trust_forward_pulse"].v_mps, 0.15)
        self.assertAlmostEqual(m0["trust_forward_pulse"].v_mps * m0["trust_forward_pulse"].duration_s, 0.324)
        self.assertEqual(M1_CASES[0].name, M0_MINI_CASE_NAME)
        self.assertAlmostEqual(m1[M0_MINI_CASE_NAME].v_mps, 0.15)
        self.assertAlmostEqual(
            m1[M0_MINI_CASE_NAME].v_mps * m1[M0_MINI_CASE_NAME].duration_s,
            0.324,
        )
        self.assertTrue(m1[M0_MINI_CASE_NAME].quality_gate)
        self.assertEqual(
            m1[M0_MINI_CASE_NAME].caster_orientation,
            "uncontrolled_initial",
        )
        self.assertAlmostEqual(
            m1[M0_MINI_CASE_NAME].caster_transient_s,
            1.0,
        )
        self.assertAlmostEqual(m1["forward"].v_mps * m1["forward"].duration_s, 0.54)
        self.assertAlmostEqual(m1["backward"].v_mps * m1["backward"].duration_s, -0.486)
        for case_name, pair in M1_CASTER_CASE_CONTRACTS.items():
            self.assertEqual(m1[case_name].caster_pair, pair)
            self.assertEqual(
                m1[case_name].caster_orientation,
                M1_CASTER_ORIENTATION,
            )
            self.assertAlmostEqual(m1[case_name].caster_transient_s, 1.0)
        self.assertAlmostEqual(m0["trust_arc_left"].v_mps, 0.225)
        self.assertAlmostEqual(m0["trust_arc_left"].v_mps * m0["trust_arc_left"].duration_s, 0.324)
        self.assertAlmostEqual(m1["arc_left"].v_mps, 0.225)
        self.assertAlmostEqual(m1["arc_left"].v_mps * m1["arc_left"].duration_s, 0.54)
        self.assertEqual(m1["rotate_left"].heading_speed_level, 1)
        self.assertEqual(m1["rotate_right"].heading_speed_level, 1)

    def test_m0_mini_pass_publishes_full_m0_equivalence_contract(self):
        warning = {
            "code": "encoder_timing_gap",
            "severity": "WARNING",
            "count": 1,
        }
        result = _m0_mini_result(
            {
                "case": M0_MINI_CASE_NAME,
                "success": True,
                "failures": [],
                "warnings": [warning],
                "metrics": self._base_metrics(),
            }
        )

        self.assertTrue(result["success"])
        self.assertEqual(result["phase"], "M0_MINI")
        self.assertEqual(result["contract_id"], M0_MINI_CONTRACT_ID)
        self.assertTrue(result["measurement_trust"]["equivalent_to_full_m0"])
        self.assertTrue(_m0_mini_allows_continuation(result))
        self.assertEqual(result["warnings"], [warning])
        self.assertEqual(result["measurement_trust"]["warnings"], [warning])
        self.assertEqual(
            result["measurement_trust"]["sensor_surface"],
            {
                "encoder_cases": 1,
                "imu_cases": 1,
                "lidar_cases": 1,
                "ekf_cases": 1,
                "motor_pwm_cases": 1,
            },
        )

    def test_m0_mini_quality_failure_is_not_equivalent_to_full_m0(self):
        result = _m0_mini_result(
            {
                "case": M0_MINI_CASE_NAME,
                "success": False,
                "failures": ["settled_wheel_speed_tracking_error_high"],
                "metrics": self._base_metrics(),
            }
        )

        self.assertFalse(result["success"])
        self.assertFalse(result["measurement_trust"]["equivalent_to_full_m0"])
        self.assertIn("settled_wheel_speed_tracking_error_high", result["failures"])
        self.assertFalse(_m0_mini_allows_continuation(result))

    def test_m0_mini_contract_drift_blocks_m1_continuation(self):
        result = _m0_mini_result(
            {
                "case": M0_MINI_CASE_NAME,
                "success": True,
                "failures": [],
                "metrics": self._base_metrics(),
            }
        )
        result["contract_id"] = "WRONG"

        self.assertFalse(_m0_mini_allows_continuation(result))

    def test_live_arc_cases_fit_kit0085_active_wheel_speed_range(self):
        contract = _validation_motion_contract(
            list(M0_CASES) + list(M1_CASES),
            track_width_m=0.3557,
            wheel_min_mps=0.15,
            wheel_max_mps=0.30,
        )

        self.assertTrue(contract["ok"])
        arc_rows = [row for row in contract["cases"] if "arc" in row["case"]]
        self.assertEqual(len(arc_rows), 4)
        for row in arc_rows:
            self.assertGreaterEqual(min(abs(row["left_mps"]), abs(row["right_mps"])), 0.15)
            self.assertLessEqual(max(abs(row["left_mps"]), abs(row["right_mps"])), 0.30)

    def test_transient_speed_uses_persisted_canonical_count_window_feedback(self):
        sample = {
            "raw_velocity": {"left_mps": 0.0, "right_mps": 0.52},
            "encoder": {
                "canonical_velocity": {"left_mps": 0.17, "right_mps": 0.19}
            },
            "diagnostics": {
                "executor": {
                    "wheel_loop_left_meas_mps": 0.01,
                    "wheel_loop_right_meas_mps": 0.02,
                }
            },
        }

        self.assertAlmostEqual(_measured_linear_speed_abs(sample), 0.18)

    def test_transient_speed_does_not_fall_back_when_canonical_feedback_missing(self):
        sample = {"raw_velocity": {"left_mps": -0.16, "right_mps": -0.20}}

        self.assertTrue(math.isnan(_measured_linear_speed_abs(sample)))

    def test_phase_tracking_retains_full_error_but_gates_fixed_settled_window(self):
        case = MeasurementCase(
            "phase_contract",
            "forward",
            0.15,
            0.0,
            1.0,
            expected_linear_sign=1,
        )
        samples = []
        for ts, active, error in (
            (0.00, False, 0.0),
            (0.10, True, 0.050),
            (0.20, True, 0.050),
            (0.31, True, 0.050),
            (0.41, True, 0.005),
            (0.50, True, 0.005),
        ):
            samples.append(
                {
                    "ts": ts,
                    "pwm": {"left": 0.20 if active else 0.0, "right": 0.20 if active else 0.0},
                    "raw_velocity": {"left_mps": 0.15 if active else 0.0, "right_mps": 0.15 if active else 0.0},
                    "diagnostics": {
                        "executor": {
                            "v_cmd": 0.15 if active else 0.0,
                            "omega_cmd": 0.0,
                            "wheel_loop_left_ref_mps": 0.15 if active else 0.0,
                            "wheel_loop_right_ref_mps": 0.15 if active else 0.0,
                            "wheel_loop_left_meas_mps": (
                                0.145 if active and error < 0.01 else 0.10 if active else 0.0
                            ),
                            "wheel_loop_right_meas_mps": (
                                0.145 if active and error < 0.01 else 0.10 if active else 0.0
                            ),
                            "wheel_loop_left_error_mps": error,
                            "wheel_loop_right_error_mps": error,
                        }
                    },
                    "motion_command": {"command_type": "set_twist", "requested_v": 0.15},
                    "pose": {"x": ts * 0.15, "y": 0.0, "theta_deg": 0.0},
                    "encoder": {
                        "left_distance_m": ts * 0.15,
                        "right_distance_m": ts * 0.15,
                        "left_pulses": int(ts * 100),
                        "right_pulses": int(ts * 100),
                        "canonical_velocity": {
                            "left_mps": (
                                0.145 if active and error < 0.01 else 0.10 if active else 0.0
                            ),
                            "right_mps": (
                                0.145 if active and error < 0.01 else 0.10 if active else 0.0
                            ),
                        },
                    },
                    "imu": {"available": True, "yaw_deg": 0.0, "gyro_z_dps": 0.0},
                    "lidar": {},
                    "safety": {"allow": True},
                }
            )

        metrics = _summarize_samples(
            case,
            samples,
            {"normal_stop_confirmed": True, "status": {}},
        )

        errors = metrics["command_fidelity"]["errors"]
        self.assertGreater(errors["wheel_speed_tracking_mae_mps"], 0.015)
        self.assertAlmostEqual(errors["settled_wheel_speed_tracking_mae_mps"], 0.005)
        executed_distance = metrics["command_fidelity"]["executed"]["distance_m"]
        encoder_distance = metrics["command_fidelity"]["actual"]["encoder_distance_m"]
        self.assertAlmostEqual(
            errors["whole_phase_linear_speed_ratio_vs_executed"],
            encoder_distance / executed_distance,
        )
        self.assertEqual(metrics["phase_tracking"]["settled"]["sample_count"], 2)
        self.assertEqual(metrics["phase_tracking"]["settled"]["independent_feedback_windows"], 1)
        self.assertEqual(
            metrics["phase_tracking"]["contract"]["settled_start_after_first_pwm_s"],
            0.30,
        )

    def test_phase_tracking_counts_each_canonical_count_window_once(self):
        case = MeasurementCase(
            "count_window_identity",
            "forward",
            0.15,
            0.0,
            1.0,
            expected_linear_sign=1,
        )

        def sample(ts, *, active, reference, window):
            measured = 0.10 if active else 0.0
            return {
                "ts": ts,
                "pwm": {
                    "left": 0.20 if active else 0.0,
                    "right": 0.20 if active else 0.0,
                },
                "raw_velocity": {"left_mps": measured, "right_mps": measured},
                "diagnostics": {
                    "executor": {
                        "v_cmd": reference if active else 0.0,
                        "omega_cmd": 0.0,
                        "wheel_loop_left_ref_mps": reference if active else 0.0,
                        "wheel_loop_right_ref_mps": reference if active else 0.0,
                    }
                },
                "motion_command": {
                    "command_type": "set_twist",
                    "requested_v": reference if active else 0.0,
                },
                "pose": {"x": ts * measured, "y": 0.0, "theta_deg": 0.0},
                "encoder": {
                    "left_distance_m": ts * measured,
                    "right_distance_m": ts * measured,
                    "left_pulses": int(ts * 100),
                    "right_pulses": int(ts * 100),
                    "canonical_velocity": {
                        "left_mps": measured,
                        "right_mps": measured,
                    },
                    "canonical_pulses_delta": dict(window or {}),
                },
                "imu": {"available": True, "yaw_deg": 0.0, "gyro_z_dps": 0.0},
                "lidar": {},
                "safety": {"allow": True},
            }

        window_a = {
            "window_start_ts": 10.0,
            "window_end_ts": 10.1,
            "left_count_start": 100,
            "left_count_end": 116,
            "right_count_start": 200,
            "right_count_end": 216,
        }
        window_b = {
            "window_start_ts": 10.1,
            "window_end_ts": 10.2,
            "left_count_start": 116,
            "left_count_end": 132,
            "right_count_start": 216,
            "right_count_end": 232,
        }
        samples = [
            sample(0.00, active=False, reference=0.0, window=None),
            sample(0.10, active=True, reference=0.15, window=window_a),
            sample(0.41, active=True, reference=0.15, window=window_a),
            sample(0.51, active=True, reference=0.15, window=window_b),
            # A late/repeated A with a changed reference is still one window.
            sample(0.61, active=True, reference=0.20, window=window_a),
        ]

        metrics = _summarize_samples(
            case,
            samples,
            {"normal_stop_confirmed": True, "status": {}},
        )
        settled = metrics["phase_tracking"]["settled"]
        self.assertEqual(settled["sample_count"], 3)
        self.assertEqual(settled["independent_feedback_windows"], 2)
        self.assertEqual(settled["wheel_error_sample_count"], 4)
        self.assertAlmostEqual(settled["wheel_speed_tracking_mae_mps"], 0.05)
        self.assertAlmostEqual(
            settled["poll_weighted_wheel_speed_tracking_mae_mps"],
            0.0666666667,
        )

    def test_caster_transient_metrics_split_at_exact_case_window(self):
        case = MeasurementCase(
            "caster_window",
            "forward",
            0.15,
            0.0,
            1.5,
            expected_linear_sign=1,
            caster_pair="forward",
            caster_orientation="reversed_180",
            caster_transient_s=1.0,
        )
        samples = []
        for index, (ts, active, measured) in enumerate(
            (
                (0.0, False, 0.0),
                (0.1, True, 0.08),
                (0.4, True, 0.12),
                (0.8, True, 0.14),
                (1.1, True, 0.145),
                (1.4, True, 0.145),
            )
        ):
            samples.append(
                {
                    "ts": ts,
                    "pwm": {
                        "left": 0.20 if active else 0.0,
                        "right": 0.20 if active else 0.0,
                    },
                    "raw_velocity": {
                        "left_mps": measured,
                        "right_mps": measured,
                    },
                    "diagnostics": {
                        "executor": {
                            "v_cmd": 0.15 if active else 0.0,
                            "omega_cmd": 0.0,
                            "wheel_loop_left_ref_mps": 0.15 if active else 0.0,
                            "wheel_loop_right_ref_mps": 0.15 if active else 0.0,
                        }
                    },
                    "motion_command": {
                        "command_type": "set_twist",
                        "requested_v": 0.15,
                    },
                    "pose": {
                        "x": ts * measured,
                        "y": 0.0,
                        "theta_deg": 0.0,
                    },
                    "encoder": {
                        "left_distance_m": ts * measured,
                        "right_distance_m": ts * measured,
                        "left_pulses": index * 10,
                        "right_pulses": index * 10,
                        "canonical_velocity": {
                            "left_mps": measured,
                            "right_mps": measured,
                        },
                        "canonical_pulses_delta": {
                            "window_start_ts": ts - 0.1,
                            "window_end_ts": ts,
                            "left_count_start": index * 10,
                            "left_count_end": index * 10 + 1,
                            "right_count_start": index * 10,
                            "right_count_end": index * 10 + 1,
                        },
                    },
                    "imu": {
                        "available": True,
                        "yaw_deg": 0.0,
                        "gyro_z_dps": 0.0,
                    },
                    "lidar": {},
                    "safety": {"allow": True},
                }
            )

        metrics = _summarize_samples(
            case,
            samples,
            {"normal_stop_confirmed": True, "status": {}},
        )

        tracking = metrics["phase_tracking"]
        self.assertEqual(
            tracking["contract"]["caster_transient_after_first_pwm_s"],
            1.0,
        )
        self.assertEqual(
            tracking["caster_transient"]["independent_feedback_windows"],
            3,
        )
        self.assertEqual(
            tracking["post_caster_transient"]["independent_feedback_windows"],
            2,
        )
        self.assertGreater(
            tracking["caster_transient"]["wheel_speed_tracking_mae_mps"],
            tracking["post_caster_transient"]["wheel_speed_tracking_mae_mps"],
        )

    def test_phase_tracking_does_not_invent_identity_from_partial_window(self):
        pulses = {
            "window_end_ts": 10.1,
            "left_count_end": 116,
            "right_count_end": 216,
        }
        self.assertIsNone(_canonical_encoder_count_window_identity(pulses))

    def _base_metrics(self):
        return {
            "command": {
                "command_motion": True,
                "expected_linear_sign": 1,
                "expected_yaw_sign": 0,
                "command_type": "set_twist",
                "duration_s": 2.0,
                "command_types_seen": {"set_twist": 8},
            },
            "samples": {
                "count": 8,
                "status_progressed": True,
            },
            "motor_pwm": {
                "max_abs_left": 0.21,
                "max_abs_right": 0.22,
                "active_side_samples": 16,
                "near_stable_floor_ratio": 0.0,
                "below_stable_floor_ratio": 0.0,
            },
            "encoder": {
                "available": True,
                "average_delta_m": 0.026,
                "differential_delta_m": 0.02,
            },
            "imu": {
                "available": True,
                "yaw_delta_deg": 0.5,
            },
            "ekf": {
                "forward_delta_m": 0.024,
                "yaw_delta_deg": 0.4,
            },
            "lidar": {
                "enabled_seen": True,
                "health_values": ["OK"],
                "latest_age_s_max": 0.22,
                "latest_confidence_median": 0.78,
                "yaw_delta_deg": 0.4,
                "pose_chord_m": 0.025,
            },
            "safety": {
                "failsafe_seen": False,
                "safety_block_seen": False,
                "normal_stop_confirmed": True,
            },
            "stop_start": {
                "stop_start_suspect": False,
            },
            "command_fidelity": {
                "executed": {
                    "distance_m": 0.10,
                },
                "errors": {
                    "executed_linear_ratio_vs_requested": 1.0,
                    "linear_speed_ratio_vs_executed": 1.0,
                    "whole_phase_linear_speed_ratio_vs_executed": 1.0,
                    "encoder_distance_error_vs_executed_m": 0.005,
                    "wheel_speed_tracking_mae_mps": 0.005,
                    "settled_wheel_speed_tracking_mae_mps": 0.005,
                    "endpoint_yaw_spread_deg": 1.0,
                },
                "transient": {
                    "settling_time_s": 0.5,
                    "linear_speed_overshoot_ratio": 1.1,
                },
            },
        }

    def _m0_mini_caster_metrics(
        self,
        *,
        full_mae=0.020,
        post_mae=0.010,
        settling_time_s=0.8,
        transient_windows=5,
        post_windows=6,
    ):
        metrics = self._base_metrics()
        metrics["case"] = M0_MINI_CASE_NAME
        metrics["command"].update(
            {
                "caster_pair": "m0_mini_first_forward",
                "caster_orientation": "uncontrolled_initial",
                "caster_transient_s": 1.0,
            }
        )
        metrics["phase_tracking"] = {
            "caster_transient": {
                "wheel_speed_tracking_mae_mps": 0.025,
                "independent_feedback_windows": transient_windows,
            },
            "post_caster_transient": {
                "wheel_speed_tracking_mae_mps": post_mae,
                "independent_feedback_windows": post_windows,
            },
        }
        metrics["command_fidelity"]["errors"][
            "settled_wheel_speed_tracking_mae_mps"
        ] = full_mae
        metrics["command_fidelity"]["transient"][
            "settling_time_s"
        ] = settling_time_s
        return metrics

    def test_forward_measurement_case_accepts_consistent_surfaces(self):
        self.assertEqual(_case_failures(self._base_metrics()), [])

    def test_m0_mini_allows_bounded_caster_transient_but_keeps_post_gate(self):
        metrics = self._m0_mini_caster_metrics(
            full_mae=0.020,
            post_mae=0.010,
        )

        failures = _case_failures(metrics)

        self.assertNotIn("settled_wheel_speed_tracking_error_high", failures)
        self.assertNotIn("post_caster_wheel_speed_tracking_error_high", failures)
        self.assertNotIn("m0_mini_full_settled_wheel_speed_unbounded", failures)

    def test_m0_mini_rejects_post_caster_tracking_error(self):
        metrics = self._m0_mini_caster_metrics(post_mae=0.016)

        failures = _case_failures(metrics)

        self.assertIn("post_caster_wheel_speed_tracking_error_high", failures)

    def test_m0_mini_rejects_unbounded_or_unproven_caster_transient(self):
        metrics = self._m0_mini_caster_metrics(
            full_mae=0.031,
            settling_time_s=1.01,
            transient_windows=2,
            post_windows=2,
        )

        failures = _case_failures(metrics)

        self.assertIn("m0_mini_full_settled_wheel_speed_unbounded", failures)
        self.assertIn("m0_mini_caster_reorientation_not_settled_by_1s", failures)
        self.assertIn("m0_mini_caster_transient_feedback_windows_low", failures)
        self.assertIn("m0_mini_post_caster_feedback_windows_low", failures)

    def test_m0_mini_rejects_missing_caster_contract_identity(self):
        metrics = self._m0_mini_caster_metrics()
        metrics["command"]["caster_orientation"] = ""

        failures = _case_failures(metrics)

        self.assertIn("m0_mini_caster_transient_contract_mismatch", failures)

    def test_non_m0_linear_case_keeps_original_settled_tracking_gate(self):
        metrics = self._base_metrics()
        metrics["case"] = "generic_forward"
        metrics["command_fidelity"]["errors"][
            "settled_wheel_speed_tracking_mae_mps"
        ] = 0.016

        failures = _case_failures(metrics)

        self.assertIn("settled_wheel_speed_tracking_error_high", failures)

    def _m1_caster_metrics(
        self,
        *,
        case_name="backward",
        settled_mae=0.025,
        post_mae=0.025,
        transient_windows=6,
        post_windows=8,
        angular_ratio=None,
    ):
        case = {item.name: item for item in M1_CASES}[case_name]
        metrics = self._base_metrics()
        metrics["case"] = case_name
        metrics["command"].update(
            {
                "v_mps": case.v_mps,
                "expected_linear_sign": case.expected_linear_sign,
                "expected_yaw_sign": case.expected_yaw_sign,
                "caster_pair": case.caster_pair,
                "caster_orientation": case.caster_orientation,
                "caster_transient_s": case.caster_transient_s,
            }
        )
        metrics["phase_tracking"] = {
            "caster_transient": {
                "independent_feedback_windows": transient_windows,
            },
            "post_caster_transient": {
                "wheel_speed_tracking_mae_mps": post_mae,
                "independent_feedback_windows": post_windows,
            },
        }
        metrics["command_fidelity"]["errors"][
            "settled_wheel_speed_tracking_mae_mps"
        ] = settled_mae
        if angular_ratio is not None:
            metrics["command_fidelity"]["errors"].update(
                {
                    "executed_angular_ratio_vs_requested": 1.0,
                    "imu_angular_speed_ratio_vs_executed": angular_ratio,
                }
            )
        return metrics

    def test_m1_caster_contract_has_bounded_speed_scaled_allowance(self):
        self.assertEqual(
            M1_CASTER_INFLUENCE_CONTRACT_ID,
            "R2B4_M1_PASSIVE_FRONT_CASTER_V1",
        )
        self.assertAlmostEqual(_m1_caster_wheel_mae_limit_mps(0.15), 0.030)
        self.assertAlmostEqual(_m1_caster_wheel_mae_limit_mps(0.225), 0.045)
        self.assertAlmostEqual(_m1_caster_wheel_mae_limit_mps(1.0), 0.050)

        failures = _case_failures(
            self._m1_caster_metrics(
                settled_mae=0.025,
                post_mae=0.020,
            )
        )

        self.assertNotIn(
            "settled_wheel_speed_tracking_error_high",
            failures,
        )
        self.assertNotIn(
            "m1_caster_settled_wheel_speed_tracking_error_high",
            failures,
        )

    def test_m1_speed_map_contract_delegates_chassis_dynamics_only(self):
        self.assertEqual(
            M1_SPEED_MAP_EXECUTION_CONTRACT_ID,
            "R2B4_M1_SPEED_MAP_EXECUTION_V1",
        )
        self.assertEqual(
            M1_CHASSIS_DYNAMICS_VALIDATOR,
            "M2_chassis_motion_dynamics_live",
        )
        cases = {item.name: item for item in M1_CASES}
        self.assertTrue(cases[M0_MINI_CASE_NAME].chassis_dynamics_verdict)
        for case_name in (
            "forward",
            "backward",
            "arc_left",
            "arc_right",
            "rotate_left",
            "rotate_right",
            "stop_hold",
        ):
            self.assertFalse(cases[case_name].chassis_dynamics_verdict)

    def test_m1_caster_contract_rejects_unbounded_or_unproven_error(self):
        metrics = self._m1_caster_metrics(
            settled_mae=0.031,
            post_mae=0.031,
            transient_windows=2,
            post_windows=2,
        )

        failures = _case_failures(metrics)

        self.assertIn(
            "m1_caster_settled_wheel_speed_tracking_error_high",
            failures,
        )
        self.assertIn(
            "m1_post_caster_wheel_speed_tracking_error_high",
            failures,
        )
        self.assertIn("m1_caster_transient_feedback_windows_low", failures)
        self.assertIn("m1_post_caster_feedback_windows_low", failures)

    def test_m1_caster_contract_identity_is_fail_closed(self):
        metrics = self._m1_caster_metrics()
        metrics["command"]["caster_pair"] = "wrong"

        failures = _case_failures(metrics)

        self.assertIn("m1_caster_influence_contract_mismatch", failures)

    def test_m1_caster_arc_allows_bounded_physical_understeer(self):
        metrics = self._m1_caster_metrics(
            case_name="arc_right",
            settled_mae=0.038,
            post_mae=0.034,
            angular_ratio=0.77,
        )

        failures = _case_failures(metrics)

        self.assertNotIn("arc_angular_speed_error_high", failures)
        self.assertNotIn(
            "m1_caster_settled_wheel_speed_tracking_error_high",
            failures,
        )

        metrics["command_fidelity"]["errors"][
            "imu_angular_speed_ratio_vs_executed"
        ] = M1_CASTER_ARC_ANGULAR_RATIO_MIN - 0.001
        self.assertIn("arc_angular_speed_error_high", _case_failures(metrics))
        metrics["command"]["chassis_dynamics_verdict"] = False
        self.assertNotIn(
            "arc_angular_speed_error_high",
            _case_failures(metrics),
        )

    def test_trust_remeasurement_retries_invalid_case_without_accepting_it(self):
        result = {
            "success": False,
            "failures": ["encoder_lidar_endpoint_distance_mismatch"],
            "metrics": {"safety": {}},
        }

        self.assertEqual(
            _case_retry_reason(
                result,
                retry_all_measurement_failures=True,
            ),
            "measurement_gate_failure",
        )
        self.assertEqual(
            _case_retry_reason(
                result,
                retry_all_measurement_failures=False,
            ),
            "",
        )

    def test_safety_intervention_is_retryable_but_success_is_not(self):
        invalid = {
            "success": False,
            "failures": ["failsafe_seen"],
            "metrics": {
                "safety": {
                    "failsafe_seen": True,
                    "safety_block_seen": False,
                    "emergency_count_delta": 1,
                }
            },
        }
        accepted = {"success": True, "failures": [], "metrics": {}}

        self.assertEqual(
            _case_retry_reason(
                invalid,
                retry_all_measurement_failures=False,
            ),
            "safety_intervention",
        )
        self.assertEqual(
            _case_retry_reason(
                accepted,
                retry_all_measurement_failures=True,
            ),
            "",
        )

    def test_encoder_timing_gap_is_warning_when_quality_gates_pass(self):
        metrics = self._base_metrics()
        metrics["runtime_timing"] = {
            "encoder_timing_gap_count_delta": 1,
            "encoder_motion_timing_gap_count_delta": 1,
            "encoder_timing_gap_observed_samples": 1,
            "encoder_timing_gap_max_s": 0.052,
            "encoder_timing_contract_missing_samples": 0,
            "gc_motion_collection_count_delta": 0,
        }

        failures = _case_failures(metrics)
        warnings = _case_warnings(metrics)

        self.assertNotIn("encoder_timing_gap", failures)
        self.assertNotIn("settled_wheel_speed_tracking_error_high", failures)
        self.assertEqual(len(warnings), 1)
        self.assertEqual(warnings[0]["code"], "encoder_timing_gap")
        self.assertEqual(warnings[0]["severity"], "WARNING")
        self.assertEqual(warnings[0]["count"], 1)
        self.assertAlmostEqual(warnings[0]["max_gap_s"], 0.052)

    def test_idle_timing_gap_does_not_fail_motion_case(self):
        metrics = self._base_metrics()
        metrics["runtime_timing"] = {
            "encoder_timing_gap_count_delta": 1,
            "encoder_motion_timing_gap_count_delta": 0,
            "encoder_idle_timing_gap_count_delta": 1,
            "encoder_timing_contract_missing_samples": 0,
            "gc_motion_collection_count_delta": 0,
        }

        self.assertNotIn("encoder_timing_gap", _case_failures(metrics))

    def test_timing_gap_warning_summary_preserves_future_trend_counts(self):
        metrics = self._base_metrics()
        metrics["runtime_timing"] = {
            "encoder_timing_gap_count_delta": 3,
            "encoder_motion_timing_gap_count_delta": 2,
            "encoder_timing_gap_observed_samples": 2,
            "encoder_timing_gap_max_s": 0.061,
        }
        summary = _timing_gap_warning_summary(
            [
                {
                    "case": "forward",
                    "warnings": _case_warnings(metrics),
                    "metrics": metrics,
                }
            ]
        )

        self.assertEqual(summary["severity"], "WARNING")
        self.assertEqual(summary["case_count"], 1)
        self.assertEqual(summary["warning_count"], 2)
        self.assertEqual(summary["motion_timing_gap_count_delta"], 2)
        self.assertEqual(summary["timing_gap_count_delta"], 3)
        self.assertTrue(summary["future_trend_monitoring_required"])

    def test_timing_warning_does_not_mask_quality_failure(self):
        metrics = self._base_metrics()
        metrics["runtime_timing"] = {
            "encoder_timing_gap_count_delta": 1,
            "encoder_motion_timing_gap_count_delta": 1,
            "encoder_timing_gap_observed_samples": 1,
            "encoder_timing_gap_max_s": 0.048,
        }
        metrics["command_fidelity"]["errors"][
            "settled_wheel_speed_tracking_mae_mps"
        ] = 0.020

        failures = _case_failures(metrics)
        warnings = _case_warnings(metrics)

        self.assertIn("settled_wheel_speed_tracking_error_high", failures)
        self.assertNotIn("encoder_timing_gap", failures)
        self.assertEqual(warnings[0]["code"], "encoder_timing_gap")

    def test_motion_active_gc_is_an_independent_m1_failure(self):
        metrics = self._base_metrics()
        metrics["runtime_timing"] = {
            "encoder_timing_gap_count_delta": 0,
            "encoder_timing_contract_missing_samples": 0,
            "gc_motion_collection_count_delta": 1,
        }

        self.assertIn("gc_forbidden_while_motion_active", _case_failures(metrics))

    def test_rotation_rejects_imu_and_ekf_yaw_sign_mismatch(self):
        metrics = self._base_metrics()
        metrics["command"]["expected_linear_sign"] = 0
        metrics["command"]["expected_yaw_sign"] = 1
        metrics["imu"]["yaw_delta_deg"] = -4.0
        metrics["ekf"]["yaw_delta_deg"] = -3.5

        failures = _case_failures(metrics)

        self.assertIn("imu_yaw_sign_or_progress_bad", failures)
        self.assertIn("ekf_yaw_sign_or_progress_bad", failures)

    def test_trust_case_rejects_sensor_endpoint_disagreement(self):
        metrics = self._base_metrics()
        metrics["command"]["quality_gate"] = False
        metrics["command"]["expected_yaw_sign"] = 1
        metrics["imu"]["yaw_delta_deg"] = 22.0
        metrics["ekf"]["yaw_delta_deg"] = 1.0
        metrics["lidar"]["yaw_delta_deg"] = 21.0
        metrics["ekf"]["forward_delta_m"] = 0.20

        failures = _case_failures(metrics)

        self.assertIn("sensor_endpoint_yaw_spread_high", failures)
        self.assertIn("encoder_ekf_endpoint_distance_mismatch", failures)

    def test_stop_start_suspect_is_a_failure(self):
        metrics = self._base_metrics()
        metrics["stop_start"]["stop_start_suspect"] = True

        self.assertIn("stop_start_suspect", _case_failures(metrics))

    def test_linear_speed_ratio_outside_twenty_percent_is_a_failure(self):
        metrics = self._base_metrics()
        metrics["command_fidelity"]["errors"]["linear_speed_ratio_vs_executed"] = 3.0

        self.assertIn("linear_speed_error_high", _case_failures(metrics))

    def test_whole_phase_speed_ratio_outside_twenty_percent_is_a_failure(self):
        metrics = self._base_metrics()
        metrics["command_fidelity"]["errors"]["whole_phase_linear_speed_ratio_vs_executed"] = 0.62

        self.assertIn("whole_phase_linear_speed_error_high", _case_failures(metrics))

    def test_deadzone_pwm_occupancy_is_a_failure(self):
        metrics = self._base_metrics()
        metrics["motor_pwm"]["near_stable_floor_ratio"] = 0.80
        metrics["motor_pwm"]["below_stable_floor_ratio"] = 0.40
        metrics["stop_start"]["stop_start_suspect"] = True

        failures = _case_failures(metrics)

        self.assertIn("near_deadzone_pwm_occupancy_high", failures)
        self.assertIn("below_stable_pwm_floor_occupancy_high", failures)

    def test_missing_wheel_speed_tracking_is_a_failure(self):
        metrics = self._base_metrics()
        metrics["command_fidelity"]["errors"]["settled_wheel_speed_tracking_mae_mps"] = None

        self.assertIn("settled_wheel_speed_tracking_missing", _case_failures(metrics))

    def test_pivot_uses_angular_and_endpoint_gates_not_linear_settled_wheel_gate(self):
        metrics = self._base_metrics()
        metrics["command"].update(
            {
                "expected_linear_sign": 0,
                "expected_yaw_sign": 1,
                "omega_rad_s": 0.0,
                "target_angle_deg": 45.0,
                "duration_s": 8.0,
            }
        )
        metrics["encoder"]["differential_delta_m"] = 0.20
        metrics["imu"]["yaw_delta_deg"] = 45.0
        metrics["ekf"]["yaw_delta_deg"] = 44.0
        metrics["lidar"]["yaw_delta_deg"] = 43.5
        metrics["motor_pwm"]["opposing_active_samples"] = 8
        metrics["command_fidelity"]["errors"].update(
            {
                "settled_wheel_speed_tracking_mae_mps": None,
                "imu_angle_error_vs_requested_deg": 0.0,
                "endpoint_yaw_spread_deg": 1.5,
            }
        )
        metrics["command_fidelity"]["transient"].update(
            {"pivot_settling_time_s": 1.1, "pivot_overshoot_deg": 0.0}
        )

        failures = _case_failures(metrics)

        self.assertNotIn("settled_wheel_speed_tracking_missing", failures)

    def test_idle_case_rejects_nonzero_pwm_and_encoder_drift(self):
        metrics = self._base_metrics()
        metrics["command"]["command_motion"] = False
        metrics["command"]["expected_linear_sign"] = 0
        metrics["motor_pwm"]["max_abs_left"] = 0.08
        metrics["encoder"]["average_delta_m"] = 0.02

        failures = _case_failures(metrics)

        self.assertIn("idle_pwm_nonzero", failures)
        self.assertIn("idle_encoder_drift_high", failures)

    def test_runtime_diagnostics_reads_nested_global_policy(self):
        diagnostics = _extract_runtime_diagnostics(
            {
                "motion_command": {
                    "global_motion_policy": {
                        "active": True,
                        "actions": ["slow_down_for_policy"],
                        "policy_state": "APPROACH",
                        "forward_clearance_m": 0.42,
                        "omega_out": -0.12,
                    }
                }
            }
        )

        global_policy = diagnostics["global_motion_policy"]
        self.assertTrue(global_policy["active"])
        self.assertEqual(global_policy["policy_state"], "APPROACH")
        self.assertAlmostEqual(global_policy["forward_clearance_m"], 0.42)
        self.assertAlmostEqual(global_policy["omega_out"], -0.12)

    def test_runtime_diagnostics_exposes_heading_ownership_and_pwm_pipeline(self):
        diagnostics = _extract_runtime_diagnostics(
            {
                "control_monitor": {
                    "heading_correction_owner": "EXECUTOR_STRAIGHT_HOLD",
                    "executor_straight_hold_owner": True,
                    "drive_yaw_hold_enabled": False,
                    "wheel_loop_enabled": True,
                    "wheel_loop_feedback_source": "encoder_canonical",
                    "wheel_loop_left_ref_mps": 0.05,
                    "wheel_loop_right_ref_mps": 0.05,
                    "wheel_loop_left_meas_mps": 0.048,
                    "wheel_loop_right_meas_mps": 0.052,
                    "wheel_loop_left_error_mps": 0.002,
                    "wheel_loop_right_error_mps": -0.002,
                    "wheel_loop_left_maintenance_floor_pwm": 0.195,
                    "wheel_loop_right_maintenance_floor_pwm": 0.195,
                    "wheel_loop_left_maintenance_floor_applied": True,
                    "wheel_loop_right_maintenance_floor_applied": False,
                },
                "pid_diag": {
                    "omega_cmd_pre_guard": 0.031,
                    "pwm_raw_l": 0.11,
                    "pwm_raw_r": 0.17,
                    "pwm_executor_l": 0.24,
                    "pwm_executor_r": 0.29,
                    "forward_dominant_guard_pre_pwm_l": -0.03,
                    "forward_dominant_balance_floor_pwm": 0.22,
                    "straight_hold": {
                        "omega_correction_rad_s": 0.031,
                        "omega_correction_target_rad_s": 0.04,
                        "heading_error_deg": 1.2,
                        "slew_limited": True,
                    },
                },
            }
        )

        executor = diagnostics["executor"]
        self.assertEqual(executor["heading_correction_owner"], "EXECUTOR_STRAIGHT_HOLD")
        self.assertTrue(executor["executor_straight_hold_owner"])
        self.assertFalse(executor["drive_yaw_hold_enabled"])
        self.assertAlmostEqual(executor["straight_hold_correction_rad_s"], 0.031)
        self.assertAlmostEqual(executor["forward_guard_pre_pwm_left"], -0.03)
        self.assertTrue(executor["straight_hold_slew_limited"])
        self.assertTrue(executor["wheel_loop_enabled"])
        self.assertAlmostEqual(executor["wheel_loop_left_error_mps"], 0.002)
        self.assertAlmostEqual(executor["wheel_loop_left_maintenance_floor_pwm"], 0.195)
        self.assertTrue(executor["wheel_loop_left_maintenance_floor_applied"])
        self.assertFalse(executor["wheel_loop_right_maintenance_floor_applied"])

    def test_runtime_diagnostics_reads_effective_pi_gain_from_public_monitor(self):
        diagnostics = _extract_runtime_diagnostics(
            {
                "control_monitor": {
                    "wheel_loop_effective_kp": 1.2,
                    "wheel_loop_left_p": -0.018,
                    "wheel_loop_right_p": 0.007,
                },
                "pid_diag": {},
            }
        )

        executor = diagnostics["executor"]
        self.assertAlmostEqual(executor["wheel_loop_effective_kp"], 1.2)
        self.assertAlmostEqual(executor["wheel_loop_left_p"], -0.018)
        self.assertAlmostEqual(executor["wheel_loop_right_p"], 0.007)

    def test_sample_status_retains_existing_canonical_encoder_window(self):
        sample = _sample_status(
            "forward",
            {
                "status_version": 18,
                "encoder": {
                    "service": {
                        "snapshot_ts_perf": 100.0,
                        "snapshot_published_ts_perf": 100.001,
                        "snapshot_age_ms": 12.5,
                        "snapshot_publish_latency_ms": 1.0,
                    },
                    "computed": {
                        "step_distance_left_m": 0.000644,
                        "step_distance_right_m": 0.000644,
                    },
                    "left": {
                        "distance_m": 0.2,
                        "pulse_count": 310,
                        "snapshot": {
                            "distance_m": 0.2,
                            "pulses": 310,
                            "pulse_delta": 5,
                        },
                    },
                    "right": {
                        "distance_m": 0.21,
                        "pulse_count": 318,
                        "snapshot": {
                            "distance_m": 0.21,
                            "pulses": 318,
                            "pulse_delta": 6,
                        },
                    },
                    "canonical": {
                        "canonical_state": "FORWARD",
                        "combined_trust": 0.92,
                        "flags": [],
                        "timing_valid": True,
                        "timing_error": "",
                        "timing_gap_s": 0.0204,
                        "timing_gap_threshold_s": 0.04,
                        "timing_gap_count": 0,
                        "motion_timing_gap_count": 0,
                        "idle_timing_gap_count": 0,
                        "canonical_velocity": {
                            "left_mps": 0.149,
                            "right_mps": 0.153,
                        },
                        "pulses_delta": {
                            "left": 24,
                            "right": 25,
                            "left_control_window": 5,
                            "right_control_window": 6,
                            "dt_control_window_s": 0.0204,
                            "dt_aggregation_window_s": 0.102,
                            "window_start_ts": 99.9,
                            "window_end_ts": 100.0,
                            "left_count_start": 286,
                            "left_count_end": 310,
                            "right_count_start": 293,
                            "right_count_end": 318,
                        },
                    },
                },
            },
        )

        encoder = sample["encoder"]
        self.assertEqual(encoder["canonical_state"], "FORWARD")
        self.assertEqual(encoder["canonical_flags"], [])
        self.assertTrue(encoder["canonical_timing_valid"])
        self.assertTrue(encoder["canonical_timing_contract_present"])
        self.assertAlmostEqual(encoder["canonical_trust"], 0.92)
        self.assertAlmostEqual(encoder["canonical_velocity"]["left_mps"], 0.149)
        self.assertEqual(encoder["canonical_pulses_delta"]["left"], 24)
        self.assertEqual(encoder["raw_counter_difference_right_minus_left"], 8)
        self.assertEqual(encoder["snapshot_pulses_delta"]["left"], 5)
        self.assertAlmostEqual(encoder["measurement_freshness_s"], 0.0125)
        self.assertAlmostEqual(encoder["publication_delay_s"], 0.001)
        self.assertAlmostEqual(encoder["step_distance_m"]["left"], 0.000644)
        self.assertAlmostEqual(
            encoder["canonical_pulses_delta"]["window_start_ts"], 99.9
        )
        self.assertEqual(
            encoder["canonical_pulses_delta"]["left_count_start"], 286
        )
        self.assertAlmostEqual(
            encoder["canonical_pulses_delta"]["dt_aggregation_window_s"],
            0.102,
        )

    def test_correction_dynamics_counts_only_meaningful_direction_changes(self):
        values = [0.0, 0.002, 0.01, 0.02, -0.001, -0.02, -0.03, 0.02]
        self.assertEqual(_direction_changes(values, 0.003), 2)
        self.assertAlmostEqual(_total_variation([0.0, 0.1, -0.1, 0.0]), 0.4)

    def test_manual_reposition_needs_measured_guard_or_rejection(self):
        reason_only = _manual_reposition_suspected(
            {
                "lidar_odom_status": {
                    "localization_health_reason": "delivery_missing_idle_stationary_guard",
                    "control_loop_lidar_apply_status": "",
                }
            }
        )
        self.assertFalse(reason_only["suspected"])

        rejected = _manual_reposition_suspected(
            {"lidar_odom_status": {"control_loop_lidar_apply_status": "rejected_idle_stationary_guard"}}
        )
        self.assertTrue(rejected["suspected"])

    def test_idle_stationary_hold_is_measurement_ready_with_fresh_lidar(self):
        snapshot = _measurement_ready_snapshot(
            {
                "state": "IDLE",
                "lidar_health": "OK",
                "lidar_odom_status": {
                    "localization_health": "DEGRADED",
                    "localization_health_reason": "delivery_missing_idle_stationary_guard",
                    "control_loop_lidar_apply_status": "not_called",
                    "latest_age_s": 0.06,
                    "latest_confidence": 0.92,
                },
            }
        )

        self.assertTrue(snapshot["idle_stationary_hold"])
        self.assertTrue(snapshot["ready"])

    def test_rejected_idle_stationary_guard_is_not_measurement_ready(self):
        snapshot = _measurement_ready_snapshot(
            {
                "state": "IDLE",
                "lidar_health": "OK",
                "lidar_odom_status": {
                    "localization_health": "DEGRADED",
                    "localization_health_reason": "delivery_missing_idle_stationary_guard",
                    "control_loop_lidar_apply_status": "rejected_idle_stationary_guard",
                    "latest_age_s": 0.06,
                    "latest_confidence": 0.92,
                },
            }
        )

        self.assertFalse(snapshot["idle_stationary_hold"])
        self.assertFalse(snapshot["ready"])

    def test_post_reset_readiness_requires_one_second_stability_by_default(self):
        import inspect

        default = inspect.signature(
            __import__(
                "tools.live_motion_measurement_validator",
                fromlist=["_wait_measurement_ready_after_reset"],
            )._wait_measurement_ready_after_reset
        ).parameters["stable_samples"].default
        self.assertEqual(default, 10)

    def test_unwrapped_yaw_delta_preserves_rotation_across_wrap(self):
        self.assertAlmostEqual(
            _unwrapped_yaw_delta_deg([170.0, 179.0, -170.0, -120.0]),
            70.0,
            places=6,
        )

    def test_integrated_pose_progress_stays_forward_after_large_arc(self):
        samples = [
            {"pose": {"x": 0.0, "y": 0.0, "theta_deg": 0.0}},
            {"pose": {"x": 1.0, "y": 0.0, "theta_deg": 90.0}},
            {"pose": {"x": 1.0, "y": 1.0, "theta_deg": 180.0}},
            {"pose": {"x": -0.5, "y": 1.0, "theta_deg": 180.0}},
        ]

        self.assertAlmostEqual(_integrated_pose_forward_delta_m(samples), 3.5, places=6)
        self.assertAlmostEqual(
            _unwrapped_yaw_delta_deg([-170.0, -179.0, 170.0, 120.0]),
            -70.0,
            places=6,
        )

    def test_quality_gate_rejects_straight_yaw_drift_and_pivot_overtravel(self):
        straight = self._base_metrics()
        straight["imu"]["yaw_delta_deg"] = 18.0
        self.assertIn("straight_yaw_drift_high", _case_failures(straight))

        pivot = self._base_metrics()
        pivot["command"].update(
            {
                "expected_linear_sign": 0,
                "expected_yaw_sign": 1,
                "omega_rad_s": 0.28,
                "duration_s": 3.0,
                "quality_gate": True,
            }
        )
        pivot["imu"]["yaw_delta_deg"] = 300.0
        pivot["ekf"]["yaw_delta_deg"] = 290.0
        pivot["lidar"]["yaw_delta_deg"] = 280.0
        pivot["motor_pwm"] = {
            **pivot["motor_pwm"],
            "opposing_active_samples": 8,
        }
        self.assertIn("yaw_overtravel_high", _case_failures(pivot))


if __name__ == "__main__":
    unittest.main()
