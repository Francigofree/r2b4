#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import shutil
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools import r2b4_test_hub as hub
from tools import live_motion_measurement_validator as motion_validator
from tools import chassis_motion_dynamics_validator as dynamics_validator


def _healthy_lidar_samples(revision: int, monotonic_ns: int):
    return (
        SimpleNamespace(
            device_id="LIDAR_LOCALIZATION",
            kind="lidar_health",
            sequence=revision,
            captured_monotonic_ns=monotonic_ns,
            values=(
                SimpleNamespace(key="age_ns", value=0),
                SimpleNamespace(key="confidence", value=0.9),
            ),
        ),
        SimpleNamespace(
            device_id="LIDAR_LOCALIZATION",
            kind="lidar_matcher_diagnostics",
            sequence=revision,
            captured_monotonic_ns=monotonic_ns,
            values=(
                SimpleNamespace(key="candidate_id", value=revision),
                SimpleNamespace(key="source_raw_scan_id", value=revision + 100),
                SimpleNamespace(key="tracking_ready", value=True),
                SimpleNamespace(key="matcher_timed_out", value=False),
                SimpleNamespace(key="matcher_degenerate", value=False),
                SimpleNamespace(key="robust_rmse_m", value=0.01),
            ),
        ),
    )


class TestR2B4TestHub(unittest.TestCase):
    def test_m0_mini_contract_id_matches_validator(self):
        self.assertEqual(
            hub.M0_MINI_CONTRACT_ID,
            motion_validator.M0_MINI_CONTRACT_ID,
        )

    def test_m1_and_m2_contract_ids_match_their_validators(self):
        self.assertEqual(
            hub.M1_SPEED_MAP_EXECUTION_CONTRACT_ID,
            motion_validator.M1_SPEED_MAP_EXECUTION_CONTRACT_ID,
        )
        self.assertEqual(
            hub.M2_CHASSIS_DYNAMICS_CONTRACT_ID,
            dynamics_validator.CONTRACT_ID,
        )

    def _local_latest_publisher(self, latest_dir: Path):
        def _publish(target: Path, latest_name: str | None = None):
            latest_dir.mkdir(parents=True, exist_ok=True)
            src = Path(target)
            dest = latest_dir / (latest_name or src.name)
            shutil.copy2(src, dest)
            return {
                "ok": True,
                "mode": "copy",
                "latest": str(dest),
                "target": str(src),
            }

        return _publish

    def test_scenario_warning_is_carried_without_changing_pass_semantics(self):
        payload = {
            "success": True,
            "warnings": [
                {
                    "code": "encoder_timing_gap",
                    "severity": "WARNING",
                    "count": 2,
                }
            ],
            "warning_summary": {
                "severity": "WARNING",
                "motion_timing_gap_count_delta": 2,
                "future_trend_monitoring_required": True,
            },
        }

        warnings = hub._scenario_warnings(payload)

        self.assertTrue(hub._payload_success(payload))
        self.assertEqual(warnings["warnings"][0]["code"], "encoder_timing_gap")
        self.assertEqual(
            warnings["warning_summary"]["motion_timing_gap_count_delta"], 2
        )

    def test_live_profile_lock_rejects_overlapping_run_before_scenario(self):
        with tempfile.TemporaryDirectory() as td:
            lock_path = Path(td) / "live_profile.lock"
            with mock.patch.object(hub, "LIVE_PROFILE_LOCK_PATH", lock_path):
                holder, owner = hub._acquire_live_profile_lock("first_live")
                self.assertIsNotNone(holder)
                try:
                    with mock.patch.object(hub, "_run_profile_unlocked") as unlocked:
                        result = hub.run_profile("M0_measurement_trust_live")
                    unlocked.assert_not_called()
                finally:
                    hub._release_live_profile_lock(holder)

            self.assertEqual(result["status"], "FAIL")
            self.assertEqual(result["error"], "live_profile_already_running")
            self.assertEqual(result["verdict"]["primary"], "LIVE_PROFILE_ALREADY_RUNNING")
            self.assertEqual(result["live_profile_lock"]["owner"], owner)

    def test_live_profile_lock_is_released_after_run_exception(self):
        with tempfile.TemporaryDirectory() as td:
            lock_path = Path(td) / "live_profile.lock"
            with mock.patch.object(hub, "LIVE_PROFILE_LOCK_PATH", lock_path):
                with mock.patch.object(
                    hub,
                    "_run_profile_unlocked",
                    side_effect=RuntimeError("synthetic"),
                ):
                    with self.assertRaisesRegex(RuntimeError, "synthetic"):
                        hub.run_profile("M0_measurement_trust_live")
                holder, _owner = hub._acquire_live_profile_lock("next_live")
                self.assertIsNotNone(holder)
                hub._release_live_profile_lock(holder)

    def test_latest_artifact_publication_uses_declared_lease(self):
        manager = mock.Mock()
        with mock.patch.object(hub, "LeaseManager", return_value=manager):
            with hub._latest_artifact_publish_lease():
                manager.acquire.assert_called_once()
                manager.release.assert_not_called()

        resource, owner = manager.acquire.call_args.args[:2]
        self.assertEqual(resource, "latest_artifact_publish")
        manager.release.assert_called_once_with("latest_artifact_publish", owner)

    def test_latest_artifact_lease_is_released_after_publish_failure(self):
        manager = mock.Mock()
        with mock.patch.object(hub, "LeaseManager", return_value=manager):
            with self.assertRaisesRegex(RuntimeError, "synthetic publish"):
                with hub._latest_artifact_publish_lease():
                    raise RuntimeError("synthetic publish")

        owner = manager.acquire.call_args.args[1]
        manager.release.assert_called_once_with("latest_artifact_publish", owner)

    def test_artifact_payload_recovery_rejects_previous_run_and_accepts_fresh_file(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            rel = "logs/latest/latest_synthetic.json"
            artifact = root / rel
            artifact.parent.mkdir(parents=True, exist_ok=True)
            artifact.write_text(json.dumps({"success": True, "status": "PASS"}), encoding="utf-8")
            session_dir = root / "logs" / "session_unit_payload_recovery"
            session_dir.mkdir(parents=True, exist_ok=True)
            old_ts = time.time() - 30.0
            os.utime(artifact, (old_ts, old_ts))
            profile = hub.ScenarioProfile(
                name="synthetic",
                family="offline",
                description="freshness test",
                live=False,
                timeout_s=1.0,
                command=(sys.executable, "synthetic.py"),
                artifact_hints=(rel,),
            )
            run_started = time.time()

            def fake_artifact_candidates(path: str, *, session_dir: Path | None = None):
                raw = Path(path)
                return [raw if raw.is_absolute() else root / raw]

            with mock.patch.object(hub, "PROJECT_ROOT", root), mock.patch.object(
                hub, "artifact_candidates", side_effect=fake_artifact_candidates
            ):
                stale = hub._recover_payload_from_artifacts(
                    profile=profile,
                    run={},
                    started_wall_s=run_started,
                )
                stale_paths = hub._collect_artifact_paths(
                    profile,
                    run_result={},
                    preflight_result=None,
                    started_wall_s=run_started,
                    session_dir=session_dir,
                )

                fresh_ts = run_started + 0.1
                os.utime(artifact, (fresh_ts, fresh_ts))
                fresh = hub._recover_payload_from_artifacts(
                    profile=profile,
                    run={},
                    started_wall_s=run_started,
                )

            self.assertEqual(stale, {})
            self.assertEqual(stale_paths["existing"], [])
            self.assertEqual(stale_paths["stale_rejected"], [rel])
            self.assertTrue(fresh["success"])

    def test_runtime_path_extraction_removes_json_quotes_and_commas(self):
        paths = hub._extract_runtime_paths_from_text(
            '{"summary": "logs/latest/latest_summary.json", '
            '"result": "logs/latest/latest_result.json"}'
        )

        self.assertEqual(
            paths,
            [
                "logs/latest/latest_summary.json",
                "logs/latest/latest_result.json",
            ],
        )

    def test_list_profiles_contains_expected_tracks(self):
        payload = hub.list_profiles()
        self.assertEqual(payload.get("status"), "PASS")
        names = {p.get("name") for p in payload.get("profiles", [])}
        self.assertIn("M0_measurement_trust_live", names)
        self.assertIn("M1_motion_baseline_live", names)
        self.assertIn("M2_chassis_motion_dynamics_live", names)
        self.assertIn("M1_1_caster_orientation_live", names)
        self.assertIn("turning_iterative_small_space", names)
        self.assertNotIn("turning_track_reference_dual_phase", names)
        self.assertIn("track_sequence_loopback_custom", names)
        self.assertIn("gentle_arc", names)
        self.assertIn("medium_arc", names)
        self.assertIn("sharp_arc", names)
        self.assertIn("straight_1m", names)
        self.assertIn("pose_target", names)
        self.assertIn("pose_target_turn", names)
        self.assertIn("pose_target_sequence", names)
        self.assertIn("pose_target_sequence_slow", names)
        self.assertIn("pose_target_sequence_sharper", names)
        self.assertIn("follow_moving_target_sim", names)
        self.assertIn("person_target_direction_live", names)
        self.assertIn("follow_forward_home_toggle_live", names)
        self.assertIn("follow_triangle_0p8_live", names)
        self.assertNotIn("room_cruise_stop_pivot_return_live", names)
        self.assertNotIn("room_cruise_arc_oblique_live", names)
        self.assertNotIn("room_cruise_arc_continuous_1min_live", names)
        self.assertNotIn("room_cruise_amr_80s_return_live", names)
        self.assertNotIn("room_cruise_arc_continuous_3min_live", names)
        self.assertIn("kit0085_reverse_0p3m", names)
        self.assertIn("kit0085_arc_left", names)
        self.assertIn("kit0085_arc_right", names)
        self.assertIn("M0_measurement_trust_live", names)
        self.assertIn("M1_motion_baseline_live", names)
        self.assertNotIn("measurement_trust_live", names)
        self.assertNotIn("motion_baseline_measurement_live", names)
        self.assertIn("M3_emberkovetes_mozgasminoseg", names)
        self.assertIn("M4_room_cruise_quality_validator", names)
        self.assertIn("M4_1_room_cruise_quality_validator", names)
        self.assertIn("M3_motion_primitive_pivot_onset_series_live", names)
        self.assertIn("M3_motion_runtime_profile_reassess_offline", names)
        self.assertIn("M3_motor_feedforward_refit_offline", names)
        self.assertIn("speed_map_calibration_acquisition_live", names)
        self.assertIn("speed_map_calibration_analyze_offline", names)
        self.assertIn("speed_map_quick_no_pi_live", names)
        self.assertIn("speed_map_quick_pi_live", names)
        self.assertIn("speed_map_candidate_M1_live", names)
        self.assertIn("speed_map_calibration_decision_offline", names)

        reassess = next(
            profile
            for profile in payload.get("profiles", [])
            if profile.get("name") == "M3_motion_runtime_profile_reassess_offline"
        )
        self.assertFalse(bool(reassess.get("live", True)))

        offline_refit = next(
            profile
            for profile in payload.get("profiles", [])
            if profile.get("name") == "M3_motor_feedforward_refit_offline"
        )
        self.assertFalse(bool(offline_refit.get("live", True)))

        m4 = next(
            profile
            for profile in payload.get("profiles", [])
            if profile.get("name") == "M4_room_cruise_quality_validator"
        )
        self.assertTrue(bool(m4.get("live", False)))

    def test_speed_map_profiles_preserve_measure_analyze_validate_roles(self):
        registry = hub._scenario_registry()
        acquisition = registry["speed_map_calibration_acquisition_live"]
        analyzer = registry["speed_map_calibration_analyze_offline"]
        supplement = registry["speed_map_calibration_supplement_live"]
        no_pi = registry["speed_map_quick_no_pi_live"]
        pi = registry["speed_map_quick_pi_live"]
        candidate_m1 = registry["speed_map_candidate_M1_live"]
        decision = registry["speed_map_calibration_decision_offline"]

        self.assertTrue(acquisition.live)
        self.assertIn("--shuttle-acquisition", acquisition.command)
        self.assertIn("--max-abs-pwm", acquisition.command)
        self.assertEqual(
            acquisition.command[acquisition.command.index("--max-abs-pwm") + 1],
            "0.64",
        )
        self.assertFalse(acquisition.requires_measurement_truth)
        self.assertEqual(acquisition.preflight_clearance_m, 1.80)
        self.assertEqual(acquisition.preflight_clearance_mode, "straight-corridor")
        self.assertFalse(analyzer.live)
        self.assertIn("tools/speed_map_calibration_analyzer.py", analyzer.command)
        self.assertTrue(supplement.live)
        self.assertIn("--shuttle-supplement", supplement.command)
        self.assertEqual(supplement.preflight_clearance_m, 1.80)
        self.assertEqual(supplement.preflight_clearance_mode, "straight-corridor")
        self.assertFalse(supplement.requires_measurement_truth)
        self.assertTrue(no_pi.live)
        self.assertFalse(no_pi.requires_measurement_truth)
        self.assertEqual(no_pi.preflight_clearance_m, 1.80)
        self.assertEqual(no_pi.preflight_clearance_mode, "straight-corridor")
        self.assertEqual(no_pi.command[no_pi.command.index("--mode") + 1], "no-pi")
        self.assertEqual(
            no_pi.command[no_pi.command.index("--max-leg-attempts") + 1],
            "3",
        )
        self.assertTrue(pi.live)
        self.assertFalse(pi.requires_measurement_truth)
        self.assertEqual(pi.command[pi.command.index("--mode") + 1], "pi")
        self.assertEqual(
            pi.command[pi.command.index("--max-leg-attempts") + 1],
            "3",
        )
        self.assertTrue(candidate_m1.live)
        self.assertEqual(
            candidate_m1.command[candidate_m1.command.index("--mode") + 1],
            "m1",
        )
        self.assertFalse(decision.live)
        self.assertNotIn("--promote", decision.command)
        self.assertNotIn(
            "M2_chassis_motion_dynamics_live",
            hub.SPEED_MAP_CALIBRATION_SEQUENCE,
        )

    def test_m2_is_live_independent_and_non_promotion_blocking(self):
        profile = hub._scenario_registry()[
            "M2_chassis_motion_dynamics_live"
        ]

        self.assertTrue(profile.live)
        self.assertTrue(profile.requires_preflight)
        self.assertTrue(profile.preflight_pose_reset)
        self.assertIn(
            "tools/chassis_motion_dynamics_validator.py",
            profile.command,
        )
        goals = " ".join(profile.goals)
        self.assertIn("effective-track-width", goals)
        self.assertIn("excluded from speed-map ACCEPT and promotion", goals)

    def test_preflight_command_carries_declared_clearance_geometry(self):
        with mock.patch.object(
            hub,
            "_run_subprocess",
            return_value={
                "return_code": 0,
                "stdout_json": {"ok": True},
            },
        ):
            result = hub._run_preflight(
                clearance_m=1.80,
                timeout_s=20.0,
                clearance_mode="straight-corridor",
            )

        command = list(result["command"])
        self.assertEqual(
            command[command.index("--forward-clearance-mode") + 1],
            "straight-corridor",
        )
        self.assertEqual(
            command[command.index("--forward-clearance-m") + 1],
            "1.80",
        )

    def test_native_v3_profile_requires_fresh_sensor_gate_and_per_run_approval(self):
        profile = hub._scenario_registry()[hub.V3_NATIVE_RAISED_STAND_PROFILE]
        listed = {
            item["name"]: item
            for item in hub.list_profiles().get("profiles", [])
        }[profile.name]

        self.assertTrue(profile.live)
        self.assertTrue(profile.requires_preflight)
        self.assertEqual(profile.preflight_kind, hub.V3_NATIVE_PREFLIGHT_KIND)
        self.assertFalse(profile.requires_managed_runtime)
        self.assertEqual(
            tuple(profile.command[-2:]),
            ("tools/r2b4_test_hub.py", hub.V3_NATIVE_MOTION_COMMAND),
        )
        self.assertNotIn(hub.V3_NATIVE_MOTION_APPROVAL, profile.command)
        self.assertEqual(listed["preflight_kind"], hub.V3_NATIVE_PREFLIGHT_KIND)
        self.assertFalse(listed["requires_managed_runtime"])

    def test_native_v3_sensor_preflight_enforces_complete_healthy_zero_window(self):
        profile = hub.SCENARIOS[hub.V3_NATIVE_RAISED_STAND_PROFILE]
        base = {
            "status": "PASS",
            "tick_count": 300,
            "healthy_tick_count": 120,
            "fault_tick_count": 0,
            "l3_estimate_count": 300,
            "operator_stopped": False,
            "all_commits_zero": True,
        }
        with tempfile.TemporaryDirectory() as td, mock.patch.object(
            hub,
            "_run_subprocess",
            return_value={
                "return_code": 0,
                "stdout_json": dict(base),
            },
        ) as run_mock:
            result = hub._run_v3_native_sensor_preflight(
                run_dir=Path(td),
                profile=profile,
                env={"unit": "true"},
                retry_delay_s=0.0,
            )

        self.assertTrue(result["ok"])
        self.assertEqual(result["payload"]["errors"], [])
        self.assertEqual(result["payload"]["hub_gate"]["minimum_healthy_tick_count"], 1)
        command = list(run_mock.call_args.args[0])
        self.assertIn("tools/v3_sensor_measurement.py", command[1])
        self.assertEqual(command[command.index("--ticks") + 1], "300")
        preflight_env = run_mock.call_args.kwargs["env"]
        self.assertEqual(
            preflight_env["PYTHONPATH"].split(os.pathsep)[0],
            str(hub.PROJECT_ROOT),
        )

        weak = {**base, "healthy_tick_count": 0}
        with tempfile.TemporaryDirectory() as td, mock.patch.object(
            hub,
            "_run_subprocess",
            return_value={"return_code": 0, "stdout_json": weak},
        ):
            rejected = hub._run_v3_native_sensor_preflight(
                run_dir=Path(td),
                profile=profile,
                env={},
                retry_delay_s=0.0,
            )
        self.assertFalse(rejected["ok"])
        self.assertIn(
            "native_sensor_healthy_window_too_short",
            rejected["payload"]["errors"],
        )

    def test_native_v3_motion_rejects_missing_approval_before_any_hardware_gate(self):
        with mock.patch.object(
            hub,
            "_v3_profile_artifact_path",
            side_effect=AssertionError("artifact path must not be evaluated"),
        ), mock.patch.object(
            hub,
            "_verify_v3_motion_leases",
            side_effect=AssertionError("leases must not be evaluated"),
        ):
            result = hub._run_v3_native_raised_stand_motion("not-approved")

        self.assertEqual(result["status"], "FAIL")
        self.assertFalse(result["success"])
        self.assertEqual(
            result["error"],
            "explicit_powered_raised_stand_approval_required",
        )

    def test_resident_v3_profile_and_approval_gate_are_explicit(self):
        profile = hub.SCENARIOS[hub.V3_NATIVE_RESIDENT_RAISED_STAND_PROFILE]

        self.assertTrue(profile.live)
        self.assertFalse(profile.requires_managed_runtime)
        self.assertEqual(profile.preflight_kind, hub.V3_NATIVE_PREFLIGHT_KIND)
        self.assertEqual(
            profile.command[-1],
            hub.V3_NATIVE_RESIDENT_MOTION_COMMAND,
        )
        with mock.patch.object(
            hub,
            "_v3_resident_profile_artifact_path",
            side_effect=AssertionError("artifact path must not be evaluated"),
        ), mock.patch.object(
            hub,
            "_verify_v3_motion_leases",
            side_effect=AssertionError("leases must not be evaluated"),
        ):
            result = hub._run_v3_native_resident_raised_stand_motion(
                "not-approved"
            )

        self.assertEqual(result["status"], "FAIL")
        self.assertFalse(result["success"])
        self.assertEqual(
            result["error"],
            "explicit_powered_resident_raised_stand_approval_required",
        )

    def test_floor_v3_profile_and_approval_gate_are_explicit(self):
        profile = hub.SCENARIOS[hub.V3_NATIVE_FLOOR_MOTION_PROFILE]

        self.assertTrue(profile.live)
        self.assertFalse(profile.requires_managed_runtime)
        self.assertEqual(profile.preflight_kind, hub.V3_NATIVE_PREFLIGHT_KIND)
        self.assertEqual(profile.preflight_clearance_m, 1.30)
        self.assertEqual(profile.command[-1], hub.V3_NATIVE_FLOOR_MOTION_COMMAND)
        self.assertNotIn(hub.V3_NATIVE_FLOOR_MOTION_APPROVAL, profile.command)
        with mock.patch.object(
            hub,
            "_v3_floor_profile_artifact_paths",
            side_effect=AssertionError("artifact paths must not be evaluated"),
        ), mock.patch.object(
            hub,
            "_verify_v3_motion_leases",
            side_effect=AssertionError("leases must not be evaluated"),
        ):
            result = hub._run_v3_native_floor_motion_capture("not-approved")

        self.assertEqual(result["status"], "FAIL")
        self.assertFalse(result["success"])
        self.assertEqual(
            result["error"],
            "explicit_floor_clearance_and_speed_approval_required",
        )

    def test_floor_v3_gateway_bounds_then_distance_completes_at_0p15_mps(self):
        from v3.contracts import CommandMode, TickContext

        gateway = hub._V3ResidentRaisedStandGateway(
            active_tick_count=hub.V3_NATIVE_FLOOR_MAX_ACTIVE_TICK_COUNT,
            maximum_active_tick_count=hub.V3_NATIVE_FLOOR_MAX_ACTIVE_TICK_COUNT,
            v_mps=hub.V3_NATIVE_FLOOR_V_MPS,
            max_v_mps=hub.V3_NATIVE_FLOOR_V_MPS,
            command_prefix="floor-unit",
        )
        healthy = {
            "fault_layer": None,
            "safety_decision": "STOP",
            "safety_reason": "NOT_ACTIVE",
            "enabled": False,
            "left_output": 0.0,
            "right_output": 0.0,
            "source_health": [
                {"device_id": device, "state": "OK"}
                for device in ("encoder", "imu", "lidar")
            ],
        }

        for tick_id, revision in ((7, 31), (8, 32)):
            self.assertEqual(
                gateway.observe(
                    {
                        **healthy,
                        "tick_id": tick_id,
                        "lidar_health": {"revision": revision},
                    },
                    arm_permitted=False,
                ),
                "WAITING",
            )
        self.assertEqual(
            gateway.observe(
                {
                    **healthy,
                    "tick_id": 9,
                    "lidar_health": {"revision": 33},
                }
            ),
            "ARMED",
        )
        self.assertEqual(gateway.active_tick_id, 10)
        self.assertEqual(gateway.post_active_idle_tick_id, 510)
        self.assertEqual(gateway.shutdown_tick_id, 511)
        gateway.complete_active_after(409)
        self.assertEqual(gateway.active_tick_count, 400)
        self.assertEqual(gateway.post_active_idle_tick_id, 410)
        self.assertEqual(gateway.shutdown_tick_id, 411)
        active = [
            tick_id
            for tick_id in range(10, 412)
            if gateway.snapshot(TickContext(tick_id, 1000 + tick_id)).mode
            is CommandMode.TELEOP
        ]
        command = gateway.snapshot(TickContext(10, 1010))
        values = {item.key: item.value for item in command.goal}

        self.assertEqual(active, list(range(10, 410)))
        self.assertEqual(values["v_mps"], 0.15)
        self.assertEqual(values["max_v_mps"], 0.15)

    def test_floor_v3_gateway_repeated_lidar_revision_does_not_arm(self):
        gateway = hub._V3ResidentRaisedStandGateway(
            required_lidar_preflight_revisions=3,
        )
        healthy = {
            "fault_layer": None,
            "safety_decision": "STOP",
            "safety_reason": "NOT_ACTIVE",
            "enabled": False,
            "left_output": 0.0,
            "right_output": 0.0,
            "source_health": [
                {"device_id": device, "state": "OK"}
                for device in ("encoder", "imu", "lidar")
            ],
        }

        for tick_id, revision in enumerate((10, 10, 11)):
            self.assertEqual(
                gateway.observe(
                    {
                        **healthy,
                        "tick_id": tick_id,
                        "lidar_health": {"revision": revision},
                    }
                ),
                "WAITING",
            )

        self.assertIsNone(gateway.active_tick_id)
        self.assertEqual(gateway.lidar_preflight_revision_count, 2)
        self.assertEqual(
            gateway.observe(
                {
                    **healthy,
                    "tick_id": 3,
                    "lidar_health": {"revision": 12},
                }
            ),
            "ARMED",
        )
        self.assertEqual(gateway.active_tick_id, 4)

    def test_floor_v3_raw_lidar_gate_is_current_and_fail_closed(self):
        snapshot = SimpleNamespace(
            raw_scan_id=12,
            raw_scan_timestamp=1.0,
            health="OK",
            raw_scan=[{"angle": 0.0, "dist": 900.0, "quality": 10}],
            summary={
                "blocked_front": False,
                "min_dist": 0.9,
                "min_dist_narrow": 0.9,
                "raw_safety_valid_point_count": 1,
                "raw_safety_min_dist_point": {
                    "angle_deg": 0.0,
                    "distance_m": 0.9,
                },
            },
        )
        service = SimpleNamespace(get_snapshot=lambda: snapshot)

        clear, raw = hub._v3_lidar_tick_evidence(
            service,
            1_100_000_000,
            clearance_m=0.50,
        )
        blocked, _ = hub._v3_lidar_tick_evidence(
            service,
            1_400_000_000,
            clearance_m=0.50,
        )

        self.assertTrue(clear["ok"])
        self.assertEqual(raw["raw_scan"][0]["dist"], 900.0)
        self.assertFalse(blocked["ok"])
        self.assertIn("LIDAR_RAW_SCAN_STALE", blocked["blockers"])

    def test_native_v3_lease_root_resolves_outer_canonical_from_nested_candidate(self):
        with tempfile.TemporaryDirectory() as td:
            canonical = Path(td) / "canonical"
            candidate = canonical / "runtime" / "agent_workspaces" / "task" / "tree"
            for root in (canonical, candidate):
                (root / "tools").mkdir(parents=True)
                (root / "tools" / "agentctl.py").touch()
                coordination = root / "runtime" / "agent_coordination"
                coordination.mkdir(parents=True)
                (coordination / "current_change.json").write_text(
                    '{}\n',
                    encoding="utf-8",
                )
            with mock.patch.object(hub, "PROJECT_ROOT", candidate), mock.patch.dict(
                os.environ,
                {},
                clear=False,
            ):
                os.environ.pop(hub.V3_AGENT_LEASE_ROOT_ENV_VAR, None)
                resolved = hub._v3_agent_lease_root()

        self.assertEqual(resolved, canonical.resolve())

    def test_native_v3_motor_evidence_requires_active_cancel_and_verified_low_close(self):
        class Backend:
            def __init__(self):
                self.levels = {}
                self.busy = set()

            def gpiochip_open(self, _chip):
                return 7

            def gpio_claim_output(self, _handle, pin, initial_level):
                self.levels[pin] = initial_level
                return 0

            def gpio_write(self, _handle, pin, level):
                self.levels[pin] = level
                return 0

            def gpio_read(self, _handle, pin):
                return self.levels[pin]

            def gpio_free(self, _handle, pin):
                self.busy.discard(pin)
                return 0

            def tx_busy(self, _handle, pin, _kind):
                return int(pin in self.busy)

            def tx_pwm(self, _handle, pin, frequency_hz, duty_cycle):
                if frequency_hz == 0 and duty_cycle == 0.0:
                    self.busy.discard(pin)
                elif duty_cycle != 0.0:
                    self.busy.add(pin)
                return 0

            def gpiochip_close(self, _handle):
                return 0

        recorder = hub._V3MotorGpioRecorder(Backend())
        handle = recorder.gpiochip_open(0)
        pins = (12, 13, 18, 19)
        for pin in pins:
            recorder.gpio_claim_output(handle, pin, 0)
        recorder.tx_pwm(handle, 12, 8_000, 0.2)
        recorder.tx_pwm(handle, 18, 8_000, 0.2)
        for pin in pins:
            if recorder.tx_busy(handle, pin, 0):
                recorder.tx_pwm(handle, pin, 0, 0.0)
        for pin in pins:
            recorder.gpio_write(handle, pin, 0)
        for pin in pins:
            self.assertEqual(recorder.gpio_read(handle, pin), 0)
        time.sleep(0.003)
        for pin in pins:
            self.assertEqual(recorder.gpio_read(handle, pin), 0)
        recorder.gpiochip_close(handle)

        evidence = hub._v3_motor_gpio_evidence(recorder, pins)

        self.assertEqual(evidence["opened_handle_count"], 1)
        self.assertTrue(evidence["all_expected_pins_claimed_low"])
        self.assertEqual(evidence["nonzero_pwm_write_count"], 2)
        self.assertTrue(evidence["all_active_pwm_cancelled"])
        self.assertTrue(evidence["all_final_verified_low"])
        self.assertGreaterEqual(evidence["minimum_verified_low_hold_ms"], 2.0)
        self.assertTrue(evidence["gpio_closed_after_verified_low"])
        self.assertEqual(evidence["failed_event_count"], 0)

    def test_floor_v3_failed_run_preserves_fault_safe_low_evidence(self):
        evidence = hub._v3_terminal_safe_low_evidence(
            {"status": "FAULT", "termination_class": "FAULT_SAFE_LOW"},
            {
                "all_expected_pins_claimed_low": True,
                "all_final_verified_low": True,
                "minimum_verified_low_hold_ms": 2.5,
                "gpio_closed_after_verified_low": True,
                "failed_event_count": 0,
            },
            {"ok": True},
        )

        self.assertEqual(evidence["classification"], "FAULT_SAFE_LOW")
        self.assertTrue(evidence["verified"])
        self.assertTrue(evidence["runtime_reported_safe_low"])

    def test_native_v3_motion_handler_records_pass_without_bypassing_canonical_runner(self):
        class Backend:
            def __init__(self):
                self.levels = {}
                self.busy = set()

            def gpiochip_open(self, _chip):
                return 11

            def gpio_claim_output(self, _handle, pin, initial_level):
                self.levels[pin] = initial_level
                return 0

            def gpio_write(self, _handle, pin, level):
                self.levels[pin] = level
                return 0

            def gpio_read(self, _handle, pin):
                return self.levels[pin]

            def gpio_free(self, _handle, pin):
                self.busy.discard(pin)
                return 0

            def tx_busy(self, _handle, pin, _kind):
                return int(pin in self.busy)

            def tx_pwm(self, _handle, pin, frequency_hz, duty_cycle):
                if frequency_hz == 0 and duty_cycle == 0.0:
                    self.busy.discard(pin)
                elif duty_cycle != 0.0:
                    self.busy.add(pin)
                return 0

            def gpiochip_close(self, _handle):
                return 0

        backend = Backend()
        pins = (12, 13, 18, 19)
        config = SimpleNamespace(
            composition=SimpleNamespace(
                motor_output=SimpleNamespace(pins=pins),
            ),
            sensor_inputs=SimpleNamespace(lidar_danger_zone_m=0.1),
            tick_period_ns=20_000_000,
        )
        run_calls = []

        class FakeOwner:
            def __init__(self, counter_gpio, open_imu_bus, open_lidar, sensor_inputs):
                self.inputs = object()
                self.open_args = (counter_gpio, open_imu_bus, open_lidar, sensor_inputs)
                self.close_calls = 0

            def publish_tick_result(self, _result):
                return None

            def close(self):
                self.close_calls += 1

        def fake_canonical_run_owned(
            sensor_inputs,
            motor_gpio,
            received_config,
            *,
            stop_requested,
            tick_observer,
        ):
            run_calls.append(
                (sensor_inputs, received_config)
            )
            self.assertFalse(stop_requested())
            handle = motor_gpio.gpiochip_open(0)
            for pin in pins:
                motor_gpio.gpio_claim_output(handle, pin, 0)
            motor_gpio.tx_pwm(handle, 12, 8_000, 0.2)
            motor_gpio.tx_pwm(handle, 18, 8_000, 0.2)
            for pin in pins:
                if motor_gpio.tx_busy(handle, pin, 0):
                    motor_gpio.tx_pwm(handle, pin, 0, 0.0)
            for pin in pins:
                motor_gpio.gpio_write(handle, pin, 0)
            for pin in pins:
                self.assertEqual(motor_gpio.gpio_read(handle, pin), 0)
            time.sleep(0.003)
            for pin in pins:
                self.assertEqual(motor_gpio.gpio_read(handle, pin), 0)
            motor_gpio.gpiochip_close(handle)
            final_tick_id = (
                hub.V3_NATIVE_START_TICK_ID + hub.V3_NATIVE_ACTIVE_TICK_COUNT
            )
            for tick_id in range(final_tick_id + 1):
                active = (
                    hub.V3_NATIVE_START_TICK_ID
                    <= tick_id
                    < final_tick_id
                )
                command = SimpleNamespace(
                    safety_decision=SimpleNamespace(value="ALLOW" if active else "STOP"),
                    reason="ACTIVE" if active else "NOT_ACTIVE",
                    enabled=active,
                    left_output=0.2 if active else 0.0,
                    right_output=0.2 if active else 0.0,
                )
                tick_observer(
                    SimpleNamespace(
                        trace=SimpleNamespace(
                            context=SimpleNamespace(
                                tick_id=tick_id,
                                monotonic_ns=1_000_000_000 + tick_id,
                            ),
                            fault_layer=None,
                            layers=(),
                        ),
                        final_actuation=command,
                    )
                )
            return 0

        with tempfile.TemporaryDirectory() as td:
            logs_dir = Path(td) / "logs"
            artifact_dir = logs_dir / "session" / "tests" / hub.V3_NATIVE_RAISED_STAND_PROFILE
            with mock.patch.object(hub, "LOGS_DIR", logs_dir), mock.patch.dict(
                os.environ,
                {
                    hub.V3_NATIVE_PROFILE_ENV_VAR: hub.V3_NATIVE_RAISED_STAND_PROFILE,
                    hub.TEST_SESSION_ENV_VAR: str(artifact_dir),
                },
                clear=False,
            ), mock.patch.object(
                hub,
                "_verify_v3_motion_leases",
                return_value={"ok": True, "task_id": "unit", "leases": {}, "errors": []},
            ), mock.patch.object(
                hub,
                "_v3_native_runtime_config",
                return_value=config,
            ), mock.patch.object(
                hub,
                "_v3_hardware_api",
                return_value={
                    "counter_gpio": object(),
                    "motor_gpio": backend,
                    "open_imu_bus": object(),
                    "lidar_service_type": object(),
                    "sensor_owner_type": FakeOwner,
                    "run_owned": fake_canonical_run_owned,
                },
            ), mock.patch.object(
                hub,
                "_v3_post_close_pin_state",
                return_value={
                    "status": "PASS",
                    "ok": True,
                    "pins": {
                        str(pin): {"output": True, "drive_low": True}
                        for pin in pins
                    },
                    "errors": [],
                },
            ):
                result = hub._run_v3_native_raised_stand_motion(
                    hub.V3_NATIVE_MOTION_APPROVAL
                )

            artifact = artifact_dir / "v3_native_raised_stand_motion.json"
            self.assertTrue(artifact.is_file())
            persisted = json.loads(artifact.read_text(encoding="utf-8"))

        self.assertEqual(len(run_calls), 1)
        self.assertEqual(result["status"], "PASS")
        self.assertEqual(persisted["final_lifecycle"], "IDLE")
        self.assertEqual(
            persisted["motor_power"],
            "ON_RAISED_STAND_BY_EXPLICIT_APPROVAL",
        )
        self.assertTrue(persisted["motor_gpio"]["all_active_pwm_cancelled"])
        self.assertTrue(persisted["motor_gpio"]["all_final_verified_low"])
        self.assertTrue(persisted["motor_gpio"]["gpio_closed_after_verified_low"])
        self.assertTrue(persisted["post_close_pins"]["ok"])

    def test_native_v3_profile_checks_legacy_runtime_is_stopped_before_custom_preflight(self):
        profile = hub.SCENARIOS[hub.V3_NATIVE_RAISED_STAND_PROFILE]
        events = []
        with tempfile.TemporaryDirectory() as td:
            run_dir = Path(td) / "logs" / "session_v3_native"
            run_dir.mkdir(parents=True)

            def runtime_action(action):
                events.append(f"runtime:{action}")
                return {
                    "ok": False,
                    "payload": {
                        "running": False,
                        "ready_for_live_tests": False,
                        "processes": [],
                    },
                    "run": {
                        "ok": True,
                        "timed_out": False,
                        "return_code": 1,
                    },
                }

            with mock.patch.object(
                hub,
                "_new_hub_session_dir",
                return_value=run_dir,
            ), mock.patch.object(
                hub,
                "apply_runtime_affinity",
                return_value={},
            ), mock.patch.object(
                hub,
                "_runtime_manager_action",
                side_effect=runtime_action,
            ), mock.patch.object(
                hub,
                "_run_v3_native_sensor_preflight",
                side_effect=lambda **_kwargs: events.append("v3-preflight")
                or {"ok": True, "payload": {"ok": True, "status": "PASS"}},
            ), mock.patch.object(
                hub,
                "_run_preflight",
                side_effect=AssertionError("managed-runtime preflight must not run"),
            ), mock.patch.object(
                hub,
                "_run_subprocess",
                side_effect=lambda command, **_kwargs: events.append(
                    "scenario:" + " ".join(command[-2:])
                )
                or {
                    "ok": True,
                    "timed_out": False,
                    "return_code": 0,
                    "duration_s": 0.1,
                    "stdout_tail": '{"success": true, "status": "PASS"}',
                    "stderr_tail": "",
                    "stdout_json": {"success": True, "status": "PASS"},
                    "stderr_json": None,
                },
            ), mock.patch.object(
                hub,
                "_logger_lifecycle_snapshot",
                side_effect=[
                    {"logger_queue_depth": 0, "dropped_messages": 0, "write_errors": 0},
                    {"logger_queue_depth": 0, "dropped_messages": 0, "write_errors": 0},
                ],
            ), mock.patch.object(
                hub,
                "_publish_hub_alias_bundle",
                return_value=[],
            ):
                result = hub._run_profile_unlocked(
                    profile.name,
                    auto_runtime=True,
                    archive_logs=False,
                    extra_args=["--", "--approval", hub.V3_NATIVE_MOTION_APPROVAL],
                )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(
            events,
            [
                "runtime:status",
                "v3-preflight",
                f"scenario:--approval {hub.V3_NATIVE_MOTION_APPROVAL}",
            ],
        )

    def test_resident_v3_handler_signals_shutdown_and_records_hard_low_pass(self):
        from v3.contracts import CommandMode, TickContext

        class Backend:
            def __init__(self):
                self.levels = {}
                self.busy = set()

            def gpiochip_open(self, _chip):
                return 31

            def gpio_claim_output(self, _handle, pin, initial_level):
                self.levels[pin] = initial_level
                return 0

            def gpio_write(self, _handle, pin, level):
                self.levels[pin] = level
                return 0

            def gpio_read(self, _handle, pin):
                return self.levels[pin]

            def gpio_free(self, _handle, pin):
                self.busy.discard(pin)
                return 0

            def tx_busy(self, _handle, pin, _kind):
                return int(pin in self.busy)

            def tx_pwm(self, _handle, pin, frequency_hz, duty_cycle):
                if frequency_hz == 0 and duty_cycle == 0.0:
                    self.busy.discard(pin)
                elif duty_cycle != 0.0:
                    self.busy.add(pin)
                return 0

            def gpiochip_close(self, _handle):
                return 0

        pins = (12, 13, 18, 19)
        backend = Backend()
        config = SimpleNamespace(
            composition=SimpleNamespace(
                motor_output=SimpleNamespace(pins=pins),
            ),
            sensor_inputs=SimpleNamespace(lidar_danger_zone_m=0.1),
            tick_period_ns=20_000_000,
        )
        run_calls = []

        def result_for(tick_id, *, active=False, healthy=True):
            monotonic_ns = 1_000_000_000 + tick_id * 20_000_000
            health = tuple(
                SimpleNamespace(
                    device_id=device,
                    state=SimpleNamespace(
                        value=(
                            "DEGRADED"
                            if device == "lidar" and not healthy
                            else "OK"
                        )
                    ),
                    reason=(
                        "LIDAR_STALE"
                        if device == "lidar" and not healthy
                        else "OK"
                    ),
                )
                for device in ("encoder", "imu", "lidar")
            )
            return SimpleNamespace(
                trace=SimpleNamespace(
                    context=SimpleNamespace(
                        tick_id=tick_id,
                        monotonic_ns=1_000_000_000 + tick_id * 20_000_000,
                    ),
                    fault_layer=None,
                    layers=(
                        SimpleNamespace(
                            layer="L1",
                            output=SimpleNamespace(
                                io_health=health,
                                samples=_healthy_lidar_samples(
                                    tick_id + 1,
                                    monotonic_ns,
                                ),
                            ),
                        ),
                    ),
                ),
                final_actuation=SimpleNamespace(
                    safety_decision=SimpleNamespace(
                        value="ALLOW" if active else "STOP"
                    ),
                    reason="ACTIVE" if active else "NOT_ACTIVE",
                    enabled=active,
                    left_output=0.2 if active else 0.0,
                    right_output=0.2 if active else 0.0,
                ),
            )

        def fake_resident_run(
            counter_gpio,
            open_imu_bus,
            open_lidar,
            command_gateway,
            motor_gpio,
            received_config,
            *,
            approval,
            stop_requested,
            tick_observer,
        ):
            run_calls.append((counter_gpio, open_imu_bus, open_lidar, received_config))
            self.assertEqual(approval, "native-resident-v3")
            self.assertFalse(stop_requested())
            handle = motor_gpio.gpiochip_open(0)
            for pin in pins:
                motor_gpio.gpio_claim_output(handle, pin, 0)
            active_ticks = []
            last_normal_tick_id = None
            for tick_id in range(20):
                context = TickContext(
                    tick_id,
                    1_000_000_000 + tick_id * 20_000_000,
                )
                command = command_gateway.snapshot(context)
                active = command.mode is CommandMode.TELEOP
                if active:
                    active_ticks.append(tick_id)
                    motor_gpio.tx_pwm(handle, 12, 8_000, 0.2)
                    motor_gpio.tx_pwm(handle, 18, 8_000, 0.2)
                if tick_id == command_gateway.post_active_idle_tick_id:
                    for pin in pins:
                        if motor_gpio.tx_busy(handle, pin, 0):
                            motor_gpio.tx_pwm(handle, pin, 0, 0.0)
                tick_observer(
                    result_for(
                        tick_id,
                        active=active,
                        healthy=tick_id >= 6,
                    )
                )
                last_normal_tick_id = tick_id
                if stop_requested():
                    break
            self.assertTrue(stop_requested())
            self.assertEqual(active_ticks, [9])
            self.assertEqual(command_gateway.active_tick_id, 9)
            self.assertEqual(command_gateway.post_active_idle_tick_id, 10)
            self.assertEqual(command_gateway.shutdown_tick_id, 11)
            self.assertEqual(last_normal_tick_id, 10)
            tick_observer(
                result_for(command_gateway.shutdown_tick_id)
            )
            for pin in pins:
                motor_gpio.gpio_write(handle, pin, 0)
            for pin in pins:
                self.assertEqual(motor_gpio.gpio_read(handle, pin), 0)
            time.sleep(0.003)
            for pin in pins:
                self.assertEqual(motor_gpio.gpio_read(handle, pin), 0)
            motor_gpio.gpiochip_close(handle)
            return SimpleNamespace(
                as_dict=lambda: {
                    "schema": "R2B4_V3_RESIDENT_RUNTIME_REPORT_V1",
                    "status": "PASS",
                    "run_status": 0,
                    "exit_reason": "STOP_REQUESTED",
                    "tick_count": command_gateway.shutdown_tick_id + 1,
                    "normal_tick_count": command_gateway.shutdown_tick_id,
                    "last_tick_id": command_gateway.shutdown_tick_id,
                    "final_lifecycle": "SHUTDOWN",
                    "final_safety_decision": "STOP",
                    "final_reason": "NOT_ACTIVE",
                    "fault_layer": None,
                    "operator_stopped": True,
                }
            )

        with tempfile.TemporaryDirectory() as td:
            logs_dir = Path(td) / "logs"
            artifact_dir = (
                logs_dir
                / "session"
                / "tests"
                / hub.V3_NATIVE_RESIDENT_RAISED_STAND_PROFILE
            )
            with mock.patch.object(hub, "LOGS_DIR", logs_dir), mock.patch.dict(
                os.environ,
                {
                    hub.V3_NATIVE_PROFILE_ENV_VAR:
                        hub.V3_NATIVE_RESIDENT_RAISED_STAND_PROFILE,
                    hub.TEST_SESSION_ENV_VAR: str(artifact_dir),
                },
                clear=False,
            ), mock.patch.object(
                hub,
                "_verify_v3_motion_leases",
                return_value={"ok": True, "task_id": "unit", "leases": {}, "errors": []},
            ), mock.patch.object(
                hub,
                "_v3_native_resident_runtime_config",
                return_value=config,
            ), mock.patch.object(
                hub,
                "_v3_resident_hardware_api",
                return_value={
                    "counter_gpio": object(),
                    "motor_gpio": backend,
                    "open_imu_bus": object(),
                    "lidar_service_type": object(),
                    "resident_approval": "native-resident-v3",
                    "run_resident": fake_resident_run,
                },
            ), mock.patch.object(
                hub,
                "_v3_post_close_pin_state",
                return_value={
                    "status": "PASS",
                    "ok": True,
                    "pins": {
                        str(pin): {"output": True, "drive_low": True}
                        for pin in pins
                    },
                    "errors": [],
                },
            ):
                result = hub._run_v3_native_resident_raised_stand_motion(
                    hub.V3_NATIVE_RESIDENT_MOTION_APPROVAL
                )

            artifact = artifact_dir / "v3_native_resident_raised_stand.json"
            persisted = json.loads(artifact.read_text(encoding="utf-8"))

        self.assertEqual(len(run_calls), 1)
        self.assertEqual(result["status"], "PASS")
        self.assertEqual(persisted["final_lifecycle"], "SHUTDOWN")
        self.assertEqual(
            persisted["tick_evidence"]["signal_raised_after_tick"],
            10,
        )
        self.assertEqual(persisted["command_window"]["active_tick_id"], 9)
        self.assertEqual(persisted["command_window"]["shutdown_tick_id"], 11)
        self.assertEqual(
            persisted["tick_evidence"]["resident_preflight"]["tick_id"],
            8,
        )
        self.assertTrue(persisted["motor_gpio"]["all_active_pwm_cancelled"])
        self.assertTrue(persisted["motor_gpio"]["all_final_verified_low"])
        self.assertTrue(persisted["post_close_pins"]["ok"])

    def test_floor_v3_handler_distance_completes_1m_and_hard_low_pass(self):
        from v3.contracts import CommandMode, TickContext

        class Backend:
            def __init__(self):
                self.levels = {}
                self.busy = set()

            def gpiochip_open(self, _chip):
                return 41

            def gpio_claim_output(self, _handle, pin, initial_level):
                self.levels[pin] = initial_level
                return 0

            def gpio_write(self, _handle, pin, level):
                self.levels[pin] = level
                return 0

            def gpio_read(self, _handle, pin):
                return self.levels[pin]

            def gpio_free(self, _handle, pin):
                self.busy.discard(pin)
                return 0

            def tx_busy(self, _handle, pin, _kind):
                return int(pin in self.busy)

            def tx_pwm(self, _handle, pin, frequency_hz, duty_cycle):
                if frequency_hz == 0 and duty_cycle == 0.0:
                    self.busy.discard(pin)
                elif duty_cycle != 0.0:
                    self.busy.add(pin)
                return 0

            def gpiochip_close(self, _handle):
                return 0

        class FakeLidar:
            def __init__(self, **_kwargs):
                self.tick_id = 0
                self.started = False

            def start(self):
                self.started = True
                return True

            def stop(self):
                self.started = False

            def get_snapshot(self):
                tick_id = self.tick_id
                return SimpleNamespace(
                    raw_scan_id=tick_id + 1,
                    raw_scan_timestamp=1.0 + tick_id * 0.02,
                    health="OK",
                    raw_scan=[
                        {"angle": 0.0, "dist": 1500.0, "quality": 15},
                        {"angle": 90.0, "dist": 1600.0, "quality": 12},
                    ],
                    summary={
                        "blocked_front": False,
                        "min_dist": 1.5,
                        "min_dist_narrow": 1.5,
                        "raw_safety_valid_point_count": 2,
                        "raw_safety_min_dist_point": {
                            "angle_deg": 0.0,
                            "distance_m": 1.5,
                        },
                        "raw_scan_id": tick_id + 1,
                    },
                )

        pins = (12, 13, 18, 19)
        backend = Backend()
        config = SimpleNamespace(
            composition=SimpleNamespace(
                motor_output=SimpleNamespace(pins=pins),
                live_control=SimpleNamespace(
                    control=SimpleNamespace(
                        motion_realization=SimpleNamespace(
                            local_clearance_m=0.2,
                        ),
                    ),
                ),
            ),
            sensor_inputs=SimpleNamespace(lidar_danger_zone_m=0.1),
            tick_period_ns=20_000_000,
        )

        def result_for(tick_id, *, active, x_m):
            monotonic_ns = 1_000_000_000 + tick_id * 20_000_000
            health = tuple(
                SimpleNamespace(
                    device_id=device,
                    state=SimpleNamespace(value="OK"),
                    reason="OK",
                )
                for device in ("encoder", "imu", "lidar")
            )
            encoder = SimpleNamespace(
                kind="wheel_velocity",
                values=(
                    SimpleNamespace(key="left_mps", value=0.15 if active else 0.0),
                    SimpleNamespace(key="right_mps", value=0.15 if active else 0.0),
                ),
            )
            l1 = SimpleNamespace(
                io_health=health,
                samples=(
                    encoder,
                    *_healthy_lidar_samples(tick_id + 1, monotonic_ns),
                ),
            )
            l3 = SimpleNamespace(
                x_m=x_m,
                y_m=0.0,
                yaw_rad=0.0,
                v_mps=0.15 if active else 0.0,
                omega_rad_s=0.0,
            )
            final = SimpleNamespace(
                safety_decision=SimpleNamespace(
                    value="ALLOW" if active else "STOP"
                ),
                reason="ACTIVE" if active else "NOT_ACTIVE",
                enabled=active,
                left_output=0.25 if active else 0.0,
                right_output=0.25 if active else 0.0,
            )
            layers = [
                SimpleNamespace(layer="L1", output=l1),
                SimpleNamespace(layer="L2", output=SimpleNamespace(tick=tick_id)),
                SimpleNamespace(layer="L3", output=l3),
            ]
            layers.extend(
                SimpleNamespace(
                    layer=f"L{layer_id}",
                    output=(final if layer_id == 12 else SimpleNamespace(tick=tick_id)),
                )
                for layer_id in range(4, 13)
            )
            return SimpleNamespace(
                trace=SimpleNamespace(
                    context=SimpleNamespace(
                        tick_id=tick_id,
                        monotonic_ns=1_000_000_000 + tick_id * 20_000_000,
                    ),
                    fault_layer=None,
                    layers=tuple(layers),
                ),
                final_actuation=final,
            )

        def fake_resident_run(
            counter_gpio,
            open_imu_bus,
            open_lidar,
            command_gateway,
            motor_gpio,
            received_config,
            *,
            approval,
            stop_requested,
            tick_observer,
        ):
            self.assertEqual(approval, "native-resident-v3")
            lidar = open_lidar(None)
            handle = motor_gpio.gpiochip_open(0)
            for pin in pins:
                motor_gpio.gpio_claim_output(handle, pin, 0)
            x_m = 0.0
            active_tick_count = 0
            last_normal_tick_id = None
            for tick_id in range(600):
                lidar.tick_id = tick_id
                command = command_gateway.snapshot(
                    TickContext(
                        tick_id,
                        1_000_000_000 + tick_id * 20_000_000,
                    )
                )
                active = command.mode is CommandMode.TELEOP
                if active:
                    active_tick_count += 1
                    x_m = active_tick_count * 0.0025
                    motor_gpio.tx_pwm(handle, 12, 8_000, 0.25)
                    motor_gpio.tx_pwm(handle, 18, 8_000, 0.25)
                if tick_id == command_gateway.post_active_idle_tick_id:
                    for pin in pins:
                        if motor_gpio.tx_busy(handle, pin, 0):
                            motor_gpio.tx_pwm(handle, pin, 0, 0.0)
                tick_observer(result_for(tick_id, active=active, x_m=x_m))
                last_normal_tick_id = tick_id
                if stop_requested():
                    break
            self.assertEqual(command_gateway.active_tick_id, 3)
            self.assertEqual(command_gateway.active_tick_count, 400)
            self.assertEqual(command_gateway.post_active_idle_tick_id, 403)
            self.assertEqual(command_gateway.shutdown_tick_id, 404)
            self.assertEqual(last_normal_tick_id, 403)
            lidar.tick_id = command_gateway.shutdown_tick_id
            tick_observer(
                result_for(
                    command_gateway.shutdown_tick_id,
                    active=False,
                    x_m=x_m,
                )
            )
            for pin in pins:
                motor_gpio.gpio_write(handle, pin, 0)
            for pin in pins:
                self.assertEqual(motor_gpio.gpio_read(handle, pin), 0)
            time.sleep(0.003)
            for pin in pins:
                self.assertEqual(motor_gpio.gpio_read(handle, pin), 0)
            motor_gpio.gpiochip_close(handle)
            lidar.stop()
            return SimpleNamespace(
                as_dict=lambda: {
                    "schema": "R2B4_V3_RESIDENT_RUNTIME_REPORT_V2",
                    "status": "PASS",
                    "run_status": 0,
                    "exit_reason": "STOP_REQUESTED",
                    "tick_count": command_gateway.shutdown_tick_id + 1,
                    "normal_tick_count": command_gateway.shutdown_tick_id,
                    "last_tick_id": command_gateway.shutdown_tick_id,
                    "final_lifecycle": "SHUTDOWN",
                    "final_safety_decision": "STOP",
                    "final_reason": "NOT_ACTIVE",
                    "fault_layer": None,
                    "operator_stopped": True,
                    "termination_class": "SHUTDOWN_SAFE_LOW",
                }
            )

        with tempfile.TemporaryDirectory() as td:
            logs_dir = Path(td) / "logs"
            artifact_dir = (
                logs_dir
                / "session"
                / "tests"
                / hub.V3_NATIVE_FLOOR_MOTION_PROFILE
            )
            with mock.patch.object(hub, "LOGS_DIR", logs_dir), mock.patch.dict(
                os.environ,
                {
                    hub.V3_NATIVE_PROFILE_ENV_VAR:
                        hub.V3_NATIVE_FLOOR_MOTION_PROFILE,
                    hub.TEST_SESSION_ENV_VAR: str(artifact_dir),
                },
                clear=False,
            ), mock.patch.object(
                hub,
                "_verify_v3_motion_leases",
                return_value={"ok": True, "task_id": "unit", "leases": {}, "errors": []},
            ), mock.patch.object(
                hub,
                "_v3_native_resident_runtime_config",
                return_value=config,
            ), mock.patch.object(
                hub,
                "_v3_resident_hardware_api",
                return_value={
                    "counter_gpio": object(),
                    "motor_gpio": backend,
                    "open_imu_bus": object(),
                    "lidar_service_type": FakeLidar,
                    "resident_approval": "native-resident-v3",
                    "run_resident": fake_resident_run,
                },
            ), mock.patch.object(
                hub,
                "_v3_post_close_pin_state",
                return_value={
                    "status": "PASS",
                    "ok": True,
                    "pins": {
                        str(pin): {"output": True, "drive_low": True}
                        for pin in pins
                    },
                    "errors": [],
                },
            ):
                result = hub._run_v3_native_floor_motion_capture(
                    hub.V3_NATIVE_FLOOR_MOTION_APPROVAL
                )

            persisted = json.loads(
                (artifact_dir / "v3_native_floor_motion_capture.json").read_text(
                    encoding="utf-8"
                )
            )
            capture = json.loads(
                (artifact_dir / "v3_native_floor_motion_ticks.json").read_text(
                    encoding="utf-8"
                )
            )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(persisted["command_window"]["v_mps"], 0.15)
        self.assertEqual(persisted["command_window"]["active_tick_count"], 400)
        self.assertEqual(
            persisted["command_window"]["maximum_active_tick_count"],
            500,
        )
        self.assertEqual(
            persisted["tick_evidence"]["allow_tick_ids"],
            list(range(3, 403)),
        )
        self.assertEqual(persisted["motion_metrics"]["active_duration_s"], 8.0)
        self.assertEqual(persisted["motion_metrics"]["target_reached_tick_id"], 402)
        self.assertEqual(
            persisted["motion_metrics"]["target_reached_displacement_m"],
            1.0,
        )
        self.assertEqual(capture["tick_count"], 405)
        self.assertEqual(capture["unique_raw_lidar_scan_count"], 405)
        self.assertEqual(
            persisted["command_window"]["required_lidar_preflight_revisions"],
            3,
        )
        self.assertEqual(
            [
                item["revision"]
                for item in persisted["tick_evidence"][
                    "lidar_matcher_preflight_revisions"
                ]
            ],
            [1, 2, 3],
        )
        self.assertEqual(
            persisted["tick_evidence"]["lidar_matcher_preflight_revisions"][
                -1
            ]["lidar_matcher_diagnostics"]["source_raw_scan_id"],
            103,
        )
        self.assertEqual(
            persisted["safety_bounds"]["active_safety_threshold_m"],
            0.2,
        )
        self.assertEqual(persisted["termination_class"], "SHUTDOWN_SAFE_LOW")
        self.assertTrue(persisted["terminal_safe_low"]["verified"])
        self.assertTrue(persisted["motor_gpio"]["all_active_pwm_cancelled"])
        self.assertTrue(persisted["motor_gpio"]["all_final_verified_low"])
        self.assertTrue(persisted["post_close_pins"]["ok"])

    def test_resident_v3_gateway_bounds_unhealthy_matcher_warmup_without_active(self):
        from v3.contracts import CommandMode, TickContext

        gateway = hub._V3ResidentRaisedStandGateway(max_warmup_tick_id=2)
        unhealthy = {
            "fault_layer": None,
            "safety_decision": "STOP",
            "safety_reason": "CRITICAL_DEVICE_DEGRADED",
            "enabled": False,
            "left_output": 0.0,
            "right_output": 0.0,
            "source_health": [
                {"device_id": "encoder", "state": "OK"},
                {"device_id": "imu", "state": "OK"},
                {"device_id": "lidar", "state": "DEGRADED"},
            ],
        }

        for tick_id, expected in ((0, "WAITING"), (1, "WAITING"), (2, "TIMEOUT")):
            command = gateway.snapshot(TickContext(tick_id, 1_000 + tick_id))
            self.assertIs(command.mode, CommandMode.STOP)
            self.assertEqual(
                gateway.observe({**unhealthy, "tick_id": tick_id}),
                expected,
            )

        self.assertIsNone(gateway.active_tick_id)
        self.assertEqual(gateway.warmup_timeout_tick_id, 2)

    def test_m4_room_cruise_profile_is_bounded_and_uses_common_speed_band(self):
        profile = hub._scenario_registry()["M4_room_cruise_quality_validator"]
        command = list(profile.command)

        self.assertEqual(command[command.index("--duration-s") + 1], "60.0")
        self.assertEqual(command[command.index("--v-max-mps") + 1], "0.30")
        self.assertEqual(command[command.index("--min-front-m") + 1], "0.27")
        self.assertTrue(profile.requires_preflight)
        self.assertTrue(profile.requires_measurement_truth)
        self.assertEqual(profile.measurement_truth_max_age_s, 3600.0)
        self.assertIn("human visual claim", " ".join(profile.goals))

    def test_m4_1_room_cruise_profile_is_bounded_and_proves_execution_surface(self):
        profile = hub._scenario_registry()["M4_1_room_cruise_quality_validator"]
        command = list(profile.command)

        self.assertEqual(command[command.index("--duration-s") + 1], "60.0")
        self.assertEqual(command[command.index("--v-max-mps") + 1], "0.30")
        self.assertEqual(command[command.index("--min-front-m") + 1], "0.27")
        self.assertTrue(profile.live)
        self.assertTrue(profile.requires_preflight)
        self.assertTrue(profile.requires_measurement_truth)
        self.assertEqual(profile.measurement_truth_max_age_s, 3600.0)
        self.assertIn("execution targets", " ".join(profile.goals))
        self.assertIn("canonical KIT0085", " ".join(profile.goals))
        self.assertIn("human observation", " ".join(profile.goals))

    def test_m0_and_m1_pose_reset_are_declared_before_strict_preflight(self):
        trust = hub._scenario_registry()["M0_measurement_trust_live"]
        profile = hub._scenario_registry()["M1_motion_baseline_live"]
        caster = hub._scenario_registry()["M1_1_caster_orientation_live"]
        repeated = hub._scenario_registry()["motion_command_fidelity_live"]

        self.assertTrue(trust.preflight_pose_reset)
        self.assertTrue(profile.preflight_pose_reset)
        self.assertTrue(caster.preflight_pose_reset)
        self.assertTrue(repeated.preflight_pose_reset)
        listed = {
            item["name"]: item
            for item in hub.list_profiles().get("profiles", [])
        }
        self.assertTrue(listed[trust.name]["preflight_pose_reset"])
        self.assertTrue(listed[profile.name]["preflight_pose_reset"])
        self.assertTrue(listed[caster.name]["preflight_pose_reset"])

    def test_run_profile_prepares_pose_reset_before_preflight_and_scenario(self):
        profile = hub.ScenarioProfile(
            name="unit_prepared_live",
            family="measurement_validation",
            description="unit",
            live=True,
            timeout_s=2.0,
            command=(sys.executable, "unit.py"),
            requires_preflight=True,
            preflight_pose_reset=True,
        )
        events = []
        with tempfile.TemporaryDirectory() as td:
            tests_dir = Path(td) / "logs" / "latest"
            run_dir = Path(td) / "logs" / "session_unit_prepared_live"
            tests_dir.mkdir(parents=True, exist_ok=True)
            run_dir.mkdir(parents=True, exist_ok=True)
            with mock.patch.dict(hub.SCENARIOS, {profile.name: profile}), mock.patch.object(
                hub, "AGENT_TESTS_DIR", tests_dir
            ), mock.patch.object(
                hub, "_new_hub_session_dir", return_value=run_dir
            ), mock.patch.object(
                hub, "publish_latest_alias", side_effect=self._local_latest_publisher(tests_dir)
            ), mock.patch.object(
                hub, "_publish_session_latest_aliases", return_value=[]
            ), mock.patch.object(
                hub, "LATEST_HUB_SUMMARY_PATH", tests_dir / "latest_hub_summary.json"
            ), mock.patch.object(
                hub, "LATEST_HUB_INCIDENT_PATH", tests_dir / "latest_hub_incident.json"
            ), mock.patch.object(
                hub, "LATEST_HUB_RUN_PATH", tests_dir / "latest_hub_run.json"
            ), mock.patch.object(
                hub, "LATEST_HUB_RUN_DIR_PATH", tests_dir / "latest_hub_run_dir.txt"
            ), mock.patch.object(
                hub,
                "_logger_lifecycle_snapshot",
                side_effect=[
                    {"status_version": 1, "logger_queue_depth": 0, "dropped_messages": 0, "write_errors": 0},
                    {"status_version": 2, "logger_queue_depth": 0, "dropped_messages": 0, "write_errors": 0},
                ],
            ), mock.patch.object(
                hub,
                "_prepare_pose_reset",
                side_effect=lambda: events.append("prepare") or {"ok": True},
            ), mock.patch.object(
                hub,
                "_run_preflight",
                side_effect=lambda **kwargs: events.append("preflight") or {"ok": True, "payload": {"ok": True}},
            ), mock.patch.object(
                hub,
                "_run_subprocess",
                side_effect=lambda *args, **kwargs: events.append("scenario") or {
                    "ok": True,
                    "timed_out": False,
                    "return_code": 0,
                    "duration_s": 0.1,
                    "stdout_tail": '{"success": true, "status": "PASS"}',
                    "stderr_tail": "",
                    "stdout_json": {"success": True, "status": "PASS"},
                    "stderr_json": None,
                },
            ):
                result = hub.run_profile(
                    profile.name,
                    auto_runtime=False,
                    archive_logs=False,
                )

        self.assertEqual(result.get("status"), "PASS")
        self.assertEqual(events, ["prepare", "preflight", "scenario"])

    def test_m3_primitive_profiles_use_common_minimum_speed(self):
        profiles = hub._scenario_registry()
        pivot_command = list(profiles["M3_motion_primitive_pivot_pair_live"].command)
        straight_arc_command = list(
            profiles["M3_motion_primitive_straight_arc_live"].command
        )

        self.assertEqual(
            pivot_command[pivot_command.index("--track-speed-mps") + 1],
            "0.150",
        )
        self.assertEqual(
            pivot_command[pivot_command.index("--case-gap-s") + 1],
            "2.0",
        )
        self.assertEqual(
            pivot_command[pivot_command.index("--poll-s") + 1],
            "0.04",
        )
        self.assertIn(
            "common 0.150 m/s minimum",
            " ".join(profiles["M3_motion_primitive_pivot_pair_live"].goals),
        )
        self.assertEqual(
            straight_arc_command[straight_arc_command.index("--poll-s") + 1],
            "0.08",
        )
        self.assertEqual(
            straight_arc_command[straight_arc_command.index("--case-gap-s") + 1],
            "2.0",
        )

    def test_m3_room_cruise_finalizer_rebuilds_stale_summary_from_base_artifact(self):
        from tools import M3_room_cruise_minoseg as m3_room

        tmp_dir = PROJECT_ROOT / "logs" / "session_unit_m3_room_finalize"
        shutil.rmtree(tmp_dir, ignore_errors=True)
        tmp_dir.mkdir(parents=True, exist_ok=True)
        try:
            base_result = tmp_dir / "latest_room_cruise_v2_result.json"
            base_summary = tmp_dir / "latest_room_cruise_v2_summary.json"
            base_payload = {
                "status": "PASS",
                "success": True,
                "summary": {"status": "PASS", "duration_s": 60.0},
                "samples": [{"ts": time.time(), "resolved_v": 0.0, "resolved_omega": 0.0}],
            }
            base_result.write_text(json.dumps(base_payload), encoding="utf-8")
            base_summary.write_text(json.dumps({"status": "PASS"}), encoding="utf-8")
            result_path = tmp_dir / "latest_M3_room_cruise_minoseg.json"
            summary_path = tmp_dir / "latest_M3_room_cruise_minoseg_summary.json"
            samples_path = tmp_dir / "M3_room_cruise_minoseg_samples.jsonl"
            incident_path = tmp_dir / "latest_M3_room_cruise_minoseg_incident.json"

            analyzed_result = {
                "schema": "M3_ROOM_CRUISE_MINOSEG_V1",
                "status": "INCONCLUSIVE",
                "success": False,
                "closure_verdict": "INSUFFICIENT_EVIDENCE",
                "metrics": {},
                "failed_gates": [],
                "inconclusive_gates": ["measurement_sufficiency"],
                "plain_summary_hu": "auto",
                "expected_live_motion_hu": "auto",
                "thresholds": {},
            }

            def fake_write(result, samples):
                summary_path.write_text(json.dumps({"status": result["status"]}), encoding="utf-8")
                result_path.write_text(json.dumps(result), encoding="utf-8")
                samples_path.write_text("", encoding="utf-8")
                incident_path.write_text("{}", encoding="utf-8")
                return {"status": result["status"]}

            with (
                mock.patch.object(m3_room, "RESULT_PATH", result_path),
                mock.patch.object(m3_room, "SUMMARY_PATH", summary_path),
                mock.patch.object(m3_room, "SAMPLES_PATH", samples_path),
                mock.patch.object(m3_room, "INCIDENT_PATH", incident_path),
                mock.patch.object(m3_room.cruise, "LATEST_RESULT", base_result),
                mock.patch.object(m3_room.cruise, "LATEST_SUMMARY", base_summary),
                mock.patch.object(m3_room, "analyze_samples", return_value=(analyzed_result, [{"sample_index": 0}])),
                mock.patch.object(m3_room, "write_artifacts", side_effect=fake_write),
            ):
                out = hub._ensure_m3_profile_artifacts(
                    profile=hub.SCENARIOS["M3_room_cruise_minoseg"],
                    run_result={"payload": {}},
                    started_wall_s=time.time() - 1.0,
                )

            self.assertTrue(out["ok"])
            self.assertEqual(out["action"], "rebuilt_from_room_cruise_v2_artifact")
            self.assertTrue(summary_path.exists())
            self.assertTrue(result_path.exists())
        finally:
            shutil.rmtree(tmp_dir, ignore_errors=True)

    def test_m3_human_follow_finalizer_rebuilds_stale_summary_from_samples(self):
        from tools import M3_emberkovetes_mozgasminoseg as m3_follow

        tmp_dir = PROJECT_ROOT / "logs" / "session_unit_m3_follow_finalize"
        shutil.rmtree(tmp_dir, ignore_errors=True)
        tmp_dir.mkdir(parents=True, exist_ok=True)
        try:
            result_path = tmp_dir / "latest_M3_emberkovetes_mozgasminoseg.json"
            summary_path = tmp_dir / "latest_M3_emberkovetes_mozgasminoseg_summary.json"
            samples_path = tmp_dir / "M3_emberkovetes_mozgasminoseg_samples.jsonl"
            incident_path = tmp_dir / "latest_M3_emberkovetes_mozgasminoseg_incident.json"
            base_result = tmp_dir / "latest_M3_emberkovetes_mozgasminoseg_follow_base.json"
            samples_path.write_text("{}\n", encoding="utf-8")
            base_result.write_text(json.dumps({"status": "PASS", "config": {"duration_s": 60.0}}), encoding="utf-8")

            analyzed_result = {
                "schema": "M3_EMBERKOVETES_MOZGASMINOSEG_V1",
                "status": "INCONCLUSIVE",
                "success": False,
                "metrics": {},
                "failed_gates": [],
                "inconclusive_gates": ["measurement_sufficiency"],
                "plain_summary_hu": "auto",
                "thresholds": {},
            }

            def fake_write(result, samples):
                summary_path.write_text(json.dumps({"status": result["status"]}), encoding="utf-8")
                result_path.write_text(json.dumps(result), encoding="utf-8")
                incident_path.write_text("{}", encoding="utf-8")
                return {"status": result["status"]}

            with (
                mock.patch.object(m3_follow, "RESULT_PATH", result_path),
                mock.patch.object(m3_follow, "SUMMARY_PATH", summary_path),
                mock.patch.object(m3_follow, "SAMPLES_PATH", samples_path),
                mock.patch.object(m3_follow, "INCIDENT_PATH", incident_path),
                mock.patch.object(m3_follow, "BASE_RESULT_PATH", base_result),
                mock.patch.object(m3_follow, "_read_jsonl", return_value=[{"sample_index": 0}]),
                mock.patch.object(m3_follow, "analyze_samples", return_value=(analyzed_result, [{"sample_index": 0}])),
                mock.patch.object(m3_follow, "write_artifacts", side_effect=fake_write),
            ):
                out = hub._ensure_m3_profile_artifacts(
                    profile=hub.SCENARIOS["M3_emberkovetes_mozgasminoseg"],
                    run_result={"payload": {}},
                    started_wall_s=time.time() - 1.0,
                )

            self.assertTrue(out["ok"])
            self.assertEqual(out["action"], "rebuilt_from_m3_samples")
            self.assertTrue(summary_path.exists())
            self.assertTrue(result_path.exists())
        finally:
            shutil.rmtree(tmp_dir, ignore_errors=True)

    def test_m3_human_follow_quality_profile_uses_existing_v2_route_and_artifacts(self):
        profile = hub.SCENARIOS["M3_emberkovetes_mozgasminoseg"]
        command = tuple(str(token) for token in profile.command)

        self.assertEqual(profile.family, "movement_quality")
        self.assertTrue(profile.live)
        self.assertTrue(profile.requires_measurement_truth)
        self.assertIn("tools/M3_emberkovetes_mozgasminoseg.py", command)
        self.assertIn("--sample-rate-hz", command)
        self.assertEqual(command[command.index("--sample-rate-hz") + 1], "10.0")
        self.assertTrue(any(path.endswith("_samples.jsonl") for path in profile.artifact_hints))
        self.assertTrue(any(path.endswith("_incident.json") for path in profile.artifact_hints))

    def test_hub_preserves_inconclusive_scenario_verdict(self):
        profile = hub.ScenarioProfile(
            name="synthetic_inconclusive",
            family="movement_quality",
            description="synthetic",
            live=False,
            timeout_s=1.0,
            command=(),
            requires_preflight=False,
        )

        verdict = hub._make_verdict(
            profile=profile,
            truth_gate_result=None,
            ekf_truth_gate_result=None,
            preflight_result=None,
            run_result={
                "run": {"return_code": 2, "timed_out": False, "duration_s": 0.1},
                "payload": {"status": "INCONCLUSIVE", "success": False},
            },
        )

        self.assertEqual(verdict["status"], "INCONCLUSIVE")

    def test_measurement_profiles_enforce_embedded_m0_mini_first_and_manual_pause(self):
        m0 = hub.SCENARIOS["M0_measurement_trust_live"]
        m1 = hub.SCENARIOS["M1_motion_baseline_live"]
        caster = hub.SCENARIOS["M1_1_caster_orientation_live"]

        self.assertEqual(m0.family, "measurement_validation")
        self.assertFalse(bool(m0.requires_measurement_truth))
        self.assertFalse(bool(m1.requires_measurement_truth))
        self.assertEqual(m1.measurement_truth_artifact_hint, "")
        for profile in (m0, m1):
            command = tuple(str(tok) for tok in profile.command)
            self.assertIn("tools/live_motion_measurement_validator.py", command)
            self.assertIn("--inter-case-pause-s", command)
            self.assertEqual(command[command.index("--inter-case-pause-s") + 1], "10.0")
            self.assertIn("--post-reset-ready-timeout-s", command)
        m0_command = tuple(str(tok) for tok in m0.command)
        m1_command = tuple(str(tok) for tok in m1.command)
        self.assertIn("--embedded-m0-mini", m1_command)
        self.assertEqual(m0_command[m0_command.index("--post-reset-ready-timeout-s") + 1], "20.0")
        self.assertEqual(
            m0_command[m0_command.index("--max-case-attempts") + 1],
            "5",
        )
        self.assertIn("--retry-all-trust-failures", m0_command)
        self.assertEqual(m0.timeout_s, 420.0)
        self.assertEqual(m1_command[m1_command.index("--post-reset-ready-timeout-s") + 1], "90.0")
        self.assertEqual(m1.timeout_s, 900.0)

        caster_command = tuple(str(tok) for tok in caster.command)
        self.assertIn("tools/caster_orientation_effect_validator.py", caster_command)
        self.assertIn("--operator-protocol-armed", caster_command)
        self.assertEqual(
            caster_command[caster_command.index("--inter-case-pause-s") + 1],
            "10.0",
        )
        self.assertTrue(caster.requires_preflight)
        self.assertFalse(caster.requires_measurement_truth)
        self.assertEqual(caster.timeout_s, 1800.0)
        self.assertIn("first 1.0 second", " ".join(caster.goals))
        self.assertTrue(m0.preflight_pose_reset)
        self.assertTrue(any("before preflight" in goal for goal in m0.goals))
        self.assertTrue(any("first movement" in goal for goal in m1.goals))
        self.assertTrue(any("FAIL stops" in goal for goal in m1.goals))

    def test_measurement_truth_gate_accepts_m0_measurement_trust_artifact(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "measurement_trust.json"
            path.write_text(
                json.dumps(
                    {
                        "success": True,
                        "phase": "M0",
                        "measurement_trust": {
                            "ok": True,
                            "sensor_surface": {
                                "encoder_cases": 2,
                                "imu_cases": 2,
                                "lidar_cases": 2,
                                "ekf_cases": 2,
                                "motor_pwm_cases": 1,
                            },
                        },
                    }
                ),
                encoding="utf-8",
            )
            profile = hub.ScenarioProfile(
                name="tmp_baseline",
                family="measurement_validation",
                description="tmp",
                live=True,
                timeout_s=1.0,
                command=(),
                requires_measurement_truth=True,
                measurement_truth_artifact_hint=str(path),
            )

            gate = hub._evaluate_measurement_truth_gate(profile)

        self.assertTrue(bool(gate.get("ok", False)))
        self.assertEqual(gate.get("errors"), [])

    def test_measurement_truth_gate_rejects_failed_m0_measurement_trust_artifact(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "measurement_trust.json"
            path.write_text(
                json.dumps(
                    {
                        "success": False,
                        "phase": "M0",
                        "measurement_trust": {
                            "ok": False,
                            "sensor_surface": {
                                "encoder_cases": 0,
                                "imu_cases": 1,
                                "lidar_cases": 1,
                                "ekf_cases": 1,
                                "motor_pwm_cases": 0,
                            },
                        },
                    }
                ),
                encoding="utf-8",
            )
            profile = hub.ScenarioProfile(
                name="tmp_baseline",
                family="measurement_validation",
                description="tmp",
                live=True,
                timeout_s=1.0,
                command=(),
                requires_measurement_truth=True,
                measurement_truth_artifact_hint=str(path),
            )

            gate = hub._evaluate_measurement_truth_gate(profile)

        self.assertFalse(bool(gate.get("ok", True)))
        self.assertIn("measurement_trust_failed", list(gate.get("errors") or []))
        self.assertIn("measurement_trust_not_ok", list(gate.get("errors") or []))

    def test_measurement_truth_gate_accepts_equivalent_m0_mini_artifact(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "measurement_trust_mini.json"
            path.write_text(
                json.dumps(
                    {
                        "success": True,
                        "phase": "M0_MINI",
                        "contract_id": hub.M0_MINI_CONTRACT_ID,
                        "measurement_trust": {
                            "ok": True,
                            "equivalent_to_full_m0": True,
                            "sensor_surface": {
                                "encoder_cases": 1,
                                "imu_cases": 1,
                                "lidar_cases": 1,
                                "ekf_cases": 1,
                                "motor_pwm_cases": 1,
                            },
                        },
                    }
                ),
                encoding="utf-8",
            )
            profile = hub.ScenarioProfile(
                name="tmp_baseline",
                family="measurement_validation",
                description="tmp",
                live=True,
                timeout_s=1.0,
                command=(),
                requires_measurement_truth=True,
                measurement_truth_artifact_hint=str(path),
            )

            gate = hub._evaluate_measurement_truth_gate(profile)

        self.assertTrue(bool(gate.get("ok", False)))
        self.assertEqual(gate.get("errors"), [])
        self.assertEqual(
            (gate.get("payload") or {}).get("contract_id"),
            hub.M0_MINI_CONTRACT_ID,
        )

    def test_measurement_truth_gate_rejects_uncontracted_m0_mini(self):
        payload = {
            "success": True,
            "phase": "M0_MINI",
            "contract_id": "WRONG",
            "measurement_trust": {
                "ok": True,
                "equivalent_to_full_m0": True,
                "sensor_surface": {
                    "encoder_cases": 1,
                    "imu_cases": 1,
                    "lidar_cases": 1,
                    "ekf_cases": 1,
                    "motor_pwm_cases": 1,
                },
            },
        }

        errors = hub._evaluate_measurement_trust_payload(payload)

        self.assertIn("measurement_trust_m0_mini_contract_mismatch", errors)

    def test_newer_m0_mini_fail_overrides_older_full_m0_pass(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            full_path = Path(tmpdir) / "full.json"
            mini_path = Path(tmpdir) / "mini.json"
            surface = {
                "encoder_cases": 1,
                "imu_cases": 1,
                "lidar_cases": 1,
                "ekf_cases": 1,
                "motor_pwm_cases": 1,
            }
            full_path.write_text(
                json.dumps(
                    {
                        "success": True,
                        "phase": "M0",
                        "measurement_trust": {
                            "ok": True,
                            "sensor_surface": surface,
                        },
                    }
                ),
                encoding="utf-8",
            )
            mini_path.write_text(
                json.dumps(
                    {
                        "success": False,
                        "phase": "M0_MINI",
                        "contract_id": hub.M0_MINI_CONTRACT_ID,
                        "failures": ["encoder_timing_gap"],
                        "measurement_trust": {
                            "ok": False,
                            "equivalent_to_full_m0": False,
                            "sensor_surface": surface,
                        },
                    }
                ),
                encoding="utf-8",
            )
            now = time.time()
            os.utime(full_path, (now - 10.0, now - 10.0))
            os.utime(mini_path, (now, now))
            profile = hub.ScenarioProfile(
                name="tmp_baseline",
                family="measurement_validation",
                description="tmp",
                live=True,
                timeout_s=1.0,
                command=(),
                requires_measurement_truth=True,
                measurement_truth_artifact_hint=str(full_path),
            )
            with mock.patch.object(
                hub, "LATEST_MEASUREMENT_TRUST_PATH", full_path
            ), mock.patch.object(hub, "LATEST_M0_MINI_TRUST_PATH", mini_path):
                gate = hub._evaluate_measurement_truth_gate(profile)

        self.assertFalse(bool(gate.get("ok", True)))
        self.assertEqual(Path(str(gate.get("path"))), mini_path)
        self.assertIn("measurement_trust_failed", list(gate.get("errors") or []))

    def test_kit0085_primitive_profiles_can_run_individually(self):
        expected_cases = {
            "kit0085_reverse_0p3m": "reverse_0p3m",
            "kit0085_arc_left": "arc_left",
            "kit0085_arc_right": "arc_right",
        }
        for name, case_name in expected_cases.items():
            profile = hub.SCENARIOS[name]
            command = tuple(str(tok) for tok in tuple(profile.command))
            self.assertEqual(profile.family, "hardware_validation")
            self.assertTrue(bool(profile.live))
            self.assertIn("tools/kit0085_motion_primitives_audit.py", command)
            self.assertIn("--cases", command)
            self.assertEqual(command[command.index("--cases") + 1], case_name)

    def test_usage_examples_place_run_flags_before_profile(self):
        payload = hub.list_profiles()
        examples = [str(item) for item in payload.get("usage_examples", [])]
        stop_examples = [item for item in examples if "--stop-runtime-after" in item]
        self.assertTrue(stop_examples)
        for example in stop_examples:
            self.assertIn("run --stop-runtime-after ", example)

    def test_motion_level_sequence_uses_only_canonical_milestone_profiles(self):
        seq = tuple(hub.SEQUENCE_PRESETS.get("motion_levels_M0_M4_1") or ())
        self.assertEqual(
            seq,
            (
                "M0_measurement_trust_live",
                "M1_motion_baseline_live",
                "M2_chassis_motion_dynamics_live",
                "M1_1_caster_orientation_live",
                "M3_room_cruise_unified_validator",
                "M4_1_room_cruise_quality_validator",
            ),
        )

    def test_speed_map_sequence_is_fail_closed_and_never_auto_promotes(self):
        seq = tuple(hub.SEQUENCE_PRESETS.get("speed_map_calibration") or ())
        self.assertEqual(
            seq,
            (
                "speed_map_calibration_acquisition_live",
                "speed_map_calibration_analyze_offline",
                "speed_map_quick_no_pi_live",
                "speed_map_quick_pi_live",
                "speed_map_candidate_M1_live",
                "speed_map_calibration_decision_offline",
            ),
        )
        decision = hub.SCENARIOS[seq[-1]]
        self.assertNotIn("--promote", decision.command)
        self.assertNotIn("M2_chassis_motion_dynamics_live", seq)
        candidate_m1 = hub.SCENARIOS["speed_map_candidate_M1_live"]
        self.assertIn(
            "unchanged embedded fail-closed M0-mini",
            " ".join(candidate_m1.goals),
        )

    def test_lidar_odometry_profiles_use_longer_preflight_timeout(self):
        straight = hub.SCENARIOS["straight_1m"]
        gentle_arc = hub.SCENARIOS["gentle_arc"]
        medium_arc = hub.SCENARIOS["medium_arc"]
        self.assertEqual(hub._preflight_timeout_for_profile(straight), 55.0)
        self.assertEqual(hub._preflight_timeout_for_profile(gentle_arc), 45.0)
        self.assertEqual(hub._preflight_timeout_for_profile(medium_arc), 45.0)

    def test_make_verdict_detects_preflight_failure(self):
        profile = hub.SCENARIOS["M1_motion_baseline_live"]
        verdict = hub._make_verdict(
            profile=profile,
            truth_gate_result=None,
            ekf_truth_gate_result=None,
            preflight_result={"ok": False, "payload": {"errors": ["no_lidar"]}},
            run_result=None,
        )
        self.assertEqual(verdict.get("status"), "FAIL")
        self.assertEqual(verdict.get("primary"), "PREFLIGHT_FAIL")
        self.assertIn("preflight_failed", str(verdict.get("reason")))

    def test_make_verdict_detects_ekf_truth_gate_failure(self):
        profile = hub.SCENARIOS["gentle_arc"]
        verdict = hub._make_verdict(
            profile=profile,
            truth_gate_result=None,
            ekf_truth_gate_result={"required": True, "ok": False, "errors": ["turn_primitive_actual_missing"]},
            preflight_result={"ok": True, "payload": {}},
            run_result={"run": {"return_code": 0, "timed_out": False}, "payload": {"success": True}},
        )
        self.assertEqual(verdict.get("status"), "FAIL")
        self.assertEqual(verdict.get("primary"), "EKF_TRUTH_GATE_FAIL")

    def test_ekf_truth_gate_accepts_valid_surface(self):
        profile = hub.SCENARIOS["gentle_arc"]
        run_result = {
            "payload": {
                "command_type": "follow_arc",
                "motion_execution_mode": "ARC_EXEC",
                "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                "truth_basis": {
                    "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                    "encoder_pose_active_samples": 0,
                    "lidar_odom_latest_age_s": 0.12,
                    "lidar_odom_latest_confidence": 0.91,
                    "turn_primitive_requested_vs_limited_match_ratio": 1.0,
                    "turn_primitive_limited_vs_executed_match_ratio": 1.0,
                    "turn_primitive_requested_vs_executed_match_ratio": 1.0,
                    "turn_primitive_executed_vs_actual_match_ratio": 1.0,
                },
                "turn_primitive_requested": "DIFF_ARC_GENTLE",
                "turn_primitive_limited": "DIFF_ARC_GENTLE",
                "turn_primitive_executed": "DIFF_ARC_GENTLE",
                "turn_primitive_actual": "DIFF_ARC_GENTLE",
                "arc_early_turning_present": True,
                "arc_no_late_snap_turn": True,
                "arc_inner_track_positive_ratio": 0.99,
                "arc_inner_track_positive_ratio_limit": 0.95,
                "arc_inner_track_min_mps": 0.03,
                "omega_tracking_error_rms_rad_s": 0.12,
                "omega_tracking_error_rms_limit_rad_s": 0.30,
                "curvature_error_rms_m_inv": 0.45,
                "curvature_error_rms_limit_m_inv": 1.40,
                "truth_surface_anchor": {
                    "used_arc_exec_anchor": True,
                    "command_type": "follow_arc",
                    "motion_execution_mode": "ARC_EXEC",
                },
            }
        }
        gate = hub._evaluate_ekf_truth_gate(profile, run_result)
        self.assertTrue(bool(gate.get("ok", False)))

    def test_pose_target_truth_gate_requires_local_planner_segment(self):
        profile = hub.SCENARIOS["pose_target"]
        base_payload = {
            "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
            "truth_basis": {
                "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                "encoder_pose_active_samples": 0,
                "lidar_odom_latest_age_s": 0.12,
                "lidar_odom_latest_confidence": 0.91,
                "turn_primitive_requested_vs_limited_match_ratio": 1.0,
                "turn_primitive_limited_vs_executed_match_ratio": 1.0,
                "turn_primitive_requested_vs_executed_match_ratio": 1.0,
                "turn_primitive_executed_vs_actual_match_ratio": 1.0,
            },
            "turn_primitive_requested": "STRAIGHT",
            "turn_primitive_limited": "STRAIGHT",
            "turn_primitive_executed": "STRAIGHT",
            "turn_primitive_actual": "STRAIGHT",
        }

        missing_gate = hub._evaluate_ekf_truth_gate(profile, {"payload": dict(base_payload)})
        self.assertFalse(bool(missing_gate.get("ok", True)))
        self.assertIn("pose_target_local_planner_segment_missing", list(missing_gate.get("errors") or []))

        payload_with_planner = dict(base_payload)
        payload_with_planner["motion_ownership"] = {
            "resolved_command_types_seen": ["go_to_pose", "local_planner_segment", "pose_closed_loop"],
        }
        ok_gate = hub._evaluate_ekf_truth_gate(profile, {"payload": payload_with_planner})
        self.assertTrue(bool(ok_gate.get("ok", False)))
        self.assertTrue(bool((ok_gate.get("surface_summary") or {}).get("local_planner_segment_observed", False)))

    def test_pose_target_turn_profile_uses_lateral_and_heading_target(self):
        profile = hub.SCENARIOS["pose_target_turn"]
        command = tuple(str(tok) for tok in tuple(profile.command))

        self.assertIn("--pose-target-lateral-m", command)
        self.assertEqual(command[command.index("--pose-target-lateral-m") + 1], "0.18")
        self.assertIn("--pose-target-heading-deg", command)
        self.assertEqual(command[command.index("--pose-target-heading-deg") + 1], "15.0")
        self.assertIn("--pose-target-heading-tolerance-deg", command)
        self.assertEqual(command[command.index("--pose-target-heading-tolerance-deg") + 1], "8.0")
        self.assertTrue(bool(profile.requires_ekf_truth_gate))

    def test_pose_target_sequence_profile_uses_two_alternating_segments(self):
        profile = hub.SCENARIOS["pose_target_sequence"]
        command = tuple(str(tok) for tok in tuple(profile.command))

        self.assertIn("--forward-repeats", command)
        self.assertEqual(command[command.index("--forward-repeats") + 1], "2")
        self.assertIn("--pose-target-alternate-sign", command)
        self.assertIn("--pose-target-continuous-sequence", command)
        self.assertIn("--forward-target-completion-ratio", command)
        self.assertEqual(command[command.index("--forward-target-completion-ratio") + 1], "0.90")
        self.assertIn("--pose-target-heading-deg", command)
        self.assertEqual(command[command.index("--pose-target-heading-deg") + 1], "9.0")
        self.assertTrue(bool(profile.requires_ekf_truth_gate))

    def test_pose_target_sequence_slow_profile_uses_slow_handoff_gate(self):
        profile = hub.SCENARIOS["pose_target_sequence_slow"]
        command = tuple(str(tok) for tok in tuple(profile.command))

        self.assertIn("--forward-speed-mps", command)
        self.assertEqual(command[command.index("--forward-speed-mps") + 1], "0.02")
        self.assertIn("--pose-target-omega-max-rad-s", command)
        self.assertEqual(command[command.index("--pose-target-omega-max-rad-s") + 1], "0.12")
        self.assertIn("--pose-target-continuous-sequence", command)
        self.assertTrue(bool(profile.requires_ekf_truth_gate))

    def test_follow_moving_target_profile_uses_new_script(self):
        profile = hub.SCENARIOS["follow_moving_target_sim"]
        command = tuple(str(tok) for tok in tuple(profile.command))

        self.assertEqual(profile.family, "amr_navigation")
        self.assertTrue(bool(profile.live))
        self.assertEqual(float(profile.timeout_s), 180.0)
        self.assertEqual(float(profile.preflight_clearance_m), 0.80)
        self.assertTrue(bool(profile.requires_preflight))
        self.assertFalse(bool(profile.requires_ekf_truth_gate))
        self.assertIn("tools/follow_moving_target_sim.py", command)
        self.assertIn("--duration-s", command)
        self.assertEqual(command[command.index("--duration-s") + 1], "60.0")
        self.assertIn("--command-rate-hz", command)
        self.assertEqual(command[command.index("--command-rate-hz") + 1], "5.0")

    def test_person_target_direction_profile_uses_new_live_script(self):
        profile = hub.SCENARIOS["person_target_direction_live"]
        command = tuple(str(tok) for tok in tuple(profile.command))

        self.assertEqual(profile.family, "amr_navigation")
        self.assertTrue(bool(profile.live))
        self.assertEqual(float(profile.timeout_s), 180.0)
        self.assertEqual(float(profile.preflight_clearance_m), 0.80)
        self.assertTrue(bool(profile.requires_preflight))
        self.assertIn("tools/person_target_direction_live.py", command)
        self.assertIn("--duration-s", command)
        self.assertEqual(command[command.index("--duration-s") + 1], "60.0")
        self.assertIn("--speed-scale", command)
        self.assertEqual(command[command.index("--speed-scale") + 1], "1.0")
        self.assertIn("--follow-distance-m", command)
        self.assertEqual(command[command.index("--follow-distance-m") + 1], "2.5")
        self.assertTrue(
            any(str(path).endswith("latest_person_target_direction_live_summary.json") for path in profile.artifact_hints)
        )

    def test_person_follow_camera_live_profile_is_bounded_camera_smoke(self):
        profile = hub.SCENARIOS["person_follow_camera_live"]
        command = tuple(str(tok) for tok in tuple(profile.command))

        self.assertEqual(profile.family, "amr_navigation")
        self.assertTrue(bool(profile.live))
        self.assertEqual(float(profile.timeout_s), 180.0)
        self.assertEqual(float(profile.preflight_clearance_m), 0.80)
        self.assertTrue(bool(profile.requires_preflight))
        self.assertFalse(bool(profile.requires_ekf_truth_gate))
        self.assertIn("tools/person_follow_camera_live.py", command)
        self.assertIn("--duration-s", command)
        self.assertEqual(command[command.index("--duration-s") + 1], "60.0")
        self.assertIn("--sample-rate-hz", command)
        self.assertEqual(command[command.index("--sample-rate-hz") + 1], "5.0")
        self.assertIn("--speed-scale", command)
        self.assertEqual(command[command.index("--speed-scale") + 1], "0.8")
        self.assertIn("--follow-distance-m", command)
        self.assertEqual(command[command.index("--follow-distance-m") + 1], "1.0")
        self.assertTrue(
            any(str(path).endswith("latest_person_follow_camera_live_summary.json") for path in profile.artifact_hints)
        )

    def test_person_follow_camera_live_v2_profile_is_strict_human_follow_gate(self):
        profile = hub.SCENARIOS["person_follow_camera_live_v2"]
        command = tuple(str(tok) for tok in tuple(profile.command))

        self.assertEqual(profile.family, "amr_navigation")
        self.assertTrue(bool(profile.live))
        self.assertEqual(float(profile.timeout_s), 180.0)
        self.assertEqual(float(profile.preflight_clearance_m), 0.80)
        self.assertTrue(bool(profile.requires_preflight))
        self.assertFalse(bool(profile.requires_ekf_truth_gate))
        self.assertIn("tools/person_follow_camera_live_v2.py", command)
        self.assertIn("--duration-s", command)
        self.assertEqual(command[command.index("--duration-s") + 1], "60.0")
        self.assertIn("--sample-rate-hz", command)
        self.assertEqual(command[command.index("--sample-rate-hz") + 1], "5.0")
        self.assertIn("--follow-distance-m", command)
        self.assertEqual(command[command.index("--follow-distance-m") + 1], "1.0")
        self.assertTrue(
            any(str(path).endswith("latest_person_follow_camera_live_v2_summary.json") for path in profile.artifact_hints)
        )

    def test_follow_forward_home_toggle_profile_uses_1p2m_10s_target_switch(self):
        profile = hub.SCENARIOS["follow_forward_home_toggle_live"]
        command = tuple(str(tok) for tok in tuple(profile.command))

        self.assertEqual(profile.family, "amr_navigation")
        self.assertTrue(bool(profile.live))
        self.assertEqual(float(profile.timeout_s), 180.0)
        self.assertEqual(float(profile.preflight_clearance_m), 0.80)
        self.assertTrue(bool(profile.requires_preflight))
        self.assertFalse(bool(profile.requires_ekf_truth_gate))
        self.assertIn("tools/follow_moving_target_sim.py", command)
        self.assertIn("--target-mode", command)
        self.assertEqual(command[command.index("--target-mode") + 1], "forward_home_toggle")
        self.assertIn("--target-forward-m", command)
        self.assertEqual(command[command.index("--target-forward-m") + 1], "1.20")
        self.assertIn("--target-toggle-interval-s", command)
        self.assertEqual(command[command.index("--target-toggle-interval-s") + 1], "10.0")
        self.assertIn("--duration-s", command)
        self.assertEqual(command[command.index("--duration-s") + 1], "60.0")

    def test_follow_triangle_profile_uses_0p8m_equilateral_target_points(self):
        profile = hub.SCENARIOS["follow_triangle_0p8_live"]
        command = tuple(str(tok) for tok in tuple(profile.command))

        self.assertEqual(profile.family, "amr_navigation")
        self.assertTrue(bool(profile.live))
        self.assertEqual(float(profile.timeout_s), 260.0)
        self.assertEqual(float(profile.preflight_clearance_m), 0.80)
        self.assertTrue(bool(profile.requires_preflight))
        self.assertFalse(bool(profile.requires_ekf_truth_gate))
        self.assertIn("tools/follow_moving_target_sim.py", command)
        self.assertIn("--target-mode", command)
        self.assertEqual(command[command.index("--target-mode") + 1], "triangle")
        self.assertIn("--target-triangle-side-m", command)
        self.assertEqual(command[command.index("--target-triangle-side-m") + 1], "0.80")
        self.assertIn("--target-triangle-interval-s", command)
        self.assertEqual(command[command.index("--target-triangle-interval-s") + 1], "30.0")
        self.assertIn("--target-triangle-direction", command)
        self.assertEqual(command[command.index("--target-triangle-direction") + 1], "auto")
        self.assertIn("--duration-s", command)
        self.assertEqual(command[command.index("--duration-s") + 1], "90.0")

    def test_follow_square_fast20_profile_scales_target_and_motion_speed(self):
        profile = hub.SCENARIOS["follow_square_0p8_right_fast20_live"]
        command = tuple(str(tok) for tok in tuple(profile.command))

        self.assertEqual(profile.family, "amr_navigation")
        self.assertTrue(bool(profile.live))
        self.assertEqual(float(profile.preflight_clearance_m), 0.80)
        self.assertTrue(bool(profile.requires_preflight))
        self.assertFalse(bool(profile.requires_ekf_truth_gate))
        self.assertIn("tools/follow_moving_target_sim.py", command)
        self.assertIn("--target-mode", command)
        self.assertEqual(command[command.index("--target-mode") + 1], "square")
        self.assertIn("--target-square-side-m", command)
        self.assertEqual(command[command.index("--target-square-side-m") + 1], "0.80")
        self.assertIn("--target-square-interval-s", command)
        self.assertEqual(command[command.index("--target-square-interval-s") + 1], "20.0")
        self.assertIn("--duration-s", command)
        self.assertEqual(command[command.index("--duration-s") + 1], "84.0")
        self.assertIn("--v-max-mps", command)
        self.assertEqual(command[command.index("--v-max-mps") + 1], "0.096")
        self.assertIn("--omega-max-rad-s", command)
        self.assertEqual(command[command.index("--omega-max-rad-s") + 1], "0.30")
        self.assertIn("--command-rate-hz", command)
        self.assertEqual(command[command.index("--command-rate-hz") + 1], "5.0")
        self.assertIn("--desired-distance-m", command)
        self.assertEqual(command[command.index("--desired-distance-m") + 1], "1.00")

    def test_pose_target_sequence_sharper_profile_uses_longer_arc_targets(self):
        profile = hub.SCENARIOS["pose_target_sequence_sharper"]
        command = tuple(str(tok) for tok in tuple(profile.command))

        self.assertIn("--forward-repeats", command)
        self.assertEqual(command[command.index("--forward-repeats") + 1], "2")
        self.assertIn("--pose-target-continuous-sequence", command)
        self.assertIn("--pose-target-lateral-m", command)
        self.assertEqual(command[command.index("--pose-target-lateral-m") + 1], "0.44")
        self.assertIn("--pose-target-heading-deg", command)
        self.assertEqual(command[command.index("--pose-target-heading-deg") + 1], "38.0")
        self.assertIn("--forward-speed-mps", command)
        self.assertEqual(command[command.index("--forward-speed-mps") + 1], "0.035")
        self.assertIn("--pose-target-omega-max-rad-s", command)
        self.assertEqual(command[command.index("--pose-target-omega-max-rad-s") + 1], "0.24")
        self.assertIn("--pose-target-positive-lateral-scale", command)
        self.assertEqual(command[command.index("--pose-target-positive-lateral-scale") + 1], "1.16")
        self.assertIn("--pose-target-negative-omega-scale", command)
        self.assertEqual(command[command.index("--pose-target-negative-omega-scale") + 1], "1.00")
        self.assertIn("--pose-target-negative-speed-scale", command)
        self.assertEqual(command[command.index("--pose-target-negative-speed-scale") + 1], "0.92")
        self.assertIn("--forward-distance-m", command)
        self.assertEqual(command[command.index("--forward-distance-m") + 1], "0.74")
        self.assertIn("--forward-target-completion-ratio", command)
        self.assertEqual(command[command.index("--forward-target-completion-ratio") + 1], "0.62")
        self.assertIn("--pose-target-handoff-completion-ratio", command)
        self.assertEqual(command[command.index("--pose-target-handoff-completion-ratio") + 1], "0.64")
        self.assertTrue(bool(profile.requires_ekf_truth_gate))

    def test_pose_target_sequence_sharper_1p5_profile_uses_larger_longer_arcs(self):
        profile = hub.SCENARIOS["pose_target_sequence_sharper_1p5"]
        command = tuple(str(tok) for tok in tuple(profile.command))

        self.assertIn("--forward-repeats", command)
        self.assertEqual(command[command.index("--forward-repeats") + 1], "2")
        self.assertIn("--pose-target-continuous-sequence", command)
        self.assertIn("--pose-target-heading-deg", command)
        self.assertEqual(command[command.index("--pose-target-heading-deg") + 1], "57.0")
        self.assertIn("--pose-target-lateral-m", command)
        self.assertEqual(command[command.index("--pose-target-lateral-m") + 1], "0.60")
        self.assertIn("--forward-distance-m", command)
        self.assertEqual(command[command.index("--forward-distance-m") + 1], "0.92")
        self.assertIn("--forward-max-runtime-s", command)
        self.assertEqual(command[command.index("--forward-max-runtime-s") + 1], "26.0")
        self.assertIn("--pose-target-handoff-completion-ratio", command)
        self.assertEqual(command[command.index("--pose-target-handoff-completion-ratio") + 1], "0.86")
        self.assertIn("--pose-target-omega-max-rad-s", command)
        self.assertEqual(command[command.index("--pose-target-omega-max-rad-s") + 1], "0.48")
        self.assertTrue(bool(profile.requires_ekf_truth_gate))

    def test_pose_target_turn_truth_gate_requires_arc_and_heading_error(self):
        profile = hub.SCENARIOS["pose_target_turn"]
        base_payload = {
            "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
            "truth_basis": {
                "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                "encoder_pose_active_samples": 0,
                "lidar_odom_latest_age_s": 0.12,
                "lidar_odom_latest_confidence": 0.91,
                "turn_primitive_requested_vs_limited_match_ratio": 1.0,
                "turn_primitive_limited_vs_executed_match_ratio": 1.0,
                "turn_primitive_requested_vs_executed_match_ratio": 1.0,
                "turn_primitive_executed_vs_actual_match_ratio": 1.0,
            },
            "motion_ownership": {
                "resolved_command_types_seen": ["local_planner_segment", "soft_stop"],
            },
            "target_heading_error_deg": 5.8,
            "heading_change_deg": 10.1,
            "pose_target": {"heading_tolerance_deg": 8.0},
        }

        straight_payload = dict(base_payload)
        straight_payload.update(
            {
                "turn_primitive_requested": "STRAIGHT",
                "turn_primitive_limited": "STRAIGHT",
                "turn_primitive_executed": "STRAIGHT",
                "turn_primitive_actual": "STRAIGHT",
            }
        )
        straight_gate = hub._evaluate_ekf_truth_gate(profile, {"payload": straight_payload})
        self.assertFalse(bool(straight_gate.get("ok", True)))
        self.assertIn("pose_target_turn_primitive_straight", list(straight_gate.get("errors") or []))

        ok_payload = dict(base_payload)
        ok_payload.update(
            {
                "turn_primitive_requested": "DIFF_ARC_GENTLE",
                "turn_primitive_limited": "DIFF_ARC_GENTLE",
                "turn_primitive_executed": "DIFF_ARC_GENTLE",
                "turn_primitive_actual": "DIFF_ARC_GENTLE",
            }
        )
        ok_gate = hub._evaluate_ekf_truth_gate(profile, {"payload": ok_payload})
        self.assertTrue(bool(ok_gate.get("ok", False)))
        self.assertAlmostEqual(float((ok_gate.get("surface_summary") or {}).get("target_heading_error_deg")), 5.8)
        self.assertAlmostEqual(float((ok_gate.get("surface_summary") or {}).get("target_heading_gate_margin_deg")), 0.5)
        self.assertAlmostEqual(float((ok_gate.get("surface_summary") or {}).get("target_heading_limit_deg")), 8.5)
        self.assertAlmostEqual(float((ok_gate.get("surface_summary") or {}).get("heading_change_deg")), 10.1)

        edge_payload = dict(ok_payload)
        edge_payload["target_heading_error_deg"] = 8.03
        edge_gate = hub._evaluate_ekf_truth_gate(profile, {"payload": edge_payload})
        self.assertTrue(bool(edge_gate.get("ok", False)))

        bad_heading_payload = dict(ok_payload)
        bad_heading_payload["target_heading_error_deg"] = 9.2
        bad_heading_gate = hub._evaluate_ekf_truth_gate(profile, {"payload": bad_heading_payload})
        self.assertFalse(bool(bad_heading_gate.get("ok", True)))
        self.assertTrue(
            any(
                str(err).startswith("pose_target_turn_heading_error_gt_tolerance")
                for err in list(bad_heading_gate.get("errors") or [])
            )
        )

        weak_turn_payload = dict(ok_payload)
        weak_turn_payload["heading_change_deg"] = 5.0
        weak_turn_gate = hub._evaluate_ekf_truth_gate(profile, {"payload": weak_turn_payload})
        self.assertFalse(bool(weak_turn_gate.get("ok", True)))
        self.assertTrue(
            any(
                str(err).startswith("pose_target_turn_heading_change_lt_min")
                for err in list(weak_turn_gate.get("errors") or [])
            )
        )

    def test_pose_target_sequence_truth_gate_requires_two_planner_arc_segments(self):
        profile = hub.SCENARIOS["pose_target_sequence"]

        def segment(
            name: str,
            heading: float = 4.0,
            *,
            planner: bool = True,
            continuous_handoff: bool = False,
            normal_stop: bool = True,
            heading_error: float = 3.2,
        ) -> dict:
            return {
                "test_name": name,
                "success": True,
                "continuous_handoff": bool(continuous_handoff),
                "normal_stop_used": bool(normal_stop),
                "actual_runtime_s": 4.0,
                "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                "truth_basis": {
                    "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                    "encoder_pose_active_samples": 0,
                    "lidar_odom_latest_age_s": 0.12,
                    "lidar_odom_latest_confidence": 0.91,
                    "turn_primitive_requested_vs_limited_match_ratio": 1.0,
                    "turn_primitive_limited_vs_executed_match_ratio": 1.0,
                    "turn_primitive_requested_vs_executed_match_ratio": 1.0,
                    "turn_primitive_executed_vs_actual_match_ratio": 1.0,
                },
                "motion_ownership": {
                    "resolved_command_types_seen": (
                        ["local_planner_segment", "soft_stop"] if planner else ["pose_closed_loop"]
                    ),
                },
                "target_heading_error_deg": float(heading_error),
                "heading_change_deg": heading,
                "estimated_distance_m": 0.50,
                "arc_track_ratio": 1.3,
                "pose_target": {"heading_tolerance_deg": 8.0},
                "turn_primitive_requested": "DIFF_ARC_GENTLE",
                "turn_primitive_limited": "DIFF_ARC_GENTLE",
                "turn_primitive_executed": "DIFF_ARC_GENTLE",
                "turn_primitive_actual": "DIFF_ARC_GENTLE",
            }

        payload = {
            "subtests": [
                segment("short_forward_a", continuous_handoff=True, normal_stop=False),
                segment("short_forward_b", heading=-6.5, continuous_handoff=False, normal_stop=True),
            ]
        }
        gate = hub._evaluate_ekf_truth_gate(profile, {"payload": payload})
        self.assertTrue(bool(gate.get("ok", False)))

        one_segment_gate = hub._evaluate_ekf_truth_gate(
            profile,
            {"payload": {"subtests": [segment("short_forward_a", normal_stop=True)]}},
        )
        self.assertFalse(bool(one_segment_gate.get("ok", True)))
        self.assertIn("pose_target_sequence_segments_lt_min:1<2", list(one_segment_gate.get("errors") or []))

        no_planner_payload = {
            "subtests": [
                segment("short_forward_a", continuous_handoff=True, normal_stop=False),
                segment("short_forward_b", planner=False, normal_stop=True),
            ]
        }
        no_planner_gate = hub._evaluate_ekf_truth_gate(profile, {"payload": no_planner_payload})
        self.assertFalse(bool(no_planner_gate.get("ok", True)))
        self.assertIn("pose_target_sequence_segment_2_local_planner_missing", list(no_planner_gate.get("errors") or []))

        missing_handoff_payload = {
            "subtests": [
                segment("short_forward_a", continuous_handoff=False, normal_stop=True),
                segment("short_forward_b", normal_stop=True),
            ]
        }
        missing_handoff_gate = hub._evaluate_ekf_truth_gate(profile, {"payload": missing_handoff_payload})
        self.assertFalse(bool(missing_handoff_gate.get("ok", True)))
        self.assertIn(
            "pose_target_sequence_segment_1_continuous_handoff_missing",
            list(missing_handoff_gate.get("errors") or []),
        )

        weak_heading_payload = {
            "subtests": [
                segment("short_forward_a", continuous_handoff=True, normal_stop=False),
                segment("short_forward_b", heading=3.0, normal_stop=True),
            ]
        }
        weak_heading_gate = hub._evaluate_ekf_truth_gate(profile, {"payload": weak_heading_payload})
        self.assertFalse(bool(weak_heading_gate.get("ok", True)))
        self.assertTrue(
            any(
                str(err).startswith("pose_target_sequence_segment_2_heading_change_lt_min")
                for err in list(weak_heading_gate.get("errors") or [])
            )
        )

        sharper_profile = hub.SCENARIOS["pose_target_sequence_sharper"]
        sharper_weak_payload = {
            "subtests": [
                segment("short_forward_a", heading=5.5, continuous_handoff=True, normal_stop=False),
                segment("short_forward_b", heading=-6.2, normal_stop=True),
            ]
        }
        sharper_weak_gate = hub._evaluate_ekf_truth_gate(sharper_profile, {"payload": sharper_weak_payload})
        self.assertFalse(bool(sharper_weak_gate.get("ok", True)))
        self.assertTrue(
            any(
                str(err).startswith("pose_target_sequence_segment_1_heading_change_lt_min")
                for err in list(sharper_weak_gate.get("errors") or [])
            )
        )

        sharper_ok_payload = {
            "subtests": [
                segment(
                    "short_forward_a",
                    heading=6.1,
                    continuous_handoff=True,
                    normal_stop=False,
                    heading_error=19.2,
                ),
                segment("short_forward_b", heading=-6.2, normal_stop=True),
            ]
        }
        sharper_ok_gate = hub._evaluate_ekf_truth_gate(sharper_profile, {"payload": sharper_ok_payload})
        self.assertTrue(bool(sharper_ok_gate.get("ok", False)))
        self.assertEqual((sharper_ok_gate.get("surface_summary") or {}).get("segment_count"), 2)

        sharper_1p5_profile = hub.SCENARIOS["pose_target_sequence_sharper_1p5"]
        sharper_1p5_ok_payload = {
            "subtests": [
                segment(
                    "short_forward_a",
                    heading=9.2,
                    continuous_handoff=True,
                    normal_stop=False,
                    heading_error=20.0,
                ),
                segment("short_forward_b", heading=-9.4, normal_stop=True, heading_error=22.0),
            ]
        }
        for item in sharper_1p5_ok_payload["subtests"]:
            item["estimated_distance_m"] = 0.58
            item["actual_runtime_s"] = 8.0
            item["pose_target"] = {"heading_tolerance_deg": 42.0}
        sharper_1p5_ok_gate = hub._evaluate_ekf_truth_gate(sharper_1p5_profile, {"payload": sharper_1p5_ok_payload})
        self.assertTrue(bool(sharper_1p5_ok_gate.get("ok", False)))
        self.assertEqual(
            (sharper_1p5_ok_gate.get("surface_summary") or {}).get("min_heading_change_deg"),
            9.0,
        )

        sharper_1p5_weak_payload = {
            "subtests": [
                segment("short_forward_a", heading=8.8, continuous_handoff=True, normal_stop=False),
                segment("short_forward_b", heading=-9.4, normal_stop=True),
            ]
        }
        for item in sharper_1p5_weak_payload["subtests"]:
            item["estimated_distance_m"] = 0.58
        sharper_1p5_weak_gate = hub._evaluate_ekf_truth_gate(sharper_1p5_profile, {"payload": sharper_1p5_weak_payload})
        self.assertFalse(bool(sharper_1p5_weak_gate.get("ok", True)))
        self.assertTrue(
            any(
                str(err).startswith("pose_target_sequence_segment_1_heading_change_lt_min")
                for err in list(sharper_1p5_weak_gate.get("errors") or [])
            )
        )

        sharper_slow_runtime_payload = {
            "subtests": [
                segment("short_forward_a", heading=6.1, continuous_handoff=True, normal_stop=False),
                segment("short_forward_b", heading=-6.2, normal_stop=True),
            ]
        }
        sharper_slow_runtime_payload["subtests"][0]["actual_runtime_s"] = 34.0
        sharper_runtime_gate = hub._evaluate_ekf_truth_gate(sharper_profile, {"payload": sharper_slow_runtime_payload})
        self.assertFalse(bool(sharper_runtime_gate.get("ok", True)))
        self.assertTrue(
            any(
                str(err).startswith("pose_target_sequence_segment_1_runtime_gt_max")
                for err in list(sharper_runtime_gate.get("errors") or [])
            )
        )

        sharper_weak_curvature_payload = {
            "subtests": [
                segment("short_forward_a", heading=6.1, continuous_handoff=True, normal_stop=False),
                segment("short_forward_b", heading=-6.2, normal_stop=True),
            ]
        }
        sharper_weak_curvature_payload["subtests"][0]["arc_track_ratio"] = 1.01
        sharper_curvature_gate = hub._evaluate_ekf_truth_gate(sharper_profile, {"payload": sharper_weak_curvature_payload})
        self.assertFalse(bool(sharper_curvature_gate.get("ok", True)))
        self.assertTrue(
            any(
                str(err).startswith("pose_target_sequence_segment_1_arc_track_ratio_lt_min")
                for err in list(sharper_curvature_gate.get("errors") or [])
            )
        )

        slow_profile = hub.SCENARIOS["pose_target_sequence_slow"]
        slow_straight_payload = {
            "subtests": [
                segment(
                    "short_forward_a",
                    heading=0.3,
                    continuous_handoff=True,
                    normal_stop=False,
                ),
                segment("short_forward_b", heading=0.2, normal_stop=True),
            ]
        }
        for item in slow_straight_payload["subtests"]:
            item["turn_primitive_actual"] = "STRAIGHT"
            item["estimated_distance_m"] = 0.18
        slow_gate = hub._evaluate_ekf_truth_gate(slow_profile, {"payload": slow_straight_payload})
        self.assertTrue(bool(slow_gate.get("ok", False)))
        self.assertEqual((slow_gate.get("surface_summary") or {}).get("segment_count"), 2)

    def test_ekf_truth_gate_requires_arc_exec_anchor_for_arc_profiles(self):
        profile = hub.SCENARIOS["medium_arc"]
        run_result = {
            "payload": {
                "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                "truth_basis": {
                    "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                    "encoder_pose_active_samples": 0,
                    "lidar_odom_latest_age_s": 0.08,
                    "lidar_odom_latest_confidence": 0.95,
                    "turn_primitive_requested_vs_limited_match_ratio": 1.0,
                    "turn_primitive_limited_vs_executed_match_ratio": 1.0,
                    "turn_primitive_requested_vs_executed_match_ratio": 1.0,
                    "turn_primitive_executed_vs_actual_match_ratio": 1.0,
                },
                "turn_primitive_requested": "DIFF_ARC_MEDIUM",
                "turn_primitive_limited": "DIFF_ARC_MEDIUM",
                "turn_primitive_executed": "DIFF_ARC_MEDIUM",
                "turn_primitive_actual": "DIFF_ARC_MEDIUM",
            }
        }
        gate = hub._evaluate_ekf_truth_gate(profile, run_result)
        self.assertFalse(bool(gate.get("ok", True)))
        self.assertIn("arc_exec_anchor_missing", list(gate.get("errors") or []))

    def test_ekf_truth_gate_for_arc_profile_uses_arc_anchor_surface(self):
        profile = hub.SCENARIOS["medium_arc"]
        run_result = {
            "payload": {
                "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                "truth_basis": {
                    "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                    "encoder_pose_active_samples": 0,
                    "lidar_odom_latest_age_s": 0.05,
                    "lidar_odom_latest_confidence": 0.97,
                    "turn_primitive_requested_vs_limited_match_ratio": 1.0,
                    "turn_primitive_limited_vs_executed_match_ratio": 1.0,
                    "turn_primitive_requested_vs_executed_match_ratio": 1.0,
                    "turn_primitive_executed_vs_actual_match_ratio": 1.0,
                },
                "turn_primitive_requested": "STRAIGHT",
                "turn_primitive_limited": "STRAIGHT",
                "turn_primitive_executed": "STRAIGHT",
                "turn_primitive_actual": "STRAIGHT",
                "subtests": [
                    {
                        "command_type": "follow_arc",
                        "motion_execution_mode": "ARC_EXEC",
                        "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                        "truth_basis": {
                            "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                            "encoder_pose_active_samples": 0,
                            "lidar_odom_latest_age_s": 0.05,
                            "lidar_odom_latest_confidence": 0.97,
                        },
                        "turn_primitive_requested": "STRAIGHT",
                        "turn_primitive_limited": "STRAIGHT",
                        "turn_primitive_executed": "STRAIGHT",
                        "turn_primitive_actual": "DIFF_ARC_GENTLE",
                        "truth_surface_anchor": {
                            "used_arc_exec_anchor": True,
                            "command_type": "follow_arc",
                            "motion_execution_mode": "ARC_EXEC",
                        },
                    }
                ],
            }
        }
        gate = hub._evaluate_ekf_truth_gate(profile, run_result)
        self.assertFalse(bool(gate.get("ok", True)))
        errors = list(gate.get("errors") or [])
        self.assertIn("arc_turn_primitive_requested_invalid:STRAIGHT", errors)
        summary = dict(gate.get("surface_summary") or {})
        self.assertEqual(summary.get("turn_primitive_actual"), "DIFF_ARC_GENTLE")

    def test_ekf_truth_gate_for_medium_arc_rejects_zero_inner_track(self):
        profile = hub.SCENARIOS["medium_arc"]
        run_result = {
            "payload": {
                "command_type": "follow_arc",
                "motion_execution_mode": "ARC_EXEC",
                "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                "truth_basis": {
                    "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                    "encoder_pose_active_samples": 0,
                    "lidar_odom_latest_age_s": 0.07,
                    "lidar_odom_latest_confidence": 0.96,
                    "turn_primitive_requested_vs_limited_match_ratio": 1.0,
                    "turn_primitive_limited_vs_executed_match_ratio": 1.0,
                    "turn_primitive_requested_vs_executed_match_ratio": 1.0,
                    "turn_primitive_executed_vs_actual_match_ratio": 1.0,
                },
                "turn_primitive_requested": "DIFF_ARC_MEDIUM",
                "turn_primitive_limited": "DIFF_ARC_MEDIUM",
                "turn_primitive_executed": "DIFF_ARC_MEDIUM",
                "turn_primitive_actual": "DIFF_ARC_MEDIUM",
                "arc_early_turning_present": True,
                "arc_no_late_snap_turn": True,
                "arc_inner_track_positive_ratio": 0.99,
                "arc_inner_track_positive_ratio_limit": 0.95,
                "arc_inner_track_min_mps": 0.0,
                "omega_tracking_error_rms_rad_s": 0.11,
                "omega_tracking_error_rms_limit_rad_s": 0.30,
                "curvature_error_rms_m_inv": 0.35,
                "curvature_error_rms_limit_m_inv": 1.40,
                "truth_surface_anchor": {
                    "used_arc_exec_anchor": True,
                    "command_type": "follow_arc",
                    "motion_execution_mode": "ARC_EXEC",
                },
            }
        }
        gate = hub._evaluate_ekf_truth_gate(profile, run_result)
        self.assertFalse(bool(gate.get("ok", True)))
        errors = list(gate.get("errors") or [])
        self.assertTrue(
            any(str(err).startswith("medium_arc_inner_track_zero_or_negative:") for err in errors),
            msg=f"unexpected errors: {errors}",
        )

    def test_ekf_truth_gate_lidar_odometry_requires_lidar_apply_evidence(self):
        profile = hub.SCENARIOS["straight_1m"]
        run_result = {
            "payload": {
                "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                "truth_basis": {
                    "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                    "odometry_mode": "LIDAR_FIRST",
                    "encoder_pose_active_samples": 0,
                    "lidar_odom_applied_samples": 0,
                    "lidar_odom_latest_age_s": 0.09,
                    "lidar_odom_latest_confidence": 0.93,
                    "turn_primitive_requested_vs_limited_match_ratio": 1.0,
                    "turn_primitive_limited_vs_executed_match_ratio": 1.0,
                    "turn_primitive_requested_vs_executed_match_ratio": 1.0,
                    "turn_primitive_executed_vs_actual_match_ratio": 1.0,
                },
                "turn_primitive_requested": "STRAIGHT",
                "turn_primitive_limited": "STRAIGHT",
                "turn_primitive_executed": "STRAIGHT",
                "turn_primitive_actual": "STRAIGHT",
            }
        }
        gate = hub._evaluate_ekf_truth_gate(profile, run_result)
        self.assertFalse(bool(gate.get("ok", True)))
        self.assertIn("lidar_odom_application_evidence_missing", list(gate.get("errors") or []))

    def test_ekf_truth_gate_lidar_odometry_accepts_lidar_accepted_total(self):
        profile = hub.SCENARIOS["straight_1m"]
        run_result = {
            "payload": {
                "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                "truth_basis": {
                    "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                    "odometry_mode": "LIDAR_FIRST",
                    "encoder_pose_active_samples": 0,
                    "lidar_odom_applied_samples": 0,
                    "lidar_odom_accepted_total": 6,
                    "lidar_odom_total_samples": 10,
                    "lidar_odom_latest_age_s": 0.09,
                    "lidar_odom_latest_confidence": 0.93,
                    "turn_primitive_requested_vs_limited_match_ratio": 1.0,
                    "turn_primitive_limited_vs_executed_match_ratio": 1.0,
                    "turn_primitive_requested_vs_executed_match_ratio": 1.0,
                    "turn_primitive_executed_vs_actual_match_ratio": 1.0,
                },
                "turn_primitive_requested": "STRAIGHT",
                "turn_primitive_limited": "STRAIGHT",
                "turn_primitive_executed": "STRAIGHT",
                "turn_primitive_actual": "STRAIGHT",
            }
        }
        gate = hub._evaluate_ekf_truth_gate(profile, run_result)
        self.assertTrue(bool(gate.get("ok", False)))

    def test_ekf_truth_gate_rejects_lidar_observation_contract_error(self):
        profile = hub.SCENARIOS["straight_1m"]
        run_result = {
            "payload": {
                "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                "truth_basis": {
                    "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                    "odometry_mode": "LIDAR_FIRST",
                    "encoder_pose_active_samples": 0,
                    "lidar_odom_applied_samples": 1,
                    "lidar_odom_accepted_total": 1,
                    "lidar_odom_total_samples": 1,
                    "lidar_odom_latest_age_s": 0.09,
                    "lidar_odom_latest_confidence": 0.93,
                    "lidar_odom_applied_missing_measurement_id_samples": 1,
                    "lidar_observation_contract_errors": [
                        "applied_measurement_id_missing"
                    ],
                    "turn_primitive_requested_vs_limited_match_ratio": 1.0,
                    "turn_primitive_limited_vs_executed_match_ratio": 1.0,
                    "turn_primitive_requested_vs_executed_match_ratio": 1.0,
                    "turn_primitive_executed_vs_actual_match_ratio": 1.0,
                },
                "turn_primitive_requested": "STRAIGHT",
                "turn_primitive_limited": "STRAIGHT",
                "turn_primitive_executed": "STRAIGHT",
                "turn_primitive_actual": "STRAIGHT",
            }
        }

        gate = hub._evaluate_ekf_truth_gate(profile, run_result)

        self.assertFalse(bool(gate.get("ok", True)))
        errors = list(gate.get("errors") or [])
        self.assertIn("lidar_applied_measurement_id_missing:1", errors)
        self.assertIn(
            "lidar_observation_contract_violation:applied_measurement_id_missing",
            errors,
        )

    def test_ekf_truth_gate_lidar_odometry_rejects_too_low_update_rate(self):
        profile = hub.SCENARIOS["straight_1m"]
        run_result = {
            "payload": {
                "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                "truth_basis": {
                    "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                    "odometry_mode": "LIDAR_FIRST",
                    "encoder_pose_active_samples": 0,
                    "lidar_odom_applied_samples": 2,
                    "lidar_odom_accepted_total": 2,
                    "lidar_odom_total_samples": 400,
                    "lidar_odom_update_rate_ratio": 0.0,
                    "lidar_odom_latest_age_s": 0.09,
                    "lidar_odom_latest_confidence": 0.93,
                    "turn_primitive_requested_vs_limited_match_ratio": 1.0,
                    "turn_primitive_limited_vs_executed_match_ratio": 1.0,
                    "turn_primitive_requested_vs_executed_match_ratio": 1.0,
                    "turn_primitive_executed_vs_actual_match_ratio": 1.0,
                },
                "turn_primitive_requested": "STRAIGHT",
                "turn_primitive_limited": "STRAIGHT",
                "turn_primitive_executed": "STRAIGHT",
                "turn_primitive_actual": "STRAIGHT",
            }
        }
        gate = hub._evaluate_ekf_truth_gate(profile, run_result)
        self.assertFalse(bool(gate.get("ok", True)))
        self.assertTrue(
            any(str(err).startswith("lidar_odom_update_rate_ratio_lt_min:") for err in list(gate.get("errors") or []))
        )

    def test_ekf_truth_gate_lidar_odometry_rejects_missing_streak_over_limit(self):
        profile = hub.SCENARIOS["straight_1m"]
        run_result = {
            "payload": {
                "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                "truth_basis": {
                    "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                    "odometry_mode": "LIDAR_FIRST",
                    "encoder_pose_active_samples": 0,
                    "lidar_odom_applied_samples": 10,
                    "lidar_odom_accepted_total": 10,
                    "lidar_odom_total_samples": 50,
                    "lidar_odom_update_rate_ratio": 0.2,
                    "lidar_odom_missing_streak_max": 30,
                    "lidar_odom_missing_streak_limit": 24,
                    "lidar_odom_localization_healthy_ratio": 0.8,
                    "lidar_odom_latest_age_s": 0.09,
                    "lidar_odom_latest_confidence": 0.93,
                    "turn_primitive_requested_vs_limited_match_ratio": 1.0,
                    "turn_primitive_limited_vs_executed_match_ratio": 1.0,
                    "turn_primitive_requested_vs_executed_match_ratio": 1.0,
                    "turn_primitive_executed_vs_actual_match_ratio": 1.0,
                },
                "turn_primitive_requested": "STRAIGHT",
                "turn_primitive_limited": "STRAIGHT",
                "turn_primitive_executed": "STRAIGHT",
                "turn_primitive_actual": "STRAIGHT",
            }
        }
        gate = hub._evaluate_ekf_truth_gate(profile, run_result)
        self.assertFalse(bool(gate.get("ok", True)))
        self.assertTrue(
            any(str(err).startswith("lidar_odom_missing_streak_gt_limit:") for err in list(gate.get("errors") or []))
        )

    def test_run_sequence_fails_fast_on_gate_failure(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            tests_dir = root / "logs" / "latest"
            run_dir = root / "logs" / "session_unit_sequence_fail"
            tests_dir.mkdir(parents=True, exist_ok=True)
            run_dir.mkdir(parents=True, exist_ok=True)

            latest_summary = tests_dir / "latest_hub_sequence_summary.json"
            latest_run = tests_dir / "latest_hub_sequence_run.json"

            call_order = []
            fake_results = {
                "M0_measurement_trust_live": {
                    "status": "PASS",
                    "profile": "M0_measurement_trust_live",
                    "duration_s": 1.2,
                    "verdict": {"primary": "PASS", "reason": "scenario completed"},
                    "measurement_truth_gate_ok": True,
                    "ekf_truth_gate_ok": True,
                    "summary_path": "logs/session_fake_a/summary.json",
                    "incident_path": "logs/session_fake_a/incident_bundle.json",
                    "run_dir": "logs/session_fake_a",
                },
                "gentle_arc": {
                    "status": "FAIL",
                    "profile": "gentle_arc",
                    "duration_s": 0.9,
                    "verdict": {"primary": "EKF_TRUTH_GATE_FAIL", "reason": "ekf_truth_gate_failed:['x']"},
                    "measurement_truth_gate_ok": True,
                    "ekf_truth_gate_ok": False,
                    "summary_path": "logs/session_fake_b/summary.json",
                    "incident_path": "logs/session_fake_b/incident_bundle.json",
                    "run_dir": "logs/session_fake_b",
                },
                "sharp_arc": {
                    "status": "PASS",
                    "profile": "sharp_arc",
                    "duration_s": 1.0,
                    "verdict": {"primary": "PASS", "reason": "scenario completed"},
                    "measurement_truth_gate_ok": True,
                    "ekf_truth_gate_ok": True,
                    "summary_path": "logs/session_fake_c/summary.json",
                    "incident_path": "logs/session_fake_c/incident_bundle.json",
                    "run_dir": "logs/session_fake_c",
                },
            }

            def _fake_run_profile(profile_name: str, **_: object):
                call_order.append(profile_name)
                return dict(fake_results[profile_name])

            with mock.patch.object(hub, "AGENT_TESTS_DIR", tests_dir), mock.patch.object(
                hub, "_new_hub_session_dir", return_value=run_dir
            ), mock.patch.object(
                hub, "publish_latest_alias", side_effect=self._local_latest_publisher(tests_dir)
            ), mock.patch.object(
                hub, "_publish_session_latest_aliases", return_value=[]
            ), mock.patch.object(
                hub, "LATEST_HUB_SEQUENCE_SUMMARY_PATH", latest_summary
            ), mock.patch.object(
                hub, "LATEST_HUB_SEQUENCE_RUN_PATH", latest_run
            ), mock.patch.object(
                hub, "run_profile", side_effect=_fake_run_profile
            ):
                out = hub.run_sequence(
                    profiles=["M0_measurement_trust_live", "gentle_arc", "sharp_arc"],
                    auto_runtime=False,
                    archive_logs=False,
                )

            self.assertEqual(out.get("status"), "FAIL")
            self.assertEqual(call_order, ["M0_measurement_trust_live", "gentle_arc"])
            self.assertTrue(bool(out.get("stopped_early", False)))
            self.assertEqual(out.get("stopped_at_profile"), "gentle_arc")
            self.assertTrue(latest_summary.exists())
            self.assertTrue(latest_run.exists())

    def test_run_sequence_passes_motion_level_ladder(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            tests_dir = root / "logs" / "latest"
            run_dir = root / "logs" / "session_unit_sequence_pass"
            tests_dir.mkdir(parents=True, exist_ok=True)
            run_dir.mkdir(parents=True, exist_ok=True)

            latest_summary = tests_dir / "latest_hub_sequence_summary.json"
            latest_run = tests_dir / "latest_hub_sequence_run.json"
            call_order = []

            def _fake_run_profile(profile_name: str, **_: object):
                call_order.append(profile_name)
                return {
                    "status": "PASS",
                    "profile": profile_name,
                    "duration_s": 0.5,
                    "verdict": {"primary": "PASS", "reason": "scenario completed"},
                    "measurement_truth_gate_ok": True,
                    "ekf_truth_gate_ok": True,
                    "summary_path": f"logs/session_{profile_name}/summary.json",
                    "incident_path": f"logs/session_{profile_name}/incident_bundle.json",
                    "run_dir": f"logs/session_{profile_name}",
                }

            with mock.patch.object(hub, "AGENT_TESTS_DIR", tests_dir), mock.patch.object(
                hub, "_new_hub_session_dir", return_value=run_dir
            ), mock.patch.object(
                hub, "publish_latest_alias", side_effect=self._local_latest_publisher(tests_dir)
            ), mock.patch.object(
                hub, "_publish_session_latest_aliases", return_value=[]
            ), mock.patch.object(
                hub, "LATEST_HUB_SEQUENCE_SUMMARY_PATH", latest_summary
            ), mock.patch.object(
                hub, "LATEST_HUB_SEQUENCE_RUN_PATH", latest_run
            ), mock.patch.object(
                hub, "run_profile", side_effect=_fake_run_profile
            ):
                out = hub.run_sequence(
                    sequence="motion_levels_M0_M4_1",
                    auto_runtime=False,
                    archive_logs=False,
                )

            self.assertEqual(out.get("status"), "PASS")
            self.assertEqual(call_order, list(hub.MOTION_LEVEL_SEQUENCE_M0_M4_1))
            self.assertFalse(bool(out.get("stopped_early", True)))
            self.assertTrue(latest_summary.exists())
            self.assertTrue(latest_run.exists())

    def test_run_profile_guarded_writes_failure_artifacts_on_exception(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            tests_dir = root / "logs" / "latest"
            run_dir = root / "logs" / "session_unit_guarded_failure"
            tests_dir.mkdir(parents=True, exist_ok=True)
            run_dir.mkdir(parents=True, exist_ok=True)
            latest_summary = tests_dir / "latest_hub_summary.json"
            latest_incident = tests_dir / "latest_hub_incident.json"
            latest_run = tests_dir / "latest_hub_run.json"
            latest_run_dir = tests_dir / "latest_hub_run_dir.txt"

            with mock.patch.object(hub, "AGENT_TESTS_DIR", tests_dir), mock.patch.object(
                hub, "_new_hub_session_dir", return_value=run_dir
            ), mock.patch.object(
                hub, "publish_latest_alias", side_effect=self._local_latest_publisher(tests_dir)
            ), mock.patch.object(
                hub, "_publish_session_latest_aliases", return_value=[]
            ), mock.patch.object(
                hub, "LATEST_HUB_SUMMARY_PATH", latest_summary
            ), mock.patch.object(
                hub, "LATEST_HUB_INCIDENT_PATH", latest_incident
            ), mock.patch.object(
                hub, "LATEST_HUB_RUN_PATH", latest_run
            ), mock.patch.object(
                hub, "LATEST_HUB_RUN_DIR_PATH", latest_run_dir
            ), mock.patch.object(
                hub, "run_profile", side_effect=RuntimeError("boom")
            ):
                out = hub._run_profile_guarded("gentle_arc", auto_runtime=False, archive_logs=False)

            self.assertEqual(out.get("status"), "FAIL")
            verdict = dict(out.get("verdict") or {})
            self.assertEqual(verdict.get("primary"), "HUB_INTERNAL_ERROR")
            self.assertTrue(latest_summary.exists())
            self.assertTrue(latest_incident.exists())
            self.assertTrue(latest_run.exists())
            self.assertTrue(latest_run_dir.exists())
            summary_obj = hub._read_json(latest_summary)
            self.assertEqual(summary_obj.get("status"), "FAIL")
            self.assertEqual(((summary_obj.get("verdict") or {}).get("primary")), "HUB_INTERNAL_ERROR")

    def test_payload_success_variants(self):
        self.assertTrue(hub._payload_success({"ok": True}))
        self.assertFalse(hub._payload_success({"ok": False}))
        self.assertTrue(hub._payload_success({"status": "PASS"}))
        self.assertFalse(hub._payload_success({"status": "FAIL"}))
        self.assertIsNone(hub._payload_success({"something": "else"}))

    def test_parse_json_from_json_result_line(self):
        payload = hub._parse_json_from_text(
            "noise before\nJSON_RESULT: {\"success\": false, \"motion_actual_ssot\": \"EKF_POSE_ODOMETRY_SSOT\"}"
        )
        self.assertIsInstance(payload, dict)
        self.assertFalse(bool(payload.get("success", True)))
        self.assertEqual(payload.get("motion_actual_ssot"), "EKF_POSE_ODOMETRY_SSOT")

    def test_archive_large_logs_moves_old_session_and_moves_runtime_log(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            logs = root / "logs"
            runtime = root / "runtime"
            save = root / "save"
            logs.mkdir(parents=True, exist_ok=True)
            runtime.mkdir(parents=True, exist_ok=True)
            save.mkdir(parents=True, exist_ok=True)

            s_old = logs / "session_20250101_000000"
            s_new = logs / "session_20250101_000001"
            s_old.mkdir(parents=True, exist_ok=True)
            s_new.mkdir(parents=True, exist_ok=True)
            (s_old / "control.jsonl").write_text("x\n" * 2000, encoding="utf-8")
            (s_new / "control.jsonl").write_text("y\n" * 20, encoding="utf-8")

            now = time.time()
            os.utime(s_old, (now - 3600, now - 3600))
            os.utime(s_new, (now - 10, now - 10))

            large_runtime = runtime / "audit.jsonl"
            large_runtime.write_text("z" * 20000, encoding="utf-8")
            os.utime(large_runtime, (now - 3600, now - 3600))

            orig_project_root = hub.PROJECT_ROOT
            try:
                hub.PROJECT_ROOT = root
                result = hub.archive_large_logs_to_save(
                    project_root=root,
                    max_file_mb=0.001,
                    keep_latest_sessions=1,
                    min_age_s=0.0,
                    dry_run=False,
                )
            finally:
                hub.PROJECT_ROOT = orig_project_root

            archived_sessions = (result.get("sessions") or {}).get("archived", [])
            archived_files = (result.get("large_files") or {}).get("archived", [])
            self.assertGreaterEqual(len(archived_sessions), 1)
            self.assertGreaterEqual(len(archived_files), 1)

            self.assertFalse(s_old.exists())
            self.assertTrue(s_new.exists())

            self.assertFalse(large_runtime.exists())

            archive_root_rel = result.get("archive_root")
            archive_root = root / archive_root_rel if isinstance(archive_root_rel, str) else None
            self.assertIsNotNone(archive_root)
            self.assertTrue(archive_root.exists())

    def test_live_run_profile_skips_automatic_log_archive(self):
        profile = hub.SCENARIOS["straight_1m"]
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            tests_dir = root / "logs" / "latest"
            run_dir = root / "logs" / "session_unit_live_archive_skip"
            tests_dir.mkdir(parents=True, exist_ok=True)
            run_dir.mkdir(parents=True, exist_ok=True)
            latest_summary = tests_dir / "latest_hub_summary.json"
            latest_incident = tests_dir / "latest_hub_incident.json"
            latest_run = tests_dir / "latest_hub_run.json"
            latest_run_dir = tests_dir / "latest_hub_run_dir.txt"

            with mock.patch.object(
                hub,
                "AGENT_TESTS_DIR",
                tests_dir,
            ), mock.patch.object(
                hub,
                "_new_hub_session_dir",
                return_value=run_dir,
            ), mock.patch.object(
                hub,
                "publish_latest_alias",
                side_effect=self._local_latest_publisher(tests_dir),
            ), mock.patch.object(
                hub,
                "_publish_session_latest_aliases",
                return_value=[],
            ), mock.patch.object(
                hub,
                "LATEST_HUB_SUMMARY_PATH",
                latest_summary,
            ), mock.patch.object(
                hub,
                "LATEST_HUB_INCIDENT_PATH",
                latest_incident,
            ), mock.patch.object(
                hub,
                "LATEST_HUB_RUN_PATH",
                latest_run,
            ), mock.patch.object(
                hub,
                "LATEST_HUB_RUN_DIR_PATH",
                latest_run_dir,
            ), mock.patch.object(
                hub,
                "_logger_lifecycle_snapshot",
                side_effect=[
                    {"status_version": 1, "logger_queue_depth": 0, "dropped_messages": 0, "write_errors": 0},
                    {"status_version": 2, "logger_queue_depth": 0, "dropped_messages": 0, "write_errors": 0},
                ],
            ), mock.patch.object(
                hub,
                "_archive_need_assessment",
                side_effect=AssertionError("live run must not assess archive"),
            ), mock.patch.object(
                hub,
                "archive_large_logs_to_save",
                side_effect=AssertionError("live run must not archive logs"),
            ), mock.patch.object(
                hub,
                "_runtime_manager_action",
                return_value={"ok": True, "payload": {"running": True, "ready_for_live_tests": True}},
            ), mock.patch.object(
                hub,
                "_run_preflight",
                return_value={"ok": True, "payload": {"ok": True}},
            ), mock.patch.object(
                hub,
                "_run_subprocess",
                return_value={
                    "ok": True,
                    "timed_out": False,
                    "return_code": 0,
                    "duration_s": 0.1,
                    "stdout_tail": '{"success": true, "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT", "truth_basis": {"motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT", "encoder_pose_active_samples": 0, "lidar_odom_applied_samples": 1, "lidar_odom_accepted_total": 1, "lidar_odom_latest_age_s": 0.1, "lidar_odom_latest_confidence": 0.9, "turn_primitive_requested_vs_limited_match_ratio": 1.0, "turn_primitive_limited_vs_executed_match_ratio": 1.0, "turn_primitive_requested_vs_executed_match_ratio": 1.0, "turn_primitive_executed_vs_actual_match_ratio": 1.0}, "turn_primitive_requested": "STRAIGHT", "turn_primitive_limited": "STRAIGHT", "turn_primitive_executed": "STRAIGHT", "turn_primitive_actual": "STRAIGHT"}',
                    "stderr_tail": "",
                    "stdout_json": {
                        "success": True,
                        "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                        "truth_basis": {
                            "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                            "encoder_pose_active_samples": 0,
                            "lidar_odom_applied_samples": 1,
                            "lidar_odom_accepted_total": 1,
                            "lidar_odom_latest_age_s": 0.1,
                            "lidar_odom_latest_confidence": 0.9,
                            "turn_primitive_requested_vs_limited_match_ratio": 1.0,
                            "turn_primitive_limited_vs_executed_match_ratio": 1.0,
                            "turn_primitive_requested_vs_executed_match_ratio": 1.0,
                            "turn_primitive_executed_vs_actual_match_ratio": 1.0,
                        },
                        "turn_primitive_requested": "STRAIGHT",
                        "turn_primitive_limited": "STRAIGHT",
                        "turn_primitive_executed": "STRAIGHT",
                        "turn_primitive_actual": "STRAIGHT",
                    },
                    "stderr_json": None,
                },
            ):
                out = hub.run_profile(profile.name, auto_runtime=True, archive_logs=True)

            self.assertEqual(out.get("status"), "PASS")
            run_payload = hub._read_json(latest_run)
            archive = dict(run_payload.get("archive") or {})
            self.assertTrue(bool(archive.get("requested", False)))
            self.assertFalse(bool(archive.get("enabled", True)))
            self.assertEqual(archive.get("skipped_reason"), "disabled_during_live_run_for_watchdog_safety")
            self.assertIsNone(archive.get("pre_check"))
            self.assertIsNone(archive.get("post_check"))

    def test_live_run_profile_reuses_ready_runtime_before_scenario(self):
        profile = hub.SCENARIOS["wall_follow_first_wall_1min_live"]
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            tests_dir = root / "logs" / "latest"
            run_dir = root / "logs" / "session_unit_reuse_ready_runtime"
            tests_dir.mkdir(parents=True, exist_ok=True)
            run_dir.mkdir(parents=True, exist_ok=True)
            latest_summary = tests_dir / "latest_hub_summary.json"
            latest_incident = tests_dir / "latest_hub_incident.json"
            latest_run = tests_dir / "latest_hub_run.json"
            latest_run_dir = tests_dir / "latest_hub_run_dir.txt"

            runtime_calls = []

            def _fake_runtime_action(action: str):
                runtime_calls.append(str(action))
                if action == "status":
                    return {
                        "ok": True,
                        "payload": {
                            "running": True,
                            "ready_for_live_tests": True,
                            "state": "IDLE",
                        },
                    }
                raise AssertionError(f"unexpected runtime action: {action}")

            with mock.patch.object(
                hub,
                "AGENT_TESTS_DIR",
                tests_dir,
            ), mock.patch.object(
                hub,
                "_new_hub_session_dir",
                return_value=run_dir,
            ), mock.patch.object(
                hub,
                "publish_latest_alias",
                side_effect=self._local_latest_publisher(tests_dir),
            ), mock.patch.object(
                hub,
                "_publish_session_latest_aliases",
                return_value=[],
            ), mock.patch.object(
                hub,
                "LATEST_HUB_SUMMARY_PATH",
                latest_summary,
            ), mock.patch.object(
                hub,
                "LATEST_HUB_INCIDENT_PATH",
                latest_incident,
            ), mock.patch.object(
                hub,
                "LATEST_HUB_RUN_PATH",
                latest_run,
            ), mock.patch.object(
                hub,
                "LATEST_HUB_RUN_DIR_PATH",
                latest_run_dir,
            ), mock.patch.object(
                hub,
                "_logger_lifecycle_snapshot",
                side_effect=[
                    {"status_version": 1, "logger_queue_depth": 0, "dropped_messages": 0, "write_errors": 0},
                    {"status_version": 2, "logger_queue_depth": 0, "dropped_messages": 0, "write_errors": 0},
                ],
            ), mock.patch.object(
                hub,
                "_runtime_manager_action",
                side_effect=_fake_runtime_action,
            ), mock.patch.object(
                hub,
                "_run_preflight",
                return_value={"ok": True, "payload": {"ok": True}},
            ) as preflight_mock, mock.patch.object(
                hub,
                "_run_subprocess",
                return_value={
                    "ok": True,
                    "timed_out": False,
                    "return_code": 0,
                    "duration_s": 0.1,
                    "stdout_tail": '{"success": true, "status": "PASS"}',
                    "stderr_tail": "",
                    "stdout_json": {"success": True, "status": "PASS"},
                    "stderr_json": None,
                },
            ):
                out = hub.run_profile(profile.name, auto_runtime=True, archive_logs=False)

            self.assertEqual(out.get("status"), "PASS")
            self.assertEqual(runtime_calls, ["status"])
            self.assertEqual(preflight_mock.call_count, 1)

            run_payload = hub._read_json(latest_run)
            runtime_block = dict(run_payload.get("runtime") or {})
            self.assertTrue(bool(runtime_block.get("ready_for_live_tests_after_recovery", False)))
            recovery = list(runtime_block.get("recovery") or [])
            self.assertEqual(len(recovery), 1)
            self.assertEqual(str(recovery[0].get("action")), "reuse_ready_runtime")

    def test_live_run_profile_starts_runtime_when_status_not_ready(self):
        profile = hub.SCENARIOS["wall_follow_first_wall_1min_live"]
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            tests_dir = root / "logs" / "latest"
            run_dir = root / "logs" / "session_unit_start_runtime"
            tests_dir.mkdir(parents=True, exist_ok=True)
            run_dir.mkdir(parents=True, exist_ok=True)
            latest_summary = tests_dir / "latest_hub_summary.json"
            latest_incident = tests_dir / "latest_hub_incident.json"
            latest_run = tests_dir / "latest_hub_run.json"
            latest_run_dir = tests_dir / "latest_hub_run_dir.txt"

            runtime_calls = []

            def _fake_runtime_action(action: str):
                runtime_calls.append(str(action))
                if action == "status":
                    return {
                        "ok": True,
                        "payload": {
                            "running": True,
                            "ready_for_live_tests": False,
                            "state": "STARTING",
                        },
                    }
                if action == "start":
                    return {
                        "ok": True,
                        "payload": {
                            "running": True,
                            "ready_for_live_tests": True,
                            "state": "IDLE",
                        },
                    }
                raise AssertionError(f"unexpected runtime action: {action}")

            with mock.patch.object(
                hub,
                "AGENT_TESTS_DIR",
                tests_dir,
            ), mock.patch.object(
                hub,
                "_new_hub_session_dir",
                return_value=run_dir,
            ), mock.patch.object(
                hub,
                "publish_latest_alias",
                side_effect=self._local_latest_publisher(tests_dir),
            ), mock.patch.object(
                hub,
                "_publish_session_latest_aliases",
                return_value=[],
            ), mock.patch.object(
                hub,
                "LATEST_HUB_SUMMARY_PATH",
                latest_summary,
            ), mock.patch.object(
                hub,
                "LATEST_HUB_INCIDENT_PATH",
                latest_incident,
            ), mock.patch.object(
                hub,
                "LATEST_HUB_RUN_PATH",
                latest_run,
            ), mock.patch.object(
                hub,
                "LATEST_HUB_RUN_DIR_PATH",
                latest_run_dir,
            ), mock.patch.object(
                hub,
                "_logger_lifecycle_snapshot",
                side_effect=[
                    {"status_version": 1, "logger_queue_depth": 0, "dropped_messages": 0, "write_errors": 0},
                    {"status_version": 2, "logger_queue_depth": 0, "dropped_messages": 0, "write_errors": 0},
                ],
            ), mock.patch.object(
                hub,
                "_runtime_manager_action",
                side_effect=_fake_runtime_action,
            ), mock.patch.object(
                hub,
                "_run_preflight",
                return_value={"ok": True, "payload": {"ok": True}},
            ), mock.patch.object(
                hub,
                "_run_subprocess",
                return_value={
                    "ok": True,
                    "timed_out": False,
                    "return_code": 0,
                    "duration_s": 0.1,
                    "stdout_tail": '{"success": true, "status": "PASS"}',
                    "stderr_tail": "",
                    "stdout_json": {"success": True, "status": "PASS"},
                    "stderr_json": None,
                },
            ):
                out = hub.run_profile(profile.name, auto_runtime=True, archive_logs=False)

            self.assertEqual(out.get("status"), "PASS")
            self.assertEqual(runtime_calls, ["status", "start"])

    def test_run_profile_does_not_recover_payload_from_artifacts_on_nonzero_return_code(self):
        profile = hub.SCENARIOS["wall_follow_first_wall_1min_live"]
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            tests_dir = root / "logs" / "latest"
            run_dir = root / "logs" / "session_unit_nonzero_return"
            tests_dir.mkdir(parents=True, exist_ok=True)
            run_dir.mkdir(parents=True, exist_ok=True)
            latest_summary = tests_dir / "latest_hub_summary.json"
            latest_incident = tests_dir / "latest_hub_incident.json"
            latest_run = tests_dir / "latest_hub_run.json"
            latest_run_dir = tests_dir / "latest_hub_run_dir.txt"

            with mock.patch.object(
                hub,
                "AGENT_TESTS_DIR",
                tests_dir,
            ), mock.patch.object(
                hub,
                "_new_hub_session_dir",
                return_value=run_dir,
            ), mock.patch.object(
                hub,
                "publish_latest_alias",
                side_effect=self._local_latest_publisher(tests_dir),
            ), mock.patch.object(
                hub,
                "_publish_session_latest_aliases",
                return_value=[],
            ), mock.patch.object(
                hub,
                "LATEST_HUB_SUMMARY_PATH",
                latest_summary,
            ), mock.patch.object(
                hub,
                "LATEST_HUB_INCIDENT_PATH",
                latest_incident,
            ), mock.patch.object(
                hub,
                "LATEST_HUB_RUN_PATH",
                latest_run,
            ), mock.patch.object(
                hub,
                "LATEST_HUB_RUN_DIR_PATH",
                latest_run_dir,
            ), mock.patch.object(
                hub,
                "_logger_lifecycle_snapshot",
                side_effect=[
                    {"status_version": 1, "logger_queue_depth": 0, "dropped_messages": 0, "write_errors": 0},
                    {"status_version": 2, "logger_queue_depth": 0, "dropped_messages": 0, "write_errors": 0},
                ],
            ), mock.patch.object(
                hub,
                "_runtime_manager_action",
                return_value={"ok": True, "payload": {"running": True, "ready_for_live_tests": True}},
            ), mock.patch.object(
                hub,
                "_run_preflight",
                return_value={"ok": True, "payload": {"ok": True}},
            ), mock.patch.object(
                hub,
                "_recover_payload_from_artifacts",
                side_effect=AssertionError("artifact recovery must not run on failed command"),
            ), mock.patch.object(
                hub,
                "_run_subprocess",
                return_value={
                    "ok": True,
                    "timed_out": False,
                    "return_code": 1,
                    "duration_s": 0.1,
                    "stdout_tail": "",
                    "stderr_tail": "ERROR: synthetic failure",
                    "stdout_json": None,
                    "stderr_json": None,
                },
            ):
                out = hub.run_profile(profile.name, auto_runtime=True, archive_logs=False)

            self.assertEqual(out.get("status"), "FAIL")


if __name__ == "__main__":
    unittest.main()
