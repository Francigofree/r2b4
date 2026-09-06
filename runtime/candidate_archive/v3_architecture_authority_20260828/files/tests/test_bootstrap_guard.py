#!/usr/bin/env python3

import hashlib
import json
import unittest
from datetime import datetime, timezone
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

from project_rules import bootstrap_guard as bg


class TestBootstrapGuard(unittest.TestCase):
    def test_missing_prompt_raises(self):
        with TemporaryDirectory() as tmp_dir:
            tmp_root = Path(tmp_dir)
            prompt_path = tmp_root / "project_rules" / "agent_system_prompt.txt"
            with patch.object(bg, "PROJECT_ROOT", tmp_root), patch.object(bg, "PROMPT_PATH", prompt_path):
                with self.assertRaises(bg.BootstrapGuardError):
                    bg.ensure_agent_system_prompt_loaded()

    def test_empty_prompt_raises(self):
        with TemporaryDirectory() as tmp_dir:
            tmp_root = Path(tmp_dir)
            prompt_path = tmp_root / "project_rules" / "agent_system_prompt.txt"
            prompt_path.parent.mkdir(parents=True, exist_ok=True)
            prompt_path.write_text("   \n", encoding="utf-8")
            with patch.object(bg, "PROJECT_ROOT", tmp_root), patch.object(bg, "PROMPT_PATH", prompt_path):
                with self.assertRaises(bg.BootstrapGuardError):
                    bg.ensure_agent_system_prompt_loaded()

    def test_non_empty_prompt_returns_content(self):
        with TemporaryDirectory() as tmp_dir:
            tmp_root = Path(tmp_dir)
            prompt_path = tmp_root / "project_rules" / "agent_system_prompt.txt"
            prompt_path.parent.mkdir(parents=True, exist_ok=True)
            expected = "SYSTEM_PROMPT_X\n"
            prompt_path.write_text(expected, encoding="utf-8")
            with patch.object(bg, "PROJECT_ROOT", tmp_root), patch.object(bg, "PROMPT_PATH", prompt_path):
                actual = bg.ensure_agent_system_prompt_loaded()
        self.assertEqual(actual, expected)

    @staticmethod
    def _write(root: Path, relative: str, content: str) -> None:
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")

    @classmethod
    def _valid_project(cls, root: Path) -> None:
        strict_document = (
            "# R2B4 szigorú réteg- és contract baseline V2.1\n\n"
            "**Contract:** `R2B4_ARCH_LAYER_CONTRACT_V2_1`\n"
            "**Szerep:** normatív architektúra-SSOT. Nem eseménynapló.\n"
        )
        identifiers = {
            "control_mode": "UNIFIED",
            "odometry_mode": "LIDAR_FIRST",
            "pose_frame_id": "R2B4_BOOT_ROBOT_MAP",
            "pose_frame_owner": "EKF_POSE_ODOMETRY_SSOT",
            "pose_frame_yaw": "CCW_POSITIVE_LEFT",
            "scan_matcher_contract_id": "R2B4_SCAN_MATCHER_PROCESS_LATEST_ONLY_V1",
            "scan_matcher_confidence_model": "R2B4_SCAN_MATCH_CONFIDENCE_V2",
            "scan_matcher_transport": "process_latest_only",
            "speed_map_schema": "R2B4_WHEEL_SPEED_MAP_V2",
            "speed_map_state": "ACTIVE",
            "speed_map_curves": ["left_forward", "left_reverse", "right_forward", "right_reverse"],
        }
        registry = {
            "schema": "R2B4_PROTECTED_BASELINE_V1",
            "documents": {
                "agent_workflow": "AGENTS.md",
                "structural_motion_architecture": "STRUKTURALIS_RETEGEK_V2_1_STRICT.md",
                "stable_baseline": "STRUKTURALIS_RETEGEK.md",
                "current_change": "project_rules/current_change.json",
                "validation_guide": "docs/AGENT_RUNTIME.md",
                "agent_prompt": "project_rules/agent_system_prompt.txt",
                "agent_infrastructure": "project_rules/agent_infrastructure.json",
            },
            "document_sha256": {
                "structural_motion_architecture": hashlib.sha256(
                    strict_document.encode("utf-8")
                ).hexdigest(),
            },
            "identifiers": identifiers,
            "required_artifacts": [
                "logs/latest/latest_hub_summary.json",
                "logs/latest/latest_hub_run.json",
            ],
            "failure_artifact": "logs/latest/latest_hub_incident.json",
            "scan_matcher_contract": {
                "constants_file": "middleware/scan_matcher_contract.py",
                "constants": {
                    "SCAN_MATCHER_CONTRACT_ID": "R2B4_SCAN_MATCHER_PROCESS_LATEST_ONLY_V1",
                    "SCAN_MATCH_CONFIDENCE_MODEL": "R2B4_SCAN_MATCH_CONFIDENCE_V2",
                    "SCAN_MATCHER_TRANSPORT": "process_latest_only",
                    "SCAN_MATCHER_PROCESS_START_METHOD": "spawn",
                    "SCAN_MATCHER_INPUT_QUEUE_SIZE": 1,
                    "SCAN_MATCHER_RESULT_QUEUE_SIZE": 1,
                    "SCAN_MATCHER_MAX_INPUT_AGE_S": 0.25,
                    "SCAN_MATCHER_MAX_RESULT_AGE_S": 0.25,
                },
                "required_config_values": {
                    "lidar_runtime.matcher_process_start_method": "spawn",
                    "lidar_runtime.latest_scan_queue_size": 1,
                    "lidar_runtime.latest_result_queue_size": 1,
                    "lidar_runtime.matcher_max_input_age_s": 0.25,
                    "lidar_runtime.matcher_max_result_age_s": 0.25,
                    "lidar_pose.matcher_budget_ms": 45.0,
                    "lidar_pose.slow_path_budget_ms": 120.0,
                    "lidar_pose.robust_inlier_distance_m": 0.18,
                    "lidar_pose.robust_trim_fraction": 0.8,
                    "lidar_pose.confidence_residual_scale_m": 0.15,
                    "lidar_pose.confidence_sector_count": 12,
                    "lidar_pose.confidence_target_sector_coverage": 0.5,
                    "lidar_pose.ambiguity_translation_m": 0.08,
                    "lidar_pose.ambiguity_rotation_rad": 0.12,
                    "lidar_pose.ambiguity_margin_scale": 0.2,
                    "lidar_pose.ambiguity_residual_margin_scale_m": 0.04,
                    "lidar_pose.observability_translation_step_m": 0.03,
                    "lidar_pose.observability_rotation_step_rad": 0.06,
                    "lidar_pose.observability_cost_scale": 0.0004,
                    "lidar_odometry.min_confidence": 0.2,
                    "lidar_odometry.max_scan_age_s": 0.25,
                },
                "required_source_tokens": {
                    "middleware/scan_matching.py": [
                        "from scipy.spatial import cKDTree",
                        "def _robust_match_metrics",
                        "ambiguous_alternative",
                        "observability_score",
                        'stats["confidence_model"] = SCAN_MATCH_CONFIDENCE_MODEL',
                    ],
                    "sensors/lidar_service.py": [
                        "validate_matcher_runtime_config(runtime_cfg)",
                        "multiprocessing.get_context",
                        "self._mp_context.Process",
                        "target=matcher_process_main",
                        "self._publish_raw_snapshot(",
                        "self._queue_latest(",
                        'packet.get("matcher_contract_id") != SCAN_MATCHER_CONTRACT_ID',
                        "int(scan_seq) != int(current_raw_scan_id)",
                        "result_age_s > self._matcher_max_result_age_s",
                    ],
                    "sensors/lidar_matcher_process.py": [
                        "def _put_latest",
                        "def _drain_latest",
                        'packet.get("matcher_contract_id") != SCAN_MATCHER_CONTRACT_ID',
                        "generation != estimator_generation",
                    ],
                    "middleware/lidar_odometry.py": [
                        "lidar_odometry_measurement_id",
                        "rejected_duplicate_matcher_result",
                        "rejected_duplicate_raw_scan",
                    ],
                },
                "forbidden_source_tokens": {
                    "sensors/lidar_service.py": [
                        "def _matcher_worker",
                        "target=self._matcher_worker",
                        '"thread_latest_only"',
                    ],
                    "middleware/scan_matching.py": [
                        "from scipy.spatial import KDTree",
                    ],
                },
            },
            "logger_jitter_contract": {
                "housekeeping": {
                    "file": "log/unified_logger.py",
                    "class": "UnifiedLogger",
                    "method": "run_housekeeping",
                    "required_queue_call": "_queue_runtime_stats",
                    "forbidden_io_calls": ["_write_runtime_stats", "write_text", "write_bytes", "open"],
                },
                "async_snapshot": {
                    "file": "log/async_logger.py",
                    "class": "AsyncLogger",
                    "enqueue_method": "write_json_snapshot",
                    "enqueue_forbidden_io_calls": ["write_text", "write_bytes", "open", "replace"],
                    "worker_method": "_flush_worker",
                    "worker_flush_call": "_flush_all",
                    "flush_method": "_flush_all",
                    "flush_writer_call": "_write_json_snapshot",
                    "writer_method": "_write_json_snapshot",
                },
            },
            "legacy_contract": {
                "motion_profiles_exactly": ["UNIFIED"],
                "legacy_motion_contract_types_empty": True,
                "legacy_resolver_command_types_empty": True,
                "gui_direct_pwm_removed_marker": "Command removed: set_motor_pwm is no longer available.",
                "legacy_tank_removed_marker": "legacy_tank_removed_from_runtime",
                "forbidden_runtime_files": [
                    "controller/motion_stack.py",
                    "controller/smooth_trajectory.py",
                    "controller/velocity_tracking.py",
                ],
                "forbidden_motion_config_keys": [
                    "motion_stack",
                    "velocity_tracking",
                    "RAW_ENCODER_DIAGNOSTIC_MODE",
                    "derivalo_tag_d",
                    "straight_hold_kd",
                    "straight_hold_yaw_deadband_rad_s",
                    "holtsav_kuszob",
                    "gyorsulasi_ramp_limit",
                    "forward_clearance_adaptive_enable",
                    "forward_blocked_front_hard_stop",
                    "forward_clearance_hard_floor_m",
                    "forward_clearance_hard_m",
                    "forward_clearance_hard_cap_m",
                    "forward_clearance_speed_gain",
                    "forward_clearance_warn_m",
                    "forward_clearance_warn_extra_m",
                    "forward_clearance_warn_speed_gain",
                    "forward_clearance_warn_cap_m",
                    "forward_clearance_min_scale",
                ],
                "required_motion_config_values": {
                    "lidar_pose.matcher_seed_translation_prior_weight": 1.0,
                    "lidar_pose.matcher_seed_rotation_prior_weight": 0.05,
                    "pid_szabalyzo.aranyos_tag_p": 0.25,
                    "pid_szabalyzo.integralo_tag_i": 0.08,
                    "pid_szabalyzo.integralo_limit": 0.18,
                },
                "required_speed_map_curve_points": {
                    "left_forward": [[0.05, 0.1], [0.1, 0.2]],
                    "right_forward": [[0.05, 0.1], [0.1, 0.2]],
                    "left_reverse": [[0.05, 0.1], [0.1, 0.2]],
                    "right_reverse": [[0.05, 0.1], [0.1, 0.2]],
                },
                "forbidden_runtime_source_tokens": {
                    "cont.py": ["motion_executor.apply_ramp", "motion_executor.ramp_limit"],
                    "motion_executor.py": ["def apply_ramp", "self._v_cmd_current", "self.ramp_limit"],
                    "controller/commands.py": ["motion_executor.ramp_limit"],
                    "controller/components.py": ["ramp_limit="],
                    "controller/routines.py": ["motion_executor.ramp_limit", "ramp_limit="],
                    "core/control_strategies.py": [
                        "LOW_SPEED_REF_MAX_MPS",
                        "LOW_SPEED_KP",
                        "LOW_SPEED_CORRECTION_MAX_PWM",
                        "track_deadband",
                        "wheel_loop_low_speed_scheduled",
                        "wheel_loop_unequal_track_schedule_active",
                    ],
                    "controller/motion_readiness.py": [
                        "forward_clearance_adaptive_enable",
                        "FORWARD_CLEARANCE_SCALED",
                        "FORWARD_BLOCKED_FRONT_ZERO",
                    ],
                },
                "required_runtime_source_tokens": {
                    "core/control_strategies.py": [
                        "maintenance_floor_pwm_l=maintenance_floor_l",
                        "maintenance_floor_pwm_r=maintenance_floor_r",
                        "wheel_loop_left_maintenance_floor_applied",
                        "wheel_loop_right_maintenance_floor_applied",
                    ],
                },
            },
        }
        cls._write(root, "AGENTS.md", "run bootstrap_guard and follow baseline\n")
        cls._write(root, ".vscode/AGENTS.md", "follow root AGENTS.md\n")
        cls._write(root, "docs/AGENT_RUNTIME.md", "validation order\n")
        cls._write(root, "project_rules/agent_system_prompt.txt", "agent prompt\n")
        cls._write(root, "project_rules/protected_baseline.json", json.dumps(registry))
        cls._write(
            root,
            "project_rules/agent_infrastructure.json",
            json.dumps(
                {
                    "schema": "R2B4_AGENT_INFRASTRUCTURE_V1",
                    "contract_id": "TEST_MINIMAL_AGENT_INFRA",
                    "default_agent_mode": "single_agent",
                    "max_auxiliary_agents": 1,
                    "recursive_delegation_allowed": False,
                    "parallel_writers_allowed": False,
                    "context_budgets_bytes": {
                        "cold_capsule": 8192,
                        "unchanged_delta": 1024,
                        "auxiliary_input": 6144,
                        "auxiliary_output": 3072,
                    },
                    "normative_authorities": {
                        "structural_motion_architecture": {
                            "authority": "NORMATIVE_SSOT",
                            "path": "STRUKTURALIS_RETEGEK_V2_1_STRICT.md",
                            "document_role": "structural_motion_architecture",
                            "contract_id": "R2B4_ARCH_LAYER_CONTRACT_V2_1",
                            "domains": ["motion_control"],
                        },
                        "v3_robot_architecture": {
                            "authority": "NORMATIVE_SSOT",
                            "path": "STRUKTURALIS_RETEGEK_V3.md",
                            "document_role": "v3_robot_architecture",
                            "contract_id": "R2B4_ARCH_LAYER_CONTRACT_V3",
                            "domains": ["robot_v3"],
                        }
                    },
                    "leases": {
                        name: {"exclusive": True, "default_ttl_s": 900}
                        for name in (
                            "workspace_write",
                            "canonical_promotion",
                            "runtime_control",
                            "live_motion",
                            "latest_artifact_publish",
                            "full_pytest",
                        )
                    },
                    "task_workspace": {
                        "enabled": True,
                        "root": "runtime/agent_workspaces",
                        "protected_store": str(root / "protected_store"),
                        "privileged_operations": False,
                        "exclude_top_level": ["runtime"],
                        "exclude_names": ["__pycache__"],
                        "exclude_paths": ["project_rules/current_change.json"],
                        "protected_infrastructure_paths": [
                            "AGENTS.md",
                            "STRUKTURALIS_RETEGEK_V2_1_STRICT.md",
                            "project_rules/agent_infrastructure.json",
                            "project_rules/bootstrap_guard.py",
                            "project_rules/protected_baseline.json",
                            "tools/agent_change_tracker.py",
                            "tools/agent_workspace.py",
                            "tools/agentctl.py",
                        ],
                        "agent_infrastructure_allowed_paths": [
                            "STRUKTURALIS_RETEGEK_V2_1_STRICT.md",
                            "project_rules/",
                            "tools/",
                        ],
                    },
                    "domains": {
                        "motion_control": {
                            "paths": ["controller/", "core/", "amr/"],
                            "sources": ["STRUKTURALIS_RETEGEK_V2_1_STRICT.md"],
                        },
                        "robot_v3": {
                            "paths": ["v3/", "STRUKTURALIS_RETEGEK_V3.md"],
                            "sources": ["STRUKTURALIS_RETEGEK_V3.md"],
                            "required_authority": "v3_robot_architecture",
                        }
                    },
                }
            ),
        )
        cls._write(
            root,
            "project_rules/current_change.json",
            json.dumps(
                {
                    "schema": "R2B4_AGENT_CHANGE_V1",
                    "task_id": "test-task",
                    "status": "COMPLETE",
                    "updated_at_utc": "2026-07-17T00:00:00Z",
                    "files": [],
                }
            ),
        )
        baseline_values = "\n".join(str(item) for value in identifiers.values() for item in (value if isinstance(value, list) else [value]))
        cls._write(root, "STRUKTURALIS_RETEGEK_V2_1_STRICT.md", strict_document)
        cls._write(
            root,
            "STRUKTURALIS_RETEGEK_V3.md",
            "# V3 architecture\n\nContract: R2B4_ARCH_LAYER_CONTRACT_V3\n",
        )
        cls._write(root, "STRUKTURALIS_RETEGEK.md", f"baseline\n{baseline_values}\n")
        cls._write(root, "conf/control_mode.json", json.dumps({"control_mode": "UNIFIED"}))
        cls._write(
            root,
            "conf/vezerles.json",
            json.dumps(
                {
                    "odometry_mode": "LIDAR_FIRST",
                    "motion_profiles": {"UNIFIED": {}},
                    "lidar_pose": {
                        "matcher_seed_translation_prior_weight": 1.0,
                        "matcher_seed_rotation_prior_weight": 0.05,
                        "matcher_budget_ms": 45.0,
                        "slow_path_budget_ms": 120.0,
                        "robust_inlier_distance_m": 0.18,
                        "robust_trim_fraction": 0.8,
                        "confidence_residual_scale_m": 0.15,
                        "confidence_sector_count": 12,
                        "confidence_target_sector_coverage": 0.5,
                        "ambiguity_translation_m": 0.08,
                        "ambiguity_rotation_rad": 0.12,
                        "ambiguity_margin_scale": 0.2,
                        "ambiguity_residual_margin_scale_m": 0.04,
                        "observability_translation_step_m": 0.03,
                        "observability_rotation_step_rad": 0.06,
                        "observability_cost_scale": 0.0004,
                    },
                    "lidar_runtime": {
                        "matcher_process_start_method": "spawn",
                        "latest_scan_queue_size": 1,
                        "latest_result_queue_size": 1,
                        "matcher_max_input_age_s": 0.25,
                        "matcher_max_result_age_s": 0.25,
                    },
                    "lidar_odometry": {
                        "min_confidence": 0.2,
                        "max_scan_age_s": 0.25,
                    },
                    "pid_szabalyzo": {
                        "aranyos_tag_p": 0.25,
                        "integralo_tag_i": 0.08,
                        "integralo_limit": 0.18,
                    },
                }
            ),
        )
        curves = {}
        for key in identifiers["speed_map_curves"]:
            side, direction = key.split("_", 1)
            curves[key] = {
                "wheel": side,
                "direction": direction,
                "points": [{"speed_mps": 0.05, "pwm": 0.1}, {"speed_mps": 0.1, "pwm": 0.2}],
            }
        cls._write(
            root,
            "conf/speed_map.json",
            json.dumps({"schema": "R2B4_WHEEL_SPEED_MAP_V2", "map_state": "ACTIVE", "curves": curves}),
        )
        cls._write(
            root,
            "middleware/robot_frame.py",
            'POSE_FRAME_ID = "R2B4_BOOT_ROBOT_MAP"\nPOSE_FRAME_OWNER = "EKF_POSE_ODOMETRY_SSOT"\n'
            'POSE_FRAME_YAW = "CCW_POSITIVE_LEFT"\n',
        )
        cls._write(
            root,
            "middleware/scan_matcher_contract.py",
            'SCAN_MATCHER_CONTRACT_ID = "R2B4_SCAN_MATCHER_PROCESS_LATEST_ONLY_V1"\n'
            'SCAN_MATCH_CONFIDENCE_MODEL = "R2B4_SCAN_MATCH_CONFIDENCE_V2"\n'
            'SCAN_MATCHER_TRANSPORT = "process_latest_only"\n'
            'SCAN_MATCHER_PROCESS_START_METHOD = "spawn"\n'
            "SCAN_MATCHER_INPUT_QUEUE_SIZE = 1\n"
            "SCAN_MATCHER_RESULT_QUEUE_SIZE = 1\n"
            "SCAN_MATCHER_MAX_INPUT_AGE_S = 0.25\n"
            "SCAN_MATCHER_MAX_RESULT_AGE_S = 0.25\n",
        )
        cls._write(
            root,
            "middleware/scan_matching.py",
            "from scipy.spatial import cKDTree\n"
            "def _robust_match_metrics():\n"
            "    ambiguous_alternative = True\n"
            "    observability_score = 1.0\n"
            "    stats = {}\n"
            "    stats[\"confidence_model\"] = SCAN_MATCH_CONFIDENCE_MODEL\n",
        )
        cls._write(
            root,
            "middleware/lidar_odometry.py",
            "lidar_odometry_measurement_id = None\n"
            "rejected_duplicate_matcher_result = 0\n"
            "rejected_duplicate_raw_scan = 0\n",
        )
        cls._write(
            root,
            "sensors/lidar_matcher_process.py",
            "def _put_latest(): pass\n"
            "def _drain_latest(): pass\n"
            "def worker(packet, generation, estimator_generation):\n"
            "    if packet.get(\"matcher_contract_id\") != SCAN_MATCHER_CONTRACT_ID: pass\n"
            "    if generation != estimator_generation: pass\n",
        )
        cls._write(
            root,
            "sensors/lidar_service.py",
            "import multiprocessing\n"
            "class LidarService:\n"
            "    def __init__(self, runtime_cfg):\n"
            "        validate_matcher_runtime_config(runtime_cfg)\n"
            "    def start(self):\n"
            "        self._mp_context = multiprocessing.get_context('spawn')\n"
            "        self._matcher_process = self._mp_context.Process(target=matcher_process_main)\n"
            "    def _driver_worker(self):\n"
            "        self._publish_raw_snapshot()\n"
            "        self._queue_latest()\n"
            "    def _matcher_result_worker(self, packet, scan_seq, current_raw_scan_id, result_age_s):\n"
            "        if packet.get(\"matcher_contract_id\") != SCAN_MATCHER_CONTRACT_ID: return\n"
            "        if int(scan_seq) != int(current_raw_scan_id): return\n"
            "        if result_age_s > self._matcher_max_result_age_s: return\n",
        )
        cls._write(
            root,
            "core/control_strategies.py",
            'CANONICAL_CONTROL_MODE = "UNIFIED"\n'
            "maintenance_floor_pwm_l=maintenance_floor_l\n"
            "maintenance_floor_pwm_r=maintenance_floor_r\n"
            "wheel_loop_left_maintenance_floor_applied\n"
            "wheel_loop_right_maintenance_floor_applied\n",
        )
        cls._write(root, "controller/motion_contract.py", "_LEGACY_TYPES = set()\n")
        cls._write(root, "controller/motion_resolver.py", "_LEGACY_COMMAND_TYPES = frozenset()\n")
        cls._write(root, "controller/motion_readiness.py", "# canonical semantics without clearance governor\n")
        cls._write(root, "controller/routines.py", "# stop paths\n")
        cls._write(root, "controller/components.py", "# init paths\n")
        cls._write(root, "controller/commands.py", "# legacy_tank_removed_from_runtime\n")
        cls._write(root, "motion_executor.py", "# single executor without command shaping\n")
        cls._write(root, "fastgui/backend_api.py", "# Command removed: set_motor_pwm is no longer available.\n")
        cls._write(root, "cont.py", "motor_l.set_pwm(pwm_l)\nmotor_r.set_pwm(pwm_r)\n")
        cls._write(
            root,
            "log/unified_logger.py",
            "class UnifiedLogger:\n"
            "    def run_housekeeping(self):\n"
            "        self._queue_runtime_stats({})\n",
        )
        cls._write(
            root,
            "log/async_logger.py",
            "class AsyncLogger:\n"
            "    def write_json_snapshot(self, filename, payload):\n"
            "        self._pending_json_snapshots = {filename: payload}\n"
            "    def _flush_worker(self):\n"
            "        self._flush_all()\n"
            "    def _flush_all(self):\n"
            "        self._write_json_snapshot(None, {})\n"
            "    def _write_json_snapshot(self, path, payload):\n"
            "        pass\n",
        )
        for relative in registry["required_artifacts"] + [registry["failure_artifact"]]:
            cls._write(root, relative, json.dumps({"status": "FAIL"}))

    def test_complete_project_contract_passes(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            report = bg.validate_project_bootstrap(root, now=datetime(2026, 7, 17, tzinfo=timezone.utc))
        self.assertEqual(report["status"], "PASS")
        self.assertIsNone(report["artifact_count"])
        self.assertFalse(report["artifacts_checked"])

    def test_brief_cli_argument_is_supported(self):
        args = bg._build_parser().parse_args(["--brief"])
        self.assertTrue(args.brief)
        self.assertFalse(args.with_artifacts)

    def test_brief_bootstrap_does_not_require_volatile_artifacts(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            for path in (root / "logs" / "latest").glob("*.json"):
                path.unlink()

            report = bg.validate_project_bootstrap(
                root,
                now=datetime(2026, 7, 17, tzinfo=timezone.utc),
            )

        self.assertEqual(report["status"], "PASS")
        self.assertFalse(report["artifacts_checked"])

    def test_parallel_agent_writers_cannot_be_enabled(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            path = root / "project_rules" / "agent_infrastructure.json"
            config = json.loads(path.read_text(encoding="utf-8"))
            config["parallel_writers_allowed"] = True
            self._write(root, "project_rules/agent_infrastructure.json", json.dumps(config))

            with self.assertRaisesRegex(
                bg.BootstrapGuardError,
                "parallel_writers_allowed must be False",
            ):
                bg.validate_project_bootstrap(
                    root,
                    now=datetime(2026, 7, 17, tzinfo=timezone.utc),
                )

    def test_isolated_workspace_cannot_be_disabled(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            path = root / "project_rules/agent_infrastructure.json"
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["task_workspace"]["enabled"] = False
            self._write(root, "project_rules/agent_infrastructure.json", json.dumps(payload))

            with self.assertRaisesRegex(bg.BootstrapGuardError, "isolated task workspace"):
                bg.validate_project_bootstrap(
                    root,
                    now=datetime(2026, 7, 17, tzinfo=timezone.utc),
                )

    def test_active_agentctl_cannot_be_removed_from_protected_scope(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            path = root / "project_rules/agent_infrastructure.json"
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["task_workspace"]["protected_infrastructure_paths"].remove(
                "tools/agentctl.py"
            )
            self._write(root, "project_rules/agent_infrastructure.json", json.dumps(payload))

            with self.assertRaisesRegex(bg.BootstrapGuardError, "does not protect every"):
                bg.validate_project_bootstrap(
                    root,
                    now=datetime(2026, 7, 17, tzinfo=timezone.utc),
                )

    def test_normative_motion_architecture_tamper_fails(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            path = root / "STRUKTURALIS_RETEGEK_V2_1_STRICT.md"
            path.write_text(path.read_text(encoding="utf-8") + "tamper\n", encoding="utf-8")

            with self.assertRaisesRegex(
                bg.BootstrapGuardError,
                "structural motion architecture authority SHA-256 mismatch",
            ):
                bg.validate_project_bootstrap(
                    root,
                    now=datetime(2026, 7, 17, tzinfo=timezone.utc),
                )

    def test_normative_motion_architecture_must_route_to_motion_and_amr(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            path = root / "project_rules" / "agent_infrastructure.json"
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["domains"]["motion_control"]["sources"] = []
            payload["domains"]["motion_control"]["paths"].remove("amr/")
            self._write(root, "project_rules/agent_infrastructure.json", json.dumps(payload))

            with self.assertRaisesRegex(
                bg.BootstrapGuardError,
                "normative authority is not routed for domain: motion_control",
            ):
                bg.validate_project_bootstrap(
                    root,
                    now=datetime(2026, 7, 17, tzinfo=timezone.utc),
                )

    def test_v3_authority_file_is_required_without_baseline_hash(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            (root / "STRUKTURALIS_RETEGEK_V3.md").unlink()

            with self.assertRaisesRegex(
                bg.BootstrapGuardError,
                "V3 robot architecture authority file is missing or empty",
            ):
                bg.validate_project_bootstrap(
                    root,
                    now=datetime(2026, 7, 17, tzinfo=timezone.utc),
                )

    def test_v3_contract_id_must_match_registry(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            self._write(
                root,
                "STRUKTURALIS_RETEGEK_V3.md",
                "Contract: R2B4_ARCH_LAYER_CONTRACT_V4\n",
            )

            with self.assertRaisesRegex(
                bg.BootstrapGuardError,
                "V3 robot architecture contract ID differs from registry",
            ):
                bg.validate_project_bootstrap(
                    root,
                    now=datetime(2026, 7, 17, tzinfo=timezone.utc),
                )

    def test_v3_domain_requires_path_route_source_and_authority_binding(self):
        mutations = (
            ("paths", "V3 robot architecture route does not cover v3/"),
            ("sources", "V3 robot architecture authority is not routed"),
            ("required_authority", "robot_v3 domain lacks fail-closed V3 authority binding"),
        )
        for field, expected_error in mutations:
            with self.subTest(field=field), TemporaryDirectory() as tmp_dir:
                root = Path(tmp_dir)
                self._valid_project(root)
                path = root / "project_rules" / "agent_infrastructure.json"
                payload = json.loads(path.read_text(encoding="utf-8"))
                domain = payload["domains"]["robot_v3"]
                if field == "paths":
                    domain["paths"].remove("v3/")
                elif field == "sources":
                    domain["sources"].remove("STRUKTURALIS_RETEGEK_V3.md")
                else:
                    domain["required_authority"] = "legacy"
                self._write(root, "project_rules/agent_infrastructure.json", json.dumps(payload))

                with self.assertRaisesRegex(bg.BootstrapGuardError, expected_error):
                    bg.validate_project_bootstrap(
                        root,
                        now=datetime(2026, 7, 17, tzinfo=timezone.utc),
                    )

    def test_v3_content_can_evolve_without_baseline_hash_update(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            path = root / "STRUKTURALIS_RETEGEK_V3.md"
            path.write_text(
                path.read_text(encoding="utf-8")
                + "\nNew layer rationale may evolve without an exact content hash.\n",
                encoding="utf-8",
            )

            report = bg.validate_project_bootstrap(
                root,
                now=datetime(2026, 7, 17, tzinfo=timezone.utc),
            )

            self.assertEqual(report["status"], "PASS")

    def test_control_mode_contradiction_fails(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            self._write(root, "conf/control_mode.json", json.dumps({"control_mode": "FULL"}))
            with self.assertRaisesRegex(bg.BootstrapGuardError, "control mode differs"):
                bg.validate_project_bootstrap(root, now=datetime(2026, 7, 17, tzinfo=timezone.utc))

    def test_scan_matcher_contract_constant_cannot_drift(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            path = root / "middleware" / "scan_matcher_contract.py"
            content = path.read_text(encoding="utf-8").replace(
                "R2B4_SCAN_MATCHER_PROCESS_LATEST_ONLY_V1",
                "BYPASS",
            )
            self._write(root, "middleware/scan_matcher_contract.py", content)
            with self.assertRaisesRegex(
                bg.BootstrapGuardError,
                "scan matcher contract constant differs",
            ):
                bg.validate_project_bootstrap(
                    root,
                    now=datetime(2026, 7, 17, tzinfo=timezone.utc),
                )

    def test_scan_matcher_queue_contract_cannot_drift(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            config_path = root / "conf" / "vezerles.json"
            config = json.loads(config_path.read_text(encoding="utf-8"))
            config["lidar_runtime"]["latest_scan_queue_size"] = 2
            self._write(root, "conf/vezerles.json", json.dumps(config))
            with self.assertRaisesRegex(
                bg.BootstrapGuardError,
                "required scan matcher config value differs",
            ):
                bg.validate_project_bootstrap(
                    root,
                    now=datetime(2026, 7, 17, tzinfo=timezone.utc),
                )

    def test_threaded_matcher_worker_cannot_return(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            path = root / "sensors" / "lidar_service.py"
            content = path.read_text(encoding="utf-8")
            content += "\ndef _matcher_worker(): pass\n"
            self._write(root, "sensors/lidar_service.py", content)
            with self.assertRaisesRegex(
                bg.BootstrapGuardError,
                "forbidden scan matcher source token exists",
            ):
                bg.validate_project_bootstrap(
                    root,
                    now=datetime(2026, 7, 17, tzinfo=timezone.utc),
                )

    def test_raw_safety_snapshot_must_precede_matcher_enqueue(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            path = root / "sensors" / "lidar_service.py"
            content = path.read_text(encoding="utf-8")
            content = content.replace(
                "        self._publish_raw_snapshot()\n"
                "        self._queue_latest()\n",
                "        self._queue_latest()\n"
                "        self._publish_raw_snapshot()\n",
            )
            self._write(root, "sensors/lidar_service.py", content)
            with self.assertRaisesRegex(
                bg.BootstrapGuardError,
                "raw LIDAR snapshot must publish before matcher IPC enqueue",
            ):
                bg.validate_project_bootstrap(
                    root,
                    now=datetime(2026, 7, 17, tzinfo=timezone.utc),
                )

    def test_current_change_cannot_hash_track_volatile_runtime_artifact(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            manifest_path = root / "project_rules" / "current_change.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["files"] = [
                {
                    "path": "runtime/status.json",
                    "after": {"exists": False, "sha256": None},
                }
            ]
            self._write(root, "project_rules/current_change.json", json.dumps(manifest))
            with self.assertRaisesRegex(
                bg.BootstrapGuardError,
                "cannot hash-track volatile runtime artifact",
            ):
                bg.validate_project_bootstrap(
                    root,
                    now=datetime(2026, 7, 17, tzinfo=timezone.utc),
                )

    def test_required_lidar_seed_prior_cannot_be_removed(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            config_path = root / "conf/vezerles.json"
            config = json.loads(config_path.read_text(encoding="utf-8"))
            del config["lidar_pose"]["matcher_seed_translation_prior_weight"]
            self._write(root, "conf/vezerles.json", json.dumps(config))
            with self.assertRaisesRegex(bg.BootstrapGuardError, "required motion config value differs"):
                bg.validate_project_bootstrap(root, now=datetime(2026, 7, 17, tzinfo=timezone.utc))

    def test_required_literal_wheel_pi_gain_cannot_drift(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            config_path = root / "conf/vezerles.json"
            config = json.loads(config_path.read_text(encoding="utf-8"))
            config["pid_szabalyzo"]["aranyos_tag_p"] = 0.7
            self._write(root, "conf/vezerles.json", json.dumps(config))
            with self.assertRaisesRegex(bg.BootstrapGuardError, "required motion config value differs"):
                bg.validate_project_bootstrap(root, now=datetime(2026, 7, 17, tzinfo=timezone.utc))

    def test_removed_wheel_speed_deadband_cannot_return(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            config_path = root / "conf/vezerles.json"
            config = json.loads(config_path.read_text(encoding="utf-8"))
            config["pid_szabalyzo"]["holtsav_kuszob"] = 0.01
            self._write(root, "conf/vezerles.json", json.dumps(config))
            with self.assertRaisesRegex(bg.BootstrapGuardError, "forbidden legacy motion config key exists"):
                bg.validate_project_bootstrap(root, now=datetime(2026, 7, 17, tzinfo=timezone.utc))

    def test_required_forward_speed_map_refit_cannot_drift(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            map_path = root / "conf/speed_map.json"
            speed_map = json.loads(map_path.read_text(encoding="utf-8"))
            speed_map["curves"]["left_forward"]["points"][1]["pwm"] = 0.19
            self._write(root, "conf/speed_map.json", json.dumps(speed_map))
            with self.assertRaisesRegex(bg.BootstrapGuardError, "required speed map curve differs"):
                bg.validate_project_bootstrap(root, now=datetime(2026, 7, 17, tzinfo=timezone.utc))

    def test_stale_active_manifest_fails(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            manifest = json.loads((root / "project_rules/current_change.json").read_text(encoding="utf-8"))
            manifest["status"] = "ACTIVE"
            manifest["updated_at_utc"] = "2026-07-01T00:00:00Z"
            self._write(root, "project_rules/current_change.json", json.dumps(manifest))
            with self.assertRaisesRegex(bg.BootstrapGuardError, "current change is stale"):
                bg.validate_project_bootstrap(root, now=datetime(2026, 7, 17, tzinfo=timezone.utc))

    def test_fresh_blocked_manifest_passes(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            manifest = json.loads((root / "project_rules/current_change.json").read_text(encoding="utf-8"))
            manifest["status"] = "BLOCKED"
            manifest["blocked_reason"] = "waiting for hardware"
            self._write(root, "project_rules/current_change.json", json.dumps(manifest))

            report = bg.validate_project_bootstrap(root, now=datetime(2026, 7, 17, 1, tzinfo=timezone.utc))

        self.assertEqual(report["current_change"]["status"], "BLOCKED")

    def test_blocked_manifest_requires_reason(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            manifest = json.loads((root / "project_rules/current_change.json").read_text(encoding="utf-8"))
            manifest["status"] = "BLOCKED"
            self._write(root, "project_rules/current_change.json", json.dumps(manifest))

            with self.assertRaisesRegex(bg.BootstrapGuardError, "lacks blocked_reason"):
                bg.validate_project_bootstrap(root, now=datetime(2026, 7, 17, 1, tzinfo=timezone.utc))

    def test_legacy_registry_and_nonzero_bypass_fail(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            self._write(root, "controller/motion_contract.py", '_LEGACY_TYPES = {"set_tank"}\n')
            self._write(root, "controller/routines.py", "ctrl.motor_l.set_pwm(0.5)\n")
            with self.assertRaisesRegex(bg.BootstrapGuardError, "legacy type registry|direct PWM"):
                bg.validate_project_bootstrap(root, now=datetime(2026, 7, 17, tzinfo=timezone.utc))

    def test_new_runtime_module_cannot_add_pwm_bypass(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            self._write(root, "safety/new_path.py", "ctrl.motor_l.set_pwm(0.2)\n")
            with self.assertRaisesRegex(bg.BootstrapGuardError, "direct PWM outside reviewed"):
                bg.validate_project_bootstrap(root, now=datetime(2026, 7, 17, tzinfo=timezone.utc))

    def test_forbidden_intermediate_motion_controller_file_fails(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            self._write(root, "controller/velocity_tracking.py", "# duplicate controller\n")
            with self.assertRaisesRegex(bg.BootstrapGuardError, "forbidden legacy runtime file exists"):
                bg.validate_project_bootstrap(root, now=datetime(2026, 7, 17, tzinfo=timezone.utc))

    def test_forbidden_intermediate_motion_config_fails(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            self._write(
                root,
                "conf/vezerles.json",
                json.dumps(
                    {
                        "odometry_mode": "LIDAR_FIRST",
                        "motion_profiles": {"UNIFIED": {}},
                        "motion_stack": {"velocity_tracking": {"k_p_v": 0.1}},
                    }
                ),
            )
            with self.assertRaisesRegex(bg.BootstrapGuardError, "forbidden legacy motion config key exists"):
                bg.validate_project_bootstrap(root, now=datetime(2026, 7, 17, tzinfo=timezone.utc))

    def test_removed_derivative_config_cannot_return(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            self._write(
                root,
                "conf/vezerles.json",
                json.dumps(
                    {
                        "odometry_mode": "LIDAR_FIRST",
                        "motion_profiles": {"UNIFIED": {}},
                        "pid_szabalyzo": {"straight_hold_kd": 0.0},
                    }
                ),
            )
            with self.assertRaisesRegex(bg.BootstrapGuardError, "forbidden legacy motion config key exists"):
                bg.validate_project_bootstrap(root, now=datetime(2026, 7, 17, tzinfo=timezone.utc))

    def test_duplicate_semantics_clearance_governor_cannot_return(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            config_path = root / "conf/vezerles.json"
            config = json.loads(config_path.read_text(encoding="utf-8"))
            config["motion_readiness"] = {
                "motion_semantics": {"forward_clearance_adaptive_enable": True}
            }
            self._write(root, "conf/vezerles.json", json.dumps(config))
            with self.assertRaisesRegex(bg.BootstrapGuardError, "forbidden legacy motion config key exists"):
                bg.validate_project_bootstrap(root, now=datetime(2026, 7, 17, tzinfo=timezone.utc))

    def test_duplicate_semantics_clearance_source_cannot_return(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            self._write(root, "controller/motion_readiness.py", "FORWARD_CLEARANCE_SCALED\n")
            with self.assertRaisesRegex(bg.BootstrapGuardError, "forbidden legacy runtime source token exists"):
                bg.validate_project_bootstrap(root, now=datetime(2026, 7, 17, tzinfo=timezone.utc))

    def test_removed_executor_ramp_cannot_return(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            self._write(root, "motion_executor.py", "def apply_ramp(self, target, dt): return target\n")
            with self.assertRaisesRegex(bg.BootstrapGuardError, "forbidden legacy runtime source token exists"):
                bg.validate_project_bootstrap(root, now=datetime(2026, 7, 17, tzinfo=timezone.utc))

    def test_wheel_map_maintenance_floor_cannot_be_bypassed(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            self._write(root, "core/control_strategies.py", 'CANONICAL_CONTROL_MODE = "UNIFIED"\n')
            with self.assertRaisesRegex(bg.BootstrapGuardError, "required runtime source token is missing"):
                bg.validate_project_bootstrap(root, now=datetime(2026, 7, 17, tzinfo=timezone.utc))

    def test_logger_housekeeping_cannot_restore_synchronous_runtime_stats_io(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            self._write(
                root,
                "log/unified_logger.py",
                "class UnifiedLogger:\n"
                "    def run_housekeeping(self):\n"
                "        self._write_runtime_stats({})\n",
            )
            with self.assertRaisesRegex(
                bg.BootstrapGuardError,
                "logger housekeeping (queue call is missing|performs forbidden control-thread I/O)",
            ):
                bg.validate_project_bootstrap(root, now=datetime(2026, 7, 17, tzinfo=timezone.utc))

    def test_missing_required_artifact_fails(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            (root / "logs/latest/latest_hub_incident.json").unlink()
            with self.assertRaisesRegex(bg.BootstrapGuardError, "required failure artifact missing"):
                bg.validate_project_bootstrap(
                    root,
                    now=datetime(2026, 7, 17, tzinfo=timezone.utc),
                    require_artifacts=True,
                )

    def test_pass_summary_does_not_require_incident_bundle(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            self._write(root, "logs/latest/latest_hub_summary.json", json.dumps({"status": "PASS"}))
            (root / "logs/latest/latest_hub_incident.json").unlink()
            report = bg.validate_project_bootstrap(
                root,
                now=datetime(2026, 7, 17, tzinfo=timezone.utc),
                require_artifacts=True,
            )
        self.assertEqual(report["artifact_count"], 2)

    def test_document_json_path_cannot_escape_project_root(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            registry_path = root / "project_rules/protected_baseline.json"
            registry = json.loads(registry_path.read_text(encoding="utf-8"))
            registry["documents"]["agent_workflow"] = "../outside/AGENTS.md"
            self._write(root, "project_rules/protected_baseline.json", json.dumps(registry))

            with self.assertRaisesRegex(bg.BootstrapGuardError, "documents.agent_workflow escapes project root"):
                bg.validate_project_bootstrap(root, now=datetime(2026, 7, 17, tzinfo=timezone.utc))

    def test_artifact_absolute_json_path_cannot_escape_project_root(self):
        with TemporaryDirectory() as tmp_dir, TemporaryDirectory() as outside_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            outside = Path(outside_dir) / "summary.json"
            outside.write_text(json.dumps({"status": "PASS"}), encoding="utf-8")
            registry_path = root / "project_rules/protected_baseline.json"
            registry = json.loads(registry_path.read_text(encoding="utf-8"))
            registry["required_artifacts"][0] = str(outside)
            self._write(root, "project_rules/protected_baseline.json", json.dumps(registry))

            with self.assertRaisesRegex(bg.BootstrapGuardError, r"required_artifacts\[0\] escapes project root"):
                bg.validate_project_bootstrap(
                    root,
                    now=datetime(2026, 7, 17, tzinfo=timezone.utc),
                    require_artifacts=True,
                )

    def test_failure_artifact_json_path_is_validated_when_summary_passes(self):
        with TemporaryDirectory() as tmp_dir, TemporaryDirectory() as outside_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            self._write(root, "logs/latest/latest_hub_summary.json", json.dumps({"status": "PASS"}))
            registry_path = root / "project_rules/protected_baseline.json"
            registry = json.loads(registry_path.read_text(encoding="utf-8"))
            registry["failure_artifact"] = str(Path(outside_dir) / "incident.json")
            self._write(root, "project_rules/protected_baseline.json", json.dumps(registry))

            with self.assertRaisesRegex(bg.BootstrapGuardError, "failure_artifact escapes project root"):
                bg.validate_project_bootstrap(
                    root,
                    now=datetime(2026, 7, 17, tzinfo=timezone.utc),
                    require_artifacts=True,
                )

    def test_manifest_files_must_be_list(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            manifest_path = root / "project_rules/current_change.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["files"] = {"path": "AGENTS.md"}
            self._write(root, "project_rules/current_change.json", json.dumps(manifest))

            with self.assertRaisesRegex(bg.BootstrapGuardError, "current change files must be a JSON list"):
                bg.validate_project_bootstrap(root, now=datetime(2026, 7, 17, tzinfo=timezone.utc))

    def test_manifest_json_path_must_be_canonical_and_inside_project(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            manifest_path = root / "project_rules/current_change.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["status"] = "ACTIVE"
            manifest["files"] = [{"path": "../outside.txt", "before": {"exists": False}}]
            self._write(root, "project_rules/current_change.json", json.dumps(manifest))

            with self.assertRaisesRegex(bg.BootstrapGuardError, r"current_change.files\[0\].path escapes project root"):
                bg.validate_project_bootstrap(root, now=datetime(2026, 7, 17, tzinfo=timezone.utc))

    def test_symlinked_json_path_cannot_escape_project_root(self):
        with TemporaryDirectory() as tmp_dir, TemporaryDirectory() as outside_dir:
            root = Path(tmp_dir)
            outside = Path(outside_dir)
            self._valid_project(root)
            (outside / "AGENTS.md").write_text("outside\n", encoding="utf-8")
            (root / "escape_link").symlink_to(outside, target_is_directory=True)
            registry_path = root / "project_rules/protected_baseline.json"
            registry = json.loads(registry_path.read_text(encoding="utf-8"))
            registry["documents"]["agent_workflow"] = "escape_link/AGENTS.md"
            self._write(root, "project_rules/protected_baseline.json", json.dumps(registry))

            with self.assertRaisesRegex(bg.BootstrapGuardError, "documents.agent_workflow escapes project root"):
                bg.validate_project_bootstrap(root, now=datetime(2026, 7, 17, tzinfo=timezone.utc))

    def test_real_project_contracts_pass(self):
        report = bg.validate_project_bootstrap(bg.PROJECT_ROOT)
        self.assertEqual(report["status"], "PASS")


if __name__ == "__main__":
    unittest.main()
