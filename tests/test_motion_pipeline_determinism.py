#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from control_loop import ControlLoop, _preserve_state_machine_motion_targets
from cont import AlbaController
from controller.command_bus import CANONICAL_STATES, normalize_command_state
from controller.commands import AsyncCommandJournalReader, poll_commands
from controller.joy_adapter import compute as joy_compute
from core.arbiter import Arbiter
from controller.motion_resolver import make_motion_proposal, resolve_motion_proposals


class _AuthOK:
    def authorize(self, token, command):
        return SimpleNamespace(ok=True, role="tester", reason="")


class _TelemetrySink:
    def emit_audit(self, *_args, **_kwargs):
        return None


def _build_ctrl(command_path: Path, max_per_tick: int = 2):
    return SimpleNamespace(
        _last_cmd_check=0.0,
        command_poll_interval_s=0.0,
        _cmd_offset=0,
        _cmd_partial_line="",
        _cmd_pending_lines=[],
        _cmd_max_per_tick=max_per_tick,
        command_path=str(command_path),
        auth=_AuthOK(),
        telemetry=_TelemetrySink(),
        mini_os=SimpleNamespace(classify_command=lambda _cmd: "navigation"),
        last_motion_denied_reason="",
        last_motion_denied_details={},
        logger=None,
    )


class _SpeedLimits:
    def __init__(self):
        self.effective_v_max = 0.6
        self.effective_w_max = 1.2
        self.gear_ratio = 1.0
        self.profile = SimpleNamespace(v_max=0.6, w_max=1.2)


class _DummyControlLoopDeps:
    def get_snapshot(self):
        return None


class _DummyEKF:
    still_this_cycle = False

    def get_state(self):
        return {}


class _DummyEKFManager:
    def __init__(self):
        self.ekf_live = _DummyEKF()
        self.ekf_shadow = _DummyEKF()

    def set_diagnostics(self, **_kwargs):
        return None

    def update(self, *_args, **_kwargs):
        return ({}, {}, False)


class _DummyStateMachine:
    def __init__(self):
        self.robot = SimpleNamespace(v_target=0.0, omega_target=0.0)

    def update(self, _dt):
        return None

    def get_current_state_name(self):
        return "IDLE"


class _DummyCore:
    def tick(self):
        return None


class TestMotionPipelineDeterminism(unittest.TestCase):
    def test_arbiter_rejects_legacy_or_incomplete_priority_lists(self):
        with self.assertRaisesRegex(ValueError, "canonical source"):
            Arbiter(priorities=["GUI", "MANUAL", "STATE", "ADAPTIVE", "AI", "CORE"])
        with self.assertRaisesRegex(ValueError, "canonical source"):
            Arbiter(priorities=["GUI_JOYSTICK", "MANUAL", "STATE"])

    def test_poll_commands_keeps_partial_json_line(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "commands.jsonl"
            head = '{"type":"noop","token":"GUI_DEFAULT","cmd_id":"c1"'
            path.write_text(head, encoding="utf-8")
            ctrl = _build_ctrl(path)

            statuses = []
            with patch("controller.commands.append_command_status", side_effect=lambda *a, **k: statuses.append((a, k))):
                poll_commands(ctrl, now=1.0)
                self.assertEqual(statuses, [])
                self.assertEqual(ctrl._cmd_pending_lines, [])
                self.assertTrue(ctrl._cmd_partial_line.startswith(head))

                with path.open("a", encoding="utf-8") as f:
                    f.write("}\n")

                poll_commands(ctrl, now=2.0)

            states = [args[1] for args, _kwargs in statuses]
            self.assertIn("applied", states)
            self.assertIn("failed", states)
            self.assertEqual(ctrl._cmd_partial_line, "")

    def test_poll_commands_process_budget_is_bounded_per_tick(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "commands.jsonl"
            rows = []
            for i in range(5):
                rows.append({"type": "noop", "token": "GUI_DEFAULT", "cmd_id": f"c{i}"})
            path.write_text("\n".join(json.dumps(r) for r in rows) + "\n", encoding="utf-8")
            ctrl = _build_ctrl(path, max_per_tick=2)

            processed = []
            with patch(
                "controller.commands.append_command_status",
                side_effect=lambda *a, **k: processed.append((a[0], a[1])),
            ):
                poll_commands(ctrl, now=1.0)
                self.assertEqual(len(ctrl._cmd_pending_lines), 3)
                poll_commands(ctrl, now=2.0)
                self.assertEqual(len(ctrl._cmd_pending_lines), 1)
                poll_commands(ctrl, now=3.0)
                self.assertEqual(len(ctrl._cmd_pending_lines), 0)

            applied_ids = [cmd_id for cmd_id, state in processed if state == "applied"]
            self.assertEqual(sorted(applied_ids), [f"c{i}" for i in range(5)])

    def test_async_command_reader_starts_from_current_offset(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "commands.jsonl"
            old = {"type": "set_twist", "token": "GUI_DEFAULT", "cmd_id": "old", "v": 0.1}
            path.write_text(json.dumps(old) + "\n", encoding="utf-8")
            initial_offset = path.stat().st_size
            new = {"type": "set_speed", "token": "GUI_DEFAULT", "cmd_id": "new", "level": 3}
            with path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(new) + "\n")

            reader = AsyncCommandJournalReader(path, initial_offset=initial_offset, max_pending=4)
            reader._poll_once()

            drained = reader.drain(4)
            self.assertEqual([cmd["cmd_id"] for cmd in drained], ["new"])
            self.assertEqual(reader.status()["pending"], 0)

    def test_async_command_reader_is_bounded(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "commands.jsonl"
            rows = [
                {"type": "set_twist", "token": "GUI_DEFAULT", "cmd_id": "c1", "v": 0.1},
                {"type": "set_twist", "token": "GUI_DEFAULT", "cmd_id": "c2", "v": 0.2},
            ]
            path.write_text("\n".join(json.dumps(row) for row in rows) + "\n", encoding="utf-8")
            reader = AsyncCommandJournalReader(path, initial_offset=0, max_pending=1)

            with patch("controller.commands.append_command_status") as append_status:
                reader._poll_once()

            status = reader.status()
            self.assertEqual(status["pending"], 1)
            self.assertEqual(status["dropped_overflow"], 1)
            self.assertEqual([cmd["cmd_id"] for cmd in reader.drain(4)], ["c1"])
            append_status.assert_called_once()

    def test_async_command_reader_default_queue_capacity_and_fifo_order(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "commands.jsonl"
            reader = AsyncCommandJournalReader(path, initial_offset=0)
            self.assertEqual(reader.status()["queue_capacity"], 256)
            reader._enqueue({"type": "set_twist", "cmd_id": "c1", "ts": time.time()})
            reader._enqueue({"type": "set_twist", "cmd_id": "c2", "ts": time.time()})

            self.assertEqual(
                [cmd["cmd_id"] for cmd in reader.drain(8)],
                ["c1", "c2"],
            )
            status = reader.status()
            self.assertEqual(status["processed"], 2)
            self.assertEqual(status["queue_depth"], 0)

    def test_async_command_reader_overflow_drops_new_normal_with_failed_status(self):
        with tempfile.TemporaryDirectory() as td, patch(
            "controller.commands.append_command_status"
        ) as append_status:
            path = Path(td) / "commands.jsonl"
            reader = AsyncCommandJournalReader(path, initial_offset=0, max_pending=2)
            reader._enqueue({"type": "set_twist", "cmd_id": "c1", "ts": time.time()})
            reader._enqueue({"type": "set_twist", "cmd_id": "c2", "ts": time.time()})
            reader._enqueue({"type": "set_twist", "cmd_id": "c3", "ts": time.time()})

            self.assertEqual([cmd["cmd_id"] for cmd in reader.drain(8)], ["c1", "c2"])
            status = reader.status()
            self.assertEqual(status["dropped_overflow"], 1)
            append_status.assert_called_once()
            self.assertEqual(append_status.call_args.args[0], "c3")
            self.assertEqual(append_status.call_args.args[1], "failed")
            self.assertEqual(
                append_status.call_args.kwargs["error_code"],
                "E_COMMAND_READER_OVERFLOW",
            )

    def test_async_command_reader_stale_commands_are_purged_by_worker_side(self):
        with tempfile.TemporaryDirectory() as td, patch(
            "controller.commands.append_command_status"
        ) as append_status:
            path = Path(td) / "commands.jsonl"
            reader = AsyncCommandJournalReader(path, initial_offset=0, max_pending=4)
            reader._enqueue(
                {
                    "type": "set_twist",
                    "cmd_id": "old",
                    "ts": time.time() - 10.0,
                    "timeout_sec": 0.1,
                }
            )
            reader._enqueue(
                {
                    "type": "set_twist",
                    "cmd_id": "fresh",
                    "ts": time.time(),
                    "timeout_sec": 10.0,
                }
            )

            reader._purge_stale_pending()

            self.assertEqual([cmd["cmd_id"] for cmd in reader.drain(8)], ["fresh"])
            status = reader.status()
            self.assertEqual(status["dropped_stale"], 1)
            self.assertEqual(status["stale_errors"], 1)
            append_status.assert_called_once()
            self.assertEqual(append_status.call_args.args[0], "old")
            self.assertEqual(
                append_status.call_args.kwargs["error_code"],
                "E_COMMAND_READER_STALE",
            )

    def test_async_command_reader_emergency_stop_drains_before_normal_backlog(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "commands.jsonl"
            reader = AsyncCommandJournalReader(path, initial_offset=0, max_pending=2)
            reader._enqueue({"type": "set_twist", "cmd_id": "c1", "ts": time.time()})
            reader._enqueue({"type": "set_twist", "cmd_id": "c2", "ts": time.time()})
            reader._enqueue({"type": "emergency_stop", "cmd_id": "estop", "ts": time.time()})

            self.assertEqual(
                [cmd["cmd_id"] for cmd in reader.drain(8)],
                ["estop", "c1", "c2"],
            )
            status = reader.status()
            self.assertEqual(status["dropped_overflow"], 0)
            self.assertEqual(status["urgent_submitted"], 1)
            self.assertEqual(status["urgent_processed"], 1)
            self.assertGreaterEqual(status["emergency_priority_bypass_count"], 1)

    def test_poll_commands_async_reader_does_not_poll_filesystem(self):
        class Reader:
            def __init__(self):
                self.drained = 0

            def drain(self, max_items):
                self.drained = max_items
                return [
                    {
                        "type": "set_speed",
                        "token": "GUI_DEFAULT",
                        "cmd_id": "speed_async",
                        "level": 4,
                        "apply_state": False,
                        "motion_source": "STATE",
                    }
                ]

            def status(self):
                return {"mode": "test_async_reader", "pending": 0}

        ctrl = _build_ctrl(Path("/must/not/be/read"), max_per_tick=2)
        reader = Reader()
        ctrl.command_input_reader = reader

        with (
            patch("controller.commands.os.path.exists", side_effect=AssertionError("filesystem poll")),
            patch("controller.commands.os.path.getsize", side_effect=AssertionError("filesystem poll")),
            patch("controller.commands.append_command_status"),
            patch("controller.commands.set_speed_level", return_value=True) as set_speed,
        ):
            poll_commands(ctrl, now=1.0)

        self.assertEqual(reader.drained, 2)
        self.assertEqual(ctrl.command_input_reader_status["mode"], "test_async_reader")
        set_speed.assert_called_once_with(ctrl, 4, source="STATE", apply_state=False)

    def test_poll_commands_strict_mode_without_reader_does_not_use_sync_fallback(self):
        ctrl = _build_ctrl(Path("/must/not/be/read"), max_per_tick=2)
        ctrl.control_thread_strict_io_free = True
        ctrl.command_input_reader = None

        with (
            patch("controller.commands.os.path.exists", side_effect=AssertionError("filesystem poll")),
            patch("controller.commands.os.path.getsize", side_effect=AssertionError("filesystem poll")),
            patch("controller.commands.json.loads", side_effect=AssertionError("json decode")),
        ):
            poll_commands(ctrl, now=1.0)

        self.assertEqual(ctrl.command_input_reader_status["mode"], "strict_reader_missing")
        self.assertFalse(ctrl.command_input_reader_status["sync_fallback_enabled"])

    def test_command_lifecycle_has_no_compatibility_aliases(self):
        self.assertEqual(CANONICAL_STATES, ("accepted", "applied", "effective", "failed"))
        self.assertEqual(normalize_command_state("applied"), ("applied", "applied"))
        self.assertEqual(normalize_command_state("executing"), ("failed", "executing"))

    def test_set_speed_dispatch_can_change_limit_without_starting_motion_state(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "commands.jsonl"
            path.write_text(
                json.dumps(
                    {
                        "type": "set_speed",
                        "token": "GUI_DEFAULT",
                        "cmd_id": "speed_limit_only",
                        "motion_source": "STATE",
                        "level": 3,
                        "apply_state": False,
                    }
                )
                + "\n",
                encoding="utf-8",
            )
            ctrl = _build_ctrl(path)

            with (
                patch("controller.commands.append_command_status"),
                patch("controller.commands.set_speed_level", return_value=True) as set_speed,
            ):
                poll_commands(ctrl, now=1.0)

            set_speed.assert_called_once_with(
                ctrl,
                3,
                source="STATE",
                apply_state=False,
            )

    def test_joy_adapter_outputs_unshaped_target_for_common_controller(self):
        ctrl = SimpleNamespace(
            joy_adapter_cfg={"joy_max_omega_rad_s": 1.2},
            turn_mix=1.0,
            speed_limits=_SpeedLimits(),
            speeds_fwd={9: 0.3},
            motion_command_source="GUI_JOYSTICK",
        )
        _v, omega = joy_compute(ctrl, x=1.0, y=0.0, dt=0.02)
        self.assertAlmostEqual(omega, -1.2, places=6)

    def test_joy_adapter_caps_omega_to_runtime_speed_profile(self):
        ctrl = SimpleNamespace(
            joy_adapter_cfg={"joy_max_omega_rad_s": 2.8},
            turn_mix=1.0,
            speed_limits=_SpeedLimits(),
            speeds_fwd={9: 0.3},
            motion_command_source="GUI_JOYSTICK",
        )
        _v, omega = joy_compute(ctrl, x=1.0, y=0.0, dt=0.02)
        self.assertAlmostEqual(omega, -1.2, places=6)

    def test_final_resolver_rejects_service_command_per_cycle(self):
        proposals = [
            make_motion_proposal(
                name="base",
                layer="MOTION_TARGET",
                source="MANUAL",
                command_type="set_twist",
                v_target=0.2,
                omega_target=0.1,
                priority=400,
            ),
            make_motion_proposal(
                name="service",
                layer="SERVICE_TEST_MOTION",
                source="SERVICE",
                command_type="set_motor_pwm",
                priority=1000,
                mode="SERVICE_TEST_MOTION",
                service_pwm={"left_pwm": 0.3, "right_pwm": -0.3},
            ),
        ]

        resolved, status = resolve_motion_proposals(proposals, active_source="MANUAL")

        self.assertNotEqual(resolved["mode"], "SERVICE_TEST_MOTION")
        self.assertEqual(resolved["command_type"], "set_twist")
        selected = [proposal for proposal in status["proposals"] if proposal["selected"]]
        self.assertEqual(len(selected), 1)
        self.assertEqual(selected[0]["name"], "base")
        self.assertEqual(int(status.get("tier_rejected_count", 0)), 1)

    def test_stale_transport_decay_does_not_inject_set_vector(self):
        cl = ControlLoop(
            encoder_service=_DummyControlLoopDeps(),
            imu_service=_DummyControlLoopDeps(),
            ekf_manager=_DummyEKFManager(),
            state_machine=_DummyStateMachine(),
            core=_DummyCore(),
            loop_hz=50.0,
        )
        ctrl = SimpleNamespace(
            recovery_mobility_mode=False,
            intent_stale_decay_s=0.2,
            _intent_was_stale=False,
            input_vector={"x": 0.4, "y": 0.2},
        )

        with patch("robot_state.get_intent", return_value=(0.4, 0.2, "GUI", 1.0, 1)), \
             patch("robot_state.is_intent_stale", return_value=True), \
             patch("robot_state.get_intent_age_s", return_value=1.05), \
             patch("robot_state.FAILSAFE_TIMEOUT_S", 1.0), \
             patch("controller.commands.set_vector") as set_vector_mock:
            cl._process_motion_intent(ctrl)

        set_vector_mock.assert_not_called()
        self.assertTrue(ctrl.transport_intent_override["active"])
        self.assertEqual(ctrl.transport_intent_override["mode"], "DECAY")

    def test_live_transport_intent_is_suppressed_during_track_reference_motion(self):
        cl = ControlLoop(
            encoder_service=_DummyControlLoopDeps(),
            imu_service=_DummyControlLoopDeps(),
            ekf_manager=_DummyEKFManager(),
            state_machine=_DummyStateMachine(),
            core=_DummyCore(),
            loop_hz=50.0,
        )
        ctrl = SimpleNamespace(
            recovery_mobility_mode=False,
            _intent_was_stale=False,
            input_vector={"x": 0.0, "y": 0.0},
            active_motion_command_layer="TRACK_REFERENCE",
            active_motion_command_type="set_track_velocity",
            _last_intent_ts=0.0,
            _last_intent_seq=0,
            transport_intent_status={},
        )

        with patch("robot_state.get_intent", return_value=(0.4, -0.2, "GUI", 10.0, 42)), \
             patch("robot_state.is_intent_stale", return_value=False), \
             patch("robot_state.get_intent_age_s", return_value=0.02), \
             patch("controller.commands.set_vector") as set_vector_mock:
            cl._process_motion_intent(ctrl)

        set_vector_mock.assert_not_called()
        self.assertEqual(ctrl._last_intent_seq, 42)
        self.assertEqual(ctrl.transport_intent_status["mode"], "SUPPRESSED_BY_ACTIVE_MOTION")
        self.assertEqual(ctrl.transport_intent_status["active_motion_layer"], "TRACK_REFERENCE")
        self.assertEqual(ctrl.transport_intent_status["active_motion_type"], "set_track_velocity")

    def test_motion_targets_publish_executed_track_surface_for_track_exec(self):
        cl = ControlLoop(
            encoder_service=_DummyControlLoopDeps(),
            imu_service=_DummyControlLoopDeps(),
            ekf_manager=_DummyEKFManager(),
            state_machine=_DummyStateMachine(),
            core=_DummyCore(),
            loop_hz=50.0,
        )
        ctrl = SimpleNamespace(
            cfg={"fizika": {"nyomtav_szelesseg_m": 0.20}},
            motion_executor=SimpleNamespace(track_width=0.20),
            track_target_left_mps=None,
            track_target_right_mps=None,
        )

        with patch("robot_state.update_targets") as update_targets_mock:
            cl._update_motion_targets(ctrl, 0.04, 0.20)

        self.assertAlmostEqual(ctrl.track_target_left_mps, 0.02, places=6)
        self.assertAlmostEqual(ctrl.track_target_right_mps, 0.06, places=6)
        update_targets_mock.assert_called_once()

    def test_arc_exec_preserves_state_machine_targets_even_with_manual_source_drift(self):
        cl = ControlLoop(
            encoder_service=SimpleNamespace(
                get_snapshot=lambda: SimpleNamespace(
                    left_velocity=0.21,
                    right_velocity=0.19,
                )
            ),
            imu_service=_DummyControlLoopDeps(),
            ekf_manager=_DummyEKFManager(),
            state_machine=_DummyStateMachine(),
            core=_DummyCore(),
            loop_hz=50.0,
        )
        cl._process_motion_intent = lambda _ctrl: None
        cl.sm.robot.v_target = 0.095
        cl.sm.robot.omega_target = 0.38
        cl.state_provider.prepare_ekf_inputs = lambda **_kwargs: {
            "dt_ekf": 0.02,
            "dt_source": "loop",
            "imu_data": {},
            "encoder_data": {},
            "gyro_z_rad": 0.0,
            "gyro_z_dps": 0.0,
            "accel_x_mps2": 0.0,
            "accel_x_g": 0.0,
            "sensor_ok": True,
            "dt_stats": {},
            "noise_stats": {},
            "encoder_enabled": False,
            "encoder_usage_gain": 0.0,
            "encoder_blend_sec": 0.0,
            "timestamps_us": {},
        }
        ctrl = SimpleNamespace(
            recovery_mobility_mode=False,
            _prev_pwm_l=0.0,
            _prev_pwm_r=0.0,
            v_target=0.0,
            omega_target=0.0,
            v_cmd=0.0,
            cfg={"vezerles": {}, "fizika": {"nyomtav_szelesseg_m": 0.2}},
            encoder_reliability=None,
            encoder_pose_fusion_active=False,
            turn_level=0,
            speed_level=0,
            motion_command_source="MANUAL",
            input_vector=None,
            turn_omega_levels={9: 1.2},
            turn_mix=1.0,
            speed_limits=_SpeedLimits(),
            motion_target_command={"active": False},
            track_velocity_command={"active": False},
            requested_motion_intent={},
            requested_track_reference={},
            active_motion_command_layer="BEHAVIOR",
            active_motion_command_type="follow_arc",
            motion_execution_mode="ARC_EXEC",
        )

        out = cl.tick(0.02, ctrl)

        self.assertAlmostEqual(ctrl.v_target, 0.095, places=6)
        self.assertAlmostEqual(ctrl.omega_target, 0.38, places=6)
        self.assertAlmostEqual(float(out["v_l_raw"]), 0.21, places=6)
        self.assertAlmostEqual(float(out["v_r_raw"]), 0.19, places=6)
        self.assertEqual(float(out["v_l"]), 0.0)
        self.assertEqual(float(out["v_r"]), 0.0)
        self.assertEqual(out["odometry_mode"], "LIDAR_FIRST")
        self.assertAlmostEqual(float(ctrl.requested_motion_intent["v"]), 0.095, places=6)
        self.assertAlmostEqual(float(ctrl.requested_motion_intent["omega"]), 0.38, places=6)

    def test_preserve_state_machine_motion_targets_stays_false_for_manual_discrete_path(self):
        ctrl = SimpleNamespace(
            motion_command_source="MANUAL",
            active_motion_command_layer="MOTION_TARGET",
            active_motion_command_type="set_speed",
            motion_execution_mode="TWIST_EXEC",
        )
        self.assertFalse(_preserve_state_machine_motion_targets(ctrl))

    def test_base_motion_proposal_uses_requested_motion_target_not_previous_shaped_output(self):
        ctrl = object.__new__(AlbaController)
        ctrl.v_target = 0.0
        ctrl.omega_target = 0.0
        ctrl.requested_motion_intent = {"v": 0.006, "omega": -0.01}
        ctrl.motion_target_command = {
            "active": True,
            "command_type": "set_twist",
            "source": "AI",
            "v": 0.065,
            "omega": -0.12,
        }
        ctrl.requested_track_reference = {}
        ctrl.motion_command_source = "AI"
        ctrl.active_motion_command_layer = "MOTION_TARGET"
        ctrl.active_motion_command_type = "set_twist"
        ctrl.motion_execution_mode = "TWIST_EXEC"

        proposal = AlbaController._base_motion_proposal(ctrl, recovery_mode=False)

        self.assertEqual(proposal["layer"], "MOTION_TARGET")
        self.assertEqual(proposal["command_type"], "set_twist")
        self.assertAlmostEqual(float(proposal["v_target"]), 0.065, places=6)
        self.assertAlmostEqual(float(proposal["omega_target"]), -0.12, places=6)

    def test_explicit_parallel_command_paths_are_reported_with_deterministic_priority(self):
        cl = ControlLoop(
            encoder_service=_DummyControlLoopDeps(),
            imu_service=_DummyControlLoopDeps(),
            ekf_manager=_DummyEKFManager(),
            state_machine=_DummyStateMachine(),
            core=_DummyCore(),
            loop_hz=50.0,
        )
        cl._process_motion_intent = lambda _ctrl: None
        cl.state_provider.prepare_ekf_inputs = lambda **_kwargs: {
            "dt_ekf": 0.02,
            "dt_source": "loop",
            "imu_data": {},
            "encoder_data": {},
            "gyro_z_rad": 0.0,
            "gyro_z_dps": 0.0,
            "accel_x_mps2": 0.0,
            "accel_x_g": 0.0,
            "sensor_ok": True,
            "dt_stats": {},
            "noise_stats": {},
            "encoder_enabled": False,
            "encoder_usage_gain": 0.0,
            "encoder_blend_sec": 0.0,
            "timestamps_us": {},
        }
        ctrl = SimpleNamespace(
            recovery_mobility_mode=False,
            _prev_pwm_l=0.0,
            _prev_pwm_r=0.0,
            v_target=0.0,
            omega_target=0.0,
            v_cmd=0.0,
            cfg={"vezerles": {}, "fizika": {"nyomtav_szelesseg_m": 0.2}},
            encoder_reliability=None,
            encoder_pose_fusion_active=False,
            turn_level=0,
            speed_level=0,
            motion_command_source="STATE",
            input_vector=None,
            turn_omega_levels={9: 1.2},
            turn_mix=1.0,
            speed_limits=_SpeedLimits(),
            motion_target_command={"active": True, "v": 0.11, "omega": 0.22},
            track_velocity_command={"active": True, "left_mps": 0.2, "right_mps": 0.2},
            requested_motion_intent={},
            requested_track_reference={},
            active_motion_command_layer="MOTION_TARGET",
            active_motion_command_type="set_twist",
            motion_execution_mode="TWIST_EXEC",
            command_arbitration_conflict_count=0,
        )

        cl.tick(0.02, ctrl)

        status = dict(getattr(ctrl, "command_arbitration_status", {}) or {})
        self.assertTrue(bool(status.get("conflict", False)))
        self.assertEqual(str(status.get("resolved_route", "")), "MOTION_TARGET_COMMAND")
        self.assertEqual(int(getattr(ctrl, "command_arbitration_conflict_count", 0)), 1)


if __name__ == "__main__":
    unittest.main()
