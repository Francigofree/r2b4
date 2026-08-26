#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import unittest
from pathlib import Path
from unittest.mock import call, patch

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from tools import agent_runtime_manager as arm


class TestAgentRuntimeManager(unittest.TestCase):
    def test_runtime_status_reports_ready_when_all_gates_pass(self):
        with patch.object(arm, "_list_os_processes", return_value=[{"pid": 111, "args": "python3 os.py"}]), \
             patch.object(arm, "_read_pid_file", return_value={"pid": 111}), \
             patch.object(arm, "_pid_exists", return_value=True), \
             patch.object(
                 arm,
                 "_status_freshness",
                 return_value={
                     "exists": True,
                     "age_s": 0.1,
                     "fresh": True,
                     "state": "IDLE",
                     "status_version": 10,
                     "runtime_pid": 111,
                     "startup_ready": True,
                     "safety_allow": True,
                     "stop_type": "NONE",
                 },
             ), \
             patch.object(arm, "_http_json", return_value=(True, {"ok": True}, "")):
            st = arm.runtime_status(gui_port=7860)

        self.assertTrue(st["running"])
        self.assertTrue(st["ready_for_live_tests"])
        self.assertTrue(st["gui"]["ok"])
        self.assertTrue(st["runtime_identity"]["ok"])

    def test_runtime_status_accepts_direct_os_launch_with_stale_manager_pid_file(self):
        with patch.object(arm, "_list_os_processes", return_value=[{"pid": 2127, "args": "python3 os.py"}]), \
             patch.object(arm, "_read_pid_file", return_value={"pid": 21457}), \
             patch.object(arm, "_pid_exists", return_value=False), \
             patch.object(
                 arm,
                 "_status_freshness",
                 return_value={
                     "exists": True,
                     "age_s": 0.1,
                     "fresh": True,
                     "state": "IDLE",
                     "status_version": 10,
                     "runtime_pid": 2127,
                     "startup_ready": True,
                     "safety_allow": True,
                     "stop_type": "NONE",
                 },
             ), \
             patch.object(arm, "_http_json", return_value=(True, {"ok": True}, "")):
            st = arm.runtime_status(gui_port=7860)

        self.assertTrue(st["ready_for_live_tests"])
        self.assertFalse(st["pid_file"]["status_pid_matches"])
        self.assertTrue(st["pid_file"]["stale"])
        self.assertTrue(st["runtime_identity"]["status_pid_is_running"])

    def test_runtime_status_rejects_status_pid_not_owned_by_running_os_process(self):
        with patch.object(arm, "_list_os_processes", return_value=[{"pid": 2127, "args": "python3 os.py"}]), \
             patch.object(arm, "_read_pid_file", return_value={}), \
             patch.object(
                 arm,
                 "_status_freshness",
                 return_value={
                     "exists": True,
                     "age_s": 0.1,
                     "fresh": True,
                     "state": "IDLE",
                     "status_version": 10,
                     "runtime_pid": 9999,
                     "startup_ready": True,
                     "safety_allow": True,
                     "stop_type": "NONE",
                 },
             ), \
             patch.object(arm, "_http_json", return_value=(True, {"ok": True}, "")):
            st = arm.runtime_status(gui_port=7860)

        self.assertFalse(st["ready_for_live_tests"])
        self.assertFalse(st["runtime_identity"]["ok"])

    def test_runtime_status_rejects_multiple_os_processes(self):
        with patch.object(
                 arm,
                 "_list_os_processes",
                 return_value=[
                     {"pid": 2127, "args": "python3 os.py"},
                     {"pid": 2128, "args": "/usr/bin/python3 /tmp/os.py"},
                 ],
             ), \
             patch.object(arm, "_read_pid_file", return_value={"pid": 2127}), \
             patch.object(arm, "_pid_exists", return_value=True), \
             patch.object(
                 arm,
                 "_status_freshness",
                 return_value={
                     "exists": True,
                     "age_s": 0.1,
                     "fresh": True,
                     "state": "IDLE",
                     "status_version": 10,
                     "runtime_pid": 2127,
                     "startup_ready": True,
                     "safety_allow": True,
                     "stop_type": "NONE",
                 },
             ), \
             patch.object(arm, "_http_json", return_value=(True, {"ok": True}, "")):
            st = arm.runtime_status(gui_port=7860)

        self.assertFalse(st["ready_for_live_tests"])
        self.assertFalse(st["runtime_identity"]["unique_os_process"])

    def test_runtime_status_blocks_ready_in_failsafe(self):
        with patch.object(arm, "_list_os_processes", return_value=[{"pid": 112, "args": "python3 os.py"}]), \
             patch.object(arm, "_read_pid_file", return_value={"pid": 112}), \
             patch.object(arm, "_pid_exists", return_value=True), \
             patch.object(
                 arm,
                 "_status_freshness",
                 return_value={
                     "exists": True,
                     "age_s": 0.2,
                     "fresh": True,
                     "state": "FAILSAFE",
                     "status_version": 20,
                     "runtime_pid": 112,
                     "startup_ready": True,
                     "safety_allow": False,
                     "stop_type": "EMERGENCY_STOP",
                 },
             ), \
             patch.object(arm, "_http_json", return_value=(True, {"ok": True}, "")):
            st = arm.runtime_status(gui_port=7860)

        self.assertTrue(st["running"])
        self.assertFalse(st["ready_for_live_tests"])

    def test_resolve_runtime_python_prefers_system_python3(self):
        def _fake_exec(path: str) -> bool:
            return str(path) == "/usr/bin/python3"

        with patch.dict(arm.os.environ, {}, clear=True), \
             patch.object(arm, "_is_executable_file", side_effect=_fake_exec), \
             patch.object(arm.shutil, "which", return_value=None), \
             patch.object(arm.sys, "executable", "/home/alba/aenv/bin/python3"):
            resolved = arm._resolve_runtime_python()

        self.assertEqual(resolved, "/usr/bin/python3")

    def test_wait_for_ready_early_exits_when_started_pid_dies(self):
        stale_status = {
            "timestamp": "2026-04-03T00:00:00Z",
            "running": False,
            "ready_for_live_tests": False,
            "processes": [],
            "status": {
                "fresh": False,
                "startup_ready": False,
                "safety_allow": True,
                "state": "IDLE",
            },
        }
        with patch.object(arm, "runtime_status", side_effect=[dict(stale_status), dict(stale_status)]), \
             patch.object(arm, "_pid_exists", return_value=False), \
             patch.object(arm.time, "monotonic", side_effect=[0.0, 0.1]), \
             patch.object(arm.time, "sleep", return_value=None):
            out = arm._wait_for_ready(gui_port=7860, timeout_s=30.0, expected_pid=9999)

        self.assertTrue(out.get("early_exit"))
        self.assertEqual(out.get("early_exit_reason"), "process_exited_before_ready")
        self.assertEqual(out.get("expected_pid"), 9999)

    def test_start_runtime_reports_startup_log_tail_on_not_ready(self):
        proc = unittest.mock.Mock()
        proc.pid = 1234
        not_ready = {
            "timestamp": "2026-04-03T00:00:00Z",
            "running": False,
            "ready_for_live_tests": False,
            "processes": [],
            "status": {"startup_ready": False, "state": "STARTING"},
        }
        with patch.object(arm, "runtime_status", return_value={"running": False}), \
             patch.object(arm.subprocess, "Popen", return_value=proc) as popen_mock, \
             patch.object(arm, "_resolve_runtime_python", return_value="/usr/bin/python3"), \
             patch.object(arm, "_write_json", return_value=None), \
             patch.object(arm, "_wait_for_ready", return_value=dict(not_ready)), \
             patch.object(arm, "_tail_file_text", return_value="[STARTUP] FAILSAFE: IMU nem elérhető (FAILSAFE)"), \
             patch.object(type(arm.LOG_PATH), "open", unittest.mock.mock_open()):
            out = arm.start_runtime(gui_port=7860, ready_timeout_s=1.0, require_ready=True)

        self.assertFalse(out.get("ready_for_live_tests"))
        self.assertIn("IMU nem elérhető", str(out.get("startup_log_tail")))
        self.assertEqual(out.get("startup_failure_hint"), "imu_not_detected")
        child_env = popen_mock.call_args.kwargs["env"]
        runtime_session = arm.LOG_PATH.parent.parent
        self.assertEqual(
            child_env[arm.SESSION_ENV_VAR],
            str(runtime_session),
        )
        self.assertEqual(
            child_env[arm.TEST_SESSION_ENV_VAR],
            str(runtime_session / "tests"),
        )

    def test_start_runtime_registers_capture_manifest_for_shutdown_coordination(self):
        proc = unittest.mock.Mock()
        proc.pid = 4321
        ready = {
            "running": True,
            "ready_for_live_tests": True,
            "processes": [{"pid": 4321, "args": "python3 os.py"}],
            "status": {"startup_ready": True, "state": "IDLE"},
        }
        captured_pid_payload = {}

        def _capture_pid_payload(_path, payload):
            captured_pid_payload.update(payload)

        with patch.dict(
                 arm.os.environ,
                 {
                     arm.REPLAYER_CAPTURE_ENV_KEY: "1",
                     arm.REPLAYER_CAPTURE_ID_ENV_KEY: "capture_shutdown_contract",
                 },
                 clear=False,
             ), \
             patch.object(arm, "runtime_status", return_value={"running": False}), \
             patch.object(arm.subprocess, "Popen", return_value=proc), \
             patch.object(arm, "_resolve_runtime_python", return_value="/usr/bin/python3"), \
             patch.object(arm, "_write_json", side_effect=_capture_pid_payload), \
             patch.object(arm, "_wait_for_ready", return_value=dict(ready)), \
             patch.object(type(arm.LOG_PATH), "open", unittest.mock.mock_open()):
            out = arm.start_runtime(gui_port=7860, ready_timeout_s=1.0, require_ready=True)

        capture = dict(captured_pid_payload.get("replayer_capture") or {})
        self.assertTrue(out["ready_for_live_tests"])
        self.assertTrue(capture["enabled"])
        self.assertEqual(capture["capture_id"], "capture_shutdown_contract")
        self.assertTrue(
            str(capture["manifest_path"]).endswith(
                "capture_shutdown_contract/capture_manifest.json"
            )
        )
        self.assertEqual(capture["close_timeout_s"], arm.RUNTIME_CAPTURE_CLOSE_TIMEOUT_S)
        self.assertEqual(capture["graceful_min_s"], arm.REPLAYER_CAPTURE_GRACEFUL_MIN_S)

    def test_capture_shutdown_gets_minimum_grace_before_forced_escalation(self):
        metadata = {"enabled": True, "capture_id": "capture_shutdown_contract"}
        active = {
            "expected": True,
            "capture_id": "capture_shutdown_contract",
            "status": "ACTIVE",
            "terminal": False,
        }
        complete = {
            "expected": True,
            "capture_id": "capture_shutdown_contract",
            "status": "COMPLETE",
            "terminal": True,
        }
        with patch.object(
                 arm,
                 "_replayer_capture_state",
                 side_effect=[active, active, complete],
             ), \
             patch.object(
                 arm,
                 "_list_os_processes",
                 side_effect=[[{"pid": 111, "args": "python3 os.py"}], []],
             ), \
             patch.object(arm.time, "monotonic", side_effect=[10.0, 10.1, 10.2, 10.3]), \
             patch.object(arm.time, "sleep", return_value=None):
            result = arm._wait_for_graceful_shutdown(
                timeout_s=6.0,
                replayer_capture=metadata,
            )

        self.assertTrue(result["stopped"])
        self.assertEqual(result["requested_timeout_s"], 6.0)
        self.assertEqual(result["effective_timeout_s"], arm.REPLAYER_CAPTURE_GRACEFUL_MIN_S)
        self.assertEqual(result["capture_after"]["status"], "COMPLETE")
        self.assertTrue(result["capture_terminal_before_process_exit"])

    def test_normal_capture_shutdown_never_escalates_after_terminal_manifest(self):
        before = {
            "running": True,
            "processes": [{"pid": 111, "args": "python3 os.py"}],
        }
        after = {"running": False, "processes": []}
        graceful = {
            "stopped": True,
            "capture_terminal_observed_s": 2.75,
            "capture_terminal_before_process_exit": True,
        }
        terminal = {
            "expected": True,
            "capture_id": "capture_shutdown_contract",
            "manifest_path": "/tmp/capture_manifest.json",
            "status": "COMPLETE",
            "terminal": True,
        }
        with patch.object(arm, "runtime_status", side_effect=[before, after]), \
             patch.object(arm, "_http_json", return_value=(True, {"ok": True}, "")), \
             patch.object(
                 arm,
                 "_read_pid_file",
                 return_value={
                     "pid": 111,
                     "replayer_capture": {
                         "enabled": True,
                         "capture_id": "capture_shutdown_contract",
                     },
                 },
             ), \
             patch.object(arm, "_pid_exists", return_value=True), \
             patch.object(arm, "_send_signal") as signal_mock, \
             patch.object(arm, "_wait_for_graceful_shutdown", return_value=graceful), \
             patch.object(arm, "_replayer_capture_state", return_value=terminal), \
             patch.object(arm.time, "sleep", return_value=None), \
             patch.object(type(arm.PID_PATH), "exists", return_value=False):
            result = arm.stop_runtime(
                gui_port=7860,
                graceful_timeout_s=6.0,
                hard_timeout_s=3.0,
            )

        self.assertTrue(result["stopped"])
        self.assertTrue(result["replayer_capture_shutdown"]["terminal"])
        self.assertEqual(result["replayer_capture_shutdown"]["terminal_observed_s"], 2.75)
        self.assertFalse(result["escalation"]["sigterm"])
        self.assertFalse(result["escalation"]["sigkill"])
        self.assertEqual(signal_mock.call_args_list, [call(111, arm.signal.SIGINT)])

    def test_forced_stop_becomes_eligible_only_after_terminal_capture_manifest(self):
        metadata = {"enabled": True, "capture_id": "capture_shutdown_contract"}
        active = {
            "expected": True,
            "capture_id": "capture_shutdown_contract",
            "status": "ACTIVE",
            "terminal": False,
        }
        complete = {
            "expected": True,
            "capture_id": "capture_shutdown_contract",
            "status": "COMPLETE",
            "terminal": True,
        }
        with patch.object(
                 arm,
                 "_replayer_capture_state",
                 side_effect=[active, active, complete],
             ), \
             patch.object(
                 arm,
                 "_list_os_processes",
                 return_value=[{"pid": 111, "args": "python3 os.py"}],
             ), \
             patch.object(arm.time, "monotonic", side_effect=[0.0, 5.0, 6.1]), \
             patch.object(arm.time, "sleep", return_value=None):
            result = arm._wait_for_graceful_shutdown(
                timeout_s=6.0,
                replayer_capture=metadata,
            )

        self.assertFalse(result["stopped"])
        self.assertEqual(
            result["escalation_eligible_reason"],
            "capture_terminal_process_still_running",
        )
        self.assertEqual(result["capture_after"]["status"], "COMPLETE")
        self.assertTrue(result["capture_terminal_before_escalation"])

    def test_restart_runtime_waits_between_stop_and_start(self):
        with patch.object(arm, "stop_runtime", return_value={"stopped": True}) as stop_mock, \
             patch.object(arm, "start_runtime", return_value={"ready_for_live_tests": True}) as start_mock, \
             patch.object(arm.time, "sleep", return_value=None) as sleep_mock:
            out = arm.restart_runtime(
                gui_port=7860,
                ready_timeout_s=1.0,
                graceful_timeout_s=1.0,
                hard_timeout_s=1.0,
                restart_settle_s=2.5,
            )

        self.assertTrue(bool(out.get("ready_for_live_tests", False)))
        self.assertEqual(out.get("restart_settle_s"), 2.5)
        stop_mock.assert_called_once()
        start_mock.assert_called_once()
        sleep_mock.assert_called_once_with(2.5)


if __name__ == "__main__":
    unittest.main()
