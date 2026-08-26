import json
import tempfile
import unittest
from pathlib import Path

from controller import control_thread_audit


class ControlThreadAuditTests(unittest.TestCase):
    def setUp(self):
        control_thread_audit.reset_for_tests()
        control_thread_audit.configure(enabled=True)

    def tearDown(self):
        control_thread_audit.reset_for_tests()

    def test_file_open_inside_motion_tick_is_counted(self):
        with tempfile.NamedTemporaryFile("w", delete=False, encoding="utf-8") as handle:
            handle.write("ok")
            path = Path(handle.name)
        try:
            control_thread_audit.begin_tick(
                cycle_id=7,
                state="FORWARD",
                motion_active=True,
                motor_output_active=True,
            )
            with path.open("r", encoding="utf-8") as handle:
                self.assertEqual(handle.read(), "ok")
            control_thread_audit.end_tick()

            status = control_thread_audit.status()
            self.assertGreaterEqual(status["file_events_control_total"], 1)
            self.assertGreaterEqual(status["file_events_motion_total"], 1)
            self.assertEqual(status["json_decode_control_total"], 0)
            self.assertEqual(status["recent_events"][-1]["kind"], "file")
            self.assertTrue(status["recent_events"][-1]["motion_context"])
        finally:
            path.unlink(missing_ok=True)

    def test_json_decode_inside_idle_tick_is_counted_as_control_only(self):
        control_thread_audit.begin_tick(
            cycle_id=8,
            state="IDLE",
            motion_active=False,
            motor_output_active=False,
        )
        self.assertEqual(json.loads('{"a": 1}')["a"], 1)
        control_thread_audit.end_tick()

        status = control_thread_audit.status()
        self.assertEqual(status["json_decode_control_total"], 1)
        self.assertEqual(status["json_decode_motion_total"], 0)
        self.assertEqual(status["file_events_control_total"], 0)

    def test_work_outside_control_tick_is_not_counted(self):
        self.assertEqual(json.loads('{"a": 2}')["a"], 2)
        status = control_thread_audit.status()
        self.assertEqual(status["json_decode_control_total"], 0)
        self.assertEqual(status["file_events_control_total"], 0)


if __name__ == "__main__":
    unittest.main()
