import os
import unittest
from unittest.mock import patch

from log.control_snapshot import compact_control_snapshot_sections
from telemetry.logger import TelemetryLogger


class _UnifiedLoggerSink:
    def __init__(self):
        self.payload = None
        self.module = None

    def emit_telemetry(self, payload, module="telemetry"):
        self.payload = dict(payload or {})
        self.module = str(module)
        return True


class LogPayloadCompactionTests(unittest.TestCase):
    def test_full_log_flag_alone_keeps_status_telemetry_compact(self):
        sink = _UnifiedLoggerSink()
        payload = {
            "status_version": 7,
            "state": "IDLE",
            "full_log_active": True,
            "encoder": {"large_debug_tree": {"values": list(range(100))}},
            "motion_command": {"command_type": "idle", "global_motion_policy": {"large": True}},
        }
        with patch("telemetry.logger.get_unified_logger", return_value=sink):
            with patch.dict(os.environ, {"R2B4_FULL_TELEMETRY_LOG": "0"}):
                TelemetryLogger("/tmp").emit_telemetry(payload)

        self.assertTrue(sink.payload["telemetry_compacted"])
        self.assertEqual(sink.payload["telemetry_schema"], "STATUS_TELEMETRY_COMPACT_V1")
        self.assertNotIn("encoder", sink.payload)
        self.assertNotIn("global_motion_policy", sink.payload["motion_command"])

    def test_explicit_full_status_telemetry_opt_in_is_preserved(self):
        sink = _UnifiedLoggerSink()
        payload = {
            "status_version": 8,
            "state": "IDLE",
            "full_log_active": False,
            "encoder": {"large_debug_tree": {"values": [1, 2, 3]}},
        }
        with patch("telemetry.logger.get_unified_logger", return_value=sink):
            with patch.dict(os.environ, {"R2B4_FULL_TELEMETRY_LOG": "1"}):
                TelemetryLogger("/tmp").emit_telemetry(payload)

        self.assertFalse(sink.payload["telemetry_compacted"])
        self.assertEqual(sink.payload["telemetry_schema"], "STATUS_TELEMETRY_FULL_V1")
        self.assertIn("encoder", sink.payload)

    def test_control_snapshot_removes_only_declared_duplicate_sections(self):
        result = compact_control_snapshot_sections(
            motion_command={
                "command_type": "set_track_velocity",
                "requested_motion_intent": {"v": 0.0, "omega": 0.2},
                "resolver": {"duplicate": True},
                "turn_semantics": {"duplicate": True},
                "global_motion_policy": {"duplicate": True},
            },
            motion_resolution={
                "resolved": {"command_type": "set_track_velocity"},
                "proposals": [{"name": "duplicate_detail"}],
            },
            motion_semantics={
                "semantic_state": "ROTATE",
                "turn_semantics": {"duplicate": True},
            },
            motion_quality={
                "quality_state": "OK",
                "heading_controller": {"duplicate": True},
            },
        )

        self.assertEqual(result["motion_command"]["command_type"], "set_track_velocity")
        self.assertIn("requested_motion_intent", result["motion_command"])
        self.assertNotIn("resolver", result["motion_command"])
        self.assertNotIn("turn_semantics", result["motion_command"])
        self.assertNotIn("global_motion_policy", result["motion_command"])
        self.assertIn("resolved", result["motion_resolution"])
        self.assertNotIn("proposals", result["motion_resolution"])
        self.assertEqual(result["motion_semantics"], {"semantic_state": "ROTATE"})
        self.assertEqual(result["motion_quality"], {"quality_state": "OK"})


if __name__ == "__main__":
    unittest.main()
