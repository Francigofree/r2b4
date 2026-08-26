#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from controller.status import (  # noqa: E402
    STATUS_TELEMETRY_EMIT_INTERVAL_SEC,
    _should_emit_status_telemetry,
)


class TestStatusTelemetryThrottle(unittest.TestCase):
    def test_status_telemetry_emit_is_throttled(self):
        ctrl = SimpleNamespace()
        start = 100.0

        self.assertTrue(_should_emit_status_telemetry(ctrl, start))
        self.assertEqual(ctrl._last_status_telemetry_emit, start)
        self.assertFalse(
            _should_emit_status_telemetry(
                ctrl,
                start + (STATUS_TELEMETRY_EMIT_INTERVAL_SEC * 0.5),
            )
        )
        self.assertEqual(ctrl._last_status_telemetry_emit, start)
        self.assertTrue(
            _should_emit_status_telemetry(
                ctrl,
                start + STATUS_TELEMETRY_EMIT_INTERVAL_SEC + 0.001,
            )
        )

    def test_status_telemetry_emit_recovers_after_clock_regression(self):
        ctrl = SimpleNamespace(_last_status_telemetry_emit=100.0)

        self.assertTrue(_should_emit_status_telemetry(ctrl, 90.0))
        self.assertEqual(ctrl._last_status_telemetry_emit, 90.0)


if __name__ == "__main__":
    unittest.main()
