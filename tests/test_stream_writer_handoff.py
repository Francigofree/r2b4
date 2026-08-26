#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import controller.stream_writer as stream_writer  # noqa: E402


class TestStreamWriterHandoff(unittest.TestCase):
    def test_idle_preview_is_disabled_by_default_config(self):
        self.assertFalse(stream_writer._idle_preview_enabled())

    def test_stream_writer_pause_active_uses_release_request_and_deadline(self):
        now = time.monotonic()
        ctrl = SimpleNamespace(_stream_writer_release_requested=False, _stream_writer_pause_until=now + 1.0)

        self.assertTrue(stream_writer._stream_writer_pause_active(ctrl, now=now))
        self.assertFalse(stream_writer._stream_writer_pause_active(ctrl, now=now + 2.0))

        ctrl._stream_writer_release_requested = True
        self.assertTrue(stream_writer._stream_writer_pause_active(ctrl, now=now + 2.0))

    def test_request_stream_writer_camera_release_waits_until_inactive(self):
        ctrl = SimpleNamespace(_stream_writer_camera_active=True)
        original_sleep = stream_writer.time.sleep

        def fake_sleep(_delay):
            ctrl._stream_writer_camera_active = False

        try:
            stream_writer.time.sleep = fake_sleep
            released = stream_writer.request_stream_writer_camera_release(ctrl, timeout_s=0.1, poll_s=0.01)
        finally:
            stream_writer.time.sleep = original_sleep

        self.assertTrue(released)
        self.assertFalse(bool(getattr(ctrl, "_stream_writer_release_requested", True)))
        self.assertGreater(float(getattr(ctrl, "_stream_writer_pause_until", 0.0)), time.monotonic())


if __name__ == "__main__":
    unittest.main()
