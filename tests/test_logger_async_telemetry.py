from pathlib import Path
from unittest.mock import Mock, patch

from controller import control_thread_audit
from log.logger import AlbaLogger
from log.unified_logger import CHANNEL_SAFETY, UnifiedLogger


def test_periodic_telemetry_uses_only_async_session_logger():
    unified = Mock()
    logger = AlbaLogger()

    with (
        patch("log.logger.get_unified_logger", return_value=unified),
        patch("log.logger.append_line") as append_line,
        patch("log.logger.time.time", return_value=1.0),
    ):
        logger.log_telemetry(
            elapsed=1.0,
            state="IDLE",
            x=0.0,
            y=0.0,
            th=0.0,
            l_sum={"min_dist": 1.2, "min_back": 1.3},
            vl=0.0,
            vr=0.0,
            pwml=0.0,
            pwmr=0.0,
            lvl=0,
            vt=0.0,
            vc=0.0,
            ot=0.0,
            tl=0.0,
            motion_src="NONE",
        )

    append_line.assert_not_called()
    unified.log_event.assert_called_once()
    assert unified.log_event.call_args.args[2] == "telemetry_line"


def test_safety_emergency_summary_does_not_json_serialize_on_caller():
    logger = UnifiedLogger(config={"enabled": True, "channels": {"safety": True}})
    logger._ctx = Mock(session_id="test-session")
    logger._async_logger = Mock()
    logger._async_logger.write_jsonl.return_value = True

    with patch("log.unified_logger.json.dumps", side_effect=AssertionError("json serialize")):
        assert logger.log_event(
            CHANNEL_SAFETY,
            "safety",
            "emergency_stop",
            {"reason": "lidar_front_blocked"},
            level="ERROR",
        )

    assert logger._summary["safety_stops"] == 1
    assert logger._summary["lidar_emergency_events"] == 1


def test_control_thread_text_log_does_not_sync_print_or_append():
    unified = Mock(session_dir=Path("/tmp/r2b4-session"))
    text_writer = Mock()
    text_writer.submit.return_value = True
    logger = AlbaLogger()

    control_thread_audit.configure(enabled=True)
    control_thread_audit.begin_tick(cycle_id=1, state="FORWARD", motion_active=True, motor_output_active=True)
    try:
        with (
            patch("log.logger.get_unified_logger", return_value=unified),
            patch("log.logger.append_line") as append_line,
            patch("builtins.print") as print_mock,
            patch("log.logger._TEXT_LINE_WRITER", text_writer),
        ):
            logger.warn("[SAFETY] test")
    finally:
        control_thread_audit.end_tick()
        control_thread_audit.reset_for_tests()

    print_mock.assert_not_called()
    append_line.assert_not_called()
    text_writer.submit.assert_called_once()
    unified.log_event.assert_called_once()
