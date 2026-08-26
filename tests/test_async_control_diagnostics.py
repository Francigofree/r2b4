import time
from types import SimpleNamespace
from unittest.mock import Mock, patch

from controller.async_control_diagnostics import AsyncControlDiagnosticsPublisher


def test_control_diagnostics_publisher_is_latest_only_and_worker_owned():
    publisher = AsyncControlDiagnosticsPublisher()
    logger = Mock()
    ctrl = SimpleNamespace(
        logger=logger,
        log_hz=1000.0,
        sm=SimpleNamespace(get_current_state_name=lambda: "FORWARD"),
        motion_command_source="STATE",
        control_mode="UNIFIED",
        encoder_enabled=True,
        encoder_usage_gain=1.0,
        motion_quality_status={},
        encoder_reliability_status={},
        v_target=0.2,
        v_cmd=0.2,
        omega_target=0.0,
        speed_level=0,
        turn_level=0,
        control_diagnostics_publisher_status={},
    )

    try:
        with patch(
            "controller.async_control_diagnostics.get_unified_logger",
            return_value=None,
        ):
            assert publisher.submit(ctrl, {"now": 1.0, "cycle_id": 1, "ekf_state": {}, "l_sum": {}})
            for idx in range(2, 25):
                assert publisher.submit(
                    ctrl,
                    {
                        "now": float(idx),
                        "cycle_id": idx,
                        "ekf_state": {"x": idx, "y": 0.0, "theta_deg": 0.0},
                        "l_sum": {"min_dist": 1.0},
                    },
                )
            deadline = time.time() + 1.0
            while time.time() < deadline and not logger.log_telemetry.called:
                time.sleep(0.01)
    finally:
        publisher.stop(timeout_s=1.0)

    status = publisher.status()
    assert status["queue_capacity"] == 1
    assert status["latest_only"]
    assert status["processed"] >= 1
    assert status["dropped_superseded"] >= 0
    logger.log_telemetry.assert_called()
