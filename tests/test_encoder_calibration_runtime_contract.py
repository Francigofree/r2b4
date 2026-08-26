from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from controller.components import _init_encoder_calibration_diagnostics


PHYSICS = {
    "lepes_hossz_m": 0.00064,
    "lepes_hossz_bal_szorzo": 1.0,
    "lepes_hossz_jobb_szorzo": 1.0,
    "nyomtav_szelesseg_m": 0.35,
}


def test_runtime_encoder_calibration_is_disabled_without_explicit_opt_in():
    ctrl = SimpleNamespace()
    with (
        patch("controller.components.EncoderCalibrationCollector") as collector,
        patch("controller.components.EncoderObservabilityGate") as gate,
    ):
        _init_encoder_calibration_diagnostics(
            ctrl,
            vezerles={},
            fizika_cfg=PHYSICS,
            track_width=0.35,
        )

    collector.assert_not_called()
    gate.assert_not_called()
    assert ctrl.encoder_calibration_runtime_collection_enabled is False
    assert ctrl.encoder_calibration_collector is None
    assert ctrl.encoder_observability_gate is None
    assert ctrl.encoder_calibration_status == {
        "runtime_collection_enabled": False,
        "state": "DISABLED",
        "reason": "EXPLICIT_OPT_IN_REQUIRED",
    }
    assert ctrl.encoder_observability_status == ctrl.encoder_calibration_status


def test_runtime_encoder_calibration_can_be_explicitly_enabled():
    ctrl = SimpleNamespace()
    collector_instance = MagicMock()
    collector_instance.get_summary.return_value = {"sample_count": 0}
    gate_instance = MagicMock()
    gate_instance.get_summary.return_value = {"calibration_allowed": False}
    with (
        patch(
            "controller.components.EncoderCalibrationCollector",
            return_value=collector_instance,
        ) as collector,
        patch(
            "controller.components.EncoderObservabilityGate",
            return_value=gate_instance,
        ) as gate,
    ):
        _init_encoder_calibration_diagnostics(
            ctrl,
            vezerles={
                "encoder_calibration": {"runtime_collection_enabled": True},
                "lidar_confidence_threshold": 0.3,
            },
            fizika_cfg=PHYSICS,
            track_width=0.35,
        )

    collector.assert_called_once()
    gate.assert_called_once()
    assert ctrl.encoder_calibration_runtime_collection_enabled is True
    assert ctrl.encoder_calibration_collector is collector_instance
    assert ctrl.encoder_observability_gate is gate_instance
    assert ctrl.encoder_calibration_status["state"] == "ACTIVE"
    assert ctrl.encoder_calibration_status["runtime_collection_enabled"] is True
    assert ctrl.encoder_observability_status["state"] == "ACTIVE"
