from dataclasses import replace

import pytest

from v3.composition.native_control import NativeControlComposition
from v3.contracts import DataField, SafetyDecision
from v3_validation_helpers import RecordingMotorSink, control_config, tick_inputs


def test_duplicate_encoder_keeps_bounded_feedback_and_checkpoint_replays():
    config = control_config()
    original = NativeControlComposition(RecordingMotorSink(), config)
    inputs = tick_inputs(5)
    original.run_tick(inputs[0])
    original.run_tick(inputs[1])
    restored = NativeControlComposition(RecordingMotorSink(), config)
    restored.restore(original.checkpoint())
    repeated = replace(inputs[2], raw_devices=replace(
        inputs[2].raw_devices,
        samples=(inputs[1].raw_devices.samples[0], *inputs[2].raw_devices.samples[1:]),
    ))
    result = original.run_tick(repeated)
    assert result.trace.fault_layer is None
    assert result.final_actuation.safety_decision is SafetyDecision.ALLOW
    assert restored.run_tick(repeated) == result


def test_frozen_imu_cannot_keep_allow_after_measurement_expires():
    config = control_config()
    production = NativeControlComposition(RecordingMotorSink(), config)
    inputs = tick_inputs(30)
    frozen = inputs[0].raw_devices.samples[1]
    for item in inputs[:15]:
        item = replace(item, raw_devices=replace(item.raw_devices, samples=tuple(
            frozen if sample.kind == "ekf_heading" else sample
            for sample in item.raw_devices.samples
        )))
        result = production.run_tick(item)
        if item.context.monotonic_ns - frozen.captured_monotonic_ns > 250_000_000:
            assert result.final_actuation.safety_decision is not SafetyDecision.ALLOW
            assert result.final_actuation.left_output == result.final_actuation.right_output == 0


@pytest.mark.parametrize("intermediate_prediction", [False, True])
def test_encoder_totals_reconcile_skipped_samples_and_intermediate_prediction(intermediate_prediction):
    from test_v3_multirate_l3_integration import (
        _admission, _batch, _encoder, _estimator, _imu, _lidar_health,
    )
    from v3.layers.l1_acquisition import acquire

    admission, estimator = _admission(), _estimator()
    t0 = 1_000_000_000

    def wheel(seq, elapsed_ns, distance):
        sample = _encoder(seq, t0 + elapsed_ns, 0.5, 0.5)
        return replace(sample, values=sample.values + (
            DataField("raw_left_distance_m", distance),
            DataField("raw_right_distance_m", distance),
            DataField("left_distance_delta_m", 0.02 if intermediate_prediction else 0.01),
            DataField("right_distance_delta_m", 0.02 if intermediate_prediction else 0.01),
        ))

    first = wheel(0, 0, 0.0)
    lidar = _lidar_health(0, t0)
    estimator(admission(acquire(_batch(0, t0, encoder=first, imu=_imu(0, t0), lidar=lidar))))
    if intermediate_prediction:
        estimator(admission(acquire(_batch(
            1, t0 + 20_000_000, encoder=first, imu=_imu(1, t0 + 20_000_000), lidar=lidar,
        ))))
    result = estimator(admission(acquire(_batch(
        2 if intermediate_prediction else 1, t0 + 40_000_000, encoder=wheel(2, 40_000_000, 0.02),
        imu=_imu(2, t0 + 40_000_000), lidar=lidar,
    ))))
    assert result.x_m == pytest.approx(0.02)
    assert result.y_m == pytest.approx(0.0)
