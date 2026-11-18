"""Missing pulses are count evidence, never a measured zero velocity."""
from dataclasses import replace

import pytest

from rig import resolved_config
from test_v3_motion_feedback_quality import _NativeFeedback
from test_v3_stationary_covariance import _frame
from v3.contracts import DataField, Observation, WheelVelocitySetpoint
from v3.layers.l11_actuator_control import WheelActuatorController
from v3.layers.l3_state_estimation import NativeStateEstimator


def test_native_pulse_loss_does_not_become_zero_measurement_or_zupt():
    resolved = resolved_config()
    config = resolved.runtime.composition.live_control.control
    native = _NativeFeedback(resolved)
    estimator = NativeStateEstimator(config.estimation)
    actuator = WheelActuatorController(config.speed_map, config.wheel_pi)
    initial_velocity = None
    fault = None
    no_edge_snapshots = 0
    for tick in range(60):
        speed = .25 if tick < 30 else 0.
        frame = native.frame(tick, speed, speed)
        wheel = frame.accepted[0]
        # A continuing gyro-rate contradicts chassis standstill throughout the
        # no-pulse interval, as in the recorded 2026-11-18 failure.
        heading = Observation('ekf_heading', 'IMU', tick, frame.context.monotonic_ns, (
            DataField('yaw_rad', 0.), DataField('omega_rad_s', .6),
            DataField('confidence', 0.), DataField('omega_confidence', 1.),
        ))
        estimate = estimator(replace(frame, accepted=frame.accepted + (heading,)))
        if tick == 29:
            initial_velocity = estimate.local_v_mps
            assert initial_velocity > .1
        if tick >= 30:
            # The physical fit endpoint has not advanced. Neither new source
            # sequence nor elapsed polling time makes an independent velocity.
            assert not any(e.update_type in {'VELOCITY', 'ZUPT'}
                           for e in estimator.last_update_evidence)
            assert estimate.local_v_mps == pytest.approx(initial_velocity)
            values = {f.key: f.value for f in wheel.values}
            if values['left_estimation_timebase'] == 'TICK_SNAPSHOT':
                no_edge_snapshots += 1
                assert values['left_mps'] == values['right_mps'] == 0.
                assert values['trust'] == 0. and values['rejection_code'] == 'BASELINE'
                assert values['left_measurement_trust'] == values['right_measurement_trust'] == 0.
        if fault is None:
            try:
                actuator(WheelVelocitySetpoint(frame.context, .19, .19), frame)
            except ValueError as exc:
                fault = exc
                assert tick >= 30
                assert 'MISSING_EDGE:left;MISSING_EDGE:right' in str(exc)
    assert no_edge_snapshots > 0
    assert fault is not None  # No change to the finite L11 edge watchdog.


@pytest.mark.parametrize('timebase_present', [True, False])
def test_old_trusted_tick_snapshot_zero_is_not_velocity_evidence(timebase_present):
    config = resolved_config().runtime.composition.live_control.control
    estimator = NativeStateEstimator(config.estimation)
    estimator(_frame(0, speed_mps=.25))
    frame = _frame(1, omega_rad_s=.6)
    wheel, *others = frame.accepted
    native_values = (
        DataField('raw_left_pulse_count', 0), DataField('raw_right_pulse_count', 0),
    )
    if timebase_present:
        native_values += (
        DataField('left_estimation_timebase', 'TICK_SNAPSHOT'),
        DataField('right_estimation_timebase', 'TICK_SNAPSHOT'),
        )
    wheel = replace(wheel, values=wheel.values + native_values)
    estimator(replace(frame, accepted=(wheel, *others)))
    assert not any(e.update_type in {'VELOCITY', 'ZUPT'} for e in estimator.last_update_evidence)


@pytest.mark.parametrize('omega,expect_zupt', [(0., True), (.6, False)])
def test_qualified_zero_requires_trusted_low_gyro_rate_for_zupt(omega, expect_zupt):
    config = resolved_config().runtime.composition.live_control.control
    estimator = NativeStateEstimator(config.estimation)
    estimator(_frame(0, omega_rad_s=omega))
    estimator(_frame(1, omega_rad_s=omega))
    assert any(e.update_type == 'ZUPT' for e in estimator.last_update_evidence) is expect_zupt


def test_missing_gyro_cannot_confirm_zupt_from_zero_wheels():
    config = resolved_config().runtime.composition.live_control.control
    estimator = NativeStateEstimator(config.estimation)
    estimator(_frame(0))
    estimator(_frame(1, include_imu=False))
    assert not any(e.update_type == 'ZUPT' for e in estimator.last_update_evidence)
