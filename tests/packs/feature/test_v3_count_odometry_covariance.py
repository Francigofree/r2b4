"""Count-owned intervals and their bounded, unmeasured prediction tail."""
from dataclasses import replace

import numpy as np
import pytest

from rig import ROOT, resolved_config
from test_v3_dual_frame_localization import observation
from test_v3_motion_feedback_quality import _NativeFeedback
from v3.capture import CaptureSink, encode_value
from v3.composition.full_fake import OfflineMotorSink
from v3.composition.native_control import NativeControlComposition
from v3.contracts import (
    CommandMode, DataField, DeviceSample,
    NavigationStatus, QualityState, TickContext,
)
from v3.contracts.navigation import Nav2NavigationStatus
from v3.execution import ExecutionRecord
from v3.layers.l3_state_estimation import NativeEstimatorStateCheckpoint, NativeStateEstimator
from v3.replay import _decode_production_value, replay_capture


LAG_NS = 30_000_000


def native_frame(native, tick, *, include_wheel=True):
    f = native.frame(tick, 0., 0.)
    wheel = replace(f.accepted[0], captured_monotonic_ns=f.context.monotonic_ns - LAG_NS)
    heading = observation("ekf_heading", tick, f.context.monotonic_ns,
        yaw_rad=0., omega_rad_s=0., confidence=0., omega_confidence=1.)
    return replace(f, accepted=((wheel,) if include_wheel else ()) + (heading,))


def test_fresh_counts_replace_velocity_uncertainty_instead_of_accumulating_it():
    resolved = resolved_config()
    config = resolved.runtime.composition.live_control.control.estimation
    # Isolate the disputed velocity term; do not infer zero speed from no edges.
    config = replace(config, process_noise=(0., 0., *config.process_noise[2:]))
    low = NativeStateEstimator(config)
    high = NativeStateEstimator(replace(config,
        initial_covariance=(*config.initial_covariance[:3], 100.01, config.initial_covariance[4])))
    native = _NativeFeedback(resolved)
    restored = None
    for tick in range(401):
        f = native_frame(native, tick)
        a, b = low(f), high(f)
        assert a.local_pose.x_m == a.local_pose.y_m == b.local_pose.x_m == b.local_pose.y_m == 0.
        assert not any(e.update_type in {"VELOCITY", "ZUPT"} for e in low.last_update_evidence)
        # The full velocity uncertainty belongs only to the 30 ms tail.
        p = low.checkpoint().local_filter.covariance
        assert p[0][0] == pytest.approx(config.initial_covariance[0] + p[3][3] * (LAG_NS/1e9)**2)
        assert high.checkpoint().local_filter.covariance[0][0] - p[0][0] == pytest.approx(.09)
        assert low.checkpoint().local_filter.encoder_anchor.captured_ns == f.context.monotonic_ns-LAG_NS
        if restored is not None:
            assert restored(f) == a
        if tick == 200:
            checkpoint = _decode_production_value(encode_value(low.checkpoint()),
                NativeEstimatorStateCheckpoint, "checkpoint")
            restored = NativeStateEstimator(config)
            restored.restore(checkpoint)
    assert p[3][3] > 4.  # No artificial cap or repeated velocity measurement.


def test_missing_counts_keep_growing_tail_and_stale_gates_then_replace_only_covered_time():
    resolved = resolved_config()
    config = resolved.runtime.composition.live_control.control.estimation
    config = replace(config, process_noise=(0., 0., *config.process_noise[2:]))
    estimator = NativeStateEstimator(config)
    native = _NativeFeedback(resolved)
    estimator(native_frame(native, 0))
    for tick in range(1, 16):
        f = native_frame(native, tick, include_wheel=False)
        result = estimator(f)
        p = estimator.checkpoint().local_filter.covariance
        age_s = (LAG_NS+tick*20_000_000)/1e9
        assert p[0][0] == pytest.approx(config.initial_covariance[0] + p[3][3]*age_s**2)
    assert result.localization_quality.local_translation is QualityState.LOST
    before = p[0][0]
    checkpoint = _decode_production_value(encode_value(estimator.checkpoint()),
        NativeEstimatorStateCheckpoint, "checkpoint")
    restored = NativeStateEstimator(config)
    restored.restore(checkpoint)
    fresh = native_frame(native, 16)
    resumed = estimator(fresh)
    assert restored(fresh) == resumed
    p = estimator.checkpoint().local_filter.covariance
    assert p[0][0] < before
    assert p[0][0] == pytest.approx(config.initial_covariance[0] + p[3][3]*(LAG_NS/1e9)**2)
    # A fresh poll cannot erase the independently required LOST recovery evidence.
    assert resumed.localization_quality.local_translation is QualityState.LOST
    assert not any(e.update_type in {"VELOCITY", "ZUPT"} for e in estimator.last_update_evidence)


@pytest.mark.parametrize("reset", ("outage", "generation", "control_gap"))
def test_count_rebase_preserves_unmeasured_motion_uncertainty_once(reset):
    resolved = resolved_config()
    config = replace(resolved.runtime.composition.live_control.control.estimation,
        process_noise=(0.,)*5, initial_covariance=(.01, .01, .01, 100., .01))
    estimator = NativeStateEstimator(config)
    native = _NativeFeedback(resolved)
    restored = None
    for tick in range(16):
        f = native_frame(native, tick, include_wheel=tick == 0 or tick >= 6)
        if tick == 0:
            source = f.accepted[0].source_device_id
        if reset == "outage" and tick == 3:
            f = replace(f, degraded_sources=(source,))
        if reset == "control_gap" and tick >= 3:
            f = replace(f, context=replace(f.context, tick_id=tick+1))
        if reset == "generation" and tick >= 6:
            wheel = replace(f.accepted[0], values=tuple(
                DataField(field.key, 1) if field.key == "counter_generation" else field
                for field in f.accepted[0].values))
            f = replace(f, accepted=(wheel, *f.accepted[1:]))
        result = estimator(f)
        # The abandoned interval remains uncertain after a new baseline;
        # later intact counts replace only the new 30 ms prediction tail.
        age_s = (LAG_NS+min(tick, 6)*20_000_000)/1e9
        if reset == "control_gap" and tick >= 3:
            age_s -= .02  # The discontinuous tick has no supported prediction.
        p = estimator.checkpoint().local_filter.covariance
        assert p[0][0] == pytest.approx(.01+100.*age_s**2)
        assert result.local_pose.x_m == result.local_pose.y_m == 0.
        if restored is not None:
            assert restored(f) == result
        if tick == 4:
            restored = NativeStateEstimator(config)
            restored.restore(estimator.checkpoint())


def test_async_count_motion_and_turns_keep_positive_semidefinite_covariance():
    resolved = resolved_config()
    config = resolved.runtime.composition.live_control.control.estimation
    estimator = NativeStateEstimator(config)
    native = _NativeFeedback(resolved)
    native.previous_ns -= LAG_NS
    restored = None
    omega = .1/config.track_width_m
    for tick in range(151):
        measured_ns = 1_000_000_000 + tick*20_000_000 - LAG_NS
        f = native.frame(tick, .2, .3, now_ns=measured_ns)
        context = TickContext(tick, measured_ns+LAG_NS)
        heading = observation("ekf_heading", tick, context.monotonic_ns,
            yaw_rad=0., omega_rad_s=omega, confidence=0., omega_confidence=1.)
        f = replace(f, context=context, accepted=(f.accepted[0], heading))
        estimate = estimator(f)
        p = np.array(estimate.covariance_5x5).reshape(5, 5)
        assert p == pytest.approx(p.T, abs=1e-12)
        assert np.linalg.eigvalsh(p).min() >= -1e-10
        if restored is not None:
            assert restored(f) == estimate
        if tick == 70:
            restored = NativeStateEstimator(config)
            restored.restore(estimator.checkpoint())
    assert np.hypot(estimate.local_pose.x_m, estimate.local_pose.y_m) > .2


def test_native_roomcruise_admission_waits_for_provider_without_false_count_hold(tmp_path):
    from test_nav2_control_integration import _config, _guidance, _inputs, _layers
    config = _config()
    resolved = resolved_config()
    native = _NativeFeedback(resolved)
    writer = OfflineMotorSink()
    control = NativeControlComposition(writer, config)
    sink = CaptureSink("count-odometry-admission", configuration={"production_control": config})
    guidance = None
    requested_since = None
    moving = False
    try:
        for tick in range(352):
            mode = CommandMode.STOP if tick < 300 else CommandMode.EXPLORE
            inputs = _inputs(config, tick, guidance, mode=mode,
                command_id="warmup" if tick < 300 else "roomcruise")
            f = native_frame(native, tick)
            wheel = f.accepted[0]
            wheel_sample = DeviceSample(wheel.source_device_id, wheel.kind,
                wheel.source_sequence, wheel.captured_monotonic_ns, wheel.values)
            samples = []
            for sample in inputs.raw_devices.samples:
                if sample.kind == "wheel_velocity":
                    sample = wheel_sample
                elif sample.kind == "ekf_heading":
                    sample = replace(sample, values=tuple(
                        DataField(field.key, 0.) if field.key == "confidence" else field
                        for field in sample.values))
                elif sample.kind == "lidar_pose":
                    # Independent delayed SLAM fixes arrive every 200 ms.
                    if tick < 10 or tick % 10:
                        continue
                    sample = replace(sample, captured_monotonic_ns=inputs.context.monotonic_ns-200_000_000)
                samples.append(sample)
            if tick >= 5 and tick % 5 == 0:
                samples.append(DeviceSample("RPLIDAR_C1", "lidar_relative_motion", tick,
                    inputs.context.monotonic_ns, tuple(DataField(k, v) for k,v in dict(
                        start_ns=inputs.context.monotonic_ns-100_000_000,
                        dx_m=0., dy_m=0., dyaw_rad=0., rmse_m=.007, observability=.5).items())))
            inputs = control.close_inputs(replace(inputs,
                raw_devices=replace(inputs.raw_devices, samples=tuple(samples))))
            result = control.run_tick(inputs)
            layers = _layers(result)
            if tick >= 300:
                estimate = layers["L3"]
                assert estimate.localization_quality.global_position is QualityState.GOOD
                request = control._navigation.pending_navigation_request
                assert request is not None  # No CANCEL/relaunch while provider starts.
                if requested_since is None:
                    requested_since = tick
                ready = tick-requested_since >= 40
                guidance = _guidance(request, sequence=tick,
                    status=Nav2NavigationStatus.ACTIVE if ready else Nav2NavigationStatus.PENDING,
                    v_mps=.25 if ready else 0.)
                if tick > requested_since+40:
                    assert layers["L6"].status is NavigationStatus.ACTIVE
                    moving |= bool(result.final_actuation.left_output or result.final_actuation.right_output)
            sink.write(ExecutionRecord(inputs, result))
    finally:
        control.close()
    assert moving and requested_since == 300
    capture = sink.finalize("PASS", tmp_path / "count-admission.json")
    replay = replay_capture(capture, project_root=ROOT)
    assert replay["status"] == "MATCH", replay["diagnostics"]
    assert replay["determinism"]["repeated_trace_match"]
