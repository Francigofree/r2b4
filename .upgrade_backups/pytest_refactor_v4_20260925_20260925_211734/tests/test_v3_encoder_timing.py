"""Physical edge evidence: resolution, dynamics, failures and canonical replay."""
from v3_config_fixtures import configured

from bisect import bisect_right
from dataclasses import replace
import math
import random
import statistics

import pytest

from v3.adapters.counter_encoder import (
    CounterEncoderBackendConfig, NativeCounterEncoderBackend,
    SignedPulseCounterSnapshot, SignedPulseEdge,
)
from v3.adapters.live_encoder import EncoderRejectionCode, NativeEncoderConfig, NativeEncoderSource
from v3.contracts import AcquisitionFrame, TickContext, WheelVelocitySetpoint
from v3.layers.l2_admission import AdmissionConfig, InputAdmission

# Geometry and timing bounds recorded in the 2026-09-14 lifted-wheel capture.
STEP = 0.000644429262323014
START = 1_000_000_000
CONFIG = CounterEncoderBackendConfig(STEP, STEP, 100_000_000, 1.5, 4, 40_000_000, 160_000_000)


class Counter:
    running = True

    def __init__(self):
        self.value = SignedPulseCounterSnapshot(0)

    def snapshot(self):
        return self.value


def constant_edges(speed, duration=2.0, *, phase=0.3, jitter_ns=0, seed=14, step=STEP):
    if speed == 0.0:
        return ()
    rng = random.Random(seed)
    period = step / abs(speed)
    sign = 1 if speed > 0 else -1
    return tuple(
        SignedPulseEdge(START + round((i + phase) * period * 1e9) + rng.randint(-jitter_ns, jitter_ns), sign * (i + 1))
        for i in range(math.floor(duration / period))
    )


def run_edges(left_edges, right_edges=None, *, times=None, config=CONFIG):
    if right_edges is None:
        right_edges = left_edges
    if times is None:
        times = tuple(START + i * 20_000_000 for i in range(101))
    left, right = Counter(), Counter()
    backend = NativeCounterEncoderBackend(left, right, config)
    readings = []
    edge_times = [[edge.timestamp_ns for edge in edges] for edges in (left_edges, right_edges)]
    for i, now in enumerate(times):
        for counter, edges, timestamps in zip((left, right), (left_edges, right_edges), edge_times):
            end = bisect_right(timestamps, now)
            history = edges[max(0, end - 512):end]
            counter.value = SignedPulseCounterSnapshot(history[-1].pulse_count if history else 0, edge_history=history)
        context = TickContext(i, now)
        # Exercise the public backend; source serialization is checked separately.
        readings.append(backend.read(context))
    return readings


def tick_count_reference(edges, times, window_ns=40_000_000):
    """The removed count/window measurement, with its physical quantization."""
    timestamps = [edge.timestamp_ns for edge in edges]
    return [
        (bisect_right(timestamps, now) - bisect_right(timestamps, now - window_ns)) * STEP * 1e9 / window_ns
        for now in times
    ]


def edge_secant_reference(reading, side="left"):
    """Old GPIO branch used only these two physical endpoints."""
    d = reading.diagnostics
    count = getattr(d, side + "_estimation_pulse_delta")
    span = getattr(d, side + "_estimation_window_ns")
    return count * STEP * 1e9 / span


@pytest.mark.parametrize("speed", (0.0, 0.01, 0.10, 0.15, 0.30, -0.15))
def test_constant_speed_resolution_and_raw_authority(speed):
    edges = constant_edges(speed)
    readings = run_edges(edges)
    for r in readings[20:]:
        assert r.left_mps == pytest.approx(speed, abs=1e-8)
        assert r.right_mps == r.left_mps
        assert not r.stale and r.timing_valid
        d = r.diagnostics
        assert d.raw_left_distance_m == d.raw_left_pulse_count * STEP
        assert d.left_distance_delta_m == d.left_pulse_delta * STEP
        if speed:
            assert r.trust == 1.0
            assert d.left_estimation_timebase == "GPIO_EDGE_HISTORY"
            assert d.left_velocity_uncertainty_mps < 1e-7


def test_015_count_quantization_is_absent_from_timing_feedback():
    edges = constant_edges(0.15)
    readings = run_edges(edges)[20:]
    old = tick_count_reference(edges, [r.captured_monotonic_ns for r in readings])
    new = [r.left_mps for r in readings]
    assert max(old) - min(old) == pytest.approx(STEP / 0.04)
    assert statistics.pstdev(old) > 0.007
    assert max(abs(v - 0.15) for v in new) < 1e-8
    assert max(abs(a - b) for a, b in zip(new, new[1:])) < 1e-8


def test_multi_edge_fit_reduces_timestamp_jitter_without_a_longer_window():
    edges = constant_edges(0.15, duration=8.0, jitter_ns=200_000)
    times = tuple(START + i * 20_000_000 for i in range(401))
    readings = run_edges(edges, times=times)[20:]
    old = [edge_secant_reference(r) for r in readings]
    new = [r.left_mps for r in readings]
    assert statistics.pstdev(new) < 0.75 * statistics.pstdev(old)
    assert statistics.mean(abs(v - 0.15) for v in new) < 0.0004
    assert max(r.diagnostics.left_estimation_window_ns for r in readings) < 46_000_000


def test_uneven_edge_spacing_and_wheel_phase_do_not_create_mean_bias():
    base = constant_edges(0.15, duration=4.0)
    # Repeatable encoder spacing error with exactly unchanged total revolutions.
    uneven = tuple(replace(e, timestamp_ns=e.timestamp_ns + (300_000 if i % 2 else -300_000)) for i, e in enumerate(base))
    readings = run_edges(uneven, constant_edges(0.15, duration=4.0, phase=0.8))
    assert abs(statistics.mean(r.left_mps for r in readings[20:]) - 0.15) < 0.0001
    assert statistics.pstdev(r.left_mps for r in readings[20:]) < 0.0002
    assert abs(statistics.mean(r.left_mps - r.right_mps for r in readings[20:])) < 0.0001


def test_equal_physical_wheel_speed_with_different_distances_per_pulse():
    config = replace(CONFIG, right_step_distance_m=STEP * 1.1)
    readings = run_edges(constant_edges(0.15), constant_edges(0.15, step=STEP * 1.1), config=config)
    assert all(abs(r.left_mps - r.right_mps) < 1e-8 for r in readings[20:])
    assert readings[-1].diagnostics.raw_left_pulse_count != readings[-1].diagnostics.raw_right_pulse_count


def ramp_edges(v0, acceleration, duration=1.0):
    distance = v0 * duration + acceleration * duration * duration / 2
    return tuple(
        SignedPulseEdge(START + round((2 * n * STEP / (v0 + math.sqrt(v0 * v0 + 2 * acceleration * n * STEP))) * 1e9), n)
        for n in range(1, math.floor(distance / STEP) + 1)
    )


@pytest.mark.parametrize("v0,acceleration", ((0.10, 0.20), (0.30, -0.20)))
def test_acceleration_and_deceleration_latency(v0, acceleration):
    readings = run_edges(ramp_edges(v0, acceleration), times=tuple(START + i * 20_000_000 for i in range(51)))
    latencies = [(v0 + acceleration * (r.captured_monotonic_ns - START) / 1e9 - r.left_mps) / acceleration for r in readings[10:]]
    assert all(0.018 < latency < 0.033 for latency in latencies)
    assert all(r.trust == 1.0 and not r.stale for r in readings[10:])


def test_reversal_does_not_mix_directions_or_reuse_the_old_speed():
    edges = tuple(SignedPulseEdge(START + i * 5_000_000, i) for i in range(1, 21))
    reverse = tuple(SignedPulseEdge(START + (20 + i) * 5_000_000, 20 - i) for i in range(1, 21))
    times = (START, START + 100_000_000, START + 105_000_000, START + 110_000_000, START + 200_000_000)
    readings = run_edges(edges + reverse, times=times)
    assert readings[1].left_mps == pytest.approx(STEP / .005)

    # A direction boundary must reacquire enough physical-time evidence before
    # the new sign becomes control-grade.  The old forward speed is never reused.
    assert readings[2].left_mps == 0.0
    assert readings[2].diagnostics.rejection_code is EncoderRejectionCode.BASELINE
    assert readings[3].left_mps == 0.0
    assert readings[3].diagnostics.rejection_code is EncoderRejectionCode.BASELINE
    assert 0.0 < readings[3].diagnostics.left_measurement_trust < 1.0

    assert readings[4].left_mps == pytest.approx(-STEP / .005)
    assert readings[4].trust == 1.0
    assert readings[-1].diagnostics.raw_left_distance_m == 0.0


def test_missing_physical_edge_is_not_invented_in_speed_or_distance():
    original = constant_edges(.15)
    # A missed hardware pulse also stays missing in the RAW count authority.
    edges = tuple(replace(e, pulse_count=i + 1) for i, e in enumerate(original[:100] + original[101:]))
    readings = run_edges(edges)
    assert readings[-1].diagnostics.raw_left_pulse_count == len(edges)
    assert min(r.left_mps for r in readings[20:30]) > 0.12
    assert all(abs(r.left_mps - .15) < 1e-8 for r in readings[30:])


def test_stop_timeout_becomes_stationary_then_reacquires_from_a_fresh_edge_window():
    early = tuple(SignedPulseEdge(START + i * 5_000_000, i) for i in range(1, 21))
    late = tuple(
        SignedPulseEdge(START + offset_ns, 21 + index)
        for index, offset_ns in enumerate(
            (500_000_000, 510_000_000, 520_000_000, 530_000_000, 540_000_000)
        )
    )
    times = (
        START,
        START + 100_000_000,
        START + 200_000_000,
        START + 200_000_001,
        START + 500_000_000,
        START + 510_000_000,
        START + 520_000_000,
        START + 530_000_000,
        START + 540_000_000,
    )
    r = run_edges(early + late, times=times)
    assert r[2].left_mps > 0.0

    # Once the old fitted edge ages out, unchanged counts become bounded
    # stationary evidence instead of a stale-device fault.
    assert r[3].left_mps == 0.0
    assert r[3].trust == pytest.approx(100_000_001 / 160_000_000)
    assert r[3].stale is False
    assert r[3].diagnostics.rejection_code is EncoderRejectionCode.BASELINE
    assert r[3].diagnostics.left_estimation_timebase == "TICK_SNAPSHOT"

    # A single edge after the long gap is not enough.  Four fresh intervals
    # spanning the configured 40 ms window restore control-grade velocity.
    for reading in r[4:8]:
        assert reading.left_mps == 0.0
        assert reading.diagnostics.rejection_code is EncoderRejectionCode.BASELINE
    assert r[8].left_mps == pytest.approx(STEP / .010)
    assert r[8].trust == 1.0
    assert r[8].diagnostics.rejection_code is EncoderRejectionCode.NONE
    assert r[8].diagnostics.raw_left_distance_m == 25 * STEP


def test_speed_below_edge_interval_resolution_never_becomes_control_grade():
    # A ~215 ms physical edge period exceeds the 100 ms edge-continuity bound.
    # Fresh unchanged snapshots may accumulate standstill confidence, but that
    # TICK_SNAPSHOT evidence must never be promoted to non-zero wheel velocity.
    r = run_edges(constant_edges(.003))
    assert all(x.left_mps == x.right_mps == 0.0 for x in r)
    assert all(not x.stale and x.timing_valid for x in r)
    assert all(
        x.diagnostics.left_estimation_timebase != "GPIO_EDGE_HISTORY"
        for x in r
        if x.diagnostics is not None
    )


@pytest.mark.parametrize("bad", ("future", "regression", "changed_count", "lost_history"))
def test_bad_timestamp_or_lost_evidence_fails_closed_through_admission(bad):
    left, right = Counter(), Counter()
    source = NativeEncoderSource(NativeCounterEncoderBackend(left, right, CONFIG), NativeEncoderConfig("encoder", .5))
    source.read(TickContext(0, START))
    edges = (SignedPulseEdge(START + 10_000_000, 1), SignedPulseEdge(START + 20_000_000, 2))
    left.value = right.value = SignedPulseCounterSnapshot(2, edge_history=edges)
    source.read(TickContext(1, START + 20_000_000))
    bad_edges = {
        "future": (SignedPulseEdge(START + 50_000_000, 3),),
        "regression": (SignedPulseEdge(START + 15_000_000, 2),),
        "changed_count": (SignedPulseEdge(START + 20_000_000, 3),),
        "lost_history": (),
    }[bad]
    left.value = SignedPulseCounterSnapshot(bad_edges[-1].pulse_count if bad_edges else 3, edge_history=bad_edges)
    snapshot = source.read(TickContext(2, START + 40_000_000))
    values = {f.key: f.value for f in snapshot.samples[0].values}
    assert values["left_mps"] == values["right_mps"] == values["trust"] == 0.0
    assert not values["measurement_timing_valid"]
    assert values["rejection_code"] == "INVALID_EDGE_TIMING"
    frame = InputAdmission(configured(AdmissionConfig, 100_000_000))(AcquisitionFrame(snapshot.context, snapshot.samples, (snapshot.health,)))
    assert frame.degraded_sources == ("encoder",)


def test_duplicate_or_decreasing_edge_timestamps_are_rejected_at_boundary():
    for last in (10, 9):
        with pytest.raises(ValueError, match="strictly increasing"):
            SignedPulseCounterSnapshot(2, edge_history=(SignedPulseEdge(10, 1), SignedPulseEdge(last, 2)))


def test_large_epoch_and_count_and_fixed_estimation_work_bound():
    edges = constant_edges(1.4)
    shifted = tuple(SignedPulseEdge(e.timestamp_ns + 10**18, e.pulse_count + 10**12) for e in edges)
    times = tuple(START + 10**18 + i * 20_000_000 for i in range(101))
    a = run_edges(edges)
    b = run_edges(shifted, times=times)
    assert [r.left_mps for r in a[20:]] == [r.left_mps for r in b[20:]]
    assert all(r.diagnostics.left_estimation_pulse_delta <= 127 for r in b[20:])


def test_native_source_l2_pi_and_persisted_replay_are_deterministic(tmp_path):
    from v3.capture import CaptureSink
    from v3.composition.native_control import NativeControlComposition
    from v3.execution import ExecutionBoundary, IterableInputSource
    from v3.replay import replay_capture
    from v3_validation_helpers import (
        RecordingMotorSink, configuration_documents, control_config, tick_inputs,
    )

    left, right = Counter(), Counter()
    source = NativeEncoderSource(NativeCounterEncoderBackend(left, right, CONFIG), NativeEncoderConfig("WHEEL_ENCODERS", .5))
    edges = constant_edges(.15, duration=1.5, jitter_ns=200_000)
    inputs = []
    timestamps = [e.timestamp_ns for e in edges]
    for base in tick_inputs(70):
        end = bisect_right(timestamps, base.context.monotonic_ns)
        history = edges[max(0, end - 512):end]
        left.value = right.value = SignedPulseCounterSnapshot(history[-1].pulse_count if history else 0, edge_history=history)
        sample = source.read(base.context)
        raw = replace(base.raw_devices, samples=sample.samples + base.raw_devices.samples[1:], device_health=(sample.health,) + base.raw_devices.device_health[1:])
        inputs.append(replace(base, raw_devices=raw))
    # The real production L11 receives the exact admitted estimate; no second
    # speed filter is inserted at the PI boundary.
    from v3.layers.l11_actuator_control import WheelActuatorController
    config = control_config()
    pi = WheelActuatorController(config.speed_map, config.wheel_pi)
    admission = InputAdmission(configured(AdmissionConfig, 100_000_000))
    outputs = []
    for base in inputs[20:-1]:
        raw = base.raw_devices
        frame = admission(AcquisitionFrame(base.context, raw.samples, raw.device_health))
        expected = tuple(f.value for f in raw.samples[0].values if f.key in ("left_mps", "right_mps"))
        assert pi._wheel_feedback(frame) == expected
        result = pi(WheelVelocitySetpoint(base.context, .15, .15), frame)
        outputs.append(result.left_normalized)
    assert statistics.pstdev(outputs) < .0002

    sink = CaptureSink("encoder-timing", configuration=configuration_documents())
    production = NativeControlComposition(RecordingMotorSink(), config)
    ExecutionBoundary(production).run(IterableInputSource(tuple(inputs)), sink)
    path = tmp_path / "encoder-timing.json"
    sink.finalize("PASS", path)
    result = replay_capture(path)
    assert result["status"] == "MATCH"
    assert result["first_divergence"] is None
    assert run_edges(edges) == run_edges(edges)


def test_count_only_motion_never_falls_back_even_when_count_stops_changing():
    left, right = Counter(), Counter()
    backend = NativeCounterEncoderBackend(left, right, CONFIG)
    backend.read(TickContext(0, START))
    left.value = right.value = SignedPulseCounterSnapshot(10)
    for i in range(1, 20):
        r = backend.read(TickContext(i, START + i * 20_000_000))
        assert r.left_mps == r.right_mps == r.trust == 0.0
        assert not r.timing_valid
        assert r.diagnostics.rejection_code is EncoderRejectionCode.INVALID_EDGE_TIMING
        assert r.diagnostics.raw_left_distance_m == 10 * STEP


def test_first_read_future_timestamp_is_invalid_and_clean_edges_can_recover():
    left, right = Counter(), Counter()
    left.value = right.value = SignedPulseCounterSnapshot(1, edge_history=(SignedPulseEdge(START + 1_000_000_000, 1),))
    backend = NativeCounterEncoderBackend(left, right, CONFIG)
    first = backend.read(TickContext(0, START))
    assert not first.timing_valid and first.trust == 0.0
    assert first.diagnostics.rejection_code is EncoderRejectionCode.INVALID_EDGE_TIMING
    for i in range(1, 4):
        edges = tuple(SignedPulseEdge(START + k * 10_000_000, k) for k in range(1, 2 * i + 1))
        left.value = right.value = SignedPulseCounterSnapshot(edges[-1].pulse_count, edge_history=edges)
        r = backend.read(TickContext(i, START + i * 20_000_000))
    assert r.timing_valid and not r.stale and r.trust == 1.0
    assert r.left_mps == pytest.approx(STEP / .010)


def test_stationary_wheel_does_not_stale_the_moving_wheel():
    r = run_edges(constant_edges(.15), ())[2]
    assert r.stale is False
    assert r.timing_valid is True
    assert r.left_mps == pytest.approx(0.15, abs=1e-8)
    assert r.right_mps == 0.0
    assert 0.0 < r.trust < 1.0
    assert r.diagnostics.rejection_code is EncoderRejectionCode.BASELINE
    assert r.diagnostics.left_estimation_timebase == "GPIO_EDGE_HISTORY"
    assert r.diagnostics.right_estimation_timebase is None


def test_startup_standstill_confidence_needs_no_tick_count_history():
    readings = run_edges(())
    assert readings[0].trust == 0.0
    assert readings[1].trust == pytest.approx(20_000_000 / CONFIG.maximum_estimation_window_ns)
    for r in readings[8:]:
        assert r.left_mps == r.right_mps == 0.0 and r.trust == 1.0
        assert not r.stale and r.timing_valid
        assert r.diagnostics.left_estimation_timebase is None
        assert r.diagnostics.left_estimation_start_edge_timestamp_ns is None
        assert r.diagnostics.raw_left_distance_m == 0.0
