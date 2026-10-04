import pytest
from v3.adapters.counter_encoder import CounterEncoderBackendConfig, NativeCounterEncoderBackend, SignedPulseCounterPairSnapshot, SignedPulseCounterSnapshot, SignedPulseEdge
from v3.adapters.live_encoder import NativeEncoderConfig, NativeEncoderSource
from v3.adapters.live_encoder import EncoderRejectionCode
from v3.contracts import DeviceHealthState, TickContext

class Counter:

    def __init__(self, snapshots: tuple[SignedPulseCounterSnapshot, ...], *, running: bool=True) -> None:
        self._snapshots = iter(snapshots)
        self.running = running
        self.calls = 0

    def snapshot(self) -> SignedPulseCounterSnapshot:
        self.calls += 1
        return next(self._snapshots)

def _snapshot(pulses: int, *, read_errors: int=0, invalid_alerts: int=0, edge_history: tuple[SignedPulseEdge, ...]=()) -> SignedPulseCounterSnapshot:
    return SignedPulseCounterSnapshot(pulses, read_errors, invalid_alerts, edge_history)

def _timed(pulses, start_count=0, start_ns=1000000000, end_ns=1100000000, **diagnostics):
    delta = pulses - start_count
    sign = 1 if delta > 0 else -1
    edges = tuple((SignedPulseEdge(start_ns + round(i * (end_ns - start_ns) / abs(delta)), start_count + sign * i) for i in range(abs(delta) + 1))) if delta else ()
    return _snapshot(pulses, edge_history=edges, **diagnostics)

def _config() -> CounterEncoderBackendConfig:
    return CounterEncoderBackendConfig(left_step_distance_m=0.001, right_step_distance_m=0.002, maximum_sample_interval_ns=200000000, maximum_abs_velocity_mps=1.5)

def _backend(left_values: tuple[SignedPulseCounterSnapshot, ...], right_values: tuple[SignedPulseCounterSnapshot, ...]):
    left = Counter(left_values)
    right = Counter(right_values)
    backend = NativeCounterEncoderBackend(left, right, _config())
    return (backend, left, right)

def _assert_rejected(reading) -> None:
    assert reading.trust == 0.0
    assert reading.left_mps == 0.0
    assert reading.right_mps == 0.0

def _sample_values(snapshot) -> dict[str, object]:
    return {field.key: field.value for field in snapshot.samples[0].values}

def test_signed_delta_uses_the_same_read_api_baseline_and_tick_time():
    backend, left, right = _backend((_snapshot(100), _timed(110, 100)), (_snapshot(200), _timed(195, 200)))
    backend.read(TickContext(0, 1000000000))
    reading = backend.read(TickContext(1, 1100000000))
    assert reading.left_mps == pytest.approx(0.1)
    assert reading.right_mps == pytest.approx(-0.1)
    assert reading.trust == 1.0
    assert reading.stale is False
    assert reading.timing_valid is True
    assert reading.diagnostics is not None
    assert reading.diagnostics.rejection_code is EncoderRejectionCode.NONE
    assert reading.diagnostics.raw_left_pulse_count == 110
    assert reading.diagnostics.raw_right_pulse_count == 195
    assert reading.diagnostics.left_pulse_delta == 10
    assert reading.diagnostics.right_pulse_delta == -5
    assert reading.diagnostics.sample_interval_ns == 100000000
    assert reading.diagnostics.computed_left_mps == pytest.approx(0.1)
    assert reading.diagnostics.computed_right_mps == pytest.approx(-0.1)
    assert left.calls == 2
    assert right.calls == 2
    assert not hasattr(backend, 'set_last_pwm')

    # The callback owner closes after source-read start. Edges in that interval
    # are valid, and both sample time and diagnostic interval must use closure.
    pairs = iter((
        SignedPulseCounterPairSnapshot(_snapshot(0), _snapshot(0), True, True, 1_010_000_000),
        SignedPulseCounterPairSnapshot(
            _timed(20, end_ns=1_110_000_000), _timed(10, end_ns=1_110_000_000),
            True, True, 1_120_000_000),
    ))
    backend = NativeCounterEncoderBackend(
        Counter(()), Counter(()), _config(), snapshot_pair=lambda context: next(pairs),
    )
    first = backend.read(TickContext(7, 1_000_000_000))
    second = backend.read(TickContext(8, 1_100_000_000))
    assert (first.sequence, first.captured_monotonic_ns) == (7, 1_010_000_000)
    assert (second.sequence, second.captured_monotonic_ns) == (8, 1_120_000_000)
    assert second.timing_valid and not second.stale
    assert second.diagnostics.sample_interval_ns == 110_000_000
    assert second.diagnostics.raw_left_distance_m == pytest.approx(.020)
    assert second.diagnostics.raw_right_distance_m == pytest.approx(.020)

    # Delayed Python callbacks preserve their physical GPIO event time. Only
    # bounded scalar delivery evidence crosses the native/process input edge.
    from rig import resolved_config
    from v3_test_fixtures import FakeGpio
    from v3.adapters.gpio_counter import NativeGpioSignedCounterPair
    inputs = resolved_config().runtime.sensor_inputs.inputs
    gpio = FakeGpio()
    clock_ns = [1_000_000_000]
    pair = NativeGpioSignedCounterPair(gpio, inputs.encoder_counter,
                                      monotonic_ns=lambda: clock_ns[0])
    source = NativeEncoderSource(NativeCounterEncoderBackend(
        pair.left_counter, pair.right_counter, inputs.encoder_backend,
        snapshot_pair=pair.snapshot_pair), inputs.encoder_source)
    try:
        source.read(TickContext(0, clock_ns[0]))
        channel = inputs.encoder_counter.left
        physical_ns = 1_020_000_000
        event_ns = physical_ns + channel.a_debounce_micros * 1_000
        clock_ns[0] = 1_070_000_000
        gpio.emit(channel.pin_a, 1, event_ns, chip=inputs.encoder_counter.gpio_chip)
        values = _sample_values(source.read(TickContext(1, clock_ns[0])))
        assert values['left_last_a_timestamp_ns'] == physical_ns
        assert values['left_last_callback_received_ns'] == clock_ns[0]
        assert values['left_last_callback_latency_ns'] == clock_ns[0] - event_ns
        assert values['left_max_callback_latency_ns'] == clock_ns[0] - event_ns
        clock_ns[0] += 20_000_000
        held = _sample_values(source.read(TickContext(2, clock_ns[0])))
        assert held['left_last_callback_received_ns'] == values['left_last_callback_received_ns']
        assert held['left_max_callback_latency_ns'] == values['left_max_callback_latency_ns']
        assert all(isinstance(v, (str, int, float, bool, type(None))) for v in held.values())
    finally:
        pair.close()

def test_short_callback_gap_keeps_physical_edge_velocity_until_delayed_batch():
    initial_edges = tuple((SignedPulseEdge(timestamp_ns, pulse_count) for pulse_count, timestamp_ns in enumerate(range(910000000, 1000000001, 10000000), start=1)))
    delayed_edges = initial_edges + tuple((SignedPulseEdge(timestamp_ns, pulse_count) for pulse_count, timestamp_ns in enumerate(range(1010000000, 1050000001, 10000000), start=11)))
    snapshots = (_snapshot(5, edge_history=initial_edges[:5]), _snapshot(10, edge_history=initial_edges), _snapshot(10, edge_history=initial_edges), _snapshot(10, edge_history=initial_edges), _snapshot(10, edge_history=initial_edges), _snapshot(15, edge_history=delayed_edges))
    backend, _, _ = _backend(snapshots, snapshots)
    readings = [backend.read(TickContext(tick_id, 1000000000 + tick_id * 40000000)) for tick_id in range(len(snapshots))]
    for reading in readings[1:]:
        assert reading.left_mps == pytest.approx(0.1)
        assert reading.right_mps == pytest.approx(0.2)
        assert reading.trust == 1.0
        assert reading.stale is False
        assert reading.diagnostics is not None
        assert reading.diagnostics.left_estimation_timebase == 'GPIO_EDGE_HISTORY'
        assert reading.diagnostics.right_estimation_timebase == 'GPIO_EDGE_HISTORY'
    assert readings[4].diagnostics is not None
    assert readings[4].diagnostics.raw_left_pulse_count == 10
    assert readings[4].diagnostics.left_pulse_delta == 0
    assert readings[4].diagnostics.raw_left_distance_m == pytest.approx(0.01)
    assert readings[4].diagnostics.left_distance_delta_m == pytest.approx(0.0)
    assert readings[5].diagnostics is not None
    assert readings[5].diagnostics.raw_left_pulse_count == 15
    assert readings[5].diagnostics.left_pulse_delta == 5
    assert readings[5].diagnostics.raw_left_distance_m == pytest.approx(0.015)
    assert readings[5].diagnostics.left_distance_delta_m == pytest.approx(0.005)

def test_initial_two_by_one_edge_fill_is_untrusted_without_becoming_stale():
    left_edges = (SignedPulseEdge(1010000000, 1), SignedPulseEdge(1020000000, 2))
    right_edges = (SignedPulseEdge(1015000000, 1), SignedPulseEdge(1025000000, 2))
    backend, _, _ = _backend((_snapshot(0), _snapshot(2, edge_history=left_edges), _snapshot(2, edge_history=left_edges)), (_snapshot(0), _snapshot(1, edge_history=right_edges[:1]), _snapshot(2, edge_history=right_edges)))
    baseline = backend.read(TickContext(0, 1000000000))
    filling = backend.read(TickContext(1, 1030000000))
    ready = backend.read(TickContext(2, 1040000000))
    _assert_rejected(filling)
    assert filling.stale is False
    assert filling.timing_valid is True
    assert filling.diagnostics is not None
    assert filling.diagnostics.rejection_code is EncoderRejectionCode.BASELINE
    assert filling.diagnostics.left_estimation_timebase == 'GPIO_EDGE_HISTORY'
    assert filling.diagnostics.right_estimation_timebase is None
    assert ready.stale is False
    assert ready.trust > 0.0
    assert ready.diagnostics is not None
    assert ready.diagnostics.left_estimation_timebase == 'GPIO_EDGE_HISTORY'
    assert ready.diagnostics.right_estimation_timebase == 'GPIO_EDGE_HISTORY'

    # Feed the native adapter's exact observations to L3. Velocity fitting is
    # still untrusted, while signed cumulative displacement remains usable.
    from rig import resolved_config
    from v3.contracts import AdmittedFrame, DataField, Observation, QualityState
    from v3.layers.l3_state_estimation import NativeStateEstimator
    class Readings:
        def __init__(self):
            self.readings = iter((baseline, filling, ready))
        def read(self, context):
            return next(self.readings)
    source = NativeEncoderSource(Readings(), NativeEncoderConfig("ENCODER", .5))
    estimator = NativeStateEstimator(resolved_config().runtime.composition.live_control.control.estimation)
    for tick, ns in enumerate((1000000000, 1030000000, 1040000000)):
        context = TickContext(tick, ns)
        sample = source.read(context).samples[0]
        wheel = Observation(sample.kind, sample.device_id, sample.sequence, sample.captured_monotonic_ns, sample.values)
        heading = Observation("ekf_heading", "IMU", tick, ns, (
            DataField("yaw_rad", 0.0), DataField("omega_rad_s", 0.0), DataField("confidence", 1.0)))
        estimate = estimator(AdmittedFrame(context, (wheel, heading), ()))
        assert estimate.localization_quality.local_translation is not QualityState.LOST
    assert estimate.local_pose.x_m == pytest.approx(.003)
    for step in range(1, 201):
        context = TickContext(2+step, 1040000000+step*20_000_000)
        values = {v.key: v.value for v in sample.values}
        values.update(left_mps=0.0, right_mps=0.0, trust=0.0, rejection_code="BASELINE",
                      raw_left_distance_m=.002+step*.02, raw_right_distance_m=.004+step*.02)
        wheel = Observation("wheel_velocity", "ENCODER", context.tick_id, context.monotonic_ns,
                            tuple(DataField(k, v) for k, v in values.items()))
        heading = Observation("ekf_heading", "IMU", context.tick_id, context.monotonic_ns, heading.values)
        estimate = estimator(AdmittedFrame(context, (wheel, heading), ()))
    # No relative scan validates these four metres. Missing velocity feedback
    # must not also hide accumulated displacement uncertainty.
    assert estimate.local_pose.x_m == pytest.approx(4.003)
    assert estimate.localization_quality.local_translation is QualityState.LOST

def test_processing_gap_without_dual_wheel_edge_proof_remains_stale():
    edges = tuple((SignedPulseEdge(timestamp_ns, pulse_count) for pulse_count, timestamp_ns in enumerate((1250000000, 1260000000, 1270000000, 1280000000, 1290000000), start=1)))
    backend, _, _ = _backend((_snapshot(0), _snapshot(5, edge_history=edges)), (_snapshot(0), _snapshot(5)))
    backend.read(TickContext(0, 1000000000))
    reading = backend.read(TickContext(1, 1300000000))
    _assert_rejected(reading)
    assert reading.stale is True
    assert reading.timing_valid is False
    assert reading.diagnostics is not None
    assert reading.diagnostics.rejection_code is EncoderRejectionCode.INVALID_EDGE_TIMING

def test_processing_gap_with_old_dual_wheel_edge_windows_remains_stale():
    old_edges = tuple((SignedPulseEdge(timestamp_ns, pulse_count) for pulse_count, timestamp_ns in enumerate((10000000, 20000000, 30000000, 40000000, 50000000), start=1)))
    backend, _, _ = _backend((_snapshot(0), _snapshot(5, edge_history=old_edges)), (_snapshot(0), _snapshot(5, edge_history=old_edges)))
    backend.read(TickContext(0, 1260000000))
    reading = backend.read(TickContext(1, 1300000000))
    _assert_rejected(reading)
    assert reading.stale is True
    assert reading.timing_valid is True
    assert reading.diagnostics is not None
    assert reading.diagnostics.sample_interval_ns == 40000000
    assert reading.diagnostics.rejection_code is EncoderRejectionCode.SAMPLE_INTERVAL_EXCEEDED

def test_stale_interval_is_untrusted_zero_and_reanchors_for_recovery():
    backend, _, _ = _backend((_snapshot(0), _timed(20, end_ns=1050000000), _timed(25, 20, 1300000000, 1400000000)), (_snapshot(0), _timed(10, end_ns=1050000000), _timed(12, 10, 1300000000, 1400000000)))
    backend.read(TickContext(0, 1000000000))
    stale = backend.read(TickContext(1, 1300000000))
    recovered = backend.read(TickContext(2, 1400000000))
    _assert_rejected(stale)
    assert stale.stale is True
    assert stale.timing_valid is True
    assert stale.diagnostics is not None
    assert stale.diagnostics.rejection_code is EncoderRejectionCode.SAMPLE_INTERVAL_EXCEEDED
    assert stale.diagnostics.left_pulse_delta == 20
    assert stale.diagnostics.right_pulse_delta == 10
    assert stale.diagnostics.sample_interval_ns == 300000000
    assert stale.diagnostics.computed_left_mps == pytest.approx(0.0666666667)
    assert stale.diagnostics.computed_right_mps == pytest.approx(0.0666666667)
    assert recovered.trust == 1.0
    assert recovered.left_mps == pytest.approx(0.05)
    assert recovered.right_mps == pytest.approx(0.04)

def test_stopped_counter_is_timing_invalid_untrusted_and_zero():
    backend, left, _ = _backend((_snapshot(0), _snapshot(10)), (_snapshot(0), _snapshot(10)))
    backend.read(TickContext(0, 1000000000))
    left.running = False
    reading = backend.read(TickContext(1, 1100000000))
    _assert_rejected(reading)
    assert reading.timing_valid is False
    assert reading.stale is False
    assert reading.diagnostics is not None
    assert reading.diagnostics.rejection_code is EncoderRejectionCode.COUNTER_NOT_RUNNING
    assert reading.diagnostics.left_counter_running is False
    assert reading.diagnostics.right_counter_running is True
    assert reading.diagnostics.computed_left_mps == pytest.approx(0.1)
    assert reading.diagnostics.computed_right_mps == pytest.approx(0.2)

def test_first_read_from_stopped_counter_is_invalid_zero_baseline():
    left = Counter((_snapshot(5),), running=False)
    right = Counter((_snapshot(8),))
    backend = NativeCounterEncoderBackend(left, right, _config())
    reading = backend.read(TickContext(0, 1000))
    _assert_rejected(reading)
    assert reading.timing_valid is False
    assert reading.diagnostics is not None
    assert reading.diagnostics.rejection_code is EncoderRejectionCode.COUNTER_NOT_RUNNING
