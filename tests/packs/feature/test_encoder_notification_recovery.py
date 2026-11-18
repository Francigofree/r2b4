"""The GPIO notification worker can die while encoder sampling stays alive.

Only process supervision and real scalar encoder transport run here. No GPIO
module, motor, device I/O or wall-time control loop is opened.
"""
from dataclasses import replace
import errno
import multiprocessing
import sys
from types import SimpleNamespace

import pytest

from rig import resolved_config
from test_sensor_recovery import _wait
from test_nav2_control_integration import _config, _inputs
from v3.adapters import process_encoder_backend as encoder
from v3.adapters.counter_encoder import SignedPulseCounterPairSnapshot, SignedPulseCounterSnapshot
from v3.adapters.live_encoder import NativeEncoderSource
from v3.async_capability import SensorUnavailable
from v3.composition.full_fake import OfflineMotorSink
from v3.composition.native_control import NativeControlComposition
from v3.contracts import DeviceHealth, DeviceHealthState, SafetyDecision, TickContext


def _edge(monkeypatch, failure='thread'):
    # Inherit only the injected GPIO seam into the child. Production keeps
    # spawn; fork here makes importing a real GPIO module impossible.
    context = multiprocessing.get_context('fork')
    alive = context.RawValue('b', True)
    monkeypatch.setattr(encoder, '_START_METHOD', 'fork')
    monkeypatch.setitem(sys.modules, 'lgpio', SimpleNamespace(
        _notify_thread=SimpleNamespace(is_alive=lambda: failure != 'thread' or bool(alive.value))))

    class Pair:
        left_counter = right_counter = SimpleNamespace(
            running=True, snapshot=lambda: SignedPulseCounterSnapshot(0))

        def __init__(self, *args, **kwargs):
            pass

        def snapshot_pair(self, context):
            stationary = SignedPulseCounterSnapshot(0)
            return SignedPulseCounterPairSnapshot(stationary, stationary, True, True,
                                                  context.monotonic_ns)

        def close(self):
            pass

        def check_notification_delivery(self, now_ns):
            if failure == 'delivery' and not alive.value:
                raise OSError(errno.EIO, 'ENCODER_NOTIFICATION_DELIVERY_STALLED:left')

    monkeypatch.setattr(encoder, 'NativeGpioSignedCounterPair', Pair)
    resolved = resolved_config()
    inputs = resolved.runtime.sensor_inputs.inputs
    edge = encoder.ProcessEncoderBackend(inputs.encoder_counter, inputs.encoder_backend,
        process_config=replace(resolved.edges.encoder_process, ready_timeout_s=.5, stop_timeout_s=.1))
    return edge, alive, inputs.encoder_source


def _read(edge):
    import time
    return edge.read(TickContext(0, time.monotonic_ns()))


@pytest.mark.parametrize('failure', ['thread', 'delivery'])
def test_notification_loss_revokes_fresh_sampling_and_recovers_new_generation(monkeypatch, failure):
    edge, alive, source_config = _edge(monkeypatch, failure)
    try:
        # A live thread without pulses is legal standstill, not a restart cue.
        initial = _read(edge)
        first = _wait(lambda: (r if (r := _read(edge)).sequence >= initial.sequence + 10 else None))
        assert first.trust == 0.  # No pulses establish no velocity measurement.
        assert first.notification_thread_alive is True
        assert edge._restarts == 0
        old_process, old_mailbox = edge._process, edge._mailbox
        alive.value = False
        _wait(lambda: not old_process.is_alive())
        with pytest.raises(SensorUnavailable):
            _read(edge)
        source = NativeEncoderSource(edge, source_config)
        stopped = source.read(TickContext(1, first.captured_monotonic_ns + 20_000_000))
        assert stopped.health.state is DeviceHealthState.DEGRADED
        assert stopped.samples == ()
        alive.value = True

        def resumed():
            try:
                reading = _read(edge)
            except SensorUnavailable:
                return None
            return reading if reading.counter_generation > first.counter_generation else None

        fresh = _wait(resumed)
        assert fresh.trust == 0. and fresh.timing_valid and not fresh.stale
        assert fresh.sequence > first.sequence
        assert fresh.captured_monotonic_ns > first.captured_monotonic_ns
        assert fresh.notification_thread_alive is True
        assert edge._process.pid != old_process.pid and not old_process.is_alive()
        assert edge._mailbox is not old_mailbox
        # Late old-generation publication has no route into the replacement.
        assert old_mailbox.publish(('reading', replace(first, sequence=fresh.sequence + 100_000)))
        assert _read(edge).counter_generation == fresh.counter_generation
        sample = source.read(TickContext(2, fresh.captured_monotonic_ns)).samples[0]
        values = {f.key: f.value for f in sample.values}
        assert values['notification_thread_alive'] is True
        assert all(type(v) in (str, int, float, bool, type(None)) for v in values.values())
    finally:
        edge.stop()
    assert not edge._process.is_alive()


def test_notification_recovery_exhaustion_is_bounded_and_terminal(monkeypatch):
    monkeypatch.setattr(encoder, '_MAX_RESTARTS', 1)
    edge, alive, _ = _edge(monkeypatch)
    try:
        alive.value = False
        _wait(lambda: edge._terminal_error)
        assert edge._attempts == 1
        assert 'ENCODER_RECOVERY_EXHAUSTED' in edge._terminal_error
        assert 'ENCODER_NOTIFICATION_THREAD_STOPPED' in edge._terminal_error
        with pytest.raises(RuntimeError, match='ENCODER_RECOVERY_EXHAUSTED'):
            _read(edge)
    finally:
        edge.stop()
    assert not edge._process.is_alive()


def test_unknown_lgpio_notification_api_is_not_assumed_healthy():
    with pytest.raises(RuntimeError, match='ENCODER_NOTIFICATION_THREAD_UNSUPPORTED'):
        encoder._require_notification_thread(SimpleNamespace())


def test_physical_pin_contradiction_detects_live_but_silent_notification_delivery():
    from v3.adapters.gpio_counter import NativeGpioSignedCounterPair
    from v3_test_fixtures import FakeGpio

    config = resolved_config().runtime.sensor_inputs.inputs.encoder_counter
    gpio = FakeGpio()
    clock = [1_000_000_000]
    pair = NativeGpioSignedCounterPair(gpio, config, monotonic_ns=lambda: clock[0])
    try:
        # Five minutes of quiet pins must never be treated as device failure.
        for tick in range(15_000):
            clock[0] += 20_000_000
            pair.check_notification_delivery(clock[0])
        channel = config.left
        gpio.emit(channel.pin_b, 1, clock[0], chip=config.gpio_chip)
        before = pair.snapshot_pair(TickContext(0, clock[0]))
        # GPIO changes physically, but no notification is delivered. Even a
        # live Python thread and fresh sampling cannot explain this as quiet.
        gpio.levels[channel.pin_b] = 0
        clock[0] += 100_000_000
        pair.check_notification_delivery(clock[0])  # tolerate bounded delay
        clock[0] += 60_000_000
        pair.check_notification_delivery(clock[0])
        # A later pin sample can alias back to its old level. The observed
        # contradiction remains until actual callback progress, not a poll.
        gpio.levels[channel.pin_b] = 1
        clock[0] += 40_000_000
        with pytest.raises(OSError, match='ENCODER_NOTIFICATION_DELIVERY_STALLED:left'):
            pair.check_notification_delivery(clock[0])
        after = pair.snapshot_pair(TickContext(1, clock[0]))
        assert after.left.pulse_count == before.left.pulse_count
        assert after.left.edge_history == before.left.edge_history
        assert after.left.last_callback_received_ns == before.left.last_callback_received_ns
        # Genuine delivery, including a B-only edge without wheel pulses,
        # clears the contradiction; no synthetic motion evidence is needed.
        gpio.emit(channel.pin_b, 0, clock[0], chip=config.gpio_chip)
        pair.check_notification_delivery(clock[0])
    finally:
        pair.close()


def test_callback_racing_physical_pin_read_cannot_trigger_restart():
    from v3.adapters.gpio_counter import NativeGpioSignedCounterPair
    from v3_test_fixtures import FakeGpio

    config = resolved_config().runtime.sensor_inputs.inputs.encoder_counter
    gpio = FakeGpio()
    clock = [1_000_000_000]
    pair = NativeGpioSignedCounterPair(gpio, config, monotonic_ns=lambda: clock[0])
    try:
        channel = config.right
        gpio.emit(channel.pin_b, 1, clock[0], chip=config.gpio_chip)
        clock[0] += 200_000_000
        original = gpio.gpio_read

        def racing_read(handle, pin):
            value = original(handle, pin)
            if pin == channel.pin_b:
                gpio.emit(pin, 0, clock[0], chip=config.gpio_chip)
            return value

        gpio.gpio_read = racing_read
        pair.check_notification_delivery(clock[0])
        gpio.gpio_read = original
        clock[0] += 200_000_000
        pair.check_notification_delivery(clock[0])
    finally:
        pair.close()


def test_restarting_health_with_published_feedback_is_still_a_hard_fault():
    config = _config()
    writer = OfflineMotorSink()
    control = NativeControlComposition(writer, config)
    try:
        control.run_tick(_inputs(config, 0))
        inputs = _inputs(config, 1)
        health = tuple(DeviceHealth(h.device_id, DeviceHealthState.DEGRADED, 'ENCODER_RESTARTING')
            if h.device_id == 'WHEEL_ENCODERS' else h for h in inputs.raw_devices.device_health)
        result = control.run_tick(replace(inputs, raw_devices=replace(inputs.raw_devices,
            device_health=health)))  # Contradiction: still contains an encoder sample.
        assert result.trace.fault_layer == 'L11'
        assert result.final_actuation.safety_decision is SafetyDecision.FAULT
        assert not result.final_actuation.enabled
        assert writer.writes[-1].left_output == writer.writes[-1].right_output == 0.
    finally:
        control.close()
