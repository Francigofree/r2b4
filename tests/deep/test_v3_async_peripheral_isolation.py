from __future__ import annotations
import threading
import time
import pytest
from v3.adapters.live_inputs import LiveDeviceSnapshot
from v3.adapters.multirate_inputs import MultiRateInputConfig, MultiRateLiveInputReader
from v3.contracts import DataField, DeviceHealth, DeviceHealthState, DeviceSample, TickContext

class FastSource:

    def __init__(self, device_id: str) -> None:
        self._device_id = device_id
        self.calls = 0
        self._lock = threading.Lock()

    @property
    def device_id(self) -> str:
        return self._device_id

    def call_count(self) -> int:
        with self._lock:
            return self.calls

    def read(self, context: TickContext) -> LiveDeviceSnapshot:
        with self._lock:
            self.calls += 1
            sequence = self.calls
        return LiveDeviceSnapshot(context, DeviceHealth(self.device_id, DeviceHealthState.OK), (DeviceSample(device_id=self.device_id, kind='test_sample', sequence=sequence, captured_monotonic_ns=context.monotonic_ns, values=(DataField('value', sequence),)),))

class BlockingAfterPrimeSource(FastSource):

    def __init__(self, device_id: str) -> None:
        super().__init__(device_id)
        self.block_entered = threading.Event()
        self.release = threading.Event()

    def read(self, context: TickContext) -> LiveDeviceSnapshot:
        with self._lock:
            self.calls += 1
            sequence = self.calls
        if sequence >= 2:
            self.block_entered.set()
            self.release.wait(timeout=2.0)
        return LiveDeviceSnapshot(context, DeviceHealth(self.device_id, DeviceHealthState.OK), (DeviceSample(device_id=self.device_id, kind='test_sample', sequence=sequence, captured_monotonic_ns=context.monotonic_ns, values=(DataField('value', sequence),)),))

class BlockingFirstReadSource(FastSource):

    def read(self, context: TickContext) -> LiveDeviceSnapshot:
        time.sleep(0.2)
        return super().read(context)

def _wait_until(predicate, timeout_s: float=1.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.002)
    raise AssertionError('condition did not become true before timeout')

def test_blocked_critical_source_does_not_stall_other_critical_streams():
    fast = FastSource('FAST')
    slow = BlockingAfterPrimeSource('SLOW')
    reader = MultiRateLiveInputReader((fast, slow), critical_device_ids=frozenset({'FAST', 'SLOW'}), config=MultiRateInputConfig(critical_default_period_ns=5000000, auxiliary_default_period_ns=20000000, history_size=8, max_snapshot_age_ns=30000000, worker_join_timeout_s=1.0, source_periods=()), monotonic_ns=time.monotonic_ns)
    try:
        assert slow.block_entered.wait(timeout=1.0)
        fast_before = fast.call_count()
        _wait_until(lambda: fast.call_count() >= fast_before + 4, timeout_s=0.5)
        now_ns = time.monotonic_ns()
        live = {item.device_id: item for item in reader.liveness_snapshot(now_ns)}
        assert live['SLOW'].read_in_flight is True
        assert live['SLOW'].read_elapsed_ns is not None
        assert live['FAST'].publication_count > live['SLOW'].publication_count
        time.sleep(0.04)
        context = reader.begin_tick(100)
        batch = reader.read(context)
        health = {item.device_id: item for item in batch.device_health}
        assert health['FAST'].state is DeviceHealthState.OK
        assert health['SLOW'].state is DeviceHealthState.UNKNOWN
        assert health['SLOW'].reason == 'L0_STREAM_EXPIRED'
    finally:
        slow.release.set()
        reader.close()

def test_critical_first_read_has_bounded_startup_wait():
    started = time.monotonic()
    with pytest.raises(RuntimeError, match='critical source prime timed out'):
        MultiRateLiveInputReader((BlockingFirstReadSource('SLOW'),), critical_device_ids=frozenset({'SLOW'}), config=MultiRateInputConfig(critical_default_period_ns=5000000, auxiliary_default_period_ns=20000000, history_size=4, worker_join_timeout_s=0.05, source_periods=()), monotonic_ns=time.monotonic_ns)
    assert time.monotonic() - started < 0.5

def test_liveness_snapshot_reports_measurement_publication_and_overruns():

    class Clock:

        def __init__(self) -> None:
            self.value = 980

        def __call__(self) -> int:
            self.value += 20
            return self.value
    source = FastSource('ENC')
    reader = MultiRateLiveInputReader((source,), critical_device_ids=frozenset({'ENC'}), config=MultiRateInputConfig(critical_default_period_ns=10, auxiliary_default_period_ns=20, history_size=4, source_periods=()), monotonic_ns=Clock(), start_workers=False)
    try:
        item = reader.liveness_snapshot(1100)[0]
        assert item.device_id == 'ENC'
        assert item.publication_count == 1
        assert item.last_measurement_ns == 1000
        assert item.last_read_duration_ns == 20
        assert item.last_publication_latency_ns == 20
        assert item.measurement_age_ns == 100
        assert item.publication_age_ns == 60
        assert item.missed_period_count == 2
        assert item.deadline_overrun_count == 1
        assert item.read_in_flight is False
    finally:
        reader.close()


def test_async_command_ingress_preserves_revision_timing_and_real_expiry(tmp_path):
    from v3.adapters.resident_command import (
        AtomicResidentCommandGateway, AsyncResidentCommandGateway,
        ResidentCommandClient, ResidentCommandMailboxConfig,
    )
    from v3.contracts import CommandMode
    config = ResidentCommandMailboxConfig(tmp_path / 'command.json')
    client = ResidentCommandClient(config)
    direct = AtomicResidentCommandGateway(config)
    edge = AsyncResidentCommandGateway(config, reader_poll_s=.002, reader_stop_timeout_s=.5)
    edge.start()
    try:
        for revision in range(3):
            client.publish_explore('same-mission', max_v_mps=.3, max_omega_rad_s=.6, ttl_ns=200_000_000)
            context = TickContext(revision, time.monotonic_ns())
            expected = direct.snapshot(context)
            _wait_until(lambda: edge.snapshot(context) == expected
                        and edge.timing_evidence.revision == client.last_revision)
            evidence = edge.timing_evidence
            assert evidence.revision == client.last_revision
            assert evidence.issued_monotonic_ns <= evidence.reader_received_ns <= evidence.observed_monotonic_ns
            assert evidence.ttl_remaining_ns == evidence.expires_monotonic_ns-evidence.observed_monotonic_ns
            assert evidence.ttl_remaining_ns > 0
        # No producer: cached bytes never acquire a new lifetime on a read.
        deadline = evidence.expires_monotonic_ns
        _wait_until(lambda: time.monotonic_ns() > deadline)
        command = edge.snapshot(TickContext(3, time.monotonic_ns()))
        assert command.mode is CommandMode.STOP
        assert '.expired.' in command.command_id
        assert edge.timing_evidence.ttl_remaining_ns < 0
        assert edge.timing_evidence.reader_received_ns == evidence.reader_received_ns
    finally:
        edge.close()


def test_encoder_process_preserves_source_time_through_stall_and_crash(tmp_path, monkeypatch):
    """Real spawn/IPC with fake GPIO: never open a device or a motor edge."""
    import os
    import signal
    from dataclasses import replace
    from rig import resolved_config
    from v3.adapters.counter_encoder import NativeCounterEncoderBackend, SignedPulseCounterSnapshot
    from v3.adapters.live_encoder import NativeEncoderSource
    from v3.adapters.process_encoder_backend import ProcessEncoderBackend

    # spawn imports this fake module ahead of any installed hardware driver.
    (tmp_path / 'lgpio.py').write_text(
        'from test_v3_encoder_ab_direction_robustness import FakeGpio\n'
        'import os\n'
        '_gpio = FakeGpio()\n'
        'RISING_EDGE = 1\nBOTH_EDGES = 3\nSET_PULL_UP = 32\n'
        'def gpiochip_open(chip):\n'
        '    if os.environ.get("R2B4_TEST_ENCODER_FAIL"): raise OSError("injected GPIO error")\n'
        '    return _gpio.gpiochip_open(chip)\n'
        'def __getattr__(name):\n    return getattr(_gpio, name)\n'
    )
    monkeypatch.syspath_prepend(str(tmp_path))
    config = resolved_config()
    inputs = config.runtime.sensor_inputs.inputs
    process_config = replace(config.edges.encoder_process, stop_timeout_s=.1)
    edge = ProcessEncoderBackend(inputs.encoder_counter, inputs.encoder_backend,
                                 process_config=process_config)
    source = NativeEncoderSource(edge, inputs.encoder_source)
    class Counter:
        running = True
        def snapshot(self):
            return SignedPulseCounterSnapshot(0)
    direct = NativeCounterEncoderBackend(Counter(), Counter(), inputs.encoder_backend)
    try:
        initial = edge.read(TickContext(0, time.monotonic_ns()))
        direct.read(TickContext(initial.sequence, initial.captured_monotonic_ns))
        _wait_until(lambda: edge.read(TickContext(0, time.monotonic_ns())).captured_monotonic_ns
                    - initial.captured_monotonic_ns > inputs.encoder_backend.maximum_estimation_window_ns)
        os.kill(edge.pid, signal.SIGSTOP)
        # Consume already completed transport values, then read the held state.
        time.sleep(.03)
        observed_ns = time.monotonic_ns()
        reading = edge.read(TickContext(1, observed_ns))
        expected = direct.read(TickContext(reading.sequence, reading.captured_monotonic_ns))
        assert (reading.left_mps, reading.right_mps, reading.trust, reading.stale,
                reading.timing_valid) == (expected.left_mps, expected.right_mps,
                expected.trust, expected.stale, expected.timing_valid)
        assert reading.diagnostics.raw_left_distance_m == expected.diagnostics.raw_left_distance_m
        assert reading.diagnostics.raw_right_distance_m == expected.diagnostics.raw_right_distance_m
        samples = []
        started_ns = time.monotonic_ns()
        for tick in range(50):
            context = TickContext(tick, time.monotonic_ns())
            samples.append(source.read(context).samples[0])
        assert time.monotonic_ns() - started_ns < 100_000_000
        assert all(s.sequence == reading.sequence and s.captured_monotonic_ns == reading.captured_monotonic_ns
                   for s in samples)
        assert all(isinstance(f.value, (str, int, float, bool, type(None))) for f in samples[-1].values)
        # Parent polling never renews the source measurement's lifetime.
        expired_ns = reading.captured_monotonic_ns + 250_000_001
        assert source.capability_snapshot(expired_ns).state.value == 'STALE'
        os.kill(edge.pid, signal.SIGKILL)
        def failed():
            try:
                source.read(TickContext(100, time.monotonic_ns()))
            except RuntimeError as exc:
                assert 'ENCODER_PROCESS_EXITED' in str(exc)
                return True
            return False
        _wait_until(failed)
    finally:
        started_ns = time.monotonic_ns()
        edge.stop()
        assert time.monotonic_ns() - started_ns < 5_000_000_000
    # A stopped process cannot handle SIGTERM. Shutdown must still be bounded,
    # without requiring it to release an Event/Condition mutex first.
    edge = ProcessEncoderBackend(inputs.encoder_counter, inputs.encoder_backend,
                                 process_config=process_config)
    os.kill(edge.pid, signal.SIGSTOP)
    started_ns = time.monotonic_ns()
    edge.stop()
    assert time.monotonic_ns() - started_ns < 1_000_000_000
    with pytest.raises(ProcessLookupError):
        os.kill(edge.pid, 0)
    monkeypatch.setenv('R2B4_TEST_ENCODER_FAIL', '1')
    with pytest.raises(RuntimeError, match='startup failed|exited during startup'):
        ProcessEncoderBackend(inputs.encoder_counter, inputs.encoder_backend,
                              process_config=process_config)
