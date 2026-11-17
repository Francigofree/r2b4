"""Motorless recovery evidence: fresh generations, bounded failure and replay."""
from dataclasses import replace
from functools import partial
import errno
import multiprocessing
from pathlib import Path
import queue
import time

import pytest

from rig import ROOT, resolved_config
from v3.adapters import process_imu_device as imu
from v3.adapters.bno055_device import NativeBno055DeviceConfig
from v3.adapters.process_lidar_port import _LidarStateMailbox
from v3.async_capability import SensorUnavailable
from v3.capture import CaptureSink
from v3.composition.full_fake import OfflineMotorSink
from v3.composition.native_control import NativeControlComposition
from v3.contracts import DataField, DeviceHealth, DeviceHealthState, SafetyDecision
from v3.execution import ExecutionRecord
from v3.replay import replay_capture
from test_nav2_control_integration import _config, _guidance, _inputs, _layers


class _RegisterBus:
    def __init__(self, failing=False):
        self.failing, self.bursts = failing, 0

    def read_byte_data(self, address, register):
        return {0: 0xA0, 0x35: 0xFF, 0x39: 5, 0x3A: 0}.get(register, 0)

    def write_byte_data(self, *args):
        pass

    def read_i2c_block_data(self, *args):
        self.bursts += 1
        if self.failing and self.bursts >= 5:
            raise OSError(errno.EREMOTEIO, "injected repeated I2C failure")
        return [0] * 32

    def close(self):
        pass


def _open_bus(marker, bus_number):
    marker = Path(marker)
    failing = not marker.exists()
    marker.touch()
    return _RegisterBus(failing)


def _wait(predicate, timeout=5.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(.005)
    raise AssertionError("recovery deadline exceeded")


def test_repeated_i2c_errors_reopen_bus_and_never_refresh_failed_sample(tmp_path):
    policy = replace(resolved_config().edges.imu_process, stop_timeout_s=.1)
    device = imu.ProcessBno055Device(NativeBno055DeviceConfig(),
        open_bus=partial(_open_bus, str(tmp_path/'opened')), process_config=policy)
    try:
        first, first_pid = device.read_sample(force=True), device._process.pid
        old_state = device._state
        _wait(lambda: old_state.failed.value)
        with pytest.raises(SensorUnavailable):
            device.read_sample(force=True)
        def recovered():
            try:
                sample = device.read_sample(force=True)
            except SensorUnavailable:
                return None
            return sample if device._process.pid != first_pid else None
        fresh = _wait(recovered)
        assert fresh['sequence'] > first['sequence']
        assert fresh['timestamp'] > first['timestamp']
        assert fresh['calibration'] == first['calibration']
        assert not old_state.process.is_alive()
        assert device.initialized and device.sensor_ok
    finally:
        device.close()
    assert not device._process.is_alive()


def test_imu_recovery_exhaustion_is_terminal_and_bounded(tmp_path, monkeypatch):
    marker = tmp_path/'opened'
    # Removing the marker makes each reopened bus fail again.
    device = imu.ProcessBno055Device(NativeBno055DeviceConfig(),
        open_bus=partial(_open_bus, str(marker)),
        process_config=replace(resolved_config().edges.imu_process, stop_timeout_s=.1))
    monkeypatch.setattr(imu, '_MAX_RESTARTS', 1)
    try:
        _wait(lambda: device._state.failed.value)
        marker.unlink()
        _wait(lambda: device._terminal_error)
        with pytest.raises(RuntimeError, match='IMU_RECOVERY_EXHAUSTED'):
            device.read_sample(force=True)
        assert not device.initialized and not device.sensor_ok
    finally:
        device.close()


def _abandon_lock(lock, ready):
    lock.acquire()
    ready.set()
    time.sleep(30)


def test_compact_lidar_transport_cannot_block_on_dead_publisher():
    context = multiprocessing.get_context('spawn')
    mailbox = _LidarStateMailbox(context)
    mailbox.put_nowait(('state', 7))
    assert mailbox.get_nowait() == ('state', 7)
    ready = context.Event()
    child = context.Process(target=_abandon_lock, args=(mailbox._lock, ready))
    child.start()
    try:
        assert ready.wait(3.)
        child.kill()
        child.join(2.)
        started = time.monotonic()
        for _ in range(100):
            with pytest.raises(queue.Empty):
                mailbox.get_nowait()
        assert time.monotonic() - started < .1
        replacement = _LidarStateMailbox(context)
        replacement.put_nowait(('state', 8))
        assert replacement.get_nowait() == ('state', 8)
    finally:
        if child.is_alive():
            child.kill()
            child.join(2.)


def test_recovering_sensors_stop_without_latching_and_replay_closed_recovery(tmp_path):
    config = _config()
    writer = OfflineMotorSink()
    control = NativeControlComposition(writer, config)
    sink = CaptureSink('sensor-recovery', configuration={'production_control': config})
    proposal = None
    mission_id = None
    try:
        for tick in range(9):
            inputs = _inputs(config, tick, proposal)
            if tick in (2, 3):
                health = tuple(DeviceHealth(h.device_id, DeviceHealthState.DEGRADED, 'IMU_RESTARTING')
                    if h.device_id == 'BNO055_IMU' else h for h in inputs.raw_devices.device_health)
                inputs = replace(inputs, raw_devices=replace(inputs.raw_devices,
                    device_health=health, samples=tuple(s for s in inputs.raw_devices.samples
                        if s.device_id != 'BNO055_IMU')))
            closed = control.close_inputs(inputs)
            result = control.run_tick(closed)
            if tick == 0:
                proposal = _guidance(control._navigation.pending_navigation_request)
                mission_id = proposal.mission_id
            if tick in (2, 3):
                assert result.final_actuation.safety_decision is SafetyDecision.STOP
                assert not result.final_actuation.enabled
                assert result.final_actuation.left_output == result.final_actuation.right_output == 0.
            assert result.final_actuation.safety_decision is not SafetyDecision.FAULT
            assert _layers(result)['L5'].mission_id == mission_id
            sink.write(ExecutionRecord(closed, result))
        assert any(w.enabled for w in writer.writes[4:])
    finally:
        control.close()
    capture = sink.finalize('PASS', tmp_path/'recovery.json')
    replay = replay_capture(capture, project_root=ROOT)
    assert replay['status'] == 'MATCH', replay['diagnostics']


def test_lidar_owner_crash_restarts_fresh_transport_and_source_identity(monkeypatch, tmp_path):
    import sys
    from types import SimpleNamespace
    from v3.adapters import process_lidar_port as lidar
    from v3.adapters.native_lidar_port import NativeRawLidarSnapshot
    context = multiprocessing.get_context('fork')
    connected = context.RawValue('b', True)
    resolved = resolved_config()
    class Port:
        def __init__(self, base):
            self.sequence = base
        def get_raw_scan_snapshot(self):
            self.sequence += 1
            stamp = time.monotonic_ns()
            return NativeRawLidarSnapshot(self.sequence, stamp/1e9, stamp, stamp, stamp,
                'OK', (), {})
        def get_matcher_result(self): return None
        def get_relative_result(self): return None
        def get_runtime_status(self):
            return dict(running=True, health='OK' if connected.value else 'ERROR',
                driver_connected=bool(connected.value), matcher_process_alive=True)
        def stop(self): pass
    monkeypatch.setattr(lidar, '_PROCESS_START_METHOD', 'fork')
    monkeypatch.setitem(sys.modules, 'serial', SimpleNamespace(Serial=None))
    monkeypatch.setattr(lidar, 'open_native_lidar_port',
        lambda *a, **kw: Port(kw.get('source_sequence_base', 0)))
    policy = replace(resolved.edges.lidar_process, stop_timeout_s=.1)
    edge = lidar.ProcessLidarPort(resolved.lidar, lambda _: None,
        process_config=policy, slam_socket_path=tmp_path/'no-slam.sock')
    try:
        first = edge.get_raw_scan_snapshot()
        pid = edge._process.pid
        connected.value = False
        def unavailable():
            try:
                edge.get_raw_scan_snapshot()
            except SensorUnavailable:
                return True
            return False
        _wait(unavailable)
        connected.value = True
        def reconnected():
            try:
                raw = edge.get_raw_scan_snapshot()
            except SensorUnavailable:
                return None
            return raw if raw.measurement_monotonic_ns > first.measurement_monotonic_ns else None
        local = _wait(reconnected)
        assert edge._process.pid == pid  # existing serial reconnect owns this case
        edge._process.kill()
        _wait(unavailable)
        old_mailbox = edge._state_queue
        def restarted():
            try:
                raw = edge.get_raw_scan_snapshot()
            except SensorUnavailable:
                return None
            return raw if edge._process.pid != pid else None
        new = _wait(restarted)
        assert edge._state_queue is not old_mailbox
        assert new.raw_scan_id > local.raw_scan_id
        assert new.measurement_monotonic_ns > local.measurement_monotonic_ns
    finally:
        edge.stop()
    assert not edge._process.is_alive()


def test_encoder_count_generation_rebases_without_synthetic_distance_and_replays(tmp_path):
    config = _config()
    control = NativeControlComposition(OfflineMotorSink(), config)
    sink = CaptureSink('counter-recovery', configuration={'production_control': config})
    last_x = None
    try:
        for tick in range(150):
            inputs = _inputs(config, tick)
            health = inputs.raw_devices.device_health
            samples = inputs.raw_devices.samples
            if 2 <= tick < 145:
                health = tuple(DeviceHealth(h.device_id, DeviceHealthState.DEGRADED, 'ENCODER_RESTARTING')
                    if h.device_id == 'WHEEL_ENCODERS' else h for h in health)
                samples = tuple(s for s in samples if s.device_id != 'WHEEL_ENCODERS')
            else:
                updated = []
                for sample in samples:
                    if sample.device_id == 'WHEEL_ENCODERS':
                        # A nonzero old cumulative count followed by a new owner's
                        # zero is a baseline change, never backwards displacement.
                        values = tuple(DataField(f.key, 2. if tick < 2 else 0.)
                            if f.key in {'raw_left_distance_m', 'raw_right_distance_m'} else f
                            for f in sample.values)
                        sample = replace(sample, values=values + (DataField('counter_generation', 0 if tick < 2 else 1),))
                    updated.append(sample)
                samples = tuple(updated)
            inputs = replace(inputs, raw_devices=replace(inputs.raw_devices, samples=samples, device_health=health))
            closed = control.close_inputs(inputs)
            result = control.run_tick(closed)
            assert result.trace.fault_layer is None
            assert result.final_actuation.safety_decision is not SafetyDecision.FAULT
            estimate = _layers(result)['L3']
            if last_x is not None:
                assert abs(estimate.local_pose.x_m-last_x) < .001
            last_x = estimate.local_pose.x_m
            if 2 <= tick < 145:
                assert result.final_actuation.safety_decision is SafetyDecision.STOP
            sink.write(ExecutionRecord(closed, result))
        assert control._estimator.checkpoint().local_filter.encoder_anchor.counter_generation == 1
    finally:
        control.close()
    capture = sink.finalize('PASS', tmp_path/'counter-recovery.json')
    replay = replay_capture(capture, project_root=ROOT)
    assert replay['status'] == 'MATCH', replay['diagnostics']


def test_resident_keeps_active_mission_during_sensor_recovery():
    from v3.adapters.live_encoder import NativeEncoderSource
    from v3.adapters.live_imu import NativeImuSource
    from v3.adapters.live_lidar import NativeLidarSource
    from v3.composition.resident_live_control import ResidentLiveControlComposition, ResidentLiveControlConfig
    from v3.contracts import CommandMode, LifecycleState, TickContext
    from v3.contracts.navigation import Nav2NavigationInput
    config = _config()
    class Reader:
        def read(self, context):
            batch = _inputs(config, context.tick_id).raw_devices
            if context.tick_id in (4, 5):
                return replace(batch, device_health=tuple(
                    DeviceHealth(h.device_id, DeviceHealthState.DEGRADED, 'IMU_RESTARTING')
                    if h.device_id == 'BNO055_IMU' else h for h in batch.device_health),
                    samples=tuple(s for s in batch.samples if s.device_id != 'BNO055_IMU'))
            return batch
    class Gateway:
        def snapshot(self, context):
            command = _inputs(config, context.tick_id).command
            return replace(command, mode=CommandMode.STOP, goal=()) if context.tick_id < 2 else command
    class Navigation:
        request, guidance = None, None
        def submit(self, request):
            self.request = request
            if request is not None:
                self.guidance = _guidance(request, produced_ns=request.context.monotonic_ns)
        def poll(self, context): return Nav2NavigationInput(context, self.guidance)
        def close(self): pass
    # Reader injection is the production composition seam; sources are never read.
    encoder = NativeEncoderSource.__new__(NativeEncoderSource)
    heading = NativeImuSource.__new__(NativeImuSource)
    lidar = NativeLidarSource.__new__(NativeLidarSource)
    writer = OfflineMotorSink()
    resident = ResidentLiveControlComposition(encoder, heading, lidar, Gateway(), writer,
        ResidentLiveControlConfig(config, required_lidar_preflight_revisions=1),
        device_reader=Reader(), navigation_backend=Navigation())
    mission = None
    for tick in range(9):
        result = resident.tick(TickContext(tick, 1_000_000_000 + tick*20_000_000))
        assert result.final_actuation.safety_decision is not SafetyDecision.FAULT
        if tick >= 2:
            assert resident.lifecycle is LifecycleState.ACTIVE
            current = _layers(result)['L5'].mission_id
            mission = current if mission is None else mission
            assert current == mission
        if tick in (4, 5):
            assert result.final_actuation.safety_decision is SafetyDecision.STOP
            assert not result.final_actuation.enabled
    assert len(writer.writes) == 9


def test_stable_imu_operation_rearms_budget_without_reusing_source_sequence(tmp_path, monkeypatch):
    monkeypatch.setattr(imu, '_MAX_RESTARTS', 1)
    monkeypatch.setattr(imu, '_RECOVERY_STABLE_NS', 100_000_000)
    device = imu.ProcessBno055Device(NativeBno055DeviceConfig(),
        open_bus=partial(_open_bus, str(tmp_path/'opened')),
        process_config=replace(resolved_config().edges.imu_process, stop_timeout_s=.1))
    try:
        first_pid = device._process.pid
        _wait(lambda: device._process.pid != first_pid and not device._recovering)
        _wait(lambda: device._attempts == 0)
        previous = device.read_sample(force=True)
        pid = device._process.pid
        device._process.kill()
        def fresh_after_crash():
            try:
                sample = device.read_sample(force=True)
            except SensorUnavailable:
                return None
            return sample if device._process.pid != pid else None
        current = _wait(fresh_after_crash)
        assert device._restarts == 2 and not device._terminal_error
        assert current['sequence'] > previous['sequence']
        assert current['timestamp'] > previous['timestamp']
    finally:
        device.close()
