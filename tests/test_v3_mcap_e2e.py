"""Real typed production -> Hub -> MCAP -> canonical replay -> Test Hub.

Only the physical motor sink is an offline recorder. No stage of the capture,
reader, bridge, production composition, replay or Test Hub is substituted.
"""
from dataclasses import replace
from pathlib import Path
import json

import pytest

from v3.adapters.native_lidar_port import NativeRawLidarSnapshot
from v3.adapters.rplidar_c1 import RplidarPoint
from v3.composition.native_control import NativeControlComposition
from v3.execution import ExecutionRecord
from v3.mcap_capture import McapCaptureConfig, McapCaptureConsumer
from v3.mcap_reader import McapReader, TICK_TOPIC, EVENT_TOPIC
from v3.mcap_replay_bridge import McapReplayBridgeError, ReplayWindow, replay_mcap
from v3.observation import ObservationHub
from v3.test_hub_v2 import diagnose_run, verify_evidence, _build_diagnosis
from v3_validation_helpers import RecordingMotorSink, control_config, tick_inputs

ROOT = Path(__file__).resolve().parents[1]


def records(count=12):
    config = control_config()
    production = NativeControlComposition(RecordingMotorSink(), config)
    result = []
    for inputs in tick_inputs(count):
        output = production.run_tick(inputs)
        result.append(ExecutionRecord(inputs, output, production.tick_evidence,
                                      production.checkpoint() if inputs.context.tick_id % 4 == 0 else None))
    return config, result


def raw(revision, monotonic_ns, points=4):
    return NativeRawLidarSnapshot(
        raw_scan_id=revision, raw_scan_timestamp=monotonic_ns / 1e9,
        scan_start_monotonic_ns=monotonic_ns, scan_end_monotonic_ns=monotonic_ns,
        measurement_monotonic_ns=monotonic_ns, health='OK',
        raw_scan=tuple(RplidarPoint(float(i % 360), 1.0, 20) for i in range(points)), summary={},
    )


def capture(tmp_path, *, count=12, capacity=100, settings=None, trigger=None, configuration=None):
    config, values = records(count)
    hub = ObservationHub()
    sub = hub.subscribe_reliable('capture', capacity=capacity,
                                 topics=('v3.capture_record', 'v3.raw_lidar'), required=True)
    consumer = McapCaptureConsumer('e2e', tmp_path / 'capture.mcap', subscription=sub,
                                  configuration={'resolved_control': config} if configuration is None else configuration,
                                  config=settings or McapCaptureConfig(mode='append_only'))
    if trigger is not None:
        consumer.trigger(*trigger)
    for record in values:
        tick = record.inputs.context.tick_id
        hub.publish(raw(tick + 1, record.inputs.context.monotonic_ns), topic='v3.raw_lidar')
        hub.publish(object(), topic='unrelated')
        hub.publish(record, topic='v3.capture_record')
    hub.close()
    result = consumer.finish()
    return result, sub


def test_real_end_to_end_and_authority_tamper(tmp_path):
    result, sub = capture(tmp_path)
    assert result.complete and result.status == 'PASS'
    assert sub.snapshot().consumed_count == sub.snapshot().accepted_count == 24
    assert sub.snapshot().queued == 0
    reader = McapReader(result.path)
    assert reader.inspect(verify_chunks=True).valid
    assert reader.capture_integrity()['captured_tick_count'] == 12
    replay = replay_mcap(result.path, project_root=ROOT)
    assert replay['status'] == 'MATCH'
    assert replay['mcap_bridge']['materialized_tick_count'] == 12
    assert replay['mcap_bridge']['preflight'] == 'PASS'
    evidence = diagnose_run(result.path, tmp_path / 'evidence', replay_mode='full', project_root=ROOT)
    assert evidence['status'] == 'PASS' and evidence['replay_status'] == 'MATCH'
    assert verify_evidence(evidence['evidence_index'])['status'] == 'PASS'
    with result.path.open('r+b') as stream:
        stream.seek(30)
        value = stream.read(1)
        stream.seek(30)
        stream.write(bytes([value[0] ^ 1]))
    assert verify_evidence(evidence['evidence_index'])['status'] == 'FAIL'
    with pytest.raises(McapReplayBridgeError, match='CRC|preflight'):
        replay_mcap(result.path, project_root=ROOT)


def test_reliable_overflow_must_fail_before_replay(tmp_path):
    result, sub = capture(tmp_path, capacity=4)
    assert sub.snapshot().lost_count == 20
    assert not result.complete and result.status == 'FAIL'
    assert McapReader(result.path).inspect(verify_chunks=True).valid
    with pytest.raises(McapReplayBridgeError, match='CAPTURE_INCOMPLETE'):
        replay_mcap(result.path, project_root=ROOT)
    evidence = diagnose_run(result.path, tmp_path / 'evidence', replay_mode='full', project_root=ROOT)
    assert evidence['status'] == 'FAIL'
    assert evidence['replay_status'] == 'ERROR'
    assert evidence['root_cause']['kind'] == 'CAPTURE_INCOMPLETE'


def test_checkpoint_short_replay_matches(tmp_path):
    result, _ = capture(tmp_path)
    replay = replay_mcap(result.path, project_root=ROOT,
                         window=ReplayWindow(requested_start_tick_id=9, requested_end_tick_id=10))
    assert replay['status'] == 'MATCH'
    assert replay['mcap_bridge']['checkpoint_used'] is True
    assert replay['mcap_bridge']['materialized_tick_count'] == 2
    with pytest.raises(McapReplayBridgeError, match='budget'):
        replay_mcap(result.path, window=ReplayWindow(max_materialized_ticks=1), project_root=ROOT)


@pytest.mark.parametrize('timestamp', [None, 0, 1_000_000_000])
def test_manual_trigger_before_first_tick_is_deferred(tmp_path, timestamp):
    result, _ = capture(tmp_path, settings=McapCaptureConfig(pre_event_ns=0, post_event_ns=0),
                        trigger=('EARLY', timestamp))
    assert result is not None
    assert McapReader(result.path).inspect(verify_chunks=True).valid
    assert result.complete


def test_requested_replay_error_or_missing_cannot_pass_and_incomplete_is_not_divergence():
    inspected = {'structure': {'valid': True}, 'final_event': {'integrity': {'complete': True}}}
    assert _build_diagnosis(inspected, {}, None, 'error')['status'] == 'FAIL'
    assert _build_diagnosis(inspected, {}, None, None)['status'] == 'FAIL'
    assert _build_diagnosis(inspected, {}, None, None, replay_requested=False)['status'] == 'PASS'
    diagnosis = _build_diagnosis(inspected, {}, {'status': 'MISMATCH', 'first_divergence':
                                {'reason': 'CAPTURE_INCOMPLETE'}}, None)
    assert diagnosis['root_cause']['kind'] == 'CAPTURE_INCOMPLETE'


def test_public_entrypoints_keep_mcap_authority_and_replay_result_checksum(tmp_path):
    from v3.replay import replay_capture, write_replay_result, verify_replay_result
    from v3.test_hub import validate_run, verify_evidence as verify_compat
    result, _ = capture(tmp_path)
    replay = replay_capture(result.path, project_root=ROOT)
    assert replay['capture']['path'] == str(result.path)
    assert replay['capture']['sha256'] == McapReader(result.path).sha256()
    output = write_replay_result(replay, tmp_path / 'replay.json')
    assert verify_replay_result(output)['status'] == 'PASS'
    evidence = validate_run(result.path, tmp_path / 'compat', project_root=ROOT)
    assert evidence['status'] == 'PASS'
    assert verify_compat(evidence['evidence_index'])['status'] == 'PASS'


def test_raw_lidar_over_limit_is_explicit_incomplete(tmp_path):
    config, values = records(2)
    hub = ObservationHub()
    sub = hub.subscribe_reliable('capture', capacity=10, required=True)
    consumer = McapCaptureConsumer('truncated', tmp_path / 'truncated.mcap', subscription=sub,
                                  configuration={'resolved_control': config}, config=McapCaptureConfig(mode='append_only'))
    for i, record in enumerate(values):
        hub.publish(raw(i + 1, record.inputs.context.monotonic_ns, 4097), topic='v3.raw_lidar')
        hub.publish(record, topic='v3.capture_record')
    hub.close()
    result = consumer.finish()
    assert result.status == 'FAIL' and not result.complete
    final = McapReader(result.path).last_json(EVENT_TOPIC)[1]
    assert final['integrity']['raw_lidar_truncated_count'] == 2


def test_session_bound_fails_explicitly_but_drains_mailbox(tmp_path):
    result, sub = capture(tmp_path, settings=McapCaptureConfig(mode='append_only', max_session_records=4))
    assert result.status == 'FAIL'
    assert sub.snapshot().queued == 0
    assert sub.snapshot().consumed_count == 24
    final = McapReader(result.path).last_json(EVENT_TOPIC)[1]
    assert 'SESSION_RECORD_LIMIT' in final['integrity']['integrity_reasons']


def test_real_replay_divergence_is_a_software_failure(tmp_path):
    config, values = records(2)
    hub = ObservationHub()
    sub = hub.subscribe_reliable('capture', capacity=10, required=True)
    consumer = McapCaptureConsumer('divergence', tmp_path / 'divergence.mcap', subscription=sub,
                                  configuration={'resolved_control': config}, config=McapCaptureConfig(mode='append_only'))
    for i, record in enumerate(values):
        if i == 1:
            layers = tuple(replace(layer, output=replace(layer.output, x_m=layer.output.x_m + 1))
                           if layer.layer == 'L3' else layer for layer in record.result.trace.layers)
            record = replace(record, result=replace(record.result, trace=replace(record.result.trace, layers=layers)))
        hub.publish(raw(i + 1, record.inputs.context.monotonic_ns), topic='v3.raw_lidar')
        hub.publish(record, topic='v3.capture_record')
    hub.close()
    result = consumer.finish()
    assert result.complete
    evidence = diagnose_run(result.path, tmp_path / 'divergence.evidence', replay_mode='full', project_root=ROOT)
    assert evidence['status'] == 'FAIL' and evidence['replay_status'] == 'MISMATCH'
    assert evidence['root_cause']['kind'] == 'SOFTWARE_REPLAY_DIVERGENCE'
    assert evidence['root_cause']['tick_id'] == 1
    assert evidence['root_cause']['layer'] == 'L3'


def test_single_data_mailbox_and_encoding_runs_only_on_consumer_thread(tmp_path, monkeypatch):
    import threading
    import v3.mcap_capture as module
    original = module.encode_capture_record
    threads = []
    def checked(record):
        threads.append(threading.current_thread().name)
        return original(record)
    monkeypatch.setattr(module, 'encode_capture_record', checked)
    result, _ = capture(tmp_path)
    assert result.complete and len(threads) == 12
    assert set(threads) == {'v3-mcap-capture-consumer'}
    assert '_queue' not in McapCaptureConsumer.__slots__


def test_actual_requested_replay_error_cannot_pass(tmp_path):
    result, _ = capture(tmp_path, configuration={})
    assert result.complete
    evidence = diagnose_run(result.path, tmp_path / 'error.evidence', replay_mode='full', project_root=ROOT)
    assert evidence['status'] == 'FAIL' and evidence['replay_status'] == 'ERROR'
    diagnosis = json.loads((tmp_path / 'error.evidence' / 'diagnosis.json').read_text())
    assert 'configuration' in diagnosis['replay_error']


def test_existing_capture_is_never_overwritten(tmp_path):
    path = tmp_path / 'capture.mcap'
    path.write_bytes(b'previous local evidence')
    with pytest.raises(RuntimeError, match='partial file kept'):
        capture(tmp_path)
    assert path.read_bytes() == b'previous local evidence'
    assert list(tmp_path.glob('.*.partial'))


def test_ram_budget_includes_retained_graphs_writer_copies_and_session_indexes():
    from v3.mcap_capture import conservative_ram_budget
    config = McapCaptureConfig()
    budget = conservative_ram_budget(config, mailbox_capacity=256,
                                     max_retained_observation_bytes=1024 * 1024,
                                     max_encoding_working_bytes=32 * 1024 * 1024)
    assert budget['mailbox_and_active_payloads'] > 256 * 1024 * 1024
    assert budget['writer_chunk_copies_and_message_indexes'] > config.chunk_target_bytes
    assert budget['session_indexes_and_diagnostics'] > 0
    assert budget['total_bytes'] > config.max_byte_capacity + 256 * 1024 * 1024


def test_complete_production_fault_is_fault_capture_and_match_evidence(tmp_path):
    from v3.contracts import LifecycleState, TickContext
    from v3.execution import EdgeFaultRecord
    config = control_config()
    production = NativeControlComposition(RecordingMotorSink(), config)
    context = TickContext(0, 1_000_000_000)
    result = production.run_fault_tick(context, LifecycleState.FAULT, 'EDGE_FAILED', 'L0')
    record = EdgeFaultRecord(context, LifecycleState.FAULT, 'EDGE_FAILED', 'L0', (), result)
    hub = ObservationHub()
    sub = hub.subscribe_reliable('capture', capacity=2, required=True)
    consumer = McapCaptureConsumer('fault', tmp_path / 'fault.mcap', subscription=sub,
                                  configuration={'resolved_control': config}, config=McapCaptureConfig(mode='append_only'))
    hub.publish(record, topic='v3.capture_record')
    hub.close()
    captured = consumer.finish()
    assert captured.complete and captured.status == 'FAULT'
    evidence = diagnose_run(captured.path, tmp_path / 'fault.evidence', replay_mode='full', project_root=ROOT)
    assert evidence['status'] == 'PASS' and evidence['replay_status'] == 'MATCH'
    assert evidence['capture_status'] == 'FAULT'
