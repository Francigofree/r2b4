from __future__ import annotations

import base64
import json
from pathlib import Path
import struct

import pytest

from tools.mcap_evidence.compiler import compile_evidence
from tools.mcap_evidence.query import query
from tools.mcap_evidence.verify import verify
from v3.mcap_reader import McapReader, McapReadError
from v3.mcap_writer import StdlibMcapWriter

pytestmark = [pytest.mark.evidence, pytest.mark.replay]


def capture(path, rows=None, *, chunk_bytes=200):
    if rows is None:
        rows = [('/r2b4/tick', json.dumps({'tick_id': i, 'expected': {'layers': {
            'L11': {'output': i / 10, 'new_field': [None, {}, []]}}}}).encode()) for i in range(5)]
    with path.open('wb') as stream:
        writer = StdlibMcapWriter(stream, chunk_target_bytes=chunk_bytes)
        writer.start()
        channels = {topic: writer.register_channel(topic) for topic in dict(rows)}
        writer.add_metadata('test', {'purpose': 'offline evidence contract'})
        for i, (topic, raw) in enumerate(rows):
            writer.add_message(channels[topic], log_time_ns=2**63 + i, data=raw, sequence=i)
        writer.finish()
    return path


def records(raw):
    pos = 8
    while pos < len(raw) - 8:
        opcode, length = struct.unpack_from('<BQ', raw, pos)
        yield pos, opcode, length
        pos += 9 + length


def test_process_determinism_full_lidar_and_indexed_query(tmp_path):
    points = [[i, i * .01, i % 255] for i in range(1024)]
    rows = [('/r2b4/tick', b'{"tick_id":1505,"expected":{"layers":{"L11":{"new_field":2}}},"inputs":{"raw_devices":{"samples":[{"kind":"lidar_health","revision":99}]}}}'),
            ('/r2b4/raw_lidar', json.dumps({'points': points, 'revision': 12, 'timestamp': 32}).encode()),
            ('/unexpected/a.b', b'{"x/y":{"~key":[null,{},[]]},"tick_id":0}'),
            ('/binary', b'\xff\x00'), ('/broken', b'{not-json'),
            ('/duplicate', b'{"a":1,"a":2}')]
    source = capture(tmp_path / 'input.mcap', rows)
    one = compile_evidence(source, tmp_path / 'one', workers=1, shard_bytes=300)
    many = compile_evidence(source, tmp_path / 'many', workers=2, shard_bytes=300)
    assert one['compiler_status'] == many['compiler_status'] == 'COMPLETE'
    assert verify(one['output'], source=source)['messages'] == len(rows)
    assert verify(many['output'], source=source)['messages'] == len(rows)
    artifacts = lambda output: {
        p.relative_to(output).as_posix() for p in Path(output).rglob('*') if p.is_file()
    }
    assert artifacts(one['output']) == artifacts(many['output'])
    for p in Path(one['output']).rglob('*'):
        # Measurements/provenance vary with worker count; exported evidence must not.
        if p.is_file() and p.name not in {'manifest.json', 'compiler_performance.json'}:
            assert p.read_bytes() == (Path(many['output']) / p.relative_to(one['output'])).read_bytes()
    source.unlink()  # Query must remain portable without MCAP access.
    result = list(query(one['output'], tick_id=1505, field='expected.layers.L11.*', layer='L11'))
    assert len(result) == 1
    assert len(list(query(one['output'], sensor='lidar_health'))) == 1
    assert base64.b64decode(result[0]['payload_base64']) == rows[0][1]
    lidar = list(query(one['output'], topic='/r2b4/raw_lidar'))[0]
    assert lidar['payload']['points'] == points
    normalized = next((Path(one['output']) / 'normalized/lidar').glob('*.ndjson'))
    assert json.loads(normalized.read_bytes())['payload']['points'] == points
    assert len(list(query(one['output'], field='/x~1y/~0key/*'))) == 1
    assert len(list(query(one['output'], start_ns=2**63, end_ns=2**63 + 1))) == 2
    assert verify(one['output'])['quarantined'] == 3


def test_salvage_crc_and_truncated_chunk_preserves_every_complete_message(tmp_path):
    source = capture(tmp_path / 'input.mcap', chunk_bytes=100000)
    original = source.read_bytes()
    chunk, _, length = next(r for r in records(original) if r[1] == 6)
    raw = bytearray(original)
    raw[chunk + 9 + 24] ^= 1  # CRC field, not message bytes.
    source.write_bytes(raw)
    crc = compile_evidence(source, tmp_path / 'crc', workers=1)
    assert crc['compiler_status'] == 'PARTIAL'
    assert crc['messages'] == 5
    assert verify(crc['output'], source=source)['status'] == 'PASS'
    # Current writer: 40-byte uncompressed chunk prefix, then nested messages.
    nested = chunk + 9 + 40
    first_length = struct.unpack_from('<Q', original, nested + 1)[0]
    source.write_bytes(original[:nested + 9 + first_length + 15])
    partial = compile_evidence(source, tmp_path / 'partial', workers=1)
    assert partial['compiler_status'] == 'PARTIAL'
    assert partial['source_integrity'] == 'TRUNCATED'
    assert partial['messages'] == 1
    assert verify(partial['output'], source=source)['status'] == 'PASS'
    with pytest.raises(McapReadError):
        McapReader(source)
    coverage = json.loads((Path(partial['output']) / 'coverage.json').read_bytes())
    assert sum(coverage['byte_ranges'].values()) == source.stat().st_size
    assert coverage['byte_ranges']['unrecoverable'] > 0


def test_missing_magic_footer_unknown_records_and_malformed_nested_record(tmp_path):
    source = capture(tmp_path / 'input.mcap')
    raw = source.read_bytes()
    footer = next(offset for offset, op, _ in records(raw) if op == 2)
    source.write_bytes(raw[8:footer])
    result = compile_evidence(source, tmp_path / 'missing', workers=1)
    assert result['compiler_status'] == 'PARTIAL' and result['messages'] == 5
    verify(result['output'], source=source)
    first_offset, _, first_length = next(records(raw))
    source.write_bytes(raw[first_offset + 9 + first_length:])
    no_header = compile_evidence(source, tmp_path / 'no-header', workers=1)
    assert no_header['messages'] == 5 and no_header['compiler_status'] == 'PARTIAL'
    verify(no_header['output'], source=source)
    # Malformed message body has a proven next boundary; the following valid message survives.
    header = b'\x01' + struct.pack('<QII', 8, 0, 0)
    malformed = b'\x05' + struct.pack('<Q', 2) + b'ab'
    unknown = b'\x80' + struct.pack('<Q', 3) + b'xyz'
    source.write_bytes(raw[:8] + header + malformed + unknown + raw[8:])
    result = compile_evidence(source, tmp_path / 'malformed', workers=1)
    assert result['compiler_status'] == 'PARTIAL' and result['messages'] == 5
    verify(result['output'], source=source)
    rows = [json.loads(line) for line in (Path(result['output']) / 'source/records.ndjson').read_text().splitlines()]
    assert any(r.get('content_base64') == base64.b64encode(b'xyz').decode() for r in rows)


def test_growing_snapshot_is_reported_without_reading_appended_bytes(tmp_path, monkeypatch):
    from tools.mcap_evidence import ingest
    source = capture(tmp_path / 'input.mcap')
    before = source.stat().st_size
    original = ingest.process_unit
    def grow(args):
        with source.open('ab') as stream:
            stream.write(b'later bytes')
        return original(args)
    monkeypatch.setattr(ingest, 'process_unit', grow)
    result = compile_evidence(source, tmp_path / 'bundle', workers=1)
    integrity = json.loads((Path(result['output']) / 'integrity.json').read_bytes())
    assert integrity['source_changed_during_compile'] is True
    assert integrity['source_snapshot_size'] == before
    assert integrity['source_size_after_compile'] > before
    assert result['compiler_status'] == 'COMPLETE'
    verify(result['output'], source=source)


def test_atomic_publication_interruption_and_legacy_protection(tmp_path, monkeypatch):
    from tools.mcap_evidence import compiler
    source = capture(tmp_path / 'input.mcap')
    legacy = tmp_path / 'legacy.evidence'
    legacy.mkdir()
    (legacy / 'manifest.json').write_text('{"schema":"R2B4_TEST_HUB_V2"}')
    with pytest.raises(ValueError, match='protected'):
        compile_evidence(source, legacy, overwrite=True)
    assert (legacy / 'manifest.json').read_text() == '{"schema":"R2B4_TEST_HUB_V2"}'
    output = tmp_path / 'new.evidence'
    compile_evidence(source, output, workers=1)
    original = (output / 'manifest.json').read_bytes()
    with pytest.raises(FileExistsError):
        compile_evidence(source, output)
    real_verify = compiler.verify_artifacts
    def fail(_root, _manifest):
        raise KeyboardInterrupt
    monkeypatch.setattr(compiler, 'verify_artifacts', fail)
    with pytest.raises(KeyboardInterrupt):
        compile_evidence(source, output, workers=1, overwrite=True)
    assert (output / 'manifest.json').read_bytes() == original
    assert not list(tmp_path.glob('*.tmp-*'))
    with pytest.raises(KeyboardInterrupt):
        compile_evidence(source, tmp_path / 'interrupted', workers=1)
    assert not (tmp_path / 'interrupted').exists()
    monkeypatch.setattr(compiler, 'verify_artifacts', real_verify)
    assert compile_evidence(source, output, workers=1, overwrite=True)['compiler_status'] == 'COMPLETE'
    # Hash verification detects output mutation even if queries still parse it.
    with (output / 'source/records.ndjson').open('ab') as stream:
        stream.write(b'{}\n')
    with pytest.raises(ValueError, match='hash mismatch'):
        verify(output)
    assert (legacy / 'manifest.json').is_file()
    # Regenerating a file hash cannot hide an unindexed exported message.
    from tools.mcap_evidence.export import sha256
    shard = next((output / 'source/messages').glob('*.ndjson'))
    with shard.open('ab') as stream:
        stream.write(shard.read_bytes().splitlines(keepends=True)[0])
    manifest = json.loads((output / 'manifest.json').read_bytes())
    for path in (shard, output / 'source/records.ndjson'):
        manifest['files'][str(path.relative_to(output))] = {'size': path.stat().st_size, 'sha256': sha256(path)}
    # Reseal provenance/accounting too, so the failure reaches the payload
    # census instead of stopping at the intentionally modified manifest hash.
    compiler._seal(output, manifest, json.loads((output / 'compiler_performance.json').read_bytes()))
    with pytest.raises(ValueError, match='unindexed'):
        verify(output)


def test_capture_process_shutdown_has_no_evidence_dependency(tmp_path):
    from v3.interface_cli import _execute, _parser
    from v3.interface_adapters import build_adapters
    from v3.launcher_cli import command_catalog
    class Interface:
        def execute(self, action):
            assert action == 'operator.runtime.stop'
            return {'state': 'STOPPED'}
    result = _execute(Interface(), _parser().parse_args(['shutdown']),
                      capture_mode='alap', capture_hz=5, no_trigger=False)
    assert result == {'state': 'STOPPED'}
    assert not any('testhub' in adapter.capability_names for adapter in build_adapters(object(), tmp_path))
    assert not any(c['name'] == 'testhub' for c in command_catalog()['robot'])
    root = Path(__file__).resolve().parents[2]
    assert not list((root / 'v3').glob('test_hub*.py'))
    for name in ('v3/process_sidecars.py', 'v3_process_runtime.py', 'v3/operator_controller.py', 'v3/replay.py'):
        text = (root / name).read_text()
        assert 'test_hub' not in text and 'mcap_evidence' not in text

    # Exercise both real capture finalizers without hardware or a mock worker.
    from types import SimpleNamespace
    from rig import resolved_config
    from v3.composition.full_fake import OfflineMotorSink
    from v3.composition.native_control import NativeControlComposition
    from v3.contracts import CommandMode, CommandRequest, LifecycleState, RawDeviceBatch, TickContext
    from v3.engine import TickInputs
    from v3.execution import ExecutionRecord
    from v3.mcap_capture import McapCaptureConfig
    from v3.process_sidecars import ProcessMcapCaptureSession
    from v3_process_runtime import McapCaptureSession
    from v3.replay import inspect_capture

    composition = NativeControlComposition(OfflineMotorSink(), resolved_config().runtime.composition.live_control.control)
    context = TickContext(0, 1_000_000_000)
    inputs = composition.close_inputs(TickInputs(
        context, RawDeviceBatch(context, (), ()),
        CommandRequest(context, 'offline-stop', CommandMode.STOP, (), 0), LifecycleState.IDLE))
    result = composition.run_tick(inputs)
    record = ExecutionRecord(inputs, result)
    config = McapCaptureConfig(mode='append_only', tick_sample_hz=10)
    ticks = []
    for cls, name in ((McapCaptureSession, 'direct'), (ProcessMcapCaptureSession, 'process')):
        kwargs = {'project_root': tmp_path} if name == 'process' else {}
        session = cls(name, tmp_path / (name + '.mcap'), configuration={}, config=config, **kwargs)
        session.start()
        try:
            session.observe(record)
        finally:
            path = session.finalize(SimpleNamespace(status=0))
        assert path is not None and not session.failed
        assert inspect_capture(path)['status'] == 'PASS'
        ticks.append([payload for _, payload in McapReader(path).iter_json_messages(topics=['/r2b4/tick'])])
        assert not path.with_suffix('.evidence').exists()
    assert len(ticks[0]) == 1 and ticks[0] == ticks[1]


def test_compressed_chunk_lineage_and_decoder_failure(tmp_path):
    import importlib.util
    # Build a legal summary-free MCAP without weakening the production reader.
    from tools.mcap_evidence.records import MAGIC
    def rec(op, body):
        return struct.pack('<BQ', op, len(body)) + body
    def string(value):
        data = value.encode()
        return struct.pack('<I', len(data)) + data
    header = rec(1, string('test') + string('test'))
    channel = rec(4, struct.pack('<HH', 1, 0) + string('/topic') + string('json') + struct.pack('<I', 0))
    nested = rec(5, struct.pack('<HIQQ', 1, 7, 10, 11) + b'{"tick_id":7}')
    def write_chunk(compression, data):
        import zlib
        body = struct.pack('<QQQI', 10, 10, len(nested), zlib.crc32(nested))
        body += string(compression) + struct.pack('<Q', len(data)) + data
        source = tmp_path / (compression + '.mcap')
        source.write_bytes(MAGIC + header + channel + rec(6, body)
                          + rec(15, struct.pack('<I', 0)) + rec(2, struct.pack('<QQI', 0, 0, 0)) + MAGIC)
        return source
    with pytest.raises(RuntimeError, match='unsupported'):
        compile_evidence(write_chunk('unknown', nested), tmp_path / 'unsupported', workers=1)
    assert not (tmp_path / 'unsupported').exists()
    if importlib.util.find_spec('lz4') is not None:
        import lz4.frame
        source = write_chunk('lz4', lz4.frame.compress(nested))
        result = compile_evidence(source, tmp_path / 'compressed', workers=1)
        assert result['compiler_status'] == 'COMPLETE'
        verify(result['output'], source=source)
        row = list(query(result['output'], tick_id=7))[0]
        assert row['source_offset'] == row['chunk_offset']
        assert row['nested_offset'] == 0
        assert row['payload'] == {'tick_id': 7}

        nested = rec(5, b'ab') + nested
        source = write_chunk('lz4', lz4.frame.compress(nested))
        partial = compile_evidence(source, tmp_path / 'compressed-partial', workers=1)
        assert partial['compiler_status'] == 'PARTIAL' and partial['messages'] == 1
        verify(partial['output'], source=source)
        ranges = (Path(partial['output']) / 'source/corrupt_ranges.ndjson').read_text()
        assert 'nested_start' in ranges
