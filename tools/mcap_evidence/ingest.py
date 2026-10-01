"""Snapshot scan and independent file-to-file worker units; no payload IPC."""
import base64
import json
import time
from pathlib import Path

from .export import Shards
from .index import add_message, connect
from .normalize import topic_name, views
from .salvage import Salvage
from .performance import worker_snapshot


def scan(source: Path, output: Path):
    return Salvage(source, output).scan()


def decode(raw: bytes):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError('duplicate JSON object key')
            result[key] = value
        return result

    def constant(value):
        raise ValueError('non-JSON numeric constant: ' + value)

    try:
        text = raw.decode('utf-8')
    except UnicodeDecodeError:
        return 'INVALID_UTF8', None
    try:
        payload = json.loads(text, object_pairs_hook=pairs, parse_constant=constant)
        # Reject non-finite values produced by numeric overflow as well.
        json.dumps(payload, allow_nan=False)
        return 'JSON', payload
    except (ValueError, RecursionError):
        return 'INVALID_JSON', None


def process_unit(args):
    cpu_started = time.process_time()
    durations = {name: 0.0 for name in (
        "decode_duration", "source_export_duration", "schema_coverage_duration",
        "normalized_export_duration", "raw_lidar_export_duration")}
    unit, output, shard_bytes = args
    root = Path(output) / '.work' / f'worker-{unit.number:08d}'
    root.mkdir()
    shards = Shards(root, unit.number, shard_bytes)
    db = connect(root / 'index.sqlite', create=True)
    try:
        with Path(unit.path).open('rb') as stream:
            for line in stream:
                started = time.perf_counter()
                source = json.loads(line)
                raw = base64.b64decode(source['payload_base64'], validate=True)
                if source['message_encoding'] == 'json':
                    status, payload = decode(raw)
                else:
                    status, payload = 'BINARY', None
                source['decode_status'] = status
                if status == 'JSON':
                    source['payload'] = payload
                    name = 'source/messages/' + topic_name(source['topic'])
                else:
                    name = 'source/quarantined_messages'
                durations['decode_duration'] += time.perf_counter() - started
                started = time.perf_counter()
                location = shards.write(name, source)
                source_duration = time.perf_counter() - started
                durations['source_export_duration'] += source_duration
                started = time.perf_counter()
                add_message(db, source, location)
                durations["schema_coverage_duration"] += time.perf_counter() - started
                started = time.perf_counter()
                for name, pointer, payload in views(source):
                    row = {'message_id': source['message_id'], 'source_pointer': pointer,
                           'topic': source['topic'], 'log_time_ns': source['log_time_ns'],
                           'payload': payload}
                    location = shards.write('normalized/' + name, row)
                    db.execute('INSERT INTO views VALUES (?,?,?,?,?)', (source['message_id'], pointer, *location))
                normalized_duration = time.perf_counter() - started
                durations['normalized_export_duration'] += normalized_duration
                if source['topic'] == '/r2b4/raw_lidar':
                    durations['raw_lidar_export_duration'] += source_duration + normalized_duration
        db.commit()
    finally:
        db.close()
        shards.close()
    # Only paths and scalar measurements cross the process boundary.
    return {'root': str(root), **worker_snapshot(cpu_started, durations)}
