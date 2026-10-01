"""Disk-backed message/field postings; no domain verdicts or field whitelist."""
import json
import sqlite3
from pathlib import Path

from .normalize import leaves, samples
from .schemas import INDEX
from .export import write_json

DDL = '''
CREATE TABLE messages (
 message_id TEXT PRIMARY KEY, topic TEXT, channel_id INTEGER, sequence INTEGER,
 source_offset INTEGER, log_time_ns TEXT, publish_time_ns TEXT, tick_id TEXT,
 file TEXT, byte_offset INTEGER, byte_length INTEGER, decode_status TEXT,
 payload_sha256 TEXT);
CREATE TABLE fields (path TEXT, message_id TEXT, PRIMARY KEY(path, message_id)) WITHOUT ROWID;
CREATE TABLE identifiers (kind TEXT, value TEXT, message_id TEXT,
 PRIMARY KEY(kind, value, message_id)) WITHOUT ROWID;
CREATE TABLE views (message_id TEXT, pointer TEXT, file TEXT, byte_offset INTEGER, byte_length INTEGER);
'''


def connect(path: Path, *, create=False):
    db = sqlite3.connect(path)
    if create:
        # These databases exist only inside the unpublished staging directory.
        # A crash discards the whole compile; rollback journals add no recovery
        # guarantee here and can multiply shard-merge I/O by the unit count.
        db.execute('PRAGMA journal_mode=OFF')
        db.execute('PRAGMA synchronous=OFF')
        db.execute('PRAGMA cache_size=-16384')
        db.executescript(DDL)
    return db


def read_only(root: Path):
    return sqlite3.connect((root / 'index.sqlite').resolve().as_uri() + '?mode=ro', uri=True)


def add_message(db, source, location):
    payload = source.get('payload')
    tick = payload.get('tick_id') if isinstance(payload, dict) else None
    db.execute('INSERT INTO messages VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?)', (
        source['message_id'], source['topic'], source['channel_id'], source['sequence'],
        source['source_offset'], f"{source['log_time_ns']:020d}", f"{source['publish_time_ns']:020d}",
        json.dumps(tick) if tick is not None else None, *location,
        source['decode_status'], source['payload_sha256']))
    if source['decode_status'] != 'JSON':
        return
    for _, sample in samples(payload):
        db.execute('INSERT OR IGNORE INTO identifiers VALUES (?,?,?)',
                   ('sensor', sample['kind'], source['message_id']))
    for path, value in leaves(payload):
        db.execute('INSERT INTO fields VALUES (?,?)', (path, source['message_id']))
        parts = path.split('/')
        kind = {'tick_id': 'tick', 'layer': 'layer', 'layer_id': 'layer',
                'sensor_kind': 'sensor', 'sensor_id': 'sensor'}.get(parts[-1])
        if kind and isinstance(value, (str, int)) and not isinstance(value, bool):
            db.execute('INSERT OR IGNORE INTO identifiers VALUES (?,?,?)', (kind, str(value), source['message_id']))
        for i, part in enumerate(parts[:-1]):
            if part in ('layers', 'sensors'):
                db.execute('INSERT OR IGNORE INTO identifiers VALUES (?,?,?)', (
                    'layer' if part == 'layers' else 'sensor', parts[i + 1], source['message_id']))


def merge(root: Path, worker_roots):
    db = connect(root / 'index.sqlite', create=True)
    try:
        for worker in worker_roots:
            db.execute('ATTACH DATABASE ? AS worker', (str(worker / 'index.sqlite'),))
            for table in ('messages', 'fields', 'identifiers', 'views'):
                db.execute(f'INSERT INTO {table} SELECT * FROM worker.{table}')
            db.commit()
            db.execute('DETACH DATABASE worker')
        db.executescript('''
        CREATE INDEX message_topic ON messages(topic, message_id);
        CREATE INDEX message_channel ON messages(channel_id, sequence);
        CREATE INDEX message_time ON messages(log_time_ns);
        CREATE INDEX message_offset ON messages(source_offset);
        CREATE INDEX field_message ON fields(message_id, path);
        CREATE INDEX view_message ON views(message_id);
        CREATE INDEX message_file ON messages(file, byte_offset);
        CREATE INDEX view_file ON views(file, byte_offset);
        ''')
        db.commit()
        topics = [dict(topic=t, messages=n) for t, n in db.execute(
            'SELECT topic,count(*) FROM messages GROUP BY topic ORDER BY topic')]
        count = db.execute('SELECT count(*) FROM messages').fetchone()[0]
        field_count = db.execute('SELECT count(*) FROM fields').fetchone()[0]
        write_json(root / 'index.json', {'schema': INDEX, 'database': 'index.sqlite',
                                       'messages': count, 'topics': topics,
                                       'time_encoding': 'zero-padded unsigned nanoseconds',
                                       'field_path_encoding': 'JSON pointer (RFC 6901)'})
        # Stream the census: path count may itself be very large.
        with (root / 'field_index.json').open('w') as stream:
            stream.write('{"schema":' + json.dumps(INDEX) + ',"paths":[')
            first = True
            for path, occurrences in db.execute('SELECT path,count(*) FROM fields GROUP BY path ORDER BY path'):
                if not first:
                    stream.write(',')
                stream.write(json.dumps({'path': path, 'messages': occurrences}, ensure_ascii=True))
                first = False
            stream.write(']}\n')
        exported_by_topic = {topic: {'messages': total, 'json_decoded': decoded}
                             for topic, total, decoded in db.execute(
                                 "SELECT topic,count(*),sum(decode_status='JSON') FROM messages GROUP BY topic")}
        return {'exported': count, 'field_occurrences': field_count,
                'exported_by_topic': exported_by_topic,
                'normalized_views': db.execute('SELECT count(*) FROM views').fetchone()[0],
                'json_decoded': db.execute("SELECT count(*) FROM messages WHERE decode_status='JSON'").fetchone()[0]}
    finally:
        db.close()


def topic_coverage(captured, exported):
    """Counts refer to recoverable snapshot messages, including quarantine bytes."""
    total = exported.get('messages', 0)
    decoded = exported.get('json_decoded', 0)
    return {
        'captured_messages': captured, 'fully_exported_messages': total,
        'json_decoded_messages': decoded, 'quarantined_messages': total - decoded,
        'normalized_messages': decoded, 'summary_exported_messages': 0,
        'payloads_omitted': captured - total,
        'exact_data_available_in_mcap': True,
        'all_captured_messages_fully_exported': captured == total,
    }
