from __future__ import annotations

import base64
import json
from pathlib import Path
import sqlite3

from tools.mcap_evidence.compiler import compile_evidence
from tools.mcap_evidence.query import query
from tools.mcap_evidence.verify import verify
from v3.mcap_writer import StdlibMcapWriter


def _capture(path: Path, rows):
    with path.open('wb') as stream:
        writer = StdlibMcapWriter(stream, chunk_target_bytes=4096)
        writer.start()
        channels = {topic: writer.register_channel(topic) for topic in dict(rows)}
        for i, (topic, raw) in enumerate(rows):
            writer.add_message(channels[topic], log_time_ns=10_000 + i, data=raw, sequence=i)
        writer.finish()
    return path


def test_bulk_array_sparse_index_keeps_lidar_lossless(tmp_path):
    points = [[i * 0.25, i * 0.01, i % 255] for i in range(1024)]
    raw_lidar = json.dumps({'revision': 12, 'points': points, 'timestamp': 32}, separators=(',', ':')).encode()
    source = _capture(tmp_path / 'input.mcap', [('/r2b4/raw_lidar', raw_lidar)])
    result = compile_evidence(source, tmp_path / 'bundle', workers=1, shard_bytes=4096)
    root = Path(result['output'])

    assert verify(root, source=source)['status'] == 'PASS'
    db = sqlite3.connect(root / 'index.sqlite')
    try:
        paths = [row[0] for row in db.execute('SELECT path FROM fields ORDER BY path')]
        bulk = list(db.execute('SELECT path,length FROM bulk_arrays ORDER BY path'))
    finally:
        db.close()

    assert '/points' not in paths
    assert not any(path.startswith('/points/') for path in paths)
    assert bulk == [('/points', 1024)]
    assert set(paths) == {'/revision', '/timestamp'}

    # Exact source bytes remain available after the MCAP is removed.
    source.unlink()
    row = list(query(root, topic='/r2b4/raw_lidar'))[0]
    assert base64.b64decode(row['payload_base64']) == raw_lidar
    assert row['payload']['points'] == points

    # Sparse candidates preserve legacy field query semantics exactly.
    assert len(list(query(root, field='/points/999/2'))) == 1
    assert len(list(query(root, field='/points/*'))) == 1
    assert len(list(query(root, field='/points/9999/2'))) == 0

    index_meta = json.loads((root / 'index.json').read_text())
    assert index_meta['field_index_policy']['mode'] == 'sparse_bulk_arrays_v1'
    field_meta = json.loads((root / 'field_index.json').read_text())
    assert field_meta['bulk_arrays'] == [
        {'path': '/points', 'messages': 1, 'min_length': 1024, 'max_length': 1024}
    ]
