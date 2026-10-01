"""Verify file hashes, source accounting, field census and projection lineage."""
import base64
from collections import Counter
import hashlib
import json
from pathlib import Path

from .export import encoded, manifest_digest, sha256
from .index import read_only, topic_coverage
from .ingest import decode
from .normalize import field_index_entries, leaves, views as expected_views
from .query import bundle_file, read_row
from .schemas import MANIFEST, COVERAGE


def _root(bundle):
    path = Path(bundle).resolve()
    return path.parent if path.is_file() and path.name == 'manifest.json' else path


def verify_manifest(bundle):
    """Check the sealed manifest, binding, final inventory and profile accounting."""
    root = _root(bundle)
    raw = bundle_file(root, 'manifest.json').read_bytes()
    manifest = json.loads(raw)
    if manifest.get('schema') != MANIFEST:
        raise ValueError('not an MCAP Evidence Compiler bundle')
    if manifest.get('complete') is not True or manifest.get('compiler_status') not in ('COMPLETE', 'PARTIAL'):
        raise ValueError('bundle has no complete final manifest')
    if manifest.get('self_sha256') != manifest_digest(manifest):
        raise ValueError('manifest self digest mismatch')
    if 'manifest.json' in manifest['files']:
        raise ValueError('manifest is covered by self_sha256, not its own inventory')
    required = {'compiler_performance.json', 'integrity.json', 'coverage.json', 'index.json', 'index.sqlite', 'field_index.json'}
    if not required <= manifest['files'].keys():
        raise ValueError('required artifact missing from manifest')
    actual_files = {str(p.relative_to(root)) for p in root.rglob('*')
                    if p.is_file() and p != root / 'manifest.json'}
    if actual_files != set(manifest['files']):
        raise ValueError('bundle file inventory mismatch')
    for relative, expected in manifest['files'].items():
        path = bundle_file(root, relative)
        if path.stat().st_size != expected['size']:
            raise ValueError('evidence file hash mismatch (size): ' + relative)
    # These two small artifacts are finalized after payload verification.
    for relative in ('compiler_performance.json', 'integrity.json'):
        if sha256(root / relative) != manifest['files'][relative]['sha256']:
            raise ValueError('evidence file hash mismatch: ' + relative)
    integrity = json.loads((root / 'integrity.json').read_bytes())
    source = manifest['source']
    if (source.get('path') != integrity['source_path']
            or source.get('sha256') != integrity['source_snapshot_sha256']
            or source.get('snapshot_size') != integrity['source_snapshot_size']
            or source.get('hash_scope') != 'snapshot_prefix'
            or manifest['source_integrity'] != integrity['source_integrity']):
        raise ValueError('manifest authority MCAP binding mismatch')
    performance = json.loads((root / 'compiler_performance.json').read_bytes())
    if (performance['bytes_written'] != len(raw) + sum(f['size'] for f in manifest['files'].values())
            or performance['input_mcap_bytes'] != integrity['source_snapshot_size']
            or performance['compiler_status'] != manifest['compiler_status']
            or performance['parameters'] != manifest['compiler']['parameters']):
        raise ValueError('compiler performance accounting mismatch')
    for field, topic in (('tick_count', '/r2b4/tick'), ('raw_lidar_count', '/r2b4/raw_lidar'),
                         ('checkpoint_count', '/r2b4/checkpoint')):
        if performance[field] != integrity['recovered_by_topic'].get(topic, 0):
            raise ValueError('compiler performance topic count mismatch: ' + topic)
    if performance['source_scan_count'] != 1 or performance['replay_invocations'] != 0:
        raise ValueError('invalid compiler extraction/replay scope')
    return manifest


def verify(bundle, *, source=None):
    root = _root(bundle)
    manifest = verify_manifest(root)
    return verify_artifacts(root, manifest, source=source)


def verify_artifacts(root, manifest, *, source=None):
    """Payload verification, also usable before the final manifest is sealed."""
    root = Path(root)
    actual_files = {str(p.relative_to(root)) for p in root.rglob('*') if p.is_file() and p != root / 'manifest.json'}
    if actual_files != set(manifest['files']):
        raise ValueError('bundle file inventory mismatch')
    for relative, expected in manifest['files'].items():
        path = bundle_file(root, relative)
        if path.stat().st_size != expected['size'] or sha256(path) != expected['sha256']:
            raise ValueError('evidence file hash mismatch: ' + relative)
    coverage = json.loads((root / 'coverage.json').read_bytes())
    integrity = json.loads((root / 'integrity.json').read_bytes())
    if coverage.get('schema') != COVERAGE:
        raise ValueError('invalid coverage schema')
    db = read_only(root)
    digest = hashlib.sha256()
    count = valid = field_count = bulk_array_count = views = 0
    topic_counts, decoded_counts = Counter(), Counter()
    has_bulk_arrays = db.execute(
        "SELECT 1 FROM sqlite_master WHERE type='table' AND name='bulk_arrays'"
    ).fetchone() is not None
    try:
        # Count physical rows too: an unindexed trailing row must not disappear
        # from accounting just because the index or manifest was regenerated.
        for table, paths in (
            ('messages', sorted((root / 'source/messages').glob('*.ndjson'))
             + sorted((root / 'source').glob('quarantined_messages*.ndjson'))),
            ('views', sorted((root / 'normalized').rglob('*.ndjson'))),
        ):
            indexed_files = {r[0] for r in db.execute(f'SELECT DISTINCT file FROM {table}')}
            if indexed_files != {str(p.relative_to(root)) for p in paths}:
                raise ValueError('indexed file inventory mismatch: ' + table)
            for path in paths:
                relative = str(path.relative_to(root))
                with path.open('rb') as stream:
                    rows = sum(1 for _ in stream)
                locators = db.execute(
                    f'SELECT count(*),count(DISTINCT byte_offset) FROM {table} WHERE file=?',
                    (relative,)).fetchone()
                if locators != (rows, rows):
                    raise ValueError('unindexed or duplicate output row: ' + relative)
        if db.execute('PRAGMA integrity_check').fetchone()[0] != 'ok':
            raise ValueError('index database integrity failure')
        for mid, file, offset, length, stored_hash, stored_status in db.execute(
                'SELECT message_id,file,byte_offset,byte_length,payload_sha256,decode_status FROM messages ORDER BY message_id'):
            row = read_row(root, file, offset, length)
            raw = base64.b64decode(row['payload_base64'], validate=True)
            if (row['message_id'] != mid or hashlib.sha256(raw).hexdigest() != row['payload_sha256']
                    or len(raw) != row['payload_size'] or row['payload_sha256'] != stored_hash
                    or row['decode_status'] != stored_status):
                raise ValueError('message identity/byte accounting mismatch: ' + mid)
            indexed = db.execute(
                'SELECT topic,channel_id,sequence,source_offset,log_time_ns,publish_time_ns,tick_id FROM messages WHERE message_id=?',
                (mid,)).fetchone()
            tick = row.get('payload', {}).get('tick_id') if isinstance(row.get('payload'), dict) else None
            expected_index = (row['topic'], row['channel_id'], row['sequence'], row['source_offset'],
                              f"{row['log_time_ns']:020d}", f"{row['publish_time_ns']:020d}",
                              json.dumps(tick) if tick is not None else None)
            if indexed != expected_index:
                raise ValueError('message index mismatch: ' + mid)
            original = {k: v for k, v in row.items() if k not in ('decode_status', 'payload')}
            digest.update(encoded(original))
            status, payload = decode(raw) if row['message_encoding'] == 'json' else ('BINARY', None)
            if status != stored_status:
                raise ValueError('decode status mismatch: ' + mid)
            topic_counts[row['topic']] += 1
            if status == 'JSON':
                decoded_counts[row['topic']] += 1
                valid += 1
                if encoded(payload) != encoded(row['payload']):
                    raise ValueError('decoded payload mismatch: ' + mid)
                if has_bulk_arrays:
                    expected_paths, expected_bulk = [], []
                    for entry_kind, path, array_length in field_index_entries(payload):
                        if entry_kind == 'bulk_array':
                            expected_bulk.append((path, array_length))
                        else:
                            expected_paths.append(path)
                    expected_paths.sort()
                    expected_bulk.sort()
                    stored_bulk = list(db.execute(
                        'SELECT path,length FROM bulk_arrays WHERE message_id=? ORDER BY path', (mid,)))
                    if expected_bulk != stored_bulk:
                        raise ValueError('bulk array census mismatch: ' + mid)
                    bulk_array_count += len(expected_bulk)
                else:
                    # Backward-compatible dense bundle produced before sparse indexing.
                    expected_paths = sorted(path for path, _ in leaves(payload))
                stored_paths = [r[0] for r in db.execute(
                    'SELECT path FROM fields WHERE message_id=? ORDER BY path', (mid,))]
                if expected_paths != stored_paths:
                    raise ValueError('field census mismatch: ' + mid)
                field_count += len(expected_paths)
            expected = {(pointer, 'normalized/' + name): value
                        for name, pointer, value in expected_views(row)}
            observed = set()
            for pointer, view_file, view_offset, view_length in db.execute(
                    'SELECT pointer,file,byte_offset,byte_length FROM views WHERE message_id=?', (mid,)):
                view = read_row(root, view_file, view_offset, view_length)
                # Strip the deterministic unit/part suffix, not user topic text.
                name = view_file.rsplit('_', 2)[0]
                key = (pointer, name)
                if (key in observed or key not in expected or view['message_id'] != mid
                        or view['source_pointer'] != pointer or view['topic'] != row['topic']
                        or view['log_time_ns'] != row['log_time_ns']
                        or encoded(view['payload']) != encoded(expected[key])):
                    raise ValueError('normalized lineage mismatch: ' + mid)
                observed.add(key)
                views += 1
            if observed != set(expected):
                raise ValueError('missing normalized view: ' + mid)
            count += 1
        if digest.hexdigest() != integrity['recovered_message_stream_sha256']:
            raise ValueError('recovered source stream does not match exports')
        if views != db.execute('SELECT count(*) FROM views').fetchone()[0]:
            raise ValueError('orphan normalized view')
        if has_bulk_arrays and bulk_array_count != db.execute('SELECT count(*) FROM bulk_arrays').fetchone()[0]:
            raise ValueError('orphan bulk array posting')
        if (count != integrity['recovered_messages'] or count != coverage['messages']['recovered']
                or count != coverage['messages']['exported'] or valid != coverage['messages']['json_decoded']
                or count - valid != coverage['messages']['quarantined']
                or field_count != coverage['field_occurrences'] or views != coverage['normalized_views']):
            raise ValueError('coverage accounting mismatch')
    finally:
        db.close()
    captured = integrity['recovered_by_topic']
    if ({k: n for k, n in captured.items() if n} != {k: n for k, n in topic_counts.items() if k is not None}
            or integrity['unknown_topic_messages'] != topic_counts[None]):
        raise ValueError('source topic accounting mismatch')
    expected_topics = {
        topic: topic_coverage(total, {'messages': topic_counts[topic], 'json_decoded': decoded_counts[topic]})
        for topic, total in captured.items()}
    expected_unknown = topic_coverage(integrity['unknown_topic_messages'], {
        'messages': topic_counts[None], 'json_decoded': decoded_counts[None]})
    complete_source = integrity['source_integrity'] == 'COMPLETE'
    if (coverage.get('topics') != expected_topics or coverage.get('unknown_topic') != expected_unknown
            or coverage.get('source_message_total_known') is not complete_source
            or coverage.get('unrecoverable_message_count') != (0 if complete_source else None)):
        raise ValueError('topic export coverage mismatch')
    if manifest['all_recoverable_messages_accounted_for'] is not True:
        raise ValueError('manifest does not account for all recovered messages')
    ranges = []
    with (root / 'source/corrupt_ranges.ndjson').open() as stream:
        for line in stream:
            row = json.loads(line)
            if 'start' in row:
                if not 0 <= row['start'] <= row['end'] <= integrity['source_snapshot_size']:
                    raise ValueError('corrupt range outside source snapshot')
                ranges.append((row['start'], row['end']))
    end = lost = 0
    for start, stop in sorted(ranges):
        lost += max(0, stop - max(start, end))
        end = max(end, stop)
    expected_ranges = {'parsed': integrity['source_snapshot_size'] - lost, 'unrecoverable': lost}
    if coverage['byte_ranges'] != expected_ranges or integrity['byte_ranges'] != expected_ranges:
        raise ValueError('byte range accounting mismatch')
    if not all(coverage['invariants'].values()):
        raise ValueError('coverage invariant failed')
    if source is not None:
        h = hashlib.sha256()
        remaining = integrity['source_snapshot_size']
        with Path(source).open('rb') as stream:
            while remaining:
                data = stream.read(min(1024 * 1024, remaining))
                if not data:
                    raise ValueError('source shorter than snapshot')
                remaining -= len(data)
                h.update(data)
        if h.hexdigest() != integrity['source_snapshot_sha256']:
            raise ValueError('source snapshot hash mismatch')
    return {'status': 'PASS', 'messages': count, 'quarantined': count - valid,
            'source_checked': source is not None, 'compiler_status': manifest['compiler_status']}
