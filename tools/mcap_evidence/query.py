"""Indexed bundle queries; the original MCAP is never opened."""
from fnmatch import fnmatchcase
import json
from pathlib import Path

from .index import read_only
from .normalize import leaves


def bundle_file(root: Path, relative: str) -> Path:
    path = root / relative
    if Path(relative).is_absolute() or '..' in Path(relative).parts:
        raise ValueError('unsafe bundle path')
    if any(parent.is_symlink() for parent in (path, *path.parents) if parent != root.parent):
        raise ValueError('symlinks are not evidence files')
    if not path.resolve().is_relative_to(root.resolve()):
        raise ValueError('path outside evidence bundle')
    return path


def read_row(root: Path, file: str, offset: int, length: int):
    if offset < 0 or length <= 0:
        raise ValueError('invalid row locator')
    with bundle_file(root, file).open('rb') as stream:
        stream.seek(offset)
        data = stream.read(length)
    if len(data) != length:
        raise ValueError('truncated evidence row')
    return json.loads(data)


def _field_matches(payload, pattern: str) -> bool:
    """Exact legacy field-query semantics, evaluated lazily on candidates."""
    if payload is None:
        return False
    return any(fnmatchcase(path, pattern) for path, _ in leaves(payload))


def query(bundle, *, topic=None, message_id=None, tick_id=None, layer=None, sensor=None,
          field=None, channel=None, sequence=None, source_offset=None,
          start_ns=None, end_ns=None, limit=100):
    root = Path(bundle).resolve()
    if limit < 1:
        raise ValueError('limit must be positive')
    clauses, parameters = [], []
    for column, value in (('topic', topic), ('message_id', message_id), ('channel_id', channel),
                          ('sequence', sequence), ('source_offset', source_offset)):
        if value is not None:
            clauses.append('m.' + column + '=?')
            parameters.append(value)
    for kind, value in (('tick', tick_id), ('layer', layer), ('sensor', sensor)):
        if value is not None:
            clauses.append('m.message_id IN (SELECT message_id FROM identifiers WHERE kind=? AND value=?)')
            parameters.extend((kind, str(value)))
    field_pattern = None
    if field is not None:
        if not field.startswith('/') and field != '':
            field = '/' + field.replace('.', '/')
        field_pattern = field
    for op, value in (('>=', start_ns), ('<=', end_ns)):
        if value is not None:
            clauses.append('m.log_time_ns' + op + '?')
            parameters.append(f'{value:020d}')
    db = read_only(root)
    try:
        has_bulk_arrays = db.execute(
            "SELECT 1 FROM sqlite_master WHERE type='table' AND name='bulk_arrays'"
        ).fetchone() is not None
        if field_pattern is not None:
            if has_bulk_arrays:
                clauses.append(
                    '(m.message_id IN (SELECT message_id FROM fields WHERE path GLOB ?) '
                    'OR m.message_id IN (SELECT message_id FROM bulk_arrays '
                    "WHERE path GLOB ? OR ? GLOB (path || '/*')))")
                parameters.extend((field_pattern, field_pattern, field_pattern))
            else:
                # Backward-compatible dense evidence bundle.
                clauses.append('m.message_id IN (SELECT message_id FROM fields WHERE path GLOB ?)')
                parameters.append(field_pattern)
        sql = 'SELECT file,byte_offset,byte_length FROM messages m'
        if clauses:
            sql += ' WHERE ' + ' AND '.join(clauses)
        sql += ' ORDER BY message_id'
        if field_pattern is None:
            sql += ' LIMIT ?'
            parameters.append(limit)
        yielded = 0
        for location in db.execute(sql, parameters):
            row = read_row(root, *location)
            if field_pattern is None or _field_matches(row.get('payload'), field_pattern):
                yield row
                yielded += 1
                if yielded >= limit:
                    break
    finally:
        db.close()
