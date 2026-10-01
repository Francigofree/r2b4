"""Indexed bundle queries; the original MCAP is never opened."""
import json
from pathlib import Path

from .index import read_only


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
    if field is not None:
        if not field.startswith('/') and field != '':
            field = '/' + field.replace('.', '/')
        clauses.append('m.message_id IN (SELECT message_id FROM fields WHERE path GLOB ?)')
        parameters.append(field)
    for op, value in (('>=', start_ns), ('<=', end_ns)):
        if value is not None:
            clauses.append('m.log_time_ns' + op + '?')
            parameters.append(f'{value:020d}')
    sql = 'SELECT file,byte_offset,byte_length FROM messages m'
    if clauses:
        sql += ' WHERE ' + ' AND '.join(clauses)
    sql += ' ORDER BY message_id LIMIT ?'
    parameters.append(limit)
    db = read_only(root)
    try:
        for location in db.execute(sql, parameters):
            yield read_row(root, *location)
    finally:
        db.close()
