"""Forward-only salvage at proven record boundaries; never guess a resync point.

The input is scanned once to a fixed fstat EOF. Work files contain original
message bytes, not references into a source that may still be changing.
"""
import base64
import hashlib
import io
import json
import os
from pathlib import Path
import struct
import zlib

from .export import encoded
from .records import (MAGIC, HEADER, FOOTER, SCHEMA, CHANNEL, MESSAGE, CHUNK,
                      METADATA, DATA_END, Cursor, WorkUnit)
from .schemas import SOURCE


def b64(data):
    return base64.b64encode(data).decode('ascii')


class Salvage:
    def __init__(self, source: Path, output: Path):
        self.source, self.output = source, output
        self.work = output / '.work'
        self.work.mkdir()
        self.channels = {}
        self.units = []
        self.messages = 0
        self.topic_counts = {}
        self.unknown_topic_messages = 0
        self.message_digest = hashlib.sha256()
        self.ranges = []
        self.issues = []
        self.files = {}
        self.seen = set()
        self.sha = hashlib.sha256()
        self.data_crc = 0
        self.summary_crc = 0
        self.in_summary = False
        self.closing = False
        self.saw_framing = False

    def emit(self, name, row):
        if name not in self.files:
            path = self.output / 'source' / (name + '.ndjson')
            path.parent.mkdir(parents=True, exist_ok=True)
            self.files[name] = path.open('wb')
        self.files[name].write(encoded(row))

    def issue(self, reason, offset, **details):
        self.issues.append({'reason': reason, 'source_offset': offset, **details})

    def corrupt(self, start, end, reason):
        if end > start:
            self.ranges.append((start, end))
            self.emit('corrupt_ranges', {'start': start, 'end': end, 'reason': reason})
        self.issue(reason, start, end=end)

    def read(self, n):
        data = self.stream.read(min(n, self.size - self.stream.tell()))
        self.sha.update(data)
        self.data_crc = zlib.crc32(data, self.data_crc)
        if self.in_summary:
            self.summary_crc = zlib.crc32(data, self.summary_crc)
        return data

    def content(self, opcode, body, offset, *, chunk_offset=None, nested_offset=None):
        location = {'source_offset': offset}
        if chunk_offset is not None:
            location.update(chunk_offset=chunk_offset, nested_offset=nested_offset)
        row = {**location, 'opcode': opcode, 'content_size': len(body)}
        if opcode != MESSAGE or len(body) < 22:
            row['content_base64'] = b64(body)
        self.emit('records', row)
        cur = Cursor(body)
        try:
            if opcode == HEADER:
                parsed = {'profile': cur.string(), 'library': cur.string()}
            elif opcode == CHANNEL:
                channel, schema = cur.unpack('<HH')
                parsed = {'channel_id': channel, 'schema_id': schema, 'topic': cur.string(),
                          'message_encoding': cur.string(), 'metadata': cur.mapping()}
                if channel in self.channels and self.channels[channel] != parsed:
                    self.issue('CONFLICTING_CHANNEL', offset, channel_id=channel)
                self.channels[channel] = parsed
                self.topic_counts.setdefault(parsed['topic'], 0)
                self.emit('channels', {**location, **parsed})
            elif opcode == SCHEMA:
                sid = cur.unpack('<H')[0]
                parsed = {'schema_id': sid, 'name': cur.string(), 'encoding': cur.string(),
                          'data_base64': b64(cur.take(cur.unpack('<I')[0]))}
                self.emit('schemas', {**location, **parsed})
            elif opcode == METADATA:
                parsed = {'name': cur.string(), 'metadata': cur.mapping()}
                self.emit('metadata', {**location, **parsed})
            elif opcode == MESSAGE:
                channel, sequence, log_time, publish_time = cur.unpack('<HIQQ')
                payload = body[cur.pos:]
                self.messages += 1
                definition = self.channels.get(channel, {})
                if not definition:
                    self.issue('UNDECLARED_CHANNEL', offset, channel_id=channel)
                    self.unknown_topic_messages += 1
                else:
                    self.topic_counts[definition['topic']] += 1
                message = {
                    'schema': SOURCE, 'message_id': f'msg-{self.messages:012d}', **location,
                    'topic': definition.get('topic'), 'channel_id': channel,
                    'message_encoding': definition.get('message_encoding'),
                    'sequence': sequence, 'log_time_ns': log_time, 'publish_time_ns': publish_time,
                    'payload_size': len(payload), 'payload_sha256': hashlib.sha256(payload).hexdigest(),
                    'payload_base64': b64(payload),
                }
                # This digest binds source message identity/bytes to the exported stream.
                self.message_digest.update(encoded(message))
                self.unit_stream.write(encoded(message))
                return
            else:
                return  # Unknown records remain intact in records.ndjson.
            if cur.pos != len(body):
                raise ValueError('trailing bytes in record')
        except (ValueError, UnicodeError, struct.error) as exc:
            self.issue('MALFORMED_RECORD', offset, opcode=opcode, detail=str(exc))
            # Framing is intact: report the content, then continue at its known end.
            if chunk_offset is None or offset != chunk_offset:
                self.corrupt(offset + 9, offset + 9 + len(body), 'MALFORMED_CONTENT')
            else:
                self.emit('corrupt_ranges', {
                    'chunk_offset': chunk_offset, 'nested_start': nested_offset + 9,
                    'nested_end': nested_offset + 9 + len(body), 'reason': 'MALFORMED_CONTENT'})

    def chunk(self, body, offset, complete):
        cur = Cursor(body)
        try:
            start, end, size, crc = cur.unpack('<QQQI')
            compression = cur.string()
            records_size = cur.unpack('<Q')[0]
        except (ValueError, UnicodeError, struct.error):
            self.corrupt(offset + 9, offset + 9 + len(body), 'TRUNCATED_CHUNK_HEADER')
            return
        raw = body[cur.pos:]
        self.emit('records', {'source_offset': offset, 'opcode': CHUNK, 'content_size': len(body),
                              'header_base64': b64(body[:cur.pos]), 'compression': compression,
                              'uncompressed_size': size, 'records_size': records_size,
                              'complete': complete})
        if len(raw) != records_size:
            self.issue('CHUNK_SIZE_MISMATCH', offset)
            complete = False
        # A declared records length is a boundary; extra bytes aren't nested records.
        if len(raw) > records_size:
            self.corrupt(offset + 9 + cur.pos + records_size, offset + 9 + len(body), 'CHUNK_TRAILING_BYTES')
            raw = raw[:records_size]
        if compression:
            path = self.output / 'source' / 'chunks' / f'{offset}.bin'
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(raw)
            try:
                if compression == 'zstd':
                    import zstandard
                    with zstandard.ZstdDecompressor().stream_reader(io.BytesIO(raw)) as stream:
                        records = stream.read()
                elif compression == 'lz4':
                    import lz4.frame
                    records = lz4.frame.decompress(raw)
                else:
                    raise RuntimeError(f'unsupported MCAP compression: {compression}')
            except ImportError as exc:
                raise RuntimeError(f'install the {compression} decoder to compile this MCAP') from exc
            except Exception as exc:
                if isinstance(exc, RuntimeError):
                    raise
                self.corrupt(offset + 9 + cur.pos, offset + 9 + cur.pos + len(raw), 'DECOMPRESSION_ERROR')
                return
        else:
            records = raw
        crc_ok = (zlib.crc32(records) & 0xffffffff) == crc if crc else None
        self.emit('chunk_checks', {'source_offset': offset, 'crc_expected': crc,
                                  'crc_ok': crc_ok, 'actual_uncompressed_size': len(records)})
        if crc_ok is False:
            self.issue('CHUNK_CRC_MISMATCH', offset)
        if len(records) != size:
            self.issue('UNCOMPRESSED_SIZE_MISMATCH', offset)
        pos = 0
        while pos < len(records):
            remaining = len(records) - pos
            length = struct.unpack_from('<Q', records, pos + 1)[0] if remaining >= 9 else remaining
            if remaining < 9 or length > remaining - 9:
                if compression:
                    self.issue('UNRECOVERABLE_NESTED_RANGE', offset, nested_start=pos, nested_end=len(records))
                    self.emit('corrupt_ranges', {'chunk_offset': offset, 'nested_start': pos,
                                                 'nested_end': len(records), 'reason': 'UNRECOVERABLE_NESTED_RANGE'})
                else:
                    base = offset + 9 + cur.pos
                    self.corrupt(base + pos, base + len(records), 'TRUNCATED_TAIL' if not complete else 'UNRECOVERABLE_RANGE')
                break
            opcode = records[pos]
            source_offset = offset if compression else offset + 9 + cur.pos + pos
            if opcode in (HEADER, FOOTER, CHUNK, DATA_END):
                self.issue('INVALID_NESTED_OPCODE', source_offset, opcode=opcode, nested_offset=pos)
                self.emit('records', {'source_offset': source_offset, 'chunk_offset': offset,
                                      'nested_offset': pos, 'opcode': opcode,
                                      'content_base64': b64(records[pos + 9:pos + 9 + length])})
            else:
                self.content(opcode, records[pos + 9:pos + 9 + length], source_offset,
                             chunk_offset=offset, nested_offset=pos)
            pos += 9 + length

    def scan(self):
        try:
            with self.source.open('rb') as self.stream:
                before = os.fstat(self.stream.fileno())
                self.size = before.st_size
                prefix = self.read(min(8, self.size))
                if prefix != MAGIC:
                    self.issue('MISSING_OPENING_MAGIC', 0)
                    # Only offsets 0 and 8 can be justified as a missing/damaged magic.
                    self.stream.seek(0)
                    self.sha = hashlib.sha256()
                    self.data_crc = 0
                    def framed_at(offset):
                        self.stream.seek(offset)
                        record_header = self.stream.read(9)
                        if len(record_header) != 9:
                            return False
                        opcode, length = struct.unpack('<BQ', record_header)
                        minimum = {HEADER: 8, SCHEMA: 14, CHANNEL: 16, MESSAGE: 22,
                                   CHUNK: 40, METADATA: 8}.get(opcode)
                        return minimum is not None and minimum <= length <= self.size - offset - 9

                    if framed_at(0):
                        self.stream.seek(0)
                    elif framed_at(8):
                        self.stream.seek(0)
                        self.read(8)
                        self.corrupt(0, 8, 'INVALID_OPENING_MAGIC')
                    else:
                        raise ValueError('MCAP framing cannot be established')
                while self.stream.tell() < self.size:
                    offset = self.stream.tell()
                    remaining = self.size - offset
                    data_crc_before = self.data_crc
                    summary_crc_before = self.summary_crc
                    header = self.read(min(9, remaining))
                    if header == MAGIC:
                        self.closing = True
                        break
                    if len(header) < 9:
                        self.corrupt(offset, self.size, 'TRUNCATED_TAIL')
                        break
                    opcode, length = struct.unpack('<BQ', header)
                    body = self.read(min(length, self.size - self.stream.tell()))
                    complete = len(body) == length
                    if not complete:
                        self.issue('TRUNCATED_TAIL', offset, declared_size=length, available_size=len(body))
                    number = len(self.units)
                    path = self.work / f'{number:08d}.ndjson'
                    count_before = self.messages
                    with path.open('wb') as self.unit_stream:
                        if opcode == CHUNK:
                            self.chunk(body, offset, complete)
                        elif complete:
                            self.content(opcode, body, offset)
                        else:
                            self.corrupt(offset, self.size, 'TRUNCATED_TAIL')
                    if self.messages > count_before:
                        self.units.append(WorkUnit(number, str(path), self.messages - count_before))
                    else:
                        path.unlink()
                    if complete:
                        self.saw_framing = True
                        self.seen.add(opcode)
                    if opcode == DATA_END and complete:
                        if len(body) != 4:
                            self.issue('INVALID_DATA_END', offset)
                        else:
                            crc = struct.unpack('<I', body)[0]
                            if crc and crc != data_crc_before & 0xffffffff:
                                self.issue('DATA_CRC_MISMATCH', offset)
                        self.in_summary = True
                        self.summary_start = self.stream.tell()
                        self.summary_crc = 0
                    if opcode == FOOTER and complete:
                        if len(body) != 20:
                            self.issue('INVALID_FOOTER', offset)
                        else:
                            start, offsets, crc = struct.unpack('<QQI', body)
                            if start and start != getattr(self, 'summary_start', None):
                                self.issue('INVALID_SUMMARY_START', offset)
                            if offsets and not start <= offsets < offset:
                                self.issue('INVALID_SUMMARY_OFFSET', offset)
                            actual_crc = zlib.crc32(header + body[:16], summary_crc_before if start else 0) & 0xffffffff
                            if crc and crc != actual_crc:
                                self.issue('SUMMARY_CRC_MISMATCH', offset)
                    if not complete:
                        break
                after = os.fstat(self.stream.fileno())
                if self.stream.tell() < self.size:
                    raise OSError('snapshot was not fully read')
            if not self.saw_framing:
                raise ValueError('MCAP framing cannot be established')
            for opcode, label in ((HEADER, 'HEADER'), (DATA_END, 'DATA_END'), (FOOTER, 'FOOTER')):
                if opcode not in self.seen:
                    self.issue('MISSING_' + label, self.size)
            if not self.closing:
                self.issue('MISSING_CLOSING_MAGIC', self.size)
            merged = []
            for start, end in sorted(self.ranges):
                if merged and start <= merged[-1][1]:
                    merged[-1][1] = max(end, merged[-1][1])
                else:
                    merged.append([start, end])
            lost = sum(end - start for start, end in merged)
            return {
                'source_path': str(self.source), 'source_snapshot_size': self.size,
                'source_snapshot_sha256': self.sha.hexdigest(),
                'source_size_after_scan': after.st_size,
                'source_stat_before': [before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns, before.st_ctime_ns],
                'source_integrity': ('TRUNCATED' if any('TRUNCATED' in i['reason'] or i['reason'].startswith('MISSING_') for i in self.issues)
                                     else 'CORRUPT') if self.issues else 'COMPLETE',
                'issues': self.issues,
                'byte_ranges': {'parsed': self.size - lost, 'unrecoverable': lost},
                'recovered_messages': self.messages,
                'recovered_by_topic': self.topic_counts,
                'unknown_topic_messages': self.unknown_topic_messages,
                'recovered_message_stream_sha256': self.message_digest.hexdigest(),
                'work_units': self.units,
            }
        finally:
            source_dir = self.output / 'source'
            source_dir.mkdir(parents=True, exist_ok=True)
            for name in ('records', 'channels', 'schemas', 'metadata', 'corrupt_ranges', 'chunk_checks'):
                if name not in self.files:
                    (source_dir / (name + '.ndjson')).touch()
            for stream in self.files.values():
                stream.close()
