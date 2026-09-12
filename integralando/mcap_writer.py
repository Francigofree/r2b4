"""Small dependency-free indexed MCAP v0 writer for R2B4.

The implementation intentionally uses only the Python standard library so it can
run on the Raspberry Pi 5 system Python without pip or a virtual environment.
It writes uncompressed chunks with per-chunk CRC32, message indexes, chunk
indexes, metadata indexes, statistics, data-section CRC32 and summary CRC32.
"""

from __future__ import annotations

import struct
import zlib
from collections import defaultdict
from dataclasses import dataclass
from typing import BinaryIO, Mapping, Sequence

MAGIC = b"\x89MCAP0\r\n"

OP_HEADER = 0x01
OP_FOOTER = 0x02
OP_SCHEMA = 0x03
OP_CHANNEL = 0x04
OP_MESSAGE = 0x05
OP_CHUNK = 0x06
OP_MESSAGE_INDEX = 0x07
OP_CHUNK_INDEX = 0x08
OP_ATTACHMENT = 0x09
OP_ATTACHMENT_INDEX = 0x0A
OP_STATISTICS = 0x0B
OP_METADATA = 0x0C
OP_METADATA_INDEX = 0x0D
OP_SUMMARY_OFFSET = 0x0E
OP_DATA_END = 0x0F


def _u8(value: int) -> bytes:
    return struct.pack("<B", value)


def _u16(value: int) -> bytes:
    return struct.pack("<H", value)


def _u32(value: int) -> bytes:
    return struct.pack("<I", value)


def _u64(value: int) -> bytes:
    return struct.pack("<Q", value)


def _mcap_string(value: str) -> bytes:
    raw = value.encode("utf-8")
    return _u32(len(raw)) + raw


def _bytes_u64(value: bytes) -> bytes:
    return _u64(len(value)) + value


def _string_map(values: Mapping[str, str] | None = None) -> bytes:
    body = bytearray()
    for key, value in (values or {}).items():
        body += _mcap_string(str(key))
        body += _mcap_string(str(value))
    return _u32(len(body)) + bytes(body)


def _u16_u64_map(values: Mapping[int, int]) -> bytes:
    body = bytearray()
    for key, value in sorted(values.items()):
        body += _u16(key)
        body += _u64(value)
    return _u32(len(body)) + bytes(body)


def _timestamp_offset_array(values: Sequence[tuple[int, int]]) -> bytes:
    body = bytearray()
    for timestamp, offset in values:
        body += _u64(timestamp)
        body += _u64(offset)
    return _u32(len(body)) + bytes(body)


def _record(opcode: int, content: bytes) -> bytes:
    return _u8(opcode) + _u64(len(content)) + content


def _message_record(
    channel_id: int,
    sequence: int,
    log_time_ns: int,
    publish_time_ns: int,
    data: bytes,
) -> bytes:
    return _record(
        OP_MESSAGE,
        _u16(channel_id)
        + _u32(sequence)
        + _u64(log_time_ns)
        + _u64(publish_time_ns)
        + data,
    )


@dataclass(frozen=True, slots=True)
class _ChannelDef:
    channel_id: int
    schema_id: int
    topic: str
    message_encoding: str
    metadata: Mapping[str, str]
    record_bytes: bytes


@dataclass(frozen=True, slots=True)
class _ChunkIndexDef:
    message_start_time: int
    message_end_time: int
    chunk_start_offset: int
    chunk_length: int
    message_index_offsets: Mapping[int, int]
    message_index_length: int
    compression: str
    compressed_size: int
    uncompressed_size: int


@dataclass(frozen=True, slots=True)
class _MetadataIndexDef:
    offset: int
    length: int
    name: str


class StdlibMcapWriter:
    """Indexed MCAP writer using only the Python standard library.

    R2B4 deliberately defaults to uncompressed chunks. This avoids any pip/venv
    dependency and minimizes CPU use on Raspberry Pi 5. Indexing is retained,
    so readers can still seek by time/topic without scanning the whole file.
    """

    def __init__(self, stream: BinaryIO, *, chunk_target_bytes: int = 1_048_576) -> None:
        if chunk_target_bytes <= 0:
            raise ValueError("chunk_target_bytes must be positive")
        if not callable(getattr(stream, "write", None)) or not callable(getattr(stream, "tell", None)):
            raise TypeError("stream must be a seekable/tellable binary writer")

        self._stream = stream
        self._chunk_target_bytes = int(chunk_target_bytes)
        self._channels: dict[int, _ChannelDef] = {}
        self._next_channel_id = 1
        self._channel_sequences: dict[int, int] = defaultdict(int)
        self._channel_message_counts: dict[int, int] = defaultdict(int)
        self._message_count = 0
        self._message_start_time: int | None = None
        self._message_end_time: int | None = None

        self._chunk_buffer = bytearray()
        self._chunk_message_offsets: dict[int, list[tuple[int, int]]] = defaultdict(list)
        self._chunk_start_time: int | None = None
        self._chunk_end_time: int | None = None
        self._chunk_indexes: list[_ChunkIndexDef] = []
        self._metadata_indexes: list[_MetadataIndexDef] = []
        self._metadata_count = 0

        self._data_crc = 0
        self._started = False
        self._finished = False

    @property
    def message_count(self) -> int:
        return self._message_count

    @property
    def chunk_count(self) -> int:
        return len(self._chunk_indexes)

    def tell(self) -> int:
        return int(self._stream.tell())

    def _write_raw(self, value: bytes) -> None:
        self._stream.write(value)

    def _write_data_bytes(self, value: bytes) -> None:
        self._write_raw(value)
        self._data_crc = zlib.crc32(value, self._data_crc)

    def _write_data_record(self, opcode: int, content: bytes) -> bytes:
        rec = _record(opcode, content)
        self._write_data_bytes(rec)
        return rec

    def start(
        self,
        *,
        profile: str = "r2b4-v3",
        library: str = "R2B4 stdlib indexed MCAP writer/1",
    ) -> None:
        if self._started:
            raise RuntimeError("writer already started")
        self._write_data_bytes(MAGIC)
        self._write_data_record(OP_HEADER, _mcap_string(profile) + _mcap_string(library))
        self._started = True

    def register_channel(
        self,
        topic: str,
        *,
        message_encoding: str = "json",
        schema_id: int = 0,
        metadata: Mapping[str, str] | None = None,
    ) -> int:
        if not self._started or self._finished:
            raise RuntimeError("writer must be started and unfinished")
        if self._chunk_buffer:
            raise RuntimeError("channels must be registered before messages")
        if not isinstance(topic, str) or not topic:
            raise ValueError("topic must be non-empty")
        if schema_id != 0:
            raise ValueError("this minimal writer currently supports schema_id=0 only")

        channel_id = self._next_channel_id
        self._next_channel_id += 1
        if channel_id > 0xFFFF:
            raise RuntimeError("too many MCAP channels")

        md = {str(key): str(value) for key, value in (metadata or {}).items()}
        rec = self._write_data_record(
            OP_CHANNEL,
            _u16(channel_id)
            + _u16(schema_id)
            + _mcap_string(topic)
            + _mcap_string(message_encoding)
            + _string_map(md),
        )
        self._channels[channel_id] = _ChannelDef(
            channel_id=channel_id,
            schema_id=schema_id,
            topic=topic,
            message_encoding=message_encoding,
            metadata=md,
            record_bytes=rec,
        )
        return channel_id

    def add_metadata(self, name: str, values: Mapping[str, str]) -> None:
        if not self._started or self._finished:
            raise RuntimeError("writer must be started and unfinished")
        if not isinstance(name, str) or not name:
            raise ValueError("metadata name must be non-empty")
        self.flush_chunk()
        content = _mcap_string(name) + _string_map(
            {str(key): str(value) for key, value in values.items()}
        )
        offset = self.tell()
        rec = self._write_data_record(OP_METADATA, content)
        self._metadata_indexes.append(_MetadataIndexDef(offset, len(rec), name))
        self._metadata_count += 1

    def add_message(
        self,
        channel_id: int,
        *,
        log_time_ns: int,
        data: bytes,
        publish_time_ns: int | None = None,
        sequence: int | None = None,
    ) -> None:
        if not self._started or self._finished:
            raise RuntimeError("writer must be started and unfinished")
        if channel_id not in self._channels:
            raise ValueError("unknown channel_id")
        if not isinstance(log_time_ns, int) or isinstance(log_time_ns, bool) or log_time_ns < 0:
            raise ValueError("log_time_ns must be a non-negative integer")
        if publish_time_ns is None:
            publish_time_ns = log_time_ns
        if not isinstance(publish_time_ns, int) or isinstance(publish_time_ns, bool) or publish_time_ns < 0:
            raise ValueError("publish_time_ns must be a non-negative integer")
        if not isinstance(data, bytes):
            raise TypeError("data must be bytes")

        if sequence is None:
            sequence = self._channel_sequences[channel_id]
            self._channel_sequences[channel_id] = (sequence + 1) & 0xFFFFFFFF
        elif not isinstance(sequence, int) or isinstance(sequence, bool) or sequence < 0:
            raise ValueError("sequence must be a non-negative integer or None")
        sequence &= 0xFFFFFFFF

        msg = _message_record(channel_id, sequence, log_time_ns, publish_time_ns, data)
        if self._chunk_buffer and len(self._chunk_buffer) + len(msg) > self._chunk_target_bytes:
            self.flush_chunk()

        offset = len(self._chunk_buffer)
        self._chunk_buffer += msg
        self._chunk_message_offsets[channel_id].append((log_time_ns, offset))

        if self._chunk_start_time is None:
            self._chunk_start_time = log_time_ns
            self._chunk_end_time = log_time_ns
        else:
            self._chunk_start_time = min(self._chunk_start_time, log_time_ns)
            self._chunk_end_time = max(self._chunk_end_time or log_time_ns, log_time_ns)

        self._message_count += 1
        self._channel_message_counts[channel_id] += 1
        if self._message_start_time is None:
            self._message_start_time = log_time_ns
            self._message_end_time = log_time_ns
        else:
            self._message_start_time = min(self._message_start_time, log_time_ns)
            self._message_end_time = max(self._message_end_time or log_time_ns, log_time_ns)

        if len(self._chunk_buffer) >= self._chunk_target_bytes:
            self.flush_chunk()

    def flush_chunk(self) -> None:
        if not self._chunk_buffer:
            return

        records = bytes(self._chunk_buffer)
        start_time = int(self._chunk_start_time or 0)
        end_time = int(self._chunk_end_time or 0)
        chunk_content = (
            _u64(start_time)
            + _u64(end_time)
            + _u64(len(records))
            + _u32(zlib.crc32(records) & 0xFFFFFFFF)
            + _mcap_string("")
            + _bytes_u64(records)
        )

        chunk_start_offset = self.tell()
        chunk_record = self._write_data_record(OP_CHUNK, chunk_content)
        chunk_length = len(chunk_record)

        message_index_offsets: dict[int, int] = {}
        message_index_start = self.tell()
        for channel_id in sorted(self._chunk_message_offsets):
            entries = self._chunk_message_offsets[channel_id]
            index_offset = self.tell()
            self._write_data_record(
                OP_MESSAGE_INDEX,
                _u16(channel_id) + _timestamp_offset_array(entries),
            )
            message_index_offsets[channel_id] = index_offset

        self._chunk_indexes.append(
            _ChunkIndexDef(
                message_start_time=start_time,
                message_end_time=end_time,
                chunk_start_offset=chunk_start_offset,
                chunk_length=chunk_length,
                message_index_offsets=message_index_offsets,
                message_index_length=self.tell() - message_index_start,
                compression="",
                compressed_size=len(records),
                uncompressed_size=len(records),
            )
        )

        self._chunk_buffer.clear()
        self._chunk_message_offsets.clear()
        self._chunk_start_time = None
        self._chunk_end_time = None

    def _chunk_index_record(self, item: _ChunkIndexDef) -> bytes:
        return _record(
            OP_CHUNK_INDEX,
            _u64(item.message_start_time)
            + _u64(item.message_end_time)
            + _u64(item.chunk_start_offset)
            + _u64(item.chunk_length)
            + _u16_u64_map(item.message_index_offsets)
            + _u64(item.message_index_length)
            + _mcap_string(item.compression)
            + _u64(item.compressed_size)
            + _u64(item.uncompressed_size),
        )

    def _metadata_index_record(self, item: _MetadataIndexDef) -> bytes:
        return _record(
            OP_METADATA_INDEX,
            _u64(item.offset) + _u64(item.length) + _mcap_string(item.name),
        )

    def _statistics_record(self) -> bytes:
        counts = bytearray()
        for channel_id, count in sorted(self._channel_message_counts.items()):
            counts += _u16(channel_id)
            counts += _u64(count)
        return _record(
            OP_STATISTICS,
            _u64(self._message_count)
            + _u16(0)  # schema_count: schema_id=0 only
            + _u32(len(self._channels))
            + _u32(0)  # attachment_count
            + _u32(self._metadata_count)
            + _u32(len(self._chunk_indexes))
            + _u64(self._message_start_time or 0)
            + _u64(self._message_end_time or 0)
            + _u32(len(counts))
            + bytes(counts),
        )

    def finish(self) -> None:
        if self._finished:
            return
        if not self._started:
            raise RuntimeError("writer not started")

        self.flush_chunk()

        # DataEnd itself is not included in data_section_crc.
        self._write_raw(_record(OP_DATA_END, _u32(self._data_crc & 0xFFFFFFFF)))

        summary_start = self.tell()
        groups: list[tuple[int, list[bytes]]] = []
        if self._channels:
            groups.append((OP_CHANNEL, [item.record_bytes for item in self._channels.values()]))
        if self._chunk_indexes:
            groups.append((OP_CHUNK_INDEX, [self._chunk_index_record(item) for item in self._chunk_indexes]))
        groups.append((OP_STATISTICS, [self._statistics_record()]))
        if self._metadata_indexes:
            groups.append((OP_METADATA_INDEX, [self._metadata_index_record(item) for item in self._metadata_indexes]))
        groups.sort(key=lambda item: item[0])

        summary = bytearray()
        group_locations: list[tuple[int, int, int]] = []
        for opcode, records in groups:
            group_start_rel = len(summary)
            for rec in records:
                summary += rec
            group_locations.append((opcode, summary_start + group_start_rel, len(summary) - group_start_rel))

        summary_offset_start = summary_start + len(summary)
        for opcode, group_start, group_length in group_locations:
            summary += _record(
                OP_SUMMARY_OFFSET,
                _u8(opcode) + _u64(group_start) + _u64(group_length),
            )

        # MCAP summary CRC includes all summary bytes plus the Footer opcode,
        # Footer record length and the first two Footer fields, but not summary_crc.
        summary_crc = zlib.crc32(bytes(summary))
        summary_crc = zlib.crc32(
            struct.pack(
                "<BQQQ",
                OP_FOOTER,
                8 + 8 + 4,
                summary_start if summary else 0,
                summary_offset_start if group_locations else 0,
            ),
            summary_crc,
        ) & 0xFFFFFFFF

        self._write_raw(bytes(summary))
        self._write_raw(
            _record(
                OP_FOOTER,
                _u64(summary_start if summary else 0)
                + _u64(summary_offset_start if group_locations else 0)
                + _u32(summary_crc),
            )
        )
        self._write_raw(MAGIC)
        self._finished = True


__all__ = ["MAGIC", "StdlibMcapWriter"]
