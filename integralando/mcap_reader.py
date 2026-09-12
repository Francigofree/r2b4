"""Dependency-free indexed MCAP reader for R2B4 captures.

This reader intentionally supports the MCAP subset produced by
``v3.mcap_writer.StdlibMcapWriter``: uncompressed chunks, JSON channels,
chunk/message indexes, metadata indexes, statistics, data CRC and summary CRC.
It performs selective chunk reads so high-level tools do not need to decode an
entire recording when only a small time/topic slice is needed.
"""

from __future__ import annotations

import hashlib
import json
import struct
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import BinaryIO, Iterable, Iterator, Mapping, Sequence

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

TICK_TOPIC = "/r2b4/tick"
RAW_LIDAR_TOPIC = "/r2b4/raw_lidar"
CHECKPOINT_TOPIC = "/r2b4/checkpoint"
EVENT_TOPIC = "/r2b4/event"
RUNTIME_TOPIC = "/r2b4/runtime"


class McapReadError(RuntimeError):
    """The MCAP capture is malformed, unsupported or fails integrity checks."""


@dataclass(frozen=True, slots=True)
class ChannelInfo:
    channel_id: int
    schema_id: int
    topic: str
    message_encoding: str
    metadata: Mapping[str, str]


@dataclass(frozen=True, slots=True)
class ChunkIndex:
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
class MetadataIndex:
    offset: int
    length: int
    name: str


@dataclass(frozen=True, slots=True)
class McapMessage:
    topic: str
    channel_id: int
    sequence: int
    log_time_ns: int
    publish_time_ns: int
    data: bytes

    def json(self) -> object:
        try:
            return json.loads(self.data.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise McapReadError(f"message on {self.topic} is not valid UTF-8 JSON") from exc


@dataclass(frozen=True, slots=True)
class McapStructureReport:
    valid: bool
    file_size: int
    profile: str
    library: str
    data_crc_ok: bool
    summary_crc_ok: bool
    chunk_crc_ok: bool
    chunk_count: int
    channel_count: int
    metadata_count: int
    message_count: int | None
    message_start_time: int | None
    message_end_time: int | None
    errors: tuple[str, ...]


class _Cursor:
    __slots__ = ("data", "pos")

    def __init__(self, data: bytes) -> None:
        self.data = data
        self.pos = 0

    def _take(self, count: int) -> bytes:
        if count < 0 or self.pos + count > len(self.data):
            raise McapReadError("truncated MCAP record")
        result = self.data[self.pos : self.pos + count]
        self.pos += count
        return result

    def u8(self) -> int:
        return struct.unpack("<B", self._take(1))[0]

    def u16(self) -> int:
        return struct.unpack("<H", self._take(2))[0]

    def u32(self) -> int:
        return struct.unpack("<I", self._take(4))[0]

    def u64(self) -> int:
        return struct.unpack("<Q", self._take(8))[0]

    def string(self) -> str:
        size = self.u32()
        try:
            return self._take(size).decode("utf-8")
        except UnicodeDecodeError as exc:
            raise McapReadError("invalid UTF-8 in MCAP string") from exc

    def bytes_u64(self) -> bytes:
        return self._take(self.u64())

    def string_map(self) -> dict[str, str]:
        size = self.u32()
        end = self.pos + size
        if end > len(self.data):
            raise McapReadError("truncated MCAP string map")
        result: dict[str, str] = {}
        while self.pos < end:
            key = self.string()
            value = self.string()
            result[key] = value
        if self.pos != end:
            raise McapReadError("invalid MCAP string map length")
        return result

    def u16_u64_map(self) -> dict[int, int]:
        size = self.u32()
        end = self.pos + size
        if end > len(self.data):
            raise McapReadError("truncated MCAP integer map")
        result: dict[int, int] = {}
        while self.pos < end:
            result[self.u16()] = self.u64()
        if self.pos != end:
            raise McapReadError("invalid MCAP integer map length")
        return result

    def remaining(self) -> bytes:
        return self._take(len(self.data) - self.pos)


class McapReader:
    """Selective reader and integrity verifier for R2B4 MCAP captures."""

    def __init__(self, path_value: str | Path) -> None:
        self.path = Path(path_value)
        if self.path.is_symlink() or not self.path.is_file():
            raise McapReadError(f"capture is not a regular non-symlink file: {self.path}")
        self.file_size = self.path.stat().st_size
        if self.file_size < len(MAGIC) * 2 + 29:
            raise McapReadError("MCAP file is too small")

        self.summary_start = 0
        self.summary_offset_start = 0
        self.summary_crc_expected = 0
        self.footer_start = 0
        self.profile = ""
        self.library = ""
        self.channels: dict[int, ChannelInfo] = {}
        self.channels_by_topic: dict[str, ChannelInfo] = {}
        self.chunk_indexes: list[ChunkIndex] = []
        self.metadata_indexes: list[MetadataIndex] = []
        self.statistics: dict[str, object] = {}
        self._read_footer_and_summary()
        self._read_header()

    # ---------- public API ----------

    def inspect(self, *, verify_chunks: bool = True) -> McapStructureReport:
        errors: list[str] = []
        try:
            data_crc_ok = self.verify_data_crc()
        except McapReadError as exc:
            data_crc_ok = False
            errors.append(str(exc))
        try:
            summary_crc_ok = self.verify_summary_crc()
        except McapReadError as exc:
            summary_crc_ok = False
            errors.append(str(exc))

        chunk_crc_ok = True
        if verify_chunks:
            for index in self.chunk_indexes:
                try:
                    self._read_chunk_records(index, verify_crc=True)
                except McapReadError as exc:
                    chunk_crc_ok = False
                    errors.append(str(exc))
                    break

        message_count = self.statistics.get("message_count")
        start = self.statistics.get("message_start_time")
        end = self.statistics.get("message_end_time")
        return McapStructureReport(
            valid=data_crc_ok and summary_crc_ok and chunk_crc_ok and not errors,
            file_size=self.file_size,
            profile=self.profile,
            library=self.library,
            data_crc_ok=data_crc_ok,
            summary_crc_ok=summary_crc_ok,
            chunk_crc_ok=chunk_crc_ok,
            chunk_count=len(self.chunk_indexes),
            channel_count=len(self.channels),
            metadata_count=len(self.metadata_indexes),
            message_count=int(message_count) if isinstance(message_count, int) else None,
            message_start_time=int(start) if isinstance(start, int) else None,
            message_end_time=int(end) if isinstance(end, int) else None,
            errors=tuple(errors),
        )

    def metadata(self, name: str | None = None) -> list[tuple[str, dict[str, str]]]:
        result: list[tuple[str, dict[str, str]]] = []
        with self.path.open("rb") as handle:
            for item in self.metadata_indexes:
                if name is not None and item.name != name:
                    continue
                opcode, content, _length = self._read_record_at(handle, item.offset)
                if opcode != OP_METADATA:
                    raise McapReadError("metadata index points to non-metadata record")
                cursor = _Cursor(content)
                actual_name = cursor.string()
                values = cursor.string_map()
                if actual_name != item.name:
                    raise McapReadError("metadata index name mismatch")
                result.append((actual_name, values))
        return result

    def latest_metadata(self, name: str) -> dict[str, str] | None:
        values = self.metadata(name)
        return values[-1][1] if values else None

    def iter_messages(
        self,
        *,
        topics: Iterable[str] | None = None,
        start_ns: int | None = None,
        end_ns: int | None = None,
        verify_crc: bool = True,
    ) -> Iterator[McapMessage]:
        wanted_topics = None if topics is None else frozenset(str(item) for item in topics)
        if start_ns is not None and start_ns < 0:
            raise ValueError("start_ns must be non-negative or None")
        if end_ns is not None and end_ns < 0:
            raise ValueError("end_ns must be non-negative or None")
        if start_ns is not None and end_ns is not None and start_ns > end_ns:
            raise ValueError("start_ns must not exceed end_ns")

        wanted_channels: set[int] | None = None
        if wanted_topics is not None:
            wanted_channels = {
                channel_id
                for channel_id, channel in self.channels.items()
                if channel.topic in wanted_topics
            }
            if not wanted_channels:
                return

        with self.path.open("rb") as handle:
            for chunk in self.chunk_indexes:
                if start_ns is not None and chunk.message_end_time < start_ns:
                    continue
                if end_ns is not None and chunk.message_start_time > end_ns:
                    continue
                if wanted_channels is not None and not (
                    wanted_channels & set(chunk.message_index_offsets)
                ):
                    continue
                records = self._read_chunk_records(chunk, handle=handle, verify_crc=verify_crc)
                for opcode, content in _iter_records_from_bytes(records):
                    if opcode != OP_MESSAGE:
                        continue
                    cursor = _Cursor(content)
                    channel_id = cursor.u16()
                    sequence = cursor.u32()
                    log_time = cursor.u64()
                    publish_time = cursor.u64()
                    data = cursor.remaining()
                    channel = self.channels.get(channel_id)
                    if channel is None:
                        raise McapReadError(f"message references unknown channel {channel_id}")
                    if wanted_channels is not None and channel_id not in wanted_channels:
                        continue
                    if start_ns is not None and log_time < start_ns:
                        continue
                    if end_ns is not None and log_time > end_ns:
                        continue
                    yield McapMessage(
                        topic=channel.topic,
                        channel_id=channel_id,
                        sequence=sequence,
                        log_time_ns=log_time,
                        publish_time_ns=publish_time,
                        data=data,
                    )

    def iter_json_messages(self, **kwargs: object) -> Iterator[tuple[McapMessage, object]]:
        for message in self.iter_messages(**kwargs):
            yield message, message.json()

    def first_json(self, topic: str) -> tuple[McapMessage, object] | None:
        for message, payload in self.iter_json_messages(topics=(topic,)):
            return message, payload
        return None

    def last_json(self, topic: str) -> tuple[McapMessage, object] | None:
        result: tuple[McapMessage, object] | None = None
        for result in self.iter_json_messages(topics=(topic,)):
            pass
        return result

    def verify_data_crc(self) -> bool:
        data_end_offset, expected = self._find_data_end()
        actual = 0
        remaining = data_end_offset
        with self.path.open("rb") as handle:
            while remaining:
                chunk = handle.read(min(1024 * 1024, remaining))
                if not chunk:
                    raise McapReadError("unexpected EOF while verifying data CRC")
                actual = zlib.crc32(chunk, actual)
                remaining -= len(chunk)
        return (actual & 0xFFFFFFFF) == expected

    def verify_summary_crc(self) -> bool:
        if self.summary_start <= 0:
            return self.summary_crc_expected == 0
        with self.path.open("rb") as handle:
            handle.seek(self.summary_start)
            summary = handle.read(self.footer_start - self.summary_start)
        crc = zlib.crc32(summary)
        crc = zlib.crc32(
            struct.pack(
                "<BQQQ",
                OP_FOOTER,
                20,
                self.summary_start,
                self.summary_offset_start,
            ),
            crc,
        ) & 0xFFFFFFFF
        return crc == self.summary_crc_expected

    def sha256(self) -> str:
        digest = hashlib.sha256()
        with self.path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()

    # ---------- parsing ----------

    def _read_footer_and_summary(self) -> None:
        footer_start = self.file_size - len(MAGIC) - (1 + 8 + 20)
        with self.path.open("rb") as handle:
            handle.seek(0)
            if handle.read(len(MAGIC)) != MAGIC:
                raise McapReadError("MCAP opening magic mismatch")
            handle.seek(self.file_size - len(MAGIC))
            if handle.read(len(MAGIC)) != MAGIC:
                raise McapReadError("MCAP closing magic mismatch")
            opcode, content, record_length = self._read_record_at(handle, footer_start)
            if opcode != OP_FOOTER or record_length != 29:
                raise McapReadError("invalid MCAP footer")
            cursor = _Cursor(content)
            self.summary_start = cursor.u64()
            self.summary_offset_start = cursor.u64()
            self.summary_crc_expected = cursor.u32()
            self.footer_start = footer_start

            if self.summary_start:
                if not (len(MAGIC) < self.summary_start <= self.footer_start):
                    raise McapReadError("invalid summary_start")
                handle.seek(self.summary_start)
                while handle.tell() < self.footer_start:
                    offset = handle.tell()
                    opcode, content, length = _read_record(handle)
                    if offset + length > self.footer_start:
                        raise McapReadError("summary record overlaps footer")
                    self._consume_summary_record(opcode, content)
                if handle.tell() != self.footer_start:
                    raise McapReadError("summary does not end at footer")

        for channel in self.channels.values():
            if channel.topic in self.channels_by_topic:
                raise McapReadError(f"duplicate MCAP topic: {channel.topic}")
            self.channels_by_topic[channel.topic] = channel

    def _read_header(self) -> None:
        with self.path.open("rb") as handle:
            handle.seek(len(MAGIC))
            opcode, content, _length = _read_record(handle)
        if opcode != OP_HEADER:
            raise McapReadError("first data record is not MCAP Header")
        cursor = _Cursor(content)
        self.profile = cursor.string()
        self.library = cursor.string()

    def _consume_summary_record(self, opcode: int, content: bytes) -> None:
        cursor = _Cursor(content)
        if opcode == OP_CHANNEL:
            channel_id = cursor.u16()
            channel = ChannelInfo(
                channel_id=channel_id,
                schema_id=cursor.u16(),
                topic=cursor.string(),
                message_encoding=cursor.string(),
                metadata=cursor.string_map(),
            )
            previous = self.channels.get(channel_id)
            if previous is not None and previous != channel:
                raise McapReadError(f"conflicting channel definition {channel_id}")
            self.channels[channel_id] = channel
        elif opcode == OP_CHUNK_INDEX:
            self.chunk_indexes.append(
                ChunkIndex(
                    message_start_time=cursor.u64(),
                    message_end_time=cursor.u64(),
                    chunk_start_offset=cursor.u64(),
                    chunk_length=cursor.u64(),
                    message_index_offsets=cursor.u16_u64_map(),
                    message_index_length=cursor.u64(),
                    compression=cursor.string(),
                    compressed_size=cursor.u64(),
                    uncompressed_size=cursor.u64(),
                )
            )
        elif opcode == OP_METADATA_INDEX:
            self.metadata_indexes.append(
                MetadataIndex(
                    offset=cursor.u64(),
                    length=cursor.u64(),
                    name=cursor.string(),
                )
            )
        elif opcode == OP_STATISTICS:
            message_count = cursor.u64()
            schema_count = cursor.u16()
            channel_count = cursor.u32()
            attachment_count = cursor.u32()
            metadata_count = cursor.u32()
            chunk_count = cursor.u32()
            message_start_time = cursor.u64()
            message_end_time = cursor.u64()
            counts_size = cursor.u32()
            counts_end = cursor.pos + counts_size
            channel_counts: dict[int, int] = {}
            while cursor.pos < counts_end:
                channel_counts[cursor.u16()] = cursor.u64()
            self.statistics = {
                "message_count": message_count,
                "schema_count": schema_count,
                "channel_count": channel_count,
                "attachment_count": attachment_count,
                "metadata_count": metadata_count,
                "chunk_count": chunk_count,
                "message_start_time": message_start_time,
                "message_end_time": message_end_time,
                "channel_message_counts": channel_counts,
            }
        elif opcode in {OP_SCHEMA, OP_ATTACHMENT_INDEX, OP_SUMMARY_OFFSET}:
            return
        else:
            raise McapReadError(f"unsupported record opcode {opcode:#x} in summary")

    def _find_data_end(self) -> tuple[int, int]:
        if self.summary_start <= 0:
            limit = self.footer_start
        else:
            limit = self.summary_start
        with self.path.open("rb") as handle:
            handle.seek(len(MAGIC))
            while handle.tell() < limit:
                offset = handle.tell()
                opcode, content, _length = _read_record(handle)
                if opcode == OP_DATA_END:
                    cursor = _Cursor(content)
                    expected = cursor.u32()
                    return offset, expected
        raise McapReadError("MCAP DataEnd record not found")

    def _read_chunk_records(
        self,
        index: ChunkIndex,
        *,
        handle: BinaryIO | None = None,
        verify_crc: bool = True,
    ) -> bytes:
        owned = handle is None
        stream = handle or self.path.open("rb")
        try:
            opcode, content, record_length = self._read_record_at(stream, index.chunk_start_offset)
            if opcode != OP_CHUNK:
                raise McapReadError("chunk index points to non-chunk record")
            if record_length != index.chunk_length:
                raise McapReadError("chunk index length mismatch")
            cursor = _Cursor(content)
            start = cursor.u64()
            end = cursor.u64()
            uncompressed_size = cursor.u64()
            expected_crc = cursor.u32()
            compression = cursor.string()
            records = cursor.bytes_u64()
            if compression not in {"", "none"}:
                raise McapReadError(f"unsupported MCAP compression: {compression}")
            if len(records) != uncompressed_size:
                raise McapReadError("chunk uncompressed size mismatch")
            if start != index.message_start_time or end != index.message_end_time:
                raise McapReadError("chunk index timestamp mismatch")
            if verify_crc and (zlib.crc32(records) & 0xFFFFFFFF) != expected_crc:
                raise McapReadError(
                    f"chunk CRC mismatch at offset {index.chunk_start_offset}"
                )
            return records
        finally:
            if owned:
                stream.close()

    @staticmethod
    def _read_record_at(handle: BinaryIO, offset: int) -> tuple[int, bytes, int]:
        handle.seek(offset)
        return _read_record(handle)


def _read_record(handle: BinaryIO) -> tuple[int, bytes, int]:
    header = handle.read(9)
    if len(header) != 9:
        raise McapReadError("truncated MCAP record header")
    opcode = header[0]
    length = struct.unpack("<Q", header[1:])[0]
    if length > 2**31:
        raise McapReadError("unreasonably large MCAP record")
    content = handle.read(length)
    if len(content) != length:
        raise McapReadError("truncated MCAP record content")
    return opcode, content, 9 + length


def _iter_records_from_bytes(data: bytes) -> Iterator[tuple[int, bytes]]:
    pos = 0
    while pos < len(data):
        if pos + 9 > len(data):
            raise McapReadError("truncated nested MCAP record header")
        opcode = data[pos]
        length = struct.unpack("<Q", data[pos + 1 : pos + 9])[0]
        start = pos + 9
        end = start + length
        if end > len(data):
            raise McapReadError("truncated nested MCAP record")
        yield opcode, data[start:end]
        pos = end
    if pos != len(data):
        raise McapReadError("invalid nested MCAP record length")


__all__ = [
    "CHECKPOINT_TOPIC",
    "ChannelInfo",
    "ChunkIndex",
    "EVENT_TOPIC",
    "McapMessage",
    "McapReadError",
    "McapReader",
    "McapStructureReport",
    "RAW_LIDAR_TOPIC",
    "RUNTIME_TOPIC",
    "TICK_TOPIC",
]
