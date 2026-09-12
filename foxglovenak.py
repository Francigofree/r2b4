#!/usr/bin/env python3
"""
foxglovenak.py

R2B4 V3 capture -> indexed Foxglove-compatible MCAP exporter.

NO external Python package is required.
NO pip, NO venv, NO ROS dependency.

Default:
  1. Finds the newest V3 capture in:
       /home/alba/project_r2b4/runtime/captures
  2. Reads the complete JSON capture.
  3. Creates next to it:
       same_name.mcap
  4. Preserves the original JSON byte-for-byte as an MCAP attachment.
  5. Publishes the capture into Foxglove-readable topics.
  6. Adds Foxglove visualization topics: robot marker, pose, path, LiDAR and costmap.
  7. Normalizes visualization to the first L3 pose (R2B4_CAPTURE_MAP).
  8. Writes Chunk + Message Index + Chunk Index + Summary + Summary Offset
     sections, so Foxglove recognizes the file as INDEXED.

Usage:
    python3 foxglovenak.py

Optional:
    python3 foxglovenak.py --input /path/to/capture.json
    python3 foxglovenak.py --capture-dir /path/to/captures
    python3 foxglovenak.py --chunk-mib 8

The source JSON capture is never modified.
Existing same-name .mcap is atomically replaced.
"""

from __future__ import annotations

import argparse
from bisect import bisect_left
import base64
import json
import math
import os
import re
import struct
import sys
import zlib
from collections import defaultdict
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence

DEFAULT_CAPTURE_DIR = Path("/home/alba/project_r2b4/runtime/captures")
DEFAULT_CHUNK_MIB = 8

MAGIC = b"\x89MCAP0\r\n"

# MCAP v0 opcodes.
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
OP_SUMMARY_OFFSET = 0x0E
OP_DATA_END = 0x0F

_CAPTURE_NAME_RE = re.compile(
    r"^v3_(?P<date>\d{8})_(?P<time>\d{6})_(?P<rest>.+)_capture\.json$"
)

# Foxglove enum value for FLOAT32 in foxglove.PackedElementField.NumericType.
FOXGLOVE_FLOAT32 = 7

POINTCLOUD_JSON_SCHEMA: dict[str, Any] = {
    "title": "foxglove.PointCloud",
    "type": "object",
    "properties": {
        "timestamp": {
            "type": "object",
            "properties": {
                "sec": {"type": "integer", "minimum": 0},
                "nsec": {"type": "integer", "minimum": 0, "maximum": 999999999},
            },
            "required": ["sec", "nsec"],
        },
        "frame_id": {"type": "string"},
        "pose": {
            "type": "object",
            "properties": {
                "position": {
                    "type": "object",
                    "properties": {
                        "x": {"type": "number"},
                        "y": {"type": "number"},
                        "z": {"type": "number"},
                    },
                    "required": ["x", "y", "z"],
                },
                "orientation": {
                    "type": "object",
                    "properties": {
                        "x": {"type": "number"},
                        "y": {"type": "number"},
                        "z": {"type": "number"},
                        "w": {"type": "number"},
                    },
                    "required": ["x", "y", "z", "w"],
                },
            },
            "required": ["position", "orientation"],
        },
        "point_stride": {"type": "integer", "minimum": 0},
        "fields": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "name": {"type": "string"},
                    "offset": {"type": "integer", "minimum": 0},
                    "type": {"type": "integer", "minimum": 0, "maximum": 8},
                },
                "required": ["name", "offset", "type"],
            },
        },
        "data": {"type": "string", "contentEncoding": "base64"},
    },
    "required": [
        "timestamp",
        "frame_id",
        "pose",
        "point_stride",
        "fields",
        "data",
    ],
}


VIZ_FRAME_ID = "R2B4_CAPTURE_MAP"

_TIMESTAMP_JSON_SCHEMA: dict[str, Any] = {
    "type": "object",
    "properties": {
        "sec": {"type": "integer", "minimum": 0},
        "nsec": {"type": "integer", "minimum": 0, "maximum": 999999999},
    },
    "required": ["sec", "nsec"],
}

_VECTOR3_JSON_SCHEMA: dict[str, Any] = {
    "type": "object",
    "properties": {
        "x": {"type": "number"},
        "y": {"type": "number"},
        "z": {"type": "number"},
    },
    "required": ["x", "y", "z"],
}

_QUATERNION_JSON_SCHEMA: dict[str, Any] = {
    "type": "object",
    "properties": {
        "x": {"type": "number"},
        "y": {"type": "number"},
        "z": {"type": "number"},
        "w": {"type": "number"},
    },
    "required": ["x", "y", "z", "w"],
}

_POSE_JSON_SCHEMA: dict[str, Any] = {
    "title": "foxglove.Pose",
    "type": "object",
    "properties": {
        "position": _VECTOR3_JSON_SCHEMA,
        "orientation": _QUATERNION_JSON_SCHEMA,
    },
    "required": ["position", "orientation"],
}

_COLOR_JSON_SCHEMA: dict[str, Any] = {
    "type": "object",
    "properties": {
        "r": {"type": "number"},
        "g": {"type": "number"},
        "b": {"type": "number"},
        "a": {"type": "number"},
    },
    "required": ["r", "g", "b", "a"],
}

POSE_IN_FRAME_JSON_SCHEMA: dict[str, Any] = {
    "title": "foxglove.PoseInFrame",
    "description": "A timestamped pose for an object or reference frame in 3D space",
    "type": "object",
    "properties": {
        "timestamp": _TIMESTAMP_JSON_SCHEMA,
        "frame_id": {"type": "string"},
        "pose": _POSE_JSON_SCHEMA,
    },
    "required": ["timestamp", "frame_id", "pose"],
}

_ARROW_JSON_SCHEMA: dict[str, Any] = {
    "type": "object",
    "properties": {
        "pose": _POSE_JSON_SCHEMA,
        "shaft_length": {"type": "number"},
        "shaft_diameter": {"type": "number"},
        "head_length": {"type": "number"},
        "head_diameter": {"type": "number"},
        "color": _COLOR_JSON_SCHEMA,
    },
    "required": [
        "pose",
        "shaft_length",
        "shaft_diameter",
        "head_length",
        "head_diameter",
        "color",
    ],
}

_LINE_JSON_SCHEMA: dict[str, Any] = {
    "type": "object",
    "properties": {
        "type": {"type": "integer", "minimum": 0, "maximum": 2},
        "pose": _POSE_JSON_SCHEMA,
        "thickness": {"type": "number"},
        "scale_invariant": {"type": "boolean"},
        "points": {"type": "array", "items": _VECTOR3_JSON_SCHEMA},
        "color": _COLOR_JSON_SCHEMA,
        "colors": {"type": "array", "items": _COLOR_JSON_SCHEMA},
        "indices": {"type": "array", "items": {"type": "integer", "minimum": 0}},
    },
    "required": [
        "type",
        "pose",
        "thickness",
        "scale_invariant",
        "points",
        "color",
        "colors",
        "indices",
    ],
}

# Foxglove's JSON-schema reader expects every array schema to define an
# element schema. R2B4 currently emits these primitive arrays empty, but an
# explicit object item type is still required or Foxglove can fail while
# parsing SceneUpdate with "Cannot read properties of undefined (reading 'type')".
def _unused_scene_primitive_schema(title: str) -> dict[str, Any]:
    return {
        "title": title,
        "type": "object",
        "properties": {},
    }


SCENE_UPDATE_JSON_SCHEMA: dict[str, Any] = {
    "title": "foxglove.SceneUpdate",
    "description": "An update to the entities displayed in a 3D scene",
    "type": "object",
    "properties": {
        "deletions": {"type": "array", "items": {"type": "object"}},
        "entities": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "timestamp": _TIMESTAMP_JSON_SCHEMA,
                    "frame_id": {"type": "string"},
                    "id": {"type": "string"},
                    "lifetime": {
                        "type": "object",
                        "properties": {
                            "sec": {"type": "integer"},
                            "nsec": {
                                "type": "integer",
                                "minimum": 0,
                                "maximum": 999999999,
                            },
                        },
                        "required": ["sec", "nsec"],
                    },
                    "frame_locked": {"type": "boolean"},
                    "metadata": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "key": {"type": "string"},
                                "value": {"type": "string"},
                            },
                            "required": ["key", "value"],
                        },
                    },
                    "arrows": {"type": "array", "items": _ARROW_JSON_SCHEMA},
                    "cubes": {
                        "type": "array",
                        "items": _unused_scene_primitive_schema("foxglove.CubePrimitive"),
                    },
                    "spheres": {
                        "type": "array",
                        "items": _unused_scene_primitive_schema("foxglove.SpherePrimitive"),
                    },
                    "cylinders": {
                        "type": "array",
                        "items": _unused_scene_primitive_schema("foxglove.CylinderPrimitive"),
                    },
                    "lines": {"type": "array", "items": _LINE_JSON_SCHEMA},
                    "triangles": {
                        "type": "array",
                        "items": _unused_scene_primitive_schema("foxglove.TriangleListPrimitive"),
                    },
                    "texts": {
                        "type": "array",
                        "items": _unused_scene_primitive_schema("foxglove.TextPrimitive"),
                    },
                    "models": {
                        "type": "array",
                        "items": _unused_scene_primitive_schema("foxglove.ModelPrimitive"),
                    },
                },
                "required": [
                    "timestamp",
                    "frame_id",
                    "id",
                    "lifetime",
                    "frame_locked",
                    "metadata",
                    "arrows",
                    "cubes",
                    "spheres",
                    "cylinders",
                    "lines",
                    "triangles",
                    "texts",
                    "models",
                ],
            },
        },
    },
    "required": ["deletions", "entities"],
}


def fail(message: str, code: int = 1) -> "NoReturn":
    print(f"ERROR: {message}", file=sys.stderr)
    raise SystemExit(code)


# ---------------------------------------------------------------------------
# MCAP primitive serialization
# ---------------------------------------------------------------------------

def u8(value: int) -> bytes:
    return struct.pack("<B", value)


def u16(value: int) -> bytes:
    return struct.pack("<H", value)


def u32(value: int) -> bytes:
    return struct.pack("<I", value)


def u64(value: int) -> bytes:
    return struct.pack("<Q", value)


def mcap_string(value: str) -> bytes:
    raw = value.encode("utf-8")
    return u32(len(raw)) + raw


def bytes_u32(value: bytes) -> bytes:
    return u32(len(value)) + value


def bytes_u64(value: bytes) -> bytes:
    return u64(len(value)) + value


def string_map(values: Mapping[str, str] | None = None) -> bytes:
    body = bytearray()
    for key, value in (values or {}).items():
        body += mcap_string(str(key))
        body += mcap_string(str(value))
    return u32(len(body)) + bytes(body)


def u16_u64_map(values: Mapping[int, int]) -> bytes:
    body = bytearray()
    for key, value in sorted(values.items()):
        body += u16(key)
        body += u64(value)
    return u32(len(body)) + bytes(body)


def timestamp_offset_array(values: Sequence[tuple[int, int]]) -> bytes:
    body = bytearray()
    for timestamp, offset in values:
        body += u64(timestamp)
        body += u64(offset)
    return u32(len(body)) + bytes(body)


def record(opcode: int, content: bytes) -> bytes:
    return u8(opcode) + u64(len(content)) + content


def message_record(
    channel_id: int,
    sequence: int,
    log_time_ns: int,
    publish_time_ns: int,
    data: bytes,
) -> bytes:
    return record(
        OP_MESSAGE,
        u16(channel_id)
        + u32(sequence)
        + u64(log_time_ns)
        + u64(publish_time_ns)
        + data,
    )


# ---------------------------------------------------------------------------
# Writer state
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class ChannelDef:
    channel_id: int
    schema_id: int
    topic: str
    message_encoding: str
    metadata: Mapping[str, str]
    record_bytes: bytes


@dataclass(frozen=True)
class SchemaDef:
    schema_id: int
    name: str
    encoding: str
    data: bytes
    record_bytes: bytes


@dataclass(frozen=True)
class ChunkIndexDef:
    message_start_time: int
    message_end_time: int
    chunk_start_offset: int
    chunk_length: int
    message_index_offsets: Mapping[int, int]
    message_index_length: int
    compression: str
    compressed_size: int
    uncompressed_size: int


@dataclass(frozen=True)
class AttachmentIndexDef:
    offset: int
    length: int
    log_time: int
    create_time: int
    data_size: int
    name: str
    media_type: str


class IndexedMcapWriter:
    """
    Dependency-free indexed MCAP v0 writer.

    Messages are placed into uncompressed Chunk records. Every chunk gets
    Message Index records. The summary contains duplicate Schema/Channel
    records, Chunk Index records, Attachment Index records, Statistics, and a
    Summary Offset section.
    """

    def __init__(self, stream, chunk_target_bytes: int) -> None:
        if chunk_target_bytes <= 0:
            raise ValueError("chunk_target_bytes must be positive")

        self.stream = stream
        self.chunk_target_bytes = chunk_target_bytes

        self.schemas: list[SchemaDef] = []
        self.channels: list[ChannelDef] = []
        self.next_schema_id = 1
        self.next_channel_id = 1

        self.channel_sequences: dict[int, int] = defaultdict(int)
        self.channel_message_counts: dict[int, int] = defaultdict(int)
        self.message_count = 0
        self.message_start_time: int | None = None
        self.message_end_time: int | None = None

        self.chunk_buffer = bytearray()
        self.chunk_message_offsets: dict[int, list[tuple[int, int]]] = defaultdict(list)
        self.chunk_start_time: int | None = None
        self.chunk_end_time: int | None = None

        self.chunk_indexes: list[ChunkIndexDef] = []
        self.attachment_indexes: list[AttachmentIndexDef] = []

        self.started = False
        self.finished = False

    def tell(self) -> int:
        return int(self.stream.tell())

    def write_raw(self, value: bytes) -> None:
        self.stream.write(value)

    def start(self) -> None:
        if self.started:
            raise RuntimeError("writer already started")
        self.write_raw(MAGIC)
        self.write_raw(
            record(
                OP_HEADER,
                mcap_string("") + mcap_string("R2B4 foxglovenak.py stdlib indexed"),
            )
        )
        self.started = True

    def add_schema(self, name: str, encoding: str, data: bytes) -> int:
        if self.chunk_buffer:
            raise RuntimeError("schemas must be registered before messages")
        schema_id = self.next_schema_id
        self.next_schema_id += 1
        if schema_id > 0xFFFF:
            raise RuntimeError("too many schemas")

        rec = record(
            OP_SCHEMA,
            u16(schema_id)
            + mcap_string(name)
            + mcap_string(encoding)
            + bytes_u32(data),
        )
        self.write_raw(rec)
        self.schemas.append(
            SchemaDef(schema_id, name, encoding, data, rec)
        )
        return schema_id

    def add_channel(
        self,
        topic: str,
        message_encoding: str,
        schema_id: int = 0,
        metadata: Mapping[str, str] | None = None,
    ) -> int:
        if self.chunk_buffer:
            raise RuntimeError("channels must be registered before messages")
        channel_id = self.next_channel_id
        self.next_channel_id += 1
        if channel_id > 0xFFFF:
            raise RuntimeError("too many channels")

        md = dict(metadata or {})
        rec = record(
            OP_CHANNEL,
            u16(channel_id)
            + u16(schema_id)
            + mcap_string(topic)
            + mcap_string(message_encoding)
            + string_map(md),
        )
        self.write_raw(rec)
        self.channels.append(
            ChannelDef(
                channel_id,
                schema_id,
                topic,
                message_encoding,
                md,
                rec,
            )
        )
        return channel_id

    def add_message(
        self,
        channel_id: int,
        log_time_ns: int,
        publish_time_ns: int,
        data: bytes,
    ) -> None:
        if not self.started:
            raise RuntimeError("writer not started")

        sequence = self.channel_sequences[channel_id]
        self.channel_sequences[channel_id] = (sequence + 1) & 0xFFFFFFFF

        msg = message_record(
            channel_id,
            sequence,
            log_time_ns,
            publish_time_ns,
            data,
        )

        # Keep chunks bounded. A single oversized message gets its own chunk.
        if self.chunk_buffer and len(self.chunk_buffer) + len(msg) > self.chunk_target_bytes:
            self.flush_chunk()

        offset = len(self.chunk_buffer)
        self.chunk_buffer += msg
        self.chunk_message_offsets[channel_id].append((log_time_ns, offset))

        if self.chunk_start_time is None:
            self.chunk_start_time = log_time_ns
            self.chunk_end_time = log_time_ns
        else:
            self.chunk_start_time = min(self.chunk_start_time, log_time_ns)
            self.chunk_end_time = max(self.chunk_end_time or log_time_ns, log_time_ns)

        self.message_count += 1
        self.channel_message_counts[channel_id] += 1
        if self.message_start_time is None:
            self.message_start_time = log_time_ns
            self.message_end_time = log_time_ns
        else:
            self.message_start_time = min(self.message_start_time, log_time_ns)
            self.message_end_time = max(self.message_end_time or log_time_ns, log_time_ns)

    def flush_chunk(self) -> None:
        if not self.chunk_buffer:
            return

        records = bytes(self.chunk_buffer)
        start_time = int(self.chunk_start_time or 0)
        end_time = int(self.chunk_end_time or 0)

        chunk_start_offset = self.tell()
        chunk_content = (
            u64(start_time)
            + u64(end_time)
            + u64(len(records))
            + u32(zlib.crc32(records) & 0xFFFFFFFF)
            + mcap_string("")       # no compression
            + bytes_u64(records)
        )
        chunk_record = record(OP_CHUNK, chunk_content)
        self.write_raw(chunk_record)
        chunk_length = len(chunk_record)

        message_index_offsets: dict[int, int] = {}
        message_index_start = self.tell()

        for channel_id in sorted(self.chunk_message_offsets):
            entries = self.chunk_message_offsets[channel_id]
            index_offset = self.tell()
            index_record = record(
                OP_MESSAGE_INDEX,
                u16(channel_id) + timestamp_offset_array(entries),
            )
            self.write_raw(index_record)
            message_index_offsets[channel_id] = index_offset

        message_index_length = self.tell() - message_index_start

        self.chunk_indexes.append(
            ChunkIndexDef(
                message_start_time=start_time,
                message_end_time=end_time,
                chunk_start_offset=chunk_start_offset,
                chunk_length=chunk_length,
                message_index_offsets=dict(message_index_offsets),
                message_index_length=message_index_length,
                compression="",
                compressed_size=len(records),
                uncompressed_size=len(records),
            )
        )

        self.chunk_buffer.clear()
        self.chunk_message_offsets.clear()
        self.chunk_start_time = None
        self.chunk_end_time = None

    def add_attachment(
        self,
        *,
        log_time_ns: int,
        create_time_ns: int,
        name: str,
        media_type: str,
        data: bytes,
    ) -> None:
        self.flush_chunk()

        prefix = (
            u64(log_time_ns)
            + u64(create_time_ns)
            + mcap_string(name)
            + mcap_string(media_type)
            + bytes_u64(data)
        )
        attachment_record = record(
            OP_ATTACHMENT,
            prefix + u32(zlib.crc32(prefix) & 0xFFFFFFFF),
        )

        offset = self.tell()
        self.write_raw(attachment_record)

        self.attachment_indexes.append(
            AttachmentIndexDef(
                offset=offset,
                length=len(attachment_record),
                log_time=log_time_ns,
                create_time=create_time_ns,
                data_size=len(data),
                name=name,
                media_type=media_type,
            )
        )

    def _chunk_index_record(self, item: ChunkIndexDef) -> bytes:
        return record(
            OP_CHUNK_INDEX,
            u64(item.message_start_time)
            + u64(item.message_end_time)
            + u64(item.chunk_start_offset)
            + u64(item.chunk_length)
            + u16_u64_map(item.message_index_offsets)
            + u64(item.message_index_length)
            + mcap_string(item.compression)
            + u64(item.compressed_size)
            + u64(item.uncompressed_size),
        )

    def _attachment_index_record(self, item: AttachmentIndexDef) -> bytes:
        return record(
            OP_ATTACHMENT_INDEX,
            u64(item.offset)
            + u64(item.length)
            + u64(item.log_time)
            + u64(item.create_time)
            + u64(item.data_size)
            + mcap_string(item.name)
            + mcap_string(item.media_type),
        )

    def _statistics_record(self) -> bytes:
        counts_body = bytearray()
        for channel_id, count in sorted(self.channel_message_counts.items()):
            counts_body += u16(channel_id)
            counts_body += u64(count)

        return record(
            OP_STATISTICS,
            u64(self.message_count)
            + u16(len(self.schemas))
            + u32(len(self.channels))
            + u32(len(self.attachment_indexes))
            + u32(0)  # metadata_count
            + u32(len(self.chunk_indexes))
            + u64(self.message_start_time or 0)
            + u64(self.message_end_time or 0)
            + u32(len(counts_body))
            + bytes(counts_body),
        )

    def finish(self) -> None:
        if self.finished:
            return
        self.flush_chunk()

        # End of Data section.
        self.write_raw(record(OP_DATA_END, u32(0)))

        summary_start = self.tell()

        # Summary records MUST be grouped by opcode.
        summary_groups: list[tuple[int, list[bytes]]] = []

        if self.schemas:
            summary_groups.append(
                (OP_SCHEMA, [item.record_bytes for item in self.schemas])
            )
        if self.channels:
            summary_groups.append(
                (OP_CHANNEL, [item.record_bytes for item in self.channels])
            )
        if self.chunk_indexes:
            summary_groups.append(
                (
                    OP_CHUNK_INDEX,
                    [self._chunk_index_record(item) for item in self.chunk_indexes],
                )
            )
        if self.attachment_indexes:
            summary_groups.append(
                (
                    OP_ATTACHMENT_INDEX,
                    [
                        self._attachment_index_record(item)
                        for item in self.attachment_indexes
                    ],
                )
            )

        # Statistics comes last in summary; there is at most one.
        summary_groups.append((OP_STATISTICS, [self._statistics_record()]))

        group_locations: list[tuple[int, int, int]] = []
        for opcode, records in summary_groups:
            group_start = self.tell()
            for rec in records:
                self.write_raw(rec)
            group_length = self.tell() - group_start
            group_locations.append((opcode, group_start, group_length))
        summary_offset_start = self.tell()

        for opcode, group_start, group_length in group_locations:
            self.write_raw(
                record(
                    OP_SUMMARY_OFFSET,
                    u8(opcode) + u64(group_start) + u64(group_length),
                )
            )

        # Footer summary_crc=0 is explicitly allowed by the MCAP spec.
        self.write_raw(
            record(
                OP_FOOTER,
                u64(summary_start)
                + u64(summary_offset_start)
                + u32(0),
            )
        )
        self.write_raw(MAGIC)
        self.finished = True


# ---------------------------------------------------------------------------
# R2B4 capture selection + timeline
# ---------------------------------------------------------------------------

def capture_sort_key(path: Path) -> tuple[int, float, str]:
    match = _CAPTURE_NAME_RE.match(path.name)
    if match:
        stamp = match.group("date") + match.group("time")
        try:
            dt = datetime.strptime(stamp, "%Y%m%d%H%M%S").replace(
                tzinfo=timezone.utc
            )
            return (1, dt.timestamp(), path.name)
        except ValueError:
            pass
    try:
        return (0, path.stat().st_mtime, path.name)
    except OSError:
        return (0, 0.0, path.name)


def find_latest_capture(capture_dir: Path) -> Path:
    if not capture_dir.is_dir():
        fail(f"capture directory does not exist: {capture_dir}")

    candidates = [
        path
        for path in capture_dir.glob("v3_*_capture.json")
        if path.is_file() and not path.name.startswith(".")
    ]
    if not candidates:
        fail(f"no V3 capture found in: {capture_dir}")

    return max(candidates, key=capture_sort_key)


def nonnegative_int(value: Any) -> int | None:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        return None
    return value


def created_at_ns(value: Any) -> int | None:
    if not isinstance(value, str):
        return None
    text = value.strip()
    if not text:
        return None
    if text.endswith("Z"):
        text = text[:-1] + "+00:00"
    try:
        dt = datetime.fromisoformat(text)
    except ValueError:
        return None
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    try:
        return max(0, int(dt.timestamp() * 1_000_000_000))
    except (OverflowError, OSError):
        return None


def scan_monotonic_ns(scan: Mapping[str, Any]) -> int | None:
    for key in (
        "measurement_monotonic_ns",
        "scan_end_monotonic_ns",
        "captured_monotonic_ns",
        "scan_start_monotonic_ns",
    ):
        value = nonnegative_int(scan.get(key))
        if value is not None:
            return value
    return None


def collect_monotonic_times(capture: Mapping[str, Any]) -> list[int]:
    values: list[int] = []

    ticks = capture.get("ticks")
    if isinstance(ticks, list):
        for tick in ticks:
            if isinstance(tick, Mapping):
                value = nonnegative_int(tick.get("monotonic_ns"))
                if value is not None:
                    values.append(value)

    scans = capture.get("raw_lidar_scans")
    if isinstance(scans, list):
        for scan in scans:
            if isinstance(scan, Mapping):
                value = scan_monotonic_ns(scan)
                if value is not None:
                    values.append(value)

    window = capture.get("capture_window")
    if isinstance(window, Mapping):
        for key in (
            "trigger_monotonic_ns",
            "captured_first_monotonic_ns",
            "captured_last_monotonic_ns",
        ):
            value = nonnegative_int(window.get(key))
            if value is not None:
                values.append(value)

    return values


class Timeline:
    """
    Maps boot-local monotonic timestamps to a Foxglove-friendly wall-clock
    timeline while preserving exact relative timing.
    """

    def __init__(self, capture: Mapping[str, Any], source: Path) -> None:
        values = collect_monotonic_times(capture)
        self.min_mono = min(values) if values else 0
        self.max_mono = max(values) if values else 0

        anchor = created_at_ns(capture.get("created_at_utc"))
        if anchor is None:
            try:
                anchor = int(source.stat().st_mtime * 1_000_000_000)
            except OSError:
                anchor = int(
                    datetime.now(timezone.utc).timestamp() * 1_000_000_000
                )
        self.wall_at_max = max(0, anchor)

    def wall_ns(self, monotonic_ns: int | None) -> int:
        if monotonic_ns is None or self.max_mono <= 0:
            return self.wall_at_max
        return max(0, self.wall_at_max - (self.max_mono - monotonic_ns))


# ---------------------------------------------------------------------------
# Foxglove conversion
# ---------------------------------------------------------------------------

def compact_json(value: Any) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")


def foxglove_timestamp(wall_ns: int) -> dict[str, int]:
    sec, nsec = divmod(int(wall_ns), 1_000_000_000)
    return {"sec": sec, "nsec": nsec}


@dataclass(frozen=True)
class PoseSample:
    tick_id: int
    monotonic_ns: int
    frame_id: str
    x_m: float
    y_m: float
    yaw_rad: float
    v_mps: float
    omega_rad_s: float
    safety_decision: str


def _finite_number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    result = float(value)
    return result if math.isfinite(result) else None


def yaw_quaternion(yaw_rad: float) -> dict[str, float]:
    half = 0.5 * yaw_rad
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(half),
        "w": math.cos(half),
    }


def pose_dict(x_m: float, y_m: float, yaw_rad: float, z_m: float = 0.0) -> dict[str, Any]:
    return {
        "position": {"x": x_m, "y": y_m, "z": z_m},
        "orientation": yaw_quaternion(yaw_rad),
    }


def identity_pose(z_m: float = 0.0) -> dict[str, Any]:
    return {
        "position": {"x": 0.0, "y": 0.0, "z": z_m},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    }


def extract_pose_samples(capture: Mapping[str, Any]) -> list[PoseSample]:
    result: list[PoseSample] = []
    ticks = capture.get("ticks")
    if not isinstance(ticks, list):
        return result

    for tick in ticks:
        if not isinstance(tick, Mapping):
            continue
        mono = nonnegative_int(tick.get("monotonic_ns"))
        tick_id = tick.get("tick_id")
        expected = tick.get("expected")
        if mono is None or not isinstance(tick_id, int) or not isinstance(expected, Mapping):
            continue
        layers = expected.get("layers")
        if not isinstance(layers, Mapping):
            continue
        l3 = layers.get("L3")
        if not isinstance(l3, Mapping):
            continue

        x_m = _finite_number(l3.get("x_m"))
        y_m = _finite_number(l3.get("y_m"))
        yaw_rad = _finite_number(l3.get("yaw_rad"))
        v_mps = _finite_number(l3.get("v_mps"))
        omega_rad_s = _finite_number(l3.get("omega_rad_s"))
        frame_id = l3.get("frame_id")
        if (
            x_m is None
            or y_m is None
            or yaw_rad is None
            or v_mps is None
            or omega_rad_s is None
            or not isinstance(frame_id, str)
            or not frame_id
        ):
            continue

        l12 = layers.get("L12")
        safety = "UNKNOWN"
        if isinstance(l12, Mapping) and isinstance(l12.get("safety_decision"), str):
            safety = str(l12["safety_decision"])

        result.append(
            PoseSample(
                tick_id=int(tick_id),
                monotonic_ns=mono,
                frame_id=frame_id,
                x_m=x_m,
                y_m=y_m,
                yaw_rad=yaw_rad,
                v_mps=v_mps,
                omega_rad_s=omega_rad_s,
                safety_decision=safety,
            )
        )

    result.sort(key=lambda item: item.monotonic_ns)
    return result


def _track_width_candidates(capture: Mapping[str, Any]) -> list[tuple[str, float]]:
    found: list[tuple[str, float]] = []

    def walk(value: Any, path: str) -> None:
        if isinstance(value, Mapping):
            for key, child in value.items():
                child_path = f"{path}/{key}"
                if key == "track_width_m":
                    number = _finite_number(child)
                    if number is not None and number > 0.0:
                        found.append((child_path, number))
                walk(child, child_path)
        elif isinstance(value, list):
            for index, child in enumerate(value):
                walk(child, f"{path}/{index}")

    walk(capture.get("configuration"), "configuration")
    return found


def track_width_from_capture(capture: Mapping[str, Any]) -> tuple[float, str]:
    candidates = _track_width_candidates(capture)
    preferred = [
        item
        for item in candidates
        if "/chassis_control/track_width_m" in item[0]
    ]
    if preferred:
        return preferred[0][1], preferred[0][0]
    if candidates:
        return candidates[0][1], candidates[0][0]
    # Visualization-only fallback. It is explicitly labeled in metadata and
    # never feeds control or replay.
    return 0.35, "visualization_fallback"


def normalize_xy(x_m: float, y_m: float, origin: PoseSample) -> tuple[float, float]:
    dx = x_m - origin.x_m
    dy = y_m - origin.y_m
    c = math.cos(origin.yaw_rad)
    s = math.sin(origin.yaw_rad)
    return c * dx + s * dy, -s * dx + c * dy


def normalize_yaw(yaw_rad: float, origin: PoseSample) -> float:
    delta = yaw_rad - origin.yaw_rad
    return math.atan2(math.sin(delta), math.cos(delta))


def normalized_pose(sample: PoseSample, origin: PoseSample) -> tuple[float, float, float]:
    x_m, y_m = normalize_xy(sample.x_m, sample.y_m, origin)
    return x_m, y_m, normalize_yaw(sample.yaw_rad, origin)


def nearest_pose_sample(samples: Sequence[PoseSample], monotonic_ns: int | None) -> PoseSample | None:
    if not samples:
        return None
    if monotonic_ns is None:
        return samples[-1]
    times = [item.monotonic_ns for item in samples]
    index = bisect_left(times, monotonic_ns)
    if index <= 0:
        return samples[0]
    if index >= len(samples):
        return samples[-1]
    before = samples[index - 1]
    after = samples[index]
    if monotonic_ns - before.monotonic_ns <= after.monotonic_ns - monotonic_ns:
        return before
    return after


def safety_color(safety_decision: str) -> dict[str, float]:
    if safety_decision == "ALLOW":
        return {"r": 0.10, "g": 0.85, "b": 0.20, "a": 1.0}
    if safety_decision == "FAULT":
        return {"r": 0.95, "g": 0.10, "b": 0.10, "a": 1.0}
    if safety_decision == "STOP":
        return {"r": 1.00, "g": 0.65, "b": 0.05, "a": 1.0}
    return {"r": 0.20, "g": 0.55, "b": 1.00, "a": 1.0}


def scene_entity(
    *,
    wall_ns: int,
    entity_id: str,
    frame_id: str,
    metadata: Sequence[Mapping[str, str]] = (),
    arrows: Sequence[Mapping[str, Any]] = (),
    lines: Sequence[Mapping[str, Any]] = (),
) -> dict[str, Any]:
    return {
        "timestamp": foxglove_timestamp(wall_ns),
        "frame_id": frame_id,
        "id": entity_id,
        "lifetime": {"sec": 0, "nsec": 0},
        "frame_locked": False,
        "metadata": list(metadata),
        "arrows": list(arrows),
        "cubes": [],
        "spheres": [],
        "cylinders": [],
        "lines": list(lines),
        "triangles": [],
        "texts": [],
        "models": [],
    }


def robot_scene_message(
    sample: PoseSample,
    origin: PoseSample,
    wall_ns: int,
    track_width_m: float,
) -> dict[str, Any]:
    x_m, y_m, yaw_rad = normalized_pose(sample, origin)
    robot_pose = pose_dict(x_m, y_m, yaw_rad, 0.03)
    color = safety_color(sample.safety_decision)
    axle = {
        "type": 2,  # LINE_LIST
        "pose": robot_pose,
        "thickness": 4.0,
        "scale_invariant": True,
        "points": [
            {"x": 0.0, "y": -track_width_m / 2.0, "z": 0.0},
            {"x": 0.0, "y": track_width_m / 2.0, "z": 0.0},
        ],
        "color": color,
        "colors": [],
        "indices": [],
    }

    centerline = {
        "type": 2,  # LINE_LIST
        "pose": robot_pose,
        "thickness": 3.0,
        "scale_invariant": True,
        "points": [
            {"x": -0.18 * track_width_m, "y": 0.0, "z": 0.0},
            {"x": 0.34 * track_width_m, "y": 0.0, "z": 0.0},
        ],
        "color": color,
        "colors": [],
        "indices": [],
    }

    heading = {
        "pose": robot_pose,
        "shaft_length": 0.55 * track_width_m,
        "shaft_diameter": 0.045 * track_width_m,
        "head_length": 0.18 * track_width_m,
        "head_diameter": 0.12 * track_width_m,
        "color": color,
    }

    entity = scene_entity(
        wall_ns=wall_ns,
        entity_id="alba_robot",
        frame_id=VIZ_FRAME_ID,
        metadata=[
            {"key": "tick_id", "value": str(sample.tick_id)},
            {"key": "source_frame", "value": sample.frame_id},
            {"key": "safety", "value": sample.safety_decision},
            {"key": "v_mps", "value": f"{sample.v_mps:.6g}"},
            {"key": "omega_rad_s", "value": f"{sample.omega_rad_s:.6g}"},
        ],
        arrows=[heading],
        lines=[axle, centerline],
    )
    return {"deletions": [], "entities": [entity]}


def path_scene_message(
    points: Sequence[tuple[float, float]],
    wall_ns: int,
) -> dict[str, Any]:
    line = {
        "type": 0,  # LINE_STRIP
        "pose": identity_pose(0.01),
        "thickness": 3.0,
        "scale_invariant": True,
        "points": [
            {"x": x_m, "y": y_m, "z": 0.0}
            for x_m, y_m in points
        ],
        "color": {"r": 0.10, "g": 0.45, "b": 1.00, "a": 0.90},
        "colors": [],
        "indices": [],
    }
    entity = scene_entity(
        wall_ns=wall_ns,
        entity_id="alba_path",
        frame_id=VIZ_FRAME_ID,
        metadata=[{"key": "point_count", "value": str(len(points))}],
        lines=[line],
    )
    return {"deletions": [], "entities": [entity]}


def pose_in_frame_message(
    sample: PoseSample,
    origin: PoseSample,
    wall_ns: int,
) -> dict[str, Any]:
    x_m, y_m, yaw_rad = normalized_pose(sample, origin)
    return {
        "timestamp": foxglove_timestamp(wall_ns),
        "frame_id": VIZ_FRAME_ID,
        "pose": pose_dict(x_m, y_m, yaw_rad, 0.0),
    }


def lidar_to_pointcloud(
    scan: Mapping[str, Any],
    wall_ns: int,
    pose_sample: PoseSample | None = None,
    origin: PoseSample | None = None,
) -> dict[str, Any] | None:
    points = scan.get("points")
    if not isinstance(points, list):
        return None

    packed = bytearray()
    count = 0

    for point in points:
        if (
            not isinstance(point, Sequence)
            or isinstance(point, (str, bytes))
            or len(point) < 2
        ):
            continue

        try:
            angle_deg = float(point[0])
            distance_m = float(point[1])
            quality = float(point[2]) if len(point) >= 3 else 0.0
        except (TypeError, ValueError):
            continue

        if not (
            math.isfinite(angle_deg)
            and math.isfinite(distance_m)
            and math.isfinite(quality)
            and distance_m >= 0.0
        ):
            continue

        angle = math.radians(angle_deg)
        x = distance_m * math.cos(angle)
        y = distance_m * math.sin(angle)

        packed += struct.pack("<ffff", x, y, 0.0, quality)
        count += 1

    if count == 0:
        return None

    if pose_sample is not None and origin is not None:
        x_m, y_m, yaw_rad = normalized_pose(pose_sample, origin)
        frame_id = VIZ_FRAME_ID
        cloud_pose = pose_dict(x_m, y_m, yaw_rad, 0.0)
    else:
        frame_id = "lidar"
        cloud_pose = identity_pose()

    return {
        "timestamp": foxglove_timestamp(wall_ns),
        "frame_id": frame_id,
        "pose": cloud_pose,
        "point_stride": 16,
        "fields": [
            {"name": "x", "offset": 0, "type": FOXGLOVE_FLOAT32},
            {"name": "y", "offset": 4, "type": FOXGLOVE_FLOAT32},
            {"name": "z", "offset": 8, "type": FOXGLOVE_FLOAT32},
            {"name": "quality", "offset": 12, "type": FOXGLOVE_FLOAT32},
        ],
        "data": base64.b64encode(bytes(packed)).decode("ascii"),
    }


def costmap_to_pointcloud(
    l4_output: Mapping[str, Any],
    wall_ns: int,
    origin: PoseSample,
) -> tuple[dict[str, Any] | None, int | None]:
    local_costmap = l4_output.get("local_costmap")
    if not isinstance(local_costmap, Mapping):
        return None, None

    revision = local_costmap.get("revision")
    if not isinstance(revision, int) or isinstance(revision, bool):
        revision = None

    resolution = _finite_number(local_costmap.get("resolution_m"))
    cells = local_costmap.get("occupied_cells")
    if resolution is None or resolution <= 0.0 or not isinstance(cells, list):
        return None, revision

    packed = bytearray()
    count = 0
    for cell in cells:
        if not isinstance(cell, Mapping):
            continue
        grid_x = cell.get("grid_x")
        grid_y = cell.get("grid_y")
        observation_count = cell.get("observation_count")
        if (
            not isinstance(grid_x, int)
            or isinstance(grid_x, bool)
            or not isinstance(grid_y, int)
            or isinstance(grid_y, bool)
            or not isinstance(observation_count, int)
            or isinstance(observation_count, bool)
        ):
            continue

        source_x = (grid_x + 0.5) * resolution
        source_y = (grid_y + 0.5) * resolution
        x_m, y_m = normalize_xy(source_x, source_y, origin)
        packed += struct.pack(
            "<ffff",
            x_m,
            y_m,
            0.0,
            float(observation_count),
        )
        count += 1

    if count == 0:
        return None, revision

    cloud = {
        "timestamp": foxglove_timestamp(wall_ns),
        "frame_id": VIZ_FRAME_ID,
        "pose": identity_pose(),
        "point_stride": 16,
        "fields": [
            {"name": "x", "offset": 0, "type": FOXGLOVE_FLOAT32},
            {"name": "y", "offset": 4, "type": FOXGLOVE_FLOAT32},
            {"name": "z", "offset": 8, "type": FOXGLOVE_FLOAT32},
            {"name": "observation_count", "offset": 12, "type": FOXGLOVE_FLOAT32},
        ],
        "data": base64.b64encode(bytes(packed)).decode("ascii"),
    }
    return cloud, revision


@dataclass(frozen=True)
class Event:
    log_time_ns: int
    order: int
    channel_id: int
    data: bytes


def layer_names_from_capture(capture: Mapping[str, Any]) -> list[str]:
    names: set[str] = set()
    ticks = capture.get("ticks")
    if not isinstance(ticks, list):
        return []
    for tick in ticks:
        if not isinstance(tick, Mapping):
            continue
        expected = tick.get("expected")
        if not isinstance(expected, Mapping):
            continue
        layers = expected.get("layers")
        if not isinstance(layers, Mapping):
            continue
        for name in layers:
            if isinstance(name, str) and name:
                names.add(name)

    def key(name: str) -> tuple[int, str]:
        match = re.fullmatch(r"L(\d+)", name)
        return (int(match.group(1)), name) if match else (9999, name)

    return sorted(names, key=key)


def export_capture(
    source: Path,
    output: Path,
    capture: Mapping[str, Any],
    source_bytes: bytes,
    chunk_mib: int,
) -> tuple[int, int, int, int, int, int, int, int]:
    timeline = Timeline(capture, source)
    temp = output.with_name(f".{output.name}.tmp.{os.getpid()}")

    if temp.exists():
        temp.unlink()

    chunk_bytes = chunk_mib * 1024 * 1024
    tick_count = 0
    layer_count = 0
    raw_scan_count = 0
    pointcloud_count = 0
    robot_scene_count = 0
    pose_message_count = 0
    costmap_count = 0

    pose_samples = extract_pose_samples(capture)
    origin = pose_samples[0] if pose_samples else None
    track_width_m, track_width_source = track_width_from_capture(capture)

    try:
        with temp.open("wb") as stream:
            writer = IndexedMcapWriter(stream, chunk_bytes)
            writer.start()

            pointcloud_schema_id = writer.add_schema(
                "foxglove.PointCloud",
                "jsonschema",
                compact_json(POINTCLOUD_JSON_SCHEMA),
            )
            pose_schema_id = writer.add_schema(
                "foxglove.PoseInFrame",
                "jsonschema",
                compact_json(POSE_IN_FRAME_JSON_SCHEMA),
            )
            scene_schema_id = writer.add_schema(
                "foxglove.SceneUpdate",
                "jsonschema",
                compact_json(SCENE_UPDATE_JSON_SCHEMA),
            )

            meta_channel = writer.add_channel(
                "/r2b4/capture/meta", "json"
            )
            viz_meta_channel = writer.add_channel(
                "/r2b4/visualization/meta", "json"
            )
            tick_channel = writer.add_channel(
                "/r2b4/ticks", "json"
            )
            inputs_channel = writer.add_channel(
                "/r2b4/inputs", "json"
            )
            raw_lidar_channel = writer.add_channel(
                "/r2b4/raw_lidar", "json"
            )
            pointcloud_channel = writer.add_channel(
                "/r2b4/lidar/pointcloud",
                "json",
                schema_id=pointcloud_schema_id,
            )
            costmap_channel = writer.add_channel(
                "/r2b4/costmap/occupied",
                "json",
                schema_id=pointcloud_schema_id,
            )
            pose_channel = writer.add_channel(
                "/r2b4/pose",
                "json",
                schema_id=pose_schema_id,
            )
            robot_scene_channel = writer.add_channel(
                "/r2b4/scene/robot",
                "json",
                schema_id=scene_schema_id,
            )
            path_scene_channel = writer.add_channel(
                "/r2b4/scene/path",
                "json",
                schema_id=scene_schema_id,
            )

            layer_channels: dict[str, int] = {}
            for layer_name in layer_names_from_capture(capture):
                layer_channels[layer_name] = writer.add_channel(
                    f"/r2b4/layers/{layer_name}",
                    "json",
                )

            events: list[Event] = []
            order = 0

            meta = {
                key: value
                for key, value in capture.items()
                if key not in ("ticks", "raw_lidar_scans")
            }
            events.append(
                Event(
                    timeline.wall_ns(timeline.min_mono),
                    order,
                    meta_channel,
                    compact_json(meta),
                )
            )
            order += 1

            viz_meta: dict[str, Any] = {
                "frame_id": VIZ_FRAME_ID,
                "normalization": "capture_start_pose_is_origin",
                "track_width_m": track_width_m,
                "track_width_source": track_width_source,
                "lidar_extrinsic": "not_found_in_capture; visualization assumes lidar origin equals L3 robot origin",
                "robot_symbol": "track-width-derived marker, not CAD geometry",
            }
            if origin is not None:
                viz_meta["source_frame_id"] = origin.frame_id
                viz_meta["source_origin"] = {
                    "x_m": origin.x_m,
                    "y_m": origin.y_m,
                    "yaw_rad": origin.yaw_rad,
                    "tick_id": origin.tick_id,
                }
            events.append(
                Event(
                    timeline.wall_ns(timeline.min_mono),
                    order,
                    viz_meta_channel,
                    compact_json(viz_meta),
                )
            )
            order += 1

            path_points: list[tuple[float, float]] = []
            last_costmap_revision: int | None = None

            ticks = capture.get("ticks")
            if isinstance(ticks, list):
                for tick in ticks:
                    if not isinstance(tick, Mapping):
                        continue

                    mono = nonnegative_int(tick.get("monotonic_ns"))
                    wall = timeline.wall_ns(mono)

                    events.append(
                        Event(wall, order, tick_channel, compact_json(tick))
                    )
                    order += 1
                    tick_count += 1

                    inputs = tick.get("inputs")
                    if isinstance(inputs, Mapping):
                        events.append(
                            Event(
                                wall,
                                order,
                                inputs_channel,
                                compact_json(
                                    {
                                        "tick_id": tick.get("tick_id"),
                                        "monotonic_ns": mono,
                                        "inputs": inputs,
                                    }
                                ),
                            )
                        )
                        order += 1

                    expected = tick.get("expected")
                    layers: Mapping[str, Any] | None = None
                    if isinstance(expected, Mapping):
                        raw_layers = expected.get("layers")
                        if isinstance(raw_layers, Mapping):
                            layers = raw_layers
                            for layer_name, layer_output in raw_layers.items():
                                channel = layer_channels.get(str(layer_name))
                                if channel is None:
                                    continue
                                events.append(
                                    Event(
                                        wall,
                                        order,
                                        channel,
                                        compact_json(
                                            {
                                                "tick_id": tick.get("tick_id"),
                                                "monotonic_ns": mono,
                                                "output": layer_output,
                                            }
                                        ),
                                    )
                                )
                                order += 1
                                layer_count += 1

                    if origin is not None and mono is not None:
                        sample = nearest_pose_sample(pose_samples, mono)
                        if sample is not None and sample.monotonic_ns == mono:
                            events.append(
                                Event(
                                    wall,
                                    order,
                                    pose_channel,
                                    compact_json(
                                        pose_in_frame_message(sample, origin, wall)
                                    ),
                                )
                            )
                            order += 1
                            pose_message_count += 1

                            events.append(
                                Event(
                                    wall,
                                    order,
                                    robot_scene_channel,
                                    compact_json(
                                        robot_scene_message(
                                            sample,
                                            origin,
                                            wall,
                                            track_width_m,
                                        )
                                    ),
                                )
                            )
                            order += 1
                            robot_scene_count += 1

                            x_m, y_m, _ = normalized_pose(sample, origin)
                            path_points.append((x_m, y_m))
                            events.append(
                                Event(
                                    wall,
                                    order,
                                    path_scene_channel,
                                    compact_json(
                                        path_scene_message(path_points, wall)
                                    ),
                                )
                            )
                            order += 1

                        if layers is not None:
                            l4 = layers.get("L4")
                            if isinstance(l4, Mapping):
                                costmap_cloud, revision = costmap_to_pointcloud(
                                    l4,
                                    wall,
                                    origin,
                                )
                                if (
                                    costmap_cloud is not None
                                    and revision != last_costmap_revision
                                ):
                                    events.append(
                                        Event(
                                            wall,
                                            order,
                                            costmap_channel,
                                            compact_json(costmap_cloud),
                                        )
                                    )
                                    order += 1
                                    costmap_count += 1
                                    last_costmap_revision = revision

            scans = capture.get("raw_lidar_scans")
            if isinstance(scans, list):
                for scan in scans:
                    if not isinstance(scan, Mapping):
                        continue

                    mono = scan_monotonic_ns(scan)
                    wall = timeline.wall_ns(mono)

                    events.append(
                        Event(
                            wall,
                            order,
                            raw_lidar_channel,
                            compact_json(scan),
                        )
                    )
                    order += 1
                    raw_scan_count += 1

                    pose_sample = nearest_pose_sample(pose_samples, mono)
                    cloud = lidar_to_pointcloud(
                        scan,
                        wall,
                        pose_sample=pose_sample,
                        origin=origin,
                    )
                    if cloud is not None:
                        events.append(
                            Event(
                                wall,
                                order,
                                pointcloud_channel,
                                compact_json(cloud),
                            )
                        )
                        order += 1
                        pointcloud_count += 1

            # Physical message order follows log time, which gives Foxglove
            # efficient forward playback in addition to random indexed access.
            events.sort(key=lambda event: (event.log_time_ns, event.order))

            for event in events:
                writer.add_message(
                    event.channel_id,
                    event.log_time_ns,
                    event.log_time_ns,
                    event.data,
                )

            # Preserve exact original capture bytes as an indexed attachment.
            attachment_time = timeline.wall_ns(timeline.max_mono)
            writer.add_attachment(
                log_time_ns=attachment_time,
                create_time_ns=attachment_time,
                name=source.name,
                media_type="application/json",
                data=source_bytes,
            )

            writer.finish()
            chunk_count = len(writer.chunk_indexes)

        os.replace(temp, output)
        try:
            output.chmod(0o600)
        except OSError:
            pass

    except Exception:
        try:
            temp.unlink()
        except OSError:
            pass
        raise

    return (
        tick_count,
        layer_count,
        raw_scan_count,
        pointcloud_count,
        robot_scene_count,
        pose_message_count,
        costmap_count,
        chunk_count,
    )


# ---------------------------------------------------------------------------
# Dependency-free structural validation
# ---------------------------------------------------------------------------

def iter_records(data: bytes) -> list[tuple[int, int, int, bytes]]:
    if not data.startswith(MAGIC) or not data.endswith(MAGIC):
        raise ValueError("invalid MCAP magic bytes")

    pos = len(MAGIC)
    end = len(data) - len(MAGIC)
    result: list[tuple[int, int, int, bytes]] = []

    while pos < end:
        if pos + 9 > end:
            raise ValueError("truncated record header")
        opcode = data[pos]
        content_len = struct.unpack_from("<Q", data, pos + 1)[0]
        content_start = pos + 9
        next_pos = content_start + content_len
        if next_pos > end:
            raise ValueError("truncated record")
        result.append(
            (opcode, pos, 9 + content_len, data[content_start:next_pos])
        )
        pos = next_pos

    if pos != end:
        raise ValueError("invalid trailing MCAP layout")
    return result


def validate_indexed_mcap(path: Path) -> dict[str, int]:
    data = path.read_bytes()
    records = iter_records(data)

    if not records or records[0][0] != OP_HEADER:
        raise ValueError("first record is not Header")
    if records[-1][0] != OP_FOOTER:
        raise ValueError("last record is not Footer")

    counts: dict[int, int] = defaultdict(int)
    for opcode, _, _, _ in records:
        counts[opcode] += 1

    footer = records[-1][3]
    if len(footer) != 20:
        raise ValueError("invalid Footer length")

    summary_start, summary_offset_start, _ = struct.unpack("<QQI", footer)
    if summary_start == 0:
        raise ValueError("summary_start is zero: file is unindexed")
    if summary_offset_start == 0:
        raise ValueError("summary_offset_start is zero")

    if counts[OP_CHUNK] == 0:
        raise ValueError("no Chunk records")
    if counts[OP_MESSAGE_INDEX] == 0:
        raise ValueError("no Message Index records")
    if counts[OP_CHUNK_INDEX] == 0:
        raise ValueError("no Chunk Index records")
    if counts[OP_SUMMARY_OFFSET] == 0:
        raise ValueError("no Summary Offset records")
    if counts[OP_STATISTICS] != 1:
        raise ValueError("Statistics record missing or duplicated")
    if counts[OP_ATTACHMENT_INDEX] == 0:
        raise ValueError("Attachment Index missing")

    return {
        "chunks": counts[OP_CHUNK],
        "message_indexes": counts[OP_MESSAGE_INDEX],
        "chunk_indexes": counts[OP_CHUNK_INDEX],
        "summary_offsets": counts[OP_SUMMARY_OFFSET],
        "attachment_indexes": counts[OP_ATTACHMENT_INDEX],
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Convert the newest R2B4 V3 capture JSON to an indexed "
            "Foxglove-compatible MCAP without external packages."
        )
    )
    parser.add_argument(
        "--capture-dir",
        type=Path,
        default=DEFAULT_CAPTURE_DIR,
        help=f"capture directory (default: {DEFAULT_CAPTURE_DIR})",
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=None,
        help="convert this capture instead of auto-selecting the newest one",
    )
    parser.add_argument(
        "--chunk-mib",
        type=int,
        default=DEFAULT_CHUNK_MIB,
        help=f"uncompressed target chunk size in MiB (default: {DEFAULT_CHUNK_MIB})",
    )
    args = parser.parse_args()

    if not (1 <= args.chunk_mib <= 256):
        fail("--chunk-mib must be between 1 and 256")

    source = (
        args.input.expanduser().resolve()
        if args.input is not None
        else find_latest_capture(args.capture_dir.expanduser().resolve())
    )

    if not source.is_file():
        fail(f"capture does not exist: {source}")
    if source.suffix.lower() != ".json":
        fail(f"input is not a JSON file: {source}")

    output = source.with_suffix(".mcap")

    print(f"R2B4 source : {source}")
    print(f"Foxglove    : {output}")
    print("Reading complete V3 capture...")

    try:
        source_bytes = source.read_bytes()
    except OSError as exc:
        fail(f"cannot read source capture: {exc}")

    try:
        capture = json.loads(source_bytes)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        fail(f"invalid UTF-8 JSON capture: {exc}")

    if not isinstance(capture, dict):
        fail("capture JSON root is not an object")

    schema = capture.get("schema")
    if not isinstance(schema, str) or "V3" not in schema.upper():
        print(
            f"WARNING: unexpected V3 schema value: {schema!r}",
            file=sys.stderr,
        )

    print(f"Writing indexed MCAP ({args.chunk_mib} MiB chunks)...")

    try:
        (
            ticks,
            layers,
            raw_scans,
            pointclouds,
            robot_scenes,
            poses,
            costmaps,
            chunks,
        ) = export_capture(
            source,
            output,
            capture,
            source_bytes,
            args.chunk_mib,
        )
        index_info = validate_indexed_mcap(output)
    except Exception as exc:
        fail(f"conversion failed: {type(exc).__name__}: {exc}")

    print()
    print("DONE - INDEXED MCAP")
    print(f"  source JSON     : {source.name}")
    print(f"  output MCAP     : {output.name}")
    print(f"  ticks           : {ticks}")
    print(f"  layer messages  : {layers}")
    print(f"  raw LiDAR       : {raw_scans}")
    print(f"  LiDAR clouds    : {pointclouds}")
    print(f"  robot scenes    : {robot_scenes}")
    print(f"  pose messages   : {poses}")
    print(f"  costmap clouds  : {costmaps}")
    print(f"  chunks          : {chunks}")
    print(f"  message indexes : {index_info['message_indexes']}")
    print(f"  chunk indexes   : {index_info['chunk_indexes']}")
    print(f"  summary offsets : {index_info['summary_offsets']}")
    print(f"  source bytes    : {len(source_bytes)}")
    print(f"  MCAP bytes      : {output.stat().st_size}")
    print()
    print("Foxglove topics:")
    print("  /r2b4/ticks")
    print("  /r2b4/inputs")
    print("  /r2b4/layers/L1 ... /r2b4/layers/L12")
    print("  /r2b4/raw_lidar")
    print("  /r2b4/pose")
    print("  /r2b4/scene/robot")
    print("  /r2b4/scene/path")
    print("  /r2b4/lidar/pointcloud")
    print("  /r2b4/costmap/occupied")
    print(f"  fixed frame: {VIZ_FRAME_ID}")
    print()
    print(
        "The complete original V3 JSON is embedded byte-for-byte "
        "as an indexed MCAP attachment."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
