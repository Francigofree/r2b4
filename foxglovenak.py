#!/usr/bin/env python3
"""Convert an R2B4 V3 MCAP capture to a Foxglove-native visualization MCAP.

Zero external dependencies: Python standard library only.

Input expected:
  R2B4_MCAP_CAPTURE_V1 written by v3.mcap_writer.StdlibMcapWriter
  with uncompressed MCAP chunks and JSON topics such as:
    /r2b4/tick
    /r2b4/raw_lidar

Output topics:
  /r2b4/foxglove/robot_pose   foxglove.PosesInFrame (one pose per tick)
  /r2b4/foxglove/full_path    foxglove.PosesInFrame (full recorded path)
  /r2b4/foxglove/lidar        foxglove.PointCloud

By default the first L3 pose becomes (0, 0, yaw=0) in frame
R2B4_CAPTURE_MAP, which makes playback easier to understand in Foxglove.
The source MCAP is never modified.

Usage:
  python3 r2b4_mcap_to_foxglove.py capture.mcap
  python3 r2b4_mcap_to_foxglove.py capture.mcap -o capture_foxglove.mcap
  python3 r2b4_mcap_to_foxglove.py capture.mcap --keep-original
  python3 r2b4_mcap_to_foxglove.py capture.mcap --no-normalize
"""

from __future__ import annotations

import argparse
import base64
from bisect import bisect_left
from dataclasses import dataclass
import json
import math
from pathlib import Path
import struct
import sys
import zlib
from typing import Dict, Iterable, Iterator, Mapping, Optional, Sequence, Tuple

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

POSE_TOPIC = "/r2b4/foxglove/robot_pose"
PATH_TOPIC = "/r2b4/foxglove/full_path"
LIDAR_TOPIC = "/r2b4/foxglove/lidar"

NORMALIZED_FRAME = "R2B4_CAPTURE_MAP"
FOXGLOVE_FLOAT32 = 7


# ---------------------------------------------------------------------------
# Foxglove JSON schemas
# ---------------------------------------------------------------------------

TIMESTAMP_SCHEMA = {
    "type": "object",
    "properties": {
        "sec": {"type": "integer", "minimum": 0},
        "nsec": {"type": "integer", "minimum": 0, "maximum": 999999999},
    },
    "required": ["sec", "nsec"],
}

VECTOR3_SCHEMA = {
    "type": "object",
    "properties": {
        "x": {"type": "number"},
        "y": {"type": "number"},
        "z": {"type": "number"},
    },
    "required": ["x", "y", "z"],
}

QUATERNION_SCHEMA = {
    "type": "object",
    "properties": {
        "x": {"type": "number"},
        "y": {"type": "number"},
        "z": {"type": "number"},
        "w": {"type": "number"},
    },
    "required": ["x", "y", "z", "w"],
}

POSE_SCHEMA = {
    "title": "foxglove.Pose",
    "type": "object",
    "properties": {
        "position": VECTOR3_SCHEMA,
        "orientation": QUATERNION_SCHEMA,
    },
    "required": ["position", "orientation"],
}

POSES_IN_FRAME_SCHEMA = {
    "title": "foxglove.PosesInFrame",
    "description": "An array of timestamped poses for an object or reference frame in 3D space",
    "type": "object",
    "properties": {
        "timestamp": TIMESTAMP_SCHEMA,
        "frame_id": {"type": "string"},
        "poses": {
            "type": "array",
            "items": POSE_SCHEMA,
        },
    },
    "required": ["timestamp", "frame_id", "poses"],
}

POINTCLOUD_SCHEMA = {
    "title": "foxglove.PointCloud",
    "description": "A collection of N-dimensional points",
    "type": "object",
    "properties": {
        "timestamp": TIMESTAMP_SCHEMA,
        "frame_id": {"type": "string"},
        "pose": POSE_SCHEMA,
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
    "required": ["timestamp", "frame_id", "pose", "point_stride", "fields", "data"],
}


# ---------------------------------------------------------------------------
# Binary helpers
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


def record(opcode: int, content: bytes) -> bytes:
    return u8(opcode) + u64(len(content)) + content


class Cursor:
    def __init__(self, data: bytes):
        self.data = data
        self.pos = 0

    def take(self, count: int) -> bytes:
        if count < 0 or self.pos + count > len(self.data):
            raise ValueError("truncated MCAP record")
        out = self.data[self.pos:self.pos + count]
        self.pos += count
        return out

    def u16(self) -> int:
        return struct.unpack("<H", self.take(2))[0]

    def u32(self) -> int:
        return struct.unpack("<I", self.take(4))[0]

    def u64(self) -> int:
        return struct.unpack("<Q", self.take(8))[0]

    def string(self) -> str:
        n = self.u32()
        return self.take(n).decode("utf-8")

    def bytes_u64(self) -> bytes:
        return self.take(self.u64())

    def remaining(self) -> bytes:
        return self.take(len(self.data) - self.pos)


# ---------------------------------------------------------------------------
# Source MCAP reader (R2B4 subset)
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class SourceChannel:
    channel_id: int
    schema_id: int
    topic: str
    message_encoding: str


@dataclass(frozen=True)
class SourceMessage:
    topic: str
    channel_id: int
    sequence: int
    log_time_ns: int
    publish_time_ns: int
    data: bytes


def iter_records_bytes(data: bytes) -> Iterator[Tuple[int, bytes]]:
    pos = 0
    size = len(data)
    while pos < size:
        if pos + 9 > size:
            raise ValueError("truncated nested MCAP record header")
        opcode = data[pos]
        length = struct.unpack_from("<Q", data, pos + 1)[0]
        start = pos + 9
        end = start + length
        if end > size:
            raise ValueError("truncated nested MCAP record")
        yield opcode, data[start:end]
        pos = end
    if pos != size:
        raise ValueError("invalid nested MCAP record boundary")


class R2B4McapReader:
    def __init__(self, path: Path):
        self.path = Path(path)
        if not self.path.is_file():
            raise FileNotFoundError(self.path)
        self.channels = self._read_channels()

    def _read_channels(self) -> Dict[int, SourceChannel]:
        channels: Dict[int, SourceChannel] = {}
        with self.path.open("rb") as f:
            if f.read(len(MAGIC)) != MAGIC:
                raise ValueError("input is not an MCAP file")
            while True:
                header = f.read(9)
                if not header:
                    raise ValueError("MCAP ended before DataEnd")
                if len(header) != 9:
                    raise ValueError("truncated MCAP record header")
                opcode = header[0]
                length = struct.unpack("<Q", header[1:])[0]
                content = f.read(length)
                if len(content) != length:
                    raise ValueError("truncated MCAP record")
                if opcode == OP_CHANNEL:
                    cur = Cursor(content)
                    channel_id = cur.u16()
                    schema_id = cur.u16()
                    topic = cur.string()
                    message_encoding = cur.string()
                    channels[channel_id] = SourceChannel(
                        channel_id, schema_id, topic, message_encoding
                    )
                elif opcode in (OP_CHUNK, OP_MESSAGE):
                    # R2B4 channels are registered before the first message/chunk.
                    if channels:
                        return channels
                elif opcode == OP_DATA_END:
                    return channels

    def _parse_message(self, content: bytes) -> SourceMessage:
        cur = Cursor(content)
        channel_id = cur.u16()
        sequence = cur.u32()
        log_time_ns = cur.u64()
        publish_time_ns = cur.u64()
        data = cur.remaining()
        channel = self.channels.get(channel_id)
        if channel is None:
            raise ValueError(f"message references unknown channel {channel_id}")
        return SourceMessage(
            topic=channel.topic,
            channel_id=channel_id,
            sequence=sequence,
            log_time_ns=log_time_ns,
            publish_time_ns=publish_time_ns,
            data=data,
        )

    def iter_messages(self) -> Iterator[SourceMessage]:
        with self.path.open("rb") as f:
            if f.read(len(MAGIC)) != MAGIC:
                raise ValueError("input is not an MCAP file")
            while True:
                header = f.read(9)
                if not header:
                    raise ValueError("MCAP ended before DataEnd")
                if len(header) != 9:
                    raise ValueError("truncated MCAP record header")
                opcode = header[0]
                length = struct.unpack("<Q", header[1:])[0]
                content = f.read(length)
                if len(content) != length:
                    raise ValueError("truncated MCAP record")

                if opcode == OP_MESSAGE:
                    yield self._parse_message(content)
                    continue

                if opcode == OP_CHUNK:
                    cur = Cursor(content)
                    _start = cur.u64()
                    _end = cur.u64()
                    uncompressed_size = cur.u64()
                    uncompressed_crc = cur.u32()
                    compression = cur.string()
                    records_blob = cur.bytes_u64()
                    if compression:
                        raise ValueError(
                            f"unsupported compressed source chunk: {compression!r}; "
                            "R2B4 stdlib captures are expected to be uncompressed"
                        )
                    if len(records_blob) != uncompressed_size:
                        raise ValueError("source chunk uncompressed_size mismatch")
                    if uncompressed_crc:
                        actual = zlib.crc32(records_blob) & 0xFFFFFFFF
                        if actual != uncompressed_crc:
                            raise ValueError("source chunk CRC mismatch")
                    for inner_opcode, inner_content in iter_records_bytes(records_blob):
                        if inner_opcode == OP_MESSAGE:
                            yield self._parse_message(inner_content)
                    continue

                if opcode == OP_DATA_END:
                    return


# ---------------------------------------------------------------------------
# Foxglove MCAP writer (schema-aware, no external package)
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class SchemaDef:
    schema_id: int
    name: str
    encoding: str
    data: bytes
    record_bytes: bytes


@dataclass(frozen=True)
class ChannelDef:
    channel_id: int
    schema_id: int
    topic: str
    message_encoding: str
    record_bytes: bytes


class FoxgloveMcapWriter:
    """Small valid MCAP v0 writer with JSON Schema support.

    Messages are written directly in the data section (no chunking). A summary
    section with schemas, channels and statistics is written for fast discovery.
    """

    def __init__(self, path: Path):
        self.path = Path(path)
        self.f = self.path.open("wb")
        self.schemas: Dict[int, SchemaDef] = {}
        self.channels: Dict[int, ChannelDef] = {}
        self.next_schema_id = 1
        self.next_channel_id = 1
        self.next_sequence: Dict[int, int] = {}
        self.message_count = 0
        self.channel_counts: Dict[int, int] = {}
        self.start_time: Optional[int] = None
        self.end_time: Optional[int] = None
        self.finished = False

        self.f.write(MAGIC)
        self.f.write(record(OP_HEADER, mcap_string("") + mcap_string("R2B4 Foxglove stdlib exporter/1")))

    def register_schema(self, name: str, schema_obj: Mapping[str, object]) -> int:
        schema_id = self.next_schema_id
        self.next_schema_id += 1
        data = json.dumps(schema_obj, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
        rec = record(
            OP_SCHEMA,
            u16(schema_id)
            + mcap_string(name)
            + mcap_string("jsonschema")
            + bytes_u32(data),
        )
        self.f.write(rec)
        self.schemas[schema_id] = SchemaDef(schema_id, name, "jsonschema", data, rec)
        return schema_id

    def register_channel(self, topic: str, schema_id: int, message_encoding: str = "json") -> int:
        if schema_id != 0 and schema_id not in self.schemas:
            raise ValueError("unknown schema_id")
        channel_id = self.next_channel_id
        self.next_channel_id += 1
        rec = record(
            OP_CHANNEL,
            u16(channel_id)
            + u16(schema_id)
            + mcap_string(topic)
            + mcap_string(message_encoding)
            + string_map({}),
        )
        self.f.write(rec)
        self.channels[channel_id] = ChannelDef(
            channel_id, schema_id, topic, message_encoding, rec
        )
        self.next_sequence[channel_id] = 0
        self.channel_counts[channel_id] = 0
        return channel_id

    def add_message(
        self,
        channel_id: int,
        log_time_ns: int,
        data: bytes,
        *,
        publish_time_ns: Optional[int] = None,
        sequence: Optional[int] = None,
    ) -> None:
        if channel_id not in self.channels:
            raise ValueError("unknown channel_id")
        if publish_time_ns is None:
            publish_time_ns = log_time_ns
        if sequence is None:
            sequence = self.next_sequence[channel_id]
            self.next_sequence[channel_id] = (sequence + 1) & 0xFFFFFFFF
        else:
            sequence &= 0xFFFFFFFF

        content = (
            u16(channel_id)
            + u32(sequence)
            + u64(log_time_ns)
            + u64(publish_time_ns)
            + data
        )
        self.f.write(record(OP_MESSAGE, content))
        self.message_count += 1
        self.channel_counts[channel_id] += 1
        if self.start_time is None or log_time_ns < self.start_time:
            self.start_time = log_time_ns
        if self.end_time is None or log_time_ns > self.end_time:
            self.end_time = log_time_ns

    def add_json(
        self,
        channel_id: int,
        log_time_ns: int,
        value: Mapping[str, object],
        *,
        publish_time_ns: Optional[int] = None,
    ) -> None:
        payload = json.dumps(
            value,
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        ).encode("utf-8")
        self.add_message(
            channel_id,
            log_time_ns,
            payload,
            publish_time_ns=publish_time_ns,
        )

    def _statistics_record(self) -> bytes:
        counts = u16_u64_map(self.channel_counts)
        content = (
            u64(self.message_count)
            + u16(len(self.schemas))
            + u32(len(self.channels))
            + u32(0)  # attachment_count
            + u32(0)  # metadata_count
            + u32(0)  # chunk_count
            + u64(self.start_time or 0)
            + u64(self.end_time or 0)
            + counts
        )
        return record(OP_STATISTICS, content)

    def finish(self) -> None:
        if self.finished:
            return

        # No data-section CRC; 0 is explicitly valid in MCAP.
        self.f.write(record(OP_DATA_END, u32(0)))

        summary_start = self.f.tell()
        groups = []

        schema_start = self.f.tell()
        schema_len = 0
        for schema_id in sorted(self.schemas):
            rec = self.schemas[schema_id].record_bytes
            self.f.write(rec)
            schema_len += len(rec)
        if schema_len:
            groups.append((OP_SCHEMA, schema_start, schema_len))

        channel_start = self.f.tell()
        channel_len = 0
        for channel_id in sorted(self.channels):
            rec = self.channels[channel_id].record_bytes
            self.f.write(rec)
            channel_len += len(rec)
        if channel_len:
            groups.append((OP_CHANNEL, channel_start, channel_len))

        stats_start = self.f.tell()
        stats = self._statistics_record()
        self.f.write(stats)
        groups.append((OP_STATISTICS, stats_start, len(stats)))

        summary_offset_start = self.f.tell()
        for opcode, group_start, group_length in groups:
            self.f.write(
                record(
                    OP_SUMMARY_OFFSET,
                    u8(opcode) + u64(group_start) + u64(group_length),
                )
            )

        # summary_crc=0 is valid and means "not available".
        self.f.write(
            record(
                OP_FOOTER,
                u64(summary_start) + u64(summary_offset_start) + u32(0),
            )
        )
        self.f.write(MAGIC)
        self.f.flush()
        self.f.close()
        self.finished = True

    def abort(self) -> None:
        if not self.f.closed:
            self.f.close()


# ---------------------------------------------------------------------------
# R2B4 -> Foxglove conversion
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class PoseSample:
    message_log_time_ns: int
    pose_time_ns: int
    frame_id: str
    x: float
    y: float
    yaw: float


def finite_number(value: object) -> Optional[float]:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    out = float(value)
    return out if math.isfinite(out) else None


def parse_json(data: bytes) -> Optional[dict]:
    try:
        value = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def extract_l3_pose(message: SourceMessage) -> Optional[PoseSample]:
    payload = parse_json(message.data)
    if payload is None:
        return None
    expected = payload.get("expected")
    if not isinstance(expected, dict):
        return None
    layers = expected.get("layers")
    if not isinstance(layers, dict):
        return None
    l3 = layers.get("L3")
    if not isinstance(l3, dict):
        return None

    x = finite_number(l3.get("x_m"))
    y = finite_number(l3.get("y_m"))
    yaw = finite_number(l3.get("yaw_rad"))
    if x is None or y is None or yaw is None:
        return None

    frame_id = l3.get("frame_id")
    if not isinstance(frame_id, str) or not frame_id:
        frame_id = "R2B4_BOOT_ROBOT_MAP"

    pose_time = message.log_time_ns
    context = l3.get("context")
    if isinstance(context, dict):
        raw_time = context.get("monotonic_ns")
        if isinstance(raw_time, int) and not isinstance(raw_time, bool) and raw_time >= 0:
            pose_time = raw_time

    return PoseSample(
        message_log_time_ns=message.log_time_ns,
        pose_time_ns=pose_time,
        frame_id=frame_id,
        x=x,
        y=y,
        yaw=yaw,
    )


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def normalize_pose(sample: PoseSample, origin: PoseSample, enabled: bool) -> PoseSample:
    if not enabled:
        return sample
    dx = sample.x - origin.x
    dy = sample.y - origin.y
    c = math.cos(origin.yaw)
    s = math.sin(origin.yaw)
    # Rotate world delta by -origin.yaw.
    x = c * dx + s * dy
    y = -s * dx + c * dy
    yaw = wrap_angle(sample.yaw - origin.yaw)
    return PoseSample(
        message_log_time_ns=sample.message_log_time_ns,
        pose_time_ns=sample.pose_time_ns,
        frame_id=NORMALIZED_FRAME,
        x=x,
        y=y,
        yaw=yaw,
    )


def quaternion_from_yaw(yaw: float) -> dict:
    half = yaw * 0.5
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(half),
        "w": math.cos(half),
    }


def foxglove_pose(sample: PoseSample) -> dict:
    return {
        "position": {"x": sample.x, "y": sample.y, "z": 0.0},
        "orientation": quaternion_from_yaw(sample.yaw),
    }


def timestamp(ns: int) -> dict:
    return {"sec": ns // 1_000_000_000, "nsec": ns % 1_000_000_000}


def nearest_pose(samples: Sequence[PoseSample], times: Sequence[int], query_ns: int) -> PoseSample:
    if not samples:
        raise ValueError("no pose samples")
    i = bisect_left(times, query_ns)
    if i <= 0:
        return samples[0]
    if i >= len(samples):
        return samples[-1]
    before = samples[i - 1]
    after = samples[i]
    if abs(query_ns - before.pose_time_ns) <= abs(after.pose_time_ns - query_ns):
        return before
    return after


def downsample(samples: Sequence[PoseSample], max_points: int) -> Sequence[PoseSample]:
    if len(samples) <= max_points:
        return samples
    if max_points < 2:
        return [samples[0]]
    out = []
    last_index = len(samples) - 1
    for i in range(max_points):
        idx = round(i * last_index / (max_points - 1))
        out.append(samples[idx])
    return out


def path_length(samples: Sequence[PoseSample]) -> float:
    total = 0.0
    for a, b in zip(samples, samples[1:]):
        total += math.hypot(b.x - a.x, b.y - a.y)
    return total


def make_pointcloud_payload(
    raw: dict,
    pose: PoseSample,
    measurement_ns: int,
) -> Optional[dict]:
    points = raw.get("points")
    if not isinstance(points, list):
        return None

    packed = bytearray()
    valid_count = 0
    for item in points:
        if not isinstance(item, list) or len(item) < 2:
            continue
        angle_deg = finite_number(item[0])
        distance_m = finite_number(item[1])
        quality = finite_number(item[2]) if len(item) >= 3 else 0.0
        if angle_deg is None or distance_m is None or quality is None:
            continue
        if distance_m <= 0.0:
            continue

        angle = math.radians(angle_deg)
        # RPLIDAR C1 angle is clockwise-positive; R2B4 +Y is left.
        x = distance_m * math.cos(angle)
        y = -distance_m * math.sin(angle)
        packed += struct.pack("<ffff", x, y, 0.0, quality)
        valid_count += 1

    if valid_count == 0:
        return None

    return {
        "timestamp": timestamp(measurement_ns),
        "frame_id": pose.frame_id,
        "pose": foxglove_pose(pose),
        "point_stride": 16,
        "fields": [
            {"name": "x", "offset": 0, "type": FOXGLOVE_FLOAT32},
            {"name": "y", "offset": 4, "type": FOXGLOVE_FLOAT32},
            {"name": "z", "offset": 8, "type": FOXGLOVE_FLOAT32},
            {"name": "intensity", "offset": 12, "type": FOXGLOVE_FLOAT32},
        ],
        "data": base64.b64encode(bytes(packed)).decode("ascii"),
    }


@dataclass(frozen=True)
class OutputEvent:
    log_time_ns: int
    order: int
    channel_id: int
    publish_time_ns: int
    data: bytes
    sequence: Optional[int] = None


def json_bytes(value: Mapping[str, object]) -> bytes:
    return json.dumps(
        value,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


def collect_poses(reader: R2B4McapReader) -> list[PoseSample]:
    samples = []
    for message in reader.iter_messages():
        if message.topic != TICK_TOPIC:
            continue
        pose = extract_l3_pose(message)
        if pose is not None:
            samples.append(pose)
    samples.sort(key=lambda p: (p.pose_time_ns, p.message_log_time_ns))
    return samples


def convert(
    input_path: Path,
    output_path: Path,
    *,
    normalize: bool,
    keep_original: bool,
    max_path_points: int,
) -> dict:
    reader = R2B4McapReader(input_path)
    raw_poses = collect_poses(reader)
    if not raw_poses:
        raise ValueError("no expected.layers.L3 RobotEstimate poses found in /r2b4/tick")

    origin = raw_poses[0]
    poses = [normalize_pose(p, origin, normalize) for p in raw_poses]
    pose_times = [p.pose_time_ns for p in poses]
    frame_id = poses[0].frame_id

    writer = FoxgloveMcapWriter(output_path)
    try:
        poses_schema_id = writer.register_schema("foxglove.PosesInFrame", POSES_IN_FRAME_SCHEMA)
        pointcloud_schema_id = writer.register_schema("foxglove.PointCloud", POINTCLOUD_SCHEMA)

        pose_channel = writer.register_channel(POSE_TOPIC, poses_schema_id, "json")
        path_channel = writer.register_channel(PATH_TOPIC, poses_schema_id, "json")
        lidar_channel = writer.register_channel(LIDAR_TOPIC, pointcloud_schema_id, "json")

        original_channels: Dict[int, int] = {}
        if keep_original:
            for source_id, source_channel in sorted(reader.channels.items()):
                original_channels[source_id] = writer.register_channel(
                    source_channel.topic,
                    0,
                    source_channel.message_encoding,
                )

        path_samples = list(downsample(poses, max_path_points))
        events: list[OutputEvent] = []
        order = 0

        # Full offline trajectory: one static path message at the first L3 pose.
        events.append(
            OutputEvent(
                log_time_ns=poses[0].message_log_time_ns,
                order=order,
                channel_id=path_channel,
                publish_time_ns=poses[0].message_log_time_ns,
                data=json_bytes(
                    {
                        "timestamp": timestamp(poses[0].pose_time_ns),
                        "frame_id": frame_id,
                        "poses": [foxglove_pose(p) for p in path_samples],
                    }
                ),
            )
        )
        order += 1

        pose_by_log_time = {p.message_log_time_ns: p for p in poses}
        lidar_scans = 0
        lidar_skipped = 0
        pose_messages = 0

        for message in reader.iter_messages():
            if keep_original:
                out_channel = original_channels.get(message.channel_id)
                if out_channel is not None:
                    events.append(
                        OutputEvent(
                            log_time_ns=message.log_time_ns,
                            order=order,
                            channel_id=out_channel,
                            publish_time_ns=message.publish_time_ns,
                            data=message.data,
                            sequence=message.sequence,
                        )
                    )
                    order += 1

            if message.topic == TICK_TOPIC:
                pose = pose_by_log_time.get(message.log_time_ns)
                if pose is None:
                    raw_pose = extract_l3_pose(message)
                    if raw_pose is None:
                        continue
                    pose = normalize_pose(raw_pose, origin, normalize)
                events.append(
                    OutputEvent(
                        log_time_ns=message.log_time_ns,
                        order=order,
                        channel_id=pose_channel,
                        publish_time_ns=message.publish_time_ns,
                        data=json_bytes(
                            {
                                "timestamp": timestamp(pose.pose_time_ns),
                                "frame_id": pose.frame_id,
                                "poses": [foxglove_pose(pose)],
                            }
                        ),
                    )
                )
                order += 1
                pose_messages += 1
                continue

            if message.topic == RAW_LIDAR_TOPIC:
                raw = parse_json(message.data)
                if raw is None:
                    lidar_skipped += 1
                    continue
                if raw.get("point_encoding") != "ANGLE_DEG_DISTANCE_M_QUALITY":
                    lidar_skipped += 1
                    continue
                measurement_ns = raw.get("measurement_monotonic_ns")
                if not isinstance(measurement_ns, int) or isinstance(measurement_ns, bool):
                    measurement_ns = message.log_time_ns
                pose = nearest_pose(poses, pose_times, measurement_ns)
                payload = make_pointcloud_payload(raw, pose, measurement_ns)
                if payload is None:
                    lidar_skipped += 1
                    continue
                events.append(
                    OutputEvent(
                        log_time_ns=message.log_time_ns,
                        order=order,
                        channel_id=lidar_channel,
                        publish_time_ns=message.publish_time_ns,
                        data=json_bytes(payload),
                    )
                )
                order += 1
                lidar_scans += 1

        # R2B4 source records are hub-order, not strictly log-time-order: raw
        # LiDAR can be published after a later control tick. Foxglove playback
        # is cleaner when the exported visualization stream is time ordered.
        events.sort(key=lambda e: (e.log_time_ns, e.order))
        for event in events:
            writer.add_message(
                event.channel_id,
                event.log_time_ns,
                event.data,
                publish_time_ns=event.publish_time_ns,
                sequence=event.sequence,
            )

        writer.finish()
    except Exception:
        writer.abort()
        try:
            output_path.unlink(missing_ok=True)
        except Exception:
            pass
        raise

    return {
        "poses": len(poses),
        "pose_messages": pose_messages,
        "lidar_scans": lidar_scans,
        "lidar_skipped": lidar_skipped,
        "path_points": len(path_samples),
        "path_length_m": path_length(poses),
        "frame_id": frame_id,
        "keep_original": keep_original,
        "output_bytes": output_path.stat().st_size,
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Convert R2B4 V3 MCAP to Foxglove-native pose/path/LiDAR MCAP."
    )
    parser.add_argument("input", type=Path, help="source R2B4 .mcap")
    parser.add_argument("-o", "--output", type=Path, default=None, help="output .mcap")
    parser.add_argument(
        "--keep-original",
        action="store_true",
        help="also copy the original R2B4 topics as schemaless channels",
    )
    parser.add_argument(
        "--no-normalize",
        action="store_true",
        help="keep original R2B4 map pose instead of starting at x=0,y=0,yaw=0",
    )
    parser.add_argument(
        "--max-path-points",
        type=int,
        default=5000,
        help="maximum poses in the static full-path topic (default: 5000)",
    )
    args = parser.parse_args(argv)

    input_path = args.input.expanduser().resolve()
    if input_path.suffix.lower() != ".mcap":
        parser.error("input must end in .mcap")
    if not input_path.is_file():
        parser.error(f"input not found: {input_path}")
    if args.max_path_points <= 0:
        parser.error("--max-path-points must be positive")

    if args.output is None:
        output_path = input_path.with_name(input_path.stem + "_foxglove.mcap")
    else:
        output_path = args.output.expanduser().resolve()
    if output_path.suffix.lower() != ".mcap":
        parser.error("output must end in .mcap")
    if output_path == input_path:
        parser.error("output must differ from input; source capture is never overwritten")

    try:
        result = convert(
            input_path,
            output_path,
            normalize=not args.no_normalize,
            keep_original=args.keep_original,
            max_path_points=args.max_path_points,
        )
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    mib = result["output_bytes"] / (1024 * 1024)
    print(f"OK: {output_path}")
    print(f"  frame: {result['frame_id']}")
    print(f"  pose messages: {result['pose_messages']}")
    print(f"  path points: {result['path_points']}")
    print(f"  LiDAR scans: {result['lidar_scans']} (skipped {result['lidar_skipped']})")
    print(f"  estimated recorded path length: {result['path_length_m']:.3f} m")
    print(f"  output size: {mib:.2f} MiB")
    print("Foxglove 3D: enable /r2b4/foxglove/robot_pose, /r2b4/foxglove/full_path, /r2b4/foxglove/lidar")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
