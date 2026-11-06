"""Bounded host spatial memory and explicit historical occupancy analysis.

The atlas worker reads its own verified assets. Its small result is strategic
knowledge; an atlas cell never establishes a transform into the running robot.
"""
from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

from .atlas_reference import AtlasReference, _asset_digest, _frame_identity
from .world_model import _freeze, _jsonable


@dataclass(frozen=True, slots=True)
class SpatialCoverage:
    map_id: str
    entity_id: str
    frame_id: str
    source_clock_epoch: str
    source_runtime_pid: int
    source_generation: int
    source_map_revision: int | str | None
    source_sequence: int | None
    source_revision: int | str | None
    last_visit_ns: int | None
    last_observation_ns: int
    confidence: float
    source: str
    lineage: object
    last_attempt_ns: int | None = None
    attempt_outcome: str | None = None
    cooldown_until_ns: int | None = None
    last_attempt_behavior_id: str | None = None
    last_attempt_command_id: str | None = None

    def to_jsonable(self):
        return {**{name: getattr(self, name) for name in self.__dataclass_fields__},
                "lineage": _jsonable(self.lineage), "freshness": "HISTORICAL_KNOWLEDGE",
                "motion_authority": False}

    @classmethod
    def from_jsonable(cls, value):
        if not isinstance(value, Mapping):
            raise ValueError("invalid spatial coverage")
        item = {key: value[key] for key in cls.__dataclass_fields__}
        for key in ("map_id", "entity_id", "frame_id", "source_clock_epoch", "source"):
            if not isinstance(item[key], str) or not 0 < len(item[key]) <= 256:
                raise ValueError("invalid spatial coverage identity")
        for key in ("source_runtime_pid", "source_generation", "last_observation_ns"):
            if type(item[key]) is not int or item[key] < 0:
                raise ValueError("invalid spatial coverage lineage")
        for key in ("last_visit_ns", "last_attempt_ns", "cooldown_until_ns"):
            if item[key] is not None and (type(item[key]) is not int or item[key] < 0):
                raise ValueError("invalid spatial coverage time")
        sequence = item["source_sequence"]
        if sequence is not None and (type(sequence) is not int or sequence < 0):
            raise ValueError("invalid spatial source sequence")
        for key in ("source_map_revision", "source_revision"):
            revision = item[key]
            if revision is not None and not (type(revision) is int and revision >= 0
                    or isinstance(revision, str) and 0 < len(revision) <= 256):
                raise ValueError("invalid spatial map revision")
        confidence = item["confidence"]
        if type(confidence) not in (int, float) or not math.isfinite(confidence) or not 0 <= confidence <= 1:
            raise ValueError("invalid spatial coverage confidence")
        outcome = item["attempt_outcome"]
        if outcome is not None and outcome not in {"ACTIVE", "COMPLETED", "FAILED", "CANCELLED"}:
            raise ValueError("invalid spatial attempt outcome")
        for key in ("last_attempt_behavior_id", "last_attempt_command_id"):
            if item[key] is not None and (not isinstance(item[key], str) or not 0 < len(item[key]) <= 256):
                raise ValueError("invalid spatial attempt identity")
        item["lineage"] = _freeze(item["lineage"])
        return cls(**item)


@dataclass(frozen=True, slots=True)
class AtlasFrontierIndex:
    map_id: str
    map_revision: int
    frame_id: str
    metadata_sha256: str
    occupancy_sha256: str
    computed_clock_epoch: str
    computed_time_ns: int
    expires_time_ns: int
    source_clock_epoch: str | None
    source_runtime_pid: int | None
    source_generation: int | None
    source_frame_id: str | None
    measurement_time_ns: int | None
    resolution_m: float
    free_cells: int
    unknown_cells: int
    occupied_cells: int
    frontier_cells: int
    candidates: tuple[tuple[str, float, float, int], ...]

    def to_jsonable(self):
        result = {name: getattr(self, name) for name in self.__dataclass_fields__ if name != "candidates"}
        result.update(candidates=[{"region_id": region, "x_m": x, "y_m": y,
                                   "unknown_neighbors": gain, "score": float(gain)}
                                  for region, x, y, gain in self.candidates],
                      freshness="HISTORICAL_GEOMETRY", motion_authority=False, current_alignment=None)
        return result

    @classmethod
    def from_jsonable(cls, value):
        _frame_identity(value["map_id"], value["frame_id"])
        item = {key: value[key] for key in cls.__dataclass_fields__ if key != "candidates"}
        for key in ("map_revision", "computed_time_ns", "expires_time_ns", "free_cells", "unknown_cells", "occupied_cells", "frontier_cells"):
            if type(item[key]) is not int or item[key] < 0:
                raise ValueError("invalid atlas frontier counters")
        if item["expires_time_ns"] < item["computed_time_ns"]:
            raise ValueError("invalid atlas frontier expiry")
        for key in ("source_runtime_pid", "source_generation", "measurement_time_ns"):
            if item[key] is not None and (type(item[key]) is not int or item[key] < 0):
                raise ValueError("invalid atlas frontier source")
        for key in ("metadata_sha256", "occupancy_sha256"):
            if not isinstance(item[key], str) or len(item[key]) != 64 or any(c not in "0123456789abcdef" for c in item[key]):
                raise ValueError("invalid atlas frontier digest")
        for key in ("source_clock_epoch", "source_frame_id", "computed_clock_epoch"):
            epoch = item[key]
            if (epoch is None and key == "computed_clock_epoch"
                    or epoch is not None and (not isinstance(epoch, str) or not 0 < len(epoch) <= 256)):
                raise ValueError("invalid atlas frontier epoch or source frame")
        resolution = item["resolution_m"]
        if type(resolution) not in (int, float) or not math.isfinite(resolution) or resolution <= 0:
            raise ValueError("invalid atlas frontier resolution")
        candidates = value["candidates"]
        if not isinstance(candidates, (list, tuple)) or len(candidates) > 64:
            raise ValueError("invalid atlas frontier capacity")
        rows = []
        for row in candidates:
            region = row["region_id"]
            if (not isinstance(region, str) or not 0 < len(region) <= 128
                    or any(type(row[key]) not in (int, float) or not math.isfinite(row[key]) for key in ("x_m", "y_m"))
                    or type(row["unknown_neighbors"]) is not int or not 1 <= row["unknown_neighbors"] <= 4):
                raise ValueError("invalid atlas frontier candidate")
            rows.append((region, row["x_m"], row["y_m"], row["unknown_neighbors"]))
        return cls(**item, candidates=tuple(rows))


def build_atlas_frontier_index(reference: AtlasReference, computed_ns: int, *, computed_clock_epoch: str) -> AtlasFrontierIndex:
    """Analyze the existing builder's P5 occupancy asset, outside host locks.

    Unknown means 205, occupied 0 and free 254 in this owning asset format.
    Four-connected free cells adjacent to an explicit unknown cell are
    frontiers. Image boundaries and occupied cells are never unknown evidence.
    """
    directory = Path(reference.metadata_path).parent
    def bounded_read(path, bound):
        with path.open("rb") as stream:
            raw = stream.read(bound + 1)
        if len(raw) > bound:
            raise ValueError("atlas asset grew beyond its read bound")
        return raw
    metadata = bounded_read(Path(reference.metadata_path), 2 * 1024 * 1024)
    if hashlib.sha256(metadata).hexdigest() != reference.metadata_sha256:
        raise ValueError("atlas metadata changed before frontier analysis")
    assets = {name: (digest, size) for name, digest, size in reference.assets}
    for name in ("map.pgm", "map.yaml"):
        digest, size = assets[name]
        if digest is None or _asset_digest(directory / name, size) != digest:
            raise ValueError("atlas geometry changed before frontier analysis")
    raw = bounded_read(directory / "map.pgm", min(64 * 1024 * 1024, assets["map.pgm"][1]))
    if hashlib.sha256(raw).hexdigest() != assets["map.pgm"][0]:
        raise ValueError("atlas occupancy changed while reading")
    # The native builder owns this deliberately small PGM/YAML dialect.
    header = raw.split(b"\n", 3)
    if len(header) != 4 or header[0] != b"P5" or header[2] != b"255":
        raise ValueError("unsupported native atlas PGM")
    width, height = (int(part) for part in header[1].split())
    pixels = header[3]
    if width <= 0 or height <= 0 or width * height > 8_000_000 or len(pixels) != width * height:
        raise ValueError("atlas occupancy exceeds its cell bound")
    if set(pixels) - {0, 205, 254}:
        raise ValueError("unsupported native atlas occupancy classes")
    yaml = {}
    yaml_raw = bounded_read(directory / "map.yaml", min(2 * 1024 * 1024, assets["map.yaml"][1]))
    if hashlib.sha256(yaml_raw).hexdigest() != assets["map.yaml"][0]:
        raise ValueError("atlas coordinates changed while reading")
    for line in yaml_raw.decode().splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            yaml[key.strip()] = value.strip()
    resolution, origin = float(yaml["resolution"]), json.loads(yaml["origin"])
    if (not math.isfinite(resolution) or resolution <= 0 or len(origin) != 3
            or any(type(value) not in (int, float) or not math.isfinite(value) for value in origin)
            or origin[2] != 0 or yaml.get("negate") != "0"):
        raise ValueError("unsupported native atlas coordinates")
    regions, frontier_count = {}, 0
    for row in range(height):
        for column in range(width):
            index = row * width + column
            if pixels[index] != 254:
                continue
            neighbors = ((pixels[index - 1],) if column else ()) + ((pixels[index + 1],) if column + 1 < width else ())
            neighbors += ((pixels[index - width],) if row else ()) + ((pixels[index + width],) if row + 1 < height else ())
            gain = neighbors.count(205)
            if not gain:
                continue
            frontier_count += 1
            x = origin[0] + (column + .5) * resolution
            y = origin[1] + (height - row - .5) * resolution
            # One representative per metre, with bounded deterministic output.
            region = (math.floor(x), math.floor(y))
            candidate = (f"frontier:{region[0]}:{region[1]}", x, y, gain)
            if region in regions:
                if gain > regions[region][3]:
                    regions[region] = candidate
            elif len(regions) < 64:
                regions[region] = candidate
    document = json.loads(metadata)
    build = document.get("build", {})
    pid = build.get("runtime_pid") if isinstance(build, Mapping) else None
    pid = pid if type(pid) is int and pid > 0 else None
    keyframes = document.get("keyframes")
    def unique(attribute, source_attribute):
        values = ({point.get(source_attribute) for point in keyframes} if keyframes is not None
                  else {getattr(point, attribute) for point in reference.viewpoints})
        return next(iter(values)) if len(values) == 1 else None
    measurements = ([point["measurement_time_ns"] for point in keyframes] if keyframes is not None
                    else [point.measurement_time_ns for point in reference.viewpoints]
                    if reference.viewpoint_count == len(reference.viewpoints) else [])
    result = AtlasFrontierIndex(reference.map_id, reference.revision, reference.frame_id,
        reference.metadata_sha256, assets["map.pgm"][0], computed_clock_epoch, computed_ns, computed_ns + 30_000_000_000,
        unique("source_clock_epoch", "source_clock_epoch"), pid, unique("source_generation", "generation"),
        unique("source_frame_id", "source_frame_id"), max(measurements, default=None), resolution,
        pixels.count(254), pixels.count(205), pixels.count(0), frontier_count,
        tuple(sorted(regions.values(), key=lambda item: (-item[3], item[0]))))
    return AtlasFrontierIndex.from_jsonable(result.to_jsonable())
