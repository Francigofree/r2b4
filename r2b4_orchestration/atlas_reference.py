"""Read-only historical atlas references, without grids or localization authority.

Only bounded metadata and viewpoints are loaded. Occupancy and graph assets stay
in their original files; loading an atlas cannot establish a present frame link.
"""
from __future__ import annotations

import csv
import hashlib
import json
import math
from dataclasses import dataclass, replace
from pathlib import Path

from .world_model import KnowledgeState


ATLAS_SCHEMA = "R2B4_ATLAS_REFERENCE_V1"
_MAX_METADATA_BYTES = 2 * 1024 * 1024
_MAX_ASSET_BYTES = 64 * 1024 * 1024
_MAX_VIEWPOINTS = 64
_ASSET_NAMES = frozenset({"map.pgm", "map.yaml", "map_data.npz", "keyframes.csv", "pose_graph.json"})


def _read(path: Path) -> bytes:
    with path.open("rb") as handle:
        raw = handle.read(_MAX_METADATA_BYTES + 1)
    if len(raw) > _MAX_METADATA_BYTES:
        raise ValueError("atlas metadata exceeds its bound")
    return raw


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def _sha_token(value, *, optional=False):
    if optional and value is None:
        return None
    if (not isinstance(value, str) or len(value) != 64
            or any(char not in "0123456789abcdef" for char in value)):
        raise ValueError("invalid atlas sha256")
    return value


def _asset_digest(path: Path, declared_bytes: int) -> str:
    """Explicit host import only; bounded streaming, without decoding geometry."""
    digest, total = hashlib.sha256(), 0
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(256 * 1024), b""):
            total += len(block)
            if total > min(declared_bytes, _MAX_ASSET_BYTES):
                raise ValueError("atlas asset exceeds its declared bound")
            digest.update(block)
    if total != declared_bytes:
        raise ValueError("atlas asset changed while loading")
    return digest.hexdigest()


def _token(value, name, *, optional=False, maximum=256):
    if optional and value is None:
        return None
    if not isinstance(value, str) or not 0 < len(value) <= maximum:
        raise ValueError("invalid atlas " + name)
    return value


def _integer(value, name):
    if type(value) is not int or value < 0:
        raise ValueError("invalid atlas " + name)
    return value


def _frame_identity(map_id, frame_id):
    """An atlas keeps its own frame even when restored from host memory."""
    _token(map_id, "map_id")
    _token(frame_id, "frame_id")
    if map_id.startswith("atlas:"):
        digest, prefix = map_id[len("atlas:"):], "R2B4_ATLAS:"
    elif map_id.startswith("historical:"):
        digest, prefix = map_id[len("historical:"):], "R2B4_HISTORICAL_ATLAS:"
    else:
        raise ValueError("invalid atlas map identity")
    if frame_id != prefix + _sha_token(digest):
        raise ValueError("atlas frame identity mismatch")


def _pose(value, *, optional=False):
    if optional and value is None:
        return None
    if (not isinstance(value, (list, tuple)) or len(value) != 3
            or any(type(item) not in (int, float) or not math.isfinite(item) for item in value)):
        raise ValueError("invalid atlas pose")
    return tuple(float(item) for item in value)


@dataclass(frozen=True, slots=True)
class AtlasViewpoint:
    map_id: str
    map_revision: int
    frame_id: str
    viewpoint_id: str
    keyframe_index: int
    pose: tuple[float, float, float]
    original_pose: tuple[float, float, float] | None
    measurement_time_ns: int
    source_frame_id: str | None
    source_clock_epoch: str | None
    source_generation: int | None
    capture_id: str | None
    place_ids: tuple[str, ...] = ()
    source_scan_revision: int | None = None
    source_scan_sequence: int | None = None
    source_tick_ids: tuple[int, ...] = ()

    def to_jsonable(self):
        return {"map_id": self.map_id, "map_revision": self.map_revision,
                "frame_id": self.frame_id, "viewpoint_id": self.viewpoint_id,
                "keyframe_index": self.keyframe_index, "pose": list(self.pose),
                "original_pose": None if self.original_pose is None else list(self.original_pose),
                "measurement_time_ns": self.measurement_time_ns,
                "source_frame_id": self.source_frame_id, "source_clock_epoch": self.source_clock_epoch,
                "source_generation": self.source_generation, "capture_id": self.capture_id,
                "source_scan_revision": self.source_scan_revision,
                "source_scan_sequence": self.source_scan_sequence,
                "source_tick_ids": list(self.source_tick_ids),
                "place_ids": list(self.place_ids), "freshness": "HISTORICAL_GEOMETRY",
                "motion_authority": False, "current_alignment": None}

    @classmethod
    def from_jsonable(cls, value):
        _frame_identity(value["map_id"], value["frame_id"])
        index = _integer(value["keyframe_index"], "keyframe_index")
        if value["viewpoint_id"] != "keyframe:" + str(index):
            raise ValueError("atlas viewpoint identity mismatch")
        places = value.get("place_ids", [])
        if not isinstance(places, (list, tuple)) or len(places) > 64:
            raise ValueError("invalid atlas place bindings")
        ticks = value.get("source_tick_ids", [])
        if not isinstance(ticks, (list, tuple)) or len(ticks) > 2:
            raise ValueError("invalid atlas source tick references")
        return cls(_token(value["map_id"], "map_id"), _integer(value["map_revision"], "revision"),
                   _token(value["frame_id"], "frame_id"), _token(value["viewpoint_id"], "viewpoint_id"),
                   _integer(value["keyframe_index"], "keyframe_index"), _pose(value["pose"]),
                   _pose(value.get("original_pose"), optional=True),
                   _integer(value["measurement_time_ns"], "measurement_time"),
                   _token(value.get("source_frame_id"), "source_frame", optional=True),
                   _token(value.get("source_clock_epoch"), "clock_epoch", optional=True),
                   None if value.get("source_generation") is None else _integer(value["source_generation"], "generation"),
                   _token(value.get("capture_id"), "capture_id", optional=True),
                   tuple(_token(item, "place_id") for item in places),
                   None if value.get("source_scan_revision") is None else _integer(value["source_scan_revision"], "scan revision"),
                   None if value.get("source_scan_sequence") is None else _integer(value["source_scan_sequence"], "scan sequence"),
                   tuple(_integer(item, "source tick id") for item in ticks))


@dataclass(frozen=True, slots=True)
class AtlasReference:
    map_id: str
    revision: int
    frame_id: str
    metadata_path: str
    metadata_sha256: str
    input_sha256: str | None
    provenance: str
    assets: tuple[tuple[str, str | None, int], ...]
    viewpoints: tuple[AtlasViewpoint, ...]
    viewpoint_count: int
    indexed_viewpoint_count: int | None = None

    def to_jsonable(self):
        return {"map_id": self.map_id, "revision": self.revision, "frame_id": self.frame_id,
                "metadata_path": self.metadata_path, "metadata_sha256": self.metadata_sha256,
                "input_sha256": self.input_sha256, "provenance": self.provenance,
                "asset_integrity": "VERIFIED_AT_IMPORT",
                "assets": [{"name": name, "sha256": digest, "bytes": size}
                           for name, digest, size in self.assets],
                "viewpoint_count": self.viewpoint_count,
                "indexed_viewpoints": len(self.viewpoints) if self.indexed_viewpoint_count is None else self.indexed_viewpoint_count,
                "truncated": (len(self.viewpoints) if self.indexed_viewpoint_count is None else self.indexed_viewpoint_count) < self.viewpoint_count,
                "freshness": "HISTORICAL_GEOMETRY", "motion_authority": False, "current_alignment": None}

    @classmethod
    def from_jsonable(cls, value):
        _frame_identity(value["map_id"], value["frame_id"])
        assets = value.get("assets", [])
        if not isinstance(assets, (list, tuple)) or len(assets) > len(_ASSET_NAMES):
            raise ValueError("invalid atlas asset summary")
        names = [item.get("name") for item in assets if isinstance(item, dict)]
        if len(names) != len(assets) or len(set(names)) != len(names) or set(names) - _ASSET_NAMES:
            raise ValueError("invalid atlas asset summary identity")
        count = _integer(value["viewpoint_count"], "viewpoint count")
        indexed = _integer(value["indexed_viewpoints"], "indexed viewpoints")
        if indexed > min(count, _MAX_VIEWPOINTS):
            raise ValueError("atlas index exceeds its bound")
        return cls(_token(value["map_id"], "map_id"), _integer(value["revision"], "revision"),
                   _token(value["frame_id"], "frame_id"), _token(value["metadata_path"], "metadata path", maximum=4096),
                   _sha_token(value["metadata_sha256"]),
                   _sha_token(value.get("input_sha256"), optional=True),
                   _token(value["provenance"], "provenance"),
                   tuple((_token(item["name"], "asset name"),
                          _sha_token(item.get("sha256"), optional=True),
                          _integer(item["bytes"], "asset bytes")) for item in assets),
                   (), count, indexed)

    def bound_places(self, facts):
        """Names and explicit teaching stay accepted Public World evidence."""
        bindings = {}
        for fact in facts:
            if (fact.domain != "room_topology" or fact.state not in {KnowledgeState.KNOWN, KnowledgeState.LIKELY}
                    or not hasattr(fact.value, "get")):
                continue
            value = fact.value
            if value.get("map_id") == self.map_id and isinstance(value.get("viewpoint_id"), str):
                bindings.setdefault(value["viewpoint_id"], set()).add(fact.entity_id)
        return tuple(replace(item, place_ids=tuple(sorted(bindings.get(item.viewpoint_id, ()))[:64]))
                     for item in self.viewpoints)


def load_atlas_reference(path: str | Path) -> AtlasReference:
    """Import an existing asset's bounded metadata, never its grid or NPZ."""
    path = Path(path).expanduser().resolve()
    if path.is_dir():
        path = path / ("atlas_reference.json" if (path / "atlas_reference.json").is_file() else "report.json")
    _token(str(path), "metadata path", maximum=4096)
    raw = _read(path)
    document = json.loads(raw)
    metadata_sha = hashlib.sha256(raw).hexdigest()
    if not isinstance(document, dict):
        raise ValueError("invalid atlas metadata")
    if document.get("schema") == ATLAS_SCHEMA:
        content = {key: value for key, value in document.items() if key not in {"map_id", "frame_id"}}
        identity = _digest(content)
        if (document.get("map_id") != "atlas:" + identity
                or document.get("frame_id") != "R2B4_ATLAS:" + identity
                or document.get("motion_authority") is not False or document.get("current_alignment") is not None):
            raise ValueError("invalid atlas identity or authority")
        map_id, frame = document["map_id"], document["frame_id"]
        revision = _integer(document.get("revision"), "revision")
        keyframes = document.get("keyframes")
        if not isinstance(keyframes, list) or not keyframes:
            raise ValueError("invalid atlas keyframe index")
        assets = []
        for item in document.get("assets", []):
            name, digest = item["name"], item["sha256"]
            if name not in _ASSET_NAMES or any(row[0] == name for row in assets):
                raise ValueError("invalid atlas asset reference")
            digest = _sha_token(digest)
            size = _integer(item["bytes"], "asset bytes")
            if sum(row[2] for row in assets) + size > _MAX_ASSET_BYTES:
                raise ValueError("atlas asset import exceeds its byte bound")
            if not (path.parent / name).is_file() or (path.parent / name).stat().st_size != size:
                raise ValueError("atlas asset unavailable or changed")
            if _asset_digest(path.parent / name, size) != digest:
                raise ValueError("atlas asset sha256 mismatch")
            assets.append((name, digest, size))
        if {item[0] for item in assets} != _ASSET_NAMES:
            raise ValueError("incomplete atlas asset references")
        source = document.get("input", {})
        input_sha = _sha_token(source.get("sha256"))
        viewpoints = []
        indexes = set()
        for item in keyframes:
            index = _integer(item["index"], "keyframe index")
            if index in indexes:
                raise ValueError("duplicate atlas viewpoint")
            indexes.add(index)
            if len(viewpoints) < _MAX_VIEWPOINTS:
                scan = next((row for row in item.get("source_scans", [])
                             if row.get("measurement_time_ns") == item["measurement_time_ns"]), {})
                viewpoints.append(AtlasViewpoint.from_jsonable({
                    "map_id": map_id, "map_revision": revision, "frame_id": frame,
                    "viewpoint_id": "keyframe:" + str(index), "keyframe_index": index,
                    "pose": item["optimized_pose"], "original_pose": item["original_pose"],
                    "measurement_time_ns": item["measurement_time_ns"],
                    "source_frame_id": item.get("source_frame_id"),
                    "source_clock_epoch": item.get("source_clock_epoch"),
                    "source_generation": item.get("generation"), "capture_id": source.get("capture_id"),
                    "source_scan_revision": scan.get("revision"), "source_scan_sequence": scan.get("sequence"),
                    "source_tick_ids": [row["tick_id"] for row in item.get("source_ticks", [])
                                        if row.get("tick_id") is not None]}))
        return AtlasReference(map_id, revision, frame, str(path), metadata_sha, input_sha,
                              "EXPORTED_SOURCE_GAUGE", tuple(assets), tuple(viewpoints), len(keyframes))
    if document.get("schema") != "R2B4_GLOBAL_MAP_BUILD_V1":
        raise ValueError("unsupported atlas metadata schema")
    # V1 seeds are already normalized. Never present them as the original gauge.
    assets = []
    for name in sorted(_ASSET_NAMES):
        asset = path.parent / name
        if not asset.is_file():
            continue
        size = asset.stat().st_size
        if sum(row[2] for row in assets) + size > _MAX_ASSET_BYTES:
            raise ValueError("atlas asset import exceeds its byte bound")
        assets.append((name, _asset_digest(asset, size), size))
    identity = _digest({"metadata_sha256": metadata_sha, "assets": assets})
    map_id, frame = "historical:" + identity, "R2B4_HISTORICAL_ATLAS:" + identity
    viewpoints = []
    csv_path = path.parent / "keyframes.csv"
    csv_raw = _read(csv_path) if csv_path.is_file() else None
    if csv_raw is not None and hashlib.sha256(csv_raw).hexdigest() != next(row[1] for row in assets if row[0] == "keyframes.csv"):
        raise ValueError("legacy atlas keyframe index changed while loading")
    rows = list(csv.DictReader(csv_raw.decode("utf-8").splitlines())) if csv_raw is not None else []
    indexes = [_integer(int(item["index"]), "keyframe index") for item in rows]
    if len(set(indexes)) != len(indexes):
        raise ValueError("duplicate legacy atlas viewpoint")
    for item in rows[:_MAX_VIEWPOINTS]:
        viewpoints.append(AtlasViewpoint(map_id, 0, frame, "keyframe:" + item["index"],
            _integer(int(item["index"]), "keyframe index"),
            _pose([float(item[name]) for name in ("opt_x", "opt_y", "opt_yaw")]), None,
            _integer(int(item["ref_ns"]), "measurement time"), None, None,
            _integer(int(item["generation"]), "generation"), None))
    return AtlasReference(map_id, 0, frame, str(path), metadata_sha,
                          _sha_token(document.get("input", {}).get("sha256"), optional=True), "LEGACY_SOURCE_GAUGE_UNAVAILABLE",
                          tuple(assets), tuple(viewpoints), len(rows))
