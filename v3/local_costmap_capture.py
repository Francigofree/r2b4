"""Bounded passive snapshots of the actual completed L4 planner costmap.

Only requested sampled ticks call the packer. It retains a small immutable
header and packs existing cell integers in C; JSON reconstruction belongs to
the capture sidecar. No raw scan, map search or motion authority lives here.
"""
from __future__ import annotations

from collections.abc import Mapping
from itertools import chain
import math
from operator import attrgetter
import struct
from typing import NamedTuple

from .capture_encoding import encode_value
from .contracts import LocalizationQuality, QualityState, TickContext
from .execution import CaptureRecord

LOCAL_COSTMAP_TOPIC = "v3.local_costmap"
LOCAL_COSTMAP_SCHEMA = "R2B4_LOCAL_COSTMAP_SNAPSHOT_V1"
LOCAL_COSTMAP_MAX_CELLS = 4096
_CELL_FIELDS = attrgetter("grid_x", "grid_y", "observation_count")
_CELL = struct.Struct("<iii")


class PackedLocalCostmap(NamedTuple):
    context: TickContext
    available: bool
    availability_reason: str | None
    frame_id: str | None
    revision: int | None
    source_sequence: int | None
    geometry_captured_ns: int | None
    freshness_ns: int | None
    resolution_m: float | None
    radius_m: float | None
    generation: int | None
    localization_quality: LocalizationQuality | None
    source_cell_count: int
    cells: bytes


def pack_local_costmap(record: CaptureRecord) -> PackedLocalCostmap:
    """Copy completed map integers once, without pickling typed cell objects."""
    closed = getattr(record, "inputs", None)
    context = closed.context if closed is not None else record.context
    result = getattr(record, "result", None)
    estimate = world = None
    if result is not None:
        if result.trace.context != context:
            raise ValueError("local costmap capture context mismatch")
        for layer in result.trace.layers:
            if layer.layer == "L3":
                estimate = layer.output
            elif layer.layer == "L4":
                world = layer.output
    if estimate is not None and estimate.context != context:
        raise ValueError("local costmap L3 context mismatch")
    quality = None if estimate is None else estimate.localization_quality
    generation = None if quality is None else quality.generation
    costmap = None if world is None else world.local_costmap
    if costmap is None:
        return PackedLocalCostmap(context, False,
            "L4_UNAVAILABLE" if world is None else "LOCAL_COSTMAP_UNAVAILABLE",
            None, None, None, None, None, None, None, generation, quality, 0, b"")
    if world.context != context or costmap.frame_id != world.frame_id:
        raise ValueError("local costmap capture frame/context mismatch")
    count = len(costmap.occupied_cells)
    if count > LOCAL_COSTMAP_MAX_CELLS:
        raise ValueError("local costmap capture cell bound exceeded")
    captured_ns = context.monotonic_ns - costmap.freshness_ns
    if captured_ns < 0:
        raise ValueError("local costmap capture has invalid measurement time")
    try:
        # attrgetter, map and chain flatten validated immutable cells in C.
        # struct rejects integer overflow; no cell is silently truncated.
        cells = struct.pack(f"<{count * 3}i", *chain.from_iterable(
            map(_CELL_FIELDS, costmap.occupied_cells)))
    except (struct.error, OverflowError) as exc:
        raise ValueError("local costmap cell exceeds the signed int32 wire range") from exc
    return PackedLocalCostmap(context, True, None, costmap.frame_id,
        costmap.revision, costmap.source_sequence, captured_ns, costmap.freshness_ns,
        costmap.resolution_m, costmap.radius_m, generation, quality, count, cells)


def unpack_local_costmap(wire: PackedLocalCostmap) -> dict[str, object]:
    """Reconstruct and validate JSON data only in the observation consumer."""
    if (not isinstance(wire, PackedLocalCostmap) or not isinstance(wire.context, TickContext)
            or type(wire.cells) is not bytes
            or type(wire.source_cell_count) is not int
            or not 0 <= wire.source_cell_count <= LOCAL_COSTMAP_MAX_CELLS
            or len(wire.cells) != wire.source_cell_count * _CELL.size):
        raise ValueError("invalid packed local costmap")
    value = {
        "schema": LOCAL_COSTMAP_SCHEMA,
        "event_type": "local_costmap_snapshot",
        "context": {"tick_id": wire.context.tick_id, "monotonic_ns": wire.context.monotonic_ns},
        "tick_id": wire.context.tick_id,
        "monotonic_ns": wire.context.monotonic_ns,
        "available": wire.available,
        "availability_reason": wire.availability_reason,
        "frame_id": wire.frame_id,
        "revision": wire.revision,
        "source_sequence": wire.source_sequence,
        "geometry_captured_ns": wire.geometry_captured_ns,
        "freshness_ns": wire.freshness_ns,
        "resolution_m": wire.resolution_m,
        "radius_m": wire.radius_m,
        "generation": wire.generation,
        "localization_quality": None if wire.localization_quality is None else encode_value(wire.localization_quality),
        "source_cell_count": wire.source_cell_count,
        "cell_count": wire.source_cell_count,
        "cell_limit": LOCAL_COSTMAP_MAX_CELLS,
        "cells_truncated": False,
        "occupied_cells": [list(cell) for cell in _CELL.iter_unpack(wire.cells)],
        "motion_authority": False,
    }
    validate_local_costmap(value)
    return value


def validate_local_costmap(value: object) -> Mapping[str, object]:
    """Validate a passive snapshot without supplying planner input authority."""
    if (not isinstance(value, Mapping) or value.get("schema") != LOCAL_COSTMAP_SCHEMA
            or value.get("event_type") != "local_costmap_snapshot"
            or type(value.get("available")) is not bool
            or value.get("motion_authority") is not False
            or value.get("cells_truncated") is not False
            or type(value.get("cell_limit")) is not int
            or value["cell_limit"] != LOCAL_COSTMAP_MAX_CELLS):
        raise ValueError("invalid local costmap snapshot")
    for name in ("tick_id", "monotonic_ns", "source_cell_count", "cell_count"):
        if type(value.get(name)) is not int or value[name] < 0:
            raise ValueError("invalid local costmap lineage/count")
    context = value.get("context")
    if (not isinstance(context, Mapping)
            or any(type(context.get(name)) is not int for name in ("tick_id", "monotonic_ns"))
            or context != {"tick_id": value["tick_id"], "monotonic_ns": value["monotonic_ns"]}):
        raise ValueError("local costmap context mismatch")
    cells = value.get("occupied_cells")
    if (not isinstance(cells, (tuple, list)) or len(cells) > LOCAL_COSTMAP_MAX_CELLS
            or value["source_cell_count"] != len(cells) or value["cell_count"] != len(cells)):
        raise ValueError("local costmap cells were lost or truncated")
    seen = set()
    for cell in cells:
        if (not isinstance(cell, (tuple, list)) or len(cell) != 3
                or any(type(n) is not int or not -(2**31) <= n < 2**31 for n in cell)
                or cell[2] <= 0 or tuple(cell[:2]) in seen):
            raise ValueError("invalid local costmap cell")
        seen.add(tuple(cell[:2]))
    quality = value.get("localization_quality")
    if quality is None:
        if value.get("generation") is not None:
            raise ValueError("local costmap generation lacks L3 evidence")
    elif (not isinstance(quality, Mapping) or type(value.get("generation")) is not int
          or value["generation"] < 0 or quality.get("generation") != value["generation"]):
        raise ValueError("local costmap generation mismatch")
    else:
        # Reuse the canonical small L3 quality contract in the consumer. No
        # second range/finite/continuity interpretation is introduced here.
        fields = dict(quality)
        if fields.pop("__type__", "LocalizationQuality") != "LocalizationQuality":
            raise ValueError("invalid local costmap localization quality type")
        try:
            for name in ("local_translation", "heading", "global_position"):
                fields[name] = QualityState(fields[name])
            LocalizationQuality(**fields)
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("invalid local costmap localization quality") from exc
    if not value["available"]:
        if (value.get("availability_reason") not in {"L4_UNAVAILABLE", "LOCAL_COSTMAP_UNAVAILABLE"}
                or cells or any(value.get(name) is not None for name in
                    ("frame_id", "revision", "source_sequence", "geometry_captured_ns",
                     "freshness_ns", "resolution_m", "radius_m"))):
            raise ValueError("unavailable local costmap carries geometry")
        return value
    if value.get("availability_reason") is not None or not isinstance(value.get("frame_id"), str) or not value["frame_id"]:
        raise ValueError("invalid local costmap availability/frame")
    for name in ("revision", "source_sequence", "geometry_captured_ns", "freshness_ns"):
        if type(value.get(name)) is not int or value[name] < 0:
            raise ValueError("invalid local costmap measurement lineage")
    if value["geometry_captured_ns"] != value["monotonic_ns"] - value["freshness_ns"]:
        raise ValueError("local costmap measurement time was rewritten")
    for name in ("resolution_m", "radius_m"):
        if type(value.get(name)) not in (int, float) or not math.isfinite(value[name]) or value[name] <= 0:
            raise ValueError("invalid local costmap geometry")
    return value
