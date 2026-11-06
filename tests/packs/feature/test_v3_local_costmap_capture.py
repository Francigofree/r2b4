"""Actual L4 cells cross the sampled observation edge without raw geometry."""
from dataclasses import replace
import json
import pickle

import pytest

from rig import healthy_localization
from v3.contracts import (
    CommandMode, CommandRequest, CostmapCell, FinalActuation, GLOBAL_FRAME_ID,
    LifecycleState, LOCAL_FRAME_ID, Pose2D, RawDeviceBatch, RobotEstimate,
    RollingLocalCostmap, SafetyDecision, TickContext, WorldSnapshot,
)
from v3.engine import LayerRecord, TickInputs, TickResult, TickTrace
from v3.execution import ExecutionRecord, WriterFailureRecord
from v3.local_costmap_capture import (
    LOCAL_COSTMAP_MAX_CELLS, LOCAL_COSTMAP_SCHEMA, PackedLocalCostmap,
    pack_local_costmap, unpack_local_costmap, validate_local_costmap,
)


def _record(cells=(CostmapCell(-3, 7, 19), CostmapCell(2, -4, 3)), *, map_available=True, layer_available=True):
    context = TickContext(17, 1_000_000_000)
    quality = healthy_localization(generation=3, accumulated_translation_bias_warning=True,
                                   encoder_lidar_consistency_m=.43)
    estimate = RobotEstimate(context, GLOBAL_FRAME_ID, 10., 20., .2, 0., 0., (0.,) * 25,
        local_pose=Pose2D(LOCAL_FRAME_ID, 1., 2., .2),
        global_pose=Pose2D(GLOBAL_FRAME_ID, 10., 20., .2),
        map_to_odom=Pose2D(GLOBAL_FRAME_ID, 9., 18., 0.), localization_quality=quality)
    costmap = RollingLocalCostmap(LOCAL_FRAME_ID, 123, .05, 6., cells, 91, 35_000_000)
    world = WorldSnapshot(context, LOCAL_FRAME_ID, 7, (), 30_000_000,
                          costmap if map_available else None)
    layers = (LayerRecord("L3", estimate),)
    if layer_available:
        layers += (LayerRecord("L4", world),)
    final = FinalActuation(context, 0., 0., False, SafetyDecision.STOP, "STOPPED", "NOT_ACTIVE")
    inputs = TickInputs(context, RawDeviceBatch(context, (), ()),
        CommandRequest(context, "snapshot", CommandMode.STOP, (), 17), LifecycleState.IDLE)
    return ExecutionRecord(inputs, TickResult(final, TickTrace(context, layers)))


def test_actual_completed_cells_frame_measurement_and_quality_survive_sidecar_wire(monkeypatch):
    import v3.local_costmap_capture as capture
    record = _record()
    source = record.result.trace.layers[1].output.local_costmap
    with monkeypatch.context() as patch:
        patch.setattr(capture, "encode_value", lambda *_: pytest.fail("control packer encoded JSON"))
        patch.setattr(CostmapCell, "__reduce_ex__", lambda *_: pytest.fail("typed cells crossed IPC"))
        wire = pack_local_costmap(record)
        transported = pickle.loads(pickle.dumps(wire))
    assert isinstance(wire, PackedLocalCostmap)
    assert len(wire.cells) == 12 * len(source.occupied_cells)
    payload = unpack_local_costmap(wire)
    assert unpack_local_costmap(transported) == payload
    assert json.loads(json.dumps(payload)) == payload
    assert payload["schema"] == LOCAL_COSTMAP_SCHEMA
    assert payload["available"] and payload["availability_reason"] is None
    assert payload["context"] == {"tick_id": 17, "monotonic_ns": 1_000_000_000}
    assert payload["frame_id"] == source.frame_id == LOCAL_FRAME_ID
    assert payload["revision"] == source.revision
    assert payload["source_sequence"] == source.source_sequence
    assert payload["geometry_captured_ns"] == 965_000_000
    assert payload["freshness_ns"] == source.freshness_ns
    assert payload["resolution_m"] == source.resolution_m
    assert payload["radius_m"] == source.radius_m
    assert payload["generation"] == payload["localization_quality"]["generation"] == 3
    assert payload["localization_quality"]["accumulated_translation_bias_warning"] is True
    assert payload["occupied_cells"] == [[c.grid_x, c.grid_y, c.observation_count] for c in source.occupied_cells]
    assert payload["source_cell_count"] == payload["cell_count"] == 2
    assert payload["cells_truncated"] is payload["motion_authority"] is False


@pytest.mark.parametrize("layer_available,reason", [(False, "L4_UNAVAILABLE"), (True, "LOCAL_COSTMAP_UNAVAILABLE")])
def test_missing_layer_or_map_is_explicit_unavailability(layer_available, reason):
    payload = unpack_local_costmap(pack_local_costmap(_record(
        map_available=False, layer_available=layer_available)))
    assert payload["available"] is False
    assert payload["availability_reason"] == reason
    assert payload["occupied_cells"] == []
    assert payload["geometry_captured_ns"] is payload["source_sequence"] is None
    assert payload["generation"] == 3


def test_writer_failure_does_not_manufacture_a_map_or_localization():
    context = TickContext(17, 1_000_000_000)
    record = WriterFailureRecord(context, LifecycleState.ACTIVE, "WRITE_FAILED", None)
    payload = unpack_local_costmap(pack_local_costmap(record))
    assert not payload["available"]
    assert payload["generation"] is payload["localization_quality"] is None
    assert not payload["occupied_cells"]


def test_actual_empty_map_is_available_and_not_mistaken_for_missing_evidence():
    payload = unpack_local_costmap(pack_local_costmap(_record(())))
    assert payload["available"] is True
    assert payload["source_cell_count"] == payload["cell_count"] == 0
    assert payload["geometry_captured_ns"] == 965_000_000


def test_cell_bound_is_explicit_and_never_truncates_actual_topology():
    cells = tuple(CostmapCell(i, -i, i + 1) for i in range(LOCAL_COSTMAP_MAX_CELLS))
    wire = pack_local_costmap(_record(cells))
    payload = unpack_local_costmap(wire)
    assert len(wire.cells) == 12 * LOCAL_COSTMAP_MAX_CELLS
    assert payload["occupied_cells"][-1] == [4095, -4095, 4096]
    assert payload["source_cell_count"] == len(cells)
    with pytest.raises(ValueError, match="cell bound exceeded"):
        pack_local_costmap(_record(cells + (CostmapCell(4096, 0, 1),)))


@pytest.mark.parametrize("cell", [CostmapCell(2**31, 0, 1), CostmapCell(0, -(2**31) - 1, 1), CostmapCell(0, 0, 2**31)])
def test_integer_overflow_is_reported_instead_of_wrapping_geometry(cell):
    with pytest.raises(ValueError, match="int32"):
        pack_local_costmap(_record((cell,)))


@pytest.mark.parametrize("change", ["lost_bytes", "rewritten_count", "duplicate_cell", "measurement_time", "generation", "nonfinite_quality", "bool_context"])
def test_corrupt_transport_or_rewritten_lineage_cannot_become_a_valid_map(change):
    wire = pack_local_costmap(_record())
    if change in {"lost_bytes", "rewritten_count"}:
        wire = wire._replace(cells=wire.cells[:-1]) if change == "lost_bytes" else wire._replace(source_cell_count=3)
        with pytest.raises(ValueError):
            unpack_local_costmap(wire)
    else:
        payload = unpack_local_costmap(wire)
        if change == "duplicate_cell":
            payload["occupied_cells"][1] = payload["occupied_cells"][0]
        elif change == "measurement_time":
            payload["geometry_captured_ns"] += 1
        elif change == "generation":
            payload["generation"] += 1
        elif change == "nonfinite_quality":
            payload["localization_quality"]["local_sigma_m"] = float("nan")
        else:
            payload["tick_id"] = 1
            payload["context"]["tick_id"] = True
        with pytest.raises(ValueError):
            validate_local_costmap(payload)


def test_completed_world_from_another_tick_is_never_silently_relabelled():
    record = _record()
    layer = record.result.trace.layers[1]
    bad_world = replace(layer.output, context=TickContext(18, 1_020_000_000))
    bad_trace = replace(record.result.trace, layers=(record.result.trace.layers[0], LayerRecord("L4", bad_world)))
    with pytest.raises(ValueError, match="frame/context"):
        pack_local_costmap(replace(record, result=replace(record.result, trace=bad_trace)))


def test_localization_quality_from_another_tick_is_not_attached_to_current_geometry():
    record = _record()
    layer = record.result.trace.layers[0]
    bad_estimate = replace(layer.output, context=TickContext(18, 1_020_000_000))
    bad_trace = replace(record.result.trace, layers=(LayerRecord("L3", bad_estimate), record.result.trace.layers[1]))
    with pytest.raises(ValueError, match="L3 context"):
        pack_local_costmap(replace(record, result=replace(record.result, trace=bad_trace)))
