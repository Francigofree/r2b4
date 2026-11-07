"""Passive bounded LiDAR maps survive the producer and spawned recorder edges."""
from dataclasses import replace
import math
import os
import queue
import sys
import threading
from types import SimpleNamespace

import pytest

from rig import ROOT, healthy_localization, resolved_config
from test_flight_recorder import closed_stop_record
from tools.mcap_evidence.compiler import compile_evidence
from tools.mcap_evidence.query import query
from tools.mcap_evidence.verify import verify
from v3.adapters import process_lidar_port
from v3.adapters.native_lidar_port import NativeRawLidarSnapshot
from v3.adapters.rplidar_c1 import RplidarPoint
from v3.mcap_capture import EVENT_TOPIC, RAW_LIDAR_TOPIC, McapCaptureConfig
from v3.mcap_reader import McapReader, McapReadError
from v3.navigation_map_capture import (
    NAVIGATION_MAP_MAX_POINTS, NAVIGATION_MAP_TOPIC, navigation_map_observation,
    validate_navigation_map,
)
from v3.process_sidecars import ProcessMcapCaptureSession


def _grid_record(*, missing=False, empty=False):
    from v3.contracts import CostmapCell, LOCAL_FRAME_ID, RobotEstimate, RollingLocalCostmap, WorldSnapshot
    from v3.engine import LayerRecord

    record = closed_stop_record()
    context = record.inputs.context
    estimate = RobotEstimate(context, LOCAL_FRAME_ID, 0., 0., 0., 0., 0., (0.,) * 25,
                             localization_quality=healthy_localization(generation=7))
    cells = () if empty else (CostmapCell(-4, 2, 1), CostmapCell(3, -7, 4), CostmapCell(9, 0, 2))
    grid = None if missing else RollingLocalCostmap(LOCAL_FRAME_ID, 73, .05, 2.5, cells, 91, 40_000_000)
    world = WorldSnapshot(context, LOCAL_FRAME_ID, 61, (), 0, grid)
    # The device-free STOP helper ends at L2 without sensor evidence. Supply
    # explicit typed completed L3/L4 outputs for this capture-boundary fixture.
    layers = record.result.trace.layers[:-1] + (
        LayerRecord("L3", estimate), LayerRecord("L4", world), record.result.trace.layers[-1])
    return replace(record, result=replace(record.result, trace=replace(record.result.trace, layers=layers))), grid


def _retime_record(record, tick, stamp):
    from v3.contracts import TickContext

    context = TickContext(tick, stamp)
    inputs = replace(record.inputs, context=context,
        raw_devices=replace(record.inputs.raw_devices, context=context),
        command=replace(record.inputs.command, context=context, expiry_tick=tick), planner_input=None)
    layers = tuple(replace(row, output=replace(row.output, context=context))
                   for row in record.result.trace.layers)
    return replace(record, inputs=inputs, result=replace(record.result,
        final_actuation=replace(record.result.final_actuation, context=context),
        trace=replace(record.result.trace, context=context, layers=layers)))


def _scan(sequence, measured, *, health="OK", count=360):
    end = measured + 40_000_000
    return NativeRawLidarSnapshot(sequence, end / 1e9, measured - 40_000_000,
        end, measured, health,
        tuple(RplidarPoint(i * 360. / count, 1. + (i % 3) * .1, 10)
              for i in range(count)), {}, observed_monotonic_ns=end + 10_000_000)


def _map(scan):
    compact = process_lidar_port._wire_control_raw(scan, minimum_range_m=.08,
        maximum_range_m=3., maximum_points=NAVIGATION_MAP_MAX_POINTS)
    return navigation_map_observation(scan, compact[6], minimum_range_m=.08,
        maximum_range_m=3., x_offset_m=-.005, y_offset_m=.012)


def _events(path):
    rows = list(McapReader(path).iter_json_messages(topics=[EVENT_TOPIC]))
    integrity = next(row["integrity"] for _, row in rows
                     if row.get("event_type") == "capture_finalized")
    return rows, integrity


def _spawn_capture(tmp_path, *, maps=(), end=None, tick_hz=5):
    control = resolved_config().runtime.composition.live_control.control
    config = McapCaptureConfig(mode="append_only", tick_sample_hz=tick_hz,
        navigation_map_hz=5 if tick_hz == 5 else 0,
        require_raw_lidar_transport_end=tick_hz == 50)
    session = ProcessMcapCaptureSession("map-offline", tmp_path / "map.mcap",
        configuration={"production_control": control}, config=config,
        project_root=tmp_path, expect_raw_lidar_end=tick_hz == 50)
    session.start()
    assert session._process.pid != os.getpid()
    try:
        session.observe(closed_stop_record())
        for observation in maps:
            session.raw_lidar_queue.put(("navigation_map", observation), timeout=2.)
        if end is not None:
            session.raw_lidar_queue.put(("raw_end", *end), timeout=2.)
    finally:
        path = session.finalize(SimpleNamespace(status=0))
    assert path is not None and not session._process.is_alive()
    return path


def test_spawned_map_capture_preserves_physical_time_geometry_and_compiler_fields(tmp_path):
    observations = (_map(_scan(17, 1_100_000_000)),
                    _map(_scan(19, 1_300_000_000, health="STALE")))
    path = _spawn_capture(tmp_path, maps=observations, end=(19, 2, 0))
    rows, integrity = _events(path)
    recorded = [(message, row) for message, row in rows
                if row.get("source_topic") == NAVIGATION_MAP_TOPIC]
    assert len(recorded) == 2
    for (message, row), observation in zip(recorded, observations):
        assert message.log_time_ns == row["monotonic_ns"] == observation["measurement_monotonic_ns"]
        assert row["payload"]["source_sequence"] == observation["source_sequence"]
        assert message.publish_time_ns == row["published_monotonic_ns"]
        assert message.publish_time_ns != message.log_time_ns
        assert row["payload"] == {**observation, "points_xy_m": [list(p) for p in observation["points_xy_m"]]}
        assert len(row["payload"]["points_xy_m"]) == 96
        assert row["payload"]["points_xy_m"][0] == pytest.approx((.995, .012))
    assert integrity["complete"] and integrity["navigation_map_complete"]
    assert integrity["navigation_map_transport"] == {
        "received_count": 2, "written_count": 2, "end": True,
        "produced_count": 2, "superseded_count": 0}
    assert integrity["sample_complete"] and not integrity["replay_complete"]
    assert not integrity["raw_evidence_requested"] and not integrity["raw_evidence_complete"]
    assert not list(McapReader(path).iter_json_messages(topics=[RAW_LIDAR_TOPIC]))
    compiled = compile_evidence(path, tmp_path / "map.evidence", workers=1)
    assert compiled["compiler_status"] == "COMPLETE"
    assert verify(compiled["output"], source=path)["messages"] > 0
    exported = [row["payload"] for row in query(compiled["output"], topic=EVENT_TOPIC)
                if row["payload"].get("source_topic") == NAVIGATION_MAP_TOPIC]
    assert exported == [row for _, row in recorded]


@pytest.mark.parametrize("case,expected", [
    ("loss", {"NAVIGATION_MAP_TRANSPORT_LOSS"}),
    ("missing_end", {"NAVIGATION_MAP_TRANSPORT_END_MISSING", "NAVIGATION_MAP_TRANSPORT_LOSS"}),
    ("zero", {"NAVIGATION_MAP_OBSERVATIONS_MISSING"}),
])
def test_map_transport_failure_is_separate_from_tick_and_raw_replay_integrity(tmp_path, case, expected):
    observations = () if case == "zero" else (_map(_scan(1, 1_100_000_000)),)
    end = None if case == "missing_end" else ((1, 2, 1) if case == "loss" else (0, 0, 0))
    path = _spawn_capture(tmp_path, maps=observations, end=end)
    _, integrity = _events(path)
    assert not integrity["complete"] and not integrity["navigation_map_complete"]
    assert set(integrity["navigation_map_integrity_reasons"]) == expected
    assert integrity["sample_complete"] and not integrity["replay_complete"]
    assert integrity["replay_integrity_reasons"] == integrity["raw_integrity_reasons"] == []
    assert not integrity["raw_evidence_requested"] and not integrity["raw_evidence_complete"]
    with pytest.raises(McapReadError, match="CAPTURE_INCOMPLETE"):
        McapReader(path).capture_integrity()


def _run_producer(monkeypatch, target, *, capacity=96):
    stop = threading.Event()
    scans = tuple(_scan(i + 1, stamp, health="ERROR" if i == 5 else "OK")
                  for i, stamp in enumerate((1_001_000_000, 1_050_000_000,
                      1_199_999_999, 1_200_000_000, 1_250_000_000, 1_400_000_000)))

    class Port:
        position = 0
        stopped = False

        def get_raw_scan_snapshot(self):
            scan = scans[self.position]
            self.position += 1
            if self.position == len(scans):
                stop.set()
            return scan

        def get_matcher_result(self): return None
        def get_relative_result(self): return None
        def get_runtime_status(self): return {"running": True}
        def stop(self): self.stopped = True

    port = Port()
    monkeypatch.setitem(sys.modules, "serial", SimpleNamespace(Serial=None))
    monkeypatch.setattr(process_lidar_port, "open_native_lidar_port", lambda *args, **kwargs: port)
    monkeypatch.setattr(process_lidar_port, "_wire_raw",
        lambda _scan: pytest.fail("sampled map capture must never encode a full raw scan"))
    resolved = resolved_config()
    lidar = replace(resolved.lidar, matcher=replace(resolved.lidar.matcher,
                    lidar_x_offset_m=-.005, lidar_y_offset_m=.012))
    state, ready = queue.Queue(32), threading.Event()
    process_lidar_port._lidar_owner_process_main(lidar, None, None, None, None,
        state, target, ready, stop, None, None, False, .08, 3., capacity,
        replace(resolved.edges.lidar_process, raw_end_timeout_s=.001), 5)
    assert ready.is_set() and port.stopped
    states = []
    while not state.empty():
        value = state.get_nowait()
        assert value[0] == "state", value
        states.append(value)
    return scans, states


def test_real_producer_uses_control_points_physical_5hz_buckets_and_no_full_raw_wire(monkeypatch):
    target = queue.Queue(16)
    scans, states = _run_producer(monkeypatch, target)
    messages = []
    while not target.empty():
        messages.append(target.get_nowait())
    assert messages[-1] == ("raw_end", 6, 3, 0)
    maps = [message[1] for message in messages[:-1]]
    assert [value["source_sequence"] for value in maps] == [1, 4, 6]
    assert [value["sensor_health"] for value in maps] == ["OK", "OK", "ERROR"]
    by_revision = {value[1][0]: value[1] for value in states}
    assert len(by_revision) == 6
    for scan, state in zip(scans, states):
        expected = process_lidar_port._wire_control_raw(scan,
            minimum_range_m=.08, maximum_range_m=3., maximum_points=96)
        assert state[1][:7] == expected[:7]
        assert state[1][8:] == expected[8:]
        assert all(state[1][7][key] == value for key, value in scan.summary.items())
        assert state[1][7]["rotation_deskew"] == "POINT_TIMING_UNAVAILABLE"
        assert state[1][7]["local_geometry_pending"] is False
        assert len(state) == 5 and len(state[1][6]) <= 96
    for observation in maps:
        compact = by_revision[observation["source_sequence"]]
        expected = tuple((distance * math.cos(math.radians(angle)) - .005,
                          -distance * math.sin(math.radians(angle)) + .012)
                         for angle, distance, _quality in compact[6])
        assert len(observation["points_xy_m"]) == len(expected)
        for actual, point in zip(observation["points_xy_m"], expected):
            assert actual == pytest.approx(point)
        assert observation["motion_authority"] is False
        assert observation["source_point_count"] == 360


def test_producer_overflow_seals_an_explicit_loss_count_before_cleanup(monkeypatch):
    target = queue.Queue(1)
    _run_producer(monkeypatch, target)
    assert target.get_nowait() == ("raw_end", 6, 3, 3)
    assert target.empty()


def test_observation_hard_96_bound_and_invalid_lineage_fail_closed():
    scan = _scan(1, 1_100_000_000)
    points = tuple((point.angle_deg, point.distance_m, point.quality) for point in scan.raw_scan)
    observation = navigation_map_observation(scan, points, minimum_range_m=.08,
        maximum_range_m=3., x_offset_m=-.005, y_offset_m=0.)
    assert len(observation["points_xy_m"]) == 96
    for bad in ({**observation, "points_xy_m": ((1., 0.),) * 97},
                {**observation, "measurement_monotonic_ns": scan.scan_end_monotonic_ns + 1},
                {**observation, "motion_authority": True},
                {**observation, "points_xy_m": ((float("nan"), 0.),)}):
        with pytest.raises(ValueError):
            validate_navigation_map(bad)


def test_existing_full_raw_50hz_lane_still_captures_and_replays_without_map_requirement(tmp_path):
    from v3.replay import replay_capture

    control = resolved_config().runtime.composition.live_control.control
    session = ProcessMcapCaptureSession("raw-offline", tmp_path / "raw.mcap",
        configuration={"production_control": control}, project_root=tmp_path,
        expect_raw_lidar_end=True, config=McapCaptureConfig(mode="append_only",
            tick_sample_hz=50, require_raw_lidar_transport_end=True))
    session.start()
    session.observe(closed_stop_record())
    scan = _scan(1, 1_100_000_000, count=4)
    session.raw_lidar_queue.put(("raw", process_lidar_port._wire_raw(scan)), timeout=2.)
    session.raw_lidar_queue.put(("raw_end", 1, 1, 0), timeout=2.)
    path = session.finalize(SimpleNamespace(status=0))
    _, integrity = _events(path)
    assert integrity["complete"] and integrity["replay_complete"] and integrity["raw_evidence_complete"]
    assert not integrity["navigation_map_requested"]
    assert integrity["navigation_map_integrity_reasons"] == []
    assert len(list(McapReader(path).iter_json_messages(topics=[RAW_LIDAR_TOPIC]))) == 1
    replay = replay_capture(path, project_root=ROOT)
    assert replay["status"] == "MATCH", replay["diagnostics"]


@pytest.mark.parametrize("case", ("occupied", "empty", "missing"))
def test_actual_l4_grid_has_direct_spawn_parity_and_one_existing_enqueue(tmp_path, case):
    from v3.local_costmap_capture import LOCAL_COSTMAP_TOPIC, pack_local_costmap, unpack_local_costmap
    from v3_process_runtime import McapCaptureSession

    record, grid = _grid_record(missing=case == "missing", empty=case == "empty")
    expected = unpack_local_costmap(pack_local_costmap(record))
    expected_cells = [] if grid is None else [[c.grid_x, c.grid_y, c.observation_count]
                                             for c in grid.occupied_cells]
    assert expected["occupied_cells"] == expected_cells
    assert expected["available"] is (case != "missing")
    payloads, ticks = [], []
    for cls, name in ((McapCaptureSession, "direct"), (ProcessMcapCaptureSession, "process")):
        output = tmp_path / (name + ".mcap")
        session = cls(name, output, configuration={}, project_root=tmp_path,
            config=McapCaptureConfig(mode="append_only", tick_sample_hz=5, navigation_map_hz=5))
        session.start()
        try:
            session.observe(record)
            observation = _map(_scan(1, record.inputs.context.monotonic_ns))
            if name == "process":
                assert session._enqueued_count == 1
                session.raw_lidar_queue.put(("navigation_map", observation), timeout=2.)
                session.raw_lidar_queue.put(("raw_end", 1, 1, 0), timeout=2.)
            else:
                session.hub.publish(observation, topic=NAVIGATION_MAP_TOPIC)
                session.hub.publish({"event_type": "navigation_map_transport_end",
                    "last_revision": 1, "produced_count": 1, "superseded_count": 0},
                    topic="v3.navigation_map_transport")
        finally:
            path = session.finalize(SimpleNamespace(status=0))
        rows, integrity = _events(path)
        costmaps = [(message, row) for message, row in rows if row.get("source_topic") == LOCAL_COSTMAP_TOPIC]
        assert len(costmaps) == 1
        message, row = costmaps[0]
        assert message.log_time_ns == record.inputs.context.monotonic_ns
        assert row["payload"] == expected
        assert row["payload"]["occupied_cells"] == expected_cells
        assert integrity["complete"] and integrity["local_costmap_snapshot_count"] == 1
        assert integrity["local_costmap_available_count"] == int(case != "missing")
        assert McapReader(path).capture_integrity()["integrity"]["complete"]
        payloads.append(row["payload"])
        ticks.append([row for _, row in McapReader(path).iter_json_messages(topics=["/r2b4/tick"])])
        if grid is not None:
            assert "occupied_cells" not in ticks[-1][0]["expected"]["layers"]["L4"]["local_costmap"]
    assert payloads[0] == payloads[1] and ticks[0] == ticks[1]
    if case == "occupied":
        compiled = compile_evidence(tmp_path / "process.mcap", tmp_path / "actual-grid.evidence", workers=1)
        assert compiled["compiler_status"] == "COMPLETE"
        verify(compiled["output"], source=tmp_path / "process.mcap")
        exported = [row["payload"]["payload"] for row in query(compiled["output"], topic=EVENT_TOPIC)
                    if row["payload"].get("source_topic") == LOCAL_COSTMAP_TOPIC]
        assert exported == [expected]


@pytest.mark.parametrize("kind", ("direct", "process"))
def test_ten_hz_capture_packs_at_most_five_physical_grid_buckets_even_with_forced_faults(tmp_path, monkeypatch, kind):
    from v3 import process_sidecars
    from v3.contracts import LifecycleState, TickContext
    from v3.execution import WriterFailureRecord
    from v3.local_costmap_capture import LOCAL_COSTMAP_TOPIC, pack_local_costmap
    import v3_process_runtime

    packed, enqueued = [], []

    def counted(record):
        packed.append(record.inputs.context if getattr(record, "inputs", None) is not None else record.context)
        return pack_local_costmap(record)

    module = process_sidecars if kind == "process" else v3_process_runtime
    cls = module.ProcessMcapCaptureSession if kind == "process" else module.McapCaptureSession
    monkeypatch.setattr(module, "pack_local_costmap", counted)
    session = cls(kind, tmp_path / (kind + ".mcap"), configuration={}, project_root=tmp_path,
        config=McapCaptureConfig(mode="append_only", tick_sample_hz=10, navigation_map_hz=5))
    if kind == "process":
        monkeypatch.setattr(ProcessMcapCaptureSession, "_enqueue",
            lambda self, key, value: enqueued.append((key, value)) or True)
    else:
        subscription = session.hub.subscribe_reliable("cadence-test", capacity=64, required=True)
    record, _ = _grid_record()
    anchor = record.inputs.context.monotonic_ns // 1_000_000_000 * 1_000_000_000
    for tick, offset in enumerate((0, 50_000_000, 100_000_000, 150_000_000, 200_000_000, 250_000_000)):
        context = TickContext(tick, anchor + offset)
        session.observe(_retime_record(record, tick, context.monotonic_ns))
        session.observe(WriterFailureRecord(context, LifecycleState.IDLE, "OFFLINE_TEST", None))
    assert [context.monotonic_ns for context in packed] == [anchor, anchor + 200_000_000]
    if kind == "process":
        assert len(enqueued) == 12
        assert sum(key == "behavior_with_map" for key, _ in enqueued) == 2
        assert all(type(value[1].cells) is bytes for key, value in enqueued if key == "behavior_with_map")
    else:
        frames = subscription.drain()
        assert sum(frame.topic == LOCAL_COSTMAP_TOPIC for frame in frames) == 2
        assert sum(frame.topic == "v3.capture_record" for frame in frames) == 12
        session.hub.close()


@pytest.mark.parametrize("kind", ("direct", "process"))
@pytest.mark.parametrize("hz", (5, 50))
def test_grid_packer_is_never_called_without_sampled_map_demand(tmp_path, monkeypatch, kind, hz):
    from v3 import process_sidecars
    import v3_process_runtime

    module = process_sidecars if kind == "process" else v3_process_runtime
    cls = module.ProcessMcapCaptureSession if kind == "process" else module.McapCaptureSession
    monkeypatch.setattr(module, "pack_local_costmap",
        lambda record: pytest.fail("grid packing without sampled map demand"))
    session = cls(kind, tmp_path / (kind + ".mcap"), configuration={}, project_root=tmp_path,
        config=McapCaptureConfig(mode="append_only", tick_sample_hz=hz, navigation_map_hz=0))
    if kind == "process":
        monkeypatch.setattr(ProcessMcapCaptureSession, "_enqueue", lambda *args: True)
    session.observe(closed_stop_record())
    if kind == "direct":
        session.hub.close()
