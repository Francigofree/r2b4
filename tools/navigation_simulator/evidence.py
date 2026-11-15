"""MCAP entry points reuse native integrity, compiler, DIAG and Replayer."""
from __future__ import annotations

from collections import Counter, deque
import hashlib
import json
import math
from pathlib import Path
import subprocess

from .scenario import SCHEMA, builtin, validate


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def file_hash(path):
    result = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            result.update(block)
    return result.hexdigest()


def write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n")


def source_snapshot(root):
    """Follow the canonical runtime import closure, including working-tree edits."""
    from v3.replay import _source_first_paths
    root = Path(root)
    paths = set(_source_first_paths(root))
    paths.update(str(path.relative_to(root)) for path in (root / "tools/navigation_simulator").glob("*.py"))
    paths.update(("tools/nav2_navigation_probe.py", "tools/nav2_planner.py", "tools/slam_toolbox.py",
                  "r2b4_orchestration/motion_simulation.py", "tools/tuners/r2b4_navigation_autotuner.py",
                  "tools/tuners/r2b4_motion_tuner.py", "conf/nav2_planner.yaml", "conf/slam_toolbox.yaml",
                  "conf/nav2_navigate.xml", "conf/frontier_exploration.yaml", "conf/nav2_fastdds.xml"))
    # ROS algorithms and the wheel-aware adapter include compiled production
    # code; a rebuilt binary must invalidate comparison evidence as well.
    for relative in ("runtime/deps/nav2-wheel-adapter/lib/libr2b4_nav2_wheel_adapter.so",
                     "runtime/deps/nav2-planner/lib/libnav2_smac_planner.so",
                     "runtime/deps/nav2-planner/lib/libmppi_controller.so",
                     "runtime/deps/nav2-planner/lib/libmppi_critics.so",
                     "runtime/deps/slam-toolbox/lib/libasync_slam_toolbox.so",
                     "runtime/deps/nav2-planner/lib/nav2_controller/controller_server",
                     "runtime/deps/nav2-planner/lib/nav2_planner/planner_server",
                     "runtime/deps/slam-toolbox/lib/slam_toolbox/async_slam_toolbox_node",
                     "runtime/deps/frontier-exploration/lib/explore_lite/explore"):
        if (root/relative).is_file():
            paths.add(relative)
    files = {path: file_hash(root / path) for path in sorted(paths) if (root / path).is_file()}
    def git(*args):
        result = subprocess.run(["git", *args], cwd=root, capture_output=True, text=True, timeout=5)
        return result.stdout.strip() if result.returncode == 0 else None
    return {"sha256": digest(files), "files": files, "git_head": git("rev-parse", "HEAD"),
            "git_status": git("status", "--short"), "scope": "current runtime import closure, simulator and navigation configs"}


def replay(capture, root, output):
    from v3.replay import replay_capture, V3ReplayError, write_replay_result
    from v3.mcap_replay_bridge import McapReplayBridgeError, ReplayWindow
    from v3.mcap_reader import McapReader, McapReadError
    if Path(capture).suffix == ".mcap":
        try:
            count = int(McapReader(capture).latest_metadata("r2b4.capture.final").get("tick_count", 0))
        except (McapReadError, ValueError):
            count = 0  # The canonical reader below owns the actual rejection.
        if count > ReplayWindow().max_materialized_ticks:
            result = _replay_windows(capture, root, output)
            write(output, result)
            return result
    try:
        result = replay_capture(capture, project_root=root)
    except (V3ReplayError,McapReplayBridgeError) as exc:
        if "bounded materialization budget" in str(exc) and Path(capture).suffix == ".mcap":
            result = _replay_windows(capture, root, output)
        else:
            result = {"status": "BLOCKED", "reason": str(exc), "physical_output": False}
        write(output, result)
    else:
        write_replay_result(result, output)
    return result


def _replay_windows(capture, root, output):
    """Use canonical checkpoint slices without raising the native memory bound."""
    from v3.mcap_reader import McapReader, TICK_TOPIC
    from v3.mcap_replay_bridge import McapReplayBridgeError
    from v3.replay import ReplaySelection, replay_capture, V3ReplayError, write_replay_result
    windows = []
    authority_sha256 = file_hash(capture)
    start = end = None
    count = size = total = 0
    ranges = []
    previous = None
    for message in McapReader(capture).iter_messages(topics=(TICK_TOPIC,)):
        if previous is not None and message.sequence != previous+1:
            return dict(status="BLOCKED", reason="REPLAY_TICK_SEQUENCE_GAP", physical_output=False)
        previous = message.sequence
        if count and (count >= 1000 or size+len(message.data) > 32*1024*1024):
            ranges.append((start, end, count))
            start, count, size = None, 0, 0
        start = message.sequence if start is None else start
        end = message.sequence
        count, size = count+1, size+len(message.data)
    if count:
        ranges.append((start, end, count))
    for start, end, count in ranges:
        path = Path(output).with_name(f"{Path(output).stem}_ticks_{start}_{end}.json")
        try:
            result = replay_capture(capture, project_root=root,
                selection=ReplaySelection(start_tick_id=start, end_tick_id=end))
        except (V3ReplayError, McapReplayBridgeError) as exc:
            result = dict(status="BLOCKED", reason=str(exc), physical_output=False)
        if result["status"] == "MATCH":
            scope = result.get("scope", {}).get("resolved", {})
            if (result.get("capture", {}).get("sha256") != authority_sha256
                    or scope.get("first_tick_id") != start or scope.get("last_tick_id") != end
                    or scope.get("tick_count") != count
                    or result.get("determinism", {}).get("selected_tick_count") != count):
                result = dict(status="BLOCKED", reason="REPLAY_WINDOW_COVERAGE_OR_SOURCE_MISMATCH", physical_output=False)
        write_replay_result(result, path)
        windows.append(dict(start_tick_id=start, end_tick_id=end, tick_count=count,
            status=result["status"], result=str(path), reason=result.get("reason")))
        total += count
        if result["status"] != "MATCH":
            break
    return dict(schema="R2B4_NATIVE_REPLAY_WINDOWS_V1", physical_output=False,
        status="MATCH" if windows and len(windows)==len(ranges)
            and all(w["status"]=="MATCH" for w in windows) else windows[-1]["status"] if windows else "BLOCKED",
        scope="all decision ticks, independent canonical checkpoint slices",
        continuous_state_comparison=False,
        capture=str(Path(capture).resolve()), source_sha256=authority_sha256,
        selected_tick_count=total, windows=windows)


def analyze(capture, root, output, *, compile_reports=True):
    from v3.mcap_inspect import inspect_mcap
    from v3.mcap_reader import McapReader, TICK_TOPIC, EVENT_TOPIC
    capture, output = Path(capture), Path(output)
    inspection = inspect_mcap(capture, deep=True)
    result = {"schema": "R2B4_LOCAL_NAVIGATION_ANALYSIS_V1", "source": str(capture.resolve()),
              "source_sha256": file_hash(capture), "inspection": inspection, "physical_output": False,
              "findings": [], "reports": {}, "analysis_source_sha256":source_snapshot(root)["sha256"]}
    if inspection["status"] != "PASS" or inspection["final_metadata"].get("complete")!="true":
        result.update(status="BLOCKED", reason="CAPTURE_INTEGRITY_FAILURE")
        write(output / "analysis.json", result)
        return result
    reader = McapReader(capture)
    counts = Counter()
    prior = None
    first = last = None
    path_length = 0.
    sample_count = 0
    findings = result["findings"]
    # Stream compact scalar timeline; full payloads remain in MCAP/EVI.
    with (output / "timeline.ndjson").open("w") as timeline:
        for message, row in reader.iter_json_messages(topics=(TICK_TOPIC, EVENT_TOPIC)):
            if message.topic == EVENT_TOPIC:
                if row.get("source_topic") != "v3.navigation_decision":
                    continue
                row = row.get("payload", {})
            layers = row.get("expected", {}).get("layers", {})
            if not layers:
                continue
            timestamp = row.get("monotonic_ns", message.log_time_ns)
            tick = row.get("tick_id", message.sequence)
            nav, mission, final = (layers.get(key, {}) for key in ("L6", "L5", "L12"))
            estimate = layers.get("L3", {})
            pose = estimate.get("global_pose") or estimate.get("local_pose")
            item = {"tick_id": tick, "measurement_monotonic_ns": timestamp, "topic": message.topic,
                    "mission_id": mission.get("mission_id"), "mode": mission.get("mode"),
                    "navigation_status": nav.get("status"), "navigation_reason": nav.get("reason"),
                    "goal": nav.get("local_goal"), "safety": final.get("safety_decision"),
                    "safety_reason": final.get("reason"), "fault_layer": row.get("expected", {}).get("fault_layer"),
                    "pose": pose}
            timeline.write(json.dumps(item, allow_nan=False) + "\n")
            counts[str(nav.get("reason"))] += 1
            if message.topic != TICK_TOPIC:
                continue
            sample_count += 1
            first = timestamp if first is None else first
            last = timestamp
            if pose and prior and timestamp > prior[0]:
                old = prior[1]
                if pose.get("frame_id") == old.get("frame_id"):
                    path_length += math.hypot(pose["x_m"]-old["x_m"], pose["y_m"]-old["y_m"])
            if pose:
                prior = timestamp, pose
            if len(findings) < 64 and (item["fault_layer"] or final.get("safety_decision") == "FAULT"):
                findings.append({"kind": "RECORDED_FAULT", "tick_id": tick, "monotonic_ns": timestamp,
                                 "layer": item["fault_layer"] or "L12", "value": final.get("reason"), "causal_proof": False})
    span = (last-first)/1e9 if first is not None and last is not None else 0.
    result.update(status="PASS", timeline=str(output / "timeline.ndjson"),
                  measurements={"sample_count": sample_count, "duration_s": span,
                                "sample_rate_hz": (sample_count-1)/span if span else None,
                                "sampled_estimated_path_length_m": path_length,
                                "navigation_reasons": dict(counts)},
                  replay_eligible=inspection["final_metadata"].get("replay_complete") == "true",
                  limitations=["Sampled estimates are not physical ground truth or a counterfactual sensor stream.",
                               "Findings identify recorded boundaries; upstream cause requires causal evidence."])
    if compile_reports:
        from tools.mcap_evidence.compiler import compile_evidence
        from tools.diag.context import open_context
        from tools.diag.registry import build_default_registry
        compiled = compile_evidence(capture, output / "evidence", workers=1)
        result["compiler"] = compiled
        context = open_context(Path(root), compiled["output"])
        registry = build_default_registry()
        for name in ("navigation", "navigation_root_cause", "motion_performance"):
            report = registry.run(name, context).as_dict()
            write(output / (name + ".json"), report)
            result["reports"][name] = str(output / (name + ".json"))
        if compiled["compiler_status"] != "COMPLETE":
            result.update(status="BLOCKED", reason="EVIDENCE_EXPORT_INCOMPLETE")
    write(output / "analysis.json", result)
    return result


def scenario_from_mcap(capture):
    """Reconstruct occupied cells only from scan-bound passive map observations.

    This yields a labelled approximate experiment, never invented replay input.
    A missing observation/pose association is an explicit blocker.
    """
    from v3.mcap_inspect import inspect_mcap
    from v3.mcap_reader import McapReader, TICK_TOPIC, EVENT_TOPIC, RUNTIME_TOPIC
    from v3.navigation_map_capture import validate_navigation_map
    capture = Path(capture)
    inspection = inspect_mcap(capture, deep=True)
    if inspection["status"] != "PASS":
        raise ValueError("MCAP integrity failed")
    reader = McapReader(capture)
    runtime = reader.first_json(RUNTIME_TOPIC)
    metadata = runtime[1].get("metadata", {}) if runtime else {}
    if metadata.get("source") == "simulation" and isinstance(metadata.get("scenario"), dict):
        original = metadata["scenario"]
        if original.get("schema") == SCHEMA:
            return validate(original)
    poses = deque(maxlen=8)
    observations = []
    incident=None
    for message,row in reader.iter_json_messages(topics=(TICK_TOPIC,EVENT_TOPIC)):
        if message.topic==EVENT_TOPIC:
            if row.get("source_topic")!="v3.navigation_decision":
                continue
            row=row.get("payload",{})
        layers=row.get("expected",{}).get("layers",{})
        fault=row.get("expected",{}).get("fault_layer")
        if fault or layers.get("L12",{}).get("safety_decision")=="FAULT" or layers.get("L6",{}).get("status")=="FAILED":
            incident={"tick_id":row.get("tick_id"),"monotonic_ns":row.get("monotonic_ns",message.log_time_ns),
                      "fault_layer":fault,"navigation_reason":layers.get("L6",{}).get("reason"),
                      "safety_reason":layers.get("L12",{}).get("reason")}
            break
    # First scan-bound scene only; avoids combining moving objects and drift
    # over an entire run into one purported static world.
    window=({"start_ns":max(0,incident["monotonic_ns"]-500_000_000),
             "end_ns":incident["monotonic_ns"]+250_000_000} if incident else {})
    for message, row in reader.iter_json_messages(topics=(TICK_TOPIC, EVENT_TOPIC),**window):
        if message.topic == TICK_TOPIC:
            pose = row.get("expected", {}).get("layers", {}).get("L3", {}).get("global_pose")
            if pose:
                poses.append((row.get("monotonic_ns", message.log_time_ns), pose))
        elif row.get("source_topic") == "v3.navigation_map":
            observation = validate_navigation_map(row.get("payload"))
            measured = observation["measurement_monotonic_ns"]
            prior = [(abs(t-measured), t, pose) for t, pose in poses if t <= measured and measured-t <= 250_000_000]
            if prior and observation["sensor_health"] == "OK":
                _, timestamp, pose = min(prior, key=lambda item: item[0])
                observations.append((observation, timestamp, pose))
                break
    if not observations:
        raise ValueError("SCENARIO_BLOCKED: no healthy navigation_map with a preceding global pose within 250 ms; supply an explicit world")
    observation, timestamp, pose = observations[0]
    cells = sorted({(math.floor(x/.2), math.floor(y/.2)) for x,y in observation["points_xy_m"]
                    if math.hypot(x,y) > .4})
    if not cells:
        raise ValueError("SCENARIO_BLOCKED: no usable occupied endpoints")
    scene = builtin("room")
    scene.update(environment_id="mcap_approximation", behavior="roomcruise",
                 expect={"goal_reached": False, "minimum_path_m": .0, "maximum_stationary_s": 15.},
                 walls=[[x*.2,y*.2,(x+1)*.2,(y+1)*.2] for x,y in cells[:64]],
                 room={"x_min_m": min(-2., min(x*.2 for x,y in cells)-.5),
                       "x_max_m": max(2., max((x+1)*.2 for x,y in cells)+.5),
                       "y_min_m": min(-2., min(y*.2 for x,y in cells)-.5),
                       "y_max_m": max(2., max((y+1)*.2 for x,y in cells)+.5)},
                 provenance={"capture": str(capture.resolve()), "capture_sha256": file_hash(capture),
                             "scan_sequence": observation["source_sequence"],
                             "measurement_monotonic_ns": observation["measurement_monotonic_ns"],
                             "pose_monotonic_ns": timestamp, "pose": pose,
                             "recorded_incident":incident,
                             "reconstruction": "single robot-base endpoint cloud, 0.2 m occupied cells, approximate room bounds",
                             "truncated_cells": max(0,len(cells)-64), "exact_replay": False,
                             "unobserved_space": "assumed empty; requires human review", "causal_reproduction_proven": False})
    return validate(scene)
