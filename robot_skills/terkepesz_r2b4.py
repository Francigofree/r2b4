"""R2B4 Map Surveyor v1 -- LLM-free survey and offline global atlas builder.

Two explicit stages, using the existing R2B4 public Robot SDK and canonical map tool:

  survey: FULL@50Hz already active; Room Cruise followed by stationary scan
          periods. The skill *never* restarts V3, changes the capture mode,
          writes motor commands directly, or claims a map before reconstruction.
  build:  after the operator finalizes the FULL MCAP, run the repository's
          tools/mcap50-to-map.py, validate its report, and optionally load a
          historical atlas reference through spatial.load_atlas.

Generated maps are historical atlases without alignment to the current V3
session. They do not autonomously become live navigation maps.

Install as <repo>/robot_skills/terkepesz_r2b4.py.
"""

from __future__ import annotations

import asyncio
import json
import math
import os
from pathlib import Path
import sys

VERSION = "1.0.0"
_RUNNING = frozenset({"STARTING", "ACTIVE"})
_TERMINAL = frozenset({"COMPLETED", "FAILED", "CANCELLED", "INTERRUPTED"})


def _number(value, label, low, high):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{label} must be finite and in [{low}, {high}]")
    return float(value)


def _source_ok(health, device):
    return any(isinstance(row, dict) and row.get("device_id") == device
               and row.get("state") == "OK" for row in health if isinstance(row, dict))


def _quality(status):
    estimate = status.get("estimate") or {}
    return estimate.get("localization_quality") or {}


def _sequence(status):
    world = status.get("world") or {}
    costmap = world.get("local_costmap") or {}
    return costmap.get("source_sequence") if isinstance(costmap.get("source_sequence"), int) else None


async def _snapshot(robot, *, pid, require_idle=False):
    """Read V3 and refuse a different session or faulted sensor state."""
    operator = await robot.read("operator.status")
    if not isinstance(operator, dict) or operator.get("runtime_running") is not True:
        raise RuntimeError("V3_RUNTIME_NOT_RUNNING")
    if operator.get("runtime_pid") != pid:
        raise RuntimeError("V3_RUNTIME_SESSION_CHANGED")
    status = await robot.read("v3.status")
    if not isinstance(status, dict) or status.get("state") != "RUNNING":
        raise RuntimeError("V3_NOT_RUNNING")
    if status.get("fault_layer") or status.get("safety_decision") == "FAULT":
        raise RuntimeError("V3_FAULT:" + str(status.get("fault_layer") or status.get("safety_reason")))
    health = status.get("source_health") or []
    for device in ("WHEEL_ENCODERS", "BNO055_IMU", "RPLIDAR_C1"):
        if not _source_ok(health, device):
            raise RuntimeError("SENSOR_NOT_OK:" + device)
    q = _quality(status)
    if (q.get("local_translation") == "LOST" or q.get("heading") == "LOST"
            or q.get("pose_discontinuity") is True):
        raise RuntimeError("LOCALIZATION_NOT_USABLE")
    if require_idle and status.get("enabled") is True:
        raise RuntimeError("EXPECTED_INACTIVE_MOTORS")
    return status


async def _stationary_window(robot, pid, seconds, *, interval_s=0.25):
    """Wait and measure stillness without sending any movement commands.

    These samples diagnose the survey; the offline builder independently
    validates 50 Hz stationary windows and decides whether keyframes exist.
    """
    loop = asyncio.get_running_loop()
    end = loop.time() + seconds
    first_seq = last_seq = None
    samples = 0
    good = 0
    best_run = 0.0
    run_started = None
    last_good = None
    while True:
        status = await _snapshot(robot, pid=pid, require_idle=True)
        seq = _sequence(status)
        if first_seq is None:
            first_seq = seq
        last_seq = seq
        e = status.get("estimate") or {}
        v, omega = e.get("v_mps"), e.get("omega_rad_s")
        ok = (type(v) in (int, float) and type(omega) in (int, float)
              and math.isfinite(v) and math.isfinite(omega)
              and abs(v) <= 0.03 and abs(omega) <= 0.05)
        t = loop.time()
        samples += 1
        if ok:
            good += 1
            if run_started is None:
                run_started = t
            last_good = t
            best_run = max(best_run, last_good - run_started)
        else:
            run_started = last_good = None
        left = end - loop.time()
        if left <= 0:
            break
        await asyncio.sleep(min(interval_s, left))
    seq_advanced = (isinstance(first_seq, int) and isinstance(last_seq, int)
                    and last_seq > first_seq)
    return {"samples": samples, "stationary_samples": good,
            "longest_stationary_s": round(best_run, 3),
            "scan_sequence_advanced": seq_advanced,
            "keyframe_candidate": best_run >= 1.2 and seq_advanced,
            "first_seq": first_seq, "last_seq": last_seq}


async def _moving_window(robot, pid, seconds, *, max_v_mps, max_omega_rad_s):
    """Run bounded native Room Cruise then explicitly end it before its deadline."""
    started = await robot.call(
        "behavior.room_cruise", max_duration_s=seconds + 8.0,
        max_v_mps=max_v_mps, max_omega_rad_s=max_omega_rad_s,
    )
    if not isinstance(started, dict) or started.get("lifecycle") not in _RUNNING:
        raise RuntimeError("ROOM_CRUISE_NOT_ACCEPTED:" + str(started))
    identity = started.get("behavior_id")
    if not isinstance(identity, str) or not identity:
        raise RuntimeError("BEHAVIOR_ID_MISSING")
    loop = asyncio.get_running_loop()
    end = loop.time() + seconds
    polls = 0
    reason = None
    try:
        while loop.time() < end:
            await _snapshot(robot, pid=pid)
            state = await robot.read("behavior.state")
            if not isinstance(state, dict) or state.get("behavior_id") != identity:
                raise RuntimeError("BEHAVIOR_IDENTITY_CHANGED")
            lifecycle = state.get("lifecycle")
            if lifecycle in _TERMINAL:
                reason = str(state.get("reason") or lifecycle)
                if lifecycle != "COMPLETED":
                    raise RuntimeError("ROOM_CRUISE_" + lifecycle + ":" + reason)
                break
            polls += 1
            await asyncio.sleep(min(0.35, max(0.0, end - loop.time())))
    finally:
        # This skill-owned behavior.cancel invokes canonical V3 STOP.  Do not
        # call robot.stop(): the global stop also revokes the active skill.
        state = await robot.read("behavior.state")
        if isinstance(state, dict) and state.get("behavior_id") == identity \
                and state.get("lifecycle") in _RUNNING:
            await robot.call("behavior.cancel", reason="SURVEY_STOP_SCAN")
    await _snapshot(robot, pid=pid, require_idle=True)
    return {"behavior_id": identity, "polls": polls,
            "ended_early": reason is not None, "reason": reason}


async def _survey(robot, duration_s, move_s, pause_s, max_v_mps, max_omega_rad_s):
    duration_s = _number(duration_s, "duration_s", 18, 1800)
    move_s = _number(move_s, "move_s", 2, 30)
    pause_s = _number(pause_s, "pause_s", 3.2, 20)
    max_v_mps = _number(max_v_mps, "max_v_mps", 0.03, 0.30)
    max_omega_rad_s = _number(max_omega_rad_s, "max_omega_rad_s", 0.1, 1.2)

    operator = await robot.read("operator.status")
    if not isinstance(operator, dict) or operator.get("runtime_running") is not True:
        return {"status": "FAILED", "reason": "START_V3_FULL_50_FIRST"}
    if operator.get("capture_mode") != "full" or operator.get("capture_hz") != 50:
        return {"status": "FAILED", "reason": "REQUIRES_FULL_50_CAPTURE",
                "actual_mode": operator.get("capture_mode"),
                "actual_hz": operator.get("capture_hz")}
    pid = operator.get("runtime_pid")
    capture = await robot.read("capture.status")
    capture_path = capture.get("path") if isinstance(capture, dict) else None
    if not isinstance(capture_path, str) or not capture_path.endswith(".mcap"):
        return {"status": "FAILED", "reason": "CAPTURE_PATH_UNAVAILABLE"}
    if not isinstance(pid, int):
        return {"status": "FAILED", "reason": "RUNTIME_PID_UNAVAILABLE"}

    stops, moves = [], []
    failure = None
    loop = asyncio.get_running_loop()
    deadline = loop.time() + duration_s
    try:
        await _snapshot(robot, pid=pid, require_idle=True)
        # A useful initial scan is essential for scan-to-map alignment.
        stops.append(await _stationary_window(robot, pid, pause_s))
        while deadline - loop.time() >= move_s + pause_s:
            moves.append(await _moving_window(robot, pid, move_s,
                max_v_mps=max_v_mps, max_omega_rad_s=max_omega_rad_s))
            stops.append(await _stationary_window(robot, pid, pause_s))
        # Use any remaining time for stationary evidence, never initiate a
        # motion phase that cannot finish its following stationary scan.
        remaining = deadline - loop.time()
        if remaining >= 3.2:
            stops.append(await _stationary_window(robot, pid, remaining))
    except asyncio.CancelledError:
        # Skill runtime revocation is handled by the host, including STOP.
        raise
    except Exception as exc:
        failure = f"{type(exc).__name__}:{exc}"[:300]
        try:
            await robot.call("behavior.cancel", reason="SURVEY_FAILED")
        except Exception:
            pass

    candidates = sum(bool(x["keyframe_candidate"]) for x in stops)
    status = "FAILED" if failure else "SURVEY_CAPTURED" if candidates >= 2 else "PARTIAL"
    output = {"status": status, "version": VERSION,
              "runtime_pid": pid, "capture_path": capture_path, "capture_mode": "full",
              "capture_hz": 50, "move_segments": len(moves),
              "stop_scan_windows": len(stops), "candidate_windows": candidates,
              "stops": stops[:32], "moves": moves[:32], "error": failure,
              "next_step": "Stop V3 to finalize MCAP; then run this skill with mode=build and capture_path."
              if not failure else "Review runtime fault before another survey."}
    try:
        await robot.call("hri.report", text=(
            f"Map survey {status}; moves={len(moves)}; candidate_stops={candidates}; "
            f"MCAP={capture_path}; error={failure or 'none'}")[:4000])
    except Exception:
        pass
    return output


async def _build(robot, capture_path, output_dir, register_atlas):
    if not isinstance(capture_path, str) or not capture_path.strip():
        return {"status": "FAILED", "reason": "CAPTURE_PATH_REQUIRED"}
    root = Path(robot.root).resolve()
    builder = root / "tools" / "mcap50-to-map.py"
    if not builder.is_file():
        return {"status": "FAILED", "reason": "CANONICAL_MAP_TOOL_MISSING", "tool": str(builder)}
    capture = Path(capture_path).expanduser().resolve()
    if not capture.is_file() or capture.suffix.lower() != ".mcap":
        return {"status": "FAILED", "reason": "FINALIZED_MCAP_REQUIRED", "path": str(capture)}
    if output_dir is None:
        output = capture.with_name(capture.stem + "_global_map")
    elif isinstance(output_dir, str) and output_dir.strip():
        output = Path(output_dir).expanduser().resolve()
    else:
        return {"status": "FAILED", "reason": "INVALID_OUTPUT_DIR"}
    if output.exists():
        return {"status": "FAILED", "reason": "OUTPUT_EXISTS_NO_OVERWRITE", "path": str(output)}
    if type(register_atlas) is not bool:
        raise ValueError("register_atlas must be boolean")

    command = [sys.executable, str(builder), str(capture), "--output", str(output)]
    try:
        process = await asyncio.create_subprocess_exec(
            *command, cwd=str(root),
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
    except Exception as exc:
        return {"status": "FAILED", "reason": "MAP_TOOL_START_FAILED", "detail": f"{type(exc).__name__}:{exc}"[:300]}
    try:
        stdout, stderr = await process.communicate()
    except asyncio.CancelledError:
        if process.returncode is None:
            process.terminate()
            try:
                await asyncio.wait_for(process.wait(), timeout=3)
            except asyncio.TimeoutError:
                process.kill()
                await process.wait()
        raise
    if process.returncode != 0:
        return {"status": "FAILED", "reason": "MAP_BUILD_REJECTED",
                "returncode": process.returncode,
                "diagnostic": stderr.decode(errors="replace")[-1800:],
                "tool_output": stdout.decode(errors="replace")[-800:],
                "capture_path": str(capture)}

    report_path = output / "report.json"
    atlas_path = output / "atlas_reference.json"
    try:
        report = json.loads(report_path.read_text(encoding="utf-8"))
        atlas = json.loads(atlas_path.read_text(encoding="utf-8"))
        if (report.get("schema") != "R2B4_GLOBAL_MAP_BUILD_V2"
                or atlas.get("schema") != "R2B4_ATLAS_REFERENCE_V1"
                or report.get("evidence_gate", {}).get("strict_zero_loss") is not True):
            raise ValueError("Unexpected report/atlas schema or evidence policy")
    except (OSError, ValueError, KeyError) as exc:
        return {"status": "FAILED", "reason": "MAP_REPORT_INVALID", "detail": str(exc)[:300],
                "output_dir": str(output)}

    reference = None
    load_error = None
    if register_atlas:
        try:
            reference = await robot.call("spatial.load_atlas", path=str(atlas_path))
        except Exception as exc:
            load_error = f"{type(exc).__name__}:{exc}"[:300]

    result = {"status": "PARTIAL" if load_error else "MAP_BUILT",
              "version": VERSION, "output_dir": str(output),
              "map_pgm": str(output / "map.pgm"),
              "map_yaml": str(output / "map.yaml"),
              "map_npz": str(output / "map_data.npz"),
              "report": str(report_path), "atlas_reference": str(atlas_path),
              "map_id": atlas.get("map_id"),
              "keyframes": report.get("stationary", {}).get("usable_keyframes"),
              "graph_edges": report.get("pose_graph", {}).get("edge_count"),
              "observed_area_m2": report.get("map", {}).get("observed_area_m2"),
              "atlas_registered": isinstance(reference, dict) and reference.get("status") == "LOADED",
              "atlas_load_error": load_error,
              "current_alignment": None,
              "live_navigation_map": False}
    try:
        await robot.call("hri.report", text=(
            f"Map {result['status']}; keyframes={result['keyframes']}; "
            f"area_m2={result['observed_area_m2']}; map={result['map_id']}; "
            f"load_error={load_error or 'none'}")[:4000])
    except Exception:
        pass
    return result


async def run(robot, mode="survey", duration_s=60, move_s=7, pause_s=3.5,
              max_v_mps=0.18, max_omega_rad_s=0.55,
              capture_path=None, output_dir=None, register_atlas=True):
    """R2B4 skill-worker entrypoint; LLM-free in both modes."""
    if mode == "survey":
        return await _survey(robot, duration_s, move_s, pause_s,
                             max_v_mps, max_omega_rad_s)
    if mode == "build":
        return await _build(robot, capture_path, output_dir, register_atlas)
    raise ValueError("mode must be survey or build")
