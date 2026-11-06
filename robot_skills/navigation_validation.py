"""One bounded live navigation observation through the injected async SDK.

The caller starts FULL 50 Hz capture and finalizes it after this skill's terminal
event. Offline EVI/DIAG/replay and independent physical measurements are separate
steps; a completed Python program is never a navigation acceptance verdict.
"""
from __future__ import annotations

import asyncio
import math
import time


POLL_S = 0.1
READ_TIMEOUT_S = 0.5
STATUS_MAX_AGE_NS = 999_000_000
RUNNING = frozenset(("STARTING", "ACTIVE"))
CRITICAL_DEVICES = frozenset(("WHEEL_ENCODERS", "BNO055_IMU", "RPLIDAR_C1"))


def _number(value, name, lo, hi):
    if type(value) not in (int, float) or not math.isfinite(value) or not lo <= value <= hi:
        raise ValueError(f"{name} must be in {lo}..{hi}")
    return float(value)


async def _read(robot, resource):
    return await asyncio.wait_for(robot.read(resource), READ_TIMEOUT_S)


def _pose(status):
    estimate = status.get("estimate") or {}
    pose = estimate.get("local_pose") or {}
    if pose.get("frame_id") != "R2B4_ODOM_LOCAL" or any(
            type(pose.get(key)) not in (int, float) or not math.isfinite(pose[key])
            for key in ("x_m", "y_m", "yaw_rad")):
        raise RuntimeError("LOCAL_POSE_UNAVAILABLE")
    return {key: pose[key] for key in ("x_m", "y_m", "yaw_rad", "frame_id")}


def _check_status(status, *, generation=None, previous=None):
    if not isinstance(status, dict):
        raise RuntimeError("STATUS_MISSING")
    stamp, tick = status.get("monotonic_ns"), status.get("tick_id")
    age = time.monotonic_ns() - stamp if type(stamp) is int else None
    if type(tick) is not int or age is None or not 0 <= age < STATUS_MAX_AGE_NS:
        raise RuntimeError("STATUS_STALE_OR_INVALID")
    if previous is not None and (tick < previous[0] or stamp < previous[1]
                                 or tick == previous[0] and stamp != previous[1]):
        raise RuntimeError("STATUS_TIME_OR_TICK_REGRESSED")
    if status.get("state") != "RUNNING" or status.get("fault_layer") is not None:
        raise RuntimeError("RUNTIME_FAULT:" + str(status.get("fault_layer")))
    if status.get("safety_decision") not in ("ALLOW", "STOP"):
        raise RuntimeError("SAFETY_FAULT_OR_MISSING")
    health = {item.get("device_id"): item.get("state")
              for item in status.get("source_health", []) if isinstance(item, dict)}
    if any(health.get(device) != "OK" for device in CRITICAL_DEVICES):
        raise RuntimeError("CRITICAL_SOURCE_HEALTH:" + repr(health)[:220])
    quality = (status.get("estimate") or {}).get("localization_quality") or {}
    actual_generation = quality.get("generation")
    permitted_quality = ("GOOD",) if generation is None else ("GOOD", "DEGRADED")
    if (quality.get("local_translation") not in permitted_quality or quality.get("heading") not in permitted_quality
            or quality.get("local_pose_continuous") is not True
            or quality.get("pose_discontinuity") is not False
            or type(actual_generation) is not int):
        raise RuntimeError("LOCALIZATION_UNUSABLE")
    if generation is not None and actual_generation != generation:
        raise RuntimeError("LOCALIZATION_GENERATION_CHANGED")
    _pose(status)
    return actual_generation, (tick, stamp), age


def _check_capture(capture, expected_path=None):
    if (not isinstance(capture, dict) or capture.get("mode") != "full"
            or capture.get("hz") != 50 or capture.get("state") != "RECORDING"
            or not isinstance(capture.get("path"), str) or not capture["path"]):
        raise RuntimeError("FULL_50HZ_RECORDING_REQUIRED")
    if expected_path is not None and capture["path"] != expected_path:
        raise RuntimeError("CAPTURE_IDENTITY_CHANGED")
    return {key: capture.get(key) for key in ("path", "mode", "hz", "state")}


async def _snapshot(robot, pid, generation, capture_path, previous=None):
    operator = await _read(robot, "operator.status")
    if (not isinstance(operator, dict) or operator.get("runtime_running") is not True
            or operator.get("runtime_pid") != pid):
        raise RuntimeError("RUNTIME_SESSION_CHANGED")
    if operator.get("capture_mode") != "full" or operator.get("capture_hz") != 50:
        raise RuntimeError("RUNTIME_CAPTURE_SCOPE_CHANGED")
    _check_capture(await _read(robot, "capture.status"), capture_path)
    status = await _read(robot, "v3.status")
    actual_generation, identity, age = _check_status(status, generation=generation, previous=previous)
    return status, actual_generation, identity, age


async def _spatial(robot):
    """Keep bounded source values, never turn historical coordinates into goals."""
    result = {}
    for kind, limit, field in (("coverage", 16, "coverage"), ("frontiers", 4, "frontier_indexes")):
        value = await asyncio.wait_for(robot.spatial_query(kind=kind, limit=limit), READ_TIMEOUT_S)
        if not isinstance(value, dict) or not isinstance(value.get(field), list):
            raise RuntimeError("SPATIAL_RESPONSE_INVALID:" + kind)
        result[field] = value[field]
        result[kind + "_truncated"] = value.get("truncated", False)
        if "clock_epoch" in result and result["clock_epoch"] != value.get("clock_epoch"):
            raise RuntimeError("SPATIAL_CLOCK_EPOCH_CHANGED")
        result.update({key: value.get(key) for key in ("revision", "clock_epoch", "observation_time_ns")})
    return result


async def _wait_idle(robot, pid, generation, capture_path, previous=None):
    deadline = time.monotonic_ns() + 2_000_000_000
    while True:
        status, _, identity, _ = await _snapshot(robot, pid, generation, capture_path, previous)
        if (status.get("enabled") is False and status.get("left_output") == 0
                and status.get("right_output") == 0 and status.get("safety_decision") == "STOP"):
            return status, identity
        if time.monotonic_ns() >= deadline:
            raise RuntimeError("CANONICAL_STOP_NOT_CONFIRMED")
        previous = identity
        await asyncio.sleep(POLL_S)


async def _observe_behavior(robot, phase, *, pid, generation, capture_path, budget_s,
                            max_radial_m, max_estimated_path_m):
    deadline = time.monotonic_ns() + round(budget_s * 1e9)
    previous = last_observed_ns = last_goal = None
    last_pose = phase["initial_pose"]
    while True:
        status, _, identity, age = await _snapshot(robot, pid, generation, capture_path, previous)
        state = await _read(robot, "behavior.state")
        now = time.monotonic_ns()
        if not isinstance(state, dict) or state.get("behavior_id") != phase["behavior_id"]:
            raise RuntimeError("BEHAVIOR_REPLACED")
        if state.get("command_id") != phase["command_id"] or state.get("mission_id") != phase["mission_id"]:
            raise RuntimeError("BEHAVIOR_COMMAND_IDENTITY_CHANGED")
        phase.update(lifecycle=state.get("lifecycle"), reason=state.get("reason"))
        if state.get("lifecycle") == "COMPLETED":
            if phase["allow_observations"] < 2:
                raise RuntimeError("INSUFFICIENT_ACTIVE_OBSERVATIONS")
            if phase["name"] == "room_cruise":
                raise RuntimeError("CRUISE_ENDED_BEFORE_OBSERVATION_WINDOW")
            stopped, _ = await _wait_idle(robot, pid, generation, capture_path, identity)
            phase.update(end_monotonic_ns=now, final_pose=_pose(stopped), stop_confirmed=True,
                         behavior_result=state.get("result", {}),
                         outcome="TARGET_COMPLETION")
            return
        if state.get("lifecycle") not in RUNNING:
            raise RuntimeError("BEHAVIOR_TERMINATED:" + str(state.get("reason")))
        if identity != previous:
            mission = status.get("mission") or {}
            navigation = status.get("navigation") or {}
            if (mission.get("mission_id") != phase["mission_id"]
                    or navigation.get("mission_id") != phase["mission_id"]):
                raise RuntimeError("MISSION_OR_NAVIGATION_IDENTITY_CHANGED")
            if status.get("safety_decision") != "ALLOW":
                raise RuntimeError("UNEXPECTED_SAFETY_STOP:" + str(status.get("safety_reason")))
            pose = _pose(status)
            phase["observations"] += 1
            phase["allow_observations"] += 1
            phase["first_tick_id"] = identity[0] if previous is None else phase["first_tick_id"]
            phase["last_tick_id"] = identity[0]
            phase["max_status_age_ns"] = max(phase["max_status_age_ns"], age)
            quality = status["estimate"]["localization_quality"]
            phase["degraded_quality_observations"] += (quality["local_translation"] == "DEGRADED"
                                                        or quality["heading"] == "DEGRADED")
            if last_observed_ns is not None:
                phase["max_observation_gap_ns"] = max(phase["max_observation_gap_ns"], now - last_observed_ns)
            if last_pose is not None:
                phase["estimated_path_m"] += math.hypot(pose["x_m"] - last_pose["x_m"], pose["y_m"] - last_pose["y_m"])
            origin = phase["initial_pose"]
            radial_m = math.hypot(pose["x_m"] - origin["x_m"], pose["y_m"] - origin["y_m"])
            phase["max_estimated_radial_m"] = max(phase["max_estimated_radial_m"], radial_m)
            goal = navigation.get("goal_selected_ns")
            if type(goal) is int and goal != last_goal:
                phase["goal_identity_changes"] += last_goal is not None
                last_goal = goal
            last_pose, last_observed_ns = pose, now
            if radial_m >= max_radial_m or phase["estimated_path_m"] >= max_estimated_path_m:
                await asyncio.wait_for(robot.call("behavior.cancel", reason="NAVIGATION_VALIDATION_BOUND_REACHED"), 5)
                stopped, _ = await _wait_idle(robot, pid, generation, capture_path, identity)
                phase.update(end_monotonic_ns=now, final_pose=_pose(stopped), stop_confirmed=True,
                             outcome="BOUND_REACHED", lifecycle="CANCELLED", reason="BOUND_REACHED")
                return
            if (phase["name"] == "room_cruise" and now - phase["observation_started_ns"]
                    >= round(phase["requested_observation_s"] * 1e9)):
                if phase["allow_observations"] < 2:
                    raise RuntimeError("INSUFFICIENT_ACTIVE_OBSERVATIONS")
                await asyncio.wait_for(robot.call("behavior.cancel", reason="NAVIGATION_VALIDATION_WINDOW_END"), 5)
                stopped, _ = await _wait_idle(robot, pid, generation, capture_path, identity)
                phase.update(end_monotonic_ns=now, final_pose=_pose(stopped), stop_confirmed=True,
                             outcome="TIMED_OBSERVATION_COMPLETE", lifecycle="CANCELLED",
                             reason="NAVIGATION_VALIDATION_WINDOW_END")
                return
        previous = identity
        if now >= deadline:
            raise TimeoutError("BEHAVIOR_COMPLETION_TIMEOUT")
        await asyncio.sleep(POLL_S)


async def run(robot, *, cruise_s=20.0, max_v_mps=0.20, max_omega_rad_s=1.10,
              candidate_places=None, strategic_budget_s=30.0,
              max_radial_m=1.25, max_estimated_path_m=3.0):
    """Observe one finite cruise, optionally one already-current strategic place.

    Requires an idle runtime and FULL 50 Hz capture prepared by the caller.
    Runtime shutdown/finalization belongs to the caller, after the skill terminal
    event, so offline proof can bind the exact source and complete run interval.
    """
    cruise_s = _number(cruise_s, "cruise_s", 2, 60)
    max_v_mps = _number(max_v_mps, "max_v_mps", .15, .25)
    max_omega_rad_s = _number(max_omega_rad_s, "max_omega_rad_s", .1, 1.4)
    strategic_budget_s = _number(strategic_budget_s, "strategic_budget_s", 2, 60)
    max_radial_m = _number(max_radial_m, "max_radial_m", .1, 1.25)
    max_estimated_path_m = _number(max_estimated_path_m, "max_estimated_path_m", .1, 3.0)
    if candidate_places is not None and (not isinstance(candidate_places, (tuple, list))
            or len(candidate_places) != 1 or not isinstance(candidate_places[0], str)
            or not 0 < len(candidate_places[0]) <= 256):
        raise ValueError("candidate_places must contain exactly one existing current place identity")

    result = {"schema": "R2B4_NAVIGATION_LIVE_VALIDATION_V1", "status": "FAILED",
              "phase": "PREFLIGHT", "reason": None, "runtime_pid": None,
              "localization_generation": None, "run_id": getattr(robot, "invocation_id", None),
              "source_hash": None, "skill_started_ns": None,
              "observed_start_ns": time.monotonic_ns(), "phases": [],
              "parameters": {"cruise_s": cruise_s, "max_v_mps": max_v_mps,
                             "max_omega_rad_s": max_omega_rad_s, "max_radial_m": max_radial_m,
                             "max_estimated_path_m": max_estimated_path_m},
              "strategic": {"status": "NOT_EXERCISED" if candidate_places is None else "REQUESTED"},
              "validation": {"status": "PENDING_OFFLINE"}, "replay": {"status": "NOT_RUN"},
              "physical": {"status": "NOT_INDEPENDENTLY_MEASURED"}}
    own_id = None
    motion_attempted = False
    try:
        operator = await _read(robot, "operator.status")
        if (not isinstance(operator, dict) or operator.get("runtime_running") is not True
                or type(operator.get("runtime_pid")) is not int):
            raise RuntimeError("CALLER_MUST_PREPARE_RUNTIME")
        pid = result["runtime_pid"] = operator["runtime_pid"]
        capture = _check_capture(await _read(robot, "capture.status"))
        result["capture"] = capture
        status, generation, _, _ = await _snapshot(robot, pid, None, capture["path"])
        result["localization_generation"] = generation
        if (status.get("ready_for_active") is not True or status.get("enabled") is not False
                or status.get("left_output") != 0 or status.get("right_output") != 0):
            raise RuntimeError("IDLE_READY_ZERO_OUTPUT_REQUIRED")
        existing = await _read(robot, "behavior.state")
        if not isinstance(existing, dict) or existing.get("lifecycle") in RUNNING:
            raise RuntimeError("EXISTING_BEHAVIOR_MUST_FINISH_FIRST")
        if result["run_id"] is not None:
            source = await robot.call("skill.status", run_id=result["run_id"])
            if source.get("run_id") != result["run_id"] or not isinstance(source.get("source_hash"), str):
                raise RuntimeError("SKILL_SOURCE_BINDING_UNAVAILABLE")
            result.update(source_hash=source["source_hash"], skill_started_ns=source.get("started_ns"))
        result["spatial_before"] = await _spatial(robot)
        result["clock_epoch"] = result["spatial_before"]["clock_epoch"]
        plans = [("room_cruise", cruise_s)]
        if candidate_places is not None:
            plans.append(("strategic_explore", strategic_budget_s))
        for name, duration in plans:
            result["phase"] = name.upper()
            phase = {"name": name, "start_monotonic_ns": time.monotonic_ns(),
                     "observations": 0, "allow_observations": 0, "estimated_path_m": 0.0,
                     "max_status_age_ns": 0, "max_observation_gap_ns": 0,
                     "goal_identity_changes": 0, "stop_confirmed": False,
                     "degraded_quality_observations": 0, "max_estimated_radial_m": 0.0,
                     "initial_pose": _pose(status), "max_radial_m": max_radial_m,
                     "max_estimated_path_m": max_estimated_path_m}
            result["phases"].append(phase)
            motion_attempted = True
            # SDK behavior starts use a watchdog, not completion-on-duration.
            # The observation window ends by an explicit owned canonical cancel.
            params = {"max_duration_s": duration + 8 if name == "room_cruise" else duration,
                      "max_v_mps": max_v_mps,
                      "max_omega_rad_s": max_omega_rad_s}
            if name == "room_cruise":
                params.update(capture=False, capture_mode="full", capture_hz=50)
                started = await robot.call("behavior.room_cruise", **params)
            else:
                params.update(name=name, candidate_places=list(candidate_places), max_targets=1,
                              target_budget_s=duration)
                started = await robot.call("behavior.start", **params)
            if not isinstance(started, dict):
                raise RuntimeError("BEHAVIOR_START_RESPONSE_INVALID")
            own_id = started.get("behavior_id")
            phase.update({key: started.get(key) for key in ("behavior_id", "command_id", "mission_id", "lifecycle", "reason")})
            if (started.get("lifecycle") not in RUNNING or any(not isinstance(started.get(key), str)
                    or not started[key] for key in ("behavior_id", "command_id", "mission_id"))):
                raise RuntimeError("BEHAVIOR_START_REJECTED:" + str(started.get("reason")))
            phase.update(observation_started_ns=time.monotonic_ns(), requested_observation_s=duration)
            await _observe_behavior(robot, phase, pid=pid, generation=generation,
                                    capture_path=capture["path"], budget_s=duration + 12,
                                    max_radial_m=max_radial_m, max_estimated_path_m=max_estimated_path_m)
            if phase["outcome"] == "BOUND_REACHED":
                result["measurement_outcome"] = "BOUND_REACHED"
                break
            if name == "strategic_explore":
                result["strategic"] = {"status": "OBSERVED_COMPLETED", "candidate_places": list(candidate_places)}
            status, _, _, _ = await _snapshot(robot, pid, generation, capture["path"])
        result["spatial_after"] = await _spatial(robot)
        if result["spatial_after"]["clock_epoch"] != result["clock_epoch"]:
            raise RuntimeError("SPATIAL_CLOCK_EPOCH_CHANGED")
        result.update(status="COMPLETED", phase="AWAITING_CALLER_FINALIZATION")
    except asyncio.CancelledError:
        raise
    except Exception as exc:
        result["reason"] = f"{type(exc).__name__}:{exc}"[:450]
    finally:
        if motion_attempted:
            try:
                state = await _read(robot, "behavior.state")
                if state.get("lifecycle") in RUNNING and (own_id is None or state.get("behavior_id") == own_id):
                    await asyncio.wait_for(robot.call("behavior.cancel", reason="NAVIGATION_VALIDATION_CLEANUP"), 5)
            except Exception as exc:
                result.update(status="FAILED", cleanup_error=f"{type(exc).__name__}:{exc}"[:300])
                # Canonical last resort: the host can revoke this worker too.
                await robot.stop()
        result["observed_end_ns"] = time.monotonic_ns()
        if result["phases"] and "end_monotonic_ns" not in result["phases"][-1]:
            result["phases"][-1]["end_monotonic_ns"] = result["observed_end_ns"]
    return result
