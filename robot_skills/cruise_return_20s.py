"""R2B4: 20s Room Cruise, then navigate to the recorded start pose and yaw.

Place in robot_skills/cruise_return_20s.py; run with `r skill cruise_return_20s --json`.
Requires a running, idle V3; uses only canonical public Robot SDK actions.
An estimated pose is not an independently measured physical position.
"""
from __future__ import annotations

import asyncio
import math
import time

RUNNING = frozenset(("STARTING", "ACTIVE"))
TERMINAL = frozenset(("COMPLETED", "FAILED", "CANCELLED", "INTERRUPTED"))
STATUS_MAX_AGE_NS = 500_000_000
LOCAL = "R2B4_ODOM_LOCAL"
GLOBAL = "R2B4_BOOT_ROBOT_MAP"


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _checked_status(status, *, generation=None):
    if not isinstance(status, dict) or status.get("state") != "RUNNING" or status.get("fault_layer"):
        raise RuntimeError("V3_NOT_RUNNING_OR_FAULT")
    timestamp = status.get("monotonic_ns")
    if type(timestamp) is not int or not 0 <= time.monotonic_ns() - timestamp < STATUS_MAX_AGE_NS:
        raise RuntimeError("V3_STATUS_STALE")
    if status.get("safety_decision") == "FAULT":
        raise RuntimeError("V3_SAFETY_FAULT")
    estimate = status.get("estimate")
    quality = estimate.get("localization_quality") if isinstance(estimate, dict) else None
    if not isinstance(quality, dict):
        raise RuntimeError("LOCALIZATION_QUALITY_MISSING")
    if (quality.get("local_translation") != "GOOD" or quality.get("heading") != "GOOD"
            or quality.get("local_pose_continuous") is not True
            or quality.get("pose_discontinuity") is not False):
        raise RuntimeError("LOCAL_POSE_NOT_CONTINUOUS_GOOD")
    g = quality.get("generation")
    if type(g) is not int:
        raise RuntimeError("LOCALIZATION_GENERATION_MISSING")
    if generation is not None and g != generation:
        raise RuntimeError("LOCALIZATION_GENERATION_CHANGED")
    return estimate, quality, g


def _read_pose(estimate, frame):
    pose = estimate.get("local_pose" if frame == LOCAL else "global_pose")
    if not isinstance(pose, dict) or pose.get("frame_id") != frame:
        raise RuntimeError("POSE_FRAME_UNAVAILABLE:" + frame)
    if not all(_finite(pose.get(k)) for k in ("x_m", "y_m", "yaw_rad")):
        raise RuntimeError("POSE_INVALID")
    return {k: float(pose[k]) for k in ("x_m", "y_m", "yaw_rad")}


async def _snapshot(robot, pid, generation=None):
    op = await robot.read("operator.status")
    if not isinstance(op, dict) or op.get("runtime_running") is not True or op.get("runtime_pid") != pid:
        raise RuntimeError("V3_RUNTIME_SESSION_CHANGED")
    status = await robot.read("v3.status")
    estimate, quality, actual_generation = _checked_status(status, generation=generation)
    return status, estimate, quality, actual_generation, op


async def _await_behavior_end(robot, pid, generation, behavior_id, deadline):
    while time.monotonic() < deadline:
        await _snapshot(robot, pid, generation)
        state = await robot.read("behavior.state")
        if not isinstance(state, dict) or state.get("behavior_id") != behavior_id:
            raise RuntimeError("CRUISE_BEHAVIOR_REPLACED")
        lifecycle = state.get("lifecycle")
        if lifecycle == "COMPLETED":
            return state
        if lifecycle in TERMINAL:
            raise RuntimeError("CRUISE_TERMINATED:" + str(state.get("reason")))
        if lifecycle not in RUNNING:
            raise RuntimeError("CRUISE_UNEXPECTED_LIFECYCLE:" + str(lifecycle))
        await asyncio.sleep(0.15)
    raise TimeoutError("CRUISE_COMPLETION_TIMEOUT")


async def run(robot, cruise_s=20.0, max_v_mps=0.18, max_omega_rad_s=0.55,
              return_timeout_s=90.0, position_tolerance_m=0.18,
              heading_tolerance_deg=12.0):
    """Fails rather than claiming a physical return when evidence is insufficient."""
    for name, value, lo, hi in (
        ("cruise_s", cruise_s, 1., 120.),
        ("max_v_mps", max_v_mps, 0.01, 0.50),
        ("max_omega_rad_s", max_omega_rad_s, 0.01, 1.20),
        ("return_timeout_s", return_timeout_s, 5., 300.),
        ("position_tolerance_m", position_tolerance_m, 0.01, 1.),
        ("heading_tolerance_deg", heading_tolerance_deg, 1., 45.),
    ):
        if not _finite(value) or not lo <= value <= hi:
            raise ValueError(name + " outside allowed interval")

    pid = None
    origin = None
    frame = None
    generation = None
    behavior_id = None
    navigation = None
    cruise_elapsed = 0.0
    phase = "PREFLIGHT"
    try:
        op = await robot.read("operator.status")
        if not isinstance(op, dict) or op.get("runtime_running") is not True or type(op.get("runtime_pid")) is not int:
            raise RuntimeError("START_V3_FIRST")
        pid = op["runtime_pid"]
        status, estimate, quality, generation, op = await _snapshot(robot, pid)
        if status.get("enabled") is True or status.get("ready_for_active") is not True:
            raise RuntimeError("V3_NOT_IDLE_OR_NOT_READY")
        # Local odometry is sufficient for a same-runtime return; do not require
        # globally aligned atlas coordinates. Global frame is used only if GOOD.
        frame = GLOBAL if quality.get("global_position") == "GOOD" else LOCAL
        origin = _read_pose(estimate, frame)
        phase = "CRUISE"
        started = await robot.call(
            "behavior.room_cruise", max_duration_s=float(cruise_s),
            max_v_mps=float(max_v_mps), max_omega_rad_s=float(max_omega_rad_s))
        if not isinstance(started, dict) or started.get("lifecycle") not in RUNNING:
            raise RuntimeError("CRUISE_START_REJECTED:" + repr(started)[:220])
        behavior_id = started.get("behavior_id")
        if not isinstance(behavior_id, str) or not behavior_id:
            raise RuntimeError("CRUISE_BEHAVIOR_ID_MISSING")
        start_time = time.monotonic()
        # The behavior itself owns the finite duration and canonical stop.
        # Avoid behavior.cancel: its STOP path may preempt the skill session.
        finished = await _await_behavior_end(
            robot, pid, generation, behavior_id,
            start_time + float(cruise_s) + 12.0)
        cruise_elapsed = time.monotonic() - start_time
        if cruise_elapsed < float(cruise_s) - 1.0:
            raise RuntimeError("CRUISE_COMPLETED_TOO_EARLY")
        phase = "RETURN_PREFLIGHT"
        status, estimate, quality, _, op = await _snapshot(robot, pid, generation)
        if frame == GLOBAL and quality.get("global_position") != "GOOD":
            raise RuntimeError("GLOBAL_ALIGNMENT_LOST_DURING_CRUISE")
        if status.get("enabled") is True:
            # Allow a short finite STOP settling period, not another motion command.
            deadline = time.monotonic() + 2.0
            while status.get("enabled") is True and time.monotonic() < deadline:
                await asyncio.sleep(0.1)
                status, estimate, quality, _, op = await _snapshot(robot, pid, generation)
            if status.get("enabled") is True:
                raise RuntimeError("CRUISE_STOP_NOT_CONFIRMED")
        phase = "RETURN"
        # SDK call() has a ~35s default response timeout. Finite navigation
        # needs the public request transport timeout to exceed its action budget.
        navigation = await robot.request(
            "call", action="v3.command.navigate",
            parameters={"frame_id": frame, **origin,
                        "max_v_mps": float(max_v_mps),
                        "max_omega_rad_s": float(max_omega_rad_s),
                        "wait_for_completion": True,
                        "finite_timeout_s": float(return_timeout_s),
                        "session_watchdog_s": float(return_timeout_s) + 2.0,
                        "capture": False},
            timeout_s=float(return_timeout_s) + 12.0)
        if not isinstance(navigation, dict) or navigation.get("status") != "COMPLETED" or navigation.get("reason") != "COMPLETE":
            raise RuntimeError("RETURN_NOT_COMPLETED:" + repr(navigation)[:300])
        phase = "VERIFY"
        status, estimate, quality, _, _ = await _snapshot(robot, pid, generation)
        if frame == GLOBAL and quality.get("global_position") != "GOOD":
            raise RuntimeError("GLOBAL_ALIGNMENT_LOST_AT_FINISH")
        if status.get("enabled") is True:
            raise RuntimeError("RETURN_STOP_NOT_CONFIRMED")
        final = _read_pose(estimate, frame)
        distance = math.hypot(final["x_m"]-origin["x_m"], final["y_m"]-origin["y_m"])
        yaw = math.degrees(abs(math.atan2(
            math.sin(final["yaw_rad"]-origin["yaw_rad"]),
            math.cos(final["yaw_rad"]-origin["yaw_rad"]))))
        if distance > position_tolerance_m or yaw > heading_tolerance_deg:
            raise RuntimeError(f"RETURN_POSE_OUTSIDE_TOLERANCE:{distance:.3f}m,{yaw:.2f}deg")
        return {"status": "COMPLETED", "frame_id": frame, "runtime_pid": pid,
                "cruise_elapsed_s": round(cruise_elapsed, 3), "origin": origin,
                "final": final, "position_error_m": round(distance, 4),
                "heading_error_deg": round(yaw, 2), "navigation": navigation,
                "note": "Errors are estimated from the robot pose, not physically measured."}
    except asyncio.CancelledError:
        raise
    except Exception as exc:
        return {"status": "FAILED", "phase": phase,
                "reason": f"{type(exc).__name__}:{exc}"[:450],
                "frame_id": frame, "runtime_pid": pid, "origin": origin,
                "cruise_elapsed_s": round(cruise_elapsed, 3), "navigation": navigation,
                "note": "Do not infer successful return from a failed result."}
    finally:
        # Cleanup a still-active cruise only. Avoid robot.stop() because that
        # can revoke the skill itself; cancellation goes through canonical host.
        if behavior_id is not None:
            try:
                state = await robot.read("behavior.state")
                if (isinstance(state, dict) and state.get("behavior_id") == behavior_id
                        and state.get("lifecycle") in RUNNING):
                    await robot.call("behavior.cancel", reason="CRUISE_RETURN_SKILL_CLEANUP")
            except Exception:
                pass
