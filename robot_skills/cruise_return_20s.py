"""20 s canonical Room Cruise, then return to initial global pose and heading.

R2B4 public SDK skill entrypoint. Requires an already running, idle V3 runtime.
No direct motors, no V3 restart, no capture mode changes.
"""
import asyncio
import math
import time

FRAME = "R2B4_BOOT_ROBOT_MAP"
RUNNING = {"STARTING", "ACTIVE"}
TERMINAL = {"COMPLETED", "FAILED", "CANCELLED", "INTERRUPTED"}


def _pose(status, *, require_good=True):
    if not isinstance(status, dict) or status.get("state") != "RUNNING":
        raise RuntimeError("V3_NOT_RUNNING")
    if status.get("fault_layer") or status.get("safety_decision") == "FAULT":
        raise RuntimeError("V3_FAULT")
    est = status.get("estimate") or {}
    quality = est.get("localization_quality") or {}
    if quality.get("pose_discontinuity") is True:
        raise RuntimeError("POSE_DISCONTINUITY")
    if require_good and (quality.get("global_position") != "GOOD" or quality.get("heading") != "GOOD"):
        raise RuntimeError("GLOBAL_POSE_NOT_GOOD")
    pose = est.get("global_pose") or {}
    if pose.get("frame_id") != FRAME:
        raise RuntimeError("GLOBAL_FRAME_UNAVAILABLE")
    for key in ("x_m", "y_m", "yaw_rad"):
        if type(pose.get(key)) not in (int, float) or not math.isfinite(pose[key]):
            raise RuntimeError("INVALID_POSE:" + key)
    return {key: float(pose[key]) for key in ("x_m", "y_m", "yaw_rad")}


async def _state(robot, pid, *, idle=False):
    op = await robot.read("operator.status")
    if not isinstance(op, dict) or op.get("runtime_running") is not True or op.get("runtime_pid") != pid:
        raise RuntimeError("RUNTIME_SESSION_CHANGED")
    status = await robot.read("v3.status")
    if idle and status.get("enabled") is True:
        raise RuntimeError("ROBOT_NOT_IDLE")
    return status


async def run(robot, cruise_s=20.0, max_v_mps=0.18, max_omega_rad_s=0.55,
              return_timeout_s=90.0):
    """Cruise for cruise_s, return to start, report verified final pose."""
    if type(cruise_s) not in (float, int) or not 1 <= cruise_s <= 120:
        raise ValueError("cruise_s outside [1, 120]")
    if not 0.01 <= max_v_mps <= 0.50 or not 0.01 <= max_omega_rad_s <= 1.20:
        raise ValueError("invalid speed caps")
    if not 5 <= return_timeout_s <= 300:
        raise ValueError("invalid return_timeout_s")
    op = await robot.read("operator.status")
    if not isinstance(op, dict) or op.get("runtime_running") is not True or type(op.get("runtime_pid")) is not int:
        return {"status": "FAILED", "reason": "START_V3_FIRST"}
    pid = op["runtime_pid"]
    started = None
    origin = None
    cruise_elapsed = 0.0
    nav_result = None
    try:
        initial = await _state(robot, pid, idle=True)
        origin = _pose(initial)
        started = await robot.call("behavior.room_cruise", max_duration_s=float(cruise_s) + 8.0,
                                   max_v_mps=float(max_v_mps), max_omega_rad_s=float(max_omega_rad_s))
        if not isinstance(started, dict) or started.get("lifecycle") not in RUNNING:
            raise RuntimeError("CRUISE_REJECTED:" + repr(started)[:250])
        behavior_id = started.get("behavior_id")
        if not isinstance(behavior_id, str) or not behavior_id:
            raise RuntimeError("MISSING_BEHAVIOR_ID")
        t0 = time.monotonic()
        deadline = t0 + cruise_s
        while time.monotonic() < deadline:
            _pose(await _state(robot, pid), require_good=False)
            behavior = await robot.read("behavior.state")
            if not isinstance(behavior, dict) or behavior.get("behavior_id") != behavior_id:
                raise RuntimeError("BEHAVIOR_ID_CHANGED")
            if behavior.get("lifecycle") in TERMINAL:
                raise RuntimeError("CRUISE_ENDED_EARLY:" + str(behavior.get("reason")))
            await asyncio.sleep(min(0.2, max(0.0, deadline - time.monotonic())))
        cruise_elapsed = time.monotonic() - t0
        # Cancel only our own behavior. robot.stop() would revoke the skill itself.
        state = await robot.read("behavior.state")
        if state.get("behavior_id") != behavior_id or state.get("lifecycle") not in RUNNING:
            raise RuntimeError("CRUISE_NO_LONGER_OWNED")
        await robot.call("behavior.cancel", reason="CRUISE_20S_COMPLETE")
        await _state(robot, pid, idle=True)
        # Blocking canonical V3 navigation, with full finite-action STOP lifecycle.
        # SDK's ordinary 35s request timeout is too short for this return trip.
        nav_result = await robot.request(
            "call", action="v3.command.navigate",
            parameters={"frame_id": FRAME, **origin, "wait_for_completion": True,
                        "max_v_mps": float(max_v_mps),
                        "max_omega_rad_s": float(max_omega_rad_s),
                        "capture": False,
                        "finite_timeout_s": float(return_timeout_s),
                        "session_watchdog_s": float(return_timeout_s) + 2.0},
            timeout_s=float(return_timeout_s) + 10.0,
        )
        if not isinstance(nav_result, dict) or (nav_result.get("status"), nav_result.get("reason")) != ("COMPLETED", "COMPLETE"):
            raise RuntimeError("RETURN_NOT_COMPLETED:" + repr(nav_result)[:350])
        last = await _state(robot, pid, idle=True)
        final = _pose(last)
        position_error_m = math.hypot(final["x_m"] - origin["x_m"], final["y_m"] - origin["y_m"])
        heading_error_deg = abs(math.degrees(math.atan2(
            math.sin(final["yaw_rad"] - origin["yaw_rad"]),
            math.cos(final["yaw_rad"] - origin["yaw_rad"]))))
        return {"status": "COMPLETED", "cruise_elapsed_s": round(cruise_elapsed, 3),
                "origin": origin, "final": final,
                "position_error_m": round(position_error_m, 4),
                "heading_error_deg": round(heading_error_deg, 2),
                "navigation": nav_result, "runtime_pid": pid,
                "note": "Pose errors are estimator measurements, not tape-measured physical accuracy."}
    except asyncio.CancelledError:
        raise
    except Exception as exc:
        return {"status": "FAILED", "reason": f"{type(exc).__name__}:{exc}"[:450],
                "origin": origin, "cruise_elapsed_s": round(cruise_elapsed, 3),
                "navigation": nav_result, "runtime_pid": pid}
    finally:
        if started and isinstance(started, dict) and started.get("behavior_id"):
            try:
                state = await robot.read("behavior.state")
                if (isinstance(state, dict) and state.get("behavior_id") == started["behavior_id"]
                        and state.get("lifecycle") in RUNNING):
                    await robot.call("behavior.cancel", reason="SKILL_CLEANUP")
            except Exception:
                pass
