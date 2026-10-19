"""Finite, LLM-free RoomCruise through the canonical behavior owner.

The host releases a runtime started by this invocation after its terminal event.
A pre-existing operator runtime and its capture settings remain session-owned.
"""
import asyncio
import math
import time


def _validate_duration(duration_s):
    if (type(duration_s) not in (int, float) or not math.isfinite(duration_s)
            or not 0 < duration_s <= 3600):
        raise ValueError("duration_s must be finite and in (0, 3600]")


async def run(robot, duration_s=300, max_v_mps=None, max_omega_rad_s=None,
              capture_mode=None, capture_hz=None):
    return await _run_timed_behavior(robot, "room_cruise", duration_s, max_v_mps,
                                     max_omega_rad_s, capture_mode, capture_hz)


async def _run_timed_behavior(robot, name, duration_s, max_v_mps,
                              max_omega_rad_s, capture_mode, capture_hz):
    # Shared by the two saved timed motion skills; navigation and person
    # selection stay entirely in their existing canonical behavior owners.
    prefix = name.upper()
    _validate_duration(duration_s)
    parameters = {key: value for key, value in {
        "max_v_mps": max_v_mps, "max_omega_rad_s": max_omega_rad_s,
        "capture_mode": capture_mode, "capture_hz": capture_hz,
    }.items() if value is not None}
    operator = await asyncio.wait_for(robot.read("operator.status"), 2)
    if operator.get("runtime_running") is True:
        parameters.update(capture_mode=operator["capture_mode"], capture_hz=operator["capture_hz"])
    elif operator.get("runtime_running") is False:
        # Cold device readiness is preparation, not part of the requested
        # execution interval or the behavior's bounded admission deadline.
        await robot.call("operator.runtime.start", **{
            key: value for key, value in parameters.items() if key in {"capture_mode", "capture_hz"}
        })
    else:
        raise RuntimeError("OPERATOR_STATUS_UNAVAILABLE")
    parameters.update(max_duration_s=duration_s, completion_on_duration=True,
                      session_watchdog_s=duration_s + 30)
    state = await robot.call("behavior." + name, **parameters)
    identity = state.get("behavior_id")
    if not isinstance(identity, str) or not identity:
        raise RuntimeError(prefix + "_IDENTITY_MISSING")
    deadline = time.monotonic() + duration_s + 30
    try:
        while True:
            if state.get("behavior_id") != identity:
                raise RuntimeError(prefix + "_REPLACED")
            lifecycle = state.get("lifecycle")
            if lifecycle == "COMPLETED":
                return {"status": "COMPLETED", "reason": state.get("reason"),
                        "behavior_id": identity, "mission_id": state.get("mission_id"),
                        "requested_duration_s": duration_s,
                        "execution_started_ns": state.get("execution_started_ns"),
                        "measurement_time_ns": state.get("measurement_time_ns")}
            if lifecycle not in {"STARTING", "ACTIVE"}:
                raise RuntimeError(prefix + "_" + str(lifecycle) + ":" + str(state.get("reason")))
            if time.monotonic() >= deadline:
                raise TimeoutError(prefix + "_COMPLETION_TIMEOUT")
            await asyncio.sleep(0.5)
            state = await asyncio.wait_for(robot.read("behavior.state"), 2)
    finally:
        # Never use global STOP here: it would revoke the skill itself. The host
        # independently stops its canonical intent on worker error/cancellation.
        if state.get("behavior_id") == identity and state.get("lifecycle") in {"STARTING", "ACTIVE"}:
            await robot.call("behavior.cancel", reason=prefix + "_SKILL_EXIT")
