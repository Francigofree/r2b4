"""Capture and revalidate one goal's origin through canonical public operations.

Origin is execution lineage, not a new world fact. The compact resident pose is
an L3 estimate at a completed tick reference time; capturing it does not invent
a fresh physical sensor measurement or retain motion authority after restart.
"""
from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass

from v3.capture_rate import DEFAULT_CAPTURE_HZ, validate_capture_hz
from v3.contracts.localization import GLOBAL_FRAME_ID, LOCAL_FRAME_ID
from v3.operator_controller import CAPTURE_MODES, DEFAULT_CAPTURE_MODE
from v3.resident_status import HOST_STATUS_MAX_AGE_NS

from .world_model import host_clock_epoch


def _cancelled(cancel_event):
    if cancel_event is not None and cancel_event.is_set():
        raise ValueError("GOAL_ORIGIN_CANCELLED")


def _capture_identity(runtime):
    mode, hz = runtime.get("capture_mode"), runtime.get("capture_hz")
    if mode not in CAPTURE_MODES or type(hz) is not int:
        raise ValueError("GOAL_ORIGIN_CAPTURE_IDENTITY_UNAVAILABLE")
    return mode, validate_capture_hz(hz)


def _runtime_context(robot, clock_ns, cancel_event=None):
    _cancelled(cancel_event)
    runtime = robot.read("operator.status")
    if (not isinstance(runtime, Mapping) or runtime.get("runtime_running") is not True
            or type(runtime.get("runtime_pid")) is not int or runtime["runtime_pid"] <= 0):
        raise ValueError("GOAL_ORIGIN_RUNTIME_SCOPE_UNAVAILABLE")
    mode, hz = _capture_identity(runtime)
    status = robot.read("v3.status")
    now = clock_ns()
    stamp = status.get("monotonic_ns") if isinstance(status, Mapping) else None
    if type(stamp) is not int or not 0 <= now - stamp < HOST_STATUS_MAX_AGE_NS:
        raise ValueError("GOAL_ORIGIN_STATUS_STALE")
    if status.get("fault_layer") or status.get("safety_decision") == "FAULT":
        raise ValueError("GOAL_ORIGIN_RUNTIME_FAULT")
    if status.get("safety_decision") not in {"ALLOW", "STOP"}:
        raise ValueError("GOAL_ORIGIN_SAFETY_UNAVAILABLE")
    if status.get("safety_decision") == "STOP" and status.get("safety_reason") != "NOT_ACTIVE":
        raise ValueError("GOAL_ORIGIN_SAFETY_STOP:" + str(status.get("safety_reason")))
    tick = status.get("tick_id")
    estimate = status.get("estimate")
    if type(tick) is not int or tick < 0 or not isinstance(estimate, Mapping):
        raise ValueError("GOAL_ORIGIN_POSE_UNAVAILABLE")
    quality = estimate.get("localization_quality")
    generation = quality.get("generation") if isinstance(quality, Mapping) else None
    if type(generation) is not int or generation < 0:
        raise ValueError("GOAL_ORIGIN_LOCALIZATION_GENERATION_UNAVAILABLE")
    if quality.get("pose_discontinuity") is True or quality.get("heading") not in {"GOOD", "DEGRADED"}:
        raise ValueError("GOAL_ORIGIN_LOCALIZATION_UNQUALIFIED")
    # Use the continuous local odometry frame when available. A new global fix
    # can adjust map coordinates without changing this goal's physical origin.
    pose = next((candidate for candidate in (estimate.get("local_pose"), estimate.get("global_pose"), estimate)
                 if isinstance(candidate, Mapping) and candidate.get("frame_id") in {LOCAL_FRAME_ID, GLOBAL_FRAME_ID}), None)
    if pose is None or any(type(pose.get(name)) not in {int, float} or not math.isfinite(pose[name])
                           for name in ("x_m", "y_m", "yaw_rad")):
        raise ValueError("GOAL_ORIGIN_POSE_UNAVAILABLE")
    position_quality = "local_translation" if pose["frame_id"] == LOCAL_FRAME_ID else "global_position"
    if quality.get(position_quality) not in {"GOOD", "DEGRADED"}:
        raise ValueError("GOAL_ORIGIN_LOCALIZATION_UNQUALIFIED")
    # Detect a host session/capture transition during the status read. The
    # physical command will also bind the returned expected_runtime_pid.
    after = robot.read("operator.status")
    if (not isinstance(after, Mapping) or after.get("runtime_running") is not True
            or after.get("runtime_pid") != runtime["runtime_pid"] or _capture_identity(after) != (mode, hz)):
        raise ValueError("GOAL_ORIGIN_RUNTIME_IDENTITY_CHANGED")
    _cancelled(cancel_event)
    now = clock_ns()
    if not 0 <= now - stamp < HOST_STATUS_MAX_AGE_NS:
        raise ValueError("GOAL_ORIGIN_STATUS_STALE")
    return runtime, status, pose, generation, now


@dataclass(frozen=True, slots=True)
class GoalOrigin:
    frame_id: str
    x_m: float
    y_m: float
    yaw_rad: float
    runtime_pid: int
    localization_generation: int
    capture_mode: str
    capture_hz: int
    measurement_time_ns: int
    observation_time_ns: int
    source_tick_id: int
    clock_epoch: str
    source_time_kind: str = "L3_ESTIMATE_REFERENCE_TIME"

    def __post_init__(self):
        if self.frame_id not in {LOCAL_FRAME_ID, GLOBAL_FRAME_ID}:
            raise ValueError("GOAL_ORIGIN_FRAME_UNSUPPORTED")
        for name in ("x_m", "y_m", "yaw_rad"):
            value = getattr(self, name)
            if type(value) not in {int, float} or not math.isfinite(value):
                raise ValueError("GOAL_ORIGIN_POSE_UNAVAILABLE")
            object.__setattr__(self, name, float(value))
        for name in ("runtime_pid", "localization_generation", "measurement_time_ns", "observation_time_ns", "source_tick_id"):
            if type(getattr(self, name)) is not int or getattr(self, name) < (1 if name == "runtime_pid" else 0):
                raise ValueError("GOAL_ORIGIN_INVALID_LINEAGE:" + name)
        if self.observation_time_ns < self.measurement_time_ns:
            raise ValueError("GOAL_ORIGIN_INVALID_MEASUREMENT_TIME")
        if self.capture_mode not in CAPTURE_MODES or type(self.capture_hz) is not int:
            raise ValueError("GOAL_ORIGIN_CAPTURE_IDENTITY_UNAVAILABLE")
        validate_capture_hz(self.capture_hz)
        if not isinstance(self.clock_epoch, str) or not self.clock_epoch or len(self.clock_epoch) > 256:
            raise ValueError("GOAL_ORIGIN_INVALID_CLOCK_EPOCH")
        if self.source_time_kind != "L3_ESTIMATE_REFERENCE_TIME":
            raise ValueError("GOAL_ORIGIN_INVALID_TIME_KIND")

    def to_jsonable(self):
        return {name: getattr(self, name) for name in self.__dataclass_fields__}

    @classmethod
    def from_jsonable(cls, value):
        if not isinstance(value, Mapping) or set(value) - set(cls.__dataclass_fields__):
            raise ValueError("invalid goal origin")
        try:
            return cls(**value)
        except TypeError as exc:
            raise ValueError("incomplete goal origin lineage") from exc

    def navigation_parameters(self, robot, clock_ns, cancel_event=None):
        if self.clock_epoch != host_clock_epoch():
            raise ValueError("GOAL_ORIGIN_CLOCK_EPOCH_CHANGED")
        runtime, _, pose, generation, _ = _runtime_context(robot, clock_ns, cancel_event)
        if (runtime["runtime_pid"] != self.runtime_pid or pose["frame_id"] != self.frame_id
                or generation != self.localization_generation
                or _capture_identity(runtime) != (self.capture_mode, self.capture_hz)):
            raise ValueError("GOAL_ORIGIN_FRAME_IDENTITY_CHANGED")
        return {"x_m": self.x_m, "y_m": self.y_m, "yaw_rad": self.yaw_rad, "frame_id": self.frame_id,
                "expected_runtime_pid": self.runtime_pid, "capture": False,
                "capture_mode": self.capture_mode, "capture_hz": self.capture_hz}


def capture_goal_origin(robot, clock_ns, cancel_event=None):
    """Stabilize the canonical runtime without motion and capture one pose."""
    _cancelled(cancel_event)
    runtime = robot.read("operator.status")
    if not isinstance(runtime, Mapping):
        raise ValueError("GOAL_ORIGIN_RUNTIME_SCOPE_UNAVAILABLE")
    if runtime.get("runtime_running") is not True:
        mode = runtime.get("capture_mode")
        hz = runtime.get("capture_hz")
        mode = DEFAULT_CAPTURE_MODE if mode is None else mode
        hz = DEFAULT_CAPTURE_HZ if hz is None else hz
        if mode not in CAPTURE_MODES:
            raise ValueError("GOAL_ORIGIN_CAPTURE_IDENTITY_UNAVAILABLE")
        hz = validate_capture_hz(hz)
        _cancelled(cancel_event)
        robot.execute("operator.runtime.start", capture_mode=mode, capture_hz=hz)
    # Runtime-start itself never requests movement. Canonical STOP establishes
    # the same idle boundary used for other finite physical actions.
    robot.stop()
    runtime, status, pose, generation, now = _runtime_context(robot, clock_ns, cancel_event)
    mode, hz = _capture_identity(runtime)
    return GoalOrigin(pose["frame_id"], pose["x_m"], pose["y_m"], pose["yaw_rad"],
                      runtime["runtime_pid"], generation, mode, hz,
                      status["monotonic_ns"], now, status["tick_id"], host_clock_epoch())


__all__ = ["GoalOrigin", "capture_goal_origin"]
