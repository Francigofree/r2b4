"""Blocking finite navigation at the host edge of canonical V3 command ingress.

This executor owns relative goal conversion and host-side waiting only. Runtime
L5/L6 remain the mission/completion authorities; OperatorController and the CLI
producer retain command publication, heartbeat, owner and watchdog ownership.
No control tick imports or waits on this module.
"""
from __future__ import annotations

import math
import os
import threading
import time
from collections.abc import Callable, Mapping

from v3.action_catalog import ACTION_CATALOG
from v3.capture_rate import DEFAULT_CAPTURE_HZ
from v3.contracts.localization import GLOBAL_FRAME_ID, LOCAL_FRAME_ID
from v3.operator_controller import DEFAULT_CAPTURE_MODE, OperatorController


class FiniteNavigationSafetyError(RuntimeError):
    """The finite action could not confirm its final canonical STOP."""

    def __init__(self, message, *, identity=None):
        super().__init__(message)
        self.identity = dict(identity or {})


def _finite(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be finite numeric")
    return float(value)


def _wrap_yaw(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def _pose_distance(first, second):
    if not isinstance(first, Mapping) or not isinstance(second, Mapping):
        return None
    if not first.get("frame_id") or first.get("frame_id") != second.get("frame_id"):
        return None
    values = tuple(row.get(name) for row in (first, second) for name in ("x_m", "y_m"))
    if not all(type(value) in {int, float} and math.isfinite(value) for value in values):
        return None
    return math.hypot(values[2] - values[0], values[3] - values[1])


class FiniteNavigationExecutor:
    def __init__(self, controller: OperatorController, *, sleep=time.sleep, monotonic=time.monotonic,
                 progress_timeout_s: float = 10.0):
        self.controller = controller
        self._sleep = sleep
        self._monotonic = monotonic
        self._progress_timeout_s = _finite(progress_timeout_s, "progress_timeout_s")
        if self._progress_timeout_s <= 0:
            raise ValueError("progress_timeout_s must be positive")

    def execute(
        self, action: str, parameters: Mapping[str, object], *,
        cancel_event: threading.Event | None = None, finite_timeout_s: float | None = None,
        capture: bool = True, capture_mode: str = DEFAULT_CAPTURE_MODE,
        capture_hz: int = DEFAULT_CAPTURE_HZ, session_owner_pid: int | None = None,
        session_watchdog_s: float | None = None,
        admission_sink: Callable[[Mapping[str, object]], None] | None = None,
    ) -> dict[str, object]:
        started = self._monotonic()
        command_id = mission_id = None
        pose: Mapping[str, object] | None = None
        final: Mapping[str, object] = {}
        target: dict[str, object] = {}
        reason = "CANCELLED"
        preparation = None
        requested = dict(parameters)
        try:
            params = dict(parameters)
            if action not in {"v3.command.move_relative", "v3.command.turn_by", "v3.command.navigate"}:
                raise ValueError(f"unsupported finite action: {action}")
            descriptor = ACTION_CATALOG[action]
            unknown = set(params) - {item.name for item in descriptor.parameters}
            if unknown:
                raise ValueError("unknown parameters: " + ", ".join(sorted(unknown)))
            for item in descriptor.parameters:
                if item.required and item.name not in params:
                    raise ValueError(f"missing required parameter: {item.name}")
            max_v = _finite(params.pop("max_v_mps", 0.20), "max_v_mps")
            max_omega = _finite(params.pop("max_omega_rad_s", 0.60), "max_omega_rad_s")
            if not 0.01 <= max_v <= 0.50 or not 0.01 <= max_omega <= 1.20:
                raise ValueError("navigation speed limits must be within v=[0.01, 0.50], omega=[0.01, 1.20]")
            if action == "v3.command.move_relative":
                forward = _finite(params.pop("forward_m"), "forward_m")
                left = _finite(params.pop("left_m", 0.0), "left_m")
                value = params.pop("final_yaw_rad", None)
                yaw_delta = None if value is None else _finite(value, "final_yaw_rad")
                frame_id = LOCAL_FRAME_ID
            elif action == "v3.command.turn_by":
                angle = _finite(params.pop("angle_deg"), "angle_deg")
                if not -180.0 <= angle <= 180.0:
                    raise ValueError("angle_deg must stay within [-180, 180]; pose goals use the shortest turn")
                forward = left = 0.0
                yaw_delta = math.radians(angle)
                frame_id = LOCAL_FRAME_ID
            elif action == "v3.command.navigate":
                frame_id = params.pop("frame_id", GLOBAL_FRAME_ID)
                if frame_id not in (LOCAL_FRAME_ID, GLOBAL_FRAME_ID):
                    raise ValueError("unsupported navigation frame")
                target = {"x_m": _finite(params.pop("x_m"), "x_m"),
                          "y_m": _finite(params.pop("y_m"), "y_m")}
                value = params.pop("yaw_rad", None)
                if value is not None:
                    target["yaw_rad"] = _wrap_yaw(_finite(value, "yaw_rad"))
            watchdog = 30.0 if session_watchdog_s is None else _finite(session_watchdog_s, "session_watchdog_s")
            if not 1.0 <= watchdog <= 600.0:
                raise ValueError("session_watchdog_s must be within [1, 600]")
            owner = os.getpid() if session_owner_pid is None else session_owner_pid
            if type(owner) is not int or owner <= 0:
                raise ValueError("session_owner_pid must be a positive integer")
            options = dict(capture=capture, capture_mode=capture_mode, capture_hz=capture_hz,
                           session_watchdog_s=watchdog, session_owner_pid=owner)
            # Leave the independent command producer watchdog as the final bound.
            timeout = watchdog - 0.5
            if finite_timeout_s is not None:
                requested_timeout = _finite(finite_timeout_s, "finite_timeout_s")
                if requested_timeout <= 0:
                    raise ValueError("finite_timeout_s must be positive")
                timeout = min(timeout, requested_timeout)
            deadline = started + timeout
            start_status: Mapping[str, object] = {}
            start_generation: object = None
            if cancel_event is None or not cancel_event.is_set():
                # P0 state/action transaction: any capture re-arm/runtime restart,
                # prior motion preemption and IDLE confirmation happen BEFORE the
                # pose snapshot. The operator lock remains held through admission.
                with self.controller.finite_motion_transaction(
                    capture=capture, capture_mode=capture_mode, capture_hz=capture_hz,
                    deadline=deadline, cancel_event=cancel_event,
                ) as preparation:
                    pose, start_status, reason = self._pose_snapshot(
                        frame_id,
                        min(deadline, self._monotonic() + 8.0),
                        cancel_event,
                        runtime_pid=preparation.runtime_pid,
                    )
                    final = start_status
                    if pose is not None:
                        if action != "v3.command.navigate":
                            yaw = float(pose["yaw_rad"])
                            target = {
                                "x_m": float(pose["x_m"]) + forward * math.cos(yaw) - left * math.sin(yaw),
                                "y_m": float(pose["y_m"]) + forward * math.sin(yaw) + left * math.cos(yaw),
                            }
                            if yaw_delta is not None:
                                target["yaw_rad"] = _wrap_yaw(yaw + yaw_delta)
                        target["frame_id"] = frame_id
                        start_generation = self._generation(start_status) if frame_id == LOCAL_FRAME_ID else None
                        if cancel_event is not None and cancel_event.is_set():
                            reason = "CANCELLED"
                        elif self._monotonic() >= deadline:
                            reason = "TIMEOUT"
                        else:
                            handle = self.controller.navigate(
                                **target,
                                max_v_mps=max_v,
                                max_omega_rad_s=max_omega,
                                preparation=preparation,
                                **options,
                            )
                            command_id = handle.get("command_id") if isinstance(handle, Mapping) else getattr(handle, "command_id", None)
                            if not isinstance(command_id, str) or not command_id:
                                raise RuntimeError("navigation returned no command identity")
                            mission_id = f"mission-{command_id}"
                if mission_id is not None and preparation is not None:
                    # A finite action can wait for much longer than admission.
                    # Publish the canonical identities before that wait, using
                    # only the completed pose source's compact provenance.
                    if admission_sink is not None:
                        try:
                            admission_sink({
                                "command_id": command_id, "mission_id": mission_id,
                                "frame_id": frame_id, "runtime_pid": preparation.runtime_pid,
                                "localization_generation": start_generation,
                                "pose_status_monotonic_ns": start_status.get("monotonic_ns"),
                            })
                        except Exception:
                            pass  # Passive evidence cannot affect execution.
                    final, reason = self._wait(
                        mission_id,
                        frame_id,
                        start_generation,
                        deadline,
                        cancel_event,
                        runtime_pid=preparation.runtime_pid,
                    )
        except TimeoutError:
            reason = "CANCELLED" if cancel_event is not None and cancel_event.is_set() else "TIMEOUT"
        finally:
            try:
                self.controller.stop()
            except Exception as exc:
                raise FiniteNavigationSafetyError(f"robot STOP failed: {exc}", identity={
                    "command_id": command_id, "mission_id": mission_id,
                    "runtime_pid": None if preparation is None else preparation.runtime_pid,
                    "status": "INTERRUPTED", "reason": reason,
                }) from exc

        navigation = final.get("navigation")
        navigation = navigation if isinstance(navigation, Mapping) else {}
        final_pose = self._pose(final, target.get("frame_id", LOCAL_FRAME_ID))
        comparable = (reason not in {"LOCALIZATION_FRAME_CHANGED", "RUNTIME_SESSION_CHANGED"}
                      and (frame_id != LOCAL_FRAME_ID or self._generation(final) == start_generation))
        return {
            "status": "COMPLETED" if reason == "COMPLETE" else "INTERRUPTED",
            "reason": reason, "command_id": command_id, "mission_id": mission_id,
            "requested": requested, "start_pose": dict(pose) if pose is not None else None,
            "target_pose": target, "final_pose": final_pose,
            "distance_requested_m": _pose_distance(pose, target),
            "distance_executed_m": _pose_distance(pose, final_pose) if comparable else None,
            "distance_remaining_m": _pose_distance(final_pose, target) if comparable else None,
            "frame_provenance": {
                "frame_id": target.get("frame_id", frame_id),
                "runtime_pid": None if preparation is None else preparation.runtime_pid,
                "localization_generation": start_generation,
                "pose_status_monotonic_ns": start_status.get("monotonic_ns"),
            },
            "elapsed_s": max(0.0, self._monotonic() - started),
            "progress": navigation.get("progress"), "navigation_reason": navigation.get("reason"),
            "safety_reason": final.get("safety_reason"), "stop": {"status": "STOPPED"},
        }

    @staticmethod
    def _pose(status: Mapping[str, object], frame_id: str) -> Mapping[str, object] | None:
        pose = status.get("estimate")
        if isinstance(pose, Mapping) and frame_id == LOCAL_FRAME_ID:
            pose = pose.get("local_pose")
        return dict(pose) if isinstance(pose, Mapping) else None

    @staticmethod
    def _generation(status: Mapping[str, object]) -> object:
        estimate = status.get("estimate")
        quality = estimate.get("localization_quality") if isinstance(estimate, Mapping) else None
        return quality.get("generation") if isinstance(quality, Mapping) else None

    @staticmethod
    def _fault(status: Mapping[str, object]) -> bool:
        return status.get("state") != "RUNNING" or bool(status.get("fault_layer")) or status.get("safety_decision") == "FAULT"

    def _pose_snapshot(
        self, frame_id: str, deadline: float, cancel_event: threading.Event | None,
        *, runtime_pid: int,
    ) -> tuple[Mapping[str, object] | None, Mapping[str, object], str]:
        status: Mapping[str, object] = {}
        while True:
            if cancel_event is not None and cancel_event.is_set():
                return None, status, "CANCELLED"
            if self._monotonic() >= deadline:
                return None, status, "TIMEOUT"
            if self.controller.snapshot().runtime_pid != runtime_pid:
                return None, status, "RUNTIME_SESSION_CHANGED"
            current = self.controller.live_runtime_status()
            if isinstance(current, Mapping):
                status = current
                if self._fault(status):
                    return None, status, "RUNTIME_FAULT"
                pose = self._pose(status, frame_id)
                if pose is not None and pose.get("frame_id") == frame_id and status.get("ready_for_active") is True:
                    for key in ("x_m", "y_m", "yaw_rad"):
                        _finite(pose.get(key), key)
                    return pose, status, "READY"
            self._sleep(min(0.05, max(0.0, deadline - self._monotonic())))

    def _wait(
        self, mission_id: str, frame_id: str, generation: object,
        deadline: float, cancel_event: threading.Event | None, *, runtime_pid: int,
    ) -> tuple[Mapping[str, object], str]:
        final: Mapping[str, object] = {}
        progress_at = self._monotonic()
        progress_pose = None
        while True:
            if cancel_event is not None and cancel_event.is_set():
                return final, "CANCELLED"
            if self.controller.snapshot().runtime_pid != runtime_pid:
                return final, "RUNTIME_SESSION_CHANGED"
            remaining = deadline - self._monotonic()
            if remaining <= 0:
                return final, "TIMEOUT"
            payload = self.controller.live_runtime_status()
            if not isinstance(payload, Mapping):
                return final, "STATUS_UNAVAILABLE"
            final = payload
            mission, navigation = payload.get("mission"), payload.get("navigation")
            if self._fault(payload):
                return final, "RUNTIME_FAULT"
            estimate = self._pose(payload, frame_id)
            if (estimate is None or estimate.get("frame_id") != frame_id
                    or (frame_id == LOCAL_FRAME_ID and generation != self._generation(payload))):
                return final, "LOCALIZATION_FRAME_CHANGED"
            if not isinstance(mission, Mapping) or mission.get("mission_id") != mission_id:
                return final, "MISSION_REPLACED"
            if not isinstance(navigation, Mapping) or navigation.get("mission_id") != mission_id:
                return final, "NAVIGATION_IDENTITY_MISMATCH"
            if mission.get("lifecycle") in {"FAILED", "CANCELLED"}:
                return final, "MISSION_" + str(mission["lifecycle"])
            nav_status = navigation.get("status")
            decision = payload.get("safety_decision")
            if decision not in {"ALLOW", "STOP"}:
                return final, "SAFETY_STATUS_UNAVAILABLE"
            if decision == "STOP" and not (
                (nav_status == "COMPLETE" and payload.get("safety_reason") == "NOT_ACTIVE")
                or (nav_status == "IDLE" and navigation.get("reason") == "LOCALIZATION_HOLD")
            ):
                return final, "SAFETY_STOP"
            if mission.get("mode") != "NAVIGATE":
                return final, "MISSION_INVALID"
            if nav_status == "COMPLETE":
                return final, "COMPLETE"
            if nav_status in {"NO_PATH", "INVALIDATED"}:
                return final, str(nav_status)
            if nav_status not in {"ACTIVE", "PENDING", "IDLE"}:
                return final, "NAVIGATION_STATUS_UNAVAILABLE"
            current_pose = tuple(estimate.get(name) for name in ("x_m", "y_m", "yaw_rad"))
            if all(type(value) in {int, float} and math.isfinite(value) for value in current_pose):
                if (progress_pose is None
                        or math.hypot(current_pose[0] - progress_pose[0], current_pose[1] - progress_pose[1]) >= .01
                        or abs(_wrap_yaw(current_pose[2] - progress_pose[2])) >= .02):
                    progress_pose, progress_at = current_pose, self._monotonic()
            if self._monotonic() - progress_at >= self._progress_timeout_s:
                return final, "NAVIGATION_STALLED"
            self._sleep(min(0.05, remaining))
