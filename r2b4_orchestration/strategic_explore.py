"""Finite place exploration through the existing internal behavior action port.

Persistent coverage orders places; only independently current Spatial locations
can supply NAVIGATE coordinates. Historical atlas frontiers remain knowledge
until the localization owner provides a real current alignment.
"""
from __future__ import annotations

import math
import time
from collections.abc import Mapping

from .behavior_system import BehaviorLifecycle, BehaviorUpdate
from .spatial_service import SpatialQuery


class StrategicExplore:
    def __init__(self, *, clock_ns=time.monotonic_ns):
        self.clock_ns = clock_ns
        self._remaining = []
        self._parameters = {}
        self._target = None
        self._context = None
        self._proposal_epoch = None
        self._started_ns = self._completed_ns = 0
        self._completed = 0
        self._target_budget_ns = 60_000_000_000
        self._retarget_interval_ns = 1_000_000_000
        self._phase = "NAVIGATING"

    @staticmethod
    def _number(value, name, minimum, maximum):
        if type(value) not in (int, float) or not math.isfinite(value) or not minimum <= value <= maximum:
            raise ValueError(f"{name} must be in {minimum}..{maximum}")
        return float(value)

    def _runtime(self, robot):
        runtime = robot.read("operator.status")
        if not isinstance(runtime, Mapping) or runtime.get("runtime_running") is not True:
            raise ValueError("STRATEGIC_RUNTIME_UNAVAILABLE")
        pid, status = runtime.get("runtime_pid"), runtime.get("status")
        if type(pid) is not int or pid <= 0 or not isinstance(status, Mapping):
            raise ValueError("STRATEGIC_RUNTIME_CONTEXT_UNAVAILABLE")
        stamp = status.get("monotonic_ns")
        if type(stamp) is not int or not 0 <= self.clock_ns() - stamp < 500_000_000:
            raise ValueError("STRATEGIC_CONTEXT_STALE")
        estimate = status.get("estimate")
        quality = estimate.get("localization_quality") if isinstance(estimate, Mapping) else None
        if (not isinstance(quality, Mapping) or quality.get("local_translation") != "GOOD"
                or quality.get("heading") != "GOOD" or quality.get("local_pose_continuous") is not True
                or quality.get("pose_discontinuity") is not False
                or type(quality.get("generation")) is not int):
            raise ValueError("STRATEGIC_LOCALIZATION_UNUSABLE")
        context = (pid, quality["generation"])
        if self._context is not None and context != self._context:
            raise ValueError("STRATEGIC_CONTEXT_CHANGED")
        self._context = context
        return runtime

    def _locations(self, robot):
        result = robot.spatial_query(SpatialQuery(kind="entities", require_current=True, limit=64))
        now = self.clock_ns()
        if not 0 <= now - result.observation_time_ns < 500_000_000:
            raise ValueError("STRATEGIC_PROPOSAL_STALE")
        if self._proposal_epoch is not None and result.clock_epoch != self._proposal_epoch:
            raise ValueError("STRATEGIC_PROPOSAL_EPOCH_CHANGED")
        self._proposal_epoch = result.clock_epoch
        locations = {}
        for entity in result.entities:
            if entity.kind != "place":
                continue
            for fact in entity.facts:
                location, observation = fact.location, fact.observation
                scope = observation.validity_scope if observation is not None else None
                if (fact.attribute == "location" and fact.freshness == "FRESH"
                        and observation.clock_epoch == self._proposal_epoch
                        and location is not None and location.frame_id in {"R2B4_BOOT_ROBOT_MAP", "R2B4_ODOM_LOCAL"}
                        and location.x_m is not None and location.y_m is not None
                        and scope is not None and scope.runtime_pid == self._context[0]
                        and scope.frame_id == location.frame_id):
                    locations[entity.entity_id] = fact
        return locations, result

    def _result(self, outcome="ACTIVE"):
        return {"strategic_target_entity_id": self._target,
                "strategic_target_started_ns": self._started_ns,
                "strategic_target_outcome": outcome, "completed_targets": self._completed}

    def _navigate(self, robot):
        runtime = self._runtime(robot)
        locations, _ = self._locations(robot)
        target = self._remaining[0]
        fact = locations.get(target)
        if fact is None:
            raise ValueError("STRATEGIC_TARGET_STALE")
        status = runtime["status"]
        quality = status["estimate"]["localization_quality"]
        location = fact.location
        if location.frame_id != "R2B4_ODOM_LOCAL" and quality.get("global_position") != "GOOD":
            raise ValueError("STRATEGIC_ALIGNMENT_UNAVAILABLE")
        mode, hz = runtime.get("capture_mode"), runtime.get("capture_hz")
        if mode not in {"alap", "full", "nincs"} or type(hz) is not int or hz <= 0:
            raise ValueError("STRATEGIC_CAPTURE_CONTEXT_UNAVAILABLE")
        parameters = {**self._parameters, "capture": False, "capture_mode": mode,
                      "capture_hz": hz, "expected_runtime_pid": self._context[0]}
        handle = robot.execute("v3.command.navigate", frame_id=location.frame_id,
                               x_m=location.x_m, y_m=location.y_m, **parameters)
        self._target = self._remaining.pop(0)
        self._started_ns, self._phase = self.clock_ns(), "NAVIGATING"
        return handle

    def start(self, robot, parameters):
        params = dict(parameters)
        requested = params.pop("candidate_places", None)
        if requested is not None and (not isinstance(requested, (tuple, list)) or not 1 <= len(requested) <= 32
                or any(not isinstance(item, str) or not 0 < len(item) <= 256 for item in requested)):
            raise ValueError("candidate_places must contain 1..32 place identities")
        maximum = params.pop("max_targets", 8)
        if type(maximum) is not int or not 1 <= maximum <= 32:
            raise ValueError("max_targets must be in 1..32")
        self._target_budget_ns = int(self._number(params.pop("target_budget_s", 60), "target_budget_s", 1, 600) * 1e9)
        self._retarget_interval_ns = int(self._number(params.pop("retarget_interval_s", 1), "retarget_interval_s", .2, 30) * 1e9)
        if set(params) - {"max_v_mps", "max_omega_rad_s", "session_owner_pid", "session_watchdog_s"}:
            raise ValueError("unsupported strategic exploration parameter")
        self._parameters = params
        self._runtime(robot)
        locations, proposal = self._locations(robot)
        names = sorted(locations) if requested is None else list(dict.fromkeys(requested))
        if not names or any(name not in locations for name in names):
            return BehaviorUpdate(BehaviorLifecycle.FAILED, "CURRENT_PLACE_LOCATIONS_REQUIRED:ATLAS_ALIGNMENT_UNAVAILABLE")
        coverage = robot.spatial_query(SpatialQuery(kind="coverage", limit=64))
        history = {}
        for item in coverage.coverage:
            # A semantic place may retain visit history across robot runs.
            # Its old coordinates and monotonic clock never become current.
            history[item.entity_id] = item
        now = self.clock_ns()
        names = [name for name in names if name not in history
                 or history[name].source_clock_epoch != proposal.clock_epoch
                 or (history[name].source_runtime_pid, history[name].source_generation) != self._context
                 or history[name].cooldown_until_ns is None or history[name].cooldown_until_ns <= now]
        def rank(name):
            item = history.get(name)
            if item is None or item.last_visit_ns is None:
                return (0, 0, name)
            current_clock = item.source_clock_epoch == proposal.clock_epoch
            return (2 if current_clock else 1, item.last_visit_ns if current_clock else 0, name)
        names.sort(key=rank)
        self._remaining = names[:maximum]
        if not self._remaining:
            return BehaviorUpdate(BehaviorLifecycle.FAILED, "STRATEGIC_TARGETS_COOLING_DOWN")
        return self._navigate(robot)

    def step(self, robot, state, status):
        mission, navigation = status.get("mission"), status.get("navigation")
        if not isinstance(mission, Mapping) or mission.get("mission_id") != state.mission_id:
            if state.lifecycle is BehaviorLifecycle.ACTIVE:
                return BehaviorUpdate(BehaviorLifecycle.CANCELLED, "MISSION_PREEMPTED", result=self._result("CANCELLED"))
            return None
        self._runtime(robot)
        if (mission.get("mode") != "NAVIGATE" or not isinstance(navigation, Mapping)
                or navigation.get("mission_id") != state.mission_id):
            return BehaviorUpdate(BehaviorLifecycle.FAILED, "STRATEGIC_MISSION_IDENTITY_MISMATCH", result=self._result("FAILED"))
        now = self.clock_ns()
        if self._phase == "COMPLETED":
            if now - self._completed_ns < self._retarget_interval_ns:
                return BehaviorUpdate(BehaviorLifecycle.ACTIVE, "STRATEGIC_COMPLETION_HYSTERESIS", result=self._result("COMPLETED"))
            handle = self._navigate(robot)
            identity = handle.get("command_id") if isinstance(handle, Mapping) else getattr(handle, "command_id", None)
            if not isinstance(identity, str) or not identity or identity == state.command_id:
                raise ValueError("STRATEGIC_NEW_COMMAND_ID_REQUIRED")
            return BehaviorUpdate(BehaviorLifecycle.STARTING, "STRATEGIC_NEXT_TARGET", command_id=identity, result=self._result())
        if navigation.get("status") == "COMPLETE" or mission.get("lifecycle") == "COMPLETED":
            self._completed += 1
            self._completed_ns, self._phase = now, "COMPLETED"
            lifecycle = BehaviorLifecycle.ACTIVE if self._remaining else BehaviorLifecycle.COMPLETED
            return BehaviorUpdate(lifecycle, "STRATEGIC_TARGET_COMPLETED", result=self._result("COMPLETED"))
        if (navigation.get("status") == "NO_PATH" or mission.get("lifecycle") in {"FAILED", "CANCELLED"}
                or now - self._started_ns >= self._target_budget_ns):
            return BehaviorUpdate(BehaviorLifecycle.FAILED, "STRATEGIC_TARGET_EXHAUSTED", result=self._result("FAILED"))
        if status.get("safety_decision") != "ALLOW" and status.get("safety_reason") != "NOT_ACTIVE":
            return BehaviorUpdate(BehaviorLifecycle.FAILED, "STRATEGIC_SAFETY_STOP", result=self._result("FAILED"))
        # Planner HOLD, changed atlas indexes and new place observations do not
        # change this finite command's identity or its existing heartbeat owner.
        return BehaviorUpdate(BehaviorLifecycle.ACTIVE, "STRATEGIC_TARGET_ACTIVE", result=self._result())
