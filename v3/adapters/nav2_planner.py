"""Nav2-specific compute edge. ROS and full maps/paths never enter control.

The blocking client is used only in the existing L6 worker process. Recorded
compact guidance is sufficient to repeat the V3 footprint/rollout computation
offline without asking a changing external map for another global path.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import json
import math
import os
import socket
import time

from v3.contracts import GLOBAL_FRAME_ID, LOCAL_FRAME_ID, QualityState, Waypoint
from v3.contracts.planner import GlobalPlanGuidance, GlobalPlanStatus, TrajectoryRolloutRequest
from v3.layers.l6_navigation import TrajectoryRolloutComputer

MAX_PACKET_BYTES = 4096


@dataclass(frozen=True, slots=True)
class Nav2PlannerConfig:
    socket_path: str
    request_timeout_s: float = .18
    max_map_age_s: float = 6.

    def __post_init__(self):
        if not isinstance(self.socket_path, str) or not self.socket_path.startswith("/") or len(self.socket_path.encode()) > 100:
            raise ValueError("Nav2 socket requires a bounded absolute path")
        for value in (self.request_timeout_s, self.max_map_age_s):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError("Nav2 deadlines must be finite and positive")
        if self.request_timeout_s > 1. or self.max_map_age_s > 30.:
            raise ValueError("Nav2 timeout/map age exceed bridge bounds")


def encode_packet(value):
    data = json.dumps(value, separators=(",", ":"), allow_nan=False).encode()
    if len(data) > MAX_PACKET_BYTES:
        raise ValueError("Nav2 packet exceeds bound")
    return data


def decode_packet(data):
    if len(data) > MAX_PACKET_BYTES:
        raise ValueError("Nav2 packet exceeds bound")
    value = json.loads(data, parse_constant=lambda _: (_ for _ in ()).throw(ValueError("nonfinite JSON")))
    if not isinstance(value, dict):
        raise ValueError("Nav2 packet must be an object")
    return value


def guidance_for(request, status, **changes):
    return GlobalPlanGuidance(request.context, status, LOCAL_FRAME_ID,
        request.estimate.localization_quality.generation, request.estimate.transform_revision, **changes)


def validate_guidance(config, request, guidance):
    if (request.global_goal is None or guidance.source_context != request.context
            or guidance.frame_id != request.world.frame_id
            or guidance.odom_generation != request.estimate.localization_quality.generation
            or guidance.transform_revision != request.estimate.transform_revision):
        raise ValueError("NAV2_GUIDANCE_LINEAGE_MISMATCH")
    if guidance.status is GlobalPlanStatus.READY:
        goal = guidance.local_goal
        if (guidance.map_revision <= 0 or guidance.map_stamp_ns <= 0
                or math.hypot(goal.x_m-request.estimate.x_m, goal.y_m-request.estimate.y_m)
                > config.local_goal_distance_m + 1e-6):
            raise ValueError("NAV2_GUIDANCE_BOUNDS_MISMATCH")


def compute_with_guidance(config, request, guidance, *, computer=None):
    validate_guidance(config, request, guidance)
    kernel = computer or TrajectoryRolloutComputer(config)
    local = replace(request, goal=guidance.local_goal or request.goal, global_goal=None)
    result = kernel.compute(local)
    if guidance.status is not GlobalPlanStatus.READY:
        result = replace(result, trajectory_candidates=tuple(
            replace(candidate, progress_viable=False) for candidate in result.trajectory_candidates),
            goal_reachable=False)
    return replace(result, global_guidance=guidance,
                   selected_goal=guidance.local_goal or request.goal)


def compute_recorded_planner_result(config, request, recorded):
    """Recompute V3 local proof from the closed external waypoint suggestion."""
    if recorded.global_guidance is None:
        if config.global_planner_enabled and request.global_goal is not None:
            raise ValueError("NAV2_GUIDANCE_MISSING")
        return TrajectoryRolloutComputer(config).compute(request)
    return compute_with_guidance(config, request, recorded.global_guidance)


class Nav2PlannerClient:
    """One bounded request at a time; caller must be the isolated L6 worker."""

    def __init__(self, config):
        self.config = config
        self.sequence = 0

    def plan(self, request, navigation_config):
        estimate = request.estimate
        transform = estimate.map_to_odom
        if (request.global_goal is None or transform is None
                or transform.frame_id != GLOBAL_FRAME_ID
                or estimate.frame_id != LOCAL_FRAME_ID
                or estimate.localization_quality.global_position is not QualityState.GOOD):
            return guidance_for(request, GlobalPlanStatus.INVALID)
        self.sequence += 1
        identity = {"request_id": self.sequence, "source_ns": request.context.monotonic_ns,
                    "tick_id": request.context.tick_id,
                    "odom_generation": estimate.localization_quality.generation,
                    "transform_revision": estimate.transform_revision}
        start = transform.apply(estimate.x_m, estimate.y_m, estimate.yaw_rad)
        packet = {**identity, "frame_id": GLOBAL_FRAME_ID,
                  "start": list(start), "goal": [request.global_goal.x_m, request.global_goal.y_m,
                      request.global_goal.yaw_rad],
                  "map_to_odom": [transform.x_m, transform.y_m, transform.yaw_rad],
                  "lookahead_m": navigation_config.local_goal_distance_m,
                  "max_map_age_ns": int(self.config.max_map_age_s * 1e9),
                  "deadline_ns": time.monotonic_ns() + int(self.config.request_timeout_s * 1e9)}
        channel = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        try:
            channel.settimeout(self.config.request_timeout_s)
            channel.bind(f"\0r2b4-nav2-{os.getpid()}-{self.sequence}")
            channel.connect(self.config.socket_path)
            channel.send(encode_packet(packet))
            reply = decode_packet(channel.recv(MAX_PACKET_BYTES + 1))
            if (any(type(reply.get(key)) is not int or reply[key] != value for key, value in identity.items())
                    or reply.get("frame_id") != LOCAL_FRAME_ID):
                return guidance_for(request, GlobalPlanStatus.INVALID)
            status = GlobalPlanStatus(reply["status"])
            if time.monotonic_ns() > packet["deadline_ns"]:
                return guidance_for(request, GlobalPlanStatus.TIMEOUT)
            if status is not GlobalPlanStatus.READY:
                return guidance_for(request, status)
            goal = reply["local_goal"]
            if not isinstance(goal, list) or len(goal) != 3:
                raise ValueError("invalid Nav2 local goal")
            guidance = guidance_for(request, status, local_goal=Waypoint(*goal),
                                    map_revision=reply["map_revision"], map_stamp_ns=reply["map_stamp_ns"])
            validate_guidance(navigation_config, request, guidance)
            return guidance
        except (TimeoutError, socket.timeout):
            return guidance_for(request, GlobalPlanStatus.TIMEOUT)
        except OSError:
            return guidance_for(request, GlobalPlanStatus.UNAVAILABLE)
        except (ValueError, KeyError, TypeError, OverflowError):
            return guidance_for(request, GlobalPlanStatus.INVALID)
        finally:
            channel.close()
