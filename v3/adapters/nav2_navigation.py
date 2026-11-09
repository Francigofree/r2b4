"""Small, nonblocking Nav2 transport; ROS and map/path payloads stay outside V3."""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
import socket

from v3.contracts import GLOBAL_FRAME_ID, LOCAL_FRAME_ID, TickContext, Waypoint
from v3.contracts.navigation import (
    Nav2NavigationInput, Nav2NavigationRequest, Nav2NavigationStatus, Nav2VelocityGuidance,
)

HEARTBEAT_MAX_AGE_NS = 350_000_000
COMMAND_TTL_NS = 200_000_000
SENSOR_MAX_AGE_NS = 250_000_000
MAX_PACKET_BYTES = 4096


def encode_packet(packet):
    payload = json.dumps(packet, allow_nan=False, separators=(",", ":")).encode()
    if len(payload) > MAX_PACKET_BYTES:
        raise ValueError("Nav2 packet exceeds bound")
    return payload


def decode_packet(payload):
    if len(payload) > MAX_PACKET_BYTES:
        raise ValueError("Nav2 packet exceeds bound")
    packet = json.loads(payload, parse_constant=lambda _: (_ for _ in ()).throw(ValueError("nonfinite JSON")))
    if not isinstance(packet, dict):
        raise ValueError("Nav2 packet must be an object")
    return packet


@dataclass(frozen=True, slots=True)
class Nav2NavigationConfig:
    socket_path: str
    heartbeat_period_s: float = .05

    def __post_init__(self):
        if (not isinstance(self.socket_path, str) or not self.socket_path.startswith("/")
                or len(self.socket_path.encode()) > 100):
            raise ValueError("Nav2 socket requires a bounded absolute path")
        if (isinstance(self.heartbeat_period_s, bool)
                or not math.isfinite(self.heartbeat_period_s)
                or not .02 <= self.heartbeat_period_s <= .1):
            raise ValueError("Nav2 heartbeat period must be between 20 and 100 ms")


def request_packet(request):
    return dict(operation="NAVIGATE", mission_id=request.mission_id, goal_id=request.goal_id,
        source_ns=request.context.monotonic_ns, source_tick_id=request.context.tick_id,
        frame_id=GLOBAL_FRAME_ID, mode=request.mode.value,
        goal=None if request.goal is None else [request.goal.x_m, request.goal.y_m, request.goal.yaw_rad],
        max_v_mps=request.max_v_mps, max_omega_rad_s=request.max_omega_rad_s,
        goal_tolerance_m=request.goal_tolerance_m, yaw_tolerance_rad=request.yaw_tolerance_rad,
        odom_generation=request.odom_generation, transform_revision=request.transform_revision)


def decode_guidance(packet):
    if packet.get("frame_id") != LOCAL_FRAME_ID:
        raise ValueError("Nav2 velocity frame mismatch")
    return Nav2VelocityGuidance(
        source_context=TickContext(packet["source_tick_id"], packet["source_ns"]),
        **{key: packet[key] for key in ("mission_id", "goal_id", "worker_generation",
            "command_sequence", "produced_ns", "valid_until_ns", "scan_measurement_ns",
            "odom_measurement_ns", "v_mps", "omega_rad_s", "odom_generation", "transform_revision")},
        status=Nav2NavigationStatus(packet["status"]),
        selected_goal=None if packet.get("selected_goal") is None else Waypoint(*packet["selected_goal"]),
        map_revision=packet.get("map_revision", 0), child_goal_id=packet.get("child_goal_id", 0))


class Nav2NavigationClient:
    """One latest state, bounded drain, no waits, threads, ROS or map decoding."""

    def __init__(self, config: Nav2NavigationConfig):
        self.config = config
        self._channel = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self._channel.setblocking(False)
        self._channel.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * MAX_PACKET_BYTES)
        self._channel.bind(f"\0r2b4-navigation-{os.getpid()}-{id(self):x}")
        self._request = None
        self._last_sent_ns = -HEARTBEAT_MAX_AGE_NS
        self._guidance = None
        self._error = None
        self._closed = False
        self._highwater = (-1, -1)

    def submit(self, request: Nav2NavigationRequest | None):
        previous, self._request = self._request, request
        changed = (previous is None or request is None or
            (previous.mission_id, previous.goal_id, previous.odom_generation, previous.goal,
             previous.mode, previous.max_v_mps, previous.max_omega_rad_s,
             previous.goal_tolerance_m, previous.yaw_tolerance_rad)
            != (request.mission_id, request.goal_id, request.odom_generation, request.goal,
                request.mode, request.max_v_mps, request.max_omega_rad_s,
                request.goal_tolerance_m, request.yaw_tolerance_rad))
        if changed:
            self._guidance = None
        if self._closed:
            return
        if request is None:
            if previous is not None:
                self._send(dict(operation="CANCEL", mission_id=previous.mission_id,
                                goal_id=previous.goal_id))
            return
        now = request.context.monotonic_ns
        if changed or now-self._last_sent_ns >= int(self.config.heartbeat_period_s * 1e9):
            if self._send(request_packet(request)):
                self._last_sent_ns = now

    def _send(self, packet):
        try:
            self._channel.sendto(encode_packet(packet), self.config.socket_path)
            self._error = None
            return True
        except (OSError, ValueError) as exc:
            self._guidance = None
            self._error = f"NAV2_TRANSPORT:{type(exc).__name__}"
            return False

    def poll(self, context: TickContext) -> Nav2NavigationInput:
        for _ in range(8):
            try:
                data, address = self._channel.recvfrom(MAX_PACKET_BYTES+1)
            except BlockingIOError:
                break
            except OSError as exc:
                self._guidance = None
                self._error = f"NAV2_TRANSPORT:{type(exc).__name__}"
                break
            try:
                if address != self.config.socket_path:
                    raise ValueError("Nav2 sender mismatch")
                guidance = decode_guidance(decode_packet(data))
                request = self._request
                if request is None or (guidance.mission_id, guidance.goal_id, guidance.odom_generation) != (
                        request.mission_id, request.goal_id, request.odom_generation):
                    continue
                if (guidance.source_context.tick_id >= context.tick_id
                        or guidance.source_context.monotonic_ns > context.monotonic_ns
                        or guidance.produced_ns > context.monotonic_ns
                        or guidance.transform_revision > request.transform_revision):
                    raise ValueError("Nav2 future lineage")
                if (guidance.worker_generation, guidance.command_sequence) <= self._highwater:
                    continue
                self._guidance, self._error = guidance, None
                self._highwater = (guidance.worker_generation, guidance.command_sequence)
            except (ValueError, KeyError, TypeError, OverflowError):
                self._guidance = None
                self._error = "NAV2_INVALID_PACKET"
        return Nav2NavigationInput(context, self._guidance, self._error)

    def close(self):
        if not self._closed:
            self.submit(None)
            self._channel.close()
            self._closed = True
