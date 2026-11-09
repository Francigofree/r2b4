"""Compact Nav2 results closed before deterministic V3 execution.

ROS, maps, paths and controller work remain in the navigation edge. These
values carry a proposal; L5 owns mission identity and L9-L12 retain execution
and safety authority.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from .base import TickContext
from .messages import CommandMode, Waypoint


class Nav2NavigationStatus(str, Enum):
    PENDING = "PENDING"
    ACTIVE = "ACTIVE"
    SUCCEEDED = "SUCCEEDED"
    NO_PATH = "NO_PATH"
    UNAVAILABLE = "UNAVAILABLE"
    STALE = "STALE"
    CANCELLED = "CANCELLED"
    INVALID = "INVALID"
    FAILED = "FAILED"


def _identity(value: str, name: str) -> None:
    if not isinstance(value, str) or not value or len(value) > 256:
        raise ValueError(f"{name} must be a bounded nonempty string")


def _nonnegative(value: int, name: str) -> None:
    if type(value) is not int or value < 0:
        raise ValueError(f"{name} must be a nonnegative integer")


@dataclass(frozen=True, slots=True)
class Nav2NavigationRequest:
    context: TickContext
    mission_id: str
    goal_id: str
    mode: CommandMode
    goal: Waypoint | None
    max_v_mps: float
    max_omega_rad_s: float
    odom_generation: int
    transform_revision: int
    # Compatibility defaults decode earlier Nav2 replay checkpoints. Live L6
    # always supplies the canonical mission's actual tolerances explicitly.
    goal_tolerance_m: float = .08
    yaw_tolerance_rad: float = .1

    def __post_init__(self) -> None:
        if not isinstance(self.context, TickContext):
            raise TypeError("navigation request requires TickContext")
        for name in ("mission_id", "goal_id"):
            _identity(getattr(self, name), name)
        if self.mode not in (CommandMode.NAVIGATE, CommandMode.EXPLORE):
            raise ValueError("Nav2 requests require NAVIGATE or EXPLORE")
        if ((self.mode is CommandMode.NAVIGATE and not isinstance(self.goal, Waypoint))
                or (self.mode is CommandMode.EXPLORE and self.goal is not None)):
            raise ValueError("only NAVIGATE carries a fixed map goal")
        for name in ("max_v_mps", "max_omega_rad_s"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        for name in ("odom_generation", "transform_revision"):
            _nonnegative(getattr(self, name), name)
        for name in ("goal_tolerance_m", "yaw_tolerance_rad"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and nonnegative")


@dataclass(frozen=True, slots=True)
class Nav2VelocityGuidance:
    source_context: TickContext
    mission_id: str
    goal_id: str
    status: Nav2NavigationStatus
    worker_generation: int
    command_sequence: int
    produced_ns: int
    valid_until_ns: int
    scan_measurement_ns: int
    odom_measurement_ns: int
    v_mps: float
    omega_rad_s: float
    odom_generation: int
    transform_revision: int
    selected_goal: Waypoint | None = None
    map_revision: int = 0
    child_goal_id: int = 0

    def __post_init__(self) -> None:
        if not isinstance(self.source_context, TickContext) or not isinstance(self.status, Nav2NavigationStatus):
            raise TypeError("Nav2 guidance requires typed source and status")
        for name in ("mission_id", "goal_id"):
            _identity(getattr(self, name), name)
        for name in ("worker_generation", "command_sequence", "produced_ns", "valid_until_ns",
                     "scan_measurement_ns", "odom_measurement_ns", "odom_generation", "transform_revision",
                     "map_revision", "child_goal_id"):
            _nonnegative(getattr(self, name), name)
        if self.selected_goal is not None and not isinstance(self.selected_goal, Waypoint):
            raise TypeError("selected goal must be a Waypoint")
        for name in ("v_mps", "omega_rad_s"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        if not self.source_context.monotonic_ns <= self.produced_ns <= self.valid_until_ns <= self.produced_ns + 200_000_000:
            raise ValueError("Nav2 velocity lifetime exceeds source/TTL bounds")
        if max(self.scan_measurement_ns, self.odom_measurement_ns) > self.produced_ns:
            raise ValueError("Nav2 measurements cannot follow command production")
        if self.status is Nav2NavigationStatus.ACTIVE:
            if min(self.scan_measurement_ns, self.odom_measurement_ns) <= 0:
                raise ValueError("active guidance requires sensor measurement lineage")
        elif self.v_mps != 0.0 or self.omega_rad_s != 0.0:
            raise ValueError("only active Nav2 guidance can propose movement")


@dataclass(frozen=True, slots=True)
class Nav2NavigationInput:
    context: TickContext
    guidance: Nav2VelocityGuidance | None = None
    error: str | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.context, TickContext):
            raise TypeError("navigation input requires TickContext")
        if self.guidance is not None:
            if not isinstance(self.guidance, Nav2VelocityGuidance):
                raise TypeError("navigation guidance must be typed")
            if self.guidance.source_context.tick_id >= self.context.tick_id:
                raise ValueError("Nav2 input must originate from an earlier completed tick")
            if self.guidance.produced_ns > self.context.monotonic_ns:
                raise ValueError("future Nav2 command cannot be closed")
        if self.error is not None:
            _identity(self.error, "navigation error")
            if self.guidance is not None:
                raise ValueError("navigation input cannot contain both guidance and error")
