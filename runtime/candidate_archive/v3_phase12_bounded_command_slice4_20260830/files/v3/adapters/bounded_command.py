"""Deterministic finite command window for controlled V3 motion tests."""

from __future__ import annotations

import math
from dataclasses import dataclass

from v3.contracts import CommandMode, CommandRequest, DataField, TickContext


def _nonnegative_integer(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


def _finite(value: object, name: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
    ):
        raise ValueError(f"{name} must be finite numeric")
    return float(value)


@dataclass(frozen=True, slots=True)
class BoundedTeleopProfile:
    """One immutable TELEOP target inside one absolute, finite tick window."""

    command_id: str
    start_tick_id: int
    active_tick_count: int
    v_mps: float
    omega_rad_s: float
    max_v_mps: float
    max_omega_rad_s: float

    def __post_init__(self) -> None:
        if not isinstance(self.command_id, str):
            raise ValueError("command_id must be a non-empty token")
        try:
            DataField(self.command_id, None)
        except ValueError as exc:
            raise ValueError("command_id must be a non-empty token") from exc
        _nonnegative_integer(self.start_tick_id, "start_tick_id")
        active_tick_count = _nonnegative_integer(
            self.active_tick_count,
            "active_tick_count",
        )
        if active_tick_count == 0:
            raise ValueError("active_tick_count must be positive")

        v_mps = _finite(self.v_mps, "v_mps")
        omega_rad_s = _finite(self.omega_rad_s, "omega_rad_s")
        max_v_mps = _finite(self.max_v_mps, "max_v_mps")
        max_omega_rad_s = _finite(self.max_omega_rad_s, "max_omega_rad_s")
        if max_v_mps <= 0.0 or max_omega_rad_s <= 0.0:
            raise ValueError("profile motion limits must be positive")
        if abs(v_mps) > max_v_mps:
            raise ValueError("abs(v_mps) cannot exceed max_v_mps")
        if abs(omega_rad_s) > max_omega_rad_s:
            raise ValueError("abs(omega_rad_s) cannot exceed max_omega_rad_s")

    @property
    def end_tick_id(self) -> int:
        """Return the exclusive end of the configured command window."""

        return self.start_tick_id + self.active_tick_count


class BoundedTeleopCommandGateway:
    """Emit one bounded TELEOP profile and STOP everywhere outside it."""

    __slots__ = ("_profile",)

    def __init__(self, profile: BoundedTeleopProfile) -> None:
        if not isinstance(profile, BoundedTeleopProfile):
            raise TypeError("profile must be BoundedTeleopProfile")
        self._profile = profile

    def snapshot(self, context: TickContext) -> CommandRequest:
        if not isinstance(context, TickContext):
            raise TypeError("context must be TickContext")
        if self._profile.start_tick_id <= context.tick_id < self._profile.end_tick_id:
            return CommandRequest(
                context=context,
                command_id=self._profile.command_id,
                mode=CommandMode.TELEOP,
                goal=(
                    DataField("v_mps", self._profile.v_mps),
                    DataField("omega_rad_s", self._profile.omega_rad_s),
                    DataField("max_v_mps", self._profile.max_v_mps),
                    DataField("max_omega_rad_s", self._profile.max_omega_rad_s),
                ),
                expiry_tick=self._profile.end_tick_id - 1,
            )
        return CommandRequest(
            context=context,
            command_id=f"{self._profile.command_id}.stop.{context.tick_id}",
            mode=CommandMode.STOP,
            goal=(),
            expiry_tick=context.tick_id,
        )


__all__ = ["BoundedTeleopCommandGateway", "BoundedTeleopProfile"]
