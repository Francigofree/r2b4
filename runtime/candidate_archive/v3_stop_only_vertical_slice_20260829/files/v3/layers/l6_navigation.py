"""L6 inert navigation for the fake-only STOP vertical slice."""

from __future__ import annotations

from v3.contracts import (
    MissionIntent,
    NavigationPlan,
    NavigationStatus,
    RobotEstimate,
    WorldSnapshot,
)


def hold_position(
    mission: MissionIntent,
    estimate: RobotEstimate,
    world: WorldSnapshot,
) -> NavigationPlan:
    return NavigationPlan(
        context=mission.context,
        route=(),
        corridor_radius_m=0.0,
        progress=0.0,
        status=NavigationStatus.IDLE,
    )


__all__ = ["hold_position"]
