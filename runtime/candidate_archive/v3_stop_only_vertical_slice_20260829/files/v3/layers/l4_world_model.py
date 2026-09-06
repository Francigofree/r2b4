"""L4 empty world model for the fake-only STOP vertical slice."""

from __future__ import annotations

from v3.contracts import AdmittedFrame, RobotEstimate, WorldSnapshot


def build_empty_world(frame: AdmittedFrame, estimate: RobotEstimate) -> WorldSnapshot:
    return WorldSnapshot(
        frame.context,
        frame_id=estimate.frame_id,
        map_revision=0,
        obstacle_tracks=(),
        freshness_ns=0,
    )


__all__ = ["build_empty_world"]
