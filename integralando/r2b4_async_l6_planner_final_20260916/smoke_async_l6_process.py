#!/usr/bin/env python3
from __future__ import annotations

import sys
import time
from pathlib import Path


def main() -> int:
    repo = Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
    sys.path.insert(0, str(repo))

    from v3.adapters.l6_planner_process import ProcessTrajectoryRolloutBackend
    from v3.contracts import (
        RobotEstimate,
        RollingLocalCostmap,
        TickContext,
        Waypoint,
        WorldSnapshot,
    )
    from v3.layers.l6_navigation import (
        InlineTrajectoryRolloutBackend,
        NavigationConfig,
        TrajectoryRolloutRequest,
    )

    context = TickContext(10, 1_200_000_000)
    covariance = tuple(0.01 if index % 6 == 0 else 0.0 for index in range(25))
    estimate = RobotEstimate(
        context,
        "R2B4_BOOT_ROBOT_MAP",
        0.10,
        0.0,
        0.0,
        0.05,
        0.0,
        covariance,
    )
    world = WorldSnapshot(
        context,
        "R2B4_BOOT_ROBOT_MAP",
        map_revision=1,
        obstacle_tracks=(),
        freshness_ns=0,
        local_costmap=RollingLocalCostmap(
            "R2B4_BOOT_ROBOT_MAP",
            revision=1,
            resolution_m=0.1,
            radius_m=2.5,
            occupied_cells=(),
            source_sequence=1,
            freshness_ns=0,
        ),
    )
    request = TrajectoryRolloutRequest(
        context=context,
        estimate=estimate,
        world=world,
        goal=Waypoint(0.70, 0.0),
        max_v_mps=0.30,
        max_omega_rad_s=0.60,
        coverage=(),
    )
    config = NavigationConfig()
    inline = InlineTrajectoryRolloutBackend(config)
    expected = inline.take(inline.submit(request))
    assert expected is not None

    worker = ProcessTrajectoryRolloutBackend(
        config,
        worker_cpu=None,
        strict_affinity=False,
    )
    try:
        request_id = worker.submit(request)
        deadline = time.monotonic() + 5.0
        actual = None
        while actual is None and time.monotonic() < deadline:
            actual = worker.take(request_id)
            if actual is None:
                time.sleep(0.01)
        if actual is None:
            raise RuntimeError("process backend did not finish within 5 seconds")
        if actual != expected:
            raise RuntimeError("process backend result differs from inline result")
    finally:
        worker.close()
        inline.close()
    print("PASS: process and inline L6 rollout results are identical")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
