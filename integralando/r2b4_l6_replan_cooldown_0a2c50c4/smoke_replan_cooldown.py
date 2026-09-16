#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

repo = Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
sys.path.insert(0, str(repo))

from v3.layers.l6_navigation import NavigationConfig, TrajectoryNavigator

cfg = NavigationConfig(
    trajectory_replan_interval_ns=100_000_000,
    trajectory_replan_min_tick_gap=5,
)
nav = TrajectoryNavigator(cfg)

# Simulate one completed heavy replan at tick 100 / t=0.
nav._last_replan_ns = 0
nav._last_replan_tick_id = 100
nav._trajectory_candidates = (object(),)

# Wall time can jump past 100 ms, but the next tick must remain cheap.
assert nav._replan_due(150_000_000, 101) is False
assert nav._replan_due(170_000_000, 104) is False

# Normal 50 Hz cadence: fifth tick may replan again.
assert nav._replan_due(100_000_000, 105) is True

# Tick gap alone must not force an early replan.
assert nav._replan_due(99_999_999, 105) is False

checkpoint = nav.checkpoint()
assert checkpoint.last_replan_ns == 0
assert checkpoint.last_replan_tick_id == 100

restored = TrajectoryNavigator(cfg)
restored.restore(checkpoint)
assert restored._last_replan_ns == 0
assert restored._last_replan_tick_id == 100
assert restored._replan_due(150_000_000, 101) is False

print("L6_REPLAN_COOLDOWN_SMOKE=PASS")
