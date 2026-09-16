#!/usr/bin/env python3
from __future__ import annotations

import os
import shutil
import sys
from pathlib import Path


TARGET = Path("v3/layers/l6_navigation.py")
MARKER = "trajectory_replan_min_tick_gap"


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly 1 source anchor, found {count}")
    return text.replace(old, new, 1)


def replace_count(text: str, old: str, new: str, expected: int, label: str) -> str:
    count = text.count(old)
    if count != expected:
        raise RuntimeError(f"{label}: expected {expected} source anchors, found {count}")
    return text.replace(old, new)


def main() -> int:
    repo = Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
    target = repo / TARGET
    if not target.is_file():
        raise RuntimeError(f"missing target: {target}")

    text = target.read_text(encoding="utf-8")
    if MARKER in text:
        print(f"already installed: {TARGET}")
        return 0

    text = replace_once(
        text,
        '''    trajectory_replan_interval_ns: int = 100_000_000
    coverage_cell_size_m: float = 0.35
''',
        '''    trajectory_replan_interval_ns: int = 100_000_000
    # Preserve the nominal 10 Hz replan cadence at a 50 Hz control rate,
    # while guaranteeing cheap control ticks after an over-budget replan.
    trajectory_replan_min_tick_gap: int = 5
    coverage_cell_size_m: float = 0.35
''',
        "NavigationConfig",
    )

    text = replace_once(
        text,
        '''            "rollout_step_count",
            "trajectory_replan_interval_ns",
        ):
''',
        '''            "rollout_step_count",
            "trajectory_replan_interval_ns",
            "trajectory_replan_min_tick_gap",
        ):
''',
        "NavigationConfig validation",
    )

    text = replace_once(
        text,
        '''    goal_selected_ns: int
    last_replan_ns: int | None
    trajectory_candidates: tuple[TrajectoryEvaluation, ...]
''',
        '''    goal_selected_ns: int
    last_replan_ns: int | None
    last_replan_tick_id: int | None
    trajectory_candidates: tuple[TrajectoryEvaluation, ...]
''',
        "NavigationStateCheckpoint",
    )

    text = replace_once(
        text,
        '''        "_initial_distance_m",
        "_last_replan_ns",
        "_local_goal",
''',
        '''        "_initial_distance_m",
        "_last_replan_ns",
        "_last_replan_tick_id",
        "_local_goal",
''',
        "TrajectoryNavigator slots",
    )

    text = replace_once(
        text,
        '''        self._last_replan_ns: int | None = None
        self._static_planning_index: _StaticPlanningIndex | None = None
''',
        '''        self._last_replan_ns: int | None = None
        self._last_replan_tick_id: int | None = None
        self._static_planning_index: _StaticPlanningIndex | None = None
''',
        "TrajectoryNavigator init",
    )

    text = replace_once(
        text,
        '''            self._goal_selected_ns,
            self._last_replan_ns,
            self._trajectory_candidates,
''',
        '''            self._goal_selected_ns,
            self._last_replan_ns,
            self._last_replan_tick_id,
            self._trajectory_candidates,
''',
        "checkpoint save",
    )

    text = replace_once(
        text,
        '''        self._goal_selected_ns = checkpoint.goal_selected_ns
        self._last_replan_ns = checkpoint.last_replan_ns
        self._trajectory_candidates = checkpoint.trajectory_candidates
''',
        '''        self._goal_selected_ns = checkpoint.goal_selected_ns
        self._last_replan_ns = checkpoint.last_replan_ns
        self._last_replan_tick_id = checkpoint.last_replan_tick_id
        self._trajectory_candidates = checkpoint.trajectory_candidates
''',
        "checkpoint restore",
    )

    text = replace_count(
        text,
        'if self._replan_due(mission.context.monotonic_ns):',
        '''if self._replan_due(
            mission.context.monotonic_ns,
            mission.context.tick_id,
        ):''',
        2,
        "replan call sites",
    )

    text = replace_once(
        text,
        '''            self._store_trajectory_plan(
                mission.context.monotonic_ns,
                local_goal,
''',
        '''            self._store_trajectory_plan(
                mission.context.monotonic_ns,
                mission.context.tick_id,
                local_goal,
''',
        "NAVIGATE replan store",
    )

    text = replace_once(
        text,
        '''            self._store_trajectory_plan(
                mission.context.monotonic_ns,
                goal,
''',
        '''            self._store_trajectory_plan(
                mission.context.monotonic_ns,
                mission.context.tick_id,
                goal,
''',
        "EXPLORE replan store",
    )

    text = replace_once(
        text,
        '''    def _replan_due(self, monotonic_ns: int) -> bool:
        previous = self._last_replan_ns
        return (
            previous is None
            or not self._trajectory_candidates
            or monotonic_ns - previous >= self._config.trajectory_replan_interval_ns
        )
''',
        '''    def _replan_due(self, monotonic_ns: int, tick_id: int) -> bool:
        previous_ns = self._last_replan_ns
        previous_tick_id = self._last_replan_tick_id
        if (
            previous_ns is None
            or previous_tick_id is None
            or not self._trajectory_candidates
        ):
            return True
        return (
            monotonic_ns - previous_ns >= self._config.trajectory_replan_interval_ns
            and tick_id - previous_tick_id
            >= self._config.trajectory_replan_min_tick_gap
        )
''',
        "replan scheduler",
    )

    text = replace_once(
        text,
        '''    def _store_trajectory_plan(
        self,
        monotonic_ns: int,
        local_goal: Waypoint,
        candidates: tuple[TrajectoryEvaluation, ...],
    ) -> None:
        self._last_replan_ns = monotonic_ns
        self._local_goal = local_goal
        self._trajectory_candidates = candidates
''',
        '''    def _store_trajectory_plan(
        self,
        monotonic_ns: int,
        tick_id: int,
        local_goal: Waypoint,
        candidates: tuple[TrajectoryEvaluation, ...],
    ) -> None:
        self._last_replan_ns = monotonic_ns
        self._last_replan_tick_id = tick_id
        self._local_goal = local_goal
        self._trajectory_candidates = candidates
''',
        "replan state store",
    )

    text = replace_once(
        text,
        '''    def _clear_trajectory_plan(self) -> None:
        self._last_replan_ns = None
        self._trajectory_candidates = ()
''',
        '''    def _clear_trajectory_plan(self) -> None:
        self._last_replan_ns = None
        self._last_replan_tick_id = None
        self._trajectory_candidates = ()
''',
        "replan state clear",
    )

    text = replace_once(
        text,
        '''        self._goal_selected_ns = 0
        self._last_replan_ns = None
        self._trajectory_candidates = ()
''',
        '''        self._goal_selected_ns = 0
        self._last_replan_ns = None
        self._last_replan_tick_id = None
        self._trajectory_candidates = ()
''',
        "replan state reset",
    )

    backup = target.with_suffix(target.suffix + ".pre_replan_cooldown")
    if not backup.exists():
        shutil.copy2(target, backup)

    tmp = target.with_suffix(target.suffix + f".tmp.{os.getpid()}")
    tmp.write_text(text, encoding="utf-8")
    os.replace(tmp, target)

    print(f"updated: {TARGET}")
    print(f"backup:  {backup.relative_to(repo)}")
    print("L6 replan cooldown installed: 100 ms AND minimum 5 tick gap")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
