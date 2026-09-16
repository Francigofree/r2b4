#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

BASE_COMMIT = '3236cada2eb12b7f63944ffa1fc55c47b39ae8be'
PATCHES = {'v3/layers/l6_navigation.py': [('    pending_rollout_request: TrajectoryRolloutRequest | None = None\n    pending_goal_selected_ns: int | None = None\n    pending_release_tick_id: int | None = None\n', '    pending_rollout_request: TrajectoryRolloutRequest | None = None\n    pending_goal_selected_ns: int | None = None\n    pending_release_tick_id: int | None = None\n    pending_release_not_before_ns: int | None = None\n'), ('@dataclass(frozen=True, slots=True)\nclass AsyncL6PlannerConfig:\n    """Deterministic handoff policy; process placement is not layer state."""\n\n    enabled: bool = False\n    release_tick_gap: int = 5\n    max_plan_age_ns: int = 350_000_000\n\n    def __post_init__(self) -> None:\n        if type(self.enabled) is not bool:\n            raise TypeError("enabled must be bool")\n        for value, name in (\n            (self.release_tick_gap, "release_tick_gap"),\n            (self.max_plan_age_ns, "max_plan_age_ns"),\n        ):\n            if not isinstance(value, int) or isinstance(value, bool) or value <= 0:\n                raise ValueError(f"{name} must be a positive integer")\n', '@dataclass(frozen=True, slots=True)\nclass AsyncL6PlannerConfig:\n    """Deterministic handoff policy; process placement is not layer state."""\n\n    enabled: bool = False\n    # Legacy replay compatibility. New production handoffs use release_delay_ns.\n    release_tick_gap: int = 5\n    max_plan_age_ns: int = 350_000_000\n    release_delay_ns: int | None = None\n\n    def __post_init__(self) -> None:\n        if type(self.enabled) is not bool:\n            raise TypeError("enabled must be bool")\n        for value, name in (\n            (self.release_tick_gap, "release_tick_gap"),\n            (self.max_plan_age_ns, "max_plan_age_ns"),\n        ):\n            if not isinstance(value, int) or isinstance(value, bool) or value <= 0:\n                raise ValueError(f"{name} must be a positive integer")\n        if self.release_delay_ns is not None:\n            if (\n                not isinstance(self.release_delay_ns, int)\n                or isinstance(self.release_delay_ns, bool)\n                or self.release_delay_ns <= 0\n            ):\n                raise ValueError("release_delay_ns must be a positive integer or None")\n            if self.release_delay_ns >= self.max_plan_age_ns:\n                raise ValueError(\n                    "release_delay_ns must be shorter than max_plan_age_ns"\n                )\n'), ('class InlineTrajectoryRolloutBackend:\n    """Pure replay/test backend; compute now, reveal only on L6 release tick."""\n', 'class InlineTrajectoryRolloutBackend:\n    """Pure replay/test backend; L6 owns deterministic result visibility."""\n'), ('        "_pending_goal_selected_ns",\n        "_pending_release_tick_id",\n        "_pending_rollout_id",\n', '        "_pending_goal_selected_ns",\n        "_pending_release_not_before_ns",\n        "_pending_release_tick_id",\n        "_pending_rollout_id",\n'), ('        "_rollout_backend",\n        "_rollout_release_tick_gap",\n        "_static_planning_index",\n', '        "_rollout_backend",\n        "_rollout_release_delay_ns",\n        "_rollout_release_tick_gap",\n        "_static_planning_index",\n'), ('        rollout_backend: TrajectoryRolloutBackend | None = None,\n        rollout_release_tick_gap: int = 5,\n        max_plan_age_ns: int = 350_000_000,\n    ) -> None:\n', '        rollout_backend: TrajectoryRolloutBackend | None = None,\n        rollout_release_tick_gap: int = 5,\n        rollout_release_delay_ns: int | None = None,\n        max_plan_age_ns: int = 350_000_000,\n    ) -> None:\n'), ('        for value, name in (\n            (rollout_release_tick_gap, "rollout_release_tick_gap"),\n            (max_plan_age_ns, "max_plan_age_ns"),\n        ):\n            if not isinstance(value, int) or isinstance(value, bool) or value <= 0:\n                raise ValueError(f"{name} must be a positive integer")\n        self._config = config\n        self._rollout_backend = rollout_backend\n        self._rollout_release_tick_gap = rollout_release_tick_gap\n        self._max_plan_age_ns = max_plan_age_ns\n', '        for value, name in (\n            (rollout_release_tick_gap, "rollout_release_tick_gap"),\n            (max_plan_age_ns, "max_plan_age_ns"),\n        ):\n            if not isinstance(value, int) or isinstance(value, bool) or value <= 0:\n                raise ValueError(f"{name} must be a positive integer")\n        if rollout_release_delay_ns is not None:\n            if (\n                not isinstance(rollout_release_delay_ns, int)\n                or isinstance(rollout_release_delay_ns, bool)\n                or rollout_release_delay_ns <= 0\n            ):\n                raise ValueError(\n                    "rollout_release_delay_ns must be a positive integer or None"\n                )\n            if rollout_release_delay_ns >= max_plan_age_ns:\n                raise ValueError(\n                    "rollout_release_delay_ns must be shorter than max_plan_age_ns"\n                )\n        self._config = config\n        self._rollout_backend = rollout_backend\n        self._rollout_release_tick_gap = rollout_release_tick_gap\n        self._rollout_release_delay_ns = rollout_release_delay_ns\n        self._max_plan_age_ns = max_plan_age_ns\n'), ('        self._pending_rollout_request: TrajectoryRolloutRequest | None = None\n        self._pending_goal_selected_ns: int | None = None\n        self._pending_release_tick_id: int | None = None\n        self._static_planning_index: _StaticPlanningIndex | None = None\n', '        self._pending_rollout_request: TrajectoryRolloutRequest | None = None\n        self._pending_goal_selected_ns: int | None = None\n        self._pending_release_tick_id: int | None = None\n        self._pending_release_not_before_ns: int | None = None\n        self._static_planning_index: _StaticPlanningIndex | None = None\n'), ('            self._pending_rollout_request,\n            self._pending_goal_selected_ns,\n            self._pending_release_tick_id,\n        )\n', '            self._pending_rollout_request,\n            self._pending_goal_selected_ns,\n            self._pending_release_tick_id,\n            self._pending_release_not_before_ns,\n        )\n'), ('            release_tick_id = checkpoint.pending_release_tick_id\n            if release_tick_id is None:\n                raise RuntimeError("async navigation checkpoint lacks release tick")\n            request_id = backend.submit(pending)\n            self._pending_rollout_id = request_id\n            self._pending_rollout_request = pending\n            self._pending_goal_selected_ns = checkpoint.pending_goal_selected_ns\n            self._pending_release_tick_id = release_tick_id\n', '            release_tick_id = checkpoint.pending_release_tick_id\n            release_not_before_ns = checkpoint.pending_release_not_before_ns\n            if (release_tick_id is None) == (release_not_before_ns is None):\n                raise RuntimeError(\n                    "async navigation checkpoint must contain exactly one release criterion"\n                )\n            if (\n                release_not_before_ns is not None\n                and release_not_before_ns <= pending.context.monotonic_ns\n            ):\n                raise RuntimeError(\n                    "async navigation checkpoint release time is not after request time"\n                )\n            request_id = backend.submit(pending)\n            self._pending_rollout_id = request_id\n            self._pending_rollout_request = pending\n            self._pending_goal_selected_ns = checkpoint.pending_goal_selected_ns\n            self._pending_release_tick_id = release_tick_id\n            self._pending_release_not_before_ns = release_not_before_ns\n'), ('    def _replan_due(self, monotonic_ns: int, tick_id: int) -> bool:\n        if self._pending_rollout_id is not None:\n            return False\n        previous_ns = self._last_replan_ns\n        previous_tick_id = self._last_replan_tick_id\n        if (\n            previous_ns is None\n            or previous_tick_id is None\n            or not self._trajectory_candidates\n        ):\n            return True\n        return (\n            monotonic_ns - previous_ns >= self._config.trajectory_replan_interval_ns\n            and tick_id - previous_tick_id\n            >= self._config.trajectory_replan_min_tick_gap\n        )\n', '    def _replan_due(self, monotonic_ns: int, tick_id: int) -> bool:\n        if self._pending_rollout_id is not None:\n            return False\n        previous_ns = self._last_replan_ns\n        previous_tick_id = self._last_replan_tick_id\n        if (\n            previous_ns is None\n            or previous_tick_id is None\n            or not self._trajectory_candidates\n        ):\n            return True\n        if monotonic_ns - previous_ns < self._config.trajectory_replan_interval_ns:\n            return False\n        # New async production timing is monotonic-time based. A pending request\n        # already bounds work to one rollout, so a second tick-count gate would\n        # only turn runtime jitter into planner latency. Legacy tick-mode replay\n        # and the synchronous planner retain the historic cooldown.\n        if self._rollout_backend is not None and self._rollout_release_delay_ns is not None:\n            return True\n        return (\n            tick_id - previous_tick_id\n            >= self._config.trajectory_replan_min_tick_gap\n        )\n'), ('        self._pending_rollout_id = request_id\n        self._pending_rollout_request = request\n        self._pending_goal_selected_ns = goal_selected_ns\n        self._pending_release_tick_id = context.tick_id + self._rollout_release_tick_gap\n', '        self._pending_rollout_id = request_id\n        self._pending_rollout_request = request\n        self._pending_goal_selected_ns = goal_selected_ns\n        if self._rollout_release_delay_ns is None:\n            # Legacy capture/replay mode: preserve the historical tick gate.\n            self._pending_release_tick_id = (\n                context.tick_id + self._rollout_release_tick_gap\n            )\n            self._pending_release_not_before_ns = None\n        else:\n            # Production mode: one canonical monotonic-time authority. The\n            # first closed tick at/after this threshold owns the handoff.\n            self._pending_release_tick_id = None\n            self._pending_release_not_before_ns = (\n                context.monotonic_ns + self._rollout_release_delay_ns\n            )\n'), ('    def _accept_pending_rollout(self, context: TickContext) -> bool:\n        request_id = self._pending_rollout_id\n        if request_id is None:\n            return False\n        request = self._pending_rollout_request\n        release_tick_id = self._pending_release_tick_id\n        backend = self._rollout_backend\n        if request is None or release_tick_id is None or backend is None:\n            raise RuntimeError("async rollout pending state is incomplete")\n        source_context = request.context\n        goal = request.goal\n        if context.tick_id < release_tick_id:\n            # Before the deterministic handoff tick, the previously accepted\n            # plan is still authoritative, so its age remains safety-critical.\n            if (\n                self._last_replan_ns is not None\n                and context.monotonic_ns - self._last_replan_ns > self._max_plan_age_ns\n            ):\n                raise RuntimeError("ASYNC_L6_PLAN_STALE")\n            return False\n        if context.tick_id > release_tick_id:\n            raise RuntimeError("ASYNC_L6_RELEASE_TICK_MISSED")\n\n        # At the handoff tick, inspect the new result before judging plan age.\n        # The previous plan may have crossed max_plan_age_ns while the new\n        # rollout is already ready and fresh enough to replace it.\n        result = backend.take(request_id)\n        if result is None:\n            raise RuntimeError("ASYNC_L6_DEADLINE_MISSED")\n        if result.source_context != source_context:\n            raise RuntimeError("ASYNC_L6_SOURCE_CONTEXT_MISMATCH")\n        if (\n            context.monotonic_ns - result.source_context.monotonic_ns\n            > self._max_plan_age_ns\n        ):\n            raise RuntimeError("ASYNC_L6_PLAN_STALE")\n        selected_ns = self._pending_goal_selected_ns\n        self._pending_rollout_id = None\n        self._pending_rollout_request = None\n        self._pending_goal_selected_ns = None\n        self._pending_release_tick_id = None\n        if selected_ns is not None:\n            self._goal_selected_ns = selected_ns\n        self._store_trajectory_plan(\n            result.source_context.monotonic_ns,\n            result.source_context.tick_id,\n            goal,\n            result.trajectory_candidates,\n        )\n        return True\n', '    def _accept_pending_rollout(self, context: TickContext) -> bool:\n        request_id = self._pending_rollout_id\n        if request_id is None:\n            return False\n        request = self._pending_rollout_request\n        release_tick_id = self._pending_release_tick_id\n        release_not_before_ns = self._pending_release_not_before_ns\n        backend = self._rollout_backend\n        if (\n            request is None\n            or backend is None\n            or (release_tick_id is None) == (release_not_before_ns is None)\n        ):\n            raise RuntimeError("async rollout pending state is incomplete")\n        source_context = request.context\n        goal = request.goal\n\n        if release_not_before_ns is not None:\n            before_handoff = context.monotonic_ns < release_not_before_ns\n        else:\n            assert release_tick_id is not None\n            before_handoff = context.tick_id < release_tick_id\n            if context.tick_id > release_tick_id:\n                raise RuntimeError("ASYNC_L6_RELEASE_TICK_MISSED")\n\n        if before_handoff:\n            # Until the deterministic handoff boundary the previously accepted\n            # plan remains authoritative, so its age is still safety-critical.\n            if (\n                self._last_replan_ns is not None\n                and context.monotonic_ns - self._last_replan_ns > self._max_plan_age_ns\n            ):\n                raise RuntimeError("ASYNC_L6_PLAN_STALE")\n            return False\n\n        # At the deterministic handoff boundary inspect the replacement first.\n        # Its immutable source snapshot, not scheduler jitter, defines freshness.\n        result = backend.take(request_id)\n        if result is None:\n            raise RuntimeError("ASYNC_L6_DEADLINE_MISSED")\n        if result.source_context != source_context:\n            raise RuntimeError("ASYNC_L6_SOURCE_CONTEXT_MISMATCH")\n        if (\n            context.monotonic_ns - result.source_context.monotonic_ns\n            > self._max_plan_age_ns\n        ):\n            raise RuntimeError("ASYNC_L6_PLAN_STALE")\n        selected_ns = self._pending_goal_selected_ns\n        self._pending_rollout_id = None\n        self._pending_rollout_request = None\n        self._pending_goal_selected_ns = None\n        self._pending_release_tick_id = None\n        self._pending_release_not_before_ns = None\n        if selected_ns is not None:\n            self._goal_selected_ns = selected_ns\n        self._store_trajectory_plan(\n            result.source_context.monotonic_ns,\n            result.source_context.tick_id,\n            goal,\n            result.trajectory_candidates,\n        )\n        return True\n'), ('        self._pending_rollout_id = None\n        self._pending_rollout_request = None\n        self._pending_goal_selected_ns = None\n        self._pending_release_tick_id = None\n', '        self._pending_rollout_id = None\n        self._pending_rollout_request = None\n        self._pending_goal_selected_ns = None\n        self._pending_release_tick_id = None\n        self._pending_release_not_before_ns = None\n')], 'v3/composition/native_control.py': [('    async_l6 = AsyncL6PlannerConfig(\n        enabled=async_enabled,\n        release_tick_gap=_positive_int(\n            async_mapping.get("release_tick_gap", 5),\n            "v3_navigation.async_l6.release_tick_gap",\n        ),\n        max_plan_age_ns=_positive_int(\n            async_mapping.get("max_plan_age_ns", 350_000_000),\n            "v3_navigation.async_l6.max_plan_age_ns",\n        ),\n    )\n', '    release_delay_value = async_mapping.get("release_delay_ns")\n    release_delay_ns = (\n        None\n        if release_delay_value is None\n        else _positive_int(\n            release_delay_value,\n            "v3_navigation.async_l6.release_delay_ns",\n        )\n    )\n    async_l6 = AsyncL6PlannerConfig(\n        enabled=async_enabled,\n        release_tick_gap=_positive_int(\n            async_mapping.get("release_tick_gap", 5),\n            "v3_navigation.async_l6.release_tick_gap",\n        ),\n        max_plan_age_ns=_positive_int(\n            async_mapping.get("max_plan_age_ns", 350_000_000),\n            "v3_navigation.async_l6.max_plan_age_ns",\n        ),\n        release_delay_ns=release_delay_ns,\n    )\n'), ('                rollout_backend=backend,\n                rollout_release_tick_gap=config.async_l6.release_tick_gap,\n                max_plan_age_ns=config.async_l6.max_plan_age_ns,\n', '                rollout_backend=backend,\n                rollout_release_tick_gap=config.async_l6.release_tick_gap,\n                rollout_release_delay_ns=config.async_l6.release_delay_ns,\n                max_plan_age_ns=config.async_l6.max_plan_age_ns,\n')], 'conf/vezerles.json': [('    "async_l6": {\n      "enabled": true,\n      "release_tick_gap": 5,\n      "max_plan_age_ns": 350000000\n    }\n', '    "async_l6": {\n      "enabled": true,\n      "release_tick_gap": 5,\n      "release_delay_ns": 100000000,\n      "max_plan_age_ns": 350000000\n    }\n')], 'STRUKTURALIS_RETEGEK_V3.md': [('A composition root egyetlen `TickEngine`-t futtat. A motor-döntést befolyásoló **authority és owned layer-state** nem költözhet rétegenkénti threadbe/processzbe, és layer-kódban továbbra sincs sleep, falióra, rejtett I/O vagy modulglobális mutable state. Drága, determinisztikus **pure computation** külön worker-processzbe tehető kizárólag a runtime/adapter szélen, composition-root által injektált typed compute-port mögött. A worker csak lezárt immutable snapshotból számolhat; nem birtokolhat command-, mission-, navigation-, lifecycle-, safety-, motor- vagy GPIO-authorityt. Az owning layer az eredményt csak előre meghatározott tick-határon fogadhatja el; worker-hiány, deadline-miss, context-eltérés vagy túl öreg elfogadott terv fail-closed hiba. A checkpointnak az esetleges pending immutable kérést és determinisztikus release tickjét is rögzítenie kell, hogy replay ugyanazt a pure számítást ugyanazon a handoff ticken tegye láthatóvá.\n', 'A composition root egyetlen `TickEngine`-t futtat. A motor-döntést befolyásoló **authority és owned layer-state** nem költözhet rétegenkénti threadbe/processzbe, és layer-kódban továbbra sincs sleep, falióra, rejtett I/O vagy modulglobális mutable state. Drága, determinisztikus **pure computation** külön worker-processzbe tehető kizárólag a runtime/adapter szélen, composition-root által injektált typed compute-port mögött. A worker csak lezárt immutable snapshotból számolhat; nem birtokolhat command-, mission-, navigation-, lifecycle-, safety-, motor- vagy GPIO-authorityt. Az owning layer az eredményt csak előre meghatározott, kizárólag a lezárt `TickContext` (`tick_id`, `monotonic_ns`) alapján determinisztikusan eldönthető tick-határon fogadhatja el; worker-hiány, deadline-miss, context-eltérés vagy túl öreg elfogadott terv fail-closed hiba. A checkpointnak az esetleges pending immutable kérést és a determinisztikus handoff-kritériumot is rögzítenie kell (új production módban például `not-before monotonic_ns`, legacy capture esetén release tick), hogy replay ugyanazon a determinisztikus handoff-határon tegye láthatóvá ugyanazt a pure számítást.\n')], 'tests/test_v3_async_l6_planner.py': [('    enabled = replace(\n        old.async_l6,\n        enabled=True,\n        release_tick_gap=5,\n        max_plan_age_ns=350_000_000,\n    )\n    base["v3_navigation"]["async_l6"] = {\n        "enabled": enabled.enabled,\n        "release_tick_gap": enabled.release_tick_gap,\n        "max_plan_age_ns": enabled.max_plan_age_ns,\n    }\n', '    enabled = replace(\n        old.async_l6,\n        enabled=True,\n        release_tick_gap=5,\n        max_plan_age_ns=350_000_000,\n        release_delay_ns=100_000_000,\n    )\n    base["v3_navigation"]["async_l6"] = {\n        "enabled": enabled.enabled,\n        "release_tick_gap": enabled.release_tick_gap,\n        "max_plan_age_ns": enabled.max_plan_age_ns,\n        "release_delay_ns": enabled.release_delay_ns,\n    }\n'), ('    assert checkpoint.pending_rollout_request is not None\n    assert checkpoint.pending_release_tick_id == 10\n    assert plans[5].trajectory_candidates == plans[4].trajectory_candidates\n', '    assert checkpoint.pending_rollout_request is not None\n    assert checkpoint.pending_release_tick_id == 10\n    assert checkpoint.pending_release_not_before_ns is None\n    assert plans[5].trajectory_candidates == plans[4].trajectory_candidates\n'), ('    assert next_checkpoint.pending_rollout_request.context.tick_id == 10\n    assert next_checkpoint.pending_release_tick_id == 15\n\n    original_backend.close()\n', '    assert next_checkpoint.pending_rollout_request.context.tick_id == 10\n    assert next_checkpoint.pending_release_tick_id == 15\n    assert next_checkpoint.pending_release_not_before_ns is None\n\n    original_backend.close()\n')]}
TEST_APPEND = '\n\ndef _evaluate_at(navigator, manager, tick_id: int, monotonic_ns: int):\n    context = TickContext(tick_id, monotonic_ns)\n    return navigator.evaluate(\n        _mission(manager, context),\n        _estimate(context, x_m=0.02 * tick_id),\n        _world(context),\n    )\n\n\ndef test_time_based_async_handoff_is_stable_under_control_tick_jitter():\n    """Reproduce the live failure shape without tying 100 ms to five ticks."""\n\n    config = NavigationConfig(\n        trajectory_replan_interval_ns=100_000_000,\n        trajectory_replan_min_tick_gap=5,\n    )\n    backend = InlineTrajectoryRolloutBackend(config)\n    navigator = TrajectoryNavigator(\n        config,\n        rollout_backend=backend,\n        rollout_release_tick_gap=5,\n        rollout_release_delay_ns=100_000_000,\n        max_plan_age_ns=350_000_000,\n    )\n    manager = MissionManager()\n\n    times = {\n        0: 1_000_000_000,\n        1: 1_020_000_000,\n        2: 1_040_000_000,\n        3: 1_060_000_000,\n        4: 1_080_000_000,\n        5: 1_100_000_000,\n        6: 1_130_000_000,\n        7: 1_165_000_000,\n        8: 1_195_000_000,\n        # Four ticks after request, but already 180 ms later.\n        9: 1_280_000_000,\n    }\n\n    for tick_id in range(6):\n        _evaluate_at(navigator, manager, tick_id, times[tick_id])\n\n    pending = navigator.checkpoint()\n    assert pending.last_replan_tick_id == 0\n    assert pending.pending_rollout_request is not None\n    assert pending.pending_rollout_request.context.tick_id == 5\n    assert pending.pending_release_tick_id is None\n    assert pending.pending_release_not_before_ns == 1_200_000_000\n\n    for tick_id in (6, 7, 8):\n        _evaluate_at(navigator, manager, tick_id, times[tick_id])\n        assert navigator.checkpoint().last_replan_tick_id == 0\n\n    # Tick 9 is the first closed tick after the 100 ms handoff threshold.\n    # The old five-tick scheme would still wait for tick 10.\n    _evaluate_at(navigator, manager, 9, times[9])\n    after_handoff = navigator.checkpoint()\n    assert after_handoff.last_replan_tick_id == 5\n\n    # Because the accepted source snapshot is already >100 ms old, the next\n    # async request is scheduled immediately. The historic min-tick-gap must\n    # not turn scheduler jitter into another planner-age failure.\n    assert after_handoff.pending_rollout_request is not None\n    assert after_handoff.pending_rollout_request.context.tick_id == 9\n    assert after_handoff.pending_release_tick_id is None\n    assert after_handoff.pending_release_not_before_ns == 1_380_000_000\n\n    backend.close()\n\n\ndef test_time_based_async_handoff_budget_must_fit_inside_plan_freshness():\n    with pytest.raises(\n        ValueError,\n        match="release_delay_ns must be shorter than max_plan_age_ns",\n    ):\n        AsyncL6PlannerConfig(\n            enabled=True,\n            release_tick_gap=5,\n            max_plan_age_ns=100_000_000,\n            release_delay_ns=100_000_000,\n        )\n'
TEST_MARKER = "test_time_based_async_handoff_is_stable_under_control_tick_jitter"


def run(repo: Path, *args: str) -> str:
    return subprocess.check_output(
        list(args), cwd=repo, text=True, stderr=subprocess.STDOUT
    ).strip()


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def runtime_pid(repo: Path) -> int | None:
    path = repo / "runtime/.r2b4_runtime_pid"
    try:
        raw = path.read_text(encoding="utf-8").strip()
    except FileNotFoundError:
        return None
    try:
        pid = int(raw)
    except ValueError:
        return None
    if pid <= 0:
        return None
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return None
    except PermissionError:
        return pid
    return pid


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("repo", nargs="?", default="/home/alba/project_r2b4")
    args = parser.parse_args()
    repo = Path(args.repo).resolve()
    package = Path(__file__).resolve().parent

    if not (repo / ".git").exists():
        raise SystemExit(f"ERROR: not a git repo: {repo}")
    pid = runtime_pid(repo)
    if pid is not None:
        raise SystemExit(
            f"ERROR: resident runtime appears active (pid {pid}). "
            "Run ./r2b4 shutdown first."
        )

    head = run(repo, "git", "rev-parse", "HEAD")
    try:
        run(repo, "git", "merge-base", "--is-ancestor", BASE_COMMIT, head)
    except subprocess.CalledProcessError:
        raise SystemExit(
            "ERROR: repo HEAD is not based on the inspected source "
            f"{BASE_COMMIT} (HEAD={head})."
        )

    targets = tuple(PATCHES)
    dirty = run(repo, "git", "status", "--porcelain", "--", *targets)
    if dirty:
        raise SystemExit(
            "ERROR: target files have uncommitted changes. "
            "Commit/stash them first:\n" + dirty
        )

    current: dict[str, str] = {}
    updated: dict[str, str] = {}
    for rel, replacements in PATCHES.items():
        path = repo / rel
        text = path.read_text(encoding="utf-8")
        current[rel] = text
        changed = text
        for old, new in replacements:
            count = changed.count(old)
            if count != 1:
                raise SystemExit(
                    f"ERROR: source anchor for {rel} matched {count} times; "
                    "repo has drifted. No files changed."
                )
            changed = changed.replace(old, new, 1)
        if rel == "tests/test_v3_async_l6_planner.py":
            if TEST_MARKER in changed:
                raise SystemExit(
                    "ERROR: new timebase regression tests already exist while "
                    "source anchors are still old; partial state detected."
                )
            changed = changed.rstrip() + TEST_APPEND + "\n"
        updated[rel] = changed

    backup = package / "backup"
    if backup.exists():
        shutil.rmtree(backup)
    backup.mkdir(parents=True)

    manifest = {
        "schema": "R2B4_ASYNC_L6_TIMEBASE_FIX_BACKUP_V1",
        "base_commit": BASE_COMMIT,
        "install_head": head,
        "files": {},
    }
    written: list[str] = []
    try:
        for rel in targets:
            src = repo / rel
            dst = backup / rel
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)
            before = current[rel].encode()
            after = updated[rel].encode()
            manifest["files"][rel] = {
                "before_sha256": sha256_bytes(before),
                "after_sha256": sha256_bytes(after),
            }

        (backup / "manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

        for rel in targets:
            (repo / rel).write_text(updated[rel], encoding="utf-8")
            written.append(rel)

        subprocess.check_call(
            [
                sys.executable, "-m", "py_compile",
                str(repo / "v3/layers/l6_navigation.py"),
                str(repo / "v3/composition/native_control.py"),
                str(repo / "tests/test_v3_async_l6_planner.py"),
            ],
            cwd=repo,
        )
    except Exception:
        for rel in written:
            backup_file = backup / rel
            if backup_file.exists():
                shutil.copy2(backup_file, repo / rel)
        raise

    print("INSTALLED: async-L6 monotonic handoff P0 fix")
    print(f"repo: {repo}")
    print("changed:")
    for rel in targets:
        print(f"  {rel}")
    print("next:")
    print(f"  bash {package / 'validate_upgrade.sh'} {repo}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
