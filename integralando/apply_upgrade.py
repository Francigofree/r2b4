#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
from pathlib import Path

BASE_COMMIT = "f065405f5092ff3ee2b3c23ce768ee05371a119b"
EXPECTED_BLOBS = {
    "v3/contracts/messages.py": "7ea79d9bfa75fa2d4c3a1c15e7c3b8635c3b1c6b",
    "v3/contracts/__init__.py": "fd36d490ad441930baa8512d08970bb35b918cd5",
    "v3/layers/l4_world_model.py": "74127000adff984c6a6548a606fe78a9aa0342da",
    "v3/layers/l6_navigation.py": "d6b4a78fcdc6a361b309138078ff392bf4b27a76",
    "tests/feature/test_v3_lidar_world_model.py": "68d5d0f58b0f514eceb4f684d810d1ef537c8b83",
    "tests/feature/test_v3_roomcruise_localization_motion.py": "1b7b2cbd3d65c0ec5872dbe715ead5f92ae73815",
}


def run(repo: Path, *args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        args,
        cwd=repo,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=check,
    )


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected one replacement site, found {count}")
    return text.replace(old, new, 1)


def verify_base(repo: Path) -> None:
    head = run(repo, "git", "rev-parse", "HEAD").stdout.strip()
    if head != BASE_COMMIT:
        raise RuntimeError(
            f"upgrade is pinned to {BASE_COMMIT}, but repository HEAD is {head}. "
            "Refresh/rebase the upgrade instead of applying it to a different source state."
        )
    for rel, expected in EXPECTED_BLOBS.items():
        actual = run(repo, "git", "hash-object", rel).stdout.strip()
        if actual != expected:
            raise RuntimeError(
                f"{rel}: expected base blob {expected}, found {actual}; refusing partial/dirty application"
            )


def patch_messages(text: str) -> str:
    anchor = '''        object.__setattr__(result, "freshness_ns", freshness_ns)\n        return result\n\n\n@dataclass(frozen=True, slots=True)\nclass WorldSnapshot:\n'''
    replacement = '''        object.__setattr__(result, "freshness_ns", freshness_ns)\n        return result\n\n\n@dataclass(frozen=True, slots=True)\nclass RobotRelativeGeometry:\n    """Fresh pose-independent LiDAR geometry lineage in the robot base frame.\n\n    The raw point cloud stays at the L2/L4 perception edge.  This compact token\n    only proves that current ROBOT_BASE geometry exists; it carries no map,\n    planning or final-safety authority.\n    """\n\n    frame_id: str\n    source_sequence: int\n    captured_monotonic_ns: int\n    point_count: int\n    freshness_ns: int\n\n    def __post_init__(self) -> None:\n        if self.frame_id != "ROBOT_BASE":\n            raise ContractValidationError(\n                "RobotRelativeGeometry must use the ROBOT_BASE frame"\n            )\n        require_nonnegative(self.source_sequence, "RobotRelativeGeometry.source_sequence")\n        require_nonnegative(\n            self.captured_monotonic_ns,\n            "RobotRelativeGeometry.captured_monotonic_ns",\n        )\n        if (\n            not isinstance(self.point_count, int)\n            or isinstance(self.point_count, bool)\n            or self.point_count < 0\n        ):\n            raise ContractValidationError(\n                "RobotRelativeGeometry.point_count must be a non-negative integer"\n            )\n        require_nonnegative(self.freshness_ns, "RobotRelativeGeometry.freshness_ns")\n\n    def with_freshness_ns(self, freshness_ns: int) -> RobotRelativeGeometry:\n        """Refresh age while reusing immutable scan lineage."""\n        require_nonnegative(freshness_ns, "RobotRelativeGeometry.freshness_ns")\n        if freshness_ns == self.freshness_ns:\n            return self\n        result = object.__new__(RobotRelativeGeometry)\n        for name in (\n            "frame_id",\n            "source_sequence",\n            "captured_monotonic_ns",\n            "point_count",\n        ):\n            object.__setattr__(result, name, getattr(self, name))\n        object.__setattr__(result, "freshness_ns", freshness_ns)\n        return result\n\n\n@dataclass(frozen=True, slots=True)\nclass WorldSnapshot:\n'''
    text = replace_once(text, anchor, replacement, "messages: RobotRelativeGeometry insertion")

    old = '''    local_costmap: RollingLocalCostmap | None = None\n    global_visited_cells: tuple[tuple[int, int, int], ...] = ()\n\n    def __post_init__(self) -> None:\n'''
    new = '''    local_costmap: RollingLocalCostmap | None = None\n    global_visited_cells: tuple[tuple[int, int, int], ...] = ()\n    robot_relative_geometry: RobotRelativeGeometry | None = None\n\n    def __post_init__(self) -> None:\n'''
    text = replace_once(text, old, new, "messages: WorldSnapshot field")

    old = '''            if self.local_costmap.frame_id != self.frame_id:\n                raise ContractValidationError(\n                    "WorldSnapshot and local costmap must use the same frame"\n                )\n\n\n@dataclass(frozen=True, slots=True)\nclass CommandRequest:\n'''
    new = '''            if self.local_costmap.frame_id != self.frame_id:\n                raise ContractValidationError(\n                    "WorldSnapshot and local costmap must use the same frame"\n                )\n        if (\n            self.robot_relative_geometry is not None\n            and not isinstance(self.robot_relative_geometry, RobotRelativeGeometry)\n        ):\n            raise ContractValidationError(\n                "WorldSnapshot.robot_relative_geometry must be RobotRelativeGeometry or None"\n            )\n\n\n@dataclass(frozen=True, slots=True)\nclass CommandRequest:\n'''
    return replace_once(text, old, new, "messages: WorldSnapshot validation")


def patch_contract_exports(text: str) -> str:
    text = replace_once(
        text,
        '''    RobotEstimate,\n    RollingLocalCostmap,\n    SafetyDecision,\n''',
        '''    RobotEstimate,\n    RobotRelativeGeometry,\n    RollingLocalCostmap,\n    SafetyDecision,\n''',
        "contracts: import",
    )
    return replace_once(
        text,
        '''    "RobotEstimate",\n    "RollingLocalCostmap",\n    "SafetyDecision",\n''',
        '''    "RobotEstimate",\n    "RobotRelativeGeometry",\n    "RollingLocalCostmap",\n    "SafetyDecision",\n''',
        "contracts: __all__",
    )


def patch_l4(text: str) -> str:
    text = replace_once(
        text,
        '''    RobotEstimate,\n    QualityState,\n    RollingLocalCostmap,\n''',
        '''    RobotEstimate,\n    RobotRelativeGeometry,\n    QualityState,\n    RollingLocalCostmap,\n''',
        "l4: import",
    )
    text = replace_once(
        text,
        '''    global_visited_cells: tuple[tuple[int, int, int], ...] = ()\n    last_global_visit: tuple[int, int] | None = None\n''',
        '''    global_visited_cells: tuple[tuple[int, int, int], ...] = ()\n    last_global_visit: tuple[int, int] | None = None\n    robot_relative_geometry: RobotRelativeGeometry | None = None\n    last_robot_relative_values: tuple[DataField, ...] | None = None\n''',
        "l4: checkpoint fields",
    )
    text = replace_once(
        text,
        '''        "_last_local_measurement_ns",\n        "_last_local_sequence",\n        "_last_local_values",\n        "_map_revision",\n''',
        '''        "_last_local_measurement_ns",\n        "_last_local_sequence",\n        "_last_local_values",\n        "_robot_relative_geometry",\n        "_last_robot_relative_values",\n        "_map_revision",\n''',
        "l4: slots",
    )
    text = replace_once(
        text,
        '''        self._last_local_measurement_ns: int | None = None\n        self._last_local_sequence: int | None = None\n        self._last_local_values: tuple[DataField, ...] | None = None\n        self._costmap_revision = 0\n''',
        '''        self._last_local_measurement_ns: int | None = None\n        self._last_local_sequence: int | None = None\n        self._last_local_values: tuple[DataField, ...] | None = None\n        self._robot_relative_geometry: RobotRelativeGeometry | None = None\n        self._last_robot_relative_values: tuple[DataField, ...] | None = None\n        self._costmap_revision = 0\n''',
        "l4: init",
    )
    text = replace_once(
        text,
        '''            self._structural_active,\n            self._global_visited_cells, self._last_global_visit,\n        )\n''',
        '''            self._structural_active,\n            self._global_visited_cells, self._last_global_visit,\n            self._robot_relative_geometry, self._last_robot_relative_values,\n        )\n''',
        "l4: checkpoint write",
    )
    text = replace_once(
        text,
        '''        self._global_visited_cells = checkpoint.global_visited_cells\n        self._last_global_visit = checkpoint.last_global_visit\n\n    def __call__(self, frame: AdmittedFrame, estimate: RobotEstimate) -> WorldSnapshot:\n''',
        '''        self._global_visited_cells = checkpoint.global_visited_cells\n        self._last_global_visit = checkpoint.last_global_visit\n        self._robot_relative_geometry = checkpoint.robot_relative_geometry\n        self._last_robot_relative_values = checkpoint.last_robot_relative_values\n\n    def __call__(self, frame: AdmittedFrame, estimate: RobotEstimate) -> WorldSnapshot:\n''',
        "l4: checkpoint restore",
    )
    text = replace_once(
        text,
        '''        self._update_lidar_health(frame)\n\n        changed_tracks = False\n''',
        '''        self._update_lidar_health(frame)\n        # Pose-independent scan lineage must continue through localization loss.\n        # Only the pose-aligned occupancy path below is allowed to freeze.\n        self._update_robot_relative_geometry(frame)\n\n        changed_tracks = False\n''',
        "l4: transient update call",
    )
    text = replace_once(
        text,
        '''            local_costmap = self._cached_costmap.with_freshness_ns(\n                max(0, now_ns - self._last_local_measurement_ns)\n            )\n        return WorldSnapshot(\n''',
        '''            local_costmap = self._cached_costmap.with_freshness_ns(\n                max(0, now_ns - self._last_local_measurement_ns)\n            )\n        robot_relative_geometry = self._robot_relative_geometry\n        if robot_relative_geometry is not None:\n            robot_relative_geometry = robot_relative_geometry.with_freshness_ns(\n                max(0, now_ns - robot_relative_geometry.captured_monotonic_ns)\n            )\n        return WorldSnapshot(\n''',
        "l4: transient freshness",
    )
    text = replace_once(
        text,
        '''            local_costmap=local_costmap,\n            global_visited_cells=self._global_visited_cells,\n        )\n''',
        '''            local_costmap=local_costmap,\n            global_visited_cells=self._global_visited_cells,\n            robot_relative_geometry=robot_relative_geometry,\n        )\n''',
        "l4: WorldSnapshot output",
    )

    anchor = '''    def _update_local_scan(self, frame: AdmittedFrame, estimate: RobotEstimate) -> bool:\n'''
    method = '''    def _update_robot_relative_geometry(self, frame: AdmittedFrame) -> None:\n        """Track fresh ROBOT_BASE geometry independently of localization quality.\n\n        This intentionally does not integrate, transform or persist obstacle\n        geometry.  It only carries scan lineage/freshness so recovery can keep\n        observing while pose-aligned mapping remains fail-closed.\n        """\n        local = tuple(item for item in frame.accepted if item.kind == "lidar_local_points")\n        if len(local) > 1:\n            raise ValueError("L4 accepts at most one lidar_local_points observation per tick")\n        if not local:\n            return\n        observation = local[0]\n        values = _values(observation)\n        if values.get("frame_id") != "ROBOT_BASE":\n            raise ValueError("lidar local points must use the ROBOT_BASE frame")\n        point_count = _integer(values, "point_count")\n        if point_count > self._config.local_costmap_max_points_per_scan:\n            raise ValueError("lidar local point_count exceeds the configured bound")\n        expected_keys = {"frame_id", "point_count"}\n        expected_keys.update(\n            f"point_{index:03d}_{suffix}"\n            for index in range(point_count)\n            for suffix in ("x_m", "y_m", "quality")\n        )\n        if set(values) != expected_keys:\n            raise ValueError("lidar local point fields do not match point_count")\n\n        previous = self._robot_relative_geometry\n        if previous is not None:\n            if observation.source_sequence < previous.source_sequence:\n                raise ValueError("L4 robot-relative perception sequence must not move backwards")\n            if observation.captured_monotonic_ns < previous.captured_monotonic_ns:\n                raise ValueError("L4 robot-relative perception time must not move backwards")\n            if observation.source_sequence == previous.source_sequence:\n                if (\n                    observation.captured_monotonic_ns != previous.captured_monotonic_ns\n                    or observation.values != self._last_robot_relative_values\n                ):\n                    raise ValueError("L4 robot-relative perception sequence was rewritten")\n                return\n\n        for index in range(point_count):\n            _number(values, f"point_{index:03d}_x_m")\n            _number(values, f"point_{index:03d}_y_m")\n            _integer(values, f"point_{index:03d}_quality")\n\n        self._robot_relative_geometry = RobotRelativeGeometry(\n            frame_id="ROBOT_BASE",\n            source_sequence=observation.source_sequence,\n            captured_monotonic_ns=observation.captured_monotonic_ns,\n            point_count=point_count,\n            freshness_ns=0,\n        )\n        self._last_robot_relative_values = observation.values\n\n'''
    return replace_once(text, anchor, method + anchor, "l4: transient method")


def patch_l6(text: str) -> str:
    old = '''        elapsed = now - self._localization_recovery_started_ns\n        costmap = world.local_costmap\n        if (mission.mode is CommandMode.TELEOP\n                or elapsed >= self._config.localization_recovery_timeout_ns\n                or quality.heading is not QualityState.GOOD\n                or quality.lidar_age_ns > self._config.max_costmap_freshness_ns\n                or estimate.frame_id != world.frame_id\n                or world.freshness_ns > self._config.max_world_freshness_ns\n                or costmap is None or costmap.freshness_ns > self._config.max_costmap_freshness_ns):\n            return self._inactive(mission, NavigationStatus.IDLE, "LOCALIZATION_HOLD")\n        # No translation; L12 independently verifies the raw side clearances.\n'''
    new = '''        elapsed = now - self._localization_recovery_started_ns\n        geometry = world.robot_relative_geometry\n        # Recovery must not depend on the pose-aligned costmap: L4 intentionally\n        # freezes that map when local translation is LOST.  Instead require a\n        # fresh ROBOT_BASE scan token.  Translation remains zero and L12 still\n        # independently owns directional raw-LiDAR collision safety.\n        if (mission.mode is CommandMode.TELEOP\n                or elapsed >= self._config.localization_recovery_timeout_ns\n                or quality.heading is not QualityState.GOOD\n                or quality.lidar_age_ns > self._config.max_costmap_freshness_ns\n                or estimate.frame_id != world.frame_id\n                or world.freshness_ns > self._config.max_world_freshness_ns\n                or geometry is None\n                or geometry.point_count <= 0\n                or geometry.freshness_ns > self._config.max_costmap_freshness_ns):\n            return self._inactive(mission, NavigationStatus.IDLE, "LOCALIZATION_HOLD")\n        # No translation; L12 independently verifies the raw side clearances.\n'''
    return replace_once(text, old, new, "l6: localization recovery dependency")


def patch_lidar_world_test(text: str) -> str:
    addition = r'''


def test_lidar_robot_relative_geometry_updates_while_pose_aligned_map_is_frozen_on_lost():
    config = _config()
    model = ShadowWorldModel(config)
    start = 1_000_000_000
    good = _tick(model, start, _scan(1, start, ((1.0, 0.2),)))
    assert good.robot_relative_geometry is not None
    assert good.robot_relative_geometry.source_sequence == 1
    assert good.robot_relative_geometry.point_count == 1
    assert good.robot_relative_geometry.freshness_ns == 0
    assert good.local_costmap is not None
    assert good.local_costmap.source_sequence == 1

    lost_quality = replace(
        healthy_localization(global_position=QualityState.LOST),
        local_translation=QualityState.LOST,
        heading=QualityState.GOOD,
        local_sigma_m=0.19,
    )
    now = start + 20_000_000
    lost = _tick(model, now, _scan(2, now, ((0.8, -0.3),)), quality=lost_quality)
    assert lost.robot_relative_geometry is not None
    assert lost.robot_relative_geometry.source_sequence == 2
    assert lost.robot_relative_geometry.freshness_ns == 0
    # The pose-aligned map deliberately did not consume scan 2.
    assert lost.local_costmap is not None
    assert lost.local_costmap.source_sequence == 1
    assert lost.local_costmap.freshness_ns == 20_000_000

    later = _tick(model, now + 20_000_000, quality=lost_quality)
    assert later.robot_relative_geometry is not None
    assert later.robot_relative_geometry.source_sequence == 2
    assert later.robot_relative_geometry.freshness_ns == 20_000_000
    assert later.local_costmap.source_sequence == 1
    assert later.local_costmap.freshness_ns == 40_000_000

    restored = ShadowWorldModel(config)
    restored.restore(pickle.loads(pickle.dumps(model.checkpoint())))
    assert _tick(restored, now + 20_000_000, quality=lost_quality) == later
'''
    if addition.strip() in text:
        raise RuntimeError("lidar world test already patched")
    return text.rstrip() + addition + "\n"


def patch_roomcruise_test(text: str) -> str:
    text = replace_once(
        text,
        '''    LocalizationRequirement, QualityState,\n    NavigationStatus, RobotEstimate, RollingLocalCostmap, TickContext, WorldSnapshot,\n''',
        '''    LocalizationRequirement, QualityState, RobotRelativeGeometry,\n    NavigationStatus, RobotEstimate, RollingLocalCostmap, TickContext, WorldSnapshot,\n''',
        "roomcruise test: import",
    )
    text = replace_once(
        text,
        '''    world = WorldSnapshot(context, "odom", 1, (), 0,\n                          RollingLocalCostmap("odom", 1, 0.1, 2.5, (), 1, 0))\n''',
        '''    world = WorldSnapshot(\n        context, "odom", 1, (), 0,\n        RollingLocalCostmap("odom", 1, 0.1, 2.5, (), 1, 0),\n        robot_relative_geometry=RobotRelativeGeometry(\n            "ROBOT_BASE", 1, context.monotonic_ns, 100, 0\n        ),\n    )\n''',
        "roomcruise test: scene geometry",
    )
    marker = '''\ndef test_roomcruise_localization_motion_keeps_freshness_heading_and_stop_gates():\n'''
    addition = r'''

def test_localization_recovery_uses_robot_relative_geometry_not_pose_aligned_costmap():
    config, manager, navigator, selector, realizer, limiter = _chain()
    estimate, world = _scene(0)
    estimate = replace(
        estimate,
        localization_quality=replace(
            estimate.localization_quality,
            local_translation=QualityState.LOST,
            heading=QualityState.GOOD,
            local_sigma_m=0.19,
            lidar_age_ns=0,
        ),
    )
    mission = manager.evaluate(
        CommandRequest(estimate.context, "recovery-geometry", CommandMode.EXPLORE, (), 0)
    )

    # P0: stale or absent pose-aligned costmap must not deadlock rotation-only recovery.
    for broken_world in (
        replace(world, local_costmap=None),
        replace(
            world,
            local_costmap=replace(
                world.local_costmap,
                freshness_ns=config.navigation.max_costmap_freshness_ns + 1,
            ),
        ),
    ):
        plan = navigator.evaluate(mission, estimate, broken_world)
        assert plan.status is NavigationStatus.ACTIVE
        assert plan.reason == "LOCALIZATION_REACQUIRE"
        assert plan.velocity_target.v_mps == 0.0
        assert plan.velocity_target.omega_rad_s != 0.0

    # P1: recovery now depends on fresh pose-independent ROBOT_BASE geometry.
    for geometry in (
        None,
        replace(
            world.robot_relative_geometry,
            freshness_ns=config.navigation.max_costmap_freshness_ns + 1,
        ),
        replace(world.robot_relative_geometry, point_count=0),
    ):
        _, case_manager, case_navigator, _, _, _ = _chain()
        case_mission = case_manager.evaluate(
            CommandRequest(estimate.context, "recovery-geometry", CommandMode.EXPLORE, (), 0)
        )
        held = case_navigator.evaluate(
            case_mission,
            estimate,
            replace(world, robot_relative_geometry=geometry),
        )
        assert held.status is NavigationStatus.IDLE
        assert held.reason == "LOCALIZATION_HOLD"

'''
    return replace_once(text, marker, addition + marker, "roomcruise test: recovery regression")


PATCHERS = {
    "v3/contracts/messages.py": patch_messages,
    "v3/contracts/__init__.py": patch_contract_exports,
    "v3/layers/l4_world_model.py": patch_l4,
    "v3/layers/l6_navigation.py": patch_l6,
    "tests/feature/test_v3_lidar_world_model.py": patch_lidar_world_test,
    "tests/feature/test_v3_roomcruise_localization_motion.py": patch_roomcruise_test,
}


def main() -> int:
    parser = argparse.ArgumentParser(description="Apply R2B4 localization recovery P0+P1 upgrade")
    parser.add_argument("repo", nargs="?", default=".", help="path to Francigofree/r2b4 checkout")
    parser.add_argument("--skip-tests", action="store_true")
    args = parser.parse_args()
    repo = Path(args.repo).resolve()
    if not (repo / ".git").exists():
        raise RuntimeError(f"not a git checkout: {repo}")

    verify_base(repo)
    for rel, patcher in PATCHERS.items():
        path = repo / rel
        original = path.read_text(encoding="utf-8")
        updated = patcher(original)
        if updated == original:
            raise RuntimeError(f"{rel}: patch produced no change")
        path.write_text(updated, encoding="utf-8")
        print(f"patched {rel}")

    print("\nUpgrade applied. Review with: git diff --check && git diff")
    check = run(repo, "git", "diff", "--check", check=False)
    if check.returncode:
        print(check.stdout, end="")
        print(check.stderr, end="")
        raise RuntimeError("git diff --check failed")

    if not args.skip_tests:
        tests = [
            "tests/feature/test_v3_lidar_world_model.py",
            "tests/feature/test_v3_roomcruise_localization_motion.py",
            "tests/feature/test_v3_dual_frame_localization.py",
            "tests/feature/test_v3_rate_only_heading_authority.py",
        ]
        command = ["python", "-m", "pytest", "-q", *tests]
        print("running:", " ".join(command))
        result = run(repo, *command, check=False)
        print(result.stdout, end="")
        print(result.stderr, end="")
        if result.returncode:
            raise RuntimeError(f"targeted pytest failed with exit code {result.returncode}")

    print("R2B4 localization recovery P0+P1 upgrade: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
