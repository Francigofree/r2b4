#!/usr/bin/env python3
"""Install the R2B4 L6/L7 progress-viability navigation upgrade.

Designed for the 2026-09-23 R2B4 V3 source line. The installer intentionally
performs only the checks required to avoid a partial/corrupt install:
- target files must exist;
- source patch anchors must be unambiguous;
- modified Python must compile;
- modified JSON must parse.

No full pytest/gate/replay/live run is executed unless --verify is requested.
All writes are atomic and any failure restores every touched file.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
from datetime import datetime
from pathlib import Path

UPGRADE_ID = "r2b4_progress_viability_20260923"
BASELINE_SHA = "14aa025eb1cbe97f39b47ddd8e92f4c8513b229e"

TARGETS = (
    Path("v3/contracts/messages.py"),
    Path("v3/layers/l6_navigation.py"),
    Path("v3/layers/l7_motion_selection.py"),
    Path("v3/composition/native_control.py"),
    Path("conf/vezerles.json"),
)
TEST_PATH = Path("tests/test_v3_progress_viability.py")

TEST_SOURCE = r'''from __future__ import annotations

from v3.contracts import (
    MissionConstraints,
    NavigationPlan,
    NavigationStatus,
    RobotEstimate,
    RollingLocalCostmap,
    TickContext,
    TrajectoryEvaluation,
    TrajectoryPose,
    Waypoint,
    WorldSnapshot,
)
from v3.contracts.planner import TrajectoryRolloutRequest
from v3.layers.l6_navigation import NavigationConfig, TrajectoryRolloutComputer
from v3.layers.l7_motion_selection import select_motion


CONSTRAINTS = MissionConstraints(0.30, 0.60, 0.0, 0.08, 0.10)


def _candidate(
    candidate_id: str,
    *,
    v_mps: float,
    omega_rad_s: float,
    total_score: float,
    potential: float,
    viable: bool,
    collision: bool = False,
) -> TrajectoryEvaluation:
    horizon_ns = 100_000_000
    return TrajectoryEvaluation(
        candidate_id=candidate_id,
        v_mps=v_mps,
        omega_rad_s=omega_rad_s,
        horizon_ns=horizon_ns,
        samples=(TrajectoryPose(v_mps * 0.1, 0.0, omega_rad_s * 0.1, horizon_ns),),
        collision=collision,
        min_clearance_m=0.50,
        progress_score=potential,
        smoothness_score=0.50,
        novelty_score=0.50,
        total_score=total_score,
        progress_potential_score=potential,
        progress_viable=viable,
    )


def _plan(candidates: tuple[TrajectoryEvaluation, ...]) -> NavigationPlan:
    return NavigationPlan(
        context=TickContext(1, 1_000_000_000),
        mission_id="progress-viability",
        route=(),
        velocity_target=None,
        constraints=CONSTRAINTS,
        corridor_radius_m=0.0,
        progress=0.0,
        status=NavigationStatus.ACTIVE,
        local_goal=Waypoint(0.6, 0.0),
        trajectory_candidates=candidates,
    )


def _estimate(context: TickContext, yaw_rad: float = 0.0) -> RobotEstimate:
    covariance = tuple(0.01 if index % 6 == 0 else 0.0 for index in range(25))
    return RobotEstimate(
        context,
        "R2B4_BOOT_ROBOT_MAP",
        0.0,
        0.0,
        yaw_rad,
        0.0,
        0.0,
        covariance,
    )


def _world(context: TickContext) -> WorldSnapshot:
    return WorldSnapshot(
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


def _rollout(goal: Waypoint):
    context = TickContext(1, 1_000_000_000)
    config = NavigationConfig(
        rollout_horizon_ns=800_000_000,
        progress_viability_floor=0.02,
    )
    request = TrajectoryRolloutRequest(
        context=context,
        estimate=_estimate(context),
        world=_world(context),
        goal=goal,
        max_v_mps=0.30,
        max_omega_rad_s=0.60,
        coverage=(),
    )
    return TrajectoryRolloutComputer(config).compute(request).trajectory_candidates


def test_progress_viable_candidate_beats_higher_scoring_stationary_candidate():
    stationary = _candidate(
        "stationary", v_mps=0.0, omega_rad_s=0.0,
        total_score=0.95, potential=0.0, viable=False,
    )
    moving = _candidate(
        "moving", v_mps=0.20, omega_rad_s=0.0,
        total_score=0.40, potential=0.10, viable=True,
    )
    objective = select_motion(_plan((stationary, moving)))
    assert objective.trajectory is not None
    assert objective.trajectory.candidate_id == "moving"


def test_escape_family_remains_selectable_when_goal_progress_is_not_available():
    pivot = _candidate(
        "escape-00-08", v_mps=0.0, omega_rad_s=0.60,
        total_score=0.80, potential=0.0, viable=False,
    )
    reverse = _candidate(
        "escape-05-04", v_mps=-0.12, omega_rad_s=0.0,
        total_score=0.40, potential=-0.10, viable=False,
    )
    objective = select_motion(_plan((pivot, reverse)))
    assert objective.trajectory is not None
    assert objective.trajectory.candidate_id == "escape-00-08"


def test_straight_goal_keeps_54_candidates_but_stationary_is_not_viable():
    candidates = _rollout(Waypoint(0.60, 0.0))
    assert len(candidates) == 54
    assert all(candidate.candidate_id.startswith("trajectory-") for candidate in candidates)
    stationary = next(item for item in candidates if item.candidate_id == "trajectory-00-04")
    forward = next(item for item in candidates if item.candidate_id == "trajectory-05-04")
    assert stationary.progress_viable is False
    assert stationary.progress_potential_score == 0.0
    assert forward.progress_viable is True
    assert forward.progress_potential_score > 0.02


def test_goal_behind_accepts_productive_pivot_as_progress_viable():
    candidates = _rollout(Waypoint(-0.60, 0.0))
    assert len(candidates) == 54
    assert all(candidate.candidate_id.startswith("trajectory-") for candidate in candidates)
    pivots = tuple(
        item
        for item in candidates
        if abs(item.v_mps) <= 1e-12 and abs(item.omega_rad_s) > 1e-12
    )
    assert any(item.progress_viable for item in pivots)
    assert max(item.progress_potential_score for item in pivots) > 0.02


def test_no_progress_potential_switches_to_existing_escape_family():
    candidates = _rollout(Waypoint(0.0, 0.0))
    assert len(candidates) == 54
    assert all(candidate.candidate_id.startswith("escape-") for candidate in candidates)
'''


def _replace_once(text: str, old: str, new: str, label: str) -> tuple[str, bool]:
    if new in text:
        return text, False
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected one patch anchor, found {count}")
    return text.replace(old, new, 1), True


def _replace_all(text: str, old: str, new: str, label: str, expected: int) -> tuple[str, bool]:
    if new in text and old not in text:
        return text, False
    count = text.count(old)
    if count != expected:
        raise RuntimeError(f"{label}: expected {expected} patch anchors, found {count}")
    return text.replace(old, new), True


def _patch_messages(text: str) -> str:
    text, _ = _replace_once(
        text,
        '''    novelty_score: float\n    total_score: float\n''',
        '''    novelty_score: float\n    total_score: float\n    # L6 navigation viability is orthogonal to weighted quality. Defaults keep\n    # older captures/manual constructors replay-compatible.\n    progress_potential_score: float = 0.0\n    progress_viable: bool = True\n''',
        "TrajectoryEvaluation fields",
    )
    text, _ = _replace_once(
        text,
        '''            "min_clearance_m",\n            "progress_score",\n            "smoothness_score",\n''',
        '''            "min_clearance_m",\n            "progress_score",\n            "progress_potential_score",\n            "smoothness_score",\n''',
        "TrajectoryEvaluation finite progress potential",
    )
    text, _ = _replace_once(
        text,
        '''        if not -1.0 <= self.progress_score <= 1.0:\n            raise ContractValidationError("progress_score must be in [-1, 1]")\n        for name in ("smoothness_score", "novelty_score"):\n''',
        '''        if not -1.0 <= self.progress_score <= 1.0:\n            raise ContractValidationError("progress_score must be in [-1, 1]")\n        if not -1.0 <= self.progress_potential_score <= 1.0:\n            raise ContractValidationError(\n                "progress_potential_score must be in [-1, 1]"\n            )\n        if type(self.progress_viable) is not bool:\n            raise ContractValidationError("progress_viable must be bool")\n        for name in ("smoothness_score", "novelty_score"):\n''',
        "TrajectoryEvaluation validation",
    )
    return text


def _patch_l6(text: str) -> str:
    text, _ = _replace_once(
        text,
        '''    novelty_weight: float = 0.22\n    face_person_min_confidence: float = 0.60\n''',
        '''    novelty_weight: float = 0.22\n    # Minimum predicted normalized improvement required for a normal trajectory.\n    # Progress may be translational or angular alignment toward the local goal.\n    progress_viability_floor: float = 0.02\n    face_person_min_confidence: float = 0.60\n''',
        "NavigationConfig progress viability field",
    )
    text, _ = _replace_once(
        text,
        '''        if sum(weights) <= 0.0:\n            raise ValueError("at least one trajectory score weight must be positive")\n        if (\n            not isinstance(self.face_person_min_confidence, (int, float))\n''',
        '''        if sum(weights) <= 0.0:\n            raise ValueError("at least one trajectory score weight must be positive")\n        if (\n            not isinstance(self.progress_viability_floor, (int, float))\n            or isinstance(self.progress_viability_floor, bool)\n            or not math.isfinite(self.progress_viability_floor)\n            or not 0.0 <= self.progress_viability_floor <= 1.0\n        ):\n            raise ValueError("progress_viability_floor must be finite and in [0, 1]")\n        if (\n            not isinstance(self.face_person_min_confidence, (int, float))\n''',
        "NavigationConfig progress viability validation",
    )

    text, _ = _replace_once(
        text,
        '''        if any(\n            not candidate.collision and candidate.v_mps > _MOTION_EPSILON\n            for candidate in evaluations\n        ):\n            return tuple(evaluations)\n''',
        '''        # Normal navigation is useful only if at least one collision-free\n        # candidate can improve distance or heading toward the local goal. Keep the\n        # full 54-candidate family for diagnostics/L7 ranking once that invariant holds.\n        if any(\n            not candidate.collision and candidate.progress_viable\n            for candidate in evaluations\n        ):\n            return tuple(evaluations)\n''',
        "synchronous progress viability fallback",
    )
    text, _ = _replace_once(
        text,
        '''        if not any(\n            not candidate.collision and candidate.v_mps > _MOTION_EPSILON\n            for candidate in evaluations\n        ):\n''',
        '''        if not any(\n            not candidate.collision and candidate.progress_viable\n            for candidate in evaluations\n        ):\n''',
        "worker progress viability fallback",
    )

    progress_anchor = '''        progress = _clamp_signed(\n            (start_distance - final_distance) / max(start_distance, 1e-9)\n        )\n        linear_change = abs(v_mps - estimate.v_mps) / max(max_v_mps, 1e-9)\n'''
    sync_progress = '''        progress = _clamp_signed(\n            (start_distance - final_distance) / max(start_distance, 1e-9)\n        )\n        progress_potential = _trajectory_progress_potential(\n            estimate, goal, x_m, y_m, yaw_rad, progress\n        )\n        progress_viable = (\n            progress_potential + 1e-12 >= self._config.progress_viability_floor\n        )\n        linear_change = abs(v_mps - estimate.v_mps) / max(max_v_mps, 1e-9)\n'''
    async_progress = '''        progress = _clamp_signed(\n            (start_distance - final_distance) / max(start_distance, 1e-9)\n        )\n        progress_potential = _trajectory_progress_potential(\n            estimate, goal, x_m, y_m, yaw_rad, progress\n        )\n        progress_viable = (\n            progress_potential + 1e-12 >= config.progress_viability_floor\n        )\n        linear_change = abs(v_mps - estimate.v_mps) / max(max_v_mps, 1e-9)\n'''
    if sync_progress not in text:
        if text.count(progress_anchor) < 1:
            raise RuntimeError("synchronous trajectory progress calculation anchor missing")
        text = text.replace(progress_anchor, sync_progress, 1)
    if async_progress not in text:
        if text.count(progress_anchor) < 1:
            raise RuntimeError("worker trajectory progress calculation anchor missing")
        text = text.replace(progress_anchor, async_progress, 1)

    text, _ = _replace_all(
        text,
        '''            progress_score=progress,\n            smoothness_score=smoothness,\n''',
        '''            progress_score=progress,\n            progress_potential_score=progress_potential,\n            progress_viable=progress_viable,\n            smoothness_score=smoothness,\n''',
        "TrajectoryEvaluation progress viability output",
        2,
    )

    helper = r'''

def _goal_heading_error(
    x_m: float,
    y_m: float,
    yaw_rad: float,
    goal: Waypoint,
) -> float:
    """Absolute heading error toward the local goal from one predicted pose."""

    dx_m = goal.x_m - x_m
    dy_m = goal.y_m - y_m
    if math.hypot(dx_m, dy_m) <= 1e-9:
        return 0.0
    return abs(_wrapped_angle(math.atan2(dy_m, dx_m) - yaw_rad))


def _trajectory_progress_potential(
    estimate: RobotEstimate,
    goal: Waypoint,
    final_x_m: float,
    final_y_m: float,
    final_yaw_rad: float,
    distance_progress_score: float,
) -> float:
    """Predict generic local progress without conflating it with quality score.

    A trajectory is useful when it either gets closer to the local goal or turns
    the robot meaningfully toward it. This admits productive in-place pivots while
    rejecting stationary/nonproductive local optima.
    """

    start_heading_error = _goal_heading_error(
        estimate.x_m,
        estimate.y_m,
        estimate.yaw_rad,
        goal,
    )
    final_heading_error = _goal_heading_error(
        final_x_m,
        final_y_m,
        final_yaw_rad,
        goal,
    )
    heading_progress = _clamp_signed(
        (start_heading_error - final_heading_error) / math.pi
    )
    return max(distance_progress_score, heading_progress)
'''
    if "def _trajectory_progress_potential(" not in text:
        marker = "\n\ndef _as_escape_candidate(\n"
        if text.count(marker) != 1:
            raise RuntimeError("progress potential helper insertion anchor missing")
        text = text.replace(marker, helper + marker, 1)
    return text


def _patch_l7(text: str) -> str:
    old = '''def _viable_trajectories(\n    plan: NavigationPlan,\n) -> tuple[TrajectoryEvaluation, ...]:\n    return tuple(\n        candidate\n        for candidate in plan.trajectory_candidates\n        if not candidate.collision\n    )\n'''
    new = '''def _viable_trajectories(\n    plan: NavigationPlan,\n) -> tuple[TrajectoryEvaluation, ...]:\n    collision_free = tuple(\n        candidate\n        for candidate in plan.trajectory_candidates\n        if not candidate.collision\n    )\n    if not collision_free:\n        return ()\n\n    # Escape is L6's explicit recovery family: by definition it is used when\n    # local-goal progress is not currently achievable, so it is ranked by the\n    # existing escape score instead of normal progress viability.\n    if all(candidate.candidate_id.startswith("escape-") for candidate in collision_free):\n        return collision_free\n\n    # L6 owns navigation viability; L7 only chooses one objective among viable\n    # normal trajectories. This prevents smooth/novel stationary local optima\n    # from beating a trajectory that can actually improve the navigation state.\n    return tuple(candidate for candidate in collision_free if candidate.progress_viable)\n'''
    text, _ = _replace_once(text, old, new, "L7 progress viability filter")
    text, _ = _replace_once(
        text,
        '''        key=lambda candidate: (\n            -candidate.total_score,\n            -candidate.min_clearance_m,\n''',
        '''        key=lambda candidate: (\n            -candidate.total_score,\n            -candidate.progress_potential_score,\n            -candidate.min_clearance_m,\n''',
        "L7 progress potential tie-break",
    )
    return text


def _patch_native_control(text: str) -> str:
    text, _ = _replace_once(
        text,
        '''        novelty_weight=_finite_float(\n            rollout.get("novelty_weight"),\n            "v3_navigation.trajectory_rollout.novelty_weight",\n        ),\n    )\n''',
        '''        novelty_weight=_finite_float(\n            rollout.get("novelty_weight"),\n            "v3_navigation.trajectory_rollout.novelty_weight",\n        ),\n        progress_viability_floor=_finite_float(\n            rollout.get("progress_viability_floor", 0.02),\n            "v3_navigation.trajectory_rollout.progress_viability_floor",\n        ),\n    )\n''',
        "native control progress viability config",
    )
    return text


def _patch_json(text: str) -> str:
    data = json.loads(text)
    try:
        rollout = data["v3_navigation"]["trajectory_rollout"]
    except (KeyError, TypeError) as exc:
        raise RuntimeError("conf/vezerles.json has no v3_navigation.trajectory_rollout") from exc
    if not isinstance(rollout, dict):
        raise RuntimeError("v3_navigation.trajectory_rollout must be an object")
    rollout["progress_viability_floor"] = 0.02
    return json.dumps(data, ensure_ascii=False, indent=2) + "\n"


def _atomic_write(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="") as handle:
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp_name, path)
    except Exception:
        try:
            os.unlink(tmp_name)
        except FileNotFoundError:
            pass
        raise


def _find_root(explicit: str | None) -> Path:
    candidates: list[Path] = []
    if explicit:
        candidates.append(Path(explicit).expanduser())
    candidates.extend((Path.cwd(), Path("/home/alba/project_r2b4")))
    for candidate in candidates:
        root = candidate.resolve()
        if (root / "v3/layers/l6_navigation.py").is_file() and (root / "conf/vezerles.json").is_file():
            return root
    raise RuntimeError("R2B4 project root not found; use --root /home/alba/project_r2b4")


def _compile_python(path: Path, source: str) -> None:
    compile(source, str(path), "exec")


def _optional_git_sha(root: Path) -> str | None:
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=root, check=False,
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
        )
    except OSError:
        return None
    sha = result.stdout.strip()
    return sha or None


def _verify(root: Path) -> None:
    command = [
        sys.executable,
        "-m",
        "pytest",
        "-q",
        "tests/test_v3_progress_viability.py",
        "tests/test_v3_l7_motion_continuity.py",
        "tests/test_v3_async_l6_planner.py::test_pure_rollout_computer_matches_the_canonical_synchronous_l6_kernel",
    ]
    print("VERIFY:", " ".join(command))
    subprocess.run(command, cwd=root, check=True)


def install(root: Path, verify: bool) -> None:
    for relative in TARGETS:
        if not (root / relative).is_file():
            raise RuntimeError(f"missing target file: {relative}")

    before: dict[Path, str | None] = {}
    for relative in (*TARGETS, TEST_PATH):
        path = root / relative
        before[relative] = path.read_text(encoding="utf-8") if path.exists() else None

    patched: dict[Path, str] = {
        Path("v3/contracts/messages.py"): _patch_messages(before[Path("v3/contracts/messages.py")] or ""),
        Path("v3/layers/l6_navigation.py"): _patch_l6(before[Path("v3/layers/l6_navigation.py")] or ""),
        Path("v3/layers/l7_motion_selection.py"): _patch_l7(before[Path("v3/layers/l7_motion_selection.py")] or ""),
        Path("v3/composition/native_control.py"): _patch_native_control(before[Path("v3/composition/native_control.py")] or ""),
        Path("conf/vezerles.json"): _patch_json(before[Path("conf/vezerles.json")] or ""),
        TEST_PATH: TEST_SOURCE,
    }

    for relative, source in patched.items():
        if relative.suffix == ".py":
            _compile_python(root / relative, source)
    json.loads(patched[Path("conf/vezerles.json")])

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_dir = root / "runtime" / "upgrade_backups" / f"{UPGRADE_ID}_{stamp}"
    backup_dir.mkdir(parents=True, exist_ok=False)
    manifest: dict[str, object] = {
        "upgrade_id": UPGRADE_ID,
        "baseline_sha": BASELINE_SHA,
        "observed_git_sha": _optional_git_sha(root),
        "created_at": stamp,
        "files": [],
    }
    for relative, original in before.items():
        if original is None:
            continue
        backup_path = backup_dir / relative
        backup_path.parent.mkdir(parents=True, exist_ok=True)
        backup_path.write_text(original, encoding="utf-8")
        manifest["files"].append(str(relative))
    (backup_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )

    try:
        for relative, source in patched.items():
            _atomic_write(root / relative, source)
        if verify:
            _verify(root)
    except Exception:
        for relative, original in before.items():
            path = root / relative
            if original is None:
                try:
                    path.unlink()
                except FileNotFoundError:
                    pass
            else:
                _atomic_write(path, original)
        raise

    observed = manifest["observed_git_sha"]
    print(f"INSTALLED: {UPGRADE_ID}")
    print(f"ROOT: {root}")
    print(f"BACKUP: {backup_dir}")
    if observed:
        print(f"GIT_HEAD: {observed}")
        if observed != BASELINE_SHA:
            print(f"NOTE: package was designed from baseline {BASELINE_SHA}; anchors matched current source.")
    print("DEFAULT_VALIDATION: source anchors + Python compile + JSON parse")
    print("TARGETED_PYTEST:", "PASS" if verify else "not requested (use --verify)")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", help="R2B4 project root; auto-detected when omitted")
    parser.add_argument(
        "--verify",
        action="store_true",
        help="after install, run only the focused progress/L7/async equivalence tests",
    )
    args = parser.parse_args()
    root = _find_root(args.root)
    try:
        install(root, args.verify)
    except Exception as exc:
        print(f"INSTALL FAILED: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
