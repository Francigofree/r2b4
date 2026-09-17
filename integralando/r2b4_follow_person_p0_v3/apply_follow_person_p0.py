#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

L6_OLD_METHOD = '    def _follow_person_plan(\n        self,\n        mission: MissionIntent,\n        estimate: RobotEstimate,\n        world: WorldSnapshot,\n    ) -> NavigationPlan:\n        if self._mission_id != mission.mission_id:\n            self._reset()\n            self._mission_id = mission.mission_id\n\n        eligible = tuple(\n            track\n            for track in world.obstacle_tracks\n            if track.track_id.startswith("person-")\n            and track.confidence >= self._config.follow_person_min_confidence\n        )\n        previous_track_id = self._face_person_track_id\n        selected = next(\n            (\n                track\n                for track in eligible\n                if track.track_id == self._face_person_track_id\n            ),\n            None,\n        )\n        if selected is None and eligible:\n            selected = min(\n                eligible,\n                key=lambda track: (\n                    -track.confidence,\n                    math.hypot(track.x_m - estimate.x_m, track.y_m - estimate.y_m),\n                    track.track_id,\n                ),\n            )\n            if selected.track_id != previous_track_id:\n                self._clear_trajectory_plan()\n            self._face_person_track_id = selected.track_id\n            self._face_person_aligned = False\n\n        if selected is None:\n            self._face_person_track_id = None\n            self._face_person_aligned = False\n            self._clear_trajectory_plan()\n            return self._inactive(\n                mission,\n                NavigationStatus.INVALIDATED,\n                "PERSON_TARGET_NOT_AVAILABLE",\n            )\n\n        dx = selected.x_m - estimate.x_m\n        dy = selected.y_m - estimate.y_m\n        distance_m = math.hypot(dx, dy)\n        if distance_m <= 1e-9:\n            desired_yaw = estimate.yaw_rad\n            heading_error = 0.0\n        else:\n            desired_yaw = math.atan2(dy, dx)\n            heading_error = _wrapped_angle(desired_yaw - estimate.yaw_rad)\n\n        if distance_m <= self._config.follow_person_min_safe_distance_m:\n            self._face_person_aligned = False\n            self._clear_trajectory_plan()\n            return self._inactive(\n                mission,\n                NavigationStatus.INVALIDATED,\n                "PERSON_TOO_CLOSE",\n            )\n\n        absolute_error = abs(heading_error)\n        if self._face_person_aligned:\n            if absolute_error > self._config.follow_person_release_tolerance_rad:\n                self._face_person_aligned = False\n        elif absolute_error <= self._config.follow_person_align_tolerance_rad:\n            self._face_person_aligned = True\n\n        if not self._face_person_aligned:\n            self._clear_trajectory_plan()\n            return NavigationPlan(\n                context=mission.context,\n                mission_id=mission.mission_id,\n                route=(Waypoint(estimate.x_m, estimate.y_m, desired_yaw),),\n                velocity_target=None,\n                constraints=mission.constraints,\n                corridor_radius_m=0.0,\n                progress=0.0,\n                status=NavigationStatus.ACTIVE,\n            )\n\n        if (\n            distance_m\n            <= self._config.follow_person_stand_off_m\n            + self._config.follow_person_distance_deadband_m\n        ):\n            self._clear_trajectory_plan()\n            return NavigationPlan(\n                context=mission.context,\n                mission_id=mission.mission_id,\n                route=(Waypoint(estimate.x_m, estimate.y_m, estimate.yaw_rad),),\n                velocity_target=None,\n                constraints=mission.constraints,\n                corridor_radius_m=0.0,\n                progress=0.0,\n                status=NavigationStatus.ACTIVE,\n            )\n\n        costmap = world.local_costmap\n        if costmap is None:\n            self._clear_trajectory_plan()\n            return self._inactive(\n                mission,\n                NavigationStatus.INVALIDATED,\n                "LOCAL_COSTMAP_MISSING",\n            )\n        if costmap.freshness_ns > self._config.max_costmap_freshness_ns:\n            self._clear_trajectory_plan()\n            return self._inactive(\n                mission,\n                NavigationStatus.INVALIDATED,\n                "LOCAL_COSTMAP_STALE",\n            )\n\n        self._accept_pending_rollout(mission.context)\n        if self._replan_due(\n            mission.context.monotonic_ns,\n            mission.context.tick_id,\n        ):\n            travel_m = min(\n                distance_m - self._config.follow_person_stand_off_m,\n                self._config.local_goal_distance_m,\n            )\n            local_goal = Waypoint(\n                estimate.x_m + travel_m * math.cos(desired_yaw),\n                estimate.y_m + travel_m * math.sin(desired_yaw),\n            )\n            self._schedule_or_store_rollout(\n                mission.context,\n                estimate,\n                world,\n                local_goal,\n                mission.constraints.max_v_mps,\n                mission.constraints.max_omega_rad_s,\n            )\n\n        local_goal = self._local_goal\n        candidates = self._trajectory_candidates\n        if local_goal is None or not candidates:\n            raise RuntimeError("follow-person trajectory cache is empty after replanning")\n        return NavigationPlan(\n            context=mission.context,\n            mission_id=mission.mission_id,\n            route=(),\n            velocity_target=None,\n            constraints=mission.constraints,\n            corridor_radius_m=mission.constraints.corridor_radius_m,\n            progress=0.0,\n            status=NavigationStatus.ACTIVE,\n            local_goal=local_goal,\n            trajectory_candidates=candidates,\n        )\n\n'
L6_NEW_METHOD = '    def _follow_person_plan(\n        self,\n        mission: MissionIntent,\n        estimate: RobotEstimate,\n        world: WorldSnapshot,\n    ) -> NavigationPlan:\n        if self._mission_id != mission.mission_id:\n            self._reset()\n            self._mission_id = mission.mission_id\n\n        eligible = tuple(\n            track\n            for track in world.obstacle_tracks\n            if track.track_id.startswith("person-")\n            and track.confidence >= self._config.follow_person_min_confidence\n        )\n\n        # Acquire exactly once per FOLLOW_PERSON mission. Another visible person\n        # may not silently replace the locked target. A new command is re-acquire.\n        if self._follow_person_track_id is None:\n            if not eligible:\n                self._clear_trajectory_plan()\n                return self._inactive(\n                    mission,\n                    NavigationStatus.INVALIDATED,\n                    "PERSON_TARGET_NOT_AVAILABLE",\n                )\n            selected = min(\n                eligible,\n                key=lambda track: (\n                    -track.confidence,\n                    math.hypot(track.x_m - estimate.x_m, track.y_m - estimate.y_m),\n                    track.track_id,\n                ),\n            )\n            self._follow_person_track_id = selected.track_id\n            self._follow_person_lost_since_ns = None\n            self._follow_person_pivoting = False\n            self._follow_person_holding = False\n            self._clear_trajectory_plan()\n        else:\n            selected = next(\n                (\n                    track\n                    for track in eligible\n                    if track.track_id == self._follow_person_track_id\n                ),\n                None,\n            )\n            if selected is None:\n                if self._follow_person_lost_since_ns is None:\n                    self._follow_person_lost_since_ns = mission.context.monotonic_ns\n                    self._clear_trajectory_plan()\n                lost_ns = mission.context.monotonic_ns - self._follow_person_lost_since_ns\n                if lost_ns <= self._config.follow_person_lost_hold_ns:\n                    return NavigationPlan(\n                        context=mission.context,\n                        mission_id=mission.mission_id,\n                        route=(Waypoint(estimate.x_m, estimate.y_m, estimate.yaw_rad),),\n                        velocity_target=None,\n                        constraints=mission.constraints,\n                        corridor_radius_m=0.0,\n                        progress=0.0,\n                        status=NavigationStatus.ACTIVE,\n                    )\n                return self._inactive(\n                    mission,\n                    NavigationStatus.INVALIDATED,\n                    "PERSON_TARGET_LOST",\n                )\n\n        assert selected is not None\n        self._follow_person_lost_since_ns = None\n\n        dx = selected.x_m - estimate.x_m\n        dy = selected.y_m - estimate.y_m\n        distance_m = math.hypot(dx, dy)\n        if distance_m <= 1e-9:\n            desired_yaw = estimate.yaw_rad\n            heading_error = 0.0\n        else:\n            desired_yaw = math.atan2(dy, dx)\n            heading_error = _wrapped_angle(desired_yaw - estimate.yaw_rad)\n\n        if distance_m <= self._config.follow_person_min_safe_distance_m:\n            self._follow_person_pivoting = False\n            self._follow_person_holding = True\n            self._clear_trajectory_plan()\n            return self._inactive(\n                mission,\n                NavigationStatus.INVALIDATED,\n                "PERSON_TOO_CLOSE",\n            )\n\n        absolute_error = abs(heading_error)\n        if self._follow_person_pivoting:\n            if absolute_error <= self._config.follow_person_release_tolerance_rad:\n                self._follow_person_pivoting = False\n        elif absolute_error >= self._config.follow_person_pivot_enter_rad:\n            self._follow_person_pivoting = True\n\n        # Large errors still pivot in place. Medium errors stay in the rollout,\n        # so the robot bends toward the person instead of stop-turn-start.\n        if self._follow_person_pivoting:\n            self._clear_trajectory_plan()\n            return NavigationPlan(\n                context=mission.context,\n                mission_id=mission.mission_id,\n                route=(Waypoint(estimate.x_m, estimate.y_m, desired_yaw),),\n                velocity_target=None,\n                constraints=mission.constraints,\n                corridor_radius_m=0.0,\n                progress=0.0,\n                status=NavigationStatus.ACTIVE,\n            )\n\n        hold_enter_m = (\n            self._config.follow_person_stand_off_m\n            + self._config.follow_person_distance_deadband_m\n        )\n        hold_release_m = hold_enter_m + self._config.follow_person_distance_deadband_m\n        if self._follow_person_holding:\n            if distance_m >= hold_release_m:\n                self._follow_person_holding = False\n        elif distance_m <= hold_enter_m:\n            self._follow_person_holding = True\n\n        if self._follow_person_holding:\n            self._clear_trajectory_plan()\n            return NavigationPlan(\n                context=mission.context,\n                mission_id=mission.mission_id,\n                route=(Waypoint(estimate.x_m, estimate.y_m, estimate.yaw_rad),),\n                velocity_target=None,\n                constraints=mission.constraints,\n                corridor_radius_m=0.0,\n                progress=0.0,\n                status=NavigationStatus.ACTIVE,\n            )\n\n        costmap = world.local_costmap\n        if costmap is None:\n            self._clear_trajectory_plan()\n            return self._inactive(\n                mission,\n                NavigationStatus.INVALIDATED,\n                "LOCAL_COSTMAP_MISSING",\n            )\n        if costmap.freshness_ns > self._config.max_costmap_freshness_ns:\n            self._clear_trajectory_plan()\n            return self._inactive(\n                mission,\n                NavigationStatus.INVALIDATED,\n                "LOCAL_COSTMAP_STALE",\n            )\n\n        # Human-specific speed shaping: slow before HOLD and while bending.\n        distance_factor = min(\n            1.0,\n            max(\n                0.0,\n                (distance_m - hold_enter_m)\n                / self._config.follow_person_slowdown_distance_m,\n            ),\n        )\n        if absolute_error <= self._config.follow_person_align_tolerance_rad:\n            heading_factor = 1.0\n        else:\n            heading_span = max(\n                self._config.follow_person_pivot_enter_rad\n                - self._config.follow_person_align_tolerance_rad,\n                1e-9,\n            )\n            ratio = min(\n                1.0,\n                max(\n                    0.0,\n                    (absolute_error - self._config.follow_person_align_tolerance_rad)\n                    / heading_span,\n                ),\n            )\n            heading_factor = 1.0 - 0.65 * ratio\n        follow_max_v_mps = mission.constraints.max_v_mps * min(\n            distance_factor,\n            heading_factor,\n        )\n\n        self._accept_pending_rollout(mission.context)\n        if self._replan_due(\n            mission.context.monotonic_ns,\n            mission.context.tick_id,\n        ):\n            travel_m = min(\n                distance_m - self._config.follow_person_stand_off_m,\n                self._config.local_goal_distance_m,\n            )\n            local_goal = Waypoint(\n                estimate.x_m + travel_m * math.cos(desired_yaw),\n                estimate.y_m + travel_m * math.sin(desired_yaw),\n            )\n            self._schedule_or_store_rollout(\n                mission.context,\n                estimate,\n                world,\n                local_goal,\n                follow_max_v_mps,\n                mission.constraints.max_omega_rad_s,\n            )\n\n        local_goal = self._local_goal\n        candidates = self._trajectory_candidates\n        if local_goal is None or not candidates:\n            raise RuntimeError("follow-person trajectory cache is empty after replanning")\n        return NavigationPlan(\n            context=mission.context,\n            mission_id=mission.mission_id,\n            route=(),\n            velocity_target=None,\n            constraints=mission.constraints,\n            corridor_radius_m=mission.constraints.corridor_radius_m,\n            progress=0.0,\n            status=NavigationStatus.ACTIVE,\n            local_goal=local_goal,\n            trajectory_candidates=candidates,\n        )\n\n'


def replace_once(text: str, old: str, new: str, name: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{name} anchor matched {count} times; refusing to edit")
    return text.replace(old, new, 1)


def transform_l6(text: str) -> str:
    if any(marker not in text for marker in (
        "FOLLOW_PERSON", "follow_person_min_confidence", "def _follow_person_plan("
    )):
        raise RuntimeError("FOLLOW_PERSON V2 is not installed in L6")
    if "follow_person_lost_hold_ns" in text or "_follow_person_lost_since_ns" in text:
        raise RuntimeError("FOLLOW_PERSON P0 changes appear to be already present")

    text = replace_once(
        text,
        "    follow_person_release_tolerance_rad: float = 0.18\n",
        "    follow_person_release_tolerance_rad: float = 0.30\n",
        "L6 release tolerance default",
    )

    text = replace_once(
        text,
        "    follow_person_distance_deadband_m: float = 0.15\n    follow_person_min_safe_distance_m: float = 0.75\n",
        "    follow_person_distance_deadband_m: float = 0.15\n    follow_person_min_safe_distance_m: float = 0.75\n    follow_person_lost_hold_ns: int = 400_000_000\n    follow_person_pivot_enter_rad: float = 0.55\n    follow_person_slowdown_distance_m: float = 0.30\n",
        "L6 config fields",
    )

    old_validation = """        if (
            self.follow_person_min_safe_distance_m
            >= self.follow_person_stand_off_m - self.follow_person_distance_deadband_m
        ):
            raise ValueError(
                \"follow person safe distance must stay below the stand-off deadband\"
            )


@dataclass(frozen=True, slots=True)
class NavigationStateCheckpoint:
"""
    new_validation = """        if (
            self.follow_person_min_safe_distance_m
            >= self.follow_person_stand_off_m - self.follow_person_distance_deadband_m
        ):
            raise ValueError(
                \"follow person safe distance must stay below the stand-off deadband\"
            )
        if (
            not isinstance(self.follow_person_lost_hold_ns, int)
            or isinstance(self.follow_person_lost_hold_ns, bool)
            or self.follow_person_lost_hold_ns <= 0
        ):
            raise ValueError(\"follow_person_lost_hold_ns must be a positive integer\")
        if (
            not isinstance(self.follow_person_pivot_enter_rad, (int, float))
            or isinstance(self.follow_person_pivot_enter_rad, bool)
            or not math.isfinite(self.follow_person_pivot_enter_rad)
            or self.follow_person_pivot_enter_rad <= 0.0
            or self.follow_person_pivot_enter_rad >= math.pi
        ):
            raise ValueError(\"follow_person_pivot_enter_rad must be in (0, pi)\")
        if self.follow_person_pivot_enter_rad <= self.follow_person_release_tolerance_rad:
            raise ValueError(
                \"follow_person_pivot_enter_rad must exceed release tolerance\"
            )
        if (
            not isinstance(self.follow_person_slowdown_distance_m, (int, float))
            or isinstance(self.follow_person_slowdown_distance_m, bool)
            or not math.isfinite(self.follow_person_slowdown_distance_m)
            or self.follow_person_slowdown_distance_m <= 0.0
        ):
            raise ValueError(\"follow_person_slowdown_distance_m must be positive\")


@dataclass(frozen=True, slots=True)
class NavigationStateCheckpoint:
"""
    text = replace_once(text, old_validation, new_validation, "L6 P0 validation")
    text = replace_once(
        text,
        "    face_person_track_id: str | None = None\n    face_person_aligned: bool = False\n",
        "    face_person_track_id: str | None = None\n    face_person_aligned: bool = False\n    follow_person_track_id: str | None = None\n    follow_person_lost_since_ns: int | None = None\n    follow_person_pivoting: bool = False\n    follow_person_holding: bool = False\n",
        "checkpoint fields",
    )
    text = replace_once(
        text,
        "        \"_face_person_aligned\",\n        \"_face_person_track_id\",\n        \"_coverage\",\n",
        "        \"_face_person_aligned\",\n        \"_face_person_track_id\",\n        \"_follow_person_holding\",\n        \"_follow_person_lost_since_ns\",\n        \"_follow_person_pivoting\",\n        \"_follow_person_track_id\",\n        \"_coverage\",\n",
        "navigator slots",
    )
    text = replace_once(
        text,
        "        self._face_person_track_id: str | None = None\n        self._face_person_aligned = False\n",
        "        self._face_person_track_id: str | None = None\n        self._face_person_aligned = False\n        self._follow_person_track_id: str | None = None\n        self._follow_person_lost_since_ns: int | None = None\n        self._follow_person_pivoting = False\n        self._follow_person_holding = False\n",
        "navigator init",
    )
    text = replace_once(
        text,
        "            self._face_person_track_id,\n            self._face_person_aligned,\n        )\n",
        "            self._face_person_track_id,\n            self._face_person_aligned,\n            self._follow_person_track_id,\n            self._follow_person_lost_since_ns,\n            self._follow_person_pivoting,\n            self._follow_person_holding,\n        )\n",
        "checkpoint serialization",
    )
    text = replace_once(
        text,
        "        self._face_person_track_id = checkpoint.face_person_track_id\n        self._face_person_aligned = checkpoint.face_person_aligned\n        # Derived acceleration state is deliberately not part of replay authority.\n",
        "        self._face_person_track_id = checkpoint.face_person_track_id\n        self._face_person_aligned = checkpoint.face_person_aligned\n        self._follow_person_track_id = checkpoint.follow_person_track_id\n        self._follow_person_lost_since_ns = checkpoint.follow_person_lost_since_ns\n        self._follow_person_pivoting = checkpoint.follow_person_pivoting\n        self._follow_person_holding = checkpoint.follow_person_holding\n        # Derived acceleration state is deliberately not part of replay authority.\n",
        "checkpoint restore",
    )
    text = replace_once(
        text,
        "        self._face_person_track_id = None\n        self._face_person_aligned = False\n\n    def _face_person_plan(\n",
        "        self._face_person_track_id = None\n        self._face_person_aligned = False\n        self._follow_person_track_id = None\n        self._follow_person_lost_since_ns = None\n        self._follow_person_pivoting = False\n        self._follow_person_holding = False\n\n    def _face_person_plan(\n",
        "navigator reset",
    )
    text = replace_once(text, L6_OLD_METHOD, L6_NEW_METHOD, "FOLLOW_PERSON method")
    compile(text, "v3/layers/l6_navigation.py", "exec")
    return text


def transform_native(text: str) -> str:
    if "follow_person_value" not in text:
        raise RuntimeError("FOLLOW_PERSON V2 config loader is not installed")
    if "follow_person_lost_hold_ns" in text:
        raise RuntimeError("FOLLOW_PERSON P0 config loader already present")
    text = replace_once(
        text,
        'follow_person.get("release_tolerance_rad", 0.18)',
        'follow_person.get("release_tolerance_rad", 0.30)',
        "native release tolerance default",
    )
    old = """        follow_person_min_safe_distance_m=_positive_float(
            follow_person.get(\"min_safe_distance_m\", 0.75),
            \"v3_navigation.follow_person.min_safe_distance_m\",
        ),
        trajectory_replan_interval_ns=_positive_int(
"""
    new = """        follow_person_min_safe_distance_m=_positive_float(
            follow_person.get(\"min_safe_distance_m\", 0.75),
            \"v3_navigation.follow_person.min_safe_distance_m\",
        ),
        follow_person_lost_hold_ns=_positive_int(
            follow_person.get(\"lost_hold_ns\", 400_000_000),
            \"v3_navigation.follow_person.lost_hold_ns\",
        ),
        follow_person_pivot_enter_rad=_positive_float(
            follow_person.get(\"pivot_enter_rad\", 0.55),
            \"v3_navigation.follow_person.pivot_enter_rad\",
        ),
        follow_person_slowdown_distance_m=_positive_float(
            follow_person.get(\"slowdown_distance_m\", 0.30),
            \"v3_navigation.follow_person.slowdown_distance_m\",
        ),
        trajectory_replan_interval_ns=_positive_int(
"""
    text = replace_once(text, old, new, "native config injection")
    compile(text, "v3/composition/native_control.py", "exec")
    return text


def transform_test(text: str) -> str:
    start = text.find("def test_follow_person_keeps_selected_track_until_it_is_lost():")
    end = text.find("def test_follow_person_target_loss_is_fail_closed_but_reacquirable():")
    if start < 0 or end <= start:
        raise RuntimeError("V2 sticky-target test anchor not found")
    replacement = Path(__file__).resolve().parent.joinpath("tests/sticky_test_replacement.txt").read_text(encoding="utf-8")
    return text[:start] + replacement + "\n\n" + text[end:]


def transform_config(data: dict) -> dict:
    root = data.get("v3_navigation")
    if not isinstance(root, dict):
        raise RuntimeError("v3_navigation config missing")
    follow = root.get("follow_person")
    if not isinstance(follow, dict):
        raise RuntimeError("FOLLOW_PERSON V2 config missing")
    if "lost_hold_ns" in follow:
        raise RuntimeError("FOLLOW_PERSON P0 config already present")
    follow["release_tolerance_rad"] = 0.30
    follow["lost_hold_ns"] = 400_000_000
    follow["pivot_enter_rad"] = 0.55
    follow["slowdown_distance_m"] = 0.30
    return data


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("repo")
    ap.add_argument("--check", action="store_true")
    ns = ap.parse_args()
    repo = Path(ns.repo).resolve()
    pkg = Path(__file__).resolve().parent
    l6_path = repo / "v3/layers/l6_navigation.py"
    native_path = repo / "v3/composition/native_control.py"
    config_path = repo / "conf/vezerles.json"
    test_path = repo / "tests/test_v3_follow_person.py"
    p0_test_src = pkg / "tests/test_v3_follow_person_p0.py"
    p0_test_dst = repo / "tests/test_v3_follow_person_p0.py"

    for path in (l6_path, native_path, config_path, test_path, p0_test_src):
        if not path.is_file():
            raise SystemExit(f"missing required file: {path}")

    l6_new = transform_l6(l6_path.read_text(encoding="utf-8"))
    native_new = transform_native(native_path.read_text(encoding="utf-8"))
    test_new = transform_test(test_path.read_text(encoding="utf-8"))
    config_new = json.dumps(
        transform_config(json.loads(config_path.read_text(encoding="utf-8"))),
        indent=2,
        ensure_ascii=False,
    ) + "\n"
    p0_test = p0_test_src.read_text(encoding="utf-8")
    compile(test_new, str(test_path), "exec")
    compile(p0_test, str(p0_test_src), "exec")

    if ns.check:
        print("FOLLOW_PERSON P0 preflight: PASS")
        return 0

    outputs = (
        (l6_path, l6_new),
        (native_path, native_new),
        (config_path, config_new),
        (test_path, test_new),
        (p0_test_dst, p0_test),
    )
    temp_paths = []
    try:
        for path, content in outputs:
            tmp = path.with_name(path.name + ".follow-p0.tmp")
            tmp.write_text(content, encoding="utf-8", newline="\n")
            temp_paths.append((tmp, path))
        for tmp, path in temp_paths:
            os.replace(tmp, path)
    finally:
        for tmp, _path in temp_paths:
            if tmp.exists():
                tmp.unlink()
    print("FOLLOW_PERSON P0 applied")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
