#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
from pathlib import Path

EXPECTED_GIT_BLOB_SHA = "504ead8082bab75c5c9a4103883e78ea71e445da"

CONFIG_FIELDS_OLD = '''    face_person_min_confidence: float = 0.60
    face_person_align_tolerance_rad: float = 0.10
    face_person_release_tolerance_rad: float = 0.16

    def __post_init__(self) -> None:
'''
CONFIG_FIELDS_NEW = '''    face_person_min_confidence: float = 0.60
    face_person_align_tolerance_rad: float = 0.10
    face_person_release_tolerance_rad: float = 0.16
    follow_person_min_confidence: float = 0.60
    follow_person_align_tolerance_rad: float = 0.10
    follow_person_release_tolerance_rad: float = 0.18
    follow_person_stand_off_m: float = 1.05
    follow_person_distance_deadband_m: float = 0.15
    follow_person_min_safe_distance_m: float = 0.75

    def __post_init__(self) -> None:
'''

CONFIDENCE_OLD = '''        ):
            raise ValueError("face_person_min_confidence must be in [0, 1]")
        for name in (
            "face_person_align_tolerance_rad",
            "face_person_release_tolerance_rad",
        ):
'''
CONFIDENCE_NEW = '''        ):
            raise ValueError("face_person_min_confidence must be in [0, 1]")
        if (
            not isinstance(self.follow_person_min_confidence, (int, float))
            or isinstance(self.follow_person_min_confidence, bool)
            or not math.isfinite(self.follow_person_min_confidence)
            or not 0.0 <= self.follow_person_min_confidence <= 1.0
        ):
            raise ValueError("follow_person_min_confidence must be in [0, 1]")
        for name in (
            "face_person_align_tolerance_rad",
            "face_person_release_tolerance_rad",
        ):
'''

VALIDATION_OLD = '''        if self.face_person_release_tolerance_rad <= self.face_person_align_tolerance_rad:
            raise ValueError(
                "face_person_release_tolerance_rad must exceed align tolerance"
            )


@dataclass(frozen=True, slots=True)
class NavigationStateCheckpoint:
'''
VALIDATION_NEW = '''        if self.face_person_release_tolerance_rad <= self.face_person_align_tolerance_rad:
            raise ValueError(
                "face_person_release_tolerance_rad must exceed align tolerance"
            )
        for name in (
            "follow_person_align_tolerance_rad",
            "follow_person_release_tolerance_rad",
        ):
            value = getattr(self, name)
            if (
                not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not math.isfinite(value)
                or value <= 0.0
                or value >= math.pi
            ):
                raise ValueError(f"{name} must be in (0, pi)")
        if self.follow_person_release_tolerance_rad <= self.follow_person_align_tolerance_rad:
            raise ValueError(
                "follow_person_release_tolerance_rad must exceed align tolerance"
            )
        for name in (
            "follow_person_stand_off_m",
            "follow_person_min_safe_distance_m",
        ):
            value = getattr(self, name)
            if (
                not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not math.isfinite(value)
                or value <= 0.0
            ):
                raise ValueError(f"{name} must be positive")
        if (
            not isinstance(self.follow_person_distance_deadband_m, (int, float))
            or isinstance(self.follow_person_distance_deadband_m, bool)
            or not math.isfinite(self.follow_person_distance_deadband_m)
            or self.follow_person_distance_deadband_m < 0.0
        ):
            raise ValueError("follow_person_distance_deadband_m cannot be negative")
        if (
            self.follow_person_min_safe_distance_m
            >= self.follow_person_stand_off_m - self.follow_person_distance_deadband_m
        ):
            raise ValueError(
                "follow person safe distance must stay below the stand-off deadband"
            )


@dataclass(frozen=True, slots=True)
class NavigationStateCheckpoint:
'''

EVALUATE_OLD = '''        if mission.mode is CommandMode.FACE_PERSON:
            return self._face_person_plan(mission, estimate, world)
        if mission.mode is CommandMode.EXPLORE:
            return self._exploration_plan(mission, estimate, world)
'''
EVALUATE_NEW = '''        if mission.mode is CommandMode.FACE_PERSON:
            return self._face_person_plan(mission, estimate, world)
        if mission.mode is CommandMode.FOLLOW_PERSON:
            return self._follow_person_plan(mission, estimate, world)
        if mission.mode is CommandMode.EXPLORE:
            return self._exploration_plan(mission, estimate, world)
'''

FOLLOW_METHOD = '''    def _follow_person_plan(
        self,
        mission: MissionIntent,
        estimate: RobotEstimate,
        world: WorldSnapshot,
    ) -> NavigationPlan:
        if self._mission_id != mission.mission_id:
            self._reset()
            self._mission_id = mission.mission_id

        eligible = tuple(
            track
            for track in world.obstacle_tracks
            if track.track_id.startswith("person-")
            and track.confidence >= self._config.follow_person_min_confidence
        )
        previous_track_id = self._face_person_track_id
        selected = next(
            (
                track
                for track in eligible
                if track.track_id == self._face_person_track_id
            ),
            None,
        )
        if selected is None and eligible:
            selected = min(
                eligible,
                key=lambda track: (
                    -track.confidence,
                    math.hypot(track.x_m - estimate.x_m, track.y_m - estimate.y_m),
                    track.track_id,
                ),
            )
            if selected.track_id != previous_track_id:
                self._clear_trajectory_plan()
            self._face_person_track_id = selected.track_id
            self._face_person_aligned = False

        if selected is None:
            self._face_person_track_id = None
            self._face_person_aligned = False
            self._clear_trajectory_plan()
            return self._inactive(
                mission,
                NavigationStatus.INVALIDATED,
                "PERSON_TARGET_NOT_AVAILABLE",
            )

        dx = selected.x_m - estimate.x_m
        dy = selected.y_m - estimate.y_m
        distance_m = math.hypot(dx, dy)
        if distance_m <= 1e-9:
            desired_yaw = estimate.yaw_rad
            heading_error = 0.0
        else:
            desired_yaw = math.atan2(dy, dx)
            heading_error = _wrapped_angle(desired_yaw - estimate.yaw_rad)

        if distance_m <= self._config.follow_person_min_safe_distance_m:
            self._face_person_aligned = False
            self._clear_trajectory_plan()
            return self._inactive(
                mission,
                NavigationStatus.INVALIDATED,
                "PERSON_TOO_CLOSE",
            )

        absolute_error = abs(heading_error)
        if self._face_person_aligned:
            if absolute_error > self._config.follow_person_release_tolerance_rad:
                self._face_person_aligned = False
        elif absolute_error <= self._config.follow_person_align_tolerance_rad:
            self._face_person_aligned = True

        if not self._face_person_aligned:
            self._clear_trajectory_plan()
            return NavigationPlan(
                context=mission.context,
                mission_id=mission.mission_id,
                route=(Waypoint(estimate.x_m, estimate.y_m, desired_yaw),),
                velocity_target=None,
                constraints=mission.constraints,
                corridor_radius_m=0.0,
                progress=0.0,
                status=NavigationStatus.ACTIVE,
            )

        if (
            distance_m
            <= self._config.follow_person_stand_off_m
            + self._config.follow_person_distance_deadband_m
        ):
            self._clear_trajectory_plan()
            return NavigationPlan(
                context=mission.context,
                mission_id=mission.mission_id,
                route=(Waypoint(estimate.x_m, estimate.y_m, estimate.yaw_rad),),
                velocity_target=None,
                constraints=mission.constraints,
                corridor_radius_m=0.0,
                progress=0.0,
                status=NavigationStatus.ACTIVE,
            )

        costmap = world.local_costmap
        if costmap is None:
            self._clear_trajectory_plan()
            return self._inactive(
                mission,
                NavigationStatus.INVALIDATED,
                "LOCAL_COSTMAP_MISSING",
            )
        if costmap.freshness_ns > self._config.max_costmap_freshness_ns:
            self._clear_trajectory_plan()
            return self._inactive(
                mission,
                NavigationStatus.INVALIDATED,
                "LOCAL_COSTMAP_STALE",
            )

        self._accept_pending_rollout(mission.context)
        if self._replan_due(
            mission.context.monotonic_ns,
            mission.context.tick_id,
        ):
            travel_m = min(
                distance_m - self._config.follow_person_stand_off_m,
                self._config.local_goal_distance_m,
            )
            local_goal = Waypoint(
                estimate.x_m + travel_m * math.cos(desired_yaw),
                estimate.y_m + travel_m * math.sin(desired_yaw),
            )
            self._schedule_or_store_rollout(
                mission.context,
                estimate,
                world,
                local_goal,
                mission.constraints.max_v_mps,
                mission.constraints.max_omega_rad_s,
            )

        local_goal = self._local_goal
        candidates = self._trajectory_candidates
        if local_goal is None or not candidates:
            raise RuntimeError("follow-person trajectory cache is empty after replanning")
        return NavigationPlan(
            context=mission.context,
            mission_id=mission.mission_id,
            route=(),
            velocity_target=None,
            constraints=mission.constraints,
            corridor_radius_m=mission.constraints.corridor_radius_m,
            progress=0.0,
            status=NavigationStatus.ACTIVE,
            local_goal=local_goal,
            trajectory_candidates=candidates,
        )

'''

METHOD_ANCHOR = '''    def _exploration_plan(
        self,
        mission: MissionIntent,
        estimate: RobotEstimate,
        world: WorldSnapshot,
    ) -> NavigationPlan:
'''

REPLACEMENTS = (
    ("config fields", CONFIG_FIELDS_OLD, CONFIG_FIELDS_NEW),
    ("confidence validation", CONFIDENCE_OLD, CONFIDENCE_NEW),
    ("follow validation", VALIDATION_OLD, VALIDATION_NEW),
    ("evaluate routing", EVALUATE_OLD, EVALUATE_NEW),
    ("follow method", METHOD_ANCHOR, FOLLOW_METHOD + METHOD_ANCHOR),
)


def git_blob_sha(data: bytes) -> str:
    header = f"blob {len(data)}\0".encode()
    return hashlib.sha1(header + data).hexdigest()


def transform(text: str) -> str:
    if "def _follow_person_plan(" in text or "follow_person_min_confidence" in text:
        raise RuntimeError("FOLLOW_PERSON L6 changes appear to be already present")
    for name, old, new in REPLACEMENTS:
        count = text.count(old)
        if count != 1:
            raise RuntimeError(f"anchor {name!r} matched {count} times; refusing to edit")
        text = text.replace(old, new, 1)
    return text


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("path", nargs="?", default="v3/layers/l6_navigation.py")
    ap.add_argument("--check", action="store_true")
    ns = ap.parse_args()
    path = Path(ns.path)
    raw = path.read_bytes()
    sha = git_blob_sha(raw)
    if sha != EXPECTED_GIT_BLOB_SHA:
        raise SystemExit(
            f"L6 source blob mismatch: expected {EXPECTED_GIT_BLOB_SHA}, got {sha}. "
            "Refusing to edit a different L6."
        )
    text = raw.decode("utf-8")
    updated = transform(text)
    compile(updated, str(path), "exec")
    if ns.check:
        print("L6 FOLLOW_PERSON transform check: PASS")
        return 0
    path.write_text(updated, encoding="utf-8", newline="\n")
    print("L6 FOLLOW_PERSON transform applied")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
