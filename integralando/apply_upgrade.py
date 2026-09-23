#!/usr/bin/env python3
from __future__ import annotations

import argparse
import ast
import hashlib
import json
import os
import py_compile
import shutil
import subprocess
import sys
import tempfile
from datetime import datetime
from pathlib import Path

UPGRADE_ID = "r2b4_follow_person_p0_v2_1_20260923"
BASE_HEAD = "f55f459b2b3e420ce992d96355329a33c5a6e566"
MARKER = "R2B4_FOLLOW_PERSON_P0_V2_20260923"

EXPECTED_GIT_BLOB_SHA1 = {
    "v3/layers/l4_temporal_tracking.py": "7b3002ebaa40c76d8cbc7b8a2869cc634a563643",
    "v3/layers/l4_world_model.py": "a66dea75b9826d9ec94ca1f6d6df4dcedc3fc3b3",
    "v3/layers/l6_navigation.py": "5bb0127d543e61a298048c59400b421c5ec0d7bb",
    "v3/composition/native_control.py": "225158f1d4c34580a499c4384be2bb4682fc5b3a",
    "v3/test_hub_task_evidence.py": "3913d5c76b6a5385bdbd744e1018aed8ebe8767a",
    "conf/vezerles.json": "47924a10a9efb506b27acaf8e4353b15abcecca7",
    "tests/test_v3_follow_person_modernization.py": "dcc07a6e3a84afa251632e86c9a9ae3120a24d12",
}


class UpgradeError(RuntimeError):
    pass


def sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def git_blob_sha1(text: str) -> str:
    data = text.encode("utf-8")
    header = f"blob {len(data)}\0".encode("ascii")
    return hashlib.sha1(header + data).hexdigest()


def read_text(root: Path, rel: str) -> str:
    path = root / rel
    if not path.is_file():
        raise UpgradeError(f"required file missing: {rel}")
    return path.read_text(encoding="utf-8")


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise UpgradeError(f"{label}: expected exactly one semantic anchor, found {count}")
    return text.replace(old, new, 1)


def parse_python(text: str, rel: str) -> ast.Module:
    try:
        return ast.parse(text, filename=rel)
    except SyntaxError as exc:
        raise UpgradeError(f"{rel}: Python syntax invalid before/after patch: {exc}") from exc


def class_names(tree: ast.Module) -> set[str]:
    return {node.name for node in tree.body if isinstance(node, ast.ClassDef)}


def module_function_names(tree: ast.Module) -> set[str]:
    return {node.name for node in tree.body if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))}


def class_method_names(tree: ast.Module, cls_name: str) -> set[str]:
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == cls_name:
            return {item.name for item in node.body if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef))}
    return set()


def replace_top_level_class(text: str, class_name: str, replacement: str, rel: str) -> str:
    tree = parse_python(text, rel)
    matches = [n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name]
    if len(matches) != 1:
        raise UpgradeError(f"{rel}: class {class_name} not uniquely found")
    node = matches[0]
    lines = text.splitlines(keepends=True)
    start = node.lineno - 1
    end = node.end_lineno
    replacement = replacement.rstrip() + "\n\n"
    return "".join(lines[:start]) + replacement + "".join(lines[end:])


def _top_level_function(text: str, function_name: str, rel: str) -> ast.FunctionDef | ast.AsyncFunctionDef:
    tree = parse_python(text, rel)
    matches = [
        node
        for node in tree.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name == function_name
    ]
    if len(matches) != 1:
        raise UpgradeError(f"{rel}: function {function_name} not uniquely found")
    return matches[0]


def insert_before_top_level_function(
    text: str, function_name: str, block: str, rel: str
) -> str:
    node = _top_level_function(text, function_name, rel)
    lines = text.splitlines(keepends=True)
    start = node.lineno - 1
    block = block.rstrip() + "\n\n"
    return "".join(lines[:start]) + block + "".join(lines[start:])


def insert_function_body_prefix(
    text: str, function_name: str, prefix: str, rel: str
) -> str:
    node = _top_level_function(text, function_name, rel)
    if not node.body:
        raise UpgradeError(f"{rel}: function {function_name} has no body")
    first = node.body[0]
    # Preserve a function docstring if one is added later.
    if (
        isinstance(first, ast.Expr)
        and isinstance(first.value, ast.Constant)
        and isinstance(first.value.value, str)
    ):
        insert_at = first.end_lineno
    else:
        insert_at = first.lineno - 1
    lines = text.splitlines(keepends=True)
    prefix = prefix.rstrip() + "\n"
    return "".join(lines[:insert_at]) + prefix + "".join(lines[insert_at:])


def semantic_preflight(root: Path) -> dict[str, str]:
    current: dict[str, str] = {}
    for rel, expected in EXPECTED_GIT_BLOB_SHA1.items():
        text = read_text(root, rel)
        blob_sha = git_blob_sha1(text)
        current[rel] = sha256_text(text)
        if rel.endswith(".py"):
            tree = parse_python(text, rel)
            if rel.endswith("l4_temporal_tracking.py"):
                if not {"TemporalTrackCheckpoint", "TemporalTrackStore"}.issubset(class_names(tree)):
                    raise UpgradeError(f"{rel}: expected temporal tracking classes missing")
            elif rel.endswith("l4_world_model.py") and "WorldModelConfig" not in class_names(tree):
                raise UpgradeError(f"{rel}: WorldModelConfig missing")
            elif rel.endswith("l6_navigation.py"):
                if "TrajectoryNavigator" not in class_names(tree):
                    raise UpgradeError(f"{rel}: TrajectoryNavigator missing")
                methods = class_method_names(tree, "TrajectoryNavigator")
                if not {"evaluate", "_follow_person_plan", "_reset"}.issubset(methods):
                    raise UpgradeError(f"{rel}: expected FOLLOW_PERSON methods missing")
            elif rel.endswith("native_control.py") and "NativeControlComposition" not in class_names(tree):
                raise UpgradeError(f"{rel}: NativeControlComposition missing")
            elif rel.endswith("test_hub_task_evidence.py"):
                funcs = module_function_names(tree)
                if not {"_navigation_configuration", "_follow_person_evidence"}.issubset(funcs):
                    raise UpgradeError(f"{rel}: expected Test Hub functions missing")
        if blob_sha != expected:
            raise UpgradeError(
                f"{rel}: source blob differs from the source-first baseline "
                f"(expected {expected[:12]}, got {blob_sha[:12]}). "
                "No files were changed. Refresh the upgrade package for this repo state."
            )
    return current


L4_CHECKPOINT = r'''
@dataclass(frozen=True, slots=True)
class TemporalTrackCheckpoint:
    states: tuple[TemporalTrackState, ...]
    next_person_track_sequence: int = 1
    # Dormant identities are replay authority but are never public obstacle tracks.
    dormant_states: tuple[TemporalTrackState, ...] = ()

    def __post_init__(self) -> None:
        value = self.next_person_track_sequence
        if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
            raise ValueError("next_person_track_sequence must be a positive integer")
        active = {state.track.track_id for state in self.states}
        dormant = {state.track.track_id for state in self.dormant_states}
        if active & dormant:
            raise ValueError("active and dormant track identities must be disjoint")
'''


L4_STORE = r'''
class TemporalTrackStore:
    """Bounded active tracking plus non-published short-lived person identity memory."""

    __slots__ = (
        "_alpha",
        "_beta",
        "_prediction_max_age_ns",
        "_max_speed_mps",
        "_person_reacquire_max_age_ns",
        "_next_person_track_sequence",
        "_states",
        "_dormant_states",
    )

    def __init__(
        self,
        *,
        alpha: float,
        beta: float,
        prediction_max_age_ns: int,
        max_speed_mps: float,
        person_reacquire_max_age_ns: int = 2_500_000_000,
    ) -> None:
        if not 0.0 < alpha <= 1.0 or not 0.0 < beta <= 1.0:
            raise ValueError("track alpha/beta must be in (0, 1]")
        if prediction_max_age_ns < 0 or max_speed_mps <= 0.0:
            raise ValueError("invalid track prediction bounds")
        if (
            not isinstance(person_reacquire_max_age_ns, int)
            or isinstance(person_reacquire_max_age_ns, bool)
            or person_reacquire_max_age_ns <= 0
        ):
            raise ValueError("person_reacquire_max_age_ns must be positive integer")
        self._alpha = alpha
        self._beta = beta
        self._prediction_max_age_ns = prediction_max_age_ns
        self._max_speed_mps = max_speed_mps
        self._person_reacquire_max_age_ns = person_reacquire_max_age_ns
        self._next_person_track_sequence = 1
        self._states: dict[str, TemporalTrackState] = {}
        self._dormant_states: dict[str, TemporalTrackState] = {}

    def clear(self) -> bool:
        changed = bool(self._states or self._dormant_states)
        self._states.clear()
        self._dormant_states.clear()
        # The monotonic person allocator intentionally survives clear().
        return changed

    def upsert_external(self, track: ObstacleTrack, captured_ns: int) -> bool:
        previous = self._states.get(track.track_id)
        if previous is None:
            previous = self._dormant_states.get(track.track_id)
        if previous is not None and captured_ns < previous.captured_ns:
            raise ValueError("L4 obstacle track time must not move backwards")
        updates = 1 if previous is None else previous.updates + 1
        state = TemporalTrackState(track, captured_ns, updates)
        changed = state != previous or track.track_id in self._dormant_states
        self._dormant_states.pop(track.track_id, None)
        self._states[track.track_id] = state
        self._observe_person_track_id(track.track_id)
        return changed

    def associate_people(
        self,
        measurements: tuple[PersonMeasurement, ...],
        *,
        captured_ns: int,
        radius_m: float,
        max_association_distance_m: float,
    ) -> bool:
        active = {
            track_id: state
            for track_id, state in self._states.items()
            if track_id.startswith("person-")
        }
        dormant = {
            track_id: state
            for track_id, state in self._dormant_states.items()
            if track_id.startswith("person-")
            and 0 < captured_ns - state.captured_ns <= self._person_reacquire_max_age_ns
        }
        used_track_ids: set[str] = set()
        changed = False
        for measurement in measurements:
            best = self._best_person_match(
                active, used_track_ids, measurement, captured_ns, max_association_distance_m
            )
            reactivated = False
            if best is None:
                best = self._best_person_match(
                    dormant, used_track_ids, measurement, captured_ns, max_association_distance_m
                )
                reactivated = best is not None

            if best is None:
                track_id = self._allocate_person_track_id()
                track = ObstacleTrack(
                    track_id=track_id,
                    x_m=measurement.x_m,
                    y_m=measurement.y_m,
                    radius_m=radius_m,
                    vx_mps=0.0,
                    vy_mps=0.0,
                    confidence=measurement.confidence,
                )
                self._states[track_id] = TemporalTrackState(track, captured_ns, 1)
                used_track_ids.add(track_id)
                changed = True
                continue

            _, track_id, state = best
            used_track_ids.add(track_id)
            if reactivated:
                self._dormant_states.pop(track_id, None)
            self._states[track_id] = self._updated_person_state(
                track_id, state, measurement, captured_ns, radius_m, reactivated=reactivated
            )
            changed = True
        return changed

    def _best_person_match(
        self,
        pool: dict[str, TemporalTrackState],
        used_track_ids: set[str],
        measurement: PersonMeasurement,
        captured_ns: int,
        max_association_distance_m: float,
    ) -> tuple[float, str, TemporalTrackState] | None:
        best: tuple[float, str, TemporalTrackState] | None = None
        for track_id in sorted(pool):
            if track_id in used_track_ids:
                continue
            state = pool[track_id]
            dt_ns = captured_ns - state.captured_ns
            if dt_ns <= 0:
                continue
            dt_s = dt_ns / 1e9
            previous = state.track
            projected_s = min(dt_ns, self._prediction_max_age_ns) / 1e9
            predicted_x = previous.x_m + previous.vx_mps * projected_s
            predicted_y = previous.y_m + previous.vy_mps * projected_s
            distance_m = math.hypot(measurement.x_m - predicted_x, measurement.y_m - predicted_y)
            raw_speed_mps = math.hypot(
                measurement.x_m - previous.x_m, measurement.y_m - previous.y_m
            ) / dt_s
            if distance_m > max_association_distance_m or raw_speed_mps > self._max_speed_mps:
                continue
            candidate = (distance_m, track_id, state)
            if best is None or candidate[:2] < best[:2]:
                best = candidate
        return best

    def _updated_person_state(
        self,
        track_id: str,
        state: TemporalTrackState,
        measurement: PersonMeasurement,
        captured_ns: int,
        radius_m: float,
        *,
        reactivated: bool,
    ) -> TemporalTrackState:
        previous = state.track
        dt_s = (captured_ns - state.captured_ns) / 1e9
        if reactivated or state.updates == 1:
            # Reactivate exactly at the fresh measurement; stale velocity is not
            # extrapolated blindly through an occlusion.
            x_m = measurement.x_m
            y_m = measurement.y_m
            vx_mps = (measurement.x_m - previous.x_m) / dt_s
            vy_mps = (measurement.y_m - previous.y_m) / dt_s
        else:
            predicted_x = previous.x_m + previous.vx_mps * dt_s
            predicted_y = previous.y_m + previous.vy_mps * dt_s
            residual_x = measurement.x_m - predicted_x
            residual_y = measurement.y_m - predicted_y
            x_m = predicted_x + self._alpha * residual_x
            y_m = predicted_y + self._alpha * residual_y
            vx_mps = previous.vx_mps + self._beta * residual_x / dt_s
            vy_mps = previous.vy_mps + self._beta * residual_y / dt_s
        speed = math.hypot(vx_mps, vy_mps)
        if speed > self._max_speed_mps:
            scale = self._max_speed_mps / speed
            vx_mps *= scale
            vy_mps *= scale
        return TemporalTrackState(
            ObstacleTrack(
                track_id=track_id,
                x_m=x_m,
                y_m=y_m,
                radius_m=radius_m,
                vx_mps=vx_mps,
                vy_mps=vy_mps,
                confidence=measurement.confidence,
            ),
            captured_ns,
            state.updates + 1,
        )

    def expire(self, now_ns: int, *, person_max_age_ns: int, other_max_age_ns: int) -> bool:
        expired_active = tuple(
            track_id
            for track_id, state in self._states.items()
            if now_ns - state.captured_ns
            > (person_max_age_ns if track_id.startswith("person-") else other_max_age_ns)
        )
        for track_id in expired_active:
            state = self._states.pop(track_id)
            if track_id.startswith("person-"):
                self._dormant_states[track_id] = state

        expired_dormant = tuple(
            track_id
            for track_id, state in self._dormant_states.items()
            if now_ns - state.captured_ns > self._person_reacquire_max_age_ns
        )
        for track_id in expired_dormant:
            del self._dormant_states[track_id]
        # Only active-set changes affect the public world snapshot/map revision.
        return bool(expired_active)

    def projected_tracks(self, now_ns: int) -> tuple[ObstacleTrack, ...]:
        result: list[ObstacleTrack] = []
        for track_id in sorted(self._states):
            state = self._states[track_id]
            track = state.track
            age_ns = max(0, now_ns - state.captured_ns)
            projected_ns = min(age_ns, self._prediction_max_age_ns)
            dt_s = projected_ns / 1e9
            result.append(
                ObstacleTrack(
                    track_id=track.track_id,
                    x_m=track.x_m + track.vx_mps * dt_s,
                    y_m=track.y_m + track.vy_mps * dt_s,
                    radius_m=track.radius_m,
                    vx_mps=track.vx_mps,
                    vy_mps=track.vy_mps,
                    confidence=track.confidence,
                )
            )
        return tuple(result)

    def checkpoint(self) -> TemporalTrackCheckpoint:
        return TemporalTrackCheckpoint(
            tuple(self._states[key] for key in sorted(self._states)),
            self._next_person_track_sequence,
            tuple(self._dormant_states[key] for key in sorted(self._dormant_states)),
        )

    def restore(self, checkpoint: TemporalTrackCheckpoint) -> None:
        if not isinstance(checkpoint, TemporalTrackCheckpoint):
            raise TypeError("checkpoint must be TemporalTrackCheckpoint")
        self._states = {state.track.track_id: state for state in checkpoint.states}
        self._dormant_states = {state.track.track_id: state for state in checkpoint.dormant_states}
        if self._states.keys() & self._dormant_states.keys():
            raise ValueError("active and dormant track identities must be disjoint")
        self._next_person_track_sequence = checkpoint.next_person_track_sequence
        for track_id in (*self._states, *self._dormant_states):
            self._observe_person_track_id(track_id)

    def _observe_person_track_id(self, track_id: str) -> None:
        if not track_id.startswith("person-"):
            return
        suffix = track_id[len("person-") :]
        if suffix.isdigit():
            self._next_person_track_sequence = max(self._next_person_track_sequence, int(suffix) + 1)

    def _allocate_person_track_id(self) -> str:
        candidate = self._next_person_track_sequence
        while f"person-{candidate}" in self._states or f"person-{candidate}" in self._dormant_states:
            candidate += 1
        self._next_person_track_sequence = candidate + 1
        return f"person-{candidate}"
'''


FOLLOW_EVIDENCE = r'''
@dataclass(frozen=True, slots=True)
class FollowPersonEvidence:
    """Passive L6 follow state for capture/Test Hub; never feeds control."""

    state: str
    locked_target_uid: str | None
    target_visible: bool
    target_confidence: float | None
    target_distance_m: float | None
    target_bearing_rad: float | None
    target_missing_age_ms: float | None
    person_candidate_count: int
    search_phase: int | None
    search_target_yaw_rad: float | None
    acquisition_min_confidence: float
    retention_min_confidence: float
    stand_off_m: float
    distance_deadband_m: float
    min_safe_distance_m: float
    lost_hold_ns: int
    search_timeout_ns: int
    search_sweep_rad: float
    search_yaw_tolerance_rad: float
'''


FOLLOW_WRAPPER = r'''
    @property
    def follow_person_evidence(self) -> FollowPersonEvidence | None:
        """Return passive evidence produced by the latest FOLLOW_PERSON tick."""
        return self._follow_person_evidence

    def _follow_person_plan(
        self,
        mission: MissionIntent,
        estimate: RobotEstimate,
        world: WorldSnapshot,
    ) -> NavigationPlan:
        plan = self._follow_person_plan_impl(mission, estimate, world)
        self._follow_person_evidence = self._build_follow_person_evidence(mission, estimate, world)
        return plan

    def _build_follow_person_evidence(
        self,
        mission: MissionIntent,
        estimate: RobotEstimate,
        world: WorldSnapshot,
    ) -> FollowPersonEvidence:
        locked_uid = self._follow_person_track_id
        retention = self._config.follow_person_retention_min_confidence
        if retention is None:
            retention = self._config.follow_person_min_confidence
        selected = next(
            (
                track
                for track in world.obstacle_tracks
                if locked_uid is not None
                and track.track_id == locked_uid
                and track.confidence >= retention
            ),
            None,
        )
        target_distance_m = target_bearing_rad = target_confidence = None
        if selected is not None:
            dx = selected.x_m - estimate.x_m
            dy = selected.y_m - estimate.y_m
            target_distance_m = math.hypot(dx, dy)
            desired_yaw = estimate.yaw_rad if target_distance_m <= 1e-9 else math.atan2(dy, dx)
            target_bearing_rad = _wrapped_angle(desired_yaw - estimate.yaw_rad)
            target_confidence = selected.confidence
        missing_age_ms = None
        if self._follow_person_lost_since_ns is not None:
            missing_age_ms = max(0.0, (mission.context.monotonic_ns - self._follow_person_lost_since_ns) / 1e6)
        return FollowPersonEvidence(
            state=self._follow_person_state.value,
            locked_target_uid=locked_uid,
            target_visible=selected is not None,
            target_confidence=target_confidence,
            target_distance_m=target_distance_m,
            target_bearing_rad=target_bearing_rad,
            target_missing_age_ms=missing_age_ms,
            person_candidate_count=sum(1 for track in world.obstacle_tracks if track.track_id.startswith("person-")),
            search_phase=self._follow_person_search_phase if self._follow_person_state is _FollowPersonState.SEARCH else None,
            search_target_yaw_rad=self._follow_person_search_target_yaw_rad if self._follow_person_state is _FollowPersonState.SEARCH else None,
            acquisition_min_confidence=self._config.follow_person_min_confidence,
            retention_min_confidence=retention,
            stand_off_m=self._config.follow_person_stand_off_m,
            distance_deadband_m=self._config.follow_person_distance_deadband_m,
            min_safe_distance_m=self._config.follow_person_min_safe_distance_m,
            lost_hold_ns=self._config.follow_person_lost_hold_ns,
            search_timeout_ns=self._config.follow_person_search_timeout_ns,
            search_sweep_rad=self._config.follow_person_search_sweep_rad,
            search_yaw_tolerance_rad=self._config.follow_person_search_yaw_tolerance_rad,
        )

'''


DIRECT_FOLLOW_HELPERS = r'''
def _captured_follow_evidence(tick: Mapping[str, object]) -> Mapping[str, object]:
    for item in _seq(tick.get("tick_evidence")):
        row = _map(item)
        if row.get("__type__") == "FollowPersonEvidence":
            return row
    return {}


def _follow_person_direct_evidence(
    episode: Mapping[str, object],
    ticks: Sequence[Mapping[str, object]],
    navigation_config: Mapping[str, object],
) -> tuple[dict[str, object], list[dict[str, object]], list[str]]:
    config = _map(navigation_config.get("follow_person"))
    rows = [(tick, evidence) for tick in ticks if (evidence := _captured_follow_evidence(tick))]
    state_counts: Counter[str] = Counter()
    search_phase_counts: Counter[str] = Counter()
    person_candidate_counts: list[float] = []
    distances: list[float] = []
    stand_off_errors: list[float] = []
    bearings: list[float] = []
    safe_margins: list[float] = []
    search_yaws: list[float] = []
    visible = missing = losses = reacquisitions = switches = 0
    longest_missing_s = 0.0
    previous_visible: bool | None = None
    previous_uid: str | None = None
    events: list[dict[str, object]] = []

    first = rows[0][1] if rows else {}
    acquire_conf = _num(first.get("acquisition_min_confidence")) or _num(config.get("minimum_confidence"))
    retain_conf = _num(first.get("retention_min_confidence")) or _num(config.get("retention_minimum_confidence"))
    stand_off = _num(first.get("stand_off_m")) or _num(config.get("stand_off_m"))
    deadband = _num(first.get("distance_deadband_m"))
    if deadband is None:
        deadband = _num(config.get("distance_deadband_m"))
    min_safe = _num(first.get("min_safe_distance_m")) or _num(config.get("min_safe_distance_m"))

    for tick, evidence in rows:
        state_counts[str(evidence.get("state") or "UNKNOWN")] += 1
        phase = evidence.get("search_phase")
        if phase is not None:
            search_phase_counts[str(phase)] += 1
        count = _integer(evidence.get("person_candidate_count"))
        if count is not None:
            person_candidate_counts.append(float(count))
        uid = evidence.get("locked_target_uid") if isinstance(evidence.get("locked_target_uid"), str) else None
        if previous_uid is not None and uid is not None and uid != previous_uid:
            switches += 1
        if uid is not None:
            previous_uid = uid
        is_visible = evidence.get("target_visible") is True
        if is_visible:
            visible += 1
            if previous_visible is False:
                reacquisitions += 1
                events.append(_event("FOLLOW_TARGET_REACQUIRED_OBSERVED", episode=episode, tick=tick, extra={"track_id": uid}))
        else:
            if uid is not None:
                missing += 1
            if previous_visible is True:
                losses += 1
                events.append(_event("FOLLOW_TARGET_LOST_OBSERVED", episode=episode, tick=tick, extra={"track_id": uid}))
        if uid is not None:
            previous_visible = is_visible
        missing_age_ms = _num(evidence.get("target_missing_age_ms"))
        if missing_age_ms is not None:
            longest_missing_s = max(longest_missing_s, missing_age_ms / 1000.0)
        distance = _num(evidence.get("target_distance_m"))
        if distance is not None:
            distances.append(distance)
            if stand_off is not None:
                stand_off_errors.append(distance - stand_off)
            if min_safe is not None:
                safe_margins.append(distance - min_safe)
        bearing = _num(evidence.get("target_bearing_rad"))
        if bearing is not None:
            bearings.append(bearing)
        search_yaw = _num(evidence.get("search_target_yaw_rad"))
        if search_yaw is not None:
            search_yaws.append(search_yaw)

    observed = visible + missing
    within_deadband = (
        sum(1 for error in stand_off_errors if deadband is not None and abs(error) <= deadband) / len(stand_off_errors)
        if stand_off_errors and deadband is not None else None
    )
    locked_ids = [str(e.get("locked_target_uid")) for _t, e in rows if isinstance(e.get("locked_target_uid"), str)]
    return {
        "target_identity": {
            "inference_source": "CAPTURED_L6_FOLLOW_PERSON_EVIDENCE",
            "locked_track_id": locked_ids[-1] if locked_ids else None,
            "observed_target_switch_count": switches,
        },
        "capture_time_follow_config": {
            "acquisition_min_confidence": acquire_conf,
            "retention_min_confidence": retain_conf,
            "stand_off_m": stand_off,
            "distance_deadband_m": deadband,
            "min_safe_distance_m": min_safe,
            "lost_hold_ns": _integer(first.get("lost_hold_ns")),
            "search_timeout_ns": _integer(first.get("search_timeout_ns")),
            "search_sweep_rad": _num(first.get("search_sweep_rad")),
            "search_yaw_tolerance_rad": _num(first.get("search_yaw_tolerance_rad")),
        },
        "supervisor_state_sample_counts": dict(sorted(state_counts.items())),
        "search_phase_sample_counts": dict(sorted(search_phase_counts.items())),
        "visibility": {
            "visible_sample_count": visible,
            "missing_sample_count": missing,
            "visible_sample_fraction": visible / observed if observed else None,
            "observed_loss_count": losses,
            "observed_reacquisition_count": reacquisitions,
            "longest_observed_missing_span_s": longest_missing_s,
        },
        "person_candidate_count": _stats(person_candidate_counts),
        "target_distance_m": _stats(distances),
        "stand_off_error_m": _error_stats(stand_off_errors),
        "within_stand_off_deadband_sample_fraction": within_deadband,
        "minimum_safe_distance_margin_m": _stats(safe_margins),
        "target_heading_error_rad": _error_stats(bearings),
        "search_target_yaw_rad": _stats(search_yaws),
    }, events, []
'''


def patch_l4_temporal(text: str) -> str:
    rel = "v3/layers/l4_temporal_tracking.py"
    text = replace_top_level_class(text, "TemporalTrackCheckpoint", L4_CHECKPOINT, rel)
    text = replace_top_level_class(text, "TemporalTrackStore", L4_STORE, rel)
    return f"# {MARKER}\n" + text


def patch_l4_world(text: str) -> str:
    text = replace_once(text, "    person_track_max_age_ns: int = 500_000_000\n", "    person_track_max_age_ns: int = 500_000_000\n    person_track_reacquire_max_age_ns: int = 2_500_000_000\n", "l4 world config")
    text = replace_once(text, '            "person_track_max_age_ns",\n', '            "person_track_max_age_ns",\n            "person_track_reacquire_max_age_ns",\n', "l4 world validation")
    needle = '                raise ValueError(f"{name} must be a positive integer")\n'
    start = text.find('"person_track_reacquire_max_age_ns"')
    idx = text.find(needle, start)
    if idx < 0:
        raise UpgradeError("l4 world consistency validation anchor missing")
    idx += len(needle)
    text = text[:idx] + '        if self.person_track_reacquire_max_age_ns <= self.person_track_max_age_ns:\n            raise ValueError("person_track_reacquire_max_age_ns must exceed active track max age")\n' + text[idx:]
    text = replace_once(text, "            max_speed_mps=config.person_track_max_speed_mps,\n        )", "            max_speed_mps=config.person_track_max_speed_mps,\n            person_reacquire_max_age_ns=config.person_track_reacquire_max_age_ns,\n        )", "l4 world store construction")
    return f"# {MARKER}\n" + text


def patch_l6(text: str) -> str:
    rel = "v3/layers/l6_navigation.py"
    tree = parse_python(text, rel)
    states = [n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "_FollowPersonState"]
    if len(states) != 1:
        raise UpgradeError("l6: _FollowPersonState not found")
    node = states[0]
    lines = text.splitlines(keepends=True)
    text = "".join(lines[:node.end_lineno]) + "\n" + FOLLOW_EVIDENCE.strip() + "\n\n" + "".join(lines[node.end_lineno:])
    text = replace_once(text, "    follow_person_search_step_ns: int = 400_000_000\n", "    # Compatibility field only; search advancement is now yaw-completion-driven.\n    follow_person_search_step_ns: int = 400_000_000\n    follow_person_search_yaw_tolerance_rad: float = 0.08\n", "l6 search tolerance")
    text = replace_once(text, '            raise ValueError("follow_person_search_sweep_rad must be in (0, pi)")\n', '            raise ValueError("follow_person_search_sweep_rad must be in (0, pi)")\n        if (\n            not isinstance(self.follow_person_search_yaw_tolerance_rad, (int, float))\n            or isinstance(self.follow_person_search_yaw_tolerance_rad, bool)\n            or not math.isfinite(self.follow_person_search_yaw_tolerance_rad)\n            or not 0.0 < self.follow_person_search_yaw_tolerance_rad < math.pi\n        ):\n            raise ValueError("follow_person_search_yaw_tolerance_rad must be in (0, pi)")\n', "l6 tolerance validation")
    text = replace_once(text, "    follow_person_state: str = _FollowPersonState.ACQUIRE.value\n", "    follow_person_state: str = _FollowPersonState.ACQUIRE.value\n    follow_person_search_phase: int | None = None\n    follow_person_search_target_yaw_rad: float | None = None\n", "l6 checkpoint fields")
    text = replace_once(text, '        "_follow_person_state",\n', '        "_follow_person_state",\n        "_follow_person_search_phase",\n        "_follow_person_search_target_yaw_rad",\n        "_follow_person_evidence",\n', "l6 slots")
    text = replace_once(
        text,
        "        self._follow_person_last_heading_rad: float | None = None\n        self._follow_person_state = _FollowPersonState.ACQUIRE\n\n    def checkpoint(self) -> NavigationStateCheckpoint:\n",
        "        self._follow_person_last_heading_rad: float | None = None\n        self._follow_person_state = _FollowPersonState.ACQUIRE\n        self._follow_person_search_phase: int | None = None\n        self._follow_person_search_target_yaw_rad: float | None = None\n        self._follow_person_evidence: FollowPersonEvidence | None = None\n\n    def checkpoint(self) -> NavigationStateCheckpoint:\n",
        "l6 init",
    )
    text = replace_once(text, "            self._follow_person_state.value,\n        )", "            self._follow_person_state.value,\n            self._follow_person_search_phase,\n            self._follow_person_search_target_yaw_rad,\n        )", "l6 checkpoint emit")
    text = replace_once(text, "        self._follow_person_state = _FollowPersonState(checkpoint.follow_person_state)\n", "        self._follow_person_state = _FollowPersonState(checkpoint.follow_person_state)\n        self._follow_person_search_phase = checkpoint.follow_person_search_phase\n        self._follow_person_search_target_yaw_rad = checkpoint.follow_person_search_target_yaw_rad\n        self._follow_person_evidence = None\n", "l6 restore")
    text = replace_once(text, "        self._closed_completion = planner_input\n", "        self._closed_completion = planner_input\n        self._follow_person_evidence = None\n", "l6 per-tick evidence reset")
    text = replace_once(text, "        self._follow_person_last_heading_rad = None\n        self._follow_person_state = _FollowPersonState.ACQUIRE\n\n    def _face_person_plan(", "        self._follow_person_last_heading_rad = None\n        self._follow_person_state = _FollowPersonState.ACQUIRE\n        self._follow_person_search_phase = None\n        self._follow_person_search_target_yaw_rad = None\n        self._follow_person_evidence = None\n\n    def _face_person_plan(", "l6 reset")
    signature = "    def _follow_person_plan(\n        self,\n        mission: MissionIntent,\n        estimate: RobotEstimate,\n        world: WorldSnapshot,\n    ) -> NavigationPlan:\n"
    text = replace_once(text, signature, FOLLOW_WRAPPER + "    def _follow_person_plan_impl(\n        self,\n        mission: MissionIntent,\n        estimate: RobotEstimate,\n        world: WorldSnapshot,\n    ) -> NavigationPlan:\n", "l6 follow wrapper")
    text = replace_once(text, "            self._follow_person_track_id = selected.track_id\n            self._follow_person_lost_since_ns = None\n            self._follow_person_pivoting = False\n", "            self._follow_person_track_id = selected.track_id\n            self._follow_person_lost_since_ns = None\n            self._follow_person_search_phase = None\n            self._follow_person_search_target_yaw_rad = None\n            self._follow_person_pivoting = False\n", "l6 acquire clears search")
    text = replace_once(text, "                if self._follow_person_lost_since_ns is None:\n                    self._follow_person_lost_since_ns = mission.context.monotonic_ns\n                    self._clear_trajectory_plan()\n", "                if self._follow_person_lost_since_ns is None:\n                    self._follow_person_lost_since_ns = mission.context.monotonic_ns\n                    self._follow_person_search_phase = None\n                    self._follow_person_search_target_yaw_rad = None\n                    self._clear_trajectory_plan()\n", "l6 loss starts search")
    old_search = '''                search_elapsed_ns = lost_ns - self._config.follow_person_lost_hold_ns
                if (
                    self._follow_person_last_heading_rad is not None
                    and search_elapsed_ns <= self._config.follow_person_search_timeout_ns
                ):
                    # Keep the locked UID and search only by rotation. The target
                    # heading sequence is deterministic and bounded around the
                    # last observation; translation remains forbidden.
                    self._follow_person_state = _FollowPersonState.SEARCH
                    search_step = (
                        search_elapsed_ns // self._config.follow_person_search_step_ns
                    )
                    if search_step == 0:
                        search_offset_rad = 0.0
                    elif search_step % 2:
                        search_offset_rad = self._config.follow_person_search_sweep_rad
                    else:
                        search_offset_rad = -self._config.follow_person_search_sweep_rad
                    search_yaw = _wrapped_angle(
                        self._follow_person_last_heading_rad + search_offset_rad
                    )
                    return NavigationPlan(
                        context=mission.context,
                        mission_id=mission.mission_id,
                        route=(Waypoint(estimate.x_m, estimate.y_m, search_yaw),),
                        velocity_target=None,
                        constraints=mission.constraints,
                        corridor_radius_m=0.0,
                        progress=0.0,
                        status=NavigationStatus.ACTIVE,
                    )

                self._follow_person_state = _FollowPersonState.LOST
'''
    new_search = '''                search_elapsed_ns = lost_ns - self._config.follow_person_lost_hold_ns
                if (
                    self._follow_person_last_heading_rad is not None
                    and search_elapsed_ns <= self._config.follow_person_search_timeout_ns
                ):
                    # Stable target yaw until the physical yaw reaches it. Overall
                    # search timeout remains the bounded failure exit.
                    self._follow_person_state = _FollowPersonState.SEARCH
                    if self._follow_person_search_phase is None:
                        self._follow_person_search_phase = 0
                        self._follow_person_search_target_yaw_rad = self._follow_person_last_heading_rad
                    assert self._follow_person_search_target_yaw_rad is not None
                    search_yaw = self._follow_person_search_target_yaw_rad
                    if abs(_wrapped_angle(search_yaw - estimate.yaw_rad)) <= self._config.follow_person_search_yaw_tolerance_rad:
                        next_phase = self._follow_person_search_phase + 1
                        search_offset_rad = self._config.follow_person_search_sweep_rad if next_phase % 2 else -self._config.follow_person_search_sweep_rad
                        self._follow_person_search_phase = next_phase
                        search_yaw = _wrapped_angle(self._follow_person_last_heading_rad + search_offset_rad)
                        self._follow_person_search_target_yaw_rad = search_yaw
                    return NavigationPlan(
                        context=mission.context,
                        mission_id=mission.mission_id,
                        route=(Waypoint(estimate.x_m, estimate.y_m, search_yaw),),
                        velocity_target=None,
                        constraints=mission.constraints,
                        corridor_radius_m=0.0,
                        progress=0.0,
                        status=NavigationStatus.ACTIVE,
                    )

                self._follow_person_state = _FollowPersonState.LOST
                self._follow_person_search_target_yaw_rad = None
'''
    text = replace_once(text, old_search, new_search, "l6 completion-driven search")
    text = replace_once(text, "        assert selected is not None\n        self._follow_person_lost_since_ns = None\n        self._follow_person_state = _FollowPersonState.FOLLOW\n", "        assert selected is not None\n        self._follow_person_lost_since_ns = None\n        self._follow_person_search_phase = None\n        self._follow_person_search_target_yaw_rad = None\n        self._follow_person_state = _FollowPersonState.FOLLOW\n", "l6 reacquire clears search")
    return f"# {MARKER}\n" + text


def patch_native_control(text: str) -> str:
    text = replace_once(text, "        person_track_max_age_ns=_positive_int(\n            person_tracking.get(\"track_max_age_ns\", 500_000_000),\n            \"v3_navigation.person_tracking.track_max_age_ns\",\n        ),\n", "        person_track_max_age_ns=_positive_int(\n            person_tracking.get(\"track_max_age_ns\", 500_000_000),\n            \"v3_navigation.person_tracking.track_max_age_ns\",\n        ),\n        person_track_reacquire_max_age_ns=_positive_int(\n            person_tracking.get(\"reacquire_max_age_ns\", 2_500_000_000),\n            \"v3_navigation.person_tracking.reacquire_max_age_ns\",\n        ),\n", "native L4 config")
    text = replace_once(text, "        follow_person_search_step_ns=_positive_int(\n            follow_person.get(\"search_step_ns\", 400_000_000),\n            \"v3_navigation.follow_person.search_step_ns\",\n        ),\n", "        follow_person_search_step_ns=_positive_int(\n            follow_person.get(\"search_step_ns\", 400_000_000),\n            \"v3_navigation.follow_person.search_step_ns\",\n        ),\n        follow_person_search_yaw_tolerance_rad=_positive_float(\n            follow_person.get(\"search_yaw_tolerance_rad\", 0.08),\n            \"v3_navigation.follow_person.search_yaw_tolerance_rad\",\n        ),\n", "native L6 config")
    text = replace_once(text, "        return self._estimator.last_update_evidence + self._engine.fault_evidence\n", "        follow = self._navigation.follow_person_evidence\n        follow_evidence = () if follow is None else (follow,)\n        return self._estimator.last_update_evidence + self._engine.fault_evidence + follow_evidence\n", "native follow evidence")
    return f"# {MARKER}\n" + text


def patch_test_hub(text: str) -> str:
    old_nav = '''def _navigation_configuration(reader: object) -> tuple[Mapping[str, object], str]:
    configuration = _runtime_configuration(reader)
    resolved = _map(configuration.get("resolved_control"))
    if resolved:
        nav = _map(resolved.get("v3_navigation"))
        if nav:
            return nav, "CAPTURE_RUNTIME.resolved_control.v3_navigation"
    nav = _map(configuration.get("v3_navigation"))
    if nav:
        return nav, "CAPTURE_RUNTIME.v3_navigation"
    return {}, "UNAVAILABLE"
'''
    new_nav = '''def _navigation_configuration(reader: object) -> tuple[Mapping[str, object], str]:
    configuration = _runtime_configuration(reader)
    resolved = _map(configuration.get("resolved_control"))
    if resolved:
        nav = _map(resolved.get("v3_navigation"))
        if nav:
            return nav, "CAPTURE_RUNTIME.resolved_control.v3_navigation"
    nav = _map(configuration.get("v3_navigation"))
    if nav:
        return nav, "CAPTURE_RUNTIME.v3_navigation"

    # Production already captures the exact resolved runtime dataclasses.
    runtime = _map(configuration.get("resolved_runtime"))
    composition = _map(runtime.get("composition"))
    live_control = _map(composition.get("live_control"))
    control = _map(live_control.get("control"))
    navigation = _map(control.get("navigation"))
    world = _map(control.get("world_model"))
    if navigation:
        return {
            "follow_person": {
                "minimum_confidence": navigation.get("follow_person_min_confidence"),
                "retention_minimum_confidence": navigation.get("follow_person_retention_min_confidence"),
                "align_tolerance_rad": navigation.get("follow_person_align_tolerance_rad"),
                "release_tolerance_rad": navigation.get("follow_person_release_tolerance_rad"),
                "stand_off_m": navigation.get("follow_person_stand_off_m"),
                "distance_deadband_m": navigation.get("follow_person_distance_deadband_m"),
                "min_safe_distance_m": navigation.get("follow_person_min_safe_distance_m"),
                "lost_hold_ns": navigation.get("follow_person_lost_hold_ns"),
                "search_timeout_ns": navigation.get("follow_person_search_timeout_ns"),
                "search_sweep_rad": navigation.get("follow_person_search_sweep_rad"),
                "search_step_ns": navigation.get("follow_person_search_step_ns"),
                "search_yaw_tolerance_rad": navigation.get("follow_person_search_yaw_tolerance_rad"),
            },
            "person_tracking": {
                "track_max_age_ns": world.get("person_track_max_age_ns"),
                "reacquire_max_age_ns": world.get("person_track_reacquire_max_age_ns"),
                "max_association_distance_m": world.get("person_track_max_association_distance_m"),
                "max_speed_mps": world.get("person_track_max_speed_mps"),
            },
            "exploration": {
                "coverage_cell_size_m": navigation.get("coverage_cell_size_m"),
                "coverage_max_cells": navigation.get("coverage_max_cells"),
                "local_goal_max_age_ns": navigation.get("local_goal_max_age_ns"),
            },
        }, "CAPTURE_RUNTIME.resolved_runtime.composition.live_control.control"
    return {}, "UNAVAILABLE"
'''
    rel = "v3/test_hub_task_evidence.py"
    text = replace_once(text, old_nav, new_nav, "Test Hub resolved runtime config")
    # Patch the intended function structurally before helper insertion. This avoids
    # the old installer bug where DIRECT_FOLLOW_HELPERS introduced a second copy
    # of the generic `config = ...follow_person...` line and made the later anchor
    # ambiguous.
    text = insert_function_body_prefix(
        text,
        "_follow_person_evidence",
        '    if any(_captured_follow_evidence(tick) for tick in ticks):\n'
        '        return _follow_person_direct_evidence(episode, ticks, navigation_config)\n\n',
        rel,
    )
    text = insert_before_top_level_function(
        text,
        "_follow_person_evidence",
        DIRECT_FOLLOW_HELPERS,
        rel,
    )
    return f"# {MARKER}\n" + text


def patch_config(text: str) -> str:
    data = json.loads(text)
    data["v3_navigation"]["person_tracking"]["reacquire_max_age_ns"] = 2_500_000_000
    data["v3_navigation"]["follow_person"]["search_yaw_tolerance_rad"] = 0.08
    return json.dumps(data, ensure_ascii=False, indent=2) + "\n"


UPDATED_MODERNIZATION_TEST = r'''from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from v3.composition.native_control import v3_navigation_config_from_mapping
from v3.contracts import CommandMode, CommandRequest, DataField, MissionConstraints, MotionObjectiveKind, NavigationPlan, NavigationStatus, ObstacleTrack, RobotEstimate, RollingLocalCostmap, TickContext, TrajectoryEvaluation, TrajectoryPose, Waypoint, WorldSnapshot
from v3.layers.l4_temporal_tracking import PersonMeasurement, TemporalTrackStore
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import TrajectoryNavigator
from v3.layers.l7_motion_selection import MotionSelector

PROJECT_ROOT = Path(__file__).resolve().parents[1]

def _navigation_config():
    raw = json.loads((PROJECT_ROOT / "conf/vezerles.json").read_text(encoding="utf-8"))
    return v3_navigation_config_from_mapping(raw).navigation

def test_person_uid_reacquires_from_dormant_then_is_not_recycled_after_ttl():
    store = TemporalTrackStore(alpha=0.6, beta=0.2, prediction_max_age_ns=500_000_000, max_speed_mps=6.0, person_reacquire_max_age_ns=2_500_000_000)
    store.associate_people((PersonMeasurement(0.9, 1.0, 0.0),), captured_ns=1_000_000_000, radius_m=0.30, max_association_distance_m=0.75)
    assert store.projected_tracks(1_000_000_000)[0].track_id == "person-1"
    store.expire(1_600_000_000, person_max_age_ns=500_000_000, other_max_age_ns=1)
    assert store.projected_tracks(1_600_000_000) == ()
    store.associate_people((PersonMeasurement(0.9, 1.2, 0.0),), captured_ns=2_000_000_000, radius_m=0.30, max_association_distance_m=0.75)
    assert store.projected_tracks(2_000_000_000)[0].track_id == "person-1"
    checkpoint = store.checkpoint()
    restored = TemporalTrackStore(alpha=0.6, beta=0.2, prediction_max_age_ns=500_000_000, max_speed_mps=6.0, person_reacquire_max_age_ns=2_500_000_000)
    restored.restore(checkpoint)
    restored.expire(5_000_000_001, person_max_age_ns=1, other_max_age_ns=1)
    restored.associate_people((PersonMeasurement(0.9, 1.4, 0.0),), captured_ns=5_000_000_002, radius_m=0.30, max_association_distance_m=0.75)
    assert restored.projected_tracks(5_000_000_002)[0].track_id == "person-2"

def _estimate(context: TickContext, yaw_rad: float = 0.0) -> RobotEstimate:
    covariance = tuple(0.01 if index % 6 == 0 else 0.0 for index in range(25))
    return RobotEstimate(context=context, frame_id="R2B4_BOOT_ROBOT_MAP", x_m=0.0, y_m=0.0, yaw_rad=yaw_rad, v_mps=0.0, omega_rad_s=0.0, covariance_5x5=covariance)

def _person(track_id: str, x: float = 2.0, y: float = 0.0) -> ObstacleTrack:
    return ObstacleTrack(track_id=track_id, x_m=x, y_m=y, radius_m=0.30, vx_mps=0.0, vy_mps=0.0, confidence=0.9)

def _world(context: TickContext, *tracks: ObstacleTrack) -> WorldSnapshot:
    costmap = RollingLocalCostmap(frame_id="R2B4_BOOT_ROBOT_MAP", revision=1, resolution_m=0.1, radius_m=2.5, occupied_cells=(), source_sequence=1, freshness_ns=0)
    return WorldSnapshot(context=context, frame_id="R2B4_BOOT_ROBOT_MAP", map_revision=1, obstacle_tracks=tuple(tracks), freshness_ns=0, local_costmap=costmap)

def _mission(context: TickContext):
    return MissionManager().evaluate(CommandRequest(context=context, command_id="modern-follow", mode=CommandMode.FOLLOW_PERSON, goal=(DataField("max_v_mps", 0.15), DataField("max_omega_rad_s", 0.30)), expiry_tick=context.tick_id))

def test_follow_search_target_is_completion_driven_not_timer_flipped():
    config = _navigation_config(); nav = TrajectoryNavigator(config)
    c1 = TickContext(1, 1_000_000_000); nav.evaluate(_mission(c1), _estimate(c1), _world(c1, _person("person-1")))
    c2 = TickContext(2, 1_020_000_000); nav.evaluate(_mission(c2), _estimate(c2), _world(c2, _person("person-2")))
    c3 = TickContext(3, c2.monotonic_ns + config.follow_person_lost_hold_ns + 1)
    right = nav.evaluate(_mission(c3), _estimate(c3, 0.0), _world(c3, _person("person-2")))
    assert right.route[0].yaw_rad == pytest.approx(config.follow_person_search_sweep_rad)
    c4 = TickContext(4, c3.monotonic_ns + config.follow_person_search_step_ns + 100_000_000)
    same = nav.evaluate(_mission(c4), _estimate(c4, 0.1), _world(c4, _person("person-2")))
    assert same.route[0].yaw_rad == pytest.approx(config.follow_person_search_sweep_rad)
    c5 = TickContext(5, c4.monotonic_ns + 100_000_000)
    left = nav.evaluate(_mission(c5), _estimate(c5, config.follow_person_search_sweep_rad), _world(c5, _person("person-2")))
    assert left.route[0].yaw_rad == pytest.approx(-config.follow_person_search_sweep_rad)

def _candidate(candidate_id: str, score: float, omega: float) -> TrajectoryEvaluation:
    horizon_ns = 100_000_000
    return TrajectoryEvaluation(candidate_id=candidate_id, v_mps=0.12, omega_rad_s=omega, horizon_ns=horizon_ns, samples=(TrajectoryPose(0.01, 0.0, 0.0, horizon_ns),), collision=False, min_clearance_m=0.6, progress_score=0.5, smoothness_score=0.5, novelty_score=0.5, total_score=score)

def _plan(tick: int, candidates: tuple[TrajectoryEvaluation, ...]) -> NavigationPlan:
    context = TickContext(tick, 5_000_000_000 + tick * 20_000_000)
    return NavigationPlan(context=context, mission_id="follow-motion", route=(), velocity_target=None, constraints=MissionConstraints(0.15, 0.30, 0.30, 0.08, 0.10), corridor_radius_m=0.30, progress=0.0, status=NavigationStatus.ACTIVE, local_goal=Waypoint(1.0, 0.0), trajectory_candidates=candidates)

def test_l7_continuity_survives_candidate_id_change_and_avoids_sign_flip():
    selector = MotionSelector(); first = selector.evaluate(_plan(1, (_candidate("old-grid", 1.0, -0.20), _candidate("other", 0.9, 0.20))))
    assert first.kind is MotionObjectiveKind.TRACK_TRAJECTORY
    second = selector.evaluate(_plan(2, (_candidate("new-best", 1.000, 0.20), _candidate("new-continuous", 0.997, -0.18))))
    assert second.trajectory is not None and second.trajectory.candidate_id == "new-continuous"

def test_follow_search_config_is_explicit_and_bounded():
    config = _navigation_config()
    assert config.follow_person_search_timeout_ns > 0
    assert config.follow_person_search_step_ns > 0
    assert 0.0 < config.follow_person_search_sweep_rad < math.pi
    assert 0.0 < config.follow_person_search_yaw_tolerance_rad < math.pi
'''


NEW_TEST = r'''from __future__ import annotations

import json
from pathlib import Path

from v3.composition.native_control import v3_navigation_config_from_mapping
from v3.contracts import CommandMode, CommandRequest, DataField, ObstacleTrack, RobotEstimate, RollingLocalCostmap, TickContext, WorldSnapshot
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import FollowPersonEvidence, TrajectoryNavigator
from v3.test_hub_task_evidence import _navigation_configuration

ROOT = Path(__file__).resolve().parents[1]

def _config():
    raw = json.loads((ROOT / "conf/vezerles.json").read_text(encoding="utf-8"))
    return v3_navigation_config_from_mapping(raw)

def _estimate(c: TickContext) -> RobotEstimate:
    covariance = tuple(0.01 if i % 6 == 0 else 0.0 for i in range(25))
    return RobotEstimate(c, "R2B4_BOOT_ROBOT_MAP", 0.0, 0.0, 0.0, 0.0, 0.0, covariance)

def _person(uid: str) -> ObstacleTrack:
    return ObstacleTrack(uid, 2.0, 0.0, 0.3, 0.0, 0.0, 0.9)

def _world(c: TickContext, *tracks: ObstacleTrack) -> WorldSnapshot:
    costmap = RollingLocalCostmap("R2B4_BOOT_ROBOT_MAP", 1, 0.1, 2.5, (), 1, 0)
    return WorldSnapshot(c, "R2B4_BOOT_ROBOT_MAP", 1, tuple(tracks), 0, costmap)

def _mission(c: TickContext):
    return MissionManager().evaluate(CommandRequest(c, "p0-v2-follow", CommandMode.FOLLOW_PERSON, (DataField("max_v_mps", 0.15), DataField("max_omega_rad_s", 0.30)), c.tick_id))

def test_follow_evidence_exposes_locked_uid_and_config():
    config = _config().navigation; nav = TrajectoryNavigator(config); c = TickContext(1, 1_000_000_000)
    nav.evaluate(_mission(c), _estimate(c), _world(c, _person("person-7")))
    evidence = nav.follow_person_evidence
    assert isinstance(evidence, FollowPersonEvidence)
    assert evidence.state == "FOLLOW" and evidence.locked_target_uid == "person-7"
    assert evidence.target_visible is True
    assert evidence.search_yaw_tolerance_rad == config.follow_person_search_yaw_tolerance_rad

class _Reader:
    def __init__(self, configuration): self.configuration = configuration
    def first_json(self, _topic): return object(), {"configuration": self.configuration}

def test_testhub_reads_navigation_from_resolved_runtime():
    nav = {"follow_person_min_confidence": 0.6, "follow_person_retention_min_confidence": 0.45, "follow_person_align_tolerance_rad": 0.22, "follow_person_release_tolerance_rad": 0.30, "follow_person_stand_off_m": 1.05, "follow_person_distance_deadband_m": 0.15, "follow_person_min_safe_distance_m": 0.75, "follow_person_lost_hold_ns": 400_000_000, "follow_person_search_timeout_ns": 2_000_000_000, "follow_person_search_sweep_rad": 0.45, "follow_person_search_step_ns": 400_000_000, "follow_person_search_yaw_tolerance_rad": 0.08, "coverage_cell_size_m": 0.25, "coverage_max_cells": 512, "local_goal_max_age_ns": 8_000_000_000}
    world = {"person_track_max_age_ns": 500_000_000, "person_track_reacquire_max_age_ns": 2_500_000_000, "person_track_max_association_distance_m": 0.75, "person_track_max_speed_mps": 6.0}
    configuration = {"resolved_runtime": {"composition": {"live_control": {"control": {"navigation": nav, "world_model": world}}}}}
    resolved, source = _navigation_configuration(_Reader(configuration))
    assert source.startswith("CAPTURE_RUNTIME.resolved_runtime")
    assert resolved["follow_person"]["minimum_confidence"] == 0.6
    assert resolved["person_tracking"]["reacquire_max_age_ns"] == 2_500_000_000
'''


def build_patches(root: Path) -> dict[str, str]:
    src = {rel: read_text(root, rel) for rel in EXPECTED_GIT_BLOB_SHA1}
    patched = {
        "v3/layers/l4_temporal_tracking.py": patch_l4_temporal(src["v3/layers/l4_temporal_tracking.py"]),
        "v3/layers/l4_world_model.py": patch_l4_world(src["v3/layers/l4_world_model.py"]),
        "v3/layers/l6_navigation.py": patch_l6(src["v3/layers/l6_navigation.py"]),
        "v3/composition/native_control.py": patch_native_control(src["v3/composition/native_control.py"]),
        "v3/test_hub_task_evidence.py": patch_test_hub(src["v3/test_hub_task_evidence.py"]),
        "conf/vezerles.json": patch_config(src["conf/vezerles.json"]),
        "tests/test_v3_follow_person_modernization.py": f"# {MARKER}\n" + UPDATED_MODERNIZATION_TEST,
        "tests/test_v3_follow_person_p0_v2.py": f"# {MARKER}\n" + NEW_TEST,
    }
    for rel, content in patched.items():
        if rel.endswith(".py"):
            parse_python(content, rel)
        elif rel.endswith(".json"):
            json.loads(content)
    return patched


def install_state(root: Path) -> str:
    checks = (
        ("v3/layers/l4_temporal_tracking.py", MARKER),
        ("v3/layers/l4_world_model.py", MARKER),
        ("v3/layers/l6_navigation.py", MARKER),
        ("v3/composition/native_control.py", MARKER),
        ("v3/test_hub_task_evidence.py", MARKER),
        ("tests/test_v3_follow_person_modernization.py", MARKER),
        ("tests/test_v3_follow_person_p0_v2.py", MARKER),
    )
    present = []
    for rel, token in checks:
        path = root / rel
        present.append(path.is_file() and token in path.read_text(encoding="utf-8"))
    config_ok = False
    config_path = root / "conf/vezerles.json"
    if config_path.is_file():
        try:
            data = json.loads(config_path.read_text(encoding="utf-8"))
            tracking = data["v3_navigation"]["person_tracking"]
            follow = data["v3_navigation"]["follow_person"]
            config_ok = (
                tracking.get("reacquire_max_age_ns") == 2_500_000_000
                and follow.get("search_yaw_tolerance_rad") == 0.08
            )
        except (KeyError, TypeError, json.JSONDecodeError):
            config_ok = False
    present.append(config_ok)
    if all(present):
        return "installed"
    if any(present):
        return "partial"
    return "baseline"


def atomic_write(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(content); handle.flush(); os.fsync(handle.fileno())
        os.replace(tmp_name, path)
    finally:
        if os.path.exists(tmp_name): os.unlink(tmp_name)


def backup_files(root: Path, patched: dict[str, str]) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = root / "runtime" / "upgrade_backups" / f"{UPGRADE_ID}_{stamp}"
    backup.mkdir(parents=True, exist_ok=False)
    for rel in patched:
        src = root / rel
        if src.exists():
            dst = backup / rel; dst.parent.mkdir(parents=True, exist_ok=True); shutil.copy2(src, dst)
    return backup


def rollback(root: Path, backup: Path, patched: dict[str, str]) -> None:
    for rel in patched:
        dst = root / rel; src = backup / rel
        if src.exists(): dst.parent.mkdir(parents=True, exist_ok=True); shutil.copy2(src, dst)
        elif dst.exists(): dst.unlink()


def compile_files(root: Path, patched: dict[str, str]) -> None:
    for rel in patched:
        if rel.endswith(".py"): py_compile.compile(str(root / rel), doraise=True)


def run_tests(root: Path) -> None:
    candidates = ["tests/test_v3_follow_person_p0_v2.py", "tests/test_v3_follow_person_modernization.py", "tests/test_v3_follow_person.py", "tests/test_v3_follow_person_p0.py", "tests/test_v3_test_hub_task_evidence.py"]
    tests = [name for name in candidates if (root / name).is_file()]
    if not tests: raise UpgradeError("no focused acceptance tests found")
    subprocess.run([sys.executable, "-m", "pytest", "-q", *tests], cwd=root, check=True)


def write_manifest(backup: Path, before: dict[str, str], patched: dict[str, str]) -> None:
    payload = {"upgrade_id": UPGRADE_ID, "source_first_base_head": BASE_HEAD, "created_at": datetime.now().isoformat(), "files": {rel: {"before_sha256": before.get(rel), "after_sha256": sha256_text(content)} for rel, content in patched.items()}}
    (backup / "MANIFEST.json").write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Install R2B4 FOLLOW_PERSON P0 v2")
    parser.add_argument("--root", default="/home/alba/project_r2b4")
    parser.add_argument("--check", action="store_true", help="validate baseline and build all patches in memory only")
    parser.add_argument("--no-tests", action="store_true", help="compile only; skip focused pytest")
    args = parser.parse_args()
    root = Path(args.root).resolve()
    if not (root / "v3").is_dir() or not (root / "conf").is_dir(): raise UpgradeError(f"not an R2B4 repo root: {root}")
    state = install_state(root)
    if state == "installed":
        print(f"{UPGRADE_ID}: already installed; no changes")
        return 0
    if state == "partial":
        raise UpgradeError(
            "partial installation detected; no files were changed. "
            "Restore the previous runtime/upgrade_backups copy or inspect the partial state before retrying."
        )
    before = semantic_preflight(root)
    patched = build_patches(root)
    if args.check: print(f"{UPGRADE_ID}: CHECK OK; {len(patched)} files would be installed"); return 0
    backup = backup_files(root, patched)
    try:
        for rel, content in patched.items(): atomic_write(root / rel, content)
        compile_files(root, patched)
        if not args.no_tests: run_tests(root)
        write_manifest(backup, before, patched)
    except BaseException:
        rollback(root, backup, patched)
        raise
    print(f"{UPGRADE_ID}: INSTALLED")
    print(f"backup: {backup}")
    print("acceptance: compile PASS" + ("; pytest SKIPPED" if args.no_tests else "; focused pytest PASS"))
    return 0


if __name__ == "__main__":
    try: raise SystemExit(main())
    except (UpgradeError, subprocess.CalledProcessError, py_compile.PyCompileError) as exc:
        print(f"INSTALL FAILED: {exc}", file=sys.stderr); raise SystemExit(1)
