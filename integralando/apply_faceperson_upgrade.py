#!/usr/bin/env python3
# R2B4 FACE_PERSON upgrade.
# Preserves the 2026-09-17 morning L6 P0-A semantics exactly:
# - production monotonic late result -> keep still-fresh cached plan
# - legacy/tick-gated handoff -> hard ASYNC_L6_DEADLINE_MISSED
# - cached plan older than max_plan_age_ns -> ASYNC_L6_PLAN_STALE

from __future__ import annotations

import argparse
import ast
import json
import py_compile
import shutil
import time
from pathlib import Path


class UpgradeError(RuntimeError):
    pass


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise UpgradeError(f"{label}: expected exactly one source anchor, found {count}")
    return text.replace(old, new, 1)


def ensure_json(path: Path, text: str) -> None:
    try:
        json.loads(text)
    except Exception as exc:
        raise UpgradeError(f"{path}: invalid JSON after transform: {exc}") from exc


def ensure_python(path: Path, text: str) -> None:
    try:
        ast.parse(text, filename=str(path))
    except SyntaxError as exc:
        raise UpgradeError(f"{path}: invalid Python after transform: {exc}") from exc


def transform_messages(text: str) -> str:
    if 'FACE_PERSON = "FACE_PERSON"' not in text:
        text = replace_once(
            text,
            '    EXPLORE = "EXPLORE"\n    SERVICE = "SERVICE"\n',
            '    EXPLORE = "EXPLORE"\n    FACE_PERSON = "FACE_PERSON"\n    SERVICE = "SERVICE"\n',
            "messages.CommandMode",
        )
    mission_section = text.split("class MissionIntent", 1)[1]
    if "CommandMode.FACE_PERSON" not in mission_section:
        text = replace_once(
            text,
            '''            if self.mode is CommandMode.EXPLORE and (
                self.target_pose is not None or self.velocity_target is not None
            ):
                raise ContractValidationError("active EXPLORE mission cannot carry a fixed target")
''',
            '''            if self.mode in (CommandMode.EXPLORE, CommandMode.FACE_PERSON) and (
                self.target_pose is not None or self.velocity_target is not None
            ):
                raise ContractValidationError(
                    f"active {self.mode.value} mission cannot carry a fixed target"
                )
''',
            "messages.MissionIntent",
        )
    return text


def transform_l5(text: str) -> str:
    if "elif command.mode is CommandMode.FACE_PERSON:" not in text:
        text = replace_once(
            text,
            '''            elif command.mode is CommandMode.EXPLORE:
                _require_keys(
                    values,
                    required=frozenset(),
                    optional=frozenset({"max_v_mps", "max_omega_rad_s"}),
                )
                target_pose = None
                velocity_target = None
            else:
''',
            '''            elif command.mode is CommandMode.EXPLORE:
                _require_keys(
                    values,
                    required=frozenset(),
                    optional=frozenset({"max_v_mps", "max_omega_rad_s"}),
                )
                target_pose = None
                velocity_target = None
            elif command.mode is CommandMode.FACE_PERSON:
                _require_keys(
                    values,
                    required=frozenset(),
                    optional=frozenset({"max_omega_rad_s"}),
                )
                target_pose = None
                velocity_target = None
            else:
''',
            "l5 FACE_PERSON",
        )
    return text


def transform_resident_command(text: str) -> str:
    if "CommandMode.FACE_PERSON" in text:
        return text

    old_modes = "(CommandMode.STOP, CommandMode.TELEOP, CommandMode.EXPLORE)"
    new_modes = "(CommandMode.STOP, CommandMode.TELEOP, CommandMode.EXPLORE, CommandMode.FACE_PERSON)"
    if text.count(old_modes) < 2:
        raise UpgradeError("resident_command: mode whitelist anchors missing")
    text = text.replace(old_modes, new_modes)
    # Keep the old substring so existing regex-based tests still match.
    text = text.replace(
        '"command mode must be STOP, TELEOP or EXPLORE"',
        '"command mode must be STOP, TELEOP or EXPLORE, or FACE_PERSON"',
    )
    text = text.replace(
        '"client mode must be STOP, TELEOP or EXPLORE"',
        '"client mode must be STOP, TELEOP or EXPLORE, or FACE_PERSON"',
    )

    text = replace_once(
        text,
        '''        expected_keys = common_keys | (
            motion_keys
            if mode is CommandMode.TELEOP
            else limit_keys if mode is CommandMode.EXPLORE else set()
        )
''',
        '''        expected_keys = common_keys | (
            motion_keys
            if mode is CommandMode.TELEOP
            else limit_keys
            if mode is CommandMode.EXPLORE
            else {"max_omega_rad_s"}
            if mode is CommandMode.FACE_PERSON
            else set()
        )
''',
        "resident expected keys",
    )

    text = replace_once(
        text,
        '''        max_v_mps = _finite(payload.get("max_v_mps"), "max_v_mps")
        max_omega_rad_s = _finite(payload.get("max_omega_rad_s"), "max_omega_rad_s")
        if not 0.0 < max_v_mps <= self._config.maximum_linear_speed_mps:
            raise ValueError("max_v_mps exceeds the resident process limit")
        if not 0.0 < max_omega_rad_s <= self._config.maximum_angular_speed_rad_s:
            raise ValueError("max_omega_rad_s exceeds the resident process limit")
        if mode is CommandMode.EXPLORE:
''',
        '''        max_omega_rad_s = _finite(payload.get("max_omega_rad_s"), "max_omega_rad_s")
        if not 0.0 < max_omega_rad_s <= self._config.maximum_angular_speed_rad_s:
            raise ValueError("max_omega_rad_s exceeds the resident process limit")
        if mode is CommandMode.FACE_PERSON:
            return CommandRequest(
                context=context,
                command_id=command_id,
                mode=CommandMode.FACE_PERSON,
                goal=(DataField("max_omega_rad_s", max_omega_rad_s),),
                expiry_tick=context.tick_id,
            )

        max_v_mps = _finite(payload.get("max_v_mps"), "max_v_mps")
        if not 0.0 < max_v_mps <= self._config.maximum_linear_speed_mps:
            raise ValueError("max_v_mps exceeds the resident process limit")
        if mode is CommandMode.EXPLORE:
''',
        "resident FACE_PERSON decode",
    )

    text = replace_once(
        text,
        '''    def _publish(
        self,
        command_id: str,
        mode: CommandMode,
''',
        '''    def publish_face_person(
        self,
        command_id: str,
        *,
        max_omega_rad_s: float,
        ttl_ns: int,
    ) -> int:
        return self._publish(
            command_id,
            CommandMode.FACE_PERSON,
            {"max_omega_rad_s": max_omega_rad_s},
            ttl_ns,
        )

    def _publish(
        self,
        command_id: str,
        mode: CommandMode,
''',
        "resident publish_face_person",
    )

    text = replace_once(
        text,
        '''        expected = (
            {"v_mps", "omega_rad_s", "max_v_mps", "max_omega_rad_s"}
            if mode is CommandMode.TELEOP
            else {"max_v_mps", "max_omega_rad_s"}
            if mode is CommandMode.EXPLORE
            else set()
        )
''',
        '''        expected = (
            {"v_mps", "omega_rad_s", "max_v_mps", "max_omega_rad_s"}
            if mode is CommandMode.TELEOP
            else {"max_v_mps", "max_omega_rad_s"}
            if mode is CommandMode.EXPLORE
            else {"max_omega_rad_s"}
            if mode is CommandMode.FACE_PERSON
            else set()
        )
''',
        "resident client expected keys",
    )

    text = replace_once(
        text,
        '''        if mode is CommandMode.STOP:
            return normalized
        max_v_mps = normalized["max_v_mps"]
        max_omega_rad_s = normalized["max_omega_rad_s"]
        if not 0.0 < max_v_mps <= self._config.maximum_linear_speed_mps:
            raise ValueError("max_v_mps exceeds the resident process limit")
        if not 0.0 < max_omega_rad_s <= self._config.maximum_angular_speed_rad_s:
            raise ValueError("max_omega_rad_s exceeds the resident process limit")
''',
        '''        if mode is CommandMode.STOP:
            return normalized
        max_omega_rad_s = normalized["max_omega_rad_s"]
        if not 0.0 < max_omega_rad_s <= self._config.maximum_angular_speed_rad_s:
            raise ValueError("max_omega_rad_s exceeds the resident process limit")
        if mode is CommandMode.FACE_PERSON:
            return normalized
        max_v_mps = normalized["max_v_mps"]
        if not 0.0 < max_v_mps <= self._config.maximum_linear_speed_mps:
            raise ValueError("max_v_mps exceeds the resident process limit")
''',
        "resident client FACE_PERSON validation",
    )
    return text


def transform_control_cli(text: str) -> str:
    if 'subcommands.add_parser("faceperson"' in text:
        return text
    text = replace_once(
        text,
        '''    explore.add_argument("--max-v-mps", type=float, default=0.30)
    explore.add_argument("--max-omega-rad-s", type=float, default=0.60)
    return parser
''',
        '''    explore.add_argument("--max-v-mps", type=float, default=0.30)
    explore.add_argument("--max-omega-rad-s", type=float, default=0.60)

    faceperson = subcommands.add_parser(
        "faceperson",
        help="heartbeat FACE_PERSON: rotate in place toward the selected person",
    )
    faceperson.add_argument("--command-id")
    faceperson.add_argument("--max-omega-rad-s", type=float, default=0.50)
    return parser
''',
        "control_cli parser",
    )
    text = replace_once(
        text,
        '''        if args.operation == "teleop":
            publish = lambda logical_id: client.publish_teleop(
                logical_id,
                v_mps=args.v_mps,
                omega_rad_s=args.omega_rad_s,
                max_v_mps=args.max_v_mps,
                max_omega_rad_s=args.max_omega_rad_s,
                ttl_ns=ttl_ns,
            )
        else:
            publish = lambda logical_id: client.publish_explore(
                logical_id,
                max_v_mps=args.max_v_mps,
                max_omega_rad_s=args.max_omega_rad_s,
                ttl_ns=ttl_ns,
            )
''',
        '''        if args.operation == "teleop":
            publish = lambda logical_id: client.publish_teleop(
                logical_id,
                v_mps=args.v_mps,
                omega_rad_s=args.omega_rad_s,
                max_v_mps=args.max_v_mps,
                max_omega_rad_s=args.max_omega_rad_s,
                ttl_ns=ttl_ns,
            )
        elif args.operation == "explore":
            publish = lambda logical_id: client.publish_explore(
                logical_id,
                max_v_mps=args.max_v_mps,
                max_omega_rad_s=args.max_omega_rad_s,
                ttl_ns=ttl_ns,
            )
        else:
            publish = lambda logical_id: client.publish_face_person(
                logical_id,
                max_omega_rad_s=args.max_omega_rad_s,
                ttl_ns=ttl_ns,
            )
''',
        "control_cli publish branch",
    )
    return text


def transform_l4(text: str) -> str:
    if "person_track_max_age_ns" in text:
        return text
    text = replace_once(
        text,
        '''    person_track_max_association_distance_m: float = 0.75
    person_track_max_speed_mps: float = 6.0
    person_track_radius_m: float = 0.30
''',
        '''    person_track_max_association_distance_m: float = 0.75
    person_track_max_speed_mps: float = 6.0
    person_track_radius_m: float = 0.30
    person_track_max_age_ns: int = 500_000_000
''',
        "L4 person max age field",
    )
    text = replace_once(
        text,
        '''            "person_lidar_max_skew_ns",
        ):
''',
        '''            "person_lidar_max_skew_ns",
            "person_track_max_age_ns",
        ):
''',
        "L4 person max age validation",
    )
    text = replace_once(
        text,
        '''        expired = tuple(
            track_id
            for track_id, (_, captured_ns) in self._tracks.items()
            if frame.context.monotonic_ns - captured_ns > self._config.max_track_age_ns
        )
''',
        '''        expired = tuple(
            track_id
            for track_id, (_, captured_ns) in self._tracks.items()
            if frame.context.monotonic_ns - captured_ns
            > (
                self._config.person_track_max_age_ns
                if track_id.startswith("person-")
                else self._config.max_track_age_ns
            )
        )
''',
        "L4 person expiration",
    )
    return text


def ensure_p0a_l6(text: str) -> str:
    exact_body = '''    def _require_fresh_cached_plan(self, context: TickContext) -> None:
        last_replan_ns = self._last_replan_ns
        if last_replan_ns is None or not self._trajectory_candidates:
            raise RuntimeError("async rollout has no authoritative cached plan")
        if context.monotonic_ns - last_replan_ns > self._max_plan_age_ns:
            raise RuntimeError("ASYNC_L6_PLAN_STALE")
'''
    exact_take = '''        result = backend.take(request_id)
        if result is None:
            if release_not_before_ns is None:
                raise RuntimeError("ASYNC_L6_DEADLINE_MISSED")
            self._require_fresh_cached_plan(context)
            return False
'''
    if "_require_fresh_cached_plan" in text:
        if exact_body not in text or exact_take not in text:
            raise UpgradeError(
                "L6 contains a different P0-A implementation; refusing to overwrite it"
            )
        return text
    text = replace_once(
        text,
        '''        result = backend.take(request_id)
        if result is None:
            raise RuntimeError("ASYNC_L6_DEADLINE_MISSED")
''',
        exact_take,
        "L6 P0-A handoff",
    )
    text = replace_once(
        text,
        '''    def _abandon_pending_rollout(self) -> None:
''',
        exact_body + "\n    def _abandon_pending_rollout(self) -> None:\n",
        "L6 P0-A freshness helper",
    )
    return text


def transform_l6(text: str) -> str:
    text = ensure_p0a_l6(text)
    if "face_person_min_confidence" in text:
        return text

    text = replace_once(
        text,
        '''    novelty_weight: float = 0.22

    def __post_init__(self) -> None:
''',
        '''    novelty_weight: float = 0.22
    face_person_min_confidence: float = 0.60
    face_person_align_tolerance_rad: float = 0.10
    face_person_release_tolerance_rad: float = 0.16

    def __post_init__(self) -> None:
''',
        "L6 face config fields",
    )
    text = replace_once(
        text,
        '''        if sum(weights) <= 0.0:
            raise ValueError("at least one trajectory score weight must be positive")
''',
        '''        if sum(weights) <= 0.0:
            raise ValueError("at least one trajectory score weight must be positive")
        if (
            not isinstance(self.face_person_min_confidence, (int, float))
            or isinstance(self.face_person_min_confidence, bool)
            or not math.isfinite(self.face_person_min_confidence)
            or not 0.0 <= self.face_person_min_confidence <= 1.0
        ):
            raise ValueError("face_person_min_confidence must be in [0, 1]")
        for name in (
            "face_person_align_tolerance_rad",
            "face_person_release_tolerance_rad",
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
        if self.face_person_release_tolerance_rad <= self.face_person_align_tolerance_rad:
            raise ValueError(
                "face_person_release_tolerance_rad must exceed align tolerance"
            )
''',
        "L6 face config validation",
    )
    text = replace_once(
        text,
        '''    pending_release_tick_id: int | None = None
    pending_release_not_before_ns: int | None = None
''',
        '''    pending_release_tick_id: int | None = None
    pending_release_not_before_ns: int | None = None
    face_person_track_id: str | None = None
    face_person_aligned: bool = False
''',
        "L6 checkpoint fields",
    )
    text = replace_once(
        text,
        '''        "_completed",
        "_config",
''',
        '''        "_completed",
        "_config",
        "_face_person_aligned",
        "_face_person_track_id",
''',
        "L6 slots",
    )
    text = replace_once(
        text,
        '''        self._trajectory_candidates: tuple[TrajectoryEvaluation, ...] = ()

    def checkpoint(self) -> NavigationStateCheckpoint:
''',
        '''        self._trajectory_candidates: tuple[TrajectoryEvaluation, ...] = ()
        self._face_person_track_id: str | None = None
        self._face_person_aligned = False

    def checkpoint(self) -> NavigationStateCheckpoint:
''',
        "L6 init face state",
    )
    text = replace_once(
        text,
        '''            self._pending_release_tick_id,
            self._pending_release_not_before_ns,
        )
''',
        '''            self._pending_release_tick_id,
            self._pending_release_not_before_ns,
            self._face_person_track_id,
            self._face_person_aligned,
        )
''',
        "L6 checkpoint values",
    )
    text = replace_once(
        text,
        '''        self._trajectory_candidates = checkpoint.trajectory_candidates
        # Derived acceleration state is deliberately not part of replay authority.
''',
        '''        self._trajectory_candidates = checkpoint.trajectory_candidates
        self._face_person_track_id = checkpoint.face_person_track_id
        self._face_person_aligned = checkpoint.face_person_aligned
        # Derived acceleration state is deliberately not part of replay authority.
''',
        "L6 restore face state",
    )
    text = replace_once(
        text,
        '''        if mission.mode is CommandMode.EXPLORE:
            return self._exploration_plan(mission, estimate, world)

        target = mission.target_pose
''',
        '''        if mission.mode is CommandMode.FACE_PERSON:
            return self._face_person_plan(mission, estimate, world)
        if mission.mode is CommandMode.EXPLORE:
            return self._exploration_plan(mission, estimate, world)

        target = mission.target_pose
''',
        "L6 evaluate FACE_PERSON branch",
    )
    text = replace_once(
        text,
        '''        self._trajectory_candidates = ()
        self._static_planning_index = None

    def _exploration_plan(
''',
        '''        self._trajectory_candidates = ()
        self._static_planning_index = None
        self._face_person_track_id = None
        self._face_person_aligned = False

    def _face_person_plan(
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
            and track.confidence >= self._config.face_person_min_confidence
        )
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
            self._face_person_track_id = selected.track_id
            self._face_person_aligned = False

        if selected is None:
            self._face_person_track_id = None
            self._face_person_aligned = False
            return self._inactive(
                mission,
                NavigationStatus.INVALIDATED,
                "PERSON_TARGET_NOT_AVAILABLE",
            )

        dx = selected.x_m - estimate.x_m
        dy = selected.y_m - estimate.y_m
        if math.hypot(dx, dy) <= 1e-9:
            desired_yaw = estimate.yaw_rad
            heading_error = 0.0
        else:
            desired_yaw = math.atan2(dy, dx)
            heading_error = _wrapped_angle(desired_yaw - estimate.yaw_rad)

        absolute_error = abs(heading_error)
        if self._face_person_aligned:
            if absolute_error > self._config.face_person_release_tolerance_rad:
                self._face_person_aligned = False
        elif absolute_error <= self._config.face_person_align_tolerance_rad:
            self._face_person_aligned = True

        target_yaw = estimate.yaw_rad if self._face_person_aligned else desired_yaw
        return NavigationPlan(
            context=mission.context,
            mission_id=mission.mission_id,
            route=(Waypoint(estimate.x_m, estimate.y_m, target_yaw),),
            velocity_target=None,
            constraints=mission.constraints,
            corridor_radius_m=0.0,
            progress=0.0,
            status=NavigationStatus.ACTIVE,
        )

    def _exploration_plan(
''',
        "L6 FACE_PERSON planner",
    )
    return text


def transform_native_control(text: str) -> str:
    if 'v3_navigation.face_person' in text:
        return text
    text = replace_once(
        text,
        '''    person_tracking_enabled = person_tracking.get("enabled", True)
    if type(person_tracking_enabled) is not bool:
        raise ValueError("v3_navigation.person_tracking.enabled must be bool")
    exploration = _mapping(root.get("exploration"), "v3_navigation.exploration")
''',
        '''    person_tracking_enabled = person_tracking.get("enabled", True)
    if type(person_tracking_enabled) is not bool:
        raise ValueError("v3_navigation.person_tracking.enabled must be bool")
    face_person_value = root.get("face_person")
    face_person = (
        {}
        if face_person_value is None
        else _mapping(face_person_value, "v3_navigation.face_person")
    )
    exploration = _mapping(root.get("exploration"), "v3_navigation.exploration")
''',
        "native_control face_person mapping",
    )
    text = replace_once(
        text,
        '''        person_track_radius_m=_positive_float(
            person_tracking.get("track_radius_m", 0.30),
            "v3_navigation.person_tracking.track_radius_m",
        ),
    )
    navigation = NavigationConfig(
''',
        '''        person_track_radius_m=_positive_float(
            person_tracking.get("track_radius_m", 0.30),
            "v3_navigation.person_tracking.track_radius_m",
        ),
        person_track_max_age_ns=_positive_int(
            person_tracking.get("track_max_age_ns", 500_000_000),
            "v3_navigation.person_tracking.track_max_age_ns",
        ),
    )
    navigation = NavigationConfig(
        face_person_min_confidence=_finite_float(
            face_person.get("minimum_confidence", 0.60),
            "v3_navigation.face_person.minimum_confidence",
        ),
        face_person_align_tolerance_rad=_positive_float(
            face_person.get("align_tolerance_rad", 0.10),
            "v3_navigation.face_person.align_tolerance_rad",
        ),
        face_person_release_tolerance_rad=_positive_float(
            face_person.get("release_tolerance_rad", 0.16),
            "v3_navigation.face_person.release_tolerance_rad",
        ),
''',
        "native_control face configs",
    )
    return text


def transform_config(text: str) -> str:
    data = json.loads(text)
    nav = data["v3_navigation"]
    nav.setdefault("person_tracking", {}).setdefault("track_max_age_ns", 500_000_000)
    nav.setdefault(
        "face_person",
        {
            "minimum_confidence": 0.60,
            "align_tolerance_rad": 0.10,
            "release_tolerance_rad": 0.16,
        },
    )
    return json.dumps(data, ensure_ascii=False, indent=2) + "\n"


def transform_operator_controller(text: str) -> str:
    if "def faceperson(" in text:
        return text
    text = replace_once(
        text,
        '''    def wheel_targets_to_twist(
''',
        '''    def faceperson(
        self,
        *,
        max_omega_rad_s: float = 0.50,
        capture: bool = True,
        capture_mode: str = DEFAULT_CAPTURE_MODE,
    ) -> MotionHandle:
        max_omega = self._finite(max_omega_rad_s, "max_omega_rad_s")
        if max_omega <= 0.0 or max_omega > 1.20:
            raise OperatorError("max_omega_rad_s must be >0 and <=1.20")
        command_id = f"operator-faceperson-{time.time_ns()}-{os.getpid()}"
        args = [
            self.python,
            "-m",
            "v3.control_cli",
            "faceperson",
            "--command-id",
            command_id,
            "--max-omega-rad-s",
            str(max_omega),
        ]
        pid, mode = self._start_motion(
            "faceperson",
            capture,
            capture_mode,
            args,
            require_real_motion=False,
        )
        return MotionHandle(
            pid=pid,
            label="faceperson",
            command_id=command_id,
            capture_mode=mode,
        )

    def wheel_targets_to_twist(
''',
        "operator_controller.faceperson",
    )
    text = replace_once(
        text,
        '''        command: list[str],
    ) -> tuple[int, str]:
''',
        '''        command: list[str],
        *,
        require_real_motion: bool = True,
    ) -> tuple[int, str]:
''',
        "operator_controller _start_motion signature",
    )
    text = replace_once(
        text,
        '''            pid = self._spawn_control_process(command)
            if not self._wait_allow(pid, baseline, label):
                raise OperatorError(f"{label} did not reach motor ALLOW")
''',
        '''            pid = self._spawn_control_process(command)
            if require_real_motion:
                accepted = self._wait_allow(pid, baseline, label)
            else:
                accepted = self._wait_allow(
                    pid,
                    baseline,
                    label,
                    require_real_motion=False,
                )
            if not accepted:
                raise OperatorError(f"{label} did not reach ACTIVE ALLOW")
''',
        "operator_controller _start_motion wait",
    )
    text = replace_once(
        text,
        '''        self._emit("info", f"{label}: STARTED")
        self._emit("info", "motor: ALLOW confirmed")
''',
        '''        self._emit("info", f"{label}: STARTED")
        if require_real_motion:
            self._emit("info", "motor: ALLOW confirmed")
        else:
            self._emit("info", "command: ACTIVE/ALLOW confirmed (zero motion is valid)")
''',
        "operator_controller start message",
    )
    text = replace_once(
        text,
        '''    def _wait_allow(self, pid: int, baseline: int, label: str) -> bool:
''',
        '''    def _wait_allow(
        self,
        pid: int,
        baseline: int,
        label: str,
        *,
        require_real_motion: bool = True,
    ) -> bool:
''',
        "operator_controller _wait_allow signature",
    )
    text = replace_once(
        text,
        '''                if self._status_has_real_allow(last):
                    return True
''',
        '''                allowed = (
                    self._status_has_real_allow(last)
                    if require_real_motion
                    else self._status_has_allow(last)
                )
                if allowed:
                    return True
''',
        "operator_controller allow predicate",
    )
    text = replace_once(
        text,
        '''    @staticmethod
    def _status_has_real_allow(status: Mapping[str, object]) -> bool:
        if status.get("state") != "RUNNING" or status.get("safety_decision") != "ALLOW" or status.get("enabled") is not True:
            return False
''',
        '''    @staticmethod
    def _status_has_allow(status: Mapping[str, object]) -> bool:
        return (
            status.get("state") == "RUNNING"
            and status.get("safety_decision") == "ALLOW"
            and status.get("enabled") is True
        )

    @staticmethod
    def _status_has_real_allow(status: Mapping[str, object]) -> bool:
        if not OperatorController._status_has_allow(status):
            return False
''',
        "operator_controller zero-motion ALLOW",
    )
    return text


def transform_operator_cli(text: str) -> str:
    if 'sub.add_parser("faceperson"' in text:
        return text
    text = text.replace(
        '{"forward", "mozog", "wheels", "roomcruise", "explore"}',
        '{"forward", "mozog", "wheels", "roomcruise", "explore", "faceperson"}',
    )
    text = replace_once(
        text,
        '''            "  ./r2b4 roomcruise\\n"
            "  ./r2b4 proba c full\\n"
''',
        '''            "  ./r2b4 roomcruise\\n"
            "  ./r2b4 faceperson c full\\n"
            "  ./r2b4 proba c full\\n"
''',
        "operator_cli example",
    )
    text = replace_once(
        text,
        '''    room = sub.add_parser("roomcruise", aliases=["explore"], help="autonomous Room Cruise / EXPLORE")
    motion_flags(room)

    panic = sub.add_parser("panic", help="stop motion and shut down the runtime")
''',
        '''    room = sub.add_parser("roomcruise", aliases=["explore"], help="autonomous Room Cruise / EXPLORE")
    motion_flags(room)

    faceperson = sub.add_parser("faceperson", help="rotate in place toward a tracked person")
    faceperson.add_argument("--max-omega", type=float, default=0.50)
    motion_flags(faceperson)

    panic = sub.add_parser("panic", help="stop motion and shut down the runtime")
''',
        "operator_cli faceperson parser",
    )
    text = replace_once(
        text,
        '''        elif args.command in {"roomcruise", "explore"}:
            controller.roomcruise(capture=not args.no_trigger, capture_mode=capture_mode)
        elif args.command == "proba":
''',
        '''        elif args.command in {"roomcruise", "explore"}:
            controller.roomcruise(capture=not args.no_trigger, capture_mode=capture_mode)
        elif args.command == "faceperson":
            controller.faceperson(
                max_omega_rad_s=args.max_omega,
                capture=not args.no_trigger,
                capture_mode=capture_mode,
            )
        elif args.command == "proba":
''',
        "operator_cli route faceperson",
    )
    return text


P0A_TESTS = r'''

class _LateReadyBackend:
    def __init__(self, config):
        self._inner = InlineTrajectoryRolloutBackend(config)
        self._take_calls = 0

    def submit(self, request):
        return self._inner.submit(request)

    def take(self, request_id):
        self._take_calls += 1
        if self._take_calls == 1:
            return None
        return self._inner.take(request_id)

    def abandon(self, request_id):
        self._inner.abandon(request_id)

    def close(self):
        self._inner.close()


def test_late_async_rollout_keeps_fresh_previous_plan():
    config = NavigationConfig(trajectory_replan_interval_ns=100_000_000)
    backend = _LateReadyBackend(config)
    navigator = TrajectoryNavigator(
        config,
        rollout_backend=backend,
        rollout_release_tick_gap=5,
        rollout_release_delay_ns=100_000_000,
        max_plan_age_ns=350_000_000,
    )
    manager = MissionManager()

    seed = _evaluate_at(navigator, manager, 0, 1_000_000_000)
    previous_candidates = seed.trajectory_candidates
    _evaluate_at(navigator, manager, 5, 1_100_000_000)

    at_release = _evaluate_at(navigator, manager, 10, 1_200_000_000)
    assert at_release.trajectory_candidates == previous_candidates
    assert navigator.checkpoint().last_replan_ns == 1_000_000_000

    accepted = _evaluate_at(navigator, manager, 11, 1_220_000_000)
    assert accepted.trajectory_candidates != previous_candidates
    assert navigator.checkpoint().last_replan_ns == 1_100_000_000
    backend.close()


def test_async_rollout_fails_when_previous_plan_becomes_stale():
    config = NavigationConfig(trajectory_replan_interval_ns=100_000_000)
    navigator = TrajectoryNavigator(
        config,
        rollout_backend=_NeverReadyBackend(),
        rollout_release_tick_gap=5,
        rollout_release_delay_ns=100_000_000,
        max_plan_age_ns=350_000_000,
    )
    manager = MissionManager()

    _evaluate_at(navigator, manager, 0, 1_000_000_000)
    _evaluate_at(navigator, manager, 5, 1_100_000_000)

    _evaluate_at(navigator, manager, 10, 1_200_000_000)

    with pytest.raises(RuntimeError, match="ASYNC_L6_PLAN_STALE"):
        _evaluate_at(navigator, manager, 18, 1_360_000_000)
'''


def transform_async_test(text: str) -> str:
    if "test_late_async_rollout_keeps_fresh_previous_plan" not in text:
        text = text.rstrip() + "\n" + P0A_TESTS + "\n"
    return text


FACE_TEST = r'''from __future__ import annotations

import math
from pathlib import Path

import pytest

from v3.adapters.resident_command import (
    AtomicResidentCommandGateway,
    ResidentCommandClient,
    ResidentCommandMailboxConfig,
)
from v3.contracts import (
    CommandMode,
    CommandRequest,
    DataField,
    MissionLifecycle,
    NavigationStatus,
    ObstacleTrack,
    RobotEstimate,
    TickContext,
    WorldSnapshot,
)
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import NavigationConfig, TrajectoryNavigator
from v3.layers.l7_motion_selection import select_motion
from v3.layers.l8_motion_realization import MotionRealizer


def _estimate(context: TickContext, *, yaw: float = 0.0) -> RobotEstimate:
    covariance = tuple(0.01 if index % 6 == 0 else 0.0 for index in range(25))
    return RobotEstimate(
        context=context,
        frame_id="R2B4_BOOT_ROBOT_MAP",
        x_m=0.0,
        y_m=0.0,
        yaw_rad=yaw,
        v_mps=0.0,
        omega_rad_s=0.0,
        covariance_5x5=covariance,
    )


def _world(context: TickContext, *tracks: ObstacleTrack) -> WorldSnapshot:
    return WorldSnapshot(
        context=context,
        frame_id="R2B4_BOOT_ROBOT_MAP",
        map_revision=1,
        obstacle_tracks=tuple(tracks),
        freshness_ns=0,
        local_costmap=None,
    )


def _person(track_id: str, x: float, y: float, confidence: float = 0.9) -> ObstacleTrack:
    return ObstacleTrack(
        track_id=track_id,
        x_m=x,
        y_m=y,
        radius_m=0.30,
        vx_mps=0.0,
        vy_mps=0.0,
        confidence=confidence,
    )


def _mission(context: TickContext, *, command_id: str = "face-1"):
    return MissionManager().evaluate(
        CommandRequest(
            context=context,
            command_id=command_id,
            mode=CommandMode.FACE_PERSON,
            goal=(DataField("max_omega_rad_s", 0.50),),
            expiry_tick=context.tick_id,
        )
    )


def test_face_person_is_an_active_targetless_mission():
    context = TickContext(1, 1_000_000_000)
    mission = _mission(context)
    assert mission.mode is CommandMode.FACE_PERSON
    assert mission.lifecycle is MissionLifecycle.ACTIVE
    assert mission.target_pose is None
    assert mission.velocity_target is None
    assert mission.constraints.max_omega_rad_s == pytest.approx(0.50)


@pytest.mark.parametrize(("y_m", "sign"), [(1.0, 1.0), (-1.0, -1.0)])
def test_face_person_uses_existing_l7_l8_path_and_never_requests_translation(y_m, sign):
    context = TickContext(2, 1_020_000_000)
    estimate = _estimate(context)
    world = _world(context, _person("person-1", 1.0, y_m))
    plan = TrajectoryNavigator().evaluate(_mission(context), estimate, world)
    assert plan.status is NavigationStatus.ACTIVE
    assert len(plan.route) == 1
    assert plan.route[0].x_m == pytest.approx(estimate.x_m)
    assert plan.route[0].y_m == pytest.approx(estimate.y_m)

    motion = MotionRealizer().evaluate(select_motion(plan), estimate, world)
    assert motion.requested_v_mps == 0.0
    assert math.copysign(1.0, motion.requested_omega_rad_s) == sign


def test_face_person_alignment_hysteresis_prevents_chatter():
    navigator = TrajectoryNavigator(
        NavigationConfig(
            face_person_align_tolerance_rad=0.10,
            face_person_release_tolerance_rad=0.16,
        )
    )
    command_id = "face-hysteresis"

    c1 = TickContext(10, 1_000_000_000)
    p1 = navigator.evaluate(
        _mission(c1, command_id=command_id),
        _estimate(c1),
        _world(c1, _person("person-1", math.cos(0.05), math.sin(0.05))),
    )
    assert navigator.checkpoint().face_person_aligned is True
    assert p1.route[0].yaw_rad == pytest.approx(0.0)

    c2 = TickContext(11, 1_020_000_000)
    p2 = navigator.evaluate(
        _mission(c2, command_id=command_id),
        _estimate(c2),
        _world(c2, _person("person-1", math.cos(0.12), math.sin(0.12))),
    )
    assert navigator.checkpoint().face_person_aligned is True
    assert p2.route[0].yaw_rad == pytest.approx(0.0)

    c3 = TickContext(12, 1_040_000_000)
    p3 = navigator.evaluate(
        _mission(c3, command_id=command_id),
        _estimate(c3),
        _world(c3, _person("person-1", math.cos(0.20), math.sin(0.20))),
    )
    assert navigator.checkpoint().face_person_aligned is False
    assert p3.route[0].yaw_rad == pytest.approx(0.20)


def test_face_person_keeps_selected_track_until_it_is_lost():
    navigator = TrajectoryNavigator()
    command_id = "face-sticky"

    c1 = TickContext(20, 2_000_000_000)
    navigator.evaluate(
        _mission(c1, command_id=command_id),
        _estimate(c1),
        _world(
            c1,
            _person("person-1", 1.0, 0.1, 0.80),
            _person("person-2", 2.0, 0.8, 0.90),
        ),
    )
    assert navigator.checkpoint().face_person_track_id == "person-2"

    c2 = TickContext(21, 2_020_000_000)
    navigator.evaluate(
        _mission(c2, command_id=command_id),
        _estimate(c2),
        _world(
            c2,
            _person("person-1", 1.0, -0.1, 0.99),
            _person("person-2", 2.0, 0.7, 0.61),
        ),
    )
    assert navigator.checkpoint().face_person_track_id == "person-2"

    c3 = TickContext(22, 2_040_000_000)
    plan = navigator.evaluate(
        _mission(c3, command_id=command_id),
        _estimate(c3),
        _world(c3, _person("person-1", 1.0, -0.1, 0.99)),
    )
    assert plan.status is NavigationStatus.ACTIVE
    assert navigator.checkpoint().face_person_track_id == "person-1"


def test_face_person_target_loss_is_fail_closed_but_reacquirable():
    navigator = TrajectoryNavigator()
    command_id = "face-loss"

    c1 = TickContext(30, 3_000_000_000)
    missing = navigator.evaluate(
        _mission(c1, command_id=command_id),
        _estimate(c1),
        _world(c1),
    )
    assert missing.status is NavigationStatus.INVALIDATED
    assert missing.reason == "PERSON_TARGET_NOT_AVAILABLE"

    c2 = TickContext(31, 3_020_000_000)
    reacquired = navigator.evaluate(
        _mission(c2, command_id=command_id),
        _estimate(c2),
        _world(c2, _person("person-7", 1.0, 0.5)),
    )
    assert reacquired.status is NavigationStatus.ACTIVE
    assert navigator.checkpoint().face_person_track_id == "person-7"


def test_face_person_checkpoint_restores_target_identity_and_alignment():
    nav = TrajectoryNavigator()
    c1 = TickContext(40, 4_000_000_000)
    nav.evaluate(
        _mission(c1, command_id="face-checkpoint"),
        _estimate(c1),
        _world(c1, _person("person-9", 1.0, 0.01)),
    )
    checkpoint = nav.checkpoint()
    assert checkpoint.face_person_track_id == "person-9"
    assert checkpoint.face_person_aligned is True

    restored = TrajectoryNavigator()
    restored.restore(checkpoint)
    assert restored.checkpoint().face_person_track_id == "person-9"
    assert restored.checkpoint().face_person_aligned is True


def test_resident_face_person_command_roundtrip(tmp_path: Path):
    path = tmp_path / "command.json"
    config = ResidentCommandMailboxConfig(path=path)
    client = ResidentCommandClient(config, monotonic_ns=lambda: 10_000_000_000)
    client.publish_face_person(
        "face-mailbox",
        max_omega_rad_s=0.50,
        ttl_ns=200_000_000,
    )

    gateway = AtomicResidentCommandGateway(
        config,
        monotonic_ns=lambda: 10_000_000_000,
    )
    command = gateway.snapshot(TickContext(50, 10_000_000_000))
    assert command.mode is CommandMode.FACE_PERSON
    assert command.goal == (DataField("max_omega_rad_s", 0.50),)
'''


TRANSFORMS = {
    "v3/contracts/messages.py": transform_messages,
    "v3/layers/l4_world_model.py": transform_l4,
    "v3/layers/l5_command_mission.py": transform_l5,
    "v3/layers/l6_navigation.py": transform_l6,
    "v3/adapters/resident_command.py": transform_resident_command,
    "v3/control_cli.py": transform_control_cli,
    "v3/operator_controller.py": transform_operator_controller,
    "v3/operator_cli.py": transform_operator_cli,
    "v3/composition/native_control.py": transform_native_control,
    "conf/vezerles.json": transform_config,
    "tests/test_v3_async_l6_planner.py": transform_async_test,
}


def build(root: Path) -> dict[str, str]:
    outputs: dict[str, str] = {}
    for relative, fn in TRANSFORMS.items():
        path = root / relative
        if not path.is_file():
            raise UpgradeError(f"missing required file: {relative}")
        outputs[relative] = fn(path.read_text(encoding="utf-8"))
    outputs["tests/test_v3_face_person.py"] = FACE_TEST
    for relative, text in outputs.items():
        path = root / relative
        if relative.endswith(".py"):
            ensure_python(path, text)
        elif relative.endswith(".json"):
            ensure_json(path, text)
    return outputs


def verify_p0a(text: str) -> None:
    required = (
        'raise RuntimeError("async rollout has no authoritative cached plan")',
        'raise RuntimeError("ASYNC_L6_PLAN_STALE")',
        'if release_not_before_ns is None:',
        'raise RuntimeError("ASYNC_L6_DEADLINE_MISSED")',
        'self._require_fresh_cached_plan(context)',
    )
    missing = [item for item in required if item not in text]
    if missing:
        raise UpgradeError(f"P0-A verification failed: missing {missing}")


def main() -> int:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--apply", action="store_true")
    parser.add_argument("root", type=Path)
    args = parser.parse_args()

    root = args.root.expanduser().resolve()
    outputs = build(root)
    verify_p0a(outputs["v3/layers/l6_navigation.py"])

    changed = []
    for relative, text in outputs.items():
        path = root / relative
        old = path.read_text(encoding="utf-8") if path.exists() else None
        if old != text:
            changed.append(relative)

    print("FACE_PERSON source check: PASS")
    print("P0-A exact guard: PASS")
    print("Files to change:")
    for relative in changed:
        print(f"  {relative}")

    if args.check:
        return 0

    stamp = time.strftime("%Y%m%d_%H%M%S")
    backup = root / "runtime" / "upgrade_backups" / f"faceperson_p0a_{stamp}"
    backup.mkdir(parents=True, exist_ok=False)
    manifest = {
        "upgrade": "R2B4_FACE_PERSON_P0A_V1",
        "changed_files": changed,
        "p0a": {
            "production_monotonic_late_result": "keep fresh cached plan",
            "legacy_tick_gate": "hard deadline unchanged",
            "max_plan_age_ns": 350_000_000,
        },
    }

    for relative in changed:
        src = root / relative
        if src.exists():
            dst = backup / relative
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)

    (backup / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n",
        encoding="utf-8",
    )

    for relative, text in outputs.items():
        path = root / relative
        if path.exists() and path.read_text(encoding="utf-8") == text:
            continue
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")

    for relative in outputs:
        if relative.endswith(".py"):
            py_compile.compile(str(root / relative), doraise=True)

    print(f"Applied. Backup: {backup}")
    print("Python compile: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
