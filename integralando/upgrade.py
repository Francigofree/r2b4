#!/usr/bin/env python3
"""R2B4 FACE_PERSON canonical V3 upgrade for the 2026-09-16 source tree.

Applies only to the exact source blobs listed in BASE_BLOBS. Runtime/capture files
and unrelated working-tree changes are ignored. On targeted test failure the
upgrade automatically restores the touched files from runtime/upgrade_backups/.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

BASE_COMMIT = "3236cada2eb12b7f63944ffa1fc55c47b39ae8be"
UPGRADE_ID = "r2b4-faceperson-v1-20260916-r2"

BASE_BLOBS = {
    "README.md": "394101bfeb94db7d665eda7771899235927e5172",
    "conf/vezerles.json": "699067c80814e44f190bc2de4cd6873672094df8",
    "v3/contracts/messages.py": "55b5388dce84b7b0790cad48a4b1fd13e94d48c1",
    "v3/layers/l5_command_mission.py": "d093630490fcfa9c0c80159ae11308417fbb6c58",
    "v3/layers/l6_navigation.py": "fc6465cd83483bc89d656c9e950a68fe3989c07a",
    "v3/layers/l7_motion_selection.py": "583b28073c5e6a3ef3bac1c9ca2c9db9cc2628f8",
    "v3/composition/native_control.py": "cbcf40e0a60481843033fa352fef86d761e16363",
    "v3/adapters/resident_command.py": "b45c7b714f7a5140cdd293c930d3a689ce94d9ee",
    "v3/control_cli.py": "5005dfdfab3f1963b73796ff42dbaebf7b631320",
    "v3/operator_controller.py": "cfbfa2b354695f891e5b3eb917cd12500bf42a7b",
    "v3/operator_cli.py": "6e041e9eedb28add67c24e09df9cae6490411217",
    "v3/adapters/person_photo_evidence.py": "eaccb2fef166d3e68ab5df9cd9a8bf4c3a09e7df",
    "tests/test_v3_resident_command.py": "47ca893603cdb02725fdf5764528b9f73980e830",
}

NEW_TEST_PATH = "tests/test_v3_face_person.py"
BACKUP_MARKER = "runtime/.faceperson_upgrade_backup"


def run(root: Path, *args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        list(args), cwd=root, text=True, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, check=check,
    )


def git_blob(root: Path, relative: str) -> str:
    result = run(root, "git", "hash-object", relative)
    return result.stdout.strip()


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly one source match, found {count}")
    return text.replace(old, new, 1)


def patch_file(root: Path, relative: str, transforms: list[tuple[str, str, str]]) -> None:
    path = root / relative
    text = path.read_text(encoding="utf-8")
    for old, new, label in transforms:
        text = replace_once(text, old, new, f"{relative}: {label}")
    path.write_text(text, encoding="utf-8")


def preflight(root: Path) -> None:
    if not (root / ".git").exists():
        raise RuntimeError(f"not a Git working tree: {root}")
    messages = (root / "v3/contracts/messages.py").read_text(encoding="utf-8")
    controller = (root / "v3/operator_controller.py").read_text(encoding="utf-8")
    if "FACE_PERSON = \"FACE_PERSON\"" in messages and "def faceperson(" in controller:
        raise SystemExit("FACE_PERSON upgrade already appears to be installed")
    mismatches: list[str] = []
    for relative, expected in BASE_BLOBS.items():
        path = root / relative
        if not path.is_file():
            mismatches.append(f"{relative}: missing")
            continue
        actual = git_blob(root, relative)
        if actual != expected:
            mismatches.append(f"{relative}: expected {expected}, got {actual}")
    l6_text = (root / "v3/layers/l6_navigation.py").read_text(encoding="utf-8")
    if "context.monotonic_ns - result.source_context.monotonic_ns" not in l6_text:
        mismatches.append(
            "v3/layers/l6_navigation.py: async-L6 handoff freshness P0 fix anchor missing"
        )
    if (root / NEW_TEST_PATH).exists():
        mismatches.append(f"{NEW_TEST_PATH}: already exists")
    if mismatches:
        joined = "\n  - ".join(mismatches)
        raise RuntimeError(
            "source preflight failed; package refuses to patch a different source tree:\n  - " + joined
        )


def backup(root: Path) -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    target = root / "runtime" / "upgrade_backups" / f"faceperson_{stamp}"
    target.mkdir(parents=True, exist_ok=False)
    for relative in BASE_BLOBS:
        source = root / relative
        destination = target / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
    manifest = {
        "upgrade_id": UPGRADE_ID,
        "base_commit": BASE_COMMIT,
        "files": sorted(BASE_BLOBS),
        "created_files": [NEW_TEST_PATH],
    }
    (target / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    marker = root / BACKUP_MARKER
    marker.parent.mkdir(parents=True, exist_ok=True)
    marker.write_text(str(target) + "\n", encoding="utf-8")
    return target


def rollback(root: Path, backup_dir: Path | None = None) -> None:
    if backup_dir is None:
        marker = root / BACKUP_MARKER
        if not marker.is_file():
            raise RuntimeError("no FACE_PERSON backup marker found")
        backup_dir = Path(marker.read_text(encoding="utf-8").strip())
    manifest_path = backup_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    for relative in manifest["files"]:
        source = backup_dir / relative
        destination = root / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
    for relative in manifest.get("created_files", []):
        try:
            (root / relative).unlink()
        except FileNotFoundError:
            pass
    print(f"ROLLBACK OK: {backup_dir}")


def apply_patches(root: Path) -> None:
    patch_file(root, "v3/contracts/messages.py", [
        (
            '    EXPLORE = "EXPLORE"\n    SERVICE = "SERVICE"',
            '    EXPLORE = "EXPLORE"\n    FACE_PERSON = "FACE_PERSON"\n    SERVICE = "SERVICE"',
            "add FACE_PERSON command mode",
        ),
        (
            '            if self.mode is CommandMode.EXPLORE and (\n'
            '                self.target_pose is not None or self.velocity_target is not None\n'
            '            ):\n'
            '                raise ContractValidationError("active EXPLORE mission cannot carry a fixed target")',
            '            if self.mode in (CommandMode.EXPLORE, CommandMode.FACE_PERSON) and (\n'
            '                self.target_pose is not None or self.velocity_target is not None\n'
            '            ):\n'
            '                raise ContractValidationError(\n'
            '                    "active EXPLORE/FACE_PERSON mission cannot carry a fixed target"\n'
            '                )',
            "allow target-free active FACE_PERSON MissionIntent",
        ),
    ])

    patch_file(root, "v3/layers/l5_command_mission.py", [
        (
            '            elif command.mode is CommandMode.EXPLORE:\n',
            '            elif command.mode in (CommandMode.EXPLORE, CommandMode.FACE_PERSON):\n',
            "L5 accepts FACE_PERSON mission",
        ),
    ])

    patch_file(root, "v3/layers/l6_navigation.py", [
        (
            '    TrajectoryPose,\n    Waypoint,\n',
            '    TrajectoryPose,\n    VelocityTarget,\n    Waypoint,\n',
            "import VelocityTarget",
        ),
        (
            '    novelty_weight: float = 0.22\n\n    def __post_init__(self) -> None:\n',
            '    novelty_weight: float = 0.22\n'
            '    face_person_confidence_floor: float = 0.50\n'
            '    face_person_yaw_deadband_rad: float = 0.10\n'
            '    face_person_yaw_gain: float = 2.5\n'
            '    face_person_min_omega_rad_s: float = 0.40\n'
            '    face_person_max_omega_rad_s: float = 0.60\n\n'
            '    def __post_init__(self) -> None:\n',
            "add FACE_PERSON navigation config",
        ),
        (
            '            raise ValueError("obstacle_confidence_floor must be in [0, 1]")\n'
            '        for name in (\n',
            '            raise ValueError("obstacle_confidence_floor must be in [0, 1]")\n'
            '        if (\n'
            '            not isinstance(self.face_person_confidence_floor, (int, float))\n'
            '            or isinstance(self.face_person_confidence_floor, bool)\n'
            '            or not math.isfinite(self.face_person_confidence_floor)\n'
            '            or not 0.0 <= self.face_person_confidence_floor <= 1.0\n'
            '        ):\n'
            '            raise ValueError("face_person_confidence_floor must be in [0, 1]")\n'
            '        for name in (\n',
            "validate FACE_PERSON confidence",
        ),
        (
            '            "clearance_score_cap_m",\n        ):\n',
            '            "clearance_score_cap_m",\n'
            '            "face_person_yaw_deadband_rad",\n'
            '            "face_person_yaw_gain",\n'
            '            "face_person_min_omega_rad_s",\n'
            '            "face_person_max_omega_rad_s",\n'
            '        ):\n',
            "validate positive FACE_PERSON gains and limits",
        ),
        (
            '        if (\n            not isinstance(self.footprint_safety_margin_m, (int, float))\n',
            '        if self.face_person_yaw_deadband_rad >= math.pi:\n'
            '            raise ValueError("face_person_yaw_deadband_rad must be below pi")\n'
            '        if self.face_person_min_omega_rad_s > self.face_person_max_omega_rad_s:\n'
            '            raise ValueError("face_person minimum omega cannot exceed maximum omega")\n'
            '        if (\n            not isinstance(self.footprint_safety_margin_m, (int, float))\n',
            "validate FACE_PERSON omega ordering",
        ),
        (
            '        if mission.mode is CommandMode.EXPLORE:\n            return self._exploration_plan(mission, estimate, world)\n',
            '        if mission.mode is CommandMode.FACE_PERSON:\n'
            '            self._reset()\n'
            '            return self._face_person_plan(mission, estimate, world)\n'
            '        if mission.mode is CommandMode.EXPLORE:\n'
            '            return self._exploration_plan(mission, estimate, world)\n',
            "route FACE_PERSON through L6",
        ),
        (
            '    def _inactive(\n',
            '    def _face_person_plan(\n'
            '        self,\n'
            '        mission: MissionIntent,\n'
            '        estimate: RobotEstimate,\n'
            '        world: WorldSnapshot,\n'
            '    ) -> NavigationPlan:\n'
            '        people = tuple(\n'
            '            track\n'
            '            for track in world.obstacle_tracks\n'
            '            if track.track_id.startswith("person-")\n'
            '            and track.confidence >= self._config.face_person_confidence_floor\n'
            '        )\n'
            '        if not people:\n'
            '            return self._inactive(\n'
            '                mission,\n'
            '                NavigationStatus.INVALIDATED,\n'
            '                "FACE_PERSON_TARGET_MISSING",\n'
            '            )\n\n'
            '        def target_key(track):\n'
            '            dx = track.x_m - estimate.x_m\n'
            '            dy = track.y_m - estimate.y_m\n'
            '            heading_error = _wrapped_angle(math.atan2(dy, dx) - estimate.yaw_rad)\n'
            '            distance_m = math.hypot(dx, dy)\n'
            '            return (abs(heading_error), distance_m, -track.confidence, track.track_id)\n\n'
            '        target = min(people, key=target_key)\n'
            '        dx = target.x_m - estimate.x_m\n'
            '        dy = target.y_m - estimate.y_m\n'
            '        heading_error = _wrapped_angle(math.atan2(dy, dx) - estimate.yaw_rad)\n'
            '        if abs(heading_error) <= self._config.face_person_yaw_deadband_rad:\n'
            '            return self._inactive(\n'
            '                mission,\n'
            '                NavigationStatus.IDLE,\n'
            '                "FACE_PERSON_ALIGNED",\n'
            '            )\n\n'
            '        max_omega = min(\n'
            '            self._config.face_person_max_omega_rad_s,\n'
            '            mission.constraints.max_omega_rad_s,\n'
            '        )\n'
            '        raw_omega = abs(self._config.face_person_yaw_gain * heading_error)\n'
            '        omega_magnitude = min(\n'
            '            max_omega,\n'
            '            max(self._config.face_person_min_omega_rad_s, raw_omega),\n'
            '        )\n'
            '        omega = math.copysign(omega_magnitude, heading_error)\n'
            '        return NavigationPlan(\n'
            '            context=mission.context,\n'
            '            mission_id=mission.mission_id,\n'
            '            route=(),\n'
            '            velocity_target=VelocityTarget(0.0, omega),\n'
            '            constraints=mission.constraints,\n'
            '            corridor_radius_m=0.0,\n'
            '            progress=0.0,\n'
            '            status=NavigationStatus.ACTIVE,\n'
            '            reason="FACE_PERSON_TRACK",\n'
            '        )\n\n'
            '    def _inactive(\n',
            "add deterministic L4-person to pivot target planner",
        ),
    ])

    patch_file(root, "v3/layers/l7_motion_selection.py", [
        (
            '    if plan.status is NavigationStatus.ACTIVE and plan.velocity_target is not None:\n'
            '        return MotionObjective(\n'
            '            context=plan.context,\n'
            '            selected_source="teleop",\n'
            '            kind=MotionObjectiveKind.VELOCITY,\n'
            '            priority=200,\n'
            '            expiry_tick=plan.context.tick_id + 1,\n'
            '            selection_reason="DIRECT_VELOCITY",\n',
            '    if plan.status is NavigationStatus.ACTIVE and plan.velocity_target is not None:\n'
            '        face_person = plan.reason == "FACE_PERSON_TRACK"\n'
            '        return MotionObjective(\n'
            '            context=plan.context,\n'
            '            selected_source="face_person" if face_person else "teleop",\n'
            '            kind=MotionObjectiveKind.VELOCITY,\n'
            '            priority=200,\n'
            '            expiry_tick=plan.context.tick_id + 1,\n'
            '            selection_reason="FACE_PERSON_TRACK" if face_person else "DIRECT_VELOCITY",\n',
            "preserve FACE_PERSON source identity in L7",
        ),
    ])

    patch_file(root, "v3/composition/native_control.py", [
        (
            '    person_tracking_enabled = person_tracking.get("enabled", True)\n'
            '    if type(person_tracking_enabled) is not bool:\n'
            '        raise ValueError("v3_navigation.person_tracking.enabled must be bool")\n'
            '    exploration = _mapping(root.get("exploration"), "v3_navigation.exploration")\n',
            '    person_tracking_enabled = person_tracking.get("enabled", True)\n'
            '    if type(person_tracking_enabled) is not bool:\n'
            '        raise ValueError("v3_navigation.person_tracking.enabled must be bool")\n'
            '    face_person_value = root.get("face_person")\n'
            '    face_person = (\n'
            '        {}\n'
            '        if face_person_value is None\n'
            '        else _mapping(face_person_value, "v3_navigation.face_person")\n'
            '    )\n'
            '    exploration = _mapping(root.get("exploration"), "v3_navigation.exploration")\n',
            "parse FACE_PERSON config section",
        ),
        (
            '    navigation = NavigationConfig(\n'
            '        trajectory_replan_interval_ns=_positive_int(\n',
            '    navigation = NavigationConfig(\n'
            '        face_person_confidence_floor=_finite_float(\n'
            '            face_person.get("confidence_floor", 0.50),\n'
            '            "v3_navigation.face_person.confidence_floor",\n'
            '        ),\n'
            '        face_person_yaw_deadband_rad=_positive_float(\n'
            '            face_person.get("yaw_deadband_rad", 0.10),\n'
            '            "v3_navigation.face_person.yaw_deadband_rad",\n'
            '        ),\n'
            '        face_person_yaw_gain=_positive_float(\n'
            '            face_person.get("yaw_gain", 2.5),\n'
            '            "v3_navigation.face_person.yaw_gain",\n'
            '        ),\n'
            '        face_person_min_omega_rad_s=_positive_float(\n'
            '            face_person.get("min_omega_rad_s", 0.40),\n'
            '            "v3_navigation.face_person.min_omega_rad_s",\n'
            '        ),\n'
            '        face_person_max_omega_rad_s=_positive_float(\n'
            '            face_person.get("max_omega_rad_s", 0.60),\n'
            '            "v3_navigation.face_person.max_omega_rad_s",\n'
            '        ),\n'
            '        trajectory_replan_interval_ns=_positive_int(\n',
            "inject FACE_PERSON config into L6",
        ),
    ])

    patch_file(root, "conf/vezerles.json", [
        (
            '      "track_radius_m": 0.3\n    },\n    "async_l6": {',
            '      "track_radius_m": 0.3\n    },\n'
            '    "face_person": {\n'
            '      "confidence_floor": 0.5,\n'
            '      "yaw_deadband_rad": 0.1,\n'
            '      "yaw_gain": 2.5,\n'
            '      "min_omega_rad_s": 0.4,\n'
            '      "max_omega_rad_s": 0.6\n'
            '    },\n'
            '    "async_l6": {',
            "add production FACE_PERSON tuning",
        ),
    ])

    patch_file(root, "v3/adapters/resident_command.py", [
        (
            '            raise ValueError("command mode must be STOP, TELEOP or EXPLORE") from exc\n'
            '        if mode not in (CommandMode.STOP, CommandMode.TELEOP, CommandMode.EXPLORE):\n'
            '            raise ValueError("command mode must be STOP, TELEOP or EXPLORE")\n',
            '            raise ValueError("command mode must be STOP, TELEOP, EXPLORE or FACE_PERSON") from exc\n'
            '        if mode not in (\n'
            '            CommandMode.STOP,\n'
            '            CommandMode.TELEOP,\n'
            '            CommandMode.EXPLORE,\n'
            '            CommandMode.FACE_PERSON,\n'
            '        ):\n'
            '            raise ValueError("command mode must be STOP, TELEOP, EXPLORE or FACE_PERSON")\n',
            "resident gateway accepts FACE_PERSON",
        ),
        (
            '            else limit_keys if mode is CommandMode.EXPLORE else set()\n',
            '            else limit_keys\n'
            '            if mode in (CommandMode.EXPLORE, CommandMode.FACE_PERSON)\n'
            '            else set()\n',
            "FACE_PERSON mailbox field shape",
        ),
        (
            '        if mode is CommandMode.EXPLORE:\n'
            '            return CommandRequest(\n'
            '                context=context,\n'
            '                command_id=command_id,\n'
            '                mode=CommandMode.EXPLORE,\n',
            '        if mode in (CommandMode.EXPLORE, CommandMode.FACE_PERSON):\n'
            '            return CommandRequest(\n'
            '                context=context,\n'
            '                command_id=command_id,\n'
            '                mode=mode,\n',
            "deserialize FACE_PERSON mission limits",
        ),
        (
            '    def _publish(\n'
            '        self,\n'
            '        command_id: str,\n',
            '    def publish_face_person(\n'
            '        self,\n'
            '        command_id: str,\n'
            '        *,\n'
            '        max_v_mps: float,\n'
            '        max_omega_rad_s: float,\n'
            '        ttl_ns: int,\n'
            '    ) -> int:\n'
            '        return self._publish(\n'
            '            command_id,\n'
            '            CommandMode.FACE_PERSON,\n'
            '            {\n'
            '                "max_v_mps": max_v_mps,\n'
            '                "max_omega_rad_s": max_omega_rad_s,\n'
            '            },\n'
            '            ttl_ns,\n'
            '        )\n\n'
            '    def _publish(\n'
            '        self,\n'
            '        command_id: str,\n',
            "add FACE_PERSON client publisher",
        ),
        (
            '        if mode not in (CommandMode.STOP, CommandMode.TELEOP, CommandMode.EXPLORE):\n'
            '            raise ValueError("client mode must be STOP, TELEOP or EXPLORE")\n',
            '        if mode not in (\n'
            '            CommandMode.STOP,\n'
            '            CommandMode.TELEOP,\n'
            '            CommandMode.EXPLORE,\n'
            '            CommandMode.FACE_PERSON,\n'
            '        ):\n'
            '            raise ValueError("client mode must be STOP, TELEOP, EXPLORE or FACE_PERSON")\n',
            "client allows FACE_PERSON mode",
        ),
        (
            '            if mode is CommandMode.EXPLORE\n            else set()\n',
            '            if mode in (CommandMode.EXPLORE, CommandMode.FACE_PERSON)\n            else set()\n',
            "validate FACE_PERSON command values",
        ),
    ])

    patch_file(root, "v3/control_cli.py", [
        (
            '    explore.add_argument("--max-v-mps", type=float, default=0.30)\n'
            '    explore.add_argument("--max-omega-rad-s", type=float, default=0.60)\n'
            '    return parser\n',
            '    explore.add_argument("--max-v-mps", type=float, default=0.30)\n'
            '    explore.add_argument("--max-omega-rad-s", type=float, default=0.60)\n\n'
            '    faceperson = subcommands.add_parser(\n'
            '        "faceperson",\n'
            '        help="heartbeat FACE_PERSON: pivot in place toward an L4 person track",\n'
            '    )\n'
            '    faceperson.add_argument("--command-id")\n'
            '    faceperson.add_argument("--max-v-mps", type=float, default=0.05)\n'
            '    faceperson.add_argument("--max-omega-rad-s", type=float, default=0.60)\n'
            '    return parser\n',
            "add machine FACE_PERSON command",
        ),
        (
            '        if args.operation == "teleop":\n'
            '            publish = lambda logical_id: client.publish_teleop(\n'
            '                logical_id,\n'
            '                v_mps=args.v_mps,\n'
            '                omega_rad_s=args.omega_rad_s,\n'
            '                max_v_mps=args.max_v_mps,\n'
            '                max_omega_rad_s=args.max_omega_rad_s,\n'
            '                ttl_ns=ttl_ns,\n'
            '            )\n'
            '        else:\n'
            '            publish = lambda logical_id: client.publish_explore(\n'
            '                logical_id,\n'
            '                max_v_mps=args.max_v_mps,\n'
            '                max_omega_rad_s=args.max_omega_rad_s,\n'
            '                ttl_ns=ttl_ns,\n'
            '            )\n',
            '        if args.operation == "teleop":\n'
            '            publish = lambda logical_id: client.publish_teleop(\n'
            '                logical_id,\n'
            '                v_mps=args.v_mps,\n'
            '                omega_rad_s=args.omega_rad_s,\n'
            '                max_v_mps=args.max_v_mps,\n'
            '                max_omega_rad_s=args.max_omega_rad_s,\n'
            '                ttl_ns=ttl_ns,\n'
            '            )\n'
            '        elif args.operation == "faceperson":\n'
            '            publish = lambda logical_id: client.publish_face_person(\n'
            '                logical_id,\n'
            '                max_v_mps=args.max_v_mps,\n'
            '                max_omega_rad_s=args.max_omega_rad_s,\n'
            '                ttl_ns=ttl_ns,\n'
            '            )\n'
            '        else:\n'
            '            publish = lambda logical_id: client.publish_explore(\n'
            '                logical_id,\n'
            '                max_v_mps=args.max_v_mps,\n'
            '                max_omega_rad_s=args.max_omega_rad_s,\n'
            '                ttl_ns=ttl_ns,\n'
            '            )\n',
            "publish FACE_PERSON heartbeat",
        ),
    ])

    patch_file(root, "v3/operator_controller.py", [
        (
            '        pid, mode = self._start_motion("roomcruise", capture, capture_mode, args)\n'
            '        return MotionHandle(pid=pid, label="roomcruise", command_id=command_id, capture_mode=mode)\n\n'
            '    def wheel_targets_to_twist(\n',
            '        pid, mode = self._start_motion("roomcruise", capture, capture_mode, args)\n'
            '        return MotionHandle(pid=pid, label="roomcruise", command_id=command_id, capture_mode=mode)\n\n'
            '    def faceperson(\n'
            '        self,\n'
            '        *,\n'
            '        capture: bool = True,\n'
            '        capture_mode: str = DEFAULT_CAPTURE_MODE,\n'
            '    ) -> MotionHandle:\n'
            '        command_id = f"operator-faceperson-{time.time_ns()}-{os.getpid()}"\n'
            '        args = [\n'
            '            self.python,\n'
            '            "-m",\n'
            '            "v3.control_cli",\n'
            '            "faceperson",\n'
            '            "--command-id",\n'
            '            command_id,\n'
            '        ]\n'
            '        pid, mode = self._start_motion(\n'
            '            "faceperson",\n'
            '            capture,\n'
            '            capture_mode,\n'
            '            args,\n'
            '            wait_for_allow=False,\n'
            '            command_id=command_id,\n'
            '            command_mode="FACE_PERSON",\n'
            '        )\n'
            '        return MotionHandle(pid=pid, label="faceperson", command_id=command_id, capture_mode=mode)\n\n'
            '    def wheel_targets_to_twist(\n',
            "add FACE_PERSON operator action",
        ),
        (
            '    def _start_motion(\n'
            '        self,\n'
            '        label: str,\n'
            '        capture: bool,\n'
            '        capture_mode: str,\n'
            '        command: list[str],\n'
            '    ) -> tuple[int, str]:\n',
            '    def _start_motion(\n'
            '        self,\n'
            '        label: str,\n'
            '        capture: bool,\n'
            '        capture_mode: str,\n'
            '        command: list[str],\n'
            '        *,\n'
            '        wait_for_allow: bool = True,\n'
            '        command_id: str | None = None,\n'
            '        command_mode: str | None = None,\n'
            '    ) -> tuple[int, str]:\n',
            "support armed autonomous behavior startup",
        ),
        (
            '        try:\n'
            '            pid = self._spawn_control_process(command)\n'
            '            if not self._wait_allow(pid, baseline, label):\n'
            '                raise OperatorError(f"{label} did not reach motor ALLOW")\n'
            '        except BaseException:\n'
            '            self.stop(wait_idle=False)\n'
            '            raise\n\n'
            '        self._emit("info", f"{label}: STARTED")\n'
            '        self._emit("info", "motor: ALLOW confirmed")\n',
            '        try:\n'
            '            pid = self._spawn_control_process(command)\n'
            '            if wait_for_allow:\n'
            '                if not self._wait_allow(pid, baseline, label):\n'
            '                    raise OperatorError(f"{label} did not reach motor ALLOW")\n'
            '            else:\n'
            '                if not command_id or not command_mode:\n'
            '                    raise OperatorError("armed behavior startup requires command identity")\n'
            '                if not self._wait_behavior_armed(\n'
            '                    pid, baseline, label, command_id, command_mode\n'
            '                ):\n'
            '                    raise OperatorError(f"{label} did not arm")\n'
            '        except BaseException:\n'
            '            self.stop(wait_idle=False)\n'
            '            raise\n\n'
            '        self._emit("info", f"{label}: STARTED")\n'
            '        if wait_for_allow:\n'
            '            self._emit("info", "motor: ALLOW confirmed")\n'
            '        else:\n'
            '            self._emit("info", "behavior: ARMED; motor may remain STOP until a valid target exists")\n',
            "do not require ALLOW before FACE_PERSON target acquisition",
        ),
        (
            '    def _wait_allow(self, pid: int, baseline: int, label: str) -> bool:\n',
            '    def _wait_behavior_armed(\n'
            '        self,\n'
            '        pid: int,\n'
            '        baseline: int,\n'
            '        label: str,\n'
            '        command_id: str,\n'
            '        command_mode: str,\n'
            '    ) -> bool:\n'
            '        last: Mapping[str, object] | None = None\n'
            '        for _ in range(50):\n'
            '            if not self._pid_matches(pid, ("v3.control_cli",)):\n'
            '                self._emit("error", f"{label} command producer stopped before arming")\n'
            '                return False\n'
            '            if self._runtime_pid() is None:\n'
            '                self._emit("error", f"{label} runtime stopped before arming")\n'
            '                return False\n'
            '            last = self._read_status_optional()\n'
            '            if last is not None and self._status_is_fault(last):\n'
            '                self._emit("error", f"{label} runtime faulted before arming")\n'
            '                return False\n'
            '            try:\n'
            '                payload = json.loads(self.command_file.read_text(encoding="utf-8"))\n'
            '            except (OSError, UnicodeError, json.JSONDecodeError):\n'
            '                payload = None\n'
            '            if (\n'
            '                isinstance(payload, dict)\n'
            '                and payload.get("command_id") == command_id\n'
            '                and payload.get("mode") == command_mode\n'
            '                and last is not None\n'
            '                and self._tick(last) > baseline\n'
            '            ):\n'
            '                return True\n'
            '            time.sleep(0.05)\n'
            '        self._emit("error", f"{label} command was not observed by a fresh runtime tick")\n'
            '        return False\n\n'
            '    def _wait_allow(self, pid: int, baseline: int, label: str) -> bool:\n',
            "add FACE_PERSON armed-state acceptance",
        ),
    ])

    patch_file(root, "v3/operator_cli.py", [
        (
            '    if len(args) >= 2 and args[0] in {"forward", "mozog", "wheels", "roomcruise", "explore"} and args[1] == "start":\n',
            '    if len(args) >= 2 and args[0] in {"forward", "mozog", "wheels", "roomcruise", "explore", "faceperson"} and args[1] == "start":\n',
            "legacy normalization includes faceperson",
        ),
        (
            '            "  ./r2b4 roomcruise\\n"\n'
            '            "  ./r2b4 proba c full\\n"\n',
            '            "  ./r2b4 roomcruise\\n"\n'
            '            "  ./r2b4 faceperson c full\\n"\n'
            '            "  ./r2b4 proba c full\\n"\n',
            "document FACE_PERSON launcher example",
        ),
        (
            '    room = sub.add_parser("roomcruise", aliases=["explore"], help="autonomous Room Cruise / EXPLORE")\n'
            '    motion_flags(room)\n\n'
            '    panic = sub.add_parser("panic", help="stop motion and shut down the runtime")\n',
            '    room = sub.add_parser("roomcruise", aliases=["explore"], help="autonomous Room Cruise / EXPLORE")\n'
            '    motion_flags(room)\n\n'
            '    faceperson = sub.add_parser(\n'
            '        "faceperson",\n'
            '        help="rotate in place to face the current L4 person track",\n'
            '    )\n'
            '    motion_flags(faceperson)\n\n'
            '    panic = sub.add_parser("panic", help="stop motion and shut down the runtime")\n',
            "add human FACE_PERSON launcher parser",
        ),
        (
            '        elif args.command in {"roomcruise", "explore"}:\n'
            '            controller.roomcruise(capture=not args.no_trigger, capture_mode=capture_mode)\n'
            '        elif args.command == "proba":\n',
            '        elif args.command in {"roomcruise", "explore"}:\n'
            '            controller.roomcruise(capture=not args.no_trigger, capture_mode=capture_mode)\n'
            '        elif args.command == "faceperson":\n'
            '            controller.faceperson(capture=not args.no_trigger, capture_mode=capture_mode)\n'
            '        elif args.command == "proba":\n',
            "dispatch FACE_PERSON launcher command",
        ),
    ])

    patch_file(root, "v3/adapters/person_photo_evidence.py", [
        (
            '"""Passive roomcruise person-photo evidence from completed V3 tick values.\n',
            '"""Passive EXPLORE/FACE_PERSON photo evidence from completed V3 tick values.\n',
            "broaden photo evidence module scope",
        ),
        (
            '    """Bounded live-only evidence policy for EXPLORE/roomcruise acceptance."""\n',
            '    """Bounded live-only evidence policy for EXPLORE/FACE_PERSON acceptance."""\n',
            "broaden photo evidence config scope",
        ),
        (
            '    """Request one JPEG per stable person-presence episode during EXPLORE.\n',
            '    """Request one JPEG per stable person-presence episode during EXPLORE/FACE_PERSON.\n',
            "broaden recorder scope",
        ),
        (
            '            or mission.mode is not CommandMode.EXPLORE\n',
            '            or mission.mode not in (CommandMode.EXPLORE, CommandMode.FACE_PERSON)\n',
            "enable FACE_PERSON photo proof",
        ),
    ])

    patch_file(root, "tests/test_v3_resident_command.py", [
        (
            '"STOP, TELEOP or EXPLORE"',
            '"STOP, TELEOP, EXPLORE or FACE_PERSON"',
            "update strict-mode diagnostic expectation",
        ),
    ])

    patch_file(root, "README.md", [
        (
            './r2b4 roomcruise\n./r2b4 proba c full\n',
            './r2b4 roomcruise\n./r2b4 faceperson c full\n./r2b4 proba c full\n',
            "document FACE_PERSON command",
        ),
    ])

    test_source = r'''import json
import os

from v3 import control_cli, operator_cli
from v3.adapters.person_photo_evidence import (
    PersonPhotoEvidenceConfig,
    PersonPhotoEvidenceRecorder,
)
from v3.adapters.resident_command import (
    AtomicResidentCommandGateway,
    ResidentCommandClient,
    ResidentCommandMailboxConfig,
)
from v3.contracts import (
    AdmittedFrame,
    CommandMode,
    CommandRequest,
    DataField,
    MissionIntent,
    MissionLifecycle,
    NavigationStatus,
    ObstacleTrack,
    Observation,
    RobotEstimate,
    TickContext,
    WorldSnapshot,
)
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import NavigationConfig, TrajectoryNavigator
from v3.layers.l7_motion_selection import select_motion
from v3.layers.l8_motion_realization import MotionRealizer


def _context(tick: int = 10, ns: int = 1_000_000_000) -> TickContext:
    return TickContext(tick, ns)


def _estimate(context: TickContext, *, yaw: float = 0.0) -> RobotEstimate:
    covariance = tuple(0.01 if index % 6 == 0 else 0.0 for index in range(25))
    return RobotEstimate(context, "MAP", 0.0, 0.0, yaw, 0.0, 0.0, covariance)


def _world(context: TickContext, *tracks: ObstacleTrack) -> WorldSnapshot:
    return WorldSnapshot(context, "MAP", 1, tuple(tracks), 0, None)


def _person(track_id: str, x: float, y: float, confidence: float = 0.9) -> ObstacleTrack:
    return ObstacleTrack(track_id, x, y, 0.30, 0.0, 0.0, confidence)


def _mission(context: TickContext):
    command = CommandRequest(
        context,
        "face-person-test",
        CommandMode.FACE_PERSON,
        (DataField("max_v_mps", 0.05), DataField("max_omega_rad_s", 0.60)),
        context.tick_id,
    )
    return MissionManager().evaluate(command)


def test_face_person_l5_is_active_target_free_mission():
    context = _context()
    mission = _mission(context)
    assert mission.mode is CommandMode.FACE_PERSON
    assert mission.lifecycle is MissionLifecycle.ACTIVE
    assert mission.target_pose is None
    assert mission.velocity_target is None
    assert mission.constraints.max_v_mps == 0.05
    assert mission.constraints.max_omega_rad_s == 0.60


def test_face_person_l6_fails_closed_without_person_track():
    context = _context()
    plan = TrajectoryNavigator().evaluate(_mission(context), _estimate(context), _world(context))
    assert plan.status is NavigationStatus.INVALIDATED
    assert plan.reason == "FACE_PERSON_TARGET_MISSING"
    assert plan.velocity_target is None


def test_face_person_ignores_low_confidence_track():
    context = _context()
    world = _world(context, _person("person-1", 1.0, 0.4, confidence=0.49))
    plan = TrajectoryNavigator().evaluate(_mission(context), _estimate(context), world)
    assert plan.status is NavigationStatus.INVALIDATED
    assert plan.reason == "FACE_PERSON_TARGET_MISSING"


def test_face_person_left_target_pivots_only_and_reaches_l8():
    context = _context()
    estimate = _estimate(context)
    world = _world(context, _person("person-1", 1.0, 0.5))
    plan = TrajectoryNavigator().evaluate(_mission(context), estimate, world)
    assert plan.status is NavigationStatus.ACTIVE
    assert plan.reason == "FACE_PERSON_TRACK"
    assert plan.velocity_target is not None
    assert plan.velocity_target.v_mps == 0.0
    assert 0.0 < plan.velocity_target.omega_rad_s <= 0.60
    objective = select_motion(plan)
    assert objective.selected_source == "face_person"
    assert objective.selection_reason == "FACE_PERSON_TRACK"
    motion = MotionRealizer().evaluate(objective, estimate, world)
    assert motion.stop_reason is None
    assert motion.requested_v_mps == 0.0
    assert motion.requested_omega_rad_s > 0.0


def test_face_person_right_target_pivots_other_direction():
    context = _context()
    plan = TrajectoryNavigator().evaluate(
        _mission(context),
        _estimate(context),
        _world(context, _person("person-1", 1.0, -0.5)),
    )
    assert plan.status is NavigationStatus.ACTIVE
    assert plan.velocity_target is not None
    assert plan.velocity_target.v_mps == 0.0
    assert plan.velocity_target.omega_rad_s < 0.0


def test_face_person_aligned_target_stops_without_ending_mission():
    context = _context()
    plan = TrajectoryNavigator().evaluate(
        _mission(context),
        _estimate(context),
        _world(context, _person("person-1", 1.0, 0.05)),
    )
    assert plan.status is NavigationStatus.IDLE
    assert plan.reason == "FACE_PERSON_ALIGNED"
    assert plan.velocity_target is None


def test_face_person_deterministically_prefers_smallest_bearing_error():
    context = _context()
    world = _world(
        context,
        _person("person-1", 1.0, 0.8, 0.95),
        _person("person-2", 1.0, 0.25, 0.80),
    )
    plan = TrajectoryNavigator(NavigationConfig(face_person_yaw_deadband_rad=0.05)).evaluate(
        _mission(context), _estimate(context), world
    )
    assert plan.velocity_target is not None
    assert plan.velocity_target.omega_rad_s > 0.0


def test_face_person_mailbox_round_trip(tmp_path):
    now = 2_000_000_000
    path = tmp_path / "command.json"
    config = ResidentCommandMailboxConfig(path=path, expected_uid=os.geteuid())
    client = ResidentCommandClient(config, monotonic_ns=lambda: now)
    client.publish_face_person(
        "face-mailbox",
        max_v_mps=0.05,
        max_omega_rad_s=0.60,
        ttl_ns=200_000_000,
    )
    gateway = AtomicResidentCommandGateway(config, monotonic_ns=lambda: now + 1)
    command = gateway.snapshot(_context(1, now + 1))
    assert command.mode is CommandMode.FACE_PERSON
    assert {field.key: field.value for field in command.goal} == {
        "max_v_mps": 0.05,
        "max_omega_rad_s": 0.60,
    }


def test_operator_launcher_accepts_faceperson_full_capture():
    clean, mode, explicit = operator_cli._extract_capture_selector(["faceperson", "c", "full"])
    assert clean == ["faceperson"]
    assert mode == "full"
    assert explicit is True
    parsed = operator_cli._parser().parse_args(clean)
    assert parsed.command == "faceperson"


def test_control_cli_parser_accepts_faceperson():
    parsed = control_cli._parser().parse_args(["faceperson", "--command-id", "face-cli"])
    assert parsed.operation == "faceperson"
    assert parsed.command_id == "face-cli"
    assert parsed.max_v_mps == 0.05
    assert parsed.max_omega_rad_s == 0.60


class _PhotoPort:
    def __init__(self):
        self.outputs = []

    def request_jpeg(self, output, *, stream_name="lores"):
        self.outputs.append((str(output), stream_name))
        return True


def test_face_person_photo_evidence_is_live_proof(tmp_path):
    context = _context()
    port = _PhotoPort()
    recorder = PersonPhotoEvidenceRecorder(
        port,
        PersonPhotoEvidenceConfig(
            enabled=True,
            directory="pic",
            stream_name="main",
            confirm_results=1,
            rearm_misses=1,
            minimum_interval_ns=1,
        ),
        project_root=tmp_path,
    )
    admitted = AdmittedFrame(
        context,
        (
            Observation(
                "person_detection",
                "PERSON_DETECTOR_FRONT",
                7,
                context.monotonic_ns,
                (
                    DataField("person_detected", True),
                    DataField("source_frame_sequence", 11),
                ),
            ),
        ),
        (),
    )
    mission = MissionIntent(
        context,
        "mission-face-photo",
        CommandMode.FACE_PERSON,
        None,
        None,
        _mission(context).constraints,
        MissionLifecycle.ACTIVE,
    )
    assert recorder.observe(admitted, mission) is True
    assert len(port.outputs) == 1
    output, stream = port.outputs[0]
    assert stream == "main"
    assert "person_" in output and "det000007_frame000011.jpg" in output
'''
    (root / NEW_TEST_PATH).write_text(test_source, encoding="utf-8")


def validate(root: Path, *, full_tests: bool = False) -> None:
    commands = [
        ["git", "diff", "--check"],
        [sys.executable, "-m", "v3.import_guard"],
        [
            sys.executable,
            "-m",
            "pytest",
            "-q",
            NEW_TEST_PATH,
            "tests/test_v3_contracts.py",
            "tests/test_v3_resident_command.py",
            "tests/test_v3_control_cli.py",
            "tests/test_v3_operator_controller.py",
            "tests/test_v3_person_detection_runtime_integration.py",
            "tests/test_v3_l5_l9_mission_navigation.py",
            "tests/test_v3_async_l6_planner.py",
            "tests/test_v3_process_runtime.py",
        ],
    ]
    if full_tests:
        commands.append([sys.executable, "-m", "pytest", "-q"] )
    for command in commands:
        print("+", " ".join(command), flush=True)
        completed = subprocess.run(command, cwd=root)
        if completed.returncode != 0:
            raise RuntimeError(f"validation failed with exit code {completed.returncode}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Install canonical V3 FACE_PERSON support")
    parser.add_argument("--root", default="/home/alba/project_r2b4")
    parser.add_argument("--check", action="store_true", help="preflight only")
    parser.add_argument("--rollback", action="store_true", help="restore latest installer backup")
    parser.add_argument("--no-tests", action="store_true", help="apply without targeted validation")
    parser.add_argument("--full-tests", action="store_true", help="also run the complete pytest suite")
    args = parser.parse_args()
    root = Path(args.root).resolve()

    if args.rollback:
        rollback(root)
        return 0

    preflight(root)
    print(f"preflight: PASS (source compatible with {BASE_COMMIT})")
    if args.check:
        return 0

    backup_dir = backup(root)
    print(f"backup: {backup_dir}")
    try:
        apply_patches(root)
        print("patch: PASS")
        if not args.no_tests:
            validate(root, full_tests=args.full_tests)
            print("validation: PASS")
    except BaseException:
        print("upgrade failed; restoring backup", file=sys.stderr)
        rollback(root, backup_dir)
        raise

    print("FACE_PERSON upgrade installed")
    print("live test: ./r2b4 faceperson c full")
    print("stop:      ./r2b4 stop")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
