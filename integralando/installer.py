#!/usr/bin/env python3
"""Install combined R2B4 Voice/Self-Knowledge + External Gateway upgrade.

Targeted at repo HEAD analyzed on 2026-09-20 (GitHub main 47bfc7cc912517319297874048eccc88b93f696f).
Preconditions are checked ONLY for files this upgrade modifies.
This revision is restart-safe after the previous package stopped at the Python 3.11
ExternalRequest dataclass collection error.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent
PAYLOAD = PACKAGE_ROOT / "payload"
UPGRADE_NAME = "r2b4_voice_selfknowledge_external_gateway"

EXPECTED_GIT_BLOB = {
    "r2b4_voice/conversation_service.py": "ffd15bac62f7b2610fc7bcda26da9088958ea950",
    "r2b4_voice/conversation_interface.py": "27e8d35a28d061c0f9475dec17a65cf7c66ca0ca",
    "r2b4_voice/prompting.py": "40164d9c39da904572647611aa440bdf0a5ee169",
    "r2b4_voice/robot_context.py": "383950bb002bd07191aeb6148760e17700c943b7",
    "conf/voice_llm_system.md": "27ba49ff8dfbb8b8f84a539e58d4a513538f9cd5",
    "r2b4_voice/voice_service.py": "ca8a6c7832e64e7378e02acba190cb36fa4ee2c8",
    "v3_process_runtime.py": "35e221fe6512fbbcdbf8ed7e1e75eee7f228702d",
}

FULL_REPLACEMENTS = (
    "r2b4_voice/conversation_service.py",
    "r2b4_voice/conversation_interface.py",
    "r2b4_voice/prompting.py",
    "r2b4_voice/robot_context.py",
    "conf/voice_llm_system.md",
)

KNOWN_REPLACEABLE_NEW_BLOBS = {
    # Exact buggy External Gateway produced by the immediately previous combined installer.
    # This is intentionally file-local; no repository-wide SHA/precondition is used.
    "v3/external_gateway.py": frozenset({"5a46302f4b67b9f26a3ec67c705bcd258ac6cece"}),
}

NEW_FILES = (
    "r2b4_voice/safety_intents.py",
    "r2b4_voice/action_executor.py",
    "r2b4_voice/self_knowledge.py",
    "tests/test_voice_safety_intents.py",
    "tests/test_voice_action_executor.py",
    "tests/test_voice_self_knowledge.py",
    "tests/test_voice_conversation_completion_cache.py",
    "tests/test_voice_v3_context_projection.py",
    "v3/external_gateway.py",
    "v3_external_gateway.py",
    "tests/test_v3_external_gateway.py",
)

TARGETED_TESTS = (
    "tests/test_voice_safety_intents.py",
    "tests/test_voice_action_executor.py",
    "tests/test_voice_self_knowledge.py",
    "tests/test_voice_conversation_completion_cache.py",
    "tests/test_voice_v3_context_projection.py",
    "tests/test_r2b4_conversation_service.py",
    "tests/test_v3_process_runtime.py",
    "tests/test_interface_p0_hardening.py",
    "tests/test_v3_external_gateway.py",
    "tests/test_v3_runtime_phase_timing.py",
)


def _git_blob_sha(path: Path) -> str:
    data = path.read_bytes()
    header = f"blob {len(data)}\0".encode("ascii")
    return hashlib.sha1(header + data).hexdigest()


def _find_root(explicit: str | None) -> Path:
    if explicit:
        root = Path(explicit).expanduser().resolve()
        if (root / "r2b4_voice").is_dir() and (root / "v3").is_dir():
            return root
        raise SystemExit(f"ERROR: invalid repo root: {root}")
    preferred = Path("/home/alba/project_r2b4")
    if (preferred / "r2b4_voice").is_dir() and (preferred / "v3").is_dir():
        return preferred.resolve()
    for candidate in (PACKAGE_ROOT, *PACKAGE_ROOT.parents):
        if (candidate / "r2b4_voice").is_dir() and (candidate / "v3").is_dir():
            return candidate.resolve()
    raise SystemExit("ERROR: R2B4 repo root not found; use --root /home/alba/project_r2b4")


def _atomic_write(path: Path, data: bytes, mode: int | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp_name = tempfile.mkstemp(prefix=f".{path.name}.upgrade.", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as handle:
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
        if mode is not None:
            os.chmod(temp_name, mode)
        os.replace(temp_name, path)
    finally:
        try:
            os.unlink(temp_name)
        except FileNotFoundError:
            pass


def _backup(root: Path, backup_root: Path, relative: str) -> None:
    source = root / relative
    if not source.exists():
        return
    target = backup_root / relative
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)


def _require_expected(relative: str, path: Path) -> None:
    expected = EXPECTED_GIT_BLOB[relative]
    actual = _git_blob_sha(path)
    if actual != expected:
        raise RuntimeError(
            f"precondition failed for {relative}: git-blob {actual}, expected {expected}. "
            "Only modified files are checked; repo-wide SHA is intentionally not used."
        )


def _replace_exact(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"patch anchor {label!r} expected once, found {count}")
    return text.replace(old, new, 1)


def _patch_voice_service(text: str) -> str:
    text = _replace_exact(
        text,
        "from .conversation_interface import VoiceInterfaceBundle, build_voice_interface\n",
        "from .action_executor import VoiceActionExecutor\n"
        "from .conversation_interface import VoiceInterfaceBundle, build_voice_interface\n",
        "voice import action executor",
    )
    text = _replace_exact(
        text,
        "from .speaker import ReadyWaveSpeaker\n",
        "from .safety_intents import is_stop_intent\nfrom .speaker import ReadyWaveSpeaker\n",
        "voice import stop intent",
    )
    text = _replace_exact(
        text,
        "        *,\n        config: VoiceServiceConfig = VoiceServiceConfig(),\n",
        "        *,\n        action_executor: VoiceActionExecutor | None = None,\n        config: VoiceServiceConfig = VoiceServiceConfig(),\n",
        "voice constructor action executor",
    )
    text = _replace_exact(
        text,
        "        self._playback = playback\n        self._config = config\n",
        "        self._playback = playback\n        self._action_executor = action_executor\n        self._config = config\n",
        "voice assign action executor",
    )
    text = _replace_exact(
        text,
        "        print(f\"voice: user={self._last_transcript!r}\", flush=True)\n\n        self._set_state(VoiceServiceState.THINKING)\n",
        "        print(f\"voice: user={self._last_transcript!r}\", flush=True)\n\n"
        "        # P0.1: deterministic STOP bypasses the network/LLM completely.\n"
        "        # It still enters only through the canonical RobotInterface command path.\n"
        "        if is_stop_intent(self._last_transcript):\n"
        "            try:\n"
        "                self._conversation_interface.execute(\"v3.command.stop\")\n"
        "                self._last_action_status = \"EXECUTED:VOICE_STOP_FAST_PATH\"\n"
        "                print(\"voice: STOP fast-path executed via RobotInterface\", flush=True)\n"
        "                self._last_error = None\n"
        "            except Exception as exc:\n"
        "                self._last_action_status = \"STOP_FAILED\"\n"
        "                self._last_error = f\"voice STOP {type(exc).__name__}: {exc}\"\n"
        "                print(f\"voice: {self._last_error}\", file=sys.stderr, flush=True)\n"
        "            self._settle_and_discard()\n"
        "            return\n\n"
        "        self._set_state(VoiceServiceState.THINKING)\n",
        "voice STOP fast path",
    )
    text = _replace_exact(
        text,
        "        proposed = result.get(\"proposed_action\")\n"
        "        if proposed is not None:\n"
        "            print(\n"
        "                \"voice: intent[SHADOW]=\" + json.dumps(proposed, ensure_ascii=False, sort_keys=True),\n"
        "                flush=True,\n"
        "            )\n",
        "        proposed = result.get(\"proposed_action\")\n"
        "        if proposed is not None:\n"
        "            mode = self._action_executor.mode.upper() if self._action_executor is not None else \"PROPOSAL_ONLY\"\n"
        "            print(\n"
        "                f\"voice: intent[{mode}]=\" + json.dumps(proposed, ensure_ascii=False, sort_keys=True),\n"
        "                flush=True,\n"
        "            )\n"
        "            if self._action_executor is not None:\n"
        "                try:\n"
        "                    execution = self._action_executor.execute_proposal(proposed)\n"
        "                    self._last_action_status = execution.status\n"
        "                    print(\n"
        "                        f\"voice: action={execution.action_name} status={execution.status}\",\n"
        "                        flush=True,\n"
        "                    )\n"
        "                except Exception as exc:\n"
        "                    # Fail closed: executor failure never falls back to a direct action path.\n"
        "                    self._last_action_status = \"REJECTED:EXECUTOR_ERROR\"\n"
        "                    self._last_error = f\"voice action {type(exc).__name__}: {exc}\"\n"
        "                    print(f\"voice: {self._last_error}\", file=sys.stderr, flush=True)\n",
        "voice action execution",
    )
    text = _replace_exact(
        text,
        "        \"action_mode\": \"SHADOW\",\n        \"motor_action_execution\": False,\n",
        "        \"action_mode\": (_setting(project_env, \"R2B4_VOICE_ACTION_MODE\") or \"shadow\").lower(),\n"
        "        \"motor_action_execution\": (_setting(project_env, \"R2B4_VOICE_ACTION_MODE\") or \"shadow\").lower() == \"execute\",\n",
        "diagnostic action mode",
    )
    text = _replace_exact(
        text,
        "    parser.add_argument(\"--capture-mode\", choices=(\"alap\", \"full\", \"nincs\"), default=\"alap\")\n    return parser\n",
        "    parser.add_argument(\"--capture-mode\", choices=(\"alap\", \"full\", \"nincs\"), default=\"alap\")\n"
        "    parser.add_argument(\"--action-mode\", choices=(\"shadow\", \"execute\"), default=None)\n"
        "    parser.add_argument(\"--action-watchdog-s\", type=float, default=None)\n"
        "    return parser\n",
        "voice parser action args",
    )
    text = _replace_exact(
        text,
        "        provider, model, llm_key = _resolved_llm(project_env)\n        if not groq_key:\n",
        "        provider, model, llm_key = _resolved_llm(project_env)\n"
        "        action_mode = (args.action_mode or _setting(project_env, \"R2B4_VOICE_ACTION_MODE\") or \"shadow\").strip().lower()\n"
        "        if action_mode not in {\"shadow\", \"execute\"}:\n"
        "            raise RuntimeError(\"R2B4_VOICE_ACTION_MODE must be shadow or execute\")\n"
        "        watchdog_raw = args.action_watchdog_s if args.action_watchdog_s is not None else (_setting(project_env, \"R2B4_VOICE_ACTION_WATCHDOG_S\") or \"30\")\n"
        "        action_watchdog_s = float(watchdog_raw)\n"
        "        if not 1.0 <= action_watchdog_s <= 600.0:\n"
        "            raise RuntimeError(\"voice action watchdog must be within [1, 600] seconds\")\n"
        "        if not groq_key:\n",
        "main resolve action mode",
    )
    text = _replace_exact(
        text,
        "            PcmWavePlayer(),\n            config=VoiceServiceConfig(keyword=args.keyword, capture_mode=args.capture_mode),\n",
        "            PcmWavePlayer(),\n"
        "            action_executor=VoiceActionExecutor(\n"
        "                bundle.interface,\n"
        "                mode=action_mode,\n"
        "                session_owner_pid=os.getpid(),\n"
        "                session_watchdog_s=action_watchdog_s,\n"
        "            ),\n"
        "            config=VoiceServiceConfig(keyword=args.keyword, capture_mode=args.capture_mode),\n",
        "main attach executor",
    )
    text = text.replace(
        "    -> SHADOW-only LLMDecision -> Gemini TTS -> Linux/PipeWire speaker.\n",
        "    -> LLMDecision proposal -> fresh VoiceActionExecutor gate -> canonical RobotInterface\n"
        "    -> Gemini TTS -> Linux/PipeWire speaker.\n",
        1,
    )
    text = text.replace(
        "The service is host-side orchestration only.  It never executes an LLM-proposed\nrobot action and never writes motor/GPIO state.",
        "The service is host-side orchestration only.  It never writes motor/GPIO state; optional\nLLM proposals can execute only through the fresh-state canonical RobotInterface gate.",
        1,
    )
    return text


def _patch_v3_process_runtime(text: str) -> str:
    text = _replace_exact(
        text,
        "from v3.contracts import AcquisitionFrame, RobotEstimate\n",
        "from v3.contracts import (\n"
        "    AcquisitionFrame,\n"
        "    MissionIntent,\n"
        "    NavigationPlan,\n"
        "    RobotEstimate,\n"
        "    WorldSnapshot,\n"
        ")\n",
        "runtime context imports",
    )
    pattern = re.compile(r"def _tick_status\(.*?\n\nclass AsyncResidentStatusPublisher:", re.S)
    match = pattern.search(text)
    if match is None:
        raise RuntimeError("_tick_status patch target not found")
    replacement = '''def _tick_status(
    result: TickResult,
    ready_for_active: bool = False,
) -> dict[str, object]:
    if type(ready_for_active) is not bool:
        raise TypeError("ready_for_active must be bool")
    acquisition = _layer(result, "L1")
    estimate = _layer(result, "L3")
    world = _layer(result, "L4")
    mission = _layer(result, "L5")
    navigation = _layer(result, "L6")
    health: list[dict[str, object]] = []
    if isinstance(acquisition, AcquisitionFrame):
        health = [
            {
                "device_id": item.device_id,
                "state": item.state.value,
                "reason": item.reason,
            }
            for item in acquisition.io_health
        ]
    estimate_payload: dict[str, object] | None = None
    if isinstance(estimate, RobotEstimate):
        estimate_payload = {
            "frame_id": estimate.frame_id,
            "x_m": estimate.x_m,
            "y_m": estimate.y_m,
            "yaw_rad": estimate.yaw_rad,
            "v_mps": estimate.v_mps,
            "omega_rad_s": estimate.omega_rad_s,
        }

    world_payload: dict[str, object] | None = None
    if isinstance(world, WorldSnapshot):
        person_tracks = [
            {
                "track_id": track.track_id,
                "x_m": track.x_m,
                "y_m": track.y_m,
                "radius_m": track.radius_m,
                "vx_mps": track.vx_mps,
                "vy_mps": track.vy_mps,
                "confidence": track.confidence,
            }
            for track in world.obstacle_tracks
            if track.track_id.startswith("person-")
        ]
        world_payload = {
            "frame_id": world.frame_id,
            "map_revision": world.map_revision,
            "freshness_ns": world.freshness_ns,
            "obstacle_track_count": len(world.obstacle_tracks),
            "person_tracks": person_tracks,
            "local_costmap": (
                None
                if world.local_costmap is None
                else {
                    "revision": world.local_costmap.revision,
                    "occupied_cell_count": len(world.local_costmap.occupied_cells),
                    "freshness_ns": world.local_costmap.freshness_ns,
                    "radius_m": world.local_costmap.radius_m,
                    "resolution_m": world.local_costmap.resolution_m,
                }
            ),
        }

    mission_payload: dict[str, object] | None = None
    if isinstance(mission, MissionIntent):
        mission_payload = {
            "mission_id": mission.mission_id,
            "mode": mission.mode.value,
            "lifecycle": mission.lifecycle.value,
            "stop_reason": mission.stop_reason,
            "constraints": {
                "max_v_mps": mission.constraints.max_v_mps,
                "max_omega_rad_s": mission.constraints.max_omega_rad_s,
            },
            "target_pose": (
                None
                if mission.target_pose is None
                else {
                    "x_m": mission.target_pose.x_m,
                    "y_m": mission.target_pose.y_m,
                    "yaw_rad": mission.target_pose.yaw_rad,
                }
            ),
        }

    navigation_payload: dict[str, object] | None = None
    if isinstance(navigation, NavigationPlan):
        navigation_payload = {
            "mission_id": navigation.mission_id,
            "status": navigation.status.value,
            "reason": navigation.reason,
            "progress": navigation.progress,
            "route_waypoint_count": len(navigation.route),
            "trajectory_candidate_count": len(navigation.trajectory_candidates),
            "local_goal": (
                None
                if navigation.local_goal is None
                else {
                    "x_m": navigation.local_goal.x_m,
                    "y_m": navigation.local_goal.y_m,
                    "yaw_rad": navigation.local_goal.yaw_rad,
                }
            ),
        }

    final = result.final_actuation
    return {
        "schema": RESIDENT_PROCESS_STATUS_SCHEMA,
        "state": "RUNNING",
        "tick_id": result.trace.context.tick_id,
        "monotonic_ns": result.trace.context.monotonic_ns,
        "fault_layer": result.trace.fault_layer,
        "safety_decision": final.safety_decision.value,
        "safety_reason": final.reason,
        "enabled": final.enabled,
        "left_output": final.left_output,
        "right_output": final.right_output,
        "ready_for_active": ready_for_active,
        "source_health": health,
        "estimate": estimate_payload,
        "world": world_payload,
        "mission": mission_payload,
        "navigation": navigation_payload,
    }


class AsyncResidentStatusPublisher:'''
    return text[: match.start()] + replacement + text[match.end() :]


def _install_full(root: Path, backup_root: Path, relative: str) -> str:
    target = root / relative
    desired = (PAYLOAD / relative).read_bytes()
    if target.is_file() and target.read_bytes() == desired:
        return "already"
    if not target.is_file():
        raise RuntimeError(f"missing modified target: {relative}")
    _require_expected(relative, target)
    _backup(root, backup_root, relative)
    mode = target.stat().st_mode & 0o777
    _atomic_write(target, desired, mode)
    return "updated"


def _install_new(root: Path, backup_root: Path, relative: str) -> str:
    source = PAYLOAD / relative
    desired = source.read_bytes()
    target = root / relative
    if target.exists():
        if target.is_file() and target.read_bytes() == desired:
            return "already"
        allowed = KNOWN_REPLACEABLE_NEW_BLOBS.get(relative, frozenset())
        if target.is_file() and _git_blob_sha(target) in allowed:
            _backup(root, backup_root, relative)
            _atomic_write(target, desired, target.stat().st_mode & 0o777)
            return "repaired"
        raise RuntimeError(f"new-file collision: {relative}")
    _atomic_write(target, desired, source.stat().st_mode & 0o777)
    return "created"


def _install_patched(root: Path, backup_root: Path, relative: str, marker: str, patcher) -> str:
    target = root / relative
    if not target.is_file():
        raise RuntimeError(f"missing modified target: {relative}")
    text = target.read_text(encoding="utf-8")
    if marker in text:
        return "already"
    _require_expected(relative, target)
    patched = patcher(text)
    if patched == text:
        raise RuntimeError(f"patch produced no changes: {relative}")
    _backup(root, backup_root, relative)
    _atomic_write(target, patched.encode("utf-8"), target.stat().st_mode & 0o777)
    return "updated"


def _run(root: Path, command: list[str]) -> None:
    print("+", " ".join(command), flush=True)
    subprocess.run(command, cwd=root, check=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default=None)
    parser.add_argument("--no-tests", action="store_true")
    args = parser.parse_args(argv)
    root = _find_root(args.root)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    backup_root = root / "runtime" / "upgrade_backups" / f"{UPGRADE_NAME}_{stamp}"
    print(f"R2B4 root: {root}")
    print("Upgrade: Voice/Self-Knowledge P0.1, P0.3-P0.5, P1.1-P1.8 + External Gateway slice; P0.2 omitted")

    changed: list[tuple[str, str]] = []
    try:
        for relative in FULL_REPLACEMENTS:
            changed.append((relative, _install_full(root, backup_root, relative)))
        for relative in NEW_FILES:
            changed.append((relative, _install_new(root, backup_root, relative)))
        changed.append(
            (
                "r2b4_voice/voice_service.py",
                _install_patched(
                    root,
                    backup_root,
                    "r2b4_voice/voice_service.py",
                    "from .action_executor import VoiceActionExecutor",
                    _patch_voice_service,
                ),
            )
        )
        changed.append(
            (
                "v3_process_runtime.py",
                _install_patched(
                    root,
                    backup_root,
                    "v3_process_runtime.py",
                    '"person_tracks": person_tracks',
                    _patch_v3_process_runtime,
                ),
            )
        )
    except Exception as exc:
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        if backup_root.exists():
            print(f"Backups: {backup_root}", file=sys.stderr)
        return 2

    for relative, status in changed:
        print(f"{status:>7}  {relative}")
    if backup_root.exists():
        print(f"Backups: {backup_root}")

    try:
        _run(root, [sys.executable, "-m", "py_compile", *[
            str(root / path) for path in (
                "r2b4_voice/safety_intents.py",
                "r2b4_voice/action_executor.py",
                "r2b4_voice/self_knowledge.py",
                "r2b4_voice/conversation_service.py",
                "r2b4_voice/conversation_interface.py",
                "r2b4_voice/prompting.py",
                "r2b4_voice/robot_context.py",
                "r2b4_voice/voice_service.py",
                "v3_process_runtime.py",
                "v3/external_gateway.py",
                "v3_external_gateway.py",
            )
        ]])
        if not args.no_tests:
            existing_tests = [path for path in TARGETED_TESTS if (root / path).is_file()]
            _run(root, [sys.executable, "-m", "pytest", "-q", *existing_tests])
    except subprocess.CalledProcessError as exc:
        print(f"ERROR: targeted validation failed with exit code {exc.returncode}", file=sys.stderr)
        print(f"Backups: {backup_root}", file=sys.stderr)
        return exc.returncode or 1

    print("\nINSTALL OK")
    print("Voice action mode default: SHADOW (safe migration).")
    print("To enable real FACE_PERSON/FOLLOW_PERSON execution: R2B4_VOICE_ACTION_MODE=execute")
    print("STOP fast-path is active independently of SHADOW/EXECUTE mode.")
    print("P0.2 was intentionally NOT implemented; half-duplex speaking behavior is unchanged.")
    print("External gateway defaults to read/capabilities/STOP only; positive actions require --allow-execute.")
    print("Gateway transport: JSONL/stdio only in this slice; no network listener is installed.")
    print("\nFuttasd a full pytest-et:")
    print("python3 -m pytest -q")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
