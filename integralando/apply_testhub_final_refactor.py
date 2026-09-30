#!/usr/bin/env python3
"""Transactional R2B4 Test Hub final refactor.

Source authority: Francigofree/r2b4
Expected HEAD: 3ee4e9caf26bf191f6c966d7264d6c8b51e90aed
"""
from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

EXPECTED_HEAD = "3ee4e9caf26bf191f6c966d7264d6c8b51e90aed"
PACKAGE_DIR = Path(__file__).resolve().parent
PAYLOAD_DIR = PACKAGE_DIR / "files"

KEEP_TESTHUB_MODULES = {
    "test_hub_runtime.py",
    "test_hub_diagnostic_coverage.py",
    "test_hub_compiler.py",
    "test_hub_facts.py",
}
PATCH_TARGETS = (
    "v3/process_sidecars.py",
    "v3/adapters/process_lidar_port.py",
    "v3_process_runtime.py",
    "v3/interface_adapters.py",
    "v3/interface_cli.py",
    "v3/replay.py",
    "v3/test_hub_runtime.py",
    "tools/v3_mcap_measurement.py",
)
PAYLOAD_TARGETS = (
    "v3/test_hub_facts.py",
    "v3/test_hub_compiler.py",
    "v3/test_hub.py",
    "tests/deep/test_v3_test_hub_evidence_compiler_contract.py",
    "tests/feature/test_v3_test_hub_diagnostic_contracts.py",
)


def run(*args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(args, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if check and result.returncode:
        print(f"COMMAND FAILED ({result.returncode}): {' '.join(args)}", file=sys.stderr)
        if result.stdout:
            print("--- stdout ---", file=sys.stderr)
            print(result.stdout, file=sys.stderr)
        if result.stderr:
            print("--- stderr ---", file=sys.stderr)
            print(result.stderr, file=sys.stderr)
        raise subprocess.CalledProcessError(
            result.returncode, args, output=result.stdout, stderr=result.stderr
        )
    return result


def git(*args: str) -> str:
    return run("git", *args).stdout.strip()


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly one anchor, found {count}")
    return text.replace(old, new, 1)


def patch_sidecars(text: str) -> str:
    text = replace_once(
        text,
        "    raw_lidar_queue: Any,\n    control_queue: Any,\n",
        "    raw_lidar_queue: Any,\n    raw_lidar_end_queue: Any,\n    control_queue: Any,\n",
        "sidecar:end-queue-arg",
    )
    text = replace_once(
        text,
        """            while True:
                try:
                    command = control_queue.get_nowait()
""",
        """            if raw_lidar_end_queue is not None:
                try:
                    end_message = raw_lidar_end_queue.get_nowait()
                except queue.Empty:
                    end_message = None
                if end_message is not None:
                    if not isinstance(end_message, tuple) or len(end_message) != 3:
                        raise RuntimeError("invalid raw LiDAR terminal marker")
                    raw_end_received = True
                    hub.publish(
                        {
                            "event_type": "raw_lidar_transport_end",
                            "last_revision": int(end_message[0]),
                            "produced_count": int(end_message[1]),
                            "superseded_count": int(end_message[2]),
                        },
                        topic="v3.raw_lidar_transport",
                    )

            while True:
                try:
                    command = control_queue.get_nowait()
""",
        "sidecar:drain-end-queue",
    )
    old_end = """                elif raw_kind == "raw_end":
                    if len(raw_message) != 4:
                        raise RuntimeError("invalid raw LiDAR end marker")
                    raw_end_received = True
                    hub.publish(
                        {
                            "event_type": "raw_lidar_transport_end",
                            "last_revision": int(raw_message[1]),
                            "produced_count": int(raw_message[2]),
                            "superseded_count": int(raw_message[3]),
                        },
                        topic="v3.raw_lidar_transport",
                    )
                else:
"""
    text = replace_once(text, old_end, "                else:\n", "sidecar:remove-data-end")
    text = replace_once(
        text, '        "_raw_lidar_queue",\n',
        '        "_raw_lidar_queue",\n        "_raw_lidar_end_queue",\n',
        "sidecar:slot",
    )
    text = replace_once(
        text,
        """        self._raw_lidar_queue = (
            context.Queue(maxsize=_RAW_LIDAR_CAPACITY) if self._config.sensor_debug else None
        )
        self._control_queue = context.Queue(maxsize=16)
""",
        """        self._raw_lidar_queue = (
            context.Queue(maxsize=_RAW_LIDAR_CAPACITY) if self._config.sensor_debug else None
        )
        self._raw_lidar_end_queue = (
            context.Queue(maxsize=1) if self._config.sensor_debug else None
        )
        self._control_queue = context.Queue(maxsize=16)
""",
        "sidecar:create-end-queue",
    )
    text = replace_once(
        text,
        """                self._data_queue,
                self._raw_lidar_queue,
                self._control_queue,
""",
        """                self._data_queue,
                self._raw_lidar_queue,
                self._raw_lidar_end_queue,
                self._control_queue,
""",
        "sidecar:process-arg",
    )
    text = replace_once(
        text,
        """    @property
    def raw_lidar_transport_capacity(self) -> int:
""",
        """    @property
    def raw_lidar_end_queue(self) -> Any:
        return self._raw_lidar_end_queue

    @property
    def raw_lidar_transport_capacity(self) -> int:
""",
        "sidecar:end-property",
    )
    text = text.replace(
        '"incident" if config.tick_sample_hz == 50 else "off"',
        '"auto" if config.tick_sample_hz == 50 else "off"',
    )
    return text


def patch_lidar_process(text: str) -> str:
    pattern = re.compile(
        r"def _put_raw_end\(target: Any, \*, last_revision: int, produced_count: int,\n"
        r"\s*superseded_count: int, timeout_s: float\) -> None:\n[\s\S]*?"
        r"(?=\n\ndef _lidar_owner_process_main\()"
    )
    matches = list(pattern.finditer(text))
    if len(matches) != 1:
        raise RuntimeError(f"lidar:end-function: expected one block, found {len(matches)}")
    replacement = """def _put_raw_end(target: Any, *, last_revision: int, produced_count: int,
                 superseded_count: int, timeout_s: float) -> None:
    marker = (int(last_revision), int(produced_count), int(superseded_count))
    try:
        target.put(marker, timeout=timeout_s)
    except queue.Full as exc:
        raise RuntimeError("raw LiDAR terminal evidence queue is full") from exc
"""
    text = text[:matches[0].start()] + replacement + text[matches[0].end():]
    text = replace_once(
        text,
        "    state_queue: Any,\n    raw_queue: Any,\n    ready_event: Any,\n",
        "    state_queue: Any,\n    raw_queue: Any,\n    raw_end_queue: Any,\n    ready_event: Any,\n",
        "lidar:end-main-arg",
    )
    text = replace_once(
        text,
        """        if raw_queue is not None:
            _put_raw_end(
                raw_queue,
""",
        """        if raw_queue is not None and raw_end_queue is not None:
            _put_raw_end(
                raw_end_queue,
""",
        "lidar:end-send",
    )
    text = replace_once(
        text,
        '"_ready_event", "_state_queue", "_raw_queue", "_status", "_stop_event",\n',
        '"_ready_event", "_state_queue", "_raw_queue", "_raw_end_queue", "_status", "_stop_event",\n',
        "lidar:slot",
    )
    text = replace_once(
        text,
        """        capture_raw_queue: Any | None = None,
        process_config: LidarProcessConfig,
""",
        """        capture_raw_queue: Any | None = None,
        capture_raw_end_queue: Any | None = None,
        process_config: LidarProcessConfig,
""",
        "lidar:init-param",
    )
    text = replace_once(
        text,
        """        self._raw_queue = capture_raw_queue
        self._ready_event = context.Event()
""",
        """        self._raw_queue = capture_raw_queue
        self._raw_end_queue = capture_raw_end_queue
        if (self._raw_queue is None) != (self._raw_end_queue is None):
            raise ValueError("raw capture data/end queues must be configured together")
        self._ready_event = context.Event()
""",
        "lidar:init-assign",
    )
    text = replace_once(
        text,
        """                self._state_queue, self._raw_queue, self._ready_event, self._stop_event,
""",
        """                self._state_queue, self._raw_queue, self._raw_end_queue,
                self._ready_event, self._stop_event,
""",
        "lidar:child-arg",
    )
    text = replace_once(
        text,
        """    capture_raw_queue: Any | None = None,
    process_config: LidarProcessConfig,
) -> ProcessLidarPort:
""",
        """    capture_raw_queue: Any | None = None,
    capture_raw_end_queue: Any | None = None,
    process_config: LidarProcessConfig,
) -> ProcessLidarPort:
""",
        "lidar:open-param",
    )
    text = replace_once(
        text,
        """        capture_raw_queue=capture_raw_queue,
        process_config=process_config,
""",
        """        capture_raw_queue=capture_raw_queue,
        capture_raw_end_queue=capture_raw_end_queue,
        process_config=process_config,
""",
        "lidar:open-forward",
    )
    return text


def patch_runtime(text: str) -> str:
    text = replace_once(
        text,
        """    capture_raw_queue: Any | None = None,
    process_config=None,
""",
        """    capture_raw_queue: Any | None = None,
    capture_raw_end_queue: Any | None = None,
    process_config=None,
""",
        "runtime:factory-param",
    )
    text = replace_once(
        text,
        """            capture_raw_queue=capture_raw_queue,
        )
""",
        """            capture_raw_queue=capture_raw_queue,
            capture_raw_end_queue=capture_raw_end_queue,
        )
""",
        "runtime:factory-forward",
    )
    text = replace_once(
        text,
        """            capture_raw_queue=(capture_session.raw_lidar_queue if capture_session is not None else None),
        )
""",
        """            capture_raw_queue=(capture_session.raw_lidar_queue if capture_session is not None else None),
            capture_raw_end_queue=(capture_session.raw_lidar_end_queue if capture_session is not None else None),
        )
""",
        "runtime:factory-call",
    )
    text = text.replace('replay_mode="incident"', 'replay_mode="auto"')
    return text


def patch_interface_adapters(text: str) -> str:
    text = replace_once(
        text, "from v3.adapters.testhub import TestHubInterfaceAdapter\n", "",
        "interface:remove-import",
    )
    return replace_once(
        text, "        TestHubInterfaceAdapter(controller, root),\n", "",
        "interface:remove-adapter",
    )


def _remove_command_branch(text: str, token: str) -> str:
    main_pos = text.find("\ndef main(")
    if main_pos < 0:
        raise RuntimeError("interface_cli: main() not found")
    pos = text.find(token, main_pos)
    if pos < 0:
        return text
    start = max(
        text.rfind("\n    elif ", main_pos, pos),
        text.rfind("\n    if ", main_pos, pos),
    )
    if start < 0:
        raise RuntimeError("interface_cli: command branch start not found")
    ends = [
        item for item in (
            text.find("\n    elif ", pos),
            text.find("\n    else:", pos),
        ) if item >= 0
    ]
    if not ends:
        raise RuntimeError("interface_cli: command branch end not found")
    return text[:start] + text[min(ends):]


def patch_interface_cli(text: str) -> str:
    text = text.replace("import threading\n", "")
    text = text.replace("from v3.pytest_profiles import pytest_profile_names\n", "")
    text = text.replace('    "th": "testhub",\n', "")
    text = text.replace("capture/Test Hub finalization", "capture finalization")
    text = text.replace(", th=Test Hub", "")
    text = re.sub(r"^TEST_HUB_[A-Z0-9_]+\s*=.*\n", "", text, flags=re.M)

    start = text.find('    testhub = add_command("testhub"')
    if start >= 0:
        end = text.find('    camera = add_command("camera"', start)
        if end < 0:
            raise RuntimeError("interface_cli: camera parser anchor missing")
        text = text[:start] + text[end:]

    text = text.replace("_shutdown_with_testhub_progress", "_shutdown_runtime")
    text = text.replace('"testhub": final,', '"shutdown": final,')

    pattern = re.compile(r"\ndef _shutdown_runtime\([\s\S]*?(?=\n\ndef main\()")
    matches = list(pattern.finditer(text))
    if len(matches) != 1:
        raise RuntimeError(f"interface_cli: shutdown/helper block count={len(matches)}")
    helper = """
def _shutdown_runtime(
    interface: RobotInterface,
    *,
    capture_mode: str | None = None,
) -> Mapping[str, object]:
    del capture_mode
    raw = interface.execute("operator.runtime.stop")
    return raw if isinstance(raw, Mapping) else {"state": "STOPPED"}
"""
    text = text[:matches[0].start()] + helper + text[matches[0].end():]
    text = _remove_command_branch(text, "_run_testhub_with_progress(")

    if "testhub." in text or '"testhub"' in text or "'testhub'" in text or "Test Hub" in text:
        raise RuntimeError("interface_cli: Test Hub RobotInterface remnants remain")
    return text


def patch_replay(text: str) -> str:
    return replace_once(
        text, "        from .test_hub_v2 import inspect_mcap\n",
        "        from .test_hub_compiler import inspect_mcap\n",
        "replay:inspect-import",
    )


def patch_runtime_handoff(text: str) -> str:
    text = text.replace('replay_mode: str = "incident"', 'replay_mode: str = "auto"')
    text = text.replace('{"off", "incident", "full"}', '{"off", "auto", "full"}')
    text = text.replace("off, incident or full", "off, auto or full")
    return text


def patch_measurement(text: str) -> str:
    text = replace_once(
        text, "from v3.test_hub_v2 import verify_evidence\n",
        "from v3.test_hub_compiler import verify_evidence\n",
        "measurement:verify-import",
    )
    text = text.replace(
        "session.evidence['status'] == 'PASS' and evidence_verification['status'] == 'PASS'",
        "session.evidence['compiler_status'] == 'COMPLETE' and evidence_verification['status'] == 'VALID'",
    )
    return text


PATCHERS = {
    "v3/process_sidecars.py": patch_sidecars,
    "v3/adapters/process_lidar_port.py": patch_lidar_process,
    "v3_process_runtime.py": patch_runtime,
    "v3/interface_adapters.py": patch_interface_adapters,
    "v3/interface_cli.py": patch_interface_cli,
    "v3/replay.py": patch_replay,
    "v3/test_hub_runtime.py": patch_runtime_handoff,
    "tools/v3_mcap_measurement.py": patch_measurement,
}


def audit_no_legacy(root: Path) -> None:
    forbidden = (
        "test_hub_v2", "test_hub_next", "test_hub_analysis",
        "test_hub_motion_quality", "test_hub_localization_quality",
        "test_hub_motion_tuning", "test_hub_task_evidence",
        "test_hub_behavior", "test_hub_sampled", "test_hub_views",
        "test_hub_portable",
    )
    offenders: list[str] = []
    for base in (root / "v3", root / "tools", root / "tests"):
        if not base.exists():
            continue
        for path in base.rglob("*.py"):
            content = path.read_text(encoding="utf-8", errors="replace")
            for token in forbidden:
                if token in content:
                    offenders.append(f"{path.relative_to(root)}: {token}")
    if offenders:
        raise RuntimeError("legacy Test Hub references remain:\n" + "\n".join(offenders[:50]))
    if (root / "v3/adapters/testhub.py").exists():
        raise RuntimeError("RobotInterface Test Hub adapter still exists")
    leftovers = [
        path.name for path in (root / "v3").glob("test_hub_*.py")
        if path.name not in KEEP_TESTHUB_MODULES
    ]
    if leftovers:
        raise RuntimeError(f"legacy Test Hub modules remain: {sorted(leftovers)}")


def audit_fact_only(root: Path) -> None:
    forbidden = (
        "root_cause_candidate", "physical_root_cause", '"root_cause"',
        "priority_evidence", "HEURISTIC_FINDING", "diagnosis_status", "finding_count",
    )
    offenders: list[str] = []
    for rel in ("v3/test_hub.py", "v3/test_hub_compiler.py", "v3/test_hub_facts.py"):
        text = (root / rel).read_text(encoding="utf-8")
        for token in forbidden:
            if token in text:
                offenders.append(f"{rel}: {token}")
    if offenders:
        raise RuntimeError("fact-only contract violation:\n" + "\n".join(offenders))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument("--no-tests", action="store_true")
    args = parser.parse_args()

    root = Path.cwd()
    if not (root / ".git").is_dir():
        raise SystemExit("Run from /home/alba/project_r2b4 (git root).")
    head = git("rev-parse", "HEAD")
    if head != EXPECTED_HEAD:
        raise SystemExit(
            f"HEAD mismatch: expected {EXPECTED_HEAD}, got {head}. "
            "Regenerate the package source-first."
        )

    for rel in PAYLOAD_TARGETS:
        if not (PAYLOAD_DIR / rel).is_file():
            raise SystemExit(f"package payload missing: {rel}")

    # P0-B is already current production source; this closure slice refuses to
    # overwrite it and verifies the required contract instead.
    l11 = (root / "v3/layers/l11_actuator_control.py").read_text(encoding="utf-8")
    wheel = (root / "v3/wheel_motion.py").read_text(encoding="utf-8")
    for token in (
        "minimum_reliable_speed_mps",
        "velocity_unreliable_below_mps",
        "integral_gain = self._config.ki * self._feedback_gain",
    ):
        if token not in l11:
            raise SystemExit(f"P0-B current-source precondition missing: {token}")
    if "def validate_velocity_quality_band" not in wheel:
        raise SystemExit("P0-B current-source velocity-quality contract missing")

    legacy_modules = [
        path for path in (root / "v3").glob("test_hub_*.py")
        if path.name not in KEEP_TESTHUB_MODULES
    ]
    removal_paths = legacy_modules + [root / "v3/adapters/testhub.py"]
    touched = set(PATCH_TARGETS) | set(PAYLOAD_TARGETS)
    touched |= {str(path.relative_to(root)) for path in removal_paths if path.exists()}
    dirty = git("status", "--porcelain", "--", *sorted(touched))
    if dirty:
        raise SystemExit("Target files have local changes; refusing overwrite:\n" + dirty)

    patched: dict[str, str] = {}
    for rel, patcher in PATCHERS.items():
        patched[rel] = patcher((root / rel).read_text(encoding="utf-8"))

    if args.check_only:
        print("CHECK=PASS")
        print(f"HEAD={head}")
        print(f"legacy_modules_to_remove={len(legacy_modules)}")
        print("P0_B_CURRENT_SOURCE_PRECONDITION=PASS")
        return 0

    backup = Path(tempfile.mkdtemp(prefix="r2b4-testhub-final-backup-"))
    originals: dict[str, bytes | None] = {}
    for rel in touched:
        path = root / rel
        originals[rel] = path.read_bytes() if path.exists() else None
        if path.exists():
            dest = backup / rel
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_bytes(path.read_bytes())

    try:
        for rel, content in patched.items():
            (root / rel).write_text(content, encoding="utf-8")
        for rel in PAYLOAD_TARGETS:
            destination = root / rel
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(PAYLOAD_DIR / rel, destination)

        for path in removal_paths:
            if path.exists():
                path.unlink()

        audit_no_legacy(root)
        audit_fact_only(root)
        run(sys.executable, "-m", "compileall", "-q", "v3", "tools")

        if not args.no_tests:
            run(
                sys.executable, "-m", "pytest", "-q",
                "tests/deep/test_v3_test_hub_evidence_compiler_contract.py",
                "tests/feature/test_v3_test_hub_diagnostic_contracts.py",
            )
            extra = [
                path for path in (
                    "tests/feature/test_v3_motion_feedback_quality.py",
                    "tests/feature/test_v3_l11_reversal_reacquisition.py",
                ) if (root / path).is_file()
            ]
            if extra:
                run(sys.executable, "-m", "pytest", "-q", *extra)

        print("APPLY=PASS")
        print(f"BASE_HEAD={head}")
        print(f"BACKUP={backup}")
        print("TEST_HUB_ROLE=OFFLINE_MCAP_FACT_COMPILER")
        print("ROBOT_INTERFACE_TESTHUB_ADAPTER=REMOVED")
        print("LEGACY_TESTHUB_MODULES=REMOVED")
        print("RAW_LIDAR_TERMINAL_LANE=DEDICATED")
        print("P0_B=VERIFIED_CURRENT_SOURCE_NOT_RETUNED")
        return 0

    except BaseException:
        for rel, raw in originals.items():
            path = root / rel
            if raw is None:
                if path.exists():
                    path.unlink()
            else:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(raw)
        print(f"APPLY=FAILED; originals restored from {backup}", file=sys.stderr)
        raise


if __name__ == "__main__":
    raise SystemExit(main())
