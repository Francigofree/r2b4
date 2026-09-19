#!/usr/bin/env python3
"""Install R2B4 V3 L7 Temporal Continuity P1-A.

Scope:
- v3/layers/l7_motion_selection.py
- v3/composition/native_control.py
- v3/composition/mission_navigation.py
- tests/test_v3_l7_motion_continuity.py (new)

Preconditions are intentionally limited to the files this upgrade modifies.
There is no full-repository SHA gate.
"""

from __future__ import annotations

import hashlib
import shutil
import subprocess
import sys
import time
from pathlib import Path


UPGRADE = "r2b4_l7_temporal_continuity_p1a"
SOURCE_COMMIT = "48678c5798ecc0dd179a6aa18760dcf92353978e"

EXPECTED_BLOBS = {
    "v3/layers/l7_motion_selection.py": "583b28073c5e6a3ef3bac1c9ca2c9db9cc2628f8",
    "v3/composition/native_control.py": "2b7242d7ac60993314a613a0ac0fc15510f0bde0",
    "v3/composition/mission_navigation.py": "20423744b45e0b2e9c7bb3a2e3e920b6e5f57b88",
}

TARGET_TEST = "tests/test_v3_l7_motion_continuity.py"


def git_blob_sha(data: bytes) -> str:
    header = f"blob {len(data)}\0".encode("ascii")
    return hashlib.sha1(header + data).hexdigest()


def find_repo_root() -> Path:
    here = Path(__file__).resolve().parent
    candidates = (here, *here.parents)
    for candidate in candidates:
        if (
            (candidate / "v3").is_dir()
            and (candidate / "tests").is_dir()
            and (candidate / "conf").is_dir()
        ):
            return candidate
    raise RuntimeError("R2B4 repo root not found above installer.py")


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(
            f"{label}: expected exactly one source match, found {count}"
        )
    return text.replace(old, new, 1)


def run(command: list[str], cwd: Path) -> None:
    print("+", " ".join(command), flush=True)
    subprocess.run(command, cwd=cwd, check=True)


def patch_native_control(text: str) -> str:
    text = replace_once(
        text,
        "from v3.layers.l7_motion_selection import select_motion\n",
        "from v3.layers.l7_motion_selection import (\n"
        "    MotionSelectionStateCheckpoint,\n"
        "    MotionSelector,\n"
        ")\n",
        "native_control import",
    )
    text = replace_once(
        text,
        "    motion_realization: MotionRealizationStateCheckpoint = MotionRealizationStateCheckpoint()\n",
        "    motion_realization: MotionRealizationStateCheckpoint = MotionRealizationStateCheckpoint()\n"
        "    motion_selection: MotionSelectionStateCheckpoint = MotionSelectionStateCheckpoint()\n",
        "native_control checkpoint field",
    )
    text = replace_once(
        text,
        '        "_mission",\n        "_motion_realization",\n',
        '        "_mission",\n        "_motion_selection",\n        "_motion_realization",\n',
        "native_control slots",
    )
    text = replace_once(
        text,
        "            backend = None\n"
        "            navigation = TrajectoryNavigator(config.navigation)\n"
        "        motion_realization = MotionRealizer(config.motion_realization)\n",
        "            backend = None\n"
        "            navigation = TrajectoryNavigator(config.navigation)\n"
        "        motion_selection = MotionSelector()\n"
        "        motion_realization = MotionRealizer(config.motion_realization)\n",
        "native_control selector construction",
    )
    text = replace_once(
        text,
        "        self._navigation = navigation\n"
        "        self._rollout_backend = backend\n"
        "        self._motion_realization = motion_realization\n",
        "        self._navigation = navigation\n"
        "        self._motion_selection = motion_selection\n"
        "        self._rollout_backend = backend\n"
        "        self._motion_realization = motion_realization\n",
        "native_control selector ownership",
    )
    text = replace_once(
        text,
        "                motion_selection=select_motion,\n",
        "                motion_selection=motion_selection.evaluate,\n",
        "native_control engine wiring",
    )
    text = replace_once(
        text,
        "            self._final_safety.checkpoint(),\n"
        "            self._motion_realization.checkpoint(),\n"
        "        )\n",
        "            self._final_safety.checkpoint(),\n"
        "            self._motion_realization.checkpoint(),\n"
        "            self._motion_selection.checkpoint(),\n"
        "        )\n",
        "native_control checkpoint capture",
    )
    text = replace_once(
        text,
        "        self._mission.restore(checkpoint.mission)\n"
        "        self._navigation.restore(checkpoint.navigation)\n"
        "        self._motion_realization.restore(checkpoint.motion_realization)\n",
        "        self._mission.restore(checkpoint.mission)\n"
        "        self._navigation.restore(checkpoint.navigation)\n"
        "        self._motion_selection.restore(checkpoint.motion_selection)\n"
        "        self._motion_realization.restore(checkpoint.motion_realization)\n",
        "native_control checkpoint restore",
    )
    return text


def patch_mission_navigation(text: str) -> str:
    text = replace_once(
        text,
        "from v3.layers.l7_motion_selection import select_motion\n",
        "from v3.layers.l7_motion_selection import MotionSelector\n",
        "mission_navigation import",
    )
    text = replace_once(
        text,
        '    __slots__ = ("_constraints", "_last_context", "_mission", "_motion", "_navigation")\n',
        '    __slots__ = (\n'
        '        "_constraints",\n'
        '        "_last_context",\n'
        '        "_mission",\n'
        '        "_motion",\n'
        '        "_motion_selection",\n'
        '        "_navigation",\n'
        '    )\n',
        "mission_navigation slots",
    )
    text = replace_once(
        text,
        "        self._navigation = TrajectoryNavigator(navigation_config)\n"
        "        self._motion = MotionRealizer(motion_config)\n",
        "        self._navigation = TrajectoryNavigator(navigation_config)\n"
        "        self._motion_selection = MotionSelector()\n"
        "        self._motion = MotionRealizer(motion_config)\n",
        "mission_navigation selector construction",
    )
    text = replace_once(
        text,
        "        objective = select_motion(navigation)\n",
        "        objective = self._motion_selection.evaluate(navigation)\n",
        "mission_navigation selector use",
    )
    return text


def payload_l7(installer_dir: Path) -> str:
    return (installer_dir / "payload" / "l7_motion_selection.py").read_text(
        encoding="utf-8"
    )


def payload_test(installer_dir: Path) -> str:
    return (installer_dir / "payload" / "test_v3_l7_motion_continuity.py").read_text(
        encoding="utf-8"
    )


def already_installed(root: Path, expected_test: str) -> bool:
    l7 = (root / "v3/layers/l7_motion_selection.py").read_text(encoding="utf-8")
    native = (root / "v3/composition/native_control.py").read_text(encoding="utf-8")
    mission = (root / "v3/composition/mission_navigation.py").read_text(encoding="utf-8")
    test_path = root / TARGET_TEST
    return bool(
        "class MotionSelector:" in l7
        and "CONTINUITY_HOLD" in l7
        and "motion_selection: MotionSelectionStateCheckpoint" in native
        and "motion_selection=motion_selection.evaluate" in native
        and "self._motion_selection = MotionSelector()" in mission
        and test_path.is_file()
        and test_path.read_text(encoding="utf-8") == expected_test
    )


def verify_preconditions(root: Path) -> None:
    for relative, expected in EXPECTED_BLOBS.items():
        path = root / relative
        if not path.is_file():
            raise RuntimeError(f"missing target file: {relative}")
        actual = git_blob_sha(path.read_bytes())
        if actual != expected:
            raise RuntimeError(
                f"precondition failed for {relative}\n"
                f" expected git-blob SHA: {expected}\n"
                f" actual   git-blob SHA: {actual}\n"
                "The file changed since the source-first design. "
                "No files were modified."
            )


def main() -> int:
    root = find_repo_root()
    installer_dir = Path(__file__).resolve().parent
    expected_l7 = payload_l7(installer_dir)
    expected_test = payload_test(installer_dir)

    print(f"repo: {root}")
    print(f"upgrade: {UPGRADE}")
    print(f"source-first base commit: {SOURCE_COMMIT}")
    print("L7 continuity: ID-HOLD, score band 0.005")

    if already_installed(root, expected_test):
        print("upgrade already installed; running targeted tests")
        run_targeted_tests(root)
        print_success(root)
        return 0

    test_target = root / TARGET_TEST
    if test_target.exists():
        raise RuntimeError(
            f"{TARGET_TEST} already exists with unexpected content; "
            "refusing to overwrite it"
        )

    verify_preconditions(root)

    stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    backup = root / "runtime" / "upgrade_backups" / f"{UPGRADE}_{stamp}"
    backup.mkdir(parents=True, exist_ok=False)

    originals: dict[str, bytes] = {}
    for relative in EXPECTED_BLOBS:
        source = root / relative
        originals[relative] = source.read_bytes()
        destination = backup / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)

    created_test = False
    try:
        l7_path = root / "v3/layers/l7_motion_selection.py"
        native_path = root / "v3/composition/native_control.py"
        mission_path = root / "v3/composition/mission_navigation.py"

        l7_path.write_text(expected_l7, encoding="utf-8")

        native_text = native_path.read_text(encoding="utf-8")
        native_path.write_text(
            patch_native_control(native_text),
            encoding="utf-8",
        )

        mission_text = mission_path.read_text(encoding="utf-8")
        mission_path.write_text(
            patch_mission_navigation(mission_text),
            encoding="utf-8",
        )

        test_target.parent.mkdir(parents=True, exist_ok=True)
        test_target.write_text(expected_test, encoding="utf-8")
        created_test = True

        run(
            [
                sys.executable,
                "-m",
                "py_compile",
                "v3/layers/l7_motion_selection.py",
                "v3/composition/native_control.py",
                "v3/composition/mission_navigation.py",
                TARGET_TEST,
            ],
            root,
        )
        run_targeted_tests(root)

    except BaseException:
        print("ERROR: install/test failed; restoring original production files")
        for relative, data in originals.items():
            (root / relative).write_bytes(data)
        if created_test:
            try:
                test_target.unlink()
            except FileNotFoundError:
                pass
        print(f"backup retained at: {backup}")
        raise

    print(f"backup: {backup}")
    print_success(root)
    return 0


def run_targeted_tests(root: Path) -> None:
    run(
        [
            sys.executable,
            "-m",
            "pytest",
            "-q",
            "tests/test_v3_l7_motion_continuity.py",
            "tests/test_v3_navigation_trajectory.py",
            "tests/test_v3_follow_person_motion_quality.py",
            "tests/test_v3_transient_sensor_timing.py",
        ],
        root,
    )


def print_success(root: Path) -> None:
    print()
    print("PASS: R2B4 V3 L7 Temporal Continuity P1-A installed.")
    print("Production behavior:")
    print("  - same mission + same collision-free candidate")
    print("  - previous candidate held only within total-score band 0.005")
    print("  - collision/missing candidate/mission or guidance change => canonical BEST/reset")
    print("  - L7 state included in native checkpoint/restore")
    print()
    print("futtasd a full pytest-et:")
    print(f"cd {root} && python3 -m pytest -q")
    print()
    print("utána élő acceptance:")
    print("r rc 12 c full")


if __name__ == "__main__":
    raise SystemExit(main())
