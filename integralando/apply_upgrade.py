#!/usr/bin/env python3
"""Apply R2B4 capture-rate sampling upgrade (default 10 Hz; 50/10/5/1 selectable).

The installer never starts the robot or resident runtime.  It patches source in
memory, syntax-checks every changed Python file, makes rollback copies, writes
atomically, and optionally runs a small no-hardware regression file.
"""
from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

PACKAGE = "r2b4_capture_hz_20260923"
EXPECTED_HEAD = "d0a91aac54e5f30d23b730e890bbfe990accbf32"
HERE = Path(__file__).resolve().parent
PAYLOAD_ROOT = HERE / "files"

PATCHED = (
    "v3/interface_cli.py",
    "v3/operator_controller.py",
    "v3/adapters/v3_control.py",
    "v3/adapters/operator.py",
    "v3/operator_cli.py",
    "v3_process_runtime.py",
    "v3_hardware_runtime.py",
    "v3_runtime.py",
    "v3/mcap_capture.py",
    "v3/mcap_reader.py",
    "v3/process_sidecars.py",
    "v3/test_hub_runtime.py",
)
PAYLOADS = (
    "v3/capture_rate.py",
    "tests/test_v3_capture_hz.py",
)


def replace_once(text: str, old: str, new: str, label: str) -> str:
    """Apply the first matching anchor only.

    The repo may legitimately contain the same source fragment in more than one
    code path.  Multiple matches are therefore accepted; only a missing anchor
    is considered an error.
    """
    if new in text:
        return text
    count = text.count(old)
    if count == 0:
        raise RuntimeError(f"{label}: anchor not found")
    return text.replace(old, new, 1)


def replace_all_checked(text: str, old: str, new: str, *, minimum: int, label: str) -> str:
    if old not in text and new in text:
        return text
    count = text.count(old)
    if count == 0:
        raise RuntimeError(f"{label}: anchor not found")
    return text.replace(old, new)


def regex_once(text: str, pattern: str, replacement: str, label: str, flags: int = 0) -> str:
    compiled = re.compile(pattern, flags)
    if compiled.search(text) is None:
        raise RuntimeError(f"{label}: regex anchor not found")
    return compiled.sub(replacement, text, count=1)


def git_head(root: Path) -> str | None:
    try:
        p = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=root, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=False,
        )
    except OSError:
        return None
    return p.stdout.strip() if p.returncode == 0 else None


def atomic_write(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="") as h:
            h.write(content)
        os.replace(tmp_name, path)
    finally:
        try:
            os.unlink(tmp_name)
        except FileNotFoundError:
            pass


def patch_interface_cli(text: str) -> str:
    text = replace_once(
        text,
        "from v3.operator_controller import CAPTURE_MODES, DEFAULT_CAPTURE_MODE, OperatorError, OperatorEvent\n",
        "from v3.capture_rate import DEFAULT_CAPTURE_HZ, validate_capture_hz\n"
        "from v3.operator_controller import CAPTURE_MODES, DEFAULT_CAPTURE_MODE, OperatorError, OperatorEvent\n",
        "interface_cli import capture rate",
    )
    old_fn = '''def _extract_capture_selector(argv: Sequence[str]) -> tuple[list[str], str, bool]:\n    clean: list[str] = []\n    mode = DEFAULT_CAPTURE_MODE\n    no_trigger = False\n    explicit = False\n    i = 0\n    values = list(argv)\n    while i < len(values):\n        token = values[i]\n        if token in {"c", "--capture"}:\n            if explicit:\n                raise ValueError("capture selector may be specified only once")\n            if i + 1 >= len(values):\n                raise ValueError("capture selector requires: alap, full or nincs")\n            mode = values[i + 1]\n            if mode not in CAPTURE_MODES:\n                raise ValueError("capture mode must be one of: alap, full, nincs")\n            explicit = True\n            i += 2\n            continue\n        if token.startswith("--capture="):\n            if explicit:\n                raise ValueError("capture selector may be specified only once")\n            mode = token.split("=", 1)[1]\n            if mode not in CAPTURE_MODES:\n                raise ValueError("capture mode must be one of: alap, full, nincs")\n            explicit = True\n            i += 1\n            continue\n        if token in {"nc", "nocapture", "--no-trigger"}:\n            no_trigger = True\n            i += 1\n            continue\n        clean.append(token)\n        i += 1\n    return clean, mode, no_trigger\n'''
    new_fn = '''def _extract_capture_selector(argv: Sequence[str]) -> tuple[list[str], str, int, bool]:\n    clean: list[str] = []\n    mode = DEFAULT_CAPTURE_MODE\n    capture_hz = DEFAULT_CAPTURE_HZ\n    no_trigger = False\n    mode_explicit = False\n    hz_explicit = False\n    i = 0\n    values = list(argv)\n\n    def set_selector(value: str) -> None:\n        nonlocal mode, capture_hz, mode_explicit, hz_explicit\n        if value in CAPTURE_MODES:\n            if mode_explicit:\n                raise ValueError("capture mode may be specified only once")\n            mode = value\n            mode_explicit = True\n            return\n        if hz_explicit:\n            raise ValueError("capture Hz may be specified only once")\n        capture_hz = validate_capture_hz(value)\n        hz_explicit = True\n\n    while i < len(values):\n        token = values[i]\n        if token in {"c", "--capture", "--capture-hz", "--capture-mode"}:\n            if i + 1 >= len(values):\n                raise ValueError("capture selector requires: 50, 10, 5, 1, alap, full or nincs")\n            value = values[i + 1]\n            if token == "--capture-hz":\n                if hz_explicit:\n                    raise ValueError("capture Hz may be specified only once")\n                capture_hz = validate_capture_hz(value)\n                hz_explicit = True\n            elif token == "--capture-mode":\n                if mode_explicit or value not in CAPTURE_MODES:\n                    raise ValueError("capture mode must be one of: alap, full, nincs")\n                mode = value\n                mode_explicit = True\n            else:\n                set_selector(value)\n            i += 2\n            continue\n        if token.startswith("--capture-hz="):\n            if hz_explicit:\n                raise ValueError("capture Hz may be specified only once")\n            capture_hz = validate_capture_hz(token.split("=", 1)[1])\n            hz_explicit = True\n            i += 1\n            continue\n        if token.startswith("--capture-mode="):\n            value = token.split("=", 1)[1]\n            if mode_explicit or value not in CAPTURE_MODES:\n                raise ValueError("capture mode must be one of: alap, full, nincs")\n            mode = value\n            mode_explicit = True\n            i += 1\n            continue\n        if token.startswith("--capture="):\n            set_selector(token.split("=", 1)[1])\n            i += 1\n            continue\n        if token in {"nc", "nocapture", "--no-trigger"}:\n            no_trigger = True\n            i += 1\n            continue\n        clean.append(token)\n        i += 1\n    return clean, mode, capture_hz, no_trigger\n'''
    text = replace_once(text, old_fn, new_fn, "interface_cli selector")
    text = replace_once(
        text,
        '            "Capture anywhere: c alap | c full | c nincs   (or --capture MODE)\\n"\n',
        '            "Capture Hz: c 50 | c 10 | c 5 | c 1   (default: 10 Hz)\\n"\n'
        '            "Legacy capture mode: c alap | c full | c nincs\\n"\n',
        "interface_cli help",
    )
    text = replace_once(
        text,
        '        "capture_mode": data.get("capture_mode"),\n',
        '        "capture_mode": data.get("capture_mode"),\n        "capture_hz": data.get("capture_hz"),\n',
        "interface status hz",
    )
    text = replace_once(
        text,
        '    print(f"capture {result.get(\'capture_mode\') or \'-\'} | pid {result.get(\'pid\') or \'-\'}")\n',
        '    print(\n        f"capture {result.get(\'capture_mode\') or \'-\'}"\n        f" @ {result.get(\'capture_hz\') if result.get(\'capture_hz\') is not None else \'-\'} Hz"\n        f" | pid {result.get(\'pid\') or \'-\'}"\n    )\n',
        "interface short status hz",
    )
    text = replace_once(
        text,
        '    capture_mode: str,\n    no_trigger: bool,\n',
        '    capture_mode: str,\n    capture_hz: int,\n    no_trigger: bool,\n',
        "interface timed motion signature",
    )
    text = replace_once(
        text,
        '    parameters.update({"capture": not no_trigger, "capture_mode": capture_mode})\n',
        '    parameters.update({\n        "capture": not no_trigger,\n        "capture_mode": capture_mode,\n        "capture_hz": capture_hz,\n    })\n',
        "interface timed motion parameters",
    )
    text = replace_once(
        text,
        '    print(f"R2B4: {command} | {\'continuous\' if seconds == 0 else f\'{seconds:g} s\'} | capture {capture_mode}")\n',
        '    print(\n        f"R2B4: {command} | {\'continuous\' if seconds == 0 else f\'{seconds:g} s\'}"\n        f" | capture {capture_mode} @ {capture_hz} Hz"\n    )\n',
        "interface timed motion print",
    )
    text = replace_once(
        text,
        '        clean, capture_mode, no_trigger = _extract_capture_selector(raw)\n',
        '        clean, capture_mode, capture_hz, no_trigger = _extract_capture_selector(raw)\n',
        "interface main selector",
    )
    text = replace_once(
        text,
        '                capture_mode=capture_mode,\n                no_trigger=no_trigger,\n',
        '                capture_mode=capture_mode,\n                capture_hz=capture_hz,\n                no_trigger=no_trigger,\n',
        "interface timed main",
    )
    text = replace_once(
        text,
        '            output = interface.execute("operator.proba", capture_mode=capture_mode)\n',
        '            output = interface.execute(\n                "operator.proba", capture_mode=capture_mode, capture_hz=capture_hz\n            )\n',
        "interface proba hz",
    )
    text = replace_once(
        text,
        '                output = interface.execute("operator.runtime.start", capture_mode=capture_mode)\n',
        '                output = interface.execute(\n                    "operator.runtime.start", capture_mode=capture_mode, capture_hz=capture_hz\n                )\n',
        "interface runtime hz",
    )
    text = replace_once(
        text,
        '                output = interface.execute("capture.start", capture_mode=capture_mode)\n',
        '                output = interface.execute(\n                    "capture.start", capture_mode=capture_mode, capture_hz=capture_hz\n                )\n',
        "interface capture start hz",
    )
    return text


def patch_operator_controller(text: str) -> str:
    text = replace_once(
        text,
        "from v3.control_cli import RESIDENT_PROCESS_STATUS_SCHEMA, _read_status\n",
        "from v3.capture_rate import DEFAULT_CAPTURE_HZ, validate_capture_hz\n"
        "from v3.control_cli import RESIDENT_PROCESS_STATUS_SCHEMA, _read_status\n",
        "operator controller capture rate import",
    )
    text = replace_once(
        text,
        '        self.capture_mode_file = self.runtime_dir / ".r2b4_capture_mode"\n',
        '        self.capture_mode_file = self.runtime_dir / ".r2b4_capture_mode"\n'
        '        self.capture_hz_file = self.runtime_dir / ".r2b4_capture_hz"\n',
        "operator controller hz file",
    )
    text = replace_once(
        text,
        '            "capture_mode": snap.capture_mode,\n',
        '            "capture_mode": snap.capture_mode,\n            "capture_hz": self.current_capture_hz(),\n',
        "operator status hz",
    )
    text = replace_once(
        text,
        '    def runtime_start(self, capture_mode: str = DEFAULT_CAPTURE_MODE) -> int:\n        mode = self._validate_capture_mode(capture_mode)\n',
        '    def runtime_start(\n        self, capture_mode: str = DEFAULT_CAPTURE_MODE, capture_hz: int = DEFAULT_CAPTURE_HZ\n    ) -> int:\n        mode = self._validate_capture_mode(capture_mode)\n        hz = self._validate_capture_hz(capture_hz)\n',
        "operator runtime_start signature",
    )
    text = replace_once(
        text,
        '        self._write_private_text(self.capture_mode_file, mode)\n',
        '        self._write_private_text(self.capture_mode_file, mode)\n        self._write_private_text(self.capture_hz_file, str(hz))\n',
        "operator write hz",
    )
    text = replace_once(
        text,
        '                    "--capture-mode",\n                    mode,\n',
        '                    "--capture-mode",\n                    mode,\n                    "--capture-hz",\n                    str(hz),\n',
        "operator runtime worker hz",
    )
    text = replace_once(
        text,
        '            self._emit("info", "capture: ARMED (ALAP, native MCAP 8+2 bounded ring)")\n',
        '            self._emit(\n                "info", f"capture: ARMED (ALAP, native MCAP 8+2 bounded ring, {hz} Hz)"\n            )\n',
        "operator alap info",
    )
    text = replace_once(
        text,
        '            self._emit("info", "capture: RECORDING (FULL append-only until runtime shutdown)")\n',
        '            self._emit(\n                "info", f"capture: RECORDING (FULL append-only, {hz} Hz until runtime shutdown)"\n            )\n',
        "operator full info",
    )
    old_ensure = '''    @_serialized_operator_transition\n    def ensure_runtime(self, capture_mode: str = DEFAULT_CAPTURE_MODE) -> int:\n        requested = self._validate_capture_mode(capture_mode)\n        pid = self._runtime_pid()\n        if pid is not None:\n            current = self.current_capture_mode() or DEFAULT_CAPTURE_MODE\n            if current == requested:\n                return pid\n            self._emit("info", f"runtime: capture mode {current} -> {requested} requires safe restart")\n        return self.runtime_start(requested)\n'''
    new_ensure = '''    @_serialized_operator_transition\n    def ensure_runtime(\n        self, capture_mode: str = DEFAULT_CAPTURE_MODE, capture_hz: int = DEFAULT_CAPTURE_HZ\n    ) -> int:\n        requested = self._validate_capture_mode(capture_mode)\n        requested_hz = self._validate_capture_hz(capture_hz)\n        pid = self._runtime_pid()\n        if pid is not None:\n            current = self.current_capture_mode() or DEFAULT_CAPTURE_MODE\n            current_hz = self.current_capture_hz() or DEFAULT_CAPTURE_HZ\n            if current == requested and current_hz == requested_hz:\n                return pid\n            self._emit(\n                "info",\n                f"runtime: capture {current}@{current_hz}Hz -> "\n                f"{requested}@{requested_hz}Hz requires safe restart",\n            )\n        return self.runtime_start(requested, requested_hz)\n'''
    text = replace_once(text, old_ensure, new_ensure, "operator ensure_runtime")

    # Add capture_hz to public motion signatures in one mechanically safe form.
    text = replace_all_checked(
        text,
        '        capture_mode: str = DEFAULT_CAPTURE_MODE,\n        session_owner_pid: int | None = None,\n',
        '        capture_mode: str = DEFAULT_CAPTURE_MODE,\n        capture_hz: int = DEFAULT_CAPTURE_HZ,\n        session_owner_pid: int | None = None,\n',
        minimum=7,
        label="operator motion signatures",
    )
    text = replace_all_checked(
        text,
        'capture=capture, capture_mode=capture_mode,\n',
        'capture=capture, capture_mode=capture_mode, capture_hz=capture_hz,\n',
        minimum=2,
        label="operator start_teleop propagation inline",
    )
    text = replace_once(
        text,
        '            capture_mode=capture_mode, session_owner_pid=session_owner_pid,\n',
        '            capture_mode=capture_mode, capture_hz=capture_hz,\n            session_owner_pid=session_owner_pid,\n',
        "operator wheels hz propagation",
    )
    text = replace_once(
        text,
        '        pid, mode = self._start_motion(\n            label, capture, capture_mode, args,\n            session_owner_pid=session_owner_pid, session_watchdog_s=session_watchdog_s,\n',
        '        pid, mode = self._start_motion(\n            label, capture, capture_mode, args,\n            capture_hz=capture_hz, session_owner_pid=session_owner_pid,\n            session_watchdog_s=session_watchdog_s,\n',
        "operator teleop start motion hz",
    )
    text = replace_once(
        text,
        '        pid, mode = self._start_motion(\n            "roomcruise", capture, capture_mode, args,\n            session_owner_pid=session_owner_pid, session_watchdog_s=session_watchdog_s,\n',
        '        pid, mode = self._start_motion(\n            "roomcruise", capture, capture_mode, args,\n            capture_hz=capture_hz, session_owner_pid=session_owner_pid,\n            session_watchdog_s=session_watchdog_s,\n',
        "operator roomcruise start motion hz",
    )
    text = replace_once(
        text,
        '        pid, mode = self._start_motion(\n            "faceperson", capture, capture_mode, args, require_real_motion=False,\n            session_owner_pid=session_owner_pid, session_watchdog_s=session_watchdog_s,\n',
        '        pid, mode = self._start_motion(\n            "faceperson", capture, capture_mode, args, require_real_motion=False,\n            capture_hz=capture_hz, session_owner_pid=session_owner_pid,\n            session_watchdog_s=session_watchdog_s,\n',
        "operator faceperson start motion hz",
    )
    text = replace_once(
        text,
        '        pid, mode = self._start_motion(\n            "followperson", capture, capture_mode, args, require_real_motion=False,\n            session_owner_pid=session_owner_pid, session_watchdog_s=session_watchdog_s,\n',
        '        pid, mode = self._start_motion(\n            "followperson", capture, capture_mode, args, require_real_motion=False,\n            capture_hz=capture_hz, session_owner_pid=session_owner_pid,\n            session_watchdog_s=session_watchdog_s,\n',
        "operator followperson start motion hz",
    )
    text = replace_once(
        text,
        '        command: list[str],\n        *,\n        require_real_motion: bool = True,\n',
        '        command: list[str],\n        *,\n        capture_hz: int = DEFAULT_CAPTURE_HZ,\n        require_real_motion: bool = True,\n',
        "operator _start_motion signature",
    )
    text = replace_once(
        text,
        '        requested = self._validate_capture_mode(capture_mode)\n        self.ensure_runtime(requested)\n        self.stop()\n',
        '        requested = self._validate_capture_mode(capture_mode)\n        hz = self._validate_capture_hz(capture_hz)\n        self.ensure_runtime(requested, hz)\n        self.stop()\n',
        "operator _start_motion ensure hz",
    )
    text = replace_once(
        text,
        '                self._emit("info", f"capture: ALAP armed for STOP/FAULT -> {self._display_path(path) if path else \'-\'}")\n',
        '                self._emit(\n                    "info",\n                    f"capture: ALAP {hz} Hz armed for STOP/FAULT -> "\n                    f"{self._display_path(path) if path else \'-\'}",\n                )\n',
        "operator motion info hz",
    )
    text = replace_once(
        text,
        '            self._emit("info", "capture: FULL continuous recording")\n',
        '            self._emit("info", f"capture: FULL continuous recording @ {hz} Hz")\n',
        "operator motion full hz",
    )

    old_current = '''    def current_capture_mode(self) -> str | None:\n        try:\n            value = self.capture_mode_file.read_text(encoding="utf-8").strip()\n        except OSError:\n            value = ""\n        if value in CAPTURE_MODES:\n            return value\n        if self._runtime_pid() is not None:\n            return DEFAULT_CAPTURE_MODE\n        return None\n'''
    new_current = old_current + '''\n    def current_capture_hz(self) -> int | None:\n        try:\n            raw = self.capture_hz_file.read_text(encoding="utf-8").strip()\n            return self._validate_capture_hz(raw)\n        except (OSError, OperatorError, ValueError):\n            if self._runtime_pid() is not None:\n                return DEFAULT_CAPTURE_HZ\n            return None\n'''
    text = replace_once(text, old_current, new_current, "operator current_capture_hz")

    old_capture_start = '''    @_serialized_operator_transition\n    def capture_start(self, capture_mode: str | None = None) -> Path | None:\n        if capture_mode is None:\n            mode = self.current_capture_mode() if self._runtime_pid() is not None else DEFAULT_CAPTURE_MODE\n        else:\n            mode = self._validate_capture_mode(capture_mode)\n        assert mode is not None\n        self.ensure_runtime(mode)\n'''
    new_capture_start = '''    @_serialized_operator_transition\n    def capture_start(\n        self, capture_mode: str | None = None, capture_hz: int | None = None\n    ) -> Path | None:\n        if capture_mode is None:\n            mode = self.current_capture_mode() if self._runtime_pid() is not None else DEFAULT_CAPTURE_MODE\n        else:\n            mode = self._validate_capture_mode(capture_mode)\n        hz = (\n            self.current_capture_hz() if capture_hz is None and self._runtime_pid() is not None\n            else DEFAULT_CAPTURE_HZ if capture_hz is None\n            else self._validate_capture_hz(capture_hz)\n        )\n        assert mode is not None and hz is not None\n        self.ensure_runtime(mode, hz)\n'''
    text = replace_once(text, old_capture_start, new_capture_start, "operator capture_start hz")
    text = replace_once(
        text,
        '    def capture_stop(self) -> dict[str, object]:\n        path = self.current_capture_path()\n        mode = self.current_capture_mode()\n',
        '    def capture_stop(self) -> dict[str, object]:\n        path = self.current_capture_path()\n        mode = self.current_capture_mode()\n        hz = self.current_capture_hz()\n',
        "operator capture stop hz local",
    )
    text = replace_once(
        text,
        '    def capture_status(self) -> dict[str, object]:\n        path = self.current_capture_path()\n        mode = self.current_capture_mode()\n',
        '    def capture_status(self) -> dict[str, object]:\n        path = self.current_capture_path()\n        mode = self.current_capture_mode()\n        hz = self.current_capture_hz()\n',
        "operator capture status hz local",
    )
    text = replace_all_checked(
        text,
        '"mode": mode, "path":',
        '"mode": mode, "hz": hz, "path":',
        minimum=3,
        label="operator capture status return hz",
    )
    text = replace_once(
        text,
        '    def run_proba(self, *, capture_mode: str = DEFAULT_CAPTURE_MODE) -> None:\n        mode = self._validate_capture_mode(capture_mode)\n        self.ensure_runtime(mode)\n',
        '    def run_proba(\n        self, *, capture_mode: str = DEFAULT_CAPTURE_MODE, capture_hz: int = DEFAULT_CAPTURE_HZ\n    ) -> None:\n        mode = self._validate_capture_mode(capture_mode)\n        hz = self._validate_capture_hz(capture_hz)\n        self.ensure_runtime(mode, hz)\n',
        "operator proba hz",
    )
    text = replace_once(
        text,
        '    def run_runtime_session(self, capture: Path, mode: str) -> int:\n        mode = self._validate_capture_mode(mode)\n',
        '    def run_runtime_session(\n        self, capture: Path, mode: str, capture_hz: int = DEFAULT_CAPTURE_HZ\n    ) -> int:\n        mode = self._validate_capture_mode(mode)\n        hz = self._validate_capture_hz(capture_hz)\n',
        "operator run_runtime_session hz",
    )
    text = replace_once(
        text,
        '        command = [self.python, "v3_process_runtime.py", "--approval", "native-resident-v3"]\n',
        '        command = [self.python, "v3_process_runtime.py", "--approval", "native-resident-v3"]\n'
        '        if mode != "nincs":\n            command += ["--capture-hz", str(hz)]\n',
        "operator process runtime hz cli",
    )
    text = replace_once(
        text,
        '        self.runtime_start("alap")\n        self._wait_ready()\n',
        '        self.runtime_start("alap", self.current_capture_hz() or DEFAULT_CAPTURE_HZ)\n        self._wait_ready()\n',
        "operator rearm preserve hz",
    )
    text = replace_once(
        text,
        '    @staticmethod\n    def _validate_capture_mode(mode: str) -> str:\n        if mode not in CAPTURE_MODES:\n            raise OperatorError("capture mode must be one of: alap, full, nincs")\n        return mode\n',
        '    @staticmethod\n    def _validate_capture_mode(mode: str) -> str:\n        if mode not in CAPTURE_MODES:\n            raise OperatorError("capture mode must be one of: alap, full, nincs")\n        return mode\n\n'
        '    @staticmethod\n    def _validate_capture_hz(value: object) -> int:\n        try:\n            return validate_capture_hz(value)\n        except ValueError as exc:\n            raise OperatorError(str(exc)) from exc\n',
        "operator capture hz validator",
    )
    return text


def patch_v3_control(text: str) -> str:
    text = replace_once(
        text,
        "from v3.operator_controller import DEFAULT_CAPTURE_MODE, OperatorController\n",
        "from v3.capture_rate import DEFAULT_CAPTURE_HZ, validate_capture_hz\n"
        "from v3.operator_controller import DEFAULT_CAPTURE_MODE, OperatorController\n",
        "v3 control hz import",
    )
    text = replace_once(
        text,
        '        capture_mode = str(params.pop("capture_mode", DEFAULT_CAPTURE_MODE))\n',
        '        capture_mode = str(params.pop("capture_mode", DEFAULT_CAPTURE_MODE))\n'
        '        capture_hz = validate_capture_hz(params.pop("capture_hz", DEFAULT_CAPTURE_HZ))\n',
        "v3 control pop hz",
    )
    text = replace_all_checked(
        text,
        'capture_mode=capture_mode, **session',
        'capture_mode=capture_mode, capture_hz=capture_hz, **session',
        minimum=7,
        label="v3 control hz propagation",
    )
    return text


def patch_operator_adapter(text: str) -> str:
    text = replace_once(
        text,
        "from v3.operator_controller import DEFAULT_CAPTURE_MODE, OperatorController\n",
        "from v3.capture_rate import DEFAULT_CAPTURE_HZ, validate_capture_hz\n"
        "from v3.operator_controller import DEFAULT_CAPTURE_MODE, OperatorController\n",
        "operator adapter hz import",
    )
    text = replace_once(
        text,
        '            mode = str(params.pop("capture_mode", DEFAULT_CAPTURE_MODE))\n            self._reject_unknown(params)\n            return {"pid": self.controller.ensure_runtime(mode), "capture_mode": mode}\n',
        '            mode = str(params.pop("capture_mode", DEFAULT_CAPTURE_MODE))\n'
        '            hz = validate_capture_hz(params.pop("capture_hz", DEFAULT_CAPTURE_HZ))\n'
        '            self._reject_unknown(params)\n'
        '            return {\n'
        '                "pid": self.controller.ensure_runtime(mode, hz),\n'
        '                "capture_mode": mode,\n'
        '                "capture_hz": hz,\n'
        '            }\n',
        "operator adapter runtime hz",
    )
    text = replace_once(
        text,
        '            mode = params.pop("capture_mode", None)\n            self._reject_unknown(params)\n            path = self.controller.capture_start(None if mode is None else str(mode))\n',
        '            mode = params.pop("capture_mode", None)\n'
        '            hz_raw = params.pop("capture_hz", None)\n'
        '            hz = None if hz_raw is None else validate_capture_hz(hz_raw)\n'
        '            self._reject_unknown(params)\n'
        '            path = self.controller.capture_start(\n'
        '                None if mode is None else str(mode), hz\n'
        '            )\n',
        "operator adapter capture start hz",
    )
    text = replace_once(
        text,
        '            mode = str(params.pop("capture_mode", DEFAULT_CAPTURE_MODE))\n            self._reject_unknown(params)\n            self.controller.run_proba(capture_mode=mode)\n',
        '            mode = str(params.pop("capture_mode", DEFAULT_CAPTURE_MODE))\n'
        '            hz = validate_capture_hz(params.pop("capture_hz", DEFAULT_CAPTURE_HZ))\n'
        '            self._reject_unknown(params)\n'
        '            self.controller.run_proba(capture_mode=mode, capture_hz=hz)\n',
        "operator adapter proba hz",
    )
    return text


def patch_operator_cli(text: str) -> str:
    text = replace_once(
        text,
        "from v3.operator_controller import (\n",
        "from v3.capture_rate import CAPTURE_HZ_VALUES, DEFAULT_CAPTURE_HZ\n"
        "from v3.operator_controller import (\n",
        "operator cli capture import",
    )
    text = replace_once(
        text,
        '    worker.add_argument("--capture-mode", choices=tuple(sorted(CAPTURE_MODES)), required=True)\n',
        '    worker.add_argument("--capture-mode", choices=tuple(sorted(CAPTURE_MODES)), required=True)\n'
        '    worker.add_argument("--capture-hz", type=int, choices=CAPTURE_HZ_VALUES, default=DEFAULT_CAPTURE_HZ)\n',
        "operator cli worker hz arg",
    )
    text = replace_once(
        text,
        '            return controller.run_runtime_session(Path(args.capture_path), args.capture_mode)\n',
        '            return controller.run_runtime_session(\n'
        '                Path(args.capture_path), args.capture_mode, args.capture_hz\n'
        '            )\n',
        "operator cli worker hz call",
    )
    return text


def patch_v3_process_runtime(text: str) -> str:
    text = replace_once(
        text,
        "from v3.capture import CaptureSink, CaptureWindowConfig, TriggeredCaptureWorker\n",
        "from v3.capture import CaptureSink, CaptureWindowConfig, TriggeredCaptureWorker\n"
        "from v3.capture_rate import CAPTURE_HZ_VALUES, CONTROL_CAPTURE_HZ, DEFAULT_CAPTURE_HZ, validate_capture_hz\n",
        "process runtime capture hz import",
    )
    text = replace_once(
        text,
        '    run_hardware: Callable[..., ResidentRuntimeReport] = run_native_hardware_resident_control,\n',
        '    run_hardware: Callable[..., ResidentRuntimeReport] = run_native_hardware_resident_control,\n'
        '    capture_hz: int = CONTROL_CAPTURE_HZ,\n',
        "run process capture hz signature",
    )
    text = replace_once(
        text,
        '    if not callable(run_hardware):\n        raise TypeError("run_hardware must be callable")\n',
        '    if not callable(run_hardware):\n        raise TypeError("run_hardware must be callable")\n'
        '    resolved_capture_hz = validate_capture_hz(capture_hz)\n',
        "run process validate capture hz",
    )
    text = replace_once(
        text,
        '        if isinstance(capture_session, McapCaptureSession):\n',
        '        if capture_session is not None:\n'
        '            hardware_kwargs["record_observer_hz"] = resolved_capture_hz\n'
        '        if isinstance(capture_session, McapCaptureSession):\n',
        "run process hardware hz kwarg",
    )
    text = replace_once(
        text,
        '    parser.add_argument("--capture-ingress-capacity", type=int, default=256)\n',
        '    parser.add_argument("--capture-ingress-capacity", type=int, default=256)\n'
        '    parser.add_argument(\n'
        '        "--capture-hz", type=int, choices=CAPTURE_HZ_VALUES, default=DEFAULT_CAPTURE_HZ\n'
        '    )\n',
        "process parser capture hz",
    )
    text = replace_once(
        text,
        '                    require_raw_lidar_transport_end=True,\n',
        '                    require_raw_lidar_transport_end=True,\n'
        '                    tick_sample_hz=args.capture_hz,\n',
        "process mcap config hz",
    )
    text = replace_once(
        text,
        '            open_imu_device=open_imu_device,\n        )\n',
        '            open_imu_device=open_imu_device,\n'
        '            capture_hz=args.capture_hz,\n'
        '        )\n',
        "process run call hz",
    )
    return text


def patch_hardware_runtime(text: str) -> str:
    text = replace_once(
        text,
        '    record_observer: Callable[[CaptureRecord], None] | None = None,\n    raw_lidar_observer: Callable[[object | None], None] | None = None,\n',
        '    record_observer: Callable[[CaptureRecord], None] | None = None,\n'
        '    record_observer_hz: int = 50,\n'
        '    raw_lidar_observer: Callable[[object | None], None] | None = None,\n',
        "hardware runtime record hz signature",
    )
    text = replace_once(
        text,
        '            record_observer=record_observer,\n            timing_enabled=bool(\n',
        '            record_observer=record_observer,\n'
        '            record_observer_hz=record_observer_hz,\n'
        '            timing_enabled=bool(\n',
        "hardware runtime pass record hz",
    )
    return text


def patch_v3_runtime(text: str) -> str:
    text = replace_once(
        text,
        "from v3.contracts import LifecycleState, SafetyDecision, TickContext\n",
        "from v3.capture_rate import CONTROL_CAPTURE_HZ, validate_capture_hz\n"
        "from v3.contracts import LifecycleState, SafetyDecision, TickContext\n",
        "v3 runtime capture rate import",
    )
    text = replace_once(
        text,
        '    record_observer: Callable[[CaptureRecord], None] | None = None,\n    timing_enabled: bool = False,\n',
        '    record_observer: Callable[[CaptureRecord], None] | None = None,\n'
        '    record_observer_hz: int = CONTROL_CAPTURE_HZ,\n'
        '    timing_enabled: bool = False,\n',
        "v3 runtime resident signature hz",
    )
    text = replace_once(
        text,
        '    if record_observer is not None and not callable(record_observer):\n        raise TypeError("record_observer must be callable or None")\n',
        '    if record_observer is not None and not callable(record_observer):\n        raise TypeError("record_observer must be callable or None")\n'
        '    resolved_record_hz = validate_capture_hz(record_observer_hz)\n'
        '    record_period_ns = 1_000_000_000 // resolved_record_hz\n',
        "v3 runtime validate hz",
    )
    text = replace_once(
        text,
        '    last_checkpoint_ns: int | None = None\n    last_result: TickResult | None = None\n',
        '    last_checkpoint_ns: int | None = None\n'
        '    next_record_observer_ns: int | None = None\n'
        '    last_result: TickResult | None = None\n',
        "v3 runtime next capture deadline",
    )
    old_observer = '''            observer_started_ns = time.perf_counter_ns()\n            if record_observer is not None:\n                checkpoint_started_ns = time.perf_counter_ns()\n                checkpoint_created = False\n                if (\n                    isinstance(record, ExecutionRecord)\n                    and record.result.trace.fault_layer is None\n                    and (\n                        last_checkpoint_ns is None\n                        or context.monotonic_ns - last_checkpoint_ns\n                        >= REPLAY_STATE_CHECKPOINT_INTERVAL_NS\n                    )\n                ):\n                    record = replace(\n                        record,\n                        state_checkpoint_after=runtime.checkpoint(),\n                    )\n                    last_checkpoint_ns = context.monotonic_ns\n                    checkpoint_created = True\n                if timing is not None and checkpoint_created:\n                    timing.observe_control_phase(\n                        "CAPTURE_CHECKPOINT",\n                        time.perf_counter_ns() - checkpoint_started_ns,\n                    )\n                capture_started_ns = time.perf_counter_ns()\n                record_observer(record)\n                if timing is not None:\n                    timing.observe_control_phase(\n                        "CAPTURE_TAP",\n                        time.perf_counter_ns() - capture_started_ns,\n                    )\n'''
    new_observer = '''            observer_started_ns = time.perf_counter_ns()\n            force_capture_record = bool(\n                last_result.trace.fault_layer is not None\n                or last_result.final_actuation.safety_decision is SafetyDecision.FAULT\n            )\n            scheduled_capture_record = bool(\n                record_observer is not None\n                and (\n                    resolved_record_hz == CONTROL_CAPTURE_HZ\n                    or next_record_observer_ns is None\n                    or context.monotonic_ns >= next_record_observer_ns\n                )\n            )\n            if record_observer is not None and (scheduled_capture_record or force_capture_record):\n                checkpoint_started_ns = time.perf_counter_ns()\n                checkpoint_created = False\n                if (\n                    isinstance(record, ExecutionRecord)\n                    and record.result.trace.fault_layer is None\n                    and (\n                        last_checkpoint_ns is None\n                        or context.monotonic_ns - last_checkpoint_ns\n                        >= REPLAY_STATE_CHECKPOINT_INTERVAL_NS\n                    )\n                ):\n                    record = replace(\n                        record,\n                        state_checkpoint_after=runtime.checkpoint(),\n                    )\n                    last_checkpoint_ns = context.monotonic_ns\n                    checkpoint_created = True\n                if timing is not None and checkpoint_created:\n                    timing.observe_control_phase(\n                        "CAPTURE_CHECKPOINT",\n                        time.perf_counter_ns() - checkpoint_started_ns,\n                    )\n                capture_started_ns = time.perf_counter_ns()\n                record_observer(record)\n                if timing is not None:\n                    timing.observe_control_phase(\n                        "CAPTURE_TAP",\n                        time.perf_counter_ns() - capture_started_ns,\n                    )\n                if scheduled_capture_record and resolved_record_hz != CONTROL_CAPTURE_HZ:\n                    if next_record_observer_ns is None:\n                        next_record_observer_ns = context.monotonic_ns + record_period_ns\n                    else:\n                        missed = max(0, context.monotonic_ns - next_record_observer_ns) // record_period_ns\n                        next_record_observer_ns += (missed + 1) * record_period_ns\n'''
    text = replace_once(text, old_observer, new_observer, "v3 runtime normal capture rate gate")
    # Owned wrapper signature and forwarding.
    text = replace_once(
        text,
        '    record_observer: Callable[[CaptureRecord], None] | None = None,\n    timing_enabled: bool = False,\n    trajectory_rollout_backend: object | None = None,\n',
        '    record_observer: Callable[[CaptureRecord], None] | None = None,\n'
        '    record_observer_hz: int = CONTROL_CAPTURE_HZ,\n'
        '    timing_enabled: bool = False,\n'
        '    trajectory_rollout_backend: object | None = None,\n',
        "v3 runtime owned signature hz",
    )
    text = replace_once(
        text,
        '            record_observer=record_observer,\n            timing_enabled=timing_enabled,\n',
        '            record_observer=record_observer,\n'
        '            record_observer_hz=record_observer_hz,\n'
        '            timing_enabled=timing_enabled,\n',
        "v3 runtime owned forward hz",
    )
    return text


def patch_mcap_capture(text: str) -> str:
    text = replace_once(
        text,
        "from .capture_compaction import compact_checkpoint_row, compact_tick_row\n",
        "from .capture_compaction import compact_checkpoint_row, compact_tick_row\n"
        "from .capture_rate import CONTROL_CAPTURE_HZ, validate_capture_hz\n",
        "mcap capture rate import",
    )
    text = replace_once(
        text,
        '    require_raw_lidar_transport_end: bool = False\n',
        '    require_raw_lidar_transport_end: bool = False\n'
        '    tick_sample_hz: int = CONTROL_CAPTURE_HZ\n',
        "mcap config tick hz field",
    )
    text = replace_once(
        text,
        '        if type(self.require_raw_lidar_transport_end) is not bool:\n            raise TypeError("require_raw_lidar_transport_end must be bool")\n\n',
        '        if type(self.require_raw_lidar_transport_end) is not bool:\n            raise TypeError("require_raw_lidar_transport_end must be bool")\n'
        '        validate_capture_hz(self.tick_sample_hz)\n\n',
        "mcap config validate hz",
    )
    text = replace_once(
        text,
        '            if previous is not None and tick_id != previous + 1:\n                self._captured_tick_gaps.append((previous, tick_id))\n                self._integrity_reasons.add("CAPTURED_TICK_SEQUENCE_GAP")\n',
        '            if previous is not None and tick_id <= previous:\n'
        '                self._captured_tick_gaps.append((previous, tick_id))\n'
        '                self._integrity_reasons.add("CAPTURED_TICK_SEQUENCE_NON_MONOTONIC")\n'
        '            elif previous is not None and tick_id != previous + 1:\n'
        '                self._captured_tick_gaps.append((previous, tick_id))\n'
        '                if self._config.tick_sample_hz == CONTROL_CAPTURE_HZ:\n'
        '                    self._integrity_reasons.add("CAPTURED_TICK_SEQUENCE_GAP")\n',
        "mcap sampled tick gaps",
    )
    text = replace_once(
        text,
        '                "compression": "none",\n',
        '                "compression": "none",\n'
        '                "tick_sample_hz": str(self._config.tick_sample_hz),\n',
        "mcap metadata sample hz",
    )
    old_complete = '''        replay_integrity_reasons = sorted(\n            reason for reason in self._integrity_reasons if not _is_raw_integrity_reason(reason)\n        )\n        replay_complete = not replay_integrity_reasons\n        raw_evidence_complete = not raw_integrity_reasons\n        complete = replay_complete and raw_evidence_complete\n'''
    new_complete = '''        replay_integrity_reasons = sorted(\n            reason for reason in self._integrity_reasons if not _is_raw_integrity_reason(reason)\n        )\n        sample_complete = not replay_integrity_reasons\n        sampled_tick_stream = self._config.tick_sample_hz != CONTROL_CAPTURE_HZ\n        replay_complete = sample_complete and not sampled_tick_stream\n        raw_evidence_complete = not raw_integrity_reasons\n        complete = sample_complete and raw_evidence_complete\n        if sampled_tick_stream:\n            integrity_warnings.append("TICK_STREAM_SAMPLED_NOT_REPLAY_COMPLETE")\n'''
    text = replace_once(text, old_complete, new_complete, "mcap sampled completeness")
    text = replace_once(
        text,
        '            "complete": complete,\n            "replay_complete": replay_complete,\n',
        '            "complete": complete,\n'
        '            "sample_complete": sample_complete,\n'
        '            "tick_sample_hz": self._config.tick_sample_hz,\n'
        '            "replay_complete": replay_complete,\n',
        "mcap final integrity hz",
    )
    text = replace_once(
        text,
        '            "capture_mode": self._config.mode,\n',
        '            "capture_mode": self._config.mode,\n'
        '            "capture_hz": self._config.tick_sample_hz,\n',
        "mcap final event hz",
    )
    text = replace_once(
        text,
        '                "complete": "true" if complete else "false",\n                "replay_complete": "true" if replay_complete else "false",\n',
        '                "complete": "true" if complete else "false",\n'
        '                "sample_complete": "true" if sample_complete else "false",\n'
        '                "tick_sample_hz": str(self._config.tick_sample_hz),\n'
        '                "replay_complete": "true" if replay_complete else "false",\n',
        "mcap final metadata hz",
    )
    return text


def patch_mcap_reader(text: str) -> str:
    text = replace_once(
        text,
        '        digest = hashlib.sha256()\n        final: dict[str, object] | None = None\n',
        '        digest = hashlib.sha256()\n'
        '        capture_meta = self.latest_metadata("r2b4.capture") or {}\n'
        '        try:\n'
        '            tick_sample_hz = int(capture_meta.get("tick_sample_hz", "50"))\n'
        '        except (TypeError, ValueError) as exc:\n'
        '            raise McapReadError("invalid tick_sample_hz metadata") from exc\n'
        '        if tick_sample_hz not in {1, 5, 10, 50}:\n'
        '            raise McapReadError("invalid tick_sample_hz metadata")\n'
        '        sampled_tick_stream = tick_sample_hz != 50\n'
        '        final: dict[str, object] | None = None\n',
        "mcap reader sample metadata",
    )
    text = replace_once(
        text,
        '                if last_tick is not None and tick != last_tick + 1:\n                    raise McapReadError("CAPTURE_INCOMPLETE: captured tick gap")\n                last_tick = tick\n',
        '                if last_tick is not None and tick <= last_tick:\n'
        '                    raise McapReadError("CAPTURE_INCOMPLETE: non-monotonic captured tick")\n'
        '                if (\n'
        '                    last_tick is not None\n'
        '                    and not sampled_tick_stream\n'
        '                    and tick != last_tick + 1\n'
        '                ):\n'
        '                    raise McapReadError("CAPTURE_INCOMPLETE: captured tick gap")\n'
        '                last_tick = tick\n',
        "mcap reader sampled gap acceptance",
    )
    old_required = '''        replay_complete = (\n            integrity.get("replay_complete", integrity.get("complete")) is True\n            and metadata.get("replay_complete", metadata.get("complete")) == "true"\n        )\n        raw_complete = (\n            integrity.get("raw_evidence_complete", integrity.get("complete")) is True\n            and metadata.get("raw_evidence_complete", metadata.get("complete")) == "true"\n        )\n        required_complete = replay_complete and (raw_complete if require_raw_evidence else True)\n'''
    new_required = '''        replay_complete = (\n            integrity.get("replay_complete", integrity.get("complete")) is True\n            and metadata.get("replay_complete", metadata.get("complete")) == "true"\n        )\n        sample_complete = (\n            integrity.get("sample_complete", integrity.get("complete")) is True\n            and metadata.get("sample_complete", metadata.get("complete")) == "true"\n        )\n        if integrity.get("tick_sample_hz", tick_sample_hz) != tick_sample_hz:\n            raise McapReadError("capture tick_sample_hz mismatch")\n        if metadata.get("tick_sample_hz", str(tick_sample_hz)) != str(tick_sample_hz):\n            raise McapReadError("capture tick_sample_hz mismatch")\n        raw_complete = (\n            integrity.get("raw_evidence_complete", integrity.get("complete")) is True\n            and metadata.get("raw_evidence_complete", metadata.get("complete")) == "true"\n        )\n        core_complete = sample_complete if sampled_tick_stream else replay_complete\n        required_complete = core_complete and (raw_complete if require_raw_evidence else True)\n'''
    text = replace_once(text, old_required, new_required, "mcap reader sampled completeness")
    return text


def patch_process_sidecars(text: str) -> str:
    text = replace_once(
        text,
        '                    evidence = postprocess_capture(\n                        result.path,\n                        project_root=Path(project_root),\n                    )\n',
        '                    evidence = postprocess_capture(\n'
        '                        result.path,\n'
        '                        project_root=Path(project_root),\n'
        '                        replay_mode=(\n'
        '                            "incident" if config.tick_sample_hz == 50 else "off"\n'
        '                        ),\n'
        '                    )\n',
        "capture sidecar sampled replay policy",
    )
    return text


def patch_test_hub_runtime(text: str) -> str:
    text = replace_once(
        text,
        '    project_root: str | Path,\n) -> dict[str, object]:\n',
        '    project_root: str | Path,\n    replay_mode: str = "incident",\n) -> dict[str, object]:\n',
        "test hub handoff signature",
    )
    text = replace_once(
        text,
        '    output_dir = capture.with_suffix(".evidence")\n    command = [\n',
        '    output_dir = capture.with_suffix(".evidence")\n'
        '    replay = str(replay_mode).strip().lower()\n'
        '    if replay not in {"off", "incident", "full"}:\n'
        '        raise ValueError("replay_mode must be off, incident or full")\n'
        '    command = [\n',
        "test hub validate replay mode",
    )
    text = replace_once(
        text,
        '        "--replay",\n        "incident",\n',
        '        "--replay",\n        replay,\n',
        "test hub replay arg",
    )
    return text


PATCHERS = {
    "v3/interface_cli.py": patch_interface_cli,
    "v3/operator_controller.py": patch_operator_controller,
    "v3/adapters/v3_control.py": patch_v3_control,
    "v3/adapters/operator.py": patch_operator_adapter,
    "v3/operator_cli.py": patch_operator_cli,
    "v3_process_runtime.py": patch_v3_process_runtime,
    "v3_hardware_runtime.py": patch_hardware_runtime,
    "v3_runtime.py": patch_v3_runtime,
    "v3/mcap_capture.py": patch_mcap_capture,
    "v3/mcap_reader.py": patch_mcap_reader,
    "v3/process_sidecars.py": patch_process_sidecars,
    "v3/test_hub_runtime.py": patch_test_hub_runtime,
}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", nargs="?", default="/home/alba/project_r2b4")
    parser.add_argument("--allow-source-drift", action="store_true")
    parser.add_argument("--run-tests", action="store_true", help="run the small capture-Hz regression after applying")
    args = parser.parse_args()
    root = Path(args.root).resolve()
    if not (root / "v3").is_dir() or not (root / "v3_process_runtime.py").is_file():
        raise RuntimeError(f"not an R2B4 repository: {root}")
    head = git_head(root)
    if head != EXPECTED_HEAD:
        print(
            f"NOTE: source HEAD is {head or 'UNKNOWN'} "
            f"(package basis was {EXPECTED_HEAD}); applying by source anchors."
        )

    transformed: dict[str, str] = {}
    for rel in PATCHED:
        path = root / rel
        if not path.is_file():
            raise RuntimeError(f"missing target file: {rel}")
        original = path.read_text(encoding="utf-8")
        updated = PATCHERS[rel](original)
        compile(updated, str(path), "exec")
        transformed[rel] = updated

    for rel in PAYLOADS:
        source = PAYLOAD_ROOT / rel
        if not source.is_file():
            raise RuntimeError(f"package payload missing: {rel}")
        content = source.read_text(encoding="utf-8")
        if rel.endswith(".py"):
            compile(content, str(root / rel), "exec")
        transformed[rel] = content

    stamp = time.strftime("%Y%m%d_%H%M%S")
    backup = root / "runtime" / "upgrade_backups" / f"{PACKAGE}_{stamp}"
    backup.mkdir(parents=True, exist_ok=False)
    written: list[Path] = []
    try:
        for rel, content in transformed.items():
            target = root / rel
            if target.exists():
                dst = backup / rel
                dst.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(target, dst)
            atomic_write(target, content)
            written.append(target)

        if args.run_tests:
            completed = subprocess.run(
                [sys.executable, "-m", "pytest", "-q", "tests/test_v3_capture_hz.py"],
                cwd=root,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=False,
            )
            print(completed.stdout, end="")
            if completed.returncode != 0:
                raise RuntimeError(
                    f"targeted regression failed with exit code {completed.returncode}"
                )
    except BaseException:
        # Roll back all pre-existing files; remove newly introduced payloads.
        for rel in transformed:
            target = root / rel
            saved = backup / rel
            if saved.exists():
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(saved, target)
            elif target.exists():
                target.unlink()
        raise

    print(f"APPLIED {PACKAGE}")
    print(f"backup: {backup}")
    print("capture default: 10 Hz")
    print("capture choices: 50, 10, 5, 1 Hz")
    print("examples: ./r rc 30 c 50   |   ./r rc 30   |   ./r rc 30 c 5")
    print("50 Hz = deterministic replay-grade; 10/5/1 Hz = sampled evidence, replay auto-OFF")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
