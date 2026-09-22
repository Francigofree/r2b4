"""Tiny runtime-to-Test-Hub handoff.

The robot runtime imports only this stdlib-only helper after hardware ownership
has ended.  Test Hub itself runs in a separate Python process, so analysis code
cannot become part of the motor/control authority path.
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


def postprocess_capture(
    capture_path: str | Path,
    *,
    project_root: str | Path,
) -> dict[str, object]:
    capture = Path(capture_path).resolve()
    root = Path(project_root).resolve()
    output_dir = capture.with_suffix(".evidence")
    command = [
        sys.executable,
        "-m",
        "v3.test_hub",
        "run",
        str(capture),
        "--output-dir",
        str(output_dir),
        "--replay",
        "incident",
        "--pytest",
        "off",
    ]
    try:
        completed = subprocess.run(
            command,
            cwd=root,
            text=True,
            capture_output=True,
            check=False,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return {
            "status": "ERROR",
            "capture": str(capture),
            "output_dir": str(output_dir),
            "error": f"Test Hub process failed to start: {exc}",
        }

    payload: dict[str, object]
    try:
        decoded = json.loads(completed.stdout)
        payload = dict(decoded) if isinstance(decoded, dict) else {}
    except json.JSONDecodeError:
        payload = {}
    if not payload:
        payload = {
            "status": "ERROR",
            "capture": str(capture),
            "output_dir": str(output_dir),
            "error": "Test Hub returned no valid JSON result",
        }
    payload["process_exit_code"] = completed.returncode
    if completed.stderr:
        payload["stderr_tail"] = completed.stderr[-20_000:]
    return payload


__all__ = ["postprocess_capture"]
