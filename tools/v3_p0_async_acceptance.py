#!/usr/bin/env python3
"""P0-4 acceptance gate for the R2B4 asynchronous resident runtime.

This is an orchestration/verification tool only.  It creates no command,
mission, safety, sensor or motor authority.

The live gate starts the normal resident runtime twice without publishing any
motion command:
    1. capture OFF (nincs)
    2. capture FULL

For both sessions it requires IDLE/SAFE-LOW behavior, verifies the configured
CPU policy when affinity is enabled (or records scheduler-managed mode when disabled),
reuses the repository's canonical timing audit,
and for FULL capture runs the canonical Test Hub replay.  The final machine-readable evidence is written under runtime/diag.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence


SCHEMA = "R2B4_P0_ASYNC_ACCEPTANCE_V1"
DEFAULT_IDLE_SECONDS = 10.0
DEFAULT_POLL_SECONDS = 0.10


REQUIRED_ANCHORS = (
    "AGENTS.md",
    "conf/vezerles.json",
    "v3/operator_controller.py",
    "v3/test_hub.py",
    "tools/v3_performance_audit.py",
)


class AcceptanceError(RuntimeError):
    """A P0-4 acceptance invariant failed."""


@dataclass(frozen=True, slots=True)
class CommandResult:
    argv: tuple[str, ...]
    returncode: int
    elapsed_s: float

    def as_dict(self) -> dict[str, object]:
        return {
            "argv": list(self.argv),
            "returncode": self.returncode,
            "elapsed_s": round(self.elapsed_s, 6),
        }


def _run(
    argv: Sequence[str],
    *,
    cwd: Path,
    check: bool = True,
) -> CommandResult:
    rendered = " ".join(str(item) for item in argv)
    print(f"$ {rendered}", flush=True)
    started = time.monotonic()
    proc = subprocess.run(tuple(str(item) for item in argv), cwd=cwd, check=False)
    result = CommandResult(
        argv=tuple(str(item) for item in argv),
        returncode=int(proc.returncode),
        elapsed_s=time.monotonic() - started,
    )
    if check and proc.returncode != 0:
        raise AcceptanceError(
            f"command failed rc={proc.returncode}: {rendered}"
        )
    return result


def _json_object(path: Path) -> dict[str, object]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise AcceptanceError(f"cannot read valid JSON: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise AcceptanceError(f"JSON root must be an object: {path}")
    return value


def _write_json(path: Path, payload: Mapping[str, object]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(
        f".{path.name}.tmp.{os.getpid()}.{time.monotonic_ns()}"
    )
    temporary.write_text(
        json.dumps(
            dict(payload),
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)
    return path


def _project_root(value: str | Path | None) -> Path:
    if value is not None:
        root = Path(value).expanduser().resolve()
    else:
        root = Path(__file__).resolve().parents[1]
    missing = [relative for relative in REQUIRED_ANCHORS if not (root / relative).is_file()]
    if missing:
        raise AcceptanceError(
            "not an R2B4 production root; missing: " + ", ".join(missing)
        )
    return root


def _idle_sample_errors(status: Mapping[str, object]) -> tuple[str, ...]:
    errors: list[str] = []
    if status.get("state") != "RUNNING":
        errors.append(f"state={status.get('state')!r}")
    if status.get("fault_layer") is not None:
        errors.append(f"fault_layer={status.get('fault_layer')!r}")
    if status.get("safety_decision") == "FAULT":
        errors.append("safety_decision=FAULT")
    if status.get("enabled") is not False:
        errors.append(f"enabled={status.get('enabled')!r}")
    for key in ("left_output", "right_output"):
        raw = status.get(key)
        if not isinstance(raw, (int, float)) or isinstance(raw, bool):
            errors.append(f"{key}={raw!r}")
            continue
        if not math.isfinite(float(raw)) or abs(float(raw)) > 1e-12:
            errors.append(f"{key}={raw!r}")
    return tuple(errors)


def _terminal_report(status: Mapping[str, object]) -> dict[str, object]:
    if status.get("state") != "STOPPED":
        raise AcceptanceError(
            f"terminal resident status is not STOPPED: {status.get('state')!r}"
        )
    report = status.get("report")
    if not isinstance(report, dict):
        raise AcceptanceError("terminal resident status has no report")
    if report.get("status") != "PASS":
        raise AcceptanceError(f"resident report status={report.get('status')!r}")
    if report.get("termination_class") != "SHUTDOWN_SAFE_LOW":
        raise AcceptanceError(
            f"termination_class={report.get('termination_class')!r}"
        )
    if report.get("fault_layer") is not None:
        raise AcceptanceError(
            f"terminal fault_layer={report.get('fault_layer')!r}"
        )
    timing = report.get("timing")
    if not isinstance(timing, dict):
        raise AcceptanceError("terminal report has no timing evidence")
    if int(timing.get("tick_count", 0)) <= 0:
        raise AcceptanceError("timing evidence contains no ticks")
    _timing_view(report)
    return dict(report)


def _timing_view(report: Mapping[str, object]) -> dict[str, int]:
    timing = report.get("timing")
    if not isinstance(timing, Mapping):
        raise AcceptanceError("report has no timing object")
    keys = (
        "target_period_ns",
        "tick_count",
        "period_count",
        "period_mean_ns",
        "period_p50_ns",
        "period_p95_ns",
        "period_p99_ns",
        "period_max_ns",
        "period_over_25ms_count",
        "period_over_40ms_count",
        "lateness_p99_ns",
        "lateness_max_ns",
        "lateness_over_2ms_count",
        "control_p99_ns",
        "control_max_ns",
        "observer_p99_ns",
        "observer_max_ns",
        "work_p99_ns",
        "work_max_ns",
        "work_over_period_count",
    )
    result: dict[str, int] = {}
    for key in keys:
        raw = timing.get(key, 0)
        if not isinstance(raw, int) or isinstance(raw, bool) or raw < 0:
            raise AcceptanceError(f"invalid timing field {key}={raw!r}")
        result[key] = raw
    return result


def _capture_delta(
    off_report: Mapping[str, object],
    full_report: Mapping[str, object],
) -> dict[str, object]:
    off = _timing_view(off_report)
    full = _timing_view(full_report)
    rows: dict[str, object] = {}
    for key in (
        "period_p95_ns",
        "period_p99_ns",
        "period_max_ns",
        "lateness_p99_ns",
        "control_p99_ns",
        "observer_p99_ns",
        "work_p99_ns",
        "work_max_ns",
    ):
        before = off[key]
        after = full[key]
        rows[key] = {
            "capture_off_ns": before,
            "capture_full_ns": after,
            "delta_ns": after - before,
            "ratio": None if before == 0 else round(after / before, 6),
        }
    rows["period_over_40ms_count"] = {
        "capture_off": off["period_over_40ms_count"],
        "capture_full": full["period_over_40ms_count"],
        "delta": (
            full["period_over_40ms_count"] - off["period_over_40ms_count"]
        ),
    }
    rows["work_over_period_count"] = {
        "capture_off": off["work_over_period_count"],
        "capture_full": full["work_over_period_count"],
        "delta": full["work_over_period_count"] - off["work_over_period_count"],
    }
    return rows



def _affinity_audit(root: Path, runtime_pid: int, *, require_capture: bool = False) -> dict[str, object]:
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    from v3.runtime_performance import load_runtime_affinity_config
    from v3.affinity_diagnostics import audit_affinity
    config = load_runtime_affinity_config(root / "conf" / "vezerles.json")
    if not config.enabled:
        return {
            "status": "NOT_APPLICABLE",
            "mode": "SCHEDULER_MANAGED",
            "reason": "runtime_affinity.enabled=false; R2B4 applies no CPU pinning",
            "runtime_pid": runtime_pid,
            "capture_required": require_capture,
        }
    roles = ("encoder", "imu", "lidar_owner", "lidar_matcher", "vision", "planner",
             "status", "command", "l0_encoder", "l0_imu", "l0_lidar", "l0_aux")
    result = audit_affinity(config, runtime_pid, project_root=root,
                            required_roles=roles + (("capture",) if require_capture else ()))
    if result["status"] != "PASS":
        raise AcceptanceError("affinity audit failed: " + json.dumps(result, sort_keys=True))
    return result


def run_offline(root: Path, *, full_pytest: bool) -> dict[str, object]:
    commands = [_run(["./r", "test"], cwd=root),
                _run(["./r", "test", "process"], cwd=root),
                _run(["./r", "test", "async"], cwd=root)]
    if full_pytest:
        commands.append(_run(["./r", "test", "full"], cwd=root))
    return {
        "status": "PASS",
        "full_pytest": full_pytest,
        "commands": [item.as_dict() for item in commands],
    }


def _wait_terminal_status(root: Path, timeout_s: float = 5.0) -> dict[str, object]:
    path = root / "runtime" / "v3_status.json"
    deadline = time.monotonic() + timeout_s
    last: dict[str, object] | None = None
    while time.monotonic() < deadline:
        if path.is_file():
            last = _json_object(path)
            if last.get("state") in {"STOPPED", "ERROR"}:
                return last
        time.sleep(0.05)
    raise AcceptanceError(
        f"resident terminal status was not published within {timeout_s:g}s: {last}"
    )


def _live_idle_session(
    root: Path,
    *,
    mode: str,
    seconds: float,
    poll_seconds: float,
    stamp: str,
) -> dict[str, object]:
    from v3.operator_controller import OperatorController

    if mode not in {"nincs", "full"}:
        raise ValueError("mode must be 'nincs' or 'full'")
    controller = OperatorController(project_root=root)
    initial = controller.snapshot()
    if initial.runtime_running:
        raise AcceptanceError(
            "resident runtime is already running; stop it before P0-4 acceptance"
        )

    samples = 0
    ready_samples = 0
    capture_path: Path | None = None
    started = False
    live_affinity: dict[str, object] | None = None
    try:
        controller.runtime_start(mode, capture_hz=50)
        started = True
        capture_path = controller.current_capture_path()

        # Allow child workers to settle before checking /proc affinity.
        settle_deadline = time.monotonic() + min(1.0, seconds / 4.0)
        while time.monotonic() < settle_deadline:
            time.sleep(min(0.05, poll_seconds))

        runtime_pid = controller.snapshot().runtime_pid
        if runtime_pid is None:
            raise AcceptanceError(f"{mode}: runtime PID unavailable for affinity audit")
        live_affinity = _affinity_audit(root, runtime_pid, require_capture=mode == "full")

        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            status = controller.live_runtime_status(max_age_ns=750_000_000)
            if status is None:
                raise AcceptanceError(
                    f"{mode}: live runtime status unavailable or stale"
                )
            errors = _idle_sample_errors(status)
            if errors:
                raise AcceptanceError(
                    f"{mode}: IDLE/SAFE-LOW invariant failed: "
                    + ", ".join(errors)
                )
            samples += 1
            if status.get("ready_for_active") is True:
                ready_samples += 1
            time.sleep(poll_seconds)
        if samples < 5:
            raise AcceptanceError(f"{mode}: too few live samples ({samples})")
        if ready_samples == 0:
            raise AcceptanceError(f"{mode}: runtime never became ready_for_active")
    finally:
        if started:
            controller.runtime_stop()

    terminal = _wait_terminal_status(root)
    report = _terminal_report(terminal)

    timing_audit = _run(
        [
            sys.executable,
            "tools/v3_performance_audit.py",
            "report",
            "--status",
            "runtime/v3_status.json",
        ],
        cwd=root,
    )

    replay: dict[str, object] | None = None
    if mode == "full":
        if capture_path is None or not capture_path.is_file():
            raise AcceptanceError(
                f"FULL capture did not publish a finalized MCAP: {capture_path}"
            )
        from v3.test_hub import inspect_mcap
        inspected = inspect_mcap(capture_path)
        profile = inspected.get("analysis_profile", {})
        if (inspected.get("status") != "PASS"
                or profile.get("tick_sample_hz") != 50
                or profile.get("exact_replay_applicable") is not True):
            raise AcceptanceError("FULL acceptance requires intact, replay-complete 50 Hz capture")
        diag_dir = root / "runtime" / "diag"
        diag_dir.mkdir(parents=True, exist_ok=True)
        replay_path = diag_dir / f"p0_async_acceptance_{stamp}_replay.json"
        replay_cmd = _run(
            [
                sys.executable,
                "-m",
                "v3.test_hub",
                "replay",
                str(capture_path),
                "--output",
                str(replay_path),
                "--project-root",
                str(root),
            ],
            cwd=root,
        )
        verify_cmd = _run(
            [
                sys.executable,
                "-m",
                "v3.test_hub",
                "verify-result",
                str(replay_path),
            ],
            cwd=root,
        )
        replay_payload = _json_object(replay_path)
        if replay_payload.get("status") != "MATCH":
            raise AcceptanceError(
                f"canonical replay status={replay_payload.get('status')!r}"
            )
        replay = {
            "capture_path": str(capture_path),
            "capture_integrity": inspected["status"],
            "capture_hz": profile["tick_sample_hz"],
            "result_path": str(replay_path),
            "status": replay_payload.get("status"),
            "first_divergence": replay_payload.get("first_divergence"),
            "replay_command": replay_cmd.as_dict(),
            "verify_command": verify_cmd.as_dict(),
        }

    return {
        "status": "PASS",
        "capture_mode": mode,
        "duration_s": seconds,
        "poll_s": poll_seconds,
        "samples": samples,
        "ready_samples": ready_samples,
        "capture_path": str(capture_path) if capture_path is not None else None,
        "affinity_audit": live_affinity,
        "timing_audit": timing_audit.as_dict(),
        "report": report,
        "timing": _timing_view(report),
        "replay": replay,
    }


def run_live(
    root: Path,
    *,
    seconds: float,
    poll_seconds: float,
) -> dict[str, object]:
    if not math.isfinite(seconds) or seconds < 2.0:
        raise AcceptanceError("seconds must be finite and >= 2")
    if not math.isfinite(poll_seconds) or not 0.02 <= poll_seconds <= 1.0:
        raise AcceptanceError("poll-seconds must be within [0.02, 1.0]")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    off = _live_idle_session(
        root,
        mode="nincs",
        seconds=seconds,
        poll_seconds=poll_seconds,
        stamp=stamp,
    )
    full = _live_idle_session(
        root,
        mode="full",
        seconds=seconds,
        poll_seconds=poll_seconds,
        stamp=stamp,
    )
    comparison = _capture_delta(off["report"], full["report"])
    payload: dict[str, object] = {
        "schema": SCHEMA,
        "status": "PASS",
        "created_at_utc": time.strftime(
            "%Y-%m-%dT%H:%M:%SZ", time.gmtime()
        ),
        "project_root": str(root),
        "motion_command_published": False,
        "sessions": {
            "capture_off": off,
            "capture_full": full,
        },
        "capture_full_minus_off": comparison,
    }
    path = root / "runtime" / "diag" / f"p0_async_acceptance_{stamp}.json"
    _write_json(path, payload)
    payload["evidence_path"] = str(path)
    print(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True))
    return payload


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root")
    commands = parser.add_subparsers(dest="command", required=True)

    offline = commands.add_parser(
        "offline",
        help="run targeted async/process tests; optionally the full pytest suite",
    )
    offline.add_argument("--full-pytest", action="store_true")

    live = commands.add_parser(
        "live",
        help="run capture OFF/FULL IDLE hardware acceptance; publishes no motion command",
    )
    live.add_argument("--seconds", type=float, default=DEFAULT_IDLE_SECONDS)
    live.add_argument("--poll-seconds", type=float, default=DEFAULT_POLL_SECONDS)

    all_parser = commands.add_parser(
        "all",
        help="run targeted tests, full pytest, then the IDLE hardware/replay gate",
    )
    all_parser.add_argument("--seconds", type=float, default=DEFAULT_IDLE_SECONDS)
    all_parser.add_argument("--poll-seconds", type=float, default=DEFAULT_POLL_SECONDS)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        root = _project_root(args.project_root)
        sys.path.insert(0, str(root))
        from v3.runtime_performance import apply_host_affinity
        apply_host_affinity(root, "diagnostics")
        if args.command == "offline":
            payload = {
                "schema": SCHEMA,
                **run_offline(root, full_pytest=bool(args.full_pytest)),
            }
        elif args.command == "live":
            payload = run_live(
                root,
                seconds=float(args.seconds),
                poll_seconds=float(args.poll_seconds),
            )
        else:
            offline = run_offline(root, full_pytest=True)
            live = run_live(
                root,
                seconds=float(args.seconds),
                poll_seconds=float(args.poll_seconds),
            )
            payload = {
                "schema": SCHEMA,
                "status": "PASS",
                "offline": offline,
                "live": live,
            }
        if args.command != "live":
            print(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True))
        return 0
    except (
        AcceptanceError,
        OSError,
        RuntimeError,
        TypeError,
        ValueError,
    ) as exc:
        print(
            json.dumps(
                {
                    "schema": SCHEMA,
                    "status": "FAIL",
                    "error_type": type(exc).__name__,
                    "error": str(exc),
                },
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
            ),
            file=sys.stderr,
        )
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
