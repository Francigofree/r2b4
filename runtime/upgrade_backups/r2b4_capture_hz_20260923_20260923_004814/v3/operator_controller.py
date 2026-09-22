"""High-level, UI-agnostic operator API for the R2B4 V3 robot.

This module sits above the canonical command/runtime boundary.  It owns host-side
orchestration only: process lifecycle, capture lifecycle, readiness/ALLOW waits,
high-level test sequencing and diagnostics.  It does not own motor authority and
never bypasses the canonical V3 CommandGateway -> L5 -> ... -> L12 path.

The public ``OperatorController`` API is intentionally independent of the shell
launcher so the same surface can be used by a CLI, GUI, agent or AI tool.
"""

from __future__ import annotations

import fcntl
import functools
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Mapping

from v3.control_cli import RESIDENT_PROCESS_STATUS_SCHEMA, _read_status
from v3.mcap_reader import McapReadError, McapReader


DEFAULT_CAPTURE_MODE = "alap"
CAPTURE_MODES = frozenset({"alap", "full", "nincs"})


class OperatorError(RuntimeError):
    """A host/operator operation could not be completed safely."""


@dataclass(frozen=True, slots=True)
class OperatorEvent:
    kind: str
    message: str
    data: Mapping[str, object] | None = None


@dataclass(frozen=True, slots=True)
class MotionHandle:
    pid: int
    label: str
    command_id: str
    capture_mode: str


@dataclass(frozen=True, slots=True)
class OperatorSnapshot:
    runtime_running: bool
    runtime_pid: int | None
    capture_mode: str | None
    status: Mapping[str, object] | None


def _serialized_operator_transition(method):
    @functools.wraps(method)
    def wrapped(self, *args, **kwargs):
        with self.operator_transition():
            return method(self, *args, **kwargs)

    return wrapped


class OperatorController:
    """General R2B4 operator/session controller.

    ``event_sink`` is optional.  A GUI/agent can consume structured events;
    callers that only need programmatic control may omit it entirely.
    """

    def __init__(
        self,
        project_root: Path | str | None = None,
        *,
        event_sink: Callable[[OperatorEvent], None] | None = None,
        python_executable: str | None = None,
    ) -> None:
        root = Path(project_root) if project_root is not None else Path(__file__).resolve().parents[1]
        self.root = root.resolve()
        self.runtime_dir = self.root / "runtime"
        self.capture_dir = self.runtime_dir / "captures"
        self.status_file = self.runtime_dir / "v3_status.json"
        self.command_file = self.runtime_dir / "v3_command.json"
        self.runtime_pid_file = self.runtime_dir / ".r2b4_runtime_pid"
        self.command_pid_file = self.runtime_dir / ".r2b4_command_pid"
        self.sequence_pid_file = self.runtime_dir / ".r2b4_sequence_pid"
        self.capture_path_file = self.runtime_dir / ".r2b4_capture_path"
        self.capture_used_file = self.runtime_dir / ".r2b4_capture_triggered"
        self.capture_mode_file = self.runtime_dir / ".r2b4_capture_mode"
        self.movement_capture_file = self.runtime_dir / ".r2b4_movement_capture"
        self.runtime_log_file = self.runtime_dir / ".r2b4_runtime_log"
        self.command_log_file = self.runtime_dir / ".r2b4_command_log"
        self.operator_lock_file = self.runtime_dir / ".r2b4_operator.lock"
        self.physics_file = self.root / "conf" / "fizika.json"
        self.python = python_executable or sys.executable
        self._event_sink = event_sink
        self._transition_thread_lock = threading.RLock()
        self._transition_depth = 0
        self._transition_stream = None

    # ------------------------------------------------------------------
    # Public API: status / lifecycle
    # ------------------------------------------------------------------

    def snapshot(self) -> OperatorSnapshot:
        pid = self._runtime_pid()
        status = self._read_status_optional()
        return OperatorSnapshot(
            runtime_running=pid is not None,
            runtime_pid=pid,
            capture_mode=self.current_capture_mode(),
            status=status,
        )

    def status(self) -> dict[str, object]:
        snap = self.snapshot()
        return {
            "runtime_running": snap.runtime_running,
            "runtime_pid": snap.runtime_pid,
            "capture_mode": snap.capture_mode,
            "status": dict(snap.status) if snap.status is not None else None,
        }

    def diagnostics(self) -> dict[str, object]:
        result = self.status()
        runtime_log = self._stored_path(self.runtime_log_file)
        command_log = self._stored_path(self.command_log_file)
        result["runtime_log"] = str(runtime_log) if runtime_log else None
        result["command_log"] = str(command_log) if command_log else None
        result["runtime_log_tail"] = self._tail(runtime_log, 20) if runtime_log else []
        result["command_log_tail"] = self._tail(command_log, 20) if command_log else []
        return result

    def live_runtime_status(self, *, max_age_ns: int = 500_000_000) -> dict[str, object] | None:
        """Return only status proven to belong to a currently live resident runtime."""
        if not isinstance(max_age_ns, int) or isinstance(max_age_ns, bool) or max_age_ns <= 0:
            raise ValueError("max_age_ns must be a positive integer")
        if self._runtime_pid() is None:
            return None
        status = self._read_status_optional()
        if status is None or status.get("state") != "RUNNING":
            return None
        stamp = status.get("monotonic_ns")
        if not isinstance(stamp, int) or isinstance(stamp, bool):
            return None
        age = time.monotonic_ns() - stamp
        if not 0 <= age < max_age_ns:
            return None
        return status

    @contextmanager
    def operator_transition(self):
        """Cross-process serialization for host/session state transitions only."""
        with self._transition_thread_lock:
            if self._transition_depth > 0:
                self._transition_depth += 1
                try:
                    yield
                finally:
                    self._transition_depth -= 1
                return

            self.runtime_dir.mkdir(parents=True, exist_ok=True)
            stream = self.operator_lock_file.open("a+")
            self.operator_lock_file.chmod(0o600)
            try:
                fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
                self._transition_stream = stream
                self._transition_depth = 1
                try:
                    yield
                finally:
                    self._transition_depth = 0
                    self._transition_stream = None
                    fcntl.flock(stream.fileno(), fcntl.LOCK_UN)
            finally:
                stream.close()

    @_serialized_operator_transition
    def runtime_start(self, capture_mode: str = DEFAULT_CAPTURE_MODE) -> int:
        mode = self._validate_capture_mode(capture_mode)
        old = self._runtime_pid()
        if old is not None:
            self._emit("info", "runtime: existing instance -> STOP")
            self.runtime_stop()
            self._emit("info", "runtime: waiting 5 s")
            time.sleep(5.0)
            if self._pid_matches(old, ("v3_process_runtime.py",)):
                raise OperatorError(f"old runtime still running (PID {old})")
            self._unlink(self.runtime_pid_file)
        else:
            self._stop_command_producers()

        self._unlink(self.command_file)
        baseline = self._read_status_optional()
        capture = self._new_capture_path()
        self._write_private_text(self.capture_path_file, str(capture))
        self._write_private_text(self.capture_mode_file, mode)
        self._unlink(self.capture_used_file)
        self._unlink(self.movement_capture_file)

        runtime_log = self._new_temp_log("r2b4-runtime.")
        self._write_private_text(self.runtime_log_file, str(runtime_log))
        self._unlink(self.runtime_pid_file)

        with runtime_log.open("ab", buffering=0) as log:
            supervisor = subprocess.Popen(
                [
                    self.python,
                    "-m",
                    "v3.operator_cli",
                    "__runtime-session",
                    "--capture-path",
                    str(capture),
                    "--capture-mode",
                    mode,
                ],
                cwd=self.root,
                stdin=subprocess.DEVNULL,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                close_fds=True,
            )

        pid: int | None = None
        for _ in range(40):
            pid = self._read_pid_file(self.runtime_pid_file)
            if pid is not None:
                break
            if supervisor.poll() is not None:
                break
            time.sleep(0.05)
        time.sleep(0.20)

        if pid is None or not self._pid_matches(pid, ("v3_process_runtime.py",)):
            self._unlink(self.runtime_pid_file)
            tail = "\n".join(self._tail(runtime_log, 30))
            raise OperatorError(f"runtime failed to start{': ' + tail if tail else ''}")

        if not self._wait_fresh_ready(pid, baseline, timeout=8.0):
            diag = self.diagnostics()
            try:
                self.runtime_stop()
            except Exception:
                pass
            raise OperatorError(f"runtime did not publish a fresh ready status: {diag}")

        self._emit("info", f"runtime: STARTED (PID {pid})")
        if mode == "alap":
            self._emit("info", "capture: ARMED (ALAP, native MCAP 8+2 bounded ring)")
            self._emit("info", f"capture file: {self._display_path(capture)}")
        elif mode == "full":
            self._emit("info", "capture: RECORDING (FULL append-only until runtime shutdown)")
            self._emit("info", f"capture file: {self._display_path(capture)}")
        else:
            self._emit("info", "capture: OFF (no native capture consumer/writer)")
        self._emit("info", f"runtime log: {runtime_log}")
        return pid

    @_serialized_operator_transition
    def ensure_runtime(self, capture_mode: str = DEFAULT_CAPTURE_MODE) -> int:
        requested = self._validate_capture_mode(capture_mode)
        pid = self._runtime_pid()
        if pid is not None:
            current = self.current_capture_mode() or DEFAULT_CAPTURE_MODE
            if current == requested:
                return pid
            self._emit("info", f"runtime: capture mode {current} -> {requested} requires safe restart")
        return self.runtime_start(requested)

    def _preempt_motion_once(self) -> Exception | None:
        error: Exception | None = None
        try:
            self._stop_command_producers()
        except Exception as exc:
            error = exc
        try:
            self._publish_stop()
        except Exception as exc:
            if error is None:
                error = exc
        return error

    def stop(self, *, wait_idle: bool = True) -> None:
        # Request STOP before waiting for the orchestration lock, then repeat it
        # under the lock so a concurrent ACTIVE transition cannot slip behind it.
        producer_error = self._preempt_motion_once()
        with self.operator_transition():
            if producer_error is None:
                producer_error = self._preempt_motion_once()
            try:
                self._trigger_movement_capture_if_needed()
            except Exception as exc:
                self._emit("warning", f"movement capture trigger failed: {exc}")
            finally:
                self._unlink(self.movement_capture_file)

            if producer_error is not None:
                self._emit("warning", "STOP could not be completed cleanly; requesting runtime shutdown")
                self._runtime_kill()
                raise OperatorError(str(producer_error))

            if wait_idle and self._runtime_pid() is not None:
                self.wait_idle()

    @_serialized_operator_transition
    def runtime_stop(self) -> None:
        try:
            self.stop(wait_idle=False)
        except Exception:
            self._runtime_kill()

        pid = self._runtime_pid()
        if pid is None:
            self._emit("info", "runtime: already STOPPED")
            return

        mode = self.current_capture_mode()
        if mode == "alap":
            if not self._capture_used():
                try:
                    self._trigger_current_capture("runtime-shutdown")
                except Exception as exc:
                    self._emit("warning", f"capture trigger failed during shutdown: {exc}")
            self._wait_triggered_capture_tail()

        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass

        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if not self._pid_matches(pid, ("v3_process_runtime.py",)):
                self._unlink(self.runtime_pid_file)
                self._emit("info", f"runtime: STOPPED (PID {pid})")
                capture = self.current_capture_path()
                if capture and capture.is_file():
                    self._emit("info", f"capture: {self._display_path(capture)}")
                    evidence = self._test_hub_evidence_dir(capture)
                    if evidence.is_dir():
                        self._emit("info", f"test hub: {self._display_path(evidence)}")
                    else:
                        self._emit("info", "test hub: finalization/analysis continuing in runtime log")
                elif mode == "nincs":
                    self._emit("info", "capture: OFF")
                return
            time.sleep(0.05)
        raise OperatorError(f"runtime did not exit after shutdown (PID {pid})")

    def shutdown(self) -> None:
        self.runtime_stop()

    def panic(self) -> None:
        self.runtime_stop()

    # ------------------------------------------------------------------
    # Public API: motion
    # ------------------------------------------------------------------

    def forward(
        self,
        speed_mps: float = 0.15,
        *,
        capture: bool = True,
        capture_mode: str = DEFAULT_CAPTURE_MODE,
        session_owner_pid: int | None = None,
        session_watchdog_s: float | None = None,
    ) -> MotionHandle:
        speed = self._positive_speed(speed_mps)
        return self.start_teleop(
            v_mps=speed, omega_rad_s=0.0, max_v_mps=speed, max_omega_rad_s=0.60,
            label=f"forward {speed:g} m/s", capture=capture, capture_mode=capture_mode,
            session_owner_pid=session_owner_pid, session_watchdog_s=session_watchdog_s,
        )

    def backward(
        self,
        speed_mps: float = 0.15,
        *,
        capture: bool = True,
        capture_mode: str = DEFAULT_CAPTURE_MODE,
        session_owner_pid: int | None = None,
        session_watchdog_s: float | None = None,
    ) -> MotionHandle:
        speed = self._positive_speed(speed_mps)
        return self.start_teleop(
            v_mps=-speed, omega_rad_s=0.0, max_v_mps=speed, max_omega_rad_s=0.60,
            label=f"backward {speed:g} m/s", capture=capture, capture_mode=capture_mode,
            session_owner_pid=session_owner_pid, session_watchdog_s=session_watchdog_s,
        )

    def start_teleop(
        self,
        *,
        v_mps: float,
        omega_rad_s: float,
        max_v_mps: float = 0.50,
        max_omega_rad_s: float = 1.20,
        label: str = "teleop",
        capture: bool = True,
        capture_mode: str = DEFAULT_CAPTURE_MODE,
        session_owner_pid: int | None = None,
        session_watchdog_s: float | None = None,
    ) -> MotionHandle:
        v = self._finite(v_mps, "v_mps")
        omega = self._finite(omega_rad_s, "omega_rad_s")
        max_v = self._finite(max_v_mps, "max_v_mps")
        max_omega = self._finite(max_omega_rad_s, "max_omega_rad_s")
        if abs(v) > 0.50:
            raise OperatorError("v_mps must stay within [-0.50, 0.50]")
        if abs(omega) > 1.20:
            raise OperatorError("omega_rad_s must stay within [-1.20, 1.20]")
        if max_v <= 0 or max_v > 0.50 or abs(v) > max_v + 1e-12:
            raise OperatorError("max_v_mps must be >0, <=0.50 and >= abs(v_mps)")
        if max_omega <= 0 or max_omega > 1.20 or abs(omega) > max_omega + 1e-12:
            raise OperatorError("max_omega_rad_s must be >0, <=1.20 and >= abs(omega_rad_s)")
        if abs(v) <= 1e-12 and abs(omega) <= 1e-12:
            raise OperatorError("zero TELEOP target is STOP; use stop()")

        command_id = f"operator-teleop-{time.time_ns()}-{os.getpid()}"
        args = [
            self.python, "-m", "v3.control_cli", "teleop", "--command-id", command_id,
            "--v-mps", str(v), "--omega-rad-s", str(omega),
            "--max-v-mps", str(max_v), "--max-omega-rad-s", str(max_omega),
        ]
        pid, mode = self._start_motion(
            label, capture, capture_mode, args,
            session_owner_pid=session_owner_pid, session_watchdog_s=session_watchdog_s,
        )
        return MotionHandle(pid=pid, label=label, command_id=command_id, capture_mode=mode)

    def wheels(
        self,
        left_mps: float,
        right_mps: float,
        *,
        capture: bool = True,
        capture_mode: str = DEFAULT_CAPTURE_MODE,
        session_owner_pid: int | None = None,
        session_watchdog_s: float | None = None,
    ) -> MotionHandle:
        v, omega, max_v, max_omega, track = self.wheel_targets_to_twist(left_mps, right_mps)
        self._emit("info", f"wheel target: left={left_mps:g} m/s, right={right_mps:g} m/s")
        self._emit("info", f"canonical target: v={v:g} m/s, omega={omega:g} rad/s (track={track:g} m)")
        return self.start_teleop(
            v_mps=v, omega_rad_s=omega, max_v_mps=max_v, max_omega_rad_s=max_omega,
            label=f"wheels L={left_mps:g} R={right_mps:g} m/s", capture=capture,
            capture_mode=capture_mode, session_owner_pid=session_owner_pid,
            session_watchdog_s=session_watchdog_s,
        )

    def roomcruise(
        self,
        *,
        capture: bool = True,
        capture_mode: str = DEFAULT_CAPTURE_MODE,
        session_owner_pid: int | None = None,
        session_watchdog_s: float | None = None,
    ) -> MotionHandle:
        command_id = f"operator-roomcruise-{time.time_ns()}-{os.getpid()}"
        args = [self.python, "-m", "v3.control_cli", "explore", "--command-id", command_id]
        pid, mode = self._start_motion(
            "roomcruise", capture, capture_mode, args,
            session_owner_pid=session_owner_pid, session_watchdog_s=session_watchdog_s,
        )
        return MotionHandle(pid=pid, label="roomcruise", command_id=command_id, capture_mode=mode)

    def faceperson(
        self,
        *,
        max_omega_rad_s: float = 0.50,
        capture: bool = True,
        capture_mode: str = DEFAULT_CAPTURE_MODE,
        session_owner_pid: int | None = None,
        session_watchdog_s: float | None = None,
    ) -> MotionHandle:
        max_omega = self._finite(max_omega_rad_s, "max_omega_rad_s")
        if max_omega <= 0.0 or max_omega > 1.20:
            raise OperatorError("max_omega_rad_s must be >0 and <=1.20")
        command_id = f"operator-faceperson-{time.time_ns()}-{os.getpid()}"
        args = [
            self.python, "-m", "v3.control_cli", "faceperson", "--command-id", command_id,
            "--max-omega-rad-s", str(max_omega),
        ]
        pid, mode = self._start_motion(
            "faceperson", capture, capture_mode, args, require_real_motion=False,
            session_owner_pid=session_owner_pid, session_watchdog_s=session_watchdog_s,
        )
        return MotionHandle(pid=pid, label="faceperson", command_id=command_id, capture_mode=mode)

    def followperson(
        self,
        *,
        max_v_mps: float = 0.15,
        max_omega_rad_s: float = 0.30,
        capture: bool = True,
        capture_mode: str = DEFAULT_CAPTURE_MODE,
        session_owner_pid: int | None = None,
        session_watchdog_s: float | None = None,
    ) -> MotionHandle:
        max_v = self._finite(max_v_mps, "max_v_mps")
        max_omega = self._finite(max_omega_rad_s, "max_omega_rad_s")
        if max_v <= 0.0 or max_v > 0.50:
            raise OperatorError("max_v_mps must be >0 and <=0.50")
        if max_omega <= 0.0 or max_omega > 1.20:
            raise OperatorError("max_omega_rad_s must be >0 and <=1.20")
        command_id = f"operator-followperson-{time.time_ns()}-{os.getpid()}"
        args = [
            self.python, "-m", "v3.control_cli", "followperson", "--command-id", command_id,
            "--max-v-mps", str(max_v), "--max-omega-rad-s", str(max_omega),
        ]
        pid, mode = self._start_motion(
            "followperson", capture, capture_mode, args, require_real_motion=False,
            session_owner_pid=session_owner_pid, session_watchdog_s=session_watchdog_s,
        )
        return MotionHandle(pid=pid, label="followperson", command_id=command_id, capture_mode=mode)

    def wheel_targets_to_twist(
        self,
        left_mps: float,
        right_mps: float,
    ) -> tuple[float, float, float, float, float]:
        left = self._finite(left_mps, "left_mps")
        right = self._finite(right_mps, "right_mps")
        if abs(left) > 0.50 or abs(right) > 0.50:
            raise OperatorError("each wheel speed must stay within [-0.50, 0.50] m/s")
        if abs(left) <= 1e-12 and abs(right) <= 1e-12:
            raise OperatorError("both wheel targets are zero; use stop()")
        try:
            physics = json.loads(self.physics_file.read_text(encoding="utf-8"))
            track = float(physics["nyomtav_szelesseg_m"])
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            raise OperatorError(f"cannot read valid track width from {self.physics_file}: {exc}") from exc
        if not math.isfinite(track) or track <= 0:
            raise OperatorError("nyomtav_szelesseg_m must be finite and positive")
        v = 0.5 * (left + right)
        omega = (right - left) / track
        if abs(v) > 0.50 + 1e-12:
            raise OperatorError(f"wheel pair requires v={v:.6f} m/s above resident limit")
        if abs(omega) > 1.20 + 1e-12:
            raise OperatorError(f"wheel pair requires omega={omega:.6f} rad/s above resident limit")
        max_v = max(abs(v), 1e-6)
        max_omega = min(1.20, abs(omega) + 0.60)
        return v, omega, max_v, max_omega, track

    # ------------------------------------------------------------------
    # Public API: capture
    # ------------------------------------------------------------------

    def current_capture_mode(self) -> str | None:
        try:
            value = self.capture_mode_file.read_text(encoding="utf-8").strip()
        except OSError:
            value = ""
        if value in CAPTURE_MODES:
            return value
        if self._runtime_pid() is not None:
            return DEFAULT_CAPTURE_MODE
        return None

    def current_capture_path(self) -> Path | None:
        try:
            raw = self.capture_path_file.read_text(encoding="utf-8").strip()
        except OSError:
            return None
        return Path(raw) if raw else None

    @_serialized_operator_transition
    def capture_start(self, capture_mode: str | None = None) -> Path | None:
        if capture_mode is None:
            mode = self.current_capture_mode() if self._runtime_pid() is not None else DEFAULT_CAPTURE_MODE
        else:
            mode = self._validate_capture_mode(capture_mode)
        assert mode is not None
        self.ensure_runtime(mode)
        if mode == "alap":
            self._ensure_fresh_capture_slot()
            self._trigger_current_capture("manual")
        elif mode == "full":
            self._emit("info", "capture: FULL already recording")
        else:
            raise OperatorError("capture is OFF (nincs)")
        return self.current_capture_path()

    @_serialized_operator_transition
    def capture_stop(self) -> dict[str, object]:
        path = self.current_capture_path()
        mode = self.current_capture_mode()
        if path is None:
            raise OperatorError("capture is not armed")
        if mode == "nincs":
            return {"state": "OFF", "mode": mode, "path": str(path)}
        if mode == "full":
            self._emit("info", "capture: FULL remains continuous until runtime stop/shutdown")
            return {"state": "RECORDING", "mode": mode, "path": str(path)}

        if self.movement_capture_file.exists() and not self.capture_used_file.exists() and not path.exists():
            self._trigger_current_capture("capture-stop")
        if not self.capture_used_file.exists() and not path.exists():
            return {"state": "ARMED", "mode": mode, "path": str(path)}
        for _ in range(70):
            if self._capture_ready(path):
                break
            time.sleep(0.05)
        return self.capture_status()

    def capture_status(self) -> dict[str, object]:
        path = self.current_capture_path()
        mode = self.current_capture_mode()
        if mode == "nincs":
            return {"state": "OFF", "mode": mode, "path": str(path) if path else None}
        if path is None:
            return {"state": "NOT_ARMED", "mode": mode, "path": None}
        if path.is_file():
            try:
                final = self._verified_capture_final(path)
                complete = True
            except (OSError, ValueError, McapReadError):
                final = {}
                complete = False
            return {
                "state": "FINALIZED" if complete else "RECORDING_OR_FINALIZING",
                "mode": mode,
                "path": str(path),
                "status": final.get("status"),
                "ticks": final.get("captured_tick_count"),
                "trigger": final.get("trigger_reason"),
                "complete": complete,
                "evidence": str(self._test_hub_evidence_dir(path)) if self._test_hub_evidence_dir(path).is_dir() else None,
            }
        if mode == "full":
            state = "RECORDING"
        elif self.capture_used_file.exists():
            state = "TRIGGERED_OR_FINALIZING"
        elif self.movement_capture_file.exists():
            state = "ARMED_FOR_MOVEMENT"
        else:
            state = "ARMED"
        return {"state": state, "mode": mode, "path": str(path)}

    # ------------------------------------------------------------------
    # Public API: integrated physical test sequence
    # ------------------------------------------------------------------

    @_serialized_operator_transition
    def run_proba(self, *, capture_mode: str = DEFAULT_CAPTURE_MODE) -> None:
        mode = self._validate_capture_mode(capture_mode)
        self.ensure_runtime(mode)
        self.stop()
        if mode == "alap":
            self._ensure_fresh_capture_slot()
            self._write_private_text(self.movement_capture_file, "movement")
        else:
            self._unlink(self.movement_capture_file)

        self._write_private_text(self.sequence_pid_file, str(os.getpid()))
        try:
            track = self._track_width()
            arc_v = 0.15
            arc_omega = arc_v / 1.0
            pivot_omega = min(0.4, 0.20 / track)
            phases = (
                (1, "előre, 5 s, 0.15 m/s", 0.15, 0.0, 0.0),
                (2, "hátra, 5 s, 0.15 m/s", -0.15, 0.0, 0.0),
                (3, "jobbra előre, 5 s, R=1.0 m", arc_v, -arc_omega, 0.0),
                (4, "jobbra hátra, 5 s, R=1.0 m", -arc_v, arc_omega, 0.0),
                (5, "balra előre, 5 s, R=1.0 m", arc_v, arc_omega, 0.0),
                (6, "balra hátra, 5 s, R=1.0 m", -arc_v, -arc_omega, 0.0),
                (7, "helyben jobbra 90°", 0.0, -0.4, -math.pi / 2),
                (8, "helyben balra 90°", 0.0, 0.4, math.pi / 2),
                (9, "jobbra 180°, jobb kerék célja 0", pivot_omega * track / 2, -pivot_omega, -math.pi),
                (10, "balra 180°, jobb kerék célja 0", -pivot_omega * track / 2, pivot_omega, math.pi),
            )
            for index, label, v, omega, angle in phases:
                self._run_proba_phase(index, label, v, omega, angle)
                self._proba_idle(5.0 if index < 10 else 0.0)
            self._emit("info", "proba: kész, IDLE")
        except KeyboardInterrupt:
            self._emit("warning", "proba: megszakítva")
            raise
        finally:
            try:
                self.stop(wait_idle=False)
            finally:
                self._unlink(self.sequence_pid_file)

    # ------------------------------------------------------------------
    # Internal worker API used by operator_cli.  Not a motor bypass.
    # ------------------------------------------------------------------

    def run_runtime_session(self, capture: Path, mode: str) -> int:
        mode = self._validate_capture_mode(mode)
        capture = Path(capture).resolve()
        command = [self.python, "v3_process_runtime.py", "--approval", "native-resident-v3"]
        if mode == "alap":
            command += [
                "--capture-path", self._display_path(capture),
                "--capture-mode", "triggered",
                "--capture-pre-event-ns", "8000000000",
                "--capture-post-event-ns", "2000000000",
                "--capture-max-ticks", "768",
                "--capture-max-raw-scans", "128",
            ]
        elif mode == "full":
            command += [
                "--capture-path", self._display_path(capture),
                "--capture-mode", "append_only",
                "--capture-max-session-records", "2147483647",
            ]

        proc = subprocess.Popen(command, cwd=self.root)
        self._write_private_text(self.runtime_pid_file, str(proc.pid))

        hub_thread = threading.Thread(
            target=self._test_hub_supervisor,
            args=(capture, proc.pid, mode),
            name="r2b4-test-hub-supervisor",
            daemon=False,
        )
        hub_thread.start()

        old_handlers: dict[int, object] = {}

        def forward_signal(signum: int, _frame: object) -> None:
            if proc.poll() is None:
                try:
                    proc.send_signal(signum)
                except ProcessLookupError:
                    pass

        for sig in (signal.SIGINT, signal.SIGTERM):
            old_handlers[sig] = signal.getsignal(sig)
            signal.signal(sig, forward_signal)
        try:
            rc = proc.wait()
        finally:
            for sig, handler in old_handlers.items():
                signal.signal(sig, handler)
            hub_thread.join()

        evidence = self._test_hub_evidence_dir(capture)
        if capture.is_file():
            self._emit("info", f"capture: finalized -> {self._display_path(capture)}")
            if evidence.is_dir():
                self._emit("info", f"test hub: results -> {self._display_path(evidence)}")
        else:
            self._emit("info", "capture: no finalized MCAP for this runtime session")
        return rc

    # ------------------------------------------------------------------
    # Internal motion/session helpers
    # ------------------------------------------------------------------

    @_serialized_operator_transition
    def _start_motion(
        self,
        label: str,
        capture: bool,
        capture_mode: str,
        command: list[str],
        *,
        require_real_motion: bool = True,
        session_owner_pid: int | None = None,
        session_watchdog_s: float | None = None,
    ) -> tuple[int, str]:
        requested = self._validate_capture_mode(capture_mode)
        self.ensure_runtime(requested)
        self.stop()

        mode = self.current_capture_mode() or DEFAULT_CAPTURE_MODE
        if mode == "alap" and capture:
            self._ensure_fresh_capture_slot()
            self._wait_ready()
            self._write_private_text(self.movement_capture_file, "movement")
        else:
            self._unlink(self.movement_capture_file)

        status = self._read_status_optional()
        baseline = self._tick(status) if status else -1
        try:
            if session_owner_pid is None and session_watchdog_s is None:
                pid = self._spawn_control_process(command)
            else:
                pid = self._spawn_control_process(
                    command, session_owner_pid=session_owner_pid,
                    session_watchdog_s=session_watchdog_s,
                )
            if require_real_motion:
                accepted = self._wait_allow(pid, baseline, label)
            else:
                accepted = self._wait_allow(pid, baseline, label, require_real_motion=False)
            if not accepted:
                raise OperatorError(f"{label} did not reach ACTIVE ALLOW")
        except BaseException:
            self.stop(wait_idle=False)
            raise

        self._emit("info", f"{label}: STARTED")
        if require_real_motion:
            self._emit("info", "motor: ALLOW confirmed")
        else:
            self._emit("info", "command: ACTIVE/ALLOW confirmed (zero motion is valid)")
        if mode == "alap":
            if capture:
                path = self.current_capture_path()
                self._emit("info", f"capture: ALAP armed for STOP/FAULT -> {self._display_path(path) if path else '-'}")
            else:
                self._emit("info", "capture: movement trigger skipped by nocapture")
        elif mode == "full":
            self._emit("info", "capture: FULL continuous recording")
        else:
            self._emit("info", "capture: OFF")
        return pid, mode

    def _spawn_control_process(
        self,
        command: list[str],
        *,
        session_owner_pid: int | None = None,
        session_watchdog_s: float | None = None,
    ) -> int:
        prepared = list(command)
        cli_options: list[str] = []
        if session_owner_pid is not None:
            if not isinstance(session_owner_pid, int) or isinstance(session_owner_pid, bool) or session_owner_pid <= 0:
                raise OperatorError("session_owner_pid must be a positive integer")
            cli_options += ["--owner-pid", str(session_owner_pid)]
        if session_watchdog_s is not None:
            watchdog = self._finite(session_watchdog_s, "session_watchdog_s")
            if watchdog <= 0.0:
                raise OperatorError("session_watchdog_s must be > 0")
            cli_options += ["--max-runtime-s", str(watchdog)]
        if cli_options:
            if len(prepared) < 4 or prepared[1:3] != ["-m", "v3.control_cli"]:
                raise OperatorError("session watchdog requires canonical v3.control_cli producer")
            prepared[3:3] = cli_options

        command_log = self._new_temp_log("r2b4-command.")
        self._write_private_text(self.command_log_file, str(command_log))
        with command_log.open("ab", buffering=0) as log:
            proc = subprocess.Popen(
                prepared, cwd=self.root, stdin=subprocess.DEVNULL,
                stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True, close_fds=True,
            )
        self._write_private_text(self.command_pid_file, str(proc.pid))
        return proc.pid

    def _wait_allow(
        self,
        pid: int,
        baseline: int,
        label: str,
        *,
        require_real_motion: bool = True,
    ) -> bool:
        last: Mapping[str, object] | None = None
        for _ in range(50):
            if not self._pid_matches(pid, ("v3.control_cli",)):
                self._failure_report(label, last)
                return False
            if self._runtime_pid() is None:
                self._failure_report(label, last)
                return False
            last = self._read_status_optional()
            if last is None:
                time.sleep(0.05)
                continue
            if self._tick(last) > baseline:
                if self._status_is_fault(last):
                    self._failure_report(label, last)
                    return False
                allowed = (
                    self._status_has_real_allow(last)
                    if require_real_motion
                    else self._status_has_allow(last)
                )
                if allowed:
                    return True
            time.sleep(0.05)
        self._failure_report(label, last)
        return False

    def _failure_report(self, label: str, status: Mapping[str, object] | None) -> None:
        self._emit("error", f"{label} did not reach motor ALLOW")
        if status is not None:
            self._emit(
                "error",
                f"state={status.get('state')} tick={status.get('tick_id')} "
                f"fault={status.get('fault_layer')} safety={status.get('safety_decision')}/{status.get('safety_reason')} "
                f"motors={status.get('left_output')}/{status.get('right_output')}",
            )

    def _run_proba_phase(self, index: int, label: str, v: float, omega: float, angle: float) -> None:
        self._emit("info", f"proba {index}/10: {label}")
        # STOP expires during the quiet pause. Refresh it before each ACTIVE
        # request and observe the runtime's own readiness decision.
        self._publish_stop()
        self.wait_idle()
        initial = self._runtime_status()
        previous_yaw = self._yaw(initial) if angle else 0.0

        command_id = f"operator-proba-{os.getpid()}-{index}-{time.monotonic_ns()}"
        command = [
            self.python, "-m", "v3.control_cli", "teleop",
            "--command-id", command_id,
            "--v-mps", str(v),
            "--omega-rad-s", str(omega),
            "--max-v-mps", "0.25",
            "--max-omega-rad-s", "0.60",
        ]
        phase_start_ns = time.monotonic_ns()
        turned = 0.0
        started = time.monotonic()
        allowed = False
        try:
            pid = self._spawn_control_process(command)
            while True:
                if not self._pid_matches(pid, ("v3.control_cli",)):
                    raise OperatorError("phase command producer stopped unexpectedly")
                now = time.monotonic()
                elapsed = now - started
                status = self._runtime_status()
                status_ns = int(status.get("monotonic_ns", 0))
                if status_ns > phase_start_ns:
                    if status.get("safety_decision") == "ALLOW" and status.get("enabled") is True:
                        if not allowed:
                            started = now
                        allowed = True
                    elif allowed or status.get("safety_reason") != "NOT_ACTIVE":
                        raise OperatorError(f"motion stopped by safety: {status}")
                if not allowed and elapsed > 1.5:
                    raise OperatorError("phase did not reach ALLOW")

                if not allowed:
                    time.sleep(0.05)
                    continue
                elapsed = now - started
                if angle:
                    current_yaw = self._yaw(status)
                    turned += math.atan2(
                        math.sin(current_yaw - previous_yaw),
                        math.cos(current_yaw - previous_yaw),
                    )
                    previous_yaw = current_yaw
                    remaining = abs(angle) - math.copysign(1.0, angle) * turned
                    if remaining <= math.radians(2):
                        break
                    if elapsed > 30.0:
                        raise OperatorError("turn did not reach its target within 30 s")
                elif elapsed >= 5.0:
                    break
                time.sleep(0.05)
        finally:
            try:
                self._stop_control_cli_only()
            finally:
                self._publish_stop()

    def _proba_idle(self, pause: float) -> None:
        # Quiet pause: the next phase requests fresh readiness. The operator
        # does not impose a health policy on the resident runtime during IDLE.
        time.sleep(pause)

    # ------------------------------------------------------------------
    # Capture/Test Hub internals
    # ------------------------------------------------------------------

    def _new_capture_path(self) -> Path:
        self.capture_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        return self.capture_dir / f"v3_{stamp}_{os.getpid()}_capture.mcap"

    def _capture_used(self) -> bool:
        if self.current_capture_mode() != "alap":
            return False
        if self.capture_used_file.exists():
            return True
        path = self.current_capture_path()
        return bool(path and path.is_file())

    def _ensure_fresh_capture_slot(self) -> None:
        if self.current_capture_mode() != "alap" or not self._capture_used():
            return
        self._emit("info", "capture: previous bounded slot used -> re-arm requires runtime restart")
        self.runtime_start("alap")
        self._wait_ready()

    def _trigger_current_capture(self, origin: str) -> None:
        pid = self._runtime_pid()
        if pid is None:
            raise OperatorError("runtime is not running")
        mode = self.current_capture_mode()
        if mode == "full":
            self._emit("info", "capture: FULL already records continuously; trigger not required")
            return
        if mode == "nincs":
            raise OperatorError("capture is OFF (nincs)")
        if mode != "alap":
            raise OperatorError("unknown runtime capture mode")
        path = self.current_capture_path()
        if path is None:
            raise OperatorError("no operator-managed capture path")
        if self._capture_used():
            return
        try:
            os.kill(pid, signal.SIGUSR1)
        except ProcessLookupError as exc:
            raise OperatorError("capture trigger failed") from exc
        self._write_private_text(
            self.capture_used_file,
            f"origin={origin}\ntriggered_at_ns={time.time_ns()}\n",
        )
        self._emit("info", f"capture: TRIGGERED ({origin})")
        self._emit("info", f"capture file: {self._display_path(path)}")

    def _trigger_movement_capture_if_needed(self) -> None:
        if self.current_capture_mode() != "alap":
            return
        if not self.movement_capture_file.exists() or self._capture_used() or self._runtime_pid() is None:
            return
        self._trigger_current_capture("movement-stop")

    def _capture_ready(self, path: Path) -> bool:
        if not path.is_file() or path.is_symlink():
            return False
        try:
            self._verified_capture_final(path)
        except (OSError, ValueError, McapReadError):
            return False
        return True

    @staticmethod
    def _verified_capture_final(path: Path) -> dict[str, object]:
        reader = McapReader(path)
        if not reader.inspect(verify_chunks=True).valid:
            raise McapReadError("capture container integrity failed")
        # The reader checks final['integrity']['complete'], including required
        # delivery loss and the message digest. There is no top-level complete.
        return reader.capture_integrity()

    def _wait_triggered_capture_tail(self) -> None:
        if self.current_capture_mode() != "alap":
            return
        path = self.current_capture_path()
        if path is None or not self.capture_used_file.exists() or self._capture_ready(path):
            return
        self._emit("info", "capture: waiting for 2 s post-event tail/finalization")
        for _ in range(70):
            if self._capture_ready(path):
                return
            time.sleep(0.05)
        self._emit("warning", "bounded capture was not finalized before runtime shutdown; native finalizer will finish it")

    def _test_hub_evidence_dir(self, capture: Path) -> Path:
        return capture.with_suffix(".evidence")

    def _run_test_hub_default(self, capture: Path) -> int:
        evidence = self._test_hub_evidence_dir(capture)
        self._emit("info", f"TEST_HUB_AUTORUN START capture={self._display_path(capture)}")
        proc = subprocess.run(
            [self.python, "-m", "v3.test_hub", "run", str(capture)],
            cwd=self.root,
            check=False,
        )
        if proc.returncode == 0:
            self._emit("info", f"TEST_HUB_AUTORUN PASS evidence={self._display_path(evidence)}")
        else:
            self._emit("error", f"TEST_HUB_AUTORUN FAIL rc={proc.returncode} capture={self._display_path(capture)}")
        return proc.returncode

    def _test_hub_supervisor(self, capture: Path, runtime_pid: int, mode: str) -> None:
        if mode == "nincs":
            return
        if mode == "alap":
            while self._pid_matches(runtime_pid, ("v3_process_runtime.py",)):
                if self._capture_ready(capture):
                    self._run_test_hub_default(capture)
                    return
                time.sleep(0.20)
        else:
            while self._pid_matches(runtime_pid, ("v3_process_runtime.py",)):
                time.sleep(0.50)
        for _ in range(100):
            if self._capture_ready(capture):
                self._run_test_hub_default(capture)
                return
            time.sleep(0.10)
        self._emit("warning", f"TEST_HUB_AUTORUN capture wait timeout: {self._display_path(capture)}")
        self._run_test_hub_default(capture)

    # ------------------------------------------------------------------
    # Runtime/status internals
    # ------------------------------------------------------------------

    def wait_idle(self, timeout: float = 3.0) -> None:
        if self._runtime_pid() is None:
            return
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            status = self._read_status_optional()
            if status is None:
                time.sleep(0.05)
                continue
            age = time.monotonic_ns() - int(status.get("monotonic_ns", 0))
            if (
                status.get("state") == "RUNNING"
                and 0 <= age < 500_000_000
                and self._stopped(status)
            ):
                return
            if status.get("state") != "RUNNING" or status.get("fault_layer") or status.get("safety_decision") == "FAULT":
                break
            time.sleep(0.05)
        raise OperatorError("runtime did not confirm ready IDLE with inactive outputs")

    def _wait_ready(self, timeout: float = 8.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._runtime_pid() is None:
                break
            status = self._read_status_optional()
            if status and status.get("state") == "RUNNING" and status.get("ready_for_active") is True:
                return
            time.sleep(0.05)
        raise OperatorError("runtime not ready")

    def _wait_fresh_ready(
        self,
        pid: int,
        baseline: Mapping[str, object] | None,
        *,
        timeout: float,
    ) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not self._pid_matches(pid, ("v3_process_runtime.py",)):
                return False
            status = self._read_status_optional()
            if (
                status is not None
                and status != baseline
                and status.get("state") == "RUNNING"
                and status.get("ready_for_active") is True
            ):
                return True
            time.sleep(0.05)
        return False

    def _read_status_optional(self) -> dict[str, object] | None:
        try:
            status = _read_status(self.status_file)
        except (OSError, ValueError):
            return None
        if status.get("schema") != RESIDENT_PROCESS_STATUS_SCHEMA:
            return None
        return status

    def _runtime_status(self) -> dict[str, object]:
        status = self._read_status_optional()
        if status is None:
            raise OperatorError("runtime status unavailable")
        age = time.monotonic_ns() - int(status.get("monotonic_ns", 0))
        if (
            status.get("state") != "RUNNING"
            or not 0 <= age < 500_000_000
            or status.get("fault_layer")
            or status.get("safety_decision") == "FAULT"
        ):
            raise OperatorError(f"runtime telemetry unavailable, stale or faulted: {status}")
        return status

    @staticmethod
    def _status_is_fault(status: Mapping[str, object]) -> bool:
        return (
            status.get("state") != "RUNNING"
            or status.get("safety_decision") == "FAULT"
            or status.get("fault_layer") is not None
        )

    @staticmethod
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
        left = status.get("left_output")
        right = status.get("right_output")
        return (
            isinstance(left, (int, float))
            and isinstance(right, (int, float))
            and (abs(float(left)) > 1e-9 or abs(float(right)) > 1e-9)
        )

    @staticmethod
    def _stopped(status: Mapping[str, object]) -> bool:
        return (
            status.get("safety_decision") == "STOP"
            and status.get("enabled") is False
            and status.get("left_output") == 0
            and status.get("right_output") == 0
            and status.get("ready_for_active") is True
        )

    @staticmethod
    def _tick(status: Mapping[str, object]) -> int:
        value = status.get("tick_id", -1)
        return int(value) if isinstance(value, int) and not isinstance(value, bool) else -1

    @staticmethod
    def _yaw(status: Mapping[str, object]) -> float:
        estimate = status.get("estimate")
        value = estimate.get("yaw_rad") if isinstance(estimate, Mapping) else None
        if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(float(value)):
            raise OperatorError("missing/invalid runtime yaw")
        return float(value)

    # ------------------------------------------------------------------
    # Command/process internals
    # ------------------------------------------------------------------

    def _publish_stop(self) -> None:
        try:
            result = subprocess.run(
                [self.python, "-m", "v3.control_cli", "stop", "--command-id",
                 f"operator-stop-{time.time_ns()}-{os.getpid()}"],
                cwd=self.root, text=True, capture_output=True, timeout=1.0, check=False,
            )
        except subprocess.TimeoutExpired as exc:
            raise OperatorError("control_cli STOP timed out") from exc
        if result.returncode != 0:
            raise OperatorError(f"control_cli STOP failed: {result.stderr.strip() or result.stdout.strip()}")

    def _stop_command_producers(self) -> None:
        seq = self._read_pid_file(self.sequence_pid_file)
        if seq is not None and seq != os.getpid() and self._pid_matches(seq, ("v3.operator_cli", "proba")):
            try:
                os.kill(seq, signal.SIGTERM)
            except ProcessLookupError:
                pass
            if not self._wait_gone(seq, 2.0):
                raise OperatorError(f"proba sequence did not stop (PID {seq})")
        if seq != os.getpid():
            self._unlink(self.sequence_pid_file)
        # A sequence started by the previous shell launcher may still exist.
        legacy = self._find_pid(("r2b4-proba",))
        if legacy is not None:
            try:
                os.kill(legacy, signal.SIGTERM)
            except ProcessLookupError:
                pass
            if not self._wait_gone(legacy, 2.0):
                raise OperatorError(f"legacy proba sequence did not stop (PID {legacy})")
        self._stop_control_cli_only()

    def _stop_control_cli_only(self) -> None:
        pid = self._read_pid_file(self.command_pid_file)
        if pid is None or not self._pid_matches(pid, ("v3.control_cli",)):
            pid = self._find_pid(("v3.control_cli",))
        while pid is not None:
            if pid != os.getpid():
                try:
                    os.kill(pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                if not self._wait_gone(pid, 1.0):
                    raise OperatorError(f"command heartbeat did not stop (PID {pid})")
            self._unlink(self.command_pid_file)
            pid = self._find_pid(("v3.control_cli",))

    def _runtime_kill(self) -> None:
        pid = self._runtime_pid()
        if pid is not None:
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

    def _runtime_pid(self) -> int | None:
        pid = self._read_pid_file(self.runtime_pid_file)
        if pid is not None and self._pid_matches(pid, ("v3_process_runtime.py",)):
            return pid
        return self._find_pid(("v3_process_runtime.py",))

    def _find_pid(self, required_args: tuple[str, ...]) -> int | None:
        proc = Path("/proc")
        if not proc.is_dir():
            return None
        for entry in proc.iterdir():
            if not entry.name.isdigit():
                continue
            pid = int(entry.name)
            if self._pid_matches(pid, required_args):
                return pid
        return None

    def _pid_matches(self, pid: int, required_args: tuple[str, ...]) -> bool:
        if pid <= 0:
            return False
        proc = Path("/proc") / str(pid)
        try:
            cwd = Path(os.readlink(proc / "cwd")).resolve()
            raw = (proc / "cmdline").read_bytes()
        except OSError:
            return False
        if cwd != self.root:
            return False
        argv = [part.decode("utf-8", "replace") for part in raw.split(b"\0") if part]
        return all(token in argv for token in required_args)

    @staticmethod
    def _wait_gone(pid: int, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                os.kill(pid, 0)
            except ProcessLookupError:
                return True
            try:
                state = (Path("/proc") / str(pid) / "stat").read_text(encoding="utf-8")
                if ") Z" in state:
                    return True
            except OSError:
                return True
            time.sleep(0.05)
        return False

    # ------------------------------------------------------------------
    # Small utilities
    # ------------------------------------------------------------------

    def _track_width(self) -> float:
        try:
            data = json.loads(self.physics_file.read_text(encoding="utf-8"))
            value = float(data["nyomtav_szelesseg_m"])
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            raise OperatorError(f"invalid track width: {exc}") from exc
        if not math.isfinite(value) or value <= 0:
            raise OperatorError("invalid track width")
        return value

    @staticmethod
    def _positive_speed(value: float) -> float:
        speed = OperatorController._finite(value, "speed_mps")
        if not 0 < speed <= 0.50:
            raise OperatorError("speed must be >0 and <=0.50 m/s")
        return speed

    @staticmethod
    def _finite(value: float, name: str) -> float:
        try:
            result = float(value)
        except (TypeError, ValueError) as exc:
            raise OperatorError(f"{name} must be numeric") from exc
        if not math.isfinite(result):
            raise OperatorError(f"{name} must be finite")
        return result

    @staticmethod
    def _validate_capture_mode(mode: str) -> str:
        if mode not in CAPTURE_MODES:
            raise OperatorError("capture mode must be one of: alap, full, nincs")
        return mode

    def _display_path(self, path: Path | None) -> str:
        if path is None:
            return "-"
        try:
            return str(path.resolve().relative_to(self.root))
        except ValueError:
            return str(path)

    @staticmethod
    def _new_temp_log(prefix: str) -> Path:
        descriptor, raw_path = tempfile.mkstemp(prefix=prefix, suffix=".log", dir="/tmp")
        os.close(descriptor)
        return Path(raw_path)

    @staticmethod
    def _tail(path: Path, lines: int) -> list[str]:
        try:
            return path.read_text(encoding="utf-8", errors="replace").splitlines()[-lines:]
        except OSError:
            return []

    @staticmethod
    def _stored_path(pointer: Path) -> Path | None:
        try:
            raw = pointer.read_text(encoding="utf-8").strip()
        except OSError:
            return None
        return Path(raw) if raw else None

    @staticmethod
    def _read_pid_file(path: Path) -> int | None:
        try:
            raw = path.read_text(encoding="ascii").strip()
            value = int(raw)
        except (OSError, ValueError):
            return None
        return value if value > 0 else None

    @staticmethod
    def _unlink(path: Path) -> None:
        try:
            path.unlink()
        except FileNotFoundError:
            pass

    @staticmethod
    def _write_private_text(path: Path, text: str) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text.rstrip("\n") + "\n", encoding="utf-8")
        path.chmod(0o600)

    def _emit(self, kind: str, message: str, data: Mapping[str, object] | None = None) -> None:
        if self._event_sink is not None:
            self._event_sink(OperatorEvent(kind=kind, message=message, data=data))


__all__ = [
    "CAPTURE_MODES",
    "DEFAULT_CAPTURE_MODE",
    "MotionHandle",
    "OperatorController",
    "OperatorError",
    "OperatorEvent",
    "OperatorSnapshot",
]
