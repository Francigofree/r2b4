"""Small host supervisor for separately interpreted, trusted Python skills."""

from __future__ import annotations

from collections import OrderedDict
import json
import os
from pathlib import Path
import selectors
import signal
import subprocess
import sys
import threading
import time
from typing import Callable, Mapping
import uuid

from r2b4_orchestration.skill_library import SkillLibrary
from r2b4_orchestration.skill_worker import MAX_RESULT_BYTES


_ACTIVE = {"STARTING", "RUNNING"}
_TERMINAL = {"SUCCEEDED", "FAILED", "CANCELLED"}


class SkillRuntime:
    """No V3 handles or motion authority; host revocation owns canonical STOP."""

    def __init__(self, library: SkillLibrary, socket_path: Path | str, *,
                 on_terminal: Callable[[dict[str, object]], None] | None = None,
                 clock_ns=time.monotonic_ns, history_limit: int = 64,
                 output_limit: int = 8192, cancellation_grace_s: float = 0.5,
                 project_root: Path | str | None = None) -> None:
        if type(history_limit) is not int or history_limit <= 0:
            raise ValueError("skill history limit must be positive")
        if type(output_limit) is not int or not 0 < output_limit <= MAX_RESULT_BYTES:
            raise ValueError("skill output limit must be bounded and positive")
        if not isinstance(cancellation_grace_s, (float, int)) or not 0 < cancellation_grace_s <= 5:
            raise ValueError("skill cancellation grace must be in (0, 5]")
        self.library, self.socket_path = library, Path(socket_path)
        self.project_root = Path(project_root or library.root.parent).resolve()
        self._on_terminal, self._clock_ns = on_terminal, clock_ns
        self._history_limit, self._output_limit = history_limit, output_limit
        self._grace_s = float(cancellation_grace_s)
        self._lock = threading.RLock()
        self._runs: OrderedDict[str, dict[str, object]] = OrderedDict()
        self._active_id: str | None = None
        self._generation = 0
        self._closed = False

    @property
    def active(self) -> bool:
        with self._lock:
            return bool(self._active_id and not self._runs[self._active_id]["_done"].is_set())

    def accepts(self, run_id: str) -> bool:
        with self._lock:
            return (not self._closed and run_id == self._active_id
                    and self._runs[run_id]["state"] in _ACTIVE)

    @staticmethod
    def _public(record: dict[str, object]) -> dict[str, object]:
        return json.loads(json.dumps({key: value for key, value in record.items() if not key.startswith("_")}))

    def status(self, run_id: str | None = None) -> dict[str, object]:
        with self._lock:
            selected = run_id or self._active_id
            if selected is None:
                return {"run_id": None, "state": "IDLE"}
            return self._public(self._runs[selected])

    def start(self, name: str, parameters: Mapping[str, object] | None = None,
              *, run_id: str | None = None) -> dict[str, object]:
        loaded = self.library.load(name)
        if parameters is not None and not isinstance(parameters, Mapping):
            raise ValueError("skill parameters must be an object")
        encoded = json.dumps(dict(parameters or {}), allow_nan=False).encode("utf-8")
        if len(encoded) > MAX_RESULT_BYTES:
            raise ValueError("skill parameters exceeded their bound")
        selected = run_id or f"skill-{uuid.uuid4().hex}"
        if not isinstance(selected, str) or not selected or len(selected) > 256:
            raise ValueError("skill run identity must be a bounded string")
        with self._lock:
            if self._closed:
                raise RuntimeError("skill runtime is closed")
            if self.active:
                raise RuntimeError("active skill must be explicitly preempted")
            if selected in self._runs:
                raise ValueError("skill run identity already used")
            self._generation += 1
            record: dict[str, object] = {
                "run_id": selected, "name": name, "state": "STARTING", "generation": self._generation,
                "revision": 1, "source_hash": loaded.source_hash, "source_path": str(loaded.path),
                "parameters": json.loads(encoded), "started_ns": self._clock_ns(), "finished_ns": None,
                "result": None, "error": None, "pid": None, "returncode": None,
                "stdout": "", "stderr": "", "output_dropped_bytes": 0,
                "finalizing": False,
                "_done": threading.Event(), "_process": None, "_notified": False,
            }
            self._runs[selected] = record
            self._active_id = selected
            self._trim()
            request = {"name": name, "run_id": selected, "source": loaded.source,
                       "source_path": str(loaded.path), "socket_path": str(self.socket_path),
                       "project_root": str(self.project_root), "parameters": record["parameters"]}
            thread = threading.Thread(target=self._supervise, args=(record, request),
                                      name=f"skill-{self._generation}", daemon=True)
            record["_thread"] = thread
            thread.start()
            return self._public(record)

    def _trim(self) -> None:
        # A revoked process may still need cleanup; never discard its supervisor state.
        for key in list(self._runs):
            if len(self._runs) <= self._history_limit:
                break
            if key != self._active_id and self._runs[key]["_done"].is_set():
                del self._runs[key]

    def revoke(self, run_id: str | None = None, reason: str = "STOP") -> dict[str, object]:
        if not isinstance(reason, str) or not reason or len(reason) > 1024:
            raise ValueError("skill stop reason must be bounded")
        with self._lock:
            selected = run_id or self._active_id
            if selected is None:
                return {"run_id": None, "state": "IDLE"}
            record = self._runs[selected]
            if record["state"] in _ACTIVE:
                record.update(state="CANCELLED", error=reason, finished_ns=self._clock_ns(),
                              revision=int(record["revision"]) + 1, finalizing=True)
            return self._public(record)

    def stop(self, run_id: str | None = None, reason: str = "STOP") -> dict[str, object]:
        snapshot = self.revoke(run_id, reason)
        selected = snapshot["run_id"]
        if selected is not None:
            with self._lock:
                done = self._runs[selected]["_done"]
            done.wait(self._grace_s + 1)
            return self.status(str(selected))
        return snapshot

    def close(self) -> None:
        with self._lock:
            self._closed = True
            ids = tuple(self._runs)
        for run_id in ids:
            self.revoke(run_id, "RUNTIME_CLOSED")
        deadline = time.monotonic() + self._grace_s + 1
        for run_id in ids:
            with self._lock:
                done = self._runs[run_id]["_done"]
            done.wait(max(0, deadline - time.monotonic()))

    @staticmethod
    def _signal_group(process: subprocess.Popen, sig: signal.Signals) -> None:
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            pass

    def _notify(self, record: dict[str, object]) -> None:
        with self._lock:
            if record["state"] not in _TERMINAL or record["_notified"]:
                return
            record["_notified"] = True
            snapshot = self._public(record)
        if self._on_terminal is not None:
            try:
                self._on_terminal(snapshot)
            except Exception as exc:
                with self._lock:
                    record["callback_error"] = f"{type(exc).__name__}:{exc}"[:1024]

    def _terminal(self, record: dict[str, object], state: str, *, result=None, error=None) -> None:
        with self._lock:
            if record["state"] in _ACTIVE:
                record.update(state=state, result=result, error=error, finished_ns=self._clock_ns(),
                              revision=int(record["revision"]) + 1, finalizing=True)

    def _supervise(self, record: dict[str, object], request: dict[str, object]) -> None:
        process = None
        result_read = result_write = None
        liveness_read = liveness_write = None
        selector = selectors.DefaultSelector()
        try:
            with self._lock:
                if record["state"] not in _ACTIVE:
                    return
            result_read, result_write = os.pipe()
            liveness_read, liveness_write = os.pipe()
            environment = os.environ.copy()
            package_root = str(Path(__file__).resolve().parent.parent)
            environment["PYTHONPATH"] = package_root + os.pathsep + environment.get("PYTHONPATH", "")
            process = subprocess.Popen(
                [sys.executable, "-m", "r2b4_orchestration.skill_worker", "--result-fd", str(result_write),
                 "--liveness-fd", str(liveness_read)],
                cwd=self.project_root, env=environment, stdin=subprocess.PIPE,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, pass_fds=(result_write, liveness_read),
                start_new_session=True, close_fds=True,
            )
            os.close(result_write)
            result_write = None
            os.close(liveness_read)
            liveness_read = None
            with self._lock:
                record.update(pid=process.pid, _process=process)
            for stream, label in ((process.stdout, "stdout"), (process.stderr, "stderr"),
                                  (result_read, "result"), (process.stdin, "stdin")):
                os.set_blocking(stream if isinstance(stream, int) else stream.fileno(), False)
                selector.register(stream, selectors.EVENT_WRITE if label == "stdin" else selectors.EVENT_READ, label)
            pending = json.dumps(request, allow_nan=False, separators=(",", ":")).encode("utf-8")
            output = {"stdout": b"", "stderr": b""}
            result_buffer = b""
            terminal_packet = None
            cancel_started = exit_seen = None
            killed = False
            while True:
                with self._lock:
                    cancelled = record["state"] == "CANCELLED"
                if cancelled and cancel_started is None:
                    cancel_started = time.monotonic()
                    self._signal_group(process, signal.SIGTERM)
                if cancel_started is not None and not killed and time.monotonic() - cancel_started >= self._grace_s:
                    self._signal_group(process, signal.SIGKILL)
                    killed = True
                # Retain the exited leader until cleanup. Reaping early would
                # allow its PID/group identity to be reused before killpg.
                exited = os.waitid(os.P_PID, process.pid, os.WEXITED | os.WNOHANG | os.WNOWAIT)
                if exited is not None:
                    exit_seen = exit_seen or time.monotonic()
                for key, _ in selector.select(0.025):
                    label = key.data
                    fd = key.fd
                    if label == "stdin":
                        try:
                            written = os.write(fd, pending[:8192])
                            pending = pending[written:]
                        except BrokenPipeError:
                            pending = b""
                        if not pending:
                            selector.unregister(key.fileobj)
                            process.stdin.close()
                        continue
                    chunk = os.read(fd, 8192)
                    if not chunk:
                        selector.unregister(key.fileobj)
                        continue
                    if label in output:
                        tail = output[label] + chunk
                        dropped = max(0, len(tail) - self._output_limit)
                        output[label] = tail[-self._output_limit:]
                        with self._lock:
                            record[label] = output[label].decode("utf-8", errors="replace")
                            record["output_dropped_bytes"] += dropped
                    else:
                        result_buffer += chunk
                        if len(result_buffer) > MAX_RESULT_BYTES:
                            self._terminal(record, "FAILED", error="WORKER_RESULT_BOUND_EXCEEDED")
                            self._signal_group(process, signal.SIGKILL)
                            result_buffer = b""
                        while b"\n" in result_buffer:
                            line, result_buffer = result_buffer.split(b"\n", 1)
                            packet = json.loads(line)
                            if packet.get("state") == "RUNNING":
                                with self._lock:
                                    if record["state"] == "STARTING":
                                        record.update(state="RUNNING", revision=int(record["revision"]) + 1)
                            elif packet.get("state") in _TERMINAL:
                                terminal_packet = packet
                # Child processes can inherit pipes. Worker exit cannot turn them into an unbounded drain wait.
                if exit_seen is not None and (not selector.get_map() or time.monotonic() - exit_seen > 0.1):
                    break
            self._signal_group(process, signal.SIGKILL)
            process.wait(timeout=1)
            with self._lock:
                record["returncode"] = process.returncode
            if terminal_packet is not None and (process.returncode == 0 or terminal_packet.get("state") == "FAILED"):
                self._terminal(record, str(terminal_packet["state"]),
                               result=terminal_packet.get("result"), error=terminal_packet.get("error"))
            else:
                self._terminal(record, "FAILED", error=f"WORKER_EXIT:{process.returncode}")
        except BaseException as exc:
            self._terminal(record, "FAILED", error=f"WORKER_ERROR:{type(exc).__name__}:{exc}"[:4096])
            if process is not None:
                self._signal_group(process, signal.SIGKILL)
                try:
                    process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    pass
        finally:
            selector.close()
            for fd in (result_read, result_write, liveness_read, liveness_write):
                if fd is not None:
                    os.close(fd)
            if process is not None:
                for stream in (process.stdin, process.stdout, process.stderr):
                    if stream is not None:
                        stream.close()
            self._notify(record)
            with self._lock:
                record["finalizing"] = False
                record["_done"].set()
                self._trim()


__all__ = ["SkillRuntime"]
