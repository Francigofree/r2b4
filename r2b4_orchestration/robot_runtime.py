"""Shared host owner for public world knowledge and behavior lifecycle.

This small, robot-specific service reads compact resident status and requests
actions through RobotInterface. V3 never imports it or waits for it. The local
socket carries only public state/behavior requests, never sensor or image data.
"""
from __future__ import annotations

import argparse
import dataclasses
import fcntl
import hashlib
import json
import os
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import uuid
from collections.abc import Mapping
from collections import deque
from pathlib import Path

from r2b4_orchestration.behavior_system import BehaviorSystem
from r2b4_orchestration.world_model import PublicWorldModel, WorldQuery, WorldQueryResult
from r2b4_orchestration.semantic_projector import SemanticProjector

SCHEMA = "R2B4_PUBLIC_ROBOT_RUNTIME_V1"
MAX_REQUEST_BYTES = 65_536
MAX_REPLY_BYTES = 1_048_576
READS = frozenset({"robot.state", "world.snapshot", "world.history", "behavior.state", "behavior.history"})
QUERIES = frozenset({"world.query"})
ACTIONS = frozenset({"world.observe", "behavior.room_cruise", "behavior.follow_person",
                     "behavior.search_person", "behavior.start", "behavior.cancel"})


def socket_path_for(root: Path) -> Path:
    identity = hashlib.sha256(str(root.resolve()).encode()).hexdigest()[:16]
    return Path(tempfile.gettempdir()) / f"r2b4_robot_{os.getuid()}_{identity}.sock"


def _jsonable(value: object) -> object:
    if hasattr(value, "to_jsonable"):
        return value.to_jsonable()
    if dataclasses.is_dataclass(value):
        return dataclasses.asdict(value)
    if isinstance(value, (tuple, list)):
        return [_jsonable(item) for item in value]
    return value


def _recv(conn: socket.socket, limit: int) -> dict:
    with conn.makefile("rb") as reader:
        line = reader.readline(limit + 1)
    if not line or len(line) > limit or not line.endswith(b"\n"):
        raise ValueError("public robot message exceeded its bound or was incomplete")
    value = json.loads(line)
    if not isinstance(value, dict):
        raise ValueError("public robot request must be an object")
    return value


def _send(conn: socket.socket, value: object, limit: int) -> None:
    payload = json.dumps(value, allow_nan=False, separators=(",", ":")).encode() + b"\n"
    if len(payload) > limit:
        raise ValueError("public robot message exceeded its bound")
    conn.sendall(payload)


class PublicRobotClient:
    """Bounded local consumer. Read/start never boots V3 on its own."""

    def __init__(self, root: Path, *, socket_path: Path | None = None, timeout_s: float = 35.0):
        self.root = root.resolve()
        self.socket_path = socket_path or socket_path_for(self.root)
        self.timeout_s = timeout_s
        self._peer_pid = None

    def request(self, operation: str, *, launch: bool = True, timeout_s: float | None = None,
                **arguments: object) -> object:
        # Validate before launching a process or sending any request.
        request = {"schema": SCHEMA, "operation": operation, **arguments}
        rendered = json.dumps(request, allow_nan=False).encode()
        if len(rendered) >= MAX_REQUEST_BYTES:
            raise ValueError("public robot request exceeded its bound")
        deadline = time.monotonic() + (self.timeout_s if timeout_s is None else timeout_s)
        conn = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        conn.settimeout(min(0.5, self.timeout_s))
        try:
            try:
                conn.connect(str(self.socket_path))
            except (FileNotFoundError, ConnectionRefusedError):
                conn.close()
                if not launch:
                    return None
                subprocess.Popen(
                    [sys.executable, "-m", "r2b4_orchestration.robot_runtime", "--root", str(self.root),
                     "--socket", str(self.socket_path)], cwd=self.root, stdin=subprocess.DEVNULL,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    start_new_session=True, close_fds=True,
                )
                while True:
                    conn = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                    conn.settimeout(0.5)
                    try:
                        conn.connect(str(self.socket_path))
                        break
                    except (FileNotFoundError, ConnectionRefusedError):
                        conn.close()
                        if time.monotonic() >= deadline:
                            raise RuntimeError("PUBLIC_ROBOT_RUNTIME_UNAVAILABLE")
                        time.sleep(0.025)
            if hasattr(socket, "SO_PEERCRED"):
                pid, uid, _ = struct.unpack("3i", conn.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))
                if uid != os.getuid():
                    raise RuntimeError("public robot owner user mismatch")
                self._peer_pid = pid
            conn.settimeout(max(0.01, deadline - time.monotonic()))
            _send(conn, request, MAX_REQUEST_BYTES)
            response = _recv(conn, MAX_REPLY_BYTES)
            if response.get("schema") != SCHEMA:
                raise RuntimeError("public robot response schema mismatch")
            if response.get("error"):
                raise RuntimeError(str(response["error"]))
            return response.get("result")
        finally:
            conn.close()

    def preempt(self, reason: str) -> object:
        try:
            return self.request("preempt", launch=False, reason=reason)
        except (OSError, RuntimeError, ValueError):
            self._terminate_unresponsive_owner()
            raise

    def query(self, query: WorldQuery) -> WorldQueryResult:
        if not isinstance(query, WorldQuery):
            raise TypeError("public world query must be a WorldQuery")
        return WorldQueryResult.from_jsonable(self.request("query", query=query.to_jsonable()))

    def revoke(self, reason: str) -> object:
        try:
            return self.request("revoke", launch=False, timeout_s=0.5, reason=reason)
        except (OSError, RuntimeError, ValueError):
            self._terminate_unresponsive_owner()
            raise

    def _terminate_unresponsive_owner(self) -> None:
        # Fail-closed fallback is limited to the verified peer of this project's
        # public host service. Its command producer also watches this owner PID.
        if self._peer_pid is None:
            return
        try:
            arguments = Path(f"/proc/{self._peer_pid}/cmdline").read_bytes().split(b"\0")
            if (b"r2b4_orchestration.robot_runtime" in arguments
                    and str(self.root).encode() in arguments
                    and str(self.socket_path).encode() in arguments):
                os.kill(self._peer_pid, signal.SIGTERM)
        except (OSError, ValueError):
            pass


class PublicRobotRuntime:
    """One host state owner; all physical realization stays in the backend."""

    def __init__(self, interface: object, *, root: Path | None = None,
                 world: PublicWorldModel | None = None, clock_ns=time.monotonic_ns):
        self.interface = interface
        self.root = root
        self.clock_ns = clock_ns
        self.world = world or PublicWorldModel(clock_ns=clock_ns)
        self._state_lock = threading.RLock()
        self._storage_lock = threading.Lock()
        self._action_lock = threading.Lock()
        self._generation = 0
        self._last_persist_ns = 0
        self._health_error = None
        self._persisted_revision = -1
        self._evidence_queue = deque(maxlen=256)
        self._evidence_lock = threading.Lock()
        self._evidence_write_lock = threading.Lock()
        self._evidence_sequence = 0
        self._evidence_dropped = 0
        self._reported_evidence_dropped = 0
        self._evidence_producer_id = str(uuid.uuid4())
        self._world_evidence_pending = 0
        self.world.set_event_sink(self._world_event)
        self.projector = SemanticProjector(self.world, clock_ns=clock_ns)
        self._caller_pid = None
        program_interface = interface
        if hasattr(interface, "adapters"):
            from v3.robot_interface import RobotInterface
            program_interface = RobotInterface(
                project_root=root, controller=interface.controller,
                adapters=(*interface.adapters, PublicRobotStateAdapter(self)), upper_runtime=False,
            )
        self.behaviors = BehaviorSystem(program_interface, clock_ns=clock_ns, event_sink=self._behavior_event)
        from r2b4_orchestration.search_person import SearchPerson
        self.behaviors.register("search_person", SearchPerson)
        if root is not None:
            path = root / "runtime" / "public_world" / "state.json"
            if path.exists():
                try:
                    if path.stat().st_size > MAX_REPLY_BYTES:
                        raise ValueError("saved world exceeded its bound")
                    self.world.restore(json.loads(path.read_text()))
                except (OSError, TypeError, ValueError, KeyError) as exc:
                    self._health_error = f"WORLD_RESTORE_FAILED:{type(exc).__name__}:{exc}"

    def _evidence(self, record: Mapping[str, object]) -> None:
        if self.root is None:
            return
        directory = self.root / "runtime" / "public_world"
        directory.mkdir(parents=True, exist_ok=True)
        with (directory / "events.ndjson").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, allow_nan=False, separators=(",", ":")) + "\n")

    def _event_record(self, kind: str, value: object) -> dict[str, object]:
        self._evidence_sequence += 1
        return {"schema": SCHEMA, "kind": kind, "publication_time_ns": self.clock_ns(),
                "clock_epoch": self.world.clock_epoch, "value": _jsonable(value),
                "producer_id": self._evidence_producer_id, "event_sequence": self._evidence_sequence,
                "evidence_dropped": self._evidence_dropped}

    def _queue_evidence(self, kind: str, value: object, *, world_input: bool = False) -> None:
        with self._evidence_lock:
            if (len(self._evidence_queue) == self._evidence_queue.maxlen
                    or world_input and self._world_evidence_pending >= 4):
                self._evidence_sequence += 1
                self._evidence_dropped += 1
                self._health_error = "PUBLIC_RUNTIME_EVIDENCE_OVERFLOW"
                return
            self._evidence_queue.append(self._event_record(kind, value))
            if world_input:
                self._world_evidence_pending += 1

    def _world_event(self, event: object) -> None:
        self._queue_evidence("observation", event)

    def _behavior_event(self, event: object) -> None:
        # Revocation and STOP never wait for filesystem evidence writes.
        self._queue_evidence("behavior", event)

    def world_input_evidence(self, snapshot: Mapping[str, object]) -> None:
        # Only behavior inputs are recorded here; ordinary UI state reads do not
        # generate repeated evidence. Keep the public, compact semantic snapshot
        # so an intent's world revision remains reconstructable after supersede.
        self._queue_evidence("world_input", snapshot, world_input=True)

    def _flush_evidence(self) -> None:
        with self._evidence_write_lock:
            with self._evidence_lock:
                events = list(self._evidence_queue)
                self._evidence_queue.clear()
                self._world_evidence_pending = 0
                if self._evidence_dropped != self._reported_evidence_dropped:
                    events.append(self._event_record("evidence_loss", {"dropped": self._evidence_dropped}))
                    self._reported_evidence_dropped = self._evidence_dropped
            for index, event in enumerate(events):
                try:
                    self._evidence(event)
                except (OSError, ValueError) as exc:
                    with self._evidence_lock:
                        self._evidence_dropped += len(events) - index
                    self._health_error = f"EVIDENCE_WRITE_FAILED:{type(exc).__name__}:{exc}"
                    break

    def ingest_status(self, status: Mapping[str, object], *, runtime_pid: object = None) -> None:
        """Consume completed status; preserve source times, frame and session."""
        self.projector.completed_status(status, runtime_pid=runtime_pid)

    def poll(self) -> None:
        try:
            status = self.interface.read("v3.status")
        except (RuntimeError, OSError, ValueError):
            status = None
        runtime_pid = None
        if isinstance(status, Mapping):
            try:
                runtime_pid = self.interface.read("operator.status").get("runtime_pid")
            except (RuntimeError, OSError, ValueError, AttributeError):
                pass
        with self._state_lock:
            if isinstance(status, Mapping):
                self.ingest_status(status, runtime_pid=runtime_pid)
        # No host lock around execution/waits: STOP must revoke STARTING too.
        with self._action_lock:
            self.behaviors.step()
        if self.behaviors.active and self._caller_pid is not None:
            try:
                os.kill(self._caller_pid, 0)
            except ProcessLookupError:
                self.preempt("REQUEST_OWNER_EXITED")
        with self._state_lock:
            self.projector.completed_behavior(self.behaviors.snapshot())
        self._persist()
        self._flush_evidence()

    def poll_safely(self) -> None:
        try:
            self.poll()
        except Exception as exc:
            self._health_error = f"PUBLIC_POLL_FAILED:{type(exc).__name__}:{exc}"
            if self.behaviors.active:
                try:
                    self.preempt("HOST_STEP_FAILED")
                except Exception:
                    pass

    def _persist(self, *, force: bool = False) -> None:
        # Storage serialization is separate from intent revocation. Slow disk
        # cannot retain the state lock needed by STOP and public state reads.
        with self._storage_lock:
            self._persist_snapshot(force=force)

    def _persist_snapshot(self, *, force: bool = False) -> None:
        if self.root is None:
            return
        now = self.clock_ns()
        if not force and now - self._last_persist_ns < 5_000_000_000:
            return
        self._last_persist_ns = now
        state = self.world.export_state()
        revision = (state.get("world_revision", state.get("revision")), state.get("event_sequence"))
        if revision == self._persisted_revision:
            return
        directory = self.root / "runtime" / "public_world"
        try:
            directory.mkdir(parents=True, exist_ok=True)
            payload = json.dumps(state, allow_nan=False, separators=(",", ":"))
            if len(payload.encode()) > MAX_REPLY_BYTES:
                raise ValueError("world state exceeded storage bound")
            with tempfile.NamedTemporaryFile("w", dir=directory, prefix=".state-", delete=False) as stream:
                temporary = Path(stream.name)
                stream.write(payload)
            try:
                os.replace(temporary, directory / "state.json")
            finally:
                temporary.unlink(missing_ok=True)
            self._persisted_revision = revision
        except (OSError, ValueError) as exc:
            self._health_error = f"WORLD_SAVE_FAILED:{type(exc).__name__}:{exc}"

    def read(self, resource: str) -> object:
        with self._state_lock:
            if resource == "world.snapshot":
                return self.world.snapshot().to_jsonable()
            if resource == "world.history":
                return _jsonable(self.world.history())
            if resource == "behavior.state":
                return self.behaviors.snapshot().to_jsonable()
            if resource == "behavior.history":
                return _jsonable(self.behaviors.history())
            if resource == "robot.state":
                return {"schema": SCHEMA, "observation_time_ns": self.clock_ns(),
                        "world": self.world.snapshot().to_jsonable(),
                        "active_behavior": self.behaviors.snapshot().to_jsonable(),
                        "active_mission": self.world.read("robot", "mission").to_jsonable(),
                        "health": {"public_runtime": "DEGRADED" if self._health_error else "AVAILABLE",
                                   "reason": self._health_error}}
        raise KeyError(resource)

    def query(self, query: WorldQuery) -> WorldQueryResult:
        with self._state_lock:
            return self.world.query(query)

    def preempt(self, reason: str) -> object:
        self.revoke(reason)
        try:
            self.interface.stop()
        finally:
            with self._action_lock:
                # A revoked submission could have passed its port check just before
                # the first STOP and entered the operator transition afterwards.
                # No replacement can be admitted until this final STOP completes.
                self.revoke(reason)
                self.interface.stop()
        return self.behaviors.snapshot().to_jsonable()

    def revoke(self, reason: str) -> object:
        with self._state_lock:
            self._generation += 1
            self.behaviors.revoke(reason)
            return self.behaviors.snapshot().to_jsonable()

    def execute(self, action: str, parameters: Mapping[str, object]) -> object:
        params = dict(parameters)
        if action == "world.observe":
            with self._state_lock:
                event = self.world.observe(**params)
            self._persist(force=True)
            return _jsonable(event)
        if action == "behavior.cancel":
            reason = params.pop("reason", "USER_CANCEL")
            if params or not isinstance(reason, str) or not reason:
                raise ValueError("behavior.cancel accepts only a nonempty reason")
            return self.preempt(reason)
        names = {"behavior.room_cruise": "room_cruise", "behavior.follow_person": "follow_person",
                 "behavior.search_person": "search_person"}
        if action == "behavior.start":
            name = params.pop("name", None)
            if name not in self.behaviors.names:
                raise ValueError("behavior.start requires a registered behavior name")
            names[action] = name
        if action not in names:
            raise KeyError(action)
        duration = params.pop("max_duration_s", params.get("session_watchdog_s", 300.0))
        # Service death ends its autonomous command at the existing producer
        # owner/watchdog boundary. Keep caller watchdog if it is more restrictive.
        caller_pid = params.pop("session_owner_pid", None)
        if caller_pid is not None and (type(caller_pid) is not int or caller_pid <= 0):
            raise ValueError("session_owner_pid must be a positive integer")
        params["session_owner_pid"] = os.getpid()
        params.setdefault("session_watchdog_s", duration)
        with self._state_lock:
            # Capture admission before waiting for an earlier action. A STOP
            # invalidates every previously queued request as well as ACTIVE.
            self._generation += 1
            generation = self._generation
            self.behaviors.revoke("PREEMPTED_BY:" + names[action])
        self.interface.stop()
        with self._action_lock:
            with self._state_lock:
                if generation != self._generation:
                    raise RuntimeError("behavior start revoked before admission")
                self._caller_pid = caller_pid
            self.interface.stop()
            with self._state_lock:
                if generation != self._generation:
                    raise RuntimeError("behavior start revoked before admission")
            state = self.behaviors.start(names[action], params, max_duration_s=duration)
            if not self.behaviors.active:
                self.interface.stop()
            return state.to_jsonable()


class PublicRobotInterfaceAdapter:
    name = "public_robot"
    capability_names = READS | QUERIES | ACTIONS

    def __init__(self, client: PublicRobotClient, controller: object | None = None):
        self.client = client
        self.controller = controller

    def capabilities(self):
        from v3.action_catalog import action_descriptor
        result = {name: {"kind": "read" if name in READS | QUERIES else "action", "supported": True,
                       "available": True, "ready": True, "owner": "host",
                       "reason": "V3_INDEPENDENT_PUBLIC_STATE" if name in READS | QUERIES or name == "world.observe"
                       else "CANONICAL_V3_EXECUTION",}
                for name in self.capability_names}
        for name, item in result.items():
            descriptor = action_descriptor(name)
            if descriptor is not None:
                item.update(descriptor.to_jsonable())
        if self.controller is not None:
            status = self.controller.live_runtime_status()
            if isinstance(status, Mapping) and (status.get("fault_layer") or status.get("safety_decision") == "FAULT"):
                for name in ("behavior.room_cruise", "behavior.follow_person", "behavior.search_person", "behavior.start"):
                    result[name].update(available=False, ready=False, reason="RUNTIME_FAULT")
            from v3.adapters.vision_media_socket import VisionClient
            camera = VisionClient(root=self.controller.root).status()
            if camera.get("camera_state") == "FAILED":
                for name in ("behavior.follow_person", "behavior.search_person"):
                    if result[name]["available"]:
                        result[name].update(available=False, ready=False, reason="VISION_UNAVAILABLE")
        return result

    def read(self, resource: str) -> object:
        return self.client.request("read", resource=resource)

    def query(self, query: WorldQuery) -> WorldQueryResult:
        if not isinstance(query, WorldQuery):
            raise TypeError("public world query must be a WorldQuery")
        return WorldQueryResult.from_jsonable(self.client.request("query", query=query.to_jsonable()))

    def execute(self, action: str, **parameters: object) -> object:
        return self.client.request("execute", action=action, parameters=parameters)


class PublicRobotStateAdapter:
    """The behavior's injected RobotInterface reads the same owned world state."""
    name = "public_world_state"
    capability_names = READS | QUERIES

    def __init__(self, runtime: PublicRobotRuntime):
        self.runtime = runtime

    def capabilities(self):
        return {name: {"kind": "read", "supported": True, "available": True, "ready": True}
                for name in self.capability_names}

    def read(self, resource: str):
        result = self.runtime.read(resource)
        if resource == "world.snapshot":
            self.runtime.world_input_evidence(result)
        return result

    def query(self, query: WorldQuery) -> WorldQueryResult:
        result = self.runtime.query(query)
        self.runtime._queue_evidence("world_query", result)
        return result

    def execute(self, action: str, **parameters: object):
        raise KeyError(action)


def serve(root: Path, socket_path: Path) -> int:
    from v3.robot_interface import RobotInterface
    from v3.runtime_performance import apply_host_affinity

    apply_host_affinity(root, "operator")
    socket_path.parent.mkdir(parents=True, exist_ok=True)
    with Path(str(socket_path) + ".lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return 0
        socket_path.unlink(missing_ok=True)
        backend = RobotInterface(project_root=root, upper_runtime=False)
        runtime = PublicRobotRuntime(backend, root=root)
        stop_event = threading.Event()
        slots = threading.BoundedSemaphore(8)
        stop_slots = threading.BoundedSemaphore(2)

        def poll():
            while not stop_event.wait(0.2):
                runtime.poll_safely()

        def handle(conn, request, admission_slots):
            with conn:
                try:
                    conn.settimeout(35.0)
                    if request.get("schema") != SCHEMA:
                        raise ValueError("public robot request schema mismatch")
                    operation = request.get("operation")
                    if operation == "read" and request.get("resource") in READS:
                        result = runtime.read(request["resource"])
                    elif operation == "query":
                        result = runtime.query(WorldQuery.from_jsonable(request.get("query"))).to_jsonable()
                    elif operation == "execute" and request.get("action") in ACTIONS:
                        result = runtime.execute(request["action"], request.get("parameters", {}))
                    elif operation == "preempt":
                        result = runtime.preempt(str(request.get("reason", "PREEMPTED")))
                    elif operation == "revoke":
                        result = runtime.revoke(str(request.get("reason", "STOP")))
                    else:
                        raise ValueError("unknown public robot operation")
                    _send(conn, {"schema": SCHEMA, "result": result}, MAX_REPLY_BYTES)
                except Exception as exc:
                    try:
                        _send(conn, {"schema": SCHEMA, "error": f"{type(exc).__name__}:{exc}"}, MAX_REPLY_BYTES)
                    except OSError:
                        pass
                finally:
                    admission_slots.release()

        poller = threading.Thread(target=poll, name="public-robot-observation", daemon=True)
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
            server.bind(str(socket_path))
            os.chmod(socket_path, 0o600)
            server.listen(8)
            poller.start()
            try:
                while True:
                    conn, _ = server.accept()
                    try:
                        conn.settimeout(0.25)
                        request = _recv(conn, MAX_REQUEST_BYTES)
                    except (OSError, ValueError):
                        conn.close()
                        continue
                    # Reserve independent admission for revocation. Eight
                    # blocked ordinary clients cannot exclude the STOP path.
                    admission_slots = stop_slots if request.get("operation") in {"preempt", "revoke"} else slots
                    if not admission_slots.acquire(blocking=False):
                        conn.close()
                        continue
                    threading.Thread(target=handle, args=(conn, request, admission_slots), daemon=True).start()
            finally:
                stop_event.set()
                runtime._persist(force=True)
                socket_path.unlink(missing_ok=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--socket", type=Path)
    args = parser.parse_args(argv)
    root = args.root.resolve()
    return serve(root, args.socket or socket_path_for(root))


if __name__ == "__main__":
    raise SystemExit(main())
