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
from r2b4_orchestration.brain_core import BrainCore
from r2b4_orchestration.world_model import PublicWorldModel, WorldQuery, WorldQueryResult
from r2b4_orchestration.semantic_projector import SemanticProjector
from r2b4_orchestration.spatial_service import SpatialQuery, SpatialQueryResult, SpatialService
from r2b4_orchestration.person_identity import PersonIdentity, PersonTeaching
from r2b4_orchestration.person_skills import PERSON_SKILLS, person_behavior_factories, skill_descriptor
from r2b4_orchestration.outcome_learning import OutcomeLearner

SCHEMA = "R2B4_PUBLIC_ROBOT_RUNTIME_V1"
MAX_REQUEST_BYTES = 65_536
MAX_REPLY_BYTES = 1_048_576
READS = frozenset({"robot.state", "world.snapshot", "world.history", "behavior.state", "behavior.history",
                   "brain.state", "brain.history", "spatial.snapshot"})
QUERIES = frozenset({"world.query", "spatial.query"})
ACTIONS = frozenset({"world.observe", "behavior.room_cruise", "behavior.follow_person",
                     "behavior.search_person", "behavior.search_any_person", "behavior.start", "behavior.cancel",
                     "brain.submit", "brain.adopt", "brain.fail", "brain.cancel", "person.teach",
                     "person.validate_target", "spatial.load_atlas", "spatial.teach_place"})
SKILL_ACTIONS = frozenset({"skill.create", "skill.update", "skill.test", "skill.run", "skill.stop", "skill.revoke",
                           "skill.status", "skill.source"})
SKILL_READS = frozenset({"skill.list"})


def skill_capabilities(library=None):
    descriptions = {
        "skill.list": "List saved Python skills without importing their code.",
        "skill.create": "Save a new ordinary Python skill with async def run(robot, **parameters).",
        "skill.update": "Save a new source version; the running worker keeps its loaded version.",
        "skill.run": "Start a saved Python skill in a separate interpreter; returns a run_id.",
        "skill.stop": "Revoke and stop a skill and its own canonical motion.",
        "skill.revoke": "Revoke an invocation immediately without waiting for worker cleanup.",
        "skill.status": "Read a run's state, returned result or error; success is program completion.",
        "skill.source": "Read the saved source and its hash for a targeted repair.",
        "skill.test": "Run the skill's optional saved pytest through the canonical launcher; no physical-goal proof.",
        "hri.report": "Record a report in the host journal; audio delivery requires a voice capability.",
    }
    params = {
        "skill.create": {"name": {"type": "string", "required": True}, "source": {"type": "string", "required": True},
                         "description": {"type": "string"}},
        "skill.update": {"name": {"type": "string", "required": True}, "source": {"type": "string", "required": True},
                         "description": {"type": "string"}},
        "skill.run": {"name": {"type": "string", "required": True}, "parameters": {"type": "object"}},
        "skill.source": {"name": {"type": "string", "required": True}},
        "skill.status": {"run_id": {"type": "string"}},
        "skill.stop": {"run_id": {"type": "string"}, "reason": {"type": "string"}},
        "skill.revoke": {"run_id": {"type": "string"}, "reason": {"type": "string"}},
        "skill.test": {"name": {"type": "string", "required": True}, "timeout_s": {"type": "number"}},
        "hri.report": {"text": {"type": "string", "required": True}, "destination": {"type": "string"}},
    }
    result = {name: {"name": name, "kind": "read" if name in SKILL_READS else "action",
                     "description": description, "parameters": params.get(name, {}),
                     "result": {"description": "Saved descriptor or invocation state/result."},
                     "supported": True, "available": True, "ready": True, "owner": "host"}
              for name, description in descriptions.items()}
    if library is not None:
        for descriptor in library.list():
            availability = descriptor.get("availability", {})
            result["skill." + descriptor["name"]] = {**descriptor, "name": "skill." + descriptor["name"],
                "skill_name": descriptor["name"], "kind": "action", "owner": "host", "supported": True,
                "available": availability.get("available", True), "ready": availability.get("ready", True)}
    return result


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

    def spatial_query(self, query: SpatialQuery) -> SpatialQueryResult:
        if not isinstance(query, SpatialQuery):
            raise TypeError("spatial query must be a SpatialQuery")
        return SpatialQueryResult.from_jsonable(self.request("spatial_query", query=query.to_jsonable()))

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
                 world: PublicWorldModel | None = None, clock_ns=time.monotonic_ns,
                 observation_hub=None, socket_path: Path | None = None):
        self.interface = interface
        self.root = root
        self.clock_ns = clock_ns
        self.world = world or PublicWorldModel(clock_ns=clock_ns)
        self.spatial = SpatialService(self.world, clock_ns=clock_ns)
        self._state_lock = threading.RLock()
        self._storage_lock = threading.Lock()
        self._action_lock = threading.RLock()
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
        self._brain_memory_sequence = 0
        # The host observation plane exists without V3 or capture. Publication
        # only enqueues immutable references; unavailable consumers never own
        # the Brain's lifecycle or determine whether a command can run.
        from v3.observation import ObservationHub
        self.observation_hub = observation_hub if observation_hub is not None else ObservationHub()
        self.world.set_event_sink(self._world_event)
        self.person_identity = PersonIdentity(self.world, clock_ns=clock_ns)
        self.learner = OutcomeLearner(self.world, clock_ns=clock_ns)
        self.projector = SemanticProjector(self.world, clock_ns=clock_ns, person_identity=self.person_identity)
        self._caller_pid = None
        self._sdk_events = deque(maxlen=128)
        self._sdk_event_sequence = 0
        self._sdk_event_lock = threading.Lock()
        self._skill_motion_run = None
        self._skill_cleanup_error = None
        self._skill_goals = {}
        self.software = None
        self.skill_library = None
        self.skills = None
        self.skill_observation = None
        if root is not None:
            from .skill_library import SkillLibrary
            from .skill_runtime import SkillRuntime
            from .skill_observation import SkillObservationAdapter
            from .software_interface import SoftwareInterfaceAdapter
            self.skill_library = SkillLibrary(root / "robot_skills")
            self.skills = SkillRuntime(self.skill_library, socket_path or socket_path_for(root),
                                       on_terminal=self._skill_terminal, clock_ns=clock_ns)
            self.skill_observation = SkillObservationAdapter(root)
            self.software = SoftwareInterfaceAdapter(root)
        program_interface = interface
        if hasattr(interface, "adapters"):
            from v3.robot_interface import RobotInterface
            if isinstance(interface, RobotInterface):
                interface.set_observation_sink(self._interface_event, clock_ns=clock_ns)
            program_interface = RobotInterface(
                project_root=root, controller=interface.controller,
                adapters=(*interface.adapters, PublicRobotStateAdapter(self)), upper_runtime=False,
                observation_sink=self._interface_event, clock_ns=clock_ns,
            )
        self.behaviors = BehaviorSystem(program_interface, clock_ns=clock_ns, event_sink=self._behavior_event)
        self.program_interface = program_interface
        for name, factory in person_behavior_factories():
            self.behaviors.register(name, factory)
        self.brain = BrainCore(program_interface, self.behaviors, self.world, clock_ns=clock_ns,
                               event_sink=self._brain_event, execution_lock=self._action_lock)
        if root is not None:
            path = root / "runtime" / "public_world" / "state.json"
            if path.exists():
                try:
                    if path.stat().st_size > MAX_REPLY_BYTES:
                        raise ValueError("saved world exceeded its bound")
                    saved = json.loads(path.read_text())
                    self.world.restore(saved)
                    if "spatial" in saved:
                        self.spatial.restore(saved["spatial"])
                    if "brain" in saved and self.brain.restore(saved["brain"]):
                        self.interface.stop()
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
        # Publish independently of journal/capture storage. The generic hub
        # neither serializes the payload nor invokes consumers synchronously.
        try:
            self.observation_hub.publish(value, topic="r2b4." + kind)
        except Exception:
            pass
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
        brain = getattr(self, "brain", None)
        if brain is not None:
            brain.notify("WORLD_FACT_UPDATED")
        self._queue_evidence("observation", event)
        self._sdk_event("world", event)

    def _behavior_event(self, event: object) -> None:
        # Revocation and STOP never wait for filesystem evidence writes.
        brain = getattr(self, "brain", None)
        if brain is not None:
            brain.notify("BEHAVIOR_UPDATED")
        self._queue_evidence("behavior", event)
        self._sdk_event("behavior", event)

    def _brain_event(self, event: object) -> None:
        self._queue_evidence("brain", event)
        self._sdk_event("brain", event)

    def _interface_event(self, event: object) -> None:
        self._queue_evidence("interface", event)
        self._sdk_event("interface", event)

    def _sdk_event(self, producer: str, event: object) -> None:
        # Only bounded immutable references are enqueued on producer callbacks.
        with self._sdk_event_lock:
            self._sdk_event_sequence += 1
            self._sdk_events.append((self._sdk_event_sequence, producer, event))

    def events(self, *, after_sequence=0, kinds=None, limit=32):
        if type(after_sequence) is not int or after_sequence < 0 or type(limit) is not int or not 1 <= limit <= 64:
            raise ValueError("invalid event cursor or page limit")
        if kinds is not None and (not isinstance(kinds, list) or any(not isinstance(k, str) for k in kinds)):
            raise ValueError("event kinds must be a list of producer names")
        with self._sdk_event_lock:
            rows = list(self._sdk_events)
            latest = self._sdk_event_sequence
        first = rows[0][0] if rows else latest + 1
        selected = [(seq, producer, value) for seq, producer, value in rows
                    if seq > after_sequence and (kinds is None or producer in kinds)]
        page = selected[:limit]
        return {"events": [{"sequence": seq, "producer": producer, "event": _jsonable(value)}
                            for seq, producer, value in page],
                "latest_sequence": latest,
                "next_sequence": page[-1][0] if len(selected) > limit else latest,
                "lost_events": max(0, first - after_sequence - 1)}

    def capabilities(self):
        catalog = self.interface.capabilities()
        items = dict(catalog.get("capabilities", {}))
        host = PublicRobotStateAdapter(self).capabilities()
        items.update(host)
        return {**catalog, "capabilities": items, "host_capabilities": list(host)}

    def _skill_terminal(self, state):
        run_id = state["run_id"]
        # Completion/error revokes further calls. A Python return never leaves
        # an accepted continuous command running without its invocation owner.
        try:
            if self._skill_motion_run == run_id:
                self.behaviors.revoke("SKILL_TERMINAL:" + state["state"])
                self.interface.stop()
                with self._action_lock:
                    if self._skill_motion_run == run_id:
                        self.interface.stop()
                        self._skill_motion_run = None
        except Exception as exc:
            self._skill_cleanup_error = "SKILL_CANONICAL_STOP_FAILED:" + type(exc).__name__
            self._health_error = self._skill_cleanup_error
            raise
        finally:
            if self.skill_observation is not None:
                self.skill_observation.close_invocation(run_id)
        self._sdk_event("skill", state)
        self._queue_evidence("skill", {key: state.get(key) for key in
                             ("run_id", "name", "state", "source_hash", "started_ns", "finished_ns", "error")})
        self.brain.notify("SKILL_UPDATED")
        self._skill_goals.pop(run_id, None)

    def close(self):
        self.preempt("HOST_CLOSED")
        if self.skills is not None:
            self.skills.close()
        if self.skill_observation is not None:
            self.skill_observation.close()

    def dispatch(self, request):
        """The socket and in-process clients use the same public owner routes."""
        if request.get("schema", SCHEMA) != SCHEMA:
            raise ValueError("public robot request schema mismatch")
        operation = request.get("operation")
        invocation = request.get("invocation_id")
        stopping = operation in {"stop", "preempt", "revoke"} or request.get("action") == "v3.command.stop"
        if invocation is not None and not stopping and (self.skills is None or not self.skills.accepts(invocation)):
            raise RuntimeError("SKILL_INVOCATION_REVOKED")
        if operation == "capabilities":
            return self.capabilities()
        if operation == "read":
            resource = request.get("resource")
            return self.read(resource) if resource in READS | SKILL_READS else self.interface.read(resource)
        if operation == "query":
            return self.query(WorldQuery.from_jsonable(request.get("query"))).to_jsonable()
        if operation == "spatial_query":
            return self.spatial_query(SpatialQuery.from_jsonable(request.get("query"))).to_jsonable()
        if operation == "events":
            return self.events(after_sequence=request.get("after_sequence", 0),
                               kinds=request.get("kinds"), limit=request.get("limit", 32))
        if operation in {"preempt", "stop"}:
            return self.preempt(str(request.get("reason", "STOP")))
        if operation == "revoke":
            return self.revoke(str(request.get("reason", "STOP")))
        if operation not in {"execute", "call"}:
            raise ValueError("unknown public robot operation")
        action, params = request.get("action"), request.get("parameters", {})
        if not isinstance(action, str) or not isinstance(params, Mapping):
            raise ValueError("action and parameters required")
        if action == "v3.command.stop":
            if params:
                raise ValueError("STOP accepts no parameters")
            return self.preempt("STOP")
        if self.skill_observation is not None and action in self.skill_observation.capability_names:
            result = self.skill_observation.call(action, dict(params), invocation_id=invocation)
            # An open request that raced revocation must release its new demand.
            if invocation is not None and not self.skills.accepts(invocation):
                self.skill_observation.close_invocation(invocation)
                raise RuntimeError("SKILL_INVOCATION_REVOKED")
            return result
        if self.software is not None and action in self.software.capability_names:
            return self.software.execute(action, **params)
        if action == "hri.report":
            return self.execute(action, params)
        if invocation is not None and action.startswith("behavior."):
            return self._skill_action(invocation, action, params)
        if action in ACTIONS | SKILL_ACTIONS or action.startswith("skill."):
            if invocation is not None and action == "skill.run":
                raise RuntimeError("NESTED_SKILL_RUN_USE_SDK_INVOKE")
            return self.execute(action, params)
        if invocation is not None:
            return self._skill_action(invocation, action, params)
        # Ordinary manual requests retain the RobotInterface preemption rules.
        if action.startswith("v3.command.") or action.startswith("operator."):
            self.preempt("PREEMPTED_BY:" + action)
        return self.interface.execute(action, **params)

    def _skill_action(self, run_id, action, parameters):
        physical = action.startswith(("v3.command.", "operator.", "behavior."))
        with self._action_lock:
            if not self.skills.accepts(run_id):
                raise RuntimeError("SKILL_INVOCATION_REVOKED")
            params = dict(parameters)
            if action.startswith("behavior."):
                if action == "behavior.cancel":
                    self.behaviors.revoke(params.get("reason", "SKILL_CANCEL"))
                    return self.interface.stop()
                name = params.pop("name", None) if action == "behavior.start" else action.removeprefix("behavior.")
                duration = params.pop("max_duration_s", 300.0)
                if self.behaviors.active:
                    raise RuntimeError("active behavior must be explicitly cancelled")
                params["session_owner_pid"] = os.getpid()
                params.setdefault("session_watchdog_s", duration)
                self._skill_motion_run = run_id
                state = self.behaviors.start(name, params, max_duration_s=duration,
                                             lineage=self._skill_goals.get(run_id))
                if not self.skills.accepts(run_id):
                    self.behaviors.revoke("SKILL_INVOCATION_REVOKED")
                    self.interface.stop()
                    raise RuntimeError("SKILL_INVOCATION_REVOKED")
                return state.to_jsonable()
            if physical:
                if self.behaviors.active:
                    self.behaviors.revoke("SKILL_MOTION_REPLACED")
                    self.interface.stop()
                self._skill_motion_run = run_id
                if action.startswith("v3.command."):
                    params["session_owner_pid"] = os.getpid()
            result = self.interface.execute(action, **params)
            if physical and not self.skills.accepts(run_id):
                self.interface.stop()
                raise RuntimeError("SKILL_INVOCATION_REVOKED")
            if physical and isinstance(result, Mapping) and result.get("status") == "COMPLETED":
                self._skill_motion_run = None
            return result

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

    def ingest_status(self, status: Mapping[str, object], *, runtime_pid: object = None,
                      vision_status: Mapping[str, object] | None = None) -> None:
        """Consume completed status; preserve source times, frame and session."""
        self.spatial.completed_status(status, runtime_pid=runtime_pid)
        self.projector.completed_status(status, runtime_pid=runtime_pid, vision_status=vision_status)
        self.spatial.sync_world()

    def poll(self) -> None:
        try:
            status = self.interface.read("v3.status")
        except (RuntimeError, OSError, ValueError):
            status = None
        runtime_pid = None
        vision_status = None
        if isinstance(status, Mapping):
            try:
                runtime_pid = self.interface.read("operator.status").get("runtime_pid")
            except (RuntimeError, OSError, ValueError, AttributeError):
                pass
        if self.person_identity.has_bindings:
            try:
                vision_status = self.interface.read("camera.status")
            except (RuntimeError, OSError, ValueError, KeyError):
                pass
        with self._state_lock:
            if isinstance(status, Mapping):
                self.ingest_status(status, runtime_pid=runtime_pid, vision_status=vision_status)
            else:
                self.person_identity.invalidate("PERSON_RUNTIME_UNAVAILABLE")
        # No host lock around execution/waits: STOP must revoke STARTING too.
        # Finite RobotInterface actions can wait in the dispatcher. Observation,
        # memory persistence and evidence drain keep running during that wait.
        self.brain.step(asynchronous=True)
        if self.skills is not None and self.skills.active and self.behaviors.active:
            self.behaviors.step()
        if self.behaviors.active and self._caller_pid is not None and not self.behaviors.snapshot().goal_id:
            try:
                os.kill(self._caller_pid, 0)
            except ProcessLookupError:
                self.preempt("REQUEST_OWNER_EXITED")
        with self._state_lock:
            self.projector.completed_behavior(self.behaviors.snapshot())
        self._project_goal_history()
        self._persist()
        self._flush_evidence()

    def poll_safely(self) -> None:
        try:
            self.poll()
        except Exception as exc:
            self._health_error = f"PUBLIC_POLL_FAILED:{type(exc).__name__}:{exc}"
            primary = self.brain.snapshot()["primary_goal"]
            if self.behaviors.active or isinstance(primary, Mapping) and primary.get("lifecycle") in {"STARTING", "ACTIVE"}:
                try:
                    self.preempt("HOST_STEP_FAILED")
                except Exception:
                    pass

    def _project_goal_history(self) -> None:
        # Read executive history directly. Passive hub/journal loss must never
        # turn command acceptance into success or erase shared task memory.
        history = self.brain.history()
        if history and history[0].sequence > self._brain_memory_sequence + 1:
            self._health_error = "BRAIN_MEMORY_HISTORY_GAP"
        for event in history:
            if event.sequence <= self._brain_memory_sequence:
                continue
            try:
                self.projector.completed_goal(event)
                self.learner.consume(event)
                if self.learner.error:
                    self._health_error = self.learner.error
            except Exception as exc:
                # Memory availability is neither physical authority nor a gate
                # for independent local actions or STOP.
                self._health_error = "BRAIN_MEMORY_PROJECTION_FAILED:" + type(exc).__name__
            self._brain_memory_sequence = event.sequence

    def _persist(self, *, force: bool = False) -> bool:
        # Storage serialization is separate from intent revocation. Slow disk
        # cannot retain the state lock needed by STOP and public state reads.
        with self._storage_lock:
            return self._persist_snapshot(force=force)

    def _persist_snapshot(self, *, force: bool = False) -> bool:
        if self.root is None:
            return False
        now = self.clock_ns()
        if not force and now - self._last_persist_ns < 5_000_000_000:
            return False
        self._last_persist_ns = now
        state = self.world.export_state()
        state["brain"] = self.brain.export_state()
        state["spatial"] = self.spatial.export_state()
        revision = (state.get("world_revision", state.get("revision")), state.get("event_sequence"),
                    state["brain"]["revision"], state["spatial"]["revision"])
        if revision == self._persisted_revision:
            return True
        directory = self.root / "runtime" / "public_world"
        try:
            directory.mkdir(parents=True, exist_ok=True)
            payload = json.dumps(state, allow_nan=False, separators=(",", ":"))
            if len(payload.encode()) > MAX_REPLY_BYTES:
                raise ValueError("world state exceeded storage bound")
            with tempfile.NamedTemporaryFile("w", dir=directory, prefix=".state-", delete=False) as stream:
                temporary = Path(stream.name)
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
            try:
                os.replace(temporary, directory / "state.json")
                descriptor = os.open(directory, os.O_RDONLY | os.O_DIRECTORY)
                try:
                    os.fsync(descriptor)
                finally:
                    os.close(descriptor)
            finally:
                temporary.unlink(missing_ok=True)
            self._persisted_revision = revision
            return True
        except (OSError, ValueError) as exc:
            self._health_error = f"WORLD_SAVE_FAILED:{type(exc).__name__}:{exc}"
            return False

    def read(self, resource: str) -> object:
        if resource == "skill.list":
            return self.skill_library.list() if self.skill_library is not None else []
        with self._state_lock:
            if resource == "world.snapshot":
                return self.world.snapshot().to_jsonable()
            if resource == "world.history":
                return _jsonable(self.world.history())
            if resource == "spatial.snapshot":
                return self.spatial.snapshot()
            if resource == "behavior.state":
                return self.behaviors.snapshot().to_jsonable()
            if resource == "behavior.history":
                return _jsonable(self.behaviors.history())
            if resource == "brain.state":
                return self.brain.snapshot()
            if resource == "brain.history":
                return _jsonable(self.brain.history())
            if resource == "robot.state":
                return {"schema": SCHEMA, "observation_time_ns": self.clock_ns(),
                        "world": self.world.snapshot().to_jsonable(),
                        "spatial": self.spatial.snapshot(),
                        "brain": self.brain.snapshot(),
                        "active_behavior": self.behaviors.snapshot().to_jsonable(),
                        "active_mission": self.world.read("robot", "mission").to_jsonable(),
                        "health": {"public_runtime": "DEGRADED" if self._health_error else "AVAILABLE",
                                   "reason": self._health_error}}
        raise KeyError(resource)

    def query(self, query: WorldQuery) -> WorldQueryResult:
        with self._state_lock:
            return self.world.query(query)

    def spatial_query(self, query: SpatialQuery) -> SpatialQueryResult:
        with self._state_lock:
            return self.spatial.query(query)

    def _person_context(self):
        """Read one stable source session without relabelling older evidence."""
        vision_before = self.interface.read("camera.status")
        runtime_before = self.interface.read("operator.status")
        status = self.interface.read("v3.status")
        vision = self.interface.read("camera.status")
        runtime = self.interface.read("operator.status")
        if (not all(isinstance(value, Mapping) for value in (vision_before, runtime_before, vision, runtime))
                or runtime_before.get("runtime_pid") != runtime.get("runtime_pid")
                or runtime_before.get("runtime_running") is not True or runtime.get("runtime_running") is not True
                or any(vision_before.get(key) != vision.get(key) for key in (
                    "owner_generation", "owner_pid", "owner_generation_started_ns"))):
            raise ValueError("PERSON_SOURCE_SESSION_CHANGED")
        return runtime, status, vision

    def preempt(self, reason: str) -> object:
        try:
            try:
                self.revoke(reason)
            finally:
                self.interface.stop()
        finally:
            with self._action_lock:
                # A revoked submission could have passed its port check just before
                # the first STOP and entered the operator transition afterwards.
                # No replacement can be admitted until this final STOP completes.
                self.revoke(reason)
                self.interface.stop()
                self._skill_motion_run = None
        if self.skills is not None:
            self.skills.stop(reason=reason)
        return self.behaviors.snapshot().to_jsonable()

    def revoke(self, reason: str) -> object:
        with self._state_lock:
            self._generation += 1
            self.brain.revoke(reason)
            self.behaviors.revoke(reason)
            if self.skills is not None:
                self.skills.revoke(reason=reason)
            return self.behaviors.snapshot().to_jsonable()

    def _execute_skill(self, action, params):
        if self.skills is None:
            raise RuntimeError("SKILL_RUNTIME_UNAVAILABLE")
        if action in {"skill.create", "skill.update"}:
            if set(params) - {"name", "source", "description", "test_source"}:
                raise ValueError("unknown skill save parameters")
            method = self.skill_library.create if action == "skill.create" else self.skill_library.update
            return method(**params)
        if action == "skill.source":
            if set(params) != {"name"}:
                raise ValueError("skill.source requires name")
            source = self.skill_library.load(params["name"])
            return {"name": source.name, "source": source.source, "source_hash": source.source_hash,
                    "source_path": str(source.path)}
        if action == "skill.status":
            return self.skills.status(**params)
        if action == "skill.test":
            from .software_interface import test_skill
            return test_skill(self.skill_library, **params)
        if action == "skill.revoke":
            return self.skills.revoke(**params)
        if action == "skill.stop":
            run_id = params.get("run_id") or self.skills.status().get("run_id")
            self.skills.revoke(**params)
            if run_id is not None and self._skill_motion_run == run_id:
                self.behaviors.revoke("SKILL_STOPPED")
                self.interface.stop()
                with self._action_lock:
                    if self._skill_motion_run == run_id:
                        self.interface.stop()
                        self._skill_motion_run = None
            return self.skills.stop(**params)
        if action != "skill.run":
            name = action.removeprefix("skill.")
            if name not in {row["name"] for row in self.skill_library.list()}:
                raise KeyError(action)
            params = {"name": name, "parameters": params}
        if self._skill_cleanup_error:
            raise RuntimeError(self._skill_cleanup_error)
        if set(params) - {"name", "parameters", "goal_id", "subtask_id"}:
            raise ValueError("unknown skill invocation parameters")
        goal_id = params.pop("goal_id", None)
        subtask_id = params.pop("subtask_id", None)
        if goal_id is None:
            # Standalone library invocation is a manual upper-intent replacement.
            self.preempt("PREEMPTED_BY:skill.run")
        with self._state_lock:
            generation = self._generation
        with self._action_lock:
            with self._state_lock:
                if generation != self._generation:
                    raise RuntimeError("skill start revoked before admission")
                if goal_id is not None:
                    goal = self.brain.goal(goal_id)
                    lifecycle = goal.get("lifecycle") if isinstance(goal, Mapping) else getattr(goal, "lifecycle", None)
                    if getattr(lifecycle, "value", lifecycle) not in {"STARTING", "ACTIVE"}:
                        raise RuntimeError("SKILL_GOAL_REVOKED")
                run_id = uuid.uuid4().hex
                self._skill_goals[run_id] = {key: value for key, value in
                    (("goal_id", goal_id), ("subtask_id", subtask_id)) if value is not None}
                try:
                    return self.skills.start(**params, run_id=run_id)
                except Exception:
                    self._skill_goals.pop(run_id, None)
                    raise

    def execute(self, action: str, parameters: Mapping[str, object]) -> object:
        params = dict(parameters)
        if action == "hri.report":
            if set(params) - {"text", "destination"} or not isinstance(params.get("text"), str) or not 0 < len(params["text"]) <= 4096:
                raise ValueError("hri.report requires bounded text")
            if params.get("destination", "journal") != "journal":
                raise RuntimeError("REPORT_AUDIO_DELIVERY_UNAVAILABLE")
            if self.root is None:
                raise RuntimeError("REPORT_JOURNAL_UNAVAILABLE")
            result = {"status": "RECORDED", "text": params["text"], "delivered_to": "journal",
                      "audio_delivered": False, "measurement_time_ns": self.clock_ns()}
            with self._evidence_write_lock:
                self._evidence({"schema": SCHEMA, "kind": "report", "value": result})
            self._sdk_event("report", result)
            return result
        if action.startswith("skill."):
            return self._execute_skill(action, params)
        if action == "spatial.load_atlas":
            if set(params) != {"path"} or not isinstance(params["path"], str) or not 0 < len(params["path"]) <= 4096:
                raise ValueError("spatial.load_atlas requires an explicit metadata path")
            reference = self.spatial.load_atlas_reference(params["path"])
            saved = self._persist(force=True)
            result = {"status": "LOADED", **reference.to_jsonable(),
                      "durability": "SAVED" if saved else "MEMORY_ONLY"}
            self._queue_evidence("atlas_reference_loaded", result)
            return result
        if action == "spatial.teach_place":
            from .world_model import _text
            if set(params) != {"entity_id", "name", "map_id", "viewpoint_id", "request_id"}:
                raise ValueError("spatial.teach_place requires entity, name, map, viewpoint and human request")
            params = {key: _text(value, key, 96 if key == "name" else 256) for key, value in params.items()}
            viewpoints = self.spatial.query(SpatialQuery(kind="viewpoints", entity_id=params["map_id"], limit=64)).viewpoints
            point = next((point for point in viewpoints if point.map_id == params["map_id"]
                          and point.viewpoint_id == params["viewpoint_id"]), None)
            if point is None:
                raise ValueError("ATLAS_VIEWPOINT_UNAVAILABLE")
            now = self.clock_ns()
            with self._state_lock:
                previous = self.world.read(params["entity_id"], "atlas_place")
                value = {key: params[key] for key in ("name", "map_id", "viewpoint_id", "request_id")}
                value.update(map_revision=point.map_revision, motion_authority=False, taught_by="HUMAN")
                # Reuse the bounded semantic facts/history, including restored
                # history. Replaying an older request must not undo a later
                # teaching or assign the same human request to another place.
                observations = [fact.observation for fact in self.world.snapshot().facts.values()]
                observations.extend(event.observation for event in self.world.history() if event.accepted)
                duplicate = False
                for observation in observations:
                    if (observation is None or observation.attribute != "atlas_place"
                            or observation.source != "place_teaching:HUMAN"
                            or not isinstance(observation.value, Mapping)
                            or observation.value.get("request_id") != params["request_id"]):
                        continue
                    if observation.entity_id != params["entity_id"] or dict(observation.value) != value:
                        raise ValueError("PLACE_TEACHING_REQUEST_CONFLICT")
                    duplicate = True
                superseded = duplicate and previous.value != value
                if not duplicate:
                    event = self.world.observe(params["entity_id"], "atlas_place", value,
                        domain="room_topology", measurement_time_ns=now, confidence=1,
                        source="place_teaching:HUMAN", lineage=(f"human:{params['request_id']}",
                            f"atlas:{point.map_id}:{point.map_revision}", f"viewpoint:{point.viewpoint_id}"))
                    if not event.accepted:
                        raise ValueError("PLACE_TEACHING_REJECTED:" + event.reason)
                self.spatial.sync_world()
            saved = self._persist(force=True)
            result = {"status": "TAUGHT", "entity_id": params["entity_id"], **value,
                      "duplicate": duplicate, "superseded": superseded,
                      "durability": "SAVED" if saved else "MEMORY_ONLY"}
            self._queue_evidence("atlas_place_taught", result)
            return result
        if action in {"person.teach", "person.validate_target"}:
            runtime, status, vision = self._person_context()
            with self._state_lock:
                if action == "person.validate_target":
                    if set(params) != {"target"} or not isinstance(params["target"], Mapping):
                        raise ValueError("person.validate_target requires a target")
                    target = self.person_identity.validate_target(params["target"], runtime=runtime,
                                                                 status=status, vision_status=vision)
                    return {"status": "VALIDATED", "target": target}
                skill_descriptor(action).validate_parameters(params)
                request = PersonTeaching(**params)
                result = self.person_identity.teach(request, runtime=runtime, status=status, vision_status=vision)
                self.spatial.sync_world()
            saved = self._persist(force=True)
            teaching_result = {**result, "status": "TAUGHT", "durability": "SAVED" if saved else "MEMORY_ONLY",
                               "storage_error": self._health_error[:256] if self.root is not None
                               and not saved and self._health_error else None}
            self._queue_evidence("person_teaching", teaching_result)
            return teaching_result
        if action.startswith("brain."):
            allowed = {"brain.submit": {"text", "source", "request_id"},
                       "brain.adopt": {"goal_id", "plan"},
                       "brain.fail": {"goal_id", "reason", "pending_only"}, "brain.cancel": {"reason"}}
            if action not in allowed or set(params) - allowed[action]:
                raise ValueError("unknown Brain parameters")
            if action == "brain.submit":
                return self.brain.submit(**params)
            if action == "brain.adopt":
                return self.brain.adopt(**params, asynchronous=True)
            if action == "brain.fail":
                return self.brain.fail(**params)
            return self.preempt(params.get("reason", "USER_CANCEL"))
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
        names = {"behavior.room_cruise": "room_cruise",
                 **{skill.action: skill.behavior_name for skill in PERSON_SKILLS if skill.behavior_name}}
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
            self.brain.revoke("PREEMPTED_BY:" + names[action])
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
    capability_names = READS | QUERIES | ACTIONS | SKILL_READS | SKILL_ACTIONS

    def __init__(self, client: PublicRobotClient, controller: object | None = None):
        self.client = client
        self.controller = controller

    def capabilities(self):
        from v3.action_catalog import action_descriptor
        result = {name: {"kind": "read" if name in READS | QUERIES else "action", "supported": True,
                       "available": True, "ready": True, "owner": "host",
                       "reason": "V3_INDEPENDENT_PUBLIC_STATE" if name in READS | QUERIES or name == "world.observe"
                       else "HOST_KNOWLEDGE_UPDATE" if name in {"person.teach", "person.validate_target",
                                                               "spatial.load_atlas", "spatial.teach_place"}
                       else "CANONICAL_V3_EXECUTION",}
                for name in self.capability_names}
        for name, item in result.items():
            descriptor = action_descriptor(name)
            if descriptor is not None:
                item.update(descriptor.to_jsonable())
            skill = skill_descriptor(name)
            if skill is not None:
                item.update(skill.to_jsonable())
        result.update(skill_capabilities())
        # Discovery is live. New library modules need no client-side allowlist.
        if isinstance(self.client, PublicRobotClient):
            from .skill_library import SkillLibrary
            from .skill_observation import SkillObservationAdapter
            from .software_interface import SoftwareInterfaceAdapter
            result.update(skill_capabilities(SkillLibrary(self.client.root / "robot_skills")))
            result.update(SkillObservationAdapter(self.client.root).descriptors())
            result.update(SoftwareInterfaceAdapter(self.client.root).capabilities())
            try:
                host = (self.client.request("capabilities", launch=False, timeout_s=0.5)
                        if self.client.socket_path.exists() else None)
                if isinstance(host, Mapping):
                    result.update({name: value for name, value in host.get("capabilities", {}).items()
                                   if name in host.get("host_capabilities", ())})
            except (OSError, RuntimeError, ValueError):
                pass
        elif hasattr(self.client, "runtime"):
            result.update(skill_capabilities(self.client.runtime.skill_library))
        if self.controller is not None:
            status = self.controller.live_runtime_status()
            if isinstance(status, Mapping) and (status.get("fault_layer") or status.get("safety_decision") == "FAULT"):
                for name in ("behavior.room_cruise", "behavior.follow_person", "behavior.search_person", "behavior.search_any_person", "behavior.start"):
                    result[name].update(available=False, ready=False, reason="RUNTIME_FAULT")
            from v3.adapters.vision_media_socket import VisionClient
            camera = VisionClient(root=self.controller.root).status()
            if camera.get("camera_state") == "FAILED":
                for name in ("behavior.follow_person", "behavior.search_person", "behavior.search_any_person"):
                    if result[name]["available"]:
                        result[name].update(available=False, ready=False, reason="VISION_UNAVAILABLE")
        return result

    def read(self, resource: str) -> object:
        return self.client.request("read", resource=resource)

    def query(self, query: WorldQuery) -> WorldQueryResult:
        if not isinstance(query, WorldQuery):
            raise TypeError("public world query must be a WorldQuery")
        return WorldQueryResult.from_jsonable(self.client.request("query", query=query.to_jsonable()))

    def spatial_query(self, query: SpatialQuery) -> SpatialQueryResult:
        if not isinstance(query, SpatialQuery):
            raise TypeError("spatial query must be a SpatialQuery")
        return SpatialQueryResult.from_jsonable(self.client.request("spatial_query", query=query.to_jsonable()))

    def execute(self, action: str, **parameters: object) -> object:
        return self.client.request("execute", action=action, parameters=parameters)


class PublicRobotStateAdapter:
    """The behavior's injected RobotInterface reads the same owned world state."""
    name = "public_world_state"
    capability_names = READS | QUERIES | {"person.teach", "person.validate_target"} | SKILL_READS | SKILL_ACTIONS

    def __init__(self, runtime: PublicRobotRuntime):
        self.runtime = runtime

    def capabilities(self):
        result = {name: {"kind": "read" if name in READS | QUERIES | SKILL_READS else "action",
                       "supported": True, "available": True, "ready": True}
                for name in self.capability_names}
        result.update(skill_capabilities(self.runtime.skill_library))
        if self.runtime.skill_observation is not None:
            result.update(self.runtime.skill_observation.descriptors())
        if self.runtime.software is not None:
            result.update(self.runtime.software.capabilities())
        return result

    def read(self, resource: str):
        result = self.runtime.read(resource)
        if resource == "world.snapshot":
            self.runtime.world_input_evidence(result)
        return result

    def query(self, query: WorldQuery) -> WorldQueryResult:
        result = self.runtime.query(query)
        self.runtime._queue_evidence("world_query", result)
        return result

    def spatial_query(self, query: SpatialQuery) -> SpatialQueryResult:
        result = self.runtime.spatial_query(query)
        self.runtime._queue_evidence("spatial_query", result)
        return result

    def execute(self, action: str, **parameters: object):
        if action in {"person.teach", "person.validate_target", "hri.report"} or action.startswith("skill."):
            return self.runtime.execute(action, parameters)
        if self.runtime.skill_observation is not None and action in self.runtime.skill_observation.capability_names:
            return self.runtime.skill_observation.call(action, parameters)
        if self.runtime.software is not None and action in self.runtime.software.capability_names:
            return self.runtime.software.execute(action, **parameters)
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
        runtime = PublicRobotRuntime(backend, root=root, socket_path=socket_path)
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
                    result = runtime.dispatch(request)
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
                    priority = (request.get("operation") in {"preempt", "revoke", "stop"}
                                or request.get("action") in {"v3.command.stop", "skill.stop", "skill.revoke"})
                    admission_slots = stop_slots if priority else slots
                    if not admission_slots.acquire(blocking=False):
                        conn.close()
                        continue
                    threading.Thread(target=handle, args=(conn, request, admission_slots), daemon=True).start()
            finally:
                stop_event.set()
                runtime.close()
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
