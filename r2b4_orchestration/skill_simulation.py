"""Motor-free skill experiments through the real host, SDK and Python worker.

Semantic recordings and a bounded differential-drive plant feed native V3.
Full native replay and a sampled learning timeline use existing MCAP paths.
Generated skills are trusted ordinary Python; this is not a Python sandbox.
"""
from __future__ import annotations

from collections.abc import Mapping
import json
import hashlib
import math
from pathlib import Path
import socket
import tempfile
import threading
import time
from types import SimpleNamespace

from .robot_runtime import MAX_REPLY_BYTES, MAX_REQUEST_BYTES, SCHEMA, PublicRobotRuntime
from .skill_library import SkillLibrary
from .motion_simulation import (DifferentialDriveExperiment, MOTION_ACTIONS, compare_trials,
                                motion_scenario, simulation_control_config)
from v3.robot_interface import RobotInterface
from v3.capture_rate import DEFAULT_CAPTURE_HZ


_OBSERVE_ACTIONS = frozenset({"observe.open", "observe.status", "observe.events", "observe.finish", "observe.close"})


def _recorded_read_name(name):
    # The host's state/lifecycle and developer tools keep their own authority.
    from .robot_runtime import READS, SKILL_READS
    return (isinstance(name, str) and 0 < len(name) <= 128 and "." in name
            and name not in READS | SKILL_READS | {"simulation.scenario", "v3.status"}
            and not name.startswith(("operator.", "system.", "capture.", "skill.", "software.", "brain.", "behavior.")))


def _resolve_recorded_observer(value):
    if (not isinstance(value, Mapping)
            or set(value) != {"parameters", "events", "start_monotonic_ns", "end_monotonic_ns", "complete"}):
        raise ValueError("observer requires parameters, events, original measurement interval and complete")
    parameters, events = value["parameters"], value["events"]
    start, end = value["start_monotonic_ns"], value["end_monotonic_ns"]
    if (not isinstance(parameters, Mapping) or set(parameters) != {"region", "object_kind"}
            or parameters["object_kind"] != "person"
            or type(start) is not int or type(end) is not int or not 0 <= start <= end
            or type(value["complete"]) is not bool or not isinstance(events, list) or len(events) > 64):
        raise ValueError("invalid recorded observer binding, measurement interval or bounded events")
    # Reuse the production image-region contract without opening its owner.
    from v3.adapters.skill_observation import _region
    _region(parameters["region"])
    previous = 0
    identities = set()
    for event in events:
        if (not isinstance(event, dict)
                or event.get("kind") not in {"qualified_presence", "coverage_gap", "capability_failed"}
                or type(event.get("sequence")) is not int or event["sequence"] <= previous
                or type(event.get("measurement_monotonic_ns")) is not int
                or not start <= event["measurement_monotonic_ns"] <= end
                or type(event.get("source_sequence")) is not int or event["source_sequence"] < 0
                or not isinstance(event.get("event_id"), str) or not 1 <= len(event["event_id"]) <= 256
                or event["event_id"] in identities
                or not isinstance(event.get("owner_generation"), str) or not 1 <= len(event["owner_generation"]) <= 256):
            raise ValueError("recorded observer events require ordered identity and original measurement lineage")
        previous = event["sequence"]
        identities.add(event["event_id"])
    return dict(value)


def threshold_scenario(value=None) -> dict[str, object]:
    """Resolve the one implemented semantic scenario, with an independent oracle."""
    if value is None or value == "threshold_detection":
        value = {}
    if not isinstance(value, Mapping) or set(value) - {"kind", "values", "threshold"}:
        raise ValueError("scenario requires kind, values and threshold")
    if value.get("kind", "threshold_detection") != "threshold_detection":
        raise ValueError("SIMULATION_SCENARIO_UNAVAILABLE")
    values, threshold = value.get("values", [1, 5, 3]), value.get("threshold", 4)
    if (not isinstance(values, list) or not 1 <= len(values) <= 64
            or any(type(item) not in {int, float} or not math.isfinite(item) for item in values)
            or type(threshold) not in {int, float} or not math.isfinite(threshold)):
        raise ValueError("scenario requires 1..64 finite samples and a finite threshold")
    return {"kind": "threshold_detection", "values": list(values), "threshold": threshold}


def resolve_scenario(value=None):
    if isinstance(value, Mapping) and value.get("kind") == "differential_drive":
        return motion_scenario(value)
    if not isinstance(value, Mapping) or value.get("kind") != "semantic_trace":
        return threshold_scenario(value)
    if set(value) - {"kind", "resources", "observer", "expected_result", "expected_reports"}:
        raise ValueError("unknown semantic_trace scenario fields")
    resources = value.get("resources")
    if (not isinstance(resources, Mapping) or not 0 <= len(resources) <= 8
            or not resources and "observer" not in value
            or any(not _recorded_read_name(name) for name in resources)
            or any(not isinstance(rows, list) or not 1 <= len(rows) <= 64
                   or any(not isinstance(row, dict) for row in rows) for rows in resources.values())):
        raise ValueError("semantic_trace requires bounded semantic read resources or a recorded observer")
    if "observer" in value:
        _resolve_recorded_observer(value["observer"])
    if "expected_result" not in value and "expected_reports" not in value:
        raise ValueError("semantic_trace requires an independent expected_result or expected_reports")
    if "expected_reports" in value and (not isinstance(value["expected_reports"], list)
            or any(not isinstance(item, str) or len(item) > 4096 for item in value["expected_reports"])):
        raise ValueError("expected_reports must contain bounded report text")
    encoded = json.dumps(value, allow_nan=False)
    if len(encoded.encode()) > 65_536:
        raise ValueError("semantic_trace exceeds its 65536 byte bound")
    return json.loads(encoded)


class _SemanticSource:
    name = "motorless_semantic_source"
    reads = frozenset({"simulation.sample", "simulation.scenario", "operator.status", "v3.status"})
    motion = frozenset({"v3.command.move_relative", "v3.command.turn_by", "v3.command.explore",
                        "v3.command.follow_person", "operator.runtime.start"})
    capability_names = reads | motion | {"v3.command.stop"}

    def __init__(self) -> None:
        self.owner = None
        self.samples = []
        self.observations = []
        self.resource_reads = {}
        self.trace_samples = {}
        self.scenario = threshold_scenario()
        self.stops = 0
        self.physics = None
        self._last_clock_ns = 0
        self._lock = threading.Lock()

    def clock_ns(self):
        if self.physics is not None:
            self._last_clock_ns = max(self._last_clock_ns, self.physics.now_ns)
        else:
            self._last_clock_ns = max(self._last_clock_ns, time.monotonic_ns())
        return self._last_clock_ns

    def prepare(self, scenario, revision):
        with self._lock:
            self.clock_ns()
            self.physics = None
            self.scenario = resolve_scenario(scenario)
            measured = self.clock_ns()
            self.observations = []
            self.resource_reads = {}
            if self.scenario["kind"] == "differential_drive":
                self.trace_samples = {}
                self.samples = []
                self.reads = type(self).reads | {"v3.pose", "v3.safety", "v3.health"}
                self.capability_names = self.reads | self.motion | MOTION_ACTIONS | {"v3.command.stop"}
                return
            if self.scenario["kind"] == "semantic_trace":
                self.trace_samples = self.scenario["resources"]
                self.samples = []
                self.reads = type(self).reads | self.trace_samples.keys()
                self.capability_names = self.reads | self.motion | {"v3.command.stop"}
                return
            self.reads = type(self).reads
            self.capability_names = type(self).capability_names
            self.trace_samples = {}
            self.samples = [{"value": value, "measurement_time_ns": measured + index,
                             "sequence": index + 1, "revision": revision,
                             "source": "simulation", "lineage": {
                                 "producer": self.name, "generation": revision, "sample": index + 1}}
                            for index, value in enumerate(self.scenario["values"])]

    def capabilities(self):
        available_motion = MOTION_ACTIONS if self.scenario["kind"] == "differential_drive" else frozenset()
        return {name: {"kind": "read" if name in self.reads else "action", "supported": True,
                       "available": name not in self.motion or name in available_motion,
                       "ready": name not in self.motion or name in available_motion,
                       "source": "simulation", "reason": "MOTORLESS_SIMULATION" if name in self.motion and name not in available_motion else None}
                for name in self.capability_names}

    def read(self, resource):
        if resource == "simulation.scenario":
            inputs = {key: value for key, value in self.scenario.items()
                      if key not in {"expected_result", "expected_reports"}}
            return {**inputs, "source": "simulation", "resource": "simulation.sample"}
        if resource == "operator.status":
            return {"runtime_running": False, "runtime_pid": None, "source": "simulation"}
        if resource == "v3.status":
            if self.physics is not None:
                return json.loads(json.dumps(self.physics.status))
            return {"state": "IDLE", "tick_id": 0, "monotonic_ns": time.monotonic_ns(),
                    "safety_decision": "STOP", "source": "simulation"}
        if self.physics is not None and resource in {"v3.pose", "v3.health", "v3.safety"}:
            status = self.physics.status
            if resource == "v3.pose":
                result = status["estimate"]
            elif resource == "v3.health":
                result = status["source_health"]
            else:
                result = {key: status[key] for key in ("state", "tick_id", "monotonic_ns", "safety_decision",
                                                       "safety_reason", "fault_layer", "enabled")}
            return json.loads(json.dumps(result))
        if resource != "simulation.sample" and resource not in self.trace_samples:
            raise KeyError(resource)
        with self._lock:
            if self.trace_samples:
                rows = self.trace_samples[resource]
                index = self.resource_reads.get(resource, 0)
                if index >= len(rows):
                    raise RuntimeError("SIMULATION_SOURCE_EXHAUSTED")
                sample = rows[index]
                self.resource_reads[resource] = index + 1
                self.observations.append({"resource": resource, "sample": sample})
            elif len(self.observations) >= len(self.samples):
                raise RuntimeError("SIMULATION_SOURCE_EXHAUSTED")
            else:
                sample = self.samples[len(self.observations)]
                self.observations.append(sample)
        self.owner._queue_evidence("simulation_observation", sample)
        # The source's concrete measurement identity is retained over the socket.
        return json.loads(json.dumps(sample))

    def execute(self, action, **parameters):
        if self.physics is not None:
            if action in MOTION_ACTIONS:
                return self.physics.execute(action, parameters)
            if action == "v3.command.stop":
                if parameters:
                    raise ValueError("STOP accepts no parameters")
                self.stops += 1
                return self.physics.stop()
        if action != "v3.command.stop":
            raise RuntimeError("MOTORLESS_SIMULATION")
        if parameters:
            raise ValueError("STOP accepts no parameters")
        self.stops += 1
        return {"status": "STOPPED", "source": "simulation", "physical_output": False}


class _RecordedObservationAdapter:
    """Replay one bounded observer session using its existing SDK contract.

    This owns only recorded host observations. No camera/detector or media
    owner is started, and recorded time never claims current live freshness.
    """
    capability_names = _OBSERVE_ACTIONS

    def __init__(self, source):
        self.source = source
        self._lock = threading.Lock()
        self._sessions = {}
        self._counter = 0
        self.delivered = []
        self.partial = False

    def reset(self):
        self.close()
        with self._lock:
            self.delivered = []
            self.partial = False

    def descriptors(self):
        available = "observer" in self.source.scenario
        return {name: {"kind": "action", "supported": True, "available": available, "ready": available,
                       "source": "recorded_semantic_trace", "owner": "recorded_observer",
                       "reason": None if available else "RECORDED_OBSERVER_UNAVAILABLE"}
                for name in self.capability_names}

    def call(self, name, parameters, invocation_id=None):
        params = dict(parameters)
        evidence = []
        with self._lock:
            recorded = self.source.scenario.get("observer")
            if recorded is None:
                raise RuntimeError("RECORDED_OBSERVER_UNAVAILABLE")
            if name == "observe.open":
                readiness = params.pop("ready_timeout_s", 5.)
                if type(readiness) not in {int, float} or not math.isfinite(readiness) or not 0 <= readiness <= 30:
                    raise ValueError("invalid recorded observer readiness timeout")
                from v3.adapters.skill_observation import _region
                if (set(params) != {"region", "object_kind"}
                        or params["object_kind"] != recorded["parameters"]["object_kind"]
                        or _region(params["region"]) != _region(recorded["parameters"]["region"])):
                    raise RuntimeError("RECORDED_OBSERVER_PARAMETERS_MISMATCH")
                if self._sessions:
                    raise RuntimeError("RECORDED_OBSERVER_SESSION_ALREADY_OPEN")
                self._counter += 1
                handle = f"recorded-observer-{self._counter}"
                session = {"handle": handle, "invocation_id": invocation_id, "until": None,
                           "delivered": set(), "recorded": recorded}
                self._sessions[handle] = session
                result = self._page(session, 0, 0)
                result["status"] = "READY"
                result["events"] = []
                return result
            handle = params.pop("handle", None)
            session = self._sessions.get(handle)
            if session is None or session["invocation_id"] != invocation_id:
                raise RuntimeError("RECORDED_OBSERVER_REVOKED")
            if name == "observe.close":
                if params:
                    raise ValueError("observe.close accepts only handle")
                del self._sessions[handle]
                return {"handle": handle, "status": "CLOSED", "source": "recorded_semantic_trace"}
            if name == "observe.finish":
                until = params.pop("until_monotonic_ns", None)
                timeout = params.pop("timeout_s", 5.)
                if (type(until) is not int or until < recorded["start_monotonic_ns"]
                        or type(timeout) not in {int, float} or not math.isfinite(timeout) or not 0 <= timeout <= 30):
                    raise ValueError("invalid recorded measurement deadline")
                if session["until"] is not None and session["until"] != until:
                    raise ValueError("recorded observer interval has already been closed")
                session["until"] = until
                self.partial |= until > recorded["end_monotonic_ns"] or not recorded["complete"]
            elif name not in {"observe.events", "observe.status"}:
                raise KeyError(name)
            cursor, limit = params.pop("after_sequence", 0), params.pop("limit", 32)
            if params or type(cursor) is not int or cursor < 0 or type(limit) is not int or not 1 <= limit <= 32:
                raise ValueError("invalid recorded observer cursor/page")
            result = self._page(session, cursor, limit)
            for event in result["events"]:
                if event["event_id"] not in session["delivered"]:
                    session["delivered"].add(event["event_id"])
                    self.delivered.append(event)
                    evidence.append(event)
        for event in evidence:
            self.source.observations.append({"resource": "observe.events", "sample": event})
            self.source.owner._queue_evidence("simulation_observation", event)
        return result

    def _page(self, session, cursor, limit):
        recorded, until = session["recorded"], session["until"]
        events = [event for event in recorded["events"]
                  if until is None or event["measurement_monotonic_ns"] <= until]
        selected = [event for event in events if event["sequence"] > cursor][:limit]
        previous = cursor
        lost = 0
        for event in selected:
            lost += max(0, event["sequence"] - previous - 1)
            previous = event["sequence"]
        self.partial |= bool(lost) or any(event["kind"] != "qualified_presence" for event in selected)
        return {"handle": session["handle"], "status": "FINISHED" if until is not None else "READY",
                "clock_domain": "recorded_monotonic", "monotonic_ns": recorded["start_monotonic_ns"],
                "source": "recorded_semantic_trace", "live_freshness": False,
                "events": json.loads(json.dumps(selected)), "lost_events": lost,
                "latest_sequence": max((event["sequence"] for event in events), default=0),
                "finished": until is not None, "partial": self.partial,
                "recorded_start_monotonic_ns": recorded["start_monotonic_ns"],
                "recorded_end_monotonic_ns": recorded["end_monotonic_ns"]}

    def close_invocation(self, invocation_id=None):
        with self._lock:
            self._sessions = {handle: session for handle, session in self._sessions.items()
                              if session["invocation_id"] != invocation_id}

    def close(self):
        with self._lock:
            self._sessions.clear()


class MotorlessSkillSimulator:
    """An explicit host-only trial; all generated evidence remains under /tmp.

    ``trial`` runs a saved skill through a Brain goal and the real SDK socket.
    It measures the full observed samples and returned detections independently.
    Scalar detection, recorded semantic traces and a deterministic bounded
    physical model use the same worker/SDK boundary; physical validation remains
    a separate live robot task.
    """

    def __init__(self, project_root: Path | str, library: SkillLibrary | None = None):
        self.project_root = Path(project_root).resolve()
        self.root = Path(tempfile.mkdtemp(prefix="r2b4-learn-sim-", dir="/tmp"))
        self.library = library or SkillLibrary(self.project_root / "robot_skills")
        self.socket_path = self.root / "robot.sock"
        self.source = _SemanticSource()
        backend = RobotInterface(project_root=self.root, controller=self.source,
                                 adapters=(self.source,), upper_runtime=False)
        # Publication/lifecycle time remains the real host clock. Accelerated
        # plant measurement time is explicit in the simulated tick payload;
        # it must not become a future host-journal publication timestamp.
        self.runtime = PublicRobotRuntime(backend, root=self.root, socket_path=self.socket_path)
        self.runtime.skill_library = self.library
        self.runtime.skills.library = self.library
        # Host tools stay scoped to temporary simulation data. Agent source and
        # evidence tools are composed separately against the actual project.
        self.runtime.skill_observation.close()
        self.recorded_observation = _RecordedObservationAdapter(self.source)
        self.runtime.skill_observation = self.recorded_observation
        self.source.owner = self.runtime
        self.interface = self.runtime.program_interface
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._server_error = None
        self._server_thread = None
        self._requests = threading.BoundedSemaphore(8)
        self._priority = threading.BoundedSemaphore(2)
        self._handlers = set()
        self._handlers_lock = threading.Lock()
        self._trials = []
        self._trial_lock = threading.Lock()
        self._closed = False
        self.goal_text = None
        self.learning_capture = self.root / "learning.mcap"
        self._learning_session = None
        self._learning_anchor_written = False

    def capture_learning_event(self, event, fields):
        """Persist the complete learning lifecycle, including between-trial events."""
        if self._learning_session is None or self._closed:
            raise RuntimeError("learning capture requires an entered simulator")
        if not isinstance(event, str) or not event.startswith(("learning_", "agent_")) or len(event) > 128 or not isinstance(fields, Mapping):
            raise ValueError("learning event requires a bounded learning_/agent_ name and fields")
        self.runtime._queue_evidence(event, {**dict(fields), "source": "simulation", "physical_output": False})
        self.runtime._flush_evidence()

    @staticmethod
    def scenario_descriptor(value=None):
        scenario = resolve_scenario(value)
        input_contract = {"scenario_read": 'await robot.read("simulation.scenario")',
                          "samples": "Each await robot.read(resource) consumes one current sample; preserve the returned object unchanged.",
                          "parameters": "Only explicit skill parameters are passed as kwargs; the scenario is read through the SDK."}
        if scenario["kind"] == "differential_drive":
            input_contract.update(motion="Use ordinary finite v3.command.move_relative, turn_by, or navigate with wait_for_completion=True in R2B4_ODOM_LOCAL. "
                "Each action advances virtual 50 Hz time and returns only after native completion/failure and STOP. "
                "Do not use host sleeps to advance this model or compare its timestamps to the host wall clock.",
                measurements="Read v3.status/pose/health/safety for native estimates. Ground truth is withheld until independent assessment.")
            return {**scenario, "source": "simulation", "input_contract": input_contract,
                    "scope": "Calibrated PWM differential-drive model, bounded room/circles, synthetic sensors and native V3; live fidelity unproven."}
        if scenario["kind"] == "semantic_trace":
            input_contract["observer"] = (
                "If observer is supplied, use the unchanged production robot.observe()/robot.events.wait()/"
                "observer.finish_events() SDK. Its recorded_monotonic clock translates deadlines; original "
                "event identity/time remain unchanged. Media and motion are unavailable.")
            return {**scenario, "input_contract": input_contract, "source": "simulation", "scope": "Recorded semantic trace; original sample lineage/time retained, no live freshness claim."}
        return {**scenario, "resource": "simulation.sample", "source": "simulation",
                "input_contract": input_contract,
                "output": "Return {detections: [complete samples with value >= threshold], samples_checked: len(values)}",
                "scope": "Semantic scalar detection and SDK/worker/Brain integration; no physical perception or motion."}

    def __enter__(self):
        if self._closed or self._server_thread is not None:
            raise RuntimeError("simulator cannot be reopened")
        self._server_thread = threading.Thread(target=self._serve, name="motorless-skill-socket", daemon=True)
        self._server_thread.start()
        if not self._ready.wait(2) or self._server_error is not None:
            self.close()
            raise RuntimeError("motorless socket failed") from self._server_error
        from v3.mcap_capture import McapCaptureConfig
        from v3_process_runtime import McapCaptureSession
        self.runtime._flush_evidence()
        self._learning_session = McapCaptureSession("skill-learning", self.learning_capture,
            configuration={"scope": "learning_observation", "native_replay_capture": "per-trial capture.mcap"},
            metadata={"source": "simulation", "physical_hardware": False}, project_root=self.root,
            config=McapCaptureConfig(mode="append_only", tick_sample_hz=DEFAULT_CAPTURE_HZ), capacity=1024)
        self._learning_session.start()
        return self

    def _serve(self):
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
                server.bind(str(self.socket_path))
                server.listen(10)
                server.settimeout(.1)
                self._ready.set()
                while not self._stop.is_set():
                    try:
                        connection, _ = server.accept()
                    except socket.timeout:
                        continue
                    connection.settimeout(.25)
                    try:
                        with connection.makefile("rb") as stream:
                            line = stream.readline(MAX_REQUEST_BYTES + 1)
                        if not line.endswith(b"\n") or len(line) > MAX_REQUEST_BYTES:
                            raise ValueError("bounded complete request required")
                        request = json.loads(line)
                        if not isinstance(request, dict):
                            raise ValueError("request must be an object")
                    except (OSError, ValueError):
                        connection.close()
                        continue
                    priority = (request.get("operation") in {"stop", "preempt", "revoke"}
                                or request.get("action") in {"skill.stop", "skill.revoke", "v3.command.stop"})
                    slots = self._priority if priority else self._requests
                    if not slots.acquire(blocking=False):
                        connection.close()
                        continue
                    thread = threading.Thread(target=self._handle, args=(connection, request, slots), daemon=True)
                    with self._handlers_lock:
                        self._handlers.add(thread)
                    thread.start()
        except Exception as exc:
            self._server_error = exc
            self._ready.set()

    def _handle(self, connection, request, slots):
        try:
            with connection:
                connection.settimeout(35)
                try:
                    result = self.runtime.dispatch(request)
                    response = {"schema": SCHEMA, "result": result}
                except Exception as exc:
                    response = {"schema": SCHEMA, "error": f"{type(exc).__name__}:{exc}"}
                payload = json.dumps(response, allow_nan=False).encode() + b"\n"
                if len(payload) > MAX_REPLY_BYTES:
                    payload = json.dumps({"schema": SCHEMA, "error": "SIMULATION_REPLY_EXCEEDED_BOUND"}).encode() + b"\n"
                connection.sendall(payload)
        except OSError:
            pass
        finally:
            slots.release()
            with self._handlers_lock:
                self._handlers.discard(threading.current_thread())

    def trial(self, name, parameters=None, scenario=None, *, timeout_s=30, cancel_event=None):
        if self._server_thread is None or self._closed:
            raise RuntimeError("motorless simulator must be entered")
        if type(timeout_s) not in {int, float} or not math.isfinite(timeout_s) or not 0 < timeout_s <= 120:
            raise ValueError("trial timeout must be within (0, 120]")
        if not self._trial_lock.acquire(blocking=False):
            raise RuntimeError("motorless trial already active")
        try:
            return self._trial(name, dict(parameters or {}), scenario, timeout_s, cancel_event)
        finally:
            self._trial_lock.release()

    def _trial(self, name, parameters, scenario, timeout_s, cancel_event):
        from v3.config import ConfigResolver
        from v3.composition.full_fake import OfflineMotorSink
        from v3.composition.native_control import NativeControlComposition
        from v3.adapters.fake_edges import FakeCommandGateway
        from v3.contracts import (DataField, DeviceHealth, DeviceHealthState, DeviceSample,
                                  LifecycleState, RawDeviceBatch, TickContext)
        from v3.engine import TickInputs
        from v3.execution import ExecutionRecord
        from v3.mcap_capture import McapCaptureConfig
        from v3_process_runtime import McapCaptureSession

        resolved_scenario = resolve_scenario(scenario)
        if resolved_scenario["kind"] == "semantic_trace":
            from v3.interface_adapters import build_adapters
            published_reads = set().union(*(adapter.read_capability_names
                                           for adapter in build_adapters(None, self.project_root)))
            unknown = [name for name in resolved_scenario["resources"]
                       if not name.startswith("simulation.") and name not in published_reads]
            if unknown:
                raise ValueError("RECORDED_READ_RESOURCE_NOT_PUBLISHED:" + ",".join(unknown))
        self.recorded_observation.reset()
        number = len(self._trials) + 1
        self.source.prepare(resolved_scenario, f"simulation-trial-{number}")
        self._report_cursor = self.runtime._sdk_event_sequence
        directory = self.root / f"trial-{number}"
        directory.mkdir()
        conf = self.project_root / "conf"
        resolved = ConfigResolver(conf / "hardver.json", conf / "fizika.json", conf / "speed_map.json",
                                  conf / "vezerles.json").resolve()
        config = resolved.runtime.composition.live_control.control
        simulation_config = None
        if resolved_scenario["kind"] == "differential_drive":
            from v3.capture_encoding import encode_value
            def config_digest(value):
                return hashlib.sha256(json.dumps(encode_value(value), sort_keys=True, separators=(",", ":"),
                                                 allow_nan=False).encode()).hexdigest()
            base_digest = config_digest(config)
            config = simulation_control_config(config, resolved_scenario)
            simulation_config = {"base_sha256": base_digest, "effective_sha256": config_digest(config),
                                 "overrides": resolved_scenario["v3_overrides"]}
        writer, gateway = OfflineMotorSink(), FakeCommandGateway()
        composition = NativeControlComposition(writer, config)
        session = McapCaptureSession(f"motorless-skill-{number}", directory / "capture.mcap",
                                    configuration={"production_control": config},
                                    metadata={"source": "simulation", "scenario": resolved_scenario,
                                              "skill": name, "physical_hardware": False,
                                              "simulation_config": simulation_config},
                                    project_root=self.root,
                                    config=McapCaptureConfig(mode="append_only", tick_sample_hz=50), capacity=2048)
        # Drain older host evidence before the follower establishes this scope.
        self.runtime._flush_evidence()
        session.start()
        start_ns = self.source.clock_ns() + 20_000_000
        physics = None
        if resolved_scenario["kind"] == "differential_drive":
            def observe_motion(record, raw_scan):
                if raw_scan is not None:
                    session.hub.publish(raw_scan, topic="v3.raw_lidar")
                session.observe(record)
                if not self._learning_anchor_written:
                    self._learning_session.observe(record)
                    self._learning_anchor_written = True
                if record.inputs.context.tick_id % 5 == 0:
                    from v3.resident_status import _tick_status
                    status = _tick_status(record.result)
                    estimate = status.get("estimate") or {}
                    layers = {row.layer: row.output for row in record.result.trace.layers}
                    native_estimate, requested, allowed = layers.get("L3"), layers.get("L8"), layers.get("L9")
                    quality = getattr(native_estimate, "localization_quality", None)
                    final = record.result.final_actuation
                    self.capture_learning_event("learning_motion_sample", {
                        "trial_number": number, "tick_id": record.inputs.context.tick_id,
                        "measurement_monotonic_ns": record.inputs.context.monotonic_ns,
                        "clock_domain": "simulation_monotonic",
                        "estimate": {key: estimate.get(key) for key in ("x_m", "y_m", "yaw_rad", "v_mps", "omega_rad_s")},
                        "frame_id": getattr(native_estimate, "frame_id", None),
                        "local_pose_generation": getattr(quality, "generation", None),
                        "local_pose_continuous": getattr(quality, "local_pose_continuous", None),
                        "requested_v_mps": getattr(requested, "requested_v_mps", None),
                        "requested_omega_rad_s": getattr(requested, "requested_omega_rad_s", None),
                        "allowed_v_mps": getattr(allowed, "allowed_v_mps", None),
                        "allowed_omega_rad_s": getattr(allowed, "allowed_omega_rad_s", None),
                        "constraint_codes": [row.value for row in getattr(allowed, "active_constraints", ())],
                        "left_output": final.left_output, "right_output": final.right_output,
                        "mission": status["mission"], "navigation": status["navigation"],
                        "safety_decision": status["safety_decision"], "safety_reason": status["safety_reason"],
                        "fault_layer": status["fault_layer"]})
                if session.failed or self._learning_session.failed:
                    raise RuntimeError("SIMULATION_CAPTURE_INTEGRITY_FAILURE")
            physics = DifferentialDriveExperiment(resolved_scenario, config, composition, writer, directory,
                start_ns, observe_motion, self.runtime._queue_evidence, lidar_config=resolved.lidar,
                lidar_pose_r_scale=resolved.runtime.sensor_inputs.inputs.lidar_backend.pose_r_scale)
            physics.simulation_config = simulation_config
            self.source.physics = physics

        def stop_tick(tick):
            context = TickContext(tick, start_ns + tick * 20_000_000)
            def sample(device, kind, **values):
                return DeviceSample(device, kind, tick + 1, context.monotonic_ns,
                                    tuple(DataField(key, value) for key, value in values.items()))

            # Synthetic stationary acquisition is sufficient for a healthy
            # native STOP chain. Missing estimator input must not turn a normal
            # semantic trial into an artificial L3 fault.
            samples = (
                sample("ENCODER", "wheel_velocity", left_mps=0., right_mps=0., trust=1.,
                       raw_left_distance_m=0., raw_right_distance_m=0.,
                       left_counter_running=True, right_counter_running=True,
                       measurement_stale=False, measurement_timing_valid=True),
                sample("IMU", "ekf_heading", yaw_rad=0., omega_rad_s=0., confidence=1., omega_confidence=1.),
                sample("RPLIDAR_C1", "lidar_health", age_ns=0, point_count=0),
                sample("RPLIDAR_C1", "lidar_local_points", frame_id="ROBOT_BASE", point_count=0),
                sample("RPLIDAR_C1", "lidar_safety_clearance", age_ns=0,
                       **{f"{sector}_{key}": value for sector in ("front", "rear", "left", "right")
                          for key, value in (("clearance_m", 2.), ("observation_count", 10))}),
            )
            health = tuple(DeviceHealth(device, DeviceHealthState.OK) for device in
                           sorted(set(config.critical_device_ids or ()) | {"ENCODER", "IMU", "RPLIDAR_C1"}))
            raw = RawDeviceBatch(context, samples, health)
            inputs = composition.close_inputs(TickInputs(context, raw, gateway.snapshot(context),
                                                         LifecycleState.IDLE))
            result = composition.run_tick(inputs)
            final = result.final_actuation
            if result.trace.fault_layer is not None:
                raise RuntimeError("SIMULATION_NATIVE_TICK_FAULT:" + result.trace.fault_layer)
            if final.left_output != 0 or final.right_output != 0:
                raise RuntimeError("SIMULATION_NONZERO_FINAL_OUTPUT")
            # 50 Hz scope requests raw evidence too. Even the synthetic empty
            # scan has its own source revision and measurement timestamp.
            session.hub.publish(SimpleNamespace(
                raw_scan_id=tick + 1, raw_scan_timestamp=context.monotonic_ns / 1e9,
                scan_start_monotonic_ns=context.monotonic_ns,
                scan_end_monotonic_ns=context.monotonic_ns,
                measurement_monotonic_ns=context.monotonic_ns,
                health="OK", raw_scan=(), summary={"source": "simulation"}), topic="v3.raw_lidar")
            session.observe(ExecutionRecord(inputs, result))
            if not self._learning_anchor_written:
                self._learning_session.observe(ExecutionRecord(inputs, result))
                self._learning_anchor_written = True

        run_id = None
        goal = None
        interrupted = None
        settled = False
        try:
            if physics is not None:
                physics.step()
            else:
                stop_tick(0)
            self.runtime._queue_evidence("simulation_trial_started", {
                "name": name, "scenario": resolved_scenario, "source": "simulation"})
            goal = self.runtime.execute("brain.submit", {"text": self.goal_text or
                f"Motor nélküli tanulási próba: {name}"})
            adopted = self.runtime.execute("brain.adopt", {"goal_id": goal["goal_id"],
                "plan": {"skill": name, "parameters": parameters}})
            deadline = time.monotonic() + timeout_s
            while True:
                self.runtime.brain.step()
                # Brain status polling produces host interface evidence too.
                # Drain it here, so a slow journal write cannot coincide with
                # an unbounded stream of new polls into the bounded queue.
                # This is offline host scheduling, outside the native ticks.
                self.runtime._flush_evidence()
                goal = self.runtime.brain.goal(goal["goal_id"])
                run_id = goal.get("skill_run_id") or run_id
                if run_id is not None and not self.runtime.skills.active:
                    break
                if goal.get("lifecycle") in {"FAILED", "CANCELLED", "COMPLETED", "INTERRUPTED"} and run_id is None:
                    break
                if cancel_event is not None and cancel_event.is_set():
                    interrupted = "LEARNING_CANCELLED"
                elif time.monotonic() >= deadline:
                    interrupted = "TRIAL_TIMEOUT"
                if interrupted:
                    self.runtime.preempt(interrupted)
                    break
                time.sleep(.01)
            state = self.runtime.skills.status(run_id) if run_id is not None else {
                "state": "FAILED", "error": "SKILL_NOT_STARTED", "source_hash": self.library.load(name).source_hash}
            self.runtime.brain.step()
            goal = self.runtime.brain.goal(goal["goal_id"])
            if physics is not None:
                physics.settle()
                settled = True
            assessment = self._assess(state, interrupted, scenario is not None)
            self.runtime._queue_evidence("simulation_trial_assessment", {
                "run_id": run_id, "goal_id": goal["goal_id"], "source_hash": state.get("source_hash"),
                "assessment": assessment, "source": "simulation"})
        finally:
            self.runtime._flush_evidence()
            try:
                if physics is None:
                    stop_tick(1)
                elif not settled:
                    physics.stop()
            finally:
                composition.close()
                try:
                    capture = session.finalize(SimpleNamespace(status=0))
                finally:
                    if physics is not None:
                        self.source.clock_ns()
                        self.source.physics = None
        result = {"runtime": state, "assessment": assessment, "observations": list(self.source.observations),
                  "brain_goal": goal, "scenario": resolved_scenario, "source": "simulation",
                  "capture": str(capture), "capture_path": str(capture),
                  "artifact_directory": str(directory), "motor_writes": len(writer.writes),
                  "learning_capture": str(self.learning_capture), "physical_output": False}
        if physics is not None:
            result["performance"] = physics.performance()
            result["simulation_config"] = simulation_config
        self._trials.append(result)
        (directory / "trial.json").write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
        return result

    def _assess(self, state, interrupted, explicit_scenario):
        if self.source.physics is not None:
            performance = self.source.physics.performance()
            scenario = self.source.scenario
            metrics = performance["metrics"]
            checks = [{"name": "independent_ground_truth_position", "passed": metrics["goal_position_error_m"] <= scenario["position_tolerance_m"],
                       "expected": scenario["position_tolerance_m"], "actual": metrics["goal_position_error_m"]},
                      {"name": "independent_ground_truth_heading", "passed": metrics["goal_yaw_error_rad"] is None or metrics["goal_yaw_error_rad"] <= scenario["yaw_tolerance_rad"],
                       "expected": scenario["yaw_tolerance_rad"], "actual": metrics["goal_yaw_error_rad"]},
                      {"name": "collision_free", "passed": metrics["collision_count"] == 0, "actual": metrics["collision_count"]},
                      {"name": "native_no_fault", "passed": not self.source.physics.fault_layer,
                       "actual": performance["first_boundary"]},
                      {"name": "canonical_final_stop", "passed": self.source.physics.output == (0., 0.),
                       "actual": list(self.source.physics.output)}]
            complete = state.get("state") in {"SUCCEEDED", "FAILED"} and not state.get("finalizing") and not state.get("callback_error") and interrupted is None
            return {"status": "INCOMPLETE" if not complete else "PASSED" if state.get("state") == "SUCCEEDED" and all(row["passed"] for row in checks) else "FAILED",
                    "checks": checks, "reason": interrupted or state.get("error"), "scope": performance["scope"],
                    "safety_fault": bool(self.source.physics.fault_layer or metrics["collision_count"] or metrics["safety_stop_count"]),
                    "performance": performance, "simulation_config": self.source.physics.simulation_config}
        if self.source.scenario["kind"] == "semantic_trace":
            return self._assess_trace(state, interrupted)
        expected = [sample for sample in self.source.samples if sample["value"] >= self.source.scenario["threshold"]]
        returned = state.get("result")
        detected = returned.get("detections") if isinstance(returned, dict) else None
        count = returned.get("samples_checked") if isinstance(returned, dict) else None
        checks = [
            {"name": "all_source_samples_observed", "passed": self.source.observations == self.source.samples,
             "expected": len(self.source.samples), "actual": len(self.source.observations)},
            {"name": "detections_and_measurement_lineage", "passed": detected == expected,
             "expected": expected, "actual": detected},
            {"name": "reported_sample_count", "passed": type(count) is int and count == len(self.source.samples),
             "expected": len(self.source.samples), "actual": count},
        ]
        complete = (explicit_scenario and state.get("state") == "SUCCEEDED" and not state.get("finalizing")
                    and not state.get("callback_error") and interrupted is None)
        status = "INCOMPLETE" if not complete else "PASSED" if all(row["passed"] for row in checks) else "FAILED"
        return {"status": status, "checks": checks,
                "reason": interrupted or state.get("error") or
                          ("INDEPENDENT_GOAL_CHECK_REQUIRED" if not explicit_scenario else None),
                "scope": "Simulated semantic measurements, source lineage and real SDK/worker/Brain lifecycle; no physical robot evidence."}

    def _assess_trace(self, state, interrupted):
        scenario = self.source.scenario
        checks = [{"name": "all_trace_samples_observed", "passed": all(
            self.source.resource_reads.get(name, 0) == len(rows) for name, rows in scenario["resources"].items()),
            "expected": {name: len(rows) for name, rows in scenario["resources"].items()},
            "actual": dict(self.source.resource_reads)}]
        if "expected_result" in scenario:
            checks.append({"name": "independent_result", "passed": state.get("result") == scenario["expected_result"],
                           "expected": scenario["expected_result"], "actual": state.get("result")})
        if "expected_reports" in scenario:
            with self.runtime._sdk_event_lock:
                reports = [value["text"] for seq, producer, value in self.runtime._sdk_events
                           if seq > self._report_cursor and producer == "report"]
            checks.append({"name": "observed_reports", "passed": reports == scenario["expected_reports"],
                           "expected": scenario["expected_reports"], "actual": reports})
        observer = scenario.get("observer")
        coverage_complete = True
        if observer is not None:
            expected_ids = [event["event_id"] for event in observer["events"]]
            delivered_ids = [event["event_id"] for event in self.recorded_observation.delivered]
            checks.append({"name": "all_recorded_observer_events_observed", "passed": expected_ids == delivered_ids,
                           "expected": expected_ids, "actual": delivered_ids})
            coverage_complete = observer["complete"] and not self.recorded_observation.partial
            checks.append({"name": "recorded_observer_coverage", "passed": coverage_complete,
                           "expected": "complete supplied measurement interval without gaps",
                           "actual": {"complete": observer["complete"], "partial": self.recorded_observation.partial}})
        complete = (state.get("state") == "SUCCEEDED" and not state.get("finalizing")
                    and not state.get("callback_error") and interrupted is None and coverage_complete)
        return {"status": "INCOMPLETE" if not complete else "PASSED" if all(row["passed"] for row in checks) else "FAILED",
                "checks": checks, "reason": interrupted or state.get("error") or
                    ("RECORDED_OBSERVER_COVERAGE_INCOMPLETE" if not coverage_complete else None),
                "scope": "Supplied semantic trace and independent expected outputs/reports; no physical scene or freshness evidence."}

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            self.runtime.close()
            self.runtime._flush_evidence()
        finally:
            self._stop.set()
            if self._server_thread is not None:
                self._server_thread.join(timeout=2)
            with self._handlers_lock:
                handlers = tuple(self._handlers)
            for thread in handlers:
                thread.join(timeout=1)
            self.socket_path.unlink(missing_ok=True)
            if self._learning_session is not None:
                self._learning_session.finalize(SimpleNamespace(status=0))

    def __exit__(self, *_):
        self.close()


__all__ = ["MotorlessSkillSimulator", "threshold_scenario", "resolve_scenario", "compare_trials"]
