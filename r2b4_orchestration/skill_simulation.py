"""Small motor-free skill trial through the real host, SDK and Python worker.

The synthetic source supplies semantic scalar samples, not camera/physics
claims. Native V3 STOP ticks and host events use the existing capture path.
Generated skills are trusted ordinary Python; this is not a Python sandbox.
"""
from __future__ import annotations

from collections.abc import Mapping
import json
import math
from pathlib import Path
import socket
import tempfile
import threading
import time
from types import SimpleNamespace

from .robot_runtime import MAX_REPLY_BYTES, MAX_REQUEST_BYTES, SCHEMA, PublicRobotRuntime
from .skill_library import SkillLibrary
from v3.robot_interface import RobotInterface


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
        self.scenario = threshold_scenario()
        self.stops = 0
        self._lock = threading.Lock()

    def prepare(self, scenario, revision):
        with self._lock:
            self.scenario = threshold_scenario(scenario)
            measured = time.monotonic_ns()
            self.samples = [{"value": value, "measurement_time_ns": measured + index,
                             "sequence": index + 1, "revision": revision,
                             "source": "simulation", "lineage": {
                                 "producer": self.name, "generation": revision, "sample": index + 1}}
                            for index, value in enumerate(self.scenario["values"])]
            self.observations = []

    def capabilities(self):
        return {name: {"kind": "read" if name in self.reads else "action", "supported": True,
                       "available": name not in self.motion, "ready": name not in self.motion,
                       "source": "simulation", "reason": "MOTORLESS_SIMULATION" if name in self.motion else None}
                for name in self.capability_names}

    def read(self, resource):
        if resource == "simulation.scenario":
            return {**self.scenario, "source": "simulation", "resource": "simulation.sample"}
        if resource == "operator.status":
            return {"runtime_running": False, "runtime_pid": None, "source": "simulation"}
        if resource == "v3.status":
            return {"state": "IDLE", "tick_id": 0, "monotonic_ns": time.monotonic_ns(),
                    "safety_decision": "STOP", "source": "simulation"}
        if resource != "simulation.sample":
            raise KeyError(resource)
        with self._lock:
            if len(self.observations) >= len(self.samples):
                raise RuntimeError("SIMULATION_SOURCE_EXHAUSTED")
            sample = self.samples[len(self.observations)]
            self.observations.append(sample)
        self.owner._queue_evidence("simulation_observation", sample)
        # The source's concrete measurement identity is retained over the socket.
        return json.loads(json.dumps(sample))

    def execute(self, action, **parameters):
        if action != "v3.command.stop":
            raise RuntimeError("MOTORLESS_SIMULATION")
        if parameters:
            raise ValueError("STOP accepts no parameters")
        self.stops += 1
        return {"status": "STOPPED", "source": "simulation", "physical_output": False}


class MotorlessSkillSimulator:
    """An explicit host-only trial; all generated evidence remains under /tmp.

    ``trial`` runs a saved skill through a Brain goal and the real SDK socket.
    It measures the full observed samples and returned detections independently.
    The simulator implements one small scenario instead of modeling imaginary
    sensors. ``compile_evidence`` is an explicit, optional offline operation.
    """

    def __init__(self, project_root: Path | str, library: SkillLibrary | None = None):
        self.project_root = Path(project_root).resolve()
        self.root = Path(tempfile.mkdtemp(prefix="r2b4-learn-sim-", dir="/tmp"))
        self.library = library or SkillLibrary(self.project_root / "robot_skills")
        self.socket_path = self.root / "robot.sock"
        self.source = _SemanticSource()
        backend = RobotInterface(project_root=self.root, controller=self.source,
                                 adapters=(self.source,), upper_runtime=False)
        self.runtime = PublicRobotRuntime(backend, root=self.root, socket_path=self.socket_path)
        self.runtime.skill_library = self.library
        self.runtime.skills.library = self.library
        # Host tools stay scoped to temporary simulation data. Agent source and
        # evidence tools are composed separately against the actual project.
        self.runtime.skill_observation.close()
        self.runtime.skill_observation = None
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

    @staticmethod
    def scenario_descriptor(value=None):
        return {**threshold_scenario(value), "resource": "simulation.sample", "source": "simulation",
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
                    priority = request.get("operation") in {"stop", "preempt", "revoke"}
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

        resolved_scenario = threshold_scenario(scenario)
        number = len(self._trials) + 1
        self.source.prepare(resolved_scenario, f"simulation-trial-{number}")
        directory = self.root / f"trial-{number}"
        directory.mkdir()
        conf = self.project_root / "conf"
        config = ConfigResolver(conf / "hardver.json", conf / "fizika.json", conf / "speed_map.json",
                                conf / "vezerles.json").resolve().runtime.composition.live_control.control
        writer, gateway = OfflineMotorSink(), FakeCommandGateway()
        composition = NativeControlComposition(writer, config)
        session = McapCaptureSession(f"motorless-skill-{number}", directory / "capture.mcap",
                                    configuration={"production_control": config},
                                    metadata={"source": "simulation", "scenario": resolved_scenario,
                                              "skill": name, "physical_hardware": False},
                                    project_root=self.root,
                                    config=McapCaptureConfig(mode="append_only", tick_sample_hz=50))
        # Drain older host evidence before the follower establishes this scope.
        self.runtime._flush_evidence()
        session.start()
        start_ns = time.monotonic_ns()

        def stop_tick(tick):
            context = TickContext(tick, start_ns + tick * 20_000_000)
            def sample(device, kind, **values):
                return DeviceSample(device, kind, tick, context.monotonic_ns,
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
            session.observe(ExecutionRecord(inputs, result))

        run_id = None
        goal = None
        interrupted = None
        try:
            stop_tick(0)
            self.runtime._queue_evidence("simulation_trial_started", {
                "name": name, "scenario": resolved_scenario, "source": "simulation"})
            goal = self.runtime.execute("brain.submit", {"text": self.goal_text or
                f"Motor nélküli tanulási próba: {name}; {json.dumps(resolved_scenario)}"})
            adopted = self.runtime.execute("brain.adopt", {"goal_id": goal["goal_id"],
                "plan": {"skill": name, "parameters": parameters}})
            deadline = time.monotonic() + timeout_s
            while True:
                self.runtime.brain.step()
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
            assessment = self._assess(state, interrupted, scenario is not None)
            self.runtime._queue_evidence("simulation_trial_assessment", {
                "run_id": run_id, "goal_id": goal["goal_id"], "source_hash": state.get("source_hash"),
                "assessment": assessment, "source": "simulation"})
        finally:
            self.runtime._flush_evidence()
            try:
                stop_tick(1)
            finally:
                composition.close()
                capture = session.finalize(SimpleNamespace(status=0))
        result = {"runtime": state, "assessment": assessment, "observations": list(self.source.observations),
                  "brain_goal": goal, "scenario": resolved_scenario, "source": "simulation",
                  "capture": str(capture), "capture_path": str(capture),
                  "artifact_directory": str(directory), "motor_writes": len(writer.writes),
                  "physical_output": False}
        self._trials.append(result)
        (directory / "trial.json").write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
        return result

    def _assess(self, state, interrupted, explicit_scenario):
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

    def close(self):
        if self._closed:
            return
        self._closed = True
        self.runtime.close()
        self.runtime._flush_evidence()
        self._stop.set()
        if self._server_thread is not None:
            self._server_thread.join(timeout=2)
        with self._handlers_lock:
            handlers = tuple(self._handlers)
        for thread in handlers:
            thread.join(timeout=1)
        self.socket_path.unlink(missing_ok=True)

    def __exit__(self, *_):
        self.close()


__all__ = ["MotorlessSkillSimulator", "threshold_scenario"]
