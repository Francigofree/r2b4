"""Offline evidence for bounded person search and exact canonical follow binding."""
from __future__ import annotations

from contextlib import nullcontext
from dataclasses import replace
from types import SimpleNamespace
import threading
import time

import pytest

from rig import ROOT, healthy_localization, resolved_config
from r2b4_orchestration.behavior_system import BehaviorLifecycle, BehaviorSystem
from r2b4_orchestration.search_person import SearchAnyPerson
from r2b4_orchestration.semantic_projector import SemanticProjector
from r2b4_orchestration.world_model import PublicWorldModel
from v3.adapters.resident_command import AtomicResidentCommandGateway, ResidentCommandClient, ResidentCommandMailboxConfig
from v3.adapters.v3_control import V3ControlInterfaceAdapter
from v3.capture_encoding import encode_value
from v3.contracts import (CommandMode, CommandRequest, DataField, LOCAL_FRAME_ID, MissionIntent,
                          MissionLifecycle, ObstacleTrack, RobotEstimate, RobotRelativeGeometry,
                          RollingLocalCostmap, TickContext, WorldSnapshot)
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import NavigationStateCheckpoint, TrajectoryNavigator
from v3.operator_controller import OperatorController
from v3.replay import _decode_production_value, _expanded_expected_layers


class Clock:
    now = 1_000_000_000

    def __call__(self):
        return self.now


class SearchRobot:
    def __init__(self):
        self.clock = Clock()
        self.world = PublicWorldModel(clock_ns=self.clock)
        self.projector = SemanticProjector(self.world, clock_ns=self.clock)
        self.pid = 123
        self.actions = []
        self.stops = 0
        self.status = {}

    def capabilities(self):
        return {"capabilities": {}}

    def query(self, query):
        return self.world.query(query)

    def read(self, resource):
        if resource == "operator.status":
            return {"runtime_running": True, "runtime_pid": self.pid, "capture_mode": "full", "capture_hz": 10}
        if resource == "v3.status":
            return self.status
        if resource == "world.snapshot":
            return self.world.snapshot().to_jsonable()
        raise KeyError(resource)

    def execute(self, action, **params):
        assert action in {"v3.command.face_person", "v3.command.navigate", "v3.command.follow_person"}
        self.actions.append((action, params))
        self.publish("mission-search-" + str(len(self.actions)),
                     mode={"v3.command.face_person": "FACE_PERSON", "v3.command.navigate": "NAVIGATE",
                           "v3.command.follow_person": "FOLLOW_PERSON"}[action])
        return {"command_id": "search-" + str(len(self.actions))}

    def publish(self, mission_id=None, *, mode=None, tracks=(), navigation="ACTIVE"):
        self.clock.now += 10_000_000
        old = self.status.get("mission", {})
        self.status = {
            "state": "RUNNING", "tick_id": self.clock.now // 10_000_000,
            "monotonic_ns": self.clock.now, "fault_layer": None,
            "safety_decision": "ALLOW" if navigation == "ACTIVE" else "STOP",
            "safety_reason": "ALLOW" if navigation == "ACTIVE" else "NOT_ACTIVE",
            "mission": {"mission_id": mission_id or old["mission_id"],
                        "mode": mode or old["mode"], "lifecycle": "ACTIVE"},
            "navigation": {"mission_id": mission_id or old["mission_id"], "status": navigation,
                           "reason": "PERSON_TARGET_NOT_AVAILABLE" if navigation == "INVALIDATED" else None},
            "estimate": {"local_pose": {"frame_id": LOCAL_FRAME_ID, "x_m": 2.0, "y_m": 3.0, "yaw_rad": 0.0},
                         "localization_quality": {"local_pose_continuous": True,
                                                  "local_translation": "GOOD", "heading": "GOOD"}},
            "world": {"frame_id": LOCAL_FRAME_ID, "person_tracks": list(tracks)},
        }
        self.projector.completed_status(self.status, runtime_pid=self.pid)

    def stop(self):
        self.stops += 1


def _person(clock, *, track="person-7", observed=True, confidence=.9):
    return {"track_id": track, "x_m": 4., "y_m": 3., "confidence": confidence,
            "measurement_monotonic_ns": clock.now, "prediction_valid_until_ns": clock.now + 500_000_000,
            "estimate_status": "OBSERVED" if observed else "PREDICTED"}


def _search(robot, **params):
    system = BehaviorSystem(robot, clock_ns=robot.clock)
    system.register("search_any_person", SearchAnyPerson)
    system.start("search_any_person", params, max_duration_s=60)
    return system


def test_search_sweeps_bounded_views_then_returns_exact_canonical_target():
    robot = SearchRobot()
    system = _search(robot, max_views=2, acquisition_duration_s=.1)
    robot.publish(navigation="INVALIDATED")
    assert system.step().lifecycle is BehaviorLifecycle.ACTIVE
    robot.clock.now += 100_000_000
    robot.publish(navigation="INVALIDATED")
    assert system.step().lifecycle is BehaviorLifecycle.STARTING
    action, params = robot.actions[-1]
    assert action == "v3.command.navigate"
    assert (params["x_m"], params["y_m"], params["frame_id"]) == (2., 3., LOCAL_FRAME_ID)
    assert params["expected_runtime_pid"] == 123 and params["capture"] is False
    robot.publish(navigation="COMPLETE")
    assert system.step().lifecycle is BehaviorLifecycle.STARTING
    assert robot.actions[-1][0] == "v3.command.face_person"
    assert robot.actions[-1][1]["expected_runtime_pid"] == 123
    robot.publish(tracks=(_person(robot.clock),))
    found = system.step()
    assert found.lifecycle is BehaviorLifecycle.COMPLETED
    assert dict(found.result)["target_track_id"] == "person-7"
    assert dict(found.result)["runtime_pid"] == 123
    assert robot.stops == 1
    assert system.history()[-1].to_jsonable()["result"]["target_track_id"] == "person-7"


def test_search_exhaustion_and_predicted_track_fail_closed():
    robot = SearchRobot()
    system = _search(robot, max_views=1, acquisition_duration_s=.1)
    robot.publish(tracks=(_person(robot.clock, observed=False),))
    assert system.step().lifecycle is BehaviorLifecycle.ACTIVE
    robot.clock.now += 100_000_000
    robot.publish(navigation="INVALIDATED")
    assert system.step().reason == "SEARCH_VIEWS_EXHAUSTED"
    assert len(robot.actions) == 1 and robot.stops == 1


def test_stopped_v3_search_requests_canonical_facing_readiness_before_status():
    class StoppedRobot(SearchRobot):
        running = False
        reads_before_start = 0

        def read(self, resource):
            if resource == "operator.status":
                return {"runtime_running": self.running, "runtime_pid": self.pid if self.running else None,
                        "capture_mode": "full", "capture_hz": 10}
            if resource == "v3.status" and not self.running:
                self.reads_before_start += 1
                raise RuntimeError("NO_LIVE_RUNTIME_STATUS")
            return super().read(resource)

        def execute(self, action, **params):
            assert action == "v3.command.face_person"
            self.running = True
            self.pid = 789
            return super().execute(action, **params)

    robot = StoppedRobot()
    system = _search(robot)
    assert system.snapshot().lifecycle is BehaviorLifecycle.STARTING
    assert robot.reads_before_start == 0
    assert [action for action, _ in robot.actions] == ["v3.command.face_person"]
    robot.publish(tracks=(_person(robot.clock),))
    completed = system.step()
    assert completed.lifecycle is BehaviorLifecycle.COMPLETED
    assert dict(completed.result)["runtime_pid"] == 789
    assert robot.stops == 1


def test_search_runtime_change_during_acquisition_fails_closed():
    robot = SearchRobot()
    system = _search(robot)
    robot.pid = 456
    robot.publish(tracks=(_person(robot.clock),))
    assert system.step().reason == "SEARCH_RUNTIME_CHANGED"
    assert len(robot.actions) == 1 and robot.stops == 1


def test_one_agent_proposal_searches_binds_and_follows_300_seconds_in_same_brain_goal():
    from r2b4_orchestration.agent_contracts import AgentModelReply
    from r2b4_orchestration.agent_core import AgentCore, AgentToolBroker
    from r2b4_orchestration.robot_runtime import PublicRobotRuntime
    from v3.action_catalog import voice_action_descriptors

    class Planner:
        model = "offline-proposal"
        calls = 0

        def complete_agent_step(self, messages, tools, actions, **options):
            self.calls += 1
            assert self.calls == 1, "admitted Brain execution must not call the Agent again"
            return AgentModelReply(self.model, goal_plan={"steps": [
                {"action": "behavior.search_any_person", "bind_target": True,
                 "parameters": {"max_duration_s": 60}},
                {"action": "behavior.follow_person", "use_bound_target": True,
                 "parameters": {"max_duration_s": 300}},
            ]})

    robot = SearchRobot()
    owner = PublicRobotRuntime(robot, world=robot.world, clock_ns=robot.clock)
    planner = Planner()
    agent = AgentCore(planner, AgentToolBroker(()))
    text = "Keress egy embert, majd kövesd 5 percig."
    pending = owner.execute("brain.submit", {"text": text, "source": "HUMAN"})
    decision = agent.run([{"role": "user", "content": text}],
                         [item.to_jsonable() for item in voice_action_descriptors()])
    accepted = owner.execute("brain.adopt", {"goal_id": pending["goal_id"], "plan": decision.goal_plan})
    deadline = time.monotonic() + 2
    while accepted["lifecycle"] == "STARTING" and time.monotonic() < deadline:
        threading.Event().wait(.005)
        accepted = owner.brain.goal(pending["goal_id"])
    assert accepted["lifecycle"] == "ACTIVE" and accepted["step_index"] == 0
    assert robot.actions[-1][0] == "v3.command.face_person"
    robot.publish(tracks=(_person(robot.clock),))
    owner.poll()
    following = owner.brain.goal(pending["goal_id"])
    assert following["lifecycle"] == "ACTIVE" and following["step_index"] == 1
    assert following["target"]["target_track_id"] == "person-7"
    action, parameters = robot.actions[-1]
    assert action == "v3.command.follow_person"
    assert parameters["target_track_id"] == "person-7" and parameters["expected_runtime_pid"] == 123
    assert parameters["capture"] is False and parameters["capture_mode"] == "full"
    assert parameters["capture_hz"] == 10
    robot.publish(tracks=(_person(robot.clock),))
    owner.poll()  # Correlated physical following starts the 300 second duration.
    execution = owner.behaviors.snapshot().execution_started_ns
    assert execution is not None
    robot.clock.now = execution + 299_000_000_000
    robot.publish(tracks=(_person(robot.clock),))
    owner.poll()
    assert owner.brain.goal(pending["goal_id"])["lifecycle"] == "ACTIVE"
    robot.clock.now = execution + 300_000_000_000
    robot.publish(tracks=(_person(robot.clock),))
    owner.poll()
    completed = owner.brain.goal(pending["goal_id"])
    assert completed["lifecycle"] == "COMPLETED" and completed["reason"] == "REQUESTED_DURATION_REACHED"
    assert completed["goal_id"] == pending["goal_id"]
    assert planner.calls == 1 and len(robot.actions) == 2
    intents = [event.to_jsonable() for event in owner.behaviors.history() if event.kind == "BEHAVIOR_INTENT"]
    assert {event["goal_id"] for event in intents} == {pending["goal_id"]}
    assert [event["subtask_id"] for event in intents] == [
        pending["goal_id"] + ":step:1", pending["goal_id"] + ":step:2"]


@pytest.mark.parametrize("bind,use", [(False, False), (True, False), (False, True)])
def test_search_follow_plan_missing_target_binding_is_rejected_before_execution(bind, use):
    from r2b4_orchestration.robot_runtime import PublicRobotRuntime
    robot = SearchRobot()
    owner = PublicRobotRuntime(robot, world=robot.world, clock_ns=robot.clock)
    pending = owner.brain.submit("Keress egy embert, majd kövesd 5 percig.")
    result = owner.brain.adopt(pending["goal_id"], {"steps": [
        {"action": "behavior.search_any_person", "bind_target": bind},
        {"action": "behavior.follow_person", "use_bound_target": use,
         "parameters": {"max_duration_s": 300}},
    ]})
    assert result["lifecycle"] == "FAILED" and "TARGET_BINDING" in result["reason"]
    assert robot.actions == []


def _scene(tick, tracks):
    context = TickContext(tick, 1_000_000_000 + tick * 20_000_000)
    estimate = RobotEstimate(context, LOCAL_FRAME_ID, 0., 0., 0., 0., 0.,
                             tuple(.01 if i in {0, 6, 12} else 0. for i in range(25)),
                             localization_quality=healthy_localization())
    world = WorldSnapshot(context, LOCAL_FRAME_ID, 1, tuple(tracks), 0,
                          RollingLocalCostmap(LOCAL_FRAME_ID, 1, .1, 2.5, (), 1, 0),
                          robot_relative_geometry=RobotRelativeGeometry("ROBOT_BASE", 1, context.monotonic_ns, 100, 0))
    return context, estimate, world


def test_bound_follow_gateway_mission_checkpoint_replay_never_switches_person(tmp_path):
    config = resolved_config().runtime.composition.live_control.control
    mailbox = ResidentCommandMailboxConfig(tmp_path / "command.json")
    client = ResidentCommandClient(mailbox, monotonic_ns=lambda: 1_000_000_000)
    client.publish_follow_person("bound-follow", max_v_mps=.25, max_omega_rad_s=.3,
                                 ttl_ns=200_000_000, target_track_id="person-7")
    context, estimate, world = _scene(0, (ObstacleTrack("person-7", 1.8, 0., .2, 0., 0., .8),
                                        ObstacleTrack("person-8", 1.6, .1, .2, 0., 0., 1.)))
    command = AtomicResidentCommandGateway(mailbox, monotonic_ns=lambda: context.monotonic_ns).snapshot(context)
    manager = MissionManager(config.mission)
    mission = manager.evaluate(command)
    assert mission.target_track_id == "person-7"
    assert _decode_production_value(encode_value(mission), MissionIntent, "mission") == mission
    navigator = TrajectoryNavigator(config.navigation, async_config=replace(config.async_l6, enabled=False, completion_inputs=False))
    navigator.evaluate(mission, estimate, world)
    assert navigator.follow_person_evidence.locked_target_uid == "person-7"
    checkpoint = _decode_production_value(encode_value(navigator.checkpoint()), NavigationStateCheckpoint, "navigation")
    replay = TrajectoryNavigator(config.navigation, async_config=replace(config.async_l6, enabled=False, completion_inputs=False))
    replay.restore(checkpoint)
    next_context, next_estimate, next_world = _scene(1, (world.obstacle_tracks[1],))
    next_mission = manager.evaluate(replace(command, context=next_context, expiry_tick=1))
    missing = navigator.evaluate(next_mission, next_estimate, next_world)
    assert replay.evaluate(next_mission, next_estimate, next_world) == missing
    assert navigator.follow_person_evidence.locked_target_uid == "person-7"
    assert missing.reason == "PERSON_OCCLUDED_HOLD"
    # A bound target unavailable at initial acquisition cannot select person-8.
    fresh = TrajectoryNavigator(config.navigation, async_config=replace(config.async_l6, enabled=False, completion_inputs=False))
    assert fresh.evaluate(next_mission, next_estimate, next_world).reason == "PERSON_TARGET_NOT_AVAILABLE"
    assert fresh.follow_person_evidence.locked_target_uid is None
    old = encode_value(replace(mission, target_track_id=None))
    del old["target_track_id"]
    assert _expanded_expected_layers({}, {"L5": old})["L5"]["target_track_id"] is None


def test_bound_follow_crosses_operator_cli_ingress_and_rejects_restarted_runtime(tmp_path, monkeypatch):
    from v3 import control_cli
    controller = OperatorController(ROOT)
    calls = []
    monkeypatch.setattr(controller, "operator_transition", lambda: nullcontext())
    monkeypatch.setattr(controller, "snapshot", lambda: SimpleNamespace(runtime_pid=123))
    monkeypatch.setattr(controller, "current_capture_mode", lambda: "full")
    monkeypatch.setattr(controller, "current_capture_hz", lambda: 10)
    monkeypatch.setattr(controller, "_start_motion", lambda *args, **kwargs: calls.append(args[3]) or (321, "full"))
    adapter = V3ControlInterfaceAdapter(controller)
    parameters = dict(target_track_id="person-7", expected_runtime_pid=123, capture=False, capture_mode="full", capture_hz=10)
    adapter.execute("v3.command.follow_person", **parameters)
    monkeypatch.setattr(control_cli, "_runtime_path", lambda value: tmp_path / value.rsplit("/", 1)[-1])
    monkeypatch.setattr(control_cli, "_active_preflight", lambda path: None)
    monkeypatch.setattr(control_cli, "_run_active", lambda client, publish, **kwargs: 0 if publish(kwargs["command_id"]) else 1)
    assert control_cli.main(calls[0][3:]) == 0
    payload = (tmp_path / "v3_command.json").read_text()
    assert '"target_track_id": "person-7"' in payload
    monkeypatch.setattr(controller, "snapshot", lambda: SimpleNamespace(runtime_pid=456))
    with pytest.raises(RuntimeError, match="runtime session changed"):
        adapter.execute("v3.command.follow_person", **parameters)
    assert len(calls) == 1
    with pytest.raises(ValueError, match="expected_runtime_pid"):
        adapter.execute("v3.command.follow_person", target_track_id="person-7")


def test_bound_follow_full_native_mcap_replayer_matches_and_preserves_target(tmp_path):
    from v3.composition.full_fake import OfflineMotorSink
    from v3.composition.native_control import NativeControlComposition
    from v3.contracts import DeviceHealth, DeviceHealthState, DeviceSample, LifecycleState, RawDeviceBatch
    from v3.engine import TickInputs
    from v3.execution import ExecutionRecord
    from v3.layers.l6_navigation import FollowPersonEvidence
    from v3.mcap_capture import McapCaptureConfig, McapCaptureConsumer, TICK_TOPIC, RUNTIME_TOPIC
    from v3.mcap_reader import McapReader
    from v3.observation import ObservationHub
    from v3.replay import replay_capture

    config = resolved_config().runtime.composition.live_control.control
    composition = NativeControlComposition(OfflineMotorSink(), config)
    hub = ObservationHub()
    consumer = McapCaptureConsumer("offline-bound-follow", tmp_path / "bound-follow.mcap",
        subscription=hub.subscribe_reliable("capture", capacity=128, required=True),
        configuration={"production_control": config},
        metadata={"evidence_scope": "offline closed canonical inputs", "target_track_id": "person-7"},
        config=McapCaptureConfig(mode="append_only", tick_sample_hz=50))
    consumer.start()
    selected = []
    final_world = None
    try:
        for tick in range(55):
            context = TickContext(tick, 4_000_000_000 + tick * 20_000_000)

            def sample(device, kind, **values):
                return DeviceSample(device, kind, tick + 1, context.monotonic_ns,
                                    tuple(DataField(key, value) for key, value in values.items()))

            samples = [
                sample("WHEEL_ENCODERS", "wheel_velocity", left_mps=0., right_mps=0., trust=1.),
                sample("BNO055_IMU", "ekf_heading", yaw_rad=0., omega_rad_s=0., confidence=1., omega_confidence=1.),
                sample("RPLIDAR_C1", "lidar_health", age_ns=0, point_count=1),
                sample("RPLIDAR_C1", "lidar_local_points", frame_id="ROBOT_BASE", point_count=1,
                       point_000_x_m=2.5, point_000_y_m=0., point_000_quality=10),
                sample("RPLIDAR_C1", "lidar_safety_clearance", age_ns=0,
                       **{f"{sector}_{key}": value for sector in ("front", "rear", "left", "right")
                          for key, value in (("clearance_m", 2.), ("observation_count", 10))}),
                sample("PERSON_DETECTOR_FRONT", "person_detection", person_count=0, person_detected=False),
                sample("PERSON_TRACK_8", "obstacle_track", track_id="person-8", x_m=1.6, y_m=.4,
                       radius_m=.2, vx_mps=0., vy_mps=0., confidence=1.),
            ]
            if tick < 20:
                samples.append(sample("PERSON_TRACK_7", "obstacle_track", track_id="person-7", x_m=1.8, y_m=0.,
                                      radius_m=.2, vx_mps=0., vy_mps=0., confidence=.8))
            health = tuple(DeviceHealth(name, DeviceHealthState.OK) for name in sorted(
                set(config.critical_device_ids) | {"PERSON_DETECTOR_FRONT", "PERSON_TRACK_7", "PERSON_TRACK_8"}))
            command = CommandRequest(context, "bound-follow-native", CommandMode.FOLLOW_PERSON,
                                     (DataField("target_track_id", "person-7"),), tick)
            inputs = composition.close_inputs(TickInputs(context, RawDeviceBatch(context, tuple(samples), health),
                                                        command, LifecycleState.ACTIVE))
            result = composition.run_tick(inputs)
            assert result.trace.fault_layer is None
            layers = {row.layer: row.output for row in result.trace.layers}
            assert layers["L5"].target_track_id == "person-7"
            final_world = layers["L4"]
            for evidence in composition.tick_evidence:
                if isinstance(evidence, FollowPersonEvidence):
                    selected.append(evidence.locked_target_uid)
                    assert evidence.locked_target_uid in {None, "person-7"}
            hub.publish(ExecutionRecord(inputs, result, evidence=composition.tick_evidence,
                                        state_checkpoint_after=composition.checkpoint()), topic="v3.capture_record")
    finally:
        composition.close()
        hub.close()
        captured = consumer.finish("PASS")
    assert "person-7" in selected
    assert {track.track_id for track in final_world.obstacle_tracks} == {"person-8"}
    assert selected[-1] == "person-7"  # Losing the target cannot acquire the visible alternative.
    assert captured is not None and captured.replay_complete
    reader = McapReader(captured.path)
    metadata = reader.first_json(RUNTIME_TOPIC)[1]["metadata"]
    assert metadata["target_track_id"] == "person-7"
    assert metadata["evidence_scope"] == "offline closed canonical inputs"
    ticks = [row for _, row in reader.iter_json_messages(topics=[TICK_TOPIC])]
    assert len(ticks) == 55
    assert all(next(field["value"] for field in row["inputs"]["command"]["goal"]
                    if field["key"] == "target_track_id") == "person-7" for row in ticks)
    assert all(row["expected"]["layers"]["L5"]["target_track_id"] == "person-7" for row in ticks)
    replay = replay_capture(captured.path, project_root=ROOT)
    assert replay["status"] == "MATCH", replay["diagnostics"]
    assert replay["determinism"]["repeated_trace_match"]
    from tools.mcap_evidence.compiler import compile_evidence
    from tools.mcap_evidence.query import query
    from tools.mcap_evidence.verify import verify
    compiled = compile_evidence(captured.path, tmp_path / "bound-follow-evidence", workers=1)
    assert compiled["compiler_status"] == "COMPLETE"
    assert verify(compiled["output"], source=captured.path)["status"] == "PASS"
    indexed = list(query(compiled["output"], topic=TICK_TOPIC, field="expected.layers.L5.target_track_id"))
    assert len(indexed) == 55
    assert all(row["payload"]["expected"]["layers"]["L5"]["target_track_id"] == "person-7" for row in indexed)


@pytest.mark.parametrize("target", ["", "person 7", "obstacle-7", 7])
def test_malformed_follow_binding_is_rejected_by_l5(target):
    config = resolved_config().runtime.composition.live_control.control
    command = CommandRequest(TickContext(0, 0), "invalid-bound-follow", CommandMode.FOLLOW_PERSON,
                             (DataField("target_track_id", target),), 0)
    assert MissionManager(config.mission).evaluate(command).lifecycle is MissionLifecycle.FAILED
