"""Motor-free checks of the real NAVIGATE ingress and L5-L9 execution path."""
from __future__ import annotations

import json
import math
import time
from dataclasses import replace
from types import SimpleNamespace

import pytest

from rig import resolved_config
from v3.adapters.resident_command import AtomicResidentCommandGateway, ResidentCommandClient, ResidentCommandMailboxConfig
from v3.contracts import CommandMode, CommandRequest, DataField, NavigationStatus, RobotEstimate, RollingLocalCostmap, TickContext, WorldSnapshot
from v3.layers.l5_command_mission import MissionManager


def test_navigate_motion_mailbox_identity_validation_and_expiry(tmp_path):
    now = 1_000_000_000
    config = ResidentCommandMailboxConfig(path=tmp_path / "command.json")
    client = ResidentCommandClient(config, monotonic_ns=lambda: now)
    gateway = AtomicResidentCommandGateway(config, monotonic_ns=lambda: now)
    manager = MissionManager(resolved_config().runtime.composition.live_control.control.mission)
    target = dict(x_m=1.0, y_m=-0.4, yaw_rad=0.9, max_v_mps=0.2, max_omega_rad_s=0.6)
    for tick in range(2):
        revision = client.publish_navigate("navigate-test", **target, ttl_ns=200_000_000)
        context = TickContext(tick, now)
        command = gateway.snapshot(context)
        direct = CommandRequest(context, "navigate-test", CommandMode.NAVIGATE,
                                tuple(DataField(key, target[key]) for key in ("x_m", "y_m", "max_v_mps", "max_omega_rad_s", "yaw_rad")), tick)
        assert command == direct
        mission = manager.evaluate(command)
        assert mission.mission_id == "mission-navigate-test" and mission.lifecycle.value == "ACTIVE"
        assert mission.target_pose.yaw_rad == 0.9
        assert revision == tick + 1
        now += 100_000_000
    now += 200_000_000
    assert gateway.snapshot(TickContext(2, now)).mode is CommandMode.STOP
    good = config.path.read_bytes()
    for changed in ({"x_m": True}, {"yaw_rad": float("nan")}, {"max_v_mps": 0.51}, {"raw_motor": 1}):
        invalid = {**target, **changed}
        with pytest.raises((ValueError, TypeError)):
            client.publish_navigate("bad", **invalid, ttl_ns=200_000_000)
        assert config.path.read_bytes() == good
        # The consumer independently validates even if a producer bypasses its client.
        payload = json.loads(good)
        payload.update(changed, revision=100, issued_monotonic_ns=now, expires_monotonic_ns=now + 200_000_000)
        config.path.write_text(json.dumps(payload))
        with pytest.raises(ValueError):
            AtomicResidentCommandGateway(config, monotonic_ns=lambda: now).snapshot(TickContext(3, now))
        config.path.write_bytes(good)
    client.publish_navigate("position-only", **{k: v for k, v in target.items() if k != "yaw_rad"}, ttl_ns=200_000_000)
    command = gateway.snapshot(TickContext(4, now))
    assert manager.evaluate(command).target_pose.yaw_rad is None
    client.publish_stop("stop", ttl_ns=200_000_000)
    assert gateway.snapshot(TickContext(5, now)).mode is CommandMode.STOP


def test_navigate_motion_interface_runs_canonical_producer_and_accepts_zero_goal(tmp_path, monkeypatch):
    from v3 import control_cli
    import v3.operator_controller as operator_module
    from v3.adapters.v3_control import V3ControlInterfaceAdapter
    from v3.external_gateway import ExternalRobotGateway, GatewayPolicy
    from v3.operator_controller import OperatorController
    from v3.robot_interface import RobotInterface

    runtime_dir = tmp_path / "runtime"
    runtime_dir.mkdir()
    status = {"schema": control_cli.RESIDENT_PROCESS_STATUS_SCHEMA, "state": "RUNNING", "ready_for_active": True,
              "tick_id": 1, "monotonic_ns": time.monotonic_ns(), "safety_decision": "STOP", "fault_layer": None,
              "enabled": False, "left_output": 0, "right_output": 0}
    (runtime_dir / "v3_status.json").write_text(json.dumps(status))
    (runtime_dir / "v3_status.json").chmod(0o600)
    produced = []
    sessions = []
    stopped = []

    class Controller(OperatorController):
        def status(self):
            return {"runtime_running": True}

        def ensure_runtime(self, *args):
            return 123

        def stop(self, **kwargs):
            stopped.append("STOP")

        def current_capture_mode(self):
            return "nincs"

        def _runtime_pid(self):
            return 123

        def _pid_matches(self, *args):
            return True

        def _read_status_optional(self):
            return dict(status)

    def run_active(client, publish, **kwargs):
        sessions.append((kwargs["owner_pid"], kwargs["max_runtime_s"]))
        config = ResidentCommandMailboxConfig(path=runtime_dir / "v3_command.json")
        gateway = AtomicResidentCommandGateway(config)
        for tick in (2, 3):
            publish(kwargs["command_id"])
            command = gateway.snapshot(TickContext(tick, time.monotonic_ns()))
            produced.append(command)
        status.update(tick_id=3, monotonic_ns=time.monotonic_ns(),
                      mission={"mission_id": f"mission-{command.command_id}"},
                      navigation={"mission_id": f"mission-{command.command_id}", "status": "COMPLETE"})
        return 0

    def popen(command, **kwargs):
        assert command[1:3] == ["-m", "v3.control_cli"]
        assert control_cli.main(command[3:]) == 0
        return SimpleNamespace(pid=456)

    monkeypatch.setattr(control_cli, "PROJECT_ROOT", tmp_path)
    monkeypatch.setattr(control_cli, "_run_active", run_active)
    monkeypatch.setattr(operator_module.subprocess, "Popen", popen)
    controller = Controller(tmp_path)
    interface = RobotInterface(tmp_path, controller=controller, adapters=[V3ControlInterfaceAdapter(controller)])
    gateway = ExternalRobotGateway(interface, policy=GatewayPolicy(allow_execute=True, session_watchdog_s=10))
    response = gateway.handle({"request_id": "test", "operation": "execute", "name": "v3.command.navigate",
                               "parameters": {"x_m": 0, "y_m": 0, "yaw_rad": 0.2, "capture": False, "capture_mode": "nincs"}})
    assert response.status == "ACCEPTED", response
    assert len(produced) == 2 and all(c.mode is CommandMode.NAVIGATE for c in produced)
    assert produced[0].goal == produced[1].goal
    assert produced[0].command_id == produced[1].command_id == response.result.command_id
    assert sessions == [(gateway.policy.session_owner_pid, 10.0)]
    assert stopped == ["STOP"] and status["enabled"] is False
    assert "v3.command.navigate" in interface.capabilities()["capabilities"]
    # Freshness gates acceptance even when an old matching COMPLETE still exists.
    status["monotonic_ns"] = time.monotonic_ns() - 1_000_000_000
    assert controller.live_runtime_status() is None


def test_navigate_motion_closed_loop_turn_translation_and_deterministic_replay():
    from v3.composition.mission_navigation import MissionNavigationComposition, MissionNavigationInputs

    config = resolved_config().runtime.composition.live_control.control

    def composition(*, closed=False):
        return MissionNavigationComposition(
            mission_config=config.mission, navigation_config=config.navigation,
            async_config=config.async_l6 if closed else replace(config.async_l6, completion_inputs=False), selection_config=config.motion_selection,
            motion_config=config.motion_realization, constraints_config=config.operational_constraints,
        )

    # Integrate the real constrained twist in a deterministic, empty local world.
    for target_x, target_y, target_yaw in ((0, 0, math.pi / 2), (0, 0, -math.pi / 4),
                                           (0, 0, math.pi), (0.4, 0, math.pi / 2)):
        runtime = composition()
        x = y = yaw = v = omega = 0.0
        frames, traces = [], []
        for tick in range(1000):
            context = TickContext(tick, 1_000_000_000 + tick * 20_000_000)
            command = CommandRequest(context, "closed-loop", CommandMode.NAVIGATE,
                                     tuple(DataField(k, val) for k, val in dict(x_m=target_x, y_m=target_y, yaw_rad=target_yaw,
                                                                               max_v_mps=0.2, max_omega_rad_s=0.6).items()), tick)
            estimate = RobotEstimate(context, "odom", x, y, yaw, v, omega, (0.0,) * 25)
            costmap = RollingLocalCostmap("odom", tick, 0.05, 2.5, (), tick, 0)
            world = WorldSnapshot(context, "odom", tick, (), 0, costmap)
            frame = MissionNavigationInputs(context, command, estimate, world)
            trace = runtime.run_tick(frame)
            frames.append(frame)
            traces.append(trace)
            if trace.navigation.status is NavigationStatus.COMPLETE:
                assert trace.motion.requested_v_mps == trace.motion.requested_omega_rad_s == 0
                break
            v, omega = trace.constrained.allowed_v_mps, trace.constrained.allowed_omega_rad_s
            if target_x == target_y == 0:
                assert v == 0  # Terminal heading never requests translation.
            x += v * math.cos(yaw) * 0.02
            y += v * math.sin(yaw) * 0.02
            yaw += omega * 0.02
        else:
            pytest.fail(f"NAVIGATE did not complete: target={(target_x, target_y, target_yaw)}, pose={(x, y, yaw)}")
        assert math.hypot(x - target_x, y - target_y) <= config.mission.default_constraints.goal_tolerance_m
        assert abs((yaw - target_yaw + math.pi) % (2 * math.pi) - math.pi) <= config.mission.default_constraints.yaw_tolerance_rad
        assert composition().replay(frames) == tuple(traces)
        if target_x == target_y == 0:
            assert composition(closed=True).replay(frames) == tuple(traces)
        # Removing or aging geometry still invalidates a terminal turn.
        first = frames[0]
        if target_x == target_y == 0:
            for invalid_world in (replace(first.world, local_costmap=None), replace(first.world, freshness_ns=10**12)):
                trace = composition().run_tick(replace(first, world=invalid_world))
                assert trace.navigation.status is NavigationStatus.INVALIDATED
                assert trace.motion.requested_v_mps == trace.motion.requested_omega_rad_s == 0


def test_navigate_motion_async_terminal_heading_discards_late_rollout_and_restores():
    from v3.contracts.planner import PlannerInput
    from v3.layers.l6_navigation import TrajectoryNavigator

    config = resolved_config().runtime.composition.live_control.control
    manager = MissionManager(config.mission)
    navigation = TrajectoryNavigator(config.navigation, async_config=config.async_l6)
    context = TickContext(0, 1_000_000_000)
    command = CommandRequest(context, "terminal-yaw", CommandMode.NAVIGATE,
                             (DataField("x_m", 0.4), DataField("y_m", 0.0), DataField("yaw_rad", 1.0)), 0)
    mission = manager.evaluate(command)
    estimate = RobotEstimate(context, "odom", 0, 0, 0, 0, 0, (0.0,) * 25)
    world = WorldSnapshot(context, "odom", 1, (), 0, RollingLocalCostmap("odom", 1, 0.05, 2.5, (), 1, 0))
    assert navigation.evaluate(mission, estimate, world).status is NavigationStatus.PENDING
    pending = navigation.pending_rollout_request
    checkpoint = navigation.checkpoint()
    assert pending is not None
    context = TickContext(1, 1_020_000_000)
    mission = replace(mission, context=context)
    estimate = replace(estimate, context=context, x_m=0.4)
    world = replace(world, context=context)
    late = PlannerInput(context, pending.context, error="ASYNC_L6_DEADLINE_MISSED")
    aligned = navigation.evaluate(mission, estimate, world, late)
    assert aligned.status is NavigationStatus.ACTIVE and aligned.route == (mission.target_pose,)
    assert not aligned.trajectory_candidates and navigation.pending_rollout_request is None
    restored = TrajectoryNavigator(config.navigation, async_config=config.async_l6)
    restored.restore(checkpoint)
    assert restored.evaluate(mission, estimate, world, late) == aligned
    # STOP removes the heading goal even if the old worker completion is still visible.
    context = TickContext(2, 1_040_000_000)
    stopped = manager.evaluate(CommandRequest(context, "stop", CommandMode.STOP, (), 2))
    plan = navigation.evaluate(stopped, replace(estimate, context=context), replace(world, context=context), replace(late, context=context))
    assert plan.status is NavigationStatus.IDLE and not plan.route and not plan.trajectory_candidates
