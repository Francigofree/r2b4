from __future__ import annotations

import json
from pathlib import Path
import time

import pytest

from v3.config import ConfigResolver


ROOT = next(path for path in Path(__file__).resolve().parents if (path / "conf" / "hardver.json").is_file())
CONFIG_NAMES = ("hardver.json", "fizika.json", "speed_map.json", "vezerles.json")


def _documents():
    return [json.loads((ROOT / "conf" / name).read_text()) for name in CONFIG_NAMES]


def test_follow_profile_is_explicit_and_diagnoses_unavailable_pivot():
    documents = _documents()
    resolved = ConfigResolver.from_documents(*documents)
    diagnostics = resolved.configuration_diagnostics()
    control = resolved.runtime.composition.live_control.control
    effective_omega = min(resolved.followperson.max_omega_rad_s,
                          control.mission.default_constraints.max_omega_rad_s,
                          control.operational_constraints.max_omega_rad_s,
                          control.motion_realization.max_requested_omega_rad_s)
    assert diagnostics["followperson"]["centered_spin_available"] == (
        effective_omega >= control.navigation.wheel_limits.minimum_center_spin_rad_s)
    if not diagnostics["followperson"]["centered_spin_available"]:
        assert any(knob["path"] == "behavior.followperson.max_omega_rad_s"
                   and knob["state"] == "PARTIAL_CAPABILITY" for knob in diagnostics["knobs"])
    documents[-1]["behavior"]["followperson"]["max_v_mps"] = (
        documents[-1]["runtime"]["command_ingress"]["maximum_linear_speed_mps"] + .1)
    with pytest.raises(ValueError, match="FollowPerson envelope"):
        ConfigResolver.from_documents(*documents)
    del documents[-1]["behavior"]["followperson"]["max_v_mps"]
    with pytest.raises(ValueError, match="max_v_mps"):
        ConfigResolver.from_documents(*documents)


def test_follow_profile_crosses_interface_cli_gateway_and_mission(tmp_path, monkeypatch):
    from v3 import control_cli, interface_cli
    from v3.adapters.resident_command import AtomicResidentCommandGateway, ResidentCommandMailboxConfig
    from v3.adapters.v3_control import V3ControlInterfaceAdapter
    from v3.contracts import CommandMode, TickContext
    from v3.layers.l5_command_mission import MissionManager
    from v3.operator_controller import OperatorController, OperatorError
    from v3.robot_interface import RobotInterface

    documents = _documents()
    documents[-1]["behavior"]["followperson"] = {"max_v_mps": .31, "max_omega_rad_s": 1.1}
    ingress = documents[-1]["runtime"]["command_ingress"]
    ingress["maximum_linear_speed_mps"] = .33
    ingress["maximum_angular_speed_rad_s"] = 1.12
    documents[-1]["behavior"]["roomcruise"].update(max_v_mps=.32, max_omega_rad_s=1.1)
    conf = tmp_path / "conf"
    conf.mkdir()
    (tmp_path / "runtime").mkdir()
    for name, document in zip(CONFIG_NAMES, documents):
        (conf / name).write_text(json.dumps(document))
    resolved = ConfigResolver.for_project(tmp_path).resolve()
    controller = OperatorController(tmp_path)
    commands = []
    monkeypatch.setattr(controller, "_start_motion", lambda *args, **kwargs: commands.append(args[3]) or (123, "nincs"))
    monkeypatch.setattr(controller, "status", lambda: {"runtime_running": False})
    monkeypatch.setattr(controller, "live_runtime_status", lambda: None)
    interface = RobotInterface(tmp_path, controller=controller, adapters=[V3ControlInterfaceAdapter(controller)])
    action, parameters = interface_cli._motion_request(interface_cli._parser().parse_args(["followperson", "1"]))
    assert parameters == {}
    interface.execute(action, capture=False, **parameters)
    monkeypatch.setattr(control_cli, "PROJECT_ROOT", tmp_path)
    monkeypatch.setattr(control_cli, "_active_preflight", lambda path: None)
    monkeypatch.setattr(control_cli, "_run_active", lambda client, publish, **kwargs: publish(kwargs["command_id"]) and 0)
    assert control_cli.main(commands[-1][3:]) == 0
    mailbox = ResidentCommandMailboxConfig.from_policy(tmp_path / "runtime" / "v3_command.json", resolved.edges.command_ingress)
    gateway = AtomicResidentCommandGateway(mailbox)
    manager = MissionManager(resolved.runtime.composition.live_control.control.mission)

    def latest(tick):
        command = gateway.snapshot(TickContext(tick, time.monotonic_ns()))
        assert command.mode is CommandMode.FOLLOW_PERSON
        return manager.evaluate(command)

    mission = latest(0)
    assert mission.constraints.max_v_mps == resolved.followperson.max_v_mps
    assert mission.constraints.max_omega_rad_s == resolved.followperson.max_omega_rad_s
    assert control_cli.main(["followperson", "--command-id", "cli-default"]) == 0
    assert latest(1).constraints == mission.constraints
    interface.execute(action, max_v_mps=.28, max_omega_rad_s=1.05, capture=False)
    assert control_cli.main(commands[-1][3:]) == 0
    assert latest(2).constraints.max_v_mps == .28
    assert latest(2).constraints.max_omega_rad_s == 1.05
    with pytest.raises(OperatorError, match="ingress acceptance"):
        controller.followperson(max_omega_rad_s=1.13, capture=False)
    assert control_cli.main(["followperson", "--max-omega-rad-s", "1.13"]) == 1


def test_follow_profile_captured_snapshot_and_historical_snapshot_decode():
    from v3.capture_encoding import encode_value
    from v3.config import ResolvedRobotConfig
    from v3.replay import _decode_production_value

    resolved = ConfigResolver.from_documents(*_documents())
    encoded = encode_value(resolved)
    assert _decode_production_value(encoded, ResolvedRobotConfig, "config") == resolved
    del encoded["followperson"]
    historical = _decode_production_value(encoded, ResolvedRobotConfig, "config")
    assert historical.followperson is None
    assert historical.runtime == resolved.runtime
