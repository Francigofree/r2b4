from __future__ import annotations

import json
from pathlib import Path

import pytest

from v3.config import ConfigResolver

ROOT = next(
    (
        path
        for path in Path(__file__).resolve().parents
        if (path / "conf" / "hardver.json").is_file() and (path / "v3").is_dir()
    ),
    Path.cwd(),
)


def _documents():
    conf = ROOT / "conf"
    documents = [
        json.loads((conf / name).read_text(encoding="utf-8"))
        for name in ("hardver.json", "fizika.json", "speed_map.json", "vezerles.json")
    ]
    return documents


def test_production_route_tuning_is_explicit_config_authority():
    documents = _documents()
    navigation = documents[-1]['behavior']['roomcruise']['preferences']
    key = 'explore_local_goal_max_distance_m'
    navigation[key] += .1
    resolved = ConfigResolver.from_documents(*documents)
    nav = resolved.runtime.composition.live_control.control.navigation
    assert nav.explore_local_goal_max_distance_m == navigation[key]
    # Missing authority must fail closed instead of restoring a test's tuning.
    del navigation[key]
    with pytest.raises(ValueError, match=key):
        ConfigResolver.from_documents(*documents)


def test_roomcruise_profile_crosses_canonical_interface_cli_gateway_and_mission(tmp_path, monkeypatch, capsys):
    import time
    from dataclasses import replace
    from v3 import control_cli
    from v3.adapters.resident_command import AtomicResidentCommandGateway, ResidentCommandMailboxConfig
    from v3.adapters.v3_control import V3ControlInterfaceAdapter
    from v3.contracts import CommandMode, CommandRequest, DataField, TickContext
    from v3.layers.l5_command_mission import MissionManager
    from v3.operator_controller import OperatorController
    from v3.robot_interface import RobotInterface

    documents = _documents()
    room = documents[-1]['behavior']['roomcruise']
    room['max_v_mps'] = .32
    room['max_omega_rad_s'] = .95
    room['preferences']['local_goal_forward_weight'] = .12
    ingress = documents[-1]['runtime']['command_ingress']
    ingress['maximum_linear_speed_mps'] = .33
    ingress['maximum_angular_speed_rad_s'] = .96
    conf = tmp_path / 'conf'
    conf.mkdir()
    (tmp_path / 'runtime').mkdir()
    for name, value in zip(('hardver.json', 'fizika.json', 'speed_map.json', 'vezerles.json'), documents):
        (conf / name).write_text(json.dumps(value))
    resolved = ConfigResolver.for_project(tmp_path).resolve()
    controller = OperatorController(tmp_path)
    commands = []
    monkeypatch.setattr(controller, '_start_motion', lambda *args, **kwargs: commands.append(args[3]) or (123, 'nincs'))
    monkeypatch.setattr(controller, 'status', lambda: {'runtime_running': False})
    monkeypatch.setattr(controller, 'live_runtime_status', lambda: None)
    interface = RobotInterface(tmp_path, controller=controller, adapters=[V3ControlInterfaceAdapter(controller)])
    interface.execute('v3.command.explore', capture=False)
    monkeypatch.setattr(control_cli, 'PROJECT_ROOT', tmp_path)
    monkeypatch.setattr(control_cli, '_active_preflight', lambda path: None)
    monkeypatch.setattr(control_cli, '_run_active', lambda client, publish, **kwargs: publish(kwargs['command_id']) and 0)
    assert control_cli.main(commands[0][3:]) == 0
    mailbox = ResidentCommandMailboxConfig.from_policy(tmp_path / 'runtime' / 'v3_command.json', resolved.edges.command_ingress)
    gateway = AtomicResidentCommandGateway(mailbox)
    context = TickContext(0, time.monotonic_ns())
    command = gateway.snapshot(context)
    assert command.mode is CommandMode.EXPLORE
    manager = MissionManager(resolved.runtime.composition.live_control.control.mission)
    mission = manager.evaluate(command)
    assert mission.explore_preferences == resolved.roomcruise.preferences
    assert mission.constraints.max_v_mps == resolved.roomcruise.max_v_mps
    assert mission.constraints.max_omega_rad_s == resolved.roomcruise.max_omega_rad_s
    # Checkpoint restores the immutable profile identity, not a live default.
    restored = MissionManager(resolved.runtime.composition.live_control.control.mission)
    restored.restore(manager.checkpoint())
    changed = replace(command, goal=tuple(DataField(f.key, .13) if f.key == 'local_goal_forward_weight' else f for f in command.goal))
    assert restored.evaluate(changed).stop_reason == 'COMMAND_ID_REUSED'
    incomplete = replace(command, command_id='incomplete', goal=command.goal[:-1])
    assert manager.evaluate(incomplete).stop_reason == 'INVALID_COMMAND'
    # No profile remains valid for old captures and non-host command sources.
    historical = CommandRequest(context, 'historical', CommandMode.EXPLORE, (), 0)
    assert manager.evaluate(historical).explore_preferences is None
    diagnostics = resolved.configuration_diagnostics()
    wheels = resolved.runtime.composition.live_control.control.navigation.wheel_limits
    assert diagnostics['minimum_center_spin_rad_s'] == 2 * wheels.minimum_mps / wheels.track_width_m
    assert diagnostics['localization_recovery_omega_rad_s'] == wheels.minimum_center_spin_rad_s
    assert not any(k['state'] == 'UNREALIZABLE' for k in diagnostics['knobs'])
    # Developer CLI defaults share the same profile and acceptance limits.
    assert control_cli.main(['explore', '--command-id', 'cli-default']) == 0
    default = gateway.snapshot(TickContext(1, time.monotonic_ns()))
    assert manager.evaluate(default).explore_preferences == mission.explore_preferences
    assert control_cli.main(['explore', '--max-v-mps', '.34']) == 1
    assert 'resident process limit' in capsys.readouterr().err
    interface.execute('v3.command.explore', max_v_mps=.30, local_goal_forward_weight=.07, capture=False)
    assert control_cli.main(commands[-1][3:]) == 0
    override = gateway.snapshot(TickContext(2, time.monotonic_ns()))
    intent = manager.evaluate(override)
    assert intent.constraints.max_v_mps == .30
    assert intent.explore_preferences == replace(mission.explore_preferences, local_goal_forward_weight=.07)
    # A partial or wrong-mode profile is rejected at the wire boundary too.
    command_path = mailbox.path
    raw = json.loads(command_path.read_text())
    for change in ('partial', 'wrong-mode', 'bool'):
        malformed = dict(raw)
        malformed['revision'] += 1
        if change == 'partial':
            del malformed['local_goal_forward_weight']
        elif change == 'wrong-mode':
            malformed['mode'] = 'FOLLOW_PERSON'
        else:
            malformed['local_goal_forward_weight'] = True
        command_path.write_text(json.dumps(malformed))
        with pytest.raises(ValueError):
            AtomicResidentCommandGateway(mailbox).snapshot(TickContext(3, time.monotonic_ns()))
    # Broken host config must never prevent publication of a fail-safe STOP.
    (conf / 'vezerles.json').write_text('{}')
    assert control_cli.main(['stop']) == 0
    assert json.loads(command_path.read_text())['mode'] == 'STOP'


def test_recovery_speed_is_derived_from_calibration_and_rejects_second_authority():
    documents = _documents()
    documents[1]['nyomtav_szelesseg_m'] *= .9
    resolved = ConfigResolver.from_documents(*documents)
    nav = resolved.runtime.composition.live_control.control.navigation
    assert nav.localization_recovery_omega_rad_s == nav.wheel_limits.minimum_center_spin_rad_s
    assert nav.wheel_limits.wheels(0, nav.localization_recovery_omega_rad_s) == (-nav.wheel_limits.minimum_mps, nav.wheel_limits.minimum_mps)
    documents[-1]['layers']['navigation']['localization_recovery_omega_rad_s'] = .2
    with pytest.raises(ValueError, match='unknown.*localization_recovery_omega_rad_s'):
        ConfigResolver.from_documents(*documents)
