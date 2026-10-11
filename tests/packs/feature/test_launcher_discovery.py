"""Launcher discovery follows owning parsers and local capabilities without execution."""
from __future__ import annotations

import argparse

import pytest

from v3 import host_cli, interface_cli, launcher_cli, launcher_extras, test_runner


@pytest.fixture
def offline_root(monkeypatch, tmp_path):
    from v3 import runtime_performance
    from v3.adapters.vision_media_socket import VisionClient

    (tmp_path / 'v3').mkdir()
    (tmp_path / 'tools' / 'mcap_evidence').mkdir(parents=True)
    (tmp_path / 'pytest.ini').write_text('[pytest]\n')
    monkeypatch.setattr(launcher_cli, 'project_root', lambda: tmp_path)
    monkeypatch.setattr(runtime_performance, 'apply_host_affinity', lambda *args: None)
    monkeypatch.setattr(VisionClient, 'status', lambda *args: pytest.fail('discovery opened live vision'))
    monkeypatch.setattr(VisionClient, 'connect', lambda *args: pytest.fail('discovery connected to vision'))
    return tmp_path


def _entry_tokens(entries):
    return {token for entry in entries for token in (entry['name'], *entry['aliases'])}


def test_catalog_names_and_aliases_match_top_completion_and_command_recognition(offline_root):
    catalog = launcher_cli.command_catalog(project_root=offline_root)
    entries = catalog['commands']
    assert len({entry['name'] for entry in entries}) == len(entries)
    for entry in entries:
        assert {'name', 'aliases', 'usage', 'description', 'group'} <= entry.keys()
        assert entry['usage'] and entry['description'] and entry['group']
    tokens = _entry_tokens(entries)
    assert {'route', 'chatgpt', 'voice', 'wake', 'cam', 'camera', 'diag', 'd', 'skill'} <= tokens
    assert tokens == launcher_cli._command_names()
    assert tokens == set(launcher_extras._top_level_commands())
    assert tokens == set(launcher_extras.completion(1, [''], offline_root)[1])
    assert tokens == set(launcher_extras.completion(2, ['help', ''], offline_root)[1])


def test_new_robot_parser_command_appears_in_catalog_help_and_completion(monkeypatch, offline_root, capsys):
    owning_parser = interface_cli._parser

    def extended_parser():
        parser = owning_parser()
        subparsers = next(action for action in parser._actions
                          if isinstance(action, argparse._SubParsersAction))
        probe = subparsers.add_parser('future_probe', description='Future robot capability.')
        probe.add_argument('--future-option', action='store_true')
        return parser

    monkeypatch.setattr(interface_cli, '_parser', extended_parser)
    entries = launcher_cli.command_catalog(project_root=offline_root)['commands']
    assert 'future_probe' in _entry_tokens(entries)
    assert launcher_cli.main(['help']) == 0
    assert 'future_probe' in capsys.readouterr().out
    assert 'future_probe' in launcher_extras.completion(1, ['future'], offline_root)[1]
    assert '--future-option' in launcher_extras.completion(2, ['future_probe', '--'], offline_root)[1]


def test_new_host_help_entry_appears_without_another_launcher_allowlist(monkeypatch, offline_root, capsys):
    monkeypatch.setitem(host_cli.COMMAND_HELP, 'future_host', ('[ARGS...]', 'Future host helper.'))
    entries = launcher_cli.command_catalog(project_root=offline_root)['commands']
    assert 'future_host' in _entry_tokens(entries)
    assert launcher_cli.main(['help']) == 0
    assert 'future_host' in capsys.readouterr().out
    assert 'future_host' in launcher_extras.completion(1, ['future_h'], offline_root)[1]
    assert launcher_cli.main(['help', 'future_host']) == 0
    assert 'Future host helper.' in capsys.readouterr().out


def test_public_catalog_and_read_execute_completion_discover_new_saved_skills(offline_root):
    from r2b4_orchestration.robot_runtime import SKILL_READS, SKILL_ACTIONS
    from r2b4_orchestration.skill_observation import SkillObservationAdapter
    from r2b4_orchestration.software_interface import SoftwareInterfaceAdapter

    skills = offline_root / 'robot_skills'
    skills.mkdir()
    source = skills / 'saved_probe.py'
    source.write_text('raise AssertionError("discovery imported skill code")\n\n'
                      'async def run(robot, count: int = 1):\n    return count\n')
    public = launcher_cli.command_catalog(project_root=offline_root)['public_robot']
    assert SKILL_READS <= set(public['resources'])
    assert SKILL_ACTIONS <= set(public['actions'])
    assert 'skill.saved_probe' in public['actions']
    assert SkillObservationAdapter.capability_names <= set(public['actions'])
    assert SoftwareInterfaceAdapter(offline_root).capability_names <= set(public['actions'])
    read = launcher_extras._robot_completion('read', (), '', root=offline_root)[1]
    execute = launcher_extras._robot_completion('execute', (), '', root=offline_root)[1]
    assert set(public['resources']) <= set(read)
    assert set(public['actions']) <= set(execute)
    assert 'saved_probe' in launcher_extras.completion(2, ['skill', 'saved'], offline_root)[1]
    source.unlink()
    assert 'skill.saved_probe' not in launcher_cli.command_catalog(project_root=offline_root)['public_robot']['actions']


def test_public_catalog_discovers_adapter_read_and_action_names_without_querying_state(monkeypatch, offline_root):
    from v3.interface_adapters import build_adapters

    adapters = build_adapters(None, offline_root)
    for adapter in adapters:
        monkeypatch.setattr(type(adapter), 'capabilities',
                            lambda *args: pytest.fail('discovery queried an adapter owner'))
    public = launcher_cli.command_catalog(project_root=offline_root)['public_robot']
    reads = set(launcher_extras._robot_completion('read', (), '', root=offline_root)[1])
    actions = set(launcher_extras._robot_completion('execute', (), '', root=offline_root)[1])
    for adapter in adapters:
        declared_reads = adapter.read_capability_names
        declared_actions = adapter.capability_names - declared_reads
        assert declared_reads <= set(public['resources'])
        assert declared_actions <= set(public['actions'])
        assert declared_reads <= reads
        assert declared_actions <= actions


@pytest.mark.parametrize('operation', (None, 'compile', 'verify', 'verify-evidence', 'query'))
def test_evidence_completion_uses_each_owning_operation_parser(operation, monkeypatch, offline_root):
    from tools.mcap_evidence import cli

    owning_parser = cli.parser

    def extended_parser(selected=None):
        parser = owning_parser(selected)
        parser.add_argument('--future-evidence-option')
        return parser

    monkeypatch.setattr(cli, 'parser', extended_parser)
    words = ['evi', 'INPUT', '--'] if operation is None else ['evi', operation, 'INPUT', '--']
    _, candidates = launcher_extras.completion(len(words), words, offline_root)
    expected = {option for action in extended_parser(operation)._actions
                for option in action.option_strings if option.startswith('--')}
    assert set(candidates) == expected
    if operation in ('verify', 'verify-evidence', 'query'):
        assert '--workers' not in candidates
    else:
        assert {'--workers', '--output'} <= set(candidates)


def test_test_pack_completion_tracks_current_pack_directories(monkeypatch, offline_root):
    packs = offline_root / 'tests' / 'packs'
    names = {'alpha', 'beta', 'core', 'feature', 'deep'}
    for name in (*names, '.hidden'):
        (packs / name).mkdir(parents=True)
    (packs / 'not_a_pack.py').write_text('')
    monkeypatch.setattr(test_runner, 'PACKS', packs)
    assert set(launcher_extras.completion(3, ['test', 'pack', ''], offline_root)[1]) == names
    modes = set(launcher_extras.completion(2, ['test', ''], offline_root)[1])
    assert {'gate', 'release', 'pack', 'all', 'endurance', 'quick', 'full', 'list', 'help'} <= modes
    called = []
    monkeypatch.setattr(test_runner, '_run', lambda *args, **kwargs: called.append(args) or 0)
    monkeypatch.setattr(test_runner, '_load_manifest', lambda name: ('tests/probe.py',))
    monkeypatch.setattr(test_runner, '_release_targets', lambda: ('tests/probe.py',))
    for mode in launcher_cli.command_catalog(project_root=offline_root)['tests']['modes']:
        assert test_runner.main([mode]) == 0


def test_chatgpt_catalog_and_nested_completion_follow_the_owning_parser(monkeypatch, offline_root):
    from r2b4_voice import openai_oauth

    owning_parser = openai_oauth.cli_parser

    def extended_parser():
        parser = owning_parser()
        subparsers = next(action for action in parser._actions
                          if isinstance(action, argparse._SubParsersAction))
        subparsers.choices['login'].add_argument('--future-login-option')
        probe = subparsers.add_parser('future_oauth', description='Future auth operation.')
        probe.add_argument('--future-auth-option')
        return parser

    monkeypatch.setattr(openai_oauth, 'cli_parser', extended_parser)
    assert 'future_oauth' in launcher_cli.command_catalog(project_root=offline_root)['chatgpt']['operations']
    assert 'future_oauth' in launcher_extras.completion(2, ['chatgpt', 'future'], offline_root)[1]
    assert '--future-auth-option' in launcher_extras.completion(3, ['chatgpt', 'future_oauth', '--'], offline_root)[1]
    assert '--future-login-option' in launcher_extras.completion(4, ['help', 'chatgpt', 'login', '--'], offline_root)[1]
    assert owning_parser().parse_args(['login', '--no-open']).no_open is True


def test_chatgpt_nested_help_never_invokes_authentication(monkeypatch, offline_root, capsys):
    from r2b4_voice import openai_oauth

    monkeypatch.setattr(openai_oauth, 'login', lambda *args, **kwargs: pytest.fail('help attempted OAuth login'))
    assert launcher_cli.main(['help', 'chatgpt', 'login']) == 0
    assert '--no-open' in capsys.readouterr().out


def test_no_argument_evi_receives_the_launcher_project_root(monkeypatch, offline_root):
    from tools.mcap_evidence import cli

    called = []
    monkeypatch.setattr(cli, 'main', lambda args, *, project_root: called.append((args, project_root)) or 0)
    assert launcher_cli.main(['evi']) == 0
    assert called == [([], offline_root)]


def test_unknown_help_topic_never_calls_llm(monkeypatch, offline_root):
    monkeypatch.setattr(launcher_cli, '_auto_prompt', lambda *args: pytest.fail('help invoked the LLM'))
    assert launcher_cli.main(['help', 'not_a_real_command']) == 2


@pytest.mark.parametrize('arguments', (
    ['help', 'route', 'arbitrary request', '--json'],
    ['route', 'arbitrary request', '--help'],
))
def test_route_help_with_request_tail_never_plans_or_writes_journal(arguments, monkeypatch, offline_root):
    from r2b4_orchestration.execution_mode import ExecutionModeSelector, RouteEvidenceJournal

    monkeypatch.setattr(ExecutionModeSelector, 'select', lambda *args, **kwargs: pytest.fail('help planned a route'))
    monkeypatch.setattr(RouteEvidenceJournal, 'emit', lambda *args, **kwargs: pytest.fail('help wrote a route journal'))
    assert launcher_cli.main(arguments) == 0
    assert not (offline_root / 'runtime' / 'execution_routes.ndjson').exists()


@pytest.mark.parametrize('arguments', (
    ['help', 'diag'], ['diag', '--help'], ['diag', 'help'], ['diag', 'full', '--help'],
))
def test_diag_help_uses_owning_cli_without_opening_evidence_or_hardware_guard(arguments, monkeypatch, offline_root, capsys):
    from tools.diag import cli

    monkeypatch.setattr(cli, 'open_context', lambda *args: pytest.fail('help opened evidence'))
    monkeypatch.setattr(host_cli, 'hardware_guard', lambda *args: pytest.fail('help tried to guard runtime'))
    assert launcher_cli.main(arguments) == 0
    output = capsys.readouterr().out
    assert 'Analyzers:' in output and '--no-save' in output


def test_runtime_d_help_keeps_the_canonical_interface_route(monkeypatch, offline_root):
    called = []
    monkeypatch.setattr(interface_cli, 'main', lambda args, **kwargs: called.append(args) or 0)
    assert launcher_cli.main(['help', 'd']) == 0
    assert called == [['d', '--help']]
