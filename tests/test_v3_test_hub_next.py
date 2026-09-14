
from v3.test_hub_views import (
    _suppress_agent_noise,
    _transition_events,
    compare_views,
)


def test_teleop_navigation_stagnation_is_agent_noise_only():
    incidents = [{
        "id": "navigation-stagnation",
        "reason": "NAVIGATION_PROGRESS_STAGNATION",
        "category": "NAVIGATION",
        "tick_id": 99,
    }]
    rows = [{"mission": {"mode": "TELEOP"}} for _ in range(5)]
    effective, suppressed = _suppress_agent_noise(incidents, rows)
    assert effective == []
    assert len(suppressed) == 1
    assert "TELEOP" in suppressed[0]["suppressed_reason"]


def test_explore_stagnation_is_not_suppressed():
    incidents = [{"reason": "NAVIGATION_PROGRESS_STAGNATION"}]
    effective, suppressed = _suppress_agent_noise(
        incidents, [{"mission": {"mode": "EXPLORE"}}]
    )
    assert effective == incidents
    assert suppressed == []


def test_short_safety_transition_is_preserved_as_event():
    before = {
        "tick_id": 10,
        "monotonic_ns": 100,
        "mission": {"mode": "TELEOP", "lifecycle": "ACTIVE", "stop_reason": None},
        "safety_decision": "ALLOW",
        "l12_reason": None,
        "l9_constraints": [],
    }
    after = {
        **before,
        "tick_id": 11,
        "monotonic_ns": 120,
        "safety_decision": "STOP",
        "l12_reason": "NOT_ACTIVE",
    }
    events = _transition_events(before, after)
    assert any(item["signal"] == "safety_decision" and item["to"] == "STOP" for item in events)
    assert any(item["signal"] == "l12_reason" and item["to"] == "NOT_ACTIVE" for item in events)


def test_compare_is_objective_delta_not_verdict():
    before = {
        "ticks": {"duration_s": 10.0},
        "phases": [],
        "effective_incidents": [{"id": "x"}],
        "windows": [{
            "wheel_control": {
                "left_error_mps": {"mean": 0.02},
                "right_error_mps": {"mean": 0.03},
            },
            "safety": {"ALLOW": 10, "STOP": 1, "FAULT": 1},
        }],
    }
    after = {
        "ticks": {"duration_s": 10.0},
        "phases": [],
        "effective_incidents": [],
        "windows": [{
            "wheel_control": {
                "left_error_mps": {"mean": 0.01},
                "right_error_mps": {"mean": 0.01},
            },
            "safety": {"ALLOW": 11, "STOP": 0, "FAULT": 0},
        }],
    }
    result = compare_views(before, after)
    assert result["verdict_policy"].startswith("No automatic")
    assert result["delta_after_minus_before"]["incident_count"] == -1.0
    assert result["delta_after_minus_before"]["safety_fault"] == -1
    assert result["delta_after_minus_before"]["mean_abs_right_wheel_error_mps"] < 0


import json
from pathlib import Path

import pytest

from test_v3_mcap_e2e import capture
from v3.test_hub import main
from v3.test_hub_next import run_default
from v3.test_hub_v2 import TestHubV2Error as HubError, verify_evidence


@pytest.mark.parametrize(
    ('options', 'exit_code', 'replay_status'),
    [({}, 0, 'MATCH'), ({'capacity': 4}, 2, 'ERROR'),
     ({'configuration': {}}, 2, 'ERROR')],
)
def test_integrated_entrypoint_native_replay(tmp_path, capsys, options, exit_code, replay_status):
    captured, _ = capture(tmp_path, **options)
    destination = tmp_path / 'next'
    assert main(['run', str(captured.path), '--output-dir', str(destination),
                 '--replay', 'full']) == exit_code
    result = json.loads(capsys.readouterr().out)
    assert result['replay_status'] == replay_status
    assert result['evidence_status'] == ('PASS' if exit_code == 0 else 'FAIL')
    assert verify_evidence(destination / 'evidence_index.json')['status'] == 'PASS'
    agent = json.loads(Path(result['agent_view']).read_text())
    assert agent['replay_status'] == replay_status
    rows = [json.loads(line) for line in Path(result['overview']).read_text().splitlines()]
    assert rows[0]['authority']['derived_only'] is True
    times = [row.get('t_start_s', row.get('t_s', 0)) for row in rows[1:]]
    assert times == sorted(times)
    assert any(row['row_type'] == 'window' for row in rows)
    if exit_code == 0:
        assert any(row.get('signal') == 'safety_decision' for row in rows)
        assert rows[0]['data_coverage']['/r2b4/tick']['captured'] == 12


def test_existing_evidence_is_preserved_and_not_reused_for_new_replay_scope(tmp_path):
    captured, _ = capture(tmp_path)
    destination = tmp_path / 'next'
    run_default(captured.path, output_dir=destination, replay_mode='off')
    original = {p.name: p.read_bytes() for p in destination.iterdir()}
    with pytest.raises(HubError, match='absent or empty'):
        run_default(captured.path, output_dir=destination, replay_mode='full')
    assert {p.name: p.read_bytes() for p in destination.iterdir()} == original


def test_no_arguments_selects_latest_capture(tmp_path, monkeypatch, capsys):
    captures = tmp_path / 'runtime' / 'captures'
    captures.mkdir(parents=True)
    captured, _ = capture(captures)
    monkeypatch.chdir(tmp_path)
    assert main([]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result['capture'] == str(captured.path.resolve())
    assert Path(result['agent_view']).is_file()


@pytest.mark.parametrize('hz', [1, 5, 10])
def test_view_and_compare_cli_preserve_existing_outputs(tmp_path, capsys, hz):
    captured, _ = capture(tmp_path)
    overview = tmp_path / 'overview.ndjson'
    args = ['view', str(captured.path), '--hz', str(hz), '--output', str(overview)]
    assert main(args) == 0
    assert json.loads(capsys.readouterr().out)['hz'] == hz
    original = overview.read_bytes()
    assert main(args) == 2
    capsys.readouterr()
    assert overview.read_bytes() == original
    assert main(['compare', str(captured.path), str(captured.path), '--hz', str(hz)]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result['verdict_policy'].startswith('No automatic')
    assert all(value in (0, None) for value in result['delta_after_minus_before'].values())
