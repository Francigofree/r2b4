"""Focused acceptance for the non-actuating RoomCruise tuner."""
from __future__ import annotations

import json
import math

from tools.tuners import r2b4_roomcruise_tuner as tuner


def test_roomcruise_tuner_smoke_uses_resolved_headless_l5_l9(tmp_path, monkeypatch):
    resolved = tuner._load_resolved(tuner.PROJECT_ROOT)
    baseline = tuner._baseline_candidate(resolved)
    assert baseline.command_max_v_mps == resolved.roomcruise.max_v_mps
    assert baseline.command_max_omega_rad_s == resolved.roomcruise.max_omega_rad_s
    result = tuner.simulate_candidate(
        resolved,
        baseline,
        tuner.scenarios()[0],
        ticks=25,
    )
    assert result.scenario == "open_room"
    assert result.ticks > 0
    assert math.isfinite(result.score)
    assert 0.0 <= result.moving_ratio <= 1.0
    assert 0.0 <= result.curved_ratio <= 1.0
    assert result.minimum_clearance_m > 0.0
    from dataclasses import replace
    import pytest
    with pytest.raises(ValueError, match='command ingress'):
        tuner.simulate_candidate(resolved, replace(baseline,
            command_max_v_mps=resolved.edges.command_ingress.maximum_linear_speed_mps + .01),
            tuner.scenarios()[0], ticks=1)
    changed = replace(baseline, command_max_v_mps=baseline.command_max_v_mps * .9,
                      local_goal_forward_weight=baseline.local_goal_forward_weight + .01)
    patch = tuner._config_patch(baseline, changed)
    assert patch['behavior']['roomcruise']['max_v_mps'] == changed.command_max_v_mps
    assert patch['behavior']['roomcruise']['preferences']['local_goal_forward_weight'] == changed.local_goal_forward_weight
    assert 'layers' not in patch
    output = tmp_path / 'tuned.json'
    tuner.emit_config(tuner.PROJECT_ROOT, patch, output)
    from v3.config import ConfigResolver
    conf = tuner.PROJECT_ROOT / 'conf'
    tuned = ConfigResolver(conf / 'hardver.json', conf / 'fizika.json', conf / 'speed_map.json', output).resolve()
    assert tuned.roomcruise.max_v_mps == changed.command_max_v_mps
    assert tuned.roomcruise.preferences.local_goal_forward_weight == changed.local_goal_forward_weight
    assert tuned.runtime.tick_period_ns == resolved.runtime.tick_period_ns
    # A slower configured tick must not reactivate the legacy tick-count gate
    # ignored by production completion-input mode. Observe actual replan times.
    documents = [json.loads((conf / name).read_text()) for name in
                 ('hardver.json', 'fizika.json', 'speed_map.json', 'vezerles.json')]
    documents[-1]['runtime']['tick_period_ns'] = 40_000_000
    slower = ConfigResolver.from_documents(*documents)
    from v3.layers.l6_navigation import TrajectoryNavigator
    replans = []
    store = TrajectoryNavigator._store_trajectory_plan

    def record_store(self, now_ns, *args, **kwargs):
        replans.append(now_ns)
        return store(self, now_ns, *args, **kwargs)

    monkeypatch.setattr(TrajectoryNavigator, '_store_trajectory_plan', record_store)
    tuner.simulate_candidate(slower, tuner._baseline_candidate(slower), tuner.scenarios()[0], ticks=10)
    interval = slower.runtime.composition.live_control.control.navigation.trajectory_replan_interval_ns
    expected_gap = math.ceil(interval / slower.runtime.tick_period_ns) * slower.runtime.tick_period_ns
    assert len(replans) >= 3
    assert all(b - a == expected_gap for a, b in zip(replans, replans[1:]))
