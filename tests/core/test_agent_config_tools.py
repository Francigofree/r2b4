from __future__ import annotations

import json
import shutil
from pathlib import Path

import pytest

from r2b4_orchestration.agent_config_tools import AgentConfigError, AgentConfigService


def test_agent_config_authority_and_canonical_patch_without_runtime(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[2]
    shutil.copytree(root / "conf", tmp_path / "conf", ignore=shutil.ignore_patterns(".wake.env"))
    service = AgentConfigService(tmp_path)
    policy = service.policy({})
    assert "hardver.json" in policy["not_writable"]
    with pytest.raises(AgentConfigError, match="CONFIG_PATH_NOT_LLM_WRITABLE"):
        service.patch({"path": "motor.some_value", "value": 1})
    value = json.loads((tmp_path / "conf" / "vezerles.json").read_text(encoding="utf-8"))
    old = value["layers"]["motion_selection"]["continuity_score_band"]
    result = service.patch({
        "path": "layers.motion_selection.continuity_score_band",
        "expected": old,
        "value": old,
    })
    assert result["status"] == "APPLIED"
    assert result["runtime_was_running"] is False
