import json
from pathlib import Path

import pytest

from v3 import operator_cli
from v3.operator_controller import OperatorController, OperatorError


def _controller(tmp_path: Path) -> OperatorController:
    (tmp_path / "runtime" / "captures").mkdir(parents=True)
    (tmp_path / "conf").mkdir()
    (tmp_path / "conf" / "fizika.json").write_text(
        json.dumps({"nyomtav_szelesseg_m": 0.3557}),
        encoding="utf-8",
    )
    return OperatorController(tmp_path)


def test_wheel_targets_to_twist_uses_canonical_track_width(tmp_path):
    c = _controller(tmp_path)
    v, omega, max_v, max_omega, track = c.wheel_targets_to_twist(0.10, 0.20)
    assert track == pytest.approx(0.3557)
    assert v == pytest.approx(0.15)
    assert omega == pytest.approx((0.20 - 0.10) / 0.3557)
    assert max_v == pytest.approx(0.15)
    assert max_omega <= 1.20


def test_wheel_targets_reject_zero_or_out_of_bounds(tmp_path):
    c = _controller(tmp_path)
    with pytest.raises(OperatorError):
        c.wheel_targets_to_twist(0.0, 0.0)
    with pytest.raises(OperatorError):
        c.wheel_targets_to_twist(0.51, 0.0)


def test_capture_selector_accepts_old_and_new_spelling():
    clean, mode, explicit = operator_cli._extract_capture_selector(
        ["forward", "0.15", "c", "full"]
    )
    assert clean == ["forward", "0.15"]
    assert mode == "full"
    assert explicit is True

    clean, mode, explicit = operator_cli._extract_capture_selector(
        ["roomcruise", "--capture=nincs"]
    )
    assert clean == ["roomcruise"]
    assert mode == "nincs"
    assert explicit is True


def test_legacy_launcher_forms_normalize_to_short_commands():
    assert operator_cli._normalize_legacy(["forward", "start", "0.15"]) == [
        "forward",
        "0.15",
    ]
    assert operator_cli._normalize_legacy(["roomcruise", "start", "nocapture"]) == [
        "roomcruise",
        "--no-trigger",
    ]


def test_operator_api_is_shell_independent(tmp_path):
    c = _controller(tmp_path)
    assert c.root == tmp_path.resolve()
    assert c.status()["runtime_running"] is False
