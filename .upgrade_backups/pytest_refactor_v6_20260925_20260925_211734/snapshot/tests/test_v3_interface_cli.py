from __future__ import annotations

import pytest

from v3.interface_cli import _canonical, _extract_capture_selector, _motion_request, _parser


def test_short_aliases_and_seconds_first_syntax():
    args = _parser().parse_args(["rc", "35"])
    assert _canonical(args.command) == "roomcruise"
    assert args.seconds == 35.0

    args = _parser().parse_args(["fp", "20", "--max-v", "0.12"])
    assert _canonical(args.command) == "followperson"
    action, parameters = _motion_request(args)
    assert action == "v3.command.follow_person"
    assert parameters["max_v_mps"] == 0.12

    args = _parser().parse_args(["m", "8", "0.10", "0.20"])
    assert _canonical(args.command) == "wheels"
    action, parameters = _motion_request(args)
    assert action == "v3.command.wheels"
    assert parameters == {"left_mps": 0.10, "right_mps": 0.20}


def test_capture_selector_can_be_written_anywhere_without_stealing_command_args():
    clean, mode, hz, no_trigger = _extract_capture_selector(["rc", "35", "c", "full", "nc"])
    assert clean == ["rc", "35"]
    assert mode == "full"
    assert hz == 10
    assert no_trigger is True

    clean, mode, hz, no_trigger = _extract_capture_selector(["--capture=nincs", "f", "4", "0.15"])
    assert clean == ["f", "4", "0.15"]
    assert mode == "nincs"
    assert hz == 10
    assert no_trigger is False


def test_invalid_capture_selector_fails_closed():
    with pytest.raises(ValueError):
        _extract_capture_selector(["rc", "10", "c", "maybe"])
