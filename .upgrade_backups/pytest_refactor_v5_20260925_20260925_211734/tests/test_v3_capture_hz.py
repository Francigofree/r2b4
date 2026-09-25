from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest

from v3.capture_rate import CAPTURE_HZ_CHOICES, DEFAULT_CAPTURE_HZ, validate_capture_hz
from v3.interface_cli import _extract_capture_selector
from v3.mcap_capture import McapCaptureConfig
from v3.test_hub_runtime import postprocess_capture
import v3_process_runtime


def test_capture_rate_policy_is_exact_and_default_is_10_hz():
    assert DEFAULT_CAPTURE_HZ == 10
    assert CAPTURE_HZ_CHOICES == {1, 5, 10, 50}
    for value in (1, 5, 10, 50, "1", "5", "10", "50"):
        assert validate_capture_hz(value) in CAPTURE_HZ_CHOICES
    for value in (0, 2, 20, 100, True, 10.5, "2", "10.0"):
        with pytest.raises(ValueError):
            validate_capture_hz(value)


def test_r_launcher_default_is_10_hz():
    clean, mode, hz, no_trigger = _extract_capture_selector(["rc"])
    assert clean == ["rc"]
    assert mode == "alap"
    assert hz == 10
    assert no_trigger is False


def test_r_launcher_accepts_requested_capture_rates():
    for hz in (50, 10, 5, 1):
        clean, mode, selected, no_trigger = _extract_capture_selector(
            ["rc", "30", "c", str(hz)]
        )
        assert clean == ["rc", "30"]
        assert mode == "alap"
        assert selected == hz
        assert no_trigger is False


def test_r_launcher_rejects_unsupported_capture_rate():
    with pytest.raises(ValueError):
        _extract_capture_selector(["rc", "30", "c", "2"])


def test_legacy_capture_mode_selector_is_preserved():
    for mode in ("alap", "full", "nincs"):
        clean, selected_mode, hz, no_trigger = _extract_capture_selector(
            ["rc", "c", mode]
        )
        assert clean == ["rc"]
        assert selected_mode == mode
        assert hz == 10
        assert no_trigger is False


def test_capture_config_marks_sampling_rate():
    for hz in (1, 5, 10, 50):
        assert McapCaptureConfig(tick_sample_hz=hz).tick_sample_hz == hz
    with pytest.raises(ValueError):
        McapCaptureConfig(tick_sample_hz=2)


def test_process_runtime_cli_defaults_to_10_hz_and_accepts_50():
    parser = v3_process_runtime._parser()
    default = parser.parse_args(["--approval", "native-resident-v3"])
    assert default.capture_hz == 10
    explicit = parser.parse_args(
        ["--approval", "native-resident-v3", "--capture-hz", "50"]
    )
    assert explicit.capture_hz == 50


def test_testhub_handoff_can_disable_replay_for_sampled_capture(tmp_path, monkeypatch):
    capture = tmp_path / "run.mcap"
    capture.write_bytes(b"placeholder")
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        return SimpleNamespace(
            returncode=0,
            stdout='{"status":"PASS"}',
            stderr="",
        )

    monkeypatch.setattr("v3.test_hub_runtime.subprocess.run", fake_run)
    result = postprocess_capture(capture, project_root=tmp_path, replay_mode="off")
    assert result["status"] == "PASS"
    command = calls[0][0]
    replay_index = command.index("--replay")
    assert command[replay_index + 1] == "off"


def test_runtime_source_contains_pre_ipc_rate_gate():
    root = Path(__file__).resolve().parents[1]
    source = (root / "v3_runtime.py").read_text(encoding="utf-8")
    assert "record_observer_hz" in source
    assert "scheduled_capture_record" in source
    assert "force_capture_record" in source
    # The rate gate must execute before record_observer(record), so sampling
    # reduces capture IPC load rather than only filtering after MCAP encoding.
    assert source.index("scheduled_capture_record") < source.index("record_observer(record)", source.index("scheduled_capture_record"))
