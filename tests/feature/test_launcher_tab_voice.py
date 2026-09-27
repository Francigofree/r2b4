from __future__ import annotations

import json
from pathlib import Path
import subprocess

from v3 import interface_cli, launcher_cli, launcher_extras


def test_tab_completion_is_parser_backed_and_operation_free(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError("TAB completion must never construct RobotInterface")

    monkeypatch.setattr(interface_cli, "RobotInterface", forbidden)
    root = launcher_cli.project_root()

    hint, top = launcher_extras.completion(1, [""], root)
    assert "r commands" in hint
    assert {"status", "runtime", "er2", "voice", "help"} <= set(top)

    hint, runtime = launcher_extras.completion(2, ["runtime", ""], root)
    assert "usage:" in hint.lower()
    assert {"start", "stop", "status", "diag"} <= set(runtime)

    _, er2 = launcher_extras.completion(2, ["er2", ""], root)
    assert er2 == ["preview", "status", "stream"]

    _, voice = launcher_extras.completion(2, ["voice", ""], root)
    assert set(voice) == {"status", "on", "off", "restart", "check"}


def test_private_completion_transport_never_becomes_user_command(capsys):
    assert launcher_cli.main(["__complete", "2", "voice", ""]) == 0
    output = capsys.readouterr().out
    assert "HINT\t" in output
    assert "CAND\ton" in output
    assert "__complete" not in json.dumps(launcher_cli.command_catalog())


def test_voice_on_uses_existing_user_systemd_service(tmp_path, monkeypatch):
    root = launcher_cli.project_root()
    calls: list[tuple[str, ...]] = []

    monkeypatch.setattr(Path, "home", classmethod(lambda cls: tmp_path))

    def fake_systemctl(argv, *, capture=False):
        calls.append(tuple(argv))
        return subprocess.CompletedProcess(["systemctl", "--user", *argv], 0, stdout="", stderr="")

    monkeypatch.setattr(launcher_extras, "_systemctl", fake_systemctl)
    assert launcher_extras.voice_command(["on"], root) == 0
    assert ("daemon-reload",) in calls
    assert ("enable", "--now", "r2b4-wake.service") in calls
    unit = tmp_path / ".config" / "systemd" / "user" / "r2b4-wake.service"
    assert unit.is_symlink()
    assert unit.resolve() == (root / "deploy" / "systemd" / "r2b4-wake.service").resolve()


def test_voice_off_does_not_touch_robot_interface(monkeypatch):
    calls = []

    def forbidden(*args, **kwargs):
        raise AssertionError("voice service lifecycle must not use RobotInterface")

    monkeypatch.setattr(interface_cli, "RobotInterface", forbidden)
    monkeypatch.setattr(
        launcher_extras,
        "_systemctl",
        lambda argv, capture=False: calls.append(tuple(argv))
        or subprocess.CompletedProcess(["systemctl"], 0, stdout="", stderr=""),
    )
    assert launcher_extras.voice_command(["off"], launcher_cli.project_root()) == 0
    assert calls == [("disable", "--now", "r2b4-wake.service")]


def test_voice_status_json_is_stable_when_service_is_off(monkeypatch, capsys):
    answers = {
        ("show", "r2b4-wake.service", "--property=LoadState", "--value"): "loaded\n",
        ("is-enabled", "r2b4-wake.service"): "disabled\n",
        ("is-active", "r2b4-wake.service"): "inactive\n",
    }

    def fake_systemctl(argv, *, capture=False):
        return subprocess.CompletedProcess(
            ["systemctl", "--user", *argv], 0, stdout=answers[tuple(argv)], stderr=""
        )

    monkeypatch.setattr(launcher_extras, "_systemctl", fake_systemctl)
    assert launcher_extras.voice_status(launcher_cli.project_root(), as_json=True) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["enabled"] == "disabled"
    assert payload["active"] == "inactive"
