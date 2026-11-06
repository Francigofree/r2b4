"""Optional launcher evidence; these tests never start a skill worker."""

from __future__ import annotations

import json
import os

import pytest

from v3 import interface_cli


def _library(tmp_path, name="survey"):
    library = tmp_path / "robot_skills"
    library.mkdir(exist_ok=True)
    path = library / f"{name}.py"
    path.write_text(
        "raise RuntimeError('discovery must not execute skill code')\n"
        "async def run(robot, **parameters):\n    return parameters\n",
        encoding="utf-8",
    )
    return path


@pytest.mark.parametrize("target_kind", ["name", "filename", "relative", "absolute"])
def test_skill_name_and_path_use_canonical_run(monkeypatch, tmp_path, capsys, target_kind):
    path = _library(tmp_path)
    target = {
        "name": path.stem,
        "filename": path.name,
        "relative": str(path.relative_to(tmp_path)),
        "absolute": str(path),
    }[target_kind]
    calls = []
    started = {"run_id": "skill-run", "state": "STARTING", "source_hash": "saved-source"}

    class Interface:
        def __init__(self, **kwargs):
            assert kwargs["project_root"] == tmp_path

        def execute(self, action, **parameters):
            calls.append((action, parameters))
            return started

    monkeypatch.setattr(interface_cli, "RobotInterface", Interface)
    assert interface_cli.main(
        ["skill", target, "--parameters", '{"seconds":2,"label":"nc"}', "--json"],
        project_root=tmp_path,
    ) == 0
    assert calls == [("skill.run", {"name": "survey", "parameters": {"seconds": 2, "label": "nc"}})]
    assert json.loads(capsys.readouterr().out) == started


def test_skill_without_target_uses_public_library_read(monkeypatch, tmp_path, capsys):
    calls = []

    class Interface:
        def __init__(self, **kwargs):
            pass

        def read(self, resource):
            calls.append(resource)
            return [{"name": "survey"}]

    monkeypatch.setattr(interface_cli, "RobotInterface", Interface)
    assert interface_cli.main(["skill", "--json"], project_root=tmp_path) == 0
    assert calls == ["skill.list"]
    assert json.loads(capsys.readouterr().out) == [{"name": "survey"}]


@pytest.mark.parametrize("name", ["c", "nc"])
def test_skill_names_are_not_capture_shorthand(monkeypatch, tmp_path, capsys, name):
    _library(tmp_path, name)
    calls = []

    class Interface:
        def __init__(self, **kwargs):
            pass

        def execute(self, action, **parameters):
            calls.append((action, parameters))
            return {"run_id": "skill-run"}

    monkeypatch.setattr(interface_cli, "RobotInterface", Interface)
    assert interface_cli.main(["--json", "skill", name], project_root=tmp_path) == 0
    assert calls == [("skill.run", {"name": name, "parameters": {}})]
    capsys.readouterr()


@pytest.mark.parametrize("case", ["external", "entry_symlink", "parent_escape", "fifo", "missing", "reserved", "non_object", "malformed"])
def test_invalid_skill_inputs_fail_before_interface_creation(monkeypatch, tmp_path, capsys, case):
    path = _library(tmp_path)
    external = tmp_path / "external"
    external.mkdir()
    outside = external / "survey.py"
    outside.write_text(path.read_text(encoding="utf-8"), encoding="utf-8")
    arguments = ["skill", str(path), "--json"]
    if case == "external":
        arguments[1] = str(outside)
    elif case == "entry_symlink":
        linked = path.with_name("linked.py")
        linked.symlink_to(path)
        arguments[1] = str(linked)
    elif case == "parent_escape":
        linked = path.parent / "escape"
        linked.symlink_to(external, target_is_directory=True)
        arguments[1] = str(linked / outside.name)
    elif case == "fifo":
        fifo = path.with_name("blocked.py")
        os.mkfifo(fifo)
        arguments[1] = str(fifo)
    elif case == "missing":
        arguments[1] = "missing"
    elif case == "reserved":
        arguments[1] = "run"
    elif case == "non_object":
        arguments.extend(["--parameters", "[]"])
    else:
        arguments.extend(["--parameters", "{broken"])
    monkeypatch.setattr(
        interface_cli, "RobotInterface",
        lambda **kwargs: pytest.fail("invalid input created the robot interface"),
    )
    assert interface_cli.main(arguments, project_root=tmp_path) == 1
    output = capsys.readouterr()
    assert output.out == ""
    assert "ERROR:" in output.err


def test_skill_help_never_creates_interface(monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(
        interface_cli, "RobotInterface", lambda **kwargs: pytest.fail("help created the robot interface"),
    )
    with pytest.raises(SystemExit) as result:
        interface_cli.main(["skill", "--help"], project_root=tmp_path)
    assert result.value.code == 0
    output = capsys.readouterr().out
    assert "skill.status" in output and "skill.stop" in output and "skill.test" in output
