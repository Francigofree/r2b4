"""Live command/status files keep one private, project-specific canonical path."""
from concurrent.futures import ThreadPoolExecutor
import os
from pathlib import Path
import shutil
import stat
from types import SimpleNamespace

import pytest

from v3.live_io_paths import live_io_directory, runtime_mailbox_path
from v3.operator_controller import OperatorController
from v3.robot_interface import RobotInterface


@pytest.fixture
def live_project(tmp_path):
    root = tmp_path / "project"
    root.mkdir()
    directory = live_io_directory(root)
    try:
        yield root, directory
    finally:
        if directory.is_symlink():
            directory.unlink()
        elif directory.exists():
            shutil.rmtree(directory)


def test_concurrent_clients_share_private_paths_and_other_projects_are_isolated(tmp_path):
    roots = (tmp_path / "first", tmp_path / "second")
    for root in roots:
        root.mkdir()
    directories = set()
    try:
        with ThreadPoolExecutor(max_workers=4) as workers:
            same_project = tuple(workers.map(live_io_directory, [roots[0]] * 12))
        directories.update(same_project)
        other = live_io_directory(roots[1])
        directories.add(other)
        assert len(set(same_project)) == 1 and other != same_project[0]
        for root, directory in ((roots[0], same_project[0]), (roots[1], other)):
            info = directory.lstat()
            assert stat.S_ISDIR(info.st_mode) and info.st_uid == os.geteuid()
            assert stat.S_IMODE(info.st_mode) == 0o700
            for name in ("v3_command.json", "v3_status.json"):
                assert runtime_mailbox_path(None, root, filename=name) == directory / name
                assert runtime_mailbox_path(str(directory / name), root, filename=name) == directory / name
        with pytest.raises(ValueError):
            runtime_mailbox_path(str(other / "v3_command.json"), roots[0], filename="v3_command.json")
    finally:
        for directory in directories:
            shutil.rmtree(directory)


@pytest.mark.parametrize("mode", [0o755, 0o770])
def test_nonprivate_live_directory_is_rejected(live_project, mode):
    root, directory = live_project
    directory.chmod(mode)
    with pytest.raises(ValueError):
        live_io_directory(root)


def test_symlinked_live_directory_is_rejected(live_project, tmp_path):
    root, directory = live_project
    directory.rmdir()
    target = tmp_path / "target"
    target.mkdir(mode=0o700)
    directory.symlink_to(target, target_is_directory=True)
    with pytest.raises(ValueError):
        live_io_directory(root)


@pytest.mark.parametrize("part", ["directory", "file"])
def test_foreign_owned_live_paths_are_rejected(live_project, monkeypatch, part):
    root, directory = live_project
    path = directory if part == "directory" else directory / "v3_command.json"
    if part == "file":
        path.touch(mode=0o600)
    identity = path.lstat()

    def change_owner(result):
        if (result.st_dev, result.st_ino) == (identity.st_dev, identity.st_ino):
            fields = list(result)
            fields[4] = result.st_uid + 1
            return os.stat_result(fields)
        return result

    if part == "directory":
        original = os.fstat
        monkeypatch.setattr(os, "fstat", lambda descriptor: change_owner(original(descriptor)))
    else:
        original = Path.lstat
        monkeypatch.setattr(Path, "lstat", lambda candidate, *args, **kwargs:
                            change_owner(original(candidate, *args, **kwargs)))
    with pytest.raises(ValueError):
        runtime_mailbox_path(str(directory / "v3_command.json"), root, filename="v3_command.json")


@pytest.mark.parametrize("kind", ["symlink", "fifo", "mode"])
def test_live_mailbox_must_be_a_private_regular_file(live_project, tmp_path, kind):
    root, directory = live_project
    path = directory / "v3_command.json"
    if kind == "symlink":
        target = tmp_path / "target.json"
        target.touch(mode=0o600)
        path.symlink_to(target)
    elif kind == "fifo":
        os.mkfifo(path, mode=0o600)
    else:
        path.touch(mode=0o644)
    with pytest.raises(ValueError):
        runtime_mailbox_path(str(path), root, filename="v3_command.json")


def test_ram_aliases_and_wrong_filename_cannot_select_another_authority(live_project, tmp_path):
    root, directory = live_project
    alias = tmp_path / "alias"
    alias.symlink_to(directory, target_is_directory=True)
    for value in (alias / "v3_command.json", directory / "v3_status.json", directory / "capture.mcap"):
        with pytest.raises(ValueError):
            runtime_mailbox_path(str(value), root, filename="v3_command.json")


def test_workspace_runtime_alias_cannot_admit_another_projects_ram_paths(live_project, tmp_path):
    import v3_process_runtime as native

    root, _directory = live_project
    other_root = tmp_path / "other"
    other_root.mkdir()
    other = live_io_directory(other_root)
    try:
        (root / "runtime").symlink_to(other, target_is_directory=True)
        with pytest.raises(ValueError):
            runtime_mailbox_path("runtime/v3_command.json", root, filename="v3_command.json")
        with pytest.raises(ValueError):
            native._runtime_owned_path("runtime/capture.mcap", root)
    finally:
        shutil.rmtree(other)


@pytest.mark.parametrize("validator", ["control", "native"])
def test_cli_validators_limit_ram_to_the_matching_mailbox_role(live_project, validator):
    from v3.control_cli import _runtime_path
    from v3_process_runtime import _runtime_owned_path

    root, directory = live_project
    validate = _runtime_path if validator == "control" else _runtime_owned_path
    for name in ("v3_command.json", "v3_status.json"):
        assert validate(None, root, filename=name) == directory / name
        assert validate(str(directory / name), root, filename=name) == directory / name
    with pytest.raises(ValueError):
        validate(str(directory / "v3_status.json"), root, filename="v3_command.json")
    with pytest.raises(ValueError):
        validate(str(directory / "capture.mcap"), root)
    assert validate("runtime/captures/test.mcap", root) == root / "runtime" / "captures" / "test.mcap"


def test_facade_uses_ram_but_explicit_and_unit_controllers_keep_their_paths(live_project):
    root, directory = live_project
    interface = RobotInterface(root, adapters=(), upper_runtime=False)
    assert interface.controller.command_file == directory / "v3_command.json"
    assert interface.controller.status_file == directory / "v3_status.json"
    assert interface.controller.capture_dir == root / "runtime" / "captures"
    legacy = OperatorController(root)
    injected = RobotInterface(root, controller=legacy, adapters=(), upper_runtime=False)
    assert injected.controller is legacy
    assert legacy.command_file == root / "runtime" / "v3_command.json"
    assert legacy.status_file == root / "runtime" / "v3_status.json"


def test_runtime_and_control_producer_receive_identical_canonical_paths(live_project, monkeypatch):
    import v3.operator_controller as module

    root, directory = live_project
    controller = OperatorController(root, live_io_dir=directory)
    calls = []
    monkeypatch.setattr(module.subprocess, "Popen", lambda command, **kwargs: (
        calls.append((command, kwargs)) or SimpleNamespace(pid=123, wait=lambda: 0, poll=lambda: 0)))
    controller.runtime_dir.mkdir()
    capture = controller.capture_dir / "test.mcap"
    assert controller.run_runtime_session(capture, "full", 5) == 0
    controller._spawn_control_process([controller.python, "-m", "v3.control_cli", "explore"])
    assert len(calls) == 2
    for command, kwargs in calls:
        assert command[command.index("--command-path") + 1] == str(controller.command_file)
        assert command[command.index("--status-path") + 1] == str(controller.status_file)
        assert kwargs["cwd"] == root
    runtime_command = calls[0][0]
    assert runtime_command[runtime_command.index("--capture-path") + 1] == "runtime/captures/test.mcap"


def test_internal_supervisor_preserves_its_explicit_mailbox_pair(live_project, monkeypatch):
    import v3.operator_cli as cli
    import v3.runtime_performance as performance

    root, directory = live_project
    calls = []
    monkeypatch.setattr(cli, "__file__", str(root / "v3" / "operator_cli.py"))
    monkeypatch.setattr(performance, "apply_host_affinity", lambda *args, **kwargs: None)
    monkeypatch.setattr(cli, "OperatorController", lambda **kwargs: OperatorController(root, **kwargs))
    monkeypatch.setattr(OperatorController, "run_runtime_session", lambda controller, capture, mode, hz:
                        calls.append((controller.command_file, controller.status_file, capture, mode, hz)) or 0)
    capture = root / "runtime" / "captures" / "test.mcap"
    assert cli.main(["__runtime-session", "--capture-path", str(capture), "--capture-mode", "full",
                     "--capture-hz", "5", "--command-path", str(directory / "v3_command.json"),
                     "--status-path", str(directory / "v3_status.json")]) == 0
    assert calls == [(directory / "v3_command.json", directory / "v3_status.json", capture, "full", 5)]


def test_producer_cleanup_only_matches_its_projects_live_mailbox(live_project, tmp_path, monkeypatch):
    root, directory = live_project
    other_root = tmp_path / "other"
    other_root.mkdir()
    other = live_io_directory(other_root)
    try:
        controller = OperatorController(root, live_io_dir=directory)
        command = [controller.python, "-m", "v3.control_cli", "--command-path", str(controller.command_file),
                   "--status-path", str(controller.status_file), "explore", "--command-id", "one"]
        monkeypatch.setattr(controller, "_pid_argv", lambda pid: command)
        assert controller._command_producer_pid_matches(123)
        command[command.index("--command-path") + 1] = str(other / "v3_command.json")
        assert not controller._command_producer_pid_matches(123)
    finally:
        shutil.rmtree(other)
