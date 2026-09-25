from __future__ import annotations

from pathlib import Path

from v3 import host_cli, interface_cli, launcher_cli


def test_root_r_is_thin_bootstrap():
    root = Path(__file__).resolve().parents[1]
    text = (root / "r").read_text(encoding="utf-8")
    assert "v3.launcher_cli" in text
    assert "/proc/stat" not in text
    assert "v3.interface_cli" not in text
    assert len(text.encode("utf-8")) < 2048


def test_launcher_module_stays_dispatch_only():
    text = Path(launcher_cli.__file__).read_text(encoding="utf-8")
    assert "/proc/stat" not in text
    assert "OperatorController" not in text
    assert "host_cli.execute" in text


def test_launcher_delegates_robot_commands_to_canonical_interface(monkeypatch):
    calls = []
    monkeypatch.setattr(launcher_cli, "project_root", lambda: Path(__file__).resolve().parents[1])
    monkeypatch.setattr(interface_cli, "main", lambda argv: calls.append(list(argv)) or 17)
    assert launcher_cli.main(["rc", "30", "c", "10"]) == 17
    assert calls == [["rc", "30", "c", "10"]]


def test_local_commands_are_routed_to_host_cli(monkeypatch):
    calls = []
    monkeypatch.setattr(launcher_cli, "project_root", lambda: Path(__file__).resolve().parents[1])
    monkeypatch.setattr(host_cli, "execute", lambda command, argv, root: calls.append((command, list(argv))) or 19)
    assert launcher_cli.main(["version"]) == 19
    assert calls == [("version", [])]


def test_command_catalog_uses_current_capture_and_pytest_ssot():
    catalog = launcher_cli.command_catalog()
    assert catalog["capture"]["default_hz"] == 10
    assert catalog["capture"]["hz"] == [1, 5, 10, 50]
    assert "gate" in catalog["testhub"]["pytest_profiles"]
    assert "control" in catalog["testhub"]["pytest_profiles"]
    robot_names = {item["name"] for item in catalog["robot"]}
    assert {"status", "roomcruise", "followperson", "testhub", "capture"} <= robot_names


def test_testhub_cli_exposes_shared_pytest_scope():
    args = interface_cli._parser().parse_args(["th", "run", "--pytest", "control"])
    assert args.pytest_scope == "control"
