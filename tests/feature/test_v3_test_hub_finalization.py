from __future__ import annotations

from pathlib import Path


def test_test_hub_has_one_canonical_public_entrypoint() -> None:
    import v3.test_hub as hub

    assert hub.APP_COMMANDS == frozenset({"run", "batch", "view", "compare", "test"})
    assert hub.EVIDENCE_COMMANDS == frozenset(
        {"inspect", "diagnose", "agent", "query", "verify-evidence"}
    )
    assert callable(hub.run_default)
    assert callable(hub.inspect_mcap)
    assert callable(hub.query_capture)
    assert callable(hub.verify_evidence)


def test_test_hub_legacy_module_files_are_removed() -> None:
    root = Path(__file__).resolve().parents[2]
    legacy_next = "test_hub_" + "next"
    legacy_v2 = "test_hub_" + "v2"
    assert not (root / "v3" / f"{legacy_next}.py").exists()
    assert not (root / "v3" / f"{legacy_v2}.py").exists()
    assert (root / "v3" / "test_hub_app.py").is_file()
    assert (root / "v3" / "test_hub_backend.py").is_file()
    assert (root / "docs" / "TEST_HUB.md").is_file()


def test_active_tree_has_no_legacy_test_hub_module_reference() -> None:
    root = Path(__file__).resolve().parents[2]
    skipped = {".git", ".upgrade_backups", "runtime", ".venv", "venv", "__pycache__", "integralando",}
    suffixes = {".py", ".md", ".rst", ".txt", ".sh", ".json", ".toml", ".yaml", ".yml"}
    bad: list[str] = []
    generated_files = {"APPLY_RESULT.json", "TEST_HUB_FINALIZATION_RESULT.json"}
    for path in root.rglob("*"):
        if not path.is_file() or path.suffix.lower() not in suffixes or path.name in generated_files:
            continue
        parts = path.relative_to(root).parts
        if any(part in skipped for part in parts):
            continue
        if any(part.startswith("r2b4_testhub_finalization_upgrade_") for part in parts):
            continue
        if path == Path(__file__).resolve():
            continue
        try:
            text = path.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            continue
        legacy_next = "test_hub_" + "next"
        legacy_v2 = "test_hub_" + "v2"
        if legacy_next in text or legacy_v2 in text:
            bad.append(path.relative_to(root).as_posix())
    assert not bad, "legacy Test Hub module references remain: " + ", ".join(bad)
