from pathlib import Path


def test_upgrade_staging_and_backups_are_git_ignored():
    root = Path(__file__).resolve().parents[1]
    text = (root / ".gitignore").read_text(encoding="utf-8")
    assert "/integralando/" in text
    assert "/runtime/upgrade_backups/" in text
