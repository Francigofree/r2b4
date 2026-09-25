from pathlib import Path


def test_upgrade_staging_and_backups_are_git_ignored():
    root = (Path(__import__("os").environ["R2B4_ROOT"]).resolve() if __import__("os").environ.get("R2B4_ROOT") else next((p for p in Path(__file__).resolve().parents if (p / "conf" / "hardver.json").is_file() and (p / "v3").is_dir()), Path.cwd()))
    text = (root / ".gitignore").read_text(encoding="utf-8")
    assert "/integralando/" in text
    assert "/runtime/upgrade_backups/" in text
