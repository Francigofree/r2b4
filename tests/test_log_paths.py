from log import log_paths


def test_session_layout_seeds_latest_files_without_repointing_latest(monkeypatch, tmp_path):
    logs_dir = tmp_path / "logs"
    previous = logs_dir / "session_20260726T190000Z"
    previous.mkdir(parents=True)
    (previous / "latest_hub_summary.json").write_text('{"status":"PASS"}\n', encoding="utf-8")
    (previous / "latest_hub_run.json").write_text('{"run_id":"old"}\n', encoding="utf-8")
    latest = logs_dir / "latest"
    latest.symlink_to(previous.name, target_is_directory=True)

    monkeypatch.setattr(log_paths, "LOGS_DIR", logs_dir)
    monkeypatch.setattr(log_paths, "LATEST_DIR", latest)
    monkeypatch.setattr(log_paths, "ARCHIVE_DIR", logs_dir / "archive")

    created = log_paths.ensure_session_layout(logs_dir / "session_20260726T191000Z")

    assert latest.resolve() == previous.resolve()
    assert (created / "runtime").is_dir()
    assert (created / "tests").is_dir()
    assert (created / "latest_hub_summary.json").read_text(encoding="utf-8") == '{"status":"PASS"}\n'
    assert (created / "latest_hub_run.json").read_text(encoding="utf-8") == '{"run_id":"old"}\n'
