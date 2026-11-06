"""Offline evidence CLI convenience preserves explicit inputs and local data."""
from __future__ import annotations

import json
import os
from pathlib import Path

import pytest

from tools.mcap_evidence import cli


def _capture(root: Path, name: str, mtime_ns: int) -> Path:
    path = root / 'runtime' / 'captures' / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b'local capture')
    os.utime(path, ns=(mtime_ns, mtime_ns))
    return path


@pytest.mark.parametrize('arguments', ([], ['latest'], ['compile'], ['compile', 'latest']))
def test_default_compilation_selects_newest_published_capture(arguments, monkeypatch, tmp_path, capsys):
    older = _capture(tmp_path, 'older.mcap', 1)
    _capture(tmp_path, 'a.mcap', 2)
    newest = _capture(tmp_path, 'z.mcap', 2)
    _capture(tmp_path, '.live.mcap.123.partial', 3)
    (older.parent / 'directory.mcap').mkdir()
    (older.parent / 'symlink.mcap').symlink_to(older)
    called = {}

    def compile_source(source, output, **options):
        called.update(source=source, output=output, **options)
        return {'compiler_status': 'COMPLETE'}

    monkeypatch.setattr(cli, 'compile_evidence', compile_source)
    assert cli.main(arguments, project_root=tmp_path) == 0
    assert called['source'] == str(newest)
    assert called['overwrite'] is False
    assert called['command'] == ['r', 'evi', *arguments]
    output = capsys.readouterr()
    assert json.loads(output.out)['compiler_status'] == 'COMPLETE'
    assert str(newest) in output.err


def test_options_without_source_compile_latest(monkeypatch, tmp_path):
    source = _capture(tmp_path, 'capture.mcap', 1)
    called = {}
    monkeypatch.setattr(cli, 'compile_evidence',
                        lambda selected, output, **options: called.update(source=selected, output=output, **options) or {})
    assert cli.main(['--workers', '1', '--output', '/tmp/separate.evidence'], project_root=tmp_path) == 0
    assert called['source'] == str(source)
    assert called['workers'] == '1'
    assert called['output'] == '/tmp/separate.evidence'


def test_missing_published_capture_fails_without_compiling(monkeypatch, tmp_path, capsys):
    _capture(tmp_path, '.active.mcap.123.partial', 1)
    monkeypatch.setattr(cli, 'compile_evidence', lambda *args, **kwargs: pytest.fail('missing input compiled'))
    assert cli.main([], project_root=tmp_path) == 1
    error = json.loads(capsys.readouterr().err)
    assert error['compiler_status'] == 'FAILED'
    assert 'no finalized MCAP' in error['error']
    assert str(tmp_path / 'runtime' / 'captures') in error['error']


@pytest.mark.parametrize('arguments', (['--help'], ['compile', '--help']))
def test_help_does_not_resolve_input(arguments, monkeypatch, capsys):
    monkeypatch.setattr(cli, 'latest_capture', lambda *args: pytest.fail('help selected an input'))
    with pytest.raises(SystemExit) as result:
        cli.main(arguments)
    assert result.value.code == 0
    assert 'latest' in capsys.readouterr().out


def test_explicit_source_is_forwarded_unchanged(monkeypatch, tmp_path, capsys):
    called = {}
    monkeypatch.setattr(cli, 'latest_capture', lambda *args: pytest.fail('explicit input selected latest'))
    monkeypatch.setattr(cli, 'compile_evidence',
                        lambda source, output, **options: called.update(source=source, output=output, **options) or {})
    assert cli.main(['damaged/explicit.mcap', '--overwrite', '--shard-bytes', '1024'], project_root=tmp_path) == 0
    assert called['source'] == 'damaged/explicit.mcap'
    assert called['overwrite'] is True
    assert called['shard_bytes'] == 1024
    assert capsys.readouterr().err == ''


@pytest.mark.parametrize('operation', ('verify', 'verify-evidence'))
def test_verify_keeps_bundle_and_source_arguments(operation, monkeypatch, tmp_path):
    called = {}
    monkeypatch.setattr(cli, 'latest_capture', lambda *args: pytest.fail('verify selected a capture'))
    monkeypatch.setattr(cli, 'verify', lambda bundle, **options: called.update(bundle=bundle, **options) or {})
    assert cli.main([operation, 'saved.evidence', '--source', 'original.mcap'], project_root=tmp_path) == 0
    assert called == {'bundle': 'saved.evidence', 'source': 'original.mcap'}


def test_query_remains_bundle_only_and_emits_rows(monkeypatch, tmp_path, capsys):
    called = {}

    def query_bundle(bundle, **options):
        called.update(bundle=bundle, **options)
        yield {'message_id': 'one'}

    monkeypatch.setattr(cli, 'latest_capture', lambda *args: pytest.fail('query selected a capture'))
    monkeypatch.setattr(cli, 'query', query_bundle)
    assert cli.main(['query', 'saved.evidence', '--limit', '3', '--tick-id', '9'], project_root=tmp_path) == 0
    assert called['bundle'] == 'saved.evidence'
    assert called['limit'] == 3
    assert called['tick_id'] == 9
    assert json.loads(capsys.readouterr().out) == {'message_id': 'one'}


def test_default_keeps_existing_evidence_and_reports_new_output_option(tmp_path, capsys):
    source = _capture(tmp_path, 'capture.mcap', 1)
    bundle = source.with_suffix('.evidence')
    bundle.mkdir()
    saved = bundle / 'saved.json'
    saved.write_text('preserved')
    assert cli.main([], project_root=tmp_path) == 1
    output = capsys.readouterr()
    error = json.loads(output.err.splitlines()[-1])
    assert 'output already exists' in error['error']
    assert '--output' in error['error']
    assert saved.read_text() == 'preserved'
    assert source.read_bytes() == b'local capture'
    assert not list(source.parent.glob('*.tmp-*'))


def test_default_evidence_and_diag_pipeline_uses_the_same_verified_synthetic_capture(tmp_path, capsys):
    from tools.diag import cli as diag_cli
    from tools.mcap_evidence.reader import EvidenceBundle
    from v3.mcap_writer import StdlibMcapWriter

    (tmp_path / 'v3').mkdir()
    (tmp_path / 'tools' / 'mcap_evidence').mkdir(parents=True)
    captures = tmp_path / 'runtime' / 'captures'
    captures.mkdir(parents=True)
    source = captures / 'synthetic.mcap'
    with source.open('wb') as stream:
        writer = StdlibMcapWriter(stream)
        writer.start()
        channel = writer.register_channel('/r2b4/tick')
        for tick_id in (1, 2):
            payload = {'tick_id': tick_id, 'expected': {'layers': {'L11': {'requested_v_mps': 0.0}}}}
            writer.add_message(channel, log_time_ns=tick_id * 100_000_000,
                               data=json.dumps(payload).encode(), sequence=tick_id)
        writer.finish()
    original = source.read_bytes()

    assert cli.main(['--workers', '1'], project_root=tmp_path) == 0
    compiled = json.loads(capsys.readouterr().out)
    bundle_path = source.with_suffix('.evidence')
    assert compiled['compiler_status'] == 'COMPLETE'
    assert Path(compiled['output']) == bundle_path
    bundle = EvidenceBundle.open(bundle_path)
    assert bundle.verification['status'] == 'PASS'
    assert bundle.message_count() == 2

    # This minimal capture intentionally lacks the views needed by some analyzers.
    assert diag_cli.main(['--json', '--no-save'], project_root=tmp_path) == 2
    diagnosed = json.loads(capsys.readouterr().out)
    assert diagnosed['source'] == 'EVI_EVIDENCE_ONLY'
    assert Path(diagnosed['evidence']['path']) == bundle_path
    assert diagnosed['evidence']['verification_status'] == 'PASS'
    assert diagnosed['evidence']['message_count'] == 2
    assert diagnosed['analyzers']
    assert source.read_bytes() == original
    assert not (tmp_path / 'runtime' / 'diag').exists()
