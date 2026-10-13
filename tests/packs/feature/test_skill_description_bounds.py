"""Manuals can grow without blocking saved source discovery or execution."""
from __future__ import annotations

import hashlib

import pytest

from r2b4_orchestration.skill_library import MAX_DESCRIPTION_BYTES, MAX_SOURCE_BYTES, SkillLibrary
from r2b4_orchestration.skill_runtime import SkillRuntime
from test_python_skill_runtime import complete


def test_long_unicode_manual_keeps_bounded_discovery_and_same_executed_source(tmp_path):
    library = SkillLibrary(tmp_path / "skills")
    source = "async def run(robot):\n    return {'value': 12}\n"
    saved = library.create("observe", source)
    # The emoji crosses the byte cutoff and must be omitted as a whole.
    prefix = "ő" * (MAX_DESCRIPTION_BYTES // 2 - 1)
    manual = prefix + "🚗" + "Megfigyelési történet.\n" * 1000
    path = library.root / "observe.md"
    path.write_text(manual, encoding="utf-8")
    loaded = library.load("observe")
    assert loaded.source == source
    assert loaded.source_hash == saved["source_hash"] == hashlib.sha256(source.encode()).hexdigest()
    assert loaded.descriptor["description"] == prefix
    assert loaded.descriptor["description_truncated"] is True
    assert len(loaded.descriptor["description"].encode("utf-8")) <= MAX_DESCRIPTION_BYTES
    assert library.list("observe")[0]["availability"]["ready"] is True
    assert library.list()[0]["source_hash"] == loaded.source_hash
    runtime = SkillRuntime(library, tmp_path / "unused.sock")
    try:
        started = runtime.start("observe")
        result = complete(runtime, started["run_id"])
        assert result["state"] == "SUCCEEDED" and result["result"] == {"value": 12}
        assert result["source_hash"] == loaded.source_hash
    finally:
        runtime.close()
    assert path.read_text(encoding="utf-8") == manual


def test_source_docstring_fallback_is_bounded_by_utf8_bytes(tmp_path):
    library = SkillLibrary(tmp_path / "skills")
    source = '"""' + "ű" * MAX_DESCRIPTION_BYTES + '"""\nasync def run(robot): return 1\n'
    descriptor = library.create("long_docstring", source)
    assert len(descriptor["description"].encode("utf-8")) == MAX_DESCRIPTION_BYTES
    assert descriptor["description_truncated"] is True
    assert library.load("long_docstring").source == source


@pytest.mark.parametrize("manual", [b"bad\xff", b"a" * (MAX_DESCRIPTION_BYTES - 1) + b"\xe2\x82",
                                   b"bad\xff" + b"a" * (MAX_DESCRIPTION_BYTES + 10)])
def test_invalid_utf8_manual_remains_an_error(tmp_path, manual):
    library = SkillLibrary(tmp_path / "skills")
    library.create("observe", "async def run(robot): return 1\n")
    (library.root / "observe.md").write_bytes(manual)
    with pytest.raises(UnicodeDecodeError):
        library.load("observe")
    assert library.list()[0]["availability"]["ready"] is False


def test_description_tolerance_preserves_strict_source_test_and_save_bounds(tmp_path):
    library = SkillLibrary(tmp_path / "skills")
    source = "async def run(robot): return 1\n"
    library.create("observe", source)
    with pytest.raises(ValueError, match="description exceeded"):
        library.update("observe", source, description="x" * (MAX_DESCRIPTION_BYTES + 1))
    test_path = library.root / "tests" / "test_observe.py"
    test_path.parent.mkdir()
    test_path.write_text("#" + "x" * MAX_SOURCE_BYTES)
    assert "exceeded" in library.load("observe").descriptor["test_error"]
    (library.root / "observe.py").write_text("#" + "x" * MAX_SOURCE_BYTES)
    with pytest.raises(ValueError, match="file exceeded"):
        library.load("observe")
