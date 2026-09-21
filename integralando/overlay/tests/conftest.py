"""Shared R2B4 pytest collection metadata.

The physical test-file layout intentionally remains flat during the Test Hub
migration.  Domain markers are derived from the same profile registry that the
Test Hub uses, so selection semantics have one source of truth.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from v3.pytest_profiles import markers_for_test_path


def pytest_collection_modifyitems(config: pytest.Config, items: list[pytest.Item]) -> None:
    root = Path(str(config.rootpath)).resolve()
    for item in items:
        path = Path(str(item.path)).resolve()
        try:
            relative = path.relative_to(root).as_posix()
        except ValueError:
            continue
        for marker_name in markers_for_test_path(relative):
            item.add_marker(getattr(pytest.mark, marker_name))
