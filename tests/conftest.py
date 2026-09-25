from __future__ import annotations

import sys
from pathlib import Path
import pytest

TESTS = Path(__file__).resolve().parent
ROOT = TESTS.parent
for path in (ROOT, TESTS, TESTS / "core", TESTS / "feature", TESTS / "deep"):
    s = str(path)
    if s not in sys.path:
        sys.path.insert(0, s)

HARD_CAP = 150


def pytest_collection_modifyitems(session, config, items):
    # The hard cap applies to the complete curated tree. Focused selections are naturally smaller.
    roots = {"core", "feature", "deep"}
    seen = set()
    for item in items:
        p = Path(str(item.fspath))
        seen |= roots.intersection(p.parts)
    if seen == roots and len(items) > HARD_CAP:
        raise pytest.UsageError(
            f"R2B4 pytest budget exceeded: {len(items)} items > {HARD_CAP}. "
            "Merge or remove an existing test before adding more."
        )
