from __future__ import annotations

import sys
from pathlib import Path

TESTS = Path(__file__).resolve().parent
ROOT = TESTS.parent

# Some historical developer packs import shared helpers as top-level modules.
# Keep that compatibility without making directory placement a regression API.
for path in (
    ROOT,
    TESTS,
    TESTS / "packs" / "core",
    TESTS / "packs" / "feature",
    TESTS / "packs" / "deep",
):
    if path.is_dir():
        value = str(path)
        if value not in sys.path:
            sys.path.insert(0, value)
