"""Best-effort ER2 provider/tool evidence over the existing R2B4 HRI journal.

This module is observation-only. It owns no command, mission, runtime, safety or
motor authority. The passive capture sidecar imports the journal into MCAP.
"""
from __future__ import annotations

import os
import time
from pathlib import Path
from typing import Any

from v3.hri_evidence import HriEventJournal, default_hri_journal


class Er2Evidence:
    """Small metadata-only journal facade for ER2 integration events."""

    def __init__(self, journal: HriEventJournal | None, *, session_id: str | None = None) -> None:
        self.journal = journal
        self.session_id = session_id or f"er2-{os.getpid()}-{time.monotonic_ns()}"

    @classmethod
    def from_project_root(cls, project_root: str | Path) -> "Er2Evidence":
        return cls(default_hri_journal(Path(project_root).resolve()))

    def emit(self, event_type: str, **fields: Any) -> dict[str, object] | None:
        if self.journal is None:
            return None
        try:
            return self.journal.append(
                event_type,
                source="ER2",
                session_id=self.session_id,
                **fields,
            )
        except Exception:
            # Evidence must never alter provider/control behavior.
            return None


__all__ = ["Er2Evidence"]
