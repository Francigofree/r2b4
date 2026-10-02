"""Deterministic contiguous episode extraction from normalized evidence views."""
from __future__ import annotations

from collections.abc import Callable

from .contracts import DiagnosticEpisode
from .profiling import evidence_ref

_MISSING = object()


def pointer_get(payload: object, field_path: str):
    if not field_path.startswith("/"):
        raise ValueError("field_path must be an absolute JSON-pointer-like path")
    current = payload
    for raw in field_path.split("/")[1:]:
        token = raw.replace("~1", "/").replace("~0", "~")
        if not isinstance(current, dict) or token not in current:
            return _MISSING
        current = current[token]
    return current


def contiguous_field_episodes(
    bundle,
    view: str,
    field_path: str,
    *,
    episode_type: str,
    include: Callable[[object], bool] | None = None,
    max_episodes: int = 256,
) -> tuple[DiagnosticEpisode, ...]:
    """Return contiguous same-value episodes in evidence row order.

    Rows where the field is absent or rejected by ``include`` terminate the
    current episode and are not represented as an episode themselves.
    """
    episodes: list[DiagnosticEpisode] = []
    current_value = _MISSING
    start_row = None
    end_row = None
    row_count = 0

    def close() -> None:
        nonlocal current_value, start_row, end_row, row_count
        if start_row is None or end_row is None:
            current_value = _MISSING
            row_count = 0
            return
        if len(episodes) < max_episodes:
            episodes.append(DiagnosticEpisode(
                episode_type=episode_type,
                view=view,
                field_path=field_path,
                value=current_value,
                start_ref=evidence_ref(start_row, field_path),
                end_ref=evidence_ref(end_row, field_path),
                row_count=row_count,
                duration_ns=max(0, int(end_row.log_time_ns) - int(start_row.log_time_ns)),
            ))
        current_value = _MISSING
        start_row = None
        end_row = None
        row_count = 0

    for row in bundle.iter_view(view):
        value = pointer_get(row.payload, field_path)
        accepted = value is not _MISSING and (include(value) if include is not None else True)
        if not accepted:
            close()
            continue
        if start_row is None:
            current_value = value
            start_row = end_row = row
            row_count = 1
            continue
        if value == current_value:
            end_row = row
            row_count += 1
            continue
        close()
        current_value = value
        start_row = end_row = row
        row_count = 1
    close()
    return tuple(episodes)


__all__ = ["contiguous_field_episodes", "pointer_get"]
