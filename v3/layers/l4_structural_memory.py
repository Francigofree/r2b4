"""Bounded deterministic structural geometry memory owned by L4.

This is deliberately not a global map or navigation graph. It accumulates only
repeated world-frame occupancy evidence and exposes confirmed cells for the
current rolling local window. Fresh scan free-space always has veto authority.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from v3.layers.l4_temporal_occupancy import ScanCellEvidence


@dataclass(frozen=True, slots=True)
class StructuralCell:
    score: int
    hit_count: int
    free_count: int
    first_hit_ns: int
    last_hit_ns: int
    last_observed_ns: int
    confirmed: bool


@dataclass(frozen=True, slots=True)
class StructuralMemoryCheckpoint:
    cells: tuple[tuple[int, int, StructuralCell], ...]
    current_hit_keys: tuple[tuple[int, int], ...]
    current_free_keys: tuple[tuple[int, int], ...]


class StructuralMemoryGrid:
    """Longer-lived L4-local geometric evidence with confirmation hysteresis."""

    __slots__ = (
        "_resolution_m",
        "_max_age_ns",
        "_max_cells",
        "_hit_increment",
        "_free_decrement",
        "_max_score",
        "_confirm_score",
        "_deconfirm_score",
        "_confirm_min_hits",
        "_confirm_min_span_ns",
        "_clear_score",
        "_cells",
        "_current_hit_keys",
        "_current_free_keys",
    )

    def __init__(
        self,
        *,
        resolution_m: float,
        max_age_ns: int,
        max_cells: int,
        hit_increment: int,
        free_decrement: int,
        max_score: int,
        confirm_score: int,
        confirm_min_hits: int,
        confirm_min_span_ns: int,
        clear_score: int,
        deconfirm_score: int | None = None,
    ) -> None:
        if not math.isfinite(resolution_m) or resolution_m <= 0.0:
            raise ValueError("structural resolution must be finite and positive")
        for value, name in (
            (max_age_ns, "max_age_ns"),
            (max_cells, "max_cells"),
            (hit_increment, "hit_increment"),
            (free_decrement, "free_decrement"),
            (max_score, "max_score"),
            (confirm_score, "confirm_score"),
            (confirm_min_hits, "confirm_min_hits"),
        ):
            if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if (
            not isinstance(confirm_min_span_ns, int)
            or isinstance(confirm_min_span_ns, bool)
            or confirm_min_span_ns < 0
        ):
            raise ValueError("confirm_min_span_ns must be non-negative integer")
        if not isinstance(clear_score, int) or isinstance(clear_score, bool) or clear_score < 0:
            raise ValueError("clear_score must be a non-negative integer")
        if deconfirm_score is None:
            deconfirm_score = max(clear_score, (clear_score + confirm_score) // 2)
        if not isinstance(deconfirm_score, int) or isinstance(deconfirm_score, bool):
            raise ValueError("deconfirm_score must be an integer or None")
        if confirm_score > max_score:
            raise ValueError("confirm_score cannot exceed max_score")
        if not clear_score <= deconfirm_score < confirm_score:
            raise ValueError("structural scores must satisfy clear <= deconfirm < confirm")
        self._resolution_m = resolution_m
        self._max_age_ns = max_age_ns
        self._max_cells = max_cells
        self._hit_increment = hit_increment
        self._free_decrement = free_decrement
        self._max_score = max_score
        self._confirm_score = confirm_score
        self._deconfirm_score = deconfirm_score
        self._confirm_min_hits = confirm_min_hits
        self._confirm_min_span_ns = confirm_min_span_ns
        self._clear_score = clear_score
        self._cells: dict[tuple[int, int], StructuralCell] = {}
        self._current_hit_keys: tuple[tuple[int, int], ...] = ()
        self._current_free_keys: tuple[tuple[int, int], ...] = ()

    @property
    def current_hit_keys(self) -> tuple[tuple[int, int], ...]:
        return self._current_hit_keys

    @property
    def current_free_keys(self) -> tuple[tuple[int, int], ...]:
        return self._current_free_keys

    def clear(self) -> bool:
        changed = bool(self._cells or self._current_hit_keys or self._current_free_keys)
        self._cells.clear()
        self._current_hit_keys = ()
        self._current_free_keys = ()
        return changed

    def integrate(
        self,
        evidence: ScanCellEvidence,
        *,
        captured_ns: int,
        ignored_hit_keys: frozenset[tuple[int, int]] = frozenset(),
    ) -> bool:
        if not isinstance(evidence, ScanCellEvidence):
            raise TypeError("evidence must be ScanCellEvidence")
        if not isinstance(captured_ns, int) or isinstance(captured_ns, bool) or captured_ns < 0:
            raise ValueError("captured_ns must be a non-negative integer")
        if not isinstance(ignored_hit_keys, frozenset):
            raise TypeError("ignored_hit_keys must be a frozenset")
        self._current_hit_keys = evidence.hit_keys
        self._current_free_keys = evidence.free_keys
        changed = False

        for key in evidence.free_keys:
            previous = self._cells.get(key)
            if previous is None:
                continue
            score = max(0, previous.score - self._free_decrement)
            free_count = previous.free_count + 1
            if score <= self._clear_score:
                del self._cells[key]
            else:
                self._cells[key] = StructuralCell(
                    score=score,
                    hit_count=previous.hit_count,
                    free_count=free_count,
                    first_hit_ns=previous.first_hit_ns,
                    last_hit_ns=previous.last_hit_ns,
                    last_observed_ns=captured_ns,
                    confirmed=previous.confirmed and score > self._deconfirm_score,
                )
            changed = True

        for key in evidence.hit_keys:
            if key in ignored_hit_keys:
                continue
            previous = self._cells.get(key)
            if previous is None:
                score = min(self._max_score, self._hit_increment)
                hit_count = 1
                first_hit_ns = captured_ns
                confirmed = False
                free_count = 0
            else:
                score = min(self._max_score, previous.score + self._hit_increment)
                hit_count = previous.hit_count + 1
                first_hit_ns = previous.first_hit_ns
                confirmed = previous.confirmed
                free_count = previous.free_count
            if (
                not confirmed
                and score >= self._confirm_score
                and hit_count >= self._confirm_min_hits
                and captured_ns - first_hit_ns >= self._confirm_min_span_ns
            ):
                confirmed = True
            self._cells[key] = StructuralCell(
                score=score,
                hit_count=hit_count,
                free_count=free_count,
                first_hit_ns=first_hit_ns,
                last_hit_ns=captured_ns,
                last_observed_ns=captured_ns,
                confirmed=confirmed,
            )
            changed = True

        if self._enforce_bound():
            changed = True
        return changed

    def prune(self, *, now_ns: int) -> bool:
        remove = tuple(
            key
            for key, cell in self._cells.items()
            if now_ns - cell.last_observed_ns > self._max_age_ns
        )
        for key in remove:
            del self._cells[key]
        changed = bool(remove)
        if self._enforce_bound():
            changed = True
        return changed

    def confirmed_cells(
        self,
        *,
        center_x_m: float,
        center_y_m: float,
        radius_m: float,
        excluded_keys: frozenset[tuple[int, int]] = frozenset(),
    ) -> tuple[tuple[int, int, int], ...]:
        if not math.isfinite(center_x_m) or not math.isfinite(center_y_m):
            raise ValueError("structural query center must be finite")
        if not math.isfinite(radius_m) or radius_m <= 0.0:
            raise ValueError("structural query radius must be finite and positive")
        result: list[tuple[int, int, int]] = []
        for key in sorted(self._cells):
            if key in excluded_keys:
                continue
            cell = self._cells[key]
            if not cell.confirmed:
                continue
            if self._cell_distance_m(key, center_x_m, center_y_m) > radius_m:
                continue
            result.append((key[0], key[1], cell.hit_count))
        return tuple(result)

    def checkpoint(self) -> StructuralMemoryCheckpoint:
        return StructuralMemoryCheckpoint(
            tuple((key[0], key[1], self._cells[key]) for key in sorted(self._cells)),
            self._current_hit_keys,
            self._current_free_keys,
        )

    def restore(self, checkpoint: StructuralMemoryCheckpoint) -> None:
        if not isinstance(checkpoint, StructuralMemoryCheckpoint):
            raise TypeError("checkpoint must be StructuralMemoryCheckpoint")
        self._cells = {(x_index, y_index): cell for x_index, y_index, cell in checkpoint.cells}
        self._current_hit_keys = checkpoint.current_hit_keys
        self._current_free_keys = checkpoint.current_free_keys

    def _cell_distance_m(self, key: tuple[int, int], x_m: float, y_m: float) -> float:
        center_x = (key[0] + 0.5) * self._resolution_m
        center_y = (key[1] + 0.5) * self._resolution_m
        return math.hypot(center_x - x_m, center_y - y_m)

    def _enforce_bound(self) -> bool:
        if len(self._cells) <= self._max_cells:
            return False
        keep = set(
            sorted(
                self._cells,
                key=lambda key: (
                    not self._cells[key].confirmed,
                    -self._cells[key].score,
                    -self._cells[key].last_observed_ns,
                    key,
                ),
            )[: self._max_cells]
        )
        changed = False
        for key in tuple(self._cells):
            if key not in keep:
                del self._cells[key]
                changed = True
        return changed


__all__ = [
    "StructuralCell",
    "StructuralMemoryCheckpoint",
    "StructuralMemoryGrid",
]
