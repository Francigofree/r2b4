"""Deterministic bounded temporal occupancy evidence for L4."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class TemporalCell:
    score: int
    observation_count: int
    last_update_ns: int


@dataclass(frozen=True, slots=True)
class TemporalOccupancyCheckpoint:
    cells: tuple[tuple[int, int, int, int, int], ...]


class TemporalOccupancyGrid:
    __slots__ = (
        "_resolution_m",
        "_radius_m",
        "_max_age_ns",
        "_max_cells",
        "_hit_increment",
        "_free_decrement",
        "_max_score",
        "_cells",
    )

    def __init__(
        self,
        *,
        resolution_m: float,
        radius_m: float,
        max_age_ns: int,
        max_cells: int,
        hit_increment: int = 1,
        free_decrement: int = 1,
        max_score: int = 8,
    ) -> None:
        if resolution_m <= 0.0 or radius_m <= 0.0:
            raise ValueError("occupancy geometry must be positive")
        if max_age_ns <= 0 or max_cells <= 0:
            raise ValueError("occupancy bounds must be positive")
        if hit_increment <= 0 or free_decrement <= 0 or max_score <= 0:
            raise ValueError("occupancy evidence bounds must be positive")
        self._resolution_m = resolution_m
        self._radius_m = radius_m
        self._max_age_ns = max_age_ns
        self._max_cells = max_cells
        self._hit_increment = hit_increment
        self._free_decrement = free_decrement
        self._max_score = max_score
        self._cells: dict[tuple[int, int], TemporalCell] = {}

    def clear(self) -> bool:
        changed = bool(self._cells)
        self._cells.clear()
        return changed

    def grid_key(self, x_m: float, y_m: float) -> tuple[int, int]:
        return math.floor(x_m / self._resolution_m), math.floor(y_m / self._resolution_m)

    def cell_distance_m(self, key: tuple[int, int], x_m: float, y_m: float) -> float:
        center_x = (key[0] + 0.5) * self._resolution_m
        center_y = (key[1] + 0.5) * self._resolution_m
        return math.hypot(center_x - x_m, center_y - y_m)

    def integrate_scan(
        self,
        *,
        origin_x_m: float,
        origin_y_m: float,
        endpoints: tuple[tuple[float, float], ...],
        captured_ns: int,
    ) -> bool:
        hit_keys = {
            self.grid_key(x_m, y_m)
            for x_m, y_m in endpoints
            if math.hypot(x_m - origin_x_m, y_m - origin_y_m) <= self._radius_m
        }
        origin_key = self.grid_key(origin_x_m, origin_y_m)
        free_keys: set[tuple[int, int]] = set()
        for hit_key in sorted(hit_keys):
            ray = self._bresenham(origin_key, hit_key)
            for key in ray[1:-1]:
                if key not in hit_keys:
                    free_keys.add(key)

        changed = False
        for key in sorted(free_keys):
            previous = self._cells.get(key)
            if previous is None:
                continue
            score = previous.score - self._free_decrement
            if score <= 0:
                del self._cells[key]
            else:
                self._cells[key] = TemporalCell(score, previous.observation_count, captured_ns)
            changed = True

        for key in sorted(hit_keys):
            previous = self._cells.get(key)
            if previous is None:
                self._cells[key] = TemporalCell(
                    min(self._max_score, self._hit_increment),
                    1,
                    captured_ns,
                )
            else:
                self._cells[key] = TemporalCell(
                    min(self._max_score, previous.score + self._hit_increment),
                    previous.observation_count + 1,
                    captured_ns,
                )
            changed = True
        return changed

    def prune(self, *, now_ns: int, center_x_m: float, center_y_m: float) -> bool:
        remove = tuple(
            key
            for key, cell in self._cells.items()
            if now_ns - cell.last_update_ns > self._max_age_ns
            or self.cell_distance_m(key, center_x_m, center_y_m) > self._radius_m
        )
        for key in remove:
            del self._cells[key]
        changed = bool(remove)
        if len(self._cells) > self._max_cells:
            keep = set(
                sorted(
                    self._cells,
                    key=lambda key: (
                        -self._cells[key].last_update_ns,
                        self.cell_distance_m(key, center_x_m, center_y_m),
                        key,
                    ),
                )[: self._max_cells]
            )
            for key in tuple(self._cells):
                if key not in keep:
                    del self._cells[key]
                    changed = True
        return changed

    def occupied_cells(self) -> tuple[tuple[int, int, int], ...]:
        return tuple(
            (key[0], key[1], self._cells[key].observation_count)
            for key in sorted(self._cells)
            if self._cells[key].score > 0
        )

    def checkpoint(self) -> TemporalOccupancyCheckpoint:
        return TemporalOccupancyCheckpoint(
            tuple(
                (x_index, y_index, cell.score, cell.observation_count, cell.last_update_ns)
                for (x_index, y_index), cell in sorted(self._cells.items())
            )
        )

    def restore(self, checkpoint: TemporalOccupancyCheckpoint) -> None:
        if not isinstance(checkpoint, TemporalOccupancyCheckpoint):
            raise TypeError("checkpoint must be TemporalOccupancyCheckpoint")
        self._cells = {
            (x_index, y_index): TemporalCell(score, count, last_update_ns)
            for x_index, y_index, score, count, last_update_ns in checkpoint.cells
        }

    @staticmethod
    def _bresenham(start: tuple[int, int], end: tuple[int, int]) -> tuple[tuple[int, int], ...]:
        x0, y0 = start
        x1, y1 = end
        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        error = dx + dy
        result: list[tuple[int, int]] = []
        while True:
            result.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            twice = 2 * error
            if twice >= dy:
                error += dy
                x0 += sx
            if twice <= dx:
                error += dx
                y0 += sy
        return tuple(result)


__all__ = [
    "TemporalCell",
    "TemporalOccupancyCheckpoint",
    "TemporalOccupancyGrid",
]
