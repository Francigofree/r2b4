"""Small Room Cruise goal policy for the isolated Nav2 ROS edge.

The grid and its searches stay in the edge process. This helper suggests only
known-free goals; Nav2 still owns path finding and obstacle avoidance, and V3
admits the resulting expiring velocity through its canonical motion chain.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math


MAX_MAP_CELLS = 250_000
MAX_VISITED_GOALS = 32
MAX_SEARCH_RADIUS_CELLS = 96


@dataclass(frozen=True, slots=True)
class OccupancyMap:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: tuple[int, ...]

    def __post_init__(self):
        if (type(self.width) is not int or type(self.height) is not int
                or self.width <= 0 or self.height <= 0
                or self.width * self.height > MAX_MAP_CELLS):
            raise ValueError("Room Cruise map dimensions exceed bounds")
        if (not isinstance(self.data, tuple) or len(self.data) != self.width * self.height
                or any(type(cell) is not int or not -1 <= cell <= 100 for cell in self.data)):
            raise ValueError("Room Cruise requires a complete immutable occupancy grid")
        for name in ("resolution", "origin_x", "origin_y", "origin_yaw"):
            value = getattr(self, name)
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError("Room Cruise map geometry must be finite")
        if self.resolution <= 0:
            raise ValueError("Room Cruise map resolution must be positive")

    def cell(self, x, y):
        cosine, sine = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        dx, dy = x - self.origin_x, y - self.origin_y
        return (math.floor((cosine * dx + sine * dy) / self.resolution),
                math.floor((-sine * dx + cosine * dy) / self.resolution))

    def position(self, x, y):
        cosine, sine = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        dx, dy = (x + .5) * self.resolution, (y + .5) * self.resolution
        return (self.origin_x + cosine * dx - sine * dy,
                self.origin_y + sine * dx + cosine * dy)


def _integral(grid, predicate):
    """Summed areas keep footprint/frontier checks constant work per cell."""
    stride = grid.width + 1
    result = [0] * ((grid.height + 1) * stride)
    for y in range(grid.height):
        count = 0
        row, previous = (y + 1) * stride, y * stride
        source = y * grid.width
        for x in range(grid.width):
            count += predicate(grid.data[source + x])
            result[row + x + 1] = result[previous + x + 1] + count
    return result


def _count(area, width, x0, y0, x1, y1):
    stride = width + 1
    return (area[y1 * stride + x1] - area[y0 * stride + x1]
            - area[y1 * stride + x0] + area[y0 * stride + x0])


@dataclass(frozen=True, slots=True)
class FrontierGoalSelector:
    robot_radius_m: float
    min_goal_distance_m: float = .5
    revisit_distance_m: float = .5
    max_goal_distance_m: float = 3.

    def __post_init__(self):
        for name in ("robot_radius_m", "min_goal_distance_m", "revisit_distance_m", "max_goal_distance_m"):
            value = getattr(self, name)
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError("Room Cruise goal distances must be finite and positive")
        if self.max_goal_distance_m < self.min_goal_distance_m:
            raise ValueError("Room Cruise maximum goal distance is below its minimum")

    def select(self, grid, start_x, start_y, visited=()):
        """Return a reachable frontier or coverage goal, or None when none is safe.

        A square enclosing the configured circular footprint must contain only
        cells explicitly marked free (0). Unknown, uncertain and occupied cells
        all exclude that footprint. A local search window bounds map work even
        when SLAM's global grid grows. With no frontier, an unvisited known-free
        coverage goal keeps Room Cruise moving in an already mapped room.
        Searches run only when the edge needs a child goal, never at control Hz.
        """
        if not isinstance(grid, OccupancyMap):
            raise TypeError("Room Cruise requires OccupancyMap")
        if any(type(value) not in (int, float) or not math.isfinite(value)
               for value in (start_x, start_y)):
            raise ValueError("Room Cruise start must be finite")
        if (not isinstance(visited, (list, tuple)) or len(visited) > MAX_VISITED_GOALS
                or any(not isinstance(point, (list, tuple)) or len(point) not in (2, 3)
                       or any(type(value) not in (int, float) or not math.isfinite(value)
                              for value in point) for point in visited)):
            raise ValueError("Room Cruise visited goals exceed bounds or are malformed")
        width, height = grid.width, grid.height
        sx, sy = grid.cell(start_x, start_y)
        # Half a cell accounts for any robot location within the source cell;
        # the enclosing square conservatively covers every circular footprint.
        if self.robot_radius_m >= grid.resolution * min(width, height) / 2:
            return None
        inset = math.ceil(self.robot_radius_m / grid.resolution + .5)
        if not inset <= sx < width - inset or not inset <= sy < height - inset:
            return None
        if inset >= MAX_SEARCH_RADIUS_CELLS:
            return None
        window = (MAX_SEARCH_RADIUS_CELLS if self.max_goal_distance_m >= grid.resolution * MAX_SEARCH_RADIUS_CELLS
                  else min(MAX_SEARCH_RADIUS_CELLS, math.ceil(self.max_goal_distance_m / grid.resolution) + inset + 2))
        x0, y0 = max(0, sx - window), max(0, sy - window)
        x1, y1 = min(width, sx + window + 1), min(height, sy + window + 1)
        if (x0, y0, x1, y1) != (0, 0, width, height):
            cosine, sine = math.cos(grid.origin_yaw), math.sin(grid.origin_yaw)
            grid = OccupancyMap(x1 - x0, y1 - y0, grid.resolution,
                grid.origin_x + (cosine * x0 - sine * y0) * grid.resolution,
                grid.origin_y + (sine * x0 + cosine * y0) * grid.resolution,
                grid.origin_yaw, tuple(grid.data[y * width + x]
                                     for y in range(y0, y1) for x in range(x0, x1)))
            sx, sy = sx - x0, sy - y0
            width, height = grid.width, grid.height
        blocked = _integral(grid, lambda value: value != 0)
        unknown = _integral(grid, lambda value: value == -1)
        safe = bytearray(width * height)
        for y in range(inset, height - inset):
            for x in range(inset, width - inset):
                if _count(blocked, width, x - inset, y - inset,
                          x + inset + 1, y + inset + 1) == 0:
                    safe[y * width + x] = 1
        start = sy * width + sx
        if not safe[start]:
            return None
        queue = deque([(start, 0)])
        safe[start] = 0
        frontier_reach = inset + 2
        best = None
        coverage = None
        minimum_squared = self.min_goal_distance_m ** 2
        maximum_squared = self.max_goal_distance_m ** 2
        revisit_squared = self.revisit_distance_m ** 2
        while queue:
            index, steps = queue.popleft()
            if best is not None and steps > best[0][0]:
                break
            x, y = index % width, index // width
            px, py = grid.position(x, y)
            distance_squared = (px - start_x) ** 2 + (py - start_y) ** 2
            if (minimum_squared <= distance_squared <= maximum_squared
                    and not any((px - point[0]) ** 2 + (py - point[1]) ** 2 < revisit_squared
                                for point in visited)):
                coverage_score = (-distance_squared, steps, index)
                if coverage is None or coverage_score < coverage[0]:
                    coverage = coverage_score, px, py
                if _count(unknown, width, max(0, x - frontier_reach), max(0, y - frontier_reach),
                          min(width, x + frontier_reach + 1), min(height, y + frontier_reach + 1)):
                    # Nearest reachable frontier first; cell index breaks ties.
                    score = (steps, distance_squared, index)
                    if best is None or score < best[0]:
                        best = score, x, y, px, py
            for nx, ny in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
                neighbor = ny * width + nx
                if 0 <= nx < width and 0 <= ny < height and safe[neighbor]:
                    safe[neighbor] = 0
                    queue.append((neighbor, steps + 1))
        if best is None:
            if coverage is None:
                return None
            _, px, py = coverage
            return px, py, math.atan2(py - start_y, px - start_x)
        _, x, y, px, py = best
        # Face the nearest unknown cell so the next stationary scan observes
        # the unexplored boundary. Never put the goal itself in unknown space.
        nearest = None
        for uy in range(max(0, y - frontier_reach), min(height, y + frontier_reach + 1)):
            for ux in range(max(0, x - frontier_reach), min(width, x + frontier_reach + 1)):
                if grid.data[uy * width + ux] == -1:
                    score = ((ux - x) ** 2 + (uy - y) ** 2, uy * width + ux)
                    if nearest is None or score < nearest[0]:
                        nearest = score, ux, uy
        _, ux, uy = nearest
        yaw = math.atan2(uy - y, ux - x) + grid.origin_yaw
        return px, py, math.atan2(math.sin(yaw), math.cos(yaw))
