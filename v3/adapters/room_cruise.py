"""ROS-edge room coverage goals from Nav2's existing global costmap.

This samples destinations, not paths. Nav2 retains reachability, footprint and
collision ownership; no map or sampling work enters the V3 control interpreter.
"""
from __future__ import annotations

from collections import deque
import math


class RoomCruiseGoals:
    """Frontier discovery followed by persistent, bounded known-room coverage."""

    def __init__(self):
        self.session = None
        self.phase = "IDLE"
        self.grid = None
        self.grid_ns = 0
        self.grid_revision = 0
        self._sample_revision = -1
        self._candidates = ()
        self._visits = {}
        self._recent = deque(maxlen=8)
        self._failed = deque(maxlen=64)
        self._last_observed_ns = 0
        self._next_selection_ns = 0
        self.goals = self.reached = self.failures = 0

    def begin(self, session):
        if self.session != session:
            self.session, self.phase = session, "FRONTIER"
            self._visits.clear()
            self._recent.clear()
            self._failed.clear()
            self._next_selection_ns = 0
            self.goals = self.reached = self.failures = 0

    def stop(self):
        self.session, self.phase = None, "IDLE"

    def enter_coverage(self):
        self.phase = "COVERAGE"

    def update_costmap(self, grid, measured_ns):
        # A single latest ROS-owned snapshot, never a queue or control payload.
        self.grid, self.grid_ns = grid, measured_ns
        self.grid_revision += 1

    @staticmethod
    def _key(x, y):
        return math.floor(x/.5), math.floor(y/.5)

    def observe_pose(self, pose, now_ns):
        if self.session is None or pose is None or now_ns-self._last_observed_ns < 200_000_000:
            return
        self._last_observed_ns = now_ns
        key = self._key(*pose[:2])
        # An ordered bounded history keeps room coverage cheap for long runs.
        self._visits.pop(key, None)
        self._visits[key] = now_ns
        if len(self._visits) > 2048:
            del self._visits[next(iter(self._visits))]

    def finish(self, goal, succeeded, now_ns):
        if goal is None:
            return
        if succeeded:
            self.reached += 1
        else:
            self.failures += 1
            self._failed.append((goal[0], goal[1], now_ns+60_000_000_000))

    def _sample(self):
        if self._sample_revision == self.grid_revision:
            return
        grid = self.grid
        width, height, resolution = grid.info.width, grid.info.height, grid.info.resolution
        # At most 1024 tiles and nine cost lookups per tile, regardless of map
        # size. Low-cost Nav2 cells favor space without adding a second margin.
        stride = max(1, math.ceil(.6/resolution), math.ceil(math.sqrt(width*height/1024)),
            math.ceil(width/1024), math.ceil(height/1024))
        while math.ceil(width/stride)*math.ceil(height/stride) > 1024:
            stride += 1
        origin = grid.info.origin
        q = origin.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        cosine, sine = math.cos(yaw), math.sin(yaw)
        candidates = []
        for top in range(0, height, stride):
            for left in range(0, width, stride):
                right, bottom = min(width, left+stride), min(height, top+stride)
                cx, cy = (left+right-1)/2, (top+bottom-1)/2
                xs = {left, int(cx), right-1}
                ys = {top, int(cy), bottom-1}
                options = [(grid.data[y*width+x], (x-cx)**2+(y-cy)**2, x, y)
                    for x in xs for y in ys if 0 <= grid.data[y*width+x] <= 60]
                if not options:
                    continue
                _, _, x, y = min(options)
                gx, gy = (x+.5)*resolution, (y+.5)*resolution
                candidates.append((origin.position.x+cosine*gx-sine*gy,
                    origin.position.y+sine*gx+cosine*gy))
        self._candidates = tuple(candidates[:1024])
        self._sample_revision = self.grid_revision

    def select(self, pose, now_ns):
        if (self.phase != "COVERAGE" or pose is None or self.grid is None
                or not 0 <= now_ns-self.grid_ns <= 3_000_000_000
                or now_ns < self._next_selection_ns):
            return None
        # Empty/unreachable rooms must not turn an idle action loop into a
        # continuously running goal scorer.
        self._next_selection_ns = now_ns+1_000_000_000
        self._sample()
        x, y, heading = pose
        scored = []
        for gx, gy in self._candidates:
            distance = math.hypot(gx-x, gy-y)
            if distance < .65 or any(until > now_ns and math.hypot(gx-fx, gy-fy) < .65
                    for fx, fy, until in self._failed):
                continue
            angle = math.atan2(gy-y, gx-x)
            kx, ky = self._key(gx, gy)
            last_visit = max((self._visits.get((kx+dx, ky+dy), 0)
                for dx in (-1, 0, 1) for dy in (-1, 0, 1)), default=0)
            novelty = 1. if not last_visit else min(1., (now_ns-last_visit)/120_000_000_000)
            repeat = max((math.exp(-math.hypot(gx-rx, gy-ry)/.65)
                for rx, ry in self._recent), default=0.)
            # Favor new room sectors and useful traversals, retaining a mild
            # forward preference to avoid selecting a pivot after each arrival.
            score = (4.*novelty-3.*repeat+.7*math.cos(angle-heading)
                +.5*min(distance, 2.)-.2*max(0., distance-3.))
            scored.append((score, -distance, gx, gy, angle))
        if not scored:
            return None
        _, _, gx, gy, yaw = max(scored)
        self._recent.append((gx, gy))
        self.goals += 1
        return [gx, gy, yaw]

    def snapshot(self):
        return dict(cruise_phase=self.phase, coverage_candidates=len(self._candidates),
            coverage_goals=self.goals, coverage_reached=self.reached,
            coverage_failed=self.failures, coverage_visited_cells=len(self._visits))
