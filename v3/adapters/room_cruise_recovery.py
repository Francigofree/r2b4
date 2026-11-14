"""One bounded ROS-local RC recovery episode; native Nav2 executes all motion."""
from collections import deque
import math

from v3.adapters.nav2_footprint import footprint_check, sweep_check


class RoomCruiseRecovery:
    def __init__(self, body_bounds_m, padding_m):
        self.bounds, self.padding = tuple(body_bounds_m), padding_m
        self.episode = self.event_count = 0
        self.events = deque(maxlen=32)
        self.counts = dict(episodes=0, attempts=0, action_succeeded=0,
            action_failed=0, progress_verified=0, exhausted=0, verification_failed=0)
        self.reset()

    def reset(self):
        self.state, self.goal, self.selected = "IDLE", None, None
        self.tried = set()
        self.anchor = self.baseline_remaining = None
        self.deadline_ns = self.feedback_ns = 0
        self.translation_m = 0.
        self.remaining_drop_m = None
        self.started_ns = 0

    def event(self, now, kind, **values):
        self.event_count += 1
        self.events.append(dict(sequence=self.event_count, measurement_monotonic_ns=now,
            episode=self.episode, component="rc_recovery", event=kind, **values))

    def begin(self, goal, now, reason):
        if self.state in ("IDLE", "RESOLVED") or self.goal != list(goal):
            self.reset()
            self.episode += 1
            self.goal = list(goal)
            self.started_ns = now
            self.counts["episodes"] += 1
            self.event(now, "episode_started", goal=self.goal, reason=reason)
        self.state = "READY"

    def choose(self, grid, grid_ns, local_pose, global_pose, now):
        if self.state != "READY":
            return None
        if (grid is None or local_pose is None or global_pose is None
                or not 0 <= now-grid_ns <= 500_000_000):
            if now-self.started_ns > 3_000_000_000:
                self.exhaust(now, "fresh_local_costmap_unavailable")
            return None
        # Never repeat a failed maneuver within this episode, even after map
        # revisions or successful native actions without subsequent progress.
        candidates = [dict(name="reverse", kind="BACKUP", v=-.19, w=0., duration=.25/.19),
            dict(name="spin_left", kind="SPIN", v=0., w=1.1, duration=.70/1.1),
            dict(name="spin_right", kind="SPIN", v=0., w=-1.1, duration=.70/1.1)]
        safe = []
        for candidate in candidates:
            name = candidate["name"]
            if name in self.tried:
                continue
            result = sweep_check(grid, local_pose, candidate["v"], candidate["w"],
                candidate["duration"], self.bounds, self.padding)
            self.event(now, "candidate_checked", candidate=name, map_measurement_ns=grid_ns,
                frame_id=grid.header.frame_id, original=[candidate["v"], candidate["w"]],
                modified=None if not result["ok"] else [candidate["v"], candidate["w"]], **result)
            if not result["ok"]:
                continue
            x, y, yaw = local_pose
            if candidate["kind"] == "BACKUP":
                x, y = x-.25*math.cos(yaw), y-.25*math.sin(yaw)
            else:
                yaw += candidate["w"]*candidate["duration"]
            opening = 0.
            for distance in (.2, .4, .6):
                if not sweep_check(grid, (x, y, yaw), .2, 0., distance/.2,
                        self.bounds, self.padding)["ok"]:
                    break
                opening = distance
            safe.append((opening+(.03 if candidate["kind"] == "SPIN" else 0.), name, candidate))
        # Detours are proposed from the current local map. The bridge requests a
        # real Smac plan and checks that complete path before NavigateToPose.
        for sign, name in ((1., "detour_left"), (-1., "detour_right")):
            if name in self.tried:
                continue
            angle = local_pose[2]+sign*1.1
            endpoint = (local_pose[0]+.65*math.cos(angle), local_pose[1]+.65*math.sin(angle), angle)
            blocking = footprint_check(grid, *endpoint, self.bounds, self.padding)
            self.event(now, "detour_endpoint_checked", candidate=name,
                map_measurement_ns=grid_ns, frame_id=grid.header.frame_id,
                pose=list(local_pose), trajectory=dict(model="smac_plan_required", endpoint=list(endpoint)),
                first_blocking=blocking, ok=blocking is None)
            if blocking is None:
                global_angle = global_pose[2]+sign*1.1
                goal = [global_pose[0]+.65*math.cos(global_angle),
                    global_pose[1]+.65*math.sin(global_angle), global_angle]
                safe.append((.25, name, dict(name=name, kind="PLAN_DETOUR", goal=goal)))
        if not safe:
            self.exhaust(now, "no_untried_costmap_safe_maneuver")
            return None
        _, name, candidate = max(safe, key=lambda row: (row[0], row[1]))
        self.tried.add(name)
        self.selected, self.state = candidate, "EXECUTING"
        self.counts["attempts"] += 1
        self.event(now, "maneuver_selected", candidate=name, tried=sorted(self.tried))
        return candidate

    def action_finished(self, success, pose, now):
        self.counts["action_succeeded" if success else "action_failed"] += 1
        self.event(now, "action_finished", candidate=self.selected["name"], action_succeeded=success)
        if success and pose is not None:
            self.anchor, self.baseline_remaining = tuple(pose), None
            self.translation_m, self.remaining_drop_m = 0., None
            self.feedback_ns = 0
            self.deadline_ns = now+7_000_000_000  # bounded even without feedback
            self.state = "VERIFY"
        else:
            self.state = "READY"

    def feedback(self, remaining, now):
        if self.state == "VERIFY" and self.baseline_remaining is None and math.isfinite(remaining):
            self.baseline_remaining, self.feedback_ns = remaining, now
            self.deadline_ns = now+5_000_000_000
            self.event(now, "verification_started", distance_remaining_m=remaining)

    def verify(self, pose, remaining, feedback_ns, now):
        if self.state != "VERIFY" or pose is None:
            return None
        self.translation_m = math.hypot(pose[0]-self.anchor[0], pose[1]-self.anchor[1])
        if (self.baseline_remaining is not None and remaining is not None
                and feedback_ns >= self.feedback_ns and 0 <= now-feedback_ns <= 1_000_000_000):
            self.remaining_drop_m = self.baseline_remaining-remaining
            if self.translation_m >= .25 and self.remaining_drop_m >= .15:
                self.resolve(now, "resumed_navigation_progress")
                return True
        if now >= self.deadline_ns:
            self.counts["verification_failed"] += 1
            self.event(now, "verification_failed", translation_m=self.translation_m,
                remaining_drop_m=self.remaining_drop_m)
            self.state = "READY"
            return False
        return None

    def resolve(self, now, reason):
        if self.state not in ("IDLE", "RESOLVED"):
            self.counts["progress_verified"] += 1
            self.event(now, "episode_resolved", reason=reason,
                translation_m=self.translation_m, remaining_drop_m=self.remaining_drop_m)
            self.state = "RESOLVED"

    def exhaust(self, now, reason):
        self.counts["exhausted"] += 1
        self.state = "EXHAUSTED"
        self.event(now, "episode_exhausted", reason=reason, tried=sorted(self.tried))

    def snapshot(self):
        return dict(state=self.state, episode=self.episode, goal=self.goal,
            selected=self.selected, tried=sorted(self.tried), counters=dict(self.counts),
            translation_m=self.translation_m, remaining_drop_m=self.remaining_drop_m,
            event_count=self.event_count, retained_events=len(self.events), events=list(self.events))
