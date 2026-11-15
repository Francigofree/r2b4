"""Explicit bounded worlds and fault schedules; no navigation policy."""
from __future__ import annotations

from copy import deepcopy
import math

from r2b4_orchestration.motion_simulation import motion_scenario, _number
from tools.nav2_navigation_probe import _ray_box

SCHEMA = "R2B4_LOCAL_NAVIGATION_SCENARIO_V1"
FAULTS = {"lidar_drop", "lidar_delay", "imu_drop", "encoder_drop", "command_loss",
          "nav2_loss", "tf_drop", "process_pause", "process_exit", "cpu_load"}
TARGETS = {"slam", "slam_bridge", "bridge", "controller_server", "planner_server",
           "behavior_server", "bt_navigator", "lifecycle_manager"}


def validate(value):
    value = deepcopy(value)
    extra = {"schema", "behavior", "walls", "moving_obstacles", "faults", "expect", "provenance"}
    if not isinstance(value, dict) or value.get("schema") != SCHEMA:
        raise ValueError("expected " + SCHEMA)
    if value.get("behavior", "navigate") not in {"navigate", "roomcruise"}:
        raise ValueError("behavior must be navigate or roomcruise")
    plant = motion_scenario({k: v for k, v in value.items() if k not in extra})
    if plant["v3_overrides"]:
        raise ValueError("use an explicit config overlay, not scenario v3_overrides")
    result = {**plant, "schema": SCHEMA, "behavior": value.get("behavior", "navigate")}
    result["walls"] = []
    for box in _rows(value.get("walls", []), "walls", 64):
        if not isinstance(box, list) or len(box) != 4:
            raise ValueError("walls must be [xmin,ymin,xmax,ymax] rectangles")
        box = [_number(v, "wall", -20, 20) for v in box]
        if box[0] >= box[2] or box[1] >= box[3]:
            raise ValueError("wall rectangle must have positive area")
        result["walls"].append(box)
    result["moving_obstacles"] = []
    for row in _rows(value.get("moving_obstacles", []), "moving_obstacles", 8):
        if not isinstance(row, dict) or set(row) != {"x_m", "y_m", "radius_m", "vx_mps", "vy_mps"}:
            raise ValueError("moving obstacle requires x_m,y_m,radius_m,vx_mps,vy_mps")
        result["moving_obstacles"].append({k: _number(v, k, .02 if k == "radius_m" else -20,
                                                     1 if k == "radius_m" else 20) for k, v in row.items()})
    result["faults"] = []
    for row in _rows(value.get("faults", []), "faults", 32):
        if not isinstance(row, dict) or set(row) - {"kind", "start_s", "duration_s", "delay_s", "target"}:
            raise ValueError("invalid fault fields")
        if row.get("kind") not in FAULTS:
            raise ValueError("unsupported fault kind")
        fault = {"kind": row["kind"], "start_s": _number(row.get("start_s"), "fault.start_s", 0, plant["duration_s"]),
                 "duration_s": _number(row.get("duration_s", .5), "fault.duration_s", .02, plant["duration_s"])}
        if fault["start_s"] + fault["duration_s"] > plant["duration_s"]:
            raise ValueError("fault must end inside scenario duration")
        if row["kind"] == "lidar_delay":
            fault["delay_s"] = _number(row.get("delay_s", .2), "delay_s", .02, 2)
        elif "delay_s" in row:
            raise ValueError("delay_s only applies to lidar_delay")
        if row["kind"] in {"process_pause", "process_exit"}:
            if row.get("target") not in TARGETS:
                raise ValueError("fault target must name an owned simulation process")
            fault["target"] = row["target"]
        elif "target" in row:
            raise ValueError("target only applies to process faults")
        result["faults"].append(fault)
    expected = value.get("expect", {})
    if not isinstance(expected, dict) or set(expected) - {"goal_reached", "minimum_path_m", "maximum_stationary_s"}:
        raise ValueError("invalid expect fields")
    result["expect"] = {"goal_reached": expected.get("goal_reached", result["behavior"] == "navigate"),
                        "minimum_path_m": _number(expected.get("minimum_path_m", 0), "minimum_path_m", 0, 100),
                        "maximum_stationary_s": _number(expected.get("maximum_stationary_s", 10), "maximum_stationary_s", .02, 30)}
    if type(result["expect"]["goal_reached"]) is not bool:
        raise ValueError("goal_reached must be boolean")
    if "provenance" in value:
        result["provenance"] = value["provenance"]
    return result


def _rows(value, name, limit):
    if not isinstance(value, list) or len(value) > limit:
        raise ValueError(f"{name} must be a list of at most {limit} items")
    return value


def builtin(name="room"):
    value = {"schema": SCHEMA, "kind": "differential_drive", "environment_id": name,
             "duration_s": 20., "seed": 7, "goal": {"x_m": 1., "y_m": .0},
             "room": {"x_min_m": -2., "y_min_m": -2., "x_max_m": 4., "y_max_m": 2.}}
    if name == "corridor":
        value.update(room={"x_min_m": -1., "x_max_m": 4., "y_min_m": -.45, "y_max_m": .45})
    elif name == "dead_end":
        value.update(behavior="roomcruise", walls=[[1.5, -.8, 1.7, .8], [-.5, .8, 1.7, 1.0], [-.5, -1., 1.7, -.8]])
    elif name == "moving_obstacle":
        value.update(moving_obstacles=[dict(x_m=.7, y_m=-1., radius_m=.2, vx_mps=0., vy_mps=.1)])
    elif name == "lidar_dropout":
        value.update(faults=[dict(kind="lidar_drop", start_s=3., duration_s=1.)])
    elif name == "explore":
        value.update(behavior="roomcruise", expect={"minimum_path_m": .1, "maximum_stationary_s": 15.})
    elif name != "room":
        raise ValueError("unknown built-in scenario: " + name)
    return validate(value)


BUILTINS = ("room", "corridor", "dead_end", "moving_obstacle", "lidar_dropout", "explore")


class World:
    def __init__(self, scenario, body_bounds):
        self.scenario, self.body_bounds = scenario, body_bounds
        room = scenario["room"]
        self.room = (room["x_min_m"], room["y_min_m"], room["x_max_m"], room["y_max_m"])

    def circles(self, elapsed):
        return [*self.scenario["obstacles"], *[
            {"x_m": row["x_m"] + row["vx_mps"] * elapsed,
             "y_m": row["y_m"] + row["vy_mps"] * elapsed, "radius_m": row["radius_m"]}
            for row in self.scenario["moving_obstacles"]]]

    def ray(self, x, y, angle, elapsed):
        dx, dy = math.cos(angle), math.sin(angle)
        distance = _ray_box(x, y, dx, dy, self.room)
        for box in self.scenario["walls"]:
            distance = min(distance, _ray_box(x, y, dx, dy, box))
        for row in self.circles(elapsed):
            cx, cy = x - row["x_m"], y - row["y_m"]
            projection = cx * dx + cy * dy
            discriminant = projection**2 - (cx*cx + cy*cy - row["radius_m"]**2)
            if discriminant >= 0 and -projection + math.sqrt(discriminant) >= 0:
                distance = min(distance, max(0., -projection - math.sqrt(discriminant)))
        return max(0., distance)

    def clearance(self, x, y, yaw, elapsed):
        rear, front, right, left = self.body_bounds
        c, s = math.cos(yaw), math.sin(yaw)
        polygon = [(x+c*a-s*b, y+s*a+c*b) for a, b in ((rear,right),(front,right),(front,left),(rear,left))]
        distance = min(min(px-self.room[0], self.room[2]-px, py-self.room[1], self.room[3]-py) for px, py in polygon)
        for row in self.circles(elapsed):
            a, b = row["x_m"]-x, row["y_m"]-y
            bx, by = c*a+s*b, -s*a+c*b
            distance = min(distance, math.hypot(max(rear-bx, 0., bx-front), max(right-by, 0., by-left))-row["radius_m"])
        for box in self.scenario["walls"]:
            other = [(box[0],box[1]),(box[2],box[1]),(box[2],box[3]),(box[0],box[3])]
            separated = False
            for ax, ay in ((1.,0.),(0.,1.),(c,s),(-s,c)):
                one, two = [ax*a+ay*b for a,b in polygon], [ax*a+ay*b for a,b in other]
                separated |= max(one) < min(two) or max(two) < min(one)
            if not separated:
                return min(0., distance)
            for points, edges in ((polygon, other), (other, polygon)):
                for point in points:
                    for a, b in zip(edges, edges[1:]+edges[:1]):
                        distance = min(distance, _segment_distance(point, a, b))
        return distance


def _segment_distance(point, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    t = max(0., min(1., ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/(dx*dx+dy*dy)))
    return math.hypot(point[0]-a[0]-t*dx, point[1]-a[1]-t*dy)
