"""Bounded, passive Nav2 diagnostics in the existing ROS edge and observers.

ROS payloads never enter V3's control interpreter. The bridge's existing atomic
status snapshot is a latest-state observation edge, not a reliable ROS bag.
Source times, truncation and missed snapshots remain explicit in capture.
"""
from __future__ import annotations

from collections import OrderedDict, deque
import json
import math
from pathlib import Path
import time

from v3.lidar_geometry import body_clearance_m
from v3.adapters.nav2_navigation import SENSOR_MAX_AGE_NS
from v3.contracts import GLOBAL_FRAME_ID


NAV2_OBSERVATION_TOPIC = "v3.nav2_observation"
NAV2_OBSERVATION_SCHEMA = "R2B4_NAV2_OBSERVATION_V1"
NAV2_SAMPLE_PERIOD_NS = 200_000_000
_RATE_WINDOW_NS = 2_000_000_000
_MAX_STATUS_BYTES = 131_072
_MAX_DIAGNOSTICS = 16
_MAX_VALUES = 8
_MAX_VISITED_CELLS = 65_536


def _age(now, source):
    return now-source if type(source) is int and 0 < source <= now else None


class Nav2Telemetry:
    """ROS-local counters and sampled geometry; no navigation/safety authority."""

    def __init__(self, generation, ros_offset_ns, *, body_bounds_m=None, lidar_xy_m=None):
        self.generation, self.ros_offset_ns = generation, ros_offset_ns
        if body_bounds_m is not None:
            if (len(body_bounds_m) != 4 or lidar_xy_m is None or len(lidar_xy_m) != 2
                    or any(not isinstance(value, (int, float)) or not math.isfinite(value)
                           for value in (*body_bounds_m, *lidar_xy_m))
                    or not -1 <= body_bounds_m[0] < body_bounds_m[1] <= 1
                    or not -1 <= body_bounds_m[2] < body_bounds_m[3] <= 1
                    or any(abs(value) > 1 for value in lidar_xy_m)):
                raise ValueError("invalid bounded robot observation geometry")
        self.body_bounds_m, self.lidar_xy_m = body_bounds_m, lidar_xy_m
        self.sequence = 0
        self.commands = deque(maxlen=256)
        self.command_count = self.command_stops = self.measured_stops = 0
        self._command_moving = self._measured_moving = None
        self.scan = None
        self.odometry = None
        self.errors = deque(maxlen=8)
        self.error_count = 0
        self.diagnostics = OrderedDict()
        self.diagnostics_truncated = 0
        self.visited = set()
        self.coverage_truncated = False
        self.distance_m = 0.0
        self.previous_pose = None
        self.latest = None

    def observe_scan(self, message, measured):
        # Keep only the already validated latest ROS scan. Geometry is inspected
        # once per 5 Hz observation, never on V3's 50 Hz control path.
        self.scan = measured, message

    def observe_odometry(self, message, measured):
        twist = message.twist.twist
        self.odometry = {"measurement_monotonic_ns": measured,
            "v_mps": float(twist.linear.x), "omega_rad_s": float(twist.angular.z)}
        moving = abs(twist.linear.x) > .005 or abs(twist.angular.z) > .02
        self.measured_stops += int(self._measured_moving is True and not moving)
        self._measured_moving = moving

    def observe_command(self, message, measured, received):
        v, omega = message.twist.linear.x, message.twist.angular.z
        if not (0 < measured <= received and math.isfinite(v) and math.isfinite(omega)):
            return
        self.commands.append((measured, received, float(v), float(omega)))
        self.command_count += 1
        moving = abs(v) > .005 or abs(omega) > .02
        self.command_stops += int(self._command_moving is True and not moving)
        self._command_moving = moving

    def record_error(self, now, kind, code=0, message="", goal_id=None):
        self.error_count += 1
        self.errors.append({"measurement_monotonic_ns": now, "kind": str(kind)[:64],
            "code": int(code), "message": str(message)[:192], "goal_id": goal_id})

    def observe_diagnostics(self, message, received):
        source = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec-self.ros_offset_ns
        self.diagnostics_truncated += max(0, len(message.status)-_MAX_DIAGNOSTICS)
        for status in message.status[:_MAX_DIAGNOSTICS]:
            name = status.name[:128]
            self.diagnostics.pop(name, None)
            if len(self.diagnostics) == _MAX_DIAGNOSTICS:
                self.diagnostics.popitem(last=False)
                self.diagnostics_truncated += 1
            truncated = max(0, len(status.values)-_MAX_VALUES)
            self.diagnostics_truncated += truncated
            level = status.level[0] if isinstance(status.level, bytes) else int(status.level)
            self.diagnostics[name] = {"name": name, "level": level,
                "message": status.message[:192], "hardware_id": status.hardware_id[:64],
                "measurement_monotonic_ns": source, "received_monotonic_ns": received,
                "values": [{"key": item.key[:64], "value": item.value[:128]}
                           for item in status.values[:_MAX_VALUES]],
                "values_truncated": truncated}

    def _rate(self, now):
        rows = [row for row in self.commands if 0 <= now-row[1] <= _RATE_WINDOW_NS]
        source_gaps = [b[0]-a[0] for a, b in zip(rows, rows[1:])]
        receipt_gaps = [b[1]-a[1] for a, b in zip(rows, rows[1:])]
        source_duration = rows[-1][0]-rows[0][0] if len(rows) > 1 else 0
        receipt_duration = rows[-1][1]-rows[0][1] if len(rows) > 1 else 0
        latest = self.commands[-1] if self.commands else None
        return {"expected_hz": 20.0, "window_ns": _RATE_WINDOW_NS,
            "sample_count": len(rows), "total_count": self.command_count,
            "source_hz": (len(rows)-1)*1e9/source_duration if source_duration > 0 else None,
            "receipt_hz": (len(rows)-1)*1e9/receipt_duration if receipt_duration > 0 else None,
            "source_max_gap_ns": max(source_gaps, default=None),
            "receipt_max_gap_ns": max(receipt_gaps, default=None),
            "source_nonincreasing_count": sum(gap <= 0 for gap in source_gaps),
            "last_command_age_ns": None if latest is None else _age(now, latest[0]),
            "last_receipt_age_ns": None if latest is None else _age(now, latest[1]),
            "measurement_monotonic_ns": None if latest is None else latest[0],
            "v_mps": None if latest is None else latest[2],
            "omega_rad_s": None if latest is None else latest[3],
            "stop_count": self.command_stops}

    def _walls(self):
        if self.scan is None:
            return None
        measured, scan = self.scan
        result = {"measurement_monotonic_ns": measured,
            "basis": "physical_body_outline" if self.body_bounds_m is not None else "laser_origin",
            "meaning": "shortest_LiDAR_observed_return_distance_not_fitted_wall",
            "front_m": None, "rear_m": None, "left_m": None, "right_m": None,
            "minimum_m": None, "valid_return_count": 0}
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max:
                continue
            angle = scan.angle_min+index*scan.angle_increment
            x, y = distance*math.cos(angle), distance*math.sin(angle)
            if self.body_bounds_m is None:
                clearance = distance
            else:
                x += self.lidar_xy_m[0]
                y += self.lidar_xy_m[1]
                clearance = body_clearance_m(x, y, self.body_bounds_m)
            side = ("front_m" if x >= 0 else "rear_m") if abs(x) >= abs(y) else (
                "left_m" if y >= 0 else "right_m")
            for key in (side, "minimum_m"):
                result[key] = clearance if result[key] is None else min(result[key], clearance)
            result["valid_return_count"] += 1
        return result

    def _coverage(self, robot_pose, measured, now):
        resolution = .10
        age = _age(now, measured)
        if (robot_pose is not None and self.body_bounds_m is not None
                and len(robot_pose) == 3 and all(math.isfinite(value) for value in robot_pose)
                and age is not None and age <= SENSOR_MAX_AGE_NS
                and (self.previous_pose is None or measured > self.previous_pose[2])):
            x, y, yaw = robot_pose
            previous = self.previous_pose
            if previous is not None:
                displacement = math.hypot(x-previous[0], y-previous[1])
                elapsed = (measured-previous[2])/1e9
                if 0 < elapsed <= 1 and displacement <= max(.10, elapsed*1.0):
                    self.distance_m += displacement
            self.previous_pose = x, y, measured
            rear, front, right, left = self.body_bounds_m
            cosine, sine = math.cos(yaw), math.sin(yaw)
            radius = math.hypot(max(abs(rear), abs(front)), max(abs(right), abs(left)))
            # At 10 cm resolution the physical 36 cm body needs <= 81 centers.
            # Bounds originate in the resolved robot geometry, not ROS padding.
            for ix in range(math.floor((x-radius)/resolution), math.floor((x+radius)/resolution)+1):
                for iy in range(math.floor((y-radius)/resolution), math.floor((y+radius)/resolution)+1):
                    dx, dy = (ix+.5)*resolution-x, (iy+.5)*resolution-y
                    local_x, local_y = cosine*dx+sine*dy, -sine*dx+cosine*dy
                    if rear <= local_x <= front and right <= local_y <= left:
                        if len(self.visited) < _MAX_VISITED_CELLS:
                            self.visited.add((ix, iy))
                        elif (ix, iy) not in self.visited:
                            self.coverage_truncated = True
        return {"frame_id": GLOBAL_FRAME_ID, "measurement_monotonic_ns": measured,
            "pose": robot_pose, "distance_m": self.distance_m,
            "footprint_raster_area_m2": len(self.visited)*resolution*resolution,
            "resolution_m": resolution, "visited_cell_count": len(self.visited),
            "truncated": self.coverage_truncated, "basis": "localized_body_footprint_cell_centers",
            "available": (self.body_bounds_m is not None and robot_pose is not None
                and age is not None and age <= SENSOR_MAX_AGE_NS)}

    def snapshot(self, now, *, scan_ns, odom_ns, tf_ns, active, robot_pose, state):
        self.sequence += 1
        diagnostics = []
        for value in self.diagnostics.values():
            row = dict(value)
            row["source_age_ns"] = _age(now, row["measurement_monotonic_ns"])
            row["stale"] = row["source_age_ns"] is None or row["source_age_ns"] > 3_000_000_000
            diagnostics.append(row)
        self.latest = {"schema": NAV2_OBSERVATION_SCHEMA, "event_type": "nav2_observation",
            "worker_generation": self.generation, "sample_sequence": self.sequence,
            "measurement_monotonic_ns": now, "sample_period_ns": NAV2_SAMPLE_PERIOD_NS,
            "state": state.get("state"), "active": active,
            "lifecycle_status": state.get("lifecycle_status"),
            "mission_id": state.get("mission_id"), "selected_goal": state.get("selected_goal"),
            "child_goal_id": state.get("child_goal_id"),
            "sensor_age_ns": {"scan": _age(now, scan_ns), "odometry": _age(now, odom_ns), "tf": _age(now, tf_ns)},
            "sensor_measurement_ns": {"scan": scan_ns, "odometry": odom_ns, "tf": tf_ns},
            "controller": self._rate(now), "odometry": self.odometry,
            "measured_stop_count": self.measured_stops, "wall_clearance": self._walls(),
            "coverage": self._coverage(robot_pose, tf_ns, now),
            "navigation_error_count": self.error_count, "navigation_errors": list(self.errors),
            "diagnostics": diagnostics, "diagnostics_truncated_count": self.diagnostics_truncated,
            "delivery_semantics": "bounded_latest_snapshot"}
        return self.latest


class Nav2Diagnostics(Nav2Telemetry):
    """Use the upstream updater's 1 Hz timer; evidence remains available at 5 Hz."""

    def __init__(self, node, generation, ros_offset_ns, **geometry):
        from diagnostic_updater import Updater
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
        super().__init__(generation, ros_offset_ns, **geometry)
        self._levels = (DiagnosticStatus.OK, DiagnosticStatus.WARN, DiagnosticStatus.ERROR, DiagnosticStatus.STALE)
        self.subscription = node.create_subscription(DiagnosticArray, "/diagnostics",
            lambda message: self.observe_diagnostics(message, time.monotonic_ns()), 4)
        self.updater = Updater(node, period=1.0)
        self.updater.setHardwareID("r2b4_nav2_edge")
        for name in ("components", "controller_frequency", "sensor_tf_freshness", "navigation_errors"):
            self.updater.add(name, lambda stat, kind=name: self._diagnostic(stat, kind))

    def _diagnostic(self, stat, kind):
        row = self.latest
        if row is None:
            stat.summary(self._levels[3], "Waiting for first bridge observation")
            return stat
        now = time.monotonic_ns()
        if kind == "components":
            stat.summary(self._levels[0 if row["state"] == "RUNNING" else 1], str(row["state"]))
            stat.add("active", str(row["active"]))
            stat.add("worker_generation", str(self.generation))
        elif kind == "controller_frequency":
            rate = self._rate(now)
            required = row["active"]
            healthy = (rate["source_hz"] is not None and 16 <= rate["source_hz"] <= 24
                and rate["last_command_age_ns"] is not None and rate["last_command_age_ns"] <= 150_000_000)
            stat.summary(self._levels[0 if healthy or not required else 1],
                "Controller command stream measured" if required else "No active navigation action")
            for key in ("source_hz", "receipt_hz", "source_max_gap_ns", "receipt_max_gap_ns", "last_command_age_ns"):
                stat.add(key, str(rate[key]))
        elif kind == "sensor_tf_freshness":
            ages = {key: _age(now, value) for key, value in row["sensor_measurement_ns"].items()}
            fresh = all(age is not None and age <= SENSOR_MAX_AGE_NS for age in ages.values())
            stat.summary(self._levels[0 if fresh else 1], "Fresh scan, odometry and TF" if fresh else "Missing or stale sensor/TF")
            for key, age in ages.items():
                stat.add(key+"_age_ns", str(age))
        else:
            recent = [error for error in self.errors if 0 <= now-error["measurement_monotonic_ns"] <= 5_000_000_000]
            stat.summary(self._levels[1 if recent else 0], "Recent navigation failure" if recent else "No recent navigation failure")
            stat.add("error_count", str(self.error_count))
            if self.errors:
                stat.add("last_error", self.errors[-1]["message"] or self.errors[-1]["kind"])
        return stat


class Nav2ObservationSampler:
    """Observation-side latest snapshot reader, polling metadata only at 20 Hz."""

    def __init__(self, project_root):
        self.path = Path(project_root)/"runtime"/"nav2_planner"/"bridge_status.json"
        self._next_poll_ns = 0
        self._identity = None
        self._previous = None
        self._last_unavailable = None
        self._last_source_ns = None

    def sample(self, monotonic_ns=None):
        now = time.monotonic_ns() if monotonic_ns is None else monotonic_ns
        if now < self._next_poll_ns:
            return None
        self._next_poll_ns = now+50_000_000
        try:
            info = self.path.stat()
            identity = info.st_ino, info.st_mtime_ns, info.st_size
            if identity == self._identity:
                if self._last_source_ns is not None and now-self._last_source_ns > 1_000_000_000:
                    raise ValueError("NAV2_OBSERVATION_STALE_OR_INVALID")
                return None
            with self.path.open("rb") as stream:
                encoded = stream.read(_MAX_STATUS_BYTES+1)
            if len(encoded) > _MAX_STATUS_BYTES:
                raise ValueError("NAV2_STATUS_EXCEEDS_BOUND")
            state = json.loads(encoded)
            if not isinstance(state, dict):
                raise ValueError("NAV2_STATUS_INVALID")
            row = state.get("nav2_observation")
            if not isinstance(row, dict) or row.get("schema") != NAV2_OBSERVATION_SCHEMA:
                self._identity = identity
                return None  # Older or planner-only edge has no requested stream.
            source = row.get("measurement_monotonic_ns")
            sequence, generation = row.get("sample_sequence"), row.get("worker_generation")
            if (type(source) is not int or not 0 <= now-source <= 1_000_000_000
                    or type(sequence) is not int or sequence <= 0
                    or type(generation) is not int or generation <= 0):
                raise ValueError("NAV2_OBSERVATION_STALE_OR_INVALID")
            previous = self._previous
            if previous == (generation, sequence):
                self._identity = identity
                return None
            skipped = max(0, sequence-previous[1]-1) if previous and generation == previous[0] else 0
            self._previous = generation, sequence
            self._last_source_ns = source
            self._identity = identity
            row = dict(row, observer_received_monotonic_ns=now, snapshots_superseded=skipped,
                available=True, recovered_after_unavailable=self._last_unavailable is not None)
            self._last_unavailable = None
            return row
        except FileNotFoundError:
            return None
        except (OSError, ValueError, TypeError, RecursionError) as exc:
            reason = str(exc)[:128]
            if self._last_unavailable == reason:
                return None
            self._last_unavailable = reason
            return {"schema": NAV2_OBSERVATION_SCHEMA, "event_type": "nav2_observation_unavailable",
                "measurement_monotonic_ns": now, "reason": reason, "available": False}
