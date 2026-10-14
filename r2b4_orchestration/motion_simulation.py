"""Bounded differential-drive experiments at the canonical offline V3 edge.

This is an intentionally modest plant: inverse calibrated PWM curves, first
order wheel response, traction, raycast room/circles and synthetic odometry.
It measures simulation ground truth independently of L3 and never opens a
device. It is useful for experiments, not a claim of physical calibration.
"""
from __future__ import annotations

from collections.abc import Mapping
from dataclasses import replace
import hashlib
import json
import math
import random
import threading
from types import SimpleNamespace


MOTION_ACTIONS = frozenset({"v3.command.move_relative", "v3.command.turn_by", "v3.command.navigate"})
_OVERRIDES = {"wheel_pi": {"kp", "ki", "integrator_limit"},
              "motion_realization": {"distance_gain", "heading_gain", "cross_track_gain", "angular_velocity_gain"}}


def _number(value, name, low=-float("inf"), high=float("inf")):
    if type(value) not in {int, float} or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{name} must be finite within [{low}, {high}]")
    return float(value)


def motion_scenario(value):
    fields = {"kind", "environment_id", "seed", "duration_s", "goal", "position_tolerance_m",
              "yaw_tolerance_rad", "plant", "room", "obstacles", "v3_overrides"}
    if not isinstance(value, Mapping) or set(value) - fields:
        raise ValueError("unknown differential_drive scenario fields")
    identity = value.get("environment_id", "open_room")
    if not isinstance(identity, str) or not 1 <= len(identity) <= 128:
        raise ValueError("environment_id must be bounded nonempty text")
    seed = value.get("seed", 0)
    if type(seed) is not int or not 0 <= seed <= 2**31 - 1:
        raise ValueError("seed must be an integer in [0, 2**31-1]")
    goal = value.get("goal", {"x_m": .8, "y_m": 0.})
    if not isinstance(goal, Mapping) or set(goal) - {"x_m", "y_m", "yaw_rad"} or not {"x_m", "y_m"} <= goal.keys():
        raise ValueError("goal requires x_m, y_m and optional yaw_rad in R2B4_ODOM_LOCAL")
    goal = {name: _number(number, "goal." + name, -10., 10.) for name, number in goal.items()}
    if "yaw_rad" in goal:
        goal["yaw_rad"] = _wrap(goal["yaw_rad"])
    plant = value.get("plant", {})
    defaults = {"response_tau_s": .08, "left_gain": 1., "right_gain": 1.,
                "traction": 1., "encoder_noise_mps": 0.}
    limits = {"response_tau_s": (0., .5), "left_gain": (.5, 1.5), "right_gain": (.5, 1.5),
              "traction": (0., 1.), "encoder_noise_mps": (0., .03)}
    if not isinstance(plant, Mapping) or set(plant) - defaults.keys():
        raise ValueError("unknown plant parameters")
    plant = {key: _number(plant.get(key, default), "plant." + key, *limits[key])
             for key, default in defaults.items()}
    room = value.get("room", {})
    bounds = {"x_min_m": -2., "x_max_m": 3., "y_min_m": -2., "y_max_m": 2.}
    if not isinstance(room, Mapping) or set(room) - bounds.keys():
        raise ValueError("room requires x_min_m, x_max_m, y_min_m and y_max_m")
    room = {key: _number(room.get(key, default), "room." + key, -20., 20.) for key, default in bounds.items()}
    if not room["x_min_m"] < 0 < room["x_max_m"] or not room["y_min_m"] < 0 < room["y_max_m"]:
        raise ValueError("room must contain the initial robot origin")
    obstacles = value.get("obstacles", [])
    if not isinstance(obstacles, list) or len(obstacles) > 8:
        raise ValueError("obstacles must contain at most eight circles")
    circles = []
    for row in obstacles:
        if not isinstance(row, Mapping) or set(row) != {"x_m", "y_m", "radius_m"}:
            raise ValueError("obstacle requires x_m, y_m and radius_m")
        circles.append({key: _number(number, "obstacle." + key, .01 if key == "radius_m" else -20.,
                                     2. if key == "radius_m" else 20.) for key, number in row.items()})
    overrides = value.get("v3_overrides", {})
    if not isinstance(overrides, Mapping) or set(overrides) - _OVERRIDES.keys():
        raise ValueError("only simulation wheel_pi and motion_realization gain overrides are supported")
    tuning = {}
    for owner, rows in overrides.items():
        if not isinstance(rows, Mapping) or set(rows) - _OVERRIDES[owner] or not rows:
            raise ValueError("unsupported simulation tuning field: " + owner)
        tuning[owner] = {key: _number(number, owner + "." + key, 0., 20.) for key, number in rows.items()}
    return {"kind": "differential_drive", "environment_id": identity, "seed": seed,
            "duration_s": _number(value.get("duration_s", 12.), "duration_s", .1, 30.), "goal": goal,
            "position_tolerance_m": _number(value.get("position_tolerance_m", .08), "position_tolerance_m", .005, .5),
            "yaw_tolerance_rad": _number(value.get("yaw_tolerance_rad", .12), "yaw_tolerance_rad", .005, .5),
            "plant": plant, "room": room, "obstacles": circles, "v3_overrides": tuning}


def simulation_control_config(config, scenario):
    """The existing immutable config owners validate each offline gain overlay."""
    changes = {owner: replace(getattr(config, owner), **fields)
               for owner, fields in scenario["v3_overrides"].items()}
    return replace(config, **changes) if changes else config


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _inverse_curve(curve, output):
    """Invert the actual monotone calibration, including its maintenance floor."""
    magnitude = abs(output)
    floor = min(curve.maintenance_output, curve.points[0].normalized_output)
    points = [(floor, 0.)] + [(point.normalized_output, point.speed_mps) for point in curve.points]
    if magnitude <= floor:
        return 0.
    for (low, low_speed), (high, high_speed) in zip(points, points[1:]):
        if magnitude <= high:
            speed = high_speed if high == low else low_speed + (magnitude-low)*(high_speed-low_speed)/(high-low)
            return math.copysign(speed, output)
    return math.copysign(points[-1][1], output)


class DifferentialDriveExperiment:
    """One finite plant owner; command input uses the production mailbox edge."""

    dt_ns = 20_000_000

    def __init__(self, scenario, config, composition, writer, directory, start_ns, observe, event):
        from v3.adapters.resident_command import AtomicResidentCommandGateway, ResidentCommandClient, ResidentCommandMailboxConfig
        self.scenario, self.config, self.composition, self.writer = scenario, config, composition, writer
        self.now_ns, self.start_ns, self.tick = start_ns, start_ns, 0
        self.observe, self.event = observe, event
        policy = ResidentCommandMailboxConfig(directory / "command.json", maximum_linear_speed_mps=.5,
                                              maximum_angular_speed_rad_s=1.2)
        self.client = ResidentCommandClient(policy, monotonic_ns=lambda: self.now_ns)
        self.gateway = AtomicResidentCommandGateway(policy, monotonic_ns=lambda: self.now_ns)
        self.random = random.Random(scenario["seed"])
        self._lock = threading.RLock()
        self._cancelled = threading.Event()
        self.x = self.y = self.yaw = self.left = self.right = self.left_distance = self.right_distance = 0.
        self.output = (0., 0.)
        self.scan_pose = (0., 0., 0.)
        self.scan_time_ns = start_ns
        self.scan_samples = ()
        self.commands = []
        self.status = {}
        self.first_boundary = None
        self.fault_layer = None
        self.path_length = self.tracking_squared = self.max_jerk = 0.
        self.previous_v = self.previous_acceleration = 0.
        self.tracking_count = self.safety_stop_count = self.collision_count = 0
        self.radius = math.hypot(config.navigation.footprint_length_m, config.navigation.footprint_width_m) / 2
        if self._collision():
            raise ValueError("scenario initially intersects the robot footprint")
        self.client.publish_stop("simulation.initial", ttl_ns=200_000_000)

    def _collision(self):
        room, radius = self.scenario["room"], self.radius
        return (not room["x_min_m"]+radius < self.x < room["x_max_m"]-radius
                or not room["y_min_m"]+radius < self.y < room["y_max_m"]-radius
                or any(math.hypot(self.x-row["x_m"], self.y-row["y_m"]) <= radius+row["radius_m"]
                       for row in self.scenario["obstacles"]))

    def _advance(self):
        dt, plant = self.dt_ns / 1e9, self.scenario["plant"]
        factor = 1. if plant["response_tau_s"] == 0 else -math.expm1(-dt / plant["response_tau_s"])
        for index, side in enumerate(("left", "right")):
            output = self.output[index]
            name = side + ("_forward" if output >= 0 else "_reverse")
            curve = next(row for row in self.config.speed_map.curves if row.name == name)
            target = _inverse_curve(curve, output) * plant[side + "_gain"]
            speed = getattr(self, side) + factor * (target-getattr(self, side))
            setattr(self, side, speed)
            setattr(self, side + "_distance", getattr(self, side + "_distance") + speed*dt)
        v = (self.left+self.right)*plant["traction"] / 2
        omega = (self.right-self.left)*plant["traction"] / self.config.chassis_control.track_width_m
        delta_yaw = omega*dt
        distance = v*dt if abs(delta_yaw) < 1e-12 else v*dt*math.sin(delta_yaw/2)/(delta_yaw/2)
        self.x += distance*math.cos(self.yaw+delta_yaw/2)
        self.y += distance*math.sin(self.yaw+delta_yaw/2)
        self.yaw = _wrap(self.yaw+delta_yaw)
        self.path_length += abs(distance)
        acceleration = (v-self.previous_v)/dt
        self.max_jerk = max(self.max_jerk, abs(acceleration-self.previous_acceleration)/dt)
        self.previous_v, self.previous_acceleration = v, acceleration
        self.now_ns += self.dt_ns

    def _scan(self, sample):
        room, ranges = self.scenario["room"], []
        for index in range(48):
            bearing = index*math.tau/48
            dx, dy = math.cos(bearing+self.yaw), math.sin(bearing+self.yaw)
            distances = []
            if abs(dx) > 1e-12:
                distances.append(((room["x_max_m"] if dx > 0 else room["x_min_m"])-self.x)/dx)
            if abs(dy) > 1e-12:
                distances.append(((room["y_max_m"] if dy > 0 else room["y_min_m"])-self.y)/dy)
            for row in self.scenario["obstacles"]:
                cx, cy = self.x-row["x_m"], self.y-row["y_m"]
                projection = cx*dx+cy*dy
                discriminant = projection**2 - (cx**2+cy**2-row["radius_m"]**2)
                if discriminant >= 0:
                    distance = -projection-math.sqrt(discriminant)
                    if distance >= 0:
                        distances.append(distance)
            ranges.append((bearing, max(0., min(distances))))
        fields = {"frame_id": "ROBOT_BASE", "point_count": len(ranges)}
        sectors = {name: [] for name in ("front", "left", "rear", "right")}
        for index, (bearing, distance) in enumerate(ranges):
            fields.update({f"point_{index:03d}_x_m": distance*math.cos(bearing),
                           f"point_{index:03d}_y_m": distance*math.sin(bearing), f"point_{index:03d}_quality": 20})
            sectors[("front", "left", "rear", "right")[int((bearing+math.pi/4) % math.tau / (math.pi/2))]].append(distance)
        safety = {f"{name}_{key}": number for name, rows in sectors.items()
                  for key, number in (("clearance_m", max(0., min(rows)-self.radius)), ("observation_count", len(rows)))}
        samples = [sample("lidar_health", age_ns=0, point_count=len(ranges)),
                   sample("lidar_local_points", **fields), sample("lidar_safety_clearance", age_ns=0, **safety)]
        if self.tick:
            x, y, yaw = self.scan_pose
            dx, dy = self.x-x, self.y-y
            samples.append(sample("lidar_relative_motion", start_ns=self.scan_time_ns,
                dx_m=dx*math.cos(yaw)+dy*math.sin(yaw), dy_m=-dx*math.sin(yaw)+dy*math.cos(yaw),
                dyaw_rad=_wrap(self.yaw-yaw), rmse_m=.001, observability=1.))
        self.scan_pose, self.scan_time_ns = (self.x, self.y, self.yaw), self.now_ns
        from v3.adapters.live_lidar import LidarPointReading
        raw = tuple(LidarPointReading((-math.degrees(bearing)) % 360., distance, 20) for bearing, distance in ranges)
        return tuple(samples), SimpleNamespace(raw_scan_id=self.tick//5+1, raw_scan_timestamp=self.now_ns/1e9,
            scan_start_monotonic_ns=self.now_ns, scan_end_monotonic_ns=self.now_ns,
            measurement_monotonic_ns=self.now_ns, health="OK", raw_scan=raw, summary={"source": "simulation"})

    def step(self, active=False):
        from v3.contracts import DataField, DeviceHealth, DeviceHealthState, DeviceSample, LifecycleState, RawDeviceBatch, TickContext
        from v3.engine import TickInputs
        from v3.execution import ExecutionRecord
        from v3.resident_status import _tick_status
        with self._lock:
            if self.tick:
                self._advance()
            context = TickContext(self.tick, self.now_ns)
            noise = self.scenario["plant"]["encoder_noise_mps"]
            def sample(device, kind, **values):
                return DeviceSample(device, kind, self.tick+1, self.now_ns,
                                    tuple(DataField(key, value) for key, value in values.items()))
            samples = [sample("ENCODER", "wheel_velocity",
                left_mps=self.left+self.random.uniform(-noise, noise), right_mps=self.right+self.random.uniform(-noise, noise),
                raw_left_distance_m=self.left_distance, raw_right_distance_m=self.right_distance,
                left_counter_running=True, right_counter_running=True, trust=1.,
                measurement_stale=False, measurement_timing_valid=True),
                sample("IMU", "ekf_heading", yaw_rad=self.yaw,
                       omega_rad_s=(self.right-self.left)*self.scenario["plant"]["traction"]/self.config.chassis_control.track_width_m,
                       confidence=1., omega_confidence=1.)]
            scan = None
            if self.tick % 5 == 0:
                self.scan_samples, scan = self._scan(lambda kind, **values: DeviceSample("RPLIDAR_C1", kind,
                    self.tick//5+1, self.now_ns, tuple(DataField(key, value) for key, value in values.items())))
            samples.extend(self.scan_samples)
            health = tuple(DeviceHealth(name, DeviceHealthState.OK) for name in
                           sorted(set(self.config.critical_device_ids or ()) | {"ENCODER", "IMU", "RPLIDAR_C1"}))
            inputs = self.composition.close_inputs(TickInputs(context, RawDeviceBatch(context, tuple(samples), health),
                self.gateway.snapshot(context), LifecycleState.ACTIVE if active else LifecycleState.IDLE))
            result = self.composition.run_tick(inputs)
            self.composition.dispatch_pending_planner_request(self.now_ns)
            self.status = {**_tick_status(result, ready_for_active=True), "source": "simulation",
                           "clock_domain": "simulation_monotonic", "live_freshness": False}
            self.output = (result.final_actuation.left_output, result.final_actuation.right_output)
            layers = {row.layer: row.output for row in result.trace.layers}
            allowed = layers.get("L9")
            if active and allowed is not None:
                # Same interval as the command that generated current motion:
                # prior L9 reference is compared to the next physical sample.
                reference = getattr(self, "_previous_reference", None)
                if reference is not None:
                    self.tracking_squared += (self.previous_v-reference)**2
                    self.tracking_count += 1
                self._previous_reference = allowed.allowed_v_mps
            if result.trace.fault_layer is not None:
                self.fault_layer = self.fault_layer or result.trace.fault_layer
                self.first_boundary = self.first_boundary or {"tick_id": self.tick, "monotonic_ns": self.now_ns,
                    "layer": self.fault_layer, "reason": result.final_actuation.reason}
            requested_motion = allowed is not None and abs(allowed.allowed_v_mps)+abs(allowed.allowed_omega_rad_s) > 1e-9
            if active and requested_motion and result.final_actuation.safety_decision.value != "ALLOW":
                self.safety_stop_count += 1
                if self.first_boundary is None and result.final_actuation.reason not in {"LIFECYCLE_INACTIVE", "ZERO_OUTPUT"}:
                    self.first_boundary = {"tick_id": self.tick, "monotonic_ns": self.now_ns,
                        "layer": result.trace.fault_layer or "L12", "reason": result.final_actuation.reason,
                        "left_output": self.output[0], "right_output": self.output[1]}
            self.observe(ExecutionRecord(inputs, result), scan)
            self.tick += 1
            if self._collision() and not self.collision_count:
                self.collision_count = 1
                self.first_boundary = self.first_boundary or {"tick_id": context.tick_id, "monotonic_ns": self.now_ns,
                    "layer": "plant", "reason": "FOOTPRINT_COLLISION", "x_m": self.x, "y_m": self.y}
            return result

    def execute(self, action, parameters):
        from v3.action_catalog import ACTION_CATALOG
        from v3.finite_navigation import _RELATIVE_COMPLETION_ERROR_FRACTION, _pose_distance, _pose_angle
        params = dict(parameters)
        for field in ("session_owner_pid", "capture", "capture_mode", "capture_hz", "session_watchdog_s"):
            params.pop(field, None)
        finite_timeout = params.pop("finite_timeout_s", self.scenario["duration_s"])
        timeout = _number(finite_timeout, "finite_timeout_s", .01, 30.)
        if action == "v3.command.navigate" and params.pop("wait_for_completion", False) is not True:
            raise ValueError("simulation navigate requires wait_for_completion=True")
        descriptor = ACTION_CATALOG[action]
        if set(params) - {row.name for row in descriptor.parameters}:
            raise ValueError("unknown finite simulation action parameters")
        for row in descriptor.parameters:
            if row.required and row.name not in params:
                raise ValueError("missing action parameter: " + row.name)
        if len(self.commands) >= 16:
            raise RuntimeError("SIMULATION_COMMAND_LIMIT")
        if self.fault_layer:
            raise RuntimeError("SIMULATION_FAULT_REQUIRES_NEW_TRIAL")
        pose = self.status["estimate"]["local_pose"]
        start_pose = {key: pose[key] for key in ("frame_id", "x_m", "y_m", "yaw_rad")}
        yaw = start_pose["yaw_rad"]
        if action == "v3.command.navigate":
            if params.pop("frame_id", "R2B4_ODOM_LOCAL") != "R2B4_ODOM_LOCAL":
                raise ValueError("simulation supports local navigation only")
            target = {key: _number(params[key], key) for key in ("x_m", "y_m")}
            if "yaw_rad" in params:
                target["yaw_rad"] = _wrap(_number(params["yaw_rad"], "yaw_rad"))
        else:
            forward = _number(params.get("forward_m", 0.), "forward_m")
            left = _number(params.get("left_m", 0.), "left_m")
            target = {"x_m": start_pose["x_m"]+forward*math.cos(yaw)-left*math.sin(yaw),
                      "y_m": start_pose["y_m"]+forward*math.sin(yaw)+left*math.cos(yaw)}
            if action == "v3.command.turn_by":
                target["yaw_rad"] = _wrap(yaw+math.radians(_number(params["angle_deg"], "angle_deg", -180., 180.)))
            elif "final_yaw_rad" in params:
                target["yaw_rad"] = _wrap(yaw+_number(params["final_yaw_rad"], "final_yaw_rad"))
        target["frame_id"] = start_pose["frame_id"]
        max_v = _number(params.get("max_v_mps", .2), "max_v_mps", .01, .5)
        max_omega = _number(params.get("max_omega_rad_s", min(1.2, self.config.navigation.wheel_limits.target_center_spin_rad_s)), "max_omega_rad_s", .01, 1.2)
        goal_tolerance = self.config.mission.default_constraints.goal_tolerance_m
        yaw_tolerance = self.config.mission.default_constraints.yaw_tolerance_rad
        if action != "v3.command.navigate":
            distance, angle = _pose_distance(start_pose, target), _pose_angle(start_pose, target)
            if distance is not None and distance > 0:
                goal_tolerance = min(goal_tolerance, distance*_RELATIVE_COMPLETION_ERROR_FRACTION)
            if angle is not None and abs(angle) > 0:
                yaw_tolerance = min(yaw_tolerance, abs(angle)*_RELATIVE_COMPLETION_ERROR_FRACTION)
        command_id = f"simulation.motion.{len(self.commands)+1}"
        started = self.now_ns
        start_tick = self.tick
        deadline = min(started+round(timeout*1e9), self.start_ns+round(self.scenario["duration_s"]*1e9))
        self._cancelled.clear()
        reason = "SIMULATION_TIME_BUDGET"
        while self.now_ns < deadline and not self._cancelled.is_set():
            with self._lock:
                if self._cancelled.is_set():
                    break
                self.client.publish_navigate(command_id, **target,
                    max_v_mps=max_v, max_omega_rad_s=max_omega, ttl_ns=200_000_000,
                    goal_tolerance_m=goal_tolerance, yaw_tolerance_rad=yaw_tolerance)
                result = self.step(active=True)
            navigation = self.status.get("navigation") or {}
            if self.collision_count:
                reason = "FOOTPRINT_COLLISION"
                break
            if result.trace.fault_layer is not None:
                reason = "V3_FAULT:" + result.trace.fault_layer
                break
            if navigation.get("status") == "COMPLETE":
                reason = "GOAL_REACHED"
                break
        if self._cancelled.is_set():
            reason = "CANCELLED"
        self.stop()
        final = self.status["estimate"]["local_pose"]
        result = {"status": "COMPLETED" if reason == "GOAL_REACHED" else "FAILED", "reason": reason,
            "command_id": command_id, "mission_id": "mission-"+command_id, "source": "simulation",
            "physical_output": False, "elapsed_s": (self.now_ns-started)/1e9,
            "request_measurement_monotonic_ns": started, "completion_status_monotonic_ns": self.now_ns,
            "request_tick_id": start_tick, "clock_domain": "simulation_monotonic",
            "start_pose": start_pose, "target_pose": target,
            "final_pose": {key: final[key] for key in ("frame_id", "x_m", "y_m", "yaw_rad")}}
        self.commands.append({"action": action, "parameters": dict(parameters), **result})
        self.event("simulation_motion_completed", result)
        return result

    def stop(self):
        self._cancelled.set()
        with self._lock:
            self.client.publish_stop("simulation.stop", ttl_ns=200_000_000)
            self.step()
        return {"status": "STOPPED", "source": "simulation", "physical_output": False}

    def settle(self):
        self.stop()
        # Brake/coast behavior remains an explicit plant assumption. Observe it
        # rather than teleporting the plant to rest when L12 emits zero output.
        ticks = max(1, math.ceil(6*self.scenario["plant"]["response_tau_s"] / (self.dt_ns/1e9)))
        for _ in range(ticks):
            self.client.publish_stop("simulation.settle", ttl_ns=200_000_000)
            self.step()

    def performance(self):
        goal = self.scenario["goal"]
        metrics = {"goal_position_error_m": math.hypot(self.x-goal["x_m"], self.y-goal["y_m"]),
            "goal_yaw_error_rad": abs(_wrap(self.yaw-goal["yaw_rad"])) if "yaw_rad" in goal else None,
            "elapsed_s": (self.now_ns-self.start_ns)/1e9, "path_length_m": self.path_length,
            "tracking_error_rms_mps": math.sqrt(self.tracking_squared/self.tracking_count) if self.tracking_count else None,
            "max_abs_jerk_mps3": self.max_jerk, "collision_count": self.collision_count,
            "safety_stop_count": self.safety_stop_count, "command_count": len(self.commands)}
        last_target = self.commands[-1]["target_pose"] if self.commands else None
        target_error = math.hypot(last_target["x_m"]-goal["x_m"], last_target["y_m"]-goal["y_m"]) if last_target else None
        passed = (metrics["goal_position_error_m"] <= self.scenario["position_tolerance_m"]
                  and (metrics["goal_yaw_error_rad"] is None or metrics["goal_yaw_error_rad"] <= self.scenario["yaw_tolerance_rad"])
                  and not self.collision_count and not self.fault_layer)
        if passed:
            owner, reason = "none", "INDEPENDENT_SIMULATED_GOAL_REACHED"
        elif not self.commands:
            owner, reason = "skill_logic", "NO_MOTION_REQUESTED"
        elif self.fault_layer:
            owner, reason = "V3_or_measurement", "FIRST_NATIVE_FAULT_REQUIRES_BOUNDARY_INSPECTION"
        elif self.collision_count:
            owner, reason = "environment_or_V3", "FOOTPRINT_COLLISION"
        elif target_error is not None and target_error > self.scenario["position_tolerance_m"] and all(row["status"] == "COMPLETED" for row in self.commands):
            owner, reason = "skill_logic", "COMPLETED_REQUESTED_TARGET_DIFFERS_FROM_INDEPENDENT_GOAL"
            self.first_boundary = self.first_boundary or {"layer": "skill_command", "reason": reason,
                "command_id": self.commands[-1]["command_id"], "requested_target": last_target,
                "independent_goal": goal, "target_error_m": target_error,
                "tick_id": self.commands[-1]["request_tick_id"],
                "monotonic_ns": self.commands[-1]["request_measurement_monotonic_ns"]}
        elif all(row["status"] == "COMPLETED" for row in self.commands):
            owner, reason = "skill_logic_or_state_estimation", "COMPLETED_COMMANDS_DID_NOT_REACH_INDEPENDENT_GOAL"
        else:
            owner, reason = "V3_or_plant_or_skill_limits", "COMMAND_FAILED_BEFORE_INDEPENDENT_GOAL"
        result = {"metrics": metrics, "ground_truth": {"x_m": self.x, "y_m": self.y, "yaw_rad": self.yaw,
            "measurement_monotonic_ns": self.now_ns, "clock_domain": "simulation_monotonic", "sequence": self.tick-1,
            "source": "differential_drive_plant", "environment_id": self.scenario["environment_id"], "seed": self.scenario["seed"]},
            "first_boundary": self.first_boundary, "commands": self.commands,
            "diagnosis": {"likely_owner": owner, "reason": reason, "inference": True,
                          "causal_proof": False}, "goal_reached": passed,
            "scope": "Synthetic ground truth; 50 Hz native V3 and calibrated PWM plant, no live physical evidence."}
        result["next_experiment"] = next_motion_experiment(result, self.scenario)
        return result


def next_motion_experiment(performance, scenario):
    """Choose a small contrast that can distinguish the observed failure owners."""
    metrics, diagnosis = performance.get("metrics", {}), performance.get("diagnosis", {})
    if diagnosis.get("likely_owner") == "skill_logic":
        return {"kind": "skill_goal_request", "factor": "skill_parameters_or_goal_selection",
                "reason": "Verify the skill branch and requested target against the independent goal before changing V3 gains.",
                "discriminating_metrics": ["command_count", "goal_position_error_m"]}
    if metrics.get("collision_count", 0) or metrics.get("safety_stop_count", 0):
        return {"kind": "paired_one_factor", "factor": "obstacle_clearance",
                "reason": "Repeat with one obstructing circle moved farther from the requested route; keep commands, gains and seed fixed.",
                "discriminating_metrics": ["collision_count", "safety_stop_count", "goal_position_error_m"]}
    if "state_estimation" in diagnosis.get("likely_owner", ""):
        traction = scenario["plant"]["traction"]
        return {"kind": "paired_one_factor", "factor": "plant.traction", "values": [traction, 1. if traction != 1. else .9],
                "reason": "Completed native commands missed the truth goal. Compare nominal traction to distinguish decision target error from odometry/slip.",
                "discriminating_metrics": ["goal_position_error_m", "L3_translation_vs_ground_truth"]}
    if metrics.get("tracking_error_rms_mps") is not None and metrics["tracking_error_rms_mps"] > .02:
        response = scenario["plant"]["response_tau_s"]
        return {"kind": "paired_one_factor", "factor": "plant.response_tau_s", "values": [response, 0. if response != 0. else .1],
                "reason": "Compare instantaneous and delayed wheel response with fixed skill/V3/seed before attributing tracking loss to controller gains.",
                "discriminating_metrics": ["tracking_error_rms_mps", "max_abs_jerk_mps3", "goal_position_error_m"]}
    gain = scenario["plant"]["right_gain"]
    return {"kind": "paired_one_factor", "factor": "plant.right_gain", "values": [gain, 1.1 if gain == 1. else 1.],
            "reason": "Test one modest wheel asymmetry with fixed skill, V3 and seed to check whether the observed behavior generalizes.",
            "discriminating_metrics": ["goal_position_error_m", "goal_yaw_error_rad", "tracking_error_rms_mps"]}


def _environment_fingerprint(scenario):
    # Labels and random seeds do not make a different physical environment.
    # Circle order is immaterial to geometry, so normalize it as well.
    scenario = motion_scenario(scenario)
    environment = {"room": scenario["room"], "plant": scenario["plant"],
                   "obstacles": sorted(scenario["obstacles"],
                                       key=lambda row: (row["x_m"], row["y_m"], row["radius_m"]))}
    return hashlib.sha256(json.dumps(environment, sort_keys=True, separators=(",", ":"),
                                    allow_nan=False).encode()).hexdigest()


def compare_trials(baseline_trials, variant_trials):
    """Paired bounded comparisons; an improvement must survive every supplied environment."""
    if (not isinstance(baseline_trials, list) or not isinstance(variant_trials, list)
            or not 1 <= len(baseline_trials) <= 8 or len(baseline_trials) != len(variant_trials)):
        raise ValueError("comparison requires equally sized lists of 1..8 paired trials")
    rows, blockers = [], []
    for index, (baseline, variant) in enumerate(zip(baseline_trials, variant_trials)):
        if not isinstance(baseline, Mapping) or not isinstance(variant, Mapping):
            raise ValueError("comparison trials must be objects")
        def scenario_key(trial):
            scenario = trial.get("scenario", {})
            return json.dumps({key: value for key, value in scenario.items() if key != "v3_overrides"},
                              sort_keys=True, allow_nan=False)
        if scenario_key(baseline) != scenario_key(variant):
            blockers.append({"pair": index, "reason": "PAIRED_ENVIRONMENT_OR_SEED_MISMATCH"})
            continue
        if baseline.get("scenario", {}).get("kind") != "differential_drive":
            blockers.append({"pair": index, "reason": "MOTION_METRICS_UNAVAILABLE"})
            continue
        base_identity = baseline.get("simulation_config", {}).get("base_sha256")
        variant_identity = variant.get("simulation_config", {}).get("base_sha256")
        if not isinstance(base_identity, str) or len(base_identity) != 64 or not isinstance(variant_identity, str):
            blockers.append({"pair": index, "reason": "BASE_CONTROL_CONFIG_IDENTITY_MISSING"})
        elif base_identity != variant_identity:
            blockers.append({"pair": index, "reason": "UNPAIRED_BASE_CONTROL_CONFIG"})
        for label, trial in (("baseline", baseline), ("variant", variant)):
            if trial.get("runtime", {}).get("state") != "SUCCEEDED":
                blockers.append({"pair": index, "side": label, "reason": "SKILL_RUNTIME_NOT_SUCCEEDED"})
            if trial.get("assessment", {}).get("status") not in {"PASSED", "FAILED"}:
                blockers.append({"pair": index, "side": label, "reason": "TRIAL_ASSESSMENT_INCOMPLETE"})
            if trial.get("performance", {}).get("goal_reached") and trial.get("assessment", {}).get("status") != "PASSED":
                blockers.append({"pair": index, "side": label, "reason": "GOAL_REACHED_WITHOUT_PASS"})
            if (trial.get("performance") != trial.get("assessment", {}).get("performance")
                    or trial.get("simulation_config") != trial.get("assessment", {}).get("simulation_config")):
                blockers.append({"pair": index, "side": label, "reason": "UNBOUND_SIMULATION_MEASUREMENTS"})
            evidence = trial.get("evidence")
            feedback = evidence.get("feedback", {}) if isinstance(evidence, Mapping) else {}
            capture = feedback.get("capture", {})
            if (feedback.get("trial_binding", {}).get("status") != "VERIFIED"
                    or feedback.get("replay", {}).get("verified_match") is not True
                    or capture.get("source_checked") is not True or capture.get("status") != "VERIFIED"
                    or capture.get("sample_complete") is not True or capture.get("raw_evidence_complete") is not True):
                blockers.append({"pair": index, "side": label, "reason": "TRIAL_EVIDENCE_NOT_VERIFIED"})
        before, after = baseline.get("performance", {}), variant.get("performance", {})
        first, second = before.get("metrics", {}), after.get("metrics", {})
        if not first or not second:
            blockers.append({"pair": index, "reason": "MOTION_METRICS_UNAVAILABLE"})
            continue
        delta = {name: second[name]-number for name, number in first.items()
                 if type(number) in {int, float} and type(second.get(name)) in {int, float}}
        improvement, regression = [], []
        if before.get("goal_reached") != after.get("goal_reached"):
            (improvement if after.get("goal_reached") else regression).append("goal_reached")
        comparable = [("collision_count", 0.), ("safety_stop_count", 0.),
                      ("goal_position_error_m", .005), ("goal_yaw_error_rad", .01)]
        # A failed short motion is not an efficient alternative to reaching the
        # goal. Compare efficiency/quality only between two successful trials.
        if before.get("goal_reached") and after.get("goal_reached"):
            # Absolute path tolerance is one centimetre. Jerk changes must
            # exceed both 1 m/s³ and 5% of the baseline to count as tradeoffs.
            jerk = first.get("max_abs_jerk_mps3")
            jerk_tolerance = max(1., abs(jerk)*.05) if type(jerk) in {int, float} else 1.
            comparable.extend((("elapsed_s", .1), ("tracking_error_rms_mps", .005),
                               ("path_length_m", .01), ("max_abs_jerk_mps3", jerk_tolerance)))
        for metric, tolerance in comparable:
            change = delta.get(metric)
            if change is not None and abs(change) > tolerance:
                (improvement if change < 0 else regression).append(metric)
        outcome = "TRADEOFF" if improvement and regression else "IMPROVED" if improvement else "REGRESSED" if regression else "UNCHANGED"
        rows.append({"environment_id": baseline["scenario"]["environment_id"], "seed": baseline["scenario"]["seed"],
            "environment_fingerprint": _environment_fingerprint(baseline["scenario"]),
            "outcome": outcome, "metric_deltas_variant_minus_baseline": delta,
            "comparison_tolerances": dict(comparable),
            "improved_metrics": improvement, "regressed_metrics": regression,
            "baseline_goal_reached": before.get("goal_reached"), "variant_goal_reached": after.get("goal_reached")})
    outcomes = {row["outcome"] for row in rows}
    status = ("INCONCLUSIVE" if blockers else "TRADEOFF" if "TRADEOFF" in outcomes or {"IMPROVED", "REGRESSED"} <= outcomes
              else "REGRESSED" if "REGRESSED" in outcomes else "IMPROVED" if "IMPROVED" in outcomes else "UNCHANGED")
    environments = len({row["environment_fingerprint"] for row in rows})
    next_trial = next((trial for trial in variant_trials if not trial.get("performance", {}).get("goal_reached")),
                      variant_trials[0])
    return {"schema": "R2B4_SKILL_COMPARISON_V1", "status": status, "pairs": rows, "blockers": blockers,
            "generalization": "MULTIPLE_SUPPLIED_ENVIRONMENTS" if environments > 1 else "ONE_SUPPLIED_ENVIRONMENT",
            "distinct_environment_count": environments,
            "change_recommended": status == "IMPROVED" and all(row["variant_goal_reached"] for row in rows),
            "physical_validation_required": status == "IMPROVED" and all(row["variant_goal_reached"] for row in rows),
            "next_experiment": {"kind": "resolve_blocker", "reason": "Resolve missing, unpaired or unverified evidence first.",
                                "blocker": blockers[0]} if blockers else next_motion_experiment(
                                    next_trial.get("performance", {}), next_trial["scenario"]),
            "scope": "Paired synthetic trials only; supplied environments do not prove live or unseen-environment improvement."}


__all__ = ["motion_scenario", "simulation_control_config", "compare_trials"]
