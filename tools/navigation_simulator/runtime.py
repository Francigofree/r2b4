"""A virtual device edge around the unchanged production execution boundary.

ROS/map/controller computation stays in the existing isolated dependency
processes. Only scan-bound compact localization and navigation enter V3.
"""
from __future__ import annotations

from collections import Counter, deque
from dataclasses import replace
from functools import partial
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace

from r2b4_orchestration.motion_simulation import DifferentialDriveExperiment
from .scenario import World, validate
from .evidence import digest, write, source_snapshot


class VirtualMotorSink:
    """Bounded passive L12 sink, with no physical device capability."""
    def __init__(self):
        self.last = None
        self.write_count = 0

    def write(self, command):
        self.last = command
        self.write_count += 1


class Stack:
    def __init__(self, root, output, resolved, domain, overlay):
        if type(domain) is not int or not 0 <= domain <= 232 or domain == 74:
            raise ValueError("simulation requires an isolated ROS domain other than live domain 74")
        self.root, self.output, self.resolved, self.domain = root, output, resolved, domain
        self.children, self.logs, self.paused = {}, [], set()
        self.expected_exits = set()
        self.extra_pids = []
        self.lock = None
        from tools.nav2_planner import _paths, _write_geometry, _commands, _navigation_environment
        from tools.slam_toolbox import _paths as slam_paths, _ros_environment
        paths = _paths(root)
        paths.update(work=output, socket=output/"nav.sock", status=output/"nav_status.json",
                     geometry=output/"geometry.yaml", params=output/"nav2.yaml")
        if len(str(paths["socket"]).encode()) > 100 or len(str(output/"slam.sock").encode()) > 100:
            raise ValueError("output path exceeds Unix socket length")
        self.paths = paths
        control = resolved.runtime.composition.live_control.control
        radius = _write_geometry(paths, control)
        for name, source in (("nav2", root/"conf/nav2_planner.yaml"), ("slam", root/"conf/slam_toolbox.yaml")):
            document = read_parameters(root,source)
            document = parameter_overlay(document, overlay.get(name, {}))
            rendered = subprocess.run([str(paths["prefix"]/"bin/python"),"-c",
                "import yaml,json,sys; print(yaml.safe_dump(json.load(sys.stdin),sort_keys=False))"],
                input=json.dumps(document,allow_nan=False),capture_output=True,text=True,timeout=5,check=True)
            (output/(name+".yaml")).write_text(rendered.stdout)
        nav_env = _navigation_environment(paths, root)
        slam = slam_paths(root)
        slam_env = _ros_environment(root, slam["prefix"])
        for env in (nav_env, slam_env):
            env.update(ROS_DOMAIN_ID=str(domain), ROS_LOG_DIR=str(output/"ros_logs"))
        geometry = {"body_bounds_m": resolved.lidar.safety_body_bounds_m,
                    "padding_m": max(control.navigation.footprint_safety_margin_m, control.lidar_safety.minimum_clearance_m),
                    "lidar_xy_m": [resolved.lidar.matcher.lidar_x_offset_m, resolved.lidar.matcher.lidar_y_offset_m]}
        limits = control.operational_constraints
        commands = _commands(paths, full=True, radius=radius, robot_geometry=geometry,
            controller_limits=(limits.max_v_mps, -min(.20, limits.max_v_mps), limits.max_omega_rad_s))
        self.commands = {name: (cmd, nav_env) for name, cmd in commands}
        self.commands.update(slam=([str(slam["prefix"]/"lib/slam_toolbox/async_slam_toolbox_node"),
                                   "--ros-args", "--params-file", str(output/"slam.yaml")], slam_env),
            slam_bridge=([str(slam["prefix"]/"bin/python"), "-m", "v3.slam_toolbox_bridge",
                          "--socket", str(output/"slam.sock"), "--status-path", str(output/"slam_status.json"),
                          "--output-dir", str(output), "--maximum-pose-age-ns", str(resolved.lidar.global_pose_max_age_ns)], slam_env))

    def start(self):
        from tools.slam_toolbox import _terminate_with_parent
        self.lock = (self.root/"tmp"/f"navigation_sim_domain_{self.domain}.lock").open("a")
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            self.lock.close()
            self.lock = None
            raise RuntimeError("simulation ROS domain is already in use") from exc
        for name, (command, env) in self.commands.items():
            if not Path(command[0]).is_file():
                raise RuntimeError("missing simulation dependency: " + command[0])
            log = (self.output/(name+".log")).open("w")
            self.logs.append(log)
            child = subprocess.Popen(command, cwd=self.root, env=env, stdin=subprocess.DEVNULL,
                                     stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                                     preexec_fn=partial(_terminate_with_parent,os.getpid()))
            self.children[name] = child
            os.setpriority(os.PRIO_PROCESS, child.pid, 10)

    def check(self, memory_limit_mb=1536):
        rss_kb = cpu_ticks = 0
        pids=[os.getpid(),*self.extra_pids]
        for name, child in self.children.items():
            if child.poll() is not None:
                if name not in self.expected_exits:
                    raise RuntimeError(f"SIMULATION_PROCESS_EXIT:{name}:{child.returncode}")
                continue
            pids.append(child.pid)
        frontier=read_status(self.output/"nav_status.json").get("frontier_pid")
        if type(frontier) is int and frontier>1:
            try:
                if os.getpgid(frontier)==self.children["bridge"].pid:
                    pids.append(frontier)
            except ProcessLookupError:
                pass
        for pid in set(pids):
            try:
                # Only owned children; include their ROS executor threads in CPU.
                rss_kb += int(Path(f"/proc/{pid}/statm").read_text().split()[1]) * os.sysconf("SC_PAGE_SIZE") // 1024
                stat = Path(f"/proc/{pid}/stat").read_text().rsplit(")",1)[1].split()
                cpu_ticks += int(stat[11]) + int(stat[12])
            except FileNotFoundError:
                pass
        if rss_kb > memory_limit_mb * 1024:
            raise RuntimeError("SIMULATION_MEMORY_BUDGET_EXCEEDED")
        return {"owned_rss_mb": rss_kb/1024, "owned_cpu_s": cpu_ticks/os.sysconf("SC_CLK_TCK")}

    def ready(self):
        from tools.nav2_planner import _bridge_ready
        nav = read_status(self.output/"nav_status.json")
        slam = read_status(self.output/"slam_status.json")
        now=time.monotonic_ns()
        window=nav.get("nav2_observation",{}).get("global_costmap_window")
        free_start=False
        if isinstance(window,dict) and window.get("robot_in_bounds"):
            pose=window["robot_pose_in_frame"]
            x=math.floor((pose[0]-window["origin_x_m"])/window["resolution_m"])
            y=math.floor((pose[1]-window["origin_y_m"])/window["resolution_m"])
            if 0<=x<window["width"] and 0<=y<window["height"]:
                free_start=0<=window["cells"][y*window["width"]+x]<99
        fresh=all(type(nav.get(key)) is int and 0<=now-nav[key]<=250_000_000
                  for key in ("scan_measurement_ns","odom_measurement_ns","tf_measurement_ns"))
        return (_bridge_ready(nav,now,pid=self.children["bridge"].pid) and fresh and free_start
                and slam.get("state") == "ACTIVE" and slam.get("poses_returned", 0) > 0)

    def fault(self, fault, active):
        kind, target = fault["kind"], fault.get("target")
        if kind == "cpu_load":
            if active:
                log = (self.output/"cpu_load.log").open("a")
                self.logs.append(log)
                name = "cpu_load_" + str(len(self.children))
                command = [sys.executable, "-c", "import time; end=time.monotonic()+float(__import__('sys').argv[1]);\nwhile time.monotonic()<end: sum(i*i for i in range(10000))", str(fault["duration_s"])]
                self.children[name] = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)
                self.expected_exits.add(name)
            return
        if target is None:
            return
        child = self.children[target]
        if kind == "process_pause":
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGSTOP if active else signal.SIGCONT)
                (self.paused.add if active else self.paused.discard)(target)
        elif kind == "process_exit" and active:
            self.expected_exits.add(target)
            os.killpg(child.pid, signal.SIGTERM)

    def close(self):
        # Resume stopped children before terminating; stop every owned process
        # group including explore_lite descendants, even if its leader exited.
        for name, child in reversed(list(self.children.items())):
            for sig in (signal.SIGCONT, signal.SIGTERM):
                try:
                    os.killpg(child.pid, sig)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + 3
        for child in reversed(list(self.children.values())):
            try:
                child.wait(timeout=max(.01, deadline-time.monotonic()))
            except subprocess.TimeoutExpired:
                pass
            try:
                os.killpg(child.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            child.wait(timeout=3)
        for log in self.logs:
            log.close()
        if self.lock:
            self.lock.close()


def read_status(path):
    try:
        return json.loads(path.read_bytes())
    except (FileNotFoundError, ValueError):
        return {}


def read_parameters(root,path):
    """YAML stays in the installed ROS environment; no control-Python dependency."""
    result = subprocess.run([str(root/"runtime/deps/nav2-planner/bin/python"),"-c",
        "import yaml,json,sys; print(json.dumps(yaml.safe_load(open(sys.argv[1]))))",str(path)],
        capture_output=True,text=True,timeout=5,check=True)
    return json.loads(result.stdout)


def parameter_overlay(document, patch, path=""):
    """Allow numeric algorithm tuning only, never topics, plugins or geometry."""
    if not isinstance(patch, dict):
        raise ValueError("parameter overlay must be an object")
    forbidden = ("footprint", "robot_radius", "padding", "use_sim_time", "frequency", "timeout", "age", "frame", "topic", "plugin")
    result = dict(document)
    for key, value in patch.items():
        if key not in document:
            raise ValueError("unknown ROS parameter: " + path + key)
        if isinstance(value, dict) and isinstance(document[key], dict):
            result[key] = parameter_overlay(document[key], value, path+key+".")
        elif (type(value) in {int,float} and math.isfinite(value) and type(document[key]) in {int,float}
              and not any(token in key for token in forbidden)):
            result[key] = value
        else:
            raise ValueError("overlay must tune existing numeric algorithm parameters: " + path + key)
    return result


class Plant(DifferentialDriveExperiment):
    def __init__(self, *args, world, resolved, sender, receiver, **kwargs):
        self.world, self.resolved, self.sender, self.receiver = world, resolved, sender, receiver
        self.faults, self.delayed = set(), deque(maxlen=24)
        self.delay_s = 0.
        self.latest_lidar = ()
        super().__init__(*args, **kwargs)

    def _collision(self):
        return self.world.clearance(self.x,self.y,self.yaw,(self.now_ns-self.start_ns)/1e9) <= 0

    def _ray_ranges(self):
        cfg = self.resolved.lidar.matcher
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        x = self.x + c*cfg.lidar_x_offset_m-s*cfg.lidar_y_offset_m
        y = self.y + s*cfg.lidar_x_offset_m+c*cfg.lidar_y_offset_m
        elapsed = (self.now_ns-self.start_ns)/1e9
        return [(angle, self.world.ray(x,y,self.yaw+angle,elapsed))
                for angle in (i*math.tau/self.lidar_ray_count for i in range(self.lidar_ray_count))]

    def _scan(self, sample):
        from v3.adapters.live_lidar import LidarPointReading
        from v3.contracts import DataField
        from v3.lidar_geometry import body_clearance_m
        samples, raw = super()._scan(sample)
        cfg = self.resolved.lidar.matcher
        driver=self.resolved.lidar.driver
        raw.raw_scan=tuple(LidarPointReading(point.angle_deg,point.distance_m,
            point.quality if driver.minimum_distance_m<=point.distance_m<=driver.maximum_distance_m else 0)
            for point in raw.raw_scan)
        sectors = {name: [] for name in ("front", "left", "rear", "right")}
        # Same physical LiDAR extrinsics for ROS scan, local geometry and safety.
        for point in raw.raw_scan:
            if point.quality<=0:
                continue
            angle = -math.radians(point.angle_deg)
            x = point.distance_m*math.cos(angle)+cfg.lidar_x_offset_m
            y = point.distance_m*math.sin(angle)+cfg.lidar_y_offset_m
            name = ("front","left","rear","right")[int((math.atan2(y,x)+math.pi/4)%math.tau/(math.pi/2))]
            sectors[name].append(body_clearance_m(x,y,self.world.body_bounds))
        output = []
        for row in samples:
            if row.kind=="lidar_health":
                row=sample("lidar_health",age_ns=0,point_count=sum(point.quality>0 for point in raw.raw_scan))
            elif row.kind == "lidar_local_points":
                values = []
                for field in row.values:
                    offset = cfg.lidar_x_offset_m if field.key.endswith("_x_m") else cfg.lidar_y_offset_m if field.key.endswith("_y_m") else 0.
                    values.append(DataField(field.key, field.value+offset if offset else field.value))
                row = replace(row, values=tuple(values))
            elif row.kind == "lidar_safety_clearance":
                row = sample("lidar_safety_clearance", age_ns=0, clearance_basis="BODY_OUTLINE", **{f"{name}_{key}": value
                    for name, distances in sectors.items() for key, value in
                    (("clearance_m", min(distances) if distances else 0.), ("observation_count",len(distances)))})
            output.append(row)
        self.delayed.append((self.now_ns+round(self.delay_s*1e9), tuple(output), raw))
        return tuple(output), raw

    def _filter_samples(self, samples):
        from v3.adapters.process_lidar_port import _slam_matcher_result
        from v3.contracts import DataField, DeviceSample
        # Truth-derived scan odometry belongs to the old modest plant. This
        # path admits only actual SLAM Toolbox localization corrections.
        rows = [row for row in samples if not row.kind.startswith("lidar_")]
        while self.delayed and self.delayed[0][0] <= self.now_ns:
            _, latest, raw = self.delayed.popleft()
            if "lidar_drop" not in self.faults:
                self.latest_lidar = tuple(row for row in latest if row.kind not in {"lidar_relative_motion", "lidar_pose"})
                self._publish_scan = raw
        rows.extend(self.latest_lidar)
        rows = [row for row in rows if not ("imu_drop" in self.faults and row.device_id == "IMU")
                and not ("encoder_drop" in self.faults and row.device_id == "ENCODER")]
        estimate = self.status.get("estimate", {})
        quality = estimate.get("localization_quality", {})
        odom = estimate.get("local_pose", {})
        for payload in self.sender.receive():
            self.receiver.accept(payload, now_ns=self.now_ns, current_generation=quality.get("generation"),
                odometry_ns=self.status.get("monotonic_ns",0),
                odometry_valid=(quality.get("local_translation") in {"GOOD", "DEGRADED"}
                    and quality.get("local_pose_continuous") is True and not quality.get("pose_discontinuity")))
        pose = self.receiver.last_pose
        matched=_slam_matcher_result(self.receiver,self.now_ns)
        if pose is not None and matched is not None:
            values = {"frame_id": "R2B4_BOOT_ROBOT_MAP", "x_m": pose.x_m,"y_m":pose.y_m,"yaw_rad":pose.yaw_rad,
                      "confidence": matched.summary["lidar_pose_confidence"],"r_scale":1.,"variance_x_m2":pose.variance_x_m2,
                      "variance_y_m2":pose.variance_y_m2,"variance_yaw_rad2":pose.variance_yaw_rad2}
            rows.append(DeviceSample("RPLIDAR_C1","lidar_pose",pose.source_sequence,pose.measurement_ns,
                                     tuple(DataField(k,v) for k,v in values.items())))
        return [replace(row,device_id=self.local_perception_config.device_id) if row.device_id=="RPLIDAR_C1" else row for row in rows]


class FaultNavigation:
    def __init__(self, client):
        self.client, self.drop = client, False

    def submit(self, request):
        if not self.drop:
            self.client.submit(request)

    def poll(self, context):
        from v3.contracts.navigation import Nav2NavigationInput
        return Nav2NavigationInput(context) if self.drop else self.client.poll(context)

    def close(self):
        self.client.close()


def slam_odometry(estimate, session_id, source_pid):
    """Match ProcessLidarPort's qualified local-pose export, including DEGRADED."""
    from v3.adapters.slam_toolbox_transport import SlamOdometryPacket
    pose, quality = estimate.local_pose, estimate.localization_quality
    continuous = quality.local_pose_continuous and not quality.pose_discontinuity
    return SlamOdometryPacket(session_id, source_pid, estimate.context.tick_id,
        estimate.context.monotonic_ns, quality.generation, pose.x_m, pose.y_m, pose.yaw_rad,
        continuous and quality.local_translation.value != "LOST",
        continuous and quality.heading.value != "LOST")


def run(root, directory, scenario, *, domain=75, overlay=None, replay=True, startup_s=25., memory_limit_mb=1536):
    from tools.tuners.r2b4_motion_tuner import resolve_config
    from v3.adapters.nav2_navigation import Nav2NavigationClient, Nav2NavigationConfig
    from v3.adapters.slam_toolbox_transport import SlamToolboxSender, SlamLocalizationReceiver
    from v3.capture_encoding import encode_value
    from v3.composition.native_control import NativeControlComposition
    from v3.mcap_capture import McapCaptureConfig
    from v3.process_sidecars import ProcessMcapCaptureSession
    root, directory = Path(root).resolve(), Path(directory).resolve()
    scenario, overlay = validate(scenario), overlay or {}
    if not isinstance(overlay,dict) or set(overlay)-{"vezerles","nav2","slam"}:
        raise ValueError("unsupported simulation overlay fields")
    resolved = resolve_config(root, overlay.get("vezerles"))
    config = resolved.runtime.composition.live_control.control
    if not config.navigation.full_nav2_enabled:
        raise ValueError("full production Nav2 must be enabled; no simulated planner fallback")
    if resolved.runtime.tick_period_ns != 20_000_000:
        raise ValueError("simulator currently requires the production 50 Hz tick contract")
    if shutil.disk_usage(root).free<256*1024*1024:
        raise RuntimeError("simulation needs at least 256 MiB free capture space")
    directory.mkdir(parents=True, exist_ok=False)
    snapshot = source_snapshot(root)
    write(directory/"source.json",snapshot)
    write(directory/"scenario.json",scenario)
    write(directory/"overlay.json",overlay)
    write(directory/"resolved_config.json",resolved.as_dict())
    stack = Stack(root,directory,resolved,domain,overlay)
    session_id = time.monotonic_ns()
    sender = SlamToolboxSender(directory/"slam.sock",reply_session_id=session_id)
    receiver = SlamLocalizationReceiver(session_id,resolved.lidar.global_pose_max_age_ns)
    backend = FaultNavigation(Nav2NavigationClient(Nav2NavigationConfig(str(directory/"nav.sock"))))
    sink = VirtualMotorSink()
    composition = NativeControlComposition(sink,config,navigation_backend=backend)
    session = ProcessMcapCaptureSession("local-navigation",directory/"capture.mcap",configuration={"production_control":config},
        metadata={"source":"simulation","scenario":scenario,"physical_hardware":False,"source_sha256":snapshot["sha256"]},
        config=McapCaptureConfig(mode="append_only",tick_sample_hz=50),capacity=256,project_root=directory)
    timeline = (directory/"timeline.ndjson").open("w")
    plant = None
    active = False
    active_start = None
    stationary = longest_stationary = 0.
    min_clearance = math.inf
    reasons = Counter()
    tick_wall_ns = tick_cpu_ns = tick_max_ns = tick_overruns = 0
    max_lateness_ns = 0
    max_pose_error = 0.
    samples = 0
    coverage_cells = set()
    resources = {"owned_rss_mb":0.,"owned_cpu_s":0.}
    phase_max_ms = {name:0. for name in ("resource_check","readiness","command_ingress","sensor_v3_observation")}
    transitions = []
    trace_previous = None
    first_failure = None
    error = None
    capture = None
    capture_started = False
    termination = "DURATION_REACHED"
    applied_faults = set()
    # Keep the canonical mailbox and validation, but isolate ephemeral command
    # metadata from capture writes on the Pi's storage. ROS uses real monotonic
    # stamps; a blocking filesystem operation must not strand the virtual clock.
    command_directory = tempfile.TemporaryDirectory(prefix="r2b4-nav-command-", dir="/dev/shm")

    def observe(record, raw):
        nonlocal stationary,longest_stationary,min_clearance,samples,max_pose_error,trace_previous,first_failure
        layers = {layer.layer:layer.output for layer in record.result.trace.layers}
        estimate = layers.get("L3")
        if estimate is not None and "tf_drop" not in plant.faults:
            sender.publish_odometry(slam_odometry(estimate, session_id, os.getpid()), now_ns=plant.now_ns)
        exported = getattr(plant,"_publish_scan",None)
        if exported is not None:
            cfg = resolved.lidar
            if sender.publish_scan(exported,now_ns=plant.now_ns,session_id=session_id,source_pid=os.getpid(),
                    minimum_range_m=cfg.driver.minimum_distance_m,maximum_range_m=cfg.driver.maximum_distance_m,
                    lidar_x_m=cfg.matcher.lidar_x_offset_m,lidar_y_m=cfg.matcher.lidar_y_offset_m,maximum_age_ns=2_000_000_000):
                receiver.remember_scan(exported)
            plant._publish_scan = None
        if raw is not None:
            session.observe_raw_lidar(raw)
        session.observe(record)
        session.observe_decision(record)
        if session.failed:
            raise RuntimeError("SIMULATION_CAPTURE_INTEGRITY_FAILURE")
        nav, mission = layers.get("L6"),layers.get("L5")
        final = record.result.final_actuation
        clearance = plant.world.clearance(plant.x,plant.y,plant.yaw,(plant.now_ns-plant.start_ns)/1e9)
        row = {"tick_id":plant.tick,"measurement_monotonic_ns":plant.now_ns,"active":active,
               "pose":[plant.x,plant.y,plant.yaw],"v_mps":plant.previous_v,
               "omega_rad_s":(plant.right-plant.left)/config.chassis_control.track_width_m,
               "clearance_m":clearance,"mission_id":getattr(mission,"mission_id",None),
               "navigation_status":getattr(getattr(nav,"status",None),"value",None),
               "navigation_reason":getattr(nav,"reason",None),"goal":encode_value(getattr(nav,"local_goal",None)),
               "safety":final.safety_decision.value,"safety_reason":final.reason,
               "left_output":final.left_output,"right_output":final.right_output,
               "fault_layer":record.result.trace.fault_layer,"faults":sorted(plant.faults)}
        if estimate is not None:
            pose = estimate.global_pose
            row["estimated_pose"] = [pose.x_m,pose.y_m,pose.yaw_rad]
            row["localization_quality"] = encode_value(estimate.localization_quality)
            if active:
                max_pose_error = max(max_pose_error,math.hypot(pose.x_m-plant.x,pose.y_m-plant.y))
        timeline.write(json.dumps(row,allow_nan=False)+"\n")
        signature = (row["navigation_reason"],row["navigation_status"],row["safety"],row["safety_reason"],row["goal"])
        if signature != trace_previous:
            if len(transitions)<256:
                transitions.append(row)
            trace_previous = signature
        if active:
            samples += 1
            coverage_cells.add((math.floor(plant.x/.25),math.floor(plant.y/.25)))
            min_clearance = min(min_clearance,clearance)
            stopped = abs(row["v_mps"]) < .02 and abs(row["omega_rad_s"]) < .1
            stationary = stationary+plant.dt_ns/1e9 if stopped else 0.
            longest_stationary = max(longest_stationary,stationary)
            reasons[row["navigation_reason"]] += 1
            if stationary>scenario["expect"]["maximum_stationary_s"] and first_failure is None:
                first_failure={"tick_id":plant.tick,"monotonic_ns":plant.now_ns,"layer":"behavior_observation",
                    "reason":"STATIONARY_INTERVAL_EXCEEDS_EXPECTATION","stationary_s":stationary,
                    "navigation_reason":row["navigation_reason"],"safety_reason":row["safety_reason"],"causal_proof":False}

    try:
        stack.start()
        session.start()
        capture_started = True
        stack.extra_pids.append(session._process.pid)
        os.setpriority(os.PRIO_PROCESS,session._process.pid,10)
        world = World(scenario,resolved.lidar.safety_body_bounds_m)
        ingress = resolved.edges.command_ingress
        plant = Plant(scenario,config,composition,sink,directory,time.monotonic_ns(),observe,lambda *args:None,
            command_directory=Path(command_directory.name),
            world=world,resolved=resolved,sender=sender,receiver=receiver,
            tick_period_ns=resolved.runtime.tick_period_ns,lidar_ray_count=360,
            local_perception_config=resolved.runtime.sensor_inputs.inputs.lidar_source,
            maximum_linear_speed_mps=ingress.maximum_linear_speed_mps,maximum_angular_speed_rad_s=ingress.maximum_angular_speed_rad_s)
        started = time.monotonic()
        last_check = 0.
        while True:
            deadline = plant.now_ns+(plant.dt_ns if plant.tick else 0)
            remaining = (deadline-time.monotonic_ns())/1e9
            if remaining>0:
                time.sleep(remaining)
            lateness = max(0,time.monotonic_ns()-deadline)
            max_lateness_ns = max(max_lateness_ns,lateness)
            if lateness>500_000_000:
                raise RuntimeError("SIMULATION_PACING_DEADLINE_EXCEEDED")
            if time.monotonic()-last_check>.25:
                phase_start=time.perf_counter_ns()
                sample = stack.check(memory_limit_mb)
                resources = {k:max(resources[k],v) for k,v in sample.items()}
                last_check = time.monotonic()
                phase_max_ms["resource_check"]=max(phase_max_ms["resource_check"],(time.perf_counter_ns()-phase_start)/1e6)
            if not active:
                phase_start=time.perf_counter_ns()
                if stack.ready() and receiver.accepted_packets>0:
                    active,active_start = True,plant.now_ns
                elif time.monotonic()-started>startup_s:
                    raise RuntimeError("SIMULATION_STARTUP_TIMEOUT")
                phase_max_ms["readiness"]=max(phase_max_ms["readiness"],(time.perf_counter_ns()-phase_start)/1e6)
            elapsed = 0 if active_start is None else (plant.now_ns-active_start)/1e9
            plant.faults = {fault["kind"] for fault in scenario["faults"] if fault["start_s"]<=elapsed<fault["start_s"]+fault["duration_s"]} if active else set()
            plant.delay_s = max((fault.get("delay_s",0) for fault in scenario["faults"] if fault["kind"]=="lidar_delay" and fault["start_s"]<=elapsed<fault["start_s"]+fault["duration_s"]),default=0)
            backend.drop = "nav2_loss" in plant.faults
            for index,fault in enumerate(scenario["faults"]):
                enabled = active and fault["start_s"]<=elapsed<fault["start_s"]+fault["duration_s"]
                if enabled != (index in applied_faults):
                    stack.fault(fault,enabled)
                    (applied_faults.add if enabled else applied_faults.discard)(index)
            if active and elapsed>=scenario["duration_s"]:
                break
            if active and "command_loss" not in plant.faults:
                phase_start=time.perf_counter_ns()
                if scenario["behavior"]=="roomcruise":
                    profile = resolved.roomcruise
                    plant.client.publish_explore("simulation.cruise",max_v_mps=profile.max_v_mps,
                        max_omega_rad_s=profile.max_omega_rad_s,preferences=profile.preferences,ttl_ns=200_000_000)
                else:
                    plant.client.publish_navigate("simulation.navigate",**scenario["goal"],frame_id="R2B4_ODOM_LOCAL",
                        max_v_mps=min(.25,ingress.maximum_linear_speed_mps),
                        max_omega_rad_s=min(1.4,ingress.maximum_angular_speed_rad_s),ttl_ns=200_000_000)
                phase_max_ms["command_ingress"]=max(phase_max_ms["command_ingress"],(time.perf_counter_ns()-phase_start)/1e6)
            before,cpu = time.perf_counter_ns(),time.thread_time_ns()
            result = plant.step(active=active)
            wall = time.perf_counter_ns()-before
            tick_wall_ns+=wall
            tick_cpu_ns+=time.thread_time_ns()-cpu
            tick_max_ns=max(tick_max_ns,wall)
            phase_max_ms["sensor_v3_observation"]=max(phase_max_ms["sensor_v3_observation"],wall/1e6)
            tick_overruns+=wall>plant.dt_ns
            if result.final_actuation.safety_decision.value=="FAULT":
                plant.fault_layer=plant.fault_layer or "L12"
                plant.first_boundary=plant.first_boundary or {"tick_id":result.trace.context.tick_id,
                    "monotonic_ns":result.trace.context.monotonic_ns,"layer":"L12","reason":result.final_actuation.reason}
            if plant.collision_count or plant.fault_layer:
                termination = "FOOTPRINT_COLLISION" if plant.collision_count else "NATIVE_FAULT"
                first_failure = plant.first_boundary
                break
            if active and scenario["behavior"]=="navigate" and plant.status.get("navigation",{}).get("status")=="COMPLETE" and not scenario["faults"]:
                termination = "GOAL_COMPLETE"
                break
    except BaseException as exc:
        error = exc
        termination = f"{type(exc).__name__}:{exc}"
        if first_failure is None:
            first_failure={"tick_id":plant.tick if plant else None,"monotonic_ns":plant.now_ns if plant else None,
                           "layer":"simulation_edge","reason":termination}
    finally:
        if plant is not None:
            # STOP uses the same gateway and V3 chain; no direct sink write.
            active = False
            try:
                plant.faults=set()
                backend.drop=False
                for _ in range(max(1,math.ceil(6*scenario["plant"]["response_tau_s"]/(plant.dt_ns/1e9)))):
                    remaining=(plant.now_ns+plant.dt_ns-time.monotonic_ns())/1e9
                    if remaining>0:
                        time.sleep(remaining)
                    plant.client.publish_stop("simulation.settle",ttl_ns=200_000_000)
                    plant.step()
            except BaseException as exc:
                error = error or exc
        try:
            composition.close()
        except BaseException as exc:
            error=error or exc
        finally:
            backend.close()
            sender.close()
            stack.close()
            timeline.close()
            command_directory.cleanup()
        if capture_started:
            try:
                capture = session.finalize(SimpleNamespace(status=0 if error is None and plant is not None and not plant.fault_layer and not plant.collision_count else 1),error=error)
            except BaseException as exc:
                error=error or exc
    expected = scenario["expect"]
    truth_error = math.hypot(plant.x-scenario["goal"]["x_m"],plant.y-scenario["goal"]["y_m"]) if plant else None
    yaw_error=abs(math.atan2(math.sin(plant.yaw-scenario["goal"]["yaw_rad"]),math.cos(plant.yaw-scenario["goal"]["yaw_rad"]))) if plant and "yaw_rad" in scenario["goal"] else None
    goal_reached = (truth_error is not None and truth_error<=scenario["position_tolerance_m"]
                    and (yaw_error is None or yaw_error<=scenario["yaw_tolerance_rad"]))
    final_stop = plant is not None and plant.output==(0.,0.)
    body_stopped=plant is not None and max(abs(plant.left),abs(plant.right))<.005
    failures = []
    if error: failures.append(f"{type(error).__name__}:{error}")
    if not samples: failures.append("NO_ACTIVE_SAMPLES")
    if not final_stop: failures.append("FINAL_STOP_NOT_PROVEN")
    if not body_stopped: failures.append("VIRTUAL_BODY_NOT_SETTLED")
    if plant and plant.collision_count: failures.append("FOOTPRINT_COLLISION")
    if plant and plant.fault_layer: failures.append("NATIVE_FAULT:"+plant.fault_layer)
    if expected["goal_reached"] and not goal_reached: failures.append("TRUTH_GOAL_NOT_REACHED")
    if plant and plant.path_length<expected["minimum_path_m"]: failures.append("INSUFFICIENT_PROGRESS")
    if longest_stationary>expected["maximum_stationary_s"]: failures.append("EXCESSIVE_STATIONARY_INTERVAL")
    if failures and first_failure is None:
        first_failure={"tick_id":plant.tick-1 if plant else None,"monotonic_ns":plant.now_ns if plant else None,
                       "layer":"experiment_expectation","reason":failures[0],"causal_proof":False}
    after = source_snapshot(root)
    if snapshot["sha256"]!=after["sha256"]: failures.append("SOURCE_CHANGED_DURING_RUN")
    result = {"schema":"R2B4_LOCAL_NAVIGATION_RESULT_V1","status":"FAIL" if failures else "PASS",
              "physical_output":False,"backend":"production_v3_slam_nav2","scenario":scenario,
              "scenario_sha256":digest(scenario),"source_sha256":snapshot["sha256"],"overlay":overlay,"overlay_sha256":digest(overlay),
              "config_snapshot_id":resolved.snapshot_id,"termination":termination,"failures":failures,
              "first_failure":first_failure,"final_stop":final_stop,"virtual_body_stopped":body_stopped,"capture":str(capture) if capture else None,
              "metrics":{"path_length_m":plant.path_length if plant else 0,"goal_error_m":truth_error,"goal_yaw_error_rad":yaw_error,
                         "goal_reached":goal_reached,"longest_stationary_s":longest_stationary,
                         "unique_coverage_cells":len(coverage_cells),"coverage_cell_size_m":.25,
                         "minimum_clearance_m":min_clearance if math.isfinite(min_clearance) else None,
                         "collision_count":plant.collision_count if plant else 0,"active_samples":samples,
                         "max_linear_jerk_mps3":plant.max_jerk if plant else 0.,
                         "active_duration_s":samples*.02,"max_localization_error_m":max_pose_error,
                         "max_tick_wall_ms":tick_max_ns/1e6,"tick_wall_s":tick_wall_ns/1e9,"tick_thread_cpu_s":tick_cpu_ns/1e9,
                         "ticks_over_20ms":tick_overruns,"max_tick_lateness_ms":max_lateness_ns/1e6,**resources},
              "phase_max_wall_ms":phase_max_ms,
              "localization":{"accepted":receiver.accepted_packets,"rejected":receiver.dropped_packets,"last_reason":receiver.last_reason},
              "navigation_reasons":dict(reasons),"transitions":transitions,
              "slam":read_status(directory/"slam_status.json"),"nav2":read_status(directory/"nav_status.json"),
              "replay":{"status":"NOT_REQUESTED"},
              "limitations":["Synthetic PWM/traction/sensors; physical calibration is not proven.",
                              "ROS scheduling is nondeterministic; compare behavior with repeated scenarios, replay only closed V3 inputs.",
                              "Tick wall time includes synthetic device generation and evidence publication, not isolated L0-L12 or GIL attribution."]}
    if replay and capture:
        from .evidence import replay as replay_run
        result["replay"] = replay_run(capture,root,directory/"replay.json")
        if result["replay"]["status"]!="MATCH":
            result["status"]="FAIL"
            failures.append("NATIVE_REPLAY_NOT_MATCH")
    write(directory/"result.json",result)
    if isinstance(error,(KeyboardInterrupt,SystemExit)):
        raise error
    return result
