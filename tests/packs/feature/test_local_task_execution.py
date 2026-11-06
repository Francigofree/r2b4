"""Local proposals execute through the real host owner and RobotInterface facade."""
import json
import math
from dataclasses import replace

import pytest

from r2b4_orchestration.local_task_planner import LocalTaskPlanner
from r2b4_orchestration.robot_runtime import PublicRobotRuntime
from r2b4_orchestration.world_model import PublicWorldModel, ValidityScope
from v3.action_catalog import ACTION_CATALOG
from v3.adapters.vision_media_contracts import CameraJpegMetadata, VisionJpeg
from v3.contracts.localization import GLOBAL_FRAME_ID, LOCAL_FRAME_ID
from v3.robot_interface import RobotInterface


class Clock:
    now = 20_000_000_000

    def __call__(self):
        return self.now


class RawEvidence:
    def __repr__(self):
        raise AssertionError("raw producer evidence must stay outside Brain snapshots")


class Backend:
    """A physical adapter with completed L3 snapshots and synthetic camera media."""
    name = "local-execution-test"
    capability_names = frozenset((*ACTION_CATALOG, "vision.observe", "v3.status", "operator.status"))

    def __init__(self, clock):
        self.clock = clock
        self.runtime_pid = 123
        self.generation = 4
        self.x_m, self.y_m, self.yaw_rad = 2.0, 3.0, 0.0
        self.tick = 1
        self.commands = 0
        self.frames = 0
        self.actions = []
        self.images = []
        self.stops = 0
        self.owner = None
        self.on_camera = None
        self.navigation_target = None
        self.mission_id = "idle"
        self.status = self.make_status()

    def capabilities(self):
        return {name: {"kind": "read" if name.endswith(".status") else "action",
                       "supported": True, "available": True, "ready": True}
                for name in self.capability_names}

    def pose(self, frame=LOCAL_FRAME_ID):
        return {"frame_id": frame, "x_m": self.x_m, "y_m": self.y_m, "yaw_rad": self.yaw_rad}

    def make_status(self, *, active=False):
        return {"monotonic_ns": self.clock.now, "tick_id": self.tick,
                "state": "RUNNING", "fault_layer": None,
                "safety_decision": "ALLOW" if active else "STOP",
                "safety_reason": "ALLOW" if active else "NOT_ACTIVE",
                "estimate": {**self.pose(GLOBAL_FRAME_ID), "local_pose": self.pose(),
                             "global_pose": self.pose(GLOBAL_FRAME_ID),
                             "localization_quality": {"generation": self.generation, "heading": "GOOD",
                                                      "local_translation": "GOOD", "global_position": "GOOD",
                                                      "pose_discontinuity": False}},
                "world": {"frame_id": GLOBAL_FRAME_ID, "map_revision": 7},
                "mission": {"mission_id": self.mission_id, "mode": "NAVIGATE",
                            "lifecycle": "ACTIVE" if active else "COMPLETED"},
                "navigation": {"mission_id": self.mission_id,
                               "status": "ACTIVE" if active else "COMPLETE", "reason": None}}

    def publish(self, *, active=False):
        self.clock.now += 10_000_000
        self.tick += 1
        self.status = self.make_status(active=active)
        if self.owner is not None:
            self.owner.ingest_status(self.status, runtime_pid=self.runtime_pid)

    def read(self, resource):
        if resource == "v3.status":
            return self.status
        if resource == "operator.status":
            return {"runtime_running": True, "runtime_pid": self.runtime_pid,
                    "capture_mode": "full", "capture_hz": 10, "status": self.status}
        raise KeyError(resource)

    def execute(self, action, **parameters):
        if action == "v3.command.stop":
            self.stops += 1
            self.publish()
            return {"status": "STOPPED"}
        self.actions.append((action, parameters))
        if action == "vision.observe":
            self.frames += 1
            self.publish()
            metadata = CameraJpegMetadata(
                source_sequence=self.frames, sensor_timestamp_ns=self.clock.now - 1,
                measurement_monotonic_ns=self.clock.now, completed_monotonic_ns=self.clock.now,
                calibration_id="calibration-a", stream="lores", width=640, height=480,
                rectified_K=((500., 0., 320.), (0., 500., 240.), (0., 0., 1.)),
                owner_generation="camera-session-a")
            image = VisionJpeg(b"\xff\xd8raw-jpeg-" + str(self.frames).encode() + b"\xff\xd9", metadata)
            self.images.append(image)
            if self.on_camera is not None:
                self.on_camera()
            return image
        self.commands += 1
        command = "command-" + str(self.commands)
        self.mission_id = "mission-" + command
        if action == "v3.command.navigate":
            self.navigation_target = parameters
            self.publish(active=True)
            return {"command_id": command, "mission_id": self.mission_id}
        start = self.pose()
        requested_angle = math.radians(parameters.get("angle_deg", 0))
        if action == "v3.command.move_relative":
            forward, left = parameters.get("forward_m", 0), parameters.get("left_m", 0)
            self.x_m += math.cos(self.yaw_rad) * forward - math.sin(self.yaw_rad) * left
            self.y_m += math.sin(self.yaw_rad) * forward + math.cos(self.yaw_rad) * left
        elif action == "v3.command.turn_by":
            self.yaw_rad = math.atan2(math.sin(self.yaw_rad + requested_angle),
                                     math.cos(self.yaw_rad + requested_angle))
        else:
            raise AssertionError("unexpected physical action: " + action)
        self.publish()
        return {"command_id": command, "mission_id": self.mission_id, "status": "COMPLETED",
                "reason": "COMPLETE", "runtime_pid": self.runtime_pid,
                "requested": {**parameters, "raw": RawEvidence()}, "start_pose": start,
                "target_pose": self.pose(), "final_pose": self.pose(),
                "completion_status_monotonic_ns": self.clock.now,
                "frame_provenance": {"frame_id": LOCAL_FRAME_ID, "runtime_pid": self.runtime_pid,
                                     "localization_generation": self.generation,
                                     "pose_status_monotonic_ns": self.clock.now, "config_snapshot_id": "test-config"},
                "angle_requested_rad": requested_angle, "angle_executed_rad": requested_angle,
                "distance_executed_m": math.hypot(self.x_m - start["x_m"], self.y_m - start["y_m"]),
                "raw": RawEvidence()}

    def complete_navigation(self):
        assert self.navigation_target is not None
        self.x_m = self.navigation_target["x_m"]
        self.y_m = self.navigation_target["y_m"]
        self.yaw_rad = self.navigation_target.get("yaw_rad", self.yaw_rad)
        self.publish()


def runtime():
    clock = Clock()
    backend = Backend(clock)
    interface = RobotInterface(controller=backend, adapters=(backend,), upper_runtime=False, clock_ns=clock)
    world = PublicWorldModel(clock_ns=clock, clock_epoch="local-execution-test")
    owner = PublicRobotRuntime(interface, world=world, clock_ns=clock)
    backend.owner = owner
    owner.ingest_status(backend.status, runtime_pid=backend.runtime_pid)
    return clock, backend, owner


def locally_adopt(owner, text):
    proposal = LocalTaskPlanner().resolve(text, owner)
    assert proposal.plan is not None, proposal
    pending = owner.brain.submit(text)
    return owner.brain.adopt(pending["goal_id"], proposal.plan)


def camera_results(owner):
    return [event.state.to_jsonable()["result"] for event in owner.brain.history()
            if event.kind == "OBSERVATION_RESULT"]


def test_offline_multi_instruction_motion_executes_requested_displacements_in_order():
    _, backend, owner = runtime()
    result = locally_adopt(owner, "Menj előre 1 métert, fordulj jobbra 90 fokot, menj még fél métert.")
    assert result["lifecycle"] == "COMPLETED", result
    assert [(action, parameters.get("forward_m", parameters.get("angle_deg")))
            for action, parameters in backend.actions] == [
        ("v3.command.move_relative", 1), ("v3.command.turn_by", -90), ("v3.command.move_relative", .5)]
    assert backend.x_m == pytest.approx(3) and backend.y_m == pytest.approx(2.5)
    assert tuple(result["constraints"]["distances_m"]) == (1, .5)
    serialized = json.dumps(owner.brain.export_state())
    assert "RawEvidence" not in serialized and '"raw"' not in serialized
    _, restored_backend, restored = runtime()
    assert not restored.brain.restore(json.loads(serialized))
    saved = restored.brain.goal(result["goal_id"])
    assert saved["lifecycle"] == "COMPLETED"
    assert saved["result"]["final_pose"] == result["result"]["final_pose"]
    assert restored_backend.actions == []


def test_look_around_uses_fresh_distinct_frames_with_each_completed_pose():
    _, backend, owner = runtime()
    result = locally_adopt(owner, "Nézz körül.")
    assert result["lifecycle"] == "COMPLETED", result
    observations = camera_results(owner)
    assert len(observations) == 5 and len(backend.images) == 5
    assert [item["source_sequence"] for item in observations] == [1, 2, 3, 4, 5]
    assert [item["pose_yaw_rad"] for item in observations] == pytest.approx([0, math.pi / 2, math.pi, -math.pi / 2, 0])
    for observed, image in zip(observations, backend.images):
        assert observed["measurement_time_ns"] == image.metadata.measurement_monotonic_ns
        assert observed["pose_status_monotonic_ns"] <= observed["measurement_time_ns"]
        assert observed["pose_frame_id"] == LOCAL_FRAME_ID
        assert observed["runtime_pid"] == 123 and observed["localization_generation"] == 4
        assert observed["owner_generation"] == image.metadata.owner_generation
    snapshot = json.dumps(owner.brain.export_state())
    assert "raw-jpeg" not in snapshot and "image_bytes" not in snapshot
    assert (backend.x_m, backend.y_m) == (2, 3)


@pytest.mark.parametrize("destination", ["move", "known_place"])
def test_go_observe_return_captures_one_origin_and_reuses_only_current_session(destination):
    clock, backend, owner = runtime()
    if destination == "known_place":
        owner.world.observe("room:kitchen", "location", {"name": "konyha", "x_m": 7, "y_m": 4,
                            "frame_id": GLOBAL_FRAME_ID}, domain="room_topology",
                            measurement_time_ns=clock.now, confidence=1, source="semantic-map",
                            lineage=("L4:map:7",), validity_scope=ValidityScope(
                                frame_id=GLOBAL_FRAME_ID, runtime_pid=123, map_revision=7))
        text = "Menj a konyhába, készíts képet, gyere vissza."
    else:
        text = "Menj előre 1 métert, készíts képet, gyere vissza."
    result = locally_adopt(owner, text)
    assert result["lifecycle"] == "ACTIVE", result
    if destination == "known_place":
        assert backend.actions[0][1]["x_m"] == 7
        backend.complete_navigation()
        owner.brain.step()
    result = owner.brain.goal(result["goal_id"])
    origin = result["origin"]
    assert (origin["x_m"], origin["y_m"], origin["yaw_rad"]) == (2, 3, 0)
    assert origin["frame_id"] == LOCAL_FRAME_ID and origin["runtime_pid"] == 123
    assert origin["localization_generation"] == 4 and origin["source_tick_id"] > 0
    assert origin["measurement_time_ns"] <= origin["observation_time_ns"]
    action, parameters = backend.actions[-1]
    assert action == "v3.command.navigate"
    assert (parameters["x_m"], parameters["y_m"], parameters["yaw_rad"]) == (2, 3, 0)
    assert parameters["frame_id"] == LOCAL_FRAME_ID and parameters["expected_runtime_pid"] == 123
    assert parameters["capture"] is False and parameters["capture_mode"] == "full"
    assert len(camera_results(owner)) == 1
    backend.complete_navigation()
    owner.brain.step()
    completed = owner.brain.goal(result["goal_id"])
    assert completed["lifecycle"] == "COMPLETED"
    assert (backend.x_m, backend.y_m, backend.yaw_rad) == (2, 3, 0)
    assert sum(event.kind == "GOAL_ORIGIN_CAPTURED" for event in owner.brain.history()) == 1


@pytest.mark.parametrize("change", ["runtime", "localization_generation"])
def test_origin_identity_change_after_camera_cannot_submit_return_motion(change):
    _, backend, owner = runtime()

    def change_context():
        if change == "runtime":
            backend.runtime_pid += 1
        else:
            backend.generation += 1
        backend.publish()

    backend.on_camera = change_context
    result = locally_adopt(owner, "Menj előre 1 métert, készíts képet, gyere vissza.")
    assert result["lifecycle"] == "FAILED", result
    assert result["failure_code"] in {"IDENTITY_INVALID", "RUNTIME_RESTART"}
    assert [action for action, _ in backend.actions] == ["v3.command.move_relative", "vision.observe"]


def test_restored_active_return_goal_keeps_origin_as_history_without_resuming_motion():
    _, _, owner = runtime()
    goal = locally_adopt(owner, "Menj előre 1 métert, készíts képet, gyere vissza.")
    assert goal["lifecycle"] == "ACTIVE"
    saved = json.loads(json.dumps(owner.brain.export_state()))
    _, backend, restored = runtime()
    assert restored.brain.restore(saved)
    result = restored.brain.goal(goal["goal_id"])
    assert result["lifecycle"] == "INTERRUPTED" and result["failure_code"] == "RUNTIME_RESTART"
    assert result["origin"] == goal["origin"]
    restored.brain.notify("STATE_UPDATED")
    restored.brain.step()
    assert backend.actions == []


def test_camera_frame_from_before_the_new_view_cannot_complete_look_around():
    _, backend, owner = runtime()
    execute = backend.execute

    def old_frame(action, **parameters):
        result = execute(action, **parameters)
        if action == "vision.observe" and backend.frames == 2:
            result = VisionJpeg(result.image_bytes, replace(result.metadata,
                measurement_monotonic_ns=result.metadata.measurement_monotonic_ns - 100_000_000))
        return result

    backend.execute = old_frame
    result = locally_adopt(owner, "Nézz körül.")
    assert result["lifecycle"] == "FAILED" and result["failure_code"] == "STALE_WORLD"
    assert backend.frames == 2
    assert len([action for action, _ in backend.actions if action == "v3.command.turn_by"]) == 1
