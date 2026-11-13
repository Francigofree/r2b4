"""ROS-local explore_lite action relay; no V3 command or motor capability.

The upstream explorer runs only for an admitted EXPLORE session. Its action is
remapped here, never to Nav2 directly. The navigation bridge remains the sole
client of the real Nav2 action and returns each child's actual outcome.
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import partial
import math
import os
import subprocess
import time

from v3.contracts import GLOBAL_FRAME_ID
from v3.adapters.nav2_navigation import HEARTBEAT_MAX_AGE_NS


def frontier_pose(goal, *, now_ns, started_ns, ros_offset_ns):
    """Validate a proposal's original stamp, frame and planar pose."""
    pose = goal.pose
    measured = pose.header.stamp.sec * 1_000_000_000 + pose.header.stamp.nanosec - ros_offset_ns
    p, q = pose.pose.position, pose.pose.orientation
    if (pose.header.frame_id != GLOBAL_FRAME_ID or goal.behavior_tree
            or not started_ns <= measured <= now_ns
            or now_ns - measured > HEARTBEAT_MAX_AGE_NS
            or not all(math.isfinite(value) for value in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
            or abs(p.z) > 1e-6 or abs(q.x) + abs(q.y) > 1e-6
            or abs(q.z*q.z + q.w*q.w - 1.) > .01):
        return None
    return p.x, p.y, math.atan2(2*q.w*q.z, 1-2*q.z*q.z)


@dataclass(slots=True)
class FrontierProposal:
    goal: tuple[float, float, float] | None
    handle: object
    completion: object


class Ros2FrontierExplorer:
    def __init__(self, node, executable, parameters, log_path, ros_offset_ns):
        from rclpy.action import ActionServer, CancelResponse, GoalResponse
        from nav2_msgs.action import NavigateToPose
        self.node, self.executable, self.parameters = node, executable, parameters
        self.ros_offset_ns = ros_offset_ns
        self.action_type = NavigateToPose
        self.process = None
        self.session = None
        self.started_ns = 0
        self.proposal = None
        self.executions = {}
        self.status = None
        self.status_subscription = None
        self.startup_timing = None
        self.accepted = self.rejected = 0
        self.log = open(log_path, "a")

        def goal_callback(goal):
            valid = (self.session is not None and self.process is not None
                and self.process.poll() is None and not self.proposal_committed
                and len(self.executions) < 2 and frontier_pose(goal,
                    now_ns=time.monotonic_ns(), started_ns=self.started_ns,
                    ros_offset_ns=self.ros_offset_ns) is not None)
            if not valid:
                self.rejected += 1
            return GoalResponse.ACCEPT if valid else GoalResponse.REJECT

        def accepted_callback(handle):
            from rclpy.task import Future
            pose = frontier_pose(handle.request, now_ns=time.monotonic_ns(),
                started_ns=self.started_ns, ros_offset_ns=self.ros_offset_ns)
            proposal = FrontierProposal(pose, handle, Future())
            self.executions[id(handle)] = proposal
            if pose is not None and self.session is not None:
                # explore_lite may preempt an old frontier as the map changes.
                self.finish(self.proposal, "ABORTED", 0)
                self.proposal = proposal
                self.accepted += 1
            else:
                proposal.goal = None
                self.rejected += 1
            handle.execute()

        async def execute_callback(handle):
            proposal = self.executions[id(handle)]
            if proposal.goal is None:
                handle.abort()
                result = NavigateToPose.Result()
                result.error_code = 900
                del self.executions[id(handle)]
                return result
            status, error_code = await proposal.completion
            if status == "SUCCEEDED":
                handle.succeed()
            elif handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
            result = NavigateToPose.Result()
            result.error_code = error_code
            del self.executions[id(handle)]
            return result

        self.server = ActionServer(node, NavigateToPose, "/r2b4/frontier/navigate_to_pose",
            execute_callback=execute_callback, goal_callback=goal_callback,
            handle_accepted_callback=accepted_callback,
            cancel_callback=lambda handle: CancelResponse.ACCEPT)
        from explore_lite_msgs.msg import ExploreStatus
        self.status_type = ExploreStatus

    @property
    def proposal_committed(self):
        # A moving frontier centroid is a map proposal, not a reason to abort
        # useful progress. Nav2 owns replanning and reports actual failure.
        # Explicit explorer cancellation still revokes this commitment.
        return (self.proposal is not None and not self.proposal.completion.done()
            and not self.proposal.handle.is_cancel_requested)


    def start(self, session):
        if self.session == session:
            if self.process is None or self.process.poll() is not None:
                raise RuntimeError("FRONTIER_EXPLORER_EXITED")
            return True
        requested_ns = time.monotonic_ns()
        self.stop()
        if self.process is not None:
            return False  # Reap the old generation before accepting another.
        from tools.slam_toolbox import _terminate_with_parent
        self.started_ns = time.monotonic_ns()
        self.status = None
        generation = self.started_ns
        status_topic = f"/r2b4/frontier/status_{generation}"
        self.startup_timing = {"generation": generation, "requested_ns": requested_ns,
            "previous_cleanup_ns": generation-requested_ns, "phase": "SUBSCRIBING"}

        def status_callback(message):
            if self.session is not None and self.started_ns == generation:
                self.status = message.status

        self.status_subscription = self.node.create_subscription(
            self.status_type, status_topic, status_callback, 1)
        subscribed_ns = time.monotonic_ns()
        self.startup_timing.update(subscription_ns=subscribed_ns-generation, phase="LAUNCHING")
        self.process = subprocess.Popen([self.executable, "--ros-args", "--params-file",
            self.parameters, "-r", "navigate_to_pose:=/r2b4/frontier/navigate_to_pose",
            "-r", f"explore/status:={status_topic}"],
            stdin=subprocess.DEVNULL, stdout=self.log, stderr=subprocess.STDOUT,
            preexec_fn=partial(_terminate_with_parent, os.getpid()))
        self.session = session
        self.startup_timing.update(launch_ns=time.monotonic_ns()-subscribed_ns, phase="READY")
        return True

    def finish(self, proposal, status, error_code=0):
        if proposal is not None and not proposal.completion.done():
            proposal.completion.set_result((status, error_code))
        if self.proposal is proposal:
            self.proposal = None

    def poll(self):
        if self.session is not None and self.process.poll() is not None:
            raise RuntimeError("FRONTIER_EXPLORER_EXITED")
        if self.process is not None and self.session is None:
            if self.process.poll() is not None:
                self.process.wait()
                self.process = None
            elif time.monotonic_ns() - self.stop_started_ns > 2_000_000_000:
                raise RuntimeError("FRONTIER_EXPLORER_STOP_TIMEOUT")
        if self.proposal is not None and self.proposal.handle.is_cancel_requested:
            self.finish(self.proposal, "CANCELLED")
        return self.proposal

    @property
    def complete(self):
        return self.status == self.status_type.EXPLORATION_COMPLETE

    def stop(self):
        self.finish(self.proposal, "CANCELLED")
        self.session = None
        if self.status_subscription is not None:
            self.node.destroy_subscription(self.status_subscription)
            self.status_subscription = None
        if self.process is not None:
            if self.process.poll() is None:
                if not hasattr(self, "stop_started_ns") or self.stop_started_ns < self.started_ns:
                    self.stop_started_ns = time.monotonic_ns()
                    self.process.terminate()
            else:
                self.process.wait()
                self.process = None

    def close(self):
        self.stop()
        if self.process is not None:
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
        self.server.destroy()
        self.log.close()
