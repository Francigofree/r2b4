"""ROS-free evidence for frontier ingress and child-goal revocation."""
from concurrent.futures import Future
from types import SimpleNamespace
import math

import pytest

from v3.adapters.nav2_navigation import HEARTBEAT_MAX_AGE_NS
from v3.adapters.ros2_frontier import FrontierProposal, Ros2FrontierExplorer, frontier_pose
from v3.contracts import GLOBAL_FRAME_ID


NOW = 4_000_000_000
OFFSET = 1_700_000_000_000_000_000
STARTED = NOW - 1_000_000_000


def _goal(measured=NOW, *, frame=GLOBAL_FRAME_ID, behavior_tree=""):
    stamp = measured + OFFSET
    return SimpleNamespace(behavior_tree=behavior_tree, pose=SimpleNamespace(
        header=SimpleNamespace(frame_id=frame,
            stamp=SimpleNamespace(sec=stamp // 1_000_000_000, nanosec=stamp % 1_000_000_000)),
        pose=SimpleNamespace(position=SimpleNamespace(x=1., y=-2., z=0.),
            orientation=SimpleNamespace(x=0., y=0., z=math.sin(.3), w=math.cos(.3)))))


def _pose(goal, *, started=STARTED):
    return frontier_pose(goal, now_ns=NOW, started_ns=started, ros_offset_ns=OFFSET)


def test_frontier_preserves_map_pose_and_original_measurement_age():
    assert _pose(_goal()) == pytest.approx((1., -2., .6))
    assert _pose(_goal(NOW - HEARTBEAT_MAX_AGE_NS)) == pytest.approx((1., -2., .6))
    assert _pose(_goal(STARTED), started=STARTED) is None  # Started is not freshness.
    assert _pose(_goal(NOW), started=NOW) == pytest.approx((1., -2., .6))


@pytest.mark.parametrize("measured,started", [
    (NOW + 1, STARTED),
    (NOW - HEARTBEAT_MAX_AGE_NS - 1, STARTED),
    (NOW - 1, NOW),  # A delayed goal from a previous explorer generation.
])
def test_frontier_rejects_future_expired_and_previous_generation_proposals(measured, started):
    assert _pose(_goal(measured), started=started) is None


@pytest.mark.parametrize("frame,behavior_tree", [
    ("odom", ""), ("", ""), (GLOBAL_FRAME_ID, "/tmp/injected-tree.xml"),
])
def test_frontier_cannot_change_frame_or_action_behavior_tree(frame, behavior_tree):
    assert _pose(_goal(frame=frame, behavior_tree=behavior_tree)) is None


@pytest.mark.parametrize("part,component", [
    ("position", "x"), ("position", "y"), ("position", "z"),
    ("orientation", "x"), ("orientation", "y"),
    ("orientation", "z"), ("orientation", "w"),
])
@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_frontier_rejects_every_nonfinite_pose_component(part, component, value):
    goal = _goal()
    setattr(getattr(goal.pose.pose, part), component, value)
    assert _pose(goal) is None


@pytest.mark.parametrize("part,component,value", [
    ("position", "z", .1), ("orientation", "x", .1),
    ("orientation", "y", .1), ("orientation", "w", 0.),
])
def test_frontier_rejects_nonplanar_or_nonunit_pose(part, component, value):
    goal = _goal()
    setattr(getattr(goal.pose.pose, part), component, value)
    assert _pose(goal) is None


class _Process:
    def __init__(self):
        self.returncode = None
        self.terminations = 0
        self.waits = 0

    def poll(self):
        return self.returncode

    def terminate(self):
        self.terminations += 1

    def wait(self):
        assert self.returncode is not None
        self.waits += 1
        return self.returncode


def _explorer():
    explorer = Ros2FrontierExplorer.__new__(Ros2FrontierExplorer)
    explorer.session = ("mission", "goal", 1)
    explorer.process = _Process()
    explorer.started_ns = 1
    explorer.status_subscription = None
    explorer.proposal = FrontierProposal((1., 2., 0.),
        SimpleNamespace(is_cancel_requested=False), Future())
    return explorer


def test_frontier_cancel_revokes_only_its_proposal_and_does_not_stop_provider():
    explorer = _explorer()
    old = explorer.proposal
    old.handle.is_cancel_requested = True
    assert explorer.poll() is None
    assert old.completion.result() == ("CANCELLED", 0)
    assert explorer.session is not None and explorer.process.terminations == 0
    replacement = FrontierProposal((3., 4., 0.), object(), Future())
    explorer.proposal = replacement
    explorer.finish(old, "SUCCEEDED")
    assert old.completion.result() == ("CANCELLED", 0)
    assert explorer.proposal is replacement and not replacement.completion.done()


def test_active_frontier_commitment_survives_new_map_goal_proposals_until_real_outcome():
    explorer = _explorer()
    original = explorer.proposal
    assert explorer.proposal_committed
    # Goal admission rejects a replacement without completing/revoking the
    # current proposal. The bridge can keep its original child identity.
    assert explorer.proposal is original and not original.completion.done()
    explorer.finish(original, "SUCCEEDED")
    assert not explorer.proposal_committed
    explorer.proposal = FrontierProposal((3., 4., 0.),
        SimpleNamespace(is_cancel_requested=True), Future())
    assert not explorer.proposal_committed
    explorer.poll()
    assert explorer.proposal is None


def test_frontier_session_stop_is_nonblocking_and_waits_for_process_reap():
    explorer = _explorer()
    proposal, process = explorer.proposal, explorer.process
    explorer.stop()
    explorer.stop()
    assert explorer.session is None and explorer.proposal is None
    assert proposal.completion.result() == ("CANCELLED", 0)
    assert process.terminations == 1 and process.waits == 0
    assert not explorer.start(("new-mission", "new-goal", 2))
    process.returncode = -15
    assert explorer.poll() is None
    assert explorer.process is None and process.waits == 1


def test_frontier_process_crash_is_not_treated_as_exploration_completion():
    explorer = _explorer()
    explorer.process.returncode = 1
    with pytest.raises(RuntimeError, match="FRONTIER_EXPLORER_EXITED"):
        explorer.poll()


def test_frontier_status_is_isolated_from_previous_explorer_generation(monkeypatch):
    explorer = _explorer()
    explorer.process = explorer.session = None
    subscriptions, destroyed, commands = [], [], []

    def subscribe(message_type, topic, callback, depth):
        subscription = SimpleNamespace(topic=topic, callback=callback)
        subscriptions.append(subscription)
        return subscription

    def launch(command, **kwargs):
        commands.append(command)
        return _Process()

    explorer.node = SimpleNamespace(create_subscription=subscribe,
        destroy_subscription=destroyed.append)
    explorer.status_type = object()
    explorer.executable, explorer.parameters, explorer.log = "/explore", "/params.yaml", object()
    monkeypatch.setattr("v3.adapters.ros2_frontier.subprocess.Popen", launch)
    assert explorer.start(("first", "goal", 1))
    timing = dict(explorer.startup_timing)
    assert timing["generation"] == explorer.started_ns
    assert timing["phase"] == "READY"
    assert all(timing[name] >= 0 for name in ("previous_cleanup_ns", "subscription_ns", "launch_ns"))
    assert explorer.start(("first", "goal", 1))
    assert explorer.startup_timing == timing  # A heartbeat cannot restart startup timing.
    old = subscriptions[-1]
    old.callback(SimpleNamespace(status="exploration_complete"))
    assert explorer.status == "exploration_complete"
    explorer.stop()
    assert old in destroyed
    explorer.process.returncode = -15
    explorer.poll()
    assert explorer.start(("second", "goal", 1))
    current = subscriptions[-1]
    assert old.topic != current.topic and explorer.status is None
    assert explorer.startup_timing["generation"] > timing["generation"]
    assert f"explore/status:={current.topic}" in commands[-1]
    old.callback(SimpleNamespace(status="exploration_complete"))
    assert explorer.status is None
    current.callback(SimpleNamespace(status="exploration_started"))
    assert explorer.status == "exploration_started"
