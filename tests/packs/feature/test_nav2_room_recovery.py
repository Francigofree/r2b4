"""Full-body RC goal and recovery evidence, without ROS or motor output."""
import math
from types import SimpleNamespace as NS

from v3.adapters.nav2_footprint import footprint_check, sweep_check
from v3.adapters.room_cruise_recovery import RoomCruiseRecovery

BOUNDS = (-.10, .26, -.18, .18)
PAD = .10


def grid(value=0):
    return NS(header=NS(frame_id="R2B4_ODOM_LOCAL"), info=NS(width=100, height=100,
        resolution=.05, origin=NS(position=NS(x=-2.5, y=-2.5),
        orientation=NS(x=0., y=0., z=0., w=1.))), data=[value]*10000)


def obstacle(g, x, y, value=100):
    ix, iy = math.floor((x+2.5)/.05), math.floor((y+2.5)/.05)
    g.data[iy*100+ix] = value
    return [ix, iy]


def test_full_asymmetric_body_blocks_side_rear_front_and_rotated_corner_with_clear_center():
    for x, y, yaw in ((0., .24, 0.), (-.18, 0., 0.), (.33, 0., 0.), (.30, .20, math.pi/4)):
        g = grid()
        cell = obstacle(g, x, y)
        hit = footprint_check(g, 0., 0., yaw, BOUNDS, PAD)
        assert hit["cell"] == cell and hit["cost"] == 100
    g = grid()
    obstacle(g, .42, .42)
    assert footprint_check(g, 0., 0., math.pi/4, BOUNDS, PAD) is None


def test_inflation_is_not_a_second_margin_and_unknown_or_map_exit_is_never_free():
    assert footprint_check(grid(99), 0., 0., 0., BOUNDS, PAD) is None
    g = grid()
    obstacle(g, .2, 0., -1)
    assert footprint_check(g, 0., 0., 0., BOUNDS, PAD)["reason"] == "unknown_cell"
    assert footprint_check(grid(), 2.3, 0., 0., BOUNDS, PAD)["reason"] == "outside_grid"
    assert not sweep_check(grid(), (0., 0., 0.), 10., 10., 10., BOUNDS, PAD)["ok"]


def test_sweep_checks_reverse_interior_and_pure_rotation_and_preserves_first_cell():
    g = grid()
    cell = obstacle(g, -.4, 0.)
    result = sweep_check(g, (0., 0., 0.), -.19, 0., .25/.19, BOUNDS, PAD)
    assert not result["ok"] and result["first_blocking"]["cell"] == cell
    assert result["first_blocking"]["sample"] > 0
    assert sweep_check(g, (0., 0., 0.), .19, 0., 1., BOUNDS, PAD)["ok"]
    g = grid()
    obstacle(g, .42, .04)
    assert footprint_check(g, 0., 0., 0., BOUNDS, PAD) is None
    assert not sweep_check(g, (0., 0., 0.), 0., -1.1, .7/1.1, BOUNDS, PAD)["ok"]


def test_recovery_consumes_failed_candidates_and_native_success_never_resolves_episode():
    r = RoomCruiseRecovery(BOUNDS, PAD)
    now = 1_000_000_000
    r.begin([1., 0., 0.], now, "stalled")
    first = r.choose(grid(), now, (0., 0., 0.), (0., 0., 0.), now)
    r.action_finished(False, (0., 0., 0.), now)
    second = r.choose(grid(), now, (0., 0., 0.), (0., 0., 0.), now)
    assert first["name"] != second["name"]
    r.action_finished(True, (-.25, 0., 0.), now)
    assert r.state == "VERIFY" and r.counts["progress_verified"] == 0
    r.feedback(1., now+1)
    # The successful .25m BackUp itself is excluded from post-maneuver progress.
    assert r.verify((-.25, 0., 0.), .8, now+2, now+2) is None
    assert r.verify((-.25, 0., 0.), .8, now+6_000_000_000, now+6_000_000_000) is False
    r.begin([1., 0., 0.], now+6_000_000_000, "stalled_again")
    assert {first["name"], second["name"]} <= r.tried
    third = r.choose(grid(), now+6_000_000_000, (0., 0., 0.), (0., 0., 0.), now+6_000_000_000)
    assert third["name"] not in {first["name"], second["name"]}


def test_recovery_requires_fresh_feedback_plus_post_maneuver_translation_and_goal_progress():
    r = RoomCruiseRecovery(BOUNDS, PAD)
    now = 1_000_000_000
    r.begin([1., 0., 0.], now, "stalled")
    r.choose(grid(), now, (0., 0., 0.), (0., 0., 0.), now)
    r.action_finished(True, (-.25, 0., 0.), now)
    r.feedback(1., now+1)
    assert r.verify((.05, 0., 0.), .99, now+2, now+2) is None
    assert r.verify((.05, 0., 0.), .8, now+2, now+2) is True
    assert r.state == "RESOLVED" and r.counts["progress_verified"] == 1
    r.reset()
    r.begin([2., 0., 0.], now, "stalled")
    assert r.choose(grid(), now-500_000_001, (0., 0., 0.), (0., 0., 0.), now) is None
    assert r.counts["attempts"] == 1


def test_initial_zero_feedback_does_not_poison_resumed_navigation_progress_baseline():
    r = RoomCruiseRecovery(BOUNDS, PAD)
    now = 1_000_000_000
    r.begin([2., 0., 0.], now, "stalled")
    r.choose(grid(), now, (0., 0., 0.), (0., 0., 0.), now)
    r.action_finished(True, (-.25, 0., 0.), now)
    r.feedback(0., now+1)
    r.feedback(-1., now+2)
    assert r.baseline_remaining is None
    assert r.deadline_ns == now+7_000_000_000
    r.feedback(2., now+3)
    assert r.verify((.05, 0., 0.), 1.8, now+4, now+4) is True
    assert r.counts["progress_verified"] == 1
