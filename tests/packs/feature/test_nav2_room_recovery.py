"""Full-body RC goal and recovery evidence, without ROS or motor output."""
import math
import random
from types import SimpleNamespace as NS

from v3.adapters.nav2_footprint import footprint_check, padded_footprint, sweep_check
from v3.adapters.room_cruise_recovery import RoomCruiseRecovery

BOUNDS = (-.10, .26, -.18, .18)
PAD = .10


def test_padding_encloses_euclidean_margin_and_preserves_asymmetric_body():
    polygon = padded_footprint(BOUNDS, PAD)
    assert len(polygon) == 16
    for index in range(4096):
        a = index*2*math.pi/4096
        nx, ny = math.cos(a), math.sin(a)
        body = max(nx*x+ny*y for x in BOUNDS[:2] for y in BOUNDS[2:])
        padded = max(nx*x+ny*y for x, y in polygon)
        assert PAD-1e-12 <= padded-body <= PAD/math.cos(math.pi/16)+1e-12


def test_live_corner_false_block_is_clear_but_inside_official_margin_is_blocked():
    # Full retained lethal cell [3.35,3.40] x [1.95,2.00], not its center.
    # Independent exact body/cell distance is .110962567494184 m.
    g = grid()
    g.info.origin.position.x, g.info.origin.position.y = 2.1, .9
    g.data[21*100+25] = 100
    pose = (3.628266586, 2.350352139, -1.422011923)
    assert footprint_check(g, *pose, BOUNDS, PAD) is None
    assert sweep_check(g, pose, -.19, 0., .25/.19, BOUNDS, PAD)["ok"]
    # A 2cm move toward the cell must still fail at t=0.
    hit = footprint_check(g, pose[0]-.02, pose[1]-.02, pose[2], BOUNDS, PAD)
    assert hit["reason"] == "lethal_cell" and hit["cell"] == [25, 21]


def test_swept_polygon_never_accepts_dense_independent_body_distance_violation():
    from tools.navigation_simulator.scenario import World
    g = grid()
    obstacle(g, .025, .025)
    world = World(dict(room=dict(x_min_m=-10., y_min_m=-10., x_max_m=10., y_max_m=10.),
        walls=[[0., 0., .05, .05]], obstacles=[], moving_obstacles=[]), BOUNDS)
    rng = random.Random(110962)
    violations = 0
    for _ in range(100):
        x, y, yaw = rng.uniform(-.7, .7), rng.uniform(-.7, .7), rng.uniform(-math.pi, math.pi)
        v, w = rng.uniform(-.3, .45), rng.uniform(-1.4, 1.4)
        minimum = 10.
        for index in range(101):
            t = index/100
            half = w*t/2
            distance = v*t*(math.sin(half)/half if abs(half) > 1e-12 else 1.)
            minimum = min(minimum, world.clearance(x+distance*math.cos(yaw+half),
                y+distance*math.sin(yaw+half), yaw+w*t, 0.))
        if minimum < PAD:
            violations += 1
            assert not sweep_check(g, (x, y, yaw), v, w, 1., BOUNDS, PAD)["ok"]
    assert violations >= 20


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
