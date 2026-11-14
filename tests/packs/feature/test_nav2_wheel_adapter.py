"""Native Nav2 wheel projection matches V3's calibrated steady envelope.

The standalone C++ helper has no ROS, GPIO, device or motor capability. The
geometry check keeps Nav2's proposal ramp separate from the physical L9 ramp.
"""
import math
import random
import re
import shutil
import subprocess

import pytest

from rig import ROOT, healthy_localization, resolved_config
from tools.nav2_planner import _write_geometry
from v3.contracts import (LOCAL_FRAME_ID, MotionIntent, RobotEstimate, TickContext)
from v3.layers.l9_operational_constraints import OperationalConstraintLayer
from v3.wheel_motion import WheelMotionLimits


def test_native_mppi_projection_recheck_rejects_collision_unknown_and_map_exit(tmp_path):
    prefix = ROOT / "runtime" / "deps" / "nav2-planner"
    if not (prefix / "lib/libmppi_controller.so").is_file():
        pytest.skip("Native MPPI integration requires the isolated ROS prefix")
    from tools.nav2_planner import _navigation_environment, _paths
    env = _navigation_environment(_paths(ROOT), ROOT)
    (tmp_path / "collision.cpp").write_text(r'''
#include "controller.cpp"
#include <iostream>
int main() {
  nav2_costmap_2d::Costmap2D map(100, 100, .05, -2.5, -2.5, 0);
  nav2_costmap_2d::Footprint footprint(4);
  footprint[0].x=-.1; footprint[0].y=-.1;
  footprint[1].x=.1; footprint[1].y=-.1;
  footprint[2].x=.1; footprint[2].y=.1;
  footprint[3].x=-.1; footprint[3].y=.1;
  auto check = [&](double x, double v, double omega) {
    std::cout << r2b4_nav2::commandCollides(&map, footprint, x, 0., 0., v, omega) << "\n";
  };
  check(0., .3, 0.); // free
  map.setCost(57, 50, nav2_costmap_2d::LETHAL_OBSTACLE);
  check(0., .3, 0.); // obstacle ahead
  check(0., -.3, 0.); // reverse remains free
  map.setCost(57, 50, nav2_costmap_2d::NO_INFORMATION);
  check(0., .3, 0.); // unknown is not free
  map.setCost(57, 50, 0);
  check(2.3, .3, 0.); // exits map
  map.setCost(54, 54, nav2_costmap_2d::LETHAL_OBSTACLE);
  check(0., .3, 1.4); // exact curved arc hits
  check(0., .3, -1.4); // opposite arc remains free
  map.setCost(50, 50, nav2_costmap_2d::LETHAL_OBSTACLE);
  check(0., 0., 0.); // t=0 checked even for a stop
}
''')
    (tmp_path / "CMakeLists.txt").write_text('''
cmake_minimum_required(VERSION 3.16)
project(collision_probe LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
add_executable(collision collision.cpp)
target_include_directories(collision PRIVATE "''' + str(ROOT / "deploy/nav2_wheel_adapter") + '''")
target_link_directories(collision PRIVATE "''' + str(prefix / "lib") + '''")
ament_target_dependencies(collision nav2_mppi_controller nav2_core nav2_costmap_2d nav2_util pluginlib)
target_link_libraries(collision "''' + str(prefix / "lib/libmppi_controller.so") + '''")
''')
    cmake = str(prefix / "bin/cmake")
    build = tmp_path / "build"
    for command in ([cmake, "-S", str(tmp_path), "-B", str(build),
                     "-DCMAKE_C_COMPILER=/usr/bin/gcc", "-DCMAKE_CXX_COMPILER=/usr/bin/g++"],
                    [cmake, "--build", str(build), "--parallel", "1"]):
        result = subprocess.run(command, env=env, capture_output=True, text=True, timeout=120)
        assert result.returncode == 0, result.stderr[-4000:]
    result = subprocess.run([str(build / "collision")], env=env,
        capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == ["0", "1", "0", "1", "1", "1", "0", "1"]


@pytest.fixture(scope="module")
def native_projector(tmp_path_factory):
    compiler = shutil.which("g++")
    if compiler is None:
        pytest.skip("C++ equivalence evidence requires g++")
    work = tmp_path_factory.mktemp("nav2-wheel-projection")
    source = work / "project.cpp"
    binary = work / "project"
    source.write_text(r'''
#include "wheel_projection.hpp"
#include <array>
#include <iomanip>
#include <iostream>
#include <string>

int main(int argc, char **) {
  std::array<double, 8> row{};
  std::string value;
  std::cout << std::setprecision(17);
  while (std::cin >> value) {
    row[0] = std::stod(value);
    for (size_t i = 1; i < row.size(); ++i) {
      if (!(std::cin >> value)) return 2;
      row[i] = std::stod(value);
    }
    const r2b4_nav2::WheelLimits limits{row[0], row[1], row[2], row[3], row[4], row[5]};
    const auto result = argc > 1 ? limits.projectMppi(row[6], row[7]) : limits.project(row[6], row[7]);
    std::cout << result.first << " " << result.second << "\n";
  }
}
''')
    build = subprocess.run([compiler, "-std=c++17", "-O2", "-I",
        str(ROOT / "deploy" / "nav2_wheel_adapter"), str(source), "-o", str(binary)],
        capture_output=True, text=True, timeout=30)
    assert build.returncode == 0, build.stderr

    def project(rows, *, mppi=False):
        request = "".join(" ".join(repr(value) for value in row) + "\n" for row in rows)
        result = subprocess.run([str(binary)] + (["mppi"] if mppi else []), input=request, capture_output=True,
            text=True, timeout=10)
        assert result.returncode == 0, result.stderr
        outputs = [tuple(float(value) for value in line.split()) for line in result.stdout.splitlines()]
        assert len(outputs) == len(rows)
        return outputs

    return project


def _assert_envelope(limits, v, omega, caps, projected):
    result_v, result_omega = projected
    tolerance = 2e-10
    assert abs(result_v) <= min(abs(v), caps[0]) + tolerance
    assert abs(result_omega) <= min(abs(omega), caps[1]) + tolerance
    if abs(result_v) > tolerance:
        assert abs(result_omega) <= caps[2] * abs(result_v) + tolerance
    source_wheels = limits.wheels(v, omega)
    wheels = limits.wheels(result_v, result_omega)
    for source, actual in zip(source_wheels, wheels):
        assert abs(actual) <= limits.maximum_mps + tolerance
        assert abs(actual) <= tolerance or abs(actual) >= limits.minimum_mps - tolerance
        if abs(source) <= 1e-12:
            assert abs(actual) <= tolerance
        else:
            assert source * actual >= -tolerance


def test_cpp_projection_matches_python_for_signed_families_floors_caps_and_fixed_seed(native_projector):
    control = resolved_config().runtime.composition.live_control.control
    production = control.operational_constraints
    limits = production.wheel_limits
    cases = []

    def add(wheels, v, omega, caps=None):
        if caps is None:
            caps = (production.max_v_mps, production.max_omega_rad_s,
                    production.max_curvature_rad_per_m)
        cases.append((wheels, v, omega, caps))

    # Known changed-curvature examples, signed rolling arcs and counter-arcs.
    for sign_v in (-1., 1.):
        for sign_omega in (-1., 1.):
            for v, omega in ((.19, .6), (.25, 1.), (.07, 1.3), (.45, 1.4), (.8, 7.)):
                add(limits, sign_v * v, sign_omega * omega)
    # The deadband, steady boundary and exact stopped-wheel family remain
    # meaningful in both wheel directions and under infeasible body caps.
    for reserve in (-1e-9, 0., 1e-9):
        wheel = limits.minimum_mps + reserve
        for sign in (-1., 1.):
            add(limits, sign * wheel, 0.)
            add(limits, 0., sign * 2 * wheel / limits.track_width_m)
            for spin in (-1., 1.):
                add(limits, sign * wheel / 2, spin * wheel / limits.track_width_m)
    for caps in ((0., 1.4, 32.), (.45, 0., 32.), (.1, .2, .5),
                 (.45, 1.4, 0.), (math.inf, math.inf, math.inf)):
        for v, omega in ((0., 0.), (.2, .7), (-.2, -.7), (0., 1.), (.05, 2.)):
            add(limits, v, omega, caps)
    rng = random.Random(0x4E415632)
    for _ in range(1000):
        wheels = WheelMotionLimits(track_width_m=rng.uniform(.2, .8),
            minimum_mps=rng.uniform(.05, .25), maximum_mps=rng.uniform(.4, 1.2))
        add(wheels, rng.uniform(-1.5, 1.5), rng.uniform(-10., 10.),
            (rng.uniform(0., 1.2), rng.uniform(0., 6.), rng.uniform(.1, 40.)))
    rows = [(wheel.track_width_m, wheel.minimum_mps, wheel.maximum_mps,
             *caps, v, omega) for wheel, v, omega, caps in cases]
    for case, actual in zip(cases, native_projector(rows)):
        wheel, v, omega, caps = case
        expected = wheel.constrain(v, omega, max_v_mps=caps[0],
            max_omega_rad_s=caps[1], max_curvature_rad_per_m=caps[2])
        assert actual == pytest.approx(expected, rel=2e-10, abs=2e-11), case
        _assert_envelope(wheel, v, omega, caps, actual)


def _scalar(text, name):
    values = re.findall(r"^\s*" + re.escape(name) + r":\s*([-+0-9.eE]+)\s*$", text, re.MULTILINE)
    assert len(values) == 1, (name, values)
    return float(values[0])


def test_nav2_geometry_crosses_deadband_but_physical_l9_ramp_and_goal_contract_stay_bounded(tmp_path):
    control = resolved_config().runtime.composition.live_control.control
    physical = control.operational_constraints
    generated = tmp_path / "geometry.yaml"
    _write_geometry({"geometry": generated, "bt": ROOT / "conf" / "nav2_navigate.xml"}, control)
    geometry = generated.read_text()
    source = (ROOT / "conf" / "nav2_planner.yaml").read_text()
    period = 1. / _scalar(source, "controller_frequency")
    proposal_acceleration = _scalar(geometry, "az_max")
    assert proposal_acceleration * period >= physical.wheel_limits.target_center_spin_rad_s
    assert _scalar(geometry, "ax_max") * period >= physical.wheel_limits.target_minimum_mps
    assert _scalar(geometry, "wz_max") >= physical.wheel_limits.target_center_spin_rad_s
    assert 0. < _scalar(geometry, "xy_goal_tolerance") <= control.mission.default_constraints.goal_tolerance_m
    assert 0. < _scalar(geometry, "yaw_goal_tolerance") <= control.mission.default_constraints.yaw_tolerance_rad
    for key, expected in {
        "track_width_m": physical.wheel_limits.track_width_m,
        "wheel_minimum_mps": physical.wheel_limits.minimum_mps,
        "wheel_maximum_mps": physical.wheel_limits.maximum_mps,
        "platform_max_v_mps": physical.max_v_mps,
        "platform_max_omega_rad_s": physical.max_omega_rad_s,
        "max_curvature_rad_per_m": physical.max_curvature_rad_per_m,
    }.items():
        assert _scalar(geometry, key) == expected

    # Feed the actual Nav2 startup proposal into canonical L9. Its physical
    # acceleration remains bounded by V3's injected platform value.
    limiter = OperationalConstraintLayer(physical)
    constraints = control.mission.default_constraints
    for tick, requested in ((0, 0.), (1, physical.wheel_limits.target_center_spin_rad_s)):
        context = TickContext(tick, 1_000_000_000 + tick * 20_000_000)
        estimate = RobotEstimate(context, LOCAL_FRAME_ID, 0., 0., 0., 0., 0.,
            (0.,) * 25, localization_quality=healthy_localization())
        motion = MotionIntent(context, 0., requested, 20_000_000, constraints,
            transition_allowed=True)
        allowed = limiter.evaluate(motion, estimate)
    assert 0. < allowed.allowed_omega_rad_s <= physical.max_angular_acceleration_rad_s2 * .02
    assert allowed.allowed_omega_rad_s < proposal_acceleration * .02


def test_mppi_stop_branches_retain_drivable_arcs_spin_and_reverse_without_amplification(native_projector):
    limits = WheelMotionLimits(track_width_m=.3557, minimum_mps=.15, maximum_mps=.582)
    caps = (.45, 1.4, 32.)
    cases = [(.1087, -.8206), (-.1087, .8206), (0., .9), (.001, 1.),
             (.2, .2), (-.2, 0.), (.1, 0.), (0., .5)]
    rng = random.Random(0x5243)
    cases += [(rng.uniform(-.45, .45), rng.uniform(-1.4, 1.4)) for _ in range(1000)]
    actual = native_projector([(limits.track_width_m, .15, .582, *caps, v, w)
                              for v, w in cases], mppi=True)
    assert actual[0] == pytest.approx((.1087, -2*.1087/.3557))
    assert actual[1] == pytest.approx((-.1087, 2*.1087/.3557))
    assert actual[2] == pytest.approx((0., .9))
    assert actual[3][0] == 0. and actual[3][1] > .9
    assert actual[4:8] == pytest.approx([(.2, .2), (-.2, 0.), (0., 0.), (0., 0.)])
    for (v, w), result in zip(cases, actual):
        _assert_envelope(limits, v, w, caps, result)
        for source, projected in zip(limits.wheels(v, w), limits.wheels(*result)):
            assert abs(projected) <= abs(source)+2e-10
        # The ROS adaptation enters V3's unchanged physical envelope unchanged.
        assert limits.constrain(*result, max_v_mps=caps[0], max_omega_rad_s=caps[1],
            max_curvature_rad_per_m=caps[2]) == pytest.approx(result, abs=2e-10)
