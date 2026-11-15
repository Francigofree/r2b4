#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace r2b4_nav2 {
// Steady differential-drive feasibility, matching v3/wheel_motion.py. No state,
// timing, acceleration, navigation choice or motor authority lives here.
struct WheelLimits {
  double track, minimum, maximum, max_v, max_omega, max_curvature;

  std::pair<double, double> project(double v, double omega, int stop_mask = -1) const {
    if (!std::isfinite(v) || !std::isfinite(omega)) return {0., 0.};
    const double half = track * .5, turn = omega * half;
    const double left = v - turn, right = v + turn;
    const auto wheel_ok = [this](double w) {
      return std::abs(w) <= maximum + 1e-12 &&
             (std::abs(w) <= 1e-12 || std::abs(w) >= minimum - 1e-12);
    };
    if (std::abs(v) <= max_v && std::abs(omega) <= max_omega &&
        (std::abs(v) <= 1e-12 || std::abs(omega) <= max_curvature * std::abs(v)) &&
        wheel_ok(left) && wheel_ok(right)) return {v, omega};

    struct Plane { double a, b, c; };
    std::vector<Plane> planes;
    planes.reserve(12);
    auto bound = [&planes](double a, double b, double value, double cap) {
      const double sign = value >= 0. ? 1. : -1.;
      planes.push_back({sign*a, sign*b, std::min(std::abs(value), cap)});
      planes.push_back({-sign*a, -sign*b, 0.});
    };
    bound(1., 0., v, max_v);
    bound(0., 1., turn, max_omega * half);
    int wheel_index = 0;
    for (auto ab : {std::pair<double, double>{1., -1.}, {1., 1.}}) {
      const double a = ab.first, b = ab.second, w = a*v+b*turn;
      const bool stopped = stop_mask >= 0 && (stop_mask & (1 << wheel_index));
      ++wheel_index;
      if (std::abs(w) <= 1e-12 || stopped) {
        planes.push_back({a, b, 0.}); planes.push_back({-a, -b, 0.});
      } else {
        const double sign = std::copysign(1., w);
        planes.push_back({sign*a, sign*b,
            stop_mask < 0 ? maximum : std::min(maximum, std::abs(w))});
        planes.push_back({-sign*a, -sign*b, -minimum});
      }
    }
    if (std::abs(v) > 1e-12 && std::isfinite(max_curvature)) {
      const double slope = max_curvature * half, sign = std::copysign(1., v);
      planes.push_back({-slope*sign, 1., 0.});
      planes.push_back({-slope*sign, -1., 0.});
    }
    std::vector<std::pair<double, double>> candidates{{v, turn}};
    candidates.reserve(100);
    for (size_t i=0; i<planes.size(); ++i) {
      const auto p = planes[i];
      const double distance = (p.a*v+p.b*turn-p.c)/(p.a*p.a+p.b*p.b);
      candidates.push_back({v-p.a*distance, turn-p.b*distance});
      for (size_t j=0; j<i; ++j) {
        const auto q = planes[j];
        const double det = p.a*q.b-p.b*q.a;
        if (std::abs(det) > 1e-12)
          candidates.push_back({(p.c*q.b-p.b*q.c)/det, (p.a*q.c-p.c*q.a)/det});
      }
    }
    bool found = false;
    std::pair<double, double> best{0., 0.};
    double score = std::numeric_limits<double>::infinity();
    for (auto c : candidates) {
      if (stop_mask < 0 && std::abs(v) > 1e-12 && std::abs(c.first) <= 1e-12) continue;
      bool feasible = true;
      for (auto p : planes)
        if (p.a*c.first+p.b*c.second > p.c+1e-12) { feasible=false; break; }
      if (!feasible) continue;
      const double d = std::pow(c.first-v, 2)+std::pow(c.second-turn, 2);
      if (!found || d < score || (d == score && c < best)) {
        found = true; score = d; best = c;
      }
    }
    if (!found) return {0., 0.};
    // Match Python's exact body envelope after floating-point intersections
    // and conversion back from wheel turn to angular velocity.
    const auto body_bound = [](double value, double requested, double cap) {
      const double magnitude = std::min({std::max(0., std::copysign(1., requested)*value),
                                         std::abs(requested), cap});
      return magnitude > 1e-12 ? std::copysign(magnitude, requested) : 0.;
    };
    return {body_bound(best.first, v, max_v), body_bound(best.second/half, omega, max_omega)};
  }

  // MPPI averages can cross the non-convex wheel deadband. Retain the canonical
  // fixed-family projection too: reducing curvature can raise the inner wheel
  // to its floor without amplifying either body axis. Stopped-wheel branches
  // add feasible pivots, but must not exclude such ordinary rolling arcs.
  std::pair<double, double> projectMppi(double v, double omega) const {
    if (!std::isfinite(v) || !std::isfinite(omega) || track <= 0.) return {0., 0.};
    const double half=track*.5, turn=omega*half;
    std::pair<double,double> best{0.,0.};
    double score=v*v+turn*turn;
    auto offer=[&](std::pair<double,double> candidate) {
      const double value=std::pow(candidate.first-v,2)+std::pow((candidate.second-omega)*half,2);
      if (value+1e-14<score) {score=value; best=candidate;}
    };
    offer(project(v,omega));
    for (int branch=0;branch<4;++branch) offer(project(v,omega,branch));
    const double left=v-turn,right=v+turn;
    if (left*right<0.) {
      const double magnitude=std::min({std::abs(turn),std::abs(left),std::abs(right),
                                     max_omega*half,maximum});
      if (magnitude>=minimum && max_v>=0.) offer({0.,std::copysign(magnitude/half,omega)});
    }
    return best;
  }
};
}  // namespace r2b4_nav2
