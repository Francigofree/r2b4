"""Pure differential-drive feasibility shared by planning and motion control.

No state or authority lives here. Configuration is derived from robot geometry
and the active speed map; consumers keep their existing layer ownership.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True, slots=True)
class WheelMotionLimits:
    track_width_m: float = 0.3557
    minimum_mps: float = 0.15
    maximum_mps: float = 0.582
    # Operating reserve, independent of calibration and encoder-fit quality.
    target_minimum_mps: float | None = None

    def __post_init__(self) -> None:
        for value in (self.track_width_m, self.minimum_mps, self.maximum_mps):
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError("wheel motion limits must be finite and positive")
        if self.minimum_mps > self.maximum_mps:
            raise ValueError("wheel minimum exceeds calibrated maximum")
        if self.target_minimum_mps is not None and (
            isinstance(self.target_minimum_mps, bool)
            or not math.isfinite(self.target_minimum_mps)
            or not self.minimum_mps <= self.target_minimum_mps <= self.maximum_mps
        ):
            raise ValueError("wheel target minimum must stay inside calibration")

    def wheels(self, v: float, omega: float) -> tuple[float, float]:
        turn = omega * self.track_width_m * 0.5
        return v - turn, v + turn

    @property
    def minimum_center_spin_rad_s(self) -> float:
        """Derived realizable spin floor; never a separately tuned parameter."""
        return 2 * self.minimum_mps / self.track_width_m

    @property
    def target_floor_mps(self) -> float:
        return self.minimum_mps if self.target_minimum_mps is None else self.target_minimum_mps

    @property
    def target_center_spin_rad_s(self) -> float:
        return 2 * self.target_floor_mps / self.track_width_m

    def linear_sample(self, index: int, count: int, maximum: float,
                      minimum_planning_mps: float = 0.0) -> float:
        """Keep the zero candidate and spend the other samples above the floor."""
        minimum = max(self.target_floor_mps, minimum_planning_mps)
        if index == 0 or maximum < minimum:
            return 0.0
        if count == 2:
            return maximum
        return minimum + (maximum-minimum) * (index-1) / (count-2)

    def constrain(self, v: float, omega: float, *, planning: bool = False,
                  max_v_mps: float = math.inf, max_omega_rad_s: float = math.inf,
                  max_curvature_rad_per_m: float = math.inf) -> tuple[float, float]:
        """Project a steady target jointly, preserving its signed wheel family.

        Body caps, curvature and wheel floors are one feasible region. Clamping
        omega before this operation can turn a one-wheel pivot into a rolling
        arc or erase a realizable counter-arc. L9 alone owns subfloor ramps.
        """
        minimum = self.target_floor_mps if planning else self.minimum_mps
        if not math.isfinite(v) or not math.isfinite(omega):
            raise ValueError("motion must be finite")
        if any(math.isnan(cap) or cap < 0 for cap in (max_v_mps, max_omega_rad_s, max_curvature_rad_per_m)):
            raise ValueError("motion caps must be nonnegative")
        half_track = self.track_width_m * .5
        turn = omega * half_track
        left, right = v - turn, v + turn
        if (abs(v) <= max_v_mps and abs(omega) <= max_omega_rad_s
                and (abs(v) <= 1e-12 or abs(omega) <= max_curvature_rad_per_m * abs(v))
                and all(abs(w) <= self.maximum_mps + 1e-12
                        and (abs(w) <= 1e-12 or abs(w) >= minimum - 1e-12)
                        for w in (left, right))):
            return v, omega

        # Fixed-family half planes a*v+b*turn <= c. This is a two-dimensional
        # projection with a fixed small candidate budget, not a search/solver.
        planes: list[tuple[float, float, float]] = []
        for a, b, value, cap in ((1., 0., v, max_v_mps), (0., 1., turn, max_omega_rad_s * half_track)):
            sign = 1. if value >= 0 else -1.
            planes.extend(((sign*a, sign*b, min(abs(value), cap)), (-sign*a, -sign*b, 0.)))
        for a, b, wheel in ((1., -1., left), (1., 1., right)):
            if abs(wheel) <= 1e-12:
                planes.extend(((a, b, 0.), (-a, -b, 0.)))
            else:
                sign = math.copysign(1., wheel)
                planes.extend(((sign*a, sign*b, self.maximum_mps), (-sign*a, -sign*b, -minimum)))
        if abs(v) > 1e-12 and math.isfinite(max_curvature_rad_per_m):
            sign = math.copysign(1., v)
            slope = max_curvature_rad_per_m * half_track
            planes.extend(((-slope*sign, 1., 0.), (-slope*sign, -1., 0.)))
        candidates = [(v, turn)]
        for index, (a, b, c) in enumerate(planes):
            distance = (a*v + b*turn - c) / (a*a + b*b)
            candidates.append((v - a*distance, turn - b*distance))
            for d, e, f in planes[:index]:
                determinant = a*e - b*d
                if abs(determinant) > 1e-12:
                    candidates.append(((c*e - b*f)/determinant, (a*f - c*d)/determinant))
        feasible = [(x, y) for x, y in candidates
                    if all(a*x + b*y <= c + 1e-12 for a, b, c in planes)
                    and (abs(v) <= 1e-12 or abs(x) > 1e-12)]
        if not feasible:
            return 0.0, 0.0
        x, y = min(feasible, key=lambda p: ((p[0]-v)**2 + (p[1]-turn)**2, p))
        return (0.0 if abs(x) <= 1e-12 else x,
                0.0 if abs(y) <= 1e-12 else y/half_track)

    def degraded_linear_cap(self, v: float, omega: float, scale: float) -> float:
        # Reserve room for steering at the reduced angular cap. The caller's
        # original envelope is always the upper bound, even below the floor.
        return min(v, max(v * scale, self.target_floor_mps + omega * scale * self.track_width_m / 2))

    def degraded_angular_cap(self, omega: float, scale: float) -> float:
        """Retain the slowest realizable turn inside the original hard cap."""
        return min(omega, max(omega * scale, self.target_center_spin_rad_s))

    def normal_samples(self, linear_count: int, angular_count: int, max_v: float,
                       max_omega: float, planning_minimum: float):
        """Bounded family with rolling arcs, one-wheel turns and tight arcs.

        Reserved cells replace duplicate/infeasible grid commands; the family
        size and stable indices do not grow on the worker-to-control transport.
        """
        single_max = min(self.maximum_mps, 2 * max_v, max_omega * self.track_width_m)
        for linear_index in range(linear_count):
            nominal_v = self.linear_sample(linear_index, linear_count, max_v, planning_minimum)
            for angular_index in range(angular_count):
                omega = max_omega * (2 * angular_index / (angular_count - 1) - 1)
                v = nominal_v
                if angular_index in (0, angular_count - 1):
                    direction = -1 if angular_index == 0 else 1
                    if linear_index in (1, 2) and single_max >= self.target_floor_mps:
                        moving = self.target_floor_mps if linear_index == 1 else single_max
                        v, omega = moving / 2, direction * moving / self.track_width_m
                    elif linear_index == 3 and max_omega > self.target_center_spin_rad_s:
                        v = min(max_v, (max_omega * self.track_width_m - 2 * self.target_floor_mps) / 2)
                        omega = direction * 2 * (v + self.target_floor_mps) / self.track_width_m
                yield linear_index, angular_index, v, omega


def velocity_quality(speed_mps: float, minimum_mps: float = 0.15,
                     unreliable_below_mps: float = 0.13) -> float:
    """Physical velocity-estimation confidence, separate from counter integrity.

    The estimator's measured confidence band is independent of actuator
    feasibility and planner sampling policy.
    A zero reading needs independent standstill evidence; it is not special
    permission to close a moving wheel's speed loop.
    """
    if abs(speed_mps) >= minimum_mps - 1e-12:
        return 1.0
    return min(1.0, max(0.0, (abs(speed_mps) - unreliable_below_mps)
                       / (minimum_mps - unreliable_below_mps)))


def validate_velocity_quality_band(unreliable_below_mps: float, reliable_mps: float) -> None:
    if any(isinstance(value, bool) or not isinstance(value, (int, float))
           or not math.isfinite(value) for value in (unreliable_below_mps, reliable_mps)):
        raise ValueError("encoder velocity quality thresholds must be finite numbers")
    if not 0.0 <= unreliable_below_mps < reliable_mps:
        raise ValueError("encoder velocity quality band must satisfy 0 <= lower < reliable")
