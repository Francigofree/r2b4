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

    def __post_init__(self) -> None:
        for value in (self.track_width_m, self.minimum_mps, self.maximum_mps):
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError("wheel motion limits must be finite and positive")
        if self.minimum_mps > self.maximum_mps:
            raise ValueError("wheel minimum exceeds calibrated maximum")

    def wheels(self, v: float, omega: float) -> tuple[float, float]:
        turn = omega * self.track_width_m * 0.5
        return v - turn, v + turn

    @property
    def minimum_center_spin_rad_s(self) -> float:
        """Derived realizable spin floor; never a separately tuned parameter."""
        return 2 * self.minimum_mps / self.track_width_m

    def linear_sample(self, index: int, count: int, maximum: float,
                      minimum_planning_mps: float = 0.0) -> float:
        """Keep the zero candidate and spend the other samples above the floor."""
        minimum = max(self.minimum_mps, minimum_planning_mps)
        if index == 0 or maximum < minimum:
            return 0.0
        if count == 2:
            return maximum
        return minimum + (maximum-minimum) * (index-1) / (count-2)

    def constrain(self, v: float, omega: float) -> tuple[float, float]:
        """Reduce a steady target to its feasible envelope; never amplify it.

        Ordinary translation keeps both wheels rolling. Exact one-wheel pivots
        and spins are allowed when their moving wheels are themselves feasible.
        A target below the floor becomes zero; L9 owns the finite transition.
        """
        left, right = self.wheels(v, omega)
        scale = min(1.0, self.maximum_mps / max(abs(left), abs(right), 1e-12))
        v, omega = v * scale, omega * scale
        left, right = self.wheels(v, omega)
        if all(abs(w) < 1e-12 or abs(w) >= self.minimum_mps - 1e-12 for w in (left, right)):
            return v, omega
        if abs(v) < self.minimum_mps - 1e-12:
            return 0.0, 0.0
        turn_limit = max(0.0, 2 * (abs(v) - self.minimum_mps) / self.track_width_m)
        return v, math.copysign(min(abs(omega), turn_limit), omega)

    def degraded_linear_cap(self, v: float, omega: float, scale: float) -> float:
        # Reserve room for steering at the reduced angular cap. The caller's
        # original envelope is always the upper bound, even below the floor.
        return min(v, max(v * scale, self.minimum_mps + omega * scale * self.track_width_m / 2))


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
