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


def velocity_quality(speed_mps: float, minimum_mps: float = 0.15) -> float:
    """Physical velocity-estimation confidence, separate from counter integrity.

    0.13--0.15 m/s is a transition band for the production 0.15 m/s boundary.
    A zero reading needs independent standstill evidence; it is not special
    permission to close a moving wheel's speed loop.
    """
    lower = minimum_mps * (13.0 / 15.0)
    return min(1.0, max(0.0, (abs(speed_mps) - lower) / (minimum_mps - lower)))
