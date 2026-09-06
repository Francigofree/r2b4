"""L3 zero-state estimator for the fake-only STOP vertical slice."""

from __future__ import annotations

from dataclasses import dataclass

from v3.contracts import AdmittedFrame, RobotEstimate


_ZERO_COVARIANCE = (0.0,) * 25


@dataclass(frozen=True, slots=True)
class ZeroStateEstimator:
    frame_id: str

    def __call__(self, frame: AdmittedFrame) -> RobotEstimate:
        return RobotEstimate(
            frame.context,
            self.frame_id,
            x_m=0.0,
            y_m=0.0,
            yaw_rad=0.0,
            v_mps=0.0,
            omega_rad_s=0.0,
            covariance_5x5=_ZERO_COVARIANCE,
        )


__all__ = ["ZeroStateEstimator"]
