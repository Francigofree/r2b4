"""L3 localization truth and motion capability requirements."""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from .base import ContractValidationError, require_finite, require_token

LOCAL_FRAME_ID = "R2B4_ODOM_LOCAL"
GLOBAL_FRAME_ID = "R2B4_BOOT_ROBOT_MAP"


class QualityState(str, Enum):
    GOOD = "GOOD"
    DEGRADED = "DEGRADED"
    LOST = "LOST"


@dataclass(frozen=True, slots=True)
class Pose2D:
    frame_id: str
    x_m: float
    y_m: float
    yaw_rad: float

    def __post_init__(self) -> None:
        require_token(self.frame_id, "Pose2D.frame_id")
        for name in ("x_m", "y_m", "yaw_rad"):
            require_finite(getattr(self, name), name)

    def apply(self, x_m: float, y_m: float, yaw_rad: float | None = None):
        c, s = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        yaw = None if yaw_rad is None else math.atan2(
            math.sin(yaw_rad + self.yaw_rad), math.cos(yaw_rad + self.yaw_rad))
        return self.x_m + c*x_m - s*y_m, self.y_m + s*x_m + c*y_m, yaw

    def inverse(self, frame_id: str = LOCAL_FRAME_ID) -> Pose2D:
        c, s = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        return Pose2D(frame_id, -c*self.x_m - s*self.y_m,
                      s*self.x_m - c*self.y_m, -self.yaw_rad)


@dataclass(frozen=True, slots=True)
class LocalizationRequirement:
    local_translation: bool = True
    heading: bool = True
    global_position: bool = True

    def __post_init__(self) -> None:
        if any(type(value) is not bool for value in (
            self.local_translation, self.heading, self.global_position,
        )):
            raise ContractValidationError("localization requirements must be bool")


@dataclass(frozen=True, slots=True)
class LocalizationQuality:
    # Absence of evidence must never produce motion authority.
    local_translation: QualityState = QualityState.LOST
    heading: QualityState = QualityState.LOST
    global_position: QualityState = QualityState.LOST
    local_sigma_m: float = 1.0
    global_sigma_m: float = 1.0
    yaw_sigma_rad: float = 1.0
    encoder_age_ns: int = 2**63-1
    imu_age_ns: int = 2**63-1
    lidar_age_ns: int = 2**63-1
    global_fix_age_ns: int = 2**63-1
    local_pose_continuous: bool = False
    pose_discontinuity: bool = False
    lidar_relative_consistency_m: float | None = None
    encoder_lidar_consistency_m: float | None = None
    generation: int = 0
    relative_age_ns: int = 2**63-1
    slip_suspected: bool = False
    observability: float = 0.0

    def __post_init__(self) -> None:
        for name in ("local_translation", "heading", "global_position"):
            if not isinstance(getattr(self, name), QualityState):
                raise ContractValidationError("invalid localization quality state")
        for name in ("local_sigma_m", "global_sigma_m", "yaw_sigma_rad", "observability",
                     "lidar_relative_consistency_m", "encoder_lidar_consistency_m"):
            value = getattr(self, name)
            if value is not None:
                require_finite(value, name)
                if value < 0:
                    raise ContractValidationError(f"negative {name}")
        for name in ("encoder_age_ns", "imu_age_ns", "lidar_age_ns", "global_fix_age_ns",
                     "relative_age_ns", "generation"):
            if type(getattr(self, name)) is not int or getattr(self, name) < 0:
                raise ContractValidationError(f"invalid {name}")
        for name in ("local_pose_continuous", "pose_discontinuity", "slip_suspected"):
            if type(getattr(self, name)) is not bool:
                raise ContractValidationError(f"invalid {name}")
        if self.observability > 1.0:
            raise ContractValidationError("observability exceeds one")
