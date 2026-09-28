from __future__ import annotations

from pathlib import Path
from v3.config import ConfigResolver

ROOT = Path(__file__).resolve().parents[1]


def resolved_config(root: Path = ROOT):
    """Resolve the same four production config authorities used by the robot.

    No unit-config copy, constructor reflection or signature-following fixture exists here.
    """
    conf = root / "conf"
    return ConfigResolver(
        conf / "hardver.json",
        conf / "fizika.json",
        conf / "speed_map.json",
        conf / "vezerles.json",
    ).resolve()


def healthy_localization(**changes):
    """Explicit synthetic L3 evidence for tests that begin downstream of L3."""
    from dataclasses import replace
    from v3.contracts import LocalizationQuality, QualityState
    return replace(LocalizationQuality(
        local_translation=QualityState.GOOD, heading=QualityState.GOOD,
        global_position=QualityState.GOOD, local_sigma_m=.01, global_sigma_m=.01,
        yaw_sigma_rad=.01, encoder_age_ns=0, imu_age_ns=0, lidar_age_ns=0,
        global_fix_age_ns=0, relative_age_ns=0, local_pose_continuous=True,
        observability=1.0,
    ), **changes)
