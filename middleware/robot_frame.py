#!/usr/bin/env python3

"""Canonical pose-frame contract shared by all localization measurements."""

from __future__ import annotations

from typing import Dict


POSE_FRAME_ID = "R2B4_BOOT_ROBOT_MAP"
POSE_FRAME_OWNER = "EKF_POSE_ODOMETRY_SSOT"
POSE_FRAME_ORIGIN = "ROBOT_CENTER_AT_BOOT_OR_ATOMIC_POSE_RESET"
POSE_FRAME_X_AXIS = "INITIAL_ROBOT_FORWARD"
POSE_FRAME_Y_AXIS = "INITIAL_ROBOT_LEFT"
POSE_FRAME_YAW = "CCW_POSITIVE_LEFT"


def pose_frame_contract() -> Dict[str, str]:
    """Return a copy so telemetry callers cannot mutate the frame SSOT."""
    return {
        "frame_id": POSE_FRAME_ID,
        "owner": POSE_FRAME_OWNER,
        "origin": POSE_FRAME_ORIGIN,
        "x_axis": POSE_FRAME_X_AXIS,
        "y_axis": POSE_FRAME_Y_AXIS,
        "yaw": POSE_FRAME_YAW,
    }
