#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Ember követése (Person Following) – kamera + LIDAR target observation.
A runtime főútban ez már csak célészlelési adatot publikál; a mozgást a
FOLLOW -> CRUISE -> local planner -> executor/safety lánc adja.
"""

import os
import math
import statistics
import threading
import time
from collections import deque
from concurrent.futures import TimeoutError as FutureTimeoutError
from typing import Any, Dict, Optional, Tuple

from driver.cam import camera_lifecycle_lock, safe_stop_close, _camera_rotation_deg, _rotate_image
from controller.target_obstacle_arbiter import TargetObstacleArbiter, TargetObstacleArbiterConfig

# State machine: FOLLOW state when adaptive motion is active
try:
    from state import RobotState
except ImportError:
    RobotState = None

# Konstansok: biztonság és hardver (nem konfig)
LIDAR_EMERGENCY_DIST_M = 0.5
LIDAR_FOLLOW_HOLD_DIST_M = 0.76
# Raspberry Pi Camera Module 3 Wide NoIR (imx708_wide_noir), vendor product brief:
# native 16:9 sensor, 102 deg horizontal FOV, 67 deg vertical FOV.
CAMERA_HFOV_DEG = 102.0
CAMERA_VFOV_DEG = 67.0
CAMERA_NATIVE_ASPECT = 16.0 / 9.0
CAMERA_CAPTURE_SIZE = (640, 360)
FOLLOW_SEARCH_MAX_ROTATIONS = 1
FOLLOW_SEARCH_ROTATION_DONE_DEG = 355.0
FOLLOW_SEARCH_TIMEOUT_S = 15.0
FOLLOW_SEARCH_FOUND_CONFIRM_FRAMES = 2
FOLLOW_SEARCH_STRONG_DETECTORS = {"mediapipe_pose", "onnx_yolov5_person"}
FOLLOW_SEARCH_TEMPLATE_RELOCK_MIN_CONFIDENCE = 0.52
FOLLOW_CANDIDATE_CONFIRM_HOLD_S = 1.0
FOLLOW_STRONG_CANDIDATE_HOLD_MIN_CONFIDENCE = 0.50
FOLLOW_REACQUIRE_CENTER_SCAN_ANGLE_DEG = 6.0
FOLLOW_REACQUIRE_CENTER_SCAN_TRIGGER_DEG = 4.0

# LIDAR zaj: 8 cm alatti pontok nem számítanak (téves vészleállítás elkerülésére)
LIDAR_MIN_VALID_M = 0.08
LIDAR_TARGET_MAX_AGE_S = 0.35
LIDAR_TARGET_CLUSTER_GAP_M = 0.22
LIDAR_TARGET_MAX_RANGE_M = 5.0
LIDAR_TARGET_EXPECTED_DISTANCE_GATE_MIN_M = 0.35
LIDAR_TARGET_EXPECTED_DISTANCE_GATE_MAX_M = 0.90
LIDAR_TARGET_EXPECTED_DISTANCE_GATE_RATIO = 0.35

CAMERA_DISTANCE_MIN_M = 0.65
CAMERA_DISTANCE_MAX_M = 3.2
CAMERA_DISTANCE_MARGINAL_BBOX_MAX_M = 1.65
CAMERA_DISTANCE_MIN_HEIGHT_RATIO = 0.10
CAMERA_DISTANCE_REFERENCE_HEIGHT_RATIO = 0.70
CAMERA_DISTANCE_LIDAR_BLEND_MAX_DELTA_M = 0.45
CAMERA_DISTANCE_LIDAR_BLEND_CAMERA_WEIGHT = 0.10
CAMERA_DISTANCE_MOTION_BLOB_LIDAR_GUARD_DELTA_M = 0.25
CAMERA_DISTANCE_MOTION_BLOB_LIDAR_GUARD_MAX_LIDAR_M = 1.80
CAMERA_DISTANCE_MOTION_BLOB_LOW_CONF_MIN_M = 0.95
CAMERA_DISTANCE_MOTION_BLOB_LOW_CONF_MAX_CONF = 0.38
CAMERA_CLOSE_BUBBLE_LIDAR_DESIRED_MAX_M = 0.70
CAMERA_CLOSE_BUBBLE_LIDAR_FRONT_MARGIN_M = 0.30
CAMERA_CLOSE_BUBBLE_LIDAR_MAX_BEARING_DEG = 18.0
CAMERA_CLOSE_BUBBLE_LIDAR_MIN_CAMERA_DELTA_M = 0.20
CAMERA_CLOSE_BUBBLE_LIDAR_MAX_CAMERA_DELTA_M = 0.45
CAMERA_ROOM_BUBBLE_LIDAR_FRONT_MARGIN_M = 0.18
CAMERA_ROOM_BUBBLE_LIDAR_MAX_BEARING_DEG = 14.0
CAMERA_ROOM_BUBBLE_LIDAR_MIN_CAMERA_DELTA_M = 0.20
CAMERA_ROOM_BUBBLE_LIDAR_MAX_CAMERA_DELTA_M = 1.10
CAMERA_ROOM_BUBBLE_LIDAR_MIN_BBOX_HEIGHT_RATIO = 0.58
CAMERA_FRONT_HOLD_ARBITRATION_MIN_CONFIDENCE = 0.20
CAMERA_FRONT_HOLD_ARBITRATION_MIN_DELTA_M = 0.35
CAMERA_FRONT_HOLD_ARBITRATION_MIN_TARGET_M = 1.05
CAMERA_DISTANCE_DETECTOR_REFERENCE = {
    "mediapipe_pose": 0.72,
    "onnx_yolov5_person": 0.70,
    "opencv_hog": 0.68,
    "opencv_template_lock": 0.68,
    "opencv_motion_blob": 0.62,
}
CAMERA_DISTANCE_DETECTOR_WEIGHT = {
    "mediapipe_pose": 0.92,
    "onnx_yolov5_person": 0.86,
    "opencv_hog": 0.72,
    "opencv_template_lock": 0.62,
    "opencv_motion_blob": 0.42,
}
CAMERA_HUMAN_BBOX_MIN_WIDTH_RATIO = 0.055
CAMERA_HUMAN_BBOX_MAX_WIDTH_RATIO = 0.72
CAMERA_HUMAN_BBOX_MIN_HEIGHT_RATIO = 0.24
CAMERA_HUMAN_BBOX_MAX_HEIGHT_RATIO = 1.01
CAMERA_HUMAN_BBOX_MIN_ASPECT = 1.05
CAMERA_HUMAN_BBOX_MAX_ASPECT = 5.30
CAMERA_MEDIAPIPE_UPPER_BODY_MIN_WIDTH_RATIO = 0.16
CAMERA_MEDIAPIPE_UPPER_BODY_MAX_WIDTH_RATIO = 0.82
CAMERA_MEDIAPIPE_UPPER_BODY_MIN_HEIGHT_RATIO = 0.055
CAMERA_MEDIAPIPE_UPPER_BODY_MAX_HEIGHT_RATIO = 0.34
CAMERA_MEDIAPIPE_UPPER_BODY_MAX_ASPECT = 1.05
CAMERA_MEDIAPIPE_UPPER_BODY_MIN_AREA_RATIO = 0.012
CAMERA_HOG_FALLBACK_ENABLED = str(os.environ.get("R2B4_CAMERA_HOG_FALLBACK", "0")).strip().lower() in {"1", "true", "yes", "on"}
CAMERA_HOG_MIN_WEIGHT = 0.42
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
PIC_DIR = os.path.join(PROJECT_ROOT, "Pic")
CAMERA_MOTION_BLOB_MAX_WIDTH_RATIO = 0.65
CAMERA_MOTION_BLOB_MIN_ASPECT = 1.05
CAMERA_MOTION_BLOB_MIN_FILL_RATIO = 0.16
CAMERA_MOTION_BLOB_MAX_FILL_RATIO = 1.01
CAMERA_ONNX_MODEL_PATH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "runtime", "models", "yolov5n.onnx")
)
CAMERA_ONNX_INPUT_SIZE = 640
CAMERA_MOTION_BLOB_RECENT_HUMAN_S = 2.50
CAMERA_ONNX_MIN_CONFIDENCE = 0.30
CAMERA_ONNX_WEAK_MIN_SCORE = 0.080
CAMERA_ONNX_WEAK_MIN_OBJECTNESS = 0.100
CAMERA_ONNX_WEAK_MIN_PERSON_CLASS = 0.55
CAMERA_ONNX_WEAK_MAX_CENTER_OFFSET_RATIO = 0.34
CAMERA_ONNX_CLOSE_PERSON_MIN_SCORE = 0.30
CAMERA_ONNX_CLOSE_PERSON_MIN_OBJECTNESS = 0.35
CAMERA_ONNX_CLOSE_PERSON_MIN_PERSON_CLASS = 0.60
CAMERA_ONNX_CLOSE_PERSON_MIN_HEIGHT_RATIO = 0.30
CAMERA_ONNX_CLOSE_PERSON_MIN_ASPECT = 0.22
CAMERA_ONNX_CLOSE_PERSON_MIN_AREA_RATIO = 0.10
CAMERA_ONNX_CLOSE_PERSON_MAX_CENTER_OFFSET_RATIO = 0.42
CAMERA_ONNX_RELOCK_MIN_SCORE = 0.060
CAMERA_ONNX_RELOCK_MIN_OBJECTNESS = 0.080
CAMERA_ONNX_RELOCK_MIN_PERSON_CLASS = 0.55
CAMERA_ONNX_RELOCK_MAX_CENTER_OFFSET_RATIO = 0.40
CAMERA_ONNX_MIN_INTERVAL_S = 0.75
CAMERA_ONNX_RETRY_INTERVAL_S = 30.0
CAMERA_CAPTURE_REQUEST_MIN_INTERVAL_S = 0.08
CAMERA_CAPTURE_WAIT_TIMEOUT_S = 0.045
CAMERA_DETECT_PROCESS_MIN_INTERVAL_S = 0.14
CAMERA_ASYNC_RESULT_MAX_AGE_S = 1.50
CAMERA_STRONG_REVALIDATE_INTERVAL_S = 1.50
CAMERA_RECENT_HOLD_FRESH_MAX_AGE_S = 0.45
CAMERA_TEMPLATE_LOCK_MIN_SCORE = 0.60
CAMERA_TEMPLATE_LOCK_UPDATE_SCORE = 0.72
CAMERA_TEMPLATE_LOCK_MIN_STD = 6.0
CAMERA_TEMPLATE_LOCK_RECENT_HUMAN_S = 18.0
CAMERA_TEMPLATE_LOCK_SCALE_FACTORS = (0.80, 0.90, 1.0, 1.12, 1.25)
CAMERA_LOCK_CONFIRM_FRAMES = 3
CAMERA_RELOCK_CONFIRM_FRAMES = 2
CAMERA_LOCK_CONFIRM_WINDOW_S = 2.80
CAMERA_LOCK_MIN_TIMESPAN_S = 0.26
CAMERA_RELOCK_MIN_TIMESPAN_S = 0.16
CAMERA_RELOCK_MAX_LAST_LOCK_AGE_S = 6.0
CAMERA_RELOCK_MIN_CONFIDENCE = 0.78
CAMERA_ONNX_STARTUP_SINGLE_FRAME_LOCK_MIN_CONFIDENCE = 0.78
CAMERA_ONNX_STARTUP_TWO_FRAME_LOCK_MIN_CONFIDENCE = 0.50
CAMERA_ONNX_STARTUP_SINGLE_FRAME_MAX_HEIGHT_RATIO = 0.58
CAMERA_ONNX_RECENT_LOCK_RELOCK_MAX_LAST_LOCK_AGE_S = 1.50
CAMERA_ONNX_RECENT_LOCK_RELOCK_MIN_CONFIDENCE = 0.55
CAMERA_ONNX_RECENT_HUMAN_SINGLE_FRAME_RELOCK_MIN_CONFIDENCE = 0.70
CAMERA_TEMPLATE_RELOCK_MAX_LAST_LOCK_AGE_S = 8.0
CAMERA_TEMPLATE_RELOCK_MIN_CONFIDENCE = 0.58
CAMERA_TEMPLATE_RECENT_SINGLE_FRAME_RELOCK_MAX_LAST_LOCK_AGE_S = 4.0
CAMERA_TEMPLATE_RECENT_SINGLE_FRAME_RELOCK_MIN_CONFIDENCE = 0.60
CAMERA_LOCK_MAX_CENTER_SPAN_RATIO = 0.30
CAMERA_LOCK_MAX_HEIGHT_SPAN_RATIO = 0.22
CAMERA_LOCK_MAX_WIDTH_SPAN_RATIO = 0.24
CAMERA_ONNX_SEEDED_FALLBACK_WINDOW_S = 2.80
CAMERA_ONNX_SEEDED_MOTION_MIN_CONFIDENCE = 0.30
CAMERA_ONNX_SEEDED_TEMPLATE_MIN_CONFIDENCE = 0.50
CAMERA_ONNX_SEEDED_FALLBACK_MAX_HEIGHT_SPAN_RATIO = 0.80
CAMERA_ONNX_SEEDED_FALLBACK_MAX_WIDTH_SPAN_RATIO = 0.45
CAMERA_LOCK_MAX_CENTER_JUMP_RATIO = 0.40
CAMERA_LOCK_STRONG_JUMP_MAX_CENTER_RATIO = 0.70
CAMERA_LOCK_STRONG_JUMP_MIN_CONFIDENCE = 0.78
CAMERA_LOCK_STRONG_JUMP_MAX_WIDTH_DELTA_RATIO = 0.12
CAMERA_LOCK_STRONG_JUMP_MAX_HEIGHT_DELTA_RATIO = 0.12
CAMERA_LOCK_MIN_CONFIDENCE = {
    "mediapipe_pose": 0.78,
    "onnx_yolov5_person": 0.32,
    "opencv_hog": 0.52,
    "opencv_template_lock": 0.58,
    "opencv_motion_blob": 0.68,
}
CAMERA_LOCK_STRONG_DETECTORS = {"onnx_yolov5_person", "mediapipe_pose"}
FOLLOW_GUI_PREVIEW_INTERVAL_S = 0.25
FOLLOW_GUI_PREVIEW_SIZE = (320, 180)
FOLLOW_GUI_PREVIEW_JPEG_QUALITY = 45
FOLLOW_START_STREAM_SEED_MAX_AGE_S = 2.5
FOLLOW_START_STREAM_SEED_WAIT_S = 1.2


def _clear_follow_search_attrs(ctrl, *, state: str = "idle") -> None:
    setattr(ctrl, "_adaptive_target_search_active", False)
    setattr(ctrl, "_follow_search_active", False)
    setattr(ctrl, "_follow_search_total_rotated_deg", 0.0)
    setattr(ctrl, "_follow_search_rotations_completed", 0)
    setattr(ctrl, "_follow_search_last_theta_deg", None)
    setattr(ctrl, "_follow_search_started_ts", None)
    setattr(ctrl, "_follow_search_found_confirm_count", 0)
    setattr(ctrl, "_follow_candidate_confirm_hold_until_ts", 0.0)
    setattr(ctrl, "follow_search_status", {"active": False, "state": str(state)})


def _follow_search_target_confirmed(ctrl, camera_status: Optional[Dict[str, Any]]) -> bool:
    if not bool(getattr(ctrl, "_follow_search_active", False)):
        return True
    status = dict(camera_status or {})
    visible = bool(status.get("target_visible", False)) and bool(status.get("target_usable", False)) and not bool(status.get("stale", False))
    detector = str(status.get("detector") or "")
    shape_ok = bool(status.get("bbox_human_shape_ok", True))
    if visible and detector in FOLLOW_SEARCH_STRONG_DETECTORS and shape_ok:
        setattr(ctrl, "_follow_search_found_confirm_count", int(FOLLOW_SEARCH_FOUND_CONFIRM_FRAMES))
        return True
    try:
        detector_confidence = float(status.get("detector_confidence") or 0.0)
    except (TypeError, ValueError):
        detector_confidence = 0.0
    if not math.isfinite(float(detector_confidence)):
        detector_confidence = 0.0
    if (
        visible
        and detector == "opencv_template_lock"
        and shape_ok
        and float(detector_confidence) >= FOLLOW_SEARCH_TEMPLATE_RELOCK_MIN_CONFIDENCE
    ):
        setattr(ctrl, "_follow_search_found_confirm_count", int(FOLLOW_SEARCH_FOUND_CONFIRM_FRAMES))
        return True
    if visible and detector not in {"", "none"} and shape_ok:
        count = int(getattr(ctrl, "_follow_search_found_confirm_count", 0) or 0) + 1
    else:
        count = 0
    setattr(ctrl, "_follow_search_found_confirm_count", int(count))
    return bool(count >= FOLLOW_SEARCH_FOUND_CONFIRM_FRAMES)


def _camera_status_fresh_usable(camera_status: Optional[Dict[str, Any]]) -> bool:
    status = dict(camera_status or {})
    return bool(
        bool(status.get("target_visible", False))
        and bool(status.get("target_usable", False))
        and not bool(status.get("stale", False))
        and str(status.get("detector") or "") not in {"", "none", "unknown"}
    )


def _follow_search_elapsed_s(ctrl) -> float:
    started = getattr(ctrl, "_follow_search_started_ts", None)
    try:
        if started is None:
            return 0.0
        elapsed = time.time() - float(started)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(float(elapsed)):
        return 0.0
    return max(0.0, float(elapsed))


def _mark_follow_target_search_found(ctrl) -> None:
    if not bool(getattr(ctrl, "_follow_search_active", False)):
        _clear_follow_search_attrs(ctrl, state="idle")
        return
    rotations = int(getattr(ctrl, "_follow_search_rotations_completed", 0) or 0)
    rotated = float(getattr(ctrl, "_follow_search_total_rotated_deg", 0.0) or 0.0)
    setattr(ctrl, "_adaptive_target_search_active", False)
    setattr(ctrl, "_follow_search_active", False)
    setattr(
        ctrl,
        "follow_search_status",
        {
            "active": False,
            "state": "found",
            "rotations_completed": int(rotations),
            "total_rotated_deg": float(rotated),
            "max_rotations": int(FOLLOW_SEARCH_MAX_ROTATIONS),
        },
    )
    try:
        from controller.status import append_camera_log
        append_camera_log(ctrl, "follow_target_search_found", rotations_completed=int(rotations), rotated_deg=float(rotated))
    except Exception:
        pass
    if hasattr(ctrl, "logger") and hasattr(ctrl.logger, "info"):
        ctrl.logger.info("[FOLLOW] Keresés célpontot talált – vissza követésbe.")


def _ctrl_yaw_deg(ctrl) -> Optional[float]:
    try:
        ekf = getattr(ctrl, "ekf", None)
        state = ekf.get_state() if ekf is not None and hasattr(ekf, "get_state") else {}
        if not isinstance(state, dict):
            return None
        if state.get("theta_deg") is not None:
            yaw = float(state.get("theta_deg"))
        elif state.get("theta") is not None:
            yaw = math.degrees(float(state.get("theta")))
        else:
            return None
        if not math.isfinite(yaw):
            return None
        return yaw
    except Exception:
        return None


def _start_follow_target_search(ctrl, *, reason: str) -> None:
    if bool(getattr(ctrl, "_follow_search_active", False)):
        return
    yaw_deg = _ctrl_yaw_deg(ctrl)
    setattr(ctrl, "_follow_search_active", True)
    setattr(ctrl, "_adaptive_target_search_active", True)
    setattr(ctrl, "_follow_search_total_rotated_deg", 0.0)
    setattr(ctrl, "_follow_search_rotations_completed", 0)
    setattr(ctrl, "_follow_search_last_theta_deg", yaw_deg)
    setattr(ctrl, "_follow_search_started_ts", time.time())
    setattr(
        ctrl,
        "follow_search_status",
        {
            "active": True,
            "state": "searching",
            "reason": str(reason or "target_lost"),
            "rotations_completed": 0,
            "total_rotated_deg": 0.0,
            "max_rotations": int(FOLLOW_SEARCH_MAX_ROTATIONS),
        },
    )
    try:
        from controller.status import append_camera_log
        append_camera_log(ctrl, "follow_target_search_start", reason=str(reason or "target_lost"))
    except Exception:
        pass
    if hasattr(ctrl, "logger") and hasattr(ctrl.logger, "info"):
        ctrl.logger.info("[FOLLOW] Cél elveszett – kereső ARC szkennelés indul.")


def _update_follow_target_search_rotation(ctrl) -> int:
    if not bool(getattr(ctrl, "_follow_search_active", False)):
        return 0
    yaw_deg = _ctrl_yaw_deg(ctrl)
    last_yaw = getattr(ctrl, "_follow_search_last_theta_deg", None)
    if yaw_deg is not None and last_yaw is not None:
        delta = (float(yaw_deg) - float(last_yaw) + 180.0) % 360.0 - 180.0
        total = float(getattr(ctrl, "_follow_search_total_rotated_deg", 0.0) or 0.0) + abs(float(delta))
        rotations = int(getattr(ctrl, "_follow_search_rotations_completed", 0) or 0)
        if total >= FOLLOW_SEARCH_ROTATION_DONE_DEG:
            rotations += 1
            total = 0.0
            try:
                from controller.status import append_camera_log
                append_camera_log(ctrl, "follow_target_search_360_done", rotations_completed=int(rotations))
            except Exception:
                pass
        setattr(ctrl, "_follow_search_total_rotated_deg", float(total))
        setattr(ctrl, "_follow_search_rotations_completed", int(rotations))
    elif yaw_deg is not None and last_yaw is None:
        setattr(ctrl, "_follow_search_last_theta_deg", float(yaw_deg))
    if yaw_deg is not None:
        setattr(ctrl, "_follow_search_last_theta_deg", float(yaw_deg))
    return int(getattr(ctrl, "_follow_search_rotations_completed", 0) or 0)


def _publish_follow_target_search(ctrl, lidar_snapshot, camera_status: Optional[Dict[str, Any]], *, reason: str) -> Tuple[float, float]:
    _start_follow_target_search(ctrl, reason=reason)
    rotations = _update_follow_target_search_rotation(ctrl)
    search_elapsed_s = _follow_search_elapsed_s(ctrl)
    if rotations >= FOLLOW_SEARCH_MAX_ROTATIONS:
        try:
            from controller.status import append_camera_log
            append_camera_log(
                ctrl,
                "follow_target_search_failed",
                rotations_completed=int(rotations),
                elapsed_s=round(float(search_elapsed_s), 3),
            )
        except Exception:
            pass
        if hasattr(ctrl, "logger"):
            ctrl.logger.warn("[FOLLOW] Ember nem található a kereső fordulat után – követés leáll.")
        if hasattr(ctrl, "brain") and getattr(ctrl.brain, "tts", None):
            try:
                ctrl.brain.tts.say("NINCS EMBER")
            except Exception:
                pass
        stop_following(ctrl)
        setattr(
            ctrl,
            "follow_search_status",
            {
                "active": False,
                "state": "failed",
                "reason": "target_not_found",
                "rotations_completed": int(rotations),
                "max_rotations": int(FOLLOW_SEARCH_MAX_ROTATIONS),
                "elapsed_s": round(float(search_elapsed_s), 3),
                "timeout_s": float(FOLLOW_SEARCH_TIMEOUT_S),
            },
        )
        return (0.0, 0.0)
    if float(search_elapsed_s) >= float(FOLLOW_SEARCH_TIMEOUT_S) and not _camera_status_fresh_usable(camera_status):
        try:
            from controller.status import append_camera_log
            append_camera_log(
                ctrl,
                "follow_target_search_timeout",
                elapsed_s=round(float(search_elapsed_s), 3),
                timeout_s=float(FOLLOW_SEARCH_TIMEOUT_S),
                rotations_completed=int(rotations),
            )
        except Exception:
            pass
        if hasattr(ctrl, "logger"):
            ctrl.logger.warn("[FOLLOW] Kereses timeout friss kamera cel nelkul - kovetes leall.")
        if hasattr(ctrl, "brain") and getattr(ctrl.brain, "tts", None):
            try:
                ctrl.brain.tts.say("NINCS EMBER")
            except Exception:
                pass
        stop_following(ctrl)
        setattr(
            ctrl,
            "follow_search_status",
            {
                "active": False,
                "state": "failed",
                "reason": "target_search_timeout",
                "rotations_completed": int(rotations),
                "max_rotations": int(FOLLOW_SEARCH_MAX_ROTATIONS),
                "elapsed_s": round(float(search_elapsed_s), 3),
                "timeout_s": float(FOLLOW_SEARCH_TIMEOUT_S),
            },
        )
        return (0.0, 0.0)

    now_wall = time.time()
    lidar_status = {
        "state": "not_evaluated_target_search",
        "source": "lidar_not_evaluated",
        "usable_distance": False,
        "stale": False,
        "missing": lidar_snapshot is None,
        "age_s": _lidar_snapshot_age_s(lidar_snapshot),
        "confidence": 0.0,
        "distance_m": None,
        "point_count": 0,
        "cluster_points": 0,
    }
    status = dict(camera_status or {})
    status.setdefault("source", "camera")
    search_side = _camera_search_side_from_zone(
        status.get("last_search_side") or status.get("target_zone"),
        str(getattr(ctrl, "_adaptive_target_search_side", "") or "left"),
    )
    status["raw_state"] = str(status.get("state") or "")
    status["state"] = "target_search_scan"
    status["target_visible"] = False
    status["target_usable"] = False
    status["stale"] = True
    status["gate"] = "target_lost_search"
    status["search_active"] = True
    status["search_side"] = str(search_side)
    status["search_rotations_completed"] = int(rotations)
    status["search_total_rotated_deg"] = round(float(getattr(ctrl, "_follow_search_total_rotated_deg", 0.0) or 0.0), 2)
    status["search_max_rotations"] = int(FOLLOW_SEARCH_MAX_ROTATIONS)
    status["search_elapsed_s"] = round(float(search_elapsed_s), 3)
    status["search_timeout_s"] = float(FOLLOW_SEARCH_TIMEOUT_S)
    setattr(ctrl, "_adaptive_target_search_active", True)
    setattr(ctrl, "_adaptive_target_search_side", str(search_side))
    setattr(ctrl, "_adaptive_target_dist_m", None)
    setattr(ctrl, "_adaptive_target_angle_deg", None)
    setattr(ctrl, "_adaptive_target_confidence", 1.0)
    setattr(ctrl, "_adaptive_target_vx_mps", 0.0)
    setattr(ctrl, "_adaptive_target_vy_mps", 0.0)
    setattr(ctrl, "_adaptive_target_last_seen_ts", now_wall)
    setattr(ctrl, "_adaptive_target_desired_distance_m", 0.0)
    setattr(ctrl, "_adaptive_target_lidar_source", "target_search")
    setattr(ctrl, "_adaptive_target_lidar_confidence", 0.0)
    setattr(ctrl, "_adaptive_target_lidar_distance_m", None)
    setattr(ctrl, "_adaptive_target_lidar_points", 0)
    setattr(ctrl, "_adaptive_target_lidar_cluster_points", 0)
    setattr(ctrl, "_adaptive_target_lidar_age_s", _lidar_snapshot_age_s(lidar_snapshot))
    setattr(ctrl, "_adaptive_target_lidar_status", dict(lidar_status))
    setattr(ctrl, "_adaptive_target_camera_status", dict(status))
    setattr(ctrl, "_adaptive_follow_state", "target_search_scan")
    setattr(
        ctrl,
        "follow_search_status",
        {
            "active": True,
            "state": "searching",
            "reason": str(reason or "target_lost"),
            "search_side": str(search_side),
            "rotations_completed": int(rotations),
            "total_rotated_deg": float(getattr(ctrl, "_follow_search_total_rotated_deg", 0.0) or 0.0),
            "max_rotations": int(FOLLOW_SEARCH_MAX_ROTATIONS),
            "elapsed_s": round(float(search_elapsed_s), 3),
            "timeout_s": float(FOLLOW_SEARCH_TIMEOUT_S),
        },
    )
    try:
        import robot_state
        robot_state.clear_tracked_target()
    except Exception:
        pass
    return (0.0, 0.0)


def _publish_follow_uncertain_hold(ctrl, lidar_snapshot, camera_status: Optional[Dict[str, Any]], *, reason: str) -> Tuple[float, float]:
    now_wall = time.time()
    params = _follower_params(ctrl)
    desired_distance_m = float(params["target_distance_m"])
    status = dict(camera_status or {})
    status.setdefault("source", "camera")
    status["raw_state"] = str(status.get("state") or "")
    status["state"] = "candidate_hold"
    status["target_usable"] = False
    status["stale"] = False
    status["gate"] = "candidate_uncertain_hold"
    status["search_active"] = False
    status["hold_reason"] = str(reason or "candidate_unconfirmed")
    confirm_hold = _strong_camera_candidate_should_hold_for_confirmation(camera_status)
    if bool(confirm_hold):
        hold_until = float(now_wall) + float(FOLLOW_CANDIDATE_CONFIRM_HOLD_S)
        setattr(ctrl, "_follow_candidate_confirm_hold_until_ts", float(hold_until))
        status["candidate_confirm_hold_active"] = True
        status["candidate_confirm_hold_s"] = float(FOLLOW_CANDIDATE_CONFIRM_HOLD_S)
    elif str(reason or "") != "candidate_confirm_wait":
        setattr(ctrl, "_follow_candidate_confirm_hold_until_ts", 0.0)
    lidar_status = {
        "state": "not_evaluated_candidate_hold",
        "source": "lidar_not_evaluated",
        "usable_distance": False,
        "stale": False,
        "missing": lidar_snapshot is None,
        "age_s": _lidar_snapshot_age_s(lidar_snapshot),
        "confidence": 0.0,
        "distance_m": None,
        "point_count": 0,
        "cluster_points": 0,
    }
    setattr(ctrl, "_adaptive_target_search_active", False)
    setattr(ctrl, "_adaptive_target_dist_m", desired_distance_m)
    setattr(ctrl, "_adaptive_target_angle_deg", 0.0)
    setattr(ctrl, "_adaptive_target_confidence", 0.05)
    setattr(ctrl, "_adaptive_target_vx_mps", 0.0)
    setattr(ctrl, "_adaptive_target_vy_mps", 0.0)
    setattr(ctrl, "_adaptive_target_last_seen_ts", now_wall)
    setattr(ctrl, "_adaptive_target_desired_distance_m", desired_distance_m)
    setattr(ctrl, "_adaptive_target_lidar_source", "candidate_hold")
    setattr(ctrl, "_adaptive_target_lidar_confidence", 0.0)
    setattr(ctrl, "_adaptive_target_lidar_distance_m", None)
    setattr(ctrl, "_adaptive_target_lidar_points", 0)
    setattr(ctrl, "_adaptive_target_lidar_cluster_points", 0)
    setattr(ctrl, "_adaptive_target_lidar_age_s", _lidar_snapshot_age_s(lidar_snapshot))
    setattr(ctrl, "_adaptive_target_lidar_status", dict(lidar_status))
    setattr(ctrl, "_adaptive_target_camera_status", dict(status))
    setattr(ctrl, "_adaptive_follow_state", "target_reacquire_hold")
    setattr(ctrl, "_adaptive_zero_track_hold_reason", str(reason or "candidate_unconfirmed"))
    try:
        import robot_state
        robot_state.clear_tracked_target()
    except Exception:
        pass
    return (0.0, 0.0)


def _weak_camera_candidate_should_continue_search(camera_status: Optional[Dict[str, Any]]) -> bool:
    status = dict(camera_status or {})
    state = str(status.get("state") or status.get("raw_state") or "")
    if state not in {"candidate_unconfirmed", "candidate_hold"}:
        return False
    if bool(status.get("lock_confirmed", False)) or bool(status.get("target_usable", False)):
        return False
    detector = str(status.get("detector") or "")
    if detector == "opencv_motion_blob":
        return True
    if detector == "opencv_template_lock":
        confidence = _clamp_float(float(status.get("detector_confidence") or 0.0), 0.0, 1.0)
        return bool(confidence < CAMERA_TEMPLATE_RELOCK_MIN_CONFIDENCE)
    return False


def _strong_camera_candidate_should_hold_for_confirmation(camera_status: Optional[Dict[str, Any]]) -> bool:
    status = dict(camera_status or {})
    state = str(status.get("state") or status.get("raw_state") or "")
    if state not in {"candidate_unconfirmed", "candidate_hold"}:
        return False
    if bool(status.get("lock_confirmed", False)) or bool(status.get("target_usable", False)):
        return False
    detector = str(status.get("detector") or "")
    if detector not in FOLLOW_SEARCH_STRONG_DETECTORS:
        return False
    if not bool(status.get("target_visible", False)):
        return False
    if not bool(status.get("bbox_human_shape_ok", True)):
        return False
    try:
        confidence = float(status.get("detector_confidence") or 0.0)
    except (TypeError, ValueError):
        confidence = 0.0
    if not math.isfinite(float(confidence)):
        confidence = 0.0
    return bool(float(confidence) >= float(FOLLOW_STRONG_CANDIDATE_HOLD_MIN_CONFIDENCE))


def _candidate_confirm_hold_active(ctrl, camera_status: Optional[Dict[str, Any]], *, now_wall: Optional[float] = None) -> bool:
    try:
        hold_until = float(getattr(ctrl, "_follow_candidate_confirm_hold_until_ts", 0.0) or 0.0)
    except (TypeError, ValueError):
        hold_until = 0.0
    if hold_until <= 0.0:
        return False
    now = time.time() if now_wall is None else float(now_wall)
    if float(now) > float(hold_until):
        setattr(ctrl, "_follow_candidate_confirm_hold_until_ts", 0.0)
        return False
    status = dict(camera_status or {})
    if _camera_status_fresh_usable(status):
        setattr(ctrl, "_follow_candidate_confirm_hold_until_ts", 0.0)
        return False
    state = str(status.get("state") or status.get("raw_state") or "")
    return bool(state not in {"candidate_unconfirmed", "candidate_hold"})


def _previous_target_age_s(previous_target: Optional[Dict[str, Any]], *, now_wall: Optional[float] = None) -> Optional[float]:
    if previous_target is None:
        return None
    now = time.time() if now_wall is None else float(now_wall)
    last_seen_ts = _finite_positive_float((previous_target or {}).get("last_seen_ts"))
    if last_seen_ts is not None:
        return max(0.0, float(now) - float(last_seen_ts))
    return _finite_nonnegative_float((previous_target or {}).get("age_s"))


def _target_loss_status_for_previous_target(
    camera_status: Optional[Dict[str, Any]],
    previous_target: Optional[Dict[str, Any]],
    *,
    now_wall: Optional[float] = None,
) -> Dict[str, Any]:
    status = dict(camera_status or {})
    age_s = _previous_target_age_s(previous_target, now_wall=now_wall)
    if age_s is not None:
        status["age_s"] = float(age_s)
    status["stale"] = True
    status["target_usable"] = False
    return status


def _clear_adaptive_target_attrs(ctrl, *, camera_status: Optional[Dict[str, Any]] = None, lidar_status: Optional[Dict[str, Any]] = None) -> None:
    setattr(ctrl, "_adaptive_target_dist_m", None)
    setattr(ctrl, "_adaptive_target_angle_deg", None)
    setattr(ctrl, "_adaptive_target_confidence", None)
    setattr(ctrl, "_adaptive_target_vx_mps", None)
    setattr(ctrl, "_adaptive_target_vy_mps", None)
    setattr(ctrl, "_adaptive_target_last_seen_ts", None)
    setattr(ctrl, "_adaptive_target_desired_distance_m", None)
    setattr(ctrl, "_adaptive_target_lidar_source", "")
    setattr(ctrl, "_adaptive_target_lidar_confidence", None)
    setattr(ctrl, "_adaptive_target_lidar_distance_m", None)
    setattr(ctrl, "_adaptive_target_lidar_points", None)
    setattr(ctrl, "_adaptive_target_lidar_cluster_points", None)
    setattr(ctrl, "_adaptive_target_lidar_age_s", None)
    setattr(ctrl, "_adaptive_target_lidar_status", dict(lidar_status or {}))
    setattr(ctrl, "_adaptive_target_camera_status", dict(camera_status or {}))
    setattr(ctrl, "_adaptive_follow_state", "lost")
    setattr(ctrl, "_adaptive_target_search_active", False)
    setattr(ctrl, "_adaptive_target_search_side", str((camera_status or {}).get("last_search_side") or "left"))


def _publish_target_persistence_hold(
    ctrl,
    lidar_snapshot,
    *,
    decision,
    previous_target: Dict[str, Any],
    camera_status: Optional[Dict[str, Any]],
    params: Dict[str, Any],
) -> Tuple[float, float]:
    now_wall = time.time()
    tracked = dict(previous_target or {})
    tracked["dist_m"] = float(decision.target_distance_m)
    tracked["angle_deg"] = float(decision.target_angle_deg or 0.0)
    if (
        str(getattr(decision, "mode", "") or "") == "target_reacquire_hold"
        and abs(float(tracked["angle_deg"])) <= float(FOLLOW_REACQUIRE_CENTER_SCAN_TRIGGER_DEG)
    ):
        scan_sign = -1.0 if float(tracked["angle_deg"]) < -0.25 else 1.0
        tracked["angle_deg"] = float(scan_sign * FOLLOW_REACQUIRE_CENTER_SCAN_ANGLE_DEG)
    tracked["confidence"] = float(decision.target_confidence)
    tracked["vx"] = 0.0
    tracked["vy"] = 0.0
    tracked.setdefault("last_seen_ts", now_wall)

    status = dict(camera_status or {})
    status.setdefault("source", "camera")
    status["raw_state"] = str(status.get("state") or "")
    status.update(dict(decision.camera_updates or {}))
    if (
        str(getattr(decision, "mode", "") or "") == "target_reacquire_hold"
        and abs(float(decision.target_angle_deg or 0.0)) <= float(FOLLOW_REACQUIRE_CENTER_SCAN_TRIGGER_DEG)
    ):
        status["reacquire_center_scan_angle_deg"] = float(tracked["angle_deg"])
        status["reacquire_center_scan_trigger_deg"] = float(FOLLOW_REACQUIRE_CENTER_SCAN_TRIGGER_DEG)
    status.setdefault("age_s", max(0.0, now_wall - float(tracked.get("last_seen_ts") or now_wall)))
    lidar_status = dict(decision.lidar_status or {})
    lidar_status["missing"] = bool(lidar_snapshot is None)
    lidar_status["age_s"] = _lidar_snapshot_age_s(lidar_snapshot)

    try:
        import robot_state
        robot_state.set_tracked_target(
            tracked["dist_m"],
            tracked["angle_deg"],
            confidence=tracked["confidence"],
            vx=tracked.get("vx"),
            vy=tracked.get("vy"),
            last_seen_ts=tracked.get("last_seen_ts", now_wall),
        )
    except Exception:
        pass

    setattr(ctrl, "_adaptive_target_search_active", False)
    setattr(ctrl, "_adaptive_target_dist_m", tracked["dist_m"])
    setattr(ctrl, "_adaptive_target_angle_deg", tracked["angle_deg"])
    setattr(ctrl, "_adaptive_target_confidence", tracked.get("confidence"))
    setattr(ctrl, "_adaptive_target_vx_mps", tracked.get("vx"))
    setattr(ctrl, "_adaptive_target_vy_mps", tracked.get("vy"))
    setattr(ctrl, "_adaptive_target_last_seen_ts", tracked.get("last_seen_ts"))
    setattr(ctrl, "_adaptive_target_desired_distance_m", float(params["target_distance_m"]))
    setattr(ctrl, "_adaptive_target_lidar_source", str(lidar_status.get("source") or "tracker_hold_camera_stale"))
    setattr(ctrl, "_adaptive_target_lidar_confidence", float(lidar_status.get("confidence") or 0.0))
    setattr(ctrl, "_adaptive_target_lidar_distance_m", lidar_status.get("distance_m"))
    setattr(ctrl, "_adaptive_target_lidar_points", int(lidar_status.get("point_count") or 0))
    setattr(ctrl, "_adaptive_target_lidar_cluster_points", int(lidar_status.get("cluster_points") or 0))
    setattr(ctrl, "_adaptive_target_lidar_age_s", lidar_status.get("age_s"))
    setattr(ctrl, "_adaptive_target_lidar_status", dict(lidar_status))
    setattr(ctrl, "_adaptive_target_camera_status", dict(status))
    follow_state = "target_reacquire_hold" if str(getattr(decision, "mode", "") or "") == "target_reacquire_hold" else "target_persistence_hold"
    setattr(ctrl, "_adaptive_follow_state", str(follow_state))
    return (0.0, 0.0)


def _follower_params(ctrl) -> dict:
    """vezerles.follower konfigból (vagy ctrl.follower_cfg); hiányzó mezők default."""
    cfg = getattr(ctrl, "follower_cfg", None) or {}
    return {
        "target_distance_m": float(cfg.get("target_distance_m", 1.2)),
        "stop_distance_m": float(cfg.get("stop_distance_m", 0.8)),
        "kp_distance": float(cfg.get("kp_distance", 0.4)),
        "k_omega": float(cfg.get("k_omega", 0.015)),
        "max_v_target": float(cfg.get("max_v_target", 0.35)),
        "max_omega": float(cfg.get("max_omega", 0.8)),
        "center_tolerance_deg": float(cfg.get("center_tolerance_deg", 10.0)),
    }


def _get_lidar_min_dist_any(lidar_snapshot) -> float:
    """LIDAR legközelebbi pont bármely irányban (m). Zaj: LIDAR_MIN_VALID_M alatt ignoráljuk."""
    if lidar_snapshot is None:
        return 99.0
    raw = getattr(lidar_snapshot, "raw_scan", None)
    if not raw:
        d = lidar_snapshot.summary.get("min_dist", 99.0)
        return d if d >= LIDAR_MIN_VALID_M else 99.0
    min_m = 99.0
    for p in raw:
        d_m = p.get("dist", 99999) / 1000.0
        if LIDAR_MIN_VALID_M <= d_m < min_m:
            min_m = d_m
    return min_m


def _lidar_snapshot_age_s(lidar_snapshot) -> Optional[float]:
    if lidar_snapshot is None:
        return None
    try:
        ts = float(getattr(lidar_snapshot, "timestamp", 0.0) or 0.0)
    except Exception:
        return None
    if not math.isfinite(ts) or ts <= 0.0:
        return None
    return max(0.0, time.monotonic() - ts)


def _scan_point_angle_distance_m(point) -> Optional[Tuple[float, float]]:
    if not isinstance(point, dict):
        return None
    dist_raw = point.get("distance_m", point.get("dist", point.get("dist_mm", None)))
    try:
        dist_value = float(dist_raw)
    except Exception:
        return None
    if not math.isfinite(dist_value) or dist_value <= 0.0:
        return None
    dist_m = dist_value / 1000.0 if dist_value > 20.0 else dist_value
    if not math.isfinite(dist_m) or dist_m < LIDAR_MIN_VALID_M or dist_m > LIDAR_TARGET_MAX_RANGE_M:
        return None

    angle_rad_raw = point.get("angle_rad")
    if angle_rad_raw is not None:
        try:
            angle_deg = math.degrees(float(angle_rad_raw))
        except Exception:
            return None
    else:
        try:
            angle_deg = float(point.get("angle_deg", point.get("angle", 0.0)))
        except Exception:
            return None
    if not math.isfinite(angle_deg):
        return None
    return angle_deg % 360.0, float(dist_m)


def _angle_delta_abs_deg(a_deg: float, b_deg: float) -> float:
    return abs(((float(a_deg) - float(b_deg) + 180.0) % 360.0) - 180.0)


def _get_lidar_min_dist_front(lidar_snapshot, window_deg: float = 42.0) -> float:
    """Front-szektor legközelebbi pontja (m); a globális safety kezeli az oldalsó akadályokat."""
    if lidar_snapshot is None:
        return 99.0
    raw = getattr(lidar_snapshot, "raw_scan", None)
    if not raw:
        summary = getattr(lidar_snapshot, "summary", {}) or {}
        d = summary.get("min_dist_narrow", summary.get("min_dist", 99.0))
        try:
            d = float(d)
        except Exception:
            return 99.0
        return d if d >= LIDAR_MIN_VALID_M else 99.0
    distances: list[float] = []
    for point in raw:
        parsed = _scan_point_angle_distance_m(point)
        if parsed is None:
            continue
        angle_deg, dist_m = parsed
        if _angle_delta_abs_deg(angle_deg, 0.0) <= float(window_deg):
            distances.append(float(dist_m))
    if not distances:
        return 99.0
    distances = sorted(distances)
    clusters: list[list[float]] = []
    for dist_m in distances:
        if not clusters or abs(float(dist_m) - float(clusters[-1][-1])) > LIDAR_TARGET_CLUSTER_GAP_M:
            clusters.append([float(dist_m)])
        else:
            clusters[-1].append(float(dist_m))
    multi_point_clusters = [cluster for cluster in clusters if len(cluster) >= 2]
    if multi_point_clusters:
        return float(min(multi_point_clusters[0]))
    return float(distances[0])


def _cluster_lidar_target_points(
    candidates: list[Tuple[float, float]],
    target_angle_deg: float,
    window_deg: float,
    expected_distance_m: Optional[float] = None,
) -> Optional[dict]:
    if not candidates:
        return None

    by_dist = sorted(candidates, key=lambda item: item[1])
    clusters: list[list[Tuple[float, float]]] = []
    for item in by_dist:
        if not clusters or abs(float(item[1]) - float(clusters[-1][-1][1])) > LIDAR_TARGET_CLUSTER_GAP_M:
            clusters.append([item])
        else:
            clusters[-1].append(item)

    multi_point = [cluster for cluster in clusters if len(cluster) >= 2]
    eligible = multi_point if multi_point else clusters

    best_cluster: Optional[list[Tuple[float, float]]] = None
    best_score = -float("inf")
    expected_dist: Optional[float]
    try:
        expected_dist = float(expected_distance_m) if expected_distance_m is not None else None
    except Exception:
        expected_dist = None
    if expected_dist is not None and (not math.isfinite(expected_dist) or expected_dist <= 0.0):
        expected_dist = None

    expected_tolerance_m: Optional[float] = None
    if expected_dist is not None:
        expected_tolerance_m = max(
            LIDAR_TARGET_EXPECTED_DISTANCE_GATE_MIN_M,
            min(
                LIDAR_TARGET_EXPECTED_DISTANCE_GATE_MAX_M,
                float(expected_dist) * LIDAR_TARGET_EXPECTED_DISTANCE_GATE_RATIO,
            ),
        )

    for cluster in eligible:
        distances = [float(d) for _, d in cluster]
        angle_deltas = [_angle_delta_abs_deg(a, target_angle_deg) for a, _ in cluster]
        median_dist = float(statistics.median(distances))
        median_angle_delta = float(statistics.median(angle_deltas))
        if expected_dist is not None and expected_tolerance_m is not None:
            if abs(median_dist - float(expected_dist)) > float(expected_tolerance_m):
                continue
        count_score = min(1.0, float(len(cluster)) / 6.0)
        distance_score = 1.0 - min(1.0, median_dist / LIDAR_TARGET_MAX_RANGE_M)
        angle_score = 1.0 - min(1.0, median_angle_delta / max(1.0, float(window_deg)))
        if expected_dist is None:
            score = (0.25 * count_score) + (0.50 * distance_score) + (0.25 * angle_score)
        else:
            expected_tolerance_m = max(0.35, min(1.20, float(expected_dist) * 0.45))
            expected_score = 1.0 - min(1.0, abs(median_dist - float(expected_dist)) / expected_tolerance_m)
            score = (
                (0.20 * count_score)
                + (0.15 * distance_score)
                + (0.25 * angle_score)
                + (0.40 * expected_score)
            )
        if score > best_score:
            best_score = float(score)
            best_cluster = list(cluster)

    if not best_cluster:
        return None

    distances = sorted(float(d) for _, d in best_cluster)
    angle_deltas = [_angle_delta_abs_deg(a, target_angle_deg) for a, _ in best_cluster]
    spread_m = float(distances[-1] - distances[0]) if len(distances) > 1 else 0.0
    count_factor = min(1.0, float(len(best_cluster)) / 5.0)
    spread_factor = 1.0 - min(1.0, spread_m / 0.40)
    angle_factor = 1.0 - min(1.0, float(statistics.median(angle_deltas)) / max(1.0, float(window_deg)))
    confidence = max(0.10, min(1.0, (0.35 * count_factor) + (0.35 * spread_factor) + (0.30 * angle_factor)))
    return {
        "distance_m": float(statistics.median(distances)),
        "confidence": float(confidence),
        "point_count": int(len(candidates)),
        "cluster_points": int(len(best_cluster)),
        "spread_m": float(spread_m),
        "source": "lidar_angle_cluster",
    }


def _get_lidar_target_measurement_at_angle_deg(
    lidar_snapshot,
    angle_deg: float,
    window_deg: float = 12.0,
    expected_distance_m: Optional[float] = None,
) -> Optional[dict]:
    if lidar_snapshot is None:
        return None
    age_s = _lidar_snapshot_age_s(lidar_snapshot)
    if age_s is not None and age_s > LIDAR_TARGET_MAX_AGE_S:
        return {
            "distance_m": None,
            "confidence": 0.0,
            "point_count": 0,
            "cluster_points": 0,
            "spread_m": None,
            "age_s": float(age_s),
            "source": "lidar_stale",
        }
    raw = getattr(lidar_snapshot, "raw_scan", None)
    if not raw:
        return None

    target_angle = float(angle_deg) % 360.0
    candidates: list[Tuple[float, float]] = []
    for point in raw:
        parsed = _scan_point_angle_distance_m(point)
        if parsed is None:
            continue
        point_angle, dist_m = parsed
        if _angle_delta_abs_deg(point_angle, target_angle) <= float(window_deg):
            candidates.append((float(point_angle), float(dist_m)))

    measurement = _cluster_lidar_target_points(
        candidates,
        target_angle,
        window_deg,
        expected_distance_m=expected_distance_m,
    )
    if measurement is not None:
        measurement["age_s"] = age_s
    return measurement


def _get_lidar_dist_at_angle_deg(lidar_snapshot, angle_deg: float, window_deg: float = 12.0) -> Optional[float]:
    """Adott szög környezetében mért legközelebbi távolság (m). angle_deg: 0 = elöl, balra negatív."""
    measurement = _get_lidar_target_measurement_at_angle_deg(lidar_snapshot, angle_deg, window_deg)
    if not measurement:
        return None
    dist = measurement.get("distance_m")
    return None if dist is None else float(dist)


def _target_lidar_status(lidar_snapshot, measurement: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    age_s = (measurement or {}).get("age_s")
    if age_s is None:
        age_s = _lidar_snapshot_age_s(lidar_snapshot)
    source = str((measurement or {}).get("source") or "")
    dist = (measurement or {}).get("distance_m")
    confidence = float((measurement or {}).get("confidence") or 0.0)
    point_count = int((measurement or {}).get("point_count") or 0)
    cluster_points = int((measurement or {}).get("cluster_points") or 0)
    stale = bool(source == "lidar_stale" or (age_s is not None and float(age_s) > LIDAR_TARGET_MAX_AGE_S))
    missing = lidar_snapshot is None

    if missing:
        state = "missing"
        source = "lidar_missing"
    elif stale:
        state = "stale"
        source = source or "lidar_stale"
    elif measurement is None:
        state = "no_target_points"
        source = "lidar_no_target_points"
    elif dist is None:
        state = "no_distance"
        source = source or "lidar_no_distance"
    elif confidence < 0.25 or cluster_points < 2:
        state = "weak"
        source = source or "lidar_angle_cluster"
    else:
        state = "ok"
        source = source or "lidar_angle_cluster"

    return {
        "state": str(state),
        "source": str(source),
        "usable_distance": bool(dist is not None and not stale and not missing),
        "stale": bool(stale),
        "missing": bool(missing),
        "age_s": None if age_s is None else float(age_s),
        "confidence": float(confidence),
        "distance_m": None if dist is None else float(dist),
        "point_count": int(point_count),
        "cluster_points": int(cluster_points),
    }


def _adaptive_follow_state(
    *,
    camera_status: Dict[str, Any],
    lidar_status: Dict[str, Any],
    dist_m: Optional[float],
    angle_deg: float,
    params: Dict[str, Any],
) -> str:
    if not bool((camera_status or {}).get("target_usable", False)):
        return "lost"
    if bool((camera_status or {}).get("stale", False)) and not bool((lidar_status or {}).get("usable_distance", False)):
        return "lost"
    if dist_m is not None and float(dist_m) <= float(params.get("stop_distance_m", 0.8)):
        return "hold"
    if bool((camera_status or {}).get("stale", False)):
        return "reacquire"
    if abs(float(angle_deg)) >= max(18.0, 2.0 * float(params.get("center_tolerance_deg", 10.0))):
        return "reacquire"
    if dist_m is not None and float(dist_m) > float(params.get("target_distance_m", 1.2)) + 0.20:
        return "approach"
    if not bool((lidar_status or {}).get("usable_distance", False)):
        return "track_low_lidar"
    return "track"


def _camera_bearing_fov_deg(
    image_width: Any = None,
    image_height: Any = None,
    rotation_deg: Any = 0,
) -> float:
    """Effective horizontal bearing FOV after software camera rotation."""
    try:
        rot = int(float(rotation_deg or 0.0)) % 360
    except Exception:
        rot = 0
    width = _finite_positive_float(image_width)
    height = _finite_positive_float(image_height)
    if rot in {90, 270}:
        return float(CAMERA_VFOV_DEG)
    if width is not None and height is not None:
        aspect = float(width) / max(1.0, float(height))
        if aspect > 0.0 and aspect < float(CAMERA_NATIVE_ASPECT):
            hfov_rad = math.radians(float(CAMERA_HFOV_DEG))
            crop_ratio = max(0.05, min(1.0, aspect / float(CAMERA_NATIVE_ASPECT)))
            return float(math.degrees(2.0 * math.atan(math.tan(hfov_rad * 0.5) * crop_ratio)))
    return float(CAMERA_HFOV_DEG)


def _bbox_center_to_angle_status(
    center_x: float,
    image_width: float,
    image_height: Any = None,
    rotation_deg: Any = 0,
) -> Tuple[float, Dict[str, Any]]:
    """Képközépponttól vízszintes eltérés → kamera képi céloldal (fok)."""
    if image_width <= 0:
        return 0.0, {
            "target_angle_deg": 0.0,
            "target_center_x_px": float(center_x),
            "target_center_offset_ratio": 0.0,
            "bearing_fov_deg": float(CAMERA_HFOV_DEG),
            "bearing_fov_source": "invalid_image_width",
        }
    fov_deg = _camera_bearing_fov_deg(image_width, image_height, rotation_deg)
    half_w = image_width / 2.0
    offset_ratio = (center_x - half_w) / half_w
    angle_deg = float(offset_ratio) * (float(fov_deg) / 2.0)
    try:
        rot = int(float(rotation_deg or 0.0)) % 360
    except Exception:
        rot = 0
    if rot in {90, 270}:
        fov_source = "imx708_wide_vertical_fov"
    else:
        try:
            aspect = float(image_width) / max(1.0, float(image_height))
        except Exception:
            aspect = float(CAMERA_NATIVE_ASPECT)
        fov_source = (
            "imx708_wide_aspect_cropped_horizontal_fov"
            if aspect < float(CAMERA_NATIVE_ASPECT) - 0.01
            else "imx708_wide_native_horizontal_fov"
        )
    return angle_deg, {
        "target_angle_deg": float(angle_deg),
        "target_center_x_px": float(center_x),
        "target_center_offset_ratio": float(offset_ratio),
        "bearing_fov_deg": float(fov_deg),
        "bearing_fov_source": fov_source,
    }


def _bbox_center_to_angle_deg(
    center_x: float,
    image_width: float,
    image_height: Any = None,
    rotation_deg: Any = 0,
) -> float:
    angle_deg, _status = _bbox_center_to_angle_status(
        center_x,
        image_width,
        image_height=image_height,
        rotation_deg=rotation_deg,
    )
    return float(angle_deg)


def _clamp_float(value: float, lo: float, hi: float) -> float:
    return max(float(lo), min(float(hi), float(value)))


def _camera_target_zone_from_center(center_x: Any, image_width: Any) -> str:
    cx = _finite_nonnegative_float(center_x)
    width = _finite_positive_float(image_width)
    if cx is None or width is None:
        return "unknown"
    ratio = float(cx) / max(1.0, float(width))
    if ratio < (1.0 / 3.0):
        return "left"
    if ratio > (2.0 / 3.0):
        return "right"
    return "center"


def _camera_search_side_from_zone(zone: Any, fallback: str = "left") -> str:
    raw = str(zone or "").strip().lower()
    if raw in {"left", "right"}:
        return raw
    fb = str(fallback or "").strip().lower()
    return fb if fb in {"left", "right"} else "left"


def _finite_positive_float(value: Any) -> Optional[float]:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    if math.isfinite(out) and out > 0.0:
        return float(out)
    return None


def _finite_nonnegative_float(value: Any) -> Optional[float]:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    if math.isfinite(out) and out >= 0.0:
        return float(out)
    return None


def _camera_distance_from_bbox(
    *,
    detector: str,
    bbox_height_px: Any,
    image_height_px: Any,
    detector_confidence: Any,
    bbox_width_px: Any = None,
    image_width_px: Any = None,
    bbox_area_ratio: Any = None,
) -> Dict[str, Any]:
    width_px = _finite_positive_float(bbox_width_px)
    height_px = _finite_positive_float(bbox_height_px)
    image_width = _finite_positive_float(image_width_px)
    image_height = _finite_positive_float(image_height_px)
    if height_px is None or image_height is None:
        return {}
    height_ratio = float(height_px) / max(1.0, float(image_height))
    width_ratio = None if width_px is None or image_width is None else float(width_px) / max(1.0, float(image_width))
    aspect = None if width_px is None else float(height_px) / max(1.0, float(width_px))
    det = str(detector or "unknown")
    area = _finite_positive_float(bbox_area_ratio)
    if area is None and width_ratio is not None:
        area = float(width_ratio) * float(height_ratio)
    if det == "mediapipe_pose" and _mediapipe_upper_body_bbox_candidate(
        width_ratio=width_ratio,
        height_ratio=height_ratio,
        aspect=aspect,
        area_ratio=area,
    ):
        return {
            "bbox_height_ratio": float(height_ratio),
            "distance_estimate_m": None,
            "distance_confidence": 0.0,
            "distance_source": "camera_bbox_upper_body_unreliable",
        }
    if height_ratio < CAMERA_DISTANCE_MIN_HEIGHT_RATIO:
        return {
            "bbox_height_ratio": float(height_ratio),
            "distance_estimate_m": None,
            "distance_confidence": 0.0,
            "distance_source": "camera_bbox_too_small",
        }
    reference = float(CAMERA_DISTANCE_DETECTOR_REFERENCE.get(det, CAMERA_DISTANCE_REFERENCE_HEIGHT_RATIO))
    distance_m = _clamp_float(
        reference / max(CAMERA_DISTANCE_MIN_HEIGHT_RATIO, float(height_ratio)),
        CAMERA_DISTANCE_MIN_M,
        CAMERA_DISTANCE_MAX_M,
    )
    det_conf = _clamp_float(float(detector_confidence or 0.0), 0.0, 1.0)
    det_weight = float(CAMERA_DISTANCE_DETECTOR_WEIGHT.get(det, 0.55))
    height_conf = _clamp_float((float(height_ratio) - CAMERA_DISTANCE_MIN_HEIGHT_RATIO) / 0.45, 0.0, 1.0)
    area_bonus = 0.0 if area is None else min(0.12, float(area) * 0.65)
    distance_conf = _clamp_float(det_conf * det_weight * (0.50 + 0.50 * height_conf) + area_bonus, 0.05, 1.0)
    if det == "opencv_motion_blob":
        distance_conf = min(distance_conf, 0.45)
        if (
            distance_m < CAMERA_DISTANCE_MOTION_BLOB_LOW_CONF_MIN_M
            and distance_conf <= CAMERA_DISTANCE_MOTION_BLOB_LOW_CONF_MAX_CONF
        ):
            distance_m = CAMERA_DISTANCE_MOTION_BLOB_LOW_CONF_MIN_M
            distance_conf = min(distance_conf, 0.30)
            distance_source = "camera_bbox_motion_blob_low_conf_floor"
        else:
            distance_source = "camera_bbox"
    else:
        distance_source = "camera_bbox"
    marginal_shape_for_distance = False
    if det == "onnx_yolov5_person":
        marginal_shape_for_distance = bool(
            height_ratio < 0.30
            or (width_ratio is not None and width_ratio > 0.80)
            or (aspect is not None and aspect < 1.05)
        )
    elif det == "opencv_template_lock":
        marginal_shape_for_distance = bool(
            height_ratio < 0.32
            or (aspect is not None and aspect < 1.10)
        )
    if marginal_shape_for_distance and distance_m > CAMERA_DISTANCE_MARGINAL_BBOX_MAX_M:
        distance_m = CAMERA_DISTANCE_MARGINAL_BBOX_MAX_M
        distance_conf = min(float(distance_conf), 0.28)
        distance_source = "camera_bbox_marginal_shape_capped"
    return {
        "bbox_height_ratio": float(height_ratio),
        "distance_estimate_m": float(distance_m),
        "distance_confidence": float(distance_conf),
        "distance_source": str(distance_source),
    }


def _mediapipe_upper_body_bbox_candidate(
    *,
    width_ratio: Any,
    height_ratio: Any,
    aspect: Any,
    area_ratio: Any,
) -> bool:
    width = _finite_positive_float(width_ratio)
    height = _finite_positive_float(height_ratio)
    aspect_v = _finite_positive_float(aspect)
    area = _finite_positive_float(area_ratio)
    if area is None and width is not None and height is not None:
        area = float(width) * float(height)
    return bool(
        width is not None
        and height is not None
        and aspect_v is not None
        and area is not None
        and CAMERA_MEDIAPIPE_UPPER_BODY_MIN_WIDTH_RATIO <= float(width) <= CAMERA_MEDIAPIPE_UPPER_BODY_MAX_WIDTH_RATIO
        and CAMERA_MEDIAPIPE_UPPER_BODY_MIN_HEIGHT_RATIO <= float(height) <= CAMERA_MEDIAPIPE_UPPER_BODY_MAX_HEIGHT_RATIO
        and float(aspect_v) <= CAMERA_MEDIAPIPE_UPPER_BODY_MAX_ASPECT
        and float(area) >= CAMERA_MEDIAPIPE_UPPER_BODY_MIN_AREA_RATIO
    )


def _camera_human_bbox_shape_status(
    *,
    detector: str,
    bbox_width_px: Any,
    bbox_height_px: Any,
    image_width_px: Any,
    image_height_px: Any,
    bbox_area_ratio: Any = None,
    bbox_fill_ratio: Any = None,
    bbox_center_offset_ratio: Any = None,
    onnx_score: Any = None,
    onnx_objectness: Any = None,
    onnx_person_class_score: Any = None,
) -> Dict[str, Any]:
    width_px = _finite_positive_float(bbox_width_px)
    height_px = _finite_positive_float(bbox_height_px)
    image_width = _finite_positive_float(image_width_px)
    image_height = _finite_positive_float(image_height_px)
    status: Dict[str, Any] = {
        "bbox_human_shape_ok": False,
        "bbox_reject_reason": "missing_bbox",
    }
    if width_px is None or height_px is None or image_width is None or image_height is None:
        return status

    width_ratio = float(width_px) / max(1.0, float(image_width))
    height_ratio = float(height_px) / max(1.0, float(image_height))
    aspect = float(height_px) / max(1.0, float(width_px))
    fill_ratio = _finite_positive_float(bbox_fill_ratio)
    area_ratio = _finite_positive_float(bbox_area_ratio)
    center_offset_ratio = _finite_nonnegative_float(bbox_center_offset_ratio)
    score = _finite_positive_float(onnx_score)
    objectness = _finite_positive_float(onnx_objectness)
    person_class_score = _finite_positive_float(onnx_person_class_score)
    det = str(detector or "unknown")
    min_width_ratio = CAMERA_HUMAN_BBOX_MIN_WIDTH_RATIO
    min_aspect = CAMERA_HUMAN_BBOX_MIN_ASPECT
    max_width_ratio = CAMERA_HUMAN_BBOX_MAX_WIDTH_RATIO
    min_height_ratio = CAMERA_HUMAN_BBOX_MIN_HEIGHT_RATIO
    if det == "opencv_motion_blob":
        min_aspect = CAMERA_MOTION_BLOB_MIN_ASPECT
        max_width_ratio = CAMERA_MOTION_BLOB_MAX_WIDTH_RATIO
    elif det == "mediapipe_pose":
        min_aspect = 0.80
        min_height_ratio = 0.16
        max_width_ratio = 0.82
    elif det == "onnx_yolov5_person":
        min_width_ratio = 0.045
        max_width_ratio = 0.76
        min_height_ratio = 0.22
        min_aspect = 1.10

    status.update(
        {
            "bbox_width_ratio": float(width_ratio),
            "bbox_height_ratio": float(height_ratio),
            "bbox_aspect_ratio": float(aspect),
            "bbox_fill_ratio": None if fill_ratio is None else float(fill_ratio),
            "bbox_area_ratio": None if area_ratio is None else float(area_ratio),
            "bbox_center_offset_ratio": None if center_offset_ratio is None else float(center_offset_ratio),
            "onnx_score": None if score is None else float(score),
            "onnx_objectness": None if objectness is None else float(objectness),
            "onnx_person_class_score": None if person_class_score is None else float(person_class_score),
        }
    )
    onnx_close_person_candidate = bool(
        det == "onnx_yolov5_person"
        and score is not None
        and objectness is not None
        and person_class_score is not None
        and area_ratio is not None
        and center_offset_ratio is not None
        and score >= CAMERA_ONNX_CLOSE_PERSON_MIN_SCORE
        and objectness >= CAMERA_ONNX_CLOSE_PERSON_MIN_OBJECTNESS
        and person_class_score >= CAMERA_ONNX_CLOSE_PERSON_MIN_PERSON_CLASS
        and height_ratio >= CAMERA_ONNX_CLOSE_PERSON_MIN_HEIGHT_RATIO
        and aspect >= CAMERA_ONNX_CLOSE_PERSON_MIN_ASPECT
        and area_ratio >= CAMERA_ONNX_CLOSE_PERSON_MIN_AREA_RATIO
        and center_offset_ratio <= CAMERA_ONNX_CLOSE_PERSON_MAX_CENTER_OFFSET_RATIO
    )
    if width_ratio < min_width_ratio:
        status["bbox_reject_reason"] = "bbox_too_narrow_for_human"
        return status
    if width_ratio > max_width_ratio:
        if det == "mediapipe_pose" and _mediapipe_upper_body_bbox_candidate(
            width_ratio=width_ratio,
            height_ratio=height_ratio,
            aspect=aspect,
            area_ratio=area_ratio,
        ):
            status["bbox_human_shape_ok"] = True
            status["bbox_reject_reason"] = ""
            status["bbox_shape_variant"] = "mediapipe_upper_body"
            return status
        status["bbox_reject_reason"] = "bbox_too_wide_for_human"
        return status
    if height_ratio < min_height_ratio:
        if det == "mediapipe_pose" and _mediapipe_upper_body_bbox_candidate(
            width_ratio=width_ratio,
            height_ratio=height_ratio,
            aspect=aspect,
            area_ratio=area_ratio,
        ):
            status["bbox_human_shape_ok"] = True
            status["bbox_reject_reason"] = ""
            status["bbox_shape_variant"] = "mediapipe_upper_body"
            return status
        status["bbox_reject_reason"] = "bbox_too_short_for_human"
        return status
    if height_ratio > CAMERA_HUMAN_BBOX_MAX_HEIGHT_RATIO:
        status["bbox_reject_reason"] = "bbox_too_tall_for_human"
        return status
    if aspect < min_aspect:
        if det == "mediapipe_pose" and _mediapipe_upper_body_bbox_candidate(
            width_ratio=width_ratio,
            height_ratio=height_ratio,
            aspect=aspect,
            area_ratio=area_ratio,
        ):
            status["bbox_human_shape_ok"] = True
            status["bbox_reject_reason"] = ""
            status["bbox_shape_variant"] = "mediapipe_upper_body"
            return status
        if onnx_close_person_candidate:
            status["bbox_human_shape_ok"] = True
            status["bbox_reject_reason"] = ""
            status["bbox_shape_variant"] = "onnx_close_upper_body"
            return status
        status["bbox_reject_reason"] = "bbox_not_upright_human"
        return status
    if aspect > CAMERA_HUMAN_BBOX_MAX_ASPECT:
        status["bbox_reject_reason"] = "bbox_too_thin_for_human"
        return status
    if det == "opencv_motion_blob" and fill_ratio is not None:
        if fill_ratio < CAMERA_MOTION_BLOB_MIN_FILL_RATIO:
            status["bbox_reject_reason"] = "motion_blob_too_sparse_for_human"
            return status
        if fill_ratio > CAMERA_MOTION_BLOB_MAX_FILL_RATIO:
            status["bbox_reject_reason"] = "motion_blob_too_solid_for_human"
            return status

    status["bbox_human_shape_ok"] = True
    status["bbox_reject_reason"] = ""
    return status


def _camera_distance_status_value(camera_status: Dict[str, Any]) -> Tuple[Optional[float], float]:
    dist = _finite_positive_float((camera_status or {}).get("distance_estimate_m"))
    if dist is None:
        return None, 0.0
    conf = _clamp_float(float((camera_status or {}).get("distance_confidence") or 0.0), 0.0, 1.0)
    return float(dist), float(conf)


class FollowerCamera:
    """
    Kamera + pózdetekció + target persistence egyetlen osztályban.
    Kiváltja a korábbi 6 darab modul-szintű globális változót.
    """

    def __init__(
        self,
        camera_open_cooldown_sec: float = 10.0,
        max_failed_sessions: int = 2,
        persistence_timeout_s: float = 2.4,
    ):
        self._camera = None
        self._pose_detector = None
        self._last_camera_fail_time: float = 0.0
        self._camera_open_cooldown_sec = camera_open_cooldown_sec
        self._failed_sessions: int = 0
        self._max_failed_sessions = max_failed_sessions
        # Target persistence: utolsó detekciós eredmény megőrzése rövid kiesésre
        # _last_center: (cx, cy, im_w) – 3-tuple a kép szélesség megőrzéséhez
        self._last_center: Optional[Tuple[float, float, float]] = None
        self._last_detection_ts: float = 0.0
        self._persistence_timeout_s = persistence_timeout_s
        self._last_result_status: Dict[str, Any] = {
            "state": "idle",
            "source": "camera",
            "stale": False,
            "target_visible": False,
            "target_usable": False,
            "frame_ok": False,
        }
        self._last_frame_rotation_deg: int = 0
        self._last_open_failed: bool = False
        self._last_open_error: str = ""
        self._pose_detector_unavailable: bool = False
        self._onnx_session = None
        self._onnx_input_name: str = ""
        self._onnx_input_size: int = CAMERA_ONNX_INPUT_SIZE
        self._onnx_detector_unavailable: bool = False
        self._onnx_detector_unavailable_ts: float = 0.0
        self._last_onnx_infer_ts: float = 0.0
        self._hog_detector = None
        self._prev_motion_gray = None
        self._last_detector_name: str = "none"
        self._last_detector_confidence: float = 0.0
        self._last_detector_error: str = ""
        self._last_bbox_status: Dict[str, Any] = {}
        self._last_human_detector_ts: float = 0.0
        self._last_human_detector_name: str = ""
        self._template_lock_gray = None
        self._template_lock_bbox: Optional[Tuple[float, float, float, float, float, float]] = None
        self._template_lock_ts: float = 0.0
        self._template_lock_score: float = 0.0
        self._candidate_history = deque(maxlen=8)
        self._lock_active: bool = False
        self._lock_id: int = 0
        self._lock_center_x_ratio: Optional[float] = None
        self._lock_width_ratio: Optional[float] = None
        self._lock_height_ratio: Optional[float] = None
        self._lock_started_ts: float = 0.0
        self._last_lock_ts: float = 0.0
        self._lost_count: int = 0
        self._relock_count: int = 0
        self._last_lock_lost_reason: str = ""
        self._last_lock_image_path: str = ""
        self._last_detector_latency_ms: float = 0.0
        self._last_search_side: str = "left"
        self._last_capture_request_ts: float = 0.0
        self._last_capture_status: str = "idle"
        self._last_detection_process_ts: float = 0.0
        self._last_usable_detector_name: str = ""
        self._last_usable_detector_confidence: float = 0.0
        self._async_lock = threading.Lock()
        self._async_thread: Optional[threading.Thread] = None
        self._async_enabled: bool = False
        self._async_generation: int = 0
        self._async_result: Optional[Tuple[float, float, float, float]] = None
        self._async_status: Dict[str, Any] = {}
        self._async_started_ts: float = 0.0
        self._async_completed_ts: float = 0.0
        self._async_update_seq: int = 0

    # ── kamera lifecycle ──

    def ensure_open(self, ctrl, width: int = CAMERA_CAPTURE_SIZE[0], height: int = CAMERA_CAPTURE_SIZE[1]) -> bool:
        """Lazy init: Picamera2 kis felbontással. Throttle + retry."""
        if self._camera is not None:
            return True
        now = time.monotonic()
        if now - self._last_camera_fail_time < self._camera_open_cooldown_sec:
            return False
        for attempt in range(3):
            try:
                from picamera2 import Picamera2
                with camera_lifecycle_lock():
                    cam = Picamera2()
                    config = cam.create_preview_configuration(main={"size": (width, height)})
                    cam.configure(config)
                    cam.start()
                time.sleep(0.3)
                self._camera = cam
                self._failed_sessions = 0
                self._last_open_failed = False
                self._last_open_error = ""
                if hasattr(ctrl, "logger"):
                    ctrl.logger.info("[FOLLOW] Kamera indítva (követéshez).")
                return True
            except Exception as e:
                safe_stop_close(locals().get("cam"))
                err_msg = str(e)
                self._last_open_error = err_msg
                if hasattr(ctrl, "logger"):
                    ctrl.logger.warn(f"[FOLLOW] Kamera hiba (próbálkozás {attempt + 1}/3): {e}")
                if "resource busy" in err_msg.lower() or "did not complete" in err_msg.lower() or "acquire" in err_msg.lower():
                    if attempt < 2:
                        time.sleep(2.0)
                else:
                    break
        self._last_camera_fail_time = now
        self._failed_sessions += 1
        self._last_open_failed = True
        return False

    def release(self, ctrl=None):
        """Kamera felszabadítása és belső állapot reset."""
        if self._camera is not None:
            safe_stop_close(self._camera)
            self._camera = None
        if ctrl and hasattr(ctrl, "logger"):
            ctrl.logger.info("[FOLLOW] Kamera leállítva.")
        self._last_center = None
        self._last_detection_ts = 0.0
        self._template_lock_gray = None
        self._template_lock_bbox = None
        self._template_lock_ts = 0.0
        self._template_lock_score = 0.0
        self._onnx_detector_unavailable = False
        self._onnx_detector_unavailable_ts = 0.0
        self._last_capture_status = "released"
        self._reset_lock_state(clear_history=True, reason="camera_release")

    @property
    def too_many_failures(self) -> bool:
        return self._failed_sessions >= self._max_failed_sessions

    def target_age_s(self) -> Optional[float]:
        if self._last_detection_ts <= 0.0:
            return None
        return max(0.0, time.monotonic() - self._last_detection_ts)

    def last_status(self) -> Dict[str, Any]:
        with self._async_lock:
            if self._async_enabled and self._async_status:
                status = dict(self._async_status)
                completed_ts = float(self._async_completed_ts or 0.0)
                result_age_s = max(0.0, time.monotonic() - completed_ts) if completed_ts > 0.0 else None
                worker_running = bool(self._async_thread is not None and self._async_thread.is_alive())
                status.update(
                    {
                        "async_worker_active": True,
                        "async_inference_running": worker_running,
                        "async_result_age_s": result_age_s,
                        "async_update_seq": int(self._async_update_seq),
                    }
                )
                if result_age_s is None or result_age_s > CAMERA_ASYNC_RESULT_MAX_AGE_S:
                    status["state"] = "async_result_stale"
                    status["stale"] = True
                    status["target_visible"] = False
                    status["target_usable"] = False
                    status["async_stale_gate"] = True
                return status
        return dict(self._last_result_status or {})

    def _async_detect_once(self, ctrl, generation: int) -> None:
        result: Optional[Tuple[float, float, float, float]] = None
        status: Dict[str, Any]
        try:
            result = self.detect_with_persistence(ctrl)
            status = dict(self._last_result_status or {})
        except Exception as exc:
            status = {
                "state": "async_detector_error",
                "source": "camera",
                "stale": True,
                "target_visible": False,
                "target_usable": False,
                "frame_ok": False,
                "detector_error": str(exc),
            }
        completed_ts = time.monotonic()
        release_after = False
        with self._async_lock:
            if self._async_enabled and int(generation) == int(self._async_generation):
                self._async_result = result
                self._async_status = dict(status)
                self._async_completed_ts = float(completed_ts)
                self._async_update_seq += 1
            release_after = not bool(self._async_enabled)
            self._async_thread = None
        if release_after:
            self.release(ctrl)

    def detect_latest(self, ctrl) -> Optional[Tuple[float, float, float, float]]:
        """Return the freshest completed detection and schedule at most one background inference."""
        now = time.monotonic()
        initial_result = self._persisted_center()
        with self._async_lock:
            if not self._async_enabled:
                self._async_enabled = True
                self._async_generation += 1
                if self._async_result is None and initial_result is not None:
                    self._async_result = initial_result
                    self._async_status = dict(self._last_result_status or {})
                    self._async_completed_ts = float(now)
            worker_running = bool(self._async_thread is not None and self._async_thread.is_alive())
            if not worker_running:
                generation = int(self._async_generation)
                thread = threading.Thread(
                    target=self._async_detect_once,
                    args=(ctrl, generation),
                    name="r2b4-follow-perception",
                    daemon=True,
                )
                self._async_thread = thread
                self._async_started_ts = float(now)
                thread.start()
            completed_ts = float(self._async_completed_ts or 0.0)
            result_age_s = max(0.0, now - completed_ts) if completed_ts > 0.0 else None
            if result_age_s is None or result_age_s > CAMERA_ASYNC_RESULT_MAX_AGE_S:
                return None
            return self._async_result

    def stop_async(self, ctrl=None) -> None:
        """Disable publication immediately; an in-flight detector releases the camera when it returns."""
        release_now = False
        with self._async_lock:
            self._async_enabled = False
            self._async_generation += 1
            self._async_result = None
            self._async_status = {}
            self._async_completed_ts = 0.0
            release_now = not bool(self._async_thread is not None and self._async_thread.is_alive())
        if release_now:
            self.release(ctrl)

    def _reset_lock_state(self, *, clear_history: bool = True, reason: str = "reset") -> None:
        if bool(getattr(self, "_lock_active", False)):
            self._lost_count += 1
        self._lock_active = False
        self._lock_center_x_ratio = None
        self._lock_width_ratio = None
        self._lock_height_ratio = None
        self._lock_started_ts = 0.0
        self._last_lock_lost_reason = str(reason or "")
        if bool(clear_history):
            self._candidate_history.clear()

    def _lock_common_status(self, *, zone: str = "unknown", reason: str = "") -> Dict[str, Any]:
        now = time.monotonic()
        lock_age_s = 0.0
        if bool(getattr(self, "_lock_active", False)) and float(getattr(self, "_lock_started_ts", 0.0) or 0.0) > 0.0:
            lock_age_s = max(0.0, now - float(self._lock_started_ts))
        return {
            "target_zone": str(zone or "unknown"),
            "lock_state": "locked" if bool(getattr(self, "_lock_active", False)) else "candidate",
            "lock_confirmed": bool(getattr(self, "_lock_active", False)),
            "lock_required_frames": int(CAMERA_LOCK_CONFIRM_FRAMES),
            "lock_confirm_count": int(len(getattr(self, "_candidate_history", []) or [])),
            "lock_reason": str(reason or ""),
            "lock_id": int(getattr(self, "_lock_id", 0) or 0),
            "lock_age_s": float(lock_age_s),
            "lost_count": int(getattr(self, "_lost_count", 0) or 0),
            "relock_count": int(getattr(self, "_relock_count", 0) or 0),
            "last_lock_lost_reason": str(getattr(self, "_last_lock_lost_reason", "") or ""),
            "last_lock_image_path": str(getattr(self, "_last_lock_image_path", "") or ""),
            "last_search_side": str(getattr(self, "_last_search_side", "left") or "left"),
            "detector_latency_ms": float(getattr(self, "_last_detector_latency_ms", 0.0) or 0.0),
        }

    def _lock_min_confidence(self, detector: str) -> float:
        return float(CAMERA_LOCK_MIN_CONFIDENCE.get(str(detector or ""), 0.60))

    def _confirmed_history(self, now: float):
        window_s = float(CAMERA_LOCK_CONFIRM_WINDOW_S)
        kept = [entry for entry in list(self._candidate_history) if float(now) - float(entry.get("ts", 0.0)) <= window_s]
        self._candidate_history.clear()
        self._candidate_history.extend(kept)
        return kept

    def _update_detection_lock(self, center: Tuple[float, float], image_width: int, image_height: int) -> Tuple[bool, Dict[str, Any]]:
        now = time.monotonic()
        detector = str(getattr(self, "_last_detector_name", "") or "unknown")
        confidence = _clamp_float(float(getattr(self, "_last_detector_confidence", 0.0) or 0.0), 0.0, 1.0)
        bbox_status = dict(getattr(self, "_last_bbox_status", {}) or {})
        zone = _camera_target_zone_from_center(center[0], image_width)
        status = self._lock_common_status(zone=zone)
        status["lock_state"] = "candidate"
        status["lock_confirmed"] = False
        status["target_zone"] = str(zone)

        if not bool(bbox_status.get("bbox_human_shape_ok", False)):
            status["lock_reason"] = str(bbox_status.get("bbox_reject_reason") or "bbox_shape_rejected")
            return False, status

        history_for_seed = self._confirmed_history(now)
        status["lock_confirm_count"] = int(len(history_for_seed))
        onnx_seed_count = sum(
            1
            for item in history_for_seed
            if str(item.get("detector") or "") == "onnx_yolov5_person"
            and float(now) - float(item.get("ts", 0.0)) <= float(CAMERA_ONNX_SEEDED_FALLBACK_WINDOW_S)
        )
        seeded_fallback_min_conf = None
        if (
            not bool(getattr(self, "_lock_active", False))
            and int(onnx_seed_count) > 0
            and detector in {"opencv_motion_blob", "opencv_template_lock"}
        ):
            seeded_fallback_min_conf = (
                float(CAMERA_ONNX_SEEDED_MOTION_MIN_CONFIDENCE)
                if detector == "opencv_motion_blob"
                else float(CAMERA_ONNX_SEEDED_TEMPLATE_MIN_CONFIDENCE)
            )
        onnx_seeded_fallback = bool(seeded_fallback_min_conf is not None and confidence >= float(seeded_fallback_min_conf))
        last_lock_ts_for_fallback = float(getattr(self, "_last_lock_ts", 0.0) or 0.0)
        recent_template_relock_context_for_min = bool(
            detector == "opencv_template_lock"
            and last_lock_ts_for_fallback > 0.0
            and (float(now) - last_lock_ts_for_fallback) <= float(CAMERA_TEMPLATE_RELOCK_MAX_LAST_LOCK_AGE_S)
        )
        recent_human_template_relock_context_for_min = bool(
            detector == "opencv_template_lock"
            and self._template_lock_allowed()
        )
        min_conf = self._lock_min_confidence(detector)
        effective_min_conf = float(seeded_fallback_min_conf if seeded_fallback_min_conf is not None else min_conf)
        if bool(recent_template_relock_context_for_min or recent_human_template_relock_context_for_min):
            effective_min_conf = min(float(effective_min_conf), float(CAMERA_TEMPLATE_RELOCK_MIN_CONFIDENCE))
        if confidence < effective_min_conf:
            reason = "confidence_below_onnx_seeded_fallback_min" if seeded_fallback_min_conf is not None else "confidence_below_lock_min"
            status["lock_reason"] = f"{reason}:{confidence:.2f}<{effective_min_conf:.2f}"
            return False, status
        recent_template_relock_allowed = bool(
            detector == "opencv_template_lock"
            and last_lock_ts_for_fallback > 0.0
            and (float(now) - last_lock_ts_for_fallback) <= float(CAMERA_TEMPLATE_RELOCK_MAX_LAST_LOCK_AGE_S)
            and float(confidence) >= float(CAMERA_TEMPLATE_RELOCK_MIN_CONFIDENCE)
        )
        recent_human_template_relock_allowed = bool(
            detector == "opencv_template_lock"
            and self._template_lock_allowed()
            and float(confidence) >= float(CAMERA_TEMPLATE_RELOCK_MIN_CONFIDENCE)
        )
        if (
            detector in {"opencv_template_lock", "opencv_motion_blob"}
            and not bool(getattr(self, "_lock_active", False))
            and not recent_template_relock_allowed
            and not recent_human_template_relock_allowed
            and not bool(onnx_seeded_fallback)
        ):
            status["lock_reason"] = "fallback_detector_requires_existing_lock"
            return False, status
        if detector == "opencv_hog" and self._ensure_onnx_person_detector():
            status["lock_reason"] = "hog_ignored_while_onnx_available"
            return False, status

        center_ratio = float(center[0]) / max(1.0, float(image_width))
        width_ratio = _finite_positive_float(bbox_status.get("bbox_width_ratio")) or 0.0
        height_ratio = _finite_positive_float(bbox_status.get("bbox_height_ratio")) or 0.0
        entry = {
            "ts": float(now),
            "detector": detector,
            "confidence": float(confidence),
            "center_x_ratio": float(center_ratio),
            "width_ratio": float(width_ratio),
            "height_ratio": float(height_ratio),
            "zone": str(zone),
            "onnx_seeded_fallback": bool(onnx_seeded_fallback),
        }

        if bool(getattr(self, "_lock_active", False)):
            last_center = self._lock_center_x_ratio
            tracking_reason = "lock_tracking"
            if last_center is not None and abs(float(center_ratio) - float(last_center)) > CAMERA_LOCK_MAX_CENTER_JUMP_RATIO:
                center_jump_ratio = abs(float(center_ratio) - float(last_center))
                last_width = _finite_positive_float(self._lock_width_ratio)
                last_height = _finite_positive_float(self._lock_height_ratio)
                width_delta = 0.0 if last_width is None else abs(float(width_ratio) - float(last_width))
                height_delta = 0.0 if last_height is None else abs(float(height_ratio) - float(last_height))
                strong_same_target_jump = bool(
                    detector == "onnx_yolov5_person"
                    and float(confidence) >= float(CAMERA_LOCK_STRONG_JUMP_MIN_CONFIDENCE)
                    and float(center_jump_ratio) <= float(CAMERA_LOCK_STRONG_JUMP_MAX_CENTER_RATIO)
                    and (last_width is None or float(width_delta) <= float(CAMERA_LOCK_STRONG_JUMP_MAX_WIDTH_DELTA_RATIO))
                    and (last_height is None or float(height_delta) <= float(CAMERA_LOCK_STRONG_JUMP_MAX_HEIGHT_DELTA_RATIO))
                )
                if not strong_same_target_jump:
                    self._reset_lock_state(clear_history=True, reason="bbox_center_jump")
                    status = self._lock_common_status(zone=zone, reason="bbox_center_jump")
                    status["lock_state"] = "candidate"
                    status["lock_confirmed"] = False
                    status["lock_center_jump_ratio"] = float(center_jump_ratio)
                    return False, status
                tracking_reason = "lock_tracking_strong_jump"
                status["lock_center_jump_ratio"] = float(center_jump_ratio)
            self._candidate_history.append(entry)
            self._lock_center_x_ratio = float(center_ratio)
            self._lock_width_ratio = float(width_ratio)
            self._lock_height_ratio = float(height_ratio)
            self._last_lock_ts = now
            if zone in {"left", "right"}:
                self._last_search_side = str(zone)
            jump_ratio = status.get("lock_center_jump_ratio")
            status = self._lock_common_status(zone=zone, reason=tracking_reason)
            if jump_ratio is not None:
                status["lock_center_jump_ratio"] = float(jump_ratio)
            return True, status

        self._candidate_history.append(entry)
        history = self._confirmed_history(now)
        last_lock_ts = float(getattr(self, "_last_lock_ts", 0.0) or 0.0)
        recent_lock_age_s = None if last_lock_ts <= 0.0 else max(0.0, float(now) - last_lock_ts)
        startup_onnx_single_frame_lock_path = bool(
            last_lock_ts <= 0.0
            and detector == "onnx_yolov5_person"
            and float(confidence) >= float(CAMERA_ONNX_STARTUP_SINGLE_FRAME_LOCK_MIN_CONFIDENCE)
            and 0.0 < height_ratio <= float(CAMERA_ONNX_STARTUP_SINGLE_FRAME_MAX_HEIGHT_RATIO)
            and len(history) >= 1
        )
        if startup_onnx_single_frame_lock_path:
            self._candidate_history.clear()
            self._candidate_history.append(entry)
            history = [entry]
        startup_onnx_two_frame_history = [
            item
            for item in history
            if str(item.get("detector") or "") == "onnx_yolov5_person"
            and float(item.get("confidence") or 0.0) >= float(CAMERA_ONNX_STARTUP_TWO_FRAME_LOCK_MIN_CONFIDENCE)
            and 0.0 < float(item.get("height_ratio") or 0.0) <= float(CAMERA_ONNX_STARTUP_SINGLE_FRAME_MAX_HEIGHT_RATIO)
        ]
        startup_onnx_two_frame_lock_path = bool(
            not startup_onnx_single_frame_lock_path
            and last_lock_ts <= 0.0
            and detector == "onnx_yolov5_person"
            and float(confidence) >= float(CAMERA_ONNX_STARTUP_TWO_FRAME_LOCK_MIN_CONFIDENCE)
            and 0.0 < height_ratio <= float(CAMERA_ONNX_STARTUP_SINGLE_FRAME_MAX_HEIGHT_RATIO)
            and len(startup_onnx_two_frame_history) >= 2
        )
        if startup_onnx_two_frame_lock_path:
            history = list(startup_onnx_two_frame_history[-2:])
            self._candidate_history.clear()
            self._candidate_history.extend(history)
        strong_relock_count = sum(
            1
            for item in history
            if str(item.get("detector") or "") == "onnx_yolov5_person"
            and float(item.get("confidence") or 0.0) >= float(CAMERA_RELOCK_MIN_CONFIDENCE)
        )
        template_relock_count = sum(
            1
            for item in history
            if str(item.get("detector") or "") == "opencv_template_lock"
            and float(item.get("confidence") or 0.0) >= float(CAMERA_TEMPLATE_RELOCK_MIN_CONFIDENCE)
        )
        relock_fast_path = bool(
            recent_lock_age_s is not None
            and float(recent_lock_age_s) <= float(CAMERA_RELOCK_MAX_LAST_LOCK_AGE_S)
            and int(strong_relock_count) >= int(CAMERA_RELOCK_CONFIRM_FRAMES)
        )
        recent_onnx_single_frame_relock_path = bool(
            recent_lock_age_s is not None
            and float(recent_lock_age_s) <= float(CAMERA_ONNX_RECENT_LOCK_RELOCK_MAX_LAST_LOCK_AGE_S)
            and detector == "onnx_yolov5_person"
            and float(confidence) >= float(CAMERA_ONNX_RECENT_LOCK_RELOCK_MIN_CONFIDENCE)
            and len(history) >= 1
        )
        recent_human_onnx_single_frame_relock_path = bool(
            detector == "onnx_yolov5_person"
            and self._template_lock_allowed()
            and float(confidence) >= float(CAMERA_ONNX_RECENT_HUMAN_SINGLE_FRAME_RELOCK_MIN_CONFIDENCE)
            and len(history) >= 1
        )
        recent_template_single_frame_relock_path = bool(
            recent_lock_age_s is not None
            and float(recent_lock_age_s) <= float(CAMERA_TEMPLATE_RECENT_SINGLE_FRAME_RELOCK_MAX_LAST_LOCK_AGE_S)
            and detector == "opencv_template_lock"
            and self._template_lock_allowed()
            and float(confidence) >= float(CAMERA_TEMPLATE_RECENT_SINGLE_FRAME_RELOCK_MIN_CONFIDENCE)
            and len(history) >= 1
        )
        if bool(
            recent_onnx_single_frame_relock_path
            or recent_human_onnx_single_frame_relock_path
            or recent_template_single_frame_relock_path
        ):
            self._candidate_history.clear()
            self._candidate_history.append(entry)
            history = [entry]
            strong_relock_count = sum(
                1
                for item in history
                if str(item.get("detector") or "") == "onnx_yolov5_person"
                and float(item.get("confidence") or 0.0) >= float(CAMERA_RELOCK_MIN_CONFIDENCE)
            )
            template_relock_count = sum(
                1
                for item in history
                if str(item.get("detector") or "") == "opencv_template_lock"
                and float(item.get("confidence") or 0.0) >= float(CAMERA_TEMPLATE_RELOCK_MIN_CONFIDENCE)
            )
        template_relock_context = bool(
            (
                recent_lock_age_s is not None
                and float(recent_lock_age_s) <= float(CAMERA_TEMPLATE_RELOCK_MAX_LAST_LOCK_AGE_S)
            )
            or recent_human_template_relock_allowed
        )
        template_relock_path = bool(
            template_relock_context
            and int(template_relock_count) >= int(CAMERA_LOCK_CONFIRM_FRAMES)
        )
        required_frames = int(
            1
            if (
                startup_onnx_single_frame_lock_path
                or recent_onnx_single_frame_relock_path
                or recent_human_onnx_single_frame_relock_path
                or recent_template_single_frame_relock_path
            )
            else (
                CAMERA_RELOCK_CONFIRM_FRAMES
                if (relock_fast_path or startup_onnx_two_frame_lock_path)
                else CAMERA_LOCK_CONFIRM_FRAMES
            )
        )
        status["lock_confirm_count"] = int(len(history))
        status["lock_required_frames"] = int(required_frames)
        if recent_lock_age_s is not None:
            status["recent_lock_age_s"] = float(recent_lock_age_s)
        if len(history) < int(required_frames):
            status["lock_reason"] = "confirming_frames"
            return False, status
        timespan = float(history[-1]["ts"]) - float(history[0]["ts"])
        min_timespan_s = float(
            0.0
            if (
                startup_onnx_single_frame_lock_path
                or recent_onnx_single_frame_relock_path
                or recent_human_onnx_single_frame_relock_path
                or recent_template_single_frame_relock_path
            )
            else (
                CAMERA_RELOCK_MIN_TIMESPAN_S
                if (relock_fast_path or startup_onnx_two_frame_lock_path)
                else CAMERA_LOCK_MIN_TIMESPAN_S
            )
        )
        if timespan < min_timespan_s:
            status["lock_reason"] = "confirm_window_too_short"
            return False, status
        strong_count = sum(1 for item in history if str(item.get("detector") or "") in CAMERA_LOCK_STRONG_DETECTORS)
        onnx_seeded_fallback_count = sum(1 for item in history if bool(item.get("onnx_seeded_fallback", False)))
        onnx_seeded_fallback_path = bool(int(strong_count) >= 1 and int(onnx_seeded_fallback_count) >= 1)
        status["lock_onnx_seeded_fallback_path"] = bool(onnx_seeded_fallback_path)
        if (
            strong_count < 2
            and not template_relock_path
            and not onnx_seeded_fallback_path
            and not startup_onnx_single_frame_lock_path
            and not startup_onnx_two_frame_lock_path
            and not recent_onnx_single_frame_relock_path
            and not recent_human_onnx_single_frame_relock_path
            and not recent_template_single_frame_relock_path
        ):
            status["lock_reason"] = "strong_detector_confirmation_missing"
            return False, status
        centers = [float(item.get("center_x_ratio", 0.0)) for item in history]
        heights = [float(item.get("height_ratio", 0.0)) for item in history if float(item.get("height_ratio", 0.0)) > 0.0]
        widths = [float(item.get("width_ratio", 0.0)) for item in history if float(item.get("width_ratio", 0.0)) > 0.0]
        max_height_span = (
            float(CAMERA_ONNX_SEEDED_FALLBACK_MAX_HEIGHT_SPAN_RATIO)
            if onnx_seeded_fallback_path
            else float(CAMERA_LOCK_MAX_HEIGHT_SPAN_RATIO)
        )
        max_width_span = (
            float(CAMERA_ONNX_SEEDED_FALLBACK_MAX_WIDTH_SPAN_RATIO)
            if onnx_seeded_fallback_path
            else float(CAMERA_LOCK_MAX_WIDTH_SPAN_RATIO)
        )
        if max(centers) - min(centers) > float(CAMERA_LOCK_MAX_CENTER_SPAN_RATIO):
            if detector in CAMERA_LOCK_STRONG_DETECTORS or bool(template_relock_context or onnx_seeded_fallback_path):
                self._candidate_history.clear()
                self._candidate_history.append(entry)
                status["lock_reason"] = "confirming_frames"
                status["lock_unstable_history_reset"] = True
                status["lock_confirm_count"] = 1
                return False, status
            status["lock_reason"] = "bbox_center_unstable"
            return False, status
        if heights and max(heights) - min(heights) > float(max_height_span):
            if detector in CAMERA_LOCK_STRONG_DETECTORS or bool(template_relock_context or onnx_seeded_fallback_path):
                self._candidate_history.clear()
                self._candidate_history.append(entry)
                status["lock_reason"] = "confirming_frames"
                status["lock_unstable_history_reset"] = True
                status["lock_confirm_count"] = 1
                return False, status
            status["lock_reason"] = "bbox_height_unstable"
            return False, status
        if widths and max(widths) - min(widths) > float(max_width_span):
            if detector in CAMERA_LOCK_STRONG_DETECTORS or bool(template_relock_context or onnx_seeded_fallback_path):
                self._candidate_history.clear()
                self._candidate_history.append(entry)
                status["lock_reason"] = "confirming_frames"
                status["lock_unstable_history_reset"] = True
                status["lock_confirm_count"] = 1
                return False, status
            status["lock_reason"] = "bbox_width_unstable"
            return False, status

        if float(getattr(self, "_last_lock_ts", 0.0) or 0.0) > 0.0:
            self._relock_count += 1
        self._lock_id += 1
        self._lock_active = True
        self._lock_center_x_ratio = float(center_ratio)
        self._lock_width_ratio = float(width_ratio)
        self._lock_height_ratio = float(height_ratio)
        self._lock_started_ts = now
        self._last_lock_ts = now
        self._last_lock_lost_reason = ""
        if zone in {"left", "right"}:
            self._last_search_side = str(zone)
        status = self._lock_common_status(zone=zone, reason="lock_confirmed")
        status["lock_new"] = True
        status["lock_confirm_count"] = int(len(history))
        status["lock_required_frames"] = int(required_frames)
        status["lock_relock_fast_path"] = bool(relock_fast_path)
        status["lock_startup_onnx_single_frame_path"] = bool(startup_onnx_single_frame_lock_path)
        status["lock_startup_onnx_two_frame_path"] = bool(startup_onnx_two_frame_lock_path)
        status["lock_recent_onnx_single_frame_relock_path"] = bool(recent_onnx_single_frame_relock_path)
        status["lock_recent_human_onnx_single_frame_relock_path"] = bool(recent_human_onnx_single_frame_relock_path)
        status["lock_recent_template_single_frame_relock_path"] = bool(recent_template_single_frame_relock_path)
        status["lock_template_relock_path"] = bool(template_relock_path)
        status["lock_onnx_seeded_fallback_path"] = bool(onnx_seeded_fallback_path)
        return True, status

    def _save_lock_audit_image(self, frame_rgb, status: Dict[str, Any]) -> str:
        try:
            from PIL import Image, ImageDraw

            os.makedirs(PIC_DIR, exist_ok=True)
            img = Image.fromarray(frame_rgb, mode="RGB")
            draw = ImageDraw.Draw(img)
            x = _finite_nonnegative_float(status.get("bbox_x_px"))
            y = _finite_nonnegative_float(status.get("bbox_y_px"))
            w = _finite_positive_float(status.get("bbox_width_px"))
            h = _finite_positive_float(status.get("bbox_height_px"))
            color = (70, 255, 120)
            if x is not None and y is not None and w is not None and h is not None:
                rect = [float(x), float(y), float(x) + float(w), float(y) + float(h)]
                draw.rectangle(rect, outline=(0, 0, 0), width=5)
                draw.rectangle(rect, outline=color, width=3)
            detector = str(status.get("detector") or "unknown")
            confidence = _clamp_float(float(status.get("detector_confidence") or status.get("confidence") or 0.0), 0.0, 1.0)
            zone = str(status.get("target_zone") or "unknown")
            lock_state = str(status.get("lock_state") or "unknown")
            label = f"{detector} conf={confidence:.2f} zone={zone} lock={lock_state}"
            draw.rectangle([2, 2, min(float(img.size[0] - 2), 310.0), 22], fill=(0, 0, 0))
            draw.text((6, 6), label, fill=color)
            ts = time.strftime("%Y%m%dT%H%M%S", time.localtime())
            ms = int((time.time() % 1.0) * 1000.0)
            conf_token = f"{confidence:.2f}".replace(".", "p")
            safe_detector = "".join(ch if ch.isalnum() or ch in {"_", "-"} else "_" for ch in detector)[:32]
            filename = f"human_lock_{ts}_{ms:03d}_{zone}_{safe_detector}_conf_{conf_token}.jpg"
            path = os.path.join(PIC_DIR, filename)
            img.save(path, "JPEG", quality=88, optimize=False)
            rel = os.path.relpath(path, PROJECT_ROOT)
            self._last_lock_image_path = rel
            return rel
        except Exception as e:
            self._last_detector_error = f"lock_audit_image_error:{e}"
            return ""

    # ── detekció ──

    def prime_from_stream_frame(self, ctrl) -> Optional[Tuple[float, float, float, float, Dict[str, Any]]]:
        """
        FOLLOW indítási seed a GUI stream utolsó frame-jéből.
        Nem nyit kamerát; csak erős ONNX emberdetektorra publikálható.
        """
        runtime_dir = os.path.dirname(str(getattr(ctrl, "status_path", "") or ""))
        if not runtime_dir:
            return None
        stream_path = os.path.join(runtime_dir, "stream_frame.jpg")
        try:
            mtime = float(os.path.getmtime(stream_path))
        except Exception:
            return None
        age_s = max(0.0, time.time() - mtime)
        if age_s > FOLLOW_START_STREAM_SEED_MAX_AGE_S:
            return None
        try:
            import numpy as np
            from PIL import Image

            with Image.open(stream_path) as img:
                frame_rgb = np.asarray(img.convert("RGB"))
            height, width = frame_rgb.shape[:2]
            if width <= 0 or height <= 0:
                return None

            self._last_detector_name = "none"
            self._last_detector_confidence = 0.0
            self._last_detector_error = ""
            self._last_bbox_status = {}
            self._last_onnx_infer_ts = 0.0
            center = self._detect_onnx_person_bbox(frame_rgb, int(width), int(height))
            detector = str(getattr(self, "_last_detector_name", "") or "")
            bbox_status = dict(getattr(self, "_last_bbox_status", {}) or {})
            if (
                center is None
                or detector not in FOLLOW_SEARCH_STRONG_DETECTORS
                or not bool(bbox_status.get("bbox_human_shape_ok", False))
            ):
                return None
            confidence = max(0.05, min(1.0, float(getattr(self, "_last_detector_confidence", 0.0) or 0.0)))
            frame_quality = self._frame_quality_status(frame_rgb, int(width), int(height))
            lock_ok, lock_status = self._update_detection_lock(center, int(width), int(height))
            bbox_status.update(lock_status)
            if not lock_ok:
                self._last_result_status = {
                    "state": "candidate_unconfirmed",
                    "source": "camera",
                    "stale": False,
                    "target_visible": True,
                    "target_usable": False,
                    "frame_ok": True,
                    "age_s": None,
                    "confidence": float(confidence),
                    "rotation_deg": int(getattr(self, "_last_frame_rotation_deg", 0) or 0),
                    "detector": detector,
                    "detector_confidence": float(confidence),
                    "detector_error": str(getattr(self, "_last_detector_error", "") or ""),
                    "stream_seed": True,
                    "stream_seed_age_s": float(age_s),
                    **frame_quality,
                    **bbox_status,
                }
                return None
            self._refresh_template_lock(frame_rgb, detector=detector)
            self._last_center = (float(center[0]), float(center[1]), float(width))
            self._last_detection_ts = time.monotonic()
            self._last_usable_detector_name = str(detector)
            self._last_usable_detector_confidence = float(confidence)
            self._last_result_status = {
                "state": "ok",
                "source": "camera",
                "stale": False,
                "target_visible": True,
                "target_usable": True,
                "frame_ok": True,
                "age_s": 0.0,
                "confidence": float(confidence),
                "rotation_deg": int(getattr(self, "_last_frame_rotation_deg", 0) or 0),
                "detector": detector,
                "detector_confidence": float(confidence),
                "detector_error": str(getattr(self, "_last_detector_error", "") or ""),
                "stream_seed": True,
                "stream_seed_age_s": float(age_s),
                **frame_quality,
                **bbox_status,
            }
            return (float(center[0]), float(center[1]), float(width), float(confidence), dict(self._last_result_status))
        except Exception as e:
            self._last_detector_error = f"stream_seed_error:{e}"
            return None

    def _frame_quality_status(self, frame_rgb, width: int, height: int) -> Dict[str, Any]:
        try:
            mean_luma = float(frame_rgb.mean())
            contrast = float(frame_rgb.std())
        except Exception:
            mean_luma = 0.0
            contrast = 0.0
        return {
            "image_width_px": int(width),
            "image_height_px": int(height),
            "frame_luma_mean": float(mean_luma),
            "frame_luma_std": float(contrast),
            "frame_too_dark": bool(mean_luma < 18.0),
            "frame_too_bright": bool(mean_luma > 238.0),
            "frame_low_contrast": bool(contrast < 8.0),
        }

    def _remember_bbox(
        self,
        *,
        detector: str,
        bbox_width_px: Any,
        bbox_height_px: Any,
        image_width_px: Any,
        image_height_px: Any,
        bbox_x_px: Any = None,
        bbox_y_px: Any = None,
        center_x_px: Any = None,
        center_y_px: Any = None,
        bbox_area_ratio: Any = None,
        bbox_fill_ratio: Any = None,
        bbox_center_offset_ratio: Any = None,
        onnx_score: Any = None,
        onnx_objectness: Any = None,
        onnx_person_class_score: Any = None,
    ) -> None:
        x_px = _finite_nonnegative_float(bbox_x_px)
        y_px = _finite_nonnegative_float(bbox_y_px)
        width_px = _finite_positive_float(bbox_width_px)
        height_px = _finite_positive_float(bbox_height_px)
        image_width = _finite_positive_float(image_width_px)
        image_height = _finite_positive_float(image_height_px)
        center_x = _finite_nonnegative_float(center_x_px)
        center_y = _finite_nonnegative_float(center_y_px)
        if center_x is None and x_px is not None and width_px is not None:
            center_x = float(x_px) + (float(width_px) / 2.0)
        if center_y is None and y_px is not None and height_px is not None:
            center_y = float(y_px) + (float(height_px) / 2.0)
        status: Dict[str, Any] = {
            "bbox_x_px": None if x_px is None else float(x_px),
            "bbox_y_px": None if y_px is None else float(y_px),
            "bbox_width_px": None if width_px is None else float(width_px),
            "bbox_height_px": None if height_px is None else float(height_px),
            "bbox_area_ratio": None if bbox_area_ratio is None else float(bbox_area_ratio),
            "bbox_fill_ratio": None if bbox_fill_ratio is None else float(bbox_fill_ratio),
            "target_center_x_px": None if center_x is None else float(center_x),
            "target_center_y_px": None if center_y is None else float(center_y),
        }
        if width_px is not None and image_width is not None:
            status["bbox_width_ratio"] = float(width_px) / max(1.0, float(image_width))
        if height_px is not None and image_height is not None:
            status["bbox_height_ratio"] = float(height_px) / max(1.0, float(image_height))
        if x_px is not None and image_width is not None:
            status["bbox_x_ratio"] = float(x_px) / max(1.0, float(image_width))
        if y_px is not None and image_height is not None:
            status["bbox_y_ratio"] = float(y_px) / max(1.0, float(image_height))
        if center_x is not None and image_width is not None:
            status["target_center_x_ratio"] = float(center_x) / max(1.0, float(image_width))
        if center_y is not None and image_height is not None:
            status["target_center_y_ratio"] = float(center_y) / max(1.0, float(image_height))
        status.update(
            _camera_human_bbox_shape_status(
                detector=detector,
                bbox_width_px=width_px,
                bbox_height_px=height_px,
                image_width_px=image_width,
                image_height_px=image_height,
                bbox_area_ratio=bbox_area_ratio,
                bbox_fill_ratio=bbox_fill_ratio,
                bbox_center_offset_ratio=bbox_center_offset_ratio,
                onnx_score=onnx_score,
                onnx_objectness=onnx_objectness,
                onnx_person_class_score=onnx_person_class_score,
            )
        )
        status.update(
            _camera_distance_from_bbox(
                detector=detector,
                bbox_width_px=width_px,
                bbox_height_px=height_px,
                image_width_px=image_width,
                image_height_px=image_height,
                detector_confidence=getattr(self, "_last_detector_confidence", 0.0),
                bbox_area_ratio=bbox_area_ratio,
            )
        )
        self._last_bbox_status = status

    def _mark_strong_human_detector(self, detector: str) -> None:
        self._last_human_detector_ts = time.monotonic()
        self._last_human_detector_name = str(detector or "")

    def _recent_strong_human_detector(self) -> bool:
        last_ts = float(getattr(self, "_last_human_detector_ts", 0.0) or 0.0)
        if last_ts <= 0.0:
            return False
        return bool(time.monotonic() - last_ts <= CAMERA_MOTION_BLOB_RECENT_HUMAN_S)

    def _ensure_pose_detector(self) -> bool:
        if self._pose_detector_unavailable:
            return False
        if self._pose_detector is not None:
            return True
        try:
            import mediapipe as mp
            mp_pose = mp.solutions.pose
            self._pose_detector = mp_pose.Pose(
                static_image_mode=False,
                model_complexity=0,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.4,
            )
            return True
        except ImportError as e:
            self._pose_detector_unavailable = True
            self._last_detector_error = str(e)
            return False
        except Exception as e:
            self._last_detector_error = str(e)
            return False

    def _detect_pose_person_bbox(self, frame_rgb, width: int, height: int) -> Optional[Tuple[float, float]]:
        if not self._ensure_pose_detector():
            return None
        try:
            results = self._pose_detector.process(frame_rgb)
            if not results.pose_landmarks:
                return None
            xs = [lm.x * width for lm in results.pose_landmarks.landmark]
            ys = [lm.y * height for lm in results.pose_landmarks.landmark]
            cx = (min(xs) + max(xs)) / 2.0
            cy = (min(ys) + max(ys)) / 2.0
            self._last_detector_name = "mediapipe_pose"
            self._last_detector_confidence = 1.0
            self._mark_strong_human_detector("mediapipe_pose")
            self._remember_bbox(
                detector="mediapipe_pose",
                bbox_x_px=min(xs),
                bbox_y_px=min(ys),
                bbox_width_px=max(xs) - min(xs),
                bbox_height_px=max(ys) - min(ys),
                image_width_px=width,
                image_height_px=height,
                center_x_px=cx,
                center_y_px=cy,
            )
            return (cx, cy)
        except Exception as e:
            self._last_detector_error = str(e)
            return None

    def _ensure_onnx_person_detector(self) -> bool:
        if self._onnx_detector_unavailable:
            now = time.monotonic()
            last_unavailable = float(getattr(self, "_onnx_detector_unavailable_ts", 0.0) or 0.0)
            if now - last_unavailable < CAMERA_ONNX_RETRY_INTERVAL_S:
                return False
            self._onnx_detector_unavailable = False
        if self._onnx_session is not None:
            return True
        if not os.path.exists(CAMERA_ONNX_MODEL_PATH):
            self._last_detector_error = "onnx_model_missing"
            self._onnx_detector_unavailable = True
            self._onnx_detector_unavailable_ts = time.monotonic()
            return False
        try:
            import onnxruntime as ort

            opts = ort.SessionOptions()
            opts.intra_op_num_threads = 1
            opts.inter_op_num_threads = 1
            opts.log_severity_level = 3
            self._onnx_session = ort.InferenceSession(
                CAMERA_ONNX_MODEL_PATH,
                sess_options=opts,
                providers=["CPUExecutionProvider"],
            )
            input_meta = self._onnx_session.get_inputs()[0]
            self._onnx_input_name = str(input_meta.name)
            shape = list(input_meta.shape or [])
            if len(shape) >= 4:
                try:
                    self._onnx_input_size = int(shape[2] or CAMERA_ONNX_INPUT_SIZE)
                except Exception:
                    self._onnx_input_size = CAMERA_ONNX_INPUT_SIZE
            self._onnx_detector_unavailable = False
            self._onnx_detector_unavailable_ts = 0.0
            return True
        except Exception as e:
            self._last_detector_error = str(e)
            self._onnx_detector_unavailable = True
            self._onnx_detector_unavailable_ts = time.monotonic()
            return False

    def _onnx_letterbox(self, frame_rgb, input_size: int):
        import cv2
        import numpy as np

        src_h, src_w = frame_rgb.shape[:2]
        size = int(max(32, input_size))
        scale = min(float(size) / max(1.0, float(src_w)), float(size) / max(1.0, float(src_h)))
        new_w = max(1, int(round(float(src_w) * scale)))
        new_h = max(1, int(round(float(src_h) * scale)))
        resized = cv2.resize(frame_rgb, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((size, size, 3), 114, dtype=np.uint8)
        pad_x = int((size - new_w) // 2)
        pad_y = int((size - new_h) // 2)
        canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized
        blob = canvas.astype(np.float32) / 255.0
        blob = np.transpose(blob, (2, 0, 1))[None, ...]
        return blob, float(scale), float(pad_x), float(pad_y)

    def _detect_onnx_person_bbox(self, frame_rgb, width: int, height: int) -> Optional[Tuple[float, float]]:
        now = time.monotonic()
        if now - float(getattr(self, "_last_onnx_infer_ts", 0.0) or 0.0) < CAMERA_ONNX_MIN_INTERVAL_S:
            return None
        if not self._ensure_onnx_person_detector():
            return None
        try:
            import numpy as np

            input_size = int(getattr(self, "_onnx_input_size", CAMERA_ONNX_INPUT_SIZE) or CAMERA_ONNX_INPUT_SIZE)
            blob, scale, pad_x, pad_y = self._onnx_letterbox(frame_rgb, input_size)
            self._last_onnx_infer_ts = now
            outputs = self._onnx_session.run(None, {self._onnx_input_name: blob})
            pred = outputs[0]
            if pred is None:
                return None
            pred = np.asarray(pred)
            if pred.ndim == 3:
                pred = pred[0]
            best = None
            best_score = 0.0
            best_seen_rank = -1.0
            best_seen_diag: Dict[str, Any] = {
                "onnx_candidate_count": 0,
                "onnx_score_reject_count": 0,
                "onnx_shape_reject_count": 0,
            }
            recent_human_relock = bool(self._recent_strong_human_detector())
            for row in pred:
                if len(row) < 6:
                    continue
                obj = float(row[4])
                cls0 = float(row[5])
                score = obj * cls0
                cx_i, cy_i, w_i, h_i = [float(v) for v in row[:4]]
                x0 = (cx_i - (w_i / 2.0) - pad_x) / max(1e-6, scale)
                y0 = (cy_i - (h_i / 2.0) - pad_y) / max(1e-6, scale)
                box_w = w_i / max(1e-6, scale)
                box_h = h_i / max(1e-6, scale)
                center_offset_ratio = abs((x0 + (box_w / 2.0)) - (width / 2.0)) / max(1.0, float(width))
                candidate_rank = float(score) * (1.0 - min(0.45, center_offset_ratio))
                if candidate_rank > best_seen_rank:
                    best_seen_rank = candidate_rank
                    best_seen_diag.update(
                        {
                            "onnx_best_score": float(score),
                            "onnx_best_objectness": float(obj),
                            "onnx_best_person_class_score": float(cls0),
                            "onnx_best_center_offset_ratio": float(center_offset_ratio),
                            "onnx_best_box_width_ratio": float(box_w) / max(1.0, float(width)),
                            "onnx_best_box_height_ratio": float(box_h) / max(1.0, float(height)),
                            "onnx_best_reject_reason": "score_below_threshold",
                        }
                    )
                weak_person_candidate = bool(
                    score < CAMERA_ONNX_MIN_CONFIDENCE
                    and score >= CAMERA_ONNX_WEAK_MIN_SCORE
                    and obj >= CAMERA_ONNX_WEAK_MIN_OBJECTNESS
                    and cls0 >= CAMERA_ONNX_WEAK_MIN_PERSON_CLASS
                    and center_offset_ratio <= CAMERA_ONNX_WEAK_MAX_CENTER_OFFSET_RATIO
                )
                relock_person_candidate = bool(
                    score < CAMERA_ONNX_MIN_CONFIDENCE
                    and not bool(weak_person_candidate)
                    and bool(recent_human_relock)
                    and score >= CAMERA_ONNX_RELOCK_MIN_SCORE
                    and obj >= CAMERA_ONNX_RELOCK_MIN_OBJECTNESS
                    and cls0 >= CAMERA_ONNX_RELOCK_MIN_PERSON_CLASS
                    and center_offset_ratio <= CAMERA_ONNX_RELOCK_MAX_CENTER_OFFSET_RATIO
                )
                if score < CAMERA_ONNX_MIN_CONFIDENCE and not (weak_person_candidate or relock_person_candidate):
                    best_seen_diag["onnx_score_reject_count"] = int(best_seen_diag.get("onnx_score_reject_count") or 0) + 1
                    continue
                best_seen_diag["onnx_candidate_count"] = int(best_seen_diag.get("onnx_candidate_count") or 0) + 1
                x0 = max(0.0, min(float(width - 1), x0))
                y0 = max(0.0, min(float(height - 1), y0))
                x1 = max(x0 + 1.0, min(float(width), x0 + box_w))
                y1 = max(y0 + 1.0, min(float(height), y0 + box_h))
                box_w = x1 - x0
                box_h = y1 - y0
                shape = _camera_human_bbox_shape_status(
                    detector="onnx_yolov5_person",
                    bbox_width_px=box_w,
                    bbox_height_px=box_h,
                    image_width_px=width,
                    image_height_px=height,
                    bbox_area_ratio=(box_w * box_h) / max(1.0, float(width * height)),
                    bbox_center_offset_ratio=center_offset_ratio,
                    onnx_score=score,
                    onnx_objectness=obj,
                    onnx_person_class_score=cls0,
                )
                if not bool(shape.get("bbox_human_shape_ok", False)):
                    self._last_detector_error = str(shape.get("bbox_reject_reason") or "onnx_shape_rejected")
                    best_seen_diag["onnx_shape_reject_count"] = int(best_seen_diag.get("onnx_shape_reject_count") or 0) + 1
                    if candidate_rank >= best_seen_rank:
                        best_seen_diag["onnx_best_reject_reason"] = str(shape.get("bbox_reject_reason") or "onnx_shape_rejected")
                    continue
                center_bias = 1.0 - min(0.45, abs((x0 + (box_w / 2.0)) - (width / 2.0)) / max(1.0, width))
                ranked = float(score) * float(center_bias)
                if weak_person_candidate:
                    ranked += 0.006
                if relock_person_candidate:
                    ranked += 0.004
                if ranked > best_score:
                    best_score = ranked
                    best = (x0, y0, box_w, box_h, score, obj, cls0, weak_person_candidate, relock_person_candidate)
            if best is None:
                self._last_detector_error = "onnx_person_not_found"
                self._last_bbox_status = dict(best_seen_diag)
                return None
            x, y, w, h, score, obj, cls0, weak_person_candidate, relock_person_candidate = best
            cx = float(x + (w / 2.0))
            cy = float(y + (h / 2.0))
            self._last_detector_name = "onnx_yolov5_person"
            if weak_person_candidate:
                self._last_detector_confidence = max(0.24, min(0.55, 0.18 + (float(cls0) * 0.28) + (float(obj) * 0.50)))
                self._last_detector_error = "onnx_weak_person_candidate"
            elif relock_person_candidate:
                self._last_detector_confidence = max(0.22, min(0.48, 0.16 + (float(cls0) * 0.30) + (float(obj) * 0.45)))
                self._last_detector_error = "onnx_relock_person_candidate"
            else:
                self._last_detector_confidence = max(0.30, min(0.95, float(score)))
                self._last_detector_error = ""
            self._mark_strong_human_detector("onnx_yolov5_person")
            self._remember_bbox(
                detector="onnx_yolov5_person",
                bbox_x_px=x,
                bbox_y_px=y,
                bbox_width_px=w,
                bbox_height_px=h,
                image_width_px=width,
                image_height_px=height,
                center_x_px=cx,
                center_y_px=cy,
                bbox_area_ratio=(float(w) * float(h)) / max(1.0, float(width * height)),
                bbox_center_offset_ratio=abs(float(cx) - (float(width) / 2.0)) / max(1.0, float(width)),
                onnx_score=score,
                onnx_objectness=obj,
                onnx_person_class_score=cls0,
            )
            self._last_bbox_status["onnx_objectness"] = float(obj)
            self._last_bbox_status["onnx_person_class_score"] = float(cls0)
            self._last_bbox_status["onnx_weak_person_candidate"] = bool(weak_person_candidate)
            self._last_bbox_status["onnx_relock_person_candidate"] = bool(relock_person_candidate)
            return (cx, cy)
        except Exception as e:
            self._last_detector_error = str(e)
            return None

    def _detect_motion_blob(self, frame_rgb, width: int, height: int) -> Optional[Tuple[float, float]]:
        try:
            import cv2
            import numpy as np
            if len(frame_rgb.shape) == 3:
                gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
            else:
                gray = frame_rgb
            gray = cv2.GaussianBlur(gray, (5, 5), 0)
            prev = self._prev_motion_gray
            self._prev_motion_gray = gray
            if prev is None or getattr(prev, "shape", None) != getattr(gray, "shape", None):
                return None
            if not self._recent_strong_human_detector():
                self._last_detector_error = "motion_blob_requires_recent_human_confirmation"
                return None
            diff = cv2.absdiff(gray, prev)
            _, mask = cv2.threshold(diff, 18, 255, cv2.THRESH_BINARY)
            kernel = np.ones((5, 5), dtype=np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.dilate(mask, kernel, iterations=2)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            min_area = max(250.0, float(width * height) * 0.012)
            best = None
            best_score = 0.0
            for contour in contours:
                area = float(cv2.contourArea(contour))
                if area < min_area:
                    continue
                x, y, w, h = cv2.boundingRect(contour)
                if h < max(24, int(height * 0.16)) or w < max(10, int(width * 0.05)):
                    continue
                if area > float(width * height) * 0.80:
                    continue
                fill_ratio = area / max(1.0, float(w * h))
                shape = _camera_human_bbox_shape_status(
                    detector="opencv_motion_blob",
                    bbox_width_px=w,
                    bbox_height_px=h,
                    image_width_px=width,
                    image_height_px=height,
                    bbox_area_ratio=area / max(1.0, float(width * height)),
                    bbox_fill_ratio=fill_ratio,
                )
                if not bool(shape.get("bbox_human_shape_ok", False)):
                    self._last_detector_error = str(shape.get("bbox_reject_reason") or "motion_blob_shape_rejected")
                    continue
                cx = float(x + (w / 2.0))
                cy = float(y + (h / 2.0))
                center_bias = 1.0 - min(0.75, abs(cx - (width / 2.0)) / max(1.0, width))
                score = area * center_bias
                if score > best_score:
                    best_score = score
                    best = (cx, cy, area, w, h, x, y, fill_ratio)
            if best is None:
                return None
            area_ratio = max(0.0, min(1.0, float(best[2]) / max(1.0, float(width * height))))
            self._last_detector_name = "opencv_motion_blob"
            self._last_detector_confidence = max(0.30, min(0.75, 0.25 + (area_ratio * 4.0)))
            self._remember_bbox(
                detector="opencv_motion_blob",
                bbox_x_px=best[5],
                bbox_y_px=best[6],
                bbox_width_px=best[3],
                bbox_height_px=best[4],
                image_width_px=width,
                image_height_px=height,
                center_x_px=best[0],
                center_y_px=best[1],
                bbox_area_ratio=area_ratio,
                bbox_fill_ratio=best[7],
            )
            return (float(best[0]), float(best[1]))
        except Exception as e:
            self._last_detector_error = str(e)
            return None

    def _ensure_hog_detector(self):
        if self._hog_detector is not None:
            return self._hog_detector
        try:
            import cv2
            hog = cv2.HOGDescriptor()
            hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            self._hog_detector = hog
            return hog
        except Exception as e:
            self._last_detector_error = str(e)
            return None

    def _detect_hog_person_bbox(self, frame_rgb, width: int, height: int) -> Optional[Tuple[float, float]]:
        try:
            hog = self._ensure_hog_detector()
            if hog is None:
                return None
            rects, weights = hog.detectMultiScale(frame_rgb, winStride=(4, 4), padding=(8, 8), scale=1.05)
            if rects is None or len(rects) == 0:
                return None
            best = None
            best_score = -float("inf")
            for idx, rect in enumerate(rects):
                x, y, w, h = [float(v) for v in rect]
                weight = float(weights[idx]) if weights is not None and len(weights) > idx else 0.0
                if weight < CAMERA_HOG_MIN_WEIGHT:
                    self._last_detector_error = "hog_weight_too_low"
                    continue
                if h < max(48.0, float(height) * 0.28) or w < max(18.0, float(width) * 0.07):
                    continue
                shape = _camera_human_bbox_shape_status(
                    detector="opencv_hog",
                    bbox_width_px=w,
                    bbox_height_px=h,
                    image_width_px=width,
                    image_height_px=height,
                )
                if not bool(shape.get("bbox_human_shape_ok", False)):
                    self._last_detector_error = str(shape.get("bbox_reject_reason") or "hog_shape_rejected")
                    continue
                score = weight + min(0.5, (w * h) / max(1.0, float(width * height)))
                if score > best_score:
                    best_score = score
                    best = (x + (w / 2.0), y + (h / 2.0), weight, w, h, x, y)
            if best is None:
                return None
            self._last_detector_name = "opencv_hog"
            self._last_detector_confidence = max(0.25, min(0.85, 0.35 + (float(best[2]) * 0.25)))
            self._mark_strong_human_detector("opencv_hog")
            self._remember_bbox(
                detector="opencv_hog",
                bbox_x_px=best[5],
                bbox_y_px=best[6],
                bbox_width_px=best[3],
                bbox_height_px=best[4],
                image_width_px=width,
                image_height_px=height,
                center_x_px=best[0],
                center_y_px=best[1],
            )
            return (float(best[0]), float(best[1]))
        except Exception as e:
            self._last_detector_error = str(e)
            return None

    def _template_lock_allowed(self) -> bool:
        if self._template_lock_gray is None or self._template_lock_bbox is None:
            return False
        last_human_ts = float(getattr(self, "_last_human_detector_ts", 0.0) or 0.0)
        if last_human_ts <= 0.0:
            return False
        return bool(time.monotonic() - last_human_ts <= CAMERA_TEMPLATE_LOCK_RECENT_HUMAN_S)

    def _gray_for_template_lock(self, frame_rgb):
        import cv2
        if len(frame_rgb.shape) == 3:
            return cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
        return frame_rgb

    def _refresh_template_lock(self, frame_rgb, *, detector: str) -> None:
        status = dict(getattr(self, "_last_bbox_status", {}) or {})
        if not bool(status.get("bbox_human_shape_ok", False)):
            return
        det = str(detector or "")
        det_conf = float(max(0.0, min(1.0, getattr(self, "_last_detector_confidence", 0.0) or 0.0)))
        if det not in {"mediapipe_pose", "onnx_yolov5_person", "opencv_template_lock"}:
            return
        if det == "opencv_template_lock" and float(getattr(self, "_template_lock_score", 0.0) or 0.0) < CAMERA_TEMPLATE_LOCK_UPDATE_SCORE:
            return
        x = _finite_nonnegative_float(status.get("bbox_x_px"))
        y = _finite_nonnegative_float(status.get("bbox_y_px"))
        w = _finite_positive_float(status.get("bbox_width_px"))
        h = _finite_positive_float(status.get("bbox_height_px"))
        if x is None or y is None or w is None or h is None:
            return
        im_h, im_w = frame_rgb.shape[:2]
        x0 = int(max(0, min(int(round(x)), im_w - 1)))
        y0 = int(max(0, min(int(round(y)), im_h - 1)))
        x1 = int(max(x0 + 1, min(int(round(float(x) + float(w))), im_w)))
        y1 = int(max(y0 + 1, min(int(round(float(y) + float(h))), im_h)))
        if (x1 - x0) < max(12, int(im_w * 0.035)) or (y1 - y0) < max(24, int(im_h * 0.12)):
            return
        try:
            gray = self._gray_for_template_lock(frame_rgb)
            crop = gray[y0:y1, x0:x1]
            if crop is None or crop.size == 0:
                return
            if float(crop.std()) < CAMERA_TEMPLATE_LOCK_MIN_STD:
                return
            self._template_lock_gray = crop.copy()
            self._template_lock_bbox = (float(x0), float(y0), float(x1 - x0), float(y1 - y0), float(im_w), float(im_h))
            self._template_lock_ts = time.monotonic()
        except Exception as e:
            self._last_detector_error = str(e)

    def _detect_template_lock(self, frame_rgb, width: int, height: int) -> Optional[Tuple[float, float]]:
        if not self._template_lock_allowed():
            return None
        try:
            import cv2

            template = self._template_lock_gray
            bbox = self._template_lock_bbox
            if template is None or bbox is None:
                return None
            prev_x, prev_y, prev_w, prev_h, prev_im_w, prev_im_h = [float(v) for v in bbox]
            if abs(float(prev_im_w) - float(width)) > 1.0 or abs(float(prev_im_h) - float(height)) > 1.0:
                return None
            tpl_h, tpl_w = template.shape[:2]
            if tpl_w < 8 or tpl_h < 16 or tpl_w >= width or tpl_h >= height:
                return None
            if float(template.std()) < CAMERA_TEMPLATE_LOCK_MIN_STD:
                return None
            gray = self._gray_for_template_lock(frame_rgb)
            pad_x = max(float(tpl_w) * 0.85, float(width) * 0.12)
            pad_y = max(float(tpl_h) * 0.45, float(height) * 0.10)
            x0 = int(max(0, math.floor(prev_x - pad_x)))
            y0 = int(max(0, math.floor(prev_y - pad_y)))
            x1 = int(min(width, math.ceil(prev_x + prev_w + pad_x)))
            y1 = int(min(height, math.ceil(prev_y + prev_h + pad_y)))
            if (x1 - x0) < tpl_w or (y1 - y0) < tpl_h:
                return None
            search = gray[y0:y1, x0:x1]
            best_match = None
            for scale in CAMERA_TEMPLATE_LOCK_SCALE_FACTORS:
                scale_f = float(scale)
                if abs(scale_f - 1.0) <= 1e-6:
                    scaled_template = template
                else:
                    scaled_w = max(8, int(round(float(tpl_w) * scale_f)))
                    scaled_h = max(16, int(round(float(tpl_h) * scale_f)))
                    if scaled_w >= width or scaled_h >= height:
                        continue
                    scaled_template = cv2.resize(template, (scaled_w, scaled_h), interpolation=cv2.INTER_LINEAR)
                cur_h, cur_w = scaled_template.shape[:2]
                if cur_w < 8 or cur_h < 16 or cur_w > search.shape[1] or cur_h > search.shape[0]:
                    continue
                if float(scaled_template.std()) < CAMERA_TEMPLATE_LOCK_MIN_STD:
                    continue
                result = cv2.matchTemplate(search, scaled_template, cv2.TM_CCOEFF_NORMED)
                _min_val, max_val, _min_loc, max_loc = cv2.minMaxLoc(result)
                score = float(max_val)
                rank = score - min(0.04, abs(scale_f - 1.0) * 0.08)
                if best_match is None or rank > float(best_match[0]):
                    best_match = (float(rank), float(score), max_loc, cur_w, cur_h)
            if best_match is None:
                return None
            _rank, score, max_loc, match_w, match_h = best_match
            self._template_lock_score = float(score)
            if score < CAMERA_TEMPLATE_LOCK_MIN_SCORE:
                self._last_detector_error = "template_lock_score_too_low"
                return None
            match_x = float(x0 + max_loc[0])
            match_y = float(y0 + max_loc[1])
            shape = _camera_human_bbox_shape_status(
                detector="opencv_template_lock",
                bbox_width_px=match_w,
                bbox_height_px=match_h,
                image_width_px=width,
                image_height_px=height,
            )
            if not bool(shape.get("bbox_human_shape_ok", False)):
                self._last_detector_error = str(shape.get("bbox_reject_reason") or "template_lock_shape_rejected")
                return None
            cx = match_x + (float(match_w) / 2.0)
            cy = match_y + (float(match_h) / 2.0)
            self._last_detector_name = "opencv_template_lock"
            self._last_detector_confidence = max(0.35, min(0.82, 0.28 + (score * 0.52)))
            self._remember_bbox(
                detector="opencv_template_lock",
                bbox_x_px=match_x,
                bbox_y_px=match_y,
                bbox_width_px=match_w,
                bbox_height_px=match_h,
                image_width_px=width,
                image_height_px=height,
                center_x_px=cx,
                center_y_px=cy,
            )
            return (float(cx), float(cy))
        except Exception as e:
            self._last_detector_error = str(e)
            return None

    def _detect_person_bbox(self, frame_rgb, width: int, height: int) -> Optional[Tuple[float, float]]:
        start = time.monotonic()
        self._last_detector_name = "none"
        self._last_detector_confidence = 0.0
        self._last_detector_error = ""
        self._last_bbox_status = {}
        try:
            now = time.monotonic()
            strong_age_s = now - float(getattr(self, "_last_human_detector_ts", 0.0) or 0.0)
            if (
                bool(getattr(self, "_lock_active", False))
                and self._template_lock_allowed()
                and strong_age_s < CAMERA_STRONG_REVALIDATE_INTERVAL_S
            ):
                center = self._detect_template_lock(frame_rgb, width, height)
                if center is not None:
                    return center
            center = self._detect_onnx_person_bbox(frame_rgb, width, height)
            if center is not None:
                return center
            center = self._detect_pose_person_bbox(frame_rgb, width, height)
            if center is not None:
                return center
            center = self._detect_template_lock(frame_rgb, width, height)
            if center is not None:
                return center
            center = self._detect_motion_blob(frame_rgb, width, height)
            if center is not None:
                return center
            if CAMERA_HOG_FALLBACK_ENABLED:
                return self._detect_hog_person_bbox(frame_rgb, width, height)
            return None
        finally:
            self._last_detector_latency_ms = max(0.0, (time.monotonic() - start) * 1000.0)

    def _frame_from_capture_array(self, arr) -> Optional[Tuple]:
        if arr is None or arr.size == 0:
            return None
        rotation_deg = _camera_rotation_deg()
        self._last_frame_rotation_deg = int(rotation_deg)
        arr = _rotate_image(arr, rotation_deg)
        h, w = arr.shape[:2]
        if len(arr.shape) == 2:
            import numpy as np

            frame_rgb = np.stack([arr] * 3, axis=-1)
        else:
            frame_rgb = arr
            if arr.shape[2] == 4:
                frame_rgb = frame_rgb[:, :, :3]
        return (frame_rgb, w, h)

    def capture_frame(self, ctrl) -> Optional[Tuple]:
        """Egy frame (RGB numpy), width, height. None ha nem sikerült."""
        if self._camera is None:
            if not self.ensure_open(ctrl):
                return None
        now = time.monotonic()
        if now - float(getattr(self, "_last_capture_request_ts", 0.0) or 0.0) < CAMERA_CAPTURE_REQUEST_MIN_INTERVAL_S:
            self._last_capture_status = "capture_throttled"
            return None
        self._last_capture_request_ts = now
        try:
            try:
                arr = self._camera.capture_array(wait=CAMERA_CAPTURE_WAIT_TIMEOUT_S)
            except TypeError:
                arr = self._camera.capture_array()
            self._last_capture_status = "frame_ready"
            return self._frame_from_capture_array(arr)
        except FutureTimeoutError:
            self._last_capture_status = "capture_timeout"
            self._last_detector_error = "capture_timeout"
            try:
                if self._camera is not None and hasattr(self._camera, "cancel_all_and_flush"):
                    self._camera.cancel_all_and_flush()
            except Exception:
                pass
            return None
        except Exception as e:
            if hasattr(ctrl, "logger"):
                ctrl.logger.warn(f"[FOLLOW] Frame hiba: {e}")
            self._last_capture_status = "capture_error"
            self._last_detector_error = str(e)
            safe_stop_close(self._camera)
            self._camera = None
            return None

    def _publish_gui_preview_frame(self, ctrl, frame_rgb, status: Dict[str, Any]) -> None:
        """Follow-owned low-rate GUI preview writer; does not open another camera."""
        now = time.monotonic()
        last = float(getattr(self, "_last_gui_preview_write_ts", 0.0) or 0.0)
        if now - last < FOLLOW_GUI_PREVIEW_INTERVAL_S:
            return
        runtime_dir = os.path.dirname(str(getattr(ctrl, "status_path", "") or ""))
        if not runtime_dir:
            return
        stream_path = os.path.join(runtime_dir, "stream_frame.jpg")
        tmp_path = stream_path + ".follow.tmp"
        try:
            from PIL import Image

            img = Image.fromarray(frame_rgb, mode="RGB")
            if img.size != FOLLOW_GUI_PREVIEW_SIZE:
                try:
                    resample = Image.Resampling.BILINEAR
                except AttributeError:
                    resample = Image.BILINEAR
                img = img.resize(FOLLOW_GUI_PREVIEW_SIZE, resample)
            img.save(tmp_path, "JPEG", quality=int(FOLLOW_GUI_PREVIEW_JPEG_QUALITY), optimize=False)
            os.replace(tmp_path, stream_path)
            self._last_gui_preview_write_ts = now
            try:
                setattr(ctrl, "_adaptive_target_camera_preview_status", dict(status or {}))
            except Exception:
                pass
        except Exception:
            try:
                if os.path.exists(tmp_path):
                    os.unlink(tmp_path)
            except Exception:
                pass

    # ── target persistence ──

    def _hold_or_missing_status(
        self,
        *,
        reason: str,
        detector_throttled: bool = False,
        capture_pending: bool = False,
        frame_ok: bool = False,
    ) -> Optional[Tuple[float, float, float, float]]:
        persisted = self._persisted_center()
        age_s = self.target_age_s()
        recent_fresh_hold = bool(
            persisted is not None
            and age_s is not None
            and float(age_s) <= float(CAMERA_RECENT_HOLD_FRESH_MAX_AGE_S)
            and bool(getattr(self, "_lock_active", False))
        )
        zone = _camera_target_zone_from_center(
            None if persisted is None else persisted[0],
            None if persisted is None else persisted[2],
        )
        base = dict(self._last_result_status or {})
        detector = str(base.get("detector") or getattr(self, "_last_detector_name", "none") or "none")
        detector_confidence = float(base.get("detector_confidence", getattr(self, "_last_detector_confidence", 0.0)) or 0.0)
        if detector in {"", "none", "unknown"} and bool(persisted is not None):
            usable_detector = str(getattr(self, "_last_usable_detector_name", "") or "")
            if usable_detector:
                detector = usable_detector
                detector_confidence = float(getattr(self, "_last_usable_detector_confidence", detector_confidence) or detector_confidence)
        if recent_fresh_hold:
            state = "target_recent_hold"
        elif persisted is not None and str(reason or "") == "frame_missing":
            state = "frame_missing_persisted"
        elif persisted is not None:
            state = "target_persisted"
        else:
            state = str(reason or "frame_missing")
        self._last_result_status = {
            "state": state,
            "source": "camera",
            "stale": bool(not recent_fresh_hold and persisted is not None),
            "target_visible": bool(recent_fresh_hold),
            "target_usable": bool(persisted is not None),
            "frame_ok": bool(frame_ok),
            "age_s": age_s,
            "confidence": 0.0 if persisted is None else float(persisted[3]),
            "rotation_deg": int(getattr(self, "_last_frame_rotation_deg", 0) or 0),
            "detector": detector,
            "detector_confidence": detector_confidence,
            "detector_error": str(getattr(self, "_last_detector_error", "") or ""),
            "detector_throttled": bool(detector_throttled),
            "capture_pending": bool(capture_pending),
            "capture_status": str(getattr(self, "_last_capture_status", "") or ""),
            **self._lock_common_status(zone=zone, reason=str(reason or "")),
        }
        return persisted

    def detect_with_persistence(self, ctrl) -> Optional[Tuple[float, float, float, float]]:
        """
        Kamera frame + ember detekció, target persistence filterrel.
        Ha az aktuális frame-en nem lát embert de a timeout nem járt le,
        az utolsó ismert pozíciót adja vissza.
        Vissza: (center_x, center_y, image_width, confidence) vagy None.
        """
        now = time.monotonic()
        if now - float(getattr(self, "_last_detection_process_ts", 0.0) or 0.0) < CAMERA_DETECT_PROCESS_MIN_INTERVAL_S:
            held = self._hold_or_missing_status(reason="detector_throttled", detector_throttled=True)
            if held is not None:
                return held
        cap = self.capture_frame(ctrl)
        if cap is None:
            capture_status = str(getattr(self, "_last_capture_status", "") or "")
            missing_reason = capture_status if capture_status not in {"", "idle", "released"} else "frame_missing"
            persisted = self._hold_or_missing_status(
                reason=missing_reason,
                capture_pending=capture_status in {"capture_pending", "capture_throttled"},
            )
            open_failed = bool(getattr(self, "_last_open_failed", False))
            self._last_result_status["open_failed"] = bool(open_failed)
            self._last_result_status["failed_sessions"] = int(getattr(self, "_failed_sessions", 0) or 0)
            self._last_result_status["last_open_error"] = str(getattr(self, "_last_open_error", "") or "")
            if open_failed and persisted is None:
                self._last_result_status["state"] = "camera_open_failed"
            return persisted
        frame_rgb, im_w, im_h = cap
        self._last_detection_process_ts = time.monotonic()
        frame_quality = self._frame_quality_status(frame_rgb, im_w, im_h)
        center = self._detect_person_bbox(frame_rgb, im_w, im_h)
        if center is not None:
            detection_confidence = max(0.05, min(1.0, float(getattr(self, "_last_detector_confidence", 1.0) or 1.0)))
            bbox_status = dict(getattr(self, "_last_bbox_status", {}) or {})
            lock_ok, lock_status = self._update_detection_lock(center, int(im_w), int(im_h))
            bbox_status.update(lock_status)
            if not lock_ok:
                persisted = self._persisted_center()
                if persisted is not None:
                    age_s = self.target_age_s()
                    zone = _camera_target_zone_from_center(persisted[0], persisted[2])
                    self._last_result_status = {
                        "state": "target_persisted",
                        "source": "camera",
                        "stale": True,
                        "target_visible": False,
                        "target_usable": True,
                        "frame_ok": True,
                        "age_s": age_s,
                        "confidence": float(persisted[3]),
                        "rotation_deg": int(getattr(self, "_last_frame_rotation_deg", 0) or 0),
                        "detector": str(getattr(self, "_last_detector_name", "unknown") or "unknown"),
                        "detector_confidence": float(detection_confidence),
                        "detector_error": str(getattr(self, "_last_detector_error", "") or ""),
                        "candidate_rejected": True,
                        "candidate_reject_reason": str(lock_status.get("lock_reason") or ""),
                        **frame_quality,
                        **bbox_status,
                        **self._lock_common_status(zone=zone, reason="target_persisted_candidate_rejected"),
                    }
                    self._publish_gui_preview_frame(ctrl, frame_rgb, self._last_result_status)
                    return persisted
                self._last_result_status = {
                    "state": "candidate_unconfirmed",
                    "source": "camera",
                    "stale": False,
                    "target_visible": True,
                    "target_usable": False,
                    "frame_ok": True,
                    "age_s": self.target_age_s(),
                    "confidence": float(detection_confidence),
                    "rotation_deg": int(getattr(self, "_last_frame_rotation_deg", 0) or 0),
                    "detector": str(getattr(self, "_last_detector_name", "unknown") or "unknown"),
                    "detector_confidence": float(detection_confidence),
                    "detector_error": str(getattr(self, "_last_detector_error", "") or ""),
                    **frame_quality,
                    **bbox_status,
                }
                self._publish_gui_preview_frame(ctrl, frame_rgb, self._last_result_status)
                return None
            self._refresh_template_lock(frame_rgb, detector=str(getattr(self, "_last_detector_name", "") or ""))
            self._last_center = (center[0], center[1], float(im_w))
            self._last_detection_ts = time.monotonic()
            detector_name = str(getattr(self, "_last_detector_name", "unknown") or "unknown")
            if detector_name not in {"", "none", "unknown"}:
                self._last_usable_detector_name = detector_name
                self._last_usable_detector_confidence = float(detection_confidence)
            self._last_result_status = {
                "state": "ok",
                "source": "camera",
                "stale": False,
                "target_visible": True,
                "target_usable": True,
                "frame_ok": True,
                "age_s": 0.0,
                "confidence": float(detection_confidence),
                "rotation_deg": int(getattr(self, "_last_frame_rotation_deg", 0) or 0),
                "detector": detector_name,
                "detector_confidence": float(detection_confidence),
                "detector_error": str(getattr(self, "_last_detector_error", "") or ""),
                **frame_quality,
                **bbox_status,
            }
            if bool(lock_status.get("lock_new", False)):
                saved = self._save_lock_audit_image(frame_rgb, self._last_result_status)
                if saved:
                    self._last_result_status["lock_image_path"] = str(saved)
                    self._last_result_status["last_lock_image_path"] = str(saved)
            self._publish_gui_preview_frame(ctrl, frame_rgb, self._last_result_status)
            return (center[0], center[1], float(im_w), float(detection_confidence))
        persisted = self._persisted_center()
        age_s = self.target_age_s()
        bbox_status = dict(getattr(self, "_last_bbox_status", {}) or {})
        zone = _camera_target_zone_from_center(
            None if persisted is None else persisted[0],
            None if persisted is None else persisted[2],
        )
        self._last_result_status = {
            "state": "target_persisted" if persisted is not None else "target_stale",
            "source": "camera",
            "stale": True,
            "target_visible": False,
            "target_usable": bool(persisted is not None),
            "frame_ok": True,
            "age_s": age_s,
            "confidence": 0.0 if persisted is None else float(persisted[3]),
            "rotation_deg": int(getattr(self, "_last_frame_rotation_deg", 0) or 0),
            "detector": str(getattr(self, "_last_detector_name", "none") or "none"),
            "detector_confidence": float(getattr(self, "_last_detector_confidence", 0.0) or 0.0),
            "detector_error": str(getattr(self, "_last_detector_error", "") or ""),
            **frame_quality,
            **bbox_status,
            **self._lock_common_status(zone=zone, reason="target_persisted" if persisted is not None else "target_stale"),
        }
        self._publish_gui_preview_frame(ctrl, frame_rgb, self._last_result_status)
        return persisted

    def _persisted_center(self) -> Optional[Tuple[float, float, float, float]]:
        """Az utolsó ismert pozíció (cx, cy, im_w, conf), ha a timeout nem járt le."""
        if self._last_center is None:
            return None
        elapsed = time.monotonic() - self._last_detection_ts
        if elapsed < self._persistence_timeout_s:
            # Perzisztált cél: alacsonyabb bizalom (0.5)
            return (self._last_center[0], self._last_center[1], self._last_center[2], 0.5)
        # Timeout lejárt – cél elveszett
        self._last_center = None
        self._last_detection_ts = 0.0
        self._reset_lock_state(clear_history=True, reason="persistence_timeout")
        return None


def _wrap_angle_deg(angle_deg: float) -> float:
    """[-180, 180) tartományba normalizálás."""
    return ((float(angle_deg) + 180.0) % 360.0) - 180.0


class TargetKinematicsTracker:
    """
    Könnyű célkövető: mérés simítás + sebességbecslés (vx, vy) robot-koordinátában.
    Szándékosan kicsi és determinisztikus, hogy ne hozzon be új rejtett útvonalat.
    """

    def __init__(self, max_speed_mps: float = 0.85):
        self._dist_est: Optional[float] = None
        self._angle_est: Optional[float] = None
        self._last_xy: Optional[Tuple[float, float]] = None
        self._last_t_mono: Optional[float] = None
        self._last_seen_ts: Optional[float] = None
        self._confidence: float = 0.0
        self._vx: Optional[float] = None
        self._vy: Optional[float] = None
        self._max_speed_mps = float(max(0.1, max_speed_mps))
        self._last_update_limited: bool = False

    def clear(self) -> None:
        self._dist_est = None
        self._angle_est = None
        self._last_xy = None
        self._last_t_mono = None
        self._last_seen_ts = None
        self._confidence = 0.0
        self._vx = None
        self._vy = None
        self._last_update_limited = False

    def snapshot(self) -> Optional[dict]:
        if self._dist_est is None or self._angle_est is None or self._last_t_mono is None:
            return None
        return {
            "dist_m": float(self._dist_est),
            "angle_deg": float(self._angle_est),
            "confidence": float(self._confidence),
            "vx": self._vx,
            "vy": self._vy,
            "last_seen_ts": float(self._last_seen_ts or time.time()),
            "measurement_limited": bool(self._last_update_limited),
        }

    def observe(
        self,
        dist_m: float,
        angle_deg: float,
        confidence: float = 1.0,
        *,
        force_distance: bool = False,
        zero_velocity: bool = False,
    ) -> dict:
        now_mono = time.monotonic()
        now_wall = time.time()
        d_meas = max(0.0, float(dist_m))
        a_meas = _wrap_angle_deg(float(angle_deg))
        conf = max(0.0, min(1.0, float(confidence)))
        update_limited = False

        if self._dist_est is None or self._angle_est is None or self._last_t_mono is None:
            self._dist_est = d_meas
            self._angle_est = a_meas
        else:
            dt = max(1e-3, now_mono - self._last_t_mono)
            if bool(force_distance):
                self._dist_est = d_meas
            else:
                conf_step_scale = 0.45 + (0.55 * conf)
                max_distance_step_m = max(0.06, min(0.45, ((self._max_speed_mps * dt) + 0.06) * conf_step_scale))
                dist_delta = d_meas - float(self._dist_est)
                if dist_delta < 0.0:
                    max_distance_step_m *= 0.65 if conf < 0.70 else 0.85
                if abs(dist_delta) > max_distance_step_m:
                    d_meas = float(self._dist_est) + math.copysign(max_distance_step_m, dist_delta)
                    update_limited = True

            angle_err_raw = _wrap_angle_deg(a_meas - float(self._angle_est))
            max_angle_step_deg = max(12.0, min(70.0, (140.0 * dt) + 8.0))
            if abs(angle_err_raw) > max_angle_step_deg:
                a_meas = _wrap_angle_deg(float(self._angle_est) + math.copysign(max_angle_step_deg, angle_err_raw))
                update_limited = True

            dt_scale = max(0.35, min(1.0, dt / 0.35))
            alpha_dist = max(0.05, min(0.28, dt_scale * (0.08 + (0.18 * conf))))
            if not bool(force_distance):
                if (d_meas - self._dist_est) < 0.0 and conf < 0.65:
                    alpha_dist *= 0.55
                self._dist_est = self._dist_est + alpha_dist * (d_meas - self._dist_est)
            alpha_angle = max(0.18, min(0.72, dt_scale * (0.24 + (0.45 * conf))))
            angle_err = _wrap_angle_deg(a_meas - self._angle_est)
            self._angle_est = _wrap_angle_deg(self._angle_est + alpha_angle * angle_err)

        angle_rad = math.radians(self._angle_est)
        x = self._dist_est * math.cos(angle_rad)
        y = self._dist_est * math.sin(angle_rad)

        if self._last_xy is not None and self._last_t_mono is not None:
            dt = max(1e-3, now_mono - self._last_t_mono)
            vx_raw = (x - self._last_xy[0]) / dt
            vy_raw = (y - self._last_xy[1]) / dt
            speed = math.hypot(vx_raw, vy_raw)
            if speed > self._max_speed_mps:
                scale = self._max_speed_mps / speed
                vx_raw *= scale
                vy_raw *= scale
            beta = max(0.18, min(0.55, dt / 0.35))
            if self._vx is None or self._vy is None:
                self._vx = vx_raw
                self._vy = vy_raw
            else:
                self._vx = (1.0 - beta) * self._vx + beta * vx_raw
                self._vy = (1.0 - beta) * self._vy + beta * vy_raw
        if bool(zero_velocity):
            self._vx = 0.0
            self._vy = 0.0

        self._last_xy = (x, y)
        self._last_t_mono = now_mono
        self._last_seen_ts = now_wall
        self._confidence = conf
        self._last_update_limited = bool(update_limited)
        return {
            "dist_m": float(self._dist_est),
            "angle_deg": float(self._angle_est),
            "confidence": conf,
            "vx": self._vx,
            "vy": self._vy,
            "last_seen_ts": now_wall,
            "measurement_limited": bool(update_limited),
        }


def _get_follower_camera(ctrl) -> FollowerCamera:
    """Lazy init: ctrl.follower_camera példány."""
    cam = getattr(ctrl, "follower_camera", None)
    if cam is None:
        cam = FollowerCamera()
        ctrl.follower_camera = cam
    return cam


def _get_target_tracker(ctrl) -> TargetKinematicsTracker:
    tracker = getattr(ctrl, "follower_target_tracker", None)
    if tracker is None:
        tracker = TargetKinematicsTracker()
        ctrl.follower_target_tracker = tracker
    return tracker


def _prime_follow_target_from_stream_seed(ctrl, fcam: FollowerCamera, params: Dict[str, Any]) -> bool:
    seed = fcam.prime_from_stream_frame(ctrl)
    if seed is None:
        return False
    center_x, _center_y, image_width, confidence, camera_status = seed
    camera_status = dict(camera_status or {})
    angle_deg, angle_status = _bbox_center_to_angle_status(
        center_x,
        image_width,
        image_height=camera_status.get("image_height_px"),
        rotation_deg=camera_status.get("rotation_deg"),
    )
    camera_status.update(angle_status)

    desired_distance_m = float(params["target_distance_m"])
    distance_m, distance_confidence = _camera_distance_status_value(camera_status)
    if distance_m is None:
        distance_m = desired_distance_m
        camera_status["distance_source"] = "stream_seed_desired_distance_fallback"
        camera_status["distance_confidence"] = 0.0
    else:
        camera_status["distance_source"] = "camera_bbox"
    camera_status["distance_used_m"] = float(distance_m)
    tracker_confidence = max(0.10, min(1.0, float(confidence) * max(0.45, float(distance_confidence))))
    tracked = _get_target_tracker(ctrl).observe(
        dist_m=float(distance_m),
        angle_deg=float(angle_deg),
        confidence=float(tracker_confidence),
        force_distance=True,
        zero_velocity=True,
    )
    now_wall = time.time()
    tracked.setdefault("last_seen_ts", now_wall)
    lidar_status = {
        "state": "not_evaluated",
        "source": "stream_seed_no_lidar",
        "usable_distance": False,
        "distance_m": None,
        "confidence": 0.0,
        "point_count": 0,
        "cluster_points": 0,
        "age_s": None,
    }
    try:
        import robot_state
        robot_state.set_tracked_target(
            tracked["dist_m"],
            tracked["angle_deg"],
            confidence=tracked["confidence"],
            vx=tracked.get("vx"),
            vy=tracked.get("vy"),
            last_seen_ts=tracked.get("last_seen_ts", now_wall),
        )
    except Exception:
        pass
    setattr(ctrl, "_adaptive_target_search_active", False)
    setattr(ctrl, "_adaptive_target_dist_m", tracked["dist_m"])
    setattr(ctrl, "_adaptive_target_angle_deg", tracked["angle_deg"])
    setattr(ctrl, "_adaptive_target_confidence", tracked.get("confidence"))
    setattr(ctrl, "_adaptive_target_vx_mps", tracked.get("vx"))
    setattr(ctrl, "_adaptive_target_vy_mps", tracked.get("vy"))
    setattr(ctrl, "_adaptive_target_last_seen_ts", tracked.get("last_seen_ts"))
    setattr(ctrl, "_adaptive_target_desired_distance_m", desired_distance_m)
    setattr(ctrl, "_adaptive_target_lidar_source", "stream_seed_no_lidar")
    setattr(ctrl, "_adaptive_target_lidar_confidence", 0.0)
    setattr(ctrl, "_adaptive_target_lidar_distance_m", None)
    setattr(ctrl, "_adaptive_target_lidar_points", 0)
    setattr(ctrl, "_adaptive_target_lidar_cluster_points", 0)
    setattr(ctrl, "_adaptive_target_lidar_age_s", None)
    setattr(ctrl, "_adaptive_target_lidar_status", dict(lidar_status))
    setattr(ctrl, "_adaptive_target_camera_status", dict(camera_status))
    setattr(ctrl, "_adaptive_follow_state", "ok")
    try:
        from controller.status import append_camera_log
        append_camera_log(
            ctrl,
            "follow_stream_seed_target",
            detector=str(camera_status.get("detector") or ""),
            confidence=float(camera_status.get("detector_confidence") or 0.0),
            distance_m=float(tracked["dist_m"]),
            angle_deg=float(tracked["angle_deg"]),
            seed_age_s=float(camera_status.get("stream_seed_age_s") or 0.0),
        )
    except Exception:
        pass
    return True


def _wait_for_follow_stream_seed_frame(ctrl, *, timeout_s: float = FOLLOW_START_STREAM_SEED_WAIT_S) -> bool:
    runtime_dir = os.path.dirname(str(getattr(ctrl, "status_path", "") or ""))
    if not runtime_dir:
        return False
    camera_enabled = bool(getattr(ctrl, "_stream_writer_camera_active", False))
    try:
        from middleware.peripheral_usage import is_peripheral_enabled
        camera_enabled = camera_enabled or bool(
            is_peripheral_enabled("camera", status_path=getattr(ctrl, "status_path", None), default=False)
        )
    except Exception:
        pass
    if not camera_enabled:
        return False
    stream_path = os.path.join(runtime_dir, "stream_frame.jpg")
    deadline = time.monotonic() + max(0.0, float(timeout_s))
    while True:
        try:
            age_s = time.time() - float(os.path.getmtime(stream_path))
            if age_s <= FOLLOW_START_STREAM_SEED_MAX_AGE_S:
                return True
        except Exception:
            pass
        if time.monotonic() >= deadline:
            return False
        time.sleep(0.05)


def _dynamic_front_hold_distance_m(target_distance_m: float) -> float:
    desired = max(0.0, float(target_distance_m))
    margin_m = 0.18 if desired <= CAMERA_CLOSE_BUBBLE_LIDAR_DESIRED_MAX_M else 0.10
    return max(
        LIDAR_EMERGENCY_DIST_M + 0.08,
        min(LIDAR_FOLLOW_HOLD_DIST_M, desired + margin_m),
    )


def _dynamic_camera_obstacle_min_target_distance_m(target_distance_m: float) -> float:
    desired = max(0.0, float(target_distance_m))
    return max(0.80, min(CAMERA_FRONT_HOLD_ARBITRATION_MIN_TARGET_M, desired + 0.35))


def _get_target_obstacle_arbiter(
    ctrl,
    *,
    front_hold_distance_m: Optional[float] = None,
    camera_obstacle_min_target_distance_m: Optional[float] = None,
) -> TargetObstacleArbiter:
    front_hold = (
        LIDAR_FOLLOW_HOLD_DIST_M
        if front_hold_distance_m is None
        else max(LIDAR_EMERGENCY_DIST_M + 0.02, min(LIDAR_FOLLOW_HOLD_DIST_M, float(front_hold_distance_m)))
    )
    min_target = (
        CAMERA_FRONT_HOLD_ARBITRATION_MIN_TARGET_M
        if camera_obstacle_min_target_distance_m is None
        else max(0.30, min(CAMERA_FRONT_HOLD_ARBITRATION_MIN_TARGET_M, float(camera_obstacle_min_target_distance_m)))
    )
    arbiter = getattr(ctrl, "target_obstacle_arbiter", None)
    if (
        arbiter is None
        or abs(float(getattr(arbiter.cfg, "front_hold_distance_m", 0.0)) - float(front_hold)) > 1e-6
        or abs(float(getattr(arbiter.cfg, "camera_obstacle_min_target_distance_m", 0.0)) - float(min_target)) > 1e-6
    ):
        arbiter = TargetObstacleArbiter(
            TargetObstacleArbiterConfig(
                front_hold_distance_m=float(front_hold),
                camera_obstacle_min_confidence=CAMERA_FRONT_HOLD_ARBITRATION_MIN_CONFIDENCE,
                camera_obstacle_min_delta_m=CAMERA_FRONT_HOLD_ARBITRATION_MIN_DELTA_M,
                camera_obstacle_min_target_distance_m=float(min_target),
            )
        )
        ctrl.target_obstacle_arbiter = arbiter
    return arbiter


def _detect_for_follow_tick(fcam, ctrl) -> Optional[Tuple[float, float, float, float]]:
    latest = getattr(fcam, "detect_latest", None)
    if callable(latest):
        return latest(ctrl)
    return fcam.detect_with_persistence(ctrl)


def tick(ctrl, lidar_snapshot) -> Tuple[float, float]:
    """
    Egy követési ciklus. Csak akkor fut, ha ctrl.following_active.
    LIDAR < 0.5 m bármely irányban → emergency_stop és követés leáll.
    Vissza: legacy semleges (0.0, 0.0); a mozgást a FOLLOW/CRUISE lánc adja.
    """
    if not getattr(ctrl, "following_active", False):
        return (0.0, 0.0)

    fcam = _get_follower_camera(ctrl)
    tracker = _get_target_tracker(ctrl)
    params = _follower_params(ctrl)
    front_hold_distance_m = _dynamic_front_hold_distance_m(float(params["target_distance_m"]))
    camera_obstacle_min_target_m = _dynamic_camera_obstacle_min_target_distance_m(float(params["target_distance_m"]))

    # 1) Biztonság: front LIDAR 0.5 m belül → vészleállítás és követés megszakad.
    # Oldal/háttér akadályokra a globális safety/local-planner réteg marad az SSOT.
    min_dist = _get_lidar_min_dist_front(lidar_snapshot)
    if min_dist < LIDAR_EMERGENCY_DIST_M:
        if hasattr(ctrl, "logger"):
            ctrl.logger.warn(f"[FOLLOW] Front LIDAR akadály {min_dist:.2f} m – vészleállítás.")
        ctrl._emergency_stop(reason="FOLLOW_LIDAR_OBSTACLE")
        ctrl.following_active = False
        stop_following(ctrl)
        return (0.0, 0.0)
    if min_dist < front_hold_distance_m:
        hold_dist_m = float(params["target_distance_m"])
        now_wall = time.time()
        previous_target = tracker.snapshot()
        center = _detect_for_follow_tick(fcam, ctrl)
        camera_status = fcam.last_status()
        if center is None and fcam.too_many_failures:
            if hasattr(ctrl, "logger"):
                ctrl.logger.warn("[FOLLOW] Kamera nem elérhető front-hold közben – követés kikapcsolva.")
            ctrl.following_active = False
            stop_following(ctrl)
            return (0.0, 0.0)
        if center is None:
            if str((camera_status or {}).get("state") or "") in {"candidate_unconfirmed", "candidate_hold"}:
                loss_status = _target_loss_status_for_previous_target(camera_status, previous_target)
                arbiter_decision = _get_target_obstacle_arbiter(
                    ctrl,
                    front_hold_distance_m=front_hold_distance_m,
                    camera_obstacle_min_target_distance_m=camera_obstacle_min_target_m,
                ).decide_target_loss(
                    camera_status=loss_status,
                    previous_target=previous_target,
                    desired_distance_m=hold_dist_m,
                )
                if arbiter_decision.allow_follow_target and previous_target is not None:
                    return _publish_target_persistence_hold(
                        ctrl,
                        lidar_snapshot,
                        decision=arbiter_decision,
                        previous_target=previous_target,
                        camera_status=loss_status,
                        params=params,
                    )
                tracker.clear()
                return _publish_follow_uncertain_hold(
                    ctrl,
                    lidar_snapshot,
                    camera_status,
                    reason="front_hold_candidate_unconfirmed",
                )
            if _candidate_confirm_hold_active(ctrl, camera_status):
                tracker.clear()
                return _publish_follow_uncertain_hold(
                    ctrl,
                    lidar_snapshot,
                    camera_status,
                    reason="candidate_confirm_wait",
                )
            arbiter_decision = _get_target_obstacle_arbiter(
                ctrl,
                front_hold_distance_m=front_hold_distance_m,
                camera_obstacle_min_target_distance_m=camera_obstacle_min_target_m,
            ).decide_target_loss(
                camera_status=dict(camera_status or {}),
                previous_target=previous_target,
                desired_distance_m=hold_dist_m,
            )
            if arbiter_decision.allow_follow_target and previous_target is not None:
                return _publish_target_persistence_hold(
                    ctrl,
                    lidar_snapshot,
                    decision=arbiter_decision,
                    previous_target=previous_target,
                    camera_status=camera_status,
                    params=params,
                )
            if _candidate_confirm_hold_active(ctrl, camera_status):
                tracker.clear()
                return _publish_follow_uncertain_hold(
                    ctrl,
                    lidar_snapshot,
                    camera_status,
                    reason="candidate_confirm_wait",
                )
            tracker.clear()
            return _publish_follow_target_search(
                ctrl,
                lidar_snapshot,
                camera_status,
                reason="front_hold_camera_target_lost",
            )
        if bool((camera_status or {}).get("stale", False)):
            arbiter_decision = _get_target_obstacle_arbiter(
                ctrl,
                front_hold_distance_m=front_hold_distance_m,
                camera_obstacle_min_target_distance_m=camera_obstacle_min_target_m,
            ).decide_target_loss(
                camera_status=dict(camera_status or {}),
                previous_target=previous_target,
                desired_distance_m=hold_dist_m,
            )
            if arbiter_decision.allow_follow_target and previous_target is not None:
                return _publish_target_persistence_hold(
                    ctrl,
                    lidar_snapshot,
                    decision=arbiter_decision,
                    previous_target=previous_target,
                    camera_status=camera_status,
                    params=params,
                )
            if _candidate_confirm_hold_active(ctrl, camera_status):
                tracker.clear()
                return _publish_follow_uncertain_hold(
                    ctrl,
                    lidar_snapshot,
                    camera_status,
                    reason="candidate_confirm_wait",
                )
            tracker.clear()
            return _publish_follow_target_search(
                ctrl,
                lidar_snapshot,
                camera_status,
                reason="front_hold_camera_target_stale",
            )
        if not _follow_search_target_confirmed(ctrl, camera_status):
            tracker.clear()
            return _publish_follow_target_search(
                ctrl,
                lidar_snapshot,
                camera_status,
                reason="front_hold_search_candidate_confirm",
            )
        _mark_follow_target_search_found(ctrl)

        cx, _cy, im_w, confidence_raw = center
        hold_angle_deg, angle_status = _bbox_center_to_angle_status(
            float(cx),
            float(im_w),
            image_height=(camera_status or {}).get("image_height_px"),
            rotation_deg=(camera_status or {}).get("rotation_deg"),
        )
        hold_confidence = max(0.05, min(1.0, float(confidence_raw)))
        camera_distance_m, camera_distance_confidence = _camera_distance_status_value(camera_status)
        camera_status = dict(camera_status or {})
        camera_status.update(angle_status)
        arbiter_decision = _get_target_obstacle_arbiter(
            ctrl,
            front_hold_distance_m=front_hold_distance_m,
            camera_obstacle_min_target_distance_m=camera_obstacle_min_target_m,
        ).decide_front_conflict(
            front_distance_m=float(min_dist),
            desired_distance_m=float(hold_dist_m),
            target_angle_deg=float(hold_angle_deg),
            target_confidence=float(hold_confidence),
            camera_status=dict(camera_status or {}),
            camera_distance_m=camera_distance_m,
            camera_distance_confidence=float(camera_distance_confidence),
            lidar_snapshot_age_s=_lidar_snapshot_age_s(lidar_snapshot),
            lidar_missing=lidar_snapshot is None,
        )
        if arbiter_decision.allow_follow_target and arbiter_decision.target_distance_m is not None:
            force_close_front_distance = bool(
                arbiter_decision.mode in {"front_hold", "front_target_confirmed"}
                or str((arbiter_decision.camera_updates or {}).get("distance_source") or "")
                == "front_lidar_close_bubble_camera_confirmed"
            )
            force_obstacle_split_distance = bool(arbiter_decision.mode == "front_obstacle_arbitrated")
            tracked = tracker.observe(
                float(arbiter_decision.target_distance_m),
                float(arbiter_decision.target_angle_deg or 0.0),
                confidence=float(arbiter_decision.target_confidence),
                force_distance=bool(force_close_front_distance or force_obstacle_split_distance),
                zero_velocity=bool(force_close_front_distance or force_obstacle_split_distance),
            )
            if force_close_front_distance:
                tracked["dist_m"] = float(arbiter_decision.target_distance_m)
                tracked["vx"] = 0.0
                tracked["vy"] = 0.0
            camera_status = dict(camera_status or {})
            camera_status.setdefault("source", "camera")
            camera_status.update(dict(arbiter_decision.camera_updates or {}))
            lidar_status = dict(arbiter_decision.lidar_status or {})
            follow_state = _adaptive_follow_state(
                camera_status=camera_status,
                lidar_status=lidar_status,
                dist_m=tracked["dist_m"],
                angle_deg=tracked["angle_deg"],
                params=params,
            ) if arbiter_decision.mode != "front_hold" else "front_lidar_hold"
            try:
                import robot_state
                robot_state.set_tracked_target(
                    tracked["dist_m"],
                    tracked["angle_deg"],
                    confidence=tracked["confidence"],
                    vx=tracked.get("vx"),
                    vy=tracked.get("vy"),
                    last_seen_ts=tracked.get("last_seen_ts", now_wall),
                )
            except Exception:
                pass
            setattr(ctrl, "_adaptive_target_dist_m", tracked["dist_m"])
            setattr(ctrl, "_adaptive_target_angle_deg", tracked["angle_deg"])
            setattr(ctrl, "_adaptive_target_confidence", tracked.get("confidence"))
            setattr(ctrl, "_adaptive_target_vx_mps", tracked.get("vx"))
            setattr(ctrl, "_adaptive_target_vy_mps", tracked.get("vy"))
            setattr(ctrl, "_adaptive_target_last_seen_ts", tracked.get("last_seen_ts", now_wall))
            setattr(ctrl, "_adaptive_target_desired_distance_m", hold_dist_m)
            setattr(ctrl, "_adaptive_target_lidar_source", str(lidar_status.get("source") or "lidar_missing"))
            setattr(ctrl, "_adaptive_target_lidar_confidence", float(lidar_status.get("confidence") or 0.0))
            setattr(ctrl, "_adaptive_target_lidar_distance_m", lidar_status.get("distance_m"))
            setattr(ctrl, "_adaptive_target_lidar_points", int(lidar_status.get("point_count") or 0))
            setattr(ctrl, "_adaptive_target_lidar_cluster_points", int(lidar_status.get("cluster_points") or 0))
            setattr(ctrl, "_adaptive_target_lidar_age_s", lidar_status.get("age_s"))
            setattr(ctrl, "_adaptive_target_lidar_status", dict(lidar_status))
            setattr(ctrl, "_adaptive_target_camera_status", dict(camera_status))
            setattr(ctrl, "_adaptive_follow_state", str(follow_state))
            return (0.0, 0.0)

    # 2) Kamera frame + ember detekció (target persistence filterrel)
    center = _detect_for_follow_tick(fcam, ctrl)
    camera_status = fcam.last_status()
    previous_target = tracker.snapshot()
    if center is None:
        if fcam.too_many_failures:
            if hasattr(ctrl, "logger"):
                ctrl.logger.warn("[FOLLOW] Kamera nem elérhető – követés kikapcsolva. Indítsd újra F-fel később.")
            ctrl.following_active = False
            stop_following(ctrl)
            return (0.0, 0.0)
        if str((camera_status or {}).get("state") or "") in {"candidate_unconfirmed", "candidate_hold"}:
            loss_status = _target_loss_status_for_previous_target(camera_status, previous_target)
            arbiter_decision = _get_target_obstacle_arbiter(
                ctrl,
                front_hold_distance_m=front_hold_distance_m,
                camera_obstacle_min_target_distance_m=camera_obstacle_min_target_m,
            ).decide_target_loss(
                camera_status=loss_status,
                previous_target=previous_target,
                desired_distance_m=float(params["target_distance_m"]),
            )
            if arbiter_decision.allow_follow_target and previous_target is not None:
                return _publish_target_persistence_hold(
                    ctrl,
                    lidar_snapshot,
                    decision=arbiter_decision,
                    previous_target=previous_target,
                    camera_status=loss_status,
                    params=params,
                )
            tracker.clear()
            if _weak_camera_candidate_should_continue_search(camera_status):
                return _publish_follow_target_search(
                    ctrl,
                    lidar_snapshot,
                    camera_status,
                    reason="weak_candidate_search_continue",
                )
            return _publish_follow_uncertain_hold(
                ctrl,
                lidar_snapshot,
                camera_status,
                reason="candidate_unconfirmed",
            )
        if _candidate_confirm_hold_active(ctrl, camera_status):
            tracker.clear()
            return _publish_follow_uncertain_hold(
                ctrl,
                lidar_snapshot,
                camera_status,
                reason="candidate_confirm_wait",
            )
        arbiter_decision = _get_target_obstacle_arbiter(
            ctrl,
            front_hold_distance_m=front_hold_distance_m,
            camera_obstacle_min_target_distance_m=camera_obstacle_min_target_m,
        ).decide_target_loss(
            camera_status=dict(camera_status or {}),
            previous_target=previous_target,
            desired_distance_m=float(params["target_distance_m"]),
        )
        if arbiter_decision.allow_follow_target and previous_target is not None:
            return _publish_target_persistence_hold(
                ctrl,
                lidar_snapshot,
                decision=arbiter_decision,
                previous_target=previous_target,
                camera_status=camera_status,
                params=params,
            )
        if _candidate_confirm_hold_active(ctrl, camera_status):
            tracker.clear()
            return _publish_follow_uncertain_hold(
                ctrl,
                lidar_snapshot,
                camera_status,
                reason="candidate_confirm_wait",
            )
        tracker.clear()
        return _publish_follow_target_search(
            ctrl,
            lidar_snapshot,
            camera_status,
            reason="camera_target_lost",
        )
    if bool((camera_status or {}).get("stale", False)):
        arbiter_decision = _get_target_obstacle_arbiter(
            ctrl,
            front_hold_distance_m=front_hold_distance_m,
            camera_obstacle_min_target_distance_m=camera_obstacle_min_target_m,
        ).decide_target_loss(
            camera_status=dict(camera_status or {}),
            previous_target=previous_target,
            desired_distance_m=float(params["target_distance_m"]),
        )
        if arbiter_decision.allow_follow_target and previous_target is not None:
            return _publish_target_persistence_hold(
                ctrl,
                lidar_snapshot,
                decision=arbiter_decision,
                previous_target=previous_target,
                camera_status=camera_status,
                params=params,
            )
        if _candidate_confirm_hold_active(ctrl, camera_status):
            tracker.clear()
            return _publish_follow_uncertain_hold(
                ctrl,
                lidar_snapshot,
                camera_status,
                reason="candidate_confirm_wait",
            )
        tracker.clear()
        return _publish_follow_target_search(
            ctrl,
            lidar_snapshot,
            camera_status,
            reason="camera_target_stale",
        )
    if not _follow_search_target_confirmed(ctrl, camera_status):
        tracker.clear()
        return _publish_follow_target_search(
            ctrl,
            lidar_snapshot,
            camera_status,
            reason="search_candidate_confirm",
        )
    _mark_follow_target_search_found(ctrl)

    center_x, _, im_w, confidence = center
    angle_deg, angle_status = _bbox_center_to_angle_status(
        center_x,
        im_w,
        image_height=(camera_status or {}).get("image_height_px"),
        rotation_deg=(camera_status or {}).get("rotation_deg"),
    )
    camera_status = dict(camera_status or {})
    camera_status.update(angle_status)

    expected_dist = None if previous_target is None else previous_target.get("dist_m")
    desired_distance_m = float(params["target_distance_m"])
    lidar_expected_dist = expected_dist
    lidar_window_deg = 12.0
    if desired_distance_m <= CAMERA_CLOSE_BUBBLE_LIDAR_DESIRED_MAX_M:
        lidar_window_deg = 18.0
        try:
            if expected_dist is not None and float(expected_dist) > desired_distance_m + 0.35:
                lidar_expected_dist = None
        except Exception:
            lidar_expected_dist = None
    lidar_measurement = _get_lidar_target_measurement_at_angle_deg(
        lidar_snapshot,
        angle_deg,
        window_deg=lidar_window_deg,
        expected_distance_m=lidar_expected_dist,
    )
    lidar_status = _target_lidar_status(lidar_snapshot, lidar_measurement)
    dist_m = lidar_status.get("distance_m") if bool(lidar_status.get("usable_distance", False)) else None
    camera_distance_m, camera_distance_confidence = _camera_distance_status_value(camera_status)
    camera_distance_active = bool(camera_distance_m is not None)
    if camera_distance_active:
        camera_status = dict(camera_status)
        camera_status["distance_used_m"] = float(camera_distance_m)
        camera_status["distance_source"] = "camera_bbox"
        lidar_dist_m = dist_m
        lidar_conf = float(lidar_status.get("confidence") or 0.0)
        if lidar_dist_m is not None:
            camera_status["lidar_distance_m"] = float(lidar_dist_m)
            camera_status["camera_lidar_delta_m"] = float(camera_distance_m) - float(lidar_dist_m)
        detector = str(camera_status.get("detector") or "")
        detector_confidence = _clamp_float(float(camera_status.get("detector_confidence") or 0.0), 0.0, 1.0)
        bbox_height_ratio = _finite_positive_float(camera_status.get("bbox_height_ratio")) or 0.0
        close_bubble_front_target_m = None
        close_bubble_front_target_source = "front_lidar_close_bubble_camera_confirmed"
        camera_front_delta_m = float(camera_distance_m) - float(min_dist)
        if (
            desired_distance_m <= CAMERA_CLOSE_BUBBLE_LIDAR_DESIRED_MAX_M
            and abs(float(angle_deg)) <= CAMERA_CLOSE_BUBBLE_LIDAR_MAX_BEARING_DEG
            and min_dist <= desired_distance_m + CAMERA_CLOSE_BUBBLE_LIDAR_FRONT_MARGIN_M
            and min_dist >= LIDAR_EMERGENCY_DIST_M
            and CAMERA_CLOSE_BUBBLE_LIDAR_MIN_CAMERA_DELTA_M
            <= camera_front_delta_m
            <= CAMERA_CLOSE_BUBBLE_LIDAR_MAX_CAMERA_DELTA_M
        ):
            close_bubble_front_target_m = float(min_dist)
        elif (
            desired_distance_m > CAMERA_CLOSE_BUBBLE_LIDAR_DESIRED_MAX_M
            and abs(float(angle_deg)) <= CAMERA_ROOM_BUBBLE_LIDAR_MAX_BEARING_DEG
            and min_dist <= desired_distance_m + CAMERA_ROOM_BUBBLE_LIDAR_FRONT_MARGIN_M
            and min_dist >= LIDAR_EMERGENCY_DIST_M
            and CAMERA_ROOM_BUBBLE_LIDAR_MIN_CAMERA_DELTA_M
            <= camera_front_delta_m
            <= CAMERA_ROOM_BUBBLE_LIDAR_MAX_CAMERA_DELTA_M
            and float(bbox_height_ratio) >= CAMERA_ROOM_BUBBLE_LIDAR_MIN_BBOX_HEIGHT_RATIO
            and (
                detector in {"mediapipe_pose", "onnx_yolov5_person", "opencv_hog"}
                or (detector == "opencv_template_lock" and detector_confidence >= 0.55)
                or (detector == "opencv_motion_blob" and detector_confidence >= 0.65)
            )
        ):
            close_bubble_front_target_m = float(min_dist)
            close_bubble_front_target_source = "front_lidar_room_bubble_camera_confirmed"
        motion_blob_lidar_guard = bool(
            detector == "opencv_motion_blob"
            and lidar_dist_m is not None
            and desired_distance_m > CAMERA_CLOSE_BUBBLE_LIDAR_DESIRED_MAX_M
            and float(lidar_dist_m) <= max(
                float(CAMERA_DISTANCE_MOTION_BLOB_LIDAR_GUARD_MAX_LIDAR_M),
                desired_distance_m + 0.55,
            )
            and abs(float(camera_distance_m) - float(lidar_dist_m))
            >= CAMERA_DISTANCE_MOTION_BLOB_LIDAR_GUARD_DELTA_M
        )
        if (
            close_bubble_front_target_m is not None
            and (
                lidar_dist_m is None
                or close_bubble_front_target_m < float(lidar_dist_m)
                or float(camera_distance_m) - float(lidar_dist_m) >= CAMERA_CLOSE_BUBBLE_LIDAR_MIN_CAMERA_DELTA_M
            )
        ):
            dist_m = float(close_bubble_front_target_m)
            lidar_dist_m = float(close_bubble_front_target_m)
            lidar_status = dict(lidar_status)
            lidar_status.update(
                {
                    "state": "ok",
                    "source": str(close_bubble_front_target_source),
                    "usable_distance": True,
                    "distance_m": float(close_bubble_front_target_m),
                    "confidence": max(0.35, float(lidar_status.get("confidence") or 0.0)),
                }
            )
            camera_status["lidar_distance_m"] = float(lidar_dist_m)
            camera_status["camera_lidar_delta_m"] = float(camera_distance_m) - float(lidar_dist_m)
            camera_status["distance_used_m"] = float(dist_m)
            camera_status["distance_source"] = str(close_bubble_front_target_source)
            confidence *= max(0.10, min(1.0, (float(camera_distance_confidence) * 0.60) + 0.25))
        elif motion_blob_lidar_guard:
            dist_m = float(lidar_dist_m)
            camera_status["distance_used_m"] = float(dist_m)
            camera_status["distance_source"] = "motion_blob_lidar_guard"
            camera_status["motion_blob_lidar_guard"] = True
            confidence *= max(0.10, min(1.0, (float(camera_distance_confidence) * 0.25) + (lidar_conf * 0.75)))
        elif lidar_dist_m is not None and abs(float(camera_distance_m) - float(lidar_dist_m)) <= CAMERA_DISTANCE_LIDAR_BLEND_MAX_DELTA_M:
            cam_weight = float(CAMERA_DISTANCE_LIDAR_BLEND_CAMERA_WEIGHT)
            dist_m = (cam_weight * float(camera_distance_m)) + ((1.0 - cam_weight) * float(lidar_dist_m))
            camera_status["distance_used_m"] = float(dist_m)
            camera_status["distance_source"] = "camera_lidar_blend"
            confidence *= max(0.10, min(1.0, (float(camera_distance_confidence) * 0.75) + (lidar_conf * 0.25)))
        else:
            dist_m = float(camera_distance_m)
            confidence *= max(0.10, min(1.0, float(camera_distance_confidence)))
    else:
        if dist_m is None:
            if previous_target is not None and previous_target.get("dist_m") is not None:
                dist_m = float(previous_target.get("dist_m"))
                lidar_status = dict(lidar_status)
                lidar_status["state"] = "held_expected_distance"
                lidar_status["source"] = "tracker_distance_hold_no_lidar_match"
                lidar_status["distance_m"] = float(dist_m)
                lidar_status["usable_distance"] = False
                confidence *= 0.45 if str(lidar_status.get("source") or "") == "lidar_stale" else 0.55
            else:
                dist_m = params["target_distance_m"]
                confidence *= 0.35 if str(lidar_status.get("source") or "") == "lidar_stale" else 0.60
        else:
            confidence *= max(0.10, min(1.0, float(lidar_status.get("confidence") or 0.0)))

    force_close_front_distance = str(camera_status.get("distance_source") or "") in {
        "front_lidar_close_bubble_camera_confirmed",
        "front_lidar_room_bubble_camera_confirmed",
    }
    force_lidar_guard_distance = str(camera_status.get("distance_source") or "") == "motion_blob_lidar_guard"
    tracked = tracker.observe(
        dist_m=dist_m,
        angle_deg=angle_deg,
        confidence=confidence,
        force_distance=bool(force_close_front_distance or force_lidar_guard_distance),
        zero_velocity=bool(force_close_front_distance or force_lidar_guard_distance),
    )
    if force_close_front_distance or force_lidar_guard_distance:
        tracked["dist_m"] = float(dist_m)
        tracked["vx"] = 0.0
        tracked["vy"] = 0.0
        tracked["measurement_limited"] = False
    if camera_distance_active:
        measured_distance_used_m = _finite_positive_float(camera_status.get("distance_used_m"))
        if measured_distance_used_m is not None:
            camera_status["distance_measurement_used_m"] = float(measured_distance_used_m)
        camera_status["distance_used_m"] = float(tracked["dist_m"])
        camera_status["tracked_distance_m"] = float(tracked["dist_m"])
    follow_state = _adaptive_follow_state(
        camera_status=camera_status,
        lidar_status=lidar_status,
        dist_m=tracked["dist_m"],
        angle_deg=tracked["angle_deg"],
        params=params,
    )

    lidar_source = str(lidar_status.get("source") or "lidar_missing")
    lidar_confidence = float(lidar_status.get("confidence") or 0.0)

    # Publikálás a megosztott robot_state-be (percepció szeparáció előkészítése)
    try:
        import robot_state
        robot_state.set_tracked_target(
            tracked["dist_m"],
            tracked["angle_deg"],
            confidence=tracked["confidence"],
            vx=tracked.get("vx"),
            vy=tracked.get("vy"),
            last_seen_ts=tracked.get("last_seen_ts"),
        )
    except Exception:
        pass

    # Telemetria/percepció: high-level célértékek (status/adaptive_motion).
    setattr(ctrl, "_adaptive_target_dist_m", tracked["dist_m"])
    setattr(ctrl, "_adaptive_target_angle_deg", tracked["angle_deg"])
    setattr(ctrl, "_adaptive_target_confidence", tracked.get("confidence"))
    setattr(ctrl, "_adaptive_target_vx_mps", tracked.get("vx"))
    setattr(ctrl, "_adaptive_target_vy_mps", tracked.get("vy"))
    setattr(ctrl, "_adaptive_target_last_seen_ts", tracked.get("last_seen_ts"))
    setattr(ctrl, "_adaptive_target_desired_distance_m", params["target_distance_m"])
    setattr(ctrl, "_adaptive_target_lidar_source", lidar_source)
    setattr(ctrl, "_adaptive_target_lidar_confidence", lidar_confidence)
    setattr(ctrl, "_adaptive_target_lidar_distance_m", None if lidar_measurement is None else lidar_measurement.get("distance_m"))
    setattr(ctrl, "_adaptive_target_lidar_points", int((lidar_measurement or {}).get("point_count") or 0))
    setattr(ctrl, "_adaptive_target_lidar_cluster_points", int((lidar_measurement or {}).get("cluster_points") or 0))
    setattr(ctrl, "_adaptive_target_lidar_age_s", (lidar_measurement or {}).get("age_s"))
    setattr(ctrl, "_adaptive_target_lidar_status", dict(lidar_status))
    setattr(ctrl, "_adaptive_target_camera_status", dict(camera_status))
    setattr(ctrl, "_adaptive_follow_state", str(follow_state))
    return (0.0, 0.0)


def get_adaptive_command(ctrl, lidar_snapshot) -> Optional[Tuple[float, float]]:
    """
    Legacy adaptive motion entry point.
    Megtartja a percepciós side effectet, de nem ad vissza közvetlen v/omega
    parancsot; a mozgás SSOT a FOLLOW -> CRUISE -> executor útvonal.
    """
    if not getattr(ctrl, "following_active", False):
        return None
    tick(ctrl, lidar_snapshot)
    return None


def compute_pursuit_target_pose(
    ctrl,
    dist_m: float,
    angle_deg: float,
    ekf_state: dict,
    look_ahead_scale: float = 1.0,
) -> Optional[Tuple[float, float, float]]:
    """
    Pursuit / predictive tracking: (dist_m, angle_deg) + EKF pose → target_pose (x, y, theta_rad).
    A célpont a robot előtt dist_m távolságra, angle_deg irányban (kamera/LIDAR);
    világkoordinátában: px + d*cos(theta+angle), py + d*sin(theta+angle), theta_ref = theta + angle.
    look_ahead_scale: cél távolság skálázása (prediktív követéshez).
    Vissza: (x_cél, y_cél, theta_cél_rad) vagy None.
    """
    px = float(ekf_state.get("x", 0.0))
    py = float(ekf_state.get("y", 0.0))
    theta = ekf_state.get("theta")
    if theta is None:
        theta = math.radians(float(ekf_state.get("theta_deg", 0.0)))
    else:
        theta = float(theta)
    d = max(0.0, float(dist_m)) * float(look_ahead_scale)
    angle_rad = math.radians(float(angle_deg))
    direction = theta + angle_rad
    x_t = px + d * math.cos(direction)
    y_t = py + d * math.sin(direction)
    theta_t = direction
    return (x_t, y_t, theta_t)


def get_adaptive_target_pose(ctrl, lidar_snapshot, ekf_state: dict) -> Optional[Tuple[float, float, float]]:
    """
    Ha follow_use_pursuit és following_active: kamera+LIDAR alapján target_pose (x, y, theta).
    Egy tickre beállítandó; a pose controller ezt követi. None = ne használj pursuit-ot (marad v/omega).
    """
    if not getattr(ctrl, "following_active", False) or not getattr(ctrl, "follow_use_pursuit", False):
        return None
    dist_m = getattr(ctrl, "_adaptive_target_dist_m", None)
    angle_deg = getattr(ctrl, "_adaptive_target_angle_deg", None)
    if dist_m is None or angle_deg is None:
        return None
    return compute_pursuit_target_pose(
        ctrl, dist_m, angle_deg, ekf_state,
        look_ahead_scale=float(getattr(ctrl, "follow_pursuit_look_ahead_scale", 1.0)),
    )


def start_following(ctrl) -> None:
    """Követés aktiválása (F billentyű); átvált FOLLOW állapotra. Formális forrás: ADAPTIVE (arbiter)."""
    if getattr(ctrl, "following_active", False):
        if hasattr(ctrl, "logger"):
            ctrl.logger.info("[FOLLOW] Már aktív; nyomd meg újra az F-et a leállításhoz.")
        return
    if getattr(ctrl, "status_path", None):
        try:
            _wait_for_follow_stream_seed_frame(ctrl, timeout_s=FOLLOW_START_STREAM_SEED_WAIT_S)
        except Exception:
            pass
    try:
        from controller.stream_writer import request_stream_writer_camera_release
        released = request_stream_writer_camera_release(ctrl, timeout_s=1.2)
        if not released and hasattr(ctrl, "logger"):
            ctrl.logger.warn("[FOLLOW] GUI stream kamera átadás timeout; követés kamera nyitása ettől még megpróbálja.")
    except Exception:
        pass
    from controller.commands import set_motion_source
    if not set_motion_source(ctrl, "ADAPTIVE"):
        if hasattr(ctrl, "logger"):
            ctrl.logger.warn("[FOLLOW] Arbiter nem engedélyezte ADAPTIVE forrást.")
        return
    # Stabil handover: GUI joystick maradék intent és állapot tisztítása.
    try:
        import robot_state
        robot_state.clear_intent()
    except Exception:
        pass
    ctrl.input_vector = {"x": 0.0, "y": 0.0}
    ctrl.joystick_active = False
    ctrl.joystick_zero_since = time.perf_counter()
    # Follow indításkor a GUI forrás ne tartsa feleslegesen a birtoklást.
    if hasattr(ctrl, "arbiter") and hasattr(ctrl.arbiter, "last_ts"):
        ctrl.arbiter.last_ts["GUI_JOYSTICK"] = 0.0
    # Kamera reset: korábbi hibák törlése
    fcam = _get_follower_camera(ctrl)
    fcam._failed_sessions = 0
    _get_target_tracker(ctrl).clear()
    _clear_follow_search_attrs(ctrl, state="idle")
    stream_seeded = False
    if getattr(ctrl, "status_path", None):
        try:
            prewarm_onnx = bool(fcam._ensure_onnx_person_detector())
            try:
                from controller.status import append_camera_log
                append_camera_log(
                    ctrl,
                    "follow_onnx_prewarm",
                    onnx_ready=bool(prewarm_onnx),
                )
            except Exception:
                pass
        except Exception as e:
            if hasattr(ctrl, "logger"):
                ctrl.logger.warn(f"[FOLLOW] ONNX előmelegítés hiba: {e}")
        try:
            stream_seeded = bool(_prime_follow_target_from_stream_seed(ctrl, fcam, _follower_params(ctrl)))
            if not stream_seeded:
                try:
                    from controller.status import append_camera_log
                    bbox_status = dict(getattr(fcam, "_last_bbox_status", {}) or {})
                    append_camera_log(
                        ctrl,
                        "follow_stream_seed_miss",
                        detector_error=str(getattr(fcam, "_last_detector_error", "") or ""),
                        onnx_best_score=bbox_status.get("onnx_best_score"),
                        onnx_best_objectness=bbox_status.get("onnx_best_objectness"),
                        onnx_best_person_class_score=bbox_status.get("onnx_best_person_class_score"),
                        onnx_best_reject_reason=bbox_status.get("onnx_best_reject_reason"),
                    )
                except Exception:
                    pass
        except Exception as e:
            stream_seeded = False
            if hasattr(ctrl, "logger"):
                ctrl.logger.warn(f"[FOLLOW] Stream seed hiba: {e}")
    ctrl.following_active = True
    if RobotState is not None and hasattr(ctrl, "sm"):
        ctrl.sm.transition_to(RobotState.FOLLOW)
    try:
        from controller.status import append_camera_log
        append_camera_log(ctrl, "follow_start")
    except Exception:
        pass
    if hasattr(ctrl, "logger"):
        seed_msg = " stream-seeddel" if stream_seeded else ""
        ctrl.logger.info(f"[FOLLOW] Ember követése BE{seed_msg} – kamera + LIDAR fúzió.")


def stop_following(ctrl) -> None:
    """Követés leállítása; kamera felszabadítása; vissza IDLE. Forrás formálisan MANUAL."""
    ctrl.following_active = False
    fcam = _get_follower_camera(ctrl)
    stop_async = getattr(fcam, "stop_async", None)
    if callable(stop_async):
        stop_async(ctrl)
    else:
        fcam.release(ctrl)
    _get_target_tracker(ctrl).clear()
    _clear_follow_search_attrs(ctrl, state="stopped")
    _clear_adaptive_target_attrs(
        ctrl,
        camera_status={"state": "stopped", "source": "camera", "target_usable": False},
        lidar_status={"state": "stopped", "source": "lidar", "usable_distance": False},
    )
    try:
        ctrl.follow_target_observation = {}
        ctrl.follow_layer_status = {"active": False, "reason": "follow_stopped"}
        ctrl.cruise_layer_status = {"active": False, "reason": "follow_stopped"}
    except Exception:
        pass
    try:
        import robot_state
        robot_state.clear_tracked_target()
    except Exception:
        pass
    try:
        from controller.commands import set_motion_source
        set_motion_source(ctrl, "MANUAL")
    except Exception:
        pass
    try:
        from controller.status import append_camera_log
        append_camera_log(ctrl, "follow_stop")
    except Exception:
        pass
    if RobotState is not None and hasattr(ctrl, "sm"):
        ctrl.sm.transition_to(RobotState.IDLE)
