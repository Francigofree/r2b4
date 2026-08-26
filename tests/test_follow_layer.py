#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import controller.tasks.follower as follower_module  # noqa: E402
from controller.cruise_layer import CruiseLayer  # noqa: E402
from controller.follow_layer import (  # noqa: E402
    FollowLayer,
    FollowLayerConfig,
    camera_observation_from_controller,
)
from controller.follow_types import (  # noqa: E402
    FRAME_WORLD,
    TARGET_SOURCE_CAMERA_SEARCH,
    TARGET_SOURCE_CAMERA_TARGET,
    TARGET_SOURCE_SIM_TARGET,
    TargetObservation,
)
from controller.target_obstacle_arbiter import TargetObstacleArbiter, TargetObstacleArbiterConfig  # noqa: E402
from controller.tasks.follower import (  # noqa: E402
    FollowerCamera,
    TargetKinematicsTracker,
    _adaptive_follow_state,
    _camera_distance_from_bbox,
    _camera_human_bbox_shape_status,
    _get_lidar_min_dist_front,
    _get_lidar_dist_at_angle_deg,
    _get_lidar_target_measurement_at_angle_deg,
    _target_lidar_status,
    start_following,
)


class TestFollowLayer(unittest.TestCase):
    def test_async_camera_detection_returns_immediately_and_publishes_latest_result(self):
        cam = FollowerCamera()
        expected = (120.0, 80.0, 320.0, 0.82)

        def fake_detect(_ctrl):
            time.sleep(0.08)
            cam._last_result_status = {
                "state": "ok",
                "source": "camera",
                "stale": False,
                "target_visible": True,
                "target_usable": True,
                "frame_ok": True,
            }
            return expected

        cam.detect_with_persistence = fake_detect
        started = time.monotonic()
        first = cam.detect_latest(SimpleNamespace())
        dispatch_s = time.monotonic() - started

        self.assertIsNone(first)
        self.assertLess(dispatch_s, 0.04)
        deadline = time.monotonic() + 1.0
        latest = None
        while time.monotonic() < deadline and latest is None:
            time.sleep(0.01)
            latest = cam.detect_latest(SimpleNamespace())
        status = cam.last_status()
        cam.stop_async()

        self.assertEqual(latest, expected)
        self.assertTrue(status["async_worker_active"])
        self.assertTrue(status["target_usable"])
        self.assertGreaterEqual(status["async_update_seq"], 1)

    def test_async_camera_stale_result_is_not_usable(self):
        cam = FollowerCamera()
        with cam._async_lock:
            cam._async_enabled = True
            cam._async_result = (120.0, 80.0, 320.0, 0.82)
            cam._async_status = {
                "state": "ok",
                "target_visible": True,
                "target_usable": True,
            }
            cam._async_completed_ts = time.monotonic() - follower_module.CAMERA_ASYNC_RESULT_MAX_AGE_S - 0.1

        status = cam.last_status()

        self.assertEqual(status["state"], "async_result_stale")
        self.assertFalse(status["target_visible"])
        self.assertFalse(status["target_usable"])
        self.assertTrue(status["async_stale_gate"])

    def test_locked_target_uses_template_before_periodic_onnx_revalidation(self):
        cam = FollowerCamera()
        calls = []
        cam._lock_active = True
        cam._last_human_detector_ts = time.monotonic()
        cam._template_lock_allowed = lambda: True
        cam._detect_template_lock = lambda *_args: calls.append("template") or (100.0, 80.0)
        cam._detect_onnx_person_bbox = lambda *_args: calls.append("onnx") or None

        center = cam._detect_person_bbox(object(), 320, 240)

        self.assertEqual(center, (100.0, 80.0))
        self.assertEqual(calls, ["template"])

    def test_camera_bbox_status_includes_gui_overlay_coordinates(self):
        cam = FollowerCamera()
        cam._last_detector_confidence = 0.70

        cam._remember_bbox(
            detector="opencv_hog",
            bbox_x_px=0,
            bbox_y_px=24,
            bbox_width_px=96,
            bbox_height_px=144,
            image_width_px=320,
            image_height_px=240,
            center_x_px=48,
            center_y_px=96,
        )

        status = cam._last_bbox_status
        self.assertEqual(status["bbox_x_px"], 0.0)
        self.assertEqual(status["bbox_y_px"], 24.0)
        self.assertEqual(status["target_center_x_px"], 48.0)
        self.assertEqual(status["target_center_y_px"], 96.0)
        self.assertAlmostEqual(status["bbox_width_ratio"], 0.30, places=3)
        self.assertAlmostEqual(status["bbox_height_ratio"], 0.60, places=3)
        self.assertTrue(status["bbox_human_shape_ok"])

    def test_camera_lock_requires_multiple_stable_onnx_frames(self):
        cam = FollowerCamera()

        def observe(x_px=134):
            cam._last_detector_name = "onnx_yolov5_person"
            cam._last_detector_confidence = 0.72
            cam._remember_bbox(
                detector="onnx_yolov5_person",
                bbox_x_px=x_px,
                bbox_y_px=32,
                bbox_width_px=70,
                bbox_height_px=170,
                image_width_px=320,
                image_height_px=240,
                center_x_px=x_px + 35,
                center_y_px=117,
            )
            return cam._update_detection_lock((x_px + 35, 117), 320, 240)

        ok, status = observe()
        self.assertFalse(ok)
        self.assertEqual(status["lock_reason"], "confirming_frames")
        for entry in cam._candidate_history:
            entry["ts"] -= 0.20
        ok, status = observe(136)
        self.assertFalse(ok)
        for entry in cam._candidate_history:
            entry["ts"] -= 0.20
        ok, status = observe(135)
        self.assertTrue(ok)
        self.assertTrue(status["lock_confirmed"])
        self.assertEqual(status["lock_state"], "locked")

    def test_camera_lock_accepts_two_stable_moderate_startup_onnx_frames(self):
        cam = FollowerCamera()

        def observe(x_px=120):
            cam._last_detector_name = "onnx_yolov5_person"
            cam._last_detector_confidence = 0.64
            cam._remember_bbox(
                detector="onnx_yolov5_person",
                bbox_x_px=x_px,
                bbox_y_px=42,
                bbox_width_px=82,
                bbox_height_px=132,
                image_width_px=320,
                image_height_px=240,
                center_x_px=x_px + 41,
                center_y_px=108,
            )
            return cam._update_detection_lock((x_px + 41, 108), 320, 240)

        ok, status = observe()
        self.assertFalse(ok)
        self.assertEqual(status["lock_reason"], "confirming_frames")
        for entry in cam._candidate_history:
            entry["ts"] -= 0.20

        ok, status = observe(122)

        self.assertTrue(ok)
        self.assertTrue(status["lock_confirmed"])
        self.assertEqual(status["lock_state"], "locked")
        self.assertEqual(status["lock_confirm_count"], 2)
        self.assertEqual(status["lock_required_frames"], 2)
        self.assertTrue(status["lock_startup_onnx_two_frame_path"])

    def test_recent_onnx_person_relocks_after_fresh_lock_loss(self):
        cam = FollowerCamera()
        cam._last_lock_ts = time.monotonic() - 0.4
        cam._last_detector_name = "onnx_yolov5_person"
        cam._last_detector_confidence = 0.60
        cam._remember_bbox(
            detector="onnx_yolov5_person",
            bbox_x_px=92,
            bbox_y_px=34,
            bbox_width_px=96,
            bbox_height_px=132,
            image_width_px=320,
            image_height_px=240,
            center_x_px=140,
            center_y_px=105,
        )

        ok, status = cam._update_detection_lock((140, 105), 320, 240)

        self.assertTrue(ok)
        self.assertTrue(status["lock_confirmed"])
        self.assertEqual(status["lock_state"], "locked")
        self.assertEqual(status["lock_confirm_count"], 1)
        self.assertTrue(status["lock_recent_onnx_single_frame_relock_path"])

    def test_recent_template_candidate_relocks_after_fresh_lock_loss(self):
        cam = FollowerCamera()
        cam._last_lock_ts = time.monotonic() - 2.0
        cam._template_lock_gray = object()
        cam._template_lock_bbox = (100.0, 30.0, 72.0, 150.0, 320.0, 240.0)
        cam._last_human_detector_ts = time.monotonic()
        cam._last_detector_name = "opencv_template_lock"
        cam._last_detector_confidence = 0.61
        cam._remember_bbox(
            detector="opencv_template_lock",
            bbox_x_px=100,
            bbox_y_px=32,
            bbox_width_px=72,
            bbox_height_px=150,
            image_width_px=320,
            image_height_px=240,
            center_x_px=136,
            center_y_px=107,
        )

        ok, status = cam._update_detection_lock((136, 107), 320, 240)

        self.assertTrue(ok)
        self.assertTrue(status["lock_confirmed"])
        self.assertEqual(status["lock_confirm_count"], 1)
        self.assertTrue(status["lock_recent_template_single_frame_relock_path"])

    def test_recent_human_onnx_relocks_from_single_frame_after_unstable_candidates(self):
        cam = FollowerCamera()
        cam._template_lock_gray = object()
        cam._template_lock_bbox = (100.0, 30.0, 72.0, 150.0, 320.0, 240.0)
        cam._last_human_detector_ts = time.monotonic()
        cam._candidate_history.extend(
            [
                {
                    "ts": time.monotonic() - 0.35,
                    "detector": "opencv_motion_blob",
                    "confidence": 0.35,
                    "center_x_ratio": 0.18,
                    "width_ratio": 0.20,
                    "height_ratio": 0.60,
                    "zone": "left",
                },
                {
                    "ts": time.monotonic() - 0.20,
                    "detector": "opencv_motion_blob",
                    "confidence": 0.38,
                    "center_x_ratio": 0.82,
                    "width_ratio": 0.22,
                    "height_ratio": 0.62,
                    "zone": "right",
                },
            ]
        )
        cam._last_detector_name = "onnx_yolov5_person"
        cam._last_detector_confidence = 0.72
        cam._remember_bbox(
            detector="onnx_yolov5_person",
            bbox_x_px=104,
            bbox_y_px=34,
            bbox_width_px=88,
            bbox_height_px=150,
            image_width_px=320,
            image_height_px=240,
            center_x_px=148,
            center_y_px=109,
        )

        ok, status = cam._update_detection_lock((148, 109), 320, 240)

        self.assertTrue(ok)
        self.assertTrue(status["lock_confirmed"])
        self.assertEqual(status["lock_confirm_count"], 1)
        self.assertTrue(status["lock_recent_human_onnx_single_frame_relock_path"])

    def test_strong_startup_onnx_person_locks_from_single_frame(self):
        cam = FollowerCamera()
        cam._candidate_history.append(
            {
                "ts": time.monotonic() - 0.2,
                "detector": "opencv_motion_blob",
                "confidence": 0.42,
                "center_x_ratio": 0.85,
                "width_ratio": 0.50,
                "height_ratio": 1.0,
                "zone": "right",
            }
        )
        cam._last_detector_name = "onnx_yolov5_person"
        cam._last_detector_confidence = 0.80
        cam._remember_bbox(
            detector="onnx_yolov5_person",
            bbox_x_px=92,
            bbox_y_px=34,
            bbox_width_px=96,
            bbox_height_px=132,
            image_width_px=320,
            image_height_px=240,
            center_x_px=140,
            center_y_px=105,
        )

        ok, status = cam._update_detection_lock((140, 105), 320, 240)

        self.assertTrue(ok)
        self.assertTrue(status["lock_confirmed"])
        self.assertEqual(status["lock_confirm_count"], 1)
        self.assertTrue(status["lock_startup_onnx_single_frame_path"])

    def test_camera_lock_allows_onnx_seeded_motion_confirmation(self):
        cam = FollowerCamera()

        def observe_onnx():
            cam._last_detector_name = "onnx_yolov5_person"
            cam._last_detector_confidence = 0.53
            cam._remember_bbox(
                detector="onnx_yolov5_person",
                bbox_x_px=70,
                bbox_y_px=1,
                bbox_width_px=138,
                bbox_height_px=238,
                image_width_px=320,
                image_height_px=240,
                center_x_px=139,
                center_y_px=120,
            )
            return cam._update_detection_lock((139, 120), 320, 240)

        def observe_motion(confidence=0.34):
            cam._last_detector_name = "opencv_motion_blob"
            cam._last_detector_confidence = confidence
            cam._remember_bbox(
                detector="opencv_motion_blob",
                bbox_x_px=108,
                bbox_y_px=54,
                bbox_width_px=60,
                bbox_height_px=92,
                image_width_px=320,
                image_height_px=240,
                center_x_px=138,
                center_y_px=100,
                bbox_fill_ratio=0.44,
            )
            return cam._update_detection_lock((138, 100), 320, 240)

        ok, status = observe_onnx()
        self.assertFalse(ok)
        self.assertEqual(status["lock_reason"], "confirming_frames")
        for entry in cam._candidate_history:
            entry["ts"] -= 0.80
        ok, status = observe_motion()
        self.assertFalse(ok)
        self.assertEqual(status["lock_reason"], "confirming_frames")
        for entry in cam._candidate_history:
            entry["ts"] -= 0.80
        ok, status = observe_motion(confidence=0.31)

        self.assertTrue(ok)
        self.assertTrue(status["lock_confirmed"])
        self.assertTrue(status["lock_onnx_seeded_fallback_path"])

    def test_camera_lock_keeps_strong_onnx_same_size_center_jump(self):
        cam = FollowerCamera()

        def observe(x_px=134, confidence=0.86, width_px=70, height_px=170):
            cam._last_detector_name = "onnx_yolov5_person"
            cam._last_detector_confidence = confidence
            cam._remember_bbox(
                detector="onnx_yolov5_person",
                bbox_x_px=x_px,
                bbox_y_px=32,
                bbox_width_px=width_px,
                bbox_height_px=height_px,
                image_width_px=320,
                image_height_px=240,
                center_x_px=x_px + (width_px / 2.0),
                center_y_px=117,
            )
            return cam._update_detection_lock((x_px + (width_px / 2.0), 117), 320, 240)

        ok, _status = observe()
        self.assertFalse(ok)
        for entry in cam._candidate_history:
            entry["ts"] -= 0.20
        ok, _status = observe(136)
        self.assertFalse(ok)
        for entry in cam._candidate_history:
            entry["ts"] -= 0.20
        ok, status = observe(135)
        self.assertTrue(ok)
        self.assertTrue(status["lock_confirmed"])

        ok, status = observe(278)

        self.assertTrue(ok)
        self.assertTrue(status["lock_confirmed"])
        self.assertEqual(status["lock_reason"], "lock_tracking_strong_jump")
        self.assertLessEqual(
            status["lock_center_jump_ratio"],
            follower_module.CAMERA_LOCK_STRONG_JUMP_MAX_CENTER_RATIO,
        )

    def test_camera_lock_rejects_low_confidence_center_jump(self):
        cam = FollowerCamera()

        def observe(x_px=134, confidence=0.86):
            cam._last_detector_name = "onnx_yolov5_person"
            cam._last_detector_confidence = confidence
            cam._remember_bbox(
                detector="onnx_yolov5_person",
                bbox_x_px=x_px,
                bbox_y_px=32,
                bbox_width_px=70,
                bbox_height_px=170,
                image_width_px=320,
                image_height_px=240,
                center_x_px=x_px + 35,
                center_y_px=117,
            )
            return cam._update_detection_lock((x_px + 35, 117), 320, 240)

        ok, _status = observe()
        self.assertFalse(ok)
        for entry in cam._candidate_history:
            entry["ts"] -= 0.20
        ok, _status = observe(136)
        self.assertFalse(ok)
        for entry in cam._candidate_history:
            entry["ts"] -= 0.20
        ok, status = observe(135)
        self.assertTrue(ok)
        self.assertTrue(status["lock_confirmed"])

        ok, status = observe(278, confidence=0.60)

        self.assertFalse(ok)
        self.assertFalse(status["lock_confirmed"])
        self.assertEqual(status["lock_reason"], "bbox_center_jump")

    def test_camera_relock_uses_single_strong_onnx_frame_after_fresh_lock(self):
        cam = FollowerCamera()

        def observe(x_px=134, confidence=0.86):
            cam._last_detector_name = "onnx_yolov5_person"
            cam._last_detector_confidence = confidence
            cam._remember_bbox(
                detector="onnx_yolov5_person",
                bbox_x_px=x_px,
                bbox_y_px=32,
                bbox_width_px=70,
                bbox_height_px=170,
                image_width_px=320,
                image_height_px=240,
                center_x_px=x_px + 35,
                center_y_px=117,
            )
            return cam._update_detection_lock((x_px + 35, 117), 320, 240)

        ok, _status = observe()
        self.assertFalse(ok)
        for entry in cam._candidate_history:
            entry["ts"] -= 0.20
        ok, _status = observe(136)
        self.assertFalse(ok)
        for entry in cam._candidate_history:
            entry["ts"] -= 0.20
        ok, status = observe(135)
        self.assertTrue(ok)
        self.assertEqual(status["lock_required_frames"], 3)

        cam._reset_lock_state(clear_history=True, reason="test_loss")
        ok, status = observe(136, confidence=0.60)

        self.assertTrue(ok)
        self.assertTrue(status["lock_recent_onnx_single_frame_relock_path"])
        self.assertEqual(status["lock_required_frames"], 1)

        cam._reset_lock_state(clear_history=True, reason="older_test_loss")
        cam._last_lock_ts = time.monotonic() - 2.0
        ok, status = observe(136, confidence=0.88)
        self.assertFalse(ok)
        for entry in cam._candidate_history:
            entry["ts"] -= 0.20
        ok, status = observe(137, confidence=0.90)

        self.assertTrue(ok)
        self.assertTrue(status["lock_relock_fast_path"])
        self.assertEqual(status["lock_required_frames"], 2)

    def test_template_lock_can_relock_only_after_recent_human_lock(self):
        cam = FollowerCamera()

        def observe(x_px=134, confidence=0.62):
            cam._last_detector_name = "opencv_template_lock"
            cam._last_detector_confidence = confidence
            cam._remember_bbox(
                detector="opencv_template_lock",
                bbox_x_px=x_px,
                bbox_y_px=32,
                bbox_width_px=70,
                bbox_height_px=170,
                image_width_px=320,
                image_height_px=240,
                center_x_px=x_px + 35,
                center_y_px=117,
            )
            return cam._update_detection_lock((x_px + 35, 117), 320, 240)

        ok, status = observe()
        self.assertFalse(ok)
        self.assertEqual(status["lock_reason"], "fallback_detector_requires_existing_lock")

        cam._last_lock_ts = time.monotonic()
        ok, status = observe()
        self.assertFalse(ok)
        self.assertEqual(status["lock_reason"], "confirming_frames")
        for entry in cam._candidate_history:
            entry["ts"] -= 0.20
        ok, status = observe(136)
        self.assertFalse(ok)
        for entry in cam._candidate_history:
            entry["ts"] -= 0.20
        ok, status = observe(135)

        self.assertTrue(ok)
        self.assertTrue(status["lock_template_relock_path"])
        self.assertEqual(status["lock_required_frames"], 3)

    def test_template_lock_can_relock_from_recent_human_template(self):
        cam = FollowerCamera()
        cam._template_lock_gray = object()
        cam._template_lock_bbox = (100.0, 30.0, 70.0, 170.0, 320.0, 240.0)
        cam._last_human_detector_ts = time.monotonic()

        def observe(x_px=134, confidence=0.62):
            cam._last_detector_name = "opencv_template_lock"
            cam._last_detector_confidence = confidence
            cam._remember_bbox(
                detector="opencv_template_lock",
                bbox_x_px=x_px,
                bbox_y_px=32,
                bbox_width_px=70,
                bbox_height_px=170,
                image_width_px=320,
                image_height_px=240,
                center_x_px=x_px + 35,
                center_y_px=117,
            )
            return cam._update_detection_lock((x_px + 35, 117), 320, 240)

        ok, status = observe()
        self.assertFalse(ok)
        self.assertEqual(status["lock_reason"], "confirming_frames")
        for entry in cam._candidate_history:
            entry["ts"] -= 0.20
        ok, status = observe(136)
        self.assertFalse(ok)
        for entry in cam._candidate_history:
            entry["ts"] -= 0.20
        ok, status = observe(135)

        self.assertTrue(ok)
        self.assertTrue(status["lock_template_relock_path"])
        self.assertEqual(status["lock_required_frames"], 3)

    def test_rejected_candidate_uses_recent_persisted_target(self):
        import numpy as np

        cam = FollowerCamera()
        cam._last_center = (160.0, 117.0, 320.0)
        cam._last_detection_ts = time.monotonic()
        cam._lock_active = True
        cam._lock_center_x_ratio = 0.50
        cam._lock_width_ratio = 0.22
        cam._lock_height_ratio = 0.70
        cam.capture_frame = lambda ctrl: (np.zeros((240, 320, 3), dtype=np.uint8), 320, 240)

        def fake_detect(_frame, _width, _height):
            cam._last_detector_name = "opencv_motion_blob"
            cam._last_detector_confidence = 0.75
            cam._remember_bbox(
                detector="opencv_motion_blob",
                bbox_x_px=0,
                bbox_y_px=32,
                bbox_width_px=70,
                bbox_height_px=170,
                image_width_px=320,
                image_height_px=240,
                center_x_px=10,
                center_y_px=117,
            )
            return (10.0, 117.0)

        cam._detect_person_bbox = fake_detect

        center = cam.detect_with_persistence(SimpleNamespace())
        status = cam.last_status()

        self.assertEqual(center, (160.0, 117.0, 320.0, 0.5))
        self.assertEqual(status["state"], "target_persisted")
        self.assertTrue(status["target_usable"])
        self.assertTrue(status["stale"])
        self.assertTrue(status["candidate_rejected"])
        self.assertEqual(status["candidate_reject_reason"], "bbox_center_jump")

    def test_camera_search_observation_carries_last_seen_side(self):
        ctrl = SimpleNamespace(
            follower_cfg={},
            follow_speed_scale=1.0,
            _adaptive_target_search_active=True,
            _adaptive_target_search_side="right",
        )

        obs = camera_observation_from_controller(ctrl)

        self.assertIsNotNone(obs)
        self.assertEqual(obs.source, TARGET_SOURCE_CAMERA_SEARCH)
        self.assertEqual(obs.target_id, "camera_target_search_right")

    def test_camera_human_bbox_shape_rejects_wide_furniture_like_box(self):
        status = _camera_human_bbox_shape_status(
            detector="opencv_motion_blob",
            bbox_width_px=220,
            bbox_height_px=70,
            image_width_px=320,
            image_height_px=240,
            bbox_fill_ratio=0.70,
        )

        self.assertFalse(status["bbox_human_shape_ok"])
        self.assertIn(status["bbox_reject_reason"], {"bbox_too_wide_for_human", "bbox_not_upright_human"})

    def test_camera_human_bbox_shape_accepts_full_height_cropped_person(self):
        status = _camera_human_bbox_shape_status(
            detector="onnx_yolov5_person",
            bbox_width_px=82,
            bbox_height_px=240,
            image_width_px=320,
            image_height_px=240,
        )

        self.assertTrue(status["bbox_human_shape_ok"])
        self.assertAlmostEqual(status["bbox_height_ratio"], 1.0)

    def test_camera_human_bbox_shape_accepts_strong_close_onnx_upper_body(self):
        status = _camera_human_bbox_shape_status(
            detector="onnx_yolov5_person",
            bbox_width_px=463,
            bbox_height_px=125,
            image_width_px=640,
            image_height_px=360,
            bbox_area_ratio=(463 * 125) / float(640 * 360),
            bbox_center_offset_ratio=0.04,
            onnx_score=0.319,
            onnx_objectness=0.479,
            onnx_person_class_score=0.666,
        )

        self.assertTrue(status["bbox_human_shape_ok"])
        self.assertEqual(status["bbox_shape_variant"], "onnx_close_upper_body")

    def test_camera_human_bbox_shape_rejects_weak_close_onnx_wide_box(self):
        status = _camera_human_bbox_shape_status(
            detector="onnx_yolov5_person",
            bbox_width_px=463,
            bbox_height_px=125,
            image_width_px=640,
            image_height_px=360,
            bbox_area_ratio=(463 * 125) / float(640 * 360),
            bbox_center_offset_ratio=0.04,
            onnx_score=0.24,
            onnx_objectness=0.479,
            onnx_person_class_score=0.666,
        )

        self.assertFalse(status["bbox_human_shape_ok"])
        self.assertEqual(status["bbox_reject_reason"], "bbox_not_upright_human")

    def test_target_obstacle_arbiter_uses_camera_distance_to_split_target_from_front_obstacle(self):
        arbiter = TargetObstacleArbiter(TargetObstacleArbiterConfig())

        decision = arbiter.decide_front_conflict(
            front_distance_m=0.62,
            desired_distance_m=1.0,
            target_angle_deg=3.0,
            target_confidence=0.8,
            camera_status={"state": "ok", "source": "camera"},
            camera_distance_m=1.45,
            camera_distance_confidence=0.70,
            lidar_snapshot_age_s=0.05,
            lidar_missing=False,
        )

        self.assertEqual(decision.mode, "front_obstacle_arbitrated")
        self.assertTrue(decision.allow_forward)
        self.assertEqual(decision.lidar_status["state"], "front_obstacle_arbitrated")
        self.assertEqual(decision.camera_updates["gate"], "front_lidar_obstacle_arbitrated_by_camera_distance")

    def test_target_obstacle_arbiter_allows_confirmed_close_front_target_for_half_meter_bubble(self):
        arbiter = TargetObstacleArbiter(TargetObstacleArbiterConfig())

        decision = arbiter.decide_front_conflict(
            front_distance_m=0.64,
            desired_distance_m=0.50,
            target_angle_deg=-4.0,
            target_confidence=0.72,
            camera_status={
                "state": "ok",
                "target_visible": True,
                "target_usable": True,
                "detector": "opencv_hog",
                "stale": False,
            },
            camera_distance_m=0.66,
            camera_distance_confidence=0.70,
            lidar_snapshot_age_s=0.05,
            lidar_missing=False,
        )

        self.assertEqual(decision.mode, "front_target_confirmed")
        self.assertTrue(decision.allow_follow_target)
        self.assertFalse(decision.allow_forward)
        self.assertAlmostEqual(float(decision.target_distance_m), 0.64)
        self.assertEqual(decision.camera_updates["gate"], "front_lidar_target_confirmed_by_camera")

    def test_target_obstacle_arbiter_does_not_turn_far_camera_target_into_close_lidar_target(self):
        arbiter = TargetObstacleArbiter(TargetObstacleArbiterConfig())

        decision = arbiter.decide_front_conflict(
            front_distance_m=0.58,
            desired_distance_m=0.50,
            target_angle_deg=2.0,
            target_confidence=0.76,
            camera_status={
                "state": "ok",
                "target_visible": True,
                "target_usable": True,
                "detector": "opencv_hog",
                "stale": False,
            },
            camera_distance_m=1.30,
            camera_distance_confidence=0.70,
            lidar_snapshot_age_s=0.04,
            lidar_missing=False,
        )

        self.assertEqual(decision.mode, "front_obstacle_arbitrated")
        self.assertAlmostEqual(float(decision.target_distance_m), 1.30, places=3)
        self.assertEqual(decision.camera_updates["distance_source"], "camera_bbox_front_obstacle_arbitrated")

    def test_target_obstacle_arbiter_holds_short_camera_dropout_without_forward(self):
        previous = {
            "dist_m": 1.80,
            "angle_deg": -8.0,
            "confidence": 0.7,
            "last_seen_ts": time.time() - 0.25,
        }
        decision = TargetObstacleArbiter().decide_target_loss(
            camera_status={"state": "target_persisted", "stale": True, "age_s": 0.25},
            previous_target=previous,
            desired_distance_m=1.0,
        )

        self.assertEqual(decision.mode, "target_persistence_hold")
        self.assertFalse(decision.allow_forward)
        self.assertAlmostEqual(float(decision.target_distance_m), 1.0)
        self.assertEqual(decision.camera_updates["gate"], "target_persistence_short_hold")

    def test_target_obstacle_arbiter_neutralizes_close_dropout_distance_to_desired(self):
        previous = {
            "dist_m": 0.72,
            "angle_deg": 5.0,
            "confidence": 0.7,
            "last_seen_ts": time.time() - 0.25,
        }

        decision = TargetObstacleArbiter().decide_target_loss(
            camera_status={"state": "target_persisted", "stale": True, "age_s": 0.25},
            previous_target=previous,
            desired_distance_m=1.0,
        )

        self.assertEqual(decision.mode, "target_persistence_hold")
        self.assertFalse(decision.allow_forward)
        self.assertAlmostEqual(float(decision.target_distance_m), 1.0)

    def test_world_sim_target_zero_standoff_goes_to_target(self):
        layer = FollowLayer(FollowLayerConfig(default_desired_distance_m=0.0))
        obs = TargetObservation(
            source=TARGET_SOURCE_SIM_TARGET,
            frame=FRAME_WORLD,
            timestamp_s=time.time(),
            x=1.0,
            y=0.25,
            theta=0.0,
            desired_distance_m=0.0,
        )

        req = layer.tick(obs, {"x": 0.0, "y": 0.0, "theta": 0.0}, source="STATE")

        self.assertTrue(req.active)
        self.assertAlmostEqual(req.goal_x, 1.0)
        self.assertAlmostEqual(req.goal_y, 0.25)
        self.assertEqual(req.target_source, TARGET_SOURCE_SIM_TARGET)

    def test_world_sim_target_default_uses_follow_bubble_standoff(self):
        layer = FollowLayer()
        obs = TargetObservation(
            source=TARGET_SOURCE_SIM_TARGET,
            frame=FRAME_WORLD,
            timestamp_s=time.time(),
            x=1.0,
            y=0.0,
            theta=0.0,
        )

        req = layer.tick(obs, {"x": 0.0, "y": 0.0, "theta": 0.0}, source="STATE")

        self.assertTrue(req.active)
        self.assertAlmostEqual(req.desired_distance_m, 0.4)
        self.assertAlmostEqual(req.goal_x, 0.6)
        self.assertAlmostEqual(req.goal_y, 0.0)

    def test_receding_target_inside_standoff_gets_small_follow_nudge(self):
        layer = FollowLayer()
        obs = TargetObservation(
            source=TARGET_SOURCE_SIM_TARGET,
            frame=FRAME_WORLD,
            timestamp_s=time.time(),
            x=0.35,
            y=0.0,
            theta=0.0,
            vx=0.04,
            vy=0.0,
            desired_distance_m=0.4,
        )

        req = layer.tick(obs, {"x": 0.0, "y": 0.0, "theta": 0.0}, source="STATE")

        self.assertTrue(req.active)
        self.assertEqual(req.reason, "inside_follow_standoff_receding")
        self.assertGreater(req.goal_x, 0.03)
        self.assertLess(req.goal_x, 0.10)
        self.assertAlmostEqual(req.goal_y, 0.0)

    def test_approaching_target_inside_standoff_keeps_geometric_hold_goal(self):
        layer = FollowLayer()
        obs = TargetObservation(
            source=TARGET_SOURCE_SIM_TARGET,
            frame=FRAME_WORLD,
            timestamp_s=time.time(),
            x=0.35,
            y=0.0,
            theta=0.0,
            vx=-0.04,
            vy=0.0,
            desired_distance_m=0.4,
        )

        req = layer.tick(obs, {"x": 0.0, "y": 0.0, "theta": 0.0}, source="STATE")

        self.assertTrue(req.active)
        self.assertEqual(req.reason, "inside_follow_standoff")
        self.assertAlmostEqual(req.goal_x, 0.0)
        self.assertAlmostEqual(req.goal_y, 0.0)

    def test_stale_target_is_inactive(self):
        layer = FollowLayer(FollowLayerConfig(max_target_age_s=0.1))
        obs = TargetObservation(
            source=TARGET_SOURCE_SIM_TARGET,
            frame=FRAME_WORLD,
            timestamp_s=time.time() - 1.0,
            x=1.0,
            y=0.0,
        )

        req = layer.tick(obs, {"x": 0.0, "y": 0.0, "theta": 0.0}, source="STATE")

        self.assertFalse(req.active)
        self.assertTrue(req.stale)
        self.assertEqual(req.reason, "target_stale")

    def test_camera_angle_is_converted_to_left_positive_robot_bearing(self):
        ctrl = SimpleNamespace(
            _adaptive_target_dist_m=1.0,
            _adaptive_target_angle_deg=20.0,
            _adaptive_target_last_seen_ts=time.time(),
            _adaptive_target_confidence=1.0,
            _adaptive_target_vx_mps=None,
            _adaptive_target_vy_mps=None,
            _adaptive_target_desired_distance_m=0.0,
            follower_cfg={},
        )
        obs = camera_observation_from_controller(ctrl)
        layer = FollowLayer(FollowLayerConfig(camera_desired_distance_m=0.0))

        req = layer.tick(obs, {"x": 0.0, "y": 0.0, "theta": 0.0}, source="ADAPTIVE")

        self.assertTrue(req.active)
        self.assertGreater(req.target_x, 0.0)
        self.assertLess(req.target_y, 0.0)
        self.assertAlmostEqual(req.target_y, -math.sin(math.radians(20.0)), places=6)

    def test_rotated_camera_uses_vertical_fov_for_bearing_angle(self):
        rotated_angle = follower_module._bbox_center_to_angle_deg(
            112.5,
            180.0,
            image_height=320.0,
            rotation_deg=90,
        )
        unrotated_angle = follower_module._bbox_center_to_angle_deg(
            200.0,
            320.0,
            image_height=180.0,
            rotation_deg=0,
        )

        self.assertAlmostEqual(rotated_angle, 8.375, places=3)
        self.assertAlmostEqual(unrotated_angle, 12.75, places=2)
        self.assertLess(rotated_angle, unrotated_angle)

    def test_front_hold_hog_detection_marks_human_confirmed_target_id(self):
        ctrl = SimpleNamespace(
            _adaptive_target_dist_m=0.62,
            _adaptive_target_angle_deg=4.0,
            _adaptive_target_last_seen_ts=time.time(),
            _adaptive_target_confidence=0.8,
            _adaptive_target_vx_mps=None,
            _adaptive_target_vy_mps=None,
            _adaptive_target_desired_distance_m=1.0,
            _adaptive_follow_state="front_lidar_hold",
            _adaptive_target_camera_status={
                "detector": "opencv_hog",
                "target_usable": True,
            },
            follower_cfg={},
        )

        obs = camera_observation_from_controller(ctrl)

        self.assertIsNotNone(obs)
        self.assertEqual(obs.target_id, "camera_front_lidar_hold_human_confirmed")

    def test_front_hold_high_confidence_motion_blob_marks_human_confirmed_target_id(self):
        ctrl = SimpleNamespace(
            _adaptive_target_dist_m=0.54,
            _adaptive_target_angle_deg=3.0,
            _adaptive_target_last_seen_ts=time.time(),
            _adaptive_target_confidence=0.74,
            _adaptive_target_vx_mps=None,
            _adaptive_target_vy_mps=None,
            _adaptive_target_desired_distance_m=0.5,
            _adaptive_follow_state="front_lidar_hold",
            _adaptive_target_camera_status={
                "detector": "opencv_motion_blob",
                "detector_confidence": 0.70,
                "target_usable": True,
                "stale": False,
            },
            follower_cfg={},
        )

        obs = camera_observation_from_controller(ctrl)

        self.assertIsNotNone(obs)
        self.assertEqual(obs.target_id, "camera_front_lidar_hold_human_confirmed")

    def test_camera_target_dry_run_reaches_cruise_track_gate(self):
        now = time.time()
        ctrl = SimpleNamespace(
            _adaptive_target_dist_m=1.20,
            _adaptive_target_angle_deg=0.0,
            _adaptive_target_last_seen_ts=now,
            _adaptive_target_confidence=0.95,
            _adaptive_target_vx_mps=None,
            _adaptive_target_vy_mps=None,
            _adaptive_target_desired_distance_m=0.40,
            follower_cfg={"max_v_target": 0.08, "max_omega": 0.35},
        )

        obs = camera_observation_from_controller(ctrl, now_s=now)
        req = FollowLayer(FollowLayerConfig(camera_desired_distance_m=0.40)).tick(
            obs,
            {"x": 0.0, "y": 0.0, "theta": 0.0},
            source="ADAPTIVE",
            now_s=now,
        )
        result = CruiseLayer().tick(
            req,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 2.0, "avg_left": 1.2, "avg_right": 1.2, "latest_confidence": 0.9},
            raw_scan=[],
            source="ADAPTIVE",
            dt=0.02,
            track_width_m=0.175,
        )

        self.assertTrue(req.active)
        self.assertEqual(req.target_source, TARGET_SOURCE_CAMERA_TARGET)
        self.assertTrue(result.status["room_cruise_chain"])
        self.assertEqual(result.proposal["command_type"], "set_track_velocity")
        self.assertEqual(result.proposal["details"]["cruise_layer"]["source"], "ADAPTIVE")
        self.assertTrue(result.proposal["details"]["cruise_layer"]["local_planner_bypassed"])

    def test_camera_close_target_retreats_under_lidar_confidence_hold_when_rear_clear(self):
        now = time.time()
        obs = TargetObservation(
            source=TARGET_SOURCE_CAMERA_TARGET,
            frame="robot",
            timestamp_s=now,
            distance_m=0.84,
            bearing_rad=0.0,
            confidence=0.80,
            desired_distance_m=1.0,
            v_max_mps=0.04,
            omega_max_rad_s=0.18,
            target_id="camera_target",
        )
        req = FollowLayer(FollowLayerConfig(camera_desired_distance_m=1.0)).tick(
            obs,
            {"x": 0.0, "y": 0.0, "theta": 0.0},
            source="ADAPTIVE",
            now_s=now,
        )

        result = CruiseLayer().tick(
            req,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={
                "min_dist": 0.80,
                "min_dist_narrow": 0.80,
                "avg_left": 1.2,
                "avg_right": 1.2,
                "avg_back": 1.2,
                "latest_confidence": 0.10,
            },
            raw_scan=[],
            source="ADAPTIVE",
            dt=0.02,
            track_width_m=0.175,
        )

        room_cruise = result.status["room_cruise"]
        follow_gate = room_cruise["follow_gate"]
        tracks = room_cruise["track_reference"]
        self.assertEqual(room_cruise["phase"], "camera_target_close_retreat")
        self.assertTrue(follow_gate["camera_simple_close_retreat_candidate"])
        self.assertTrue(follow_gate["rear_clear_for_retreat"])
        self.assertLess(tracks["left_mps"], 0.0)
        self.assertLess(tracks["right_mps"], 0.0)

    def test_camera_target_alignment_is_aggressive_with_center_hysteresis(self):
        now = time.time()
        cruise = CruiseLayer()

        def run_at_bearing(bearing_rad):
            obs = TargetObservation(
                source=TARGET_SOURCE_CAMERA_TARGET,
                frame="robot",
                timestamp_s=now,
                distance_m=1.2,
                bearing_rad=float(bearing_rad),
                confidence=0.90,
                desired_distance_m=1.2,
                v_max_mps=0.08,
                omega_max_rad_s=0.80,
                target_id="camera_target",
            )
            req = FollowLayer(FollowLayerConfig(camera_desired_distance_m=1.2)).tick(
                obs,
                {"x": 0.0, "y": 0.0, "theta": 0.0},
                source="ADAPTIVE",
                now_s=now,
            )
            return cruise.tick(
                req,
                ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
                lidar_summary={"min_dist": 2.0, "min_dist_narrow": 2.0, "avg_left": 1.2, "avg_right": 1.2, "avg_back": 1.2},
                raw_scan=[],
                source="ADAPTIVE",
                dt=0.02,
                track_width_m=0.175,
            ).status["room_cruise"]

        far = run_at_bearing(0.30)
        self.assertEqual(far["phase"], "camera_target_one_track_align")
        self.assertGreater(float(far["track_reference"]["right_mps"]), 0.040)
        self.assertAlmostEqual(float(far["track_reference"]["left_mps"]), 0.0, places=5)

        inside_center_but_latched = run_at_bearing(0.14)
        self.assertEqual(inside_center_but_latched["phase"], "camera_target_one_track_align")
        self.assertTrue(inside_center_but_latched["follow_gate"]["camera_target_align_latched"])
        self.assertLess(float(inside_center_but_latched["track_reference"]["right_mps"]), float(far["track_reference"]["right_mps"]))

        centered = run_at_bearing(0.08)
        self.assertEqual(centered["phase"], "camera_target_center_hold")
        self.assertFalse(centered["follow_gate"]["camera_target_align_latched"])

    def test_camera_target_zone_overrides_angle_hysteresis_for_center_third(self):
        now = time.time()
        cruise = CruiseLayer()

        def run_with_zone(zone, bearing_rad):
            obs = TargetObservation(
                source=TARGET_SOURCE_CAMERA_TARGET,
                frame="robot",
                timestamp_s=now,
                distance_m=1.2,
                bearing_rad=float(bearing_rad),
                confidence=0.90,
                desired_distance_m=1.2,
                v_max_mps=0.08,
                omega_max_rad_s=0.80,
                target_id="camera_target",
                target_zone=str(zone),
            )
            req = FollowLayer(FollowLayerConfig(camera_desired_distance_m=1.2)).tick(
                obs,
                {"x": 0.0, "y": 0.0, "theta": 0.0},
                source="ADAPTIVE",
                now_s=now,
            )
            return cruise.tick(
                req,
                ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
                lidar_summary={"min_dist": 2.0, "min_dist_narrow": 2.0, "avg_left": 1.2, "avg_right": 1.2, "avg_back": 1.2},
                raw_scan=[],
                source="ADAPTIVE",
                dt=0.02,
                track_width_m=0.175,
            ).status["room_cruise"]

        center = run_with_zone("center", 0.13)
        self.assertEqual(center["phase"], "camera_target_center_hold")
        self.assertEqual(center["follow_gate"]["camera_target_zone"], "center")
        self.assertFalse(center["follow_gate"]["camera_target_align_latched"])

        left = run_with_zone("left", 0.08)
        self.assertEqual(left["phase"], "camera_target_one_track_align")
        self.assertEqual(left["follow_gate"]["camera_target_turn_side"], "left")
        self.assertGreater(float(left["track_reference"]["right_mps"]), 0.0)
        self.assertAlmostEqual(float(left["track_reference"]["left_mps"]), 0.0, places=5)

    def test_camera_direction_only_uses_in_place_align(self):
        now = time.time()
        obs = TargetObservation(
            source=TARGET_SOURCE_CAMERA_TARGET,
            frame="robot",
            timestamp_s=now,
            distance_m=1.2,
            bearing_rad=0.12,
            confidence=0.90,
            desired_distance_m=2.5,
            v_max_mps=0.08,
            omega_max_rad_s=0.80,
            target_id="camera_target",
            target_zone="left",
        )
        req = FollowLayer(FollowLayerConfig(camera_desired_distance_m=2.5)).tick(
            obs,
            {"x": 0.0, "y": 0.0, "theta": 0.0},
            source="ADAPTIVE",
            now_s=now,
        )

        room = CruiseLayer().tick(
            req,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist": 2.0, "min_dist_narrow": 2.0, "avg_left": 1.2, "avg_right": 1.2, "avg_back": 1.2},
            raw_scan=[],
            source="ADAPTIVE",
            dt=0.02,
            track_width_m=0.175,
        ).status["room_cruise"]

        self.assertEqual(room["phase"], "camera_target_in_place_align")
        self.assertLess(float(room["track_reference"]["left_mps"]), 0.0)
        self.assertGreater(float(room["track_reference"]["right_mps"]), 0.0)
        self.assertAlmostEqual(
            float(room["track_reference"]["left_mps"]) + float(room["track_reference"]["right_mps"]),
            0.0,
            places=5,
        )

    def test_camera_target_search_uses_reduced_turn_speed(self):
        now = time.time()
        ctrl = SimpleNamespace(
            _adaptive_target_search_active=True,
            _adaptive_target_search_side="right",
            follower_cfg={"max_v_target": 0.08, "max_omega": 0.80},
        )
        obs = camera_observation_from_controller(ctrl, now_s=now)
        req = FollowLayer().tick(obs, {"x": 0.0, "y": 0.0, "theta": 0.0}, source="ADAPTIVE", now_s=now)

        self.assertLess(req.goal_theta, 0.0)
        self.assertEqual(req.target_zone, "right")
        result = CruiseLayer().tick(
            req,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist": 2.0, "min_dist_narrow": 2.0, "avg_left": 1.2, "avg_right": 1.2, "avg_back": 1.2},
            raw_scan=[],
            source="ADAPTIVE",
            dt=0.02,
            track_width_m=0.175,
        )

        room = result.status["room_cruise"]
        self.assertEqual(room["phase"], "target_search_one_track")
        self.assertAlmostEqual(float(room["track_reference"]["left_mps"]), 0.020, places=4)
        self.assertAlmostEqual(float(room["track_reference"]["right_mps"]), 0.0, places=5)

    def test_camera_target_observation_caps_search_pivot_speed_separately(self):
        now = time.time()
        target_ctrl = SimpleNamespace(
            _adaptive_target_dist_m=1.20,
            _adaptive_target_angle_deg=32.0,
            _adaptive_target_last_seen_ts=now,
            _adaptive_target_confidence=0.95,
            _adaptive_target_vx_mps=None,
            _adaptive_target_vy_mps=None,
            _adaptive_target_desired_distance_m=1.0,
            follower_cfg={"max_v_target": 0.08, "max_omega": 0.80},
        )
        search_ctrl = SimpleNamespace(
            _adaptive_target_search_active=True,
            _adaptive_target_search_side="right",
            follower_cfg={"max_v_target": 0.08, "max_omega": 0.80},
        )

        target_obs = camera_observation_from_controller(target_ctrl, now_s=now)
        search_obs = camera_observation_from_controller(search_ctrl, now_s=now)

        self.assertIsNotNone(target_obs)
        self.assertIsNotNone(search_obs)
        self.assertAlmostEqual(target_obs.omega_max_rad_s, 0.42)
        self.assertAlmostEqual(search_obs.omega_max_rad_s, 0.08)
        self.assertLess(search_obs.bearing_rad, 0.0)

    def test_camera_direction_only_search_uses_in_place_turn(self):
        now = time.time()
        ctrl = SimpleNamespace(
            _adaptive_target_search_active=True,
            _adaptive_target_search_side="right",
            follower_cfg={"target_distance_m": 2.5, "max_v_target": 0.08, "max_omega": 0.80},
        )
        obs = camera_observation_from_controller(ctrl, now_s=now)
        self.assertEqual(obs.target_id, "camera_target_search_right_direction_only")
        self.assertEqual(obs.target_zone, "right")
        req = FollowLayer().tick(obs, {"x": 0.0, "y": 0.0, "theta": 0.0}, source="ADAPTIVE", now_s=now)
        self.assertLess(req.goal_theta, 0.0)

        room = CruiseLayer().tick(
            req,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist": 2.0, "min_dist_narrow": 2.0, "avg_left": 1.2, "avg_right": 1.2, "avg_back": 1.2},
            raw_scan=[],
            source="ADAPTIVE",
            dt=0.02,
            track_width_m=0.175,
        ).status["room_cruise"]

        self.assertEqual(room["phase"], "target_search_in_place")
        self.assertGreater(float(room["track_reference"]["left_mps"]), 0.0)
        self.assertLess(float(room["track_reference"]["right_mps"]), 0.0)
        self.assertAlmostEqual(
            float(room["track_reference"]["left_mps"]) + float(room["track_reference"]["right_mps"]),
            0.0,
            places=5,
        )

    def test_camera_follow_global_half_meter_clearance_is_hard_gate(self):
        now = time.time()
        obs = TargetObservation(
            source=TARGET_SOURCE_CAMERA_TARGET,
            frame="robot",
            timestamp_s=now,
            distance_m=1.5,
            bearing_rad=0.0,
            confidence=0.90,
            desired_distance_m=1.2,
            v_max_mps=0.08,
            omega_max_rad_s=0.80,
            target_id="camera_target",
        )
        req = FollowLayer(FollowLayerConfig(camera_desired_distance_m=1.2)).tick(
            obs,
            {"x": 0.0, "y": 0.0, "theta": 0.0},
            source="ADAPTIVE",
            now_s=now,
        )

        result = CruiseLayer().tick(
            req,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist": 0.49, "min_dist_narrow": 2.0, "avg_left": 1.2, "avg_right": 1.2, "avg_back": 1.2},
            raw_scan=[],
            source="ADAPTIVE",
            dt=0.02,
            track_width_m=0.175,
        )

        room = result.status["room_cruise"]
        self.assertEqual(room["phase"], "obstacle_stop_hold")
        self.assertTrue(room["follow_gate"]["global_clearance_hard_gate"])
        self.assertFalse(room["follow_gate"]["global_clear_for_retreat"])
        self.assertAlmostEqual(float(room["track_reference"]["left_mps"]), 0.0, places=6)
        self.assertAlmostEqual(float(room["track_reference"]["right_mps"]), 0.0, places=6)

    def test_camera_follow_prefers_target_forward_above_half_meter_wall_buffer(self):
        now = time.time()
        obs = TargetObservation(
            source=TARGET_SOURCE_CAMERA_TARGET,
            frame="robot",
            timestamp_s=now,
            distance_m=1.5,
            bearing_rad=0.0,
            confidence=0.90,
            desired_distance_m=1.2,
            v_max_mps=0.08,
            omega_max_rad_s=0.80,
            target_id="camera_target",
        )
        req = FollowLayer(FollowLayerConfig(camera_desired_distance_m=1.2)).tick(
            obs,
            {"x": 0.0, "y": 0.0, "theta": 0.0},
            source="ADAPTIVE",
            now_s=now,
        )

        result = CruiseLayer().tick(
            req,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist": 0.90, "min_dist_narrow": 0.90, "avg_left": 1.2, "avg_right": 1.2, "avg_back": 1.2},
            raw_scan=[],
            source="ADAPTIVE",
            dt=0.02,
            track_width_m=0.175,
        )

        room = result.status["room_cruise"]
        self.assertEqual(room["phase"], "camera_target_center_forward")
        self.assertGreater(float(room["track_reference"]["left_mps"]), 0.0)
        self.assertGreater(float(room["track_reference"]["right_mps"]), 0.0)

    def test_camera_observation_applies_follow_speed_scale_to_motion_limits(self):
        now = time.time()
        ctrl = SimpleNamespace(
            _adaptive_target_dist_m=1.20,
            _adaptive_target_angle_deg=0.0,
            _adaptive_target_last_seen_ts=now,
            _adaptive_target_confidence=0.95,
            _adaptive_target_vx_mps=None,
            _adaptive_target_vy_mps=None,
            _adaptive_target_desired_distance_m=1.0,
            follow_speed_scale=0.5,
            follower_cfg={"max_v_target": 0.08, "max_omega": 0.36},
        )

        obs = camera_observation_from_controller(ctrl, now_s=now)

        self.assertIsNotNone(obs)
        self.assertAlmostEqual(obs.v_max_mps, 0.04)
        self.assertAlmostEqual(obs.omega_max_rad_s, 0.18)

    def test_follow_speed_scale_command_accepts_only_non_amplifying_limits(self):
        import controller.commands as commands

        ctrl = SimpleNamespace(last_motion_denied_reason="")

        self.assertTrue(commands.set_follow_speed_scale(ctrl, 0.5, source="GUI"))
        self.assertAlmostEqual(ctrl.follow_speed_scale, 0.5)
        self.assertEqual(ctrl.follow_speed_scale_status["source"], "GUI")
        self.assertFalse(commands.set_follow_speed_scale(ctrl, 1.2, source="GUI"))
        self.assertEqual(ctrl.last_motion_denied_reason, "invalid_follow_speed_scale")

    def test_follow_search_pivot_omega_command_accepts_tuning_range(self):
        import controller.commands as commands

        ctrl = SimpleNamespace(last_motion_denied_reason="")

        self.assertTrue(commands.set_follow_search_pivot_omega(ctrl, 0.02, source="GUI"))
        self.assertAlmostEqual(ctrl.follow_search_pivot_omega_rad_s, 0.02)
        self.assertEqual(ctrl.follow_search_pivot_omega_status["source"], "GUI")
        self.assertFalse(commands.set_follow_search_pivot_omega(ctrl, 0.35, source="GUI"))
        self.assertEqual(ctrl.last_motion_denied_reason, "invalid_follow_search_pivot_omega")

    def test_candidate_hold_publishes_v2_reacquire_hold_without_track_reference(self):
        ctrl = SimpleNamespace(
            follower_cfg={"target_distance_m": 1.0, "stop_distance_m": 0.7},
            logger=SimpleNamespace(warn=lambda *args, **kwargs: None),
        )

        follower_module._publish_follow_uncertain_hold(
            ctrl,
            SimpleNamespace(timestamp=time.monotonic()),
            {
                "state": "candidate_unconfirmed",
                "source": "camera",
                "target_visible": True,
                "target_usable": False,
                "stale": False,
                "detector": "opencv_template_lock",
                "detector_confidence": 0.61,
            },
            reason="candidate_unconfirmed",
        )

        self.assertFalse(hasattr(ctrl, "requested_track_reference"))
        self.assertAlmostEqual(ctrl._adaptive_target_dist_m, 1.0)
        self.assertAlmostEqual(ctrl._adaptive_target_angle_deg, 0.0)
        self.assertEqual(ctrl._adaptive_follow_state, "target_reacquire_hold")
        self.assertEqual(ctrl._adaptive_target_camera_status["state"], "candidate_hold")
        self.assertEqual(ctrl._adaptive_zero_track_hold_reason, "candidate_unconfirmed")
        obs = camera_observation_from_controller(ctrl)
        self.assertIsNotNone(obs)
        self.assertEqual(obs.source, TARGET_SOURCE_CAMERA_TARGET)
        self.assertEqual(obs.target_id, "camera_target_reacquire")

    def test_follow_distance_command_updates_target_and_inner_stop_distance(self):
        import controller.commands as commands

        ctrl = SimpleNamespace(last_motion_denied_reason="", follower_cfg={"target_distance_m": 1.0, "stop_distance_m": 0.7})

        self.assertTrue(commands.set_follow_distance(ctrl, 0.5, source="GUI"))
        self.assertAlmostEqual(ctrl.follower_cfg["target_distance_m"], 0.5)
        self.assertAlmostEqual(ctrl.follower_cfg["stop_distance_m"], 0.35)
        self.assertAlmostEqual(ctrl._adaptive_target_desired_distance_m, 0.5)
        self.assertEqual(ctrl.follow_distance_status["source"], "GUI")
        self.assertFalse(commands.set_follow_distance(ctrl, 0.2, source="GUI"))
        self.assertEqual(ctrl.last_motion_denied_reason, "invalid_follow_distance")

    def test_follower_tick_publishes_target_without_direct_motion_command(self):
        class DummyCamera:
            too_many_failures = False

            def detect_with_persistence(self, ctrl):
                return (160.0, 100.0, 320.0, 0.9)

            def last_status(self):
                return {
                    "state": "detected",
                    "source": "camera",
                    "stale": False,
                    "target_visible": True,
                    "target_usable": True,
                    "frame_ok": True,
                }

        ctrl = SimpleNamespace(
            following_active=True,
            v_target=0.21,
            omega_target=0.12,
            follower_camera=DummyCamera(),
            follower_cfg={"target_distance_m": 1.2, "stop_distance_m": 0.8},
            logger=SimpleNamespace(warn=lambda *args, **kwargs: None),
        )
        ctrl._emergency_stop = lambda reason="": setattr(ctrl, "emergency_reason", reason)
        snap = SimpleNamespace(
            timestamp=time.monotonic(),
            raw_scan=[
                {"angle": 359.0, "dist": 1390.0},
                {"angle": 0.0, "dist": 1400.0},
                {"angle": 1.0, "dist": 1410.0},
            ],
        )

        command = follower_module.tick(ctrl, snap)

        self.assertEqual(command, (0.0, 0.0))
        self.assertAlmostEqual(ctrl.v_target, 0.21)
        self.assertAlmostEqual(ctrl.omega_target, 0.12)
        self.assertAlmostEqual(float(ctrl._adaptive_target_dist_m), 1.40, places=2)
        self.assertAlmostEqual(float(ctrl._adaptive_target_angle_deg), 0.0, places=2)
        obs = camera_observation_from_controller(ctrl)
        self.assertIsNotNone(obs)
        self.assertEqual(obs.source, TARGET_SOURCE_CAMERA_TARGET)

    def test_follower_side_lidar_close_point_does_not_trip_front_emergency(self):
        class DummyCamera:
            too_many_failures = False

            def detect_with_persistence(self, ctrl):
                return (160.0, 100.0, 320.0, 0.8)

            def last_status(self):
                return {
                    "state": "ok",
                    "source": "camera",
                    "stale": False,
                    "target_visible": True,
                    "target_usable": True,
                    "frame_ok": True,
                }

        ctrl = SimpleNamespace(
            following_active=True,
            follower_camera=DummyCamera(),
            follower_cfg={"target_distance_m": 1.0, "stop_distance_m": 0.7},
            logger=SimpleNamespace(warn=lambda *args, **kwargs: None),
        )
        ctrl._emergency_stop = lambda reason="": setattr(ctrl, "emergency_reason", reason)
        snap = SimpleNamespace(
            timestamp=time.monotonic(),
            raw_scan=[
                {"angle": 90.0, "dist": 300.0},
                {"angle": 0.0, "dist": 1500.0},
                {"angle": 1.0, "dist": 1510.0},
                {"angle": 359.0, "dist": 1490.0},
            ],
        )

        command = follower_module.tick(ctrl, snap)

        self.assertEqual(command, (0.0, 0.0))
        self.assertFalse(hasattr(ctrl, "emergency_reason"))
        self.assertTrue(ctrl.following_active)
        self.assertAlmostEqual(float(ctrl._adaptive_target_dist_m), 1.5, places=1)

    def test_camera_lidar_distance_blend_leans_on_lidar_for_standoff(self):
        class DummyCamera:
            too_many_failures = False

            def detect_with_persistence(self, ctrl):
                return (160.0, 100.0, 320.0, 0.85)

            def last_status(self):
                return {
                    "state": "ok",
                    "source": "camera",
                    "stale": False,
                    "target_visible": True,
                    "target_usable": True,
                    "frame_ok": True,
                    "detector": "onnx_yolov5_person",
                    "detector_confidence": 0.85,
                    "distance_estimate_m": 1.60,
                    "distance_confidence": 0.70,
                }

        ctrl = SimpleNamespace(
            following_active=True,
            follower_camera=DummyCamera(),
            follower_cfg={"target_distance_m": 1.0, "stop_distance_m": 0.7},
            logger=SimpleNamespace(warn=lambda *args, **kwargs: None),
        )
        ctrl._emergency_stop = lambda reason="": setattr(ctrl, "emergency_reason", reason)
        snap = SimpleNamespace(
            timestamp=time.monotonic(),
            raw_scan=[
                {"angle": 359.0, "dist": 1200.0},
                {"angle": 0.0, "dist": 1200.0},
                {"angle": 1.0, "dist": 1200.0},
            ],
        )

        follower_module.tick(ctrl, snap)

        self.assertFalse(hasattr(ctrl, "emergency_reason"))
        self.assertEqual(ctrl._adaptive_target_camera_status["distance_source"], "camera_lidar_blend")
        self.assertAlmostEqual(float(ctrl._adaptive_target_camera_status["distance_used_m"]), 1.24, places=2)
        self.assertAlmostEqual(float(ctrl._adaptive_target_dist_m), 1.24, places=2)

    def test_motion_blob_distance_uses_lidar_guard_when_camera_bbox_jumps_far(self):
        class DummyCamera:
            too_many_failures = False

            def detect_with_persistence(self, ctrl):
                return (160.0, 100.0, 320.0, 0.44)

            def last_status(self):
                return {
                    "state": "ok",
                    "source": "camera",
                    "stale": False,
                    "target_visible": True,
                    "target_usable": True,
                    "frame_ok": True,
                    "detector": "opencv_motion_blob",
                    "detector_confidence": 0.44,
                    "distance_estimate_m": 2.60,
                    "distance_confidence": 0.15,
                    "distance_source": "camera_bbox",
                }

        ctrl = SimpleNamespace(
            following_active=True,
            follower_camera=DummyCamera(),
            follower_cfg={"target_distance_m": 1.0, "stop_distance_m": 0.7},
            logger=SimpleNamespace(warn=lambda *args, **kwargs: None),
        )
        ctrl._emergency_stop = lambda reason="": setattr(ctrl, "emergency_reason", reason)
        snap = SimpleNamespace(
            timestamp=time.monotonic(),
            raw_scan=[
                {"angle": 359.0, "dist": 1740.0},
                {"angle": 0.0, "dist": 1750.0},
                {"angle": 1.0, "dist": 1760.0},
            ],
        )

        follower_module.tick(ctrl, snap)

        self.assertFalse(hasattr(ctrl, "emergency_reason"))
        self.assertEqual(ctrl._adaptive_target_camera_status["distance_source"], "motion_blob_lidar_guard")
        self.assertAlmostEqual(float(ctrl._adaptive_target_camera_status["distance_measurement_used_m"]), 1.75, places=2)
        self.assertAlmostEqual(float(ctrl._adaptive_target_camera_status["distance_used_m"]), 1.75, places=2)
        self.assertAlmostEqual(float(ctrl._adaptive_target_dist_m), 1.75, places=2)

    def test_hog_room_bubble_uses_front_lidar_as_confirmed_human_distance(self):
        class DummyCamera:
            too_many_failures = False

            def detect_with_persistence(self, ctrl):
                return (160.0, 100.0, 320.0, 0.82)

            def last_status(self):
                return {
                    "state": "ok",
                    "source": "camera",
                    "stale": False,
                    "target_visible": True,
                    "target_usable": True,
                    "frame_ok": True,
                    "detector": "onnx_yolov5_person",
                    "detector_confidence": 0.82,
                    "distance_estimate_m": 1.62,
                    "distance_confidence": 0.62,
                    "distance_source": "camera_bbox",
                    "bbox_height_ratio": 0.74,
                }

        ctrl = SimpleNamespace(
            following_active=True,
            follower_camera=DummyCamera(),
            follower_cfg={"target_distance_m": 1.0, "stop_distance_m": 0.7},
            logger=SimpleNamespace(warn=lambda *args, **kwargs: None),
        )
        ctrl._emergency_stop = lambda reason="": setattr(ctrl, "emergency_reason", reason)
        snap = SimpleNamespace(
            timestamp=time.monotonic(),
            raw_scan=[
                {"angle": 359.0, "dist": 850.0},
                {"angle": 0.0, "dist": 860.0},
                {"angle": 1.0, "dist": 870.0},
            ],
        )

        follower_module.tick(ctrl, snap)

        self.assertFalse(hasattr(ctrl, "emergency_reason"))
        self.assertEqual(
            ctrl._adaptive_target_camera_status["distance_source"],
            "front_lidar_room_bubble_camera_confirmed",
        )
        self.assertAlmostEqual(float(ctrl._adaptive_target_camera_status["distance_used_m"]), 0.85, places=2)
        self.assertAlmostEqual(float(ctrl._adaptive_target_dist_m), 0.85, places=2)
        obs = camera_observation_from_controller(ctrl)
        self.assertIsNotNone(obs)
        self.assertEqual(obs.target_id, "camera_front_lidar_hold_human_confirmed")

    def test_room_follow_keeps_far_camera_distance_when_bbox_is_not_close(self):
        class DummyCamera:
            too_many_failures = False

            def detect_with_persistence(self, ctrl):
                return (160.0, 100.0, 320.0, 0.86)

            def last_status(self):
                return {
                    "state": "ok",
                    "source": "camera",
                    "stale": False,
                    "target_visible": True,
                    "target_usable": True,
                    "frame_ok": True,
                    "detector": "onnx_yolov5_person",
                    "detector_confidence": 0.86,
                    "distance_estimate_m": 1.49,
                    "distance_confidence": 0.70,
                    "distance_source": "camera_bbox",
                    "bbox_height_ratio": 0.46,
                }

        ctrl = SimpleNamespace(
            following_active=True,
            follower_camera=DummyCamera(),
            follower_cfg={"target_distance_m": 1.2, "stop_distance_m": 0.7},
            logger=SimpleNamespace(warn=lambda *args, **kwargs: None),
        )
        ctrl._emergency_stop = lambda reason="": setattr(ctrl, "emergency_reason", reason)
        snap = SimpleNamespace(
            timestamp=time.monotonic(),
            raw_scan=[
                {"angle": 359.0, "dist": 789.0},
                {"angle": 0.0, "dist": 790.0},
                {"angle": 1.0, "dist": 792.0},
            ],
        )

        follower_module.tick(ctrl, snap)

        self.assertFalse(hasattr(ctrl, "emergency_reason"))
        self.assertEqual(ctrl._adaptive_target_camera_status["distance_source"], "camera_bbox")
        self.assertAlmostEqual(float(ctrl._adaptive_target_dist_m), 1.49, places=2)

    def test_close_half_meter_follow_treats_far_camera_bbox_as_front_obstacle(self):
        class DummyCamera:
            too_many_failures = False

            def detect_with_persistence(self, ctrl):
                return (160.0, 100.0, 320.0, 0.75)

            def last_status(self):
                return {
                    "state": "ok",
                    "source": "camera",
                    "stale": False,
                    "target_visible": True,
                    "target_usable": True,
                    "frame_ok": True,
                    "detector": "opencv_hog",
                    "detector_confidence": 0.75,
                    "distance_estimate_m": 1.12,
                    "distance_confidence": 0.68,
                }

        ctrl = SimpleNamespace(
            following_active=True,
            follower_camera=DummyCamera(),
            follower_cfg={"target_distance_m": 0.5, "stop_distance_m": 0.35},
            logger=SimpleNamespace(warn=lambda *args, **kwargs: None),
        )
        ctrl._emergency_stop = lambda reason="": setattr(ctrl, "emergency_reason", reason)
        snap = SimpleNamespace(
            timestamp=time.monotonic(),
            raw_scan=[
                {"angle": 359.0, "dist": 550.0},
                {"angle": 0.0, "dist": 552.0},
                {"angle": 1.0, "dist": 558.0},
            ],
        )

        command = follower_module.tick(ctrl, snap)

        self.assertEqual(command, (0.0, 0.0))
        self.assertFalse(hasattr(ctrl, "emergency_reason"))
        self.assertAlmostEqual(float(ctrl._adaptive_target_dist_m), 1.12, places=3)
        self.assertEqual(
            ctrl._adaptive_target_camera_status["gate"],
            "front_lidar_obstacle_arbitrated_by_camera_distance",
        )
        self.assertEqual(
            ctrl._adaptive_target_camera_status["distance_source"],
            "camera_bbox_front_obstacle_arbitrated",
        )
        self.assertAlmostEqual(float(ctrl._adaptive_target_camera_status["front_obstacle_distance_m"]), 0.55, places=3)

    def test_close_half_meter_front_hold_keeps_far_camera_target_when_obstacle_split(self):
        class DummyCamera:
            too_many_failures = False

            def detect_with_persistence(self, ctrl):
                return (160.0, 100.0, 320.0, 0.75)

            def last_status(self):
                return {
                    "state": "ok",
                    "source": "camera",
                    "stale": False,
                    "target_visible": True,
                    "target_usable": True,
                    "frame_ok": True,
                    "detector": "opencv_hog",
                    "detector_confidence": 0.75,
                    "distance_estimate_m": 1.12,
                    "distance_confidence": 0.68,
                }

        tracker = TargetKinematicsTracker()
        tracker.observe(1.20, 0.0, confidence=0.7)
        ctrl = SimpleNamespace(
            following_active=True,
            follower_camera=DummyCamera(),
            follower_target_tracker=tracker,
            follower_cfg={"target_distance_m": 0.5, "stop_distance_m": 0.35},
            logger=SimpleNamespace(warn=lambda *args, **kwargs: None),
        )
        ctrl._emergency_stop = lambda reason="": setattr(ctrl, "emergency_reason", reason)
        snap = SimpleNamespace(
            timestamp=time.monotonic(),
            raw_scan=[
                {"angle": 359.0, "dist": 550.0},
                {"angle": 0.0, "dist": 552.0},
                {"angle": 1.0, "dist": 558.0},
            ],
        )

        follower_module.tick(ctrl, snap)

        self.assertFalse(hasattr(ctrl, "emergency_reason"))
        self.assertAlmostEqual(float(ctrl._adaptive_target_dist_m), 1.12, places=3)
        self.assertAlmostEqual(float(tracker.snapshot()["dist_m"]), 1.12, places=3)
        self.assertEqual(
            ctrl._adaptive_target_camera_status["gate"],
            "front_lidar_obstacle_arbitrated_by_camera_distance",
        )
        self.assertEqual(
            ctrl._adaptive_target_camera_status["distance_source"],
            "camera_bbox_front_obstacle_arbitrated",
        )

    def test_front_lidar_min_ignores_single_close_outlier_when_cluster_exists(self):
        snap = SimpleNamespace(
            timestamp=time.monotonic(),
            raw_scan=[
                {"angle": 0.0, "dist": 212.0},
                {"angle": 2.0, "dist": 594.0},
                {"angle": 358.0, "dist": 598.0},
                {"angle": 90.0, "dist": 180.0},
            ],
        )

        self.assertAlmostEqual(_get_lidar_min_dist_front(snap), 0.594, places=3)

    def test_close_half_meter_lidar_obstacle_does_not_replace_far_camera_target(self):
        class DummyCamera:
            too_many_failures = False

            def detect_with_persistence(self, ctrl):
                return (160.0, 100.0, 320.0, 0.75)

            def last_status(self):
                return {
                    "state": "ok",
                    "source": "camera",
                    "stale": False,
                    "target_visible": True,
                    "target_usable": True,
                    "frame_ok": True,
                    "detector": "opencv_motion_blob",
                    "detector_confidence": 0.75,
                    "distance_estimate_m": 3.20,
                    "distance_confidence": 0.40,
                }

        tracker = TargetKinematicsTracker()
        tracker.observe(1.20, 0.0, confidence=0.7)
        ctrl = SimpleNamespace(
            following_active=True,
            follower_camera=DummyCamera(),
            follower_target_tracker=tracker,
            follower_cfg={"target_distance_m": 0.5, "stop_distance_m": 0.35},
            logger=SimpleNamespace(warn=lambda *args, **kwargs: None),
        )
        ctrl._emergency_stop = lambda reason="": setattr(ctrl, "emergency_reason", reason)
        snap = SimpleNamespace(
            timestamp=time.monotonic(),
            raw_scan=[
                {"angle": 359.0, "dist": 658.0},
                {"angle": 0.0, "dist": 660.0},
                {"angle": 1.0, "dist": 662.0},
            ],
        )

        command = follower_module.tick(ctrl, snap)

        self.assertEqual(command, (0.0, 0.0))
        self.assertFalse(hasattr(ctrl, "emergency_reason"))
        self.assertAlmostEqual(float(ctrl._adaptive_target_dist_m), 3.20, places=3)
        self.assertEqual(
            ctrl._adaptive_target_camera_status["gate"],
            "front_lidar_obstacle_arbitrated_by_camera_distance",
        )
        self.assertEqual(
            ctrl._adaptive_target_camera_status["distance_source"],
            "camera_bbox_front_obstacle_arbitrated",
        )

    def test_close_half_meter_nonhold_lidar_does_not_replace_far_camera_target(self):
        class DummyCamera:
            too_many_failures = False

            def detect_with_persistence(self, ctrl):
                return (160.0, 100.0, 320.0, 0.75)

            def last_status(self):
                return {
                    "state": "ok",
                    "source": "camera",
                    "stale": False,
                    "target_visible": True,
                    "target_usable": True,
                    "frame_ok": True,
                    "detector": "opencv_motion_blob",
                    "detector_confidence": 0.75,
                    "distance_estimate_m": 1.70,
                    "distance_confidence": 0.42,
                }

        tracker = TargetKinematicsTracker()
        tracker.observe(1.20, 0.0, confidence=0.7)
        ctrl = SimpleNamespace(
            following_active=True,
            follower_camera=DummyCamera(),
            follower_target_tracker=tracker,
            follower_cfg={"target_distance_m": 0.5, "stop_distance_m": 0.35},
            logger=SimpleNamespace(warn=lambda *args, **kwargs: None),
        )
        ctrl._emergency_stop = lambda reason="": setattr(ctrl, "emergency_reason", reason)
        snap = SimpleNamespace(
            timestamp=time.monotonic(),
            raw_scan=[
                {"angle": 359.0, "dist": 740.0},
                {"angle": 0.0, "dist": 742.0},
                {"angle": 1.0, "dist": 744.0},
            ],
        )

        follower_module.tick(ctrl, snap)

        self.assertFalse(hasattr(ctrl, "emergency_reason"))
        self.assertGreater(float(ctrl._adaptive_target_dist_m), 1.0)
        self.assertEqual(
            ctrl._adaptive_target_camera_status["distance_source"],
            "camera_bbox",
        )

    def test_follower_front_warning_hold_pauses_without_emergency(self):
        class DummyCamera:
            too_many_failures = False

            def detect_with_persistence(self, ctrl):
                return (200.0, 100.0, 320.0, 0.8)

            def last_status(self):
                return {
                    "state": "ok",
                    "source": "camera",
                    "stale": False,
                    "target_visible": True,
                    "target_usable": True,
                    "frame_ok": True,
                    "rotation_deg": 0,
                    "image_height_px": 180,
                    "detector": "test",
                    "detector_confidence": 0.8,
                }

        ctrl = SimpleNamespace(
            following_active=True,
            follower_camera=DummyCamera(),
            follower_cfg={"target_distance_m": 1.0, "stop_distance_m": 0.7},
            logger=SimpleNamespace(warn=lambda *args, **kwargs: None),
        )
        ctrl._emergency_stop = lambda reason="": setattr(ctrl, "emergency_reason", reason)
        snap = SimpleNamespace(
            timestamp=time.monotonic(),
            raw_scan=[
                {"angle": 0.0, "dist": 600.0},
                {"angle": 1.0, "dist": 610.0},
            ],
        )

        command = follower_module.tick(ctrl, snap)

        self.assertEqual(command, (0.0, 0.0))
        self.assertFalse(hasattr(ctrl, "emergency_reason"))
        self.assertTrue(ctrl.following_active)
        self.assertEqual(ctrl._adaptive_follow_state, "front_lidar_hold")
        self.assertAlmostEqual(float(ctrl._adaptive_target_dist_m), 0.60)
        self.assertAlmostEqual(float(ctrl._adaptive_target_angle_deg), 12.75)
        self.assertEqual(ctrl._adaptive_target_lidar_status["state"], "front_hold")
        self.assertEqual(ctrl._adaptive_target_camera_status["raw_state"], "ok")
        self.assertEqual(ctrl._adaptive_target_camera_status["rotation_deg"], 0)

    def test_follower_front_warning_uses_camera_distance_when_lidar_is_obstacle_not_target(self):
        class DummyCamera:
            too_many_failures = False

            def detect_with_persistence(self, ctrl):
                return (160.0, 100.0, 320.0, 0.8)

            def last_status(self):
                return {
                    "state": "ok",
                    "source": "camera",
                    "stale": False,
                    "target_visible": True,
                    "target_usable": True,
                    "frame_ok": True,
                    "rotation_deg": 0,
                    "detector": "opencv_hog",
                    "detector_confidence": 0.8,
                    "distance_estimate_m": 1.45,
                    "distance_confidence": 0.70,
                    "distance_source": "camera_bbox",
                }

        ctrl = SimpleNamespace(
            following_active=True,
            follower_camera=DummyCamera(),
            follower_cfg={"target_distance_m": 1.0, "stop_distance_m": 0.7},
            logger=SimpleNamespace(warn=lambda *args, **kwargs: None),
        )
        ctrl._emergency_stop = lambda reason="": setattr(ctrl, "emergency_reason", reason)
        snap = SimpleNamespace(
            timestamp=time.monotonic(),
            raw_scan=[
                {"angle": 0.0, "dist": 700.0},
                {"angle": 1.0, "dist": 710.0},
            ],
        )

        command = follower_module.tick(ctrl, snap)

        self.assertEqual(command, (0.0, 0.0))
        self.assertFalse(hasattr(ctrl, "emergency_reason"))
        self.assertGreater(float(ctrl._adaptive_target_dist_m), 1.30)
        self.assertEqual(ctrl._adaptive_target_lidar_status["state"], "front_obstacle_arbitrated")
        self.assertEqual(
            ctrl._adaptive_target_camera_status["gate"],
            "front_lidar_obstacle_arbitrated_by_camera_distance",
        )
        obs = camera_observation_from_controller(ctrl)
        self.assertIsNotNone(obs)
        self.assertEqual(obs.target_id, "camera_front_obstacle_arbitrated")
        self.assertAlmostEqual(obs.front_obstacle_distance_m, 0.70, places=2)
        self.assertEqual(ctrl._adaptive_follow_state, "approach")

    def test_follower_front_lidar_close_point_trips_emergency(self):
        class DummyCamera:
            too_many_failures = False

            def detect_with_persistence(self, ctrl):
                return (160.0, 100.0, 320.0, 0.8)

            def last_status(self):
                return {"state": "ok", "source": "camera"}

            def release(self, ctrl=None):
                return None

        ctrl = SimpleNamespace(
            following_active=True,
            follower_camera=DummyCamera(),
            follower_cfg={"target_distance_m": 1.0, "stop_distance_m": 0.7},
            logger=SimpleNamespace(warn=lambda *args, **kwargs: None, info=lambda *args, **kwargs: None),
        )
        ctrl._emergency_stop = lambda reason="": setattr(ctrl, "emergency_reason", reason)
        snap = SimpleNamespace(
            timestamp=time.monotonic(),
            raw_scan=[
                {"angle": 0.0, "dist": 450.0},
                {"angle": 90.0, "dist": 300.0},
            ],
        )

        command = follower_module.tick(ctrl, snap)

        self.assertEqual(command, (0.0, 0.0))
        self.assertEqual(ctrl.emergency_reason, "FOLLOW_LIDAR_OBSTACLE")
        self.assertFalse(ctrl.following_active)

    def test_camera_bbox_distance_estimate_from_person_height(self):
        status = _camera_distance_from_bbox(
            detector="mediapipe_pose",
            bbox_height_px=144.0,
            image_height_px=240.0,
            detector_confidence=0.9,
        )

        self.assertEqual(status["distance_source"], "camera_bbox")
        self.assertAlmostEqual(float(status["bbox_height_ratio"]), 0.60, places=2)
        self.assertAlmostEqual(float(status["distance_estimate_m"]), 1.20, places=2)
        self.assertGreater(float(status["distance_confidence"]), 0.6)

    def test_mediapipe_upper_body_bbox_locks_without_distance_claim(self):
        shape = _camera_human_bbox_shape_status(
            detector="mediapipe_pose",
            bbox_width_px=112.0,
            bbox_height_px=30.0,
            image_width_px=320.0,
            image_height_px=240.0,
        )

        self.assertTrue(shape["bbox_human_shape_ok"])
        self.assertEqual(shape["bbox_shape_variant"], "mediapipe_upper_body")

        distance = _camera_distance_from_bbox(
            detector="mediapipe_pose",
            bbox_width_px=112.0,
            bbox_height_px=30.0,
            image_width_px=320.0,
            image_height_px=240.0,
            detector_confidence=1.0,
        )

        self.assertEqual(distance["distance_source"], "camera_bbox_upper_body_unreliable")
        self.assertIsNone(distance["distance_estimate_m"])
        self.assertEqual(float(distance["distance_confidence"]), 0.0)

    def test_mediapipe_wide_upper_body_bbox_locks_without_distance_claim(self):
        shape = _camera_human_bbox_shape_status(
            detector="mediapipe_pose",
            bbox_width_px=454.0,
            bbox_height_px=104.0,
            image_width_px=640.0,
            image_height_px=360.0,
        )

        self.assertTrue(shape["bbox_human_shape_ok"])
        self.assertEqual(shape["bbox_shape_variant"], "mediapipe_upper_body")

        distance = _camera_distance_from_bbox(
            detector="mediapipe_pose",
            bbox_width_px=454.0,
            bbox_height_px=104.0,
            image_width_px=640.0,
            image_height_px=360.0,
            detector_confidence=1.0,
        )

        self.assertEqual(distance["distance_source"], "camera_bbox_upper_body_unreliable")
        self.assertIsNone(distance["distance_estimate_m"])

    def test_target_obstacle_arbiter_switches_to_search_after_short_reacquire_timeout(self):
        previous = {
            "dist_m": 1.20,
            "angle_deg": 4.0,
            "confidence": 0.7,
            "last_seen_ts": time.time() - 2.2,
        }

        decision = TargetObstacleArbiter().decide_target_loss(
            camera_status={"state": "target_stale", "stale": True, "age_s": 2.2},
            previous_target=previous,
            desired_distance_m=1.0,
        )

        self.assertEqual(decision.mode, "target_lost_search")
        self.assertTrue(decision.search_required)
        self.assertFalse(decision.allow_follow_target)

    def test_weak_camera_candidate_continues_search_instead_of_zero_hold(self):
        self.assertTrue(
            follower_module._weak_camera_candidate_should_continue_search(
                {
                    "state": "candidate_unconfirmed",
                    "detector": "opencv_motion_blob",
                    "target_usable": False,
                    "lock_confirmed": False,
                }
            )
        )
        self.assertTrue(
            follower_module._weak_camera_candidate_should_continue_search(
                {
                    "state": "candidate_hold",
                    "detector": "opencv_template_lock",
                    "detector_confidence": 0.42,
                    "target_usable": False,
                    "lock_confirmed": False,
                }
            )
        )
        self.assertFalse(
            follower_module._weak_camera_candidate_should_continue_search(
                {
                    "state": "candidate_unconfirmed",
                    "detector": "onnx_yolov5_person",
                    "target_usable": False,
                    "lock_confirmed": False,
                }
            )
        )
        self.assertFalse(
            follower_module._weak_camera_candidate_should_continue_search(
                {
                    "state": "candidate_hold",
                    "detector": "opencv_template_lock",
                    "detector_confidence": 0.72,
                    "target_usable": False,
                    "lock_confirmed": False,
                }
            )
        )

    def test_marginal_onnx_bbox_distance_is_capped_for_cautious_follow(self):
        status = _camera_distance_from_bbox(
            detector="onnx_yolov5_person",
            bbox_width_px=270.0,
            bbox_height_px=72.0,
            image_width_px=320.0,
            image_height_px=240.0,
            detector_confidence=0.38,
        )

        self.assertEqual(status["distance_source"], "camera_bbox_marginal_shape_capped")
        self.assertLessEqual(float(status["distance_estimate_m"]), follower_module.CAMERA_DISTANCE_MARGINAL_BBOX_MAX_M)
        self.assertLessEqual(float(status["distance_confidence"]), 0.28)

    def test_motion_blob_low_confidence_distance_does_not_jump_to_close_floor(self):
        status = _camera_distance_from_bbox(
            detector="opencv_motion_blob",
            bbox_height_px=220.0,
            image_height_px=240.0,
            detector_confidence=0.35,
            bbox_area_ratio=0.08,
        )

        self.assertEqual(status["distance_source"], "camera_bbox_motion_blob_low_conf_floor")
        self.assertGreaterEqual(float(status["distance_estimate_m"]), 0.95)
        self.assertLessEqual(float(status["distance_confidence"]), 0.30)

    def test_follower_tick_prefers_camera_distance_over_near_wall_lidar(self):
        class DummyCamera:
            too_many_failures = False

            def detect_with_persistence(self, ctrl):
                return (160.0, 100.0, 320.0, 0.9)

            def last_status(self):
                return {
                    "state": "ok",
                    "source": "camera",
                    "stale": False,
                    "target_visible": True,
                    "target_usable": True,
                    "frame_ok": True,
                    "distance_estimate_m": 2.10,
                    "distance_confidence": 0.82,
                    "distance_source": "camera_bbox",
                    "bbox_height_ratio": 0.34,
                }

        ctrl = SimpleNamespace(
            following_active=True,
            follower_camera=DummyCamera(),
            follower_cfg={"target_distance_m": 1.0, "stop_distance_m": 0.7},
            logger=SimpleNamespace(warn=lambda *args, **kwargs: None),
        )
        ctrl._emergency_stop = lambda reason="": setattr(ctrl, "emergency_reason", reason)
        snap = SimpleNamespace(
            timestamp=time.monotonic(),
            raw_scan=[
                {"angle": 359.0, "dist": 860.0},
                {"angle": 0.0, "dist": 870.0},
                {"angle": 1.0, "dist": 880.0},
            ],
        )

        follower_module.tick(ctrl, snap)

        self.assertGreater(float(ctrl._adaptive_target_dist_m), 2.0)
        self.assertEqual(ctrl._adaptive_target_camera_status["distance_source"], "camera_bbox")
        self.assertAlmostEqual(ctrl._adaptive_target_camera_status["lidar_distance_m"], 0.87, places=2)
        self.assertEqual(ctrl._adaptive_follow_state, "approach")

    def test_follower_lost_target_publishes_camera_search_request(self):
        class DummyCamera:
            too_many_failures = False

            def detect_with_persistence(self, ctrl):
                return None

            def last_status(self):
                return {
                    "state": "target_stale",
                    "source": "camera",
                    "stale": True,
                    "target_visible": False,
                    "target_usable": False,
                    "frame_ok": True,
                    "rotation_deg": 0,
                }

        class DummyEkf:
            def get_state(self):
                return {"theta_deg": 0.0}

        ctrl = SimpleNamespace(
            following_active=True,
            follower_camera=DummyCamera(),
            follower_cfg={"target_distance_m": 1.0, "stop_distance_m": 0.7, "max_v_target": 0.08, "max_omega": 0.8},
            ekf=DummyEkf(),
            logger=SimpleNamespace(warn=lambda *args, **kwargs: None, info=lambda *args, **kwargs: None),
        )
        snap = SimpleNamespace(
            timestamp=time.monotonic(),
            raw_scan=[
                {"angle": 0.0, "dist": 1600.0},
                {"angle": 1.0, "dist": 1610.0},
                {"angle": 359.0, "dist": 1590.0},
            ],
        )

        follower_module.tick(ctrl, snap)
        obs = camera_observation_from_controller(ctrl)
        req = FollowLayer().tick(obs, {"x": 0.0, "y": 0.0, "theta": 0.0}, source="ADAPTIVE")

        self.assertEqual(ctrl._adaptive_follow_state, "target_search_scan")
        self.assertEqual(ctrl._adaptive_target_camera_status["state"], "target_search_scan")
        self.assertIsNotNone(obs)
        self.assertEqual(obs.source, TARGET_SOURCE_CAMERA_SEARCH)
        self.assertTrue(req.active)
        self.assertEqual(req.target_source, TARGET_SOURCE_CAMERA_SEARCH)
        self.assertEqual(req.reason, "target_search_scan")

    def test_follower_search_stops_after_three_rotations_without_target(self):
        class DummyCamera:
            too_many_failures = False

            def detect_with_persistence(self, ctrl):
                return None

            def last_status(self):
                return {"state": "target_stale", "source": "camera", "stale": True}

            def release(self, ctrl=None):
                return None

        class DummyEkf:
            def get_state(self):
                return {"theta_deg": 2.0}

        ctrl = SimpleNamespace(
            following_active=True,
            follower_camera=DummyCamera(),
            follower_cfg={"target_distance_m": 1.0, "stop_distance_m": 0.7},
            ekf=DummyEkf(),
            logger=SimpleNamespace(warn=lambda *args, **kwargs: None, info=lambda *args, **kwargs: None),
            _follow_search_active=True,
            _adaptive_target_search_active=True,
            _follow_search_total_rotated_deg=354.0,
            _follow_search_rotations_completed=2,
            _follow_search_last_theta_deg=0.0,
        )
        snap = SimpleNamespace(
            timestamp=time.monotonic(),
            raw_scan=[{"angle": 0.0, "dist": 1600.0}],
        )

        follower_module.tick(ctrl, snap)

        self.assertFalse(ctrl.following_active)
        self.assertEqual(ctrl.follow_search_status["state"], "failed")
        self.assertEqual(ctrl.follow_search_status["rotations_completed"], 3)

    def test_follower_search_times_out_without_fresh_usable_camera_target(self):
        class DummyCamera:
            too_many_failures = False

            def detect_with_persistence(self, ctrl):
                return None

            def last_status(self):
                return {
                    "state": "target_stale",
                    "source": "camera",
                    "stale": True,
                    "target_visible": False,
                    "target_usable": False,
                    "detector": "none",
                }

            def release(self, ctrl=None):
                return None

        class DummyEkf:
            def get_state(self):
                return {"theta_deg": 2.0}

        ctrl = SimpleNamespace(
            following_active=True,
            follower_camera=DummyCamera(),
            follower_cfg={"target_distance_m": 1.0, "stop_distance_m": 0.7},
            ekf=DummyEkf(),
            logger=SimpleNamespace(warn=lambda *args, **kwargs: None, info=lambda *args, **kwargs: None),
            _follow_search_active=True,
            _adaptive_target_search_active=True,
            _follow_search_total_rotated_deg=20.0,
            _follow_search_rotations_completed=0,
            _follow_search_last_theta_deg=0.0,
            _follow_search_started_ts=time.time() - follower_module.FOLLOW_SEARCH_TIMEOUT_S - 0.2,
        )
        snap = SimpleNamespace(
            timestamp=time.monotonic(),
            raw_scan=[{"angle": 0.0, "dist": 1600.0}],
        )

        follower_module.tick(ctrl, snap)

        self.assertFalse(ctrl.following_active)
        self.assertEqual(ctrl.follow_search_status["state"], "failed")
        self.assertEqual(ctrl.follow_search_status["reason"], "target_search_timeout")
        self.assertGreaterEqual(ctrl.follow_search_status["elapsed_s"], follower_module.FOLLOW_SEARCH_TIMEOUT_S)

    def test_follower_search_timeout_matches_live_search_gate(self):
        self.assertAlmostEqual(follower_module.FOLLOW_SEARCH_TIMEOUT_S, 15.0)

    def test_follower_search_accepts_single_strong_camera_target(self):
        class DummyCamera:
            too_many_failures = False

            def detect_with_persistence(self, ctrl):
                return (160.0, 100.0, 320.0, 0.8)

            def last_status(self):
                return {
                    "state": "ok",
                    "source": "camera",
                    "stale": False,
                    "target_visible": True,
                    "target_usable": True,
                    "frame_ok": True,
                    "detector": "onnx_yolov5_person",
                }

        class DummyEkf:
            def get_state(self):
                return {"theta_deg": 0.0}

        ctrl = SimpleNamespace(
            following_active=True,
            follower_camera=DummyCamera(),
            follower_cfg={"target_distance_m": 1.0, "stop_distance_m": 0.7},
            ekf=DummyEkf(),
            logger=SimpleNamespace(warn=lambda *args, **kwargs: None, info=lambda *args, **kwargs: None),
            _follow_search_active=True,
            _adaptive_target_search_active=True,
            _follow_search_total_rotated_deg=20.0,
            _follow_search_rotations_completed=0,
            _follow_search_last_theta_deg=0.0,
            _follow_search_found_confirm_count=0,
        )
        snap = SimpleNamespace(
            timestamp=time.monotonic(),
            raw_scan=[
                {"angle": 0.0, "dist": 1600.0},
                {"angle": 1.0, "dist": 1610.0},
                {"angle": 359.0, "dist": 1590.0},
            ],
        )

        follower_module.tick(ctrl, snap)
        self.assertFalse(ctrl._follow_search_active)
        self.assertFalse(ctrl._adaptive_target_search_active)
        self.assertEqual(ctrl.follow_search_status["state"], "found")
        self.assertNotEqual(ctrl._adaptive_follow_state, "target_search_scan")

    def test_follow_search_motion_blob_still_needs_confirmation(self):
        ctrl = SimpleNamespace(_follow_search_active=True, _follow_search_found_confirm_count=0)
        status = {
            "target_visible": True,
            "target_usable": True,
            "stale": False,
            "detector": "opencv_motion_blob",
            "bbox_human_shape_ok": True,
        }

        self.assertFalse(follower_module._follow_search_target_confirmed(ctrl, status))
        self.assertTrue(follower_module._follow_search_target_confirmed(ctrl, status))

    def test_follow_search_accepts_high_confidence_template_relock_once(self):
        ctrl = SimpleNamespace(_follow_search_active=True, _follow_search_found_confirm_count=0)
        status = {
            "target_visible": True,
            "target_usable": True,
            "stale": False,
            "detector": "opencv_template_lock",
            "detector_confidence": 0.54,
            "bbox_human_shape_ok": True,
        }

        self.assertTrue(follower_module._follow_search_target_confirmed(ctrl, status))

    def test_follow_search_low_confidence_template_still_needs_confirmation(self):
        ctrl = SimpleNamespace(_follow_search_active=True, _follow_search_found_confirm_count=0)
        status = {
            "target_visible": True,
            "target_usable": True,
            "stale": False,
            "detector": "opencv_template_lock",
            "detector_confidence": 0.50,
            "bbox_human_shape_ok": True,
        }

        self.assertFalse(follower_module._follow_search_target_confirmed(ctrl, status))
        self.assertTrue(follower_module._follow_search_target_confirmed(ctrl, status))

    def test_follow_search_hog_still_needs_confirmation(self):
        ctrl = SimpleNamespace(_follow_search_active=True, _follow_search_found_confirm_count=0)
        status = {
            "target_visible": True,
            "target_usable": True,
            "stale": False,
            "detector": "opencv_hog",
            "bbox_human_shape_ok": True,
        }

        self.assertFalse(follower_module._follow_search_target_confirmed(ctrl, status))
        self.assertTrue(follower_module._follow_search_target_confirmed(ctrl, status))

    def test_follower_capture_frame_uses_configured_camera_rotation(self):
        import numpy as np

        class DummyCamera:
            def capture_array(self):
                return np.zeros((2, 3, 3), dtype=np.uint8)

        cam = FollowerCamera()
        cam._camera = DummyCamera()
        calls = []
        original_rotation = follower_module._camera_rotation_deg
        original_rotate_image = follower_module._rotate_image

        def fake_rotate_image(arr, rotation_deg):
            calls.append(int(rotation_deg))
            return np.zeros((3, 2, 3), dtype=np.uint8)

        try:
            follower_module._camera_rotation_deg = lambda: 90
            follower_module._rotate_image = fake_rotate_image

            frame_rgb, im_w, im_h = cam.capture_frame(SimpleNamespace())

            self.assertEqual(calls, [90])
            self.assertEqual((im_w, im_h), (2, 3))
            self.assertEqual(frame_rgb.shape[:2], (3, 2))
            self.assertEqual(cam._last_frame_rotation_deg, 90)
        finally:
            follower_module._camera_rotation_deg = original_rotation
            follower_module._rotate_image = original_rotate_image

    def test_follower_capture_frame_keeps_rotated_output_landscape(self):
        import numpy as np

        class DummyCamera:
            def capture_array(self):
                return np.zeros((360, 640, 3), dtype=np.uint8)

        cam = FollowerCamera()
        cam._camera = DummyCamera()
        original_rotation = follower_module._camera_rotation_deg

        try:
            follower_module._camera_rotation_deg = lambda: 180

            frame_rgb, im_w, im_h = cam.capture_frame(SimpleNamespace())

            self.assertEqual((im_w, im_h), (640, 360))
            self.assertGreater(im_w, im_h)
            self.assertEqual(frame_rgb.shape[:2], (360, 640))
            self.assertEqual(cam._last_frame_rotation_deg, 180)
        finally:
            follower_module._camera_rotation_deg = original_rotation

    def test_follower_status_reports_camera_open_failure(self):
        cam = FollowerCamera()
        cam._last_open_failed = True
        cam._failed_sessions = 2
        cam._last_open_error = "Camera __init__ sequence did not complete."
        cam.capture_frame = lambda ctrl: None

        center = cam.detect_with_persistence(SimpleNamespace())
        status = cam.last_status()

        self.assertIsNone(center)
        self.assertEqual(status["state"], "camera_open_failed")
        self.assertTrue(status["open_failed"])
        self.assertEqual(status["failed_sessions"], 2)
        self.assertIn("Camera __init__", status["last_open_error"])

    def test_detect_with_persistence_uses_detector_confidence_metadata(self):
        import numpy as np

        cam = FollowerCamera()
        cam.capture_frame = lambda ctrl: (np.zeros((120, 160, 3), dtype=np.uint8), 160, 120)

        def fake_detect(_frame, _width, _height):
            cam._last_detector_name = "onnx_yolov5_person"
            cam._last_detector_confidence = 0.55
            cam._remember_bbox(
                detector="onnx_yolov5_person",
                bbox_x_px=48,
                bbox_y_px=10,
                bbox_width_px=64,
                bbox_height_px=108,
                image_width_px=160,
                image_height_px=120,
                center_x_px=80,
                center_y_px=60,
            )
            return (80.0, 60.0)

        cam._detect_person_bbox = fake_detect

        center = cam.detect_with_persistence(SimpleNamespace())
        status = cam.last_status()

        self.assertIsNone(center)
        self.assertEqual(status["state"], "candidate_unconfirmed")
        self.assertFalse(status["target_usable"])
        self.assertEqual(status["detector"], "onnx_yolov5_person")
        self.assertAlmostEqual(status["detector_confidence"], 0.55)
        self.assertAlmostEqual(status["confidence"], 0.55)
        self.assertEqual(status["image_width_px"], 160)
        self.assertEqual(status["image_height_px"], 120)
        self.assertTrue(status["frame_low_contrast"])

    def test_onnx_person_detector_decodes_person_bbox(self):
        import numpy as np

        cam = FollowerCamera()

        class FakeOnnxSession:
            def run(self, *_args, **_kwargs):
                pred = np.zeros((1, 25200, 85), dtype=np.float32)
                pred[0, 0, 0:4] = [320.0, 320.0, 160.0, 300.0]
                pred[0, 0, 4] = 0.90
                pred[0, 0, 5] = 0.80
                return [pred]

        cam._onnx_session = FakeOnnxSession()
        cam._onnx_input_name = "images"
        cam._onnx_input_size = 640
        cam._ensure_onnx_person_detector = lambda: True

        center = cam._detect_onnx_person_bbox(np.zeros((120, 160, 3), dtype=np.uint8), 160, 120)

        self.assertIsNotNone(center)
        self.assertAlmostEqual(center[0], 80.0, delta=1.0)
        self.assertAlmostEqual(center[1], 60.0, delta=1.0)
        self.assertEqual(cam._last_detector_name, "onnx_yolov5_person")
        self.assertAlmostEqual(cam._last_detector_confidence, 0.72, places=2)
        self.assertTrue(cam._last_bbox_status["bbox_human_shape_ok"])

    def test_onnx_person_detector_accepts_centered_weak_person_candidate(self):
        import numpy as np

        cam = FollowerCamera()

        class FakeOnnxSession:
            def run(self, *_args, **_kwargs):
                pred = np.zeros((1, 25200, 85), dtype=np.float32)
                pred[0, 0, 0:4] = [330.0, 340.0, 140.0, 320.0]
                pred[0, 0, 4] = 0.04
                pred[0, 0, 5] = 0.60
                return [pred]

        cam._onnx_session = FakeOnnxSession()
        cam._onnx_input_name = "images"
        cam._onnx_input_size = 640
        cam._ensure_onnx_person_detector = lambda: True

        center = cam._detect_onnx_person_bbox(np.zeros((120, 160, 3), dtype=np.uint8), 160, 120)

        self.assertIsNone(center)
        self.assertEqual(cam._last_detector_error, "onnx_person_not_found")
        self.assertEqual(cam._last_bbox_status["onnx_best_reject_reason"], "score_below_threshold")

    def test_onnx_person_detector_accepts_very_weak_centered_person_candidate(self):
        import numpy as np

        cam = FollowerCamera()

        class FakeOnnxSession:
            def run(self, *_args, **_kwargs):
                pred = np.zeros((1, 25200, 85), dtype=np.float32)
                pred[0, 0, 0:4] = [320.0, 320.0, 100.0, 300.0]
                pred[0, 0, 4] = 0.026
                pred[0, 0, 5] = 0.56
                return [pred]

        cam._onnx_session = FakeOnnxSession()
        cam._onnx_input_name = "images"
        cam._onnx_input_size = 640
        cam._ensure_onnx_person_detector = lambda: True

        center = cam._detect_onnx_person_bbox(np.zeros((120, 160, 3), dtype=np.uint8), 160, 120)

        self.assertIsNone(center)
        self.assertEqual(cam._last_detector_error, "onnx_person_not_found")
        self.assertEqual(cam._last_bbox_status["onnx_best_reject_reason"], "score_below_threshold")

    def test_onnx_person_detector_accepts_recent_human_relock_candidate(self):
        import numpy as np

        cam = FollowerCamera()
        cam._mark_strong_human_detector("onnx_yolov5_person")

        class FakeOnnxSession:
            def run(self, *_args, **_kwargs):
                pred = np.zeros((1, 25200, 85), dtype=np.float32)
                pred[0, 0, 0:4] = [320.0, 320.0, 110.0, 300.0]
                pred[0, 0, 4] = 0.008
                pred[0, 0, 5] = 0.54
                return [pred]

        cam._onnx_session = FakeOnnxSession()
        cam._onnx_input_name = "images"
        cam._onnx_input_size = 640
        cam._ensure_onnx_person_detector = lambda: True

        center = cam._detect_onnx_person_bbox(np.zeros((120, 160, 3), dtype=np.uint8), 160, 120)

        self.assertIsNone(center)
        self.assertEqual(cam._last_detector_error, "onnx_person_not_found")
        self.assertEqual(cam._last_bbox_status["onnx_best_reject_reason"], "score_below_threshold")

    def test_onnx_person_detector_reports_best_rejected_candidate_status(self):
        import numpy as np

        cam = FollowerCamera()

        class FakeOnnxSession:
            def run(self, *_args, **_kwargs):
                pred = np.zeros((1, 25200, 85), dtype=np.float32)
                pred[0, 0, 0:4] = [320.0, 320.0, 120.0, 280.0]
                pred[0, 0, 4] = 0.015
                pred[0, 0, 5] = 0.40
                return [pred]

        cam._onnx_session = FakeOnnxSession()
        cam._onnx_input_name = "images"
        cam._onnx_input_size = 640
        cam._ensure_onnx_person_detector = lambda: True

        center = cam._detect_onnx_person_bbox(np.zeros((120, 160, 3), dtype=np.uint8), 160, 120)

        self.assertIsNone(center)
        self.assertEqual(cam._last_detector_error, "onnx_person_not_found")
        self.assertGreater(cam._last_bbox_status["onnx_best_score"], 0.0)
        self.assertEqual(cam._last_bbox_status["onnx_best_reject_reason"], "score_below_threshold")

    def test_prime_from_stream_frame_uses_onnx_human_bbox_without_camera_open(self):
        import tempfile
        import numpy as np
        from PIL import Image

        with tempfile.TemporaryDirectory() as tmp:
            runtime_dir = Path(tmp)
            stream_path = runtime_dir / "stream_frame.jpg"
            Image.fromarray(np.full((120, 160, 3), 96, dtype=np.uint8), mode="RGB").save(stream_path)
            ctrl = SimpleNamespace(status_path=str(runtime_dir / "status.json"))
            cam = FollowerCamera()

            def fake_detect(frame_rgb, width, height):
                self.assertEqual((width, height), (160, 120))
                cam._last_detector_name = "onnx_yolov5_person"
                cam._last_detector_confidence = 0.72
                cam._mark_strong_human_detector("onnx_yolov5_person")
                cam._remember_bbox(
                    detector="onnx_yolov5_person",
                    bbox_x_px=58,
                    bbox_y_px=10,
                    bbox_width_px=44,
                    bbox_height_px=108,
                    image_width_px=160,
                    image_height_px=120,
                    center_x_px=80,
                    center_y_px=64,
                )
                return (80.0, 64.0)

            cam._detect_onnx_person_bbox = fake_detect

            seed = cam.prime_from_stream_frame(ctrl)

            self.assertIsNone(seed)
            self.assertIsNone(cam._camera)
            status = cam.last_status()
            self.assertEqual(status["state"], "candidate_unconfirmed")
            self.assertTrue(status["stream_seed"])
            self.assertEqual(status["detector"], "onnx_yolov5_person")
            self.assertTrue(status["bbox_human_shape_ok"])

    def test_onnx_detector_unavailable_latch_retries_after_cooldown(self):
        cam = FollowerCamera()
        cam._onnx_session = object()
        cam._onnx_detector_unavailable = True
        cam._onnx_detector_unavailable_ts = time.monotonic()

        self.assertFalse(cam._ensure_onnx_person_detector())

        cam._onnx_detector_unavailable_ts = (
            time.monotonic() - follower_module.CAMERA_ONNX_RETRY_INTERVAL_S - 1.0
        )

        self.assertTrue(cam._ensure_onnx_person_detector())
        self.assertFalse(cam._onnx_detector_unavailable)

    def test_template_lock_tracks_after_strong_human_bbox(self):
        import numpy as np

        cam = FollowerCamera()
        frame_a = np.zeros((120, 160, 3), dtype=np.uint8)
        frame_a[25:105, 65:100, :] = 80
        frame_a[25:105, 78:82, :] = 210
        frame_a[50:55, 65:100, :] = 160
        cam._last_detector_confidence = 0.80
        cam._remember_bbox(
            detector="onnx_yolov5_person",
            bbox_x_px=65,
            bbox_y_px=25,
            bbox_width_px=35,
            bbox_height_px=80,
            image_width_px=160,
            image_height_px=120,
        )
        cam._mark_strong_human_detector("onnx_yolov5_person")
        cam._refresh_template_lock(frame_a, detector="onnx_yolov5_person")

        frame_b = np.zeros((120, 160, 3), dtype=np.uint8)
        frame_b[25:105, 73:108, :] = 80
        frame_b[25:105, 86:90, :] = 210
        frame_b[50:55, 73:108, :] = 160

        center = cam._detect_template_lock(frame_b, 160, 120)

        self.assertIsNotNone(center)
        self.assertAlmostEqual(center[0], 90.5, delta=2.0)
        self.assertAlmostEqual(center[1], 65.0, delta=2.0)
        self.assertEqual(cam._last_detector_name, "opencv_template_lock")
        self.assertGreaterEqual(cam._last_detector_confidence, 0.55)
        self.assertTrue(cam._last_bbox_status["bbox_human_shape_ok"])

    def test_template_lock_tracks_small_scale_change(self):
        import cv2
        import numpy as np

        cam = FollowerCamera()
        frame_a = np.zeros((120, 160, 3), dtype=np.uint8)
        frame_a[25:105, 65:100, :] = 80
        frame_a[25:105, 78:82, :] = 210
        frame_a[50:55, 65:100, :] = 160
        cam._last_detector_confidence = 0.80
        cam._remember_bbox(
            detector="onnx_yolov5_person",
            bbox_x_px=65,
            bbox_y_px=25,
            bbox_width_px=35,
            bbox_height_px=80,
            image_width_px=160,
            image_height_px=120,
        )
        cam._mark_strong_human_detector("onnx_yolov5_person")
        cam._refresh_template_lock(frame_a, detector="onnx_yolov5_person")

        frame_b = np.zeros((120, 160, 3), dtype=np.uint8)
        scaled_person = cv2.resize(frame_a[25:105, 65:100, :], (39, 90), interpolation=cv2.INTER_LINEAR)
        frame_b[20:110, 73:112, :] = scaled_person

        center = cam._detect_template_lock(frame_b, 160, 120)

        self.assertIsNotNone(center)
        self.assertAlmostEqual(center[0], 92.5, delta=3.0)
        self.assertAlmostEqual(center[1], 65.0, delta=3.0)
        self.assertEqual(cam._last_detector_name, "opencv_template_lock")
        self.assertGreater(cam._last_bbox_status["bbox_width_px"], 35.0)
        self.assertGreater(cam._last_bbox_status["bbox_height_px"], 80.0)

    def test_motion_blob_detector_tracks_large_moving_region(self):
        import numpy as np

        cam = FollowerCamera()
        cam._mark_strong_human_detector("opencv_hog")
        frame_a = np.zeros((120, 160, 3), dtype=np.uint8)
        frame_b = np.zeros((120, 160, 3), dtype=np.uint8)
        frame_b[25:105, 65:100, :] = 255

        self.assertIsNone(cam._detect_motion_blob(frame_a, 160, 120))
        center = cam._detect_motion_blob(frame_b, 160, 120)

        self.assertIsNotNone(center)
        self.assertAlmostEqual(center[0], 82.5, delta=12.0)
        self.assertAlmostEqual(center[1], 65.0, delta=12.0)
        self.assertEqual(cam._last_detector_name, "opencv_motion_blob")
        self.assertGreater(cam._last_detector_confidence, 0.3)

    def test_motion_blob_detector_rejects_without_recent_human_confirmation(self):
        import numpy as np

        cam = FollowerCamera()
        frame_a = np.zeros((120, 160, 3), dtype=np.uint8)
        frame_b = np.zeros((120, 160, 3), dtype=np.uint8)
        frame_b[25:105, 65:100, :] = 255

        self.assertIsNone(cam._detect_motion_blob(frame_a, 160, 120))
        center = cam._detect_motion_blob(frame_b, 160, 120)

        self.assertIsNone(center)
        self.assertEqual(cam._last_detector_error, "motion_blob_requires_recent_human_confirmation")

    def test_motion_blob_detector_rejects_wide_nonhuman_motion(self):
        import numpy as np

        cam = FollowerCamera()
        cam._mark_strong_human_detector("opencv_hog")
        frame_a = np.zeros((120, 160, 3), dtype=np.uint8)
        frame_b = np.zeros((120, 160, 3), dtype=np.uint8)
        frame_b[40:75, 15:145, :] = 255

        self.assertIsNone(cam._detect_motion_blob(frame_a, 160, 120))
        cam._mark_strong_human_detector("opencv_hog")
        center = cam._detect_motion_blob(frame_b, 160, 120)

        self.assertIsNone(center)
        self.assertIn(cam._last_detector_error, {"bbox_too_wide_for_human", "bbox_not_upright_human"})

    def test_start_following_requests_stream_writer_camera_handover_first(self):
        import controller.commands as commands
        import controller.stream_writer as stream_writer

        calls = []
        original_request = stream_writer.request_stream_writer_camera_release
        original_set_motion_source = commands.set_motion_source

        def fake_request(ctrl, timeout_s=1.0, poll_s=0.02):
            calls.append(("handover", bool(getattr(ctrl, "following_active", False)), timeout_s))
            return True

        def fake_set_motion_source(ctrl, source):
            calls.append(("motion_source", bool(getattr(ctrl, "following_active", False)), source))
            return True

        try:
            stream_writer.request_stream_writer_camera_release = fake_request
            commands.set_motion_source = fake_set_motion_source
            ctrl = SimpleNamespace(
                following_active=False,
                input_vector={},
                joystick_active=True,
                arbiter=SimpleNamespace(last_ts={"GUI_JOYSTICK": 123.0}),
                logger=SimpleNamespace(info=lambda *args, **kwargs: None, warn=lambda *args, **kwargs: None),
            )

            start_following(ctrl)
        finally:
            stream_writer.request_stream_writer_camera_release = original_request
            commands.set_motion_source = original_set_motion_source

        self.assertEqual(calls[0][0], "handover")
        self.assertFalse(calls[0][1])
        self.assertEqual(calls[1][0], "motion_source")
        self.assertFalse(calls[1][1])
        self.assertTrue(ctrl.following_active)

    def test_start_following_primes_camera_target_from_stream_seed(self):
        import tempfile
        import controller.commands as commands
        import controller.stream_writer as stream_writer

        original_request = stream_writer.request_stream_writer_camera_release
        original_set_motion_source = commands.set_motion_source
        original_prime = FollowerCamera.prime_from_stream_frame
        original_ensure = FollowerCamera._ensure_onnx_person_detector
        original_wait = follower_module._wait_for_follow_stream_seed_frame

        def fake_request(_ctrl, timeout_s=1.0, poll_s=0.02):
            return True

        def fake_set_motion_source(_ctrl, source):
            return source == "ADAPTIVE"

        def fake_ensure(_self):
            return True

        def fake_prime(_self, _ctrl):
            return (
                160.0,
                120.0,
                320.0,
                0.82,
                {
                    "state": "ok",
                    "source": "camera",
                    "stale": False,
                    "target_visible": True,
                    "target_usable": True,
                    "frame_ok": True,
                    "detector": "onnx_yolov5_person",
                    "detector_confidence": 0.82,
                    "bbox_human_shape_ok": True,
                    "image_width_px": 320,
                    "image_height_px": 240,
                    "rotation_deg": 0,
                    "distance_estimate_m": 1.36,
                    "distance_confidence": 0.70,
                    "stream_seed": True,
                    "stream_seed_age_s": 0.25,
                },
            )

        try:
            stream_writer.request_stream_writer_camera_release = fake_request
            commands.set_motion_source = fake_set_motion_source
            FollowerCamera._ensure_onnx_person_detector = fake_ensure
            FollowerCamera.prime_from_stream_frame = fake_prime
            follower_module._wait_for_follow_stream_seed_frame = lambda *_args, **_kwargs: True
            with tempfile.TemporaryDirectory() as tmp:
                ctrl = SimpleNamespace(
                    following_active=False,
                    input_vector={},
                    joystick_active=True,
                    arbiter=SimpleNamespace(last_ts={"GUI_JOYSTICK": 123.0}),
                    logger=SimpleNamespace(info=lambda *args, **kwargs: None, warn=lambda *args, **kwargs: None),
                    status_path=str(Path(tmp) / "status.json"),
                    follower_cfg={"target_distance_m": 1.0, "max_v_target": 0.20, "max_omega": 0.30},
                )

                start_following(ctrl)
        finally:
            stream_writer.request_stream_writer_camera_release = original_request
            commands.set_motion_source = original_set_motion_source
            FollowerCamera.prime_from_stream_frame = original_prime
            FollowerCamera._ensure_onnx_person_detector = original_ensure
            follower_module._wait_for_follow_stream_seed_frame = original_wait

        self.assertTrue(ctrl.following_active)
        self.assertFalse(ctrl._adaptive_target_search_active)
        self.assertTrue(ctrl._adaptive_target_camera_status["stream_seed"])
        self.assertAlmostEqual(ctrl._adaptive_target_dist_m, 1.36, places=2)
        obs = camera_observation_from_controller(ctrl)
        self.assertIsNotNone(obs)
        self.assertEqual(obs.source, TARGET_SOURCE_CAMERA_TARGET)
        self.assertAlmostEqual(obs.distance_m, 1.36, places=2)

    def test_legacy_adaptive_command_api_never_emits_direct_velocity(self):
        calls = []
        original_tick = follower_module.tick

        def fake_tick(ctrl, lidar_snapshot):
            calls.append((ctrl, lidar_snapshot))
            return (0.2, 0.3)

        try:
            follower_module.tick = fake_tick
            ctrl = SimpleNamespace(following_active=True)
            lidar_snapshot = object()

            command = follower_module.get_adaptive_command(ctrl, lidar_snapshot)

            self.assertIsNone(command)
            self.assertEqual(calls, [(ctrl, lidar_snapshot)])
        finally:
            follower_module.tick = original_tick

    def test_lidar_target_distance_uses_cluster_not_single_closest_noise(self):
        snap = SimpleNamespace(
            timestamp=time.monotonic(),
            raw_scan=[
                {"angle": 10.0, "dist": 420.0},
                {"angle": 9.0, "dist": 1390.0},
                {"angle": 10.0, "dist": 1420.0},
                {"angle": 11.0, "dist": 1410.0},
                {"angle": 10.0, "dist": 2600.0},
            ],
        )

        measurement = _get_lidar_target_measurement_at_angle_deg(snap, 10.0)

        self.assertIsNotNone(measurement)
        self.assertAlmostEqual(float(measurement["distance_m"]), 1.41, places=2)
        self.assertEqual(int(measurement["cluster_points"]), 3)
        self.assertGreater(float(measurement["confidence"]), 0.5)
        self.assertAlmostEqual(_get_lidar_dist_at_angle_deg(snap, 10.0), 1.41, places=2)

    def test_lidar_target_distance_prefers_expected_cluster_over_close_obstacle(self):
        snap = SimpleNamespace(
            timestamp=time.monotonic(),
            raw_scan=[
                {"angle": 2.0, "dist": 520.0},
                {"angle": 3.0, "dist": 540.0},
                {"angle": 1.0, "dist": 560.0},
                {"angle": 0.0, "dist": 1660.0},
                {"angle": 1.0, "dist": 1680.0},
                {"angle": 359.0, "dist": 1700.0},
            ],
        )

        measurement = _get_lidar_target_measurement_at_angle_deg(
            snap,
            0.0,
            expected_distance_m=1.65,
        )

        self.assertIsNotNone(measurement)
        self.assertAlmostEqual(float(measurement["distance_m"]), 1.68, places=2)
        self.assertEqual(int(measurement["cluster_points"]), 3)

    def test_lidar_target_distance_rejects_cluster_outside_expected_gate(self):
        snap = SimpleNamespace(
            timestamp=time.monotonic(),
            raw_scan=[
                {"angle": 0.0, "dist": 860.0},
                {"angle": 1.0, "dist": 880.0},
                {"angle": 359.0, "dist": 870.0},
            ],
        )

        measurement = _get_lidar_target_measurement_at_angle_deg(
            snap,
            0.0,
            expected_distance_m=1.80,
        )

        self.assertIsNone(measurement)

    def test_target_tracker_limits_transient_distance_jump(self):
        tracker = TargetKinematicsTracker(max_speed_mps=0.85)
        tracker.observe(1.80, 0.0, confidence=0.8)
        tracker._last_t_mono = time.monotonic() - 0.20

        tracked = tracker.observe(0.55, 0.0, confidence=0.8)

        self.assertTrue(tracked["measurement_limited"])
        self.assertGreater(float(tracked["dist_m"]), 1.55)

    def test_follower_tick_short_stale_camera_holds_target_without_forward_search(self):
        class DummyCamera:
            too_many_failures = False

            def detect_with_persistence(self, ctrl):
                return (160.0, 100.0, 320.0, 0.5)

            def last_status(self):
                return {
                    "state": "target_persisted",
                    "source": "camera",
                    "stale": True,
                    "target_visible": False,
                    "target_usable": True,
                    "frame_ok": True,
                }

        tracker = TargetKinematicsTracker(max_speed_mps=0.85)
        tracker.observe(1.80, 0.0, confidence=0.7)
        ctrl = SimpleNamespace(
            following_active=True,
            follower_camera=DummyCamera(),
            follower_target_tracker=tracker,
            follower_cfg={"target_distance_m": 1.0, "stop_distance_m": 0.7},
            logger=SimpleNamespace(warn=lambda *args, **kwargs: None),
        )
        ctrl._emergency_stop = lambda reason="": setattr(ctrl, "emergency_reason", reason)
        snap = SimpleNamespace(
            timestamp=time.monotonic(),
            raw_scan=[
                {"angle": 0.0, "dist": 1160.0},
                {"angle": 1.0, "dist": 1170.0},
                {"angle": 359.0, "dist": 1150.0},
            ],
        )

        follower_module.tick(ctrl, snap)

        self.assertFalse(ctrl._adaptive_target_search_active)
        self.assertEqual(ctrl._adaptive_follow_state, "target_persistence_hold")
        self.assertAlmostEqual(float(ctrl._adaptive_target_dist_m), 1.0)
        self.assertEqual(ctrl._adaptive_target_camera_status["gate"], "target_persistence_short_hold")
        self.assertFalse(ctrl._adaptive_target_camera_status["forward_allowed"])
        obs = camera_observation_from_controller(ctrl)
        self.assertIsNotNone(obs)
        self.assertEqual(obs.source, TARGET_SOURCE_CAMERA_TARGET)
        self.assertEqual(obs.target_id, "camera_target_persisted")
        req = FollowLayer().tick(obs, {"x": 0.0, "y": 0.0, "theta": 0.0}, source="ADAPTIVE")
        self.assertTrue(req.active)
        self.assertEqual(req.reason, "inside_follow_standoff")
        self.assertAlmostEqual(req.goal_x, 0.0, places=3)

    def test_follower_tick_weak_candidate_after_recent_target_uses_persistence_hold(self):
        class DummyCamera:
            too_many_failures = False

            def detect_with_persistence(self, ctrl):
                return None

            def last_status(self):
                return {
                    "state": "candidate_unconfirmed",
                    "source": "camera",
                    "stale": False,
                    "target_visible": True,
                    "target_usable": False,
                    "frame_ok": True,
                    "detector": "opencv_motion_blob",
                    "detector_confidence": 0.52,
                    "lock_confirmed": False,
                }

        tracker = TargetKinematicsTracker(max_speed_mps=0.85)
        tracker.observe(1.08, -6.0, confidence=0.8)
        ctrl = SimpleNamespace(
            following_active=True,
            follower_camera=DummyCamera(),
            follower_target_tracker=tracker,
            follower_cfg={"target_distance_m": 1.0, "stop_distance_m": 0.7},
            logger=SimpleNamespace(warn=lambda *args, **kwargs: None, info=lambda *args, **kwargs: None),
        )
        snap = SimpleNamespace(
            timestamp=time.monotonic(),
            raw_scan=[
                {"angle": 0.0, "dist": 1160.0},
                {"angle": 1.0, "dist": 1170.0},
                {"angle": 359.0, "dist": 1150.0},
            ],
        )

        follower_module.tick(ctrl, snap)

        self.assertFalse(ctrl._adaptive_target_search_active)
        self.assertEqual(ctrl._adaptive_follow_state, "target_persistence_hold")
        self.assertEqual(ctrl._adaptive_target_camera_status["gate"], "target_persistence_short_hold")
        self.assertTrue(ctrl._adaptive_target_camera_status["target_usable"])
        obs = camera_observation_from_controller(ctrl)
        self.assertIsNotNone(obs)
        self.assertEqual(obs.target_id, "camera_target_persisted")

    def test_follower_tick_recent_onnx_candidate_holds_during_capture_throttle(self):
        class DummyCamera:
            too_many_failures = False

            def __init__(self):
                self.calls = 0
                self.statuses = [
                    {
                        "state": "candidate_unconfirmed",
                        "source": "camera",
                        "stale": False,
                        "target_visible": True,
                        "target_usable": False,
                        "frame_ok": True,
                        "detector": "onnx_yolov5_person",
                        "detector_confidence": 0.53,
                        "lock_confirmed": False,
                    },
                    {
                        "state": "capture_throttled",
                        "source": "camera",
                        "stale": True,
                        "target_visible": False,
                        "target_usable": False,
                        "frame_ok": True,
                        "detector": "none",
                        "detector_confidence": 0.0,
                    },
                ]

            def detect_with_persistence(self, ctrl):
                self.calls += 1
                return None

            def last_status(self):
                return dict(self.statuses[min(self.calls - 1, len(self.statuses) - 1)])

        ctrl = SimpleNamespace(
            following_active=True,
            follower_camera=DummyCamera(),
            follower_cfg={"target_distance_m": 1.0, "stop_distance_m": 0.7},
            logger=SimpleNamespace(warn=lambda *args, **kwargs: None, info=lambda *args, **kwargs: None),
        )
        snap = SimpleNamespace(
            timestamp=time.monotonic(),
            raw_scan=[
                {"angle": 0.0, "dist": 1600.0},
                {"angle": 1.0, "dist": 1610.0},
                {"angle": 359.0, "dist": 1590.0},
            ],
        )

        follower_module.tick(ctrl, snap)
        self.assertFalse(ctrl._adaptive_target_search_active)
        self.assertEqual(ctrl._adaptive_follow_state, "target_reacquire_hold")
        self.assertTrue(ctrl._adaptive_target_camera_status["candidate_confirm_hold_active"])

        follower_module.tick(ctrl, snap)

        self.assertFalse(ctrl._adaptive_target_search_active)
        self.assertEqual(ctrl._adaptive_follow_state, "target_reacquire_hold")
        self.assertEqual(ctrl._adaptive_target_camera_status["hold_reason"], "candidate_confirm_wait")
        obs = camera_observation_from_controller(ctrl)
        self.assertIsNotNone(obs)
        self.assertEqual(obs.source, TARGET_SOURCE_CAMERA_TARGET)
        self.assertEqual(obs.target_id, "camera_target_reacquire")

    def test_follower_tick_mid_stale_camera_reacquires_without_search(self):
        class DummyCamera:
            too_many_failures = False

            def detect_with_persistence(self, ctrl):
                return (160.0, 100.0, 320.0, 0.5)

            def last_status(self):
                return {
                    "state": "target_stale",
                    "source": "camera",
                    "stale": True,
                    "target_visible": False,
                    "target_usable": False,
                    "frame_ok": True,
                }

        tracker = TargetKinematicsTracker(max_speed_mps=0.85)
        tracker.observe(1.80, 0.0, confidence=0.7)
        tracker._last_seen_ts = time.time() - 1.2
        ctrl = SimpleNamespace(
            following_active=True,
            follower_camera=DummyCamera(),
            follower_target_tracker=tracker,
            follower_cfg={"target_distance_m": 1.0, "stop_distance_m": 0.7},
            logger=SimpleNamespace(warn=lambda *args, **kwargs: None, info=lambda *args, **kwargs: None),
        )
        snap = SimpleNamespace(
            timestamp=time.monotonic(),
            raw_scan=[
                {"angle": 0.0, "dist": 1160.0},
                {"angle": 1.0, "dist": 1170.0},
                {"angle": 359.0, "dist": 1150.0},
            ],
        )

        follower_module.tick(ctrl, snap)

        self.assertFalse(ctrl._adaptive_target_search_active)
        self.assertEqual(ctrl._adaptive_follow_state, "target_reacquire_hold")
        obs = camera_observation_from_controller(ctrl)
        self.assertIsNotNone(obs)
        self.assertEqual(obs.source, TARGET_SOURCE_CAMERA_TARGET)
        self.assertEqual(obs.target_id, "camera_target_reacquire")
        req = FollowLayer().tick(obs, {"x": 0.0, "y": 0.0, "theta": 0.0}, source="ADAPTIVE")
        self.assertTrue(req.active)
        self.assertEqual(req.target_source, TARGET_SOURCE_CAMERA_TARGET)
        self.assertEqual(req.target_id, "camera_target_reacquire")
        self.assertEqual(req.reason, "inside_follow_standoff")

    def test_follower_tick_very_long_stale_camera_starts_search(self):
        class DummyCamera:
            too_many_failures = False

            def detect_with_persistence(self, ctrl):
                return (160.0, 100.0, 320.0, 0.5)

            def last_status(self):
                return {
                    "state": "target_stale",
                    "source": "camera",
                    "stale": True,
                    "target_visible": False,
                    "target_usable": False,
                    "frame_ok": True,
                }

        tracker = TargetKinematicsTracker(max_speed_mps=0.85)
        tracker.observe(1.80, 0.0, confidence=0.7)
        tracker._last_seen_ts = time.time() - 12.4
        ctrl = SimpleNamespace(
            following_active=True,
            follower_camera=DummyCamera(),
            follower_target_tracker=tracker,
            follower_cfg={"target_distance_m": 1.0, "stop_distance_m": 0.7},
            logger=SimpleNamespace(warn=lambda *args, **kwargs: None, info=lambda *args, **kwargs: None),
        )
        snap = SimpleNamespace(
            timestamp=time.monotonic(),
            raw_scan=[
                {"angle": 0.0, "dist": 1160.0},
                {"angle": 1.0, "dist": 1170.0},
                {"angle": 359.0, "dist": 1150.0},
            ],
        )

        follower_module.tick(ctrl, snap)

        self.assertTrue(ctrl._adaptive_target_search_active)
        self.assertEqual(ctrl._adaptive_follow_state, "target_search_scan")
        obs = camera_observation_from_controller(ctrl)
        self.assertIsNotNone(obs)
        self.assertEqual(obs.source, TARGET_SOURCE_CAMERA_SEARCH)

    def test_follower_tick_holds_tracker_distance_when_lidar_matches_near_wall(self):
        class DummyCamera:
            too_many_failures = False

            def detect_with_persistence(self, ctrl):
                return (160.0, 100.0, 320.0, 0.8)

            def last_status(self):
                return {
                    "state": "ok",
                    "source": "camera",
                    "stale": False,
                    "target_visible": True,
                    "target_usable": True,
                    "frame_ok": True,
                }

        tracker = TargetKinematicsTracker(max_speed_mps=0.85)
        tracker.observe(1.80, 0.0, confidence=0.8)
        ctrl = SimpleNamespace(
            following_active=True,
            follower_camera=DummyCamera(),
            follower_target_tracker=tracker,
            follower_cfg={"target_distance_m": 1.0, "stop_distance_m": 0.7},
            logger=SimpleNamespace(warn=lambda *args, **kwargs: None),
        )
        ctrl._emergency_stop = lambda reason="": setattr(ctrl, "emergency_reason", reason)
        snap = SimpleNamespace(
            timestamp=time.monotonic(),
            raw_scan=[
                {"angle": 0.0, "dist": 860.0},
                {"angle": 1.0, "dist": 880.0},
                {"angle": 359.0, "dist": 870.0},
            ],
        )

        follower_module.tick(ctrl, snap)

        self.assertGreater(float(ctrl._adaptive_target_dist_m), 1.7)
        self.assertEqual(ctrl._adaptive_target_lidar_source, "tracker_distance_hold_no_lidar_match")

    def test_lidar_target_distance_ignores_stale_scan(self):
        snap = SimpleNamespace(
            timestamp=time.monotonic() - 2.0,
            raw_scan=[{"angle": 0.0, "dist": 1000.0}],
        )

        measurement = _get_lidar_target_measurement_at_angle_deg(snap, 0.0)

        self.assertIsNotNone(measurement)
        self.assertEqual(measurement["source"], "lidar_stale")
        self.assertIsNone(measurement["distance_m"])
        self.assertIsNone(_get_lidar_dist_at_angle_deg(snap, 0.0))

        status = _target_lidar_status(snap, measurement)
        self.assertEqual(status["state"], "stale")
        self.assertFalse(status["usable_distance"])

    def test_lidar_target_status_reports_ok_cluster(self):
        snap = SimpleNamespace(
            timestamp=time.monotonic(),
            raw_scan=[
                {"angle": 0.0, "dist": 1190.0},
                {"angle": 1.0, "dist": 1210.0},
                {"angle": 359.0, "dist": 1205.0},
            ],
        )

        measurement = _get_lidar_target_measurement_at_angle_deg(snap, 0.0)
        status = _target_lidar_status(snap, measurement)

        self.assertEqual(status["state"], "ok")
        self.assertTrue(status["usable_distance"])
        self.assertGreaterEqual(status["cluster_points"], 2)

    def test_camera_status_separates_persisted_target_from_fresh_detection(self):
        cam = FollowerCamera(persistence_timeout_s=1.0)
        cam._last_center = (160.0, 100.0, 320.0)
        cam._last_detection_ts = time.monotonic() - 0.2
        cam.capture_frame = lambda ctrl: None

        center = cam.detect_with_persistence(SimpleNamespace())
        status = cam.last_status()

        self.assertIsNotNone(center)
        self.assertEqual(status["state"], "frame_missing_persisted")
        self.assertTrue(status["stale"])
        self.assertTrue(status["target_usable"])
        self.assertFalse(status["frame_ok"])

    def test_fresh_camera_lock_uses_bounded_recent_hold_between_detector_cycles(self):
        cam = FollowerCamera(persistence_timeout_s=1.0)
        cam._last_center = (160.0, 100.0, 320.0)
        cam._last_detection_ts = time.monotonic() - 0.05
        cam._last_detection_process_ts = time.monotonic()
        cam._lock_active = True
        cam._last_result_status = {
            "state": "ok",
            "source": "camera",
            "stale": False,
            "target_visible": True,
            "target_usable": True,
            "frame_ok": True,
            "detector": "onnx_yolov5_person",
            "detector_confidence": 0.82,
        }

        def fail_capture(_ctrl):
            raise AssertionError("capture should be throttled while the lock is fresh")

        cam.capture_frame = fail_capture

        center = cam.detect_with_persistence(SimpleNamespace())
        status = cam.last_status()

        self.assertIsNotNone(center)
        self.assertEqual(status["state"], "target_recent_hold")
        self.assertTrue(status["detector_throttled"])
        self.assertFalse(status["stale"])
        self.assertTrue(status["target_usable"])
        self.assertEqual(status["detector"], "onnx_yolov5_person")

    def test_adaptive_follow_state_lost_when_camera_stale_without_lidar(self):
        state = _adaptive_follow_state(
            camera_status={"target_usable": True, "stale": True},
            lidar_status={"usable_distance": False},
            dist_m=1.2,
            angle_deg=0.0,
            params={"stop_distance_m": 0.8, "target_distance_m": 1.2, "center_tolerance_deg": 10.0},
        )

        self.assertEqual(state, "lost")

    def test_adaptive_follow_state_reacquire_for_large_angle(self):
        state = _adaptive_follow_state(
            camera_status={"target_usable": True, "stale": False},
            lidar_status={"usable_distance": True},
            dist_m=1.2,
            angle_deg=25.0,
            params={"stop_distance_m": 0.8, "target_distance_m": 1.2, "center_tolerance_deg": 10.0},
        )

        self.assertEqual(state, "reacquire")


if __name__ == "__main__":
    unittest.main()
