#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import os
import py_compile
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

UPGRADE_ID = "camera_fp_geometry_ssot_v1_20260926"
EXPECTED_HEAD = "d8c831020d992079857aaff6343f5d8fffb39e8a"
EXPECTED_BLOBS = {
    "conf/vezerles.json": "13fb0db4768d22cda36f736c8f47d918abb9b96d",
    "v3/config.py": "147f6b66e0b755b39dbffec80faaada6432441be",
    "v3/layers/l4_world_model.py": "724be1cbe96cda194ff33c3e8ec01401d834d746",
    "v3/adapters/person_detection.py": "228bfc29ecd83f7809944ffb47fe1b8401feb93f",
    "v3/adapters/live_person_detection.py": "3a2486e67d55e1ca20c26f211b4dcd1abc1663f3",
    "v3/adapters/process_vision_port.py": "9dd09476c12eb8091fbaf4dff6150095ceaf6b52",
    "v3_hardware_runtime.py": "97f3c01b2bf203b64d56dd9ea4d944256e03cb8f",
    "tools/v3_person_detection_test.py": "18ff3bc71642a034a084f1b9d0212f531eb2eb12",
}
NEW_FILES = (
    "tests/core/test_v3_camera_geometry_ssot.py",
    "tests/feature/test_v3_person_geometry_projection.py",
)


def _replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly one source marker, found {count}")
    return text.replace(old, new, 1)


def _patch_control_json(text: str) -> str:
    old = '''      "person_tracking_enabled": true,\n      "person_camera_horizontal_fov_rad": 1.1519173063162575,\n      "person_camera_yaw_offset_rad": 0.0,\n      "person_lidar_max_skew_ns": 150000000,\n'''
    new = '''      "person_tracking_enabled": true,\n      "person_lidar_max_skew_ns": 150000000,\n'''
    return _replace_once(text, old, new, "remove legacy live camera geometry authority")


def _patch_config(text: str) -> str:
    text = _replace_once(
        text,
        'from v3.adapters.gpio_motor import GpioMotorFrameSinkConfig\n',
        'from v3.adapters.camera_geometry import camera_geometry_config_from_mapping\nfrom v3.adapters.gpio_motor import GpioMotorFrameSinkConfig\n',
        "config camera geometry import",
    )
    text = _replace_once(
        text,
        '''            "world_model": {"local_costmap_max_points_per_scan"},\n''',
        '''            "world_model": {\n                "local_costmap_max_points_per_scan",\n                "person_camera_horizontal_fov_rad",\n                "person_camera_yaw_offset_rad",\n            },\n''',
        "config world-model derived geometry fields",
    )
    text = _replace_once(
        text,
        '''        layers["estimation"] = {**layers["estimation"], "track_width_m":p["nyomtav_szelesseg_m"], "frame_id":POSE_FRAME_ID}\n        layers["navigation"] = {**layers["navigation"], "footprint_length_m":p["footprint_length_m"], "footprint_width_m":p["footprint_width_m"]}\n        layers["world_model"] = {**layers["world_model"], "local_costmap_max_points_per_scan":local["max_points"]}\n''',
        '''        camera_geometry = None\n        camera_value = h.get("camera")\n        if isinstance(camera_value, dict) and camera_value.get("enabled") is True:\n            geometry_value = camera_value.get("geometry")\n            if not isinstance(geometry_value, dict):\n                raise ValueError("enabled camera requires hardver.camera.geometry")\n            camera_geometry = camera_geometry_config_from_mapping(geometry_value)\n        world_tracking_enabled = layers["world_model"].get("person_tracking_enabled")\n        if type(world_tracking_enabled) is not bool:\n            raise ValueError("layers.world_model.person_tracking_enabled must be bool")\n        if world_tracking_enabled and camera_geometry is None:\n            raise ValueError("person tracking requires enabled camera geometry")\n\n        layers["estimation"] = {**layers["estimation"], "track_width_m":p["nyomtav_szelesseg_m"], "frame_id":POSE_FRAME_ID}\n        layers["navigation"] = {**layers["navigation"], "footprint_length_m":p["footprint_length_m"], "footprint_width_m":p["footprint_width_m"]}\n        layers["world_model"] = {\n            **layers["world_model"],\n            "local_costmap_max_points_per_scan": local["max_points"],\n            # Compatibility-only fallback for historical observations/captures.\n            # New live person detections carry frame-specific projected bearings.\n            "person_camera_horizontal_fov_rad": (\n                None\n                if camera_geometry is None\n                else math.radians(camera_geometry.factory.horizontal_fov_deg)\n            ),\n            "person_camera_yaw_offset_rad": (\n                None\n                if camera_geometry is None\n                else math.radians(camera_geometry.mount.yaw_deg)\n            ),\n        }\n''',
        "config derive legacy fallback from geometry SSOT",
    )
    return text


def _patch_l4(text: str) -> str:
    text = _replace_once(
        text,
        '''    person_tracking_enabled: bool\n    person_camera_horizontal_fov_rad: float  # 66 deg Camera Module 3\n    person_camera_yaw_offset_rad: float\n    person_lidar_max_skew_ns: int\n''',
        '''    person_tracking_enabled: bool\n    # Compatibility-only fallback for historical captures that predate projected\n    # person bearings. New live observations receive geometry from the vision owner.\n    person_camera_horizontal_fov_rad: float | None\n    person_camera_yaw_offset_rad: float | None\n    person_lidar_max_skew_ns: int\n''',
        "l4 legacy camera fields",
    )
    text = _replace_once(
        text,
        '''        if (\n            not math.isfinite(self.person_camera_horizontal_fov_rad)\n            or not 0.0 < self.person_camera_horizontal_fov_rad < math.pi\n        ):\n            raise ValueError("person_camera_horizontal_fov_rad must be in (0, pi)")\n        if not math.isfinite(self.person_camera_yaw_offset_rad):\n            raise ValueError("person_camera_yaw_offset_rad must be finite")\n''',
        '''        if self.person_tracking_enabled and (\n            self.person_camera_horizontal_fov_rad is None\n            or self.person_camera_yaw_offset_rad is None\n        ):\n            raise ValueError("person tracking requires legacy camera projection fallback")\n        if self.person_camera_horizontal_fov_rad is not None and (\n            not math.isfinite(self.person_camera_horizontal_fov_rad)\n            or not 0.0 < self.person_camera_horizontal_fov_rad < math.pi\n        ):\n            raise ValueError("person_camera_horizontal_fov_rad must be in (0, pi) or None")\n        if self.person_camera_yaw_offset_rad is not None and not math.isfinite(\n            self.person_camera_yaw_offset_rad\n        ):\n            raise ValueError("person_camera_yaw_offset_rad must be finite or None")\n''',
        "l4 legacy fallback validation",
    )
    text = _replace_once(
        text,
        '''class _PersonImageDetection:\n    confidence: float\n    xmin: float\n    xmax: float\n    ymin: float | None = None\n    ymax: float | None = None\n''',
        '''class _PersonImageDetection:\n    confidence: float\n    xmin: float\n    xmax: float\n    ymin: float | None = None\n    ymax: float | None = None\n    left_bearing_rad: float | None = None\n    right_bearing_rad: float | None = None\n''',
        "l4 projected person bearings",
    )
    old_method = '''    def _person_image_detections(self, observation: Observation) -> tuple[_PersonImageDetection, ...]:\n        values = _values(observation)\n        detected = _boolean(values, "person_detected")\n        if not detected:\n            return ()\n        emitted = values.get("emitted_person_count")\n        if emitted is None:\n            confidence = _unit_number(values, "primary_confidence")\n            xmin = _unit_number(values, "primary_xmin")\n            xmax = _unit_number(values, "primary_xmax")\n            if xmax <= xmin:\n                raise ValueError("person detection bounding box must have positive width")\n            ymin, ymax = self._person_vertical_extent(values, "primary")\n            return (_PersonImageDetection(confidence, xmin, xmax, ymin, ymax),)\n        count = _integer(values, "emitted_person_count")\n        detections: list[_PersonImageDetection] = []\n        for index in range(count):\n            prefix = f"person_{index:03d}"\n            confidence = _unit_number(values, f"{prefix}_confidence")\n            xmin = _unit_number(values, f"{prefix}_xmin")\n            xmax = _unit_number(values, f"{prefix}_xmax")\n            if xmax <= xmin:\n                raise ValueError("person detection bounding box must have positive width")\n            ymin, ymax = self._person_vertical_extent(values, prefix)\n            detections.append(_PersonImageDetection(confidence, xmin, xmax, ymin, ymax))\n        return tuple(detections)\n'''
    new_method = '''    def _person_image_detections(self, observation: Observation) -> tuple[_PersonImageDetection, ...]:\n        values = _values(observation)\n        detected = _boolean(values, "person_detected")\n        if not detected:\n            return ()\n        projection_state = values.get("geometry_projection_state")\n        if projection_state is not None:\n            if projection_state not in {"VALID", "DEGRADED", "INVALID"}:\n                raise ValueError("person detection geometry_projection_state is invalid")\n            if projection_state == "INVALID":\n                # Preserve 2D detection capability, but fail closed for spatial fusion.\n                return ()\n        emitted = values.get("emitted_person_count")\n        if emitted is None:\n            confidence = _unit_number(values, "primary_confidence")\n            xmin = _unit_number(values, "primary_xmin")\n            xmax = _unit_number(values, "primary_xmax")\n            if xmax <= xmin:\n                raise ValueError("person detection bounding box must have positive width")\n            ymin, ymax = self._person_vertical_extent(values, "primary")\n            left, right = self._person_projected_bearings(values, "primary", projection_state)\n            return (_PersonImageDetection(confidence, xmin, xmax, ymin, ymax, left, right),)\n        count = _integer(values, "emitted_person_count")\n        detections: list[_PersonImageDetection] = []\n        for index in range(count):\n            prefix = f"person_{index:03d}"\n            confidence = _unit_number(values, f"{prefix}_confidence")\n            xmin = _unit_number(values, f"{prefix}_xmin")\n            xmax = _unit_number(values, f"{prefix}_xmax")\n            if xmax <= xmin:\n                raise ValueError("person detection bounding box must have positive width")\n            ymin, ymax = self._person_vertical_extent(values, prefix)\n            left, right = self._person_projected_bearings(values, prefix, projection_state)\n            detections.append(\n                _PersonImageDetection(confidence, xmin, xmax, ymin, ymax, left, right)\n            )\n        return tuple(detections)\n\n    @staticmethod\n    def _person_projected_bearings(\n        values: dict[str, object],\n        prefix: str,\n        projection_state: object,\n    ) -> tuple[float | None, float | None]:\n        left_key = f"{prefix}_bearing_left_rad"\n        right_key = f"{prefix}_bearing_right_rad"\n        has_left = left_key in values\n        has_right = right_key in values\n        if projection_state is None:\n            if has_left or has_right:\n                raise ValueError("legacy person detection must not carry partial projected bearings")\n            return None, None\n        if projection_state == "INVALID":\n            return None, None\n        if not has_left or not has_right:\n            raise ValueError("projected person detection is missing bearing bounds")\n        return _wrapped_angle(_number(values, left_key)), _wrapped_angle(_number(values, right_key))\n'''
    text = _replace_once(text, old_method, new_method, "l4 consume projected person bearings")
    text = _replace_once(
        text,
        '''        for detection in detections:\n            left = self._pixel_bearing(detection.xmin)\n            right = self._pixel_bearing(detection.xmax)\n            lower = min(left, right) - self._config.person_lidar_angular_margin_rad\n''',
        '''        for detection in detections:\n            if detection.left_bearing_rad is None:\n                left = self._legacy_pixel_bearing(detection.xmin)\n                right = self._legacy_pixel_bearing(detection.xmax)\n            else:\n                if detection.right_bearing_rad is None:\n                    raise ValueError("projected person detection has incomplete bearing bounds")\n                left = detection.left_bearing_rad\n                right = detection.right_bearing_rad\n            lower = min(left, right) - self._config.person_lidar_angular_margin_rad\n''',
        "l4 use projected bearings",
    )
    text = _replace_once(
        text,
        '''    def _pixel_bearing(self, normalized_x: float) -> float:\n        focal = 0.5 / math.tan(self._config.person_camera_horizontal_fov_rad * 0.5)\n        return self._config.person_camera_yaw_offset_rad + math.atan2(0.5 - normalized_x, focal)\n''',
        '''    def _legacy_pixel_bearing(self, normalized_x: float) -> float:\n        # Historical replay fallback only. Live detections carry geometry-owner bearings.\n        fov = self._config.person_camera_horizontal_fov_rad\n        yaw = self._config.person_camera_yaw_offset_rad\n        if fov is None or yaw is None:\n            raise ValueError("legacy person bearing fallback is unavailable")\n        focal = 0.5 / math.tan(fov * 0.5)\n        return yaw + math.atan2(0.5 - normalized_x, focal)\n''',
        "l4 legacy pixel bearing rename",
    )
    return text


def _patch_person_detection(text: str) -> str:
    text = _replace_once(
        text,
        'from typing import Protocol\n\n\n',
        'from typing import Protocol\n\nfrom .camera_geometry import (\n    CameraGeometryConfig,\n    CameraGeometryStatus,\n    effective_geometry,\n)\n\n\n',
        "person detection geometry imports",
    )
    text = _replace_once(
        text,
        '''@dataclass(frozen=True, slots=True)\nclass PersonDetection:\n    confidence: float\n    box: PersonBox\n\n    def __post_init__(self) -> None:\n        _unit(self.confidence, "confidence")\n        if not isinstance(self.box, PersonBox):\n            raise TypeError("box must be PersonBox")\n\n\n@dataclass(frozen=True, slots=True)\nclass PersonDetectionSnapshot:\n''',
        '''@dataclass(frozen=True, slots=True)\nclass PersonDetection:\n    confidence: float\n    box: PersonBox\n\n    def __post_init__(self) -> None:\n        _unit(self.confidence, "confidence")\n        if not isinstance(self.box, PersonBox):\n            raise TypeError("box must be PersonBox")\n\n\n@dataclass(frozen=True, slots=True)\nclass PersonDetectionProjection:\n    """Compact camera-geometry result in the robot base-frame convention."""\n\n    left_bearing_rad: float\n    right_bearing_rad: float\n    geometry_quality: str\n\n    def __post_init__(self) -> None:\n        for value, name in (\n            (self.left_bearing_rad, "left_bearing_rad"),\n            (self.right_bearing_rad, "right_bearing_rad"),\n        ):\n            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):\n                raise ValueError(f"{name} must be finite")\n        if self.geometry_quality not in {"factory_nominal", "empirical", "degraded"}:\n            raise ValueError("geometry_quality is invalid")\n\n\n@dataclass(frozen=True, slots=True)\nclass PersonDetectionSnapshot:\n''',
        "person projection contract",
    )
    text = _replace_once(
        text,
        '''    inference_duration_ns: int\n    detections: tuple[PersonDetection, ...]\n\n    def __post_init__(self) -> None:\n''',
        '''    inference_duration_ns: int\n    detections: tuple[PersonDetection, ...]\n    geometry_state: str | None = None\n    geometry_reason: str | None = None\n    projections: tuple[PersonDetectionProjection, ...] = ()\n\n    def __post_init__(self) -> None:\n''',
        "person snapshot geometry fields",
    )
    text = _replace_once(
        text,
        '''        if tuple(sorted(self.detections, key=lambda item: item.confidence, reverse=True)) != self.detections:\n            raise ValueError("detections must be sorted by descending confidence")\n\n    @property\n''',
        '''        if tuple(sorted(self.detections, key=lambda item: item.confidence, reverse=True)) != self.detections:\n            raise ValueError("detections must be sorted by descending confidence")\n        if self.geometry_state is None:\n            if self.geometry_reason is not None or self.projections:\n                raise ValueError("legacy detection snapshots cannot carry geometry projection data")\n        else:\n            if self.geometry_state not in {"VALID", "DEGRADED", "INVALID"}:\n                raise ValueError("geometry_state is invalid")\n            if self.geometry_reason is not None and not isinstance(self.geometry_reason, str):\n                raise TypeError("geometry_reason must be str or None")\n            if not isinstance(self.projections, tuple) or not all(\n                isinstance(item, PersonDetectionProjection) for item in self.projections\n            ):\n                raise TypeError("projections must be tuple[PersonDetectionProjection, ...]")\n            if self.geometry_state == "INVALID":\n                if self.projections:\n                    raise ValueError("invalid geometry must not publish projected bearings")\n            elif len(self.projections) != len(self.detections):\n                raise ValueError("geometry projections must preserve detection ordering")\n\n    @property\n''',
        "person snapshot geometry validation",
    )
    text = _replace_once(
        text,
        '''        "_backend",\n        "_camera",\n        "_condition",\n''',
        '''        "_backend",\n        "_camera",\n        "_camera_geometry_config",\n        "_condition",\n''',
        "person detector geometry slot",
    )
    text = _replace_once(
        text,
        '''        camera: CameraFramePortLike,\n        backend: PersonDetectorBackend,\n        *,\n        worker_poll_s: float = 0.20,\n''',
        '''        camera: CameraFramePortLike,\n        backend: PersonDetectorBackend,\n        *,\n        camera_geometry_config: CameraGeometryConfig | None = None,\n        worker_poll_s: float = 0.20,\n''',
        "person detector geometry arg",
    )
    text = _replace_once(
        text,
        '''        if not callable(getattr(backend, "detect", None)):\n            raise TypeError("backend must provide detect")\n        for value, name in (\n''',
        '''        if not callable(getattr(backend, "detect", None)):\n            raise TypeError("backend must provide detect")\n        if camera_geometry_config is not None and not isinstance(\n            camera_geometry_config, CameraGeometryConfig\n        ):\n            raise TypeError("camera_geometry_config must be CameraGeometryConfig or None")\n        for value, name in (\n''',
        "person detector geometry validation",
    )
    text = _replace_once(
        text,
        '''        self._camera = camera\n        self._backend = backend\n        self._worker_poll_s = float(worker_poll_s)\n''',
        '''        self._camera = camera\n        self._backend = backend\n        self._camera_geometry_config = camera_geometry_config\n        self._worker_poll_s = float(worker_poll_s)\n''',
        "person detector geometry assignment",
    )
    text = _replace_once(
        text,
        '''                ordered = tuple(sorted(detections, key=lambda item: item.confidence, reverse=True))\n                with self._condition:\n                    self._result_sequence += 1\n                    self._latest = PersonDetectionSnapshot(\n                        sequence=self._result_sequence,\n                        source_frame_sequence=frame.sequence,\n                        measurement_monotonic_ns=frame.measurement_monotonic_ns,\n                        completed_monotonic_ns=completed_ns,\n                        inference_duration_ns=max(0, completed_ns - started_ns),\n                        detections=ordered,\n                    )\n''',
        '''                ordered = tuple(sorted(detections, key=lambda item: item.confidence, reverse=True))\n                geometry_state, geometry_reason, projections = self._project_detections(\n                    frame, ordered\n                )\n                with self._condition:\n                    self._result_sequence += 1\n                    self._latest = PersonDetectionSnapshot(\n                        sequence=self._result_sequence,\n                        source_frame_sequence=frame.sequence,\n                        measurement_monotonic_ns=frame.measurement_monotonic_ns,\n                        completed_monotonic_ns=completed_ns,\n                        inference_duration_ns=max(0, completed_ns - started_ns),\n                        detections=ordered,\n                        geometry_state=geometry_state,\n                        geometry_reason=geometry_reason,\n                        projections=projections,\n                    )\n''',
        "person detector publish projections",
    )
    insert_marker = '''    def _run(self) -> None:\n'''
    helper = '''    def _project_detections(\n        self,\n        frame: CameraFrameLike,\n        detections: tuple[PersonDetection, ...],\n    ) -> tuple[str | None, str | None, tuple[PersonDetectionProjection, ...]]:\n        config = self._camera_geometry_config\n        if config is None:\n            return None, None, ()\n        try:\n            status_getter = getattr(self._camera, "get_camera_geometry_status", None)\n            status = status_getter() if callable(status_getter) else None\n            if status is not None and not isinstance(status, CameraGeometryStatus):\n                return "INVALID", "CAMERA_GEOMETRY_STATUS_INVALID", ()\n            if status is not None and status.state == "INVALID":\n                return "INVALID", status.reason or "CAMERA_GEOMETRY_INVALID", ()\n            runtime_status = status\n            state = "DEGRADED" if status is None else status.state\n            reason = (\n                "CAMERA_GEOMETRY_STATUS_UNAVAILABLE"\n                if status is None\n                else (status.reason or None)\n            )\n            geometry = effective_geometry(\n                config,\n                output_width_px=frame.width,\n                output_height_px=frame.height,\n                sensor_crop=getattr(frame, "sensor_crop", None),\n                runtime_status=runtime_status,\n            )\n            projection_quality = (\n                "degraded" if state == "DEGRADED" else geometry.geometry_quality\n            )\n            projections: list[PersonDetectionProjection] = []\n            for detection in detections:\n                v_px = detection.box.center_y * frame.height\n                left_ray = geometry.pixel_to_base_ray(detection.box.xmin * frame.width, v_px)\n                right_ray = geometry.pixel_to_base_ray(detection.box.xmax * frame.width, v_px)\n                projections.append(\n                    PersonDetectionProjection(\n                        left_bearing_rad=math.atan2(left_ray[1], left_ray[0]),\n                        right_bearing_rad=math.atan2(right_ray[1], right_ray[0]),\n                        geometry_quality=projection_quality,\n                    )\n                )\n            return state, reason, tuple(projections)\n        except Exception as exc:\n            # 2D detections stay usable, but geometry-dependent spatial fusion must not.\n            return "INVALID", f"{type(exc).__name__}:{exc}", ()\n\n'''
    text = _replace_once(text, insert_marker, helper + insert_marker, "person detector projection helper")
    text = _replace_once(
        text,
        '''    "PersonDetection",\n    "PersonDetectionPort",\n''',
        '''    "PersonDetection",\n    "PersonDetectionProjection",\n    "PersonDetectionPort",\n''',
        "person projection export",
    )
    return text


def _patch_live_person(text: str) -> str:
    text = _replace_once(
        text,
        '''        stale = self._last_capability.state is CapabilityState.STALE\n        state = DeviceHealthState.DEGRADED if stale else DeviceHealthState.OK\n        reason = "PERSON_DETECTOR_RESULT_STALE" if stale else None\n        detections = result.detections[: self._config.maximum_detections]\n''',
        '''        stale = self._last_capability.state is CapabilityState.STALE\n        geometry_degraded = result.geometry_state in {"DEGRADED", "INVALID"}\n        state = (\n            DeviceHealthState.DEGRADED\n            if stale or geometry_degraded\n            else DeviceHealthState.OK\n        )\n        if stale:\n            reason = "PERSON_DETECTOR_RESULT_STALE"\n        elif result.geometry_state == "INVALID":\n            reason = "PERSON_GEOMETRY_INVALID"\n        elif result.geometry_state == "DEGRADED":\n            reason = "PERSON_GEOMETRY_DEGRADED"\n        else:\n            reason = None\n        detections = result.detections[: self._config.maximum_detections]\n''',
        "live person geometry health",
    )
    text = _replace_once(
        text,
        '''        if primary is not None:\n            values.extend(\n                (\n                    DataField("primary_confidence", primary.confidence),\n                    DataField("primary_xmin", primary.box.xmin),\n                    DataField("primary_ymin", primary.box.ymin),\n                    DataField("primary_xmax", primary.box.xmax),\n                    DataField("primary_ymax", primary.box.ymax),\n                    DataField("primary_center_x", primary.box.center_x),\n                    DataField("primary_center_y", primary.box.center_y),\n                    DataField("primary_area", primary.box.area),\n                )\n            )\n        for index, detection in enumerate(detections):\n''',
        '''        if result.geometry_state is not None:\n            values.append(DataField("geometry_projection_state", result.geometry_state))\n            if result.geometry_reason:\n                values.append(DataField("geometry_projection_reason", result.geometry_reason))\n        projections = result.projections[: len(detections)]\n        if primary is not None:\n            values.extend(\n                (\n                    DataField("primary_confidence", primary.confidence),\n                    DataField("primary_xmin", primary.box.xmin),\n                    DataField("primary_ymin", primary.box.ymin),\n                    DataField("primary_xmax", primary.box.xmax),\n                    DataField("primary_ymax", primary.box.ymax),\n                    DataField("primary_center_x", primary.box.center_x),\n                    DataField("primary_center_y", primary.box.center_y),\n                    DataField("primary_area", primary.box.area),\n                )\n            )\n            if projections:\n                values.extend(\n                    (\n                        DataField("primary_bearing_left_rad", projections[0].left_bearing_rad),\n                        DataField("primary_bearing_right_rad", projections[0].right_bearing_rad),\n                        DataField("primary_geometry_quality", projections[0].geometry_quality),\n                    )\n                )\n        for index, detection in enumerate(detections):\n''',
        "live person projection metadata",
    )
    text = _replace_once(
        text,
        '''            values.extend(\n                (\n                    DataField(f"{prefix}_confidence", detection.confidence),\n                    DataField(f"{prefix}_xmin", box.xmin),\n                    DataField(f"{prefix}_ymin", box.ymin),\n                    DataField(f"{prefix}_xmax", box.xmax),\n                    DataField(f"{prefix}_ymax", box.ymax),\n                    DataField(f"{prefix}_center_x", box.center_x),\n                    DataField(f"{prefix}_center_y", box.center_y),\n                    DataField(f"{prefix}_area", box.area),\n                )\n            )\n''',
        '''            values.extend(\n                (\n                    DataField(f"{prefix}_confidence", detection.confidence),\n                    DataField(f"{prefix}_xmin", box.xmin),\n                    DataField(f"{prefix}_ymin", box.ymin),\n                    DataField(f"{prefix}_xmax", box.xmax),\n                    DataField(f"{prefix}_ymax", box.ymax),\n                    DataField(f"{prefix}_center_x", box.center_x),\n                    DataField(f"{prefix}_center_y", box.center_y),\n                    DataField(f"{prefix}_area", box.area),\n                )\n            )\n            if index < len(projections):\n                projection = projections[index]\n                values.extend(\n                    (\n                        DataField(f"{prefix}_bearing_left_rad", projection.left_bearing_rad),\n                        DataField(f"{prefix}_bearing_right_rad", projection.right_bearing_rad),\n                        DataField(f"{prefix}_geometry_quality", projection.geometry_quality),\n                    )\n                )\n''',
        "live person indexed projection metadata",
    )
    return text


def _patch_process_vision(text: str) -> str:
    return _replace_once(
        text,
        '            detector = NativePersonDetector(camera, backend)\n',
        '            detector = NativePersonDetector(\n                camera, backend, camera_geometry_config=camera_geometry\n            )\n',
        "process vision detector geometry",
    )


def _patch_hardware_runtime(text: str) -> str:
    return _replace_once(
        text,
        '                            detector = NativePersonDetector(camera, backend)\n',
        '                            detector = NativePersonDetector(\n                                camera,\n                                backend,\n                                camera_geometry_config=config.camera_geometry,\n                            )\n',
        "direct vision detector geometry",
    )


def _patch_person_tool(text: str) -> str:
    text = _replace_once(
        text,
        '''from v3.adapters.litert_person_detector import (  # noqa: E402\n''',
        '''from v3.adapters.camera_geometry import camera_geometry_config_from_mapping  # noqa: E402\nfrom v3.adapters.litert_person_detector import (  # noqa: E402\n''',
        "person tool geometry import",
    )
    text = _replace_once(
        text,
        '''    camera = {\n        key: value\n        for key, value in camera.items()\n        if key not in {"person_detection", "geometry"}\n    }\n    return picamera2_camera_config_from_mapping(camera)\n''',
        '''    geometry = camera.get("geometry")\n    if not isinstance(geometry, dict):\n        raise SystemExit("conf/hardver.json enabled camera requires geometry")\n    camera_geometry = camera_geometry_config_from_mapping(geometry)\n    camera_device = {\n        key: value\n        for key, value in camera.items()\n        if key not in {"person_detection", "geometry"}\n    }\n    return picamera2_camera_config_from_mapping(camera_device), camera_geometry\n''',
        "person tool parse geometry",
    )
    text = _replace_once(
        text,
        '''    _refuse_parallel_runtime_owner()\n    camera = NativePicamera2Camera(\n        _camera_config(),\n        picamera_factory=default_picamera2_factory,\n''',
        '''    _refuse_parallel_runtime_owner()\n    camera_config, camera_geometry = _camera_config()\n    camera = NativePicamera2Camera(\n        camera_config,\n        camera_geometry_config=camera_geometry,\n        picamera_factory=default_picamera2_factory,\n''',
        "person tool camera geometry",
    )
    text = _replace_once(
        text,
        '        detector = NativePersonDetector(camera, backend)\n',
        '        detector = NativePersonDetector(\n            camera, backend, camera_geometry_config=camera_geometry\n        )\n',
        "person tool detector geometry",
    )
    text = _replace_once(
        text,
        '''                    "inference_ms": result.inference_duration_ns / 1_000_000.0,\n                    "primary": (\n''',
        '''                    "inference_ms": result.inference_duration_ns / 1_000_000.0,\n                    "geometry_state": result.geometry_state,\n                    "geometry_reason": result.geometry_reason,\n                    "primary_bearing": (\n                        {\n                            "left_rad": result.projections[0].left_bearing_rad,\n                            "right_rad": result.projections[0].right_bearing_rad,\n                            "quality": result.projections[0].geometry_quality,\n                        }\n                        if result.projections\n                        else None\n                    ),\n                    "primary": (\n''',
        "person tool geometry output",
    )
    return text


PATCHERS = {
    "conf/vezerles.json": _patch_control_json,
    "v3/config.py": _patch_config,
    "v3/layers/l4_world_model.py": _patch_l4,
    "v3/adapters/person_detection.py": _patch_person_detection,
    "v3/adapters/live_person_detection.py": _patch_live_person,
    "v3/adapters/process_vision_port.py": _patch_process_vision,
    "v3_hardware_runtime.py": _patch_hardware_runtime,
    "tools/v3_person_detection_test.py": _patch_person_tool,
}


def _git(root: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", *args], cwd=root, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE
    )
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or f"git {' '.join(args)} failed")
    return result.stdout.strip()


def _blob_hash(root: Path, rel: str) -> str:
    return _git(root, "hash-object", rel)


def _payload_root() -> Path:
    return Path(__file__).resolve().parent / "payload"


def _prepare(root: Path, *, allow_head_mismatch: bool, allow_dirty: bool) -> dict[str, str]:
    if not (root / ".git").exists():
        raise RuntimeError(f"not a git working tree: {root}")
    head = _git(root, "rev-parse", "HEAD")
    if head != EXPECTED_HEAD and not allow_head_mismatch:
        raise RuntimeError(
            f"HEAD mismatch: package targets {EXPECTED_HEAD}, current {head}; "
            "review first or use --allow-head-mismatch"
        )
    patched: dict[str, str] = {}
    for rel, patcher in PATCHERS.items():
        path = root / rel
        if not path.is_file():
            raise RuntimeError(f"missing target file: {rel}")
        actual_blob = _blob_hash(root, rel)
        expected_blob = EXPECTED_BLOBS[rel]
        if actual_blob != expected_blob and not allow_dirty:
            raise RuntimeError(
                f"source drift/local modification in {rel}: expected blob {expected_blob}, "
                f"found {actual_blob}; refuse to overwrite without --allow-dirty"
            )
        text = path.read_text(encoding="utf-8")
        patched[rel] = patcher(text)
        if patched[rel] == text:
            raise RuntimeError(f"patch made no change: {rel}")
    for rel in NEW_FILES:
        source = _payload_root() / rel
        if not source.is_file():
            raise RuntimeError(f"package payload missing: {rel}")
        destination = root / rel
        if destination.exists() and not allow_dirty:
            raise RuntimeError(f"new file already exists: {rel}; refuse without --allow-dirty")
    # Validate resulting JSON before touching the working tree.
    json.loads(patched["conf/vezerles.json"])
    return patched


def _restore(root: Path, backup: Path, created: list[str]) -> None:
    for rel in PATCHERS:
        source = backup / rel
        if source.is_file():
            target = root / rel
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
    for rel in created:
        try:
            (root / rel).unlink()
        except FileNotFoundError:
            pass


def apply(root: Path, *, check_only: bool, allow_head_mismatch: bool, allow_dirty: bool) -> None:
    root = root.resolve()
    patched = _prepare(
        root, allow_head_mismatch=allow_head_mismatch, allow_dirty=allow_dirty
    )
    if check_only:
        print(json.dumps({
            "status": "CHECK_OK",
            "upgrade_id": UPGRADE_ID,
            "target_head": EXPECTED_HEAD,
            "files_to_patch": sorted(PATCHERS),
            "files_to_add": list(NEW_FILES),
        }, indent=2))
        return

    backup = Path(tempfile.mkdtemp(prefix=f"r2b4-{UPGRADE_ID}-", dir="/tmp"))
    created: list[str] = []
    try:
        for rel in PATCHERS:
            source = root / rel
            target = backup / rel
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
        for rel, text in patched.items():
            target = root / rel
            temp = target.with_name(target.name + ".upgrade.tmp")
            original_mode = target.stat().st_mode
            temp.write_text(text, encoding="utf-8")
            os.chmod(temp, original_mode & 0o7777)
            os.replace(temp, target)
        for rel in NEW_FILES:
            source = _payload_root() / rel
            target = root / rel
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
            created.append(rel)

        python_files = [rel for rel in PATCHERS if rel.endswith(".py")] + list(NEW_FILES)
        for rel in python_files:
            py_compile.compile(str(root / rel), doraise=True)
        targeted = subprocess.run(
            [
                sys.executable,
                "-m",
                "pytest",
                "-q",
                "tests/core/test_v3_camera_geometry_ssot.py",
                "tests/feature/test_v3_person_geometry_projection.py",
            ],
            cwd=root,
        )
        if targeted.returncode != 0:
            raise RuntimeError(
                f"targeted regression tests failed with exit code {targeted.returncode}"
            )
    except BaseException:
        _restore(root, backup, created)
        raise

    print(json.dumps({
        "status": "APPLIED",
        "upgrade_id": UPGRADE_ID,
        "target_head": EXPECTED_HEAD,
        "backup": str(backup),
        "patched": sorted(PATCHERS),
        "added": list(NEW_FILES),
        "next": [
            "./r test",
            "./r pytest -q tests/core/test_v3_camera_geometry_ssot.py tests/feature/test_v3_person_geometry_projection.py",
            "./r test perception",
            "./r test follow",
            "./r test process",
        ],
    }, indent=2))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Apply R2B4 camera/FP geometry SSOT upgrade")
    parser.add_argument("root", nargs="?", default="/home/alba/project_r2b4")
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--allow-head-mismatch", action="store_true")
    parser.add_argument("--allow-dirty", action="store_true")
    args = parser.parse_args(argv)
    try:
        apply(
            Path(args.root),
            check_only=args.check,
            allow_head_mismatch=args.allow_head_mismatch,
            allow_dirty=args.allow_dirty,
        )
        return 0
    except Exception as exc:
        print(f"UPGRADE_FAILED: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
