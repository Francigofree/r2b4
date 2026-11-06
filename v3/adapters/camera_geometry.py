"""Camera geometry SSOT for the R2B4 front Camera Module 3 Wide NoIR.

This module deliberately separates acquisition policy from geometry.  Factory
optical data are nominal seeds, not an empirical OpenCV calibration.  In
particular, an unknown distortion model is never represented as D=0.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass


_GEOMETRY_STATES = {"VALID", "DEGRADED", "INVALID"}
_DISTORTION_MODELS = {"unknown", "opencv_pinhole", "opencv_fisheye"}
_INTRINSIC_SOURCES = {"factory_nominal", "empirical"}


def _finite(value: object, name: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
    ):
        raise ValueError(f"{name} must be finite")
    return float(value)


def _positive(value: object, name: str) -> float:
    result = _finite(value, name)
    if result <= 0.0:
        raise ValueError(f"{name} must be positive")
    return result


def _positive_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _mapping(value: object, name: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be an object")
    return value


def _nonempty(value: object, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty string")
    return value.strip()


def _pair(value: object) -> tuple[int, int] | None:
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)) and len(value) == 2:
        left, right = value[0], value[1]
    elif hasattr(value, "width") and hasattr(value, "height"):
        left, right = getattr(value, "width"), getattr(value, "height")
    else:
        return None
    if any(not isinstance(item, int) or isinstance(item, bool) or item <= 0 for item in (left, right)):
        return None
    return int(left), int(right)


@dataclass(frozen=True, slots=True)
class SensorCrop:
    x: int
    y: int
    width: int
    height: int

    def __post_init__(self) -> None:
        for value, name in ((self.x, "x"), (self.y, "y")):
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                raise ValueError(f"sensor crop {name} must be a non-negative integer")
        _positive_int(self.width, "sensor crop width")
        _positive_int(self.height, "sensor crop height")


def sensor_crop_from_value(value: object) -> SensorCrop | None:
    if value is None:
        return None
    if isinstance(value, SensorCrop):
        return value
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)) and len(value) == 4:
        raw = tuple(value)
    elif all(hasattr(value, name) for name in ("x", "y", "width", "height")):
        raw = (
            getattr(value, "x"),
            getattr(value, "y"),
            getattr(value, "width"),
            getattr(value, "height"),
        )
    else:
        return None
    if any(not isinstance(item, int) or isinstance(item, bool) for item in raw):
        return None
    try:
        return SensorCrop(int(raw[0]), int(raw[1]), int(raw[2]), int(raw[3]))
    except ValueError:
        return None


@dataclass(frozen=True, slots=True)
class CameraFactoryGeometry:
    profile_name: str
    sensor_model: str
    native_width_px: int
    native_height_px: int
    pixel_pitch_x_um: float
    pixel_pitch_y_um: float
    focal_length_mm: float
    horizontal_fov_deg: float
    vertical_fov_deg: float
    diagonal_fov_deg: float
    tv_distortion_bound: float
    ir_cut_filter: bool

    def __post_init__(self) -> None:
        _nonempty(self.profile_name, "profile_name")
        _nonempty(self.sensor_model, "sensor_model")
        _positive_int(self.native_width_px, "native_width_px")
        _positive_int(self.native_height_px, "native_height_px")
        _positive(self.pixel_pitch_x_um, "pixel_pitch_x_um")
        _positive(self.pixel_pitch_y_um, "pixel_pitch_y_um")
        _positive(self.focal_length_mm, "focal_length_mm")
        for value, name in (
            (self.horizontal_fov_deg, "horizontal_fov_deg"),
            (self.vertical_fov_deg, "vertical_fov_deg"),
            (self.diagonal_fov_deg, "diagonal_fov_deg"),
        ):
            angle = _positive(value, name)
            if angle >= 180.0:
                raise ValueError(f"{name} must be below 180 degrees")
        bound = _finite(self.tv_distortion_bound, "tv_distortion_bound")
        if not 0.0 <= bound < 1.0:
            raise ValueError("tv_distortion_bound must be in [0, 1)")
        if type(self.ir_cut_filter) is not bool:
            raise TypeError("ir_cut_filter must be bool")

    @property
    def nominal_fx_px(self) -> float:
        return self.focal_length_mm / (self.pixel_pitch_x_um / 1000.0)

    @property
    def nominal_fy_px(self) -> float:
        return self.focal_length_mm / (self.pixel_pitch_y_um / 1000.0)


IMX708_WIDE_NOIR = CameraFactoryGeometry(
    profile_name="imx708_wide_noir",
    sensor_model="imx708",
    native_width_px=4608,
    native_height_px=2592,
    pixel_pitch_x_um=1.4,
    pixel_pitch_y_um=1.4,
    focal_length_mm=2.75,
    horizontal_fov_deg=102.0,
    vertical_fov_deg=67.0,
    diagonal_fov_deg=120.0,
    tv_distortion_bound=0.116,
    ir_cut_filter=False,
)
_FACTORY_PROFILES = {IMX708_WIDE_NOIR.profile_name: IMX708_WIDE_NOIR}


@dataclass(frozen=True, slots=True)
class CameraMountPose:
    x_m: float
    y_m: float
    z_m: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float

    def __post_init__(self) -> None:
        for value, name in (
            (self.x_m, "x_m"),
            (self.y_m, "y_m"),
            (self.z_m, "z_m"),
            (self.roll_deg, "roll_deg"),
            (self.pitch_deg, "pitch_deg"),
            (self.yaw_deg, "yaw_deg"),
        ):
            _finite(value, name)
        if self.z_m < 0.0:
            raise ValueError("camera mount z_m must be non-negative")

    @property
    def base_from_camera(self) -> tuple[tuple[float, float, float, float], ...]:
        """4x4 transform: OpenCV camera frame -> robot base frame.

        Robot: +X forward, +Y left, +Z up.
        Camera: +x image-right, +y image-down, +z optical-forward.
        yaw is positive to robot-left; pitch is positive above the horizon;
        roll is right-handed about the optical axis.
        """
        yaw = math.radians(self.yaw_deg)
        pitch = math.radians(self.pitch_deg)
        roll = math.radians(self.roll_deg)
        forward = (
            math.cos(pitch) * math.cos(yaw),
            math.cos(pitch) * math.sin(yaw),
            math.sin(pitch),
        )
        right0 = (math.sin(yaw), -math.cos(yaw), 0.0)
        down0 = _cross(forward, right0)
        cr, sr = math.cos(roll), math.sin(roll)
        right = tuple(cr * right0[i] + sr * down0[i] for i in range(3))
        down = tuple(-sr * right0[i] + cr * down0[i] for i in range(3))
        return (
            (right[0], down[0], forward[0], self.x_m),
            (right[1], down[1], forward[1], self.y_m),
            (right[2], down[2], forward[2], self.z_m),
            (0.0, 0.0, 0.0, 1.0),
        )


@dataclass(frozen=True, slots=True)
class CameraIntrinsicCalibration:
    source: str
    reference_width_px: int
    reference_height_px: int
    fx_px: float
    fy_px: float
    cx_px: float
    cy_px: float
    distortion_model: str
    distortion_coefficients: tuple[float, ...] | None
    reference_lens_position: float | None = None
    lens_position_tolerance: float | None = None

    def __post_init__(self) -> None:
        if self.source not in _INTRINSIC_SOURCES:
            raise ValueError(f"unsupported intrinsic source: {self.source}")
        _positive_int(self.reference_width_px, "reference_width_px")
        _positive_int(self.reference_height_px, "reference_height_px")
        _positive(self.fx_px, "fx_px")
        _positive(self.fy_px, "fy_px")
        _finite(self.cx_px, "cx_px")
        _finite(self.cy_px, "cy_px")
        if self.distortion_model not in _DISTORTION_MODELS:
            raise ValueError(f"unsupported distortion model: {self.distortion_model}")
        if self.distortion_model == "unknown" and self.distortion_coefficients is not None:
            raise ValueError("unknown distortion must not carry coefficients")
        if self.distortion_coefficients is not None:
            if not self.distortion_coefficients:
                raise ValueError("distortion coefficients must be non-empty or None")
            for index, value in enumerate(self.distortion_coefficients):
                _finite(value, f"distortion_coefficients[{index}]")
        if self.reference_lens_position is not None:
            _finite(self.reference_lens_position, "reference_lens_position")
        if self.lens_position_tolerance is not None:
            _positive(self.lens_position_tolerance, "lens_position_tolerance")
            if self.reference_lens_position is None:
                raise ValueError("lens_position_tolerance requires reference_lens_position")

    @property
    def can_rectify(self) -> bool:
        return self.distortion_model != "unknown" and self.distortion_coefficients is not None


@dataclass(frozen=True, slots=True)
class CameraGeometryConfig:
    factory: CameraFactoryGeometry
    mount: CameraMountPose
    intrinsic: CameraIntrinsicCalibration

    def __post_init__(self) -> None:
        if not isinstance(self.factory, CameraFactoryGeometry):
            raise TypeError("factory must be CameraFactoryGeometry")
        if not isinstance(self.mount, CameraMountPose):
            raise TypeError("mount must be CameraMountPose")
        if not isinstance(self.intrinsic, CameraIntrinsicCalibration):
            raise TypeError("intrinsic must be CameraIntrinsicCalibration")
        if (
            self.intrinsic.reference_width_px != self.factory.native_width_px
            or self.intrinsic.reference_height_px != self.factory.native_height_px
        ):
            raise ValueError("intrinsic reference size must match native sensor geometry")


@dataclass(frozen=True, slots=True)
class CameraEffectiveGeometry:
    output_width_px: int
    output_height_px: int
    sensor_crop: SensorCrop
    fx_px: float
    fy_px: float
    cx_px: float
    cy_px: float
    base_from_camera: tuple[tuple[float, float, float, float], ...]
    geometry_quality: str
    distortion_model: str
    distortion_coefficients: tuple[float, ...] | None

    def __post_init__(self) -> None:
        _positive_int(self.output_width_px, "output_width_px")
        _positive_int(self.output_height_px, "output_height_px")
        if not isinstance(self.sensor_crop, SensorCrop):
            raise TypeError("sensor_crop must be SensorCrop")
        _positive(self.fx_px, "fx_px")
        _positive(self.fy_px, "fy_px")
        _finite(self.cx_px, "cx_px")
        _finite(self.cy_px, "cy_px")
        if self.geometry_quality not in {"factory_nominal", "empirical", "degraded"}:
            raise ValueError("invalid geometry_quality")

    @property
    def K(self) -> tuple[tuple[float, float, float], ...]:
        return (
            (self.fx_px, 0.0, self.cx_px),
            (0.0, self.fy_px, self.cy_px),
            (0.0, 0.0, 1.0),
        )

    def pixel_to_camera_ray(self, u_px: float, v_px: float) -> tuple[float, float, float]:
        """Central-ray approximation in OpenCV camera coordinates.

        With factory_nominal/unknown distortion this is intentionally only an
        approximation, especially near the wide-angle image edges.
        """
        u = _finite(u_px, "u_px")
        v = _finite(v_px, "v_px")
        ray = ((u - self.cx_px) / self.fx_px, (v - self.cy_px) / self.fy_px, 1.0)
        return _normalize(ray)

    def camera_ray_to_base(self, ray: Sequence[float]) -> tuple[float, float, float]:
        if len(ray) != 3:
            raise ValueError("ray must have three components")
        vector = tuple(_finite(ray[i], f"ray[{i}]") for i in range(3))
        matrix = self.base_from_camera
        transformed = tuple(sum(matrix[row][col] * vector[col] for col in range(3)) for row in range(3))
        return _normalize(transformed)

    def pixel_to_base_ray(self, u_px: float, v_px: float) -> tuple[float, float, float]:
        return self.camera_ray_to_base(self.pixel_to_camera_ray(u_px, v_px))

    @property
    def camera_origin_base_m(self) -> tuple[float, float, float]:
        return (
            self.base_from_camera[0][3],
            self.base_from_camera[1][3],
            self.base_from_camera[2][3],
        )

    def ground_intersection(self, u_px: float, v_px: float, *, ground_z_m: float = 0.0) -> tuple[float, float, float] | None:
        ground = _finite(ground_z_m, "ground_z_m")
        origin = self.camera_origin_base_m
        direction = self.pixel_to_base_ray(u_px, v_px)
        if abs(direction[2]) < 1e-12:
            return None
        scale = (ground - origin[2]) / direction[2]
        if scale <= 0.0:
            return None
        return tuple(origin[i] + scale * direction[i] for i in range(3))


@dataclass(frozen=True, slots=True)
class CameraGeometryStatus:
    state: str
    reason: str
    camera_model: str | None
    pixel_array_size: tuple[int, int] | None
    unit_cell_size_nm: tuple[int, int] | None
    rotation: str | None
    sensor_crop: SensorCrop

    def __post_init__(self) -> None:
        if self.state not in _GEOMETRY_STATES:
            raise ValueError("invalid camera geometry state")
        if not isinstance(self.reason, str):
            raise TypeError("reason must be str")
        if not isinstance(self.sensor_crop, SensorCrop):
            raise TypeError("sensor_crop must be SensorCrop")


def factory_profile(name: str) -> CameraFactoryGeometry:
    key = _nonempty(name, "factory_profile").casefold()
    try:
        return _FACTORY_PROFILES[key]
    except KeyError as exc:
        raise ValueError(f"unsupported camera factory_profile: {name}") from exc


def factory_nominal_intrinsic(factory: CameraFactoryGeometry) -> CameraIntrinsicCalibration:
    return CameraIntrinsicCalibration(
        source="factory_nominal",
        reference_width_px=factory.native_width_px,
        reference_height_px=factory.native_height_px,
        fx_px=factory.nominal_fx_px,
        fy_px=factory.nominal_fy_px,
        cx_px=factory.native_width_px / 2.0,
        cy_px=factory.native_height_px / 2.0,
        distortion_model="unknown",
        distortion_coefficients=None,
    )


def camera_geometry_config_from_mapping(value: Mapping[str, object]) -> CameraGeometryConfig:
    geometry = _mapping(value, "camera.geometry")
    allowed = {"factory_profile", "mount", "intrinsic"}
    unknown = sorted(set(geometry) - allowed)
    if unknown:
        raise ValueError("unknown camera.geometry keys: " + ", ".join(unknown))
    factory = factory_profile(_nonempty(geometry.get("factory_profile"), "camera.geometry.factory_profile"))

    mount_raw = _mapping(geometry.get("mount"), "camera.geometry.mount")
    mount_allowed = {"x_m", "y_m", "z_m", "roll_deg", "pitch_deg", "yaw_deg"}
    mount_unknown = sorted(set(mount_raw) - mount_allowed)
    if mount_unknown:
        raise ValueError("unknown camera.geometry.mount keys: " + ", ".join(mount_unknown))
    mount = CameraMountPose(
        x_m=_finite(mount_raw.get("x_m"), "camera.geometry.mount.x_m"),
        y_m=_finite(mount_raw.get("y_m"), "camera.geometry.mount.y_m"),
        z_m=_finite(mount_raw.get("z_m"), "camera.geometry.mount.z_m"),
        roll_deg=_finite(mount_raw.get("roll_deg", 0.0), "camera.geometry.mount.roll_deg"),
        pitch_deg=_finite(mount_raw.get("pitch_deg", 0.0), "camera.geometry.mount.pitch_deg"),
        yaw_deg=_finite(mount_raw.get("yaw_deg", 0.0), "camera.geometry.mount.yaw_deg"),
    )

    intrinsic_raw = _mapping(geometry.get("intrinsic"), "camera.geometry.intrinsic")
    intrinsic_allowed = {
        "source",
        "reference_size",
        "K",
        "distortion_model",
        "distortion_coefficients",
        "reference_lens_position",
        "lens_position_tolerance",
    }
    intrinsic_unknown = sorted(set(intrinsic_raw) - intrinsic_allowed)
    if intrinsic_unknown:
        raise ValueError("unknown camera.geometry.intrinsic keys: " + ", ".join(intrinsic_unknown))
    source = _nonempty(intrinsic_raw.get("source"), "camera.geometry.intrinsic.source").casefold()
    distortion_model = _nonempty(
        intrinsic_raw.get("distortion_model", "unknown"),
        "camera.geometry.intrinsic.distortion_model",
    ).casefold()
    coefficients_raw = intrinsic_raw.get("distortion_coefficients")
    coefficients: tuple[float, ...] | None
    if coefficients_raw is None:
        coefficients = None
    elif isinstance(coefficients_raw, Sequence) and not isinstance(coefficients_raw, (str, bytes)):
        coefficients = tuple(_finite(item, "camera.geometry.intrinsic.distortion_coefficients") for item in coefficients_raw)
    else:
        raise ValueError("camera.geometry.intrinsic.distortion_coefficients must be an array or null")

    if source == "factory_nominal":
        if intrinsic_raw.get("reference_size") is not None or intrinsic_raw.get("K") is not None:
            raise ValueError("factory_nominal intrinsic must derive K from the factory profile")
        if distortion_model != "unknown" or coefficients is not None:
            raise ValueError("factory_nominal intrinsic requires distortion_model=unknown and null coefficients")
        intrinsic = factory_nominal_intrinsic(factory)
    elif source == "empirical":
        size = _pair(intrinsic_raw.get("reference_size"))
        if size is None:
            raise ValueError("empirical intrinsic requires reference_size [width, height]")
        matrix = intrinsic_raw.get("K")
        if not isinstance(matrix, Sequence) or isinstance(matrix, (str, bytes)) or len(matrix) != 3:
            raise ValueError("empirical intrinsic requires a 3x3 K matrix")
        rows = []
        for row_index, row in enumerate(matrix):
            if not isinstance(row, Sequence) or isinstance(row, (str, bytes)) or len(row) != 3:
                raise ValueError("empirical intrinsic requires a 3x3 K matrix")
            rows.append(tuple(_finite(item, f"camera.geometry.intrinsic.K[{row_index}]") for item in row))
        if abs(rows[0][1]) > 1e-12 or rows[2] != (0.0, 0.0, 1.0):
            raise ValueError("empirical K must have zero skew and homogeneous row [0,0,1]")
        if distortion_model == "unknown" or coefficients is None:
            raise ValueError("empirical intrinsic requires measured distortion model and coefficients")
        ref_lens = intrinsic_raw.get("reference_lens_position")
        tolerance = intrinsic_raw.get("lens_position_tolerance")
        intrinsic = CameraIntrinsicCalibration(
            source=source,
            reference_width_px=size[0],
            reference_height_px=size[1],
            fx_px=rows[0][0],
            fy_px=rows[1][1],
            cx_px=rows[0][2],
            cy_px=rows[1][2],
            distortion_model=distortion_model,
            distortion_coefficients=coefficients,
            reference_lens_position=None if ref_lens is None else _finite(ref_lens, "reference_lens_position"),
            lens_position_tolerance=None if tolerance is None else _positive(tolerance, "lens_position_tolerance"),
        )
    else:
        raise ValueError(f"unsupported intrinsic source: {source}")
    return CameraGeometryConfig(factory=factory, mount=mount, intrinsic=intrinsic)


def camera_geometry_status_from_properties(
    config: CameraGeometryConfig,
    properties: Mapping[str, object],
) -> CameraGeometryStatus:
    if not isinstance(config, CameraGeometryConfig):
        raise TypeError("config must be CameraGeometryConfig")
    if not isinstance(properties, Mapping):
        raise TypeError("properties must be a mapping")
    factory = config.factory
    model_raw = properties.get("Model")
    model = str(model_raw) if model_raw is not None else None
    pixel_array = _pair(properties.get("PixelArraySize"))
    unit_cell = _pair(properties.get("UnitCellSize"))
    crop = sensor_crop_from_value(properties.get("ScalerCropMaximum"))
    if crop is None:
        crop = SensorCrop(0, 0, factory.native_width_px, factory.native_height_px)
    rotation_raw = properties.get("Rotation")
    rotation = None if rotation_raw is None else str(rotation_raw)

    invalid_reasons: list[str] = []
    degraded_reasons: list[str] = []
    if model is None:
        degraded_reasons.append("MODEL_MISSING")
    elif factory.sensor_model.casefold() not in model.casefold():
        invalid_reasons.append("MODEL_MISMATCH")
    expected_array = (factory.native_width_px, factory.native_height_px)
    if pixel_array is None:
        degraded_reasons.append("PIXEL_ARRAY_MISSING")
    elif pixel_array != expected_array:
        invalid_reasons.append("PIXEL_ARRAY_MISMATCH")
    expected_cell = (round(factory.pixel_pitch_x_um * 1000.0), round(factory.pixel_pitch_y_um * 1000.0))
    if unit_cell is None:
        degraded_reasons.append("UNIT_CELL_MISSING")
    elif unit_cell != expected_cell:
        invalid_reasons.append("UNIT_CELL_MISMATCH")
    if properties.get("ScalerCropMaximum") is None:
        degraded_reasons.append("SCALER_CROP_MAXIMUM_MISSING")
    if crop.x + crop.width > factory.native_width_px or crop.y + crop.height > factory.native_height_px:
        invalid_reasons.append("SCALER_CROP_OUTSIDE_SENSOR")
    if rotation is None:
        degraded_reasons.append("ROTATION_MISSING")

    if invalid_reasons:
        state = "INVALID"
        reason = ",".join(invalid_reasons + degraded_reasons)
    elif degraded_reasons:
        state = "DEGRADED"
        reason = ",".join(degraded_reasons)
    else:
        state = "VALID"
        reason = ""
    return CameraGeometryStatus(
        state=state,
        reason=reason,
        camera_model=model,
        pixel_array_size=pixel_array,
        unit_cell_size_nm=unit_cell,
        rotation=rotation,
        sensor_crop=crop,
    )


def effective_geometry(
    config: CameraGeometryConfig,
    *,
    output_width_px: int,
    output_height_px: int,
    sensor_crop: SensorCrop | None = None,
    runtime_status: CameraGeometryStatus | None = None,
) -> CameraEffectiveGeometry:
    width = _positive_int(output_width_px, "output_width_px")
    height = _positive_int(output_height_px, "output_height_px")
    factory = config.factory
    crop = sensor_crop or (runtime_status.sensor_crop if runtime_status is not None else None)
    if crop is None:
        crop = SensorCrop(0, 0, factory.native_width_px, factory.native_height_px)
    if crop.x + crop.width > factory.native_width_px or crop.y + crop.height > factory.native_height_px:
        raise ValueError("sensor crop falls outside native factory sensor geometry")
    intrinsic = config.intrinsic
    sx = width / crop.width
    sy = height / crop.height
    fx = intrinsic.fx_px * sx
    fy = intrinsic.fy_px * sy
    cx = (intrinsic.cx_px - crop.x) * sx
    cy = (intrinsic.cy_px - crop.y) * sy
    quality = "empirical" if intrinsic.source == "empirical" else "factory_nominal"
    if runtime_status is not None and runtime_status.state != "VALID":
        quality = "degraded"
    return CameraEffectiveGeometry(
        output_width_px=width,
        output_height_px=height,
        sensor_crop=crop,
        fx_px=fx,
        fy_px=fy,
        cx_px=cx,
        cy_px=cy,
        base_from_camera=config.mount.base_from_camera,
        geometry_quality=quality,
        distortion_model=intrinsic.distortion_model,
        distortion_coefficients=intrinsic.distortion_coefficients,
    )


def _cross(left: Sequence[float], right: Sequence[float]) -> tuple[float, float, float]:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _normalize(vector: Sequence[float]) -> tuple[float, float, float]:
    if len(vector) != 3:
        raise ValueError("vector must have three components")
    norm = math.sqrt(sum(float(item) * float(item) for item in vector))
    if norm <= 0.0 or not math.isfinite(norm):
        raise ValueError("vector must have finite non-zero norm")
    return tuple(float(item) / norm for item in vector)  # type: ignore[return-value]


__all__ = [
    "CameraEffectiveGeometry",
    "CameraFactoryGeometry",
    "CameraGeometryConfig",
    "CameraGeometryStatus",
    "CameraIntrinsicCalibration",
    "CameraMountPose",
    "IMX708_WIDE_NOIR",
    "SensorCrop",
    "camera_geometry_config_from_mapping",
    "camera_geometry_status_from_properties",
    "effective_geometry",
    "factory_nominal_intrinsic",
    "factory_profile",
    "sensor_crop_from_value",
]
