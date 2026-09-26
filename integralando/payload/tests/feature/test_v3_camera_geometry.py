from __future__ import annotations

import math

from v3.adapters.camera_geometry import (
    IMX708_WIDE_NOIR,
    SensorCrop,
    camera_geometry_config_from_mapping,
    camera_geometry_status_from_properties,
    effective_geometry,
)


def _config():
    return camera_geometry_config_from_mapping(
        {
            "factory_profile": "imx708_wide_noir",
            "mount": {
                "x_m": 0.05,
                "y_m": 0.0,
                "z_m": 0.19,
                "roll_deg": 0.0,
                "pitch_deg": 15.0,
                "yaw_deg": 0.0,
            },
            "intrinsic": {
                "source": "factory_nominal",
                "distortion_model": "unknown",
                "distortion_coefficients": None,
            },
        }
    )


def _properties():
    return {
        "Model": "imx708",
        "PixelArraySize": (4608, 2592),
        "UnitCellSize": (1400, 1400),
        "Rotation": 0,
        "ScalerCropMaximum": (0, 0, 4608, 2592),
    }


def test_factory_profile_matches_camera_module_3_wide_noir_nominal_data():
    factory = IMX708_WIDE_NOIR
    assert (factory.native_width_px, factory.native_height_px) == (4608, 2592)
    assert factory.pixel_pitch_x_um == factory.pixel_pitch_y_um == 1.4
    assert factory.focal_length_mm == 2.75
    assert factory.horizontal_fov_deg == 102.0
    assert factory.vertical_fov_deg == 67.0
    assert factory.diagonal_fov_deg == 120.0
    assert factory.tv_distortion_bound == 0.116
    assert factory.ir_cut_filter is False
    assert math.isclose(factory.nominal_fx_px, 1964.2857142857144, rel_tol=0.0, abs_tol=1e-9)


def test_nominal_full_crop_intrinsics_scale_to_lores_and_main():
    config = _config()
    lores = effective_geometry(config, output_width_px=640, output_height_px=360)
    main = effective_geometry(config, output_width_px=1280, output_height_px=720)
    assert math.isclose(lores.fx_px, 272.8174603174603, abs_tol=1e-9)
    assert math.isclose(lores.fy_px, 272.8174603174603, abs_tol=1e-9)
    assert math.isclose(lores.cx_px, 320.0, abs_tol=1e-12)
    assert math.isclose(lores.cy_px, 180.0, abs_tol=1e-12)
    assert math.isclose(main.fx_px, 545.6349206349206, abs_tol=1e-9)
    assert math.isclose(main.cx_px, 640.0, abs_tol=1e-12)
    assert math.isclose(main.cy_px, 360.0, abs_tol=1e-12)


def test_crop_transform_preserves_native_intrinsic_lineage():
    config = _config()
    geometry = effective_geometry(
        config,
        output_width_px=640,
        output_height_px=360,
        sensor_crop=SensorCrop(320, 180, 3968, 2232),
    )
    expected_scale = 640 / 3968
    assert math.isclose(geometry.fx_px, config.intrinsic.fx_px * expected_scale, abs_tol=1e-9)
    assert math.isclose(geometry.cx_px, (2304 - 320) * expected_scale, abs_tol=1e-9)


def test_mount_transform_matches_nominal_robot_frame_and_center_ray():
    config = _config()
    geometry = effective_geometry(config, output_width_px=640, output_height_px=360)
    transform = geometry.base_from_camera
    assert math.isclose(transform[0][2], 0.9659258262890683, abs_tol=1e-12)
    assert math.isclose(transform[1][2], 0.0, abs_tol=1e-12)
    assert math.isclose(transform[2][2], 0.25881904510252074, abs_tol=1e-12)
    assert geometry.camera_origin_base_m == (0.05, 0.0, 0.19)
    center = geometry.pixel_to_base_ray(320.0, 180.0)
    assert all(math.isclose(center[i], transform[i][2], abs_tol=1e-12) for i in range(3))
    image_right = geometry.pixel_to_base_ray(400.0, 180.0)
    assert image_right[1] < 0.0


def test_factory_nominal_distortion_is_explicitly_unknown_and_not_rectifiable():
    config = _config()
    assert config.intrinsic.distortion_model == "unknown"
    assert config.intrinsic.distortion_coefficients is None
    assert config.intrinsic.can_rectify is False


def test_runtime_properties_validate_without_crashing_on_mismatch():
    config = _config()
    valid = camera_geometry_status_from_properties(config, _properties())
    assert valid.state == "VALID"
    mismatch = camera_geometry_status_from_properties(
        config,
        {
            **_properties(),
            "PixelArraySize": (2304, 1296),
        },
    )
    assert mismatch.state == "INVALID"
    assert "PIXEL_ARRAY_MISMATCH" in mismatch.reason


def test_missing_runtime_properties_degrades_to_full_sensor_crop():
    status = camera_geometry_status_from_properties(_config(), {"Model": "imx708"})
    assert status.state == "DEGRADED"
    assert status.sensor_crop == SensorCrop(0, 0, 4608, 2592)


def test_bottom_center_nominal_ray_intersects_ground_in_front_of_robot():
    geometry = effective_geometry(_config(), output_width_px=640, output_height_px=360)
    hit = geometry.ground_intersection(320.0, 359.0)
    assert hit is not None
    assert 0.55 < hit[0] < 0.75
    assert abs(hit[1]) < 1e-12
    assert abs(hit[2]) < 1e-12
