#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import asyncio
import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from fastgui import backend_api  # noqa: E402


class TestFastGuiBackendApi(unittest.TestCase):
    def test_gui_has_only_motion_console_and_no_ad_hoc_encoder_motion_route(self):
        route_paths = {str(getattr(route, "path", "")) for route in backend_api.router.routes}
        self.assertNotIn("/api/raw-encoder-pulse-test", route_paths)

        index_text = (PROJECT_ROOT / "fastgui" / "templates" / "index.html").read_text(
            encoding="utf-8"
        )
        self.assertIn('data-page="motion-console"', index_text)
        self.assertNotIn("tank-simulator", index_text.lower())
        self.assertTrue((PROJECT_ROOT / "fastgui" / "static" / "css" / "motion_console.css").is_file())
        self.assertTrue((PROJECT_ROOT / "fastgui" / "static" / "js" / "pages" / "motion-console.js").is_file())
        self.assertFalse((PROJECT_ROOT / "fastgui" / "static" / "css" / "tank_new_gui.css").exists())
        self.assertFalse((PROJECT_ROOT / "fastgui" / "static" / "js" / "pages" / "tank-simulator.js").exists())

    def test_control_mode_reporting_does_not_mask_invalid_runtime_value(self):
        self.assertEqual(backend_api._normalize_control_mode(" unified "), "UNIFIED")
        self.assertEqual(backend_api._normalize_control_mode("BASIC"), "BASIC")
        self.assertEqual(backend_api._control_mode_to_profile("BASIC"), "")

    def test_tools_live_catalog_has_default_and_human_follow_profiles(self):
        profiles = backend_api._collect_tools_live_motion_profiles()
        by_name = {str(item["name"]): item for item in profiles}

        self.assertGreater(len(profiles), 0)
        self.assertEqual(profiles[0]["name"], backend_api.TOOLS_LIVE_DEFAULT_PROFILE)
        self.assertEqual(profiles[0]["script"], "tools/follow_moving_target_sim.py")
        self.assertEqual(profiles[0]["family"], "amr_navigation")
        self.assertEqual(by_name[backend_api.HUMAN_FOLLOW_TOOLS_LIVE_PROFILE]["script"], "tools/person_target_direction_live.py")
        self.assertEqual(by_name[backend_api.HUMAN_FOLLOW_TOOLS_LIVE_PROFILE]["family"], "amr_navigation")
        quality = by_name[backend_api.HUMAN_FOLLOW_QUALITY_TOOLS_LIVE_PROFILE]
        self.assertEqual(quality["script"], "tools/M3_emberkovetes_mozgasminoseg.py")
        self.assertEqual(quality["family"], "movement_quality")
        self.assertTrue(quality["requires_measurement_truth"])
        room_quality = by_name[backend_api.ROOM_CRUISE_QUALITY_TOOLS_LIVE_PROFILE]
        self.assertEqual(room_quality["script"], "tools/M4_1_room_cruise_quality_validator.py")
        self.assertEqual(room_quality["family"], "movement_quality")
        self.assertTrue(room_quality["requires_measurement_truth"])
        m3_unified = by_name[backend_api.M3_UNIFIED_TOOLS_LIVE_PROFILE]
        self.assertEqual(m3_unified["script"], "tools/M3_room_cruise_unified_validator.py")
        self.assertEqual(m3_unified["family"], "movement_quality")
        self.assertTrue(m3_unified["requires_measurement_truth"])

        catalog = asyncio.run(backend_api.api_test_hub_tools_live_catalog())
        self.assertEqual(catalog["m3_unified_profile"], backend_api.M3_UNIFIED_TOOLS_LIVE_PROFILE)

    def test_tools_live_run_args_use_existing_gui_runtime(self):
        args = backend_api._build_tools_live_hub_run_args(
            {"stop_runtime_after": True},
            "person_follow_camera_live",
        )

        self.assertIn("--no-auto-runtime", args)
        self.assertNotIn("--stop-runtime-after", args)
        self.assertLess(args.index("--no-auto-runtime"), args.index("person_follow_camera_live"))

    def test_m3_gui_run_only_emits_supported_runtime_options(self):
        args = backend_api._build_tools_live_hub_run_args(
            {"unsupported_option": True},
            backend_api.HUMAN_FOLLOW_QUALITY_TOOLS_LIVE_PROFILE,
        )

        self.assertIn("--no-auto-runtime", args)

        room_args = backend_api._build_tools_live_hub_run_args(
            {"unsupported_option": True},
            backend_api.ROOM_CRUISE_QUALITY_TOOLS_LIVE_PROFILE,
        )
        self.assertIn("--no-auto-runtime", room_args)
        self.assertEqual(room_args[-1], "M4_1_room_cruise_quality_validator")

        m3_unified_args = backend_api._build_tools_live_hub_run_args(
            {"stop_runtime_after": True},
            backend_api.M3_UNIFIED_TOOLS_LIVE_PROFILE,
        )
        self.assertEqual(
            m3_unified_args,
            ["run", "--no-auto-runtime", "M3_room_cruise_unified_validator"],
        )

    def test_regular_hub_run_args_keep_runtime_flags_explicit(self):
        default_args = backend_api._build_hub_run_args({}, "person_follow_camera_live")
        explicit_args = backend_api._build_hub_run_args(
            {"no_auto_runtime": True, "stop_runtime_after": True},
            "person_follow_camera_live",
        )

        self.assertNotIn("--no-auto-runtime", default_args)
        self.assertEqual(default_args[-1], "person_follow_camera_live")
        self.assertIn("--no-auto-runtime", explicit_args)
        self.assertIn("--stop-runtime-after", explicit_args)
        self.assertLess(explicit_args.index("--no-auto-runtime"), explicit_args.index("person_follow_camera_live"))
        self.assertLess(explicit_args.index("--stop-runtime-after"), explicit_args.index("person_follow_camera_live"))

    def test_gui_exposes_canonical_m0_m1_and_m3_unified_through_shared_hub_routes(self):
        index_text = (PROJECT_ROOT / "fastgui" / "templates" / "index.html").read_text(
            encoding="utf-8"
        )
        page_script = (
            PROJECT_ROOT / "fastgui" / "static" / "js" / "pages" / "log-audit-page.js"
        ).read_text(encoding="utf-8")
        motion_page = index_text.split('<section class="r2-page" id="page-motion">', 1)[1].split(
            '<!-- PAGE: SAFETY -->', 1
        )[0]

        self.assertIn('id="hub-btn-run-m0"', index_text)
        self.assertIn('id="hub-btn-run-m1"', index_text)
        self.assertIn('id="motion-btn-run-m0-live"', motion_page)
        self.assertIn('id="motion-btn-run-m3-unified-live"', motion_page)
        self.assertIn("M0_measurement_trust_live", page_script)
        self.assertIn("M1_motion_baseline_live", page_script)
        self.assertIn("M3_room_cruise_unified_validator", page_script)
        self.assertIn("runCanonicalMotionLevel(M0_MEASUREMENT_PROFILE)", page_script)
        self.assertIn("runCanonicalMotionLevel(M1_MOTION_PROFILE)", page_script)
        self.assertIn(
            "el('motion-btn-run-m0-live')?.addEventListener('click', () => "
            "runCanonicalMotionLevel(M0_MEASUREMENT_PROFILE));",
            page_script,
        )
        self.assertIn(
            "el('motion-btn-run-m3-unified-live')?.addEventListener('click', runM3UnifiedToolLive);",
            page_script,
        )
        self.assertNotIn("/api/test/m0", page_script)
        self.assertNotIn("/api/test/m1", page_script)
        self.assertNotIn("/api/test/m3", page_script)

        m0_args = backend_api._build_hub_run_args({}, "M0_measurement_trust_live")
        m1_args = backend_api._build_hub_run_args({}, "M1_motion_baseline_live")
        self.assertEqual(m0_args, ["run", "M0_measurement_trust_live"])
        self.assertEqual(m1_args, ["run", "M1_motion_baseline_live"])

    def test_gui_test_hub_rejects_concurrent_job(self):
        acquired = backend_api._test_hub_lock.acquire(blocking=False)
        self.assertTrue(acquired)
        try:
            result = backend_api._start_test_hub_job(
                "tools-live-run",
                ["run", "--no-auto-runtime", backend_api.ROOM_CRUISE_QUALITY_TOOLS_LIVE_PROFILE],
                timeout_s=30.0,
            )
        finally:
            backend_api._test_hub_lock.release()

        self.assertFalse(result["ok"])
        self.assertFalse(result["accepted"])
        self.assertEqual(result["status"], "running")

    def test_camera_target_box_scales_detection_to_stream_frame(self):
        box = backend_api._camera_target_box(
            {
                "target_visible": True,
                "target_usable": True,
                "stale": False,
                "detector": "opencv_motion_blob",
                "image_width_px": 640,
                "image_height_px": 480,
                "bbox_x_px": 160,
                "bbox_y_px": 80,
                "bbox_width_px": 200,
                "bbox_height_px": 300,
            },
            320,
            240,
        )

        self.assertIsNotNone(box)
        self.assertAlmostEqual(box["x1"], 80.0)
        self.assertAlmostEqual(box["y1"], 40.0)
        self.assertAlmostEqual(box["x2"], 180.0)
        self.assertAlmostEqual(box["y2"], 190.0)

    def test_camera_target_box_ignores_stale_or_unknown_target(self):
        box = backend_api._camera_target_box(
            {
                "target_visible": True,
                "target_usable": True,
                "stale": True,
                "detector": "opencv_motion_blob",
                "bbox_width_ratio": 0.3,
                "bbox_height_ratio": 0.5,
                "target_center_x_ratio": 0.5,
                "target_center_y_ratio": 0.5,
            },
            320,
            240,
        )

        self.assertIsNone(box)


if __name__ == "__main__":
    unittest.main()
