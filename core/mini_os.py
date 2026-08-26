#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Dict, List


@dataclass
class AppContract:
    app_id: str
    title: str
    permissions: List[str]
    command_prefixes: List[str]
    interfaces: List[str]
    enabled: bool = True


class MiniOSRuntime:
    """
    App-szerű modulregiszter jogokkal és interfész-szerződéssel.
    """

    def __init__(self, apps: Dict[str, AppContract]):
        self.apps = apps

    @classmethod
    def default(cls) -> "MiniOSRuntime":
        apps = {
            "navigation": AppContract(
                app_id="navigation",
                title="Navigation",
                permissions=["motion.write", "pose.read"],
                command_prefixes=[
                    "set_vector",
                    "set_speed",
                    "step_speed",
                    "turn",
                    "set_twist",
                    "set_motion_target",
                    "set_track_velocity",
                    "go_to_pose",
                    "set_follow_target",
                    "set_follow_speed_scale",
                    "set_follow_distance",
                    "follow_waypoints",
                    "rotate_to_heading",
                    "square",
                    "circle",
                    "patrol",
                ],
                interfaces=["/api/command", "/api/runtime/motion_state"],
            ),
            "safety": AppContract(
                app_id="safety",
                title="Safety",
                permissions=["safety.read", "safety.write"],
                command_prefixes=["stop", "cancel_motion", "strong_reset", "full_reset", "calibrate"],
                interfaces=["/api/status", "/api/command", "/api/log/latest", "/api/log/stats"],
            ),
            "sensors": AppContract(
                app_id="sensors",
                title="Sensors",
                permissions=["sensor.read"],
                command_prefixes=["toggle_camera", "capture_photo", "toggle_video_recording"],
                interfaces=["/api/lidar-scan", "/api/camera-status", "/api/camera-stream"],
            ),
            "ai": AppContract(
                app_id="ai",
                title="AI",
                permissions=["ai.control"],
                command_prefixes=["toggle_follow", "search_person", "toggle_listen"],
                interfaces=["/api/command", "/api/ai-flow"],
            ),
        }
        return cls(apps)

    def classify_command(self, cmd_type: str) -> str:
        cmd = str(cmd_type or "")
        for app_id, app in self.apps.items():
            if any(cmd.startswith(prefix) for prefix in app.command_prefixes):
                return app_id
        return "navigation"

    def list_apps(self) -> List[Dict]:
        out = []
        for app in self.apps.values():
            out.append(asdict(app))
        return out
