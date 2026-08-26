#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from .task_model import RobotTask, TaskType
import sys
import os

# Try to import RobotState from parent directory structure
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from state import RobotState

class SecurityGuard:
    def __init__(self, controller):
        self.controller = controller
        self.danger_dist = 0.35  # Meters

    def check_emergency(self) -> bool:
        """
        Called every tick. Returns True if immediate stop is required.
        """
        # Safety SSOT: a globális SafetySupervisor döntését tiszteljük.
        # Így elkerüljük a duplikált (és eltérő) emergency logikát.
        safety = getattr(self.controller, "safety", None)
        if safety is not None:
            st = safety.status()
            return not bool(st.get("allow", True))
        return False

    def can_execute(self, task: RobotTask) -> tuple[bool, str]:
        """
        Pre-execution check.
        Returns: (Allowed, Reason)
        """
        state = self.controller.sm.current_enum
        lidar = self.controller.lidar_summary

        # 1. State Machine Authority
        if state == RobotState.CALIBRATING:
            return False, "Rendszer kalibráció alatt."
        
        if state == RobotState.FAILSAFE and task.type != TaskType.STOP:
            return False, "Vészhelyzeti mód aktív."

        # 2. Physical Constraints
        if task.type == TaskType.MOVE:
            direction = task.params.get('direction', 1)
            dist = task.params.get('distance', 0)
            
            if dist < 0:
                return False, "Negatív távolság nem értelmezhető."
            
            if direction > 0 and lidar.get('blocked_front', False):
                return False, "Akadály elöl."
            
            if direction < 0 and lidar.get('blocked_back', False):
                return False, "Akadály hátul."
        
        # APPROACH feladat esetén ellenőrizzük, hogy van-e LIDAR adat
        if task.type == TaskType.APPROACH:
            if lidar.get('min_dist', 10.0) > 4.0:
                return False, "Nincs észlelhető célpont (LIDAR > 4m)."

        return True, "OK"