#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from .task_model import RobotTask, TaskType
import math
import time
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from state import RobotState

class TaskExecutor:
    def __init__(self, controller):
        self.ctrl = controller
        self.current_task: RobotTask | None = None
        
        # Execution tracking
        self.start_dist_l = 0
        self.start_dist_r = 0
        self.target_val = 0
        self.is_running = False
        
        # Time tracking for duration-based tasks (e.g. PATROL)
        self.task_start_time = 0.0
        self.task_duration = 0.0
        self.docking_active = False
        self.dock_dist = 0.0
        self.dock_speed = 1
        self.task_source = "AI"

    def _encoder_distances(self):
        """Read task distance from the single EncoderService snapshot path."""
        service = getattr(self.ctrl, "encoder_service", None)
        snapshot = service.get_snapshot() if service is not None else None
        if snapshot is None or str(getattr(snapshot, "health", "OK")) != "OK":
            return None
        try:
            left = float(snapshot.left_distance)
            right = float(snapshot.right_distance)
        except (AttributeError, TypeError, ValueError):
            return None
        if not (math.isfinite(left) and math.isfinite(right)):
            return None
        return left, right

    def start_task(self, task: RobotTask):
        self.current_task = task
        self.is_running = True
        self.task_start_time = time.time()
        self.task_source = task.params.get("source", "AI")
        
        if task.type == TaskType.MOVE:
            distances = self._encoder_distances()
            if distances is None:
                if hasattr(self.ctrl, "logger") and self.ctrl.logger:
                    self.ctrl.logger.warn("[EXECUTOR] MOVE blokkolva: nincs friss encoder snapshot.")
                self.is_running = False
                self.current_task = None
                return
            self.start_dist_l, self.start_dist_r = distances
            self._setup_move(task)
        elif task.type == TaskType.TURN:
            self._setup_turn(task)
        elif task.type == TaskType.STOP:
            self._setup_stop()
        elif task.type == TaskType.WAIT:
            self._setup_wait(task)
        elif task.type == TaskType.PATROL:
            self.task_duration = task.params.get('duration', 0.0)
            try:
                from controller.commands import set_motion_source
                set_motion_source(self.ctrl, "STATE")
            except Exception:
                pass
            self.ctrl.sm.transition_to(RobotState.PATROL)
            if self.task_duration > 0:
                self.is_running = True
            else:
                self.is_running = False 
                self.current_task = None
        elif task.type == TaskType.APPROACH:
            self.ctrl.sm.transition_to(RobotState.APPROACH)
        elif task.type == TaskType.SAY:
            if hasattr(self.ctrl, 'brain') and self.ctrl.brain.tts:
                self.ctrl.brain.tts.say(task.params.get('text', ''))
            self.is_running = False
            self.current_task = None

    def _setup_move(self, task):
        dist = task.params.get('distance', 0.0)
        direction = task.params.get('direction', 1)
        speed_lvl = task.params.get('speed_level', 5)
        speed_lvl = max(0, min(9, int(speed_lvl)))
        source = self.task_source
        self.docking_active = bool(task.params.get("dock", False))
        self.dock_dist = float(task.params.get("dock_dist", 0.15))
        self.dock_speed = int(task.params.get("dock_speed", 1))
        self.target_val = dist
        if hasattr(self.ctrl, "dock_active"):
            self.ctrl.dock_active = False
            self.ctrl.dock_speed_level = self.dock_speed
            self.ctrl.dock_dir = direction
        if hasattr(self.ctrl, "set_speed_level"):
            ok = self.ctrl.set_speed_level(speed_lvl, source=source, apply_state=False)
            if not ok:
                self.ctrl.logger.warn(f"[ARBITER] {source} MOVE blokkolva.")
                self.is_running = False
                self.current_task = None
                return
        else:
            self.ctrl.speed_level = speed_lvl
        if direction > 0: self.ctrl.sm.transition_to(RobotState.FORWARD)
        else: self.ctrl.sm.transition_to(RobotState.BACKWARD)

    def _setup_turn(self, task):
        angle = task.params.get('angle', 0.0)
        source = self.task_source
        if hasattr(self.ctrl, "set_motion_source") and not self.ctrl.set_motion_source(source):
            self.ctrl.logger.warn(f"[ARBITER] {source} TURN blokkolva.")
            self.is_running = False
            self.current_task = None
            return
        if task.params.get("dock_before"):
            dock_speed = int(task.params.get("dock_speed", 1))
            if hasattr(self.ctrl, "set_speed_level"):
                self.ctrl.set_speed_level(dock_speed, source=source, apply_state=False)
        self.ctrl.sm.transition_to(RobotState.ROTATE, delta=angle)
        
    def _setup_stop(self):
        try:
            from controller.commands import soft_stop

            soft_stop(self.ctrl, reason="TASK_EXECUTOR_STOP", source="CORE")
        except Exception:
            self.ctrl._emergency_stop("TASK_EXECUTOR_STOP_FALLBACK")
        self.is_running = False
        self.current_task = None

    def _setup_wait(self, task):
        duration = float(task.params.get('duration', 0.0) or 0.0)
        self.task_duration = max(0.0, duration)
        # Biztos leállás a várakozás idejére
        try:
            self.ctrl.v_target = 0.0
            self.ctrl.omega_target = 0.0
            self.ctrl.turn_level = 0
        except Exception:
            pass
        try:
            self.ctrl.sm.transition_to(RobotState.IDLE)
        except Exception:
            pass
        if self.task_duration <= 0.0:
            self.is_running = False
            self.current_task = None

    def tick(self):
        if not self.is_running or not self.current_task: return
        
        if self.current_task.type == TaskType.MOVE: 
            self._monitor_move()
        elif self.current_task.type == TaskType.TURN: 
            self._monitor_turn()
        elif self.current_task.type == TaskType.WAIT:
            self._monitor_wait()
        elif self.current_task.type == TaskType.PATROL:
            self._monitor_patrol()
        elif self.current_task.type == TaskType.APPROACH:
            self._monitor_approach()

    def _monitor_move(self):
        distances = self._encoder_distances()
        if distances is None:
            return
        left_distance, right_distance = distances
        dl = abs(left_distance - self.start_dist_l)
        dr = abs(right_distance - self.start_dist_r)
        d_avg = (dl + dr) / 2.0
        remaining = self.target_val - d_avg

        if self.docking_active and remaining <= self.dock_dist:
            if hasattr(self.ctrl, "dock_active") and not self.ctrl.dock_active:
                self.ctrl.dock_active = True
                self.ctrl.dock_speed_level = self.dock_speed
                self.ctrl.dock_dir = 1 if (self.current_task.params.get("direction", 1) >= 0) else -1
                self.ctrl.sm.transition_to(RobotState.DOCK)
            if hasattr(self.ctrl, "set_speed_level"):
                self.ctrl.set_speed_level(self.dock_speed, source=self.task_source, apply_state=False)
            else:
                self.ctrl.speed_level = self.dock_speed

        if d_avg >= self.target_val:
            self.ctrl.sm.transition_to(RobotState.IDLE)
            if hasattr(self.ctrl, "dock_active"):
                self.ctrl.dock_active = False
            self.is_running = False
            self.current_task = None

    def _monitor_turn(self):
        if self.ctrl.sm.current_enum == RobotState.IDLE:
            self.is_running = False
            self.current_task = None

    def _monitor_patrol(self):
        if self.task_duration > 0:
            if (time.time() - self.task_start_time) > self.task_duration:
                self.ctrl.logger.info(f"[EXECUTOR] Patrol duration ({self.task_duration}s) expired.")
                self.ctrl.sm.transition_to(RobotState.IDLE)
                self.is_running = False
                self.current_task = None

    def _monitor_wait(self):
        if self.task_duration <= 0:
            self.is_running = False
            self.current_task = None
            return
        if (time.time() - self.task_start_time) >= self.task_duration:
            self.is_running = False
            self.current_task = None

    def _monitor_approach(self):
        # Az ApproachState automatikusan visszavált IDLE-be, ha eléri a célt vagy timeoutol
        if self.ctrl.sm.current_enum == RobotState.IDLE:
            self.ctrl.logger.success("[EXECUTOR] Approach task completed.")
            if hasattr(self.ctrl, 'brain') and self.ctrl.brain.tts:
                self.ctrl.brain.tts.say("Megérkeztem.")
            self.is_running = False
            self.current_task = None
