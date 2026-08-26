#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json

from .task_queue import TaskQueue
from .executor import TaskExecutor
from .security_guard import SecurityGuard
from .task_model import RobotTask, TaskType, TaskPriority
from .llm_json import validate_llm_json, task_factory

class AlbaCore:
    def __init__(self, controller):
        self.controller = controller
        
        # Initialize Components
        self.queue = TaskQueue()
        self.executor = TaskExecutor(controller)
        self.guard = SecurityGuard(controller)
        
        self.controller.logger.info("ALBA CORE (Deterministic) Initialized.")

    def process_input(self, text: str):
        """Entry point for STT/Text commands. STRICTLY JSON-ONLY for LLM; invalid → NO ACTION + audit."""
        if hasattr(self.controller, "mark_input"):
            self.controller.mark_input("AI")

        text = (text or "").strip()
        if not text:
            self._audit_reject("empty_input", text)
            return

        # 1. Try parse as JSON
        try:
            data = json.loads(text)
        except json.JSONDecodeError as e:
            self._audit_reject("invalid_json", text, str(e))
            return

        if not isinstance(data, dict):
            self._audit_reject("root_not_object", text)
            return

        # 2. Validate fixed schema
        ok, reason = validate_llm_json(data)
        if not ok:
            self._audit_reject("schema_mismatch", text, reason)
            return

        # 3. TaskFactory: valid JSON → TaskQueue entries
        tasks = task_factory(data)
        if not tasks:
            # QUERY or no-op; still accepted
            if hasattr(self.controller, "telemetry"):
                self.controller.telemetry.emit_audit(
                    "TASK_QUEUE", "CORE",
                    details={"intent": data.get("intent"), "count": 0, "queue_size": len(self.queue)}
                )
            return

        for task in tasks:
            if task.priority == TaskPriority.CRITICAL:
                self.queue.clear()
                self.queue.inject(task)
                self.executor.is_running = False
            else:
                self.queue.add(task)

        if hasattr(self.controller, "telemetry"):
            self.controller.telemetry.emit_audit(
                "TASK_QUEUE",
                "CORE",
                details={"count": len(tasks), "queue_size": len(self.queue)}
            )

        self.controller.logger.info(f"[CORE] Added {len(tasks)} tasks. Queue size: {len(self.queue)}")
        if hasattr(self.controller.logger, "log_full_extra"):
            self.controller.logger.log_full_extra("LLM_IN", intent=data.get("intent"), count=len(tasks), queue=len(self.queue))

    def _audit_reject(self, reason: str, raw: str, detail: str = ""):
        """Invalid JSON or schema mismatch: NO ACTION, audit log entry.
        Example audit.jsonl line:
        {"ts": 1234567890.1, "type": "audit", "event": "LLM_REJECT", "source": "AI", "severity": "WARN",
         "details": {"reason": "schema_mismatch", "detail": "intent must be one of ...", "raw_preview": "..."},
         "prev_hash": "...", "hash": "..."}
        """
        if hasattr(self.controller, "telemetry"):
            self.controller.telemetry.emit_audit(
                "LLM_REJECT",
                "AI",
                severity="WARN",
                details={"reason": reason, "detail": detail, "raw_preview": (raw[:500] if raw else "")},
            )
        self.controller.logger.warn(f"[CORE] LLM output rejected: {reason} {detail}")
        if hasattr(self.controller.logger, "log_full_extra"):
            self.controller.logger.log_full_extra("LLM_REJECT", reason=reason, detail=detail or "")

    def tick(self):
        """Called at 50Hz from cont.py"""
        
        # 1. Emergency Check (Always runs)
        if self.guard.check_emergency():
            if self.controller.sm.get_current_state_name() not in ('IDLE', 'FAILSAFE'):
                self.controller.logger.error("[CORE] SECURITY GUARD TRIGGERED STOP")
                self.controller._emergency_stop()
                self.queue.clear()
                self.executor.is_running = False
                return

        # 2. Update current execution
        if self.executor.is_running:
            self.executor.tick()
            return

        # 3. Pick next task if idle
        if not self.queue.is_empty() and not self.executor.is_running:
            next_task = self.queue.peek()
            
            # Security Check
            allowed, reason = self.guard.can_execute(next_task)
            
            if allowed:
                # Pop and Execute
                task = self.queue.pop()
                self.controller.logger.info(f"[CORE] Executing: {task}")
                if hasattr(self.controller.logger, "log_full_extra"):
                    self.controller.logger.log_full_extra("LLM_EXEC", task=str(task))
                self.executor.start_task(task)
            else:
                # Reject task
                self.controller.logger.warn(f"[CORE] Task Rejected: {reason}")
                if hasattr(self.controller.logger, "log_full_extra"):
                    self.controller.logger.log_full_extra("LLM_REJECT", reason=reason, task=str(next_task))
                self.queue.pop() # Remove rejected task
                if hasattr(self.controller, 'brain') and self.controller.brain.tts:
                    self.controller.brain.tts.say(f"Nem hajtható végre. {reason}")
