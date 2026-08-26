#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from collections import deque
from .task_model import RobotTask, TaskPriority

class TaskQueue:
    def __init__(self):
        self._queue = deque()

    def add(self, task: RobotTask):
        """Standard add to end of queue."""
        self._queue.append(task)

    def inject(self, task: RobotTask):
        """Add to front (High Priority)."""
        self._queue.appendleft(task)

    def clear(self):
        self._queue.clear()

    def pop(self) -> RobotTask:
        if not self.is_empty():
            return self._queue.popleft()
        return None

    def peek(self) -> RobotTask:
        if not self.is_empty():
            return self._queue[0]
        return None

    def is_empty(self) -> bool:
        return len(self._queue) == 0

    def __len__(self):
        return len(self._queue)
