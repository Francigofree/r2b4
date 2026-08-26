#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from enum import Enum, auto
from dataclasses import dataclass, field
import uuid
import time

class TaskType(Enum):
    MOVE = auto()       # Linear movement
    TURN = auto()       # Rotate in place
    STOP = auto()       # Emergency/Standard stop
    WAIT = auto()       # Pause execution
    PATROL = auto()     # Enter patrol state
    SAY = auto()        # TTS feedback
    APPROACH = auto()   # Precision approach to target

class TaskPriority(Enum):
    NORMAL = 10
    HIGH = 50
    CRITICAL = 100      # Clears queue immediately

@dataclass
class RobotTask:
    type: TaskType
    params: dict = field(default_factory=dict)
    priority: TaskPriority = TaskPriority.NORMAL
    id: str = field(default_factory=lambda: str(uuid.uuid4())[:8])
    created_at: float = field(default_factory=time.time)

    def __repr__(self):
        return f"<Task {self.type.name} P={self.params}>"