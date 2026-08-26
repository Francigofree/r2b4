#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
R2B4 Startup Pipeline – állapotgépes indítás és supervisor.
A robot indítása explicit startup state machine-en keresztül történik.
"""

from startup.state_machine import (
    StartupState,
    StartupStateMachine,
)
from startup.supervisor import StartupSupervisor
from startup.phases import run_startup_pipeline
from startup.sensor_health import run_all_checks

__all__ = [
    "StartupState",
    "StartupStateMachine",
    "StartupSupervisor",
    "run_startup_pipeline",
    "run_all_checks",
]
