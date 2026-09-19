"""Static composition of thin RobotInterface adapters.

This is host-side composition only. It owns no runtime/control authority and
stores no registry or duplicated robot state.
"""

from __future__ import annotations

from pathlib import Path

from v3.adapters.camera import CameraInterfaceAdapter
from v3.adapters.operator import OperatorInterfaceAdapter
from v3.adapters.system import SystemInterfaceAdapter
from v3.adapters.testhub import TestHubInterfaceAdapter
from v3.adapters.v3_control import V3ControlInterfaceAdapter


def build_adapters(controller: object, root: Path) -> tuple[object, ...]:
    """Build the live RobotInterface adapter set from the canonical adapters."""
    return (
        V3ControlInterfaceAdapter(controller),
        OperatorInterfaceAdapter(controller),
        CameraInterfaceAdapter(controller),
        TestHubInterfaceAdapter(controller, root),
        SystemInterfaceAdapter(root),
    )


__all__ = ["build_adapters"]
