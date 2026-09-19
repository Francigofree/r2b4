"""Static composition of thin RobotInterface adapters.

The V3 import guard intentionally forbids dynamic imports.  Adapter discovery is
therefore source-first and static here rather than using importlib/pkgutil.  This
file is composition only; it stores no runtime registry or duplicated state.
"""

from __future__ import annotations

from pathlib import Path

from v3.interface_adapters.camera import CameraInterfaceAdapter
from v3.interface_adapters.operator import OperatorInterfaceAdapter
from v3.interface_adapters.system import SystemInterfaceAdapter
from v3.interface_adapters.testhub import TestHubInterfaceAdapter
from v3.interface_adapters.v3_control import V3ControlInterfaceAdapter
from v3.operator_controller import OperatorController


def build_adapters(controller: OperatorController, root: Path) -> tuple[object, ...]:
    return (
        V3ControlInterfaceAdapter(controller),
        OperatorInterfaceAdapter(controller),
        CameraInterfaceAdapter(controller),
        TestHubInterfaceAdapter(controller, root),
        SystemInterfaceAdapter(root),
    )


__all__ = ["build_adapters"]
