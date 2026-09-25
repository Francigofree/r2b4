from __future__ import annotations

import pytest

from v3.robot_interface import ROBOT_INTERFACE_SCHEMA, RobotInterface, RobotInterfaceError


class _FakeAdapter:
    name = "fake"
    capability_names = frozenset({"fake.read", "fake.action"})

    def __init__(self) -> None:
        self.value = 1

    def capabilities(self):
        return {
            "fake.read": {
                "kind": "read",
                "supported": True,
                "available": True,
                "ready": True,
            },
            "fake.action": {
                "kind": "action",
                "supported": True,
                "available": True,
                "ready": False,
                "reason": "ACTION_MAY_TRANSITION_TO_READY",
            },
        }

    def read(self, resource):
        if resource != "fake.read":
            raise KeyError(resource)
        return {"value": self.value}

    def execute(self, action, **parameters):
        if action != "fake.action":
            raise KeyError(action)
        self.value = int(parameters["value"])
        return {"value": self.value}


class _DuplicateAdapter(_FakeAdapter):
    name = "duplicate"
    capability_names = frozenset({"fake.read", "fake.action"})


class _DummyController:
    pass


def _interface(*adapters):
    return RobotInterface(controller=_DummyController(), adapters=adapters)


def test_interface_capabilities_are_live_and_dispatch_without_registry():
    adapter = _FakeAdapter()
    interface = _interface(adapter)

    capabilities = interface.capabilities()
    assert capabilities["schema"] == ROBOT_INTERFACE_SCHEMA
    assert capabilities["capabilities"]["fake.read"]["adapter"] == "fake"
    assert interface.read("fake.read") == {"value": 1}

    # ready=False is informative, not a generic gate: the owner action may make
    # the subsystem ready through its canonical path.
    assert interface.execute("fake.action", value=7) == {"value": 7}
    assert interface.read("fake.read") == {"value": 7}


def test_interface_rejects_unknown_or_wrong_kind_requests():
    interface = _interface(_FakeAdapter())
    with pytest.raises(RobotInterfaceError, match="unknown interface read"):
        interface.read("missing")
    with pytest.raises(RobotInterfaceError, match="not 'read'"):
        interface.read("fake.action")
    with pytest.raises(RobotInterfaceError, match="not 'action'"):
        interface.execute("fake.read")


def test_interface_fails_closed_on_duplicate_capability_names():
    interface = _interface(_FakeAdapter(), _DuplicateAdapter())
    with pytest.raises(RobotInterfaceError, match="duplicate interface capability"):
        interface.capabilities()
    with pytest.raises(RobotInterfaceError, match="ambiguous interface capability"):
        interface.read("fake.read")
