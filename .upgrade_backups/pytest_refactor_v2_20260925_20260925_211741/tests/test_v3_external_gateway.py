import json

from v3.external_gateway import ExternalRobotGateway, GatewayPolicy
from v3_external_gateway import handle_json_line


class FakeInterface:
    def __init__(self):
        self.calls = []
        self.stops = 0

    def capabilities(self):
        return {"schema": "X", "capabilities": {"system.status": {"kind": "read"}}}

    def read(self, resource):
        self.calls.append(("read", resource))
        if resource == "missing":
            raise RuntimeError("resource unavailable: missing")
        return {"resource": resource}

    def execute(self, action, **parameters):
        self.calls.append(("execute", action, parameters))
        if action == "bad":
            raise ValueError("bad action")
        return {"action": action, "parameters": parameters}

    def stop(self):
        self.stops += 1
        return {"status": "STOPPED"}


def test_read_and_capabilities_delegate_without_positive_authority():
    fake = FakeInterface()
    gateway = ExternalRobotGateway(fake)

    caps = gateway.handle({"request_id": "1", "operation": "capabilities"})
    read = gateway.handle({"request_id": "2", "operation": "read", "name": "system.status"})

    assert caps.status == "COMPLETED"
    assert read.status == "COMPLETED"
    assert read.result == {"resource": "system.status"}
    assert fake.calls == [("read", "system.status")]


def test_stop_is_always_available_through_explicit_fail_safe_operation():
    fake = FakeInterface()
    gateway = ExternalRobotGateway(fake, policy=GatewayPolicy(allow_execute=False))

    response = gateway.handle({"request_id": "stop-1", "operation": "stop"})
    also_execute_form = gateway.handle(
        {"request_id": "stop-2", "operation": "execute", "name": "v3.command.stop"}
    )

    assert response.status == "COMPLETED"
    assert also_execute_form.status == "COMPLETED"
    assert fake.stops == 2


def test_positive_execute_is_denied_by_default():
    fake = FakeInterface()
    response = ExternalRobotGateway(fake).handle(
        {"request_id": "3", "operation": "execute", "name": "v3.command.face_person"}
    )

    assert response.status == "DENIED"
    assert response.error == "POSITIVE_ACTION_EXECUTION_DISABLED"
    assert fake.calls == []


def test_enabled_motion_execution_gets_gateway_owner_and_watchdog():
    fake = FakeInterface()
    gateway = ExternalRobotGateway(
        fake,
        policy=GatewayPolicy(
            allow_execute=True,
            session_owner_pid=4321,
            session_watchdog_s=17.5,
        ),
    )

    response = gateway.handle(
        {
            "request_id": "4",
            "operation": "execute",
            "name": "v3.command.follow_person",
            "parameters": {"max_v_mps": 0.12, "max_omega_rad_s": 0.25},
        }
    )

    assert response.status == "ACCEPTED"
    assert fake.calls == [
        (
            "execute",
            "v3.command.follow_person",
            {
                "max_v_mps": 0.12,
                "max_omega_rad_s": 0.25,
                "session_owner_pid": 4321,
                "session_watchdog_s": 17.5,
            },
        )
    ]


def test_client_cannot_override_gateway_session_ownership():
    fake = FakeInterface()
    gateway = ExternalRobotGateway(fake, policy=GatewayPolicy(allow_execute=True))

    response = gateway.handle(
        {
            "request_id": "5",
            "operation": "execute",
            "name": "v3.command.explore",
            "parameters": {"session_owner_pid": 1},
        }
    )

    assert response.status == "REJECTED"
    assert "gateway owns session parameters" in response.error
    assert fake.calls == []


def test_jsonl_transport_returns_closed_response_for_bad_json_and_unavailable_read():
    gateway = ExternalRobotGateway(FakeInterface())

    bad = json.loads(handle_json_line(gateway, "{not-json"))
    missing = json.loads(
        handle_json_line(
            gateway,
            json.dumps({"request_id": "6", "operation": "read", "name": "missing"}),
        )
    )

    assert bad["status"] == "REJECTED"
    assert missing["status"] == "UNAVAILABLE"
    assert missing["request_id"] == "6"
