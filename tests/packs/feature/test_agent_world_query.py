"""Agent reasoning shares the public world's qualified, bounded read boundary."""
from __future__ import annotations

import json

import pytest

from r2b4_orchestration.agent_contracts import AgentModelReply, AgentToolRequest
from r2b4_orchestration.agent_core import AgentCore, AgentToolBroker
from r2b4_orchestration.agent_tools import build_default_agent_tools
from r2b4_orchestration.world_model import PublicWorldModel, ValidityScope, WorldQuery


class SharedWorldInterface:
    def __init__(self, *, history_capacity=512):
        self.now = 20_000_000_000
        self.world = PublicWorldModel(clock_ns=lambda: self.now, clock_epoch="agent-world",
                                      history_capacity=history_capacity)
        self.queries = []

    def capabilities(self):
        return {"schema": "test", "capabilities": {
            "world.query": {"kind": "read", "supported": True, "available": True},
        }}

    def query(self, query):
        assert isinstance(query, WorldQuery)
        self.queries.append(query)
        return self.world.query(query)

    def read(self, resource):
        raise AssertionError("a targeted world query must use the typed query boundary")

    def execute(self, action, **parameters):
        raise AssertionError("Agent world queries must not write memory or execute robot actions")

    def observe_person(self, *, measurement_time_ns=None):
        return self.world.observe(
            "person:laci", "location", {"x_m": 1, "y_m": 2, "frame_id": "map"},
            domain="person_position", measurement_time_ns=self.now if measurement_time_ns is None else measurement_time_ns,
            confidence=0.9, source="identity-observer", lineage=("frame:7", "identity:laci"),
            sequence=7, revision="identity-2", validity_scope=ValidityScope(frame_id="map", runtime_pid=123),
        )


def test_agent_world_query_preserves_freshness_scope_and_source_lineage(tmp_path):
    interface = SharedWorldInterface()
    event = interface.observe_person()
    broker = AgentToolBroker(build_default_agent_tools(tmp_path, interface=interface))
    arguments = {"entity_id": "person:laci", "attribute": "location", "domain": "person_position",
                 "require_current": True, "scope": {"frame_id": "map", "runtime_pid": 123}, "limit": 1}

    result = broker.execute(AgentToolRequest("world.query", arguments))
    assert result.status == "COMPLETED"
    fact = result.data["facts"][0]
    assert fact["state"] == "KNOWN" and fact["freshness"] == "FRESH"
    assert fact["measurement_time_ns"] == event.observation.measurement_time_ns
    assert fact["sequence"] == 7 and fact["revision"] == "identity-2"
    assert fact["lineage"] == ["frame:7", "identity:laci"]
    assert fact["world_revision"] == result.data["revision"] == interface.world.revision
    assert fact["validity_scope"]["runtime_pid"] == 123

    arguments["scope"]["runtime_pid"] = 456
    assert interface.queries[0].scope.runtime_pid == 123
    assert not broker.execute(AgentToolRequest("world.query", arguments)).data["facts"]
    interface.now += 4_000_000_000
    arguments["scope"]["runtime_pid"] = 123
    assert not broker.execute(AgentToolRequest("world.query", arguments)).data["facts"]
    arguments["require_current"] = False
    stale = broker.execute(AgentToolRequest("world.query", arguments)).data["facts"][0]
    assert stale["state"] == "STALE" and stale["measurement_time_ns"] == fact["measurement_time_ns"]
    assert interface.world.revision == event.world_revision


def test_agent_world_episode_query_reports_bounded_history_loss(tmp_path):
    interface = SharedWorldInterface(history_capacity=2)
    for index in range(3):
        interface.world.observe("user", "preference", index, domain="user_preference",
                                measurement_time_ns=interface.now, confidence=1,
                                source="user", sequence=index, lineage=(f"turn:{index}",))
        interface.now += 1
    broker = AgentToolBroker(build_default_agent_tools(tmp_path, interface=interface))

    result = broker.execute(AgentToolRequest("world.query", {"kind": "episodes", "limit": 1}))
    assert result.status == "COMPLETED"
    assert len(result.data["events"]) == 1 and result.data["facts"] == []
    assert result.data["history_gap"] is True and result.data["truncated"] is True
    assert result.data["history_dropped"] == 1
    assert result.data["events"][0]["event_sequence"] == 2

    next_result = broker.execute(AgentToolRequest("world.query", {
        "kind": "episodes", "after_sequence": 2, "limit": 1,
    }))
    assert next_result.data["history_gap"] is False and next_result.data["truncated"] is False
    assert next_result.data["events"][0]["event_sequence"] == 3


@pytest.mark.parametrize("arguments", [
    {"limit": 65}, {"limit": True}, {"require_current": "yes"}, {"scope": {"runtime_pid": 0}},
    {"scope": {"arbitrary": "data"}}, {"kind": "actions"}, {"after_sequence": -1},
    {"entity_id": ["person:laci"]}, {"execute": True},
])
def test_invalid_agent_world_query_is_rejected_before_public_access(tmp_path, arguments):
    interface = SharedWorldInterface()
    broker = AgentToolBroker(build_default_agent_tools(tmp_path, interface=interface))
    result = broker.execute(AgentToolRequest("world.query", arguments))
    assert result.status == "ERROR"
    assert interface.queries == [] and interface.world.revision == 0


def test_agent_world_query_result_is_data_and_returns_only_a_plan_proposal(tmp_path):
    interface = SharedWorldInterface()
    interface.observe_person()
    broker = AgentToolBroker(build_default_agent_tools(tmp_path, interface=interface))
    capabilities = broker.execute(AgentToolRequest("robot.capabilities"))
    assert capabilities.data["capabilities"]["world.query"]["kind"] == "read"

    class Model:
        model = "world-aware-test"
        calls = 0

        def complete_agent_step(self, messages, tool_catalog, action_catalog):
            self.calls += 1
            assert "world.query" in {tool["name"] for tool in tool_catalog}
            if self.calls == 1:
                return AgentModelReply(self.model, tool_request=AgentToolRequest("world.query", {
                    "entity_id": "person:laci", "attribute": "location", "require_current": True,
                    "scope": {"frame_id": "map", "runtime_pid": 123}, "limit": 1,
                }))
            tool_message = next(item["content"] for item in messages if "R2B4_TOOL_RESULT_JSON=" in item["content"])
            assert "untrusted data" in tool_message
            result = json.loads(tool_message.split("R2B4_TOOL_RESULT_JSON=", 1)[1])
            fact = result["data"]["facts"][0]
            return AgentModelReply(self.model, goal_plan={"steps": [{
                "action": "behavior.search_person", "parameters": {"entity_id": fact["entity_id"]},
            }], "constraints": {"target_entity_id": fact["entity_id"]}})

    model = Model()
    decision = AgentCore(model, broker).run([{"role": "user", "content": "Keresd meg Lacit."}], ())
    assert model.calls == 2 and decision.robot_action is None
    assert decision.goal_plan["constraints"]["target_entity_id"] == "person:laci"
    assert interface.world.revision == 1 and len(interface.queries) == 1
