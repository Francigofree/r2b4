"""Offline evidence for the reported skill-creation discovery sequence."""
from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path

from r2b4_orchestration.agent_contracts import AgentModelReply, AgentToolRequest
from r2b4_orchestration.agent_core import AgentCore, AgentToolBroker
from r2b4_orchestration.agent_tools import build_default_agent_tools
from v3.action_catalog import action_catalog_jsonable, voice_action_descriptors


def _compact(value: object) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))


def _expand_capabilities(value: dict[str, object]) -> dict[str, object]:
    result = deepcopy(value)
    defaults = result.pop("capability_defaults", {})
    result.pop("capability_encoding", None)
    actions = result["action_catalog"]["actions"]
    expanded = {}
    for name, entry in result["capabilities"].items():
        entry = dict(entry)
        reference = entry.pop("action_catalog_ref", None)
        expanded[name] = {**defaults, **actions.get(reference, {}), **entry}
    result["capabilities"] = expanded
    return result


def test_full_skill_discovery_survives_prompt_growth_without_losing_tool_data(tmp_path: Path) -> None:
    docs_line = "robot.call teleop uses the published canonical action."
    source_line = "    async def call(self, name, **parameters):"
    (tmp_path / "sdk.md").write_text(docs_line + "\n", encoding="utf-8")
    (tmp_path / "sdk.py").write_text("class Robot:\n" + source_line + "\n        pass\n", encoding="utf-8")
    action_catalog = action_catalog_jsonable()
    capabilities = {}
    for name, descriptor in action_catalog["actions"].items():
        capabilities[name] = {
            **deepcopy(descriptor), "kind": "action", "supported": True,
            "available": True, "ready": True, "reason": "CANONICAL_V3_EXECUTION",
            "result": {"description": "Complete owning capability result."},
        }
    for index in range(48):
        name = f"software.discovery_{index}"
        capabilities[name] = {
            "name": name, "kind": "action", "supported": True, "available": True,
            "ready": True, "reason": "HOST_KNOWLEDGE_UPDATE", "parameters": {},
            "description": f"Published capability {index}.",
            "result": {"description": "Complete owning capability result."},
        }
    capabilities["software.discovery_0"].update(
        available=False, ready=False, reason="STALE", extension={
            "measurement_time_ns": 123, "source_sequence": 45,
            "full_requested_data": "unaltered evidence; " * 3_000 + "END_OF_FULL_EVIDENCE",
        },
    )
    full_catalog = {
        "schema": "R2B4_ROBOT_INTERFACE_V2", "action_catalog": action_catalog,
        "capabilities": capabilities, "future_extension": {"preserve": "exactly"},
    }
    saved = []

    class Interface:
        def capabilities(self):
            return deepcopy(full_catalog)

        def execute(self, action, **arguments):
            assert action == "skill.create"
            saved.append(dict(arguments))
            return {"name": arguments["name"]}

    requests = (
        AgentToolRequest("robot.capabilities", {}),
        AgentToolRequest("docs.search", {"limit": 5, "query": "robot.call teleop"}),
        AgentToolRequest("source.search", {"limit": 5, "query": "async def call("}),
    )
    calls = []
    tool_results = []
    skill_source = "async def run(robot, **parameters):\n    return {'prepared': True}\n"

    class Model:
        model = "offline"

        def complete_agent_step(self, messages, tool_catalog, actions, **_options):
            calls.append(deepcopy(messages))
            if len(calls) <= len(requests):
                return AgentModelReply(self.model, tool_request=requests[len(calls) - 1])
            for message in messages:
                marker = "R2B4_TOOL_RESULT_JSON="
                if marker in message["content"]:
                    tool_results.append(json.loads(message["content"].split(marker, 1)[1]))
            return AgentModelReply(self.model, skill_proposal={
                "kind": "create_skill", "name": "slow", "source": skill_source,
                "parameters": {}, "description": "offline preparation",
            })

    user_text = (
        "hozz létre egy új skillt, amely egyenesen előre indítja, majd megállítja a robotot, "
        "10mp-ig fut és ezalatt 0-0.2 m/s-ra gyorsít 5mp alatt, majd 0.2-0-ra lassít "
        "5 mp alatt. a skill neve legyen slow"
    )
    events = []
    core = AgentCore(Model(), AgentToolBroker(build_default_agent_tools(tmp_path, interface=Interface())))
    decision = core.run(
        [{"role": "system", "content": "policy " * 2_900},
         {"role": "user", "content": user_text}],
        tuple(descriptor.to_jsonable() for descriptor in voice_action_descriptors()),
        event_sink=lambda kind, fields: events.append((kind, dict(fields))),
    )

    assert len(calls) == 4
    assert decision.goal_plan == {"skill": "slow", "parameters": {}}
    assert saved[0]["source"] == skill_source
    assert [result["name"] for result in tool_results] == [request.name for request in requests]
    assert all(result["status"] == "COMPLETED" for result in tool_results)
    assert _expand_capabilities(tool_results[0]["data"]) == full_catalog
    assert len(_compact(tool_results[0]["data"])) < len(_compact(full_catalog))
    assert tool_results[1]["data"]["hits"] == [{"path": "sdk.md", "line": 1, "text": docs_line}]
    assert tool_results[2]["data"]["hits"] == [{"path": "sdk.py", "line": 2, "text": source_line.strip()}]
    assert all(call[-1] == {"role": "user", "content": user_text} for call in calls)
    started = [fields for kind, fields in events if kind == "agent_llm_started"]
    assert started[-1]["provider_request_chars_estimate"] > 64_000
    assert not any(kind == "agent_prompt_rejected" for kind, _fields in events)
