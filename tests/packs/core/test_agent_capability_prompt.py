from __future__ import annotations

from copy import deepcopy
import json

import pytest

from r2b4_orchestration.agent_contracts import AgentToolRequest
from r2b4_orchestration.agent_core import AgentToolBroker
from r2b4_orchestration.agent_tools import _capability_prompt, build_default_agent_tools
from r2b4_orchestration.robot_runtime import skill_capabilities
from r2b4_orchestration.software_interface import SoftwareInterfaceAdapter
from v3.action_catalog import ACTION_CATALOG
from v3.adapters.v3_control import V3ControlInterfaceAdapter
from v3.robot_interface import RobotInterface


def _json(value):
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"), sort_keys=True)


def _restore(value):
    if "capability_encoding" not in value:
        return value
    result = {key: item for key, item in value.items()
              if key not in {"capability_encoding", "capability_defaults"}}
    capabilities = {}
    for name, item in value["capabilities"].items():
        restored = dict(value["capability_defaults"])
        if "action_catalog_ref" in item:
            restored.update(value["action_catalog"]["actions"][item["action_catalog_ref"]])
        restored.update({key: field for key, field in item.items() if key != "action_catalog_ref"})
        capabilities[name] = restored
    result["capabilities"] = capabilities
    return result


def test_agent_discovery_preserves_all_canonical_descriptors_and_public_interface(tmp_path):
    class CanonicalAdapter:
        name = "canonical"

        def capabilities(self):
            return {name: V3ControlInterfaceAdapter._action(descriptor, True, True, "READY")
                    for name, descriptor in ACTION_CATALOG.items()}

    class SkillsAdapter:
        name = "skills"

        def capabilities(self):
            return skill_capabilities()

    # Explicit adapters publish actual canonical contracts without touching a
    # controller, public-runtime process, provider or physical device.
    interface = RobotInterface(tmp_path, controller=object(), upper_runtime=False,
                               adapters=(CanonicalAdapter(), SkillsAdapter(), SoftwareInterfaceAdapter(tmp_path)))
    original = interface.capabilities()
    broker = AgentToolBroker(build_default_agent_tools(tmp_path, interface=interface))
    result = broker.execute(AgentToolRequest("robot.capabilities"))
    assert result.status == "COMPLETED"
    compact = json.loads(_json(result.data))
    assert _json(_restore(compact)) == _json(original)
    assert interface.capabilities() == original
    assert compact["action_catalog"] == original["action_catalog"]
    assert set(compact["capabilities"]) == set(original["capabilities"])
    assert all(compact["capabilities"][name]["action_catalog_ref"] == name for name in ACTION_CATALOG)
    assert len(_json(compact)) < len(_json(original))


def test_discovery_preserves_missing_null_dynamic_status_and_extension_values():
    value = {"schema": "interface", "revision": 42,
             "lineage": {"measurement_time_ns": 100, "source": "adapter"},
             "unknown_metadata": {"nested": [False, 0, None]}, "capabilities": {}}
    for index in range(24):
        item = {"name": f"extension.{index}", "kind": "action", "supported": True,
                "available": True, "ready": True, "reason": None,
                "description": "Complete documentation with no truncation. " * 6,
                "parameters": {"required": {"type": "string", "description": "exact argument"}},
                "measurement_time_ns": 10_000 + index, "sequence": index,
                "lineage": [f"frame-{index}"], "optional": None,
                "extension": {"exact": [None, True, 1, 1.0, "arbitrary"]}}
        value["capabilities"][item["name"]] = item
    value["capabilities"]["extension.0"].pop("optional")
    value["capabilities"]["extension.1"].update(available=False, ready=False, reason="STALE")
    value["capabilities"]["extension.2"].update(ready=None, reason=0)
    value["capabilities"]["extension.3"].update(reason=False)
    before = deepcopy(value)
    compact = _capability_prompt(value)
    assert value == before
    assert "optional" not in compact["capability_defaults"]
    assert _json(_restore(json.loads(_json(compact)))) == _json(value)
    assert len(_json(compact)) < len(_json(value))


def test_catalog_overrides_and_missing_catalog_fields_restore_exactly():
    from v3.action_catalog import action_catalog_jsonable

    catalogue = action_catalog_jsonable()
    capabilities = {name: {**descriptor, "available": True, "ready": True,
                           "reason": None, "extension": {"source": "live"}}
                    for name, descriptor in catalogue["actions"].items()}
    names = list(capabilities)
    capabilities[names[0]]["description"] = "Live documentation override"
    capabilities[names[0]]["parameters"] = {"extension": {"type": "number"}}
    capabilities[names[1]]["requirements"] = None
    capabilities[names[2]].pop("completion_required")
    value = {"action_catalog": catalogue, "capabilities": capabilities}
    compact = _capability_prompt(value)
    assert compact["capabilities"][names[0]]["description"] == "Live documentation override"
    assert compact["capabilities"][names[0]]["parameters"] == {"extension": {"type": "number"}}
    assert compact["capabilities"][names[1]]["requirements"] is None
    assert "action_catalog_ref" not in compact["capabilities"][names[2]]
    assert _json(_restore(json.loads(_json(compact)))) == _json(value)


@pytest.mark.parametrize("reserved", ["capability_defaults", "capability_encoding", "action_catalog_ref"])
def test_reserved_extension_keys_do_not_change_original_meaning(reserved):
    value = {"capabilities": {f"action.{index}": {"kind": "action", "supported": True,
              "description": "Repeated long documentation. " * 20} for index in range(8)}}
    if reserved == "action_catalog_ref":
        value["capabilities"]["action.0"][reserved] = "existing extension"
    else:
        value[reserved] = {"existing": "metadata"}
    assert _capability_prompt(value) is value


def test_small_or_empty_results_keep_original_without_encoding_overhead():
    for value in ({"capabilities": {}}, {"capabilities": {"read": {"kind": "read"}}},
                  {"capabilities": {"one": {"kind": "read"}, "two": {"kind": "read"}}}):
        assert _capability_prompt(value) is value
