from __future__ import annotations

from r2b4_voice.action_validation import RobotActionValidator
from r2b4_voice.conversation_contracts import RobotAction, RobotContextSnapshot
from r2b4_voice.llm_decision import build_decision_schema, parse_llm_decision
from v3.action_catalog import ACTION_CATALOG, action_catalog_jsonable


def _context(*names: str) -> RobotContextSnapshot:
    actions = []
    for name in names:
        descriptor = ACTION_CATALOG[name].to_jsonable()
        descriptor.update({"available": True, "ready": True, "reason": None})
        actions.append(descriptor)
    return RobotContextSnapshot(
        schema="CTX",
        runtime={"state": "RUNNING"},
        pose=None,
        safety=None,
        health=(),
        available_actions=tuple(actions),
    )


def test_catalog_contains_every_current_v3_command_and_all_are_explicitly_exposed():
    assert set(ACTION_CATALOG) == {
        "v3.command.stop",
        "v3.command.forward",
        "v3.command.backward",
        "v3.command.teleop",
        "v3.command.wheels",
        "v3.command.explore",
        "v3.command.face_person",
        "v3.command.follow_person",
    }
    assert all(item.voice_exposed is True for item in ACTION_CATALOG.values())
    assert action_catalog_jsonable()["schema"] == "R2B4_ACTION_CATALOG_V1"


def test_dynamic_schema_is_generated_from_supplied_catalog_not_hardcoded_enum():
    context = _context("v3.command.forward", "v3.command.explore")
    schema = build_decision_schema(context.available_actions)
    enum = schema["properties"]["action_name"]["enum"]
    assert enum == [None, "v3.command.forward", "v3.command.explore"]
    params = schema["properties"]["action_parameters"]["properties"]
    assert set(params) == {"speed_mps"}


def test_parser_and_validator_share_catalog_parameter_contract():
    context = _context("v3.command.wheels")
    raw = {
        "spoken_text": "Rendben.",
        "action_name": "v3.command.wheels",
        "action_parameters": {"left_mps": 0.1, "right_mps": 0.1},
    }
    decision = parse_llm_decision(raw, model="fake", action_catalog=context.available_actions)
    assert decision.robot_action is not None
    assert RobotActionValidator().validate(decision.robot_action, context).accepted is True

    missing = RobotAction("v3.command.wheels", (("left_mps", 0.1),))
    result = RobotActionValidator().validate(missing, context)
    assert result.reason == "MISSING_REQUIRED_PARAMETER:right_mps"


def test_unknown_action_fails_closed_without_private_voice_allowlist():
    context = _context("v3.command.forward")
    result = RobotActionValidator().validate(RobotAction("v3.command.not_real"), context)
    assert result.accepted is False
    assert result.reason == "ACTION_NOT_CATALOGED"
