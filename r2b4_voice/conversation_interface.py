"""RobotInterface adapter and composition for host-side R2B4 Agent Core conversation."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

from r2b4_orchestration.agent_core import AgentCore, AgentToolBroker
from r2b4_orchestration.agent_tools import build_default_agent_tools

from .conversation_journal import ConversationJournal
from .conversation_service import ConversationService, ConversationServiceConfig
from .llm_provider import build_llm_client
from .prompting import PromptAssembler
from .robot_context import RobotContextBuilder


class ConversationInterfaceAdapter:
    name = "conversation"
    capability_names = frozenset({
        "conversation.submit_text",
        "conversation.status",
        "conversation.last_turn",
    })

    def __init__(self, service: ConversationService, *, developer_mode: bool = False) -> None:
        self.service = service
        self.developer_mode = developer_mode

    def capabilities(self) -> Mapping[str, Mapping[str, object]]:
        status = self.service.status()
        running = status.get("state") == "RUNNING" and status.get("worker_alive") is True
        return {
            "conversation.submit_text": {
                "kind": "action",
                "supported": True,
                "available": running,
                "ready": running,
                "reason": None if running else "CONVERSATION_SERVICE_NOT_RUNNING",
                "description": "Submit one user text turn into the R2B4 Agent Core conversation orchestrator.",
                "parameters": {"text": "string", "source": "string=stt"},
                "agent_mode": "DEVELOPER" if self.developer_mode else "RUNTIME",
            },
            "conversation.status": {
                "kind": "read",
                "supported": True,
                "available": True,
                "ready": True,
                "description": "Read conversation/Agent Core worker, queue and model status.",
            },
            "conversation.last_turn": {
                "kind": "read",
                "supported": True,
                "available": True,
                "ready": True,
                "description": "Read the latest completed Agent Core turn, including any proposed robot intent.",
            },
        }

    def read(self, resource: str) -> object:
        if resource == "conversation.status":
            return self.service.status()
        if resource == "conversation.last_turn":
            return self.service.last_turn()
        raise KeyError(resource)

    def execute(self, action: str, **parameters: object) -> object:
        if action != "conversation.submit_text":
            raise KeyError(action)
        params = dict(parameters)
        text = params.pop("text", None)
        source = params.pop("source", "stt")
        if params:
            raise ValueError(f"unknown parameters: {', '.join(sorted(params))}")
        if not isinstance(text, str):
            raise ValueError("text must be a string")
        if not isinstance(source, str):
            raise ValueError("source must be a string")
        turn_id = self.service.submit_text(text, source=source)
        return {
            "status": "ACCEPTED", "turn_id": turn_id, "action_mode": "PROPOSAL_ONLY",
            "agent_mode": "DEVELOPER" if self.developer_mode else "RUNTIME",
        }


@dataclass(slots=True)
class VoiceInterfaceBundle:
    interface: object
    conversation: ConversationService

    def close(self) -> None:
        self.conversation.close()

    def __enter__(self) -> "VoiceInterfaceBundle":
        return self

    def __exit__(self, _exc_type, _exc, _tb) -> None:
        self.close()


def build_voice_interface(
    project_root: Path | str | None = None,
    *,
    api_key: str | None = None,
    provider: str | None = None,
    model: str | None = None,
    developer_mode: bool = False,
    service_config: ConversationServiceConfig = ConversationServiceConfig(),
) -> VoiceInterfaceBundle:
    """Build one RobotInterface facade plus provider-neutral bounded Agent Core.

    The model receives only typed capability descriptions/results. It never gets a
    motor/GPIO/runtime handle. Robot actions remain proposals for the existing
    fresh-state VoiceActionExecutor gate.
    """
    from v3.operator_controller import OperatorController
    from v3.robot_interface import RobotInterface

    root = Path(project_root) if project_root is not None else Path(__file__).resolve().parents[1]
    root = root.resolve()
    if type(developer_mode) is not bool:
        raise TypeError("developer_mode must be a boolean")
    controller = OperatorController(project_root=root)
    core_interface = RobotInterface(project_root=root, controller=controller)

    llm = build_llm_client(provider=provider, api_key=api_key, model=model, project_root=root)
    broker = AgentToolBroker(build_default_agent_tools(
        root, interface=core_interface, developer_mode=developer_mode,
    ))
    agent = AgentCore(llm, broker, max_tool_rounds=4)
    prompt = PromptAssembler(
        root / "conf" / "r2b4_agent_system.md",
        max_history_turns=service_config.max_history_turns,
    )
    journal = ConversationJournal(root / "runtime" / "conversations")
    service = ConversationService(
        llm=llm,
        agent=agent,
        robot_context=RobotContextBuilder(core_interface),
        prompt_assembler=prompt,
        journal=journal,
        # Dynamic Agent Core tools supersede the legacy Test-Hub-era evidence
        # snippets in SelfKnowledgeProvider. Keep that module available only for
        # compatibility callers; do not inject stale evidence into new turns.
        self_knowledge=None,
        config=service_config,
    )
    public_adapters = core_interface.adapters + (ConversationInterfaceAdapter(service, developer_mode=developer_mode),)
    public_interface = RobotInterface(project_root=root, controller=controller, adapters=public_adapters)
    return VoiceInterfaceBundle(public_interface, service)


__all__ = ["ConversationInterfaceAdapter", "VoiceInterfaceBundle", "build_voice_interface"]
