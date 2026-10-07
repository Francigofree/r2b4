"""RobotInterface composition for local task planning and on-demand specialists."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
import queue
import threading
import time

from r2b4_orchestration.agent_core import AgentCore, AgentToolBroker
from r2b4_orchestration.agent_tools import build_default_agent_tools

from .conversation_journal import ConversationJournal
from .conversation_service import ConversationService, ConversationServiceConfig
from .llm_provider import build_llm_client, default_model_for, resolve_llm_provider
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
                "description": "Submit one Brain-owned text request for local planning or specialist interpretation.",
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


class _OnDemandLLM:
    """Authentication/provider initialization belongs to specialist escalation."""

    def __init__(self, **settings):
        self._settings = settings
        self._client = None
        self._configured_model = settings.get("model") or default_model_for(resolve_llm_provider(settings.get("provider")))

    @property
    def model(self):
        return self._client.model if self._client is not None else self._configured_model

    def _get(self):
        if self._client is None:
            self._client = build_llm_client(**self._settings)
        return self._client

    def complete(self, messages):
        return self._get().complete(messages)

    def complete_agent_step(self, *args, **kwargs):
        return self._get().complete_agent_step(*args, **kwargs)


@dataclass(slots=True)
class VoiceInterfaceBundle:
    interface: object
    conversation: ConversationService
    observation: "_AgentObservationJournal | None" = None

    def close(self) -> None:
        try:
            self.conversation.close()
        finally:
            if self.observation is not None:
                self.observation.close()

    def __enter__(self) -> "VoiceInterfaceBundle":
        return self

    def __exit__(self, _exc_type, _exc, _tb) -> None:
        self.close()


class _AgentObservationJournal:
    """Small host journal writer; Agent publication only offers scalar rows."""

    def __init__(self, journal) -> None:
        self._journal = journal
        self._queue = queue.Queue(maxsize=256)
        self._closed = threading.Event()
        self._admission_lock = threading.Lock()
        self._last_fields = None
        self._offer_dropped = 0
        self._write_dropped = 0
        self._thread = threading.Thread(target=self._run, name="r2b4-agent-evidence", daemon=True)
        self._thread.start()

    def offer(self, event: str, fields: Mapping[str, object]) -> None:
        with self._admission_lock:
            if self._closed.is_set():
                raise RuntimeError("agent observation journal is closed")
            self._last_fields = fields
            try:
                self._queue.put_nowait((event, fields))
            except queue.Full:
                self._offer_dropped += 1
                raise

    def _run(self) -> None:
        while True:
            try:
                event, fields = self._queue.get(timeout=0.05)
            except queue.Empty:
                if self._closed.is_set():
                    self._finish_loss()
                    return
                continue
            try:
                self._journal.append(event, **{
                    **fields, "evidence_dropped": int(fields.get("evidence_dropped", 0)) + self._write_dropped,
                })
            except Exception:
                self._write_dropped += 1
            finally:
                self._queue.task_done()

    def close(self) -> None:
        with self._admission_lock:
            self._closed.set()
        self._thread.join(timeout=2.0)

    def _finish_loss(self) -> None:
        fields = self._last_fields
        if fields is None or not (self._offer_dropped or self._write_dropped):
            return
        try:
            self._journal.append("AGENT_EVIDENCE_LOSS", **{
                **fields, "monotonic_ns": time.monotonic_ns(),
                "event_sequence": int(fields["event_sequence"]) + 1,
                "evidence_dropped": self._offer_dropped + self._write_dropped,
            })
        except Exception:
            pass


def build_voice_interface(
    project_root: Path | str | None = None,
    *,
    api_key: str | None = None,
    provider: str | None = None,
    model: str | None = None,
    developer_mode: bool = False,
    service_config: ConversationServiceConfig = ConversationServiceConfig(),
) -> VoiceInterfaceBundle:
    """Build one RobotInterface facade with local planning before Agent Core.

    The model receives only typed capability descriptions/results. It never gets a
    motor/GPIO/runtime handle. Local and specialist plans remain proposals for
    Brain admission and execution through the canonical RobotInterface.
    """
    from v3.operator_controller import OperatorController
    from v3.robot_interface import RobotInterface
    from v3.hri_evidence import default_hri_journal

    root = Path(project_root) if project_root is not None else Path(__file__).resolve().parents[1]
    root = root.resolve()
    if type(developer_mode) is not bool:
        raise TypeError("developer_mode must be a boolean")
    controller = OperatorController(project_root=root)
    core_interface = RobotInterface(project_root=root, controller=controller)

    llm = _OnDemandLLM(provider=provider, api_key=api_key, model=model, project_root=root)
    broker = AgentToolBroker(build_default_agent_tools(
        root, interface=core_interface, developer_mode=developer_mode,
    ))
    agent = AgentCore(llm, broker, max_tool_rounds=4)
    prompt = PromptAssembler(
        root / "conf" / "r2b4_agent_system.md",
        max_history_turns=service_config.max_history_turns,
    )
    journal = ConversationJournal(root / "runtime" / "conversations")
    observation = _AgentObservationJournal(default_hri_journal(root))
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
        brain_interface=core_interface,
        observation_sink=observation.offer,
    )
    public_adapters = core_interface.adapters + (ConversationInterfaceAdapter(service, developer_mode=developer_mode),)
    public_interface = RobotInterface(project_root=root, controller=controller, adapters=public_adapters)
    return VoiceInterfaceBundle(public_interface, service, observation)


__all__ = ["ConversationInterfaceAdapter", "VoiceInterfaceBundle", "build_voice_interface"]
