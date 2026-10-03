"""Execution of system-level routing plans through existing canonical boundaries."""
from __future__ import annotations

import json
import os
from pathlib import Path

from .execution_mode import ExecutionMode, ExecutionModeSelector, ExecutionPlan, RouteEvidenceJournal


def _speech_chunks(text: str, max_chars: int = 320) -> tuple[str, ...]:
    remaining = text.strip()
    if not remaining:
        return ()
    chunks: list[str] = []
    while len(remaining) > max_chars:
        window = remaining[: max_chars + 1]
        split_at = max(window.rfind(". "), window.rfind("! "), window.rfind("? "), window.rfind("\n"))
        if split_at < max_chars // 3:
            split_at = window.rfind(" ")
        if split_at < max_chars // 3:
            split_at = max_chars
        elif window[split_at : split_at + 2] in {". ", "! ", "? "}:
            split_at += 1
        piece = remaining[:split_at].strip()
        if piece:
            chunks.append(piece)
        remaining = remaining[split_at:].strip()
    if remaining:
        chunks.append(remaining)
    return tuple(chunks)


def speak_text(text: str, *, project_root: str | Path) -> None:
    if not isinstance(text, str) or not text.strip():
        return
    from r2b4_voice.tts_provider import build_tts_client
    from r2b4_voice.voice_output import PcmWavePlayer
    root = Path(project_root).expanduser().resolve()
    tts = build_tts_client(project_root=root, resident=True)
    player = PcmWavePlayer()
    for chunk in _speech_chunks(text):
        player.play(tts.synthesize(chunk))


def status_text(status: object) -> str:
    if not isinstance(status, dict):
        return "A robot futási állapota most nem olvasható."
    running = status.get("runtime_running")
    if running is not True:
        return "A V3 jelenleg nem fut. A beszélgetési rendszer ettől még használható."
    live = status.get("status")
    if isinstance(live, dict):
        state = live.get("state"); ready = live.get("ready_for_active"); fault = live.get("fault_layer")
        if fault:
            return f"A V3 fut, de hibát jelez a(z) {fault} rétegen."
        if state == "RUNNING" and ready is True:
            return "A V3 fut és végrehajtásra kész."
        return f"A V3 fut. Állapot: {state or 'ismeretlen'}, végrehajtási készség: {bool(ready)}."
    return "A V3 fut, de friss részletes runtime-állapot most nem érhető el."


def _agent_return_code(action_status: str | None) -> int:
    # Keep pure conversation successful, but expose physical action failure to shell/automation.
    if action_status is None or action_status in {"NONE", "COMPLETED", "EXECUTED"}:
        return 0
    if action_status.startswith("FAILED:") or action_status.startswith("REJECTED:"):
        return 2
    return 0


def execute_plan(plan: ExecutionPlan, *, project_root: str | Path) -> int:
    root = Path(project_root).expanduser().resolve()
    evidence = RouteEvidenceJournal(root)
    evidence.emit("ROUTE_EXECUTION_START", plan)
    try:
        if plan.mode is ExecutionMode.AGENT:
            from .agent_runner import run_agent_prompt
            result = run_agent_prompt(plan.text, project_root=root)
            print(result.text, flush=True)
            speak_text(result.text, project_root=root)
            return_code = _agent_return_code(result.action_status)
            evidence.emit(
                "ROUTE_EXECUTION_COMPLETE",
                plan,
                return_code=return_code,
                action_status=result.action_status,
            )
            return return_code

        # Legacy explicit plans remain supported, but the natural-language selector
        # no longer creates them except DIRECT_V3 STOP.
        if plan.mode is ExecutionMode.HOST_READ:
            from v3.robot_interface import RobotInterface
            payload = RobotInterface(project_root=root).read(plan.capability or "operator.status")
            text = status_text(payload)
            print(text, flush=True); speak_text(text, project_root=root)
            evidence.emit("ROUTE_EXECUTION_COMPLETE", plan, return_code=0)
            return 0

        if plan.mode in {ExecutionMode.OBSERVATION, ExecutionMode.ER2_PREVIEW, ExecutionMode.ER2_STREAM}:
            from r2b4_er2.executor import run_er2_task
            mode = "preview" if plan.mode in {ExecutionMode.OBSERVATION, ExecutionMode.ER2_PREVIEW} else "stream"
            result = run_er2_task(plan.text, project_root=root, mode=mode, camera=plan.camera, tools_enabled=plan.tools)
            if not result.text:
                raise RuntimeError("ER2 returned no final text")
            print(result.text, flush=True); speak_text(result.text, project_root=root)
            evidence.emit("ROUTE_EXECUTION_COMPLETE", plan, return_code=0, er2_mode=result.mode)
            return 0

        if plan.mode is ExecutionMode.DIRECT_V3:
            from v3.robot_interface import RobotInterface
            interface = RobotInterface(project_root=root)
            if plan.action_name == "v3.command.stop":
                interface.execute("v3.command.stop")
                text = "Megálltam."
                print(text, flush=True); speak_text(text, project_root=root)
                evidence.emit("ROUTE_EXECUTION_COMPLETE", plan, return_code=0, action_status="EXECUTED")
                return 0
            from r2b4_voice.action_executor import VoiceActionExecutor
            execution = VoiceActionExecutor(interface, mode="execute", session_owner_pid=os.getpid(), session_watchdog_s=30.0).execute_proposal({
                "name": plan.action_name, "parameters": dict(plan.action_parameters),
            })
            text = "Rendben." if execution.executed else "A parancsot most nem tudom végrehajtani."
            print(text, flush=True); speak_text(text, project_root=root)
            evidence.emit("ROUTE_EXECUTION_COMPLETE", plan, return_code=0 if execution.executed else 2, action_status=execution.status)
            return 0 if execution.executed else 2
        raise RuntimeError(f"unsupported execution mode: {plan.mode.value}")
    except Exception as exc:
        evidence.emit("ROUTE_EXECUTION_ERROR", plan, error_type=type(exc).__name__, error=str(exc)[:500])
        raise


def execute_text(text: str, *, project_root: str | Path, source: str = "launcher") -> int:
    root = Path(project_root).expanduser().resolve()
    plan = ExecutionModeSelector().select(text, source=source)
    RouteEvidenceJournal(root).emit("ROUTE_SELECTED", plan)
    return execute_plan(plan, project_root=root)


def route_json(text: str, *, source: str = "launcher") -> str:
    return json.dumps(ExecutionModeSelector().select(text, source=source).to_jsonable(), ensure_ascii=False, indent=2, sort_keys=True)


__all__ = ["execute_plan", "execute_text", "route_json", "speak_text", "status_text"]
