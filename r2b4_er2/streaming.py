"""Gemini Robotics ER 2 Streaming over the Gemini Live API."""
from __future__ import annotations

import asyncio
from collections.abc import Callable, Mapping
from dataclasses import dataclass

from .config import Er2Config, api_key_from_env
from .evidence import Er2Evidence
from .media import VisionMediaClient
from .tool_bridge import Er2RobotTools


class Er2StreamingError(RuntimeError):
    pass


@dataclass(frozen=True, slots=True)
class Er2StreamingResult:
    reconnect_count: int
    latest_resumption_handle: str | None
    stopped_cleanly: bool


class Er2StreamingClient:
    def __init__(
        self,
        tools: Er2RobotTools,
        media: VisionMediaClient,
        config: Er2Config | None = None,
        *,
        api_key: str | None = None,
        client: object | None = None,
        on_text: Callable[[str], None] | None = None,
        evidence: Er2Evidence | None = None,
    ) -> None:
        self.tools = tools
        self.media = media
        self.config = config or tools.config
        self._api_key = api_key
        self._client = client
        self._on_text = on_text or (lambda text: print(text, end="", flush=True))
        self.evidence = evidence
        self._resume_handle: str | None = None

    def _emit(self, event_type: str, **fields: object) -> None:
        if self.evidence is not None:
            self.evidence.emit(event_type, provider="gemini", endpoint="streaming", **fields)

    def _sdk(self):
        try:
            from google import genai
            from google.genai import types
        except ImportError as exc:
            raise Er2StreamingError("google-genai is not installed") from exc
        client = self._client or genai.Client(api_key=self._api_key or api_key_from_env())
        return client, types

    async def run_async(self, task: str, *, duration_s: float | None = None) -> Er2StreamingResult:
        if not isinstance(task, str) or not task.strip():
            raise ValueError("task must be non-empty")
        if duration_s is not None and duration_s <= 0:
            raise ValueError("duration_s must be positive or None")

        client, types = self._sdk()
        stop_event = asyncio.Event()
        reconnect_count = 0
        deadline = None if duration_s is None else asyncio.get_running_loop().time() + float(duration_s)
        initial_task = task.strip()
        first_connection = True
        clean = False
        self._emit(
            "ER2_STREAM_START",
            model=self.config.streaming_model,
            task_chars=len(initial_task),
            bounded_duration_s=duration_s,
        )
        try:
            while not stop_event.is_set():
                if deadline is not None and asyncio.get_running_loop().time() >= deadline:
                    clean = True
                    break
                resumed = bool(self._resume_handle)
                self._emit(
                    "ER2_STREAM_CONNECT_ATTEMPT",
                    model=self.config.streaming_model,
                    resumed=resumed,
                    reconnect_count=reconnect_count,
                )
                config_kwargs: dict[str, object] = {
                    "response_modalities": ["TEXT"],
                    "tools": self.tools.live_tools(),
                    "system_instruction": types.Content(
                        parts=[types.Part(text=_system_instruction(self.config))]
                    ),
                    "context_window_compression": types.ContextWindowCompressionConfig(
                        sliding_window=types.SlidingWindow()
                    ),
                    "session_resumption": (
                        types.SessionResumptionConfig(handle=self._resume_handle)
                        if self._resume_handle
                        else types.SessionResumptionConfig()
                    ),
                }
                live_config = types.LiveConnectConfig(**config_kwargs)
                reconnect_requested = asyncio.Event()
                turn_done = asyncio.Event()
                connection_was_initial = first_connection
                try:
                    async with client.aio.live.connect(
                        model=self.config.streaming_model,
                        config=live_config,
                    ) as session:
                        self._emit(
                            "ER2_STREAM_CONNECTED",
                            model=self.config.streaming_model,
                            resumed=resumed,
                            reconnect_count=reconnect_count,
                        )
                        recv_task = asyncio.create_task(
                            self._receive_loop(session, types, turn_done, reconnect_requested, stop_event)
                        )
                        heartbeat_task: asyncio.Task | None = None
                        timer_task: asyncio.Task | None = None
                        try:
                            if connection_was_initial:
                                frame = await self.media.latest_jpeg(stream_name="lores")
                                await session.send_client_content(
                                    turns=types.Content(
                                        role="user",
                                        parts=[
                                            types.Part(inline_data=types.Blob(data=frame, mime_type="image/jpeg")),
                                            types.Part(text=initial_task),
                                        ],
                                    ),
                                    turn_complete=True,
                                )
                                first_connection = False
                            # First connection waits for the initial user turn to
                            # complete. A resumed connection has no new initial turn,
                            # so its first heartbeat must be allowed immediately.
                            heartbeat_task = asyncio.create_task(
                                self._heartbeat_loop(
                                    session,
                                    types,
                                    turn_done,
                                    stop_event,
                                    wait_for_initial_turn=connection_was_initial,
                                )
                            )
                            if deadline is not None:
                                timer_task = asyncio.create_task(
                                    asyncio.sleep(max(0.0, deadline - asyncio.get_running_loop().time()))
                                )
                            waiters: set[asyncio.Task] = {recv_task, heartbeat_task}
                            if timer_task is not None:
                                waiters.add(timer_task)
                            done, _ = await asyncio.wait(waiters, return_when=asyncio.FIRST_COMPLETED)
                            if timer_task is not None and timer_task in done:
                                clean = True
                                stop_event.set()
                            if reconnect_requested.is_set():
                                reconnect_count += 1
                                self._emit("ER2_STREAM_RECONNECT", reason="GO_AWAY", reconnect_count=reconnect_count)
                            elif recv_task in done and not stop_event.is_set():
                                reconnect_count += 1
                                self._emit("ER2_STREAM_RECONNECT", reason="RECEIVE_ENDED", reconnect_count=reconnect_count)
                            elif heartbeat_task in done and heartbeat_task.exception() is not None:
                                raise heartbeat_task.exception()  # type: ignore[misc]
                        finally:
                            tasks = [t for t in (heartbeat_task, timer_task, recv_task) if t is not None]
                            for task_obj in tasks:
                                if not task_obj.done():
                                    task_obj.cancel()
                            await asyncio.gather(*tasks, return_exceptions=True)
                            self._emit(
                                "ER2_STREAM_DISCONNECTED",
                                resumed=resumed,
                                reconnect_count=reconnect_count,
                                reconnect_requested=reconnect_requested.is_set(),
                            )
                except asyncio.CancelledError:
                    raise
                except Exception as exc:
                    if stop_event.is_set():
                        break
                    reconnect_count += 1
                    self._emit(
                        "ER2_STREAM_RECONNECT",
                        reason="EXCEPTION",
                        reconnect_count=reconnect_count,
                        error_type=type(exc).__name__,
                        error=str(exc)[:300],
                    )
                    if reconnect_count > 5:
                        raise Er2StreamingError(f"ER2 Live reconnect budget exhausted: {exc}") from exc
                    await asyncio.sleep(min(2.0, 0.25 * reconnect_count))
                    continue

                if stop_event.is_set():
                    break
                if reconnect_count > 5:
                    raise Er2StreamingError("ER2 Live reconnect budget exhausted")
                await asyncio.sleep(0.10)
        except Exception as exc:
            self._emit(
                "ER2_STREAM_ERROR",
                model=self.config.streaming_model,
                reconnect_count=reconnect_count,
                error_type=type(exc).__name__,
                error=str(exc)[:300],
            )
            raise
        finally:
            try:
                await asyncio.to_thread(self.tools.robot_stop)
            except Exception:
                pass
        result = Er2StreamingResult(reconnect_count, self._resume_handle, clean)
        self._emit(
            "ER2_STREAM_COMPLETE",
            model=self.config.streaming_model,
            reconnect_count=result.reconnect_count,
            resumable=bool(result.latest_resumption_handle),
            stopped_cleanly=result.stopped_cleanly,
        )
        return result

    def run(self, task: str, *, duration_s: float | None = None) -> Er2StreamingResult:
        return asyncio.run(self.run_async(task, duration_s=duration_s))

    async def _receive_loop(self, session, types, turn_done: asyncio.Event, reconnect_requested: asyncio.Event, stop_event: asyncio.Event) -> None:
        async for message in session.receive():
            update = getattr(message, "session_resumption_update", None)
            if update is not None and getattr(update, "resumable", False):
                handle = getattr(update, "new_handle", None)
                if isinstance(handle, str) and handle:
                    self._resume_handle = handle
                    self._emit("ER2_STREAM_RESUMABLE", has_handle=True)

            go_away = getattr(message, "go_away", None)
            if go_away is not None:
                reconnect_requested.set()
                self._emit(
                    "ER2_STREAM_GOAWAY",
                    time_left=str(getattr(go_away, "time_left", ""))[:120],
                    resumable=bool(self._resume_handle),
                )

            server_content = getattr(message, "server_content", None)
            if server_content is not None:
                model_turn = getattr(server_content, "model_turn", None)
                for part in tuple(getattr(model_turn, "parts", ()) or ()):
                    text = getattr(part, "text", None)
                    if isinstance(text, str) and text:
                        self._on_text(text)
                if getattr(server_content, "turn_complete", False):
                    turn_done.set()

            tool_call = getattr(message, "tool_call", None)
            if tool_call is not None:
                responses = []
                for call in tuple(getattr(tool_call, "function_calls", ()) or ()):
                    name = getattr(call, "name", None)
                    args = getattr(call, "args", {}) or {}
                    call_id = getattr(call, "id", None)
                    if not isinstance(name, str) or not isinstance(args, Mapping):
                        result = {"status": "ERROR", "error": "malformed ER2 tool call"}
                    else:
                        try:
                            result = await asyncio.to_thread(self.tools.execute, name, args)
                        except Exception as exc:
                            result = {"status": "ERROR", "error": f"{type(exc).__name__}: {exc}"}
                    responses.append(types.FunctionResponse(name=name or "invalid", response=result, id=call_id))
                if responses:
                    await session.send_tool_response(function_responses=responses)
            if reconnect_requested.is_set() or stop_event.is_set():
                return

    async def _heartbeat_loop(
        self,
        session,
        types,
        turn_done: asyncio.Event,
        stop_event: asyncio.Event,
        *,
        wait_for_initial_turn: bool = True,
    ) -> None:
        # The official robotics heartbeat sends one frame+text prompt, then waits
        # for that model turn to complete before sending the next heartbeat. On
        # the first connection we additionally wait for the initial task turn;
        # on a resumed WebSocket there is no new initial turn, so we send at once.
        if wait_for_initial_turn:
            await turn_done.wait()
            turn_done.clear()
        while not stop_event.is_set():
            started = asyncio.get_running_loop().time()
            frame = await self.media.latest_jpeg(stream_name="lores")
            await session.send_realtime_input(video=types.Blob(data=frame, mime_type="image/jpeg"))
            status = await asyncio.to_thread(self.tools.robot_status)
            await session.send_realtime_input(
                text=(
                    "[R2B4 HEARTBEAT] Observe the latest frame and robot status. "
                    "Continue the active task only through declared blocking tools. "
                    "If no movement is needed, do not invent one. Robot status: "
                    + _compact(status)
                )
            )
            await turn_done.wait()
            turn_done.clear()
            remaining = self.config.heartbeat_s - (asyncio.get_running_loop().time() - started)
            if remaining > 0:
                await asyncio.sleep(remaining)


def _compact(value: object, limit: int = 2500) -> str:
    import json
    text = json.dumps(value, ensure_ascii=False, separators=(",", ":"), default=str)
    return text if len(text) <= limit else text[:limit] + "..."


def _system_instruction(config: Er2Config) -> str:
    return (
        "You are the embodied high-level controller for the R2B4 differential-drive robot. "
        "The local R2B4 safety/control stack is authoritative. Never assume a motion succeeded: "
        "use tool results and fresh observations. Physical motion is available only through "
        "robot_drive, which is bounded to short segments and blocks until STOP. For longer travel, "
        "use multiple short robot_drive segments and inspect fresh robot_status between them. "
        f"Limits: |v| <= {config.max_v_mps:g} m/s, |omega| <= {config.max_omega_rad_s:g} rad/s, "
        f"segment <= {config.max_segment_s:g} s. If uncertain, call robot_stop or robot_status. "
        "Do not request direct motor, GPIO, L12, safety-limit or configuration access."
    )


__all__ = ["Er2StreamingClient", "Er2StreamingError", "Er2StreamingResult"]
