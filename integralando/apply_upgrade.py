#!/usr/bin/env python3
"""Transactional R2B4 HRI P0 upgrade installer.

Design:
- source-first preflight against known Git blob identities;
- no partial editing and no long fragile replace_once blocks;
- all transformations are built and syntax-checked in memory first;
- persistent repo-local backup before any write;
- atomic same-directory replacement;
- targeted regression tests after install;
- automatic rollback on write/test failure.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

BASE_COMMIT = "04b5cb704b1f532028c49058f4fc416a1c5deb9d"
UPGRADE_ID = "r2b4_hri_p0_20260923_v1"
MARKER = "R2B4_HRI_P0_V1"

EXPECTED_BLOBS = {
    "r2b4_voice/action_executor.py": "41bf9d0f420e7a1ef39e1578c56aeab9da5e0d0a",
    "r2b4_voice/voice_service.py": "30ea393905b6270660609dc637d084bc18b0ab72",
    "v3/process_sidecars.py": "2c84fea69c9f151d13ef480fe9f31d04226fabba",
    "v3/mcap_capture.py": "85bc246806e542556f4318975bcfd551e44f5ccf",
    "v3/test_hub_next.py": "32489500690dc2fa836eeb25027fbd7d378eba2f",
}

NEW_FILES = (
    "v3/hri_evidence.py",
    "tests/test_voice_hri_p0.py",
    "tests/test_v3_hri_evidence.py",
)

PAYLOAD_SHA256 = {
    "r2b4_voice/action_executor.py": "354c0692f6e6b901cd8862b5516de57bac703c90cff63d54485c59050827ac03",
    "v3/hri_evidence.py": "bb7a8430ed8b76f19801c8285eb542a45d8ef7933379259e0b3f35e958aec64b",
    "tests/test_voice_hri_p0.py": "941a14cfc5a22e34a69e63d75c8d1b76ebada5c0dd11d701ce2a5958819ee5a8",
    "tests/test_v3_hri_evidence.py": "2787dd1cbfcd6d8199a9d2b45f47aa9d5441f11a2209b33210b58af2aea21f16",
}

TARGETED_TESTS = (
    "tests/test_voice_action_executor.py",
    "tests/test_voice_safety_intents.py",
    "tests/test_r2b4_voice_conversation_service.py",
    "tests/test_voice_hri_p0.py",
    "tests/test_v3_hri_evidence.py",
    "tests/test_v3_capture_refactor_p0.py",
    "tests/test_v3_test_hub_behavior.py",
)


class UpgradeError(RuntimeError):
    pass


def _root_from_args(value: str | None) -> Path:
    if value:
        root = Path(value).expanduser().resolve()
    elif os.environ.get("R2B4_ROOT"):
        root = Path(os.environ["R2B4_ROOT"]).expanduser().resolve()
    else:
        cwd = Path.cwd().resolve()
        root = cwd if (cwd / "v3").is_dir() else Path("/home/alba/project_r2b4")
    if not (root / "v3").is_dir() or not (root / "r2b4_voice").is_dir():
        raise UpgradeError(f"not an R2B4 repo root: {root}")
    return root


def _verify_payload(package: Path) -> None:
    problems: list[str] = []
    for relative, expected in PAYLOAD_SHA256.items():
        path = package / "payload" / relative
        if not path.is_file():
            problems.append(f"{relative}: missing")
            continue
        actual = hashlib.sha256(path.read_bytes()).hexdigest()
        if actual != expected:
            problems.append(f"{relative}: expected sha256 {expected}, got {actual}")
    if problems:
        raise UpgradeError(
            "package payload integrity failed before repo access\n  "
            + "\n  ".join(problems)
        )

def _git_blob(root: Path, relative: str) -> str:
    completed = subprocess.run(
        ["git", "hash-object", relative], cwd=root, text=True, capture_output=True, check=False
    )
    if completed.returncode != 0:
        raise UpgradeError(f"git hash-object failed for {relative}: {completed.stderr.strip()}")
    return completed.stdout.strip()


def _replace(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise UpgradeError(f"semantic anchor {label!r} expected once, found {count}")
    return text.replace(old, new, 1)


def _insert_after(text: str, anchor: str, addition: str, label: str) -> str:
    return _replace(text, anchor, anchor + addition, label)


def _voice_service(source: str) -> str:
    if MARKER in source:
        return source
    text = source
    text = _insert_after(
        text,
        "import os\n",
        "import queue\n",
        "voice queue import",
    )
    text = _insert_after(
        text,
        "from v3.adapters.microphone import MicrophoneHealth, MicrophoneState, NativeUsbMicrophone\n",
        "from v3.hri_evidence import HriBehaviorObserver, HriEventJournal\n",
        "voice hri import",
    )
    text = _insert_after(
        text,
        "from .wake_core import EnergyUtteranceBuilder, WakePhraseMatcher, WakeVoiceActivityConfig\n",
        f"\n# {MARKER}\n",
        "voice marker",
    )
    text = _replace(
        text,
        "for wake and conversation; microphone frames are deliberately not consumed while\nTHINKING/SPEAKING and the capture tail is discarded before returning to LISTENING\nso Alba does not transcribe its own voice.\n",
        "for wake and conversation. Normal dialogue remains half-duplex; during\nTHINKING/SPEAKING only a separate exact-STOP interrupt lane may read the same\nbounded microphone frame port. The acoustic tail is discarded before normal listening.\n",
        "voice architecture docstring",
    )
    text = _replace(
        text,
        "        activity_config: WakeVoiceActivityConfig = WakeVoiceActivityConfig(),\n        status_file: Path | str | None = None,\n",
        "        activity_config: WakeVoiceActivityConfig = WakeVoiceActivityConfig(),\n        hri_journal: HriEventJournal | None = None,\n        status_file: Path | str | None = None,\n",
        "voice init hri parameter",
    )
    text = _insert_after(
        text,
        "        self._builder = EnergyUtteranceBuilder(activity_config)\n",
        "        self._activity_config = activity_config\n"
        "        self._hri_journal = hri_journal\n"
        "        self._behavior_feedback_queue = queue.Queue(maxsize=4)\n"
        "        self._behavior_observer = HriBehaviorObserver(\n"
        "            conversation_interface, hri_journal, feedback_sink=self._queue_behavior_feedback,\n"
        "        )\n"
        "        self._interaction_counter = 0\n"
        "        self._interrupt_enabled = threading.Event()\n"
        "        self._stop_interrupt_latched = threading.Event()\n"
        "        self._interrupt_thread = threading.Thread(\n"
        "            target=self._interrupt_stop_loop,\n"
        "            name=\"r2b4-voice-stop-interrupt\",\n"
        "            daemon=True,\n"
        "        )\n",
        "voice init hri state",
    )
    text = _replace(
        text,
        "    def run_forever(self) -> int:\n        self._publish_status()\n        try:\n",
        "    def run_forever(self) -> int:\n        self._publish_status()\n        if not self._interrupt_thread.is_alive():\n            self._interrupt_thread.start()\n        try:\n",
        "voice interrupt thread start",
    )
    text = _insert_after(
        text,
        "            while not self._stop_event.is_set():\n",
        "                if self._deliver_behavior_feedback():\n                    continue\n",
        "voice behavior feedback drain",
    )
    text = _replace(
        text,
        "        finally:\n            try:\n                self._microphone.stop()\n",
        "        finally:\n            self._interrupt_enabled.clear()\n            self._behavior_observer.close()\n            self._stop_event.set()\n            if self._interrupt_thread.is_alive():\n                self._interrupt_thread.join(timeout=1.0)\n            try:\n                self._microphone.stop()\n",
        "voice interrupt thread stop",
    )
    text = _replace(
        text,
        "        # P0.1: deterministic STOP bypasses the network/LLM completely.\n        # It still enters only through the canonical RobotInterface command path.\n",
        "        # Deterministic STOP bypasses the LLM/action-proposal path after STT.\n        # It still enters only through the canonical RobotInterface command path.\n",
        "voice truthful stop comment",
    )
    text = _insert_after(
        text,
        "        print(f\"voice: user={self._last_transcript!r}\", flush=True)\n",
        "        interaction_id = self._next_interaction_id()\n"
        "        self._stop_interrupt_latched.clear()\n"
        "        self._hri_event(\n"
        "            \"STT_RESULT\", interaction_id=interaction_id, text=self._last_transcript,\n"
        "            phase=self._state.value,\n"
        "        )\n",
        "voice stt evidence",
    )
    old_fast = '''        if is_stop_intent(self._last_transcript):\n            try:\n                self._conversation_interface.execute("v3.command.stop")\n                self._last_action_status = "EXECUTED:VOICE_STOP_FAST_PATH"\n                print("voice: STOP fast-path executed via RobotInterface", flush=True)\n                self._last_error = None\n            except Exception as exc:\n                self._last_action_status = "STOP_FAILED"\n                self._last_error = f"voice STOP {type(exc).__name__}: {exc}"\n                print(f"voice: {self._last_error}", file=sys.stderr, flush=True)\n            self._settle_and_discard()\n            return\n'''
    new_fast = '''        if is_stop_intent(self._last_transcript):\n            self._execute_voice_stop(\n                interaction_id=interaction_id, source="FAST_PATH", transcript=self._last_transcript\n            )\n            self._settle_and_discard()\n            return\n'''
    text = _replace(text, old_fast, new_fast, "voice fast stop helper")
    old_wait = '''            if not isinstance(accepted, Mapping) or not isinstance(accepted.get("turn_id"), str):\n                raise RuntimeError("conversation.submit_text did not return a turn_id")\n            result = self._conversation.wait_for_turn(\n                accepted["turn_id"],\n                timeout_s=self._config.llm_timeout_s,\n            )\n            if result is None:\n                raise TimeoutError("conversation turn timed out")\n'''
    new_wait = '''            if not isinstance(accepted, Mapping) or not isinstance(accepted.get("turn_id"), str):\n                raise RuntimeError("conversation.submit_text did not return a turn_id")\n            turn_id = accepted["turn_id"]\n            self._hri_event(\n                "TURN_ACCEPTED", interaction_id=interaction_id, turn_id=turn_id,\n                text=self._last_transcript, phase=self._state.value,\n            )\n            self._interrupt_enabled.set()\n            try:\n                result = self._conversation.wait_for_turn(\n                    turn_id, timeout_s=self._config.llm_timeout_s,\n                )\n            finally:\n                self._interrupt_enabled.clear()\n            if result is None:\n                raise TimeoutError("conversation turn timed out")\n'''
    text = _replace(text, old_wait, new_wait, "voice thinking interrupt lane")
    text = _insert_after(
        text,
        "        self._last_action_status = str(result.get(\"action_status\")) if result.get(\"action_status\") is not None else None\n",
        "        defer_action_feedback = False\n",
        "voice deferred behavior feedback flag",
    )
    text = _insert_after(
        text,
        "        proposed = result.get(\"proposed_action\")\n        if proposed is not None:\n",
        "            self._hri_event(\n"
        "                \"INTENT_PROPOSED\", interaction_id=interaction_id, turn_id=turn_id,\n"
        "                proposal=proposed, phase=self._state.value,\n"
        "            )\n"
        "            if self._stop_interrupt_latched.is_set():\n"
        "                self._last_action_status = \"REJECTED:INTERRUPTED_BY_STOP\"\n"
        "                self._hri_event(\n"
        "                    \"ACTION_REJECTED\", interaction_id=interaction_id, turn_id=turn_id,\n"
        "                    action_status=self._last_action_status, reason=\"INTERRUPTED_BY_STOP\",\n"
        "                )\n"
        "                proposed = None\n"
        "        if proposed is not None:\n",
        "voice intent evidence and stop latch",
    )
    # The prior insertion intentionally introduces a second if; remove the now-empty
    # first branch's duplicated mode line indentation by replacing the exact block.
    text = _replace(
        text,
        '''        if proposed is not None:\n            self._hri_event(\n                "INTENT_PROPOSED", interaction_id=interaction_id, turn_id=turn_id,\n                proposal=proposed, phase=self._state.value,\n            )\n            if self._stop_interrupt_latched.is_set():\n                self._last_action_status = "REJECTED:INTERRUPTED_BY_STOP"\n                self._hri_event(\n                    "ACTION_REJECTED", interaction_id=interaction_id, turn_id=turn_id,\n                    action_status=self._last_action_status, reason="INTERRUPTED_BY_STOP",\n                )\n                proposed = None\n        if proposed is not None:\n            mode = self._action_executor.mode.upper() if self._action_executor is not None else "PROPOSAL_ONLY"\n''',
        '''        if proposed is not None:\n            self._hri_event(\n                "INTENT_PROPOSED", interaction_id=interaction_id, turn_id=turn_id,\n                proposal=proposed, phase=self._state.value,\n            )\n            if self._stop_interrupt_latched.is_set():\n                self._last_action_status = "REJECTED:INTERRUPTED_BY_STOP"\n                self._hri_event(\n                    "ACTION_REJECTED", interaction_id=interaction_id, turn_id=turn_id,\n                    action_status=self._last_action_status, reason="INTERRUPTED_BY_STOP",\n                )\n                proposed = None\n        if proposed is not None:\n            mode = self._action_executor.mode.upper() if self._action_executor is not None else "PROPOSAL_ONLY"\n''',
        "voice normalize intent block",
    )
    text = _insert_after(
        text,
        "                    self._last_action_status = execution.status\n",
        "                    self._hri_event(\n"
        "                        \"ACTION_EXECUTED\" if execution.executed else \"ACTION_REJECTED\",\n"
        "                        interaction_id=interaction_id, turn_id=turn_id,\n"
        "                        action_name=execution.action_name, action_status=execution.status,\n"
        "                        command_id=execution.command_id, mission_id=execution.mission_id,\n"
        "                    )\n"
        "                    if (\n"
        "                        execution.executed and execution.action_name != \"v3.command.stop\"\n"
        "                        and execution.command_id and execution.mission_id\n"
        "                    ):\n"
        "                        defer_action_feedback = True\n"
        "                        self._behavior_observer.observe(\n"
        "                            interaction_id=interaction_id, turn_id=turn_id,\n"
        "                            session_id=self._conversation.session_id,\n"
        "                            action_name=execution.action_name, command_id=execution.command_id,\n"
        "                            mission_id=execution.mission_id,\n"
        "                        )\n",
        "voice action receipt evidence",
    )
    text = _replace(
        text,
        "                        f\"voice: action={execution.action_name} status={execution.status}\",\n",
        "                        f\"voice: action={execution.action_name} status={execution.status} command_id={execution.command_id} mission_id={execution.mission_id}\",\n",
        "voice action receipt log",
    )
    text = _insert_after(
        text,
        "                    self._last_error = f\"voice action {type(exc).__name__}: {exc}\"\n",
        "                    self._hri_event(\n"
        "                        \"ACTION_REJECTED\", interaction_id=interaction_id, turn_id=turn_id,\n"
        "                        action_status=self._last_action_status, reason=self._last_error,\n"
        "                    )\n",
        "voice executor error evidence",
    )
    text = _replace(
        text,
        "        spoken = result.get(\"spoken_text\")\n",
        "        if self._stop_interrupt_latched.is_set():\n"
        "            spoken = \"Megálltam.\"\n"
        "        elif defer_action_feedback:\n"
        "            spoken = \"Rendben.\"\n"
        "        else:\n"
        "            spoken = result.get(\"spoken_text\")\n",
        "voice truthful post-action speech",
    )
    text = _insert_after(
        text,
        "        self._last_spoken_text = spoken.strip()\n",
        "        self._hri_event(\n"
        "            \"HRI_RESPONSE\", interaction_id=interaction_id, turn_id=turn_id,\n"
        "            text=self._last_spoken_text, action_status=self._last_action_status,\n"
        "        )\n",
        "voice response evidence",
    )
    text = _replace(
        text,
        "        self._set_state(VoiceServiceState.SPEAKING)\n        try:\n            speech = self._tts.synthesize(self._last_spoken_text)\n",
        "        self._set_state(VoiceServiceState.SPEAKING)\n        self._interrupt_enabled.set()\n        try:\n            speech = self._tts.synthesize(self._last_spoken_text)\n",
        "voice speaking interrupt enable",
    )
    text = _replace(
        text,
        "        finally:\n            # Half-duplex P0: everything captured during THINKING/SPEAKING plus\n",
        "        finally:\n            self._interrupt_enabled.clear()\n            # Half-duplex remains the normal dialogue path; the separate exact STOP\n            # lane is the only microphone consumer allowed during THINKING/SPEAKING.\n",
        "voice speaking interrupt disable",
    )
    helper = r'''
    def _queue_behavior_feedback(self, text: str, fields: Mapping[str, object]) -> None:
        item = (str(text), dict(fields))
        try:
            self._behavior_feedback_queue.put_nowait(item)
        except queue.Full:
            try:
                self._behavior_feedback_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._behavior_feedback_queue.put_nowait(item)
            except queue.Full:
                pass

    def _deliver_behavior_feedback(self) -> bool:
        try:
            text, fields = self._behavior_feedback_queue.get_nowait()
        except queue.Empty:
            return False
        self._last_spoken_text = text
        self._hri_event("BEHAVIOR_FEEDBACK_SPEAKING", text=text, **fields)
        self._set_state(VoiceServiceState.SPEAKING)
        self._interrupt_enabled.set()
        try:
            speech = self._tts.synthesize(text)
            backend = self._playback.play(speech)
            print(f"voice: behavior_feedback={text!r}; player={backend}", flush=True)
            self._last_error = None
        except Exception as exc:
            self._last_error = f"behavior feedback {type(exc).__name__}: {exc}"
            self._hri_event("BEHAVIOR_FEEDBACK_FAILED", text=text, reason=self._last_error, **fields)
            print(f"voice: {self._last_error}", file=sys.stderr, flush=True)
        finally:
            self._interrupt_enabled.clear()
            self._settle_and_discard()
        return True

    def _next_interaction_id(self) -> str:
        self._interaction_counter += 1
        return f"voice-{os.getpid()}-{self._monotonic_ns()}-{self._interaction_counter}"

    def _hri_event(self, event_type: str, **fields: object) -> None:
        if self._hri_journal is None:
            return
        try:
            self._hri_journal.append(
                event_type,
                session_id=self._conversation.session_id,
                **fields,
            )
        except Exception:
            # HRI evidence is passive; it must never perturb dialogue or control.
            return

    def _execute_voice_stop(
        self, *, interaction_id: str, source: str, transcript: str | None = None
    ) -> bool:
        if source == "INTERRUPT":
            self._stop_interrupt_latched.set()
        self._hri_event(
            "STOP_REQUESTED",
            interaction_id=interaction_id,
            source=source,
            transcript=transcript,
            phase=self._state.value,
        )
        try:
            self._conversation_interface.execute("v3.command.stop")
            status = (
                "EXECUTED:VOICE_STOP_INTERRUPT"
                if source == "INTERRUPT"
                else "EXECUTED:VOICE_STOP_FAST_PATH"
            )
            self._last_action_status = status
            self._last_error = None
            self._hri_event(
                "INTERRUPT_STOP_EXECUTED" if source == "INTERRUPT" else "STOP_EXECUTED",
                interaction_id=interaction_id,
                source=source,
                action_name="v3.command.stop",
                action_status=status,
                phase=self._state.value,
            )
            print(f"voice: STOP {source.lower()} executed via RobotInterface", flush=True)
            return True
        except Exception as exc:
            self._last_action_status = "STOP_FAILED"
            self._last_error = f"voice STOP {type(exc).__name__}: {exc}"
            self._hri_event(
                "INTERRUPT_STOP_FAILED" if source == "INTERRUPT" else "STOP_FAILED",
                interaction_id=interaction_id,
                source=source,
                action_name="v3.command.stop",
                action_status=self._last_action_status,
                reason=self._last_error,
                phase=self._state.value,
            )
            print(f"voice: {self._last_error}", file=sys.stderr, flush=True)
            return False

    def _handle_interrupt_transcript(self, transcript: str, *, phase: str) -> bool:
        normalized = transcript.strip()
        if not normalized or not is_stop_intent(normalized):
            return False
        interaction_id = self._next_interaction_id()
        self._hri_event(
            "INTERRUPT_STOP_DETECTED",
            interaction_id=interaction_id,
            transcript=normalized,
            phase=phase,
        )
        return self._execute_voice_stop(
            interaction_id=interaction_id,
            source="INTERRUPT",
            transcript=normalized,
        )

    def _interrupt_stop_loop(self) -> None:
        builder = EnergyUtteranceBuilder(self._activity_config)
        sequence = 0
        lane_active = False
        while not self._stop_event.is_set():
            if not self._interrupt_enabled.is_set():
                lane_active = False
                self._stop_event.wait(0.05)
                continue
            health = self._microphone.health()
            if health.state is not MicrophoneState.CAPTURING:
                self._stop_event.wait(0.05)
                continue
            if not lane_active:
                sequence = health.sequence
                builder.reset(reset_sequence=True)
                lane_active = True
                # Let transient THINKING/SPEAKING phases collapse without touching
                # the audio ring; real speech remains buffered behind this cursor.
                if self._stop_event.wait(0.02):
                    continue
                if not self._interrupt_enabled.is_set():
                    continue
            frame = self._microphone.port.read_after(sequence, timeout_s=0.10)
            if frame is None:
                continue
            sequence = frame.sequence
            try:
                utterance = builder.feed(frame)
            except Exception:
                builder.reset(reset_sequence=True)
                sequence = self._microphone.health().sequence
                continue
            if utterance is None:
                continue
            try:
                transcript = self._transcriber.transcribe(utterance).strip()
            except Exception as exc:
                self._hri_event(
                    "INTERRUPT_STT_ERROR",
                    phase=self._state.value,
                    reason=f"{type(exc).__name__}: {exc}",
                )
                continue
            if self._handle_interrupt_transcript(transcript, phase=self._state.value):
                builder.reset(reset_sequence=True)
                sequence = self._microphone.health().sequence

'''
    text = _replace(text, "    def _read_utterance(self):\n", helper + "    def _read_utterance(self):\n", "voice hri helpers")
    text = _replace(
        text,
        "            action_executor=VoiceActionExecutor(\n",
        "            hri_journal=HriEventJournal((runtime / \"hri_events.ndjson\").resolve()),\n            action_executor=VoiceActionExecutor(\n",
        "voice main journal wiring",
    )
    return text


def _process_sidecars(source: str) -> str:
    if MARKER in source:
        return source
    text = source
    text = _insert_after(
        text,
        "from v3.capture_ipc import CaptureCoreExpander\n",
        "from v3.hri_evidence import HRI_EVENT_TOPIC, load_hri_events_for_capture\n",
        "sidecar hri import",
    )
    text = _insert_after(
        text,
        "_SIDECAR_FINISH_TIMEOUT_S = 120.0\n",
        f"\n# {MARKER}\n",
        "sidecar marker",
    )
    text = _replace(
        text,
        '''            topics=(\n                "v3.capture_record", "v3.raw_lidar",\n                "v3.raw_lidar_transport", "v3.capture_transport",\n            ),\n''',
        '''            topics=(\n                "v3.capture_record", "v3.raw_lidar",\n                "v3.raw_lidar_transport", "v3.capture_transport", HRI_EVENT_TOPIC,\n            ),\n''',
        "sidecar hri subscription",
    )
    text = _insert_after(
        text,
        "        consumer.start()\n",
        "        capture_started_ns = time.monotonic_ns()\n",
        "sidecar capture start timestamp",
    )
    text = _replace(
        text,
        '''                hub.close()\n                result = consumer.finish(finish_request[0], terminal=finish_request[1])\n''',
        '''                for hri_event in load_hri_events_for_capture(\n                    Path(project_root), capture_started_ns, time.monotonic_ns()\n                ):\n                    hub.publish(hri_event, topic=HRI_EVENT_TOPIC)\n                hub.close()\n                result = consumer.finish(finish_request[0], terminal=finish_request[1])\n''',
        "sidecar hri import before close",
    )
    return text


def _mcap_capture(source: str) -> str:
    if MARKER in source:
        return source
    text = source
    text = _insert_after(
        text,
        "from .capture_ipc import IPC_CHECKPOINT_KEY, IPC_TRIGGER_REASON_KEY\n",
        "from .hri_evidence import HRI_EVENT_TOPIC\n",
        "mcap hri import",
    )
    text = _insert_after(
        text,
        "RUNTIME_TOPIC = \"/r2b4/runtime\"\n",
        f"\n# {MARKER}\n",
        "mcap marker",
    )
    text = _replace(
        text,
        '''                if record.monotonic_ns <= upper:\n                    self._write_encoded_record(record)\n''',
        '''                if record.source_topic == HRI_EVENT_TOPIC or record.monotonic_ns <= upper:\n                    # HRI rows are imported by the passive sidecar at finalize. Their\n                    # original monotonic time remains inside the payload; allow the\n                    # late evidence message itself to be appended after post-window.\n                    self._write_encoded_record(record)\n''',
        "mcap late hri evidence",
    )
    return text


def _test_hub_next(source: str) -> str:
    if MARKER in source:
        return source
    text = source
    text = _insert_after(
        text,
        "from .test_hub_behavior import build_behavior_evidence\n",
        "from .hri_evidence import build_hri_evidence\n",
        "testhub hri import",
    )
    text = _insert_after(
        text,
        "DEFAULT_HZ = 10\n",
        f"\n# {MARKER}\n",
        "testhub marker",
    )
    text = _insert_after(
        text,
        "    behavior = build_behavior_evidence(reader, destination, triage=triage)\n",
        "    hri = build_hri_evidence(\n"
        "        reader, destination,\n"
        "        behavior_timeline_path=destination / \"behavior_timeline.ndjson\",\n"
        "    )\n",
        "testhub hri build",
    )
    text = _insert_after(
        text,
        "        \"behavior\": behavior,\n",
        "        \"hri\": hri,\n        \"hri_timeline\": \"hri_timeline.ndjson\",\n",
        "testhub agent hri",
    )
    text = _insert_after(
        text,
        "        \"motion_tuning_segment_count\": motion_tuning.get(\"segment_count\"),\n",
        "        \"hri_event_count\": hri.get(\"event_count\"),\n"
        "        \"hri_correlated_event_count\": hri.get(\"correlated_event_count\"),\n",
        "testhub result hri",
    )
    return text


TRANSFORMS = {
    "r2b4_voice/voice_service.py": _voice_service,
    "v3/process_sidecars.py": _process_sidecars,
    "v3/mcap_capture.py": _mcap_capture,
    "v3/test_hub_next.py": _test_hub_next,
}


def _atomic_write(path: Path, data: bytes, mode: int | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        if mode is not None:
            os.fchmod(fd, mode)
        with os.fdopen(fd, "wb") as handle:
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass


def _stage(root: Path, package: Path) -> dict[str, bytes]:
    staged: dict[str, bytes] = {}
    mismatches: list[str] = []
    for relative, expected in EXPECTED_BLOBS.items():
        path = root / relative
        if not path.is_file():
            mismatches.append(f"{relative}: missing")
            continue
        current = path.read_text(encoding="utf-8")
        if MARKER in current:
            continue
        actual = _git_blob(root, relative)
        if actual != expected:
            mismatches.append(f"{relative}: expected blob {expected}, got {actual}")
    if mismatches:
        raise UpgradeError(
            "source preflight failed before any write; package targets base "
            + BASE_COMMIT
            + "\n  "
            + "\n  ".join(mismatches)
        )

    action_payload = package / "payload" / "r2b4_voice" / "action_executor.py"
    action_data = action_payload.read_bytes()
    try:
        ast.parse(action_data.decode("utf-8"), filename="r2b4_voice/action_executor.py")
    except SyntaxError as exc:
        raise UpgradeError(f"payload syntax failure in r2b4_voice/action_executor.py: {exc}") from exc
    staged["r2b4_voice/action_executor.py"] = action_data

    for relative, transform in TRANSFORMS.items():
        source = (root / relative).read_text(encoding="utf-8")
        transformed = transform(source)
        try:
            ast.parse(transformed, filename=relative)
        except SyntaxError as exc:
            raise UpgradeError(f"staged syntax failure in {relative}: {exc}") from exc
        staged[relative] = transformed.encode("utf-8")

    for relative in NEW_FILES:
        payload_path = package / "payload" / relative
        if not payload_path.is_file():
            raise UpgradeError(f"package payload missing: {relative}")
        data = payload_path.read_bytes()
        if relative.endswith(".py"):
            try:
                ast.parse(data.decode("utf-8"), filename=relative)
            except SyntaxError as exc:
                raise UpgradeError(f"payload syntax failure in {relative}: {exc}") from exc
        staged[relative] = data
    return staged


def _backup(root: Path, paths: set[str]) -> tuple[Path, dict[str, bool]]:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    backup = root / "runtime" / "upgrade_backups" / f"hri_p0_{stamp}"
    existed: dict[str, bool] = {}
    for relative in sorted(paths):
        src = root / relative
        existed[relative] = src.exists()
        if src.exists():
            dst = backup / relative
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)
    backup.mkdir(parents=True, exist_ok=True)
    (backup / "manifest.json").write_text(
        json.dumps(
            {
                "upgrade_id": UPGRADE_ID,
                "base_commit": BASE_COMMIT,
                "files": sorted(paths),
                "existed": existed,
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    return backup, existed


def _rollback(root: Path, backup: Path, existed: dict[str, bool]) -> None:
    for relative, did_exist in existed.items():
        target = root / relative
        saved = backup / relative
        if did_exist and saved.is_file():
            _atomic_write(target, saved.read_bytes(), saved.stat().st_mode & 0o777)
        elif not did_exist:
            try:
                target.unlink()
            except FileNotFoundError:
                pass


def _run_tests(root: Path) -> None:
    command = [sys.executable, "-m", "pytest", "-q", *TARGETED_TESTS]
    completed = subprocess.run(command, cwd=root, text=True)
    if completed.returncode != 0:
        raise UpgradeError(f"targeted regression failed with exit code {completed.returncode}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root")
    parser.add_argument("--no-tests", action="store_true")
    parser.add_argument("--check-only", action="store_true", help="preflight/stage/AST only; write nothing")
    args = parser.parse_args(argv)
    root = _root_from_args(args.root)
    package = Path(__file__).resolve().parent

    try:
        _verify_payload(package)
        staged = _stage(root, package)
        if args.check_only:
            print(f"PREFLIGHT PASS: {UPGRADE_ID}")
            print(f"base: {BASE_COMMIT}")
            print("no files written (--check-only)")
            for relative in sorted(staged):
                print(f"  {relative}")
            return 0
        backup, existed = _backup(root, set(staged))
        try:
            for relative, data in staged.items():
                target = root / relative
                mode = (target.stat().st_mode & 0o777) if target.exists() else 0o644
                _atomic_write(target, data, mode)
            if not args.no_tests:
                _run_tests(root)
        except BaseException:
            _rollback(root, backup, existed)
            raise
    except Exception as exc:
        print(f"UPGRADE FAILED: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1

    print(f"UPGRADE PASS: {UPGRADE_ID}")
    print(f"base: {BASE_COMMIT}")
    print(f"backup: {backup}")
    print("installed files:")
    for relative in sorted(staged):
        print(f"  {relative}")
    if args.no_tests:
        print("targeted tests: SKIPPED (--no-tests)")
    else:
        print("targeted tests: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
