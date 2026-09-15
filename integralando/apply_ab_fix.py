#!/usr/bin/env python3
"""Apply the post-live-test R2B4 encoder A/B robustness upgrade.

Targets the verified 2026-09-15 main revision. The installer is intentionally
fail-closed: it validates Git blob SHAs before changing production sources,
creates /tmp backups, and rolls back on syntax failure.
"""
from __future__ import annotations

import hashlib
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

KNOWN = {
    "v3/adapters/gpio_counter.py": "c3fefa616395cc08f81baaf440b9cb18417d357e",
    "v3/adapters/counter_encoder.py": "d67e0cbcafd65eb8d1cf2ec1bb70e0ab9245fc9f",
    "v3/adapters/live_encoder.py": "351b85c5fcaeca1cf27ed66c1384eb999b427e5a",
    "v3_bounded_config.py": "388fb318683272cdcff7d0f7a0bb11fe9218c67e",
    "conf/hardver.json": "73efc47f8c345facc895f9acf69d0b5efccc0d4a",
}


def git_blob_sha(data: bytes) -> str:
    return hashlib.sha1(f"blob {len(data)}\0".encode() + data).hexdigest()


def replace_once(text: str, old: str, new: str, label: str, already: str | None = None) -> str:
    if already and already in text:
        return text
    if old in text:
        if text.count(old) != 1:
            raise RuntimeError(f"{label}: old block occurs {text.count(old)} times")
        return text.replace(old, new, 1)
    raise RuntimeError(f"{label}: expected source block not found")


def patch_gpio(text: str) -> str:
    text = replace_once(
        text,
        '''    pull_up: bool = False\n    a_debounce_micros: int = 0\n''',
        '''    pull_up: bool = False\n    a_debounce_micros: int = 0\n    # Around an A physical edge, B must be stable for this interval.  The\n    # production config uses 50 us; zero preserves the generic X1 primitive.\n    direction_guard_micros: int = 0\n    # A genuine sign reversal must persist for this many A-rising candidates\n    # before it is committed to signed edge history.\n    direction_change_confirm_edges: int = 1\n    direction_change_confirm_window_micros: int = 250_000\n''',
        "gpio channel robust fields",
        "direction_change_confirm_edges: int = 1",
    )
    text = replace_once(
        text,
        '''        _bool(self.invert, "invert")\n        _bool(self.pull_up, "pull_up")\n        _nonnegative_int(self.a_debounce_micros, "a_debounce_micros")\n''',
        '''        _bool(self.invert, "invert")\n        _bool(self.pull_up, "pull_up")\n        _nonnegative_int(self.a_debounce_micros, "a_debounce_micros")\n        _nonnegative_int(self.direction_guard_micros, "direction_guard_micros")\n        confirm_edges = _nonnegative_int(\n            self.direction_change_confirm_edges,\n            "direction_change_confirm_edges",\n        )\n        if not 1 <= confirm_edges <= 8:\n            raise ValueError("direction_change_confirm_edges must be within [1, 8]")\n        confirm_window = _nonnegative_int(\n            self.direction_change_confirm_window_micros,\n            "direction_change_confirm_window_micros",\n        )\n        if confirm_window == 0:\n            raise ValueError("direction_change_confirm_window_micros must be positive")\n''',
        "gpio channel robust validation",
        'direction_guard_micros, "direction_guard_micros"',
    )
    text = replace_once(
        text,
        '''    gpio_chip: int = 0\n    edge_history_capacity: int = 512\n''',
        '''    gpio_chip: int = 0\n    edge_history_capacity: int = 512\n    diagnostic_event_capacity: int = 2048\n''',
        "gpio diagnostic capacity field",
        "diagnostic_event_capacity: int = 2048",
    )
    text = replace_once(
        text,
        '''        if not 2 <= capacity <= 4096:\n            raise ValueError("edge_history_capacity must be within [2, 4096]")\n        if len(set(self.pins)) != 4:\n''',
        '''        if not 2 <= capacity <= 4096:\n            raise ValueError("edge_history_capacity must be within [2, 4096]")\n        diagnostic_capacity = _nonnegative_int(\n            self.diagnostic_event_capacity,\n            "diagnostic_event_capacity",\n        )\n        if not 16 <= diagnostic_capacity <= 16384:\n            raise ValueError("diagnostic_event_capacity must be within [16, 16384]")\n        if len(set(self.pins)) != 4:\n''',
        "gpio diagnostic capacity validation",
        'diagnostic_event_capacity must be within [16, 16384]',
    )

    marker = '''@dataclass(slots=True)\nclass _CounterState:\n'''
    if "class QuadratureAlertEvent:" not in text:
        insertion = '''@dataclass(frozen=True, slots=True)\nclass QuadratureAlertEvent:\n    """One bounded raw A/B callback decision for offline diagnosis."""\n\n    channel: str\n    callback_timestamp_ns: int\n    physical_timestamp_ns: int | None\n    level: int\n    accepted: bool\n    reason: str\n    candidate_direction: int | None\n    pulse_count_after: int\n\n    def __post_init__(self) -> None:\n        if self.channel not in ("A", "B"):\n            raise ValueError("channel must be A or B")\n        _nonnegative_int(self.callback_timestamp_ns, "callback_timestamp_ns")\n        if self.physical_timestamp_ns is not None:\n            _nonnegative_int(self.physical_timestamp_ns, "physical_timestamp_ns")\n        if self.level not in (0, 1):\n            raise ValueError("level must be 0 or 1")\n        _bool(self.accepted, "accepted")\n        if not isinstance(self.reason, str) or not self.reason:\n            raise ValueError("reason must be a non-empty string")\n        if self.candidate_direction not in (None, -1, 1):\n            raise ValueError("candidate_direction must be -1, 1 or None")\n        if not isinstance(self.pulse_count_after, int) or isinstance(self.pulse_count_after, bool):\n            raise ValueError("pulse_count_after must be an integer")\n\n\n'''
        text = replace_once(text, marker, insertion + marker, "quadrature event dataclass")

    text = replace_once(
        text,
        '''    b_history: deque[tuple[int, int]] | None = None\n    last_a_timestamp_ns: int | None = None\n''',
        '''    b_history: deque[tuple[int, int]] | None = None\n    last_a_timestamp_ns: int | None = None\n    last_b_timestamp_ns: int | None = None\n    last_b_level: int | None = None\n    confirmed_direction: int = 0\n    pending_direction: int = 0\n    pending_a_timestamps: list[int] | None = None\n    quadrature_rejections: int = 0\n    direction_change_candidates: int = 0\n    direction_changes_confirmed: int = 0\n    diagnostic_events: deque[QuadratureAlertEvent] | None = None\n''',
        "counter robust state",
        "direction_changes_confirmed: int = 0",
    )
    text = replace_once(
        text,
        '''            side: _CounterState(\n                edge_history=deque(maxlen=config.edge_history_capacity),\n                b_history=deque(maxlen=config.edge_history_capacity),\n            )\n''',
        '''            side: _CounterState(\n                edge_history=deque(maxlen=config.edge_history_capacity),\n                b_history=deque(maxlen=config.edge_history_capacity),\n                pending_a_timestamps=[],\n                diagnostic_events=deque(maxlen=config.diagnostic_event_capacity),\n            )\n''',
        "counter robust state init",
        "diagnostic_events=deque(maxlen=config.diagnostic_event_capacity)",
    )

    old = '''    def _b_handler(\n        self,\n        side: str,\n        expected_pin: int,\n    ) -> Callable[[int, int, int, int], None]:\n        def handle(chip: int, gpio: int, level: int, tick: int) -> None:\n            with self._lock:\n                if self._closed:\n                    return\n                state = self._states[side]\n                if (\n                    not self._valid_alert(chip, gpio, expected_pin)\n                    or not isinstance(level, int)\n                    or isinstance(level, bool)\n                    or level not in (0, 1)\n                    or not self._valid_tick(tick)\n                ):\n                    state.invalid_alerts += 1\n                    return\n\n                assert state.b_history is not None\n                # A B transition which belongs at/before an already committed\n                # A edge arrived too late to establish that A edge's direction.\n                # Reject it rather than silently changing future sign state.\n                if (\n                    state.last_a_timestamp_ns is not None\n                    and tick <= state.last_a_timestamp_ns\n                ):\n                    state.invalid_alerts += 1\n                    return\n                if state.b_history and tick <= state.b_history[-1][0]:\n                    state.invalid_alerts += 1\n                    return\n\n                state.b_history.append((tick, level))\n                state.level_b = level\n\n        return handle\n\n    @staticmethod\n    def _level_b_at_physical_a(state: _CounterState, physical_tick: int) -> int | None:\n        assert state.b_history is not None\n        # Simultaneous A/B timestamps are ambiguous for X1 direction decoding.\n        if any(timestamp == physical_tick for timestamp, _ in reversed(state.b_history)):\n            return None\n        for timestamp, level in reversed(state.b_history):\n            if timestamp < physical_tick:\n                return level\n        return state.initial_level_b\n'''
    new = '''    @staticmethod\n    def _record_event(\n        state: _CounterState,\n        *,\n        channel: str,\n        callback_timestamp_ns: int,\n        physical_timestamp_ns: int | None,\n        level: int,\n        accepted: bool,\n        reason: str,\n        candidate_direction: int | None = None,\n    ) -> None:\n        assert state.diagnostic_events is not None\n        state.diagnostic_events.append(\n            QuadratureAlertEvent(\n                channel=channel,\n                callback_timestamp_ns=callback_timestamp_ns,\n                physical_timestamp_ns=physical_timestamp_ns,\n                level=level,\n                accepted=accepted,\n                reason=reason,\n                candidate_direction=candidate_direction,\n                pulse_count_after=state.pulse_count,\n            )\n        )\n\n    def _b_handler(\n        self,\n        side: str,\n        expected_pin: int,\n    ) -> Callable[[int, int, int, int], None]:\n        def handle(chip: int, gpio: int, level: int, tick: int) -> None:\n            with self._lock:\n                if self._closed:\n                    return\n                state = self._states[side]\n                if (\n                    not self._valid_alert(chip, gpio, expected_pin)\n                    or not isinstance(level, int)\n                    or isinstance(level, bool)\n                    or level not in (0, 1)\n                    or not self._valid_tick(tick)\n                ):\n                    state.invalid_alerts += 1\n                    return\n\n                assert state.b_history is not None\n                # A B transition which belongs at/before an already observed\n                # physical A edge arrived too late to establish that A edge's\n                # direction. This remains a hard ordering diagnostic.\n                if (\n                    state.last_a_timestamp_ns is not None\n                    and tick <= state.last_a_timestamp_ns\n                ):\n                    state.invalid_alerts += 1\n                    self._record_event(\n                        state, channel="B", callback_timestamp_ns=tick,\n                        physical_timestamp_ns=tick, level=level, accepted=False,\n                        reason="B_LATE_AFTER_A",\n                    )\n                    return\n                if state.b_history and tick <= state.b_history[-1][0]:\n                    state.invalid_alerts += 1\n                    self._record_event(\n                        state, channel="B", callback_timestamp_ns=tick,\n                        physical_timestamp_ns=tick, level=level, accepted=False,\n                        reason="B_NONMONOTONIC",\n                    )\n                    return\n                if level == state.level_b:\n                    state.quadrature_rejections += 1\n                    self._record_event(\n                        state, channel="B", callback_timestamp_ns=tick,\n                        physical_timestamp_ns=tick, level=level, accepted=False,\n                        reason="B_DUPLICATE_LEVEL",\n                    )\n                    return\n\n                state.b_history.append((tick, level))\n                state.level_b = level\n                state.last_b_timestamp_ns = tick\n                state.last_b_level = level\n                self._record_event(\n                    state, channel="B", callback_timestamp_ns=tick,\n                    physical_timestamp_ns=tick, level=level, accepted=True,\n                    reason="B_TRANSITION",\n                )\n\n        return handle\n\n    @staticmethod\n    def _b_evidence_at_physical_a(\n        state: _CounterState,\n        physical_tick: int,\n        guard_ns: int,\n    ) -> tuple[int | None, str]:\n        assert state.b_history is not None\n        previous: tuple[int, int] | None = None\n        following: tuple[int, int] | None = None\n        for event in state.b_history:\n            if event[0] < physical_tick:\n                previous = event\n                continue\n            if event[0] == physical_tick:\n                return None, "B_AT_A_TIMESTAMP"\n            following = event\n            break\n        if guard_ns > 0:\n            if previous is not None and physical_tick - previous[0] <= guard_ns:\n                return None, "B_UNSTABLE_BEFORE_A"\n            if following is not None and following[0] - physical_tick <= guard_ns:\n                return None, "B_UNSTABLE_AFTER_A"\n        return (\n            previous[1] if previous is not None else state.initial_level_b,\n            "B_STABLE_AT_A",\n        )\n\n    @staticmethod\n    def _append_signed_edge(\n        state: _CounterState,\n        timestamp_ns: int,\n        direction: int,\n    ) -> None:\n        assert state.edge_history is not None\n        state.pulse_count += direction\n        state.edge_history.append(SignedPulseEdge(timestamp_ns, state.pulse_count))\n\n    @staticmethod\n    def _reject_pending_direction(state: _CounterState) -> None:\n        assert state.pending_a_timestamps is not None\n        if state.pending_a_timestamps:\n            state.quadrature_rejections += len(state.pending_a_timestamps)\n        state.pending_a_timestamps.clear()\n        state.pending_direction = 0\n\n    def _commit_a_direction(\n        self,\n        state: _CounterState,\n        channel: GpioCounterChannelConfig,\n        physical_tick: int,\n        direction: int,\n    ) -> tuple[bool, str]:\n        assert state.pending_a_timestamps is not None\n        confirm_edges = channel.direction_change_confirm_edges\n        if state.confirmed_direction == 0:\n            state.confirmed_direction = direction\n            self._append_signed_edge(state, physical_tick, direction)\n            return True, "A_INITIAL_DIRECTION"\n\n        if direction == state.confirmed_direction:\n            if state.pending_a_timestamps:\n                self._reject_pending_direction(state)\n            self._append_signed_edge(state, physical_tick, direction)\n            return True, "A_CONFIRMED_DIRECTION"\n\n        if confirm_edges == 1:\n            state.direction_change_candidates += 1\n            state.direction_changes_confirmed += 1\n            state.confirmed_direction = direction\n            self._append_signed_edge(state, physical_tick, direction)\n            return True, "A_REVERSAL_CONFIRMED"\n\n        window_ns = channel.direction_change_confirm_window_micros * 1_000\n        if state.pending_direction != direction:\n            self._reject_pending_direction(state)\n        elif (\n            state.pending_a_timestamps\n            and physical_tick - state.pending_a_timestamps[0] > window_ns\n        ):\n            self._reject_pending_direction(state)\n\n        if not state.pending_a_timestamps:\n            state.pending_direction = direction\n        state.pending_a_timestamps.append(physical_tick)\n        state.direction_change_candidates += 1\n\n        if len(state.pending_a_timestamps) < confirm_edges:\n            return False, "A_REVERSAL_PENDING"\n\n        pending = tuple(state.pending_a_timestamps)\n        state.pending_a_timestamps.clear()\n        state.pending_direction = 0\n        state.confirmed_direction = direction\n        state.direction_changes_confirmed += 1\n        for timestamp_ns in pending:\n            self._append_signed_edge(state, timestamp_ns, direction)\n        return True, "A_REVERSAL_CONFIRMED"\n'''
    text = replace_once(text, old, new, "replace B/A evidence helpers", "A_REVERSAL_PENDING")

    old = '''                assert state.edge_history is not None\n                if (\n                    state.edge_history\n                    and physical_tick <= state.edge_history[-1].timestamp_ns\n                ):\n                    state.invalid_alerts += 1\n                    return\n\n                level_b = self._level_b_at_physical_a(state, physical_tick)\n                if level_b is None:\n                    state.invalid_alerts += 1\n                    return\n\n                direction = 1 if level_b == channel.forward_b_level else -1\n                if channel.invert:\n                    direction = -direction\n                state.pulse_count += direction\n                state.edge_history.append(\n                    SignedPulseEdge(physical_tick, state.pulse_count),\n                )\n                state.last_a_timestamp_ns = physical_tick\n'''
    new = '''                if (\n                    state.last_a_timestamp_ns is not None\n                    and physical_tick <= state.last_a_timestamp_ns\n                ):\n                    state.invalid_alerts += 1\n                    return\n\n                guard_ns = channel.direction_guard_micros * 1_000\n                level_b, evidence_reason = self._b_evidence_at_physical_a(\n                    state, physical_tick, guard_ns\n                )\n                if level_b is None:\n                    state.quadrature_rejections += 1\n                    state.last_a_timestamp_ns = physical_tick\n                    self._record_event(\n                        state, channel="A", callback_timestamp_ns=tick,\n                        physical_timestamp_ns=physical_tick, level=1, accepted=False,\n                        reason=evidence_reason,\n                    )\n                    return\n\n                direction = 1 if level_b == channel.forward_b_level else -1\n                if channel.invert:\n                    direction = -direction\n                accepted, reason = self._commit_a_direction(\n                    state, channel, physical_tick, direction\n                )\n                state.last_a_timestamp_ns = physical_tick\n                self._record_event(\n                    state, channel="A", callback_timestamp_ns=tick,\n                    physical_timestamp_ns=physical_tick, level=1, accepted=accepted,\n                    reason=reason, candidate_direction=direction,\n                )\n'''
    text = replace_once(text, old, new, "A robust commit block", "guard_ns = channel.direction_guard_micros")

    old = '''            return SignedPulseCounterSnapshot(\n                pulse_count=state.pulse_count,\n                read_errors=state.read_errors,\n                invalid_alerts=state.invalid_alerts,\n                edge_history=tuple(state.edge_history),\n            )\n'''
    new = '''            return SignedPulseCounterSnapshot(\n                pulse_count=state.pulse_count,\n                read_errors=state.read_errors,\n                invalid_alerts=state.invalid_alerts,\n                edge_history=tuple(state.edge_history),\n                quadrature_rejections=state.quadrature_rejections,\n                direction_change_candidates=state.direction_change_candidates,\n                direction_changes_confirmed=state.direction_changes_confirmed,\n                confirmed_direction=state.confirmed_direction,\n                pending_direction=state.pending_direction,\n                pending_direction_edges=len(state.pending_a_timestamps or ()),\n                last_a_timestamp_ns=state.last_a_timestamp_ns,\n                last_b_timestamp_ns=state.last_b_timestamp_ns,\n                last_b_level=state.last_b_level,\n            )\n\n    def diagnostic_events(self, side: str) -> tuple[QuadratureAlertEvent, ...]:\n        """Return the bounded raw A/B decision trace without motor authority."""\n\n        if side not in self._states:\n            raise ValueError("side must be left or right")\n        with self._lock:\n            events = self._states[side].diagnostic_events\n            assert events is not None\n            return tuple(events)\n'''
    text = replace_once(text, old, new, "snapshot robust diagnostics", "def diagnostic_events(self, side: str)")
    text = replace_once(
        text,
        '''    "GpioCounterPairConfig",\n    "NativeGpioSignedCounterPair",\n]''',
        '''    "GpioCounterPairConfig",\n    "NativeGpioSignedCounterPair",\n    "QuadratureAlertEvent",\n]''',
        "gpio exports",
        '    "QuadratureAlertEvent",\n]',
    )
    return text


def patch_counter(text: str) -> str:
    text = replace_once(
        text,
        '''    invalid_alerts: int = 0\n    edge_history: tuple[SignedPulseEdge, ...] = ()\n''',
        '''    invalid_alerts: int = 0\n    edge_history: tuple[SignedPulseEdge, ...] = ()\n    quadrature_rejections: int = 0\n    direction_change_candidates: int = 0\n    direction_changes_confirmed: int = 0\n    confirmed_direction: int = 0\n    pending_direction: int = 0\n    pending_direction_edges: int = 0\n    last_a_timestamp_ns: int | None = None\n    last_b_timestamp_ns: int | None = None\n    last_b_level: int | None = None\n''',
        "snapshot quadrature fields",
        "quadrature_rejections: int = 0",
    )
    text = replace_once(
        text,
        '''        _nonnegative_int(self.read_errors, "read_errors")\n        _nonnegative_int(self.invalid_alerts, "invalid_alerts")\n        if not isinstance(self.edge_history, tuple) or any(\n''',
        '''        _nonnegative_int(self.read_errors, "read_errors")\n        _nonnegative_int(self.invalid_alerts, "invalid_alerts")\n        _nonnegative_int(self.quadrature_rejections, "quadrature_rejections")\n        _nonnegative_int(self.direction_change_candidates, "direction_change_candidates")\n        _nonnegative_int(self.direction_changes_confirmed, "direction_changes_confirmed")\n        _nonnegative_int(self.pending_direction_edges, "pending_direction_edges")\n        for value, name in (\n            (self.confirmed_direction, "confirmed_direction"),\n            (self.pending_direction, "pending_direction"),\n        ):\n            if value not in (-1, 0, 1):\n                raise ValueError(f"{name} must be -1, 0 or 1")\n        for value, name in (\n            (self.last_a_timestamp_ns, "last_a_timestamp_ns"),\n            (self.last_b_timestamp_ns, "last_b_timestamp_ns"),\n        ):\n            if value is not None:\n                _nonnegative_int(value, name)\n        if self.last_b_level not in (None, 0, 1):\n            raise ValueError("last_b_level must be 0, 1 or None")\n        if not isinstance(self.edge_history, tuple) or any(\n''',
        "snapshot quadrature validation",
        'pending_direction_edges, "pending_direction_edges"',
    )

    # Add new diagnostics to the constructor call. EncoderEdgeDiagnostics has
    # backwards-compatible defaults, so old fake snapshots remain valid.
    text = replace_once(
        text,
        '''            maximum_abs_velocity_mps=config.maximum_abs_velocity_mps,\n            rejection_code=rejection_code,\n        )\n''',
        '''            maximum_abs_velocity_mps=config.maximum_abs_velocity_mps,\n            rejection_code=rejection_code,\n            left_quadrature_rejections=current.left.quadrature_rejections,\n            right_quadrature_rejections=current.right.quadrature_rejections,\n            left_quadrature_rejection_delta=(\n                None if previous is None else\n                current.left.quadrature_rejections - previous.left.quadrature_rejections\n            ),\n            right_quadrature_rejection_delta=(\n                None if previous is None else\n                current.right.quadrature_rejections - previous.right.quadrature_rejections\n            ),\n            left_direction_change_candidates=current.left.direction_change_candidates,\n            right_direction_change_candidates=current.right.direction_change_candidates,\n            left_direction_changes_confirmed=current.left.direction_changes_confirmed,\n            right_direction_changes_confirmed=current.right.direction_changes_confirmed,\n            left_confirmed_direction=current.left.confirmed_direction,\n            right_confirmed_direction=current.right.confirmed_direction,\n            left_pending_direction=current.left.pending_direction,\n            right_pending_direction=current.right.pending_direction,\n            left_pending_direction_edges=current.left.pending_direction_edges,\n            right_pending_direction_edges=current.right.pending_direction_edges,\n            left_last_a_timestamp_ns=current.left.last_a_timestamp_ns,\n            right_last_a_timestamp_ns=current.right.last_a_timestamp_ns,\n            left_last_b_timestamp_ns=current.left.last_b_timestamp_ns,\n            right_last_b_timestamp_ns=current.right.last_b_timestamp_ns,\n            left_last_b_level=current.left.last_b_level,\n            right_last_b_level=current.right.last_b_level,\n        )\n''',
        "encoder quadrature diagnostics",
        "left_quadrature_rejections=current.left.quadrature_rejections",
    )
    return text


def patch_live(text: str) -> str:
    text = replace_once(
        text,
        '''    maximum_abs_velocity_mps: float\n    rejection_code: EncoderRejectionCode\n''',
        '''    maximum_abs_velocity_mps: float\n    rejection_code: EncoderRejectionCode\n    left_quadrature_rejections: int = 0\n    right_quadrature_rejections: int = 0\n    left_quadrature_rejection_delta: int | None = None\n    right_quadrature_rejection_delta: int | None = None\n    left_direction_change_candidates: int = 0\n    right_direction_change_candidates: int = 0\n    left_direction_changes_confirmed: int = 0\n    right_direction_changes_confirmed: int = 0\n    left_confirmed_direction: int = 0\n    right_confirmed_direction: int = 0\n    left_pending_direction: int = 0\n    right_pending_direction: int = 0\n    left_pending_direction_edges: int = 0\n    right_pending_direction_edges: int = 0\n    left_last_a_timestamp_ns: int | None = None\n    right_last_a_timestamp_ns: int | None = None\n    left_last_b_timestamp_ns: int | None = None\n    right_last_b_timestamp_ns: int | None = None\n    left_last_b_level: int | None = None\n    right_last_b_level: int | None = None\n''',
        "live encoder quadrature diagnostic fields",
        "left_quadrature_rejections: int = 0",
    )
    # Validate added diagnostics just before rejection-code type validation.
    text = replace_once(
        text,
        '''        if not isinstance(self.rejection_code, EncoderRejectionCode):\n            raise TypeError("rejection_code must be EncoderRejectionCode")\n''',
        '''        for value, name in (\n            (self.left_quadrature_rejections, "left_quadrature_rejections"),\n            (self.right_quadrature_rejections, "right_quadrature_rejections"),\n            (self.left_direction_change_candidates, "left_direction_change_candidates"),\n            (self.right_direction_change_candidates, "right_direction_change_candidates"),\n            (self.left_direction_changes_confirmed, "left_direction_changes_confirmed"),\n            (self.right_direction_changes_confirmed, "right_direction_changes_confirmed"),\n            (self.left_pending_direction_edges, "left_pending_direction_edges"),\n            (self.right_pending_direction_edges, "right_pending_direction_edges"),\n        ):\n            _nonnegative_integer(value, name)\n        for value, name in (\n            (self.left_quadrature_rejection_delta, "left_quadrature_rejection_delta"),\n            (self.right_quadrature_rejection_delta, "right_quadrature_rejection_delta"),\n            (self.left_last_a_timestamp_ns, "left_last_a_timestamp_ns"),\n            (self.right_last_a_timestamp_ns, "right_last_a_timestamp_ns"),\n            (self.left_last_b_timestamp_ns, "left_last_b_timestamp_ns"),\n            (self.right_last_b_timestamp_ns, "right_last_b_timestamp_ns"),\n        ):\n            _optional_integer(value, name)\n        for value, name in (\n            (self.left_confirmed_direction, "left_confirmed_direction"),\n            (self.right_confirmed_direction, "right_confirmed_direction"),\n            (self.left_pending_direction, "left_pending_direction"),\n            (self.right_pending_direction, "right_pending_direction"),\n        ):\n            if value not in (-1, 0, 1):\n                raise ValueError(f"{name} must be -1, 0 or 1")\n        for value, name in (\n            (self.left_last_b_level, "left_last_b_level"),\n            (self.right_last_b_level, "right_last_b_level"),\n        ):\n            if value not in (None, 0, 1):\n                raise ValueError(f"{name} must be 0, 1 or None")\n        if not isinstance(self.rejection_code, EncoderRejectionCode):\n            raise TypeError("rejection_code must be EncoderRejectionCode")\n''',
        "live quadrature validation",
        'left_quadrature_rejections, "left_quadrature_rejections"',
    )
    text = replace_once(
        text,
        '''            DataField(\n                "maximum_abs_velocity_mps",\n                diagnostics.maximum_abs_velocity_mps,\n            ),\n        )\n''',
        '''            DataField(\n                "maximum_abs_velocity_mps",\n                diagnostics.maximum_abs_velocity_mps,\n            ),\n            DataField("left_quadrature_rejections", diagnostics.left_quadrature_rejections),\n            DataField("right_quadrature_rejections", diagnostics.right_quadrature_rejections),\n            DataField("left_quadrature_rejection_delta", diagnostics.left_quadrature_rejection_delta),\n            DataField("right_quadrature_rejection_delta", diagnostics.right_quadrature_rejection_delta),\n            DataField("left_direction_change_candidates", diagnostics.left_direction_change_candidates),\n            DataField("right_direction_change_candidates", diagnostics.right_direction_change_candidates),\n            DataField("left_direction_changes_confirmed", diagnostics.left_direction_changes_confirmed),\n            DataField("right_direction_changes_confirmed", diagnostics.right_direction_changes_confirmed),\n            DataField("left_confirmed_direction", diagnostics.left_confirmed_direction),\n            DataField("right_confirmed_direction", diagnostics.right_confirmed_direction),\n            DataField("left_pending_direction", diagnostics.left_pending_direction),\n            DataField("right_pending_direction", diagnostics.right_pending_direction),\n            DataField("left_pending_direction_edges", diagnostics.left_pending_direction_edges),\n            DataField("right_pending_direction_edges", diagnostics.right_pending_direction_edges),\n            DataField("left_last_a_timestamp_ns", diagnostics.left_last_a_timestamp_ns),\n            DataField("right_last_a_timestamp_ns", diagnostics.right_last_a_timestamp_ns),\n            DataField("left_last_b_timestamp_ns", diagnostics.left_last_b_timestamp_ns),\n            DataField("right_last_b_timestamp_ns", diagnostics.right_last_b_timestamp_ns),\n            DataField("left_last_b_level", diagnostics.left_last_b_level),\n            DataField("right_last_b_level", diagnostics.right_last_b_level),\n        )\n''',
        "live diagnostic DataFields",
        'DataField("left_quadrature_rejections"',
    )
    return text


def patch_bounded(text: str) -> str:
    text = replace_once(
        text,
        '''    forward_b_level = encoders.get("forward_b_level")\n    debounce_micros = encoders.get("a_debounce_micros")\n    pull_up = _required_bool(\n''',
        '''    forward_b_level = encoders.get("forward_b_level")\n    debounce_micros = encoders.get("a_debounce_micros")\n    direction_guard_micros = encoders.get("direction_guard_micros", 50)\n    direction_change_confirm_edges = encoders.get("direction_change_confirm_edges", 3)\n    direction_change_confirm_window_micros = encoders.get(\n        "direction_change_confirm_window_micros", 250_000\n    )\n    pull_up = _required_bool(\n''',
        "bounded robust encoder config read",
        'direction_guard_micros = encoders.get("direction_guard_micros", 50)',
    )
    old = '''            pull_up=pull_up,\n            a_debounce_micros=debounce_micros,  # type: ignore[arg-type]\n        ),\n'''
    new = '''            pull_up=pull_up,\n            a_debounce_micros=debounce_micros,  # type: ignore[arg-type]\n            direction_guard_micros=direction_guard_micros,  # type: ignore[arg-type]\n            direction_change_confirm_edges=direction_change_confirm_edges,  # type: ignore[arg-type]\n            direction_change_confirm_window_micros=(\n                direction_change_confirm_window_micros  # type: ignore[arg-type]\n            ),\n        ),\n'''
    if old in text:
        if text.count(old) != 2:
            raise RuntimeError(f"bounded per-wheel block: expected 2 occurrences, found {text.count(old)}")
        text = text.replace(old, new, 2)
    elif "direction_change_confirm_edges=direction_change_confirm_edges" not in text:
        raise RuntimeError("bounded per-wheel block not found")
    return text


def patch_hardware(data: bytes) -> bytes:
    obj = json.loads(data.decode("utf-8"))
    enc = obj.get("encoderek")
    if not isinstance(enc, dict):
        raise RuntimeError("conf/hardver.json missing encoderek object")
    desired = {
        "direction_guard_micros": 50,
        "direction_change_confirm_edges": 3,
        "direction_change_confirm_window_micros": 250000,
    }
    for key, value in desired.items():
        existing = enc.get(key, value)
        if existing != value:
            raise RuntimeError(f"conf/hardver.json {key} already has unexpected value {existing!r}")
        enc[key] = value
    return (json.dumps(obj, indent=2, ensure_ascii=False) + "\n").encode("utf-8")


def main(argv: list[str]) -> int:
    root = Path(argv[1] if len(argv) > 1 else ".").resolve()
    package = Path(__file__).resolve().parent
    paths = {name: root / name for name in KNOWN}
    for name, path in paths.items():
        if not path.is_file():
            raise SystemExit(f"missing required file: {path}")

    originals = {name: path.read_bytes() for name, path in paths.items()}
    for name, expected in KNOWN.items():
        actual = git_blob_sha(originals[name])
        # Already-patched source is allowed by semantic marker below, otherwise
        # an unknown revision is never overwritten blindly.
        if actual != expected:
            text = originals[name].decode("utf-8", errors="ignore")
            marker_ok = {
                "v3/adapters/gpio_counter.py": "A_REVERSAL_PENDING" in text,
                "v3/adapters/counter_encoder.py": "quadrature_rejections: int = 0" in text,
                "v3/adapters/live_encoder.py": "left_quadrature_rejections: int = 0" in text,
                "v3_bounded_config.py": "direction_change_confirm_edges = encoders.get" in text,
                "conf/hardver.json": '"direction_change_confirm_edges": 3' in text,
            }[name]
            if not marker_ok:
                raise RuntimeError(
                    f"{name}: unknown revision (git blob {actual}); refusing blind overwrite"
                )

    patched = dict(originals)
    patched["v3/adapters/gpio_counter.py"] = patch_gpio(originals["v3/adapters/gpio_counter.py"].decode()).encode()
    patched["v3/adapters/counter_encoder.py"] = patch_counter(originals["v3/adapters/counter_encoder.py"].decode()).encode()
    patched["v3/adapters/live_encoder.py"] = patch_live(originals["v3/adapters/live_encoder.py"].decode()).encode()
    patched["v3_bounded_config.py"] = patch_bounded(originals["v3_bounded_config.py"].decode()).encode()
    patched["conf/hardver.json"] = patch_hardware(originals["conf/hardver.json"])

    stamp = time.strftime("%Y%m%d_%H%M%S")
    backup = Path("/tmp") / f"r2b4_encoder_ab_backup_{stamp}"
    backup.mkdir(parents=True)
    for name, data in originals.items():
        dest = backup / name
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_bytes(data)

    added = [
        "tests/test_v3_encoder_ab_direction_robustness.py",
        "tools/v3_encoder_ab_probe.py",
    ]
    prior_added = {name: (root / name).read_bytes() if (root / name).exists() else None for name in added}
    try:
        for name, data in patched.items():
            paths[name].write_bytes(data)
        for name in added:
            src = package / name
            dest = root / name
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dest)
        subprocess.run(
            [sys.executable, "-m", "py_compile",
             str(root / "v3/adapters/gpio_counter.py"),
             str(root / "v3/adapters/counter_encoder.py"),
             str(root / "v3/adapters/live_encoder.py"),
             str(root / "v3_bounded_config.py"),
             str(root / "tools/v3_encoder_ab_probe.py"),
             str(root / "tests/test_v3_encoder_ab_direction_robustness.py")],
            cwd=root, check=True,
        )
    except BaseException:
        for name, data in originals.items():
            paths[name].write_bytes(data)
        for name, prior in prior_added.items():
            dest = root / name
            if prior is None:
                dest.unlink(missing_ok=True)
            else:
                dest.write_bytes(prior)
        print("Install failed; production files rolled back.", file=sys.stderr)
        print(f"Backups: {backup}", file=sys.stderr)
        raise

    print("Encoder A/B robustness upgrade installed.")
    print(f"Backups: {backup}")
    print("Production policy: B guard=50 us, reversal confirmation=3 A-rising edges, window=250 ms")
    print("Run first:")
    print("  python3 -m pytest -q tests/test_v3_encoder_ab_direction_robustness.py")
    print("Then:")
    print("  python3 -m pytest -q tests/test_v3_gpio_counter_owner.py tests/test_v3_encoder_robustness_regressions.py tests/test_v3_encoder_p0_integration.py")
    print("Motor-output-free A/B probe (runtime must be stopped):")
    print("  python3 tools/v3_encoder_ab_probe.py --seconds 15")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
