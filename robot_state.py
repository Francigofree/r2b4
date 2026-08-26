#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Megosztott mozgási állapot a GUI és a vezérlő hurok között.

Thread-safe: az API szál ír (set_intent), a vezérlő hurok olvas és frissít.
A GUI és a controller ugyanabban a processben futnak (külön szálak),
így a threading.Lock elegendő szinkronizációhoz.
"""

import math
import threading
import time

_lock = threading.Lock()

motion_state = {
    "intent_x": 0.0,
    "intent_y": 0.0,
    "intent_source": "",
    "intent_ts": 0.0,
    "intent_mono_ts": 0.0,
    "intent_ts_us": 0,
    "intent_seq": 0,
    "intent_client_ts": 0.0,
    "target_left": 0.0,
    "target_right": 0.0,
    "actual_left": 0.0,
    "actual_right": 0.0,
    "timestamp": 0.0,
    # Perception → control: követendő célszemely pozíciója (thread-safe megosztva)
    "tracked_target": {
        "dist_m": None,
        "angle_deg": None,
        "confidence": 0.0,
        "vx": None,
        "vy": None,
        "last_seen_ts": 0.0,
        "ts": 0.0,
    },
}

FAILSAFE_TIMEOUT_S = 1.0


def set_intent(x, y, source="", seq=None, client_ts=None):
    """GUI/API hívja: új mozgási szándék beállítása (seq-védelemmel)."""
    with _lock:
        current_seq = int(motion_state.get("intent_seq", 0))
        try:
            next_seq = int(seq) if seq is not None else (current_seq + 1)
        except Exception:
            next_seq = current_seq + 1
        # Régebbi csomagokat eldobjuk (reconnect/out-of-order védelem).
        if next_seq < current_seq:
            return False
        motion_state["intent_x"] = max(-1.0, min(1.0, float(x)))
        motion_state["intent_y"] = max(-1.0, min(1.0, float(y)))
        motion_state["intent_source"] = str(source)
        motion_state["intent_seq"] = next_seq
        try:
            motion_state["intent_client_ts"] = float(client_ts) if client_ts is not None else 0.0
        except Exception:
            motion_state["intent_client_ts"] = 0.0
        motion_state["intent_ts"] = time.time()
        motion_state["intent_mono_ts"] = time.perf_counter()
        motion_state["intent_ts_us"] = int(time.perf_counter_ns() // 1_000)
        return True


def get_intent():
    """Vezérlő hurok hívja: aktuális intent lekérése (x, y, source, mono_ts, seq)."""
    with _lock:
        return (
            motion_state["intent_x"],
            motion_state["intent_y"],
            motion_state["intent_source"],
            motion_state.get("intent_mono_ts", 0.0) or motion_state.get("intent_ts", 0.0),
            int(motion_state.get("intent_seq", 0)),
        )


def update_targets(left, right):
    """Vezérlő hurok: számított célértékek írása (tank mix az intent-ből)."""
    with _lock:
        motion_state["target_left"] = round(float(left), 4)
        motion_state["target_right"] = round(float(right), 4)
        motion_state["timestamp"] = time.time()


def update_actuals(left, right):
    """Vezérlő hurok: tényleges motor PWM visszaírása."""
    with _lock:
        motion_state["actual_left"] = round(float(left), 4)
        motion_state["actual_right"] = round(float(right), 4)


def get_full_state():
    """Telemetria/SSE: teljes állapot másolata."""
    with _lock:
        return dict(motion_state)


def is_intent_stale():
    """Failsafe: True ha az intent régebbi mint FAILSAFE_TIMEOUT_S."""
    return get_intent_age_s() > FAILSAFE_TIMEOUT_S


def get_intent_age_s() -> float:
    """Intent életkora másodpercben, monotonic órával ha elérhető."""
    with _lock:
        mono_ts = float(motion_state.get("intent_mono_ts", 0.0) or 0.0)
        wall_ts = float(motion_state.get("intent_ts", 0.0) or 0.0)
        if mono_ts > 0.0:
            return max(0.0, time.perf_counter() - mono_ts)
        if wall_ts > 0.0:
            return max(0.0, time.time() - wall_ts)
        return 0.0


def clear_intent():
    """Failsafe: intent nullázása (robot megáll)."""
    with _lock:
        motion_state["intent_x"] = 0.0
        motion_state["intent_y"] = 0.0
        motion_state["intent_mono_ts"] = 0.0
        motion_state["intent_ts_us"] = 0


# ── Tracked target (perception → control) ──

def set_tracked_target(dist_m, angle_deg, confidence=1.0, vx=None, vy=None, last_seen_ts=None):
    """Perception szál hívja: követendő ember pozíciójának frissítése."""
    with _lock:
        tt = motion_state["tracked_target"]
        tt["dist_m"] = float(dist_m) if dist_m is not None else None
        tt["angle_deg"] = float(angle_deg) if angle_deg is not None else None
        tt["confidence"] = max(0.0, min(1.0, float(confidence)))
        if vx is not None:
            vx_f = float(vx)
            tt["vx"] = vx_f if math.isfinite(vx_f) else None
        else:
            tt["vx"] = None
        if vy is not None:
            vy_f = float(vy)
            tt["vy"] = vy_f if math.isfinite(vy_f) else None
        else:
            tt["vy"] = None
        if last_seen_ts is None:
            if dist_m is not None and angle_deg is not None:
                tt["last_seen_ts"] = time.time()
        else:
            ts_f = float(last_seen_ts)
            tt["last_seen_ts"] = ts_f if math.isfinite(ts_f) else 0.0
        tt["ts"] = time.monotonic()


def get_tracked_target():
    """Kontroll szál hívja: utolsó ismert célpozíció másolata."""
    with _lock:
        return dict(motion_state["tracked_target"])


def clear_tracked_target():
    """Követés leállításakor: cél törlése."""
    with _lock:
        tt = motion_state["tracked_target"]
        tt["dist_m"] = None
        tt["angle_deg"] = None
        tt["confidence"] = 0.0
        tt["vx"] = None
        tt["vy"] = None
        tt["last_seen_ts"] = 0.0
        tt["ts"] = 0.0
