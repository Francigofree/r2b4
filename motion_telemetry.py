#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Motion telemetria: robot_state alapú mozgás-visszacsatolás az SSE streamhez.

A GUI ezt kapja valós időben, hogy a kezelőfelület tükrözze
a tényleges robot állapotot (intent → target → actual).
"""

import robot_state


def get_motion_telemetry():
    """
    Teljes motion state csomag SSE streaming-hez.

    Mezők:
        intent_x, intent_y  – GUI szándék
        target_left/right   – vezérlő hurok által számított célérték
        actual_left/right   – tényleges motor PWM
        stale               – True ha a GUI kapcsolat megszakadt (>1s)
    """
    ms = robot_state.get_full_state()
    return {
        "intent_x": ms["intent_x"],
        "intent_y": ms["intent_y"],
        "intent_source": ms.get("intent_source", ""),
        "intent_seq": int(ms.get("intent_seq", 0)),
        "intent_client_ts": ms.get("intent_client_ts", 0.0),
        "target_left": ms["target_left"],
        "target_right": ms["target_right"],
        "actual_left": ms["actual_left"],
        "actual_right": ms["actual_right"],
        "intent_ts": ms["intent_ts"],
        "intent_ts_us": ms.get("intent_ts_us", 0),
        "timestamp": ms["timestamp"],
        "stale": robot_state.is_intent_stale(),
    }
