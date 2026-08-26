#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Explicit motion arbiter: egyetlen döntéshozó a mozgásforrásokra.

- Determinisztikus prioritási modell: alacsonyabb index = magasabb prioritás.
  Vészleállítás (emergency_stop) az arbiteren kívül van, mindig nyer.
- Formális forrásállapot: minden váltás set_motion_source / allow_source-on keresztül
  (controller.commands); kivéve vészleállítás után közvetlen MANUAL + arbiter sync.
- Behavior izoláció: a control_loop csak az aktuális forrás (motion_command_source)
  szerint írja v_target/omega_target-ot; ADAPTIVE/STATE/AI ne írjuk felül külső forrásból.
"""

import time

# Rendszer szintű prioritás (konfig felülírhatja):
# GUI joystick elsődleges, MANUAL másodlagos; vészstop az arbiteren kívül mindig nyer.
DEFAULT_PRIORITIES = [
    "GUI_JOYSTICK", # GUI joystick / set_vector (elsődleges kezelő)
    "MANUAL",       # Billentyűzet (WASD, stb.) - SPACE/stop külön biztonsági út
    "STATE",        # Állapotgép rutinok (CIRCLE, PATROL, stb.)
    "ADAPTIVE",     # Követés (follower)
    "AI",           # AI / Core parancsok
    "CORE",         # Core executor
]


class Arbiter:
    """
    Explicit motion arbiter: források determinisztikus prioritása + idő alapú birtoklás (hold_sec).
    Csak egy forrás aktív; váltás: timeout (hold_sec eltelt) vagy magasabb prioritású kérés.
    """
    def __init__(self, priorities=None, hold_sec=2.0):
        raw = list(DEFAULT_PRIORITIES) if priorities is None else [str(item) for item in priorities]
        if len(raw) != len(DEFAULT_PRIORITIES) or set(raw) != set(DEFAULT_PRIORITIES):
            raise ValueError("arbiter priorities must contain every canonical source exactly once")
        self.priorities = raw
        self.hold_sec = float(hold_sec)
        if self.hold_sec < 0.0:
            raise ValueError("arbiter hold_sec must be non-negative")
        self.active = None
        self.last_ts = {}
        self.last_switch = {"ts": 0.0, "from": None, "to": None, "reason": ""}

    def _prio(self, source):
        try:
            return self.priorities.index(source)
        except ValueError:
            return len(self.priorities)

    def touch(self, source, now=None):
        if now is None:
            now = time.monotonic()
        self.last_ts[source] = now

        if self.active is None:
            self._switch(source, now, "init")
            return True

        if self.active == source:
            return True

        active_age = now - self.last_ts.get(self.active, 0)
        if active_age > self.hold_sec:
            self._switch(source, now, "timeout")
            return True

        if self._prio(source) < self._prio(self.active):
            self._switch(source, now, "priority")
            return True

        return False

    def allow(self, source, now=None):
        self.touch(source, now)
        return self.active == source

    def decide(self, source, now=None):
        """
        Döntés (engedélyezett-e a forrás).
        """
        if now is None:
            now = time.monotonic()
        self.touch(source, now)
        if self.active == source:
            return True, "active"
        active_age = now - self.last_ts.get(self.active, 0) if self.active else 0
        if active_age > self.hold_sec:
            self._switch(source, now, "timeout")
            return True, "timeout_switch"
        if self._prio(source) < self._prio(self.active):
            self._switch(source, now, "priority_switch")
            return True, "priority_switch"
        return False, "blocked_by_active"

    def _switch(self, source, now, reason):
        prev = self.active
        self.active = source
        self.last_switch = {"ts": now, "from": prev, "to": source, "reason": reason}

    def status(self, now=None):
        if now is None:
            now = time.monotonic()
        ages = {
            src: int((now - ts) * 1000)
            for src, ts in self.last_ts.items()
        }
        return {
            "active": self.active,
            "hold_sec": self.hold_sec,
            "ages_ms": ages,
            "last_switch": self.last_switch,
        }
