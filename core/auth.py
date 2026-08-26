#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from config_manager import config as global_config


@dataclass
class AuthResult:
    ok: bool
    role: str | None = None
    reason: str = ""


class AuthManager:
    """
    Egyszerű token-alapú jogosultsági réteg.
    """
    def __init__(self):
        self._reload()

    def _reload(self):
        sec = global_config.get("security", default={})
        self.roles = sec.get("roles", {})
        self.tokens = sec.get("tokens", {})

    def authorize(self, token: str | None, command: str) -> AuthResult:
        self._reload()
        if not token:
            return AuthResult(False, reason="hiányzó token")

        t = self.tokens.get(token)
        if not t or not t.get("enabled", False):
            return AuthResult(False, reason="token tiltott")

        role = t.get("role", "")
        allowed = self.roles.get(role, {}).get("commands", [])
        if command not in allowed:
            return AuthResult(False, role=role, reason="parancs tiltott")

        return AuthResult(True, role=role, reason="ok")

