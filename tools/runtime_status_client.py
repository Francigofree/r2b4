#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Shared cached runtime JSON reader for live tools."""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Any, Dict, Union


class RuntimeStatusClient:
    """Cached JSON file reader with a minimum poll interval per path."""

    def __init__(self, min_poll_interval_s: float = 0.10):
        self._min_poll_interval_s = max(0.01, float(min_poll_interval_s))
        self._cache: Dict[str, Dict[str, Any]] = {}
        self._lock = threading.Lock()

    def set_min_poll_interval(self, min_poll_interval_s: float) -> None:
        with self._lock:
            self._min_poll_interval_s = max(0.01, float(min_poll_interval_s))

    def read_json(
        self,
        path: Union[str, Path],
        *,
        force: bool = False,
        min_poll_interval_s: float | None = None,
    ) -> Dict[str, Any]:
        p = Path(path)
        key = str(p.resolve())
        now = time.monotonic()
        interval = (
            max(0.01, float(min_poll_interval_s))
            if min_poll_interval_s is not None
            else self._min_poll_interval_s
        )

        with self._lock:
            cached = self._cache.get(key)
            if cached and (not force):
                age_s = now - float(cached.get("last_read_mono", 0.0))
                if age_s < interval:
                    return dict(cached.get("data") or {})

        data: Dict[str, Any] = {}
        try:
            if p.exists():
                raw = p.read_text(encoding="utf-8")
                loaded = json.loads(raw)
                if isinstance(loaded, dict):
                    data = loaded
        except Exception:
            data = {}

        with self._lock:
            self._cache[key] = {
                "last_read_mono": now,
                "data": dict(data or {}),
            }
        return data


_DEFAULT_CLIENT = RuntimeStatusClient()


def get_runtime_status_client() -> RuntimeStatusClient:
    return _DEFAULT_CLIENT
