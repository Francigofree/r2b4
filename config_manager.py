#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import sys
import time

# Színkódok a konzolhoz (ha hiba van a betöltésnél)
CLR_ERR = "\033[91m"
CLR_RESET = "\033[0m"

class AlbaConfig:
    """
    Központi konfiguráció kezelő osztály.
    Feladata: A src/conf/ mappában lévő darabolt JSON fájlok (fizika, vezérlés, hardver, intelligencia)
    betöltése és egyetlen, könnyen elérhető adatszerkezetté (szótárrá) összefűzése.
    """
    def __init__(self):
        # A konfig mappa abszolút elérési útja
        self.conf_dir = os.path.join(os.path.dirname(__file__), "conf")
        self._data = {}
        self._files = {
            "fizika": "fizika.json",
            "vezerles": "vezerles.json",
            "hardver": "hardver.json",
            "intelligencia": "intelligencia.json",
            "speed_map": "speed_map.json",
            "control_mode": "control_mode.json",
            "security": "security.json",
            "cam": "cam.json",
        }
        self._mtimes = {}
        self._last_check = 0.0
        self.auto_reload = True
        self.reload_interval_sec = 0.5
        self.load_all()

    def load_all(self):
        """Minden konfigurációs modul betöltése."""
        for key, filename in self._files.items():
            self._data[key] = self._load_json(filename)
            self._mtimes[filename] = self._get_mtime(filename)

    def reload(self):
        """Kézi újratöltés az összes JSON-ra."""
        self.load_all()

    def path(self, filename):
        """Konfig fájl abszolút elérési útja a conf mappában."""
        return os.path.join(self.conf_dir, filename)

    def _load_json(self, filename):
        """Egyedi JSON fájl beolvasása hibakezeléssel."""
        path = os.path.join(self.conf_dir, filename)
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except FileNotFoundError:
            print(f"{CLR_ERR}[KONFIG HIBA] A '{filename}' fájl nem található a {self.conf_dir} mappában!{CLR_RESET}")
            return {}
        except json.JSONDecodeError as e:
            print(f"{CLR_ERR}[KONFIG HIBA] A '{filename}' fájl hibás (szintaxis hiba): {e}{CLR_RESET}")
            return {}

    def get(self, section, key=None, default=None):
        """Biztonságos érték lekérdezés."""
        self._maybe_reload()
        if section not in self._data:
            return default
        if key is None:
            return self._data[section]
        return self._data[section].get(key, default)

    def set_path(self, section, key_path, value, persist=False):
        """Érték beállítása (opcionálisan azonnali mentéssel)."""
        self._maybe_reload()
        if section not in self._data or not isinstance(self._data[section], dict):
            self._data[section] = {}

        keys = key_path.split(".") if isinstance(key_path, str) else list(key_path)
        if not keys:
            return False
        node = self._data[section]
        for k in keys[:-1]:
            if k not in node or not isinstance(node[k], dict):
                node[k] = {}
            node = node[k]
        node[keys[-1]] = value

        if persist:
            filename = self._files.get(section)
            return self._save_json(filename, self._data[section])
        return True

    def _get_mtime(self, filename):
        path = os.path.join(self.conf_dir, filename)
        try:
            return os.path.getmtime(path)
        except OSError:
            return None

    def _save_json(self, filename, data):
        if not filename:
            return False
        path = self.path(filename)
        tmp_path = f"{path}.tmp"
        try:
            with open(tmp_path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            os.replace(tmp_path, path)
            self._mtimes[filename] = self._get_mtime(filename)
            return True
        except Exception as e:
            print(f"{CLR_ERR}[KONFIG HIBA] Nem sikerült menteni: {filename} ({e}){CLR_RESET}")
            return False

    def _maybe_reload(self):
        if not self.auto_reload:
            return
        now = time.monotonic()
        if (now - self._last_check) < self.reload_interval_sec:
            return
        self._last_check = now

        for key, filename in self._files.items():
            mtime = self._get_mtime(filename)
            if self._mtimes.get(filename) != mtime:
                self._data[key] = self._load_json(filename)
                self._mtimes[filename] = mtime

    @property
    def data(self):
        """Teljes konfig szótár (hot-reload biztosítva)."""
        self._maybe_reload()
        return self._data

# Globális példány, hogy ne kelljen mindenhol újra példányosítani
config = AlbaConfig()
