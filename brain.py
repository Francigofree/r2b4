#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import io
import json
import wave
import time
import datetime
import threading
import numpy as np
import os
from typing import Optional

from ai.gemini import AlbaGemini
from driver.microphone import SunFounderMic
from ai.txt_hang import AlbaTTS
from ai.groq_txt import AlbaGroqSTT
from config_manager import config as global_config
from log.runtime_debug import append_line

def timestamp():
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]


def _extract_json_only(raw: str) -> str:
    """Strip markdown/code fences so only JSON is passed to Core. No parsing here."""
    s = (raw or "").strip()
    if s.startswith("```"):
        parts = s.split("```")
        if len(parts) >= 2:
            inner = parts[1].strip()
            if inner.lower().startswith("json"):
                inner = inner[4:].lstrip()
            s = inner
    return s.strip()


def _tts_message_for_json(json_str: str) -> Optional[str]:
    """Short Hungarian TTS message from valid JSON intent; no motor/state changes."""
    try:
        d = json.loads(json_str)
        intent = (d.get("intent") or "").upper()
        if intent == "STOP":
            return "Megállok."
        if intent == "MOVE":
            return "Megyek."
        if intent == "ROTATE":
            return "Fordulok."
        if intent == "FOLLOW":
            return "Közelítek."
        if intent == "QUERY":
            return "Rendben."
        return "Parancs fogadva."
    except Exception:
        return None


def log(msg):
    print(f"\033[96m[{timestamp()}]\033[0m {msg}")

class AlbaBrain:
    """
    A robot 'AGY' modulja.
    Összefogja a hallást (Mic), a beszédértést (STT), a gondolkodást (Gemini)
    és a beszédet (TTS).
    """
    def __init__(self, controller):
        self.controller = controller
        
        # Konfiguráció betöltése az új 'intelligencia' szekcióból
        ai_cfg = global_config.get("intelligencia", default={})
        
        audio_cfg = ai_cfg.get("hang_bemenet", {})
        self.sample_rate = audio_cfg.get("mintavetelezes_hz", 16000)
        self.max_duration = audio_cfg.get("max_felvetel_hossz_sec", 8.0)
        
        gemini_cfg = ai_cfg.get("gemini_agy", {})
        self.robot_prefix = gemini_cfg.get("robot_mod_prefix", "[R]")

        self.ai = AlbaGemini()
        self.groq = AlbaGroqSTT()
        self.mic = SunFounderMic(samplerate=self.sample_rate)
        self.tts = AlbaTTS()
        
        self.is_listening = False
        self.audio_chunks = []
        self.stop_timer = None

        # --- DUMALOG INIT ---
        ts_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        log_dir = os.path.join(os.path.dirname(__file__), "log", "dumalog")
        os.makedirs(log_dir, exist_ok=True)
        self.log_file = os.path.join(log_dir, f"dumalog_{ts_str}.txt")
        self._log_chat("SYSTEM", "Beszélgetés naplózása elindítva.")

    def _log_chat(self, direction, text):
        try:
            ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
            clean_text = text.replace('\n', ' ').strip()
            append_line(self.log_file, f"[{ts}] [{direction}] {clean_text}")
        except Exception as e:
            print(f"[LOG HIBA] Nem sikerült írni a logfájlba: {e}")

    def toggle_listening(self, source="MANUAL"):
        if self.is_listening:
            self.stop_listening_and_think()
        else:
            self.start_listening(source=source)

    def start_listening(self, source="MANUAL"):
        if source != "MANUAL":
            log("LLM indítás csak manuális forrásból engedélyezett.")
            return
        if getattr(self.controller.sm, "get_current_state_name", lambda: "NONE")() != "IDLE":
            log("LLM indítás csak IDLE módban engedélyezett.")
            return
        if not self.is_listening:
            self.audio_chunks = []
            self.is_listening = True
            log(f"\033[93m>>> Hangrögzítés INDULT (Max {self.max_duration}s)...\033[0m")
            self.stop_timer = threading.Timer(self.max_duration, self._timeout_handler)
            self.stop_timer.start()
            self.mic.start(raw_callback=self._process_raw_audio)

    def _timeout_handler(self):
        if self.is_listening:
            log(f"Időkorlát elérve, automata leállítás.")
            self.stop_listening_and_think()

    def stop_listening_and_think(self):
        if self.stop_timer:
            self.stop_timer.cancel()
            self.stop_timer = None

        if self.is_listening:
            self.is_listening = False
            
            # Adatok másolása a háttérszálnak
            chunks_to_process = list(self.audio_chunks)
            self.audio_chunks = []
            
            # Feldolgozás külön szálon, hogy ne akasszuk meg a robotot
            threading.Thread(target=self._process_audio_thread, args=(chunks_to_process,), daemon=True).start()

    def _process_audio_thread(self, chunks):
        try:
            self.mic.stop()
            
            if not chunks:
                log("HIBA: Üres hangfelvétel.")
                return

            log("Audio feldolgozása (WAV konverzió)...")
            data = np.concatenate(chunks)
            if data.dtype == np.float32:
                data = (data * 32767).astype(np.int16)

            wav_io = io.BytesIO()
            with wave.open(wav_io, 'wb') as wav_file:
                wav_file.setnchannels(1)
                wav_file.setsampwidth(2)
                wav_file.setframerate(self.mic.samplerate)
                wav_file.writeframes(data.tobytes())
            
            wav_bytes = wav_io.getvalue()
            response_text = ""
            
            log(f"Küldés Groq STT-nek ({len(wav_bytes)} bájt)...")
            transcribed_text = self.groq.transcribe(wav_bytes)

            if transcribed_text:
                clean_txt = transcribed_text.strip()
                log(f"\033[95m[GROQ STT]\033[0m {clean_txt}")
                self._log_chat("INPUT", clean_txt)
                
                if len(clean_txt) < 5:
                    log(f"\033[93m[SZŰRÉS] Túl rövid, figyelmen kívül hagyva.\033[0m")
                    return 

                # Állapotküldés LLM felé: állapotcsomag a promptba + képernyőre
                state_packet = self.controller.get_llm_state_packet()
                state_json_full = json.dumps(state_packet, indent=2, ensure_ascii=False)
                log("\033[94m[ÁLLAPOTKÜLDÉS LLM felé]\033[0m (teljes JSON):")
                print(state_json_full)
                state_json_inline = json.dumps(state_packet, ensure_ascii=False)
                prompt = (
                    f"ROBOT_STATE: {state_json_inline}\n\n"
                    f"{self.robot_prefix} {clean_txt}\n\n"
                    "Ha robot parancs: csak egy JSON objektum (intent, params, confidence). Ha kérdés vagy chat: normál magyar szöveg."
                )
                try:
                    response_text = self.ai.ask(prompt)
                except Exception as e:
                    log(f"\033[91m[HIBA] Gemini API hiba: {e}\033[0m")
            else:
                log("\033[91m[HIBA] STT sikertelen.\033[0m")

            if response_text:
                raw_response = response_text
                response_text = _extract_json_only(response_text)
                log(f"\033[92m[GEMINI VÁLASZ]\033[0m {response_text}")
                self._log_chat("OUTPUT", response_text)

                is_robot_command = False
                try:
                    data = json.loads(response_text)
                    if isinstance(data, dict) and "intent" in data:
                        is_robot_command = True
                except (json.JSONDecodeError, TypeError):
                    pass

                if is_robot_command:
                    if hasattr(self.controller, "core"):
                        self.controller.core.process_input(response_text)
                        log("Parancs feldolgozva (Core, JSON).")
                    else:
                        log("Parancs elutasítva: canonical Core nem érhető el.")
                    tts_msg = _tts_message_for_json(response_text)
                else:
                    tts_msg = (raw_response or response_text).strip()
                    if not tts_msg:
                        tts_msg = response_text.strip()
                    log("Sima válasz (chat), nincs parancs.")

                if tts_msg:
                    log("TTS felolvasás...")
                    self.tts.say(tts_msg)
            
            log("Várakozás 'H' gombra...")

        except Exception as e:
            log(f"\033[91m[THREAD ERROR] {e}\033[0m")

    def _process_raw_audio(self, indata, status):
        if self.is_listening:
            self.audio_chunks.append(indata)

    def _execute_brain_command(self, text: str):
        core = getattr(self.controller, "core", None)
        if core is None:
            raise RuntimeError("canonical_core_unavailable")
        core.process_input(text)
