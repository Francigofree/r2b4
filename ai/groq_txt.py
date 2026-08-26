#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import io
try:
    from groq import Groq
except ImportError:
    Groq = None
from config_manager import config as global_config

class AlbaGroqSTT:
    """
    Groq Cloud STT Interfész (Whisper).
    """
    def __init__(self, config_path=None):
        self.config = global_config.get("intelligencia", "groq_stt", {})
        
        # API Kulcs és Modell
        self.api_key = "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"
        self.model = self.config.get("modell", "whisper-large-v3")
        self.client = None

        if Groq is None:
            print("[FIGYELEM] A 'groq' csomag hiányzik!")
        elif not self.api_key:
            print("[FIGYELEM] Nincs Groq API kulcs!")
        else:
            try:
                self.client = Groq(api_key=self.api_key)
            except Exception as e:
                print(f"[HIBA] Groq kliens hiba: {e}")

    def transcribe(self, audio_bytes: bytes) -> str | None:
        if not self.client:
            return None
        try:
            audio_file = io.BytesIO(audio_bytes)
            audio_file.name = "recording.wav"
            
            transcription = self.client.audio.transcriptions.create(
                file=(audio_file.name, audio_file.read()),
                model=self.model,
                language="hu",
                response_format="text"
            )
            return transcription.strip()
            
        except Exception as e:
            print(f"[HIBA] Groq STT hiba: {e}")
            return None
