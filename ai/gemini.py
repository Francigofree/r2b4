#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from pathlib import Path
from google import genai
from google.genai import types
from config_manager import config as global_config

PROJECT_ROOT = Path(__file__).resolve().parents[1]
PROJECT_SYSTEM_PROMPT_PATH = PROJECT_ROOT / "project_rules" / "agent_system_prompt.txt"


def _load_project_system_prompt(max_chars: int = 12000) -> str:
    """
    Projekt-szintű, külső system prompt betöltése.
    Tokenkímélés miatt limitáljuk a hosszt.
    """
    try:
        if not PROJECT_SYSTEM_PROMPT_PATH.exists():
            return ""
        text = PROJECT_SYSTEM_PROMPT_PATH.read_text(encoding="utf-8").strip()
        if not text:
            return ""
        if len(text) > int(max_chars):
            return text[: int(max_chars)].rstrip()
        return text
    except Exception:
        return ""


class AlbaGemini:
    """
    Project Alba - Gemini AI Interfész (Modernizált 2026 Verzió).
    Kezeli a robot személyiségét (System Prompt) és a Google API kommunikációt.
    """
    def __init__(self, config_path=None):
        # A config_manager automatikusan betölt mindent, a path argumentum már nem szükséges,
        # de a kompatibilitás miatt meghagyjuk.
        self.cfg = global_config.get("intelligencia", "gemini_agy", {})
            
        # -----------------------------------------------------------
        # API KULCS KEZELÉS
        # -----------------------------------------------------------
        self.api_key = os.environ.get("API_KEY", "AIzaSyB7Kxs3Yc4NHWXqvjVmIdJBfaxC2Y4-t3E")

        if not self.api_key:
            print("[FIGYELEM] Az API kulcs nincs beállítva!")

        try:
            self.client = genai.Client(api_key=self.api_key)
        except Exception as e:
            print(f"[HIBA] Nem sikerült a klienst elindítani: {e}")
        
        self.model_name = self.cfg.get("modell", "models/gemma-3-27b-it")
        self.robot_prefix = self.cfg.get("robot_mod_prefix", "[R]")
        self.project_system_prompt = _load_project_system_prompt()

    def ask(self, prompt: str, attachment_data: bytes = None, mime_type: str = None) -> str:
        """
        Kérdés küldése a modellnek.
        Automatikus rendszerutasítás választás a prompt tartalma alapján.
        """
        # JAVÍTÁS: Nem startswith, hanem tartalmazás vizsgálat, 
        # mivel a Brain modul a ROBOT_STATE JSON-t a prompt elejére fűzi.
        is_robot_mode = self.robot_prefix in prompt

        # Hibrid: parancs → csak JSON; kérdés/chat → sima szöveg (magyarul)
        json_only_instruction = (
            "Te egy segítőkész asszisztens vagy, aki robotot is tud irányítani. "
            "Ha a felhasználó üzenete egyértelműen robot parancs (pl. menj előre, állj meg, fordulj jobbra, menj 1 métert, közelíts), "
            "válaszolj CSAK egyetlen JSON objektummal, semmi más szöveggel, magyarázat nélkül. "
            "Érvényes séma: {\"intent\": \"MOVE\" | \"ROTATE\" | \"FOLLOW\" | \"STOP\" | \"QUERY\" | \"SEQUENCE\", "
            "\"params\": {\"distance_m\": number vagy null, \"angle_deg\": number vagy null, \"target_id\": string vagy null}, "
            "SEQUENCE esetén \"params\": {\"steps\": [{\"intent\": \"MOVE\"|\"ROTATE\"|\"FOLLOW\", \"params\": {...}}, ...]}, "
            "\"confidence\": 0 és 1 közötti szám}. "
            "MOVE=egyenes mozgás (distance_m méter), ROTATE=fordulás helyben (angle_deg), FOLLOW=közelítés, STOP=végszünet, QUERY=nincs mozgás. "
            "Ha a felhasználó üzenete kérdés, köszönés, vagy általános csevegés (pl. 'mi a neved?', 'hogy vagy?', 'köszönöm', 'mi az idő?'), "
            "válaszolj normál magyar szöveggel, NEM JSON-nel, barátságosan és röviden."
        )
        default_std_instr = "Te egy segítőkész asszisztens vagy."

        if is_robot_mode:
            system_instruction = json_only_instruction  # STRICTLY JSON-ONLY; no config override
        else:
            system_instruction = self.cfg.get("rendszer_utasitas_alap", default_std_instr)
        if self.project_system_prompt:
            system_instruction = f"{self.project_system_prompt}\n\n{system_instruction}"

        # Gemma modellek (pl. gemma-3-27b-it) nem támogatják a system_instruction paramétert külön
        is_gemma = "gemma" in self.model_name.lower()

        if is_gemma:
            # Ha Gemma, a system promptot a user prompt elé fűzzük ("Hard Prompting")
            final_prompt = f"{system_instruction}\n\n{prompt}"
            config = types.GenerateContentConfig()
        else:
            final_prompt = prompt
            config = types.GenerateContentConfig(system_instruction=system_instruction)

        # Tartalom összeállítása
        contents = [final_prompt]
        if attachment_data and mime_type:
            try:
                contents.append(types.Part.from_bytes(data=attachment_data, mime_type=mime_type))
            except Exception as e:
                print(f"[HIBA] Csatolmány hozzáadása sikertelen: {e}")

        try:
            response = self.client.models.generate_content(
                model=self.model_name,
                config=config,
                contents=contents
            )
            return response.text
            
        except Exception as e:
            error_msg = f"Hiba történt: {str(e)}"
            print(f"[GEMINI HIBA] {error_msg}")
            return f"{error_msg} [STOP]" if is_robot_mode else error_msg

if __name__ == "__main__":
    print("Alba Gemini Interfész Teszt (Q a kilépéshez)")
    ai = AlbaGemini()
    print(f"Modell: {ai.model_name}")
    
    while True:
        try:
            user_in = input("\nÜzenet: ")
        except EOFError: break
        if user_in.lower() == 'q': break
        print(f"\nGemini válasza:\n{'-'*20}\n{ai.ask(user_in)}\n{'-'*20}")
