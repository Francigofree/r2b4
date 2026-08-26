
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import threading
import queue
import numpy as np
import sounddevice as sd
from scipy.signal import butter, lfilter
from faster_whisper import WhisperModel

# ANSI Színek a visszajelzéshez
CLR_VOICE = "\033[92m"  # Zöld
CLR_AI = "\033[94m"     # Kék
CLR_WARN = "\033[93m"   # Sárga
CLR_ERR = "\033[91m"    # Piros
CLR_RESET = "\033[0m"

class AlbaVoiceManager:
    """
    Aszinkron hangfeldolgozó egység Project Alba robothoz.
    Whisper AI alapú parancsértelmezés dedikált szállal.
    """
    def __init__(self, model_size="small", language="hu", callback=None):
        print(f"{CLR_AI}[AI] Whisper Engine inicializálása ({model_size})...{CLR_RESET}")
        
        # Modell betöltése (RPi5 optimalizált: int8 + 4 threads)
        self.model = WhisperModel(
            model_size, 
            device="cpu", 
            compute_type="int8", 
            cpu_threads=4,
            #download_root="./models"
        )
        
        self.language = language
        self.callback = callback # Függvény, amit meghívunk a felismert szöveggel
        
        self.samplerate = 16000
        self.is_recording = False
        self.running = True
        
        # Puffer és szálkezelés
        self.audio_queue = queue.Queue()
        self.processing_thread = threading.Thread(target=self._worker, daemon=True)
        self.processing_thread.start()
        
        # Sávszűrő együtthatók (80Hz - 7500Hz)
        nyq = 0.5 * self.samplerate
        self.b, self.a = butter(3, [80/nyq, 7500/nyq], btype="band")

    def _filter_audio(self, data):
        """Alapvető zajszűrés a tisztább felismerésért."""
        return lfilter(self.b, self.a, data).astype(np.float32)

    def _audio_callback(self, indata, frames, time_info, status):
        """A mikrofonból érkező nyers adatok elkapása."""
        if self.is_recording:
            self.current_buffer.append(indata.copy())

    def start_recording(self):
        """Felvétel indítása (puffer ürítése)."""
        if not self.is_recording:
            self.current_buffer = []
            self.is_recording = True
            print(f"{CLR_VOICE}[VOICE] Hallgatózás...{CLR_RESET}")

    def stop_recording(self):
        """Felvétel lezárása és küldése a feldolgozó szálnak."""
        if self.is_recording:
            self.is_recording = False
            if self.current_buffer:
                raw_audio = np.concatenate(self.current_buffer).flatten()
                self.audio_queue.put(raw_audio)
                print(f"{CLR_AI}[AI] Feldolgozás alatt...{CLR_RESET}")

    def _worker(self):
        """Dedikált szál a transzkripcióhoz, hogy ne blokkolja a robotot."""
        while self.running:
            try:
                # Várakozás audio adatra a sorban
                audio_data = self.audio_queue.get(timeout=0.5)
                
                # Előfeldolgozás (szűrés + normalizálás)
                audio_data = self._filter_audio(audio_data)
                max_val = np.max(np.abs(audio_data))
                if max_val > 0:
                    audio_data /= max_val
                
                # Transzkripció
                start_t = time.perf_counter()
                segments, info = self.model.transcribe(
                    audio_data, 
                    language=self.language,
                    beam_size=1,
                    initial_prompt="Robot irányítás: előre, hátra, állj, járőrözés."
                )
                
                text = " ".join(seg.text for seg in segments).strip()
                dt = time.perf_counter() - start_t
                
                if text:
                    print(f"{CLR_VOICE}>>> Felismert: '{text}' ({dt:.2f}s){CLR_RESET}")
                    if self.callback:
                        self.callback(text)
                else:
                    print(f"{CLR_WARN}[!] Nem sikerült felismerni beszédet.{CLR_RESET}")
                
                self.audio_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                print(f"{CLR_ERR}[HIBA] AI feldolgozási hiba: {e}{CLR_RESET}")

    def run_standalone(self):
        """Teszt üzemmód: Space billentyűvel indítható/leállítható felvétel."""
        import sys
        import tty
        import termios

        print(f"\n{CLR_AI}--- ALBA VOICE STANDALONE TEST ---{CLR_RESET}")
        print("Nyomj SPACE-t a felvételhez, Q-t a kilépéshez!")

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        
        try:
            tty.setcbreak(fd)
            with sd.InputStream(samplerate=self.samplerate, channels=1, callback=self._audio_callback):
                while self.running:
                    ch = sys.stdin.read(1)
                    if ch == " ":
                        if not self.is_recording:
                            self.start_recording()
                        else:
                            self.stop_recording()
                    elif ch.lower() == "q":
                        self.running = False
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

if __name__ == "__main__":
    # Egyszerű parancsértelmező példa
    def simple_cmd_handler(text):
        t = text.lower()
        if "előre" in t: print(">>> PARANCS: Mozgás előre")
        elif "hátra" in t: print(">>> PARANCS: Mozgás hátra")
        elif "állj" in t: print(">>> PARANCS: Megállás")

    manager = AlbaVoiceManager(callback=simple_cmd_handler)
    manager.run_standalone()
