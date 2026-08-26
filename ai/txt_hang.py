#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import re
import threading
import queue
import subprocess
from config_manager import config as global_config

class AlbaTTS:
    """
    Text-to-Speech modul (espeak). Nem-blocking: say() csak beteszi a szöveget a sorba,
    a _worker szál futtatja az espeak-et; a vezérlőciklus soha nem vár a TTS-re.
    A beállításokat a conf/intelligencia.json fájlból olvassa.
    (Formális követelmény: TTS soha ne blokkolja a main/control loop-ot.)
    """
    def __init__(self, config_path=None):
        # Konfiguráció betöltése
        self.config = global_config.get("intelligencia", "felolvasas_tts", {})

        # Magyar kulcsok használata
        self.lang_code = self.config.get("nyelv_kod", "hu")
        self.rate = self.config.get("sebesseg", 160)
        self.volume = self.config.get("hangero", 1.0)
        self.enabled = self.config.get("engedelyezve", True)

        self.queue = queue.Queue()
        self.running = True
        
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def say(self, text):
        if not self.enabled or not text:
            return

        # Szögletes zárójeles parancsok törlése a kimondott szövegből
        clean_text = re.sub(r'\[.*?\]', '', text, flags=re.DOTALL)
        clean_text = " ".join(clean_text.split())
        
        if not clean_text:
            return

        self.queue.put(clean_text)

    def _worker(self):
        while self.running:
            try:
                text = self.queue.get(timeout=0.5)
                try:
                    cmd = [
                        'espeak', 
                        '-v', self.lang_code, 
                        '-s', str(self.rate), 
                        '-a', str(int(self.volume * 200)), 
                        text
                    ]
                    subprocess.run(cmd, stderr=subprocess.DEVNULL)
                except FileNotFoundError:
                    print("\033[91m[TTS HIBA] 'espeak' nincs telepítve!\033[0m")
                except Exception as e:
                    print(f"\033[91m[TTS HIBA] {e}\033[0m")

                self.queue.task_done()
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"\033[91m[TTS WORKER HIBA] {e}\033[0m")

    def stop(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)

if __name__ == "__main__":
    tts = AlbaTTS()
    tts.say("TTS rendszer tesztelése.")
