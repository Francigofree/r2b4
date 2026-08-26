
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sounddevice as sd
import numpy as np
import threading
import time

class SunFounderMic:
    def __init__(self, samplerate=16000, blocksize=1024):
        self.samplerate = samplerate
        self.blocksize = blocksize
        self.volume = 0.0
        self.is_running = False
        self._stream = None
        self._callback = None
        self._raw_callback = None
        self._lock = threading.Lock()

    def _audio_callback(self, indata, frames, time_info, status):
        if status:
            print(f"[WARN] {status}")
        
        # RMS számítás hangerőhöz
        rms = np.sqrt(np.mean(indata**2))
        with self._lock:
            self.volume = rms
            
        # Hangerő esemény
        if self._callback:
            self._callback(rms)
            
        # Nyers adat továbbítása (ha kérték) - pl. Brain modulnak
        if self._raw_callback:
            self._raw_callback(indata.copy(), status)

    def start(self, event_callback=None, raw_callback=None):
        """
        Indítja a mikrofon figyelést.
        :param event_callback: RMS (hangerő) változáskor hívódik meg.
        :param raw_callback: Nyers audio chunk-okat kap (numpy array).
        """
        if self.is_running:
            return
        self._callback = event_callback
        self._raw_callback = raw_callback
        
        self._stream = sd.InputStream(
            channels=1,
            samplerate=self.samplerate,
            blocksize=self.blocksize,
            callback=self._audio_callback
        )
        self._stream.start()
        self.is_running = True
        # print("SunFounder mikrofon driver engedélyezve.") # Log tisztítása miatt kikommentelve

    def stop(self):
        if not self.is_running:
            return
        self._stream.stop()
        self._stream.close()
        self._stream = None
        self.is_running = False
        # print("SunFounder mikrofon driver leállítva.") # Log tisztítása miatt kikommentelve

    def read_volume(self):
        with self._lock:
            return self.volume

# ===== Példa használat =====
if __name__ == "__main__":
    def volume_event(rms_value):
        print(f"[Event] Hangerő: {rms_value:.4f}")

    mic = SunFounderMic()
    mic.start(event_callback=volume_event)

    try:
        while True:
            print(f"Polling: {mic.read_volume():.4f}")
            time.sleep(0.2)  # 5 Hz frissítés
    except KeyboardInterrupt:
        mic.stop()
