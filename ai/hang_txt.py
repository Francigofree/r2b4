#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import tty
import time
import termios
import signal
import threading
import warnings
import numpy as np
import sounddevice as sd
from scipy.signal import butter, lfilter
from faster_whisper import WhisperModel
from log.runtime_debug import append_line

# --------- ENV / LOG CSENDESÍTÉS ----------
os.environ["OTEL_SDK_DISABLED"] = "true"
os.environ["ONNXRUNTIME_LOGGING_LEVEL"] = "3"
warnings.filterwarnings("ignore")
# -----------------------------------------


# ---------------- CONFIG ------------------
SAMPLE_RATE = 16000
LOG_FILE = "felismert_szoveg.txt"

MODEL_SIZE = "small"          # RPi5-re optimális
COMPUTE_TYPE = "int8"

BEAM_SIZE = 1
BEST_OF = 1

INITIAL_PROMPT = (
    "Magyar nyelvű robot parancsok. "
    "Kulcsszavak: ALBA, HÉJ, méter, centi, járőrözés, követés, térkép. "
)

HP_FREQ = 80
LP_FREQ = 7500
FILTER_ORDER = 3
# ------------------------------------------


def bandpass_filter(data, lowcut=HP_FREQ, highcut=LP_FREQ,
                    fs=SAMPLE_RATE, order=FILTER_ORDER):
    nyq = 0.5 * fs
    b, a = butter(order, [lowcut / nyq, highcut / nyq], btype="band")
    return lfilter(b, a, data).astype(np.float32)


class AlbaSTT:

    def __init__(self):
        print("\n--- ALBA PROJECT STT (RPi5 · Whisper tiny · HU) ---")

        self.model = WhisperModel(
            MODEL_SIZE,
            device="cpu",
            compute_type=COMPUTE_TYPE,
            cpu_threads=4,            
        )

        self.is_recording = False
        self.audio_buffer = []
        self.running = True

        signal.signal(signal.SIGINT, self.handle_exit)

    def handle_exit(self, sig, frame):
        print("\nKilépés...")
        self.running = False
        sys.exit(0)

    def audio_callback(self, indata, frames, time_info, status):
        if self.is_recording:
            self.audio_buffer.append(indata.copy())

    def key_listener(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)

        try:
            while self.running:
                ch = sys.stdin.read(1)
                if ch == " ":
                    if not self.is_recording:
                        self.audio_buffer = []
                        self.is_recording = True
                        print("\r\033[91m[FELVÉTEL...]\033[0m", end="", flush=True)
                    else:
                        self.is_recording = False
                        print("\r[FELDOLGOZÁS...]", end="", flush=True)
                        self.process_audio()
                elif ch.lower() == "q":
                    self.running = False
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def process_audio(self):
        if not self.audio_buffer:
            return

        raw_audio = np.concatenate(self.audio_buffer).flatten().astype(np.float32)

        if len(raw_audio) < SAMPLE_RATE // 2:
            print("\n[?] Túl rövid felvétel.")
            return

        audio = bandpass_filter(raw_audio)

        max_amp = np.max(np.abs(audio))
        if max_amp > 0:
            audio /= max_amp

        audio = np.clip(audio * 1.05, -1.0, 1.0).astype(np.float32)

        print("\n" + "-" * 40)
        print("[AI] Magyar beszédfelismerés...")
        start_time = time.time()

        segments, _ = self.model.transcribe(
            audio,
            language="hu",
            task="transcribe",
            beam_size=BEAM_SIZE,
            best_of=BEST_OF,
            temperature=0.0,
            word_timestamps=False,
            suppress_blank=True,

            # VAD kikapcsolva – bandpass már szűr
            vad_filter=False,

            # Zaj / hallucináció csökkentés
            no_speech_threshold=0.6,

            initial_prompt=INITIAL_PROMPT,
            condition_on_previous_text=False,
        )

        text = " ".join(seg.text for seg in segments).strip()
        runtime = time.time() - start_time

        if len(text) > 2:
            print(f"\n\033[92m>>> {text}\033[0m ({runtime:.1f}s)")
            ts = time.strftime("%Y-%m-%d %H:%M:%S")
            append_line(LOG_FILE, f"[{ts}] {text}")
        else:
            print("\n[?] Nincs értelmezhető beszéd.")

        print("-" * 40)

    def run(self):
        print("Készen áll. [SPACE]=Felvétel | [Q]=Kilépés | [Ctrl+C]=Azonnali kilépés")

        key_thread = threading.Thread(target=self.key_listener, daemon=True)
        key_thread.start()

        with sd.InputStream(
            samplerate=SAMPLE_RATE,
            channels=1,
            dtype="float32",
            callback=self.audio_callback
        ):
            while self.running:
                time.sleep(0.1)


if __name__ == "__main__":
    AlbaSTT().run()
