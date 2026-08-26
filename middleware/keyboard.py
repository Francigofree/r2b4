#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import select
import termios
import tty
import threading
import queue
import time

# Importáljuk az állapotokat a parancsok leképezéséhez
from state import RobotState
from config_manager import config as global_config
from controller.routines import EMERGENCY_STOP_REASON_SPACE

class AlbaKeyboard(threading.Thread):
    """
    Modern, háttérben futó billentyűzet kezelő SSH-hoz.
    Kezeli a nyíl billentyűket és egyéb escape szekvenciákat is.
    """
    def __init__(self):
        super().__init__(name="KeyboardThread", daemon=True)
        self.is_atty = sys.stdin.isatty()
        self.fd = sys.stdin.fileno() if self.is_atty else None
        self.old_settings = None
        self.key_queue = queue.Queue()
        self.running = False
        
        # Speciális billentyűk térképe
        self.KEY_MAP = {
            '\x1b[A': 'up',
            '\x1b[B': 'down',
            '\x1b[C': 'right',
            '\x1b[D': 'left',
            '\x1b': 'esc',
            ' ': 'space'
        }

    def __enter__(self):
        self.start_capture()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop_capture()

    def start_capture(self):
        if self.is_atty and not self.running:
            self.old_settings = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
            self.running = True
            self.start()

    def stop_capture(self):
        self.running = False
        if self.is_atty and self.old_settings:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def run(self):
        """A háttérszál loopja."""
        while self.running:
            try:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    char = sys.stdin.read(1)
                    if char == '\x1b':
                        time.sleep(0.01)
                        if select.select([sys.stdin], [], [], 0)[0]:
                            char += sys.stdin.read(2)
                    
                    final_key = self.KEY_MAP.get(char, char)
                    self.key_queue.put(final_key)
            except Exception:
                break

    def get_key(self):
        try:
            return self.key_queue.get_nowait()
        except queue.Empty:
            return None

    def get_all_keys(self):
        keys = []
        while not self.key_queue.empty():
            keys.append(self.key_queue.get())
        return keys

    def dispatch_commands(self, robot):
        """
        Központosított parancs-leképezés. 
        A robot példányt paraméterként kapja meg, és közvetlenül vezérli azt.
        """
        for key_raw in self.get_all_keys():
            key = key_raw.lower() if isinstance(key_raw, str) else key_raw
            # Valódi manuális input időbélyeg: csak tényleges billentyű esetén frissül.
            robot.last_manual_input_ts = time.monotonic()
            # Full log (L): billentyű input – funkció/input tesztekhez
            if hasattr(robot, "logger") and hasattr(robot.logger, "log_full_extra"):
                robot.logger.log_full_extra("KEY", key=key)
            # Segéd: egységes beállítás
            def set_speed_level(level):
                if hasattr(robot, "set_speed_level"):
                    robot.set_speed_level(level, source="MANUAL")
                else:
                    level = max(-9, min(9, int(level)))
                    robot.mark_input("MANUAL")
                    robot.speed_level = level
                    robot.default_speed_level = level
                    global_config.set_path(
                        "vezerles",
                        "sebesseg_kezeles.alap_fokozat",
                        level,
                        persist=True
                    )

                    if level > 0:
                        robot.sm.transition_to(RobotState.FORWARD)
                    elif level < 0:
                        robot.sm.transition_to(RobotState.BACKWARD)
                    else:
                        robot.v_target = 0.0
                        robot.omega_target = 0.0
                        robot.turn_level = 0
                        robot.sm.transition_to(RobotState.IDLE)

            # Sebességváltó (0-9)
            if key.isdigit():
                set_speed_level(int(key))
                continue

            # Előre / Hátra (Sebesség fokozatok)
            if key in ['w', 'up']: 
                robot.mark_input("MANUAL")
                set_speed_level(robot.speed_level + 1)
            elif key in ['s', 'down']: 
                robot.mark_input("MANUAL")
                set_speed_level(robot.speed_level - 1)
            
            # Fordulás (Intenzitás szabályozás)
            elif key in ['a', 'left']:
                # Balra: Csökkentjük a szintet (negatív irány)
                if hasattr(robot, "set_turn"):
                    new_turn = max(-9, min(9, robot.turn_level - 1))
                    robot.set_turn(new_turn, source="MANUAL")
                else:
                    robot.sm.transition_to(RobotState.ROTATE) # Manuális mód (nincs delta)
                    robot.mark_input("MANUAL")
                    robot.turn_level = max(-9, min(9, robot.turn_level - 1))
                
            elif key in ['d', 'right']:
                # Jobbra: Növeljük a szintet (pozitív irány)
                if hasattr(robot, "set_turn"):
                    new_turn = max(-9, min(9, robot.turn_level + 1))
                    robot.set_turn(new_turn, source="MANUAL")
                else:
                    robot.sm.transition_to(RobotState.ROTATE) # Manuális mód (nincs delta)
                    robot.mark_input("MANUAL")
                    robot.turn_level = max(-9, min(9, robot.turn_level + 1))
                
            # --- SPACE = VÉSZLEÁLLÍTÁS + TELJES RESET (betonozott, mindig él) ---
            # Biztonságkritikus: SPACE mindig azonnali megállás + reset (motor 0, EKF, LIDAR, kamera, minden stop).
            elif key in ['space', ' ']:
                if hasattr(robot, "input_vector"):
                    robot.input_vector = {"x": 0.0, "y": 0.0}
                if hasattr(robot, "set_motion_source"):
                    robot.set_motion_source("MANUAL")
                if hasattr(robot, "strong_reset"):
                    robot.strong_reset(reason=EMERGENCY_STOP_REASON_SPACE)
                else:
                    robot._emergency_stop(reason=EMERGENCY_STOP_REASON_SPACE) 
            elif key == 'p':
                # P = Fotó készítése (közepes felbontás, Pic/, megjelenik app 6. oldalán)
                if getattr(robot, "following_active", False) and hasattr(robot, "stop_following"):
                    robot.stop_following()
                if hasattr(robot, "capture_photo"):
                    robot.capture_photo(resolution_preset="kozepes")
                else:
                    if hasattr(robot, "set_motion_source"):
                        robot.set_motion_source("STATE")
                    robot.sm.transition_to(RobotState.PATROL)
                    robot.speed_level = 5
            elif key == 'v':
                # V = Videó felvétel start/stop toggle (vid/, max 5 perc)
                if getattr(robot, "following_active", False) and hasattr(robot, "stop_following"):
                    robot.stop_following()
                if hasattr(robot, "toggle_video_recording"):
                    robot.toggle_video_recording()
                else:
                    robot.logger.info("Videó felvétel nem elérhető (toggle_video_recording hiányzik).")
            elif key == 'f':
                # F = Ember követése BE/KI (kamera + LIDAR fúzió)
                if hasattr(robot, "toggle_following"):
                    robot.toggle_following()
                else:
                    robot.logger.info("Ember követés nem elérhető (toggle_following hiányzik).")
            elif key == 'k': 
                robot.full_calibration()
            elif key == 'r':
                # R = EKF reset + pozíció nullázás (odometria, EKF x/y/theta 0)
                if hasattr(robot, "reset_position"):
                    robot.reset_position()
            elif key == 'h':
                # H = KERESD AZ EMBERT: 360° forgatás, kis felbontás, alacsony fps; első ember → STOP + "EMBER"
                if hasattr(robot, "start_search_person"):
                    robot.start_search_person()
                else:
                    if getattr(robot.sm, "get_current_state_name", lambda: "NONE")() != "IDLE":
                        robot.logger.warn("LLM indítás csak IDLE módban lehetséges (H).")
                        continue
                    robot.brain.toggle_listening(source="MANUAL")
            elif key == 'l':
                if hasattr(robot, "toggle_full_log"):
                    robot.toggle_full_log()
                else:
                    robot.logger.info("Szimulált LLM parancs triggerelve: [STOP]")
                    robot.brain._execute_brain_command("Kérésre azonnal megállok. [STOP]")
            elif key == 'n' or key_raw == 'Q':
                robot.logger.info("1m négyzet mozgás indítása (N / Shift+Q).")
                if hasattr(robot, "start_square"):
                    robot.start_square(source="MANUAL")
            elif key == 'o':
                # O = KARIKA: 60cm sugarú kör, visszatérés kiindulási pontra (ugyanaz, mint a GUI "KARIKA (O)" gomb)
                if hasattr(robot, "start_circle"):
                    robot.start_circle(source="MANUAL")
                else:
                    robot.logger.info("Kör rutin nem elérhető (start_circle hiányzik).")
            elif key in ['x', 'esc']:
                robot.logger.info("Kilépés parancs (X) érzékelve.")
                robot.running = False

# --- Teszt üzemmód ---
if __name__ == "__main__":
    print("AlbaKeyboard Teszt...")
    with AlbaKeyboard() as kb:
        while True:
            key = kb.get_key()
            if key:
                print(f"Key: {key}")
                if str(key).lower() == 'x': break
            time.sleep(0.01)
