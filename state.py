#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from enum import Enum
from dataclasses import dataclass
import os
import time

# =========================
# Állapot enum
# =========================

class RobotState(Enum):
    IDLE = 0
    FORWARD = 1
    BACKWARD = 2
    CALIBRATING = 3
    FAILSAFE = 4
    ROTATE = 5
    PATROL = 6
    APPROACH = 7  # Precíziós megközelítés
    DOCK = 8      # Precíz dokkolás/final approach
    FOLLOW = 9    # Adaptív mozgás (ember követés): v/omega a high-level rétegből
    CIRCLE = 10   # Kör megrajzolása: folyamatos jobbra mozgás, visszatérés kiindulási pozícióba
    ARC = 11      # Ívmozgás: megadott sugár, szög, sebesség paraméterekkel


# =========================
# State kimeneti struktúra
# =========================

@dataclass
class StateResult:
    request_state: RobotState | None = None
    v_target: float | None = None
    omega_target: float | None = None
    track_left_mps: float | None = None
    track_right_mps: float | None = None
    flags: dict | None = None


# =========================
# Alap State osztály
# =========================

class AlbaState:
    def __init__(self, machine):
        self.machine = machine
        self.name = self.__class__.__name__

    def on_enter(self, **kwargs):
        pass

    def on_exit(self):
        pass

    def update(self, dt, robot) -> StateResult:
        return StateResult()


# =========================
# Állapotgép
# =========================

class StateMachine:
    def __init__(self, robot):
        self.robot = robot
        self.states = {}
        self.current_state: AlbaState | None = None
        self.current_enum: RobotState | None = None
        self.time_in_state = 0.0
        self.dynamic_states = {} # Registry for LLM-generated behaviors

    def load_dynamic_scripts(self):
        """
        Dinamikus szkriptek betöltése a runtime/scripts mappából.
        Az LLM-alapú fejlesztési mód (PRO) számára.
        """
        import importlib.util
        import glob
        import sys
        
        script_dir = os.path.join(os.path.dirname(__file__), "runtime", "scripts")
        if not os.path.exists(script_dir):
            return

        for script_path in glob.glob(os.path.join(script_dir, "*.py")):
            try:
                module_name = os.path.basename(script_path)[:-3]
                
                # Hot-reload támogatás: ha már be van töltve, töröljük a sys.modules-ból
                if module_name in sys.modules:
                    del sys.modules[module_name]
                
                spec = importlib.util.spec_from_file_location(module_name, script_path)
                module = importlib.util.module_from_spec(spec)
                sys.modules[module_name] = module # Regisztráció
                spec.loader.exec_module(module)
                
                # Keressünk AlbaState leszármazottat
                for attr_name in dir(module):
                    attr = getattr(module, attr_name)
                    if (isinstance(attr, type) and issubclass(attr, AlbaState) 
                        and attr is not AlbaState):
                        # Force update: felülírjuk a régi instance-t
                        self.dynamic_states[module_name] = attr(self)
                        print(f"[FSM] Dinamikus állapot betöltve/frissítve: {module_name}")
            except Exception as e:
                print(f"[FSM] Hiba a {script_path} betöltésekor: {e}")

    def add_state(self, state_enum: RobotState, state_instance: AlbaState):
        self.states[state_enum] = state_instance

    def get_current_state_name(self) -> str:
        """Állapot megjelenítendő neve: enum (RobotState) vagy dinamikus (str) egyaránt."""
        e = self.current_enum
        if e is None:
            return "NONE"
        return getattr(e, "name", None) or str(e)

    def transition_to(self, state_enum: RobotState | str, **kwargs):
        force_reenter = bool(kwargs.pop("force_reenter", False))
        if isinstance(state_enum, str):
            if state_enum not in self.dynamic_states:
                print(f"[FSM] Ismeretlen dinamikus állapot: {state_enum}")
                return
            target_instance = self.dynamic_states[state_enum]
            state_name = state_enum
        else:
            if state_enum not in self.states:
                print(f"[FSM] Ismeretlen vagy nincs regisztrálva állapot: {state_enum}")
                return
            target_instance = self.states[state_enum]
            state_name = state_enum.name

        # Ha már ebben az állapotban vagyunk, ne lépjünk be újra
        # (megelőzi az on_exit/on_enter/audit spam-et ismétlődő parancsoknál).
        if self.current_enum == state_enum and not force_reenter:
            return

        if self.current_state:
            self.current_state.on_exit()

        self.current_enum = state_enum
        self.current_state = target_instance
        self.time_in_state = 0.0

        print(f"\033[94m[STATE] -> {state_name}\033[0m")
        if hasattr(self.robot, "telemetry"):
            self.robot.telemetry.emit_audit(
                "STATE_TRANSITION",
                "FSM",
                details={"state": state_name}
            )
        self.current_state.on_enter(**kwargs)

    def update(self, dt: float):
        if not self.current_state:
            return

        self.time_in_state += dt
        result = self.current_state.update(dt, self.robot)

        if not result:
            return

        if result.request_state:
            self.robot.state_track_reference = {"left_mps": None, "right_mps": None}
            self.transition_to(result.request_state)
            return

        if result.v_target is not None:
            self.robot.v_target = result.v_target

        if result.omega_target is not None:
            self.robot.omega_target = result.omega_target

        self.robot.state_track_reference = {
            "left_mps": result.track_left_mps,
            "right_mps": result.track_right_mps,
        }

        if result.flags:
            self.robot.flags.update(result.flags)


# =========================
# IDLE
# =========================

class IdleState(AlbaState):
    def on_enter(self, **kwargs):
        # IDLE belépéskor PWM nullázás (beragadt parancsok elkerülésére)
        try:
            if hasattr(self.robot, "motor_l") and self.robot.motor_l:
                self.robot.motor_l.set_pwm(0.0)
                self.robot.motor_l.stop()
            if hasattr(self.robot, "motor_r") and self.robot.motor_r:
                self.robot.motor_r.set_pwm(0.0)
                self.robot.motor_r.stop()
        except Exception:
            pass

    def update(self, dt, robot):
        return StateResult(
            v_target=0.0,
            flags={
                "lidar_enable": False,
                "motors_enable": False
            }
        )


# =========================
# FORWARD
# =========================

class ForwardState(AlbaState):
    def on_enter(self, **kwargs):
        pass

    def update(self, dt, robot):
        if robot.lidar_summary.get("blocked_front", False):
            return StateResult(
                request_state=RobotState.IDLE,
                flags={"speed_level_reset": True}
            )

        level = max(0, min(9, abs(robot.speed_level)))
        return StateResult(
            v_target=robot.speeds_fwd[level],
            flags={"lidar_enable": True}
        )


# =========================
# BACKWARD
# =========================

class BackwardState(AlbaState):
    def on_enter(self, **kwargs):
        pass

    def update(self, dt, robot):
        if robot.lidar_summary.get("blocked_back", False):
            return StateResult(
                request_state=RobotState.IDLE,
                flags={"speed_level_reset": True}
            )

        level = max(0, min(9, abs(robot.speed_level)))
        return StateResult(
            v_target=-robot.speeds_rev[level],
            flags={"lidar_enable": True}
        )


# =========================
# DOCK
# =========================

class DockState(AlbaState):
    def on_enter(self, **kwargs):
        pass

    def update(self, dt, robot):
        direction = 1 if getattr(robot, "dock_dir", 1) >= 0 else -1
        level = max(0, min(9, abs(getattr(robot, "dock_speed_level", 1))))

        if direction > 0 and robot.lidar_summary.get("blocked_front", False):
            return StateResult(
                request_state=RobotState.IDLE,
                flags={"speed_level_reset": True}
            )
        if direction < 0 and robot.lidar_summary.get("blocked_back", False):
            return StateResult(
                request_state=RobotState.IDLE,
                flags={"speed_level_reset": True}
            )

        if direction > 0:
            v_target = robot.speeds_fwd[level]
        else:
            v_target = -robot.speeds_rev[level]

        return StateResult(
            v_target=v_target,
            flags={"lidar_enable": True}
        )


# =========================
# ROTATE
# =========================

class RotateState(AlbaState):
    def on_enter(self, **kwargs):
        self.delta = kwargs.get('delta', 0)
        self.target_heading_deg = kwargs.get("target_heading_deg")
        self.manual_mode = (self.delta == 0 and self.target_heading_deg is None)
        self.use_heading_controller = False
        self.heading_controller = getattr(self.machine.robot, "heading_controller", None)

        if not self.manual_mode:
            curr = self.machine.robot.ekf.get_state()
            if self.target_heading_deg is None:
                self.target_heading_deg = (curr["theta_deg"] + self.delta + 360.0) % 360.0
            else:
                self.target_heading_deg = float(self.target_heading_deg) % 360.0

            if self.heading_controller is not None:
                try:
                    self.heading_controller.start(
                        target_heading_deg=self.target_heading_deg,
                        current_heading_deg=float(curr.get("theta_deg", 0.0)),
                        pose_x=float(curr.get("x", 0.0)),
                        pose_y=float(curr.get("y", 0.0)),
                        source=str(getattr(self.machine.robot, "motion_command_source", "STATE") or "STATE"),
                        settle_tolerance_deg=kwargs.get("tolerance_deg"),
                        settle_time_s=kwargs.get("settle_time_s"),
                        max_duration_s=kwargs.get("max_duration_s"),
                        speed_level=kwargs.get("speed_level"),
                    )
                    self.use_heading_controller = True
                except Exception:
                    self.use_heading_controller = False

    def on_exit(self):
        if self.use_heading_controller and self.heading_controller is not None:
            try:
                if self.heading_controller.status().get("active"):
                    self.heading_controller.cancel(reason="SAFETY_ABORT")
            except Exception:
                pass

    def update(self, dt, robot):
        if self.manual_mode:
            if robot.turn_level == 0:
                return StateResult(v_target=0.0, omega_target=0.0)
            
            # Multi-level turn handling: magnitude comes from turn_level
            direction = -1.0 if robot.turn_level < 0 else 1.0
            turn_mag = max(0, min(9, abs(robot.turn_level)))
            
            # Apply turn_min_level if set (ensure minimum rotation speed)
            min_level = max(0, min(9, int(getattr(robot, "turn_min_level", 0) or 0)))
            if min_level:
                turn_mag = max(turn_mag, min_level)
                
            omega = robot.turn_omega_levels.get(turn_mag, 0.0) * direction
            
            # Kérés: terminál A és D gomb (MANUAL) esetén 65%-ra csökkentés
            if getattr(robot, "motion_command_source", None) == "MANUAL":
                # Itt a 65% a kényelmesebb terminál vezetéshez kell
                omega *= 0.65
                
            return StateResult(v_target=0.0, omega_target=omega)

        if self.use_heading_controller and self.heading_controller is not None:
            curr = robot.ekf.get_state()
            tick_out = self.heading_controller.tick(
                current_heading_deg=float(curr.get("theta_deg", 0.0)),
                pose_x=float(curr.get("x", 0.0)),
                pose_y=float(curr.get("y", 0.0)),
                v_l_raw=float(getattr(robot, "_last_v_l_raw", 0.0)),
                v_r_raw=float(getattr(robot, "_last_v_r_raw", 0.0)),
                gyro_z_rad_s=float(getattr(robot, "_last_gyro_z_rad", 0.0)),
                lidar_status=dict(getattr(robot, "lidar_odom_runtime_status", {}) or {}),
                odometry_mode=str(getattr(robot, "odometry_mode", "LIDAR_FIRST") or "LIDAR_FIRST"),
                dt=float(dt),
                now=time.monotonic(),
            )
            if tick_out is None:
                return StateResult(request_state=RobotState.IDLE, v_target=0.0, omega_target=0.0)
            if tick_out.get("done"):
                return StateResult(request_state=RobotState.IDLE, v_target=0.0, omega_target=0.0)
            track_ref = dict(tick_out.get("track_reference") or {})
            return StateResult(
                v_target=float(tick_out.get("v_target", 0.0)),
                omega_target=float(tick_out.get("omega_target", 0.0)),
                track_left_mps=track_ref.get("left_mps"),
                track_right_mps=track_ref.get("right_mps"),
            )

        curr = robot.ekf.get_state()
        err = (self.target_heading_deg - curr["theta_deg"] + 180) % 360 - 180

        if abs(err) < 5.0:
            return StateResult(request_state=RobotState.IDLE, v_target=0.0, omega_target=0.0)

        level = max(0, min(9, abs(robot.speed_level)))
        min_level = max(0, min(9, int(getattr(robot, "turn_min_level", 0) or 0)))
        if min_level:
            level = max(level, min_level)
        if level == 0:
            return StateResult(request_state=RobotState.IDLE, v_target=0.0, omega_target=0.0)
        base = robot.turn_omega_levels.get(level, 0.8)
        omega = base if err > 0 else -base
        return StateResult(v_target=0.0, omega_target=omega)


# =========================
# PATROL
# =========================

class PatrolState(AlbaState):
    def on_enter(self, **kwargs):
        self.phase = 0
        self.target_angle = None

    def update(self, dt, robot):
        curr = robot.ekf.get_state()
        l_sum = robot.lidar_summary

        if self.phase == 0:
            if l_sum.get("blocked_front", False):
                self.phase = 1
                self.target_angle = (curr["theta_deg"] + 115) % 360
                return StateResult(v_target=0.0)

            level = max(0, min(9, abs(robot.speed_level)))
            return StateResult(
                v_target=robot.speeds_fwd[level],
                flags={"lidar_enable": True}
            )

        elif self.phase == 1:
            err = (self.target_angle - curr["theta_deg"] + 180) % 360 - 180

            if abs(err) < 5.0:
                self.phase = 0
                return StateResult(v_target=0.0, flags={"lidar_enable": True})

            level = max(0, min(9, abs(robot.speed_level)))
            if level == 0:
                return StateResult(request_state=RobotState.IDLE, v_target=0.0, omega_target=0.0, flags={"lidar_enable": True})
            base = robot.turn_omega_levels.get(level, 0.8)
            omega = base if err > 0 else -base
            return StateResult(
                v_target=0.0,
                omega_target=omega,
                flags={"lidar_enable": True}
            )


# =========================
# FOLLOW (Adaptive motion: human-following)
# =========================

class FollowState(AlbaState):
    """
    High-level adaptive motion: v_target and omega_target are supplied by the
    adaptive controller (e.g. follower). This state does NOT overwrite them;
    it preserves whatever the main loop set from get_adaptive_command().
    """
    def on_enter(self, **kwargs):
        pass

    def update(self, dt, robot):
        # Do not set v_target/omega_target; leave them from adaptive layer.
        return StateResult(flags={"lidar_enable": True})


# =========================
# CIRCLE (Kör megrajzolása)
# =========================

class CircleState(AlbaState):
    """
    Kör megrajzolása: folyamatos jobbra mozgás (v_target + omega_target),
    visszatérés kiindulási pozícióba (EKF alapján).
    
    Paraméterek:
    - circle_radius_m: kör sugara (0.3m = 60cm átmérő / 2)
    - circle_v_m_s: lineáris sebesség (0.15 m/s, lassú)
    - circle_omega_rad_s: szögsebesség (v / r = 0.15 / 0.3 = 0.5 rad/s)
    - start_x, start_y: kiindulási pozíció (EKF)
    - return_threshold_m: visszatérés küszöbérték (0.1m)
    """
    def on_enter(self, **kwargs):
        # Kiindulási pozíció mentése
        ekf_state = self.machine.robot.ekf.get_state()
        self.start_x = ekf_state.get("x", 0.0)
        self.start_y = ekf_state.get("y", 0.0)
        self.start_time = self.machine.time_in_state
        
        # Kör paraméterek
        self.circle_radius_m = 0.3  # 60cm átmérő / 2
        self.circle_v_m_s = 0.15   # Lassú, egyenletes sebesség
        self.circle_omega_rad_s = self.circle_v_m_s / self.circle_radius_m  # 0.5 rad/s
        self.return_threshold_m = 0.1  # Visszatérés küszöbérték
        
        if hasattr(self.machine.robot, "logger"):
            self.machine.robot.logger.info(
                f"[CIRCLE] Kör indítva: r={self.circle_radius_m:.2f}m, "
                f"v={self.circle_v_m_s:.2f}m/s, omega={self.circle_omega_rad_s:.2f}rad/s"
            )

    def on_exit(self):
        """Kör vége: mozgásforrás formálisan vissza MANUAL (arbiter)."""
        try:
            from controller.commands import set_motion_source
            set_motion_source(self.machine.robot, "MANUAL")
        except Exception:
            pass

    def update(self, dt, robot):
        # LIDAR akadály ellenőrzés
        if robot.lidar_summary.get("blocked_front", False):
            if hasattr(robot, "logger"):
                robot.logger.warn("[CIRCLE] Akadály észlelve, leállítás.")
            return StateResult(
                request_state=RobotState.IDLE,
                v_target=0.0,
                omega_target=0.0
            )
        
        # Aktuális pozíció (EKF)
        ekf_state = robot.ekf.get_state()
        curr_x = ekf_state.get("x", 0.0)
        curr_y = ekf_state.get("y", 0.0)
        
        # Visszatérés ellenőrzése: távolság kiindulási pozíciótól
        dx = curr_x - self.start_x
        dy = curr_y - self.start_y
        dist_from_start = (dx**2 + dy**2)**0.5
        
        # Ha visszaértünk a kiindulási pozícióba (és legalább 1 másodperc eltelt)
        if dist_from_start < self.return_threshold_m and self.machine.time_in_state > 1.0:
            if hasattr(robot, "logger"):
                robot.logger.info(
                    f"[CIRCLE] Kör befejezve: visszatérés {dist_from_start:.3f}m távolságra "
                    f"({self.machine.time_in_state:.1f}s alatt)"
                )
            return StateResult(
                request_state=RobotState.IDLE,
                v_target=0.0,
                omega_target=0.0
            )
        
        # Folyamatos kör mozgás: előre + jobbra fordulás
        return StateResult(
            v_target=self.circle_v_m_s,
            omega_target=self.circle_omega_rad_s,  # Pozitív = jobbra
            flags={"lidar_enable": True}
        )


# =========================
# ARC (Ívmozgás primitív)
# =========================

class ArcState(AlbaState):
    """
    Arc motion primitive: v/omega from arc_controller each tick.
    The ArcController is accessed via robot.arc_controller.
    """
    def on_enter(self, **kwargs):
        try:
            self.machine.robot.arc_runtime_status = {}
        except Exception:
            pass

    def on_exit(self):
        try:
            from controller.commands import set_motion_source
            set_motion_source(self.machine.robot, "MANUAL")
        except Exception:
            pass
        try:
            arc_ctrl = getattr(self.machine.robot, "arc_controller", None)
            if arc_ctrl is not None:
                self.machine.robot.arc_runtime_status = dict(arc_ctrl.status() or {})
        except Exception:
            pass

    def update(self, dt, robot):
        arc_ctrl = getattr(robot, "arc_controller", None)
        if arc_ctrl is None or not arc_ctrl.active:
            try:
                if arc_ctrl is not None:
                    robot.arc_runtime_status = dict(arc_ctrl.status() or {})
            except Exception:
                pass
            return StateResult(request_state=RobotState.IDLE, v_target=0.0, omega_target=0.0)

        if robot.lidar_summary.get("blocked_front", False):
            arc_ctrl.cancel()
            try:
                robot.arc_runtime_status = dict(arc_ctrl.status() or {})
                robot.arc_runtime_status["reason"] = "blocked_front_cancelled"
            except Exception:
                pass
            return StateResult(request_state=RobotState.IDLE, v_target=0.0, omega_target=0.0)

        ekf_state = robot.ekf.get_state()
        v_cmd, omega_cmd, done, status = arc_ctrl.tick(
            ekf_state,
            dt,
            gyro_z_rad_s=getattr(robot, "_last_gyro_z_rad", None),
        )
        try:
            runtime_status = dict(status or {})
            runtime_status.setdefault("mode", "FOLLOW_ARC")
            robot.arc_runtime_status = runtime_status
            behavior = dict(getattr(robot, "behavior_motion_status", {}) or {})
            if str(behavior.get("mode", "")).strip().upper() == "FOLLOW_ARC":
                behavior.update(
                    {
                        "arc_inner_track_min_mps": runtime_status.get("arc_inner_track_min_mps"),
                        "arc_track_ratio": runtime_status.get("arc_track_ratio"),
                        "arc_pivot_like_samples": runtime_status.get("arc_pivot_like_samples"),
                        "arc_inner_track_positive_ratio": runtime_status.get("arc_inner_track_positive_ratio"),
                        "arc_sample_count": runtime_status.get("arc_sample_count"),
                    }
                )
                robot.behavior_motion_status = behavior
        except Exception:
            pass
        if done:
            try:
                robot.arc_runtime_status = dict(arc_ctrl.status() or {})
                robot.arc_runtime_status.update(dict(status or {}))
            except Exception:
                pass
            return StateResult(request_state=RobotState.IDLE, v_target=0.0, omega_target=0.0)
        return StateResult(v_target=v_cmd, omega_target=omega_cmd, flags={"lidar_enable": True})


# =========================
# FAILSAFE (Vészhelyzeti mód)
# =========================

class FailSafeState(AlbaState):
    """Vészleállítás utáni állapot: motor 0, mozgás tiltva. Feloldás: full_reset / strong_reset → IDLE."""

    def on_enter(self, **kwargs):
        try:
            if hasattr(self.robot, "motor_l") and self.robot.motor_l:
                self.robot.motor_l.set_pwm(0.0)
                self.robot.motor_l.stop()
            if hasattr(self.robot, "motor_r") and self.robot.motor_r:
                self.robot.motor_r.set_pwm(0.0)
                self.robot.motor_r.stop()
        except Exception:
            pass

    def update(self, dt, robot):
        return StateResult(
            v_target=0.0,
            omega_target=0.0,
            flags={
                "lidar_enable": False,
                "motors_enable": False
            }
        )


# =========================
# CALIBRATING
# =========================

class CalibratingState(AlbaState):
    def update(self, dt, robot):
        return StateResult(
            v_target=0.0,
            flags={"motors_enable": False, "calibration_active": True}
        )

# =========================
# APPROACH (Precíziós megközelítés)
# =========================

class ApproachState(AlbaState):
    def on_enter(self, **kwargs):
        # Konfiguráció betöltése
        app_cfg = self.machine.robot.cfg.get("vezerles", {}).get("kozelites", {})
        self.target_dist = app_cfg.get("cel_tavolsag_m", 0.03)
        self.slow_speed_lvl = app_cfg.get("sebesseg_fokozat", 1)
        self.timeout = app_cfg.get("max_ido_sec", 15.0)
        self.start_time = 0.0 # Will be set in update or handled via dt
        
        # Sebesség beállítása lassúra
        self.machine.robot.speed_level = max(0, min(9, int(self.slow_speed_lvl)))

    def update(self, dt, robot):
        # Timeout védelem
        if self.machine.time_in_state > self.timeout:
            print("[APPROACH] Timeout.")
            return StateResult(request_state=RobotState.IDLE, v_target=0.0)

        l_sum = robot.lidar_summary
        current_dist = l_sum.get("min_dist", 10.0)
        
        # Távolság ellenőrzése
        if current_dist <= self.target_dist:
            print(f"[APPROACH] Cél elérve: {current_dist:.3f}m")
            return StateResult(request_state=RobotState.IDLE, v_target=0.0)
        
        # Haladás (SecurityGuard ezt a state-et átengedi)
        level = max(0, min(9, int(self.slow_speed_lvl)))
        return StateResult(
            v_target=robot.speeds_fwd[level],
            flags={"lidar_enable": True}
        )
