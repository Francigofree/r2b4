#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Joy illesztő réteg: harmonikus differenciál mozgás joystick vezérléssel.
- Konfigolható max omega; a rámpázás kizárólag a közös MotionController dolga.
- turn_mix bevonása a joy forgásába.
- Egyetlen UNIFIED joystick→twist leképezés.
- Küszöb: |x|,|y| < 0.01 → 0,0.
A control_loop GUI_JOYSTICK forrásnál ezt a réteget hívja; a MotionExecutor továbbra is v_cmd, omega_cmd → PWM.

Tesztelés: Indítsd a vezérlőt (os.py), nyisd meg a GUI-t (FastAPI vagy app), moogasd a joystickot.
A runtime/status.json-ban joy_adapter_active: true lesz joy használatkor. Konfig: conf/vezerles.json → mozgas
(joy_max_omega_rad_s).
"""


# Küszöb: ennél kisebb stick érték = nulla (beragadás elkerülés)
JOY_ZERO_THRESHOLD = 0.01


def compute(ctrl, x: float, y: float, dt: float):
    """
    Joy (x, y) ∈ [-1, 1] → (v_target, omega_target) harmonikus differenciál vezérléshez.
    Használja: ctrl.turn_mix és a joy adapter max-omega konfigurációját.
    Az adapter állapotmentes; a közös MotionController végzi a shapinget és slew-t.
    """
    cfg = getattr(ctrl, "joy_adapter_cfg", None) or {}
    max_omega = float(cfg.get("joy_max_omega_rad_s", 1.2))
    turn_mix = getattr(ctrl, "turn_mix", 1.0)

    if abs(x) < JOY_ZERO_THRESHOLD and abs(y) < JOY_ZERO_THRESHOLD:
        return 0.0, 0.0

    limits = getattr(ctrl, "speed_limits", None)
    motion_src = str(getattr(ctrl, "motion_command_source", "") or "")
    if limits is not None:
        max_v = float(limits.effective_v_max)
        max_w_profile = float(getattr(limits, "effective_w_max", getattr(limits.profile, "w_max", max_omega)))
        max_omega = max(0.01, min(float(max_omega), float(max_w_profile)))
        if max_v <= 0.0:
            # GUI analóg joy esetén a speed_level=0 nem blokkolhatja a mozgást.
            # Ilyenkor az egyetlen UNIFIED profil plafonja marad érvényes.
            if motion_src in ("GUI_JOYSTICK", "MANUAL"):
                max_v = float(limits.profile.v_max)
            else:
                max_v = 0.0
    else:
        max_v = ctrl.speeds_fwd.get(9, 0.3)

    v_des = y * max_v
    # x: balra pozitív → negatív omega (balra fordulás)
    omega_des_raw = -x * abs(x) * max_omega * turn_mix

    return v_des, omega_des_raw
