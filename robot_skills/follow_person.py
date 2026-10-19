"""Follow the person selected by live V3 perception for a finite interval.

No historical person/location fact can substitute for a current detector track.
Target loss uses the canonical zero-motion hold and never starts room searching.
"""
from robot_skills.room_cruise import _run_timed_behavior, _validate_duration


async def run(robot, duration_s=300, max_v_mps=None, max_omega_rad_s=None,
              capture_mode=None, capture_hz=None):
    _validate_duration(duration_s)
    # Detector readiness is preparation. A cold camera/model must not consume
    # the requested following interval; the owning SDK session releases demand
    # on success, error or cancellation. No images enter the host/control path.
    async with robot.observe(object_kind="person", ready_timeout_s=15):
        return await _run_timed_behavior(robot, "follow_person", duration_s, max_v_mps,
                                         max_omega_rad_s, capture_mode, capture_hz)
