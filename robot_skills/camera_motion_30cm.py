"""R2B4 camera motion trigger -> finite 0.30m V3 move -> Hungarian speech.

Canonical SDK owns all robot control, FULL@10Hz capture, and lifecycle.
Only calibrated JPEG frames are processed in the skill worker.  The shared
local-first TTS backend is a host-side audio consumer, NOT a motor authority.
EVI starts only after V3 exit and the exact run MCAP has been finalized.

Install: /home/alba/project_r2b4/robot_skills/camera_motion_30cm.py
Run:     r skill camera_motion_30cm --json
Safety:  run only with an operator supervising a clear test area.
"""
from __future__ import annotations

import asyncio
import io
import time
from pathlib import Path

ACTIVE_LIMIT_S = 60.0
CAPTURE_HZ = 10
SPEED_MPS = 0.30
DISTANCE_M = 0.30
SPEECH = "SIKERES SKILL VÉGREHAJTÁS"
SPEECH_BUDGET_S = 9.0
MOTION_BUDGET_S = 14.0
MIN_ACTION_WINDOW_S = 12.0
STATUS_MAX_AGE_NS = 500_000_000
OBSERVE_PERIOD_S = 0.22
MIN_INITIAL_FRAMES = 5


def _frame(jpeg):
    """Decode the canonical VisionJpeg to a cheap monochrome array."""
    import numpy as np
    from PIL import Image

    metadata = jpeg.metadata
    if not jpeg.image_bytes or metadata.stream != "lores":
        raise RuntimeError("INVALID_CANONICAL_CAMERA_FRAME")
    with Image.open(io.BytesIO(jpeg.image_bytes)) as image:
        if image.format != "JPEG":
            raise RuntimeError("CAMERA_NOT_JPEG")
        gray = image.convert("L").resize((320, 180))
        return np.asarray(gray, dtype=np.uint8).copy()


class FrameMotion:
    """Local movement indicator, not object identification or range finding.

    Adjacent frames are compared after compensating an overall exposure shift.
    A small localized change must recur twice; large/global changes are
    explicitly rejected. This is a conservative heuristic, not a proof that
    the object is a Matchbox, or that it is exactly one metre away.
    """

    def __init__(self):
        self.previous = None
        self.previous_meta = None
        self.observations = 0
        self.streak = 0
        self.max_changed_pixels = 0
        self.last_changed_pixels = 0
        self.last_changed_fraction = 0.0
        self.rejected_global = 0

    def update(self, frame, metadata):
        import numpy as np

        current_meta = (metadata.owner_generation, metadata.calibration_id,
                        metadata.source_sequence, metadata.measurement_monotonic_ns)
        if self.previous_meta is not None:
            old_generation, old_calibration, old_seq, old_ns = self.previous_meta
            if (current_meta[0] != old_generation or current_meta[1] != old_calibration):
                self.previous = None
                self.streak = 0
                self.observations = 0
            elif current_meta[2] <= old_seq or current_meta[3] <= old_ns:
                raise RuntimeError("DUPLICATE_OR_REORDERED_CAMERA_FRAME")
        prev = self.previous
        self.previous = frame
        self.previous_meta = current_meta
        self.observations += 1
        if prev is None:
            return False

        # Reject any obvious camera-frame jump before checking local motion.
        if prev.shape != frame.shape or frame.shape != (180, 320):
            raise RuntimeError("UNEXPECTED_FRAME_GEOMETRY")
        # Skip frame borders and image extremes where distortion and clipping
        # are more common; preserve the central/lower foreground at ~1 m.
        a = prev[18:174, 24:296].astype(np.int16)
        b = frame[18:174, 24:296].astype(np.int16)
        delta = b - a
        shift = int(np.median(delta))
        changed = np.abs(delta - shift) >= 24
        count = int(np.count_nonzero(changed))
        fraction = count / changed.size
        self.last_changed_pixels = count
        self.last_changed_fraction = round(fraction, 6)
        self.max_changed_pixels = max(self.max_changed_pixels, count)

        # Ignore one-pixel noise; reject broad lighting/exposure/frame changes.
        if count < 24 or fraction >= 0.075:
            if fraction >= 0.075:
                self.rejected_global += 1
            self.streak = 0
            return False

        # Local coherence: >= 8 changed pixels within a compact 8x8 tile.
        height, width = changed.shape
        blocks = changed[:height // 8 * 8, :width // 8 * 8]
        tiles = blocks.reshape(height // 8, 8, width // 8, 8).sum(axis=(1, 3))
        localized = bool(np.any(tiles >= 8))
        self.streak = self.streak + 1 if localized else 0
        return self.observations >= MIN_INITIAL_FRAMES and self.streak >= 2


async def _read(robot, resource, *, timeout_s=1.5):
    return await asyncio.wait_for(robot.read(resource), timeout=timeout_s)


def _valid_runtime(status, *, require_idle=True):
    now_ns = time.monotonic_ns()
    stamp = status.get("monotonic_ns")
    if (type(stamp) is not int or not 0 <= now_ns - stamp < STATUS_MAX_AGE_NS):
        raise RuntimeError("V3_STATUS_MISSING_OR_STALE")
    if status.get("state") != "RUNNING" or status.get("fault_layer") is not None:
        raise RuntimeError("V3_RUNTIME_FAULT")
    if status.get("ready_for_active") is not True:
        raise RuntimeError("V3_NOT_READY")
    if require_idle and status.get("enabled") is True:
        raise RuntimeError("V3_MOTORS_NOT_IDLE")
    health = {r.get("device_id"): r.get("state")
              for r in status.get("source_health", []) if isinstance(r, dict)}
    for device in ("WHEEL_ENCODERS", "BNO055_IMU", "RPLIDAR_C1"):
        if health.get(device) != "OK":
            raise RuntimeError("V3_SENSOR_NOT_OK:" + device)
    safety = status.get("safety_decision")
    if safety == "FAULT":
        raise RuntimeError("V3_SAFETY_FAULT")


async def _ensure_same_session(robot, pid):
    op = await _read(robot, "operator.status")
    if (op.get("runtime_running") is not True or op.get("runtime_pid") != pid
            or op.get("capture_mode") != "full" or op.get("capture_hz") != CAPTURE_HZ):
        raise RuntimeError("V3_RUNTIME_OR_CAPTURE_SESSION_CHANGED")
    return await _read(robot, "v3.status")


async def _wait_ready(robot, pid, deadline):
    while time.monotonic() < deadline:
        status = await _ensure_same_session(robot, pid)
        if (status.get("state") == "RUNNING" and status.get("ready_for_active") is True):
            _valid_runtime(status)
            return
        if status.get("fault_layer") or status.get("state") not in ("STARTING", "RUNNING"):
            raise RuntimeError("V3_STARTUP_FAILED")
        await asyncio.sleep(0.15)
    raise TimeoutError("V3_STARTUP_TIMEOUT")


def _speak_synchronously(root):
    # hri.report is journal-only. Use the existing host Piper/TTS + playback;
    # do not invent a non-existent Robot SDK speech action.
    from r2b4_voice.tts_provider import build_tts_client
    from r2b4_voice.voice_output import PcmWavePlayer
    tts = build_tts_client(project_root=root, resident=True)
    output = PcmWavePlayer()
    speech = tts.synthesize(SPEECH)
    backend = output.play(speech)
    return {"spoken_text": SPEECH, "player": str(backend),
            "tts_model": getattr(speech, "model", None)}


def _evi_after_shutdown(capture_path, root):
    from tools.mcap_evidence.compiler import compile_evidence
    from tools.mcap_evidence.verify import verify

    source = Path(capture_path).resolve()
    allowed = (Path(root).resolve() / "runtime" / "captures").resolve()
    if not source.is_relative_to(allowed) or source.suffix != ".mcap" or not source.is_file():
        raise RuntimeError("EVI_CAPTURE_PATH_NOT_VALID")
    output = source.with_suffix(".evidence")
    # Never overwrite the evidence of a different/previous run.
    if output.exists() or output.is_symlink():
        raise FileExistsError("EVI_OUTPUT_EXISTS:" + str(output))
    compiled = compile_evidence(source, output, workers=1)
    checked = verify(output, source=source)
    return {"path": str(output), "compiler_status": compiled.get("compiler_status"),
            "verification_status": checked.get("status"), "capture_path": str(source)}


async def run(robot):
    """Single run, one motion trigger at most, no further movement/retry."""
    start = time.monotonic()
    deadline = start + ACTIVE_LIMIT_S
    runtime_attempted = False
    runtime_stopped = False
    camera_owned = False
    capture = {}
    pid = None
    detection = FrameMotion()
    latest_capture = None
    failure = None
    motion = None
    speech = None
    outcome = "TIMEOUT"
    vision_frames = 0
    trigger_ns = None
    trigger_source_sequence = None
    active_end = None
    evi = {"status": "NOT_STARTED"}

    try:
        initial = await _read(robot, "operator.status")
        if initial.get("runtime_running") is not False:
            return {"status": "FAILED", "reason": "V3_ALREADY_RUNNING_NOT_OWNED"}

        # Start V3 and FULL flight recording before acquiring a first camera frame.
        runtime_attempted = True
        runtime = await robot.call("operator.runtime.start", capture_mode="full", capture_hz=CAPTURE_HZ)
        pid = runtime.get("pid")
        if type(pid) is not int or pid <= 0:
            raise RuntimeError("V3_START_DID_NOT_RETURN_PID")
        c = await _read(robot, "capture.status", timeout_s=8)
        if c.get("mode") != "full" or c.get("hz") != CAPTURE_HZ or not c.get("path"):
            raise RuntimeError("FULL_10_MCAP_NOT_ARMED")
        latest_capture = c["path"]
        capture = c
        await _wait_ready(robot, pid, min(deadline - 12, time.monotonic() + 9))

        # Keep one stable camera producer generation for adjacent frames.
        # Brief SDK vision.observe calls otherwise release their own demand.
        camera_before = await _read(robot, "camera.status")
        if camera_before.get("manual_demand") is not True:
            await robot.call("camera.on")
            camera_owned = True

        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                outcome = "TIMEOUT"
                break
            v3 = await _ensure_same_session(robot, pid)
            _valid_runtime(v3)
            try:
                jpeg = await asyncio.wait_for(robot.call("vision.observe", stream="lores"),
                                               timeout=min(3.0, remaining))
            except asyncio.TimeoutError as exc:
                raise RuntimeError("VISION_READ_TIMEOUT") from exc
            array = _frame(jpeg)
            vision_frames += 1
            triggered = detection.update(array, jpeg.metadata)
            if triggered and remaining >= MIN_ACTION_WINDOW_S:
                # Re-check the V3 gate at the moment the camera fires.
                status = await _ensure_same_session(robot, pid)
                _valid_runtime(status)
                trigger_ns = jpeg.metadata.measurement_monotonic_ns
                trigger_source_sequence = jpeg.metadata.source_sequence
                try:
                    await asyncio.wait_for(robot.call("hri.report", text=(
                        f"CAMERA_MOTION_TRIGGER source_sequence={trigger_source_sequence} "
                        f"measurement_ns={trigger_ns} changed_pixels={detection.last_changed_pixels} "
                        f"changed_fraction={detection.last_changed_fraction}")), timeout=1.0)
                except Exception:
                    pass
                # Finite navigation is the single motion authority. The action
                # itself always issues STOP before returning, successful or not.
                motion = await asyncio.wait_for(
                    robot.call("v3.command.move_relative", forward_m=DISTANCE_M,
                               max_v_mps=SPEED_MPS, capture=False,
                               capture_mode="full", capture_hz=CAPTURE_HZ,
                               finite_timeout_s=min(MOTION_BUDGET_S, max(1.0, deadline - time.monotonic() - SPEECH_BUDGET_S)),
                               session_watchdog_s=MOTION_BUDGET_S + 2.0),
                    timeout=min(MOTION_BUDGET_S + 3.0, max(1.0, deadline - time.monotonic() - 2.0)),
                )
                if not isinstance(motion, dict) or (motion.get("status"), motion.get("reason")) != ("COMPLETED", "COMPLETE"):
                    raise RuntimeError("FINITE_MOTION_NOT_VERIFIED:" + repr(motion)[:400])
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("SPEECH_DEADLINE_REACHED")
                speech = await asyncio.wait_for(
                    asyncio.to_thread(_speak_synchronously, robot.root),
                    timeout=min(SPEECH_BUDGET_S, remaining),
                )
                # A report is evidence, not a substitute for audio.
                try:
                    await asyncio.wait_for(robot.call("hri.report", text=SPEECH), timeout=1.0)
                except Exception:
                    pass
                outcome = "COMPLETED"
                break
            await asyncio.sleep(min(OBSERVE_PERIOD_S, max(0.0, deadline - time.monotonic())))
    except asyncio.CancelledError:
        raise
    except Exception as exc:
        failure = f"{type(exc).__name__}: {exc}"[:500]
        outcome = "FAILED"
    finally:
        active_end = time.monotonic()
        if camera_owned:
            try:
                await robot.call("camera.off")
            except Exception as exc:
                failure = ((failure + "; ") if failure else "") + "CAMERA_SHUTDOWN:" + str(exc)[:180]
                outcome = "FAILED"
        if runtime_attempted:
            try:
                # Stop the actual motors, terminate resident V3 and finalize MCAP.
                # robot.stop() would revoke this very skill before postprocessing.
                await robot.call("operator.runtime.stop")
                op = await _read(robot, "operator.status", timeout_s=5)
                runtime_stopped = op.get("runtime_running") is False
                if not runtime_stopped:
                    raise RuntimeError("RUNTIME_STOP_NOT_CONFIRMED")
                final = await _read(robot, "capture.status", timeout_s=12)
                if latest_capture and final.get("path") != latest_capture:
                    raise RuntimeError("CAPTURE_IDENTITY_CHANGED")
                capture = final
            except Exception as exc:
                failure = ((failure + "; ") if failure else "") + "SHUTDOWN:" + f"{type(exc).__name__}:{exc}"[:400]
                outcome = "FAILED"
                # Canonical emergency preemption; can also revoke this worker.
                if not runtime_stopped:
                    try:
                        await robot.stop()
                    except Exception:
                        pass

    # Offline step is outside the physical one-minute active-run budget.
    if runtime_stopped and capture.get("path") and capture.get("state") in ("FINALIZED", "FINALIZED_INCOMPLETE"):
        try:
            evi = await asyncio.to_thread(_evi_after_shutdown, capture["path"], robot.root)
            evi["status"] = "COMPLETE" if (evi["compiler_status"] == "COMPLETE"
                                            and evi["verification_status"] == "PASS") else "PARTIAL"
        except Exception as exc:
            evi = {"status": "FAILED", "reason": f"{type(exc).__name__}:{exc}"[:500],
                   "capture_path": capture.get("path")}
    elif latest_capture:
        evi = {"status": "NOT_STARTED", "reason": "MCAP_NOT_FINALIZED_OR_RUNTIME_NOT_STOPPED",
               "capture_path": latest_capture}

    if not failure and outcome == "COMPLETED" and (
        evi.get("status") != "COMPLETE" or capture.get("complete") is not True
    ):
        outcome = "PARTIAL"

    return {
        "status": "FAILED" if failure else outcome,
        "reason": failure or ("NO_MOTION_DETECTED_BEFORE_DEADLINE" if outcome == "TIMEOUT" else None),
        "active_elapsed_s": round((active_end or time.monotonic()) - start, 3),
        "vision_frames": vision_frames,
        "detector": {"trigger_ns": trigger_ns, "trigger_source_sequence": trigger_source_sequence,
                     "max_changed_pixels": detection.max_changed_pixels,
                     "last_changed_fraction": detection.last_changed_fraction,
                     "rejected_global": detection.rejected_global},
        "motion": motion, "speech": speech, "runtime_pid": pid,
        "runtime_stopped": runtime_stopped, "capture": capture, "evi": evi,
        "note": "Camera motion indicates image change, not a known object or measured object range.",
    }
