"""R2B4: stationary 20-second low-light camera comparison skill.

Install to /home/alba/project_r2b4/robot_skills/camera_dark_test.py.
Run: r skill camera_dark_test --json

Photos use ONLY the public robot.call('vision.observe') SDK path. Video uses
robot.call('camera.video'). Speech uses the project's existing TTS + speaker
implementation because the public Robot SDK has no spoken-audio action.

IMPORTANT: JPEG quality, brightness and contrast are POST-PROCESSING settings,
not camera shutter/exposure/analogue gain controls. The camera owner's actual
sensor acquisition settings are not changed by this skill.
"""

import asyncio
from datetime import datetime, timezone
import hashlib
from io import BytesIO
import json
from pathlib import Path
import time
from uuid import uuid4


ROOT = Path('/home/alba/project_r2b4')
CAPTURE_SECONDS = 20.0
# Offset from capture-window start; images also retain their untouched original.
# Each video is exactly 2 s requested duration; encoding/IPC can take longer.
PLAN = (
    (0.0, 'photo', 'lores', 35, 1.0, 1.0),
    (1.0, 'photo', 'main', 90, 1.0, 1.0),
    (2.5, 'video', 'low_bitrate', 1_000_000, None, None),
    (6.0, 'photo', 'lores', 80, 2.0, 1.1),
    (7.5, 'video', 'high_bitrate', 6_000_000, None, None),
    (11.0, 'photo', 'main', 95, 3.0, 1.2),
    (13.0, 'photo', 'main', 60, 1.0, 2.0),
    (15.0, 'photo', 'lores', 90, 4.0, 1.0),
    (17.0, 'photo', 'main', 95, 1.0, 1.0),
    (18.4, 'photo', 'lores', 50, 1.5, 2.0),
)


def _write_json(path, value):
    with path.open('x', encoding='utf-8') as handle:
        json.dump(value, handle, ensure_ascii=False, indent=2, allow_nan=False)
        handle.write('\n')


def _new_output_dir(output_dir):
    if output_dir is not None:
        if not isinstance(output_dir, str) or not output_dir.strip():
            raise ValueError('output_dir must be a non-empty path string')
        parent = Path(output_dir).expanduser().resolve()
    else:
        parent = ROOT / 'runtime' / 'camera_dark_tests'
    parent.mkdir(parents=True, exist_ok=True)
    folder = parent / ('dark_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
                       + '_' + uuid4().hex[:8])
    folder.mkdir(mode=0o750, exist_ok=False)
    return folder


def _prepare_speech():
    """Pre-synthesize before the timed countdown, reusing configured R2B4 TTS."""
    from r2b4_voice.tts_provider import build_tts_client
    from r2b4_voice.voice_output import PcmWavePlayer

    tts = build_tts_client(project_root=ROOT, resident=True)
    player = PcmWavePlayer()
    if player.available_player() is None:
        raise RuntimeError('NO_SPEAKER_PLAYER: pw-play, paplay or aplay unavailable')
    return player, [tts.synthesize(word) for word in
                    ('Öt', 'Négy', 'Három', 'Kettő', 'Egy', 'Nulla')]


async def _speak_countdown():
    player, prepared = await asyncio.to_thread(_prepare_speech)
    beginning = time.monotonic()
    stamps = []
    for index, speech in enumerate(prepared):
        target = beginning + index  # one-second nominal step; playback may be longer
        delay = target - time.monotonic()
        if delay > 0:
            await asyncio.sleep(delay)
        start = time.monotonic()
        backend = await asyncio.to_thread(player.play, speech)
        stamps.append({'number': 5-index, 'started_after_s': round(start-beginning, 3),
                       'player': backend})
    return {'spoken': True, 'numbers': stamps, 'elapsed_s': round(time.monotonic()-beginning, 3)}


async def _photo(robot, folder, index, stream, quality, brightness, contrast):
    frame = await robot.call('vision.observe', stream=stream)
    # The canonical SDK VisionJpeg exposes bytes + calibrated producer metadata.
    metadata = frame.metadata.to_jsonable()
    original_name = f'{index:02d}_{stream}_original.jpg'
    with (folder / original_name).open('xb') as f:
        f.write(frame.image_bytes)
    result = {
        'kind': 'photo', 'stream': stream,
        'original': original_name, 'sha256': hashlib.sha256(frame.image_bytes).hexdigest(),
        'producer_metadata': metadata,
        'software_jpeg_quality': quality,
        'software_brightness': brightness,
        'software_contrast': contrast,
        'sensor_controls_modified': False,
        'postprocessed': None,
        'mean_luma_0_255': None,
        'dark_pixel_fraction_under_16': None,
    }
    # Pillow is optional; even without it the unmodified camera originals remain.
    try:
        from PIL import Image, ImageEnhance
        with Image.open(BytesIO(frame.image_bytes)) as incoming:
            original_rgb = incoming.convert('RGB')
            grayscale = original_rgb.convert('L')
            hist = grayscale.histogram()
            total = sum(hist)
            result['mean_luma_0_255'] = round(sum(i*n for i, n in enumerate(hist))/total, 2)
            result['dark_pixel_fraction_under_16'] = round(sum(hist[:16])/total, 5)
            changed = ImageEnhance.Brightness(original_rgb).enhance(brightness)
            changed = ImageEnhance.Contrast(changed).enhance(contrast)
            derived_name = f'{index:02d}_{stream}_q{quality}_b{brightness:g}_c{contrast:g}.jpg'
            with (folder / derived_name).open('xb') as output:
                changed.save(output, format='JPEG', quality=quality)
            result['postprocessed'] = derived_name
    except ImportError:
        result['postprocess_warning'] = 'Pillow (PIL) unavailable: original saved; software variants skipped'
    return result


async def _video(robot, folder, index, label, bitrate):
    filename = f'{index:02d}_{label}_{bitrate//1_000_000}Mbps.mp4'
    # Host media owner captures and writes the asset, not the skill worker.
    reply = await robot.call('camera.video', output=str(folder / filename),
                             duration_s=2.0, bitrate=bitrate)
    return {'kind': 'video', 'file': filename, 'bitrate': bitrate,
            'requested_duration_s': 2.0, 'owner_reply': reply}


async def run(robot, output_dir=None):
    """Say 5..0, then compare photographs and 2 s clips while stationary."""
    folder = _new_output_dir(output_dir)
    manifest = {
        'schema': 'R2B4_DARK_CAMERA_TEST_V1',
        'started_utc': datetime.now(timezone.utc).isoformat(),
        'output_dir': str(folder),
        'target_window_s': CAPTURE_SECONDS,
        'camera_setting_scope': 'public SDK streams + video bitrate + SOFTWARE JPEG/brightness/contrast',
        'sensor_exposure_gain_shutter_modified': False,
        'stationary': True,
        'camera_status_before': None,
        'countdown': None,
        'captures': [],
    }
    try:
        manifest['camera_status_before'] = await robot.read('camera.status')
        # Startup/warmup is OUTSIDE the requested 20 s measurement window.
        preflight = await robot.call('vision.observe', stream='lores')
        manifest['warmup_metadata'] = preflight.metadata.to_jsonable()
        manifest['countdown'] = await _speak_countdown()
        beginning = time.monotonic()
        deadline = beginning + CAPTURE_SECONDS
        for index, (offset, kind, profile, value, brightness, contrast) in enumerate(PLAN, 1):
            wait_s = beginning + offset - time.monotonic()
            if wait_s > 0:
                await asyncio.sleep(wait_s)
            remaining = deadline - time.monotonic()
            # 2 s video + a bounded allowance for camera and encoder overhead.
            minimum = 4.0 if kind == 'video' else 0.6
            if remaining < minimum:
                manifest['captures'].append({'index': index, 'kind': kind,
                    'status': 'SKIPPED_INSUFFICIENT_TIME', 'remaining_s': round(remaining, 3)})
                continue
            started = time.monotonic()
            try:
                if kind == 'photo':
                    result = await _photo(robot, folder, index, profile, value, brightness, contrast)
                else:
                    result = await _video(robot, folder, index, profile, value)
                result['status'] = 'SAVED'
            except Exception as exc:
                result = {'kind': kind, 'status': 'FAILED',
                          'error': f'{type(exc).__name__}:{exc}'[:400]}
            result.update(index=index, offset_s=offset,
                          started_after_s=round(started-beginning, 3),
                          finished_after_s=round(time.monotonic()-beginning, 3))
            manifest['captures'].append(result)
        spare = deadline - time.monotonic()
        if spare > 0:
            await asyncio.sleep(spare)
        manifest['capture_window_elapsed_s'] = round(time.monotonic()-beginning, 3)
        photos = sum(x['status'] == 'SAVED' and x['kind'] == 'photo' for x in manifest['captures'])
        videos = sum(x['status'] == 'SAVED' and x['kind'] == 'video' for x in manifest['captures'])
        manifest['photos_saved'] = photos
        manifest['videos_saved'] = videos
        manifest['status'] = ('COMPLETE' if photos >= 5 and videos == 2
                              and manifest['capture_window_elapsed_s'] <= CAPTURE_SECONDS+0.15
                              else 'PARTIAL')
        manifest['camera_status_after'] = await robot.read('camera.status')
    except BaseException as exc:
        manifest['status'] = 'INTERRUPTED'
        manifest['error'] = f'{type(exc).__name__}:{exc}'[:400]
        raise
    finally:
        manifest['finished_utc'] = datetime.now(timezone.utc).isoformat()
        _write_json(folder / 'manifest.json', manifest)
    return {'status': manifest['status'], 'photos_saved': manifest.get('photos_saved', 0),
            'videos_saved': manifest.get('videos_saved', 0),
            'capture_window_elapsed_s': manifest.get('capture_window_elapsed_s'),
            'manifest': str(folder / 'manifest.json'), 'output_dir': str(folder)}
