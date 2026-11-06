"""One physical-camera exclusion shared by production and maintenance."""
from __future__ import annotations

import fcntl
import os
import tempfile
from contextlib import contextmanager
from pathlib import Path


@contextmanager
def camera_device_lock():
    path = Path(tempfile.gettempdir()) / f"r2b4_camera_{os.getuid()}.lock"
    with path.open("a+b") as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError("physical camera is already owned") from exc
        try:
            yield
        finally:
            fcntl.flock(handle, fcntl.LOCK_UN)
