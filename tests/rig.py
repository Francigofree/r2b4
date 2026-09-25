from __future__ import annotations

from pathlib import Path
from v3.config import ConfigResolver

ROOT = Path(__file__).resolve().parents[1]


def resolved_config(root: Path = ROOT):
    """Resolve the same four production config authorities used by the robot.

    No unit-config copy, constructor reflection or signature-following fixture exists here.
    """
    conf = root / "conf"
    return ConfigResolver(
        conf / "hardver.json",
        conf / "fizika.json",
        conf / "speed_map.json",
        conf / "vezerles.json",
    ).resolve()
