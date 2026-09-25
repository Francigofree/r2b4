from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


def load_module():
    path = (Path(__import__("os").environ["R2B4_ROOT"]).resolve() if __import__("os").environ.get("R2B4_ROOT") else next((p for p in Path(__file__).resolve().parents if (p / "conf" / "hardver.json").is_file() and (p / "v3").is_dir()), Path.cwd())) / "tools/r2b4_cpu2.py"
    spec = importlib.util.spec_from_file_location("r2b4_cpu2_test_target", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_proc_stat_field_positions_are_linux_positions():
    module = load_module()
    parsed = module.parse_proc_stat(module.make_selftest_stat())
    assert parsed["pid"] == 123
    assert parsed["comm"] == "r2b4 runtime"
    assert parsed["utime"] == 120
    assert parsed["stime"] == 30
    assert parsed["starttime"] == 999
    assert parsed["processor"] == 3


def test_throttle_flags_are_direct_bit_decode():
    module = load_module()
    assert module.decode_throttled((1 << 2) | (1 << 18)) == [
        "THROTTLED_NOW",
        "THROTTLED_OCCURRED",
    ]
