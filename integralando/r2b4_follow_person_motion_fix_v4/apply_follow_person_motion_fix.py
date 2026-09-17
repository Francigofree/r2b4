#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import py_compile
import tempfile
from pathlib import Path


L6_FIELDS_OLD = """\
    follow_person_align_tolerance_rad: float = 0.10
    follow_person_release_tolerance_rad: float = 0.30
    follow_person_stand_off_m: float = 1.05
    follow_person_distance_deadband_m: float = 0.15
    follow_person_min_safe_distance_m: float = 0.75
    follow_person_lost_hold_ns: int = 400_000_000
    follow_person_pivot_enter_rad: float = 0.55
    follow_person_slowdown_distance_m: float = 0.30
"""

L6_FIELDS_NEW = """\
    follow_person_align_tolerance_rad: float = 0.22
    follow_person_release_tolerance_rad: float = 0.30
    follow_person_stand_off_m: float = 1.05
    follow_person_distance_deadband_m: float = 0.15
    follow_person_min_safe_distance_m: float = 0.75
    follow_person_lost_hold_ns: int = 400_000_000
    follow_person_pivot_enter_rad: float = 0.55
    follow_person_hold_release_margin_m: float = 0.05
    follow_person_slowdown_distance_m: float = 0.18
    follow_person_minimum_follow_speed_mps: float = 0.10
    follow_person_heading_min_factor: float = 0.75
"""

L6_VALIDATION_OLD = """\
        if (
            not isinstance(self.follow_person_slowdown_distance_m, (int, float))
            or isinstance(self.follow_person_slowdown_distance_m, bool)
            or not math.isfinite(self.follow_person_slowdown_distance_m)
            or self.follow_person_slowdown_distance_m <= 0.0
        ):
            raise ValueError("follow_person_slowdown_distance_m must be positive")
"""

L6_VALIDATION_NEW = """\
        for name in (
            "follow_person_hold_release_margin_m",
            "follow_person_slowdown_distance_m",
            "follow_person_minimum_follow_speed_mps",
        ):
            value = getattr(self, name)
            if (
                not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not math.isfinite(value)
                or value <= 0.0
            ):
                raise ValueError(f"{name} must be positive")
        if (
            not isinstance(self.follow_person_heading_min_factor, (int, float))
            or isinstance(self.follow_person_heading_min_factor, bool)
            or not math.isfinite(self.follow_person_heading_min_factor)
            or not 0.0 < self.follow_person_heading_min_factor <= 1.0
        ):
            raise ValueError("follow_person_heading_min_factor must be in (0, 1]")
"""

HOLD_OLD = (
    "        hold_release_m = hold_enter_m + "
    "self._config.follow_person_distance_deadband_m\n"
)
HOLD_NEW = (
    "        hold_release_m = hold_enter_m + "
    "self._config.follow_person_hold_release_margin_m\n"
)

SPEED_START = "        # Human-specific speed shaping: slow before HOLD and while bending.\n"
SPEED_END = "        self._accept_pending_rollout(mission.context)\n"

SPEED_NEW = """\
        # FOLLOW motion shaping keeps HOLD, distance approach and heading response
        # independent. HOLD is the only behavior-level zero-motion state here.
        # The minimum speed is a rollout-envelope floor, not a forced command:
        # rollout still contains v=0 and downstream L7-L12 remain authoritative.
        max_follow_v_mps = mission.constraints.max_v_mps
        minimum_follow_v_mps = min(
            self._config.follow_person_minimum_follow_speed_mps,
            max_follow_v_mps,
        )

        distance_ratio = min(
            1.0,
            max(
                0.0,
                (distance_m - hold_enter_m)
                / self._config.follow_person_slowdown_distance_m,
            ),
        )
        distance_cap_mps = min(
            max_follow_v_mps,
            max(
                minimum_follow_v_mps,
                max_follow_v_mps * distance_ratio,
            ),
        )

        if absolute_error <= self._config.follow_person_align_tolerance_rad:
            heading_factor = 1.0
        else:
            heading_span = max(
                self._config.follow_person_pivot_enter_rad
                - self._config.follow_person_align_tolerance_rad,
                1e-9,
            )
            heading_ratio = min(
                1.0,
                max(
                    0.0,
                    (
                        absolute_error
                        - self._config.follow_person_align_tolerance_rad
                    )
                    / heading_span,
                ),
            )
            heading_factor = 1.0 - (
                1.0 - self._config.follow_person_heading_min_factor
            ) * heading_ratio

        heading_cap_mps = max_follow_v_mps * heading_factor
        follow_max_v_mps = min(
            max_follow_v_mps,
            max(
                minimum_follow_v_mps,
                min(distance_cap_mps, heading_cap_mps),
            ),
        )

"""

NATIVE_OLD = """\
        follow_person_align_tolerance_rad=_positive_float(
            follow_person.get("align_tolerance_rad", 0.10),
            "v3_navigation.follow_person.align_tolerance_rad",
        ),
"""

NATIVE_NEW = """\
        follow_person_align_tolerance_rad=_positive_float(
            follow_person.get("align_tolerance_rad", 0.22),
            "v3_navigation.follow_person.align_tolerance_rad",
        ),
"""

NATIVE_TAIL_OLD = """\
        follow_person_pivot_enter_rad=_positive_float(
            follow_person.get("pivot_enter_rad", 0.55),
            "v3_navigation.follow_person.pivot_enter_rad",
        ),
        follow_person_slowdown_distance_m=_positive_float(
            follow_person.get("slowdown_distance_m", 0.30),
            "v3_navigation.follow_person.slowdown_distance_m",
        ),
"""

NATIVE_TAIL_NEW = """\
        follow_person_pivot_enter_rad=_positive_float(
            follow_person.get("pivot_enter_rad", 0.55),
            "v3_navigation.follow_person.pivot_enter_rad",
        ),
        follow_person_hold_release_margin_m=_positive_float(
            follow_person.get("hold_release_margin_m", 0.05),
            "v3_navigation.follow_person.hold_release_margin_m",
        ),
        follow_person_slowdown_distance_m=_positive_float(
            follow_person.get("slowdown_distance_m", 0.18),
            "v3_navigation.follow_person.slowdown_distance_m",
        ),
        follow_person_minimum_follow_speed_mps=_positive_float(
            follow_person.get("minimum_follow_speed_mps", 0.10),
            "v3_navigation.follow_person.minimum_follow_speed_mps",
        ),
        follow_person_heading_min_factor=_positive_float(
            follow_person.get("heading_min_factor", 0.75),
            "v3_navigation.follow_person.heading_min_factor",
        ),
"""

P0_CONFIG_OLD = """\
    assert config.follow_person_lost_hold_ns == 400_000_000
    assert config.follow_person_release_tolerance_rad == pytest.approx(0.30)
    assert config.follow_person_pivot_enter_rad == pytest.approx(0.55)
    assert config.follow_person_slowdown_distance_m == pytest.approx(0.30)
"""

P0_CONFIG_NEW = """\
    assert config.follow_person_lost_hold_ns == 400_000_000
    assert config.follow_person_align_tolerance_rad == pytest.approx(0.22)
    assert config.follow_person_release_tolerance_rad == pytest.approx(0.30)
    assert config.follow_person_pivot_enter_rad == pytest.approx(0.55)
    assert config.follow_person_hold_release_margin_m == pytest.approx(0.05)
    assert config.follow_person_slowdown_distance_m == pytest.approx(0.18)
    assert config.follow_person_minimum_follow_speed_mps == pytest.approx(0.10)
    assert config.follow_person_heading_min_factor == pytest.approx(0.75)
"""

P0_HYSTERESIS_OLD = """\
    # 1.25 m is above the 1.20 m entry threshold but below the 1.35 m release
    # threshold, so a held robot stays held instead of chattering on/off.
    c2 = TickContext(41, 5_020_000_000)
    still_hold = nav.evaluate(
        _mission(c2, command_id), _estimate(c2), _world(c2, _person("person-a", 1.25, 0.0))
    )
    assert len(still_hold.route) == 1
    assert nav.checkpoint().follow_person_holding is True

    c3 = TickContext(42, 5_040_000_000)
    resumed = nav.evaluate(
        _mission(c3, command_id), _estimate(c3), _world(c3, _person("person-a", 1.36, 0.0))
    )
    assert resumed.route == ()
    assert resumed.trajectory_candidates
    assert nav.checkpoint().follow_person_holding is False
    max_candidate_v = max(candidate.v_mps for candidate in resumed.trajectory_candidates)
    assert 0.0 < max_candidate_v < 0.15
"""

P0_HYSTERESIS_NEW = """\
    # HOLD hysteresis is intentionally narrow: 1.20 m entry, 1.25 m release.
    # This prevents chatter without making the robot wait 15 cm before reacting.
    c2 = TickContext(41, 5_020_000_000)
    still_hold = nav.evaluate(
        _mission(c2, command_id), _estimate(c2), _world(c2, _person("person-a", 1.23, 0.0))
    )
    assert len(still_hold.route) == 1
    assert nav.checkpoint().follow_person_holding is True

    c3 = TickContext(42, 5_040_000_000)
    resumed = nav.evaluate(
        _mission(c3, command_id), _estimate(c3), _world(c3, _person("person-a", 1.26, 0.0))
    )
    assert resumed.route == ()
    assert resumed.trajectory_candidates
    assert nav.checkpoint().follow_person_holding is False
    max_candidate_v = max(candidate.v_mps for candidate in resumed.trajectory_candidates)
    assert 0.10 <= max_candidate_v < 0.15
"""


class TransformError(RuntimeError):
    pass


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise TransformError(f"{label}: expected one old anchor, found {count}")
    return text.replace(old, new, 1)


def transform_l6(text: str) -> str:
    if "follow_person_minimum_follow_speed_mps: float = 0.10" in text:
        return text
    text = replace_once(text, L6_FIELDS_OLD, L6_FIELDS_NEW, "L6 config fields")
    text = replace_once(text, L6_VALIDATION_OLD, L6_VALIDATION_NEW, "L6 validation")
    text = replace_once(text, HOLD_OLD, HOLD_NEW, "L6 HOLD release")

    start = text.find(SPEED_START)
    if start < 0:
        raise TransformError("L6 speed shaping start marker not found")
    end = text.find(SPEED_END, start)
    if end < 0:
        raise TransformError("L6 speed shaping end marker not found")
    if text.find(SPEED_START, start + 1) >= 0:
        raise TransformError("L6 speed shaping marker is not unique")
    text = text[:start] + SPEED_NEW + text[end:]
    return text


def transform_native(text: str) -> str:
    if "follow_person_minimum_follow_speed_mps=_positive_float(" in text:
        return text
    text = replace_once(text, NATIVE_OLD, NATIVE_NEW, "native align default")
    text = replace_once(text, NATIVE_TAIL_OLD, NATIVE_TAIL_NEW, "native FOLLOW fields")
    return text


def transform_p0_test(text: str) -> str:
    if "follow_person_hold_release_margin_m == pytest.approx(0.05)" in text:
        return text
    text = replace_once(text, P0_CONFIG_OLD, P0_CONFIG_NEW, "P0 config test")
    text = replace_once(
        text,
        P0_HYSTERESIS_OLD,
        P0_HYSTERESIS_NEW,
        "P0 hysteresis test",
    )
    return text


def transform_control_json(text: str) -> str:
    data = json.loads(text)
    follow = data["v3_navigation"]["follow_person"]
    follow["align_tolerance_rad"] = 0.22
    follow["hold_release_margin_m"] = 0.05
    follow["slowdown_distance_m"] = 0.18
    follow["minimum_follow_speed_mps"] = 0.10
    follow["heading_min_factor"] = 0.75
    return json.dumps(data, indent=2, ensure_ascii=False) + "\n"


def compile_text(source: str, filename: str) -> None:
    compile(source, filename, "exec")


def atomic_write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp_name = tempfile.mkstemp(
        prefix=f".{path.name}.",
        suffix=".tmp",
        dir=path.parent,
        text=True,
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as handle:
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temp_name, path)
    except Exception:
        try:
            os.unlink(temp_name)
        except FileNotFoundError:
            pass
        raise


def build(project_root: Path, package_root: Path):
    targets = {
        "l6": project_root / "v3/layers/l6_navigation.py",
        "native": project_root / "v3/composition/native_control.py",
        "config": project_root / "conf/vezerles.json",
        "p0_test": project_root / "tests/test_v3_follow_person_p0.py",
    }
    for label, path in targets.items():
        if not path.is_file():
            raise TransformError(f"{label}: missing {path}")

    original = {name: path.read_text(encoding="utf-8") for name, path in targets.items()}
    changed = {
        "l6": transform_l6(original["l6"]),
        "native": transform_native(original["native"]),
        "config": transform_control_json(original["config"]),
        "p0_test": transform_p0_test(original["p0_test"]),
    }

    motion_test_source = (
        package_root / "tests/test_v3_follow_person_motion_quality.py"
    ).read_text(encoding="utf-8")

    compile_text(changed["l6"], str(targets["l6"]))
    compile_text(changed["native"], str(targets["native"]))
    compile_text(changed["p0_test"], str(targets["p0_test"]))
    compile_text(
        motion_test_source,
        str(project_root / "tests/test_v3_follow_person_motion_quality.py"),
    )
    json.loads(changed["config"])

    return targets, changed, motion_test_source


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("project_root", type=Path)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    project_root = args.project_root.resolve()
    package_root = Path(__file__).resolve().parent

    targets, changed, motion_test_source = build(project_root, package_root)

    if args.check:
        print("FOLLOW_PERSON motion-quality source-anchor check: PASS")
        return 0

    atomic_write(targets["l6"], changed["l6"])
    atomic_write(targets["native"], changed["native"])
    atomic_write(targets["config"], changed["config"])
    atomic_write(targets["p0_test"], changed["p0_test"])
    atomic_write(
        project_root / "tests/test_v3_follow_person_motion_quality.py",
        motion_test_source,
    )

    print("FOLLOW_PERSON motion-quality fix applied.")
    print("No commit/hash verification was performed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
