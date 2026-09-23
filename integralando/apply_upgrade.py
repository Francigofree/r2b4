#!/usr/bin/env python3
"""Apply the R2B4 Test Hub high-level evidence upgrade.

Run from the repository root:
    python3 integralando/apply_upgrade.py

The script backs up every replaced/modified file under runtime/upgrade_backups.
It does not touch runtime/control/capture production code.
"""

from __future__ import annotations

import shutil
import time
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
PAYLOAD = SCRIPT_DIR / "payload"
ROOT = Path.cwd().resolve()
BACKUP = ROOT / "runtime" / "upgrade_backups" / f"testhub_level100_{time.strftime('%Y%m%d_%H%M%S')}"


def require_repo() -> None:
    required = (ROOT / "v3", ROOT / "tests", ROOT / "conf")
    if not all(path.exists() for path in required):
        raise RuntimeError("Run this script from the r2b4 repository root")


def backup(path: Path) -> None:
    if not path.exists():
        return
    relative = path.relative_to(ROOT)
    target = BACKUP / relative
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(path, target)


def install_payload(relative: str) -> None:
    source = PAYLOAD / relative
    target = ROOT / relative
    if not source.is_file():
        raise RuntimeError(f"missing payload file: {source}")
    backup(target)
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)


def replace_once(relative: str, old: str, new: str) -> bool:
    path = ROOT / relative
    text = path.read_text(encoding="utf-8")
    if new in text:
        return False
    if old not in text:
        raise RuntimeError(f"{relative}: patch anchor not found")
    backup(path)
    path.write_text(text.replace(old, new, 1), encoding="utf-8")
    return True


def replace_all_exact(relative: str, old: str, new: str, expected_min: int = 1) -> int:
    path = ROOT / relative
    text = path.read_text(encoding="utf-8")
    if old not in text:
        if new in text:
            return 0
        raise RuntimeError(f"{relative}: patch anchor not found: {old!r}")
    count = text.count(old)
    if count < expected_min:
        raise RuntimeError(f"{relative}: expected at least {expected_min} anchors, found {count}")
    backup(path)
    path.write_text(text.replace(old, new), encoding="utf-8")
    return count


def main() -> int:
    require_repo()

    # New derived-only analyzers and focused regression tests.
    install_payload("v3/test_hub_task_evidence.py")
    install_payload("v3/test_hub_motion_tuning.py")
    install_payload("tests/test_v3_test_hub_task_evidence.py")

    replace_once(
        "v3/test_hub_next.py",
        "from .test_hub_behavior import build_behavior_evidence\n",
        "from .test_hub_behavior import build_behavior_evidence\n"
        "from .test_hub_task_evidence import build_task_evidence\n"
        "from .test_hub_motion_tuning import build_motion_tuning_evidence\n",
    )
    replace_once("v3/test_hub_next.py", "DEFAULT_HZ = 5\n", "DEFAULT_HZ = 10\n")

    anchor = (
        "    behavior_episode_path = destination / \"behavior_episodes.ndjson\"\n"
        "    behavior_episode_source = behavior_episode_path if behavior_episode_path.is_file() else None\n\n"
        "    motion_quality_path = destination / \"motion_quality.json\"\n"
    )
    replacement = (
        "    behavior_episode_path = destination / \"behavior_episodes.ndjson\"\n"
        "    behavior_episode_source = behavior_episode_path if behavior_episode_path.is_file() else None\n\n"
        "    # Level-100 task evidence and movement-tuning evidence are derived-only.\n"
        "    # They never change the canonical Test Hub status/diagnosis/replay result.\n"
        "    try:\n"
        "        task_evidence = build_task_evidence(\n"
        "            reader, destination, behavior_episodes_path=behavior_episode_source\n"
        "        )\n"
        "    except (OSError, TypeError, ValueError, RuntimeError) as exc:\n"
        "        task_evidence = {\n"
        "            \"schema\": \"R2B4_TEST_HUB_TASK_EVIDENCE_V1\",\n"
        "            \"policy\": \"DESCRIPTIVE_EVIDENCE_ONLY_NO_AUTOMATIC_VERDICT\",\n"
        "            \"availability\": \"ERROR\",\n"
        "            \"error\": str(exc),\n"
        "        }\n"
        "        _write_json(destination / \"task_evidence_summary.json\", task_evidence)\n\n"
        "    try:\n"
        "        motion_tuning = build_motion_tuning_evidence(\n"
        "            reader, destination, behavior_episodes_path=behavior_episode_source\n"
        "        )\n"
        "    except (OSError, TypeError, ValueError, RuntimeError) as exc:\n"
        "        motion_tuning = {\n"
        "            \"schema\": \"R2B4_TEST_HUB_MOTION_TUNING_V1\",\n"
        "            \"policy\": \"DESCRIPTIVE_TUNING_EVIDENCE_ONLY_NO_AUTOMATIC_VERDICT\",\n"
        "            \"availability\": \"ERROR\",\n"
        "            \"error\": str(exc),\n"
        "        }\n"
        "        _write_json(destination / \"motion_tuning_summary.json\", motion_tuning)\n"
        "        (destination / \"motion_tuning_segments.ndjson\").write_text(\"\", encoding=\"utf-8\")\n\n"
        "    motion_quality_path = destination / \"motion_quality.json\"\n"
    )
    replace_once("v3/test_hub_next.py", anchor, replacement)

    replace_once(
        "v3/test_hub_next.py",
        "        \"behavior\": behavior,\n        \"replay_status\": replay_status,\n",
        "        \"behavior\": behavior,\n"
        "        \"task_evidence\": task_evidence,\n"
        "        \"motion_tuning\": motion_tuning,\n"
        "        \"replay_status\": replay_status,\n",
    )

    replace_once(
        "v3/test_hub_next.py",
        "        \"localization_quality_status\": localization_quality.get(\"status\"),\n"
        "        \"note\": \"One .evidence directory is the portable agent package; MCAP remains local authority.\",\n",
        "        \"localization_quality_status\": localization_quality.get(\"status\"),\n"
        "        \"task_evidence_episode_count\": task_evidence.get(\"episode_count\"),\n"
        "        \"motion_tuning_segment_count\": motion_tuning.get(\"segment_count\"),\n"
        "        \"note\": \"One .evidence directory is the portable agent package; MCAP remains local authority.\",\n",
    )

    # Agent-facing view defaults now match the default 10 Hz capture. This only
    # changes presentation density; the analysis profile still comes from MCAP metadata.
    replace_once("v3/test_hub_views.py", "    hz: int = 5,\n", "    hz: int = 10,\n")
    replace_all_exact("v3/adapters/testhub.py", 'params.pop("hz", 5)', 'params.pop("hz", 10)', expected_min=2)

    # Surface the new evidence references through the RobotInterface adapter.
    replace_all_exact(
        "v3/adapters/testhub.py",
        '                "behavior_status": agent.get("behavior_status") if agent else None,\n',
        '                "behavior_status": agent.get("behavior_status") if agent else None,\n'
        '                "task_evidence": agent.get("task_evidence") if agent else None,\n'
        '                "motion_tuning": agent.get("motion_tuning") if agent else None,\n',
        expected_min=1,
    )
    replace_once(
        "v3/adapters/testhub.py",
        '            "behavior_status": agent.get("behavior_status"),\n        }\n\n    def _selected_capture',
        '            "behavior_status": agent.get("behavior_status"),\n'
        '            "task_evidence": agent.get("task_evidence"),\n'
        '            "motion_tuning": agent.get("motion_tuning"),\n'
        '        }\n\n    def _selected_capture',
    )

    # Minimal documentation update; no architecture authority docs are changed.
    readme = ROOT / "README.md"
    if readme.is_file():
        text = readme.read_text(encoding="utf-8")
        changed = False
        if "és 5 Hz-es áttekintést" in text:
            text = text.replace("és 5 Hz-es áttekintést", "és 10 Hz-es áttekintést", 1)
            changed = True
        marker = (
            "A számlálók rögzített mintákat számolnak, nem teljes control\n"
            "tick-számot vagy időarányt; a köztes, nem rögzített állapotok nem rekonstruálhatók.\n"
        )
        addition = marker + (
            "\nA magasabb szintű, döntésmentes evidence réteg `task_evidence_summary.json`,\n"
            "`task_evidence_episodes.ndjson` és `task_evidence_timeline.ndjson` fájlokban\n"
            "méri az EXPLORE/FOLLOW_PERSON/NAVIGATE feladatspecifikus capture-tényeket.\n"
            "A `motion_tuning_summary.json` és `motion_tuning_segments.ndjson` cross-layer\n"
            "mozgás-, kerék-, actuator- és planner-metrikákat ad hangoláshoz. Ezek nem\n"
            "adnak GOOD/BAD minősítést, root cause diagnózist vagy javítási javaslatot.\n"
        )
        if marker in text and "task_evidence_summary.json" not in text:
            text = text.replace(marker, addition, 1)
            changed = True
        if changed:
            backup(readme)
            readme.write_text(text, encoding="utf-8")

    print("R2B4 Test Hub high-level evidence upgrade applied.")
    print(f"Backup: {BACKUP}")
    print("New artifacts on new/rebuilt captures:")
    print("  task_evidence_summary.json")
    print("  task_evidence_episodes.ndjson")
    print("  task_evidence_timeline.ndjson")
    print("  motion_tuning_summary.json")
    print("  motion_tuning_segments.ndjson")
    print("Suggested validation:")
    print("  python3 -m pytest -q tests/test_v3_test_hub_task_evidence.py")
    print("  python3 -m pytest -q -m testhub")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
