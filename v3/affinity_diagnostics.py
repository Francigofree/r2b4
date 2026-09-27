"""Read-only /proc evidence for the configured R2B4 scheduling roles."""
from __future__ import annotations

from pathlib import Path

from v3.runtime_performance import RuntimeAffinityConfig


def cpu_list(value: str) -> set[int]:
    result: set[int] = set()
    for part in value.strip().split(","):
        if part:
            ends = part.split("-")
            result.update(range(int(ends[0]), int(ends[-1]) + 1))
    return result


def task_rows(pid: int) -> list[dict[str, object]]:
    rows = []
    for task in sorted(Path(f"/proc/{pid}/task").iterdir()):
        try:
            status = (task / "status").read_text()
            allowed = next(line.split(":", 1)[1] for line in status.splitlines()
                           if line.startswith("Cpus_allowed_list:"))
            rows.append({"pid": pid, "tid": int(task.name),
                         "name": (task / "comm").read_text().strip(),
                         "allowed_cpus": sorted(cpu_list(allowed))})
        except (OSError, StopIteration):
            continue  # Task exited during the read.
    return rows


# Linux comm is limited to 15 bytes. These names come from the production
# affinity entry points, including names inherited by queue feeder threads.
_TASK_ROLES = {
    ("r2b4-" + name)[:15]: role
    for role, names in {
        "encoder": ("encoder-owner-process",),
        "imu": ("imu-acquire",),
        "lidar_owner": ("lidar-owner-process", "lidar-ipc"),
        "lidar_matcher": ("lidar-matcher",),
        "vision": ("vision-owner-process", "vision-ipc", "vision", "person-evidence"),
        "planner": ("l6-planner", "l6-planner-feeder", "l6-result", "l6-result-start", "l6-recovery"),
        "capture": ("capture", "capture-feeder"),
        "status": ("status", "status-feeder"),
        "command": ("command",),
        "l0_encoder": ("l0-encoder",),
        "l0_imu": ("l0-imu",),
        "l0_lidar": ("l0-lidar",),
        "l0_aux": ("l0-aux",),
        "runtime_background": ("background", "resource-tracker"),
        "operator": ("operator",),
        "voice": ("voice",),
        "er2": ("er2",),
        "diagnostics": ("diagnostics",),
    }.items() for name in names
}


def audit_affinity(config: RuntimeAffinityConfig, runtime_pid: int, *,
                   project_root: Path | None = None,
                   required_roles: tuple[str, ...] = ()) -> dict[str, object]:
    """Check role masks and control exclusivity across the tree and local services.

    Unnamed native tasks inherit their process role. An unclassified runtime
    helper must fit the background policy; it never gains the control mask.
    """
    processes: dict[int, tuple[int, str]] = {}
    selected: dict[int, str] = {runtime_pid: "runtime_background"}
    services = {"r2b4_voice.voice_service": "voice", "r2b4_er2": "er2",
                "v3.launcher_cli": "operator", "v3.operator_cli": "operator",
                "v3.test_hub": "diagnostics", "v3.test_runner": "diagnostics"}
    for path in Path("/proc").iterdir():
        if not path.name.isdigit():
            continue
        try:
            ppid = next(int(line.split(":", 1)[1]) for line in
                        (path / "status").read_text().splitlines() if line.startswith("PPid:"))
            command = (path / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
            pid = int(path.name)
            processes[pid] = (ppid, command)
            if project_root is not None and (path / "cwd").resolve() == project_root.resolve():
                for marker, role in services.items():
                    if marker in command:
                        selected.setdefault(pid, role)
                        break
        except (OSError, StopIteration):
            continue
    # Include grandchildren (matcher, Piper, resource trackers), not just owners.
    pending = list(selected)
    rows: list[dict[str, object]] = []
    visited: set[int] = set()
    while pending:
        pid = pending.pop(0)
        if pid in visited:
            continue
        visited.add(pid)
        try:
            tasks = task_rows(pid)
        except OSError:
            continue
        main = next((row for row in tasks if row["tid"] == pid), None)
        process_role = _TASK_ROLES.get(str(main["name"]), selected[pid]) if main else selected[pid]
        for row in tasks:
            role = "control" if pid == runtime_pid and row["tid"] == pid else (
                _TASK_ROLES.get(str(row["name"]), process_role))
            actual = set(row["allowed_cpus"])
            expected = set(getattr(config, role + "_cpus"))
            row.update(role=role, configured_cpus=sorted(expected),
                       matches=bool(actual) and (actual == expected if role == "control" else actual <= expected))
            rows.append(row)
        for child, (parent, _) in processes.items():
            if parent == pid and child not in visited:
                selected.setdefault(child, process_role)
                pending.append(child)
    present = {row["role"] for row in rows}
    missing = sorted(set(required_roles) - present)
    checks = {
        "control_main_present": "control" in present,
        "all_tasks_match_role": bool(rows) and all(row["matches"] for row in rows),
        "control_cpu_exclusive": all(
            row["role"] == "control" or not set(row["allowed_cpus"]).intersection(config.control_cpus)
            for row in rows),
        "required_roles_present": not missing,
    }
    return {"status": "PASS" if all(checks.values()) else "FAIL",
            "runtime_pid": runtime_pid, "config": config.as_dict(),
            "checks": checks, "missing_roles": missing, "tasks": rows}
