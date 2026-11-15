"""Explicit source-change regression loop; every run imports fresh source."""
from __future__ import annotations

import subprocess
import sys
import time

from .evidence import source_snapshot, write


def watch(root, output, baseline, *, scenario=None, domain=75, max_runs=20):
    if type(max_runs) is not int or not 1 <= max_runs <= 1000:
        raise ValueError("max-runs must be 1..1000")
    paths = source_snapshot(root)["files"]

    def signature():
        values = []
        for name in paths:
            try:
                stat = (root / name).stat()
                values.append((name, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns))
            except FileNotFoundError:
                values.append((name, None))
        return values

    previous = None
    results = []
    while len(results) < max_runs:
        current = signature()
        if current == previous:
            time.sleep(1.)
            continue
        # Save bursts as one edit; the child still rejects mid-run changes.
        time.sleep(1.)
        previous = signature()
        target = output / f"change_{len(results):04d}"
        command = [sys.executable, "-m", "tools.navigation_simulator", "regression",
                   "--baseline", str(baseline.resolve()), "--ros-domain-id", str(domain), "--output", str(target)]
        if scenario:
            command += ["--scenario", scenario]
        child = subprocess.run(command, cwd=root, check=False)
        results.append({"returncode": child.returncode, "output": str(target)})
        result = {"status": "RUNNING" if child.returncode == 0 else "FAIL", "runs": results,
                  "physical_output": False, "stop_on_failure": True}
        write(output / "watch.json", result)
        if child.returncode != 0:
            return result
        paths = source_snapshot(root)["files"]
        previous = signature()
    result = {"status": "PASS", "runs": results, "physical_output": False}
    write(output / "watch.json", result)
    return result
