"""Canonical, protected runtime contract for the R2B4 scan matcher."""

from __future__ import annotations

import math
from typing import Any, Dict, Mapping


SCAN_MATCHER_CONTRACT_ID = "R2B4_SCAN_MATCHER_PROCESS_LATEST_ONLY_V1"
SCAN_MATCH_CONFIDENCE_MODEL = "R2B4_SCAN_MATCH_CONFIDENCE_V2"
SCAN_MATCH_INTEGRITY_MODEL = "R2B4_SCAN_MATCH_BASIN_INTEGRITY_V1"
SCAN_MATCHER_TRANSPORT = "process_latest_only"
SCAN_MATCHER_PROCESS_START_METHOD = "spawn"
SCAN_MATCHER_INPUT_QUEUE_SIZE = 1
SCAN_MATCHER_RESULT_QUEUE_SIZE = 1
SCAN_MATCHER_MAX_INPUT_AGE_S = 0.25
SCAN_MATCHER_MAX_RESULT_AGE_S = 0.25

SCAN_MATCHER_REQUIRED_RUNTIME_CONFIG = {
    "matcher_process_start_method": SCAN_MATCHER_PROCESS_START_METHOD,
    "latest_scan_queue_size": SCAN_MATCHER_INPUT_QUEUE_SIZE,
    "latest_result_queue_size": SCAN_MATCHER_RESULT_QUEUE_SIZE,
    "matcher_max_input_age_s": SCAN_MATCHER_MAX_INPUT_AGE_S,
    "matcher_max_result_age_s": SCAN_MATCHER_MAX_RESULT_AGE_S,
}


def _contract_value_matches(actual: Any, expected: Any) -> bool:
    if isinstance(expected, float):
        try:
            value = float(actual)
        except (TypeError, ValueError):
            return False
        return math.isfinite(value) and math.isclose(
            value,
            expected,
            rel_tol=0.0,
            abs_tol=1e-12,
        )
    if isinstance(expected, int):
        return type(actual) is int and actual == expected
    return actual == expected


def validate_matcher_runtime_config(config: Mapping[str, Any]) -> Dict[str, Any]:
    """
    Validate the non-negotiable process/queue/freshness contract.

    Missing keys receive the canonical value for isolated unit construction.
    Explicitly configured deviations fail closed during LidarService creation.
    """
    runtime_cfg = dict(config or {})
    resolved: Dict[str, Any] = {}
    violations = []
    for key, expected in SCAN_MATCHER_REQUIRED_RUNTIME_CONFIG.items():
        actual = runtime_cfg.get(key, expected)
        if not _contract_value_matches(actual, expected):
            violations.append(f"{key}={actual!r},expected={expected!r}")
        resolved[key] = expected
    if violations:
        raise ValueError(
            "scan_matcher_contract_violation:" + ";".join(violations)
        )
    return resolved


def matcher_contract_status() -> Dict[str, Any]:
    """Return a detached runtime status representation of the stable contract."""
    return {
        "matcher_contract_id": SCAN_MATCHER_CONTRACT_ID,
        "matcher_confidence_model": SCAN_MATCH_CONFIDENCE_MODEL,
        "matcher_integrity_model": SCAN_MATCH_INTEGRITY_MODEL,
        "matcher_transport": SCAN_MATCHER_TRANSPORT,
        "matcher_process_start_method": SCAN_MATCHER_PROCESS_START_METHOD,
        "matcher_input_queue_capacity": SCAN_MATCHER_INPUT_QUEUE_SIZE,
        "matcher_result_queue_capacity": SCAN_MATCHER_RESULT_QUEUE_SIZE,
        "matcher_max_input_age_s": SCAN_MATCHER_MAX_INPUT_AGE_S,
        "matcher_max_result_age_s": SCAN_MATCHER_MAX_RESULT_AGE_S,
    }
