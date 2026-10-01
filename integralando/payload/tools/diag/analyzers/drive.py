"""Drive/actuation command and observed-sensor measurements."""
from __future__ import annotations

from collections import Counter

from ..contracts import AnalyzerContract, AnalyzerOutput, DiagnosticObservation, ObservationKind
from ..profiling import profile_views

LAYERS = ("layers/L8", "layers/L9", "layers/L10", "layers/L11", "layers/L12")
TOKENS = (
    "requested", "allowed", "left", "right", "wheel", "motor", "encoder", "velocity",
    "mps", "normalized", "output", "saturated", "enabled", "quality", "health", "fault",
)

CONTRACT = AnalyzerContract(
    analyzer_id="drive",
    description="L8-L12 drive command chain plus wheel/encoder/motor sensor evidence.",
    required_views=LAYERS,
    required_topics=("/r2b4/tick",),
)


def _number(value):
    return float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) else None


def _pair_activity(context, view: str, left_key: str, right_key: str, epsilon: float = 1e-9):
    counts = Counter()
    for row in context.bundle.iter_view(view):
        payload = row.payload if isinstance(row.payload, dict) else {}
        left, right = _number(payload.get(left_key)), _number(payload.get(right_key))
        if left is None or right is None:
            counts["missing_pair"] += 1
            continue
        counts["pairs"] += 1
        if abs(left) <= epsilon and abs(right) <= epsilon:
            counts["both_zero"] += 1
        else:
            counts["any_nonzero"] += 1
        if left * right < 0:
            counts["opposite_sign"] += 1
        elif left * right > 0:
            counts["same_nonzero_sign"] += 1
    return dict(counts)


def run(context, registry) -> AnalyzerOutput:
    del registry
    sensor_views = tuple(
        view for view in context.bundle.view_names()
        if view.startswith("sensors/") and any(token in view.lower() for token in ("encoder", "wheel", "motor"))
    )
    views = (*LAYERS, *sensor_views)
    profiles = profile_views(context.bundle, views, tokens=TOKENS, max_examples=1, top_values=10)
    activity = {
        "l10_wheel_targets": _pair_activity(context, "layers/L10", "left_mps", "right_mps"),
        "l11_normalized": _pair_activity(context, "layers/L11", "left_normalized", "right_normalized"),
        "l12_outputs": _pair_activity(context, "layers/L12", "left_output", "right_output"),
    }
    observations = (
        DiagnosticObservation(
            kind=ObservationKind.DERIVED_MEASUREMENT,
            code="DRIVE_COMMAND_ACTIVITY_COUNTS",
            message="Wheel-target, normalized-actuator and final-output zero/nonzero/sign relationships are counted independently per layer.",
            values=activity,
        ),
    )
    return AnalyzerOutput(
        metrics={"sensor_views": list(sensor_views), "profiles": profiles, "pair_activity": activity},
        observations=observations,
    )
