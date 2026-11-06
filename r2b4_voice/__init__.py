"""Host-side R2B4 voice and wake orchestration.

The package is deliberately outside the production L0-L12 control layers.  It
may consume edge audio and use public operator/runtime boundaries, but it owns no
motor, safety, mission or production lifecycle state.
"""

__all__: list[str] = []
