"""What the camera side tells the decision side.

An Observation is a structured fact ("a 0.6 %-of-frame object has sat still in the
floor zone for 4 s"), never a verdict. Deciders turn observations into Decisions.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any

# Observation kinds
DROPPED_ITEM = "dropped_item"       # a small item appeared and stopped moving while the robot was picking
SPILL = "spill"                     # a larger irregular patch appeared: a mess the robot cannot fix
CAMERA_FAULT = "camera_fault"       # the monitor itself cannot be trusted right now


@dataclass
class Observation:
    kind: str
    zone: str
    ts: float
    summary: str
    features: dict[str, Any] = field(default_factory=dict)
    bbox: tuple[int, int, int, int] | None = None      # x, y, w, h in full-resolution pixels
    centroid: tuple[float, float] | None = None        # normalised 0..1 (x, y)
    track_id: int | None = None

    def to_state(self) -> dict[str, Any]:
        """What a text decision model reads: one plain sentence (see decision/describe.py)."""
        from overwatch.decision.describe import describe
        return {"report": describe(self.kind, self.features, self.summary)}

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class Decision:
    action: str                       # ignore | propose_recovery | escalate
    severity: int                     # 0 harmless .. 3 critical
    recoverable: bool
    reasons: list[str]
    decider: str
    p_recoverable: float | None = None
    confidence: float | None = None
    latency_ms: float | None = None
    extra: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


ACTIONS = ("ignore", "propose_recovery", "escalate")
SEVERITIES = ("harmless", "minor", "major", "critical")
