"""The two yes/no questions asked of a decision model about every candidate drop.

  real_drop    did the robot really drop an item, or is this a false alarm?
  recoverable  if it is real, can the arm safely pick it up and put it back?

The final action follows from the two answers:
    not real_drop                 -> ignore            (false alarm)
    real_drop and recoverable     -> propose_recovery  (admin approves, robot recovers)
    real_drop and not recoverable -> escalate          (a person must deal with it)

One definition serves inference and training export, so they cannot drift apart.
"""

from __future__ import annotations

from overwatch.events import ACTIONS

QUESTIONS: dict[str, dict] = {
    "real_drop": {
        "type": "noul",
        "instructions": (
            "A camera watches a table while a robot picks items. Did the robot really drop a small item "
            "on the table? Answer false for a false alarm: a shadow, reflection, lighting change, a "
            "person's hand, or a part of the robot itself."
        ),
    },
    "recoverable": {
        "type": "noul",
        "instructions": (
            "Is this a single small rigid item that a tabletop robot arm can safely pick up and put back? "
            "Answer false for liquids, spills, broken or sharp debris, or anything large."
        ),
    },
}


def action_from_answers(real_drop: bool, recoverable: bool) -> str:
    if not real_drop:
        return "ignore"
    return "propose_recovery" if recoverable else "escalate"


assert set(("ignore", "propose_recovery", "escalate")) == set(ACTIONS)

# What an admin verdict says the correct answers were.
VERDICT_ACTION = {"approve_recovery": "propose_recovery", "escalate": "escalate", "dismiss": "ignore"}


def expected_from_verdict(verdict: str, severity: int | None = None) -> dict:
    """Gold labels for a training row, or {} if the verdict carries none."""
    if verdict == "approve_recovery":
        return {"real_drop": True, "recoverable": True}
    if verdict == "escalate":
        return {"real_drop": True, "recoverable": False}
    if verdict == "dismiss":                      # a false alarm says nothing about recoverability
        return {"real_drop": False}
    return {}
