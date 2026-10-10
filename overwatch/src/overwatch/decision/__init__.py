"""Decision layer: observation in, decision out.

`Decider` is the seam. RulesDecider always runs; LayaDecider (optional) answers the same
case in parallel. Another model is one more class with `decide(observation, ctx)`.
"""

from __future__ import annotations

from typing import Protocol

from overwatch.events import Decision, Observation

# A candidate Laya is this sure is NOT a real drop gets filtered (mode "filter").
FALSE_ALARM_BELOW = 0.25


class Decider(Protocol):
    name: str

    def decide(self, obs: Observation, ctx: dict | None = None) -> Decision | None: ...


def combine(rules: Decision, model: Decision | None, mode: str) -> Decision:
    """Final decision from the rules floor and an optional model.

    mode "off" / "shadow": the rules decide; the model is only recorded alongside.
    mode "filter":         the model may call a rules-approved drop a false alarm when it is
                           confident (P(real drop) below FALSE_ALARM_BELOW), and may say a real drop
                           is not recoverable (-> a person). It never turns an alert into a
                           recovery the rules did not offer, and never touches camera faults.
    """
    if model is None or mode != "filter" or rules.action == "escalate":
        return rules
    p_real = model.extra.get("p_real_drop")
    if p_real is not None and p_real < FALSE_ALARM_BELOW:
        return Decision("ignore", 0, False,
                        [f"Laya judged this a likely false alarm (P(real drop) = {p_real:.0%})."] + rules.reasons,
                        "rules+laya", p_recoverable=model.p_recoverable, confidence=model.confidence,
                        latency_ms=model.latency_ms, extra={"p_real_drop": p_real})
    if model.action == "escalate":
        return Decision("escalate", max(rules.severity, model.severity), False,
                        rules.reasons + model.reasons, "rules+laya", p_recoverable=model.p_recoverable,
                        confidence=model.confidence, latency_ms=model.latency_ms,
                        extra={"p_real_drop": p_real})
    return rules
