"""The deterministic floor. Always runs, needs no model.

By the time a drop reaches here the detector has already required: the robot was picking,
the thing is item-sized and compact, it appeared during the pickup and has stopped moving.
So the rules only decide what to offer: a recovery, or a person.
"""

from __future__ import annotations

from overwatch.events import CAMERA_FAULT, DROPPED_ITEM, Decision, Observation

MAX_PICKABLE_PCT = 1.0       # of the picture: bigger than this is not one small item the arm can carry


class RulesDecider:
    name = "rules"

    def decide(self, obs: Observation, ctx: dict | None = None) -> Decision:
        if obs.kind == CAMERA_FAULT:
            fault = obs.features.get("fault", "unknown")
            return Decision("escalate", 1, False,
                            [f"The monitor camera cannot be trusted ({fault}); drops may go unseen."],
                            self.name)
        if obs.kind != DROPPED_ITEM:
            return Decision("escalate", 1, False, [f"Unknown observation kind '{obs.kind}'"], self.name)

        area = float(obs.features.get("area_pct", 0.0))
        if area > MAX_PICKABLE_PCT:
            return Decision("escalate", 2, False,
                            [f"Something appeared during the pickup but it covers {area:.2f}% of the view, "
                             "too big to be one small item. A person should look."], self.name)
        return Decision("propose_recovery", 1, True,
                        [f"A small item ({area:.2f}% of the view) appeared during the pickup and stayed put: "
                         "looks like a dropped item. Needs admin approval before the robot moves."], self.name)
