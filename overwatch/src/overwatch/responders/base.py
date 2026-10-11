"""Responders turn an admin-approved decision into something the robot does.

Two verbs only, because those are the two things the Pi can safely do today:
  * hold()    stop the base (all-zero drive command);
  * recover() run a pre-recorded recovery motion.

`NullResponder` is the default: it does nothing and says so. A responder never decides
anything; the admin's click (or the explicit AUTO_HOLD setting for `hold`) does.
"""

from __future__ import annotations

from typing import Any, Protocol


class Responder(Protocol):
    name: str
    dry_run: bool

    def hold(self) -> dict[str, Any]: ...
    def recover(self, plan: dict[str, Any]) -> dict[str, Any]: ...
    def status(self) -> dict[str, Any]: ...


class NullResponder:
    name = "null"
    dry_run = True

    def hold(self) -> dict[str, Any]:
        return {"ok": True, "dry_run": True, "detail": "Dry run: no robot is connected, nothing was sent."}

    def recover(self, plan: dict[str, Any]) -> dict[str, Any]:
        return {"ok": True, "dry_run": True,
                "detail": "Dry run: recovery approved and recorded, but no robot is connected. "
                          "Set OVERWATCH_RESPONDER=pi to send it to the Pi."}

    def status(self) -> dict[str, Any]:
        return {"name": self.name, "dry_run": True, "reachable": None}
