"""When is the robot picking? Drop alerts only count inside this window.

A "drop" only means something right after the robot picked something up. Outside a pickup,
nothing new on the table is the robot's fault, so the detector is simply not asked.

The window is open while ANY of these says the robot is working, plus a grace period after
it stops (a dropped item can land a few seconds after the arm lets go):

  * a person pressed "Robot picking" on the dashboard (manual, for tests and demos)
  * UthaLe  GET /api/robot/status   mode == "executing"
  * the Pi  GET :8003/status        arm mode "play", or the base is being driven
  * ACT     GET :8004/state         mode == "driving"

`epoch` increases every time the window opens, so the pipeline can take a fresh baseline
picture of the arena at that moment.
"""

from __future__ import annotations

import json
import logging
import threading
import time
import urllib.error
import urllib.request

log = logging.getLogger(__name__)


class WatchWindow:
    def __init__(self, grace_s: float = 20.0, manual_s: float = 90.0):
        self.grace_s, self.manual_s = grace_s, manual_s
        self._lock = threading.Lock()
        self._manual_until = 0.0
        self._grace_until = 0.0
        self._signals: dict[str, tuple[bool, str]] = {}
        self._was_watching = False
        self._opened_at: float | None = None
        self.epoch = 0

    # -- inputs -------------------------------------------------------------
    def arm_manual(self, seconds: float | None = None, now: float | None = None) -> None:
        now = time.time() if now is None else now
        with self._lock:
            self._manual_until = now + (seconds if seconds and seconds > 0 else self.manual_s)

    def disarm(self, now: float | None = None) -> None:
        """Pickup finished: stop the manual window now, keep watching for the grace period."""
        now = time.time() if now is None else now
        with self._lock:
            if self._manual_until > now:
                self._manual_until = now
                self._grace_until = now + self.grace_s

    def set_signal(self, source: str, active: bool, detail: str = "", now: float | None = None) -> None:
        now = time.time() if now is None else now
        with self._lock:
            was = self._signals.get(source, (False, ""))[0]
            self._signals[source] = (active, detail)
            if was and not active:                       # the robot just finished: drops may still land
                self._grace_until = max(self._grace_until, now + self.grace_s)

    # -- output -------------------------------------------------------------
    def state(self, now: float | None = None) -> dict:
        now = time.time() if now is None else now
        with self._lock:
            live = [s for s, (a, _) in self._signals.items() if a]
            manual = now < self._manual_until
            grace = now < self._grace_until
            watching = bool(live) or manual or grace
            if watching and not self._was_watching:
                self.epoch += 1
                self._opened_at = now
            if not watching:
                self._opened_at = None
            self._was_watching = watching
            if live:
                why = "robot is working (" + ", ".join(live) + ")"
            elif manual:
                why = "pickup started from the dashboard"
            elif grace:
                why = "pickup just ended: still watching for a late drop"
            else:
                why = "robot is not picking: not watching for drops"
            ends = max(self._manual_until, self._grace_until) if not live else None
            return {
                "watching": watching, "why": why, "epoch": self.epoch, "opened_at": self._opened_at,
                "seconds_left": round(max(0.0, ends - now), 1) if (watching and ends) else None,
                "sources": {s: {"active": a, "detail": d} for s, (a, d) in self._signals.items()},
            }


def _get_json(url: str, timeout: float = 1.5):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return json.loads(r.read())


class RobotActivityPoller:
    """Polls whichever robot endpoints are configured and feeds the WatchWindow."""

    def __init__(self, window: WatchWindow, uthale_url: str = "", pi_url: str = "", act_url: str = "",
                 period_s: float = 1.0):
        self.window, self.period_s = window, period_s
        self.targets: dict[str, tuple[str, callable]] = {}
        if uthale_url:
            self.targets["uthale"] = (uthale_url.rstrip("/") + "/api/robot/status",
                                      lambda d: (d.get("mode") == "executing", d.get("current_task_title") or ""))
        if pi_url:
            self.targets["pi"] = (pi_url.rstrip("/") + "/status", self._pi_active)
        if act_url:
            self.targets["act"] = (act_url.rstrip("/") + "/state",
                                   lambda d: (d.get("mode") == "driving", str(d.get("mode"))))
        self._stop = threading.Event()
        self._t: threading.Thread | None = None

    @staticmethod
    def _pi_active(d: dict) -> tuple[bool, str]:
        if (d.get("arm") or {}).get("mode") == "play":
            return True, "arm trajectory playing"
        cmd = d.get("cmd") or {}
        if any(abs(float(cmd.get(k, 0) or 0)) > 0 for k in ("vx", "vy", "wz")):
            return True, "base driving"
        return False, ""

    def start(self) -> None:
        if not self.targets or (self._t and self._t.is_alive()):
            return
        self._stop.clear()
        self._t = threading.Thread(target=self._run, name="overwatch-robot-poll", daemon=True)
        self._t.start()

    def stop(self) -> None:
        self._stop.set()
        if self._t:
            self._t.join(2.0)

    def _run(self) -> None:
        warned: set[str] = set()
        while not self._stop.is_set():
            for name, (url, fn) in self.targets.items():
                try:
                    active, detail = fn(_get_json(url))
                    self.window.set_signal(name, active, detail)
                    warned.discard(name)
                except (urllib.error.URLError, OSError, ValueError, TimeoutError) as e:
                    self.window.set_signal(name, False, "unreachable")
                    if name not in warned:
                        warned.add(name)
                        log.warning("Robot activity source %s unreachable (%s): treating as not picking", name, e)
            self._stop.wait(self.period_s)
