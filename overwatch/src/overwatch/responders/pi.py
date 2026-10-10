"""Responder for the Raspberry Pi running firmware/combined-web/web_combined.py (:8003).

Endpoints used (verified against that file):
  POST /drive {vx, vy, wz}   all zeros = stop the base
  POST /play  {name}         play a recorded arm trajectory from ~/.so101_arm/trajs
  GET  /status               connectivity / serial link

Limits, stated plainly:
  * `hold` stops the wheels only. It does not freeze the arm and is NOT an emergency stop.
  * `recover` plays a *fixed* recorded motion. It cannot target an arbitrary dropped item;
    that needs pixel -> table calibration and the ACT policy (see README roadmap).
    If no trajectory is configured, recover() refuses instead of pretending.
"""

from __future__ import annotations

from typing import Any

import httpx


class PiResponder:
    name = "pi"
    dry_run = False

    def __init__(self, base_url: str, recover_traj: str = "", timeout: float = 4.0):
        self.base_url = base_url.rstrip("/")
        self.recover_traj = recover_traj
        self._client = httpx.Client(timeout=timeout)

    def _post(self, path: str, body: dict) -> dict[str, Any]:
        try:
            r = self._client.post(f"{self.base_url}{path}", json=body)
            data = r.json() if r.content else {}
            ok = r.status_code == 200 and bool(data.get("ok", True))
            return {"ok": ok, "dry_run": False, "status_code": r.status_code, "response": data,
                    "detail": "sent" if ok else f"Pi refused: {data.get('error', r.text[:120])}"}
        except (httpx.HTTPError, ValueError) as e:
            return {"ok": False, "dry_run": False, "detail": f"Could not reach the Pi at {self.base_url}: {e}"}

    def hold(self) -> dict[str, Any]:
        return self._post("/drive", {"vx": 0, "vy": 0, "wz": 0})

    def recover(self, plan: dict[str, Any]) -> dict[str, Any]:
        if not self.recover_traj:
            return {"ok": False, "dry_run": False,
                    "detail": "No recovery trajectory configured (OVERWATCH_PI_RECOVER_TRAJ). "
                              "Robot held; a person must put the item back."}
        out = self._post("/play", {"name": self.recover_traj})
        out["trajectory"] = self.recover_traj
        return out

    def status(self) -> dict[str, Any]:
        try:
            r = self._client.get(f"{self.base_url}/status", timeout=2.0)
            d = r.json()
            return {"name": self.name, "dry_run": False, "reachable": True,
                    "connected": d.get("connected"), "camera": d.get("camera")}
        except (httpx.HTTPError, ValueError) as e:
            return {"name": self.name, "dry_run": False, "reachable": False, "error": str(e)}
