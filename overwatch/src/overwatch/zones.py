"""Watch / ignore regions, stored as normalised polygons so they survive a resolution change."""

from __future__ import annotations

import json
import threading
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


@dataclass
class Zone:
    name: str
    points: list[tuple[float, float]]   # normalised 0..1
    mode: str = "watch"                 # watch | ignore

    def to_dict(self) -> dict:
        return {"name": self.name, "points": [list(p) for p in self.points], "mode": self.mode}

    @staticmethod
    def from_dict(d: dict) -> "Zone":
        pts = [(float(x), float(y)) for x, y in d["points"]]
        if len(pts) < 3:
            raise ValueError("a zone needs at least 3 points")
        if any(not (0.0 <= v <= 1.0) for p in pts for v in p):
            raise ValueError("zone points must be normalised to 0..1")
        mode = d.get("mode", "watch")
        if mode not in ("watch", "ignore"):
            raise ValueError("zone mode must be 'watch' or 'ignore'")
        name = str(d.get("name") or "").strip()
        if not name:
            raise ValueError("a zone needs a name")
        return Zone(name=name, points=pts, mode=mode)


class ZoneSet:
    """No zones = watch the whole frame as one zone called "scene"."""

    def __init__(self, zones: list[Zone] | None = None):
        self._lock = threading.Lock()
        self._zones: list[Zone] = list(zones or [])

    @property
    def zones(self) -> list[Zone]:
        with self._lock:
            return list(self._zones)

    def replace(self, zones: list[Zone]) -> None:
        with self._lock:
            self._zones = list(zones)

    def mask(self, width: int, height: int) -> np.ndarray:
        """uint8 mask, 255 where the detector may look."""
        zones = self.zones
        watch = [z for z in zones if z.mode == "watch"]
        out = np.zeros((height, width), np.uint8) if watch else np.full((height, width), 255, np.uint8)
        for z in watch:
            cv2.fillPoly(out, [self._px(z, width, height)], 255)
        for z in zones:
            if z.mode == "ignore":
                cv2.fillPoly(out, [self._px(z, width, height)], 0)
        return out

    def zone_at(self, x: float, y: float) -> str:
        """Name of the watch zone containing the normalised point, else "scene"."""
        for z in self.zones:
            if z.mode != "watch":
                continue
            poly = np.array(z.points, np.float32)
            if cv2.pointPolygonTest(poly, (float(x), float(y)), False) >= 0:
                return z.name
        return "scene"

    @staticmethod
    def _px(z: Zone, w: int, h: int) -> np.ndarray:
        return np.array([[int(x * (w - 1)), int(y * (h - 1))] for x, y in z.points], np.int32)

    # persistence -----------------------------------------------------------
    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps([z.to_dict() for z in self.zones], indent=2))

    @classmethod
    def load(cls, path: Path) -> "ZoneSet":
        if not path.is_file():
            return cls()
        return cls([Zone.from_dict(d) for d in json.loads(path.read_text())])
