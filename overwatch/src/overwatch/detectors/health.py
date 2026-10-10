"""Is the monitor itself trustworthy? A camera that is covered, frozen or pointed at a
blank wall silently reports "all clear", which is the worst failure for a monitor.

Each fault must hold for `hold_s` before it is reported, and is reported once per
onset. `active` lists the faults currently present.
"""

from __future__ import annotations

import cv2
import numpy as np

from overwatch.events import CAMERA_FAULT, Observation

WORK_W = 320


class HealthMonitor:
    def __init__(self, black_luma: float = 8.0, frozen_diff: float = 0.12, flat_lapvar: float = 6.0,
                 flat_std: float = 14.0, hold_s: float = 5.0):
        self.black_luma, self.frozen_diff = black_luma, frozen_diff
        self.flat_lapvar, self.flat_std = flat_lapvar, flat_std
        self.hold_s = hold_s
        self._prev: np.ndarray | None = None
        self._since: dict[str, float] = {}
        self._reported: set[str] = set()
        self.metrics: dict[str, float] = {}

    @property
    def active(self) -> set[str]:
        return set(self._reported)

    def reset(self) -> None:
        self._prev, self._since, self._reported = None, {}, set()

    def process(self, bgr: np.ndarray, now: float) -> list[Observation]:
        h, w = bgr.shape[:2]
        small = cv2.resize(bgr, (WORK_W, max(1, int(h * WORK_W / w))), interpolation=cv2.INTER_AREA)
        g = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

        luma = float(g.mean())
        std = float(g.std())
        lapvar = float(cv2.Laplacian(g, cv2.CV_64F).var())
        move = float(np.abs(g.astype(np.int16) - self._prev.astype(np.int16)).mean()) if self._prev is not None else 99.0
        self._prev = g
        self.metrics = {"luma": round(luma, 1), "contrast": round(std, 1),
                        "detail": round(lapvar, 1), "frame_delta": round(move, 2)}

        conditions = {
            "black": luma < self.black_luma,
            "frozen": move < self.frozen_diff,
            # A real scene has edges. A smooth grey frame means lens covered, too close, or a plain wall.
            "no_scene_detail": lapvar < self.flat_lapvar and std < self.flat_std and luma >= self.black_luma,
        }
        messages = {
            "black": "Camera image is black: lens covered or the device is not really streaming.",
            "frozen": "Camera image has not changed at all: feed is frozen.",
            "no_scene_detail": "Camera sees almost no detail: lens covered, too close to a surface, out of "
                               "focus, or pointed at a blank wall. The monitor cannot see mistakes like this.",
        }
        out: list[Observation] = []
        for name, bad in conditions.items():
            if bad:
                first = self._since.setdefault(name, now)
                if now - first >= self.hold_s and name not in self._reported:
                    self._reported.add(name)
                    out.append(Observation(
                        kind=CAMERA_FAULT, zone="camera", ts=now, summary=messages[name],
                        features={"fault": name, **self.metrics, "held_s": round(now - first, 1)},
                    ))
            else:
                self._since.pop(name, None)
                self._reported.discard(name)
        return out
