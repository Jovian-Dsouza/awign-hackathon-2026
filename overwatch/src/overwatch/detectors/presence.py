"""Is the robot in view and moving? (the fallback when no tag is on the robot)

Frame-to-frame motion only, no reference picture: a robot-sized region that is moving means
"the robot is working", so the watch window opens. A camera being adjusted by hand moves the
WHOLE picture, which is rejected (that is not the robot). A person walking through is also
robot-sized: put the tag on the robot to tell them apart exactly.
"""

from __future__ import annotations

import cv2
import numpy as np

WORK_W = 160


class MotionPresence:
    def __init__(self, min_frac: float = 0.008, max_global_frac: float = 0.5, diff: int = 22,
                 need: int = 3, window: int = 8):
        self.min_frac, self.max_global_frac, self.diff = min_frac, max_global_frac, diff
        self.need, self.window = need, window
        self._prev: np.ndarray | None = None
        self._hist: list[bool] = []
        self.active = False

    def process(self, bgr: np.ndarray) -> bool:
        h, w = bgr.shape[:2]
        g = cv2.cvtColor(cv2.resize(bgr, (WORK_W, max(1, int(h * WORK_W / w))), interpolation=cv2.INTER_AREA),
                         cv2.COLOR_BGR2GRAY)
        g = cv2.GaussianBlur(g, (5, 5), 0)
        moving = False
        if self._prev is not None and self._prev.shape == g.shape:
            d = cv2.absdiff(g, self._prev)
            m = (d > self.diff).astype(np.uint8)
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
            frac = float(m.sum()) / m.size
            if frac <= self.max_global_frac:                   # not the whole picture shifting
                n, _, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
                biggest = max((int(stats[i, cv2.CC_STAT_AREA]) for i in range(1, n)), default=0)
                moving = biggest / m.size >= self.min_frac
        self._prev = g
        self._hist = (self._hist + [moving])[-self.window:]
        self.active = sum(self._hist) >= self.need
        return self.active
