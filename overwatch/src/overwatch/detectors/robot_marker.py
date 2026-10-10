"""Find the robot by an ArUco tag stuck on top of it (DICT_4X4_50), so the scene-change
detector can ignore the robot itself instead of calling it a foreign object.

Without a tag the detector still works: a moving robot never sits still long enough to
be reported, but a robot that pauses for `persist_s` would be. The tag removes that case.
"""

from __future__ import annotations

import cv2
import numpy as np


class RobotMarker:
    def __init__(self, marker_id: int, grow: float = 3.0):
        self.marker_id, self.grow = marker_id, grow
        self.enabled = marker_id >= 0
        self.visible = False
        if self.enabled:
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self._detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())

    def mask(self, bgr: np.ndarray) -> np.ndarray | None:
        """255 where the robot is (marker box grown by `grow`), same HxW as `bgr`; None if not visible."""
        self.visible = False
        if not self.enabled:
            return None
        corners, ids, _ = self._detector.detectMarkers(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
        if ids is None:
            return None
        for quad, mid in zip(corners, ids.flatten()):
            if int(mid) != self.marker_id:
                continue
            pts = quad.reshape(4, 2)
            center = pts.mean(axis=0)
            grown = (pts - center) * self.grow + center
            mask = np.zeros(bgr.shape[:2], np.uint8)
            cv2.fillConvexPoly(mask, grown.astype(np.int32), 255)
            self.visible = True
            return mask
        return None
