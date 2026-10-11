"""Keep an eye on the robot: find it by an ArUco tag stuck on top of it (DICT_4X4_50).

With the tag the system knows exactly (a) whether the robot is in the picture at all,
(b) where it is, so the robot's own body is never mistaken for a dropped item or a spill.

Generate the tag with:  uv run python -m overwatch.detectors.robot_marker  (writes robot_tag.png)
Print it (about 5 cm square, with the white border), stick it flat on top of the robot, and
set OVERWATCH_ROBOT_MARKER_ID=0.
"""

from __future__ import annotations

import cv2
import numpy as np


class RobotMarker:
    def __init__(self, marker_id: int, grow: float = 4.0):
        self.marker_id, self.grow = marker_id, grow
        self.enabled = marker_id >= 0
        self.visible = False
        self.center: tuple[float, float] | None = None      # pixels
        if self.enabled:
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self._detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())

    def observe(self, bgr: np.ndarray) -> np.ndarray | None:
        """Updates `visible`/`center`. Returns the robot's region (tag box grown by `grow`) as a
        255-mask the size of `bgr`, or None if the tag is not seen or tracking is off."""
        self.visible, self.center = False, None
        if not self.enabled:
            return None
        corners, ids, _ = self._detector.detectMarkers(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
        if ids is None:
            return None
        for quad, mid in zip(corners, ids.flatten()):
            if int(mid) != self.marker_id:
                continue
            pts = quad.reshape(4, 2)
            c = pts.mean(axis=0)
            grown = (pts - c) * self.grow + c
            region = np.zeros(bgr.shape[:2], np.uint8)
            cv2.fillConvexPoly(region, grown.astype(np.int32), 255)
            self.visible, self.center = True, (float(c[0]), float(c[1]))
            return region
        return None


def write_tag(path: str = "robot_tag.png", marker_id: int = 0, size_px: int = 700) -> str:
    """A printable tag with a wide white border (the detector needs the quiet zone)."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    tag = cv2.aruco.generateImageMarker(dictionary, marker_id, size_px)
    border = size_px // 5
    img = cv2.copyMakeBorder(tag, border, border, border, border, cv2.BORDER_CONSTANT, value=255)
    cv2.imwrite(path, img)
    return path


if __name__ == "__main__":
    print("wrote", write_tag())
