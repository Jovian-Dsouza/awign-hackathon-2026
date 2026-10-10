"""Dropped-item detector.

Question it answers: "since the picture was taken (the moment the robot started picking),
has a small, compact object appeared in the watched area and stopped moving?"

That is deliberately narrow. It is NOT "anything that changed":
  * the robot is far bigger than an item and keeps moving      -> too big, ignored
  * a person's arm or body is big, ragged and moving            -> ignored
  * sensor noise, a flicker of light                            -> too small, ignored
  * an item leaving the rack, or arriving in the tray           -> draw those as ignore zones
Only blobs that are item-sized AND compact AND sit still for `persist_s` are reported.

Sizes come from the team's own sim frames (robot-sim/renders): at that camera distance a
soup can is ~0.05 % of the picture and the robot is 2 %+. Tune `item_min_frac` /
`item_max_frac` to the real mount; every incident records its measured size.

Cheap on purpose (no network): it runs every frame on a laptop CPU.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np

from overwatch.events import CAMERA_FAULT, DROPPED_ITEM, Observation
from overwatch.zones import ZoneSet


@dataclass
class _Track:
    id: int
    anchor: tuple[float, float]
    centroid: tuple[float, float]
    first_seen: float
    last_seen: float
    reported: bool = False


@dataclass
class Blob:
    """What the overlay draws: an item-sized candidate, whether or not it has been reported yet."""
    bbox: tuple[int, int, int, int]   # full-res x, y, w, h
    track_id: int
    age_s: float
    reported: bool


@dataclass
class Ignored:
    """Why changes were NOT turned into candidates (shown in status so spam can be explained)."""
    too_big: int = 0
    too_small: int = 0
    not_compact: int = 0
    outside_zone: int = 0
    counts: dict[str, int] = field(default_factory=dict)


class SceneChangeDetector:
    def __init__(self, zones: ZoneSet, diff_threshold: int = 28,
                 item_min_frac: float = 0.0002, item_max_frac: float = 0.015,
                 max_aspect: float = 4.0, min_solidity: float = 0.6,
                 persist_s: float = 3.0, static_frac: float = 0.02, lost_s: float = 1.5,
                 max_area_frac: float = 0.45, max_shift_frac: float = 0.008, min_response: float = 0.1,
                 work_w: int = 640):
        self.zones = zones
        self.diff_threshold = diff_threshold
        self.item_min_frac, self.item_max_frac = item_min_frac, item_max_frac
        self.max_aspect, self.min_solidity = max_aspect, min_solidity
        self.persist_s, self.static_frac, self.lost_s = persist_s, static_frac, lost_s
        self.max_area_frac = max_area_frac
        self.max_shift_frac, self.min_response = max_shift_frac, min_response
        self.work_w = work_w
        self.alignment: dict[str, float] = {}
        self.ignored = Ignored()

        self._ref: np.ndarray | None = None
        self._ref_gray: np.ndarray | None = None
        self._win: np.ndarray | None = None
        self._ref_size: tuple[int, int] | None = None
        self._tracks: dict[int, _Track] = {}
        self._next_id = 1
        self._shift_reported = False
        self.blobs: list[Blob] = []

    # -- baseline ------------------------------------------------------------
    @property
    def has_reference(self) -> bool:
        return self._ref is not None

    def set_reference(self, bgr: np.ndarray) -> None:
        """The picture to compare against: the arena as it looked when the robot started picking."""
        self._ref = self._prep(bgr)
        self._ref_gray = self._gray32(self._ref)
        self._win = cv2.createHanningWindow((self._ref.shape[1], self._ref.shape[0]), cv2.CV_32F)
        self._ref_size = (bgr.shape[1], bgr.shape[0])
        self._tracks.clear()
        self.blobs = []
        self._shift_reported = False
        self.alignment = {}
        self.ignored = Ignored()

    def clear_reference(self) -> None:
        self._ref, self._ref_gray, self._ref_size = None, None, None
        self._tracks.clear()
        self.blobs = []
        self.alignment = {}

    @staticmethod
    def _gray32(prepped: np.ndarray) -> np.ndarray:
        return cv2.cvtColor(np.clip(prepped, 0, 255).astype(np.uint8), cv2.COLOR_BGR2GRAY).astype(np.float32)

    def _prep(self, bgr: np.ndarray) -> np.ndarray:
        """Blurred, still in colour: packaging often matches the floor in brightness but not hue."""
        h, w = bgr.shape[:2]
        if w > self.work_w:
            bgr = cv2.resize(bgr, (self.work_w, max(1, int(h * self.work_w / w))), interpolation=cv2.INTER_AREA)
        return cv2.GaussianBlur(bgr, (5, 5), 0).astype(np.int16)

    # -- per frame -----------------------------------------------------------
    def process(self, bgr: np.ndarray, now: float, exclude: np.ndarray | None = None
                ) -> tuple[list[Observation], list[int]]:
        """Returns (new dropped-item observations, ids of reported tracks whose item is gone)."""
        if self._ref is None or self._ref_size is None:
            return [], []
        if (bgr.shape[1], bgr.shape[0]) != self._ref_size:
            if self._shift_reported:
                return [], []
            self._shift_reported = True
            return [self._fault(now, "reference_mismatch",
                                "Camera resolution changed during the watch. Detection paused.")], []

        cur = self._prep(bgr)
        hh, ww = cur.shape[:2]

        # Has the camera itself moved? Then every edge looks "new": pause and say so, once.
        # .copy(): phaseCorrelate multiplies its inputs by the window IN PLACE when the size is an
        # optimal FFT size, which would erode the stored reference a little on every frame.
        (dx, dy), response = cv2.phaseCorrelate(self._ref_gray.copy(), self._gray32(cur), self._win)
        shift = math.hypot(dx, dy)
        self.alignment = {"shift_px": round(shift, 1), "match": round(float(response), 2)}
        if shift > self.max_shift_frac * ww or response < self.min_response:
            self.blobs = []
            self._tracks.clear()     # no "item gone" events: the camera moved, the item did not
            if self._shift_reported:
                return [], []
            self._shift_reported = True
            return [self._fault(
                now, "scene_shift",
                f"Camera view moved during the watch (shifted {shift:.1f} px). Detection paused until it is "
                "back in place.")], []

        diff = cur - self._ref
        diff -= np.median(diff.reshape(-1, 3), axis=0).astype(np.int16)   # auto-exposure / white balance drift
        mag = np.abs(diff).max(axis=2)                                    # biggest change in any colour channel
        mask = (mag > self.diff_threshold).astype(np.uint8) * 255
        watch = self.zones.mask(ww, hh)          # NOT applied yet: a robot half inside an ignore zone
        if exclude is not None:                  # must still be measured as the whole robot
            ex = cv2.resize(exclude, (ww, hh), interpolation=cv2.INTER_NEAREST)
            mask[ex > 0] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        observations: list[Observation] = []
        changed_frac = float(((mask > 0) & (watch > 0)).sum()) / max(1, int((watch > 0).sum()))
        if changed_frac > self.max_area_frac:
            self.blobs = []
            if not self._shift_reported:
                self._shift_reported = True
                observations.append(self._fault(
                    now, "scene_shift",
                    f"{changed_frac:.0%} of the view changed at once (camera moved or lighting changed). "
                    "Detection paused."))
            return observations, []
        self._shift_reported = False

        n, labels, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
        seen: set[int] = set()
        blobs: list[Blob] = []
        ig = Ignored()
        sx, sy = bgr.shape[1] / ww, bgr.shape[0] / hh
        static_px = self.static_frac * ww
        for i in range(1, n):
            x, y, w, h, area = (int(v) for v in stats[i])
            frac = area / (ww * hh)
            if frac < self.item_min_frac:
                ig.too_small += 1
                continue
            if frac > self.item_max_frac:
                ig.too_big += 1          # the robot, a person, a big lighting change
                continue
            aspect = max(w, h) / max(1, min(w, h))
            if aspect > self.max_aspect:
                ig.not_compact += 1      # a thin streak: an arm segment, a cable, a shadow edge
                continue
            sub_in_watch = (labels[y:y + h, x:x + w] == i) & (watch[y:y + h, x:x + w] > 0)
            if sub_in_watch.sum() < 0.5 * area:  # mostly in a rack / tray / outside the table: not a drop
                ig.outside_zone += 1
                continue
            c = (float(cents[i][0]), float(cents[i][1]))
            track = self._match(c, seen, static_px * 2)
            if track is None:
                track = _Track(self._next_id, c, c, now, now)
                self._next_id += 1
                self._tracks[track.id] = track
            seen.add(track.id)
            track.last_seen, track.centroid = now, c
            if math.dist(c, track.anchor) > static_px:       # it moved: restart the stillness clock
                track.anchor, track.first_seen = c, now
            full = (int(x * sx), int(y * sy), max(1, int(w * sx)), max(1, int(h * sy)))
            age = now - track.first_seen
            blobs.append(Blob(full, track.id, age, track.reported))

            if age >= self.persist_s and not track.reported:
                feats = self._features(mask, cur, mag, (x, y, w, h), area, ww, hh)
                if feats["solidity"] < self.min_solidity:
                    ig.not_compact += 1
                    continue
                track.reported = True
                zone = self.zones.zone_at(c[0] / ww, c[1] / hh)
                feats.update(stationary_s=round(age, 1))
                observations.append(Observation(
                    kind=DROPPED_ITEM, zone=zone, ts=now,
                    summary=(f"Small object ({feats['area_pct']}% of the view) appeared and has been still for "
                             f"{age:.0f}s in '{zone}' during the pickup"),
                    features=feats, bbox=full, centroid=(c[0] / ww, c[1] / hh), track_id=track.id,
                ))
        self.blobs = blobs
        self.ignored = ig

        gone: list[int] = []
        for tid in [t for t, tr in self._tracks.items() if now - tr.last_seen > self.lost_s]:
            if self._tracks[tid].reported:
                gone.append(tid)
            del self._tracks[tid]
        return observations, gone

    # -- helpers -------------------------------------------------------------
    def _match(self, c: tuple[float, float], taken: set[int], radius: float) -> _Track | None:
        best, best_d = None, radius
        for t in self._tracks.values():
            if t.id in taken:
                continue
            d = math.dist(c, t.centroid)
            if d <= best_d:
                best, best_d = t, d
        return best

    @staticmethod
    def _features(mask: np.ndarray, cur: np.ndarray, mag: np.ndarray, box: tuple[int, int, int, int],
                  area: int, ww: int, hh: int) -> dict:
        x, y, w, h = box
        sub = mask[y:y + h, x:x + w]
        cnts, _ = cv2.findContours(sub, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        solidity = 1.0
        if cnts:
            cnt = max(cnts, key=cv2.contourArea)
            hull = cv2.contourArea(cv2.convexHull(cnt))
            solidity = float(cv2.contourArea(cnt)) / hull if hull > 0 else 1.0
        g8 = cv2.cvtColor(np.clip(cur[y:y + h, x:x + w], 0, 255).astype(np.uint8), cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(g8, 40, 120)
        inside = sub > 0
        return {
            "area_pct": round(100.0 * area / (ww * hh), 3),
            "solidity": round(solidity, 2),
            "aspect": round(max(w, h) / max(1, min(w, h)), 1),
            "edge_density": round(float((edges > 0)[inside].mean()) if inside.any() else 0.0, 3),
            "contrast": round(float(mag[y:y + h, x:x + w][inside].mean()) if inside.any() else 0.0, 1),
        }

    @staticmethod
    def _fault(now: float, fault: str, message: str) -> Observation:
        return Observation(kind=CAMERA_FAULT, zone="camera", ts=now, summary=message, features={"fault": fault})
