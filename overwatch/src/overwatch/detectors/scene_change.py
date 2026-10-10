"""What did the robot leave behind?

While the robot is working, compare the picture with how the table looked when it started.
Something NEW that appeared and then stopped moving is one of two things:

  * a small, compact object      -> DROPPED_ITEM  (a packet, a bottle: the robot can pick it up)
  * a larger, irregular patch    -> SPILL         (a mess: a person must clean it)
                                    [needs the robot tag, so the robot's own body is excluded]

Everything else is ignored on purpose, and counted so the spam can be explained:
  * the robot itself: it is far bigger than an item, and with the tag the whole connected
    region around the tag is the robot and is skipped
  * noise and flicker: too small
  * rack and tray: draw them as ignore zones (items legitimately change there)
  * anything that keeps moving never sits still long enough to be reported

Sizes come from the team's own sim frames (robot-sim/renders): at that camera distance a soup
can is ~0.05-0.1 % of the picture and the robot 2 %+. Tune ITEM_MIN/MAX_FRAC to the real mount.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

from overwatch.events import DROPPED_ITEM, SPILL, Observation
from overwatch.zones import ZoneSet


@dataclass
class _Track:
    id: int
    anchor: tuple[float, float]
    centroid: tuple[float, float]
    first_seen: float
    last_seen: float
    kind: str
    reported: bool = False
    rejected: bool = False     # checked once: it is something that LEFT, not something that appeared


@dataclass
class Blob:
    """What the overlay draws: a candidate, whether or not it has been reported yet."""
    bbox: tuple[int, int, int, int]   # full-res x, y, w, h
    track_id: int
    age_s: float
    reported: bool
    kind: str


@dataclass
class Ignored:
    """Why changes were NOT turned into candidates (shown in status so spam can be explained)."""
    robot: int = 0
    too_big: int = 0
    too_small: int = 0
    not_compact: int = 0
    outside_zone: int = 0
    left: int = 0          # something was removed (an item picked off the rack), nothing new appeared


class SceneChangeDetector:
    def __init__(self, zones: ZoneSet, diff_threshold: int = 28,
                 item_min_frac: float = 0.0002, item_max_frac: float = 0.015,
                 spill_max_frac: float = 0.08, spill_min_frac: float = 0.0015,
                 max_aspect: float = 4.0, min_solidity: float = 0.6,
                 persist_s: float = 3.0, spill_persist_s: float = 5.0,
                 static_frac: float = 0.02, lost_s: float = 1.5,
                 max_area_frac: float = 0.45, max_shift_frac: float = 0.008, min_response: float = 0.1,
                 work_w: int = 640):
        self.zones = zones
        self.diff_threshold = diff_threshold
        self.item_min_frac, self.item_max_frac = item_min_frac, item_max_frac
        self.spill_max_frac, self.spill_min_frac = spill_max_frac, spill_min_frac
        self.max_aspect, self.min_solidity = max_aspect, min_solidity
        self.persist_s, self.spill_persist_s = persist_s, spill_persist_s
        self.static_frac, self.lost_s = static_frac, lost_s
        self.max_area_frac = max_area_frac
        self.max_shift_frac, self.min_response = max_shift_frac, min_response
        self.work_w = work_w
        self.alignment: dict[str, float] = {}
        self.ignored = Ignored()
        # Set while detection is paused because the camera view itself moved (not an incident:
        # nothing to do with the robot, just "I can't see properly right now").
        self.paused_reason: str | None = None

        self._ref: np.ndarray | None = None
        self._ref_gray: np.ndarray | None = None
        self._win: np.ndarray | None = None
        self._ref_size: tuple[int, int] | None = None
        self._tracks: dict[int, _Track] = {}
        self._next_id = 1
        self._baseline_region: np.ndarray | None = None
        self.blobs: list[Blob] = []

    # -- baseline ------------------------------------------------------------
    @property
    def has_reference(self) -> bool:
        return self._ref is not None

    def set_reference(self, bgr: np.ndarray, robot_region: np.ndarray | None = None) -> None:
        """The picture to compare against: the table as it looked when the robot started working.

        robot_region: where the robot stood at that moment. When it drives away, the patch of
        table it uncovers looks "new"; that spot is the robot's old footprint, not a drop."""
        self._baseline_region = robot_region
        self._ref = self._prep(bgr)
        self._ref_gray = self._gray32(self._ref)
        self._win = cv2.createHanningWindow((self._ref.shape[1], self._ref.shape[0]), cv2.CV_32F)
        self._ref_size = (bgr.shape[1], bgr.shape[0])
        self._tracks.clear()
        self.blobs = []
        self.alignment = {}
        self.ignored = Ignored()
        self.paused_reason = None

    def clear_reference(self) -> None:
        self._ref, self._ref_gray, self._ref_size = None, None, None
        self._tracks.clear()
        self.blobs = []
        self.alignment = {}
        self.paused_reason = None

    @staticmethod
    def _gray32(prepped: np.ndarray) -> np.ndarray:
        return cv2.cvtColor(np.clip(prepped, 0, 255).astype(np.uint8), cv2.COLOR_BGR2GRAY).astype(np.float32)

    def _prep(self, bgr: np.ndarray) -> np.ndarray:
        """Blurred, still in colour: packaging often matches the table in brightness but not hue."""
        h, w = bgr.shape[:2]
        if w > self.work_w:
            bgr = cv2.resize(bgr, (self.work_w, max(1, int(h * self.work_w / w))), interpolation=cv2.INTER_AREA)
        return cv2.GaussianBlur(bgr, (5, 5), 0).astype(np.int16)

    # -- per frame -----------------------------------------------------------
    def process(self, bgr: np.ndarray, now: float, robot_region: np.ndarray | None = None,
                spills: bool = False) -> tuple[list[Observation], list[int]]:
        """Returns (new observations, ids of reported tracks whose item is gone).

        robot_region: 255-mask of where the robot is (tag box grown). Any changed region touching
                      it is the robot and is skipped as a whole.
        spills:       look for spills too (only meaningful when the robot can be excluded exactly).
        """
        if self._ref is None or self._ref_size is None:
            return [], []
        if (bgr.shape[1], bgr.shape[0]) != self._ref_size:
            self.paused_reason = "The camera resolution changed. Set the position again."
            return [], []

        cur = self._prep(bgr)
        hh, ww = cur.shape[:2]

        if not self._aligned(cur):
            return [], []

        diff = cur - self._ref
        diff -= np.median(diff.reshape(-1, 3), axis=0).astype(np.int16)   # auto-exposure / white balance drift
        mag = np.abs(diff).max(axis=2)                                    # biggest change in any colour channel
        mask = (mag > self.diff_threshold).astype(np.uint8) * 255         # whole picture: blobs measured whole
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        watch = self.zones.mask(ww, hh)
        changed_frac = float(((mask > 0) & (watch > 0)).sum()) / max(1, int((watch > 0).sum()))
        if changed_frac > self.max_area_frac:
            self.blobs = []
            self._tracks.clear()
            self.paused_reason = "Most of the view changed at once (camera moved or lighting changed)."
            return [], []
        self.paused_reason = None

        region = None
        for r in (robot_region, self._baseline_region):          # where the robot is, and where it was
            if r is not None:
                r = cv2.resize(r, (ww, hh), interpolation=cv2.INTER_NEAREST)
                region = r if region is None else np.maximum(region, r)

        n, labels, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
        observations: list[Observation] = []
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
            sub = labels[y:y + h, x:x + w] == i
            if region is not None and (sub & (region[y:y + h, x:x + w] > 0)).any():
                ig.robot += 1                    # touches the robot: this is the robot (or what it holds)
                continue
            if (sub & (watch[y:y + h, x:x + w] > 0)).sum() < 0.5 * area:
                ig.outside_zone += 1             # mostly in a rack / tray / off the table
                continue
            aspect = max(w, h) / max(1, min(w, h))

            if frac <= self.item_max_frac and aspect <= self.max_aspect:
                kind, need_s = DROPPED_ITEM, self.persist_s
            elif spills and self.spill_min_frac <= frac <= self.spill_max_frac:
                kind, need_s = SPILL, self.spill_persist_s
            else:
                if frac > self.item_max_frac:
                    ig.too_big += 1              # the robot (no tag), a person, a big lighting change
                else:
                    ig.not_compact += 1          # a thin streak: cable, shadow edge, arm segment
                continue

            c = (float(cents[i][0]), float(cents[i][1]))
            track = self._match(c, seen, static_px * 2)
            if track is None or track.kind != kind:
                track = _Track(self._next_id, c, c, now, now, kind)
                self._next_id += 1
                self._tracks[track.id] = track
            seen.add(track.id)
            track.last_seen, track.centroid = now, c
            if math.dist(c, track.anchor) > static_px:       # it moved: restart the stillness clock
                track.anchor, track.first_seen = c, now
            full = (int(x * sx), int(y * sy), max(1, int(w * sx)), max(1, int(h * sy)))
            age = now - track.first_seen
            blobs.append(Blob(full, track.id, age, track.reported, kind))

            if age >= need_s and not track.reported and track.rejected:
                ig.left += 1
                continue
            if age >= need_s and not track.reported:
                if not self._appeared(cur, labels == i, (x, y, w, h)):
                    track.rejected = True        # the baseline had the thing, now it is gone: a pick, not a drop
                    ig.left += 1
                    continue
                feats = self._features(mask, cur, mag, (x, y, w, h), area, ww, hh)
                if kind == DROPPED_ITEM and feats["solidity"] < self.min_solidity:
                    ig.not_compact += 1
                    continue
                track.reported = True
                zone = self.zones.zone_at(c[0] / ww, c[1] / hh)
                feats.update(stationary_s=round(age, 1))
                what = "Small object" if kind == DROPPED_ITEM else "Large irregular patch"
                observations.append(Observation(
                    kind=kind, zone=zone, ts=now,
                    summary=(f"{what} ({feats['area_pct']}% of the view) appeared and has been still for "
                             f"{age:.0f}s in '{zone}' while the robot was working"),
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

    def check_view(self, bgr: np.ndarray) -> bool:
        """Has the camera view moved since the baseline? Cheap; works even while the robot is idle,
        so "someone moved the camera, set the position again" is noticed straight away."""
        if self._ref is None or self._ref_size is None:
            return True
        if (bgr.shape[1], bgr.shape[0]) != self._ref_size:
            self.paused_reason = "The camera resolution changed. Set the position again."
            return False
        ok = self._aligned(self._prep(bgr))
        if ok:
            self.paused_reason = None
        return ok

    def _aligned(self, cur: np.ndarray) -> bool:
        """Same view as the baseline? Every edge looks "new" if the camera moved, so detection pauses
        quietly. .copy(): phaseCorrelate multiplies its inputs by the window IN PLACE for FFT-friendly
        sizes, which would erode the stored baseline frame by frame."""
        ww = cur.shape[1]
        (dx, dy), response = cv2.phaseCorrelate(self._ref_gray.copy(), self._gray32(cur), self._win)
        shift = math.hypot(dx, dy)
        self.alignment = {"shift_px": round(shift, 1), "match": round(float(response), 2)}
        if shift > self.max_shift_frac * ww or response < self.min_response:
            self.blobs = []
            self._tracks.clear()
            self.paused_reason = "The camera view moved since monitoring started. Set the position again."
            return False
        return True

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

    def _appeared(self, cur: np.ndarray, blob: np.ndarray, box: tuple[int, int, int, int]) -> bool:
        """Did this blob APPEAR (it stands out from its surroundings now, but did not in the baseline)
        or did it LEAVE (the baseline had a thing there and the table now looks like its surroundings)?

        Picking an item off a rack reveals the empty slot: a real change, but not a drop. Comparing
        the blob with a ring of its own surroundings, in the baseline and in the current picture,
        tells the two apart without needing rack/tray zones to be drawn exactly."""
        x, y, w, h = box
        m = max(4, int(0.8 * max(w, h)))
        H, W = blob.shape
        x0, y0, x1, y1 = max(0, x - m), max(0, y - m), min(W, x + w + m), min(H, y + h + m)
        inner = blob[y0:y1, x0:x1]
        grown = cv2.dilate(inner.astype(np.uint8), np.ones((3, 3), np.uint8), iterations=max(2, m // 3)) > 0
        ring = grown & ~inner
        if inner.sum() < 3 or ring.sum() < 8:
            return True                      # cannot tell: do not hide a possible drop

        def standout(img: np.ndarray) -> float:
            patch = img[y0:y1, x0:x1].astype(np.float32)
            return float(np.linalg.norm(patch[inner].mean(axis=0) - patch[ring].mean(axis=0)))

        return standout(cur) >= standout(self._ref)

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
