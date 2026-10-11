"""The one flow: keep an eye on the robot; it drops an item or spills something; say what to do."""

import time

import cv2
import numpy as np
import pytest

from overwatch.decision.rules import RulesDecider
from overwatch.detectors.presence import MotionPresence
from overwatch.detectors.robot_marker import RobotMarker, write_tag
from overwatch.detectors.scene_change import SceneChangeDetector
from overwatch.events import DROPPED_ITEM, SPILL
from overwatch.zones import ZoneSet

from conftest import textured_background, with_box


def put_tag(img, cx, cy, size=48, marker_id=0):
    """Paste a printed robot tag (with its white border) centred at (cx, cy)."""
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    tag = cv2.aruco.generateImageMarker(d, marker_id, size)
    tag = cv2.copyMakeBorder(tag, 8, 8, 8, 8, cv2.BORDER_CONSTANT, value=255)
    tag = cv2.cvtColor(tag, cv2.COLOR_GRAY2BGR)
    out = img.copy()
    h, w = tag.shape[:2]
    out[cy - h // 2:cy - h // 2 + h, cx - w // 2:cx - w // 2 + w] = tag
    return out


def test_the_tag_is_found_and_the_robot_region_covers_its_surroundings():
    m = RobotMarker(0)
    frame = put_tag(textured_background(), 200, 300)
    region = m.observe(frame)
    assert m.visible and region is not None and abs(m.center[0] - 200) < 3 and abs(m.center[1] - 300) < 3
    assert region[300, 200] == 255 and region[300, 330] == 0           # robot area is grown, not the whole frame


def test_no_tag_means_robot_not_in_view():
    m = RobotMarker(0)
    assert m.observe(textured_background()) is None and m.visible is False


def test_wrong_tag_id_is_not_the_robot():
    m = RobotMarker(0)
    m.observe(put_tag(textured_background(), 200, 300, marker_id=7))
    assert m.visible is False


def test_printable_tag_is_written(tmp_path):
    path = write_tag(str(tmp_path / "t.png"))
    img = cv2.imread(path)
    assert img is not None and min(img.shape[:2]) > 500


def _watch(det, frames, region_fn=None, spills=False, step=0.5):
    obs, t = [], 0.0
    for f in frames:
        o, _ = det.process(f, t, region_fn(f) if region_fn else None, spills=spills)
        obs += o
        t += step
    return obs


def test_a_spill_next_to_the_robot_is_a_spill_not_an_item():
    bg = textured_background()
    base = put_tag(bg, 120, 380)
    det = SceneChangeDetector(ZoneSet(), persist_s=1.0, spill_persist_s=2.0)
    m = RobotMarker(0)
    det.set_reference(base, m.observe(base))
    # a large soft patch on the table, away from the robot
    scene = base.copy()
    cv2.ellipse(scene, (430, 250), (70, 38), 20, 0, 360, (150, 120, 60), -1)
    obs = _watch(det, [scene] * 12, lambda f: m.observe(f), spills=True)
    assert [o.kind for o in obs] == [SPILL]
    d = RulesDecider().decide(obs[0])
    assert d.action == "escalate" and "person" in d.reasons[0]


def test_without_the_tag_a_big_patch_is_never_called_a_spill():
    """The robot itself is a big still patch when parked. Without the tag we cannot tell, so no spills."""
    bg = textured_background()
    det = SceneChangeDetector(ZoneSet(), persist_s=1.0, spill_persist_s=2.0)
    det.set_reference(bg)
    scene = bg.copy()
    cv2.ellipse(scene, (430, 250), (70, 38), 20, 0, 360, (150, 120, 60), -1)
    assert _watch(det, [scene] * 12, spills=False) == []


def test_the_robot_body_touching_the_tag_is_never_reported_even_when_parked():
    bg = textured_background()
    base = put_tag(bg, 120, 380)
    det = SceneChangeDetector(ZoneSet(), persist_s=1.0, spill_persist_s=2.0)
    m = RobotMarker(0)
    det.set_reference(base, m.observe(base))        # the robot starts at the dock...
    body = with_box(bg, 290, 215, 80, 80, color=(200, 80, 20))     # the robot's body parked mid-table...
    body = put_tag(body, 330, 255)                                  # ...with its tag on top, as printed
    assert _watch(det, [body] * 14, lambda f: m.observe(f), spills=True) == []
    assert det.ignored.robot >= 1


def test_an_item_that_lands_away_from_the_robot_is_a_drop():
    bg = textured_background()
    base = put_tag(bg, 120, 380)
    det = SceneChangeDetector(ZoneSet(), persist_s=1.0)
    m = RobotMarker(0)
    det.set_reference(base, m.observe(base))
    scene = with_box(base, 420, 200, 22, 30)         # a packet-sized thing
    obs = _watch(det, [scene] * 8, lambda f: m.observe(f), spills=True)
    assert [o.kind for o in obs] == [DROPPED_ITEM]
    assert RulesDecider().decide(obs[0]).action == "propose_recovery"


# ---- no tag: "is the robot working?" from motion alone ------------------------------

def _moving_frames(n=12, size=60):
    bg = textured_background()
    return [with_box(bg, 60 + i * 30, 200, size, size) for i in range(n)]


def test_robot_sized_motion_counts_as_the_robot_working():
    p = MotionPresence()
    results = [p.process(f) for f in _moving_frames()]
    assert results[-1] is True


def test_a_still_scene_is_not_the_robot():
    bg = textured_background()
    p = MotionPresence()
    assert not any(p.process(bg) for _ in range(12))


def test_the_whole_picture_shifting_is_a_camera_being_adjusted_not_the_robot():
    bg = textured_background()
    p = MotionPresence()
    results = [p.process(np.roll(bg, 25 * (i % 4), axis=1)) for i in range(12)]
    assert not any(results)


def test_small_moving_things_are_not_the_robot():
    bg = textured_background()
    p = MotionPresence()
    frames = [with_box(bg, 60 + i * 30, 200, 10, 10) for i in range(12)]    # an item-sized mover
    assert not any(p.process(f) for f in frames)
