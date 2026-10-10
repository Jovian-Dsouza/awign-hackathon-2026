import numpy as np

from overwatch.detectors.health import HealthMonitor
from overwatch.detectors.scene_change import SceneChangeDetector
from overwatch.events import CAMERA_FAULT, DROPPED_ITEM
from overwatch.zones import Zone, ZoneSet

from conftest import textured_background, with_box


def run(det, frames_at):
    """frames_at: list of (t, image). Returns all observations and gone ids."""
    obs, gone = [], []
    for t, img in frames_at:
        o, g = det.process(img, t)
        obs += o
        gone += g
    return obs, gone


def test_still_object_is_reported_once_after_persist(bg):
    det = SceneChangeDetector(ZoneSet(), persist_s=3.0)
    det.set_reference(bg)
    scene = with_box(bg, 300, 200, 40, 40)
    obs, _ = run(det, [(i * 0.5, scene) for i in range(0, 14)])   # 6.5 s
    assert [o.kind for o in obs] == [DROPPED_ITEM]
    o = obs[0]
    assert 0.5 < o.features["area_pct"] < 1.5          # 40x40 of 640x480 is about 0.52 %
    assert o.features["solidity"] > 0.9
    assert o.bbox is not None and abs(o.bbox[0] - 300) < 12 and abs(o.bbox[1] - 200) < 12


def test_not_reported_before_persist(bg):
    det = SceneChangeDetector(ZoneSet(), persist_s=5.0)
    det.set_reference(bg)
    obs, _ = run(det, [(i * 0.5, with_box(bg, 300, 200, 40, 40)) for i in range(0, 8)])  # 3.5 s
    assert obs == []


def test_moving_object_never_reported(bg):
    det = SceneChangeDetector(ZoneSet(), persist_s=2.0)
    det.set_reference(bg)
    frames = [(i * 0.25, with_box(bg, 100 + i * 12, 200, 40, 40)) for i in range(0, 40)]   # 10 s, moving
    obs, _ = run(det, frames)
    assert obs == []


def test_object_removed_is_reported_gone(bg):
    det = SceneChangeDetector(ZoneSet(), persist_s=1.0, lost_s=1.0)
    det.set_reference(bg)
    scene = with_box(bg, 300, 200, 40, 40)
    frames = [(i * 0.5, scene) for i in range(0, 6)] + [(3 + i * 0.5, bg) for i in range(0, 6)]
    obs, gone = run(det, frames)
    assert len(obs) == 1
    assert gone == [obs[0].track_id]


def test_global_brightness_shift_is_not_an_object(bg):
    det = SceneChangeDetector(ZoneSet(), persist_s=1.0)
    det.set_reference(bg)
    brighter = np.clip(bg.astype(np.int16) + 35, 0, 255).astype(np.uint8)
    obs, _ = run(det, [(i * 0.5, brighter) for i in range(0, 10)])
    assert obs == []


def test_camera_bumped_reports_scene_shift_not_objects(bg):
    det = SceneChangeDetector(ZoneSet(), persist_s=1.0)
    det.set_reference(bg)
    rng = np.random.default_rng(99)                # an entirely different, high-contrast view
    other = np.kron(rng.integers(0, 256, (60, 80, 3), dtype=np.uint8), np.ones((8, 8, 1), np.uint8))
    obs, _ = run(det, [(i * 0.5, other) for i in range(0, 8)])
    assert [o.kind for o in obs] == [CAMERA_FAULT]
    assert obs[0].features["fault"] == "scene_shift"


def test_object_in_ignored_zone_is_ignored(bg):
    zones = ZoneSet([Zone("dock", [(0.4, 0.3), (0.7, 0.3), (0.7, 0.7), (0.4, 0.7)], "ignore")])
    det = SceneChangeDetector(zones, persist_s=1.0)
    det.set_reference(bg)
    scene = with_box(bg, 300, 200, 40, 40)         # inside the ignored rectangle
    obs, _ = run(det, [(i * 0.5, scene) for i in range(0, 8)])
    assert obs == []


def test_zone_name_is_attached(bg):
    zones = ZoneSet([Zone("floor", [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)], "watch")])
    det = SceneChangeDetector(zones, persist_s=1.0)
    det.set_reference(bg)
    obs, _ = run(det, [(i * 0.5, with_box(bg, 300, 200, 40, 40)) for i in range(0, 6)])
    assert obs and obs[0].zone == "floor"


def test_no_reference_means_no_detection(bg):
    det = SceneChangeDetector(ZoneSet(), persist_s=0.5)
    obs, gone = run(det, [(i * 0.5, with_box(bg, 300, 200, 40, 40)) for i in range(0, 6)])
    assert obs == [] and gone == []


# ---- health -----------------------------------------------------------------

def test_black_frame_fault():
    h = HealthMonitor(hold_s=1.0)
    black = np.zeros((480, 640, 3), np.uint8)
    obs = []
    for i in range(6):
        obs += h.process(black, i * 0.5)
    assert [o.features["fault"] for o in obs if o.features["fault"] == "black"] == ["black"]


def test_flat_grey_wall_is_no_scene_detail():
    h = HealthMonitor(hold_s=1.0)
    grad = np.tile(np.linspace(120, 135, 640, dtype=np.uint8), (480, 1))
    wall = np.dstack([grad, grad, grad])
    faults = set()
    for i in range(8):
        faults |= {o.features["fault"] for o in h.process(wall, i * 0.5)}
    assert "no_scene_detail" in faults


def test_normal_scene_has_no_fault(bg):
    h = HealthMonitor(hold_s=1.0)
    obs = []
    for i in range(8):
        frame = np.clip(bg.astype(np.int16) + np.random.default_rng(i).normal(0, 2, bg.shape), 0, 255).astype(np.uint8)
        obs += h.process(frame, i * 0.5)
    assert obs == [] and h.active == set()


def test_frozen_feed_fault(bg):
    h = HealthMonitor(hold_s=1.0)
    faults = set()
    for i in range(8):
        faults |= {o.features["fault"] for o in h.process(bg, i * 0.5)}
    assert "frozen" in faults


# ---- the failure seen on real hardware: a camera that is handled or drifts ----

def test_slightly_moved_camera_pauses_detection_instead_of_flooding_objects(bg):
    """Camera nudged ~1.5 % of the frame width, then held still for 10 s. Every texture edge
    differs from the reference; none of it may be reported as an object."""
    det = SceneChangeDetector(ZoneSet(), persist_s=1.0)
    det.set_reference(bg)
    nudged = np.roll(bg, 10, axis=1)
    obs, _ = run(det, [(i * 0.5, nudged) for i in range(0, 20)])
    assert [o.features.get("fault") for o in obs] == ["scene_shift"]
    assert all(o.kind == CAMERA_FAULT for o in obs)


def test_camera_returning_to_place_resumes_detection(bg):
    det = SceneChangeDetector(ZoneSet(), persist_s=1.0)
    det.set_reference(bg)
    nudged = np.roll(bg, 10, axis=1)
    frames = [(i * 0.5, nudged) for i in range(0, 4)]
    frames += [(2 + i * 0.5, with_box(bg, 300, 200, 40, 40)) for i in range(0, 8)]
    obs, _ = run(det, frames)
    kinds = [o.kind for o in obs]
    assert kinds == [CAMERA_FAULT, DROPPED_ITEM]


def test_object_in_view_does_not_look_like_camera_motion(bg):
    det = SceneChangeDetector(ZoneSet(), persist_s=60.0)
    det.set_reference(bg)
    det.process(with_box(bg, 300, 200, 80, 80), 0.0)
    assert det.alignment["shift_px"] < 1.0 and det.alignment["match"] > 0.2   # threshold is 0.1


def test_still_camera_stays_aligned_over_a_long_run(bg):
    """Regression: cv2.phaseCorrelate used to erode the stored reference in place, so a perfectly
    still, noisy camera slowly looked 'moved'. 400 frames of sensor noise must stay aligned."""
    def noisy(img, seed):
        n = np.random.default_rng(seed).normal(0, 2.0, img.shape)
        return np.clip(img.astype(np.float32) + n, 0, 255).astype(np.uint8)
    det = SceneChangeDetector(ZoneSet(), persist_s=1e9)
    det.set_reference(noisy(bg, 0))
    faults = []
    for i in range(1, 401):
        o, _ = det.process(noisy(with_box(bg, 300, 200, 40, 40), i), i * 0.1)
        faults += o
    assert faults == []
    assert det.alignment["shift_px"] < 0.5 and det.alignment["match"] > 0.2


# ---- real frames from the team's sim (robot-sim/renders): what a pick and a drop look like ----

import pathlib

import cv2

SIM = pathlib.Path(__file__).parent / "fixtures" / "sim"


def _sim(name):
    img = cv2.imread(str(SIM / name))
    assert img is not None, name
    return img


def _sim_zones():
    """Watch the table; the rack and the tray are where items legitimately change."""
    return ZoneSet([
        Zone("table", [(0.0, 0.41), (0.86, 0.41), (1.0, 0.41), (1.0, 0.72), (0.0, 0.72)], "watch"),
        Zone("rack", [(0.19, 0.25), (0.43, 0.25), (0.43, 0.49), (0.19, 0.49)], "ignore"),
        Zone("tray", [(0.72, 0.45), (0.91, 0.45), (0.91, 0.60), (0.72, 0.60)], "ignore"),
    ])


def _hold(det, frame, seconds=5.0, step=0.5, t0=0.0):
    obs, t = [], t0
    for _ in range(int(seconds / step)):
        o, _g = det.process(frame, t)
        obs += o
        t += step
    return obs


def test_a_successful_pick_raises_nothing():
    """Robot drives to the rack, grasps, carries to the tray, finishes: every frame held 5 s."""
    det = SceneChangeDetector(_sim_zones(), persist_s=3.0)
    det.set_reference(_sim("00_start_overview.png"))
    for name in ["02_reached_overview.png", "03_grasped_overview.png",
                 "04_at_tray_overview.png", "05_done_overview.png"]:
        assert _hold(det, _sim(name)) == [], name


def test_a_can_left_on_the_table_is_one_incident():
    det = SceneChangeDetector(_sim_zones(), persist_s=3.0)
    det.set_reference(_sim("00_start_overview.png"))
    obs = _hold(det, _sim("restock_00_item_on_table.png"))
    assert [o.kind for o in obs] == [DROPPED_ITEM]
    o = obs[0]
    assert o.zone == "table" and 0.03 < o.features["area_pct"] < 0.2     # the can is ~0.08 % of the view
    x, y, w, h = o.bbox
    assert abs((x + w / 2) - 395) < 15 and abs((y + h / 2) - 256) < 15   # where the can really is


def test_the_robot_is_never_the_alert():
    """Robot parked mid-table for 30 s (it pauses while picking): too big to be an item."""
    det = SceneChangeDetector(_sim_zones(), persist_s=3.0)
    det.set_reference(_sim("00_start_overview.png"))
    assert _hold(det, _sim("03_grasped_overview.png"), seconds=30.0) == []
    assert det.ignored.too_big >= 1


def test_a_person_sized_blob_is_not_an_item(bg):
    det = SceneChangeDetector(ZoneSet(), persist_s=1.0)
    det.set_reference(bg)
    person = with_box(bg, 100, 100, 200, 300)                 # ~20 % of the picture
    assert [o.kind for o in run(det, [(i * 0.5, person) for i in range(10)])[0]] == []
    assert det.ignored.too_big == 1


def test_a_thin_streak_is_not_an_item(bg):
    det = SceneChangeDetector(ZoneSet(), persist_s=1.0)
    det.set_reference(bg)
    streak = with_box(bg, 100, 200, 160, 8)                   # a cable / shadow edge
    assert run(det, [(i * 0.5, streak) for i in range(10)])[0] == []
