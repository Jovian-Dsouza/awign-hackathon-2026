import math

import numpy as np
import pytest

from qc_warehouse.autonomy.grasping import GraspError, extents_along, lift_for, plan_grasp
from qc_warehouse.autonomy.perception import Detection
from qc_warehouse.autonomy.poses import SHELF_STANDOFF, reach_pose, stage_pose
from qc_warehouse.autonomy.stretch import BasePose

ARM_NORTH = np.array([0.0, 1.0])  # arm reaching into a south-facing shelf


def _det(name, lo, hi):
    lo, hi = np.array(lo, float), np.array(hi, float)
    return Detection(name, 500, (lo + hi) / 2, lo, hi)


def test_extents_along_arm():
    det = _det("box", [0, 0, 0], [0.1, 0.3, 0.2])
    assert extents_along(det, ARM_NORTH) == pytest.approx((0.1, 0.3))
    assert extents_along(det, np.array([1.0, 0.0])) == pytest.approx((0.3, 0.1))


def test_tall_item_gets_side_grasp_at_mid_height():
    plan = plan_grasp(_det("ketchup", [0.96, 3.58, 0.55], [1.04, 3.64, 0.77]), ARM_NORTH, board_z=0.55)
    assert plan.strategy == "side"
    # 6 cm visible depth < 8 cm width -> assumed 8 cm deep, centre 4 cm behind the front face.
    # 22 cm tall -> grasped 8 cm above its base (capped below mid-height).
    assert plan.target == pytest.approx([1.0, 3.62, 0.63])
    assert plan.hold_offset == pytest.approx(0.08)


def test_squat_item_falls_back_to_top_grasp():
    # 7 cm tall: above the flat threshold but too short to side-grasp clear of the board.
    plan = plan_grasp(_det("jar", [0, 3.5, 0.55], [0.07, 3.57, 0.62]), ARM_NORTH, board_z=0.55)
    assert plan.strategy == "top"


def test_top_grasp_needs_headroom():
    det = _det("butter", [0, 3.5, 0.55], [0.11, 3.56, 0.58])
    plan_grasp(det, ARM_NORTH, board_z=0.55, ceiling_z=1.025)
    with pytest.raises(GraspError, match="headroom"):
        plan_grasp(det, ARM_NORTH, board_z=0.55, ceiling_z=0.925)


def test_front_face_only_detection_pushes_center_back():
    # Only the front face (2 cm deep) of an 8 cm wide bottle was seen.
    plan = plan_grasp(_det("bottle", [0.96, 3.58, 0.55], [1.04, 3.60, 0.77]), ARM_NORTH, board_z=0.55)
    assert plan.target[1] == pytest.approx(3.58 + 0.04)
    assert plan.target[0] == pytest.approx(1.0)


def test_short_can_side_grasp_kept_clear_of_board():
    plan = plan_grasp(_det("soup", [0, 3.58, 0.55], [0.066, 3.65, 0.65]), ARM_NORTH, board_z=0.55)
    assert plan.strategy == "side" and plan.target[2] == pytest.approx(0.60)


def test_flat_item_gets_top_grasp_with_roll_across_narrow_side():
    deep = plan_grasp(_det("butter", [0, 3.5, 0.55], [0.11, 3.56, 0.58]), ARM_NORTH, board_z=0.55)
    assert deep.strategy == "top" and deep.roll == pytest.approx(math.pi / 2) and deep.width == pytest.approx(0.06)
    wide = plan_grasp(_det("cookies", [0, 3.5, 0.55], [0.09, 3.62, 0.58]), ARM_NORTH, board_z=0.55)
    assert wide.roll == 0.0 and wide.width == pytest.approx(0.09)


@pytest.mark.parametrize(
    ("lo", "hi", "board", "message"),
    [
        ([0, 3.5, 0.55], [0.16, 3.57, 0.76], 0.55, "wide"),        # tall but too wide
        ([0, 3.5, 0.55], [0.15, 3.65, 0.58], 0.55, "too wide"),    # flat and both sides too wide
        ([0, 3.5, 1.05], [0.08, 3.56, 1.08], 1.05, "lift"),        # flat on the upper level: out of lift range
    ],
)
def test_ungraspable(lo, hi, board, message):
    with pytest.raises(GraspError, match=message):
        plan_grasp(_det("x", lo, hi), ARM_NORTH, board_z=board)


def test_lift_for_modes():
    assert lift_for(False, 0.7) == pytest.approx(0.588)
    assert lift_for(True, 0.6) == pytest.approx(0.731)


def test_reach_pose_for_south_facing_shelf():
    pose = reach_pose(np.array([1.0, 3.6]), np.array([0.0, -1.0]), SHELF_STANDOFF)
    assert abs(abs(pose.yaw) - math.pi) < 1e-9          # heading west
    assert pose.y == pytest.approx(3.6 - SHELF_STANDOFF)
    # The side grasp centre at ext = standoff - 0.416 lands on the target.
    local = np.array([-0.022, -(0.416 + SHELF_STANDOFF - 0.416)])
    assert pose.to_world(local) == pytest.approx([1.0, 3.6])


def test_stage_pose_is_behind():
    pose = BasePose(0.0, 0.0, 0.0)
    assert stage_pose(pose, 1.0).xy == pytest.approx([-1.0, 0.0])


def test_base_pose_roundtrip():
    pose = BasePose(1.0, 2.0, 0.7)
    assert pose.to_local(pose.to_world(np.array([0.3, -0.2]))) == pytest.approx([0.3, -0.2])
