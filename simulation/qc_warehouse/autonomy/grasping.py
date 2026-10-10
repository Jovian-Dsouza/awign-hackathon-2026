"""Grasp selection from a perceived item (pure geometry, no simulation).

Tall items are grasped from the side (wrist level, fingers closing horizontally
across the item). Flat packs are grasped top-down (wrist pitched -90 deg), with
the wrist rolled so the fingers close across the item's narrower side.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from qc_warehouse.autonomy.perception import Detection
from qc_warehouse.autonomy.stretch import LIFT_RANGE, PITCH_DOWN, PITCH_SIDE, SIDE_GRASP_OFFSET, TOP_GRASP_OFFSET

FLAT_ITEM_MAX_HEIGHT = 0.06   # below this, grasp top-down
GRIPPER_MAX_WIDTH = 0.12      # side grasp: pads open 0.132 m, minus margin
GRIPPER_MAX_WIDTH_TOP = 0.108 # wrist pitched down: pads open only 0.118 m
SIDE_MIN_ABOVE_BOARD = 0.05   # keep the gripper body clear of the board
SIDE_MAX_BELOW_TOP = 0.03
SIDE_MAX_ABOVE_BOTTOM = 0.08  # grasp tall items low: less lift needed, steadier carry
TOP_ARM_ABOVE_GRASP = 0.36    # arm tube top above the grasp centre when the wrist points down
TOP_GRASP_ABOVE_BOARD = 0.024
TOP_PREGRASP_CLEARANCE = 0.04  # hover height above the grasp height (wrist headroom is tight)
SIDE_PREGRASP_BACKOFF = 0.12   # stop short along the arm before the final approach
VISIBLE_DEPTH_CAP = 0.10       # max depth assumed when only the front face is seen


class GraspError(RuntimeError):
    """The item cannot be grasped with this gripper / reach."""


@dataclass(frozen=True)
class GraspPlan:
    item: str
    strategy: str              # "side" | "top"
    target: np.ndarray         # world grasp-centre position
    pitch: float
    roll: float
    hold_offset: float         # grasp z minus item bottom z (used when placing)
    width: float               # item width between the fingers
    top_offset: float = 0.0    # item top z minus grasp z (clearance needed after release)

    @property
    def is_top(self) -> bool:
        return self.strategy == "top"


def extents_along(det: Detection, arm_dir_xy: np.ndarray) -> tuple[float, float]:
    """(width across the arm, depth along the arm) of the detection's bounding box."""
    size = det.size
    a = np.abs(np.asarray(arm_dir_xy, dtype=float))
    depth = a[0] * size[0] + a[1] * size[1]
    width = a[1] * size[0] + a[0] * size[1]
    return float(width), float(depth)


def plan_grasp(det: Detection, arm_dir_xy: np.ndarray, board_z: float, ceiling_z: float = math.inf) -> GraspPlan:
    """Choose side vs top-down grasp and compute the grasp-centre target.

    ceiling_z is the underside of whatever is above the item (next shelf board);
    top-down grasps need the arm, ~0.35 m above the grasp centre, to fit under it.
    """
    width, depth = extents_along(det, arm_dir_xy)
    bottom = max(board_z, float(det.min_xyz[2]))
    top = float(det.max_xyz[2])
    height = top - bottom
    center_xy = _center_along_arm(det, np.asarray(arm_dir_xy, dtype=float), width, depth)

    z_min, z_max = board_z + SIDE_MIN_ABOVE_BOARD, top - SIDE_MAX_BELOW_TOP
    if height >= FLAT_ITEM_MAX_HEIGHT and z_min <= z_max:  # tall enough to pinch clear of the board
        if width > GRIPPER_MAX_WIDTH:
            raise GraspError(f"{det.name}: {width:.3f} m wide, gripper opens {GRIPPER_MAX_WIDTH:.3f} m")
        z = float(np.clip(bottom + min(height / 2, SIDE_MAX_ABOVE_BOTTOM), z_min, z_max))
        _check_lift(det.name, z - SIDE_GRASP_OFFSET[2])
        return GraspPlan(det.name, "side", np.array([*center_xy, z]), PITCH_SIDE, 0.0, z - bottom, width, top - z)

    narrow = min(width, depth)
    if narrow > GRIPPER_MAX_WIDTH_TOP:
        raise GraspError(f"{det.name}: flat pack {width:.3f} x {depth:.3f} m, too wide for the gripper")
    roll = 0.0 if width <= depth else math.pi / 2  # roll 0: fingers close across the arm
    z = board_z + TOP_GRASP_ABOVE_BOARD
    hover = z + TOP_PREGRASP_CLEARANCE
    _check_lift(det.name, hover - TOP_GRASP_OFFSET[2])
    if hover + TOP_ARM_ABOVE_GRASP > ceiling_z:
        raise GraspError(f"{det.name}: no headroom for a top-down grasp ({ceiling_z - hover:.2f} m above, need {TOP_ARM_ABOVE_GRASP} m)")
    return GraspPlan(det.name, "top", np.array([*center_xy, z]), PITCH_DOWN, roll, z - bottom, narrow, max(0.0, top - z))


def _center_along_arm(det: Detection, arm_dir: np.ndarray, width: float, depth: float) -> np.ndarray:
    """Bounding-box centre, corrected when only the front face was visible.

    A camera in front of the shelf mostly sees front faces, so the visible depth
    under-estimates the item's depth. Assume a cross-section at least as deep as
    it is wide (cans, bottles) and push the centre back from the front face.
    """
    front = min(det.min_xyz[:2] @ arm_dir, det.max_xyz[:2] @ arm_dir)
    est_depth = max(depth, min(width, VISIBLE_DEPTH_CAP))
    along = front + est_depth / 2
    center = det.center[:2]
    return center + arm_dir * (along - center @ arm_dir)


def lift_for(plan_is_top: bool, grasp_z: float) -> float:
    """Lift joint value that puts the grasp centre at `grasp_z`."""
    offset = TOP_GRASP_OFFSET[2] if plan_is_top else SIDE_GRASP_OFFSET[2]
    return grasp_z - offset


def _check_lift(name: str, lift: float) -> None:
    if not LIFT_RANGE[0] <= lift <= LIFT_RANGE[1]:
        raise GraspError(f"{name}: needs lift {lift:.2f} m, outside Stretch range {LIFT_RANGE}")
