"""Where the base must stand to reach a shelf slot or a tote (pure geometry).

Stretch's arm extends toward base -y, so to reach into a surface whose outward
normal is n, the base faces along n rotated by +90 deg (base +y = n) and
stands `standoff` metres from the target along n.
"""

from __future__ import annotations

import math

import numpy as np

from qc_warehouse.autonomy.stretch import SIDE_GRASP_OFFSET, BasePose

# Base-centre to target distance along the arm axis. 0.60 m works for both
# grasp modes: side ext = 0.60 - 0.416, top-down ext = 0.60 - 0.121 (max 0.5).
SHELF_STANDOFF = 0.60
STATION_STANDOFF = 0.60
# Side grasps hold the item ~0.42 m out with the arm retracted; standing a bit
# farther back keeps it clear of the shelf face so the wrist can fold it in.
SIDE_GRASP_EXTRA_STANDOFF = 0.08
LATERAL_OFFSET = SIDE_GRASP_OFFSET[0]  # grasp centre sits slightly behind the base origin


def outward_normal(yaw_deg: float, local_normal: tuple[float, float]) -> np.ndarray:
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return np.array([c * local_normal[0] - s * local_normal[1], s * local_normal[0] + c * local_normal[1]])


def reach_pose(target_xy: np.ndarray, normal: np.ndarray, standoff: float) -> BasePose:
    """Base pose whose arm axis passes through `target_xy`, standing `standoff` away along `normal`."""
    n = np.asarray(normal, dtype=float) / np.linalg.norm(normal)
    yaw = math.atan2(n[1], n[0]) - math.pi / 2
    forward = np.array([math.cos(yaw), math.sin(yaw)])
    base = np.asarray(target_xy, dtype=float) + n * standoff - forward * LATERAL_OFFSET
    return BasePose(float(base[0]), float(base[1]), (yaw + math.pi) % (2 * math.pi) - math.pi)


def stage_pose(pose: BasePose, distance: float) -> BasePose:
    """Pose `distance` behind `pose` (same heading) so the final approach drives forward."""
    return BasePose(pose.x - distance * math.cos(pose.yaw), pose.y - distance * math.sin(pose.yaw), pose.yaw)
