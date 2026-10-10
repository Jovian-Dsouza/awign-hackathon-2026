"""Planar (SE2) poses and frame algebra shared by odometry, SLAM and control.

A BasePose `a_T_b` is the pose of frame b expressed in frame a.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


@dataclass(frozen=True)
class BasePose:
    x: float
    y: float
    yaw: float

    @property
    def xy(self) -> np.ndarray:
        return np.array([self.x, self.y])

    def to_world(self, local_xy: np.ndarray) -> np.ndarray:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.xy + np.array([c * local_xy[0] - s * local_xy[1], s * local_xy[0] + c * local_xy[1]])

    def to_local(self, world_xy: np.ndarray) -> np.ndarray:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        d = np.asarray(world_xy) - self.xy
        return np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1]])


IDENTITY = BasePose(0.0, 0.0, 0.0)


def compose(a_T_b: BasePose, b_T_c: BasePose) -> BasePose:
    """a_T_c = a_T_b * b_T_c."""
    x, y = a_T_b.to_world(np.array([b_T_c.x, b_T_c.y]))
    return BasePose(float(x), float(y), wrap_angle(a_T_b.yaw + b_T_c.yaw))


def inverse(a_T_b: BasePose) -> BasePose:
    """b_T_a."""
    x, y = BasePose(0.0, 0.0, -a_T_b.yaw).to_world(-a_T_b.xy)
    return BasePose(float(x), float(y), wrap_angle(-a_T_b.yaw))


def yaw_to_quat(yaw: float) -> dict[str, float]:
    """ROS geometry_msgs/Quaternion (x, y, z, w) for a rotation about +z."""
    return {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2), "w": math.cos(yaw / 2)}


def quat_to_yaw(q: dict[str, float]) -> float:
    """Yaw of a ROS quaternion (ignores roll/pitch)."""
    x, y, z, w = (float(q[k]) for k in ("x", "y", "z", "w"))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
