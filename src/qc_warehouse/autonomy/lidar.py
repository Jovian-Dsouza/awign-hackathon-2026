"""Simulated 2-D lidar (Stretch 3's RPLidar) using MuJoCo ray casting.

Beam i points at angle `-pi + i * 2pi/beams` in the laser body frame (x forward,
y left), matching the ROS LaserScan the SLAM node receives. Beams that hit the
robot itself, miss everything, or are dropped are reported as `no_return`
(beyond range_max), which slam_toolbox ignores.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import mujoco
import numpy as np


@dataclass(frozen=True)
class LidarSpec:
    beams: int = 360
    range_min: float = 0.15
    range_max: float = 12.0
    noise_sigma: float = 0.01   # m, Gaussian range noise
    dropout: float = 0.005      # probability a beam returns nothing
    scan_period: float = 0.1    # s (10 Hz)

    @property
    def angle_increment(self) -> float:
        return 2 * math.pi / self.beams

    @property
    def angles(self) -> np.ndarray:
        return -math.pi + np.arange(self.beams) * self.angle_increment

    @property
    def no_return(self) -> float:
        return self.range_max + 1.0


class Lidar2D:
    def __init__(
        self,
        model: mujoco.MjModel,
        laser_body: str,
        robot_root_body: str,
        spec: LidarSpec = LidarSpec(),
        rng: np.random.Generator | None = None,
    ):
        self._model, self.spec = model, spec
        self._body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, laser_body)
        root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, robot_root_body)
        if self._body < 0 or root < 0:
            missing = laser_body if self._body < 0 else robot_root_body
            raise ValueError(f"lidar body {missing!r} not in model")
        self._self_geoms = model.body_rootid[model.geom_bodyid] == model.body_rootid[root]
        self._rng = rng or np.random.default_rng(0)
        angles = spec.angles
        self._local_dirs = np.stack([np.cos(angles), np.sin(angles), np.zeros_like(angles)], axis=1)
        self._geomid = np.zeros(spec.beams, dtype=np.int32)
        self._dist = np.zeros(spec.beams, dtype=np.float64)

    def scan(self, data: mujoco.MjData) -> np.ndarray:
        """One full revolution (taken instantaneously) as a float array of ranges."""
        spec = self.spec
        dirs = self._local_dirs @ data.xmat[self._body].reshape(3, 3).T
        mujoco.mj_multiRay(
            self._model, data, data.xpos[self._body], dirs.ravel(), None, 1, self._body,
            self._geomid, self._dist, None, spec.beams, spec.range_max,
        )
        ranges = self._dist.copy()
        if spec.noise_sigma:
            ranges += self._rng.normal(0.0, spec.noise_sigma, spec.beams)
        invalid = (self._geomid < 0) | (ranges <= spec.range_min) | (ranges >= spec.range_max)
        invalid |= np.where(self._geomid >= 0, self._self_geoms[np.clip(self._geomid, 0, None)], False)
        if spec.dropout:
            invalid |= self._rng.random(spec.beams) < spec.dropout
        return np.where(invalid, spec.no_return, ranges)
