"""Where the robot thinks it is.

SlamLocalizer:   world_T_base = world_T_map (dock, from layout) * map_T_odom (SLAM) * odom_T_base (odometry)
EstimatedKinematics: forward kinematics of the whole robot placed at the *estimated* base
pose, so camera and gripper positions live in the same (estimated) frame as the base.
LocalizationMonitor: compares estimate against simulator ground truth, for reporting only.
"""

from __future__ import annotations

import contextlib
import math
from collections.abc import Callable, Iterator
from dataclasses import dataclass
from typing import Protocol

import mujoco
import numpy as np

from qc_warehouse.autonomy.frames import IDENTITY, BasePose, compose, wrap_angle
from qc_warehouse.autonomy.odometry import WheelGyroOdometry


class LocalizationLostError(RuntimeError):
    """The robot can no longer localize (e.g. the SLAM service went away); stop moving."""


class Localizer(Protocol):
    def base_pose(self) -> BasePose: ...

    def freeze(self) -> contextlib.AbstractContextManager[None]:
        """Hold SLAM corrections back while the robot does precision work in a fixed frame."""


class SlamLocalizer:
    def __init__(self, odometry: WheelGyroOdometry, world_T_map: BasePose):
        self._odometry = odometry
        self.world_T_map = world_T_map
        self._map_T_odom = IDENTITY
        self._pending: BasePose | None = None
        self._frozen = 0
        self.corrections = 0

    @property
    def map_T_odom(self) -> BasePose:
        return self._map_T_odom

    def base_pose(self) -> BasePose:
        return compose(compose(self.world_T_map, self._map_T_odom), self._odometry.pose)

    def map_pose(self) -> BasePose:
        return compose(self._map_T_odom, self._odometry.pose)

    def apply_correction(self, map_T_odom: BasePose) -> None:
        self.corrections += 1
        if self._frozen:
            self._pending = map_T_odom
        else:
            self._map_T_odom = map_T_odom

    @contextlib.contextmanager
    def freeze(self) -> Iterator[None]:
        self._frozen += 1
        try:
            yield
        finally:
            self._frozen -= 1
            if not self._frozen and self._pending is not None:
                self._map_T_odom, self._pending = self._pending, None


class EstimatedKinematics:
    """A scratch MjData mirroring the robot's joint encoders, with the base at an estimated pose.

    Heading comes from the estimate; roll/pitch (the small tilt when the arm reaches out)
    are kept from the physical state, as an IMU's gravity vector would provide them.
    """

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, base_body: int, pose_fn: Callable[[], BasePose]):
        jnt = model.body_jntadr[base_body]
        if jnt < 0 or model.jnt_type[jnt] != mujoco.mjtJoint.mjJNT_FREE:
            raise ValueError("estimated kinematics needs a free-floating base")
        self._model, self._data, self._pose_fn = model, data, pose_fn
        self._adr = int(model.jnt_qposadr[jnt])
        self._scratch = mujoco.MjData(model)
        self._key: tuple[float, BasePose, bytes] | None = None

    def data(self) -> mujoco.MjData:
        pose = self._pose_fn()
        qpos = self._data.qpos.copy()
        key = (float(self._data.time), pose, qpos.tobytes())
        if key == self._key:
            return self._scratch
        true_quat = qpos[self._adr + 3 : self._adr + 7]
        true_yaw = _yaw_of_quat(true_quat)
        heading_fix = np.zeros(4)
        mujoco.mju_axisAngle2Quat(heading_fix, np.array([0.0, 0.0, 1.0]), wrap_angle(pose.yaw - true_yaw))
        corrected = np.zeros(4)
        mujoco.mju_mulQuat(corrected, heading_fix, true_quat)
        qpos[self._adr : self._adr + 2] = (pose.x, pose.y)
        qpos[self._adr + 3 : self._adr + 7] = corrected
        self._scratch.qpos[:] = qpos
        mujoco.mj_kinematics(self._model, self._scratch)
        mujoco.mj_camlight(self._model, self._scratch)
        self._key = key
        return self._scratch


def _yaw_of_quat(q: np.ndarray) -> float:
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


@dataclass(frozen=True)
class LocalizationStats:
    samples: int
    mean_xy_error: float
    max_xy_error: float
    final_xy_error: float
    mean_yaw_error_deg: float
    max_yaw_error_deg: float


class LocalizationMonitor:
    """Accumulates |estimate - ground truth| every control tick (evaluation only)."""

    def __init__(self, estimate: Callable[[], BasePose], truth: Callable[[], BasePose]):
        self._estimate, self._truth = estimate, truth
        self._xy: list[float] = []
        self._yaw: list[float] = []
        self.trajectory: list[tuple[BasePose, BasePose]] = []  # (estimate, truth), decimated

    def __call__(self) -> None:
        est, true = self._estimate(), self._truth()
        self._xy.append(float(np.linalg.norm(est.xy - true.xy)))
        self._yaw.append(abs(wrap_angle(est.yaw - true.yaw)))
        if len(self._xy) % 10 == 1:
            self.trajectory.append((est, true))

    def stats(self) -> LocalizationStats:
        if not self._xy:
            return LocalizationStats(0, 0.0, 0.0, 0.0, 0.0, 0.0)
        return LocalizationStats(
            samples=len(self._xy),
            mean_xy_error=float(np.mean(self._xy)),
            max_xy_error=float(np.max(self._xy)),
            final_xy_error=self._xy[-1],
            mean_yaw_error_deg=math.degrees(float(np.mean(self._yaw))),
            max_yaw_error_deg=math.degrees(float(np.max(self._yaw))),
        )
