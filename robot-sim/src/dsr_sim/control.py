"""Command interface for the rover and arm (mecanum IK, body-frame twist, arm poses in degrees)."""

from __future__ import annotations

import mujoco
import numpy as np

from dsr_sim.robot import (
    ARM_JOINTS,
    ARM_PREFIX,
    HALF_TRACK,
    HALF_WHEELBASE,
    POSES_DEG,
    WHEEL_RADIUS,
    WHEELS,
)


def wheel_speeds(vx: float, vy: float, wz: float) -> dict[str, float]:
    """Body twist (m/s, m/s, rad/s; x forward, y left) -> wheel angular speeds (rad/s, + = forward)."""
    k = HALF_WHEELBASE + HALF_TRACK
    r = WHEEL_RADIUS
    return {
        "fl": (vx - vy - k * wz) / r,
        "fr": (vx + vy + k * wz) / r,
        "rl": (vx + vy - k * wz) / r,
        "rr": (vx - vy + k * wz) / r,
    }


class Robot:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self.model, self.data = model, data
        mujoco.mj_forward(model, data)  # populate xpos so `pose` is valid before the first step
        self._act = {n: model.actuator(n).id for n in ("robot/vx", "robot/vy", "robot/wz")}
        self._wheel_act = {t: model.actuator(f"robot/wheel_{t}").id for t in WHEELS}
        self._arm_act = {j: model.actuator(ARM_PREFIX + j).id for j in ARM_JOINTS}
        self._qx = model.jnt_qposadr[model.joint("robot/base_x").id]
        self._qy = model.jnt_qposadr[model.joint("robot/base_y").id]
        self._qyaw = model.jnt_qposadr[model.joint("robot/base_yaw").id]
        self._x0 = np.array([data.qpos[self._qx], data.qpos[self._qy]])

    @property
    def pose(self) -> tuple[float, float, float]:
        """World x, y (m) and yaw (rad) of the rover."""
        base = self.model.body("robot/base").id
        xy = self.data.xpos[base][:2]
        return float(xy[0]), float(xy[1]), float(self.data.qpos[self._qyaw])

    def set_twist(self, vx: float, vy: float, wz: float) -> None:
        """Command a body-frame velocity. Slide actuators act in the world frame, so rotate by yaw."""
        yaw = self.data.qpos[self._qyaw]
        c, s = np.cos(yaw), np.sin(yaw)
        self.data.ctrl[self._act["robot/vx"]] = c * vx - s * vy
        self.data.ctrl[self._act["robot/vy"]] = s * vx + c * vy
        self.data.ctrl[self._act["robot/wz"]] = wz
        for tag, w in wheel_speeds(vx, vy, wz).items():
            self.data.ctrl[self._wheel_act[tag]] = w

    def stop(self) -> None:
        self.set_twist(0.0, 0.0, 0.0)

    def set_arm_deg(self, angles: tuple[float, ...] | str) -> None:
        values = POSES_DEG[angles] if isinstance(angles, str) else angles
        for joint, deg in zip(ARM_JOINTS, values):
            self.data.ctrl[self._arm_act[joint]] = np.deg2rad(deg)

    def arm_deg(self) -> dict[str, float]:
        out = {}
        for joint in ARM_JOINTS:
            jid = self.model.joint(ARM_PREFIX + joint).id
            out[joint] = float(np.rad2deg(self.data.qpos[self.model.jnt_qposadr[jid]]))
        return out
