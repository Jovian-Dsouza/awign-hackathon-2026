"""Thin read/command interface to the simulated Stretch 3.

Base frame: +x forward, +y left. The telescoping arm extends toward base -y
(the robot's right side). Measured kinematics (Menagerie model):
  wrist pitch 0  (side grasp):     grasp centre ~ (-0.022, -(0.416 + ext), lift + 0.112)
  wrist pitch -pi/2 (top-down):    grasp centre ~ (-0.021, -(0.121 + ext), lift - 0.131)
"""

from __future__ import annotations

import contextlib
import math

import mujoco
import numpy as np

from qc_warehouse.autonomy.frames import BasePose
from qc_warehouse.autonomy.localization import EstimatedKinematics, Localizer
from qc_warehouse.robot import ROBOT_PREFIX

WHEEL_RADIUS = 0.0508
WHEEL_SEPARATION = 0.3407
ARM_RANGE = (0.0, 0.50)
LIFT_RANGE = (0.05, 1.08)
GRIPPER_OPEN = 0.04
GRIPPER_CLOSED = -0.02
GRIPPER_GAP_PER_Q = 2.95   # inner pad gap (m) per unit of gripper slide (measured: 0.118 m at 0.04)
GRIPPER_MIN_PRESHAPE = 0.012
# Side grasps (wrist level): pad gap ~= 0.011 + 3.03 * q, and the curved fingers pull
# the pads back toward the palm by ~1.2 cm per 0.01 of closing (measured).
SIDE_GAP_OFFSET, SIDE_GAP_PER_Q = 0.011, 3.03
SIDE_PAD_RETRACT_PER_Q = 1.2
SIDE_GRASP_OFFSET = np.array([-0.022, -0.416, 0.112])  # (x, y at ext=0, z above lift)
TOP_GRASP_OFFSET = np.array([-0.021, -0.121, -0.131])
PITCH_SIDE = 0.0
PITCH_DOWN = -math.pi / 2
MAX_LINEAR_ACCEL = 0.25    # m/s^2
MAX_ANGULAR_ACCEL = 1.0    # rad/s^2
DRIVE_DT = 0.02            # control period the acceleration limits assume
JOINT_ACTUATORS = ("lift", "arm", "wrist_yaw", "wrist_pitch", "wrist_roll", "gripper", "head_pan", "head_tilt")


def _name(kind: mujoco.mjtObj, model: mujoco.MjModel, name: str) -> int:
    idx = mujoco.mj_name2id(model, kind, ROBOT_PREFIX + name)
    if idx < 0:
        raise ValueError(f"Stretch element {ROBOT_PREFIX + name!r} missing from model")
    return idx


class Stretch:
    """Robot interface. `base_pose()` is the robot's own estimate when a localizer is attached
    (see `use_localizer`); otherwise it falls back to simulator ground truth."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.model, self.data = model, data
        self.localizer: Localizer | None = None
        self._kinematics: EstimatedKinematics | None = None
        A = mujoco.mjtObj.mjOBJ_ACTUATOR
        self._act = {n: _name(A, model, n) for n in (*JOINT_ACTUATORS, "left_wheel_vel", "right_wheel_vel")}
        self._wheel_gear = float(model.actuator_gear[self._act["left_wheel_vel"], 0])
        self._wheel_limit = float(model.actuator_ctrlrange[self._act["left_wheel_vel"], 1])
        self.base_body = _name(mujoco.mjtObj.mjOBJ_BODY, model, "base_link")
        lift_j = _name(mujoco.mjtObj.mjOBJ_JOINT, model, "joint_lift")
        arm_j = [_name(mujoco.mjtObj.mjOBJ_JOINT, model, f"joint_arm_l{i}") for i in range(4)]
        self._lift_q, self._lift_v = model.jnt_qposadr[lift_j], model.jnt_dofadr[lift_j]
        self._arm_q = [model.jnt_qposadr[j] for j in arm_j]
        self._arm_v = [model.jnt_dofadr[j] for j in arm_j]
        self._grip_q = model.jnt_qposadr[_name(mujoco.mjtObj.mjOBJ_JOINT, model, "joint_gripper_slide")]
        self._pad_geoms = [
            next(g for g in range(model.ngeom) if model.geom_bodyid[g] == _name(mujoco.mjtObj.mjOBJ_BODY, model, tip) and model.geom_contype[g])
            for tip in ("rubber_tip_left", "rubber_tip_right")
        ]
        self.head_camera = ROBOT_PREFIX + "d435i_camera_rgb"
        self._wheel_q = [model.jnt_qposadr[_name(mujoco.mjtObj.mjOBJ_JOINT, model, f"joint_{side}_wheel")] for side in ("left", "right")]
        self._gyro_adr = int(model.sensor_adr[_name(mujoco.mjtObj.mjOBJ_SENSOR, model, "base_gyro")])
        imu_site = _name(mujoco.mjtObj.mjOBJ_SITE, model, "base_imu")
        self._imu_to_base = self._site_rotation_in_base(imu_site)
        self._last_v = 0.0
        self._last_w = 0.0

    def _site_rotation_in_base(self, site: int) -> np.ndarray:
        scratch = mujoco.MjData(self.model)
        mujoco.mj_kinematics(self.model, scratch)
        base = scratch.xmat[self.base_body].reshape(3, 3)
        return base.T @ scratch.site_xmat[site].reshape(3, 3)

    def use_localizer(self, localizer: Localizer) -> None:
        """Drive on `localizer`'s estimate; arm/camera kinematics follow the same estimate."""
        self.localizer = localizer
        self._kinematics = EstimatedKinematics(self.model, self.data, self.base_body, localizer.base_pose)

    def freeze_localization(self) -> contextlib.AbstractContextManager[None]:
        return self.localizer.freeze() if self.localizer is not None else contextlib.nullcontext()

    # ---- state -------------------------------------------------------------
    def true_base_pose(self) -> BasePose:
        """Simulator ground truth (evaluation only once a localizer is attached)."""
        xmat = self.data.xmat[self.base_body].reshape(3, 3)
        pos = self.data.xpos[self.base_body]
        return BasePose(float(pos[0]), float(pos[1]), math.atan2(xmat[1, 0], xmat[0, 0]))

    def base_pose(self) -> BasePose:
        return self.localizer.base_pose() if self.localizer is not None else self.true_base_pose()

    def estimated_data(self) -> mujoco.MjData:
        """Kinematic state (body/geom/camera poses) as the robot believes it to be."""
        return self._kinematics.data() if self._kinematics is not None else self.data

    def pinch_point(self) -> np.ndarray:
        """Midpoint between the two finger pads: where an object actually gets squeezed."""
        data = self.estimated_data()
        return (data.geom_xpos[self._pad_geoms[0]] + data.geom_xpos[self._pad_geoms[1]]) / 2

    def wheel_angles(self) -> tuple[float, float]:
        """Wheel encoder readings (rad): left, right."""
        return float(self.data.qpos[self._wheel_q[0]]), float(self.data.qpos[self._wheel_q[1]])

    def yaw_rate(self) -> float:
        """Base yaw rate (rad/s) from the IMU gyro, rotated into the base frame."""
        gyro = self.data.sensordata[self._gyro_adr : self._gyro_adr + 3]
        return float((self._imu_to_base @ gyro)[2])

    def lift(self) -> float:
        return float(self.data.qpos[self._lift_q])

    def arm_extension(self) -> float:
        return float(sum(self.data.qpos[a] for a in self._arm_q))

    def gripper(self) -> float:
        return float(self.data.qpos[self._grip_q])

    def ctrl(self, name: str) -> float:
        return float(self.data.ctrl[self._act[name]])

    # ---- commands ----------------------------------------------------------
    def clip_ctrl(self, name: str, value: float) -> float:
        lo, hi = self.model.actuator_ctrlrange[self._act[name]]
        return float(np.clip(value, lo, hi))

    def command(self, **targets: float) -> None:
        """Set position targets, e.g. command(lift=0.6, arm=0.2). Values are clipped to ctrl ranges."""
        for name, value in targets.items():
            if name not in JOINT_ACTUATORS:
                raise KeyError(f"unknown joint actuator {name!r}")
            idx = self._act[name]
            lo, hi = self.model.actuator_ctrlrange[idx]
            self.data.ctrl[idx] = float(np.clip(value, lo, hi))

    def drive(self, v: float, w: float) -> None:
        """Differential drive: forward speed v (m/s), yaw rate w (rad/s), acceleration-limited.

        Assumes one call per control tick (CONTROL_DT), like a real base controller.
        """
        v = self._last_v + float(np.clip(v - self._last_v, -MAX_LINEAR_ACCEL * DRIVE_DT, MAX_LINEAR_ACCEL * DRIVE_DT))
        w = self._last_w + float(np.clip(w - self._last_w, -MAX_ANGULAR_ACCEL * DRIVE_DT, MAX_ANGULAR_ACCEL * DRIVE_DT))
        self._last_v, self._last_w = v, w
        left = (v - w * WHEEL_SEPARATION / 2) / WHEEL_RADIUS * self._wheel_gear
        right = (v + w * WHEEL_SEPARATION / 2) / WHEEL_RADIUS * self._wheel_gear
        scale = max(1.0, abs(left) / self._wheel_limit, abs(right) / self._wheel_limit)
        self.data.ctrl[self._act["left_wheel_vel"]] = left / scale
        self.data.ctrl[self._act["right_wheel_vel"]] = right / scale

    @staticmethod
    def preshape_for(width: float, margin: float = 0.03, minimum: float = GRIPPER_MIN_PRESHAPE) -> float:
        """Gripper command for a pad gap of `width + margin` (negative margin = squeeze)."""
        return float(np.clip((width + margin) / GRIPPER_GAP_PER_Q, minimum, GRIPPER_OPEN))

    @staticmethod
    def side_pad_retraction(preshape_q: float, width: float) -> float:
        """How far (m) the pads slide back toward the palm while closing from preshape onto `width`."""
        contact_q = (width - SIDE_GAP_OFFSET) / SIDE_GAP_PER_Q
        return max(0.0, SIDE_PAD_RETRACT_PER_Q * (preshape_q - contact_q))

    def stop(self) -> None:
        """Immediate stop (wheel velocity servo to zero)."""
        self._last_v = self._last_w = 0.0
        self.data.ctrl[self._act["left_wheel_vel"]] = 0.0
        self.data.ctrl[self._act["right_wheel_vel"]] = 0.0

    def lift_stalled(self, still: float = 0.003, min_error: float = 0.01) -> bool:
        """Lift stopped short of its target (blocked by contact)."""
        return abs(self.data.qvel[self._lift_v]) < still and abs(self.lift() - self.ctrl("lift")) > min_error

    def joint_targets_reached(self, tol: float = 0.01, settled_tol: float = 0.03, still: float = 0.003) -> bool:
        """Lift and arm are on target, or have stopped close to it (the lift sags ~1 cm under load)."""
        lift_err = abs(self.lift() - self.ctrl("lift"))
        arm_err = abs(self.arm_extension() - self.ctrl("arm"))
        if lift_err < tol and arm_err < tol:
            return True
        moving = abs(self.data.qvel[self._lift_v]) > still or any(abs(self.data.qvel[a]) > still for a in self._arm_v)
        return not moving and lift_err < settled_tol and arm_err < settled_tol
