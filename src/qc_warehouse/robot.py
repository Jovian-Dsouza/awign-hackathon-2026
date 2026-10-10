"""Attach the Hello Robot Stretch 3 (MuJoCo Menagerie) to the warehouse scene.

All robot element names get the ROBOT_PREFIX, e.g. `robot/joint_lift`.
"""

from __future__ import annotations

from pathlib import Path

import mujoco

from qc_warehouse.config import DockConfig
from qc_warehouse.constants import THIRD_PARTY_DIR
from qc_warehouse.items import AssetError, absolutize_files
from qc_warehouse.materials import yaw_quat

ROBOT_PREFIX = "robot/"
STRETCH_XML = THIRD_PARTY_DIR / "stretch3" / "stretch.xml"
STRETCH_ROOT_BODY = "base_link"

# Menagerie limits wheel velocity ctrl to +-6 (~0.1 m/s). The real Stretch 3
# drives at up to ~0.3 m/s, so allow ~0.24 m/s.
WHEEL_CTRL_LIMIT = 15.0
WHEEL_ACTUATORS = ("left_wheel_vel", "right_wheel_vel")
FINGERTIP_PAD_JOINTS = ("rubber_left_x", "rubber_left_y", "rubber_right_x", "rubber_right_y")

# Stowed posture used for the scene keyframe: lift low, arm in, wrist folded.
STOW_CTRL = {"lift": 0.35, "arm": 0.0, "wrist_yaw": 0.0, "wrist_pitch": 0.0, "wrist_roll": 0.0, "gripper": 0.0, "head_pan": -1.57, "head_tilt": -0.5}


def stretch_xml_path(path: Path = STRETCH_XML) -> Path:
    if not path.exists():
        raise AssetError(f"Stretch 3 model not found at {path}. Run `uv run python scripts/fetch_assets.py` first.")
    return path


def _load_stretch_spec(path: Path) -> mujoco.MjSpec:
    child = mujoco.MjSpec.from_file(str(path))
    for key in list(child.keys):
        child.delete(key)  # scene-level keyframes are added after attaching
    # The Menagerie fingertip pads hang on near-free hinges (stiffness 1). Held items
    # pivot about the pinch line and creep out within seconds; stiffening them makes
    # the solver crawl. Mount the pads rigidly instead.
    for joint in list(child.joints):
        if joint.name in FINGERTIP_PAD_JOINTS:
            child.delete(joint)
    for actuator in child.actuators:
        if actuator.name in WHEEL_ACTUATORS:
            actuator.ctrlrange = [-WHEEL_CTRL_LIMIT, WHEEL_CTRL_LIMIT]
    absolutize_files(child, path.parent)
    return child


def add_stretch(spec: mujoco.MjSpec, dock: DockConfig, path: Path = STRETCH_XML) -> mujoco.MjsBody:
    """Attach Stretch 3 standing on the dock pad, heading along +x rotated by dock yaw."""
    child = _load_stretch_spec(stretch_xml_path(path))
    root = child.body(STRETCH_ROOT_BODY)
    if root is None:
        raise AssetError(f"{path}: body {STRETCH_ROOT_BODY!r} not found")
    frame = spec.worldbody.add_frame(pos=[*dock.pos, 0.0], quat=yaw_quat(dock.yaw_deg))
    return frame.attach_body(root, ROBOT_PREFIX, "")


def add_stow_keyframe(spec: mujoco.MjSpec, model: mujoco.MjModel) -> None:
    """Add a 'stow' keyframe holding the robot's default posture (call after attaching)."""
    ctrl = [0.0] * model.nu
    for name, value in STOW_CTRL.items():
        ctrl[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, ROBOT_PREFIX + name)] = value
    spec.add_key(name="stow", qpos=list(model.qpos0), ctrl=ctrl)
