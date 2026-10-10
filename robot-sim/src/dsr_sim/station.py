"""Drop tray (where delivered items end up) and the robot dock marker. Both sit on the table top."""

from __future__ import annotations

import mujoco

from dsr_sim.config import DockConfig, TrayConfig
from dsr_sim.constants import SHELF_FRICTION
from dsr_sim.materials import add_box, look_at_xyaxes, yaw_quat

WALL = 0.006
FLOOR = 0.006


def add_drop_tray(spec: mujoco.MjSpec, cfg: TrayConfig) -> mujoco.MjsBody:
    """Open cardboard tray; `tote_0` and `drop_zone` sites mark the inside floor."""
    body = spec.worldbody.add_body(name="drop_tray", pos=[*cfg.pos, 0.0], quat=yaw_quat(cfg.yaw_deg))
    w, d, h = cfg.size
    hw, hd, hh, t = w / 2, d / 2, h / 2, WALL / 2
    add_box(body, "drop_tray_floor", (hw, hd, FLOOR / 2), (0, 0, FLOOR / 2), "cardboard_dark", friction=SHELF_FRICTION)
    add_box(body, "drop_tray_wall_n", (hw, t, hh), (0, hd - t, hh), "cardboard")
    add_box(body, "drop_tray_wall_s", (hw, t, hh), (0, -hd + t, hh), "cardboard")
    add_box(body, "drop_tray_wall_e", (t, hd - 2 * t, hh), (hw - t, 0, hh), "cardboard")
    add_box(body, "drop_tray_wall_w", (t, hd - 2 * t, hh), (-hw + t, 0, hh), "cardboard")
    body.add_site(name="tote_0", pos=[0, 0, FLOOR], size=[0.02, 0, 0], rgba=[0.1, 0.9, 0.3, 0.0], group=4)
    body.add_site(name="drop_zone", pos=[0, 0, FLOOR + 0.05], size=[0.03, 0, 0], rgba=[0.1, 0.9, 0.3, 0.35], group=4)
    cam_pos = [0.0, -0.45, 0.45]
    body.add_camera(name="drop_tray_cam", pos=cam_pos, xyaxes=look_at_xyaxes(cam_pos, [0, 0, 0.03]), fovy=50)
    return body


def add_robot_dock(spec: mujoco.MjSpec, cfg: DockConfig) -> None:
    world = spec.worldbody
    add_box(world, "robot_dock_pad", (cfg.pad_size[0] / 2, cfg.pad_size[1] / 2, 0.0006), (*cfg.pos, 0.0006), "dock", collide=False)
    world.add_site(name="robot_spawn", pos=[*cfg.pos, 0.0], size=[0.05, 0, 0], rgba=[0.2, 0.45, 0.85, 0.0], group=4)
