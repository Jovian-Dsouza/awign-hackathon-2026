"""Packing / dispatch station (table + delivery totes) and the robot dock pad.

Station local frame: origin on the floor under the table centre; the robot-facing
side of the table is local +y (towards the shelves), the packer stands at -y.
"""

from __future__ import annotations

import mujoco
import numpy as np

from qc_warehouse.config import DockConfig, StationConfig
from qc_warehouse.constants import SHELF_FRICTION
from qc_warehouse.materials import add_box, look_at_xyaxes, rotate_xy, yaw_quat

TABLE_TOP_THICKNESS = 0.04
LEG_SIZE = 0.05
TOTE_WALL = 0.01
SIGN_SIZE = (0.5, 0.01, 0.12)  # half sizes of the DISPATCH sign plate
PARCEL_SIZES = ((0.18, 0.14, 0.08), (0.16, 0.12, 0.07), (0.14, 0.10, 0.06))
# Modesty panels around the bench base. A base-mounted 2-D lidar only sees the legs
# otherwise, and a planner would happily route the robot's mast under the table top.
SKIRT_THICKNESS = 0.02
SKIRT_Z = (0.05, 0.45)
TOTE_GAP = 0.1


def tote_site_name(index: int) -> str:
    return f"tote_{index}"


def tote_local_pos(cfg: StationConfig, index: int) -> np.ndarray:
    """Tote `index` inner floor centre in the station frame."""
    if not 0 <= index < cfg.num_totes:
        raise IndexError(f"tote {index} does not exist (station has {cfg.num_totes})")
    spacing = cfg.tote_size[0] + TOTE_GAP
    x = -(cfg.num_totes - 1) * spacing / 2 + index * spacing
    return np.array([x, 0.0, cfg.table_size[2] + TOTE_WALL])


def tote_world_pos(cfg: StationConfig, index: int) -> np.ndarray:
    local = tote_local_pos(cfg, index)
    xy = rotate_xy(local[:2], cfg.yaw_deg) + np.asarray(cfg.pos)
    return np.array([xy[0], xy[1], local[2]])


def _add_skirts(body: mujoco.MjsBody, cfg: StationConfig) -> None:
    w, d, _ = cfg.table_size
    half_t, half_h, mid_z = SKIRT_THICKNESS / 2, (SKIRT_Z[1] - SKIRT_Z[0]) / 2, sum(SKIRT_Z) / 2
    for side in (-1, 1):
        add_box(body, f"station_skirt_{'s' if side < 0 else 'n'}", (w / 2, half_t, half_h), (0, side * (d / 2 - half_t), mid_z), "table_leg")
        add_box(body, f"station_skirt_{'w' if side < 0 else 'e'}", (half_t, d / 2 - SKIRT_THICKNESS, half_h), (side * (w / 2 - half_t), 0, mid_z), "table_leg")


def _add_table(body: mujoco.MjsBody, cfg: StationConfig) -> float:
    w, d, h = cfg.table_size
    top_z = h - TABLE_TOP_THICKNESS / 2
    add_box(body, "station_table_top", (w / 2, d / 2, TABLE_TOP_THICKNESS / 2), (0, 0, top_z), "table_top", friction=SHELF_FRICTION)
    leg_h = (h - TABLE_TOP_THICKNESS) / 2
    lx, ly = w / 2 - LEG_SIZE, d / 2 - LEG_SIZE
    for i, (sx, sy) in enumerate(((-1, -1), (1, -1), (-1, 1), (1, 1))):
        add_box(body, f"station_leg{i}", (LEG_SIZE / 2, LEG_SIZE / 2, leg_h), (sx * lx, sy * ly, leg_h), "table_leg")
    _add_skirts(body, cfg)
    return h


def _add_tote(body: mujoco.MjsBody, cfg: StationConfig, index: int) -> None:
    center_x, _, inner_floor = tote_local_pos(cfg, index)
    floor_z = inner_floor - TOTE_WALL
    w, d, h = cfg.tote_size
    hw, hd, hh, t = w / 2, d / 2, h / 2, TOTE_WALL / 2
    name = tote_site_name(index)
    add_box(body, f"{name}_bottom", (hw, hd, t), (center_x, 0, floor_z + t), "tote", friction=SHELF_FRICTION)
    add_box(body, f"{name}_wall_n", (hw, t, hh), (center_x, hd - t, floor_z + hh), "tote")
    add_box(body, f"{name}_wall_s", (hw, t, hh), (center_x, -hd + t, floor_z + hh), "tote")
    add_box(body, f"{name}_wall_e", (t, hd - 2 * t, hh), (center_x + hw - t, 0, floor_z + hh), "tote")
    add_box(body, f"{name}_wall_w", (t, hd - 2 * t, hh), (center_x - hw + t, 0, floor_z + hh), "tote")
    body.add_site(name=name, pos=[center_x, 0, floor_z + TOTE_WALL], size=[0.02, 0, 0], rgba=[0.1, 0.9, 0.3, 0.0], group=4)


def _add_decor(body: mujoco.MjsBody, cfg: StationConfig, top_z: float) -> None:
    w, d, _ = cfg.table_size
    # Sign on two posts rising from the packer-side (-y) table corners.
    sign_z = top_z + 0.55
    post_y = -d / 2 + LEG_SIZE / 2
    add_box(body, "dispatch_sign", SIGN_SIZE, (0, post_y, sign_z), "sign", collide=False)
    post_half_h = (sign_z + SIGN_SIZE[2] - top_z) / 2
    for side in (-1, 1):
        x = side * (w / 2 - LEG_SIZE)
        add_box(body, f"dispatch_sign_post_{'w' if side < 0 else 'e'}", (0.012, 0.012, post_half_h), (x, post_y, top_z + post_half_h), "table_leg", collide=False)
    add_box(body, "dispatch_sign_bar", (w / 2 - LEG_SIZE, 0.012, 0.012), (0, post_y, sign_z + SIGN_SIZE[2]), "table_leg", collide=False)

    z = 0.0
    for i, (pw, pd, ph) in enumerate(PARCEL_SIZES):
        add_box(body, f"parcel_stack{i}", (pw / 2, pd / 2, ph / 2), (w / 2 + 0.25, -0.1, z + ph / 2), "cardboard", collide=False)
        z += ph


def add_packing_station(spec: mujoco.MjSpec, cfg: StationConfig) -> mujoco.MjsBody:
    body = spec.worldbody.add_body(name="packing_station", pos=[*cfg.pos, 0.0], quat=yaw_quat(cfg.yaw_deg))
    top_z = _add_table(body, cfg)

    for i in range(cfg.num_totes):
        _add_tote(body, cfg, i)

    # Primary drop target = centre of the first tote, just above its floor.
    first_x = tote_local_pos(cfg, 0)[0]
    body.add_site(name="drop_zone", pos=[first_x, 0, top_z + 0.05], size=[0.03, 0, 0], rgba=[0.1, 0.9, 0.3, 0.35], group=4)
    _add_decor(body, cfg, top_z)

    cam_pos = [1.4, 1.6, 1.9]
    body.add_camera(name="packing_station_cam", pos=cam_pos, xyaxes=look_at_xyaxes(cam_pos, [0, 0, top_z]), fovy=50)
    return body


def add_robot_dock(spec: mujoco.MjSpec, cfg: DockConfig) -> None:
    world = spec.worldbody
    add_box(world, "robot_dock_pad", (cfg.pad_size[0] / 2, cfg.pad_size[1] / 2, 0.001), (*cfg.pos, 0.001), "dock", collide=False)
    world.add_site(name="robot_spawn", pos=[*cfg.pos, 0.0], size=[0.05, 0, 0], rgba=[0.2, 0.45, 0.85, 0.0], group=4)
