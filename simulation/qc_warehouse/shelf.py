"""Procedural pallet-rack style shelving unit.

Local frame: origin at the centre of the footprint on the floor, the open front
faces local -y, and slot 0 is at local -x (the left side when facing the shelf).
"""

from __future__ import annotations

import mujoco
import numpy as np

from qc_warehouse.config import Placement, ShelfConfig
from qc_warehouse.constants import SHELF_FRICTION
from qc_warehouse.materials import add_box, look_at_xyaxes, rotate_xy, yaw_quat

SLOT_DEPTH_OFFSET = -0.04  # slot centres sit slightly toward the open front
LIP_THICKNESS = 0.015
BACK_PANEL_THICKNESS = 0.008
HEADER_HEIGHT = 0.12
BEAM_HALF_HEIGHT = 0.015  # slim front beam: leaves headroom for a top-down wrist
STOCK_BOX_SIZES = ((0.12, 0.16, 0.11), (0.13, 0.17, 0.15), (0.11, 0.16, 0.09), (0.12, 0.17, 0.13))
SHELF_HEADER_COLORS = {"A": (0.85, 0.20, 0.20, 1), "B": (0.20, 0.45, 0.85, 1), "C": (0.20, 0.70, 0.35, 1)}
CAMERA_STANDOFF = 1.9
CAMERA_HEIGHT = 1.3


def slot_site_name(shelf: str, level: int, slot: int) -> str:
    return f"shelf_{shelf}_L{level}_S{slot}"


def slot_width(cfg: ShelfConfig) -> float:
    """Slots are spread over the inner width (between the posts) so edge slots leave
    room for open fingers next to the uprights."""
    return (cfg.width - 2 * cfg.post_size) / cfg.slots_per_level


def slot_local_pos(cfg: ShelfConfig, level: int, slot: int) -> np.ndarray:
    width = slot_width(cfg)
    x = -width * cfg.slots_per_level / 2 + (slot + 0.5) * width
    return np.array([x, SLOT_DEPTH_OFFSET, cfg.board_heights[level]])


def slot_world_pos(cfg: ShelfConfig, placement: Placement, level: int, slot: int) -> np.ndarray:
    local = slot_local_pos(cfg, level, slot)
    xy = rotate_xy(local[:2], placement.yaw_deg) + np.asarray(placement.pos)
    return np.array([xy[0], xy[1], local[2]])


def _add_frame(body: mujoco.MjsBody, cfg: ShelfConfig, prefix: str) -> None:
    half_post = cfg.post_size / 2
    px, py = cfg.width / 2 - half_post, cfg.depth / 2 - half_post
    for i, (sx, sy) in enumerate(((-1, -1), (1, -1), (-1, 1), (1, 1))):
        add_box(body, f"{prefix}_post{i}", (half_post, half_post, cfg.height / 2), (sx * px, sy * py, cfg.height / 2), "rack_steel")

    back_y = cfg.depth / 2 - BACK_PANEL_THICKNESS / 2
    add_box(body, f"{prefix}_back", (cfg.width / 2 - cfg.post_size, BACK_PANEL_THICKNESS / 2, cfg.height / 2), (0, back_y, cfg.height / 2), "back_panel")


def _add_levels(body: mujoco.MjsBody, cfg: ShelfConfig, prefix: str) -> None:
    half_t = cfg.board_thickness / 2
    lip_y = -cfg.depth / 2 + LIP_THICKNESS / 2
    for level, top in enumerate(cfg.board_heights):
        add_box(body, f"{prefix}_L{level}_board", (cfg.width / 2, cfg.depth / 2, half_t), (0, 0, top - half_t), "board", friction=SHELF_FRICTION)
        add_box(body, f"{prefix}_L{level}_beam", (cfg.width / 2, LIP_THICKNESS / 2, BEAM_HALF_HEIGHT), (0, lip_y, top - BEAM_HALF_HEIGHT), "lip")
        if cfg.lip_height > 0:
            add_box(body, f"{prefix}_L{level}_lip", (cfg.width / 2, LIP_THICKNESS / 2, cfg.lip_height / 2), (0, lip_y, top + cfg.lip_height / 2), "lip")


def _add_slot_sites(body: mujoco.MjsBody, cfg: ShelfConfig, shelf: str) -> None:
    for level in range(len(cfg.board_heights)):
        for slot in range(cfg.slots_per_level):
            body.add_site(
                name=slot_site_name(shelf, level, slot),
                pos=list(slot_local_pos(cfg, level, slot)),
                size=[0.01, 0, 0],
                rgba=[0, 1, 0, 0.0],
                group=4,
            )


def _add_stock_boxes(body: mujoco.MjsBody, cfg: ShelfConfig, prefix: str) -> None:
    """Fill non-pickable levels with static cardboard back-stock for realism."""
    for level in range(len(cfg.board_heights)):
        if level in cfg.pickable_levels:
            continue
        gap = (cfg.board_heights[level + 1] - cfg.board_heights[level]) if level + 1 < len(cfg.board_heights) else cfg.height - cfg.board_heights[level]
        for slot in range(cfg.slots_per_level):
            w, d, h = STOCK_BOX_SIZES[(slot + level) % len(STOCK_BOX_SIZES)]
            h = min(h, gap - 0.05)
            x, y, z = slot_local_pos(cfg, level, slot)
            material = "cardboard" if (slot + level) % 2 else "cardboard_dark"
            add_box(body, f"{prefix}_L{level}_stock{slot}", (w, d, h / 2), (x, y + 0.04, z + h / 2), material)


def add_shelf_unit(spec: mujoco.MjSpec, cfg: ShelfConfig, placement: Placement) -> mujoco.MjsBody:
    """Add a static shelving unit body named shelf_<name> to the world."""
    prefix = f"shelf_{placement.name}"
    body = spec.worldbody.add_body(name=prefix, pos=[*placement.pos, 0.0], quat=yaw_quat(placement.yaw_deg))

    _add_frame(body, cfg, prefix)
    _add_levels(body, cfg, prefix)
    _add_stock_boxes(body, cfg, prefix)
    _add_slot_sites(body, cfg, placement.name)

    header_rgba = SHELF_HEADER_COLORS.get(placement.name, (0.9, 0.9, 0.9, 1))
    header = add_box(body, f"{prefix}_header", (cfg.width / 2, 0.01, HEADER_HEIGHT / 2), (0, -cfg.depth / 2 - 0.01, cfg.height - HEADER_HEIGHT / 2), "lip", collide=False)
    header.material = ""
    header.rgba = list(header_rgba)

    cam_pos = [0.0, -CAMERA_STANDOFF, CAMERA_HEIGHT]
    body.add_camera(name=f"{prefix}_cam", pos=cam_pos, xyaxes=look_at_xyaxes(cam_pos, [0, 0, 0.75]), fovy=55)
    return body
