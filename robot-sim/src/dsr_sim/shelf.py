"""Procedural pallet-rack style shelving unit.

Local frame: origin at the centre of the footprint on the floor, the open front
faces local -y, and slot 0 is at local -x (the left side when facing the shelf).
"""

from __future__ import annotations

import mujoco
import numpy as np

from dsr_sim.config import Placement, ShelfConfig
from dsr_sim.constants import SHELF_FRICTION
from dsr_sim.materials import add_box, look_at_xyaxes, rotate_xy, yaw_quat

SLOT_DEPTH_OFFSET = -0.02  # slot centres sit slightly toward the open front
CARDBOARD_T = 0.008
LIP_THICKNESS = 0.01
BACK_PANEL_THICKNESS = 0.008
HEADER_HEIGHT = 0.05
BEAM_HALF_HEIGHT = 0.008  # slim front beam: leaves headroom for a top-down wrist
STOCK_BOX_SIZES = ((0.12, 0.16, 0.11), (0.13, 0.17, 0.15), (0.11, 0.16, 0.09), (0.12, 0.17, 0.13))
SHELF_HEADER_COLORS = {"A": (0.85, 0.20, 0.20, 1), "B": (0.20, 0.45, 0.85, 1), "C": (0.20, 0.70, 0.35, 1)}
CAMERA_STANDOFF = 0.7
CAMERA_HEIGHT = 0.35


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


def _add_cardboard(body: mujoco.MjsBody, cfg: ShelfConfig, prefix: str) -> None:
    """Open-front cardboard cubby unit: side panels, back, a floor per level, dividers, lid."""
    t = CARDBOARD_T / 2
    hw, hd = cfg.width / 2, cfg.depth / 2
    for sx, tag in ((-1, "w"), (1, "e")):
        add_box(body, f"{prefix}_side_{tag}", (t, hd, cfg.height / 2), (sx * (hw - t), 0, cfg.height / 2), "cardboard")
    add_box(body, f"{prefix}_back", (hw, t, cfg.height / 2), (0, hd - t, cfg.height / 2), "cardboard_dark")
    for level, top in enumerate(cfg.board_heights):
        add_box(body, f"{prefix}_L{level}_board", (hw, hd, cfg.board_thickness / 2), (0, 0, top - cfg.board_thickness / 2), "cardboard", friction=SHELF_FRICTION)
    add_box(body, f"{prefix}_lid", (hw, hd, t), (0, 0, cfg.height - t), "cardboard")
    inner = cfg.width - 2 * CARDBOARD_T
    width = inner / cfg.slots_per_level
    base = cfg.board_heights[0] - cfg.board_thickness
    for i in range(1, cfg.slots_per_level):
        x = -inner / 2 + i * width
        add_box(body, f"{prefix}_divider{i}", (t, hd, (cfg.height - base) / 2), (x, 0, (cfg.height + base) / 2), "cardboard")


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


def add_shelf_unit(spec: mujoco.MjSpec, cfg: ShelfConfig, placement: Placement) -> mujoco.MjsBody:
    """Add a static cardboard cubby shelf body named shelf_<name> to the world."""
    prefix = f"shelf_{placement.name}"
    body = spec.worldbody.add_body(name=prefix, pos=[*placement.pos, 0.0], quat=yaw_quat(placement.yaw_deg))
    _add_cardboard(body, cfg, prefix)
    _add_slot_sites(body, cfg, placement.name)
    cam_pos = [0.0, -CAMERA_STANDOFF, CAMERA_HEIGHT]
    body.add_camera(name=f"{prefix}_cam", pos=cam_pos, xyaxes=look_at_xyaxes(cam_pos, [0, 0, 0.2]), fovy=50)
    return body
