"""Load third-party grocery item MJCFs and attach them as free bodies on shelf slots."""

from __future__ import annotations

import math
from pathlib import Path

import mujoco
import numpy as np

from dsr_sim.config import Item, Layout
from dsr_sim.constants import ITEM_DROP_CLEARANCE, ITEM_SOURCES, THIRD_PARTY_DIR
from dsr_sim.materials import euler_quat, quat_mul, rotate_xy, yaw_quat
from dsr_sim.shelf import SLOT_DEPTH_OFFSET, slot_width, slot_world_pos

_BOX_CORNERS = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)


class AssetError(FileNotFoundError):
    """Raised when an item's asset files are missing."""


def item_body_name(item: Item) -> str:
    return f"item_{item.name}"


def item_asset_path(item: Item, root: Path = THIRD_PARTY_DIR) -> Path:
    path = root / ITEM_SOURCES[item.source].format(asset=item.asset)
    if not path.exists():
        raise AssetError(f"asset for item {item.name!r} not found at {path}. Run `uv run python scripts/fetch_assets.py` first.")
    return path


def item_quat(item: Item, shelf_yaw_deg: float) -> np.ndarray:
    """World orientation: upright the asset (euler), then yaw with the shelf + item yaw."""
    return quat_mul(yaw_quat(shelf_yaw_deg + item.yaw_deg), euler_quat(item.euler_deg))


def collision_bounds(child: mujoco.MjSpec, quat: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Axis-aligned bounds of the item's collision geoms after rotating by `quat`."""
    model = child.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    rot = np.zeros(9)
    mujoco.mju_quat2Mat(rot, quat)
    rot = rot.reshape(3, 3)

    points = []
    for g in range(model.ngeom):
        if not (model.geom_contype[g] or model.geom_conaffinity[g]):
            continue
        center, half = model.geom_aabb[g, :3], model.geom_aabb[g, 3:]
        local = _BOX_CORNERS * half + center
        world = local @ data.geom_xmat[g].reshape(3, 3).T + data.geom_xpos[g]
        points.append(world @ rot.T)
    if not points:
        raise AssetError("item model has no collision geoms")
    stacked = np.concatenate(points)
    return stacked.min(axis=0), stacked.max(axis=0)


def scale_spec(child: mujoco.MjSpec, factor: float) -> None:
    """Uniformly scale an item model's geometry; explicit geom masses scale with volume.

    Covers what HOPE/YCB use (body/geom/site positions, primitive sizes, mesh
    scale, geom mass/density). Explicit <inertial>, fromto and joints are not
    scaled. Rigid items have no joints, and add_item rejects any asset whose top
    body has one.
    """
    if factor == 1.0:
        return
    for body in child.bodies:
        body.pos = np.asarray(body.pos) * factor
    for site in child.sites:
        site.pos = np.asarray(site.pos) * factor
    for mesh in child.meshes:
        mesh.scale = np.asarray(mesh.scale) * factor
    for geom in child.geoms:
        geom.pos = np.asarray(geom.pos) * factor
        if geom.type != mujoco.mjtGeom.mjGEOM_MESH:
            geom.size = np.asarray(geom.size) * factor
        if not math.isnan(geom.mass) and geom.mass > 0:
            geom.mass = geom.mass * factor**3


def absolutize_files(child: mujoco.MjSpec, xml_dir: Path) -> None:
    """Make mesh/texture paths absolute so they resolve once attached to another spec.

    build_scene rewrites them relative to the scene file when saving.
    """
    for collection, subdir in ((child.meshes, child.meshdir), (child.textures, child.texturedir)):
        for asset in collection:
            if asset.file:
                asset.file = str((xml_dir / subdir / asset.file).resolve())
    child.meshdir = ""
    child.texturedir = ""


def add_item(spec: mujoco.MjSpec, item: Item, layout: Layout, asset_root: Path = THIRD_PARTY_DIR) -> mujoco.MjsBody:
    """Attach `item` as a free body resting (with a small clearance) on its slot."""
    path = item_asset_path(item, asset_root)
    child = mujoco.MjSpec.from_file(str(path))
    top = child.worldbody.first_body()
    if top is None:
        raise AssetError(f"{path} has no body in its worldbody")
    scale_spec(child, item.effective_scale)

    if top.first_joint() is not None:
        raise AssetError(f"{path}: top body already has a joint; expected a rigid object")

    placement = next(s for s in layout.shelves if s.name == item.shelf)
    # Bounds in the shelf's local frame (shelf yaw = 0) so the fit check is yaw-independent.
    lo, hi = collision_bounds(child, item_quat(item, 0.0))
    check_fits_slot(item, layout, hi - lo)
    absolutize_files(child, path.parent)

    slot = slot_world_pos(layout.shelf, placement, item.level, item.slot)
    footprint_center = rotate_xy((lo[:2] + hi[:2]) / 2, placement.yaw_deg)
    pos = [slot[0] - footprint_center[0], slot[1] - footprint_center[1], slot[2] - lo[2] + ITEM_DROP_CLEARANCE]

    name = item_body_name(item)
    frame = spec.worldbody.add_frame(pos=pos, quat=list(item_quat(item, placement.yaw_deg)))
    body = frame.attach_body(top, f"{name}/", "")
    body.name = name
    body.add_freejoint(name=f"{name}_joint")
    return body


def check_fits_slot(item: Item, layout: Layout, extent: np.ndarray) -> None:
    """Raise AssetError if an item's collision box would overlap neighbours or the board above."""
    cfg = layout.shelf
    width = slot_width(cfg)
    usable_depth = cfg.depth - 2 * abs(SLOT_DEPTH_OFFSET)
    if extent[0] > width or extent[1] > usable_depth:
        raise AssetError(
            f"item {item.name!r} is too wide for its slot: footprint {extent[0]:.3f} x {extent[1]:.3f} m, "
            f"slot {width:.3f} x {usable_depth:.3f} m (reduce scale or slots_per_level)"
        )
    above = cfg.board_heights[item.level + 1] - cfg.board_thickness if item.level + 1 < len(cfg.board_heights) else cfg.height
    clearance = above - cfg.board_heights[item.level] - ITEM_DROP_CLEARANCE
    if extent[2] > clearance:
        raise AssetError(f"item {item.name!r} is too tall: {extent[2]:.3f} m, level clearance {clearance:.3f} m")


def add_items(spec: mujoco.MjSpec, items: tuple[Item, ...], layout: Layout, asset_root: Path = THIRD_PARTY_DIR) -> None:
    for item in items:
        add_item(spec, item, layout, asset_root)
