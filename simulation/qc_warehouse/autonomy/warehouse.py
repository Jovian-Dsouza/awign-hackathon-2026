"""What the robot knows about the warehouse (the 'WMS' view): slots, shelf faces, totes.

Everything the robot plans with comes from the layout config (the site plan), not
from the simulator. Item *positions* are not known up front either: the slot only
tells the robot where to go and look, and perception finds the actual item.
`item_position`/`item_in_tote` read simulator ground truth and exist only to grade
the outcome.
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from qc_warehouse.autonomy.poses import outward_normal
from qc_warehouse.config import Item, Layout
from qc_warehouse.items import item_body_name
from qc_warehouse.shelf import slot_world_pos
from qc_warehouse.station import TOTE_WALL, tote_world_pos

SHELF_FRONT_NORMAL = (0.0, -1.0)    # shelf local frame: open front faces -y
STATION_ROBOT_NORMAL = (0.0, 1.0)   # station local frame: robot works from +y


@dataclass(frozen=True)
class Tote:
    index: int
    center: np.ndarray   # xy
    floor_z: float
    rim_z: float
    half_inner: np.ndarray  # inner half extents (x, y)


class Warehouse:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, layout: Layout, inventory: tuple[Item, ...]):
        self.model, self.data, self.layout = model, data, layout
        self.inventory = {item.name: item for item in inventory}
        self.item_bodies = {name: model.body(item_body_name(item)).id for name, item in self.inventory.items()}
        self._shelves = {s.name: s for s in layout.shelves}
        self.station_normal = outward_normal(layout.station.yaw_deg, STATION_ROBOT_NORMAL)

    def item(self, name: str) -> Item:
        if name not in self.inventory:
            raise KeyError(f"{name!r} is not stocked")
        return self.inventory[name]

    def slot_position(self, item: Item) -> np.ndarray:
        return slot_world_pos(self.layout.shelf, self._shelves[item.shelf], item.level, item.slot)

    def shelf_normal(self, item: Item) -> np.ndarray:
        return outward_normal(self._shelves[item.shelf].yaw_deg, SHELF_FRONT_NORMAL)

    def board_z(self, item: Item) -> float:
        return self.layout.shelf.board_heights[item.level]

    def ceiling_z(self, item: Item) -> float:
        """Underside of the board above the item's level (or the rack top)."""
        cfg = self.layout.shelf
        if item.level + 1 < len(cfg.board_heights):
            return cfg.board_heights[item.level + 1] - cfg.board_thickness
        return cfg.height

    def tote(self, index: int) -> Tote:
        cfg = self.layout.station
        site = tote_world_pos(cfg, index)
        half_inner = np.array([cfg.tote_size[0] / 2 - TOTE_WALL, cfg.tote_size[1] / 2 - TOTE_WALL])
        floor_z = float(site[2])
        return Tote(index, site[:2].copy(), floor_z, floor_z - TOTE_WALL + cfg.tote_size[2], half_inner)

    def item_position(self, name: str) -> np.ndarray:
        """Ground truth (grading/reporting only)."""
        return self.data.xpos[self.item_bodies[name]].copy()

    def item_in_tote(self, name: str, tote: Tote) -> bool:
        """Ground-truth check used to *verify* a placement (not to plan it)."""
        pos = self.data.xpos[self.item_bodies[name]]
        inside_xy = np.all(np.abs(pos[:2] - tote.center) <= tote.half_inner)
        return bool(inside_xy and tote.floor_z - 0.02 <= pos[2] <= tote.rim_z + 0.05)
