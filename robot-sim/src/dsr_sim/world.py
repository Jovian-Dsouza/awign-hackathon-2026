"""Facts about the demo table (shelf, tray, dock), run parameters and their validation, and simple
rover path / footprint checks. Pure geometry: no simulation state needed.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, hypot, sin

import numpy as np

from dsr_sim.config import load_inventory, load_layout
from dsr_sim.constants import DEFAULT_INVENTORY, DEFAULT_LAYOUT
from dsr_sim.shelf import slot_world_pos

LAYOUT = load_layout(DEFAULT_LAYOUT)
INVENTORY = load_inventory(DEFAULT_INVENTORY, LAYOUT)
ITEM = "happydent_wave"  # the misplaced item of the restock run: the pot the real ACT demos were recorded with
ITEM_RADIUS = 0.0215  # half the pot's wider side, m

ROVER_HALF = (0.165, 0.125)  # half length (along heading) and half width, including the wheels
ROVER_CIRCLE = 0.19  # circle that always covers the rover; used for path clearance
SHELF_MARGIN = 0.05  # keep-out around the shelf / tray for the item (room for the jaws)
TABLE_INSET_ITEM = 0.07
CORRIDOR_Y = -0.28  # free lane along the near edge of the table (where the dock is)
LEVELS = len(LAYOUT.shelf.board_heights)
SLOTS = LAYOUT.shelf.slots_per_level


@dataclass(frozen=True)
class Rect:
    x0: float
    y0: float
    x1: float
    y1: float

    def inflate(self, m: float) -> "Rect":
        return Rect(self.x0 - m, self.y0 - m, self.x1 + m, self.y1 + m)

    def contains(self, x: float, y: float) -> bool:
        return self.x0 <= x <= self.x1 and self.y0 <= y <= self.y1

    def distance(self, x: float, y: float) -> float:
        dx = max(self.x0 - x, 0.0, x - self.x1)
        dy = max(self.y0 - y, 0.0, y - self.y1)
        return hypot(dx, dy)

    def corners(self) -> np.ndarray:
        return np.array([[self.x0, self.y0], [self.x1, self.y0], [self.x1, self.y1], [self.x0, self.y1]])


def _centered(pos, size_x: float, size_y: float) -> Rect:
    return Rect(pos[0] - size_x / 2, pos[1] - size_y / 2, pos[0] + size_x / 2, pos[1] + size_y / 2)


SHELF_PLACEMENT = LAYOUT.shelves[0]
TABLE = _centered((0.0, 0.0), *LAYOUT.table.size)
SHELF = _centered(SHELF_PLACEMENT.pos, LAYOUT.shelf.width, LAYOUT.shelf.depth)
TRAY = _centered(LAYOUT.tray.pos, LAYOUT.tray.size[0], LAYOUT.tray.size[1])
DOCK = (LAYOUT.robot_dock.pos[0], LAYOUT.robot_dock.pos[1], np.deg2rad(LAYOUT.robot_dock.yaw_deg))
OBSTACLES = (SHELF, TRAY)


def rack_pos(level: int, slot: int) -> np.ndarray:
    """World position of a rack's slot reference point (x, y, top of the board)."""
    return slot_world_pos(LAYOUT.shelf, SHELF_PLACEMENT, level, slot)


def home_rack(name: str = ITEM) -> tuple[int, int]:
    for item in INVENTORY:
        if item.name == name:
            return item.level, item.slot
    raise KeyError(name)


def item_at(rack: tuple[int, int]) -> str | None:
    """Name of the item that sits in the rack, or None if it is empty."""
    for item in INVENTORY:
        if (item.level, item.slot) == tuple(rack):
            return item.name
    return None


def target_item(scenario: str, p: "Params") -> str:
    """The item a run handles: whatever sits in the chosen rack, or the Happydent pot for the restock run."""
    if scenario == "shelf_to_tray":
        return item_at(p.rack) or ITEM
    return ITEM


def occupied_racks(excluding: str = ITEM) -> dict[tuple[int, int], str]:
    return {(i.level, i.slot): i.name for i in INVENTORY if i.name != excluding}


# ------------------------------------------------------------------ rover footprint & paths
def rover_polygon(x: float, y: float, yaw: float) -> np.ndarray:
    c, s = cos(yaw), sin(yaw)
    hl, hw = ROVER_HALF
    local = np.array([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]])
    return local @ np.array([[c, s], [-s, c]]) + np.array([x, y])


def _axes(poly: np.ndarray) -> list[np.ndarray]:
    out = []
    for i in range(len(poly)):
        e = poly[(i + 1) % len(poly)] - poly[i]
        n = np.array([-e[1], e[0]])
        out.append(n / (np.linalg.norm(n) + 1e-12))
    return out


def polygons_overlap(a: np.ndarray, b: np.ndarray) -> bool:
    for axis in _axes(a) + _axes(b):
        pa, pb = a @ axis, b @ axis
        if pa.max() < pb.min() or pb.max() < pa.min():
            return False
    return True


def rover_pose_ok(x: float, y: float, yaw: float, margin: float = 0.02) -> bool:
    """Rover footprint fully on the table and clear of the shelf and tray."""
    poly = rover_polygon(x, y, yaw)
    inner = TABLE.inflate(-margin)
    if not all(inner.contains(px, py) for px, py in poly):
        return False
    return not any(polygons_overlap(poly, o.inflate(0.03).corners()) for o in OBSTACLES)


def segment_clear(p0, p1, radius: float = ROVER_CIRCLE, leaving: float = 0.0) -> bool:
    """True if a rover of `radius` can travel p0 -> p1 without touching the shelf/tray or leaving the table.

    `leaving`: the first stretch of this length is not checked against obstacles (the rover may already be
    standing next to one, e.g. just after delivering to the tray).
    """
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    length = float(np.linalg.norm(p1 - p0))
    n = max(2, int(length / 0.03))
    for t in np.linspace(0.0, 1.0, n):
        x, y = p0 + (p1 - p0) * t
        if t * length < leaving:
            continue
        if any(o.distance(x, y) < radius for o in OBSTACLES):
            return False
        if not TABLE.inflate(-0.05).contains(x, y):
            return False
    return True


def _passes_clear(p0, p1, avoid) -> bool:
    """True if the segment p0 -> p1 stays at least avoid[2] away from the point (avoid[0], avoid[1])."""
    p0, p1, c = np.asarray(p0, float), np.asarray(p1, float), np.asarray(avoid[:2], float)
    d = p1 - p0
    t = 0.0 if not d.any() else float(np.clip(np.dot(c - p0, d) / np.dot(d, d), 0.0, 1.0))
    return float(np.linalg.norm(p0 + t * d - c)) >= avoid[2]


def find_route(start_xy, goal_xy, leaving: float = 0.30, avoid: tuple[float, float, float] | None = None) -> list[tuple[float, float]] | None:
    """Waypoints (excluding the start) from start to goal: straight if clear, else via a lane along the table.

    `avoid` = (x, y, radius): a tall item standing on the table that the rover must not run over (it drives
    over short ones, but a deck-high one would be knocked over). With it, parallel lanes are tried too."""
    clear = lambda a, b, **kw: segment_clear(a, b, **kw) and (avoid is None or _passes_clear(a, b, avoid))  # noqa: E731
    if clear(start_xy, goal_xy, leaving=leaving):
        return [tuple(goal_xy)]
    lanes = (CORRIDOR_Y,) if avoid is None else (CORRIDOR_Y, -0.17, -0.07, 0.03)
    for lane in lanes:
        a = (start_xy[0], lane)
        b = (goal_xy[0], lane)
        if clear(start_xy, a, leaving=leaving) and clear(a, b) and clear(b, goal_xy):
            return [a, b, tuple(goal_xy)]
    return None


# ------------------------------------------------------------------ run parameters
class ParamError(ValueError):
    """The requested parameters cannot be run; the message is meant for the user."""


@dataclass(frozen=True)
class Params:
    rack: tuple[int, int] = (0, 2)  # shelf_to_tray: (level, slot) the item is picked from; level 0 = bottom
    item_xy: tuple[float, float] = (0.30, -0.02)  # restock_misplaced: where the item was left on the table
    dest_rack: tuple[int, int] | None = None  # restock_misplaced: rack to put it in (None = its home rack)

    @classmethod
    def from_dict(cls, raw: dict | None) -> "Params":
        raw = raw or {}
        kw: dict = {}
        if raw.get("rack") is not None:
            kw["rack"] = tuple(int(v) for v in raw["rack"])
        if raw.get("item_xy") is not None:
            kw["item_xy"] = tuple(float(v) for v in raw["item_xy"])
        if raw.get("dest_rack") is not None:
            kw["dest_rack"] = tuple(int(v) for v in raw["dest_rack"])
        return cls(**kw)

    def as_dict(self) -> dict:
        return {"rack": list(self.rack), "item_xy": list(self.item_xy), "dest_rack": list(self.dest_rack) if self.dest_rack else None}


def _check_rack(rack, what: str) -> None:
    level, slot = rack
    if not (0 <= level < LEVELS and 0 <= slot < SLOTS):
        raise ParamError(f"{what}: there is no rack at level {level}, slot {slot} (levels 0-{LEVELS - 1}, slots 0-{SLOTS - 1})")


def item_position_problem(x: float, y: float) -> str | None:
    """Why an item cannot be left at (x, y) on the table, or None if the rover can fetch it."""
    if not TABLE.inflate(-TABLE_INSET_ITEM).contains(x, y):
        return "too close to the edge of the table"
    for name, rect in (("shelf", SHELF), ("tray", TRAY)):
        if rect.inflate(SHELF_MARGIN).contains(x, y):
            return f"too close to the {name}"
    d = hypot(x - DOCK[0], y - DOCK[1])
    if d < 0.18:
        return "on top of the rover's start spot"
    return None


def validate(scenario: str, p: Params) -> None:
    if scenario == "shelf_to_tray":
        _check_rack(p.rack, "pick-from rack")
        if item_at(p.rack) is None:
            raise ParamError(f"pick-from rack: level {p.rack[0]}, slot {p.rack[1]} is empty, there is nothing to pick")
    elif scenario == "restock_misplaced":
        problem = item_position_problem(*p.item_xy)
        if problem:
            raise ParamError(f"item position ({p.item_xy[0]:.2f}, {p.item_xy[1]:.2f}): {problem}")
        if p.dest_rack is not None:
            _check_rack(p.dest_rack, "destination rack")
            taken = occupied_racks().get(tuple(p.dest_rack))
            if taken:
                raise ParamError(f"destination rack level {p.dest_rack[0]}, slot {p.dest_rack[1]} already holds the {taken}")


def layout_info() -> dict:
    """Geometry for the page's table map."""
    r = lambda rect: [rect.x0, rect.y0, rect.x1, rect.y1]  # noqa: E731
    return {
        "table": r(TABLE),
        "shelf": r(SHELF),
        "tray": r(TRAY),
        "dock": [DOCK[0], DOCK[1]],
        "keep_out": [r(SHELF.inflate(SHELF_MARGIN)), r(TRAY.inflate(SHELF_MARGIN))],
        "inset": TABLE_INSET_ITEM,
        "levels": LEVELS,
        "slots": SLOTS,
        "occupied": [[lv, sl, nm] for (lv, sl), nm in occupied_racks().items()],
        "stock": [[i.level, i.slot, i.name] for i in INVENTORY],
        "home_rack": list(home_rack()),
        "defaults": Params().as_dict(),
    }
