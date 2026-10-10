"""2-D navigation: occupancy grid from the MuJoCo scene, A* planning, pure pursuit.

Everything here is pure (no simulation stepping) so it can be unit-tested.
"""

from __future__ import annotations

import heapq
import math
from collections.abc import Sequence
from dataclasses import dataclass

import mujoco
import numpy as np

from qc_warehouse.autonomy.frames import wrap_angle

GRID_RESOLUTION = 0.05      # m per cell
OBSTACLE_MIN_Z = 0.02       # ignore floor tape etc.
OBSTACLE_MAX_Z = 2.0        # robot height incl. mast
_SQRT2 = math.sqrt(2.0)
_NEIGHBOURS = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0), (1, 1, _SQRT2), (1, -1, _SQRT2), (-1, 1, _SQRT2), (-1, -1, _SQRT2))
_BOX_CORNERS = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)


class PlanningError(RuntimeError):
    """No collision-free path exists."""


@dataclass(frozen=True)
class OccupancyGrid:
    origin: tuple[float, float]  # world xy of cell (0, 0) centre
    resolution: float
    occupied: np.ndarray         # bool [nx, ny], read-only

    def to_cell(self, xy: Sequence[float]) -> tuple[int, int]:
        return (round((xy[0] - self.origin[0]) / self.resolution), round((xy[1] - self.origin[1]) / self.resolution))

    def to_world(self, cell: tuple[int, int]) -> np.ndarray:
        return np.array([self.origin[0] + cell[0] * self.resolution, self.origin[1] + cell[1] * self.resolution])

    def in_bounds(self, cell: tuple[int, int]) -> bool:
        return 0 <= cell[0] < self.occupied.shape[0] and 0 <= cell[1] < self.occupied.shape[1]

    def is_free(self, cell: tuple[int, int]) -> bool:
        return self.in_bounds(cell) and not self.occupied[cell]

    def is_free_xy(self, xy: Sequence[float]) -> bool:
        return self.is_free(self.to_cell(xy))


def _static_obstacle_footprints(model: mujoco.MjModel, data: mujoco.MjData) -> list[np.ndarray]:
    """XY corner sets of colliding geoms rigidly attached to the world."""
    footprints = []
    for g in range(model.ngeom):
        if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE:
            continue
        if model.body_weldid[model.geom_bodyid[g]] != 0:
            continue  # moving bodies (robot, items) are not static obstacles
        if not (model.geom_contype[g] or model.geom_conaffinity[g]):
            continue
        center, half = model.geom_aabb[g, :3], model.geom_aabb[g, 3:]
        corners = (_BOX_CORNERS * half + center) @ data.geom_xmat[g].reshape(3, 3).T + data.geom_xpos[g]
        if corners[:, 2].max() < OBSTACLE_MIN_Z or corners[:, 2].min() > OBSTACLE_MAX_Z:
            continue
        footprints.append(corners[:, :2])
    return footprints


def inflate(occupied: np.ndarray, radius_cells: int) -> np.ndarray:
    nx, ny = occupied.shape
    padded = np.pad(occupied, radius_cells, constant_values=True)
    out = np.zeros_like(occupied)
    for dx in range(-radius_cells, radius_cells + 1):
        for dy in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy <= radius_cells * radius_cells:
                out |= padded[radius_cells + dx : radius_cells + dx + nx, radius_cells + dy : radius_cells + dy + ny]
    return out


def build_occupancy_grid(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    bounds: tuple[float, float, float, float],
    inflation: float,
    resolution: float = GRID_RESOLUTION,
) -> OccupancyGrid:
    """Rasterise static obstacles inside bounds (xmin, ymin, xmax, ymax), inflated by `inflation` m."""
    xmin, ymin, xmax, ymax = bounds
    nx, ny = int(round((xmax - xmin) / resolution)) + 1, int(round((ymax - ymin) / resolution)) + 1
    xs = xmin + np.arange(nx) * resolution
    ys = ymin + np.arange(ny) * resolution
    occupied = np.zeros((nx, ny), dtype=bool)
    for corners in _static_obstacle_footprints(model, data):
        lo, hi = corners.min(axis=0), corners.max(axis=0)
        occupied |= ((xs >= lo[0] - resolution / 2) & (xs <= hi[0] + resolution / 2))[:, None] & (
            (ys >= lo[1] - resolution / 2) & (ys <= hi[1] + resolution / 2)
        )[None, :]
    inflated = inflate(occupied, int(math.ceil(inflation / resolution)))
    inflated.setflags(write=False)
    return OccupancyGrid(origin=(xmin, ymin), resolution=resolution, occupied=inflated)


def _nearest_free(grid: OccupancyGrid, cell: tuple[int, int], max_radius: int = 20) -> tuple[int, int] | None:
    if grid.is_free(cell):
        return cell
    for r in range(1, max_radius + 1):
        ring = [(cell[0] + dx, cell[1] + dy) for dx in range(-r, r + 1) for dy in range(-r, r + 1) if max(abs(dx), abs(dy)) == r]
        free = [c for c in ring if grid.is_free(c)]
        if free:
            return min(free, key=lambda c: (c[0] - cell[0]) ** 2 + (c[1] - cell[1]) ** 2)
    return None


def astar(grid: OccupancyGrid, start_xy: Sequence[float], goal_xy: Sequence[float]) -> list[np.ndarray]:
    """8-connected A* from start to goal. A start inside an obstacle snaps to the nearest free cell."""
    start = _nearest_free(grid, grid.to_cell(start_xy))
    goal = grid.to_cell(goal_xy)
    if start is None:
        raise PlanningError(f"no free cell near start {tuple(np.round(start_xy, 2))}")
    if not grid.is_free(goal):
        raise PlanningError(f"goal {tuple(np.round(goal_xy, 2))} is inside an (inflated) obstacle")

    def h(c: tuple[int, int]) -> float:
        dx, dy = abs(c[0] - goal[0]), abs(c[1] - goal[1])
        return (dx + dy) + (_SQRT2 - 2) * min(dx, dy)

    open_heap = [(h(start), 0.0, start)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    cost = {start: 0.0}
    while open_heap:
        _, g, cell = heapq.heappop(open_heap)
        if cell == goal:
            path = [cell]
            while path[-1] in came_from:
                path.append(came_from[path[-1]])
            return [grid.to_world(c) for c in reversed(path)]
        if g > cost.get(cell, math.inf):
            continue
        for dx, dy, step in _NEIGHBOURS:
            nxt = (cell[0] + dx, cell[1] + dy)
            if not grid.is_free(nxt):
                continue
            if dx and dy and not (grid.is_free((cell[0] + dx, cell[1])) and grid.is_free((cell[0], cell[1] + dy))):
                continue  # no corner cutting on diagonal moves
            new_cost = g + step
            if new_cost < cost.get(nxt, math.inf):
                cost[nxt] = new_cost
                came_from[nxt] = cell
                heapq.heappush(open_heap, (new_cost + h(nxt), new_cost, nxt))
    raise PlanningError(f"no path from {tuple(np.round(start_xy, 2))} to {tuple(np.round(goal_xy, 2))}")


def line_is_free(grid: OccupancyGrid, a: np.ndarray, b: np.ndarray) -> bool:
    n = max(2, int(np.linalg.norm(b - a) / (grid.resolution / 2)) + 1)
    return all(grid.is_free_xy(a + (b - a) * t) for t in np.linspace(0.0, 1.0, n))


def shortcut_path(grid: OccupancyGrid, path: list[np.ndarray]) -> list[np.ndarray]:
    """Greedy line-of-sight smoothing: keep only the waypoints needed to stay collision-free."""
    if len(path) <= 2:
        return list(path)
    result = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not line_is_free(grid, path[i], path[j]):
            j -= 1
        result.append(path[j])
        i = j
    return result


@dataclass(frozen=True)
class PursuitParams:
    lookahead: float = 0.35
    max_speed: float = 0.22       # m/s
    max_turn: float = 1.0         # rad/s
    turn_in_place: float = 0.6    # rad heading error above which the robot only rotates
    heading_gain: float = 2.0


def pure_pursuit(pose: tuple[float, float, float], path: Sequence[np.ndarray], params: PursuitParams = PursuitParams()) -> tuple[float, float]:
    """Return (v, w) steering toward the lookahead point on the (remaining) path."""
    x, y, yaw = pose
    position = np.array([x, y])
    # Lookahead target: first waypoint farther than lookahead, else the final waypoint.
    target = path[-1]
    for waypoint in path:
        if np.linalg.norm(waypoint - position) >= params.lookahead:
            target = waypoint
            break
    heading_error = wrap_angle(math.atan2(target[1] - y, target[0] - x) - yaw)
    w = float(np.clip(params.heading_gain * heading_error, -params.max_turn, params.max_turn))
    if abs(heading_error) > params.turn_in_place:
        return 0.0, w
    distance_to_goal = float(np.linalg.norm(path[-1] - position))
    v = min(params.max_speed, 0.8 * distance_to_goal + 0.03) * math.cos(heading_error)
    return v, w


def advance_path(position: np.ndarray, path: list[np.ndarray], reached: float = 0.25) -> list[np.ndarray]:
    """Drop every waypoint up to the farthest one already reached (keeps at least the goal)."""
    reached_idx = [i for i, p in enumerate(path) if np.linalg.norm(p - position) < reached]
    if not reached_idx:
        return list(path)
    return list(path[min(reached_idx[-1] + 1, len(path) - 1) :])
