import math

import mujoco
import numpy as np
import pytest

from qc_warehouse.autonomy.navigation import (
    OccupancyGrid,
    PlanningError,
    advance_path,
    astar,
    build_occupancy_grid,
    line_is_free,
    pure_pursuit,
    shortcut_path,
    wrap_angle,
)


def _grid_with_wall() -> OccupancyGrid:
    """10 x 10 m grid at 0.1 m with a vertical wall at x=5 from y=0 to y=8."""
    occ = np.zeros((101, 101), dtype=bool)
    occ[50, :81] = True
    occ.setflags(write=False)
    return OccupancyGrid(origin=(0.0, 0.0), resolution=0.1, occupied=occ)


def test_astar_goes_around_wall():
    grid = _grid_with_wall()
    path = astar(grid, (2.0, 2.0), (8.0, 2.0))
    assert np.allclose(path[0], (2.0, 2.0)) and np.allclose(path[-1], (8.0, 2.0))
    assert max(p[1] for p in path) > 8.0  # had to pass above the wall
    assert all(grid.is_free_xy(p) for p in path)


def test_astar_rejects_blocked_goal():
    with pytest.raises(PlanningError, match="inside"):
        astar(_grid_with_wall(), (2.0, 2.0), (5.0, 2.0))


def test_astar_unreachable():
    occ = np.zeros((21, 21), dtype=bool)
    occ[10, :] = True
    occ.setflags(write=False)
    with pytest.raises(PlanningError, match="no path"):
        astar(OccupancyGrid((0.0, 0.0), 0.1, occ), (0.2, 0.2), (1.8, 0.2))


def test_shortcut_keeps_path_collision_free():
    grid = _grid_with_wall()
    path = astar(grid, (2.0, 2.0), (8.0, 2.0))
    short = shortcut_path(grid, path)
    assert len(short) < len(path)
    assert all(line_is_free(grid, a, b) for a, b in zip(short, short[1:]))


def test_grid_from_mujoco_box():
    spec = mujoco.MjSpec()
    spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[5, 5, 0.1])
    spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.5, 0.5, 0.5], pos=[0, 0, 0.5])
    free_body = spec.worldbody.add_body(pos=[2, 2, 0.5])
    free_body.add_freejoint()
    free_body.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.2, 0.2, 0.2])
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    grid = build_occupancy_grid(model, data, (-3, -3, 3, 3), inflation=0.3)
    assert not grid.is_free_xy((0.0, 0.0))
    assert not grid.is_free_xy((0.75, 0.0))  # within inflation
    assert grid.is_free_xy((1.0, 0.0))
    assert grid.is_free_xy((2.0, 2.0))  # free bodies are not static obstacles
    assert not grid.occupied.flags.writeable


def test_pure_pursuit_turns_in_place_when_facing_away():
    v, w = pure_pursuit((0.0, 0.0, 0.0), [np.array([0.0, 0.0]), np.array([-2.0, 0.0])])
    assert v == 0.0 and abs(w) > 0


def test_pure_pursuit_drives_forward_when_aligned():
    v, w = pure_pursuit((0.0, 0.0, 0.0), [np.array([1.0, 0.0]), np.array([3.0, 0.0])])
    assert v > 0.1 and abs(w) < 1e-6


def test_advance_path_drops_reached_waypoints():
    path = [np.array([0.0, 0.0]), np.array([1.0, 0.0]), np.array([2.0, 0.0])]
    assert len(advance_path(np.array([0.05, 0.0]), path)) == 2
    assert len(advance_path(np.array([2.0, 0.0]), path)) == 1


@pytest.mark.parametrize(("angle", "expected"), [(3 * math.pi, math.pi), (-math.pi / 2, -math.pi / 2), (2 * math.pi + 0.1, 0.1)])
def test_wrap_angle(angle, expected):
    assert math.isclose(abs(wrap_angle(angle)), abs(expected), abs_tol=1e-9)
