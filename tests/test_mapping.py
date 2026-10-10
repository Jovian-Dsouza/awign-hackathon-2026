import numpy as np
import pytest

from qc_warehouse.autonomy import ros_messages as rm
from qc_warehouse.autonomy.frames import BasePose
from qc_warehouse.autonomy.mapping import OCCUPIED_THRESHOLD, SlamMap, StaticMap, grid_from_ros_map
from qc_warehouse.autonomy.navigation import OccupancyGrid
from qc_warehouse.autonomy.slam_bridge import FakeSlamBackend

BOUNDS = (-1.0, -1.0, 1.0, 1.0)


def _grid_with_obstacle() -> OccupancyGrid:
    occupied = np.zeros((41, 41), dtype=bool)
    occupied[30, 20] = True  # world (0.5, 0.0)
    return OccupancyGrid(origin=(-1.0, -1.0), resolution=0.05, occupied=occupied)


def test_no_map_yet_means_all_unknown_and_free():
    grid, values = grid_from_ros_map(None, BasePose(0, 0, 0), BOUNDS, inflation=0.1)
    assert (values == -1).all()
    assert not grid.occupied[2:-2, 2:-2].any()  # only the map border (beyond = blocked) is inflated
    assert not grid.occupied.flags.writeable


@pytest.mark.parametrize("dock", [BasePose(0, 0, 0), BasePose(-0.4, 0.3, 1.2)])
def test_fake_map_round_trips_through_ros_format(dock):
    backend = FakeSlamBackend(lambda: BasePose(0, 0, 0), dock, _grid_with_obstacle())
    grid, values = grid_from_ros_map(backend.latest_map(0, 0.0), dock, BOUNDS, inflation=0.0)
    assert values[30, 20] >= OCCUPIED_THRESHOLD and grid.is_free_xy((0.0, 0.0))
    assert not grid.is_free_xy((0.5, 0.0))
    inflated, _ = grid_from_ros_map(backend.latest_map(0, 0.0), dock, BOUNDS, inflation=0.1)
    assert not inflated.is_free_xy((0.4, 0.0))


def test_slam_map_refreshes_only_for_newer_scans():
    backend = FakeSlamBackend(lambda: BasePose(0, 0, 0), BasePose(0, 0, 0), _grid_with_obstacle())
    latest = {"ns": -1}
    maps = SlamMap(backend, lambda: latest["ns"], BasePose(0, 0, 0), BOUNDS, inflation=0.0)
    assert maps.refresh().is_free_xy((0.5, 0.0))  # nothing scanned yet
    latest["ns"] = 0
    assert not maps.refresh().is_free_xy((0.5, 0.0))
    assert maps.current() is maps.refresh()        # same stamp -> cached


def test_slam_map_ignores_malformed_messages():
    class Broken:
        def latest_map(self, min_stamp_ns, timeout):
            return {"header": {"stamp": rm.stamp(5)}, "info": {"width": 2}}

    maps = SlamMap(Broken(), lambda: 5, BasePose(0, 0, 0), BOUNDS, inflation=0.0)
    assert not maps.refresh().occupied.any()


def test_static_map():
    grid = _grid_with_obstacle()
    maps = StaticMap(grid)
    assert maps.current() is grid and maps.refresh() is grid
