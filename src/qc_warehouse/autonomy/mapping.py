"""Where the planner's occupancy grid comes from.

StaticMap: a fixed grid (e.g. rasterised from the scene; ground-truth mode and tests).
SlamMap:   the live slam_toolbox map, resampled into the layout frame and inflated.
Unknown space is treated as free (optimistic exploration); paths are re-checked
as the map fills in.
"""

from __future__ import annotations

import logging
from collections.abc import Callable
from typing import Any, Protocol

import numpy as np

from qc_warehouse.autonomy import ros_messages as rm
from qc_warehouse.autonomy.frames import BasePose
from qc_warehouse.autonomy.navigation import GRID_RESOLUTION, OccupancyGrid, inflate

log = logging.getLogger(__name__)

OCCUPIED_THRESHOLD = 65   # ROS map_server convention: >= 65 % is an obstacle
MAP_WAIT = 3.0            # wall seconds to wait for a map that includes the latest scan


class MapProvider(Protocol):
    def current(self) -> OccupancyGrid: ...

    def refresh(self) -> OccupancyGrid:
        """Pull the newest map (may block briefly) and return it."""


class StaticMap:
    def __init__(self, grid: OccupancyGrid):
        self._grid = grid

    def current(self) -> OccupancyGrid:
        return self._grid

    def refresh(self) -> OccupancyGrid:
        return self._grid


class MapSource(Protocol):
    def latest_map(self, min_stamp_ns: int, timeout: float) -> dict[str, Any] | None: ...


def grid_from_ros_map(
    msg: dict[str, Any] | None,
    world_T_map: BasePose,
    bounds: tuple[float, float, float, float],
    inflation: float,
    resolution: float = GRID_RESOLUTION,
) -> tuple[OccupancyGrid, np.ndarray]:
    """Layout-frame planning grid (inflated) and the raw resampled occupancy values."""
    xmin, ymin, xmax, ymax = bounds
    shape = (int(round((xmax - xmin) / resolution)) + 1, int(round((ymax - ymin) / resolution)) + 1)
    if msg is None:
        values = np.full(shape, -1, dtype=np.int8)
    else:
        values = rm.map_to_layout(msg, world_T_map, (xmin, ymin), shape, resolution)
    occupied = inflate(values >= OCCUPIED_THRESHOLD, int(np.ceil(inflation / resolution)))
    occupied.setflags(write=False)
    values.setflags(write=False)
    return OccupancyGrid(origin=(xmin, ymin), resolution=resolution, occupied=occupied), values


class SlamMap:
    def __init__(
        self,
        source: MapSource,
        latest_scan_ns: Callable[[], int],
        world_T_map: BasePose,
        bounds: tuple[float, float, float, float],
        inflation: float,
    ):
        self._source, self._latest_scan_ns = source, latest_scan_ns
        self._world_T_map, self._bounds, self._inflation = world_T_map, bounds, inflation
        self._grid, self.values = grid_from_ros_map(None, world_T_map, bounds, inflation)
        self._stamp_ns = -1

    def current(self) -> OccupancyGrid:
        return self._grid

    def refresh(self) -> OccupancyGrid:
        wanted = self._latest_scan_ns()
        if wanted <= self._stamp_ns:
            return self._grid
        try:
            msg = self._source.latest_map(wanted, MAP_WAIT)
            if msg is None:
                log.warning("no SLAM map received yet; planning on an empty map")
                return self._grid
            stamp = rm.parse_stamp(msg["header"]["stamp"])
            self._grid, self.values = grid_from_ros_map(msg, self._world_T_map, self._bounds, self._inflation)
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            log.warning("ignoring malformed SLAM map: %s", exc)
            return self._grid
        if stamp < wanted:
            log.debug("SLAM map lags the latest scan by %.2fs", (wanted - stamp) * 1e-9)
        self._stamp_ns = stamp
        return self._grid
