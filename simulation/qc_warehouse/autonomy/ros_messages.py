"""ROS 2 message dictionaries for rosbridge (JSON), and parsers for what slam_toolbox sends back.

Simulation time t (integer ns) is published as t + CLOCK_OFFSET: tf2 treats a
zero stamp as "latest", so the clock must never start at 0.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from qc_warehouse.autonomy.frames import BasePose, compose, inverse, quat_to_yaw, yaw_to_quat
from qc_warehouse.autonomy.lidar import LidarSpec

CLOCK_OFFSET_NS = 1_000_000_000_000  # 1000 s
NS_PER_S = 1_000_000_000
MAX_MAP_CELLS = 4_000_000  # 100 x 100 m at 5 cm; anything bigger is rejected


def validate_map(msg: Any) -> dict[str, Any]:
    """Check a nav_msgs/OccupancyGrid dict before anything allocates from it.

    Raises ValueError for anything malformed or implausibly large (rosbridge input is untrusted).
    """
    try:
        info = msg["info"]
        width, height, res = int(info["width"]), int(info["height"]), float(info["resolution"])
        parse_stamp(msg["header"]["stamp"])
        position, orientation = info["origin"]["position"], info["origin"]["orientation"]
        floats = [float(position["x"]), float(position["y"]), *(float(orientation[k]) for k in ("x", "y", "z", "w"))]
        data = msg["data"]
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"malformed map message ({type(exc).__name__}: {exc})") from exc
    if not (0 < width * height <= MAX_MAP_CELLS) or width <= 0 or height <= 0:
        raise ValueError(f"map size {width}x{height} out of range")
    if not (math.isfinite(res) and res > 0) or not all(math.isfinite(f) for f in floats):
        raise ValueError("map resolution/origin must be finite (resolution > 0)")
    if not isinstance(data, (list, tuple)) or len(data) != width * height:
        raise ValueError(f"map data has {len(data) if isinstance(data, (list, tuple)) else '?'} cells, expected {width}x{height}")
    return msg


def stamp(sim_ns: int) -> dict[str, int]:
    total = int(sim_ns) + CLOCK_OFFSET_NS
    return {"sec": total // NS_PER_S, "nanosec": total % NS_PER_S}


def parse_stamp(msg_stamp: dict[str, Any]) -> int:
    return int(msg_stamp["sec"]) * NS_PER_S + int(msg_stamp["nanosec"]) - CLOCK_OFFSET_NS


def clock_msg(sim_ns: int) -> dict[str, Any]:
    return {"clock": stamp(sim_ns)}


def tf_msg(sim_ns: int, parent: str, child: str, pose: BasePose) -> dict[str, Any]:
    return {
        "transforms": [
            {
                "header": {"stamp": stamp(sim_ns), "frame_id": parent},
                "child_frame_id": child,
                "transform": {"translation": {"x": float(pose.x), "y": float(pose.y), "z": 0.0}, "rotation": yaw_to_quat(pose.yaw)},
            }
        ]
    }


def scan_msg(sim_ns: int, ranges: np.ndarray, spec: LidarSpec, frame_id: str = "laser") -> dict[str, Any]:
    return {
        "header": {"stamp": stamp(sim_ns), "frame_id": frame_id},
        "angle_min": -math.pi,
        "angle_max": -math.pi + (spec.beams - 1) * spec.angle_increment,
        "angle_increment": spec.angle_increment,
        "time_increment": 0.0,
        "scan_time": spec.scan_period,
        "range_min": spec.range_min,
        "range_max": spec.range_max,
        "ranges": [round(float(r), 3) for r in ranges],
        "intensities": [],
    }


def parse_pose(msg: dict[str, Any]) -> tuple[int, BasePose]:
    """geometry_msgs/PoseWithCovarianceStamped -> (sim ns, map-frame base pose)."""
    pose = msg["pose"]["pose"]
    position = pose["position"]
    return parse_stamp(msg["header"]["stamp"]), BasePose(float(position["x"]), float(position["y"]), quat_to_yaw(pose["orientation"]))


def map_to_layout(
    msg: dict[str, Any], world_T_map: BasePose, origin: tuple[float, float], shape: tuple[int, int], resolution: float
) -> np.ndarray:
    """Resample a nav_msgs/OccupancyGrid onto a layout-frame grid ([nx, ny], cell centres at origin + i*resolution).

    Each layout cell takes the *maximum* over the ROS cells it covers (unknown < free < occupied),
    so thin walls survive downsampling. Returns int8 values (-1 unknown, 0..100); outside = unknown.
    """
    validate_map(msg)
    info = msg["info"]
    width, height, res = int(info["width"]), int(info["height"]), float(info["resolution"])
    try:
        data = np.asarray(msg["data"], dtype=np.int8)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"map cells must be int8 occupancy values ({exc})") from exc
    cells = data.reshape(height, width)  # row-major: index = x + y * width
    position, orientation = info["origin"]["position"], info["origin"]["orientation"]
    map_T_rosorigin = BasePose(float(position["x"]), float(position["y"]), quat_to_yaw(orientation))
    rosorigin_T_world = inverse(compose(world_T_map, map_T_rosorigin))

    xs = origin[0] + np.arange(shape[0]) * resolution
    ys = origin[1] + np.arange(shape[1]) * resolution
    wx, wy = np.meshgrid(xs, ys, indexing="ij")
    c, s = math.cos(rosorigin_T_world.yaw), math.sin(rosorigin_T_world.yaw)
    # Sample points no further apart than one ROS cell across each layout cell's footprint.
    steps = max(1, math.ceil(resolution / res)) + 1
    offsets = (np.arange(steps) / (steps - 1) - 0.5) * resolution * (1 - 1e-6)
    out = np.full(shape, -1, dtype=np.int8)
    for dx in offsets:
        for dy in offsets:
            px, py = wx + dx, wy + dy
            mx = c * px - s * py + rosorigin_T_world.x
            my = s * px + c * py + rosorigin_T_world.y
            ix, iy = np.floor(mx / res).astype(int), np.floor(my / res).astype(int)
            inside = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
            sample = np.full(shape, -1, dtype=np.int8)
            sample[inside] = cells[iy[inside], ix[inside]]
            np.maximum(out, sample, out=out)
    return out
