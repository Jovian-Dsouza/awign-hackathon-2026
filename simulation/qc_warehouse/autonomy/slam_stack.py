"""Assemble the SLAM localization + mapping pipeline around a robot and a SimRunner."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Any

import numpy as np

from qc_warehouse.autonomy.frames import BasePose
from qc_warehouse.autonomy.lidar import Lidar2D, LidarSpec
from qc_warehouse.autonomy.localization import LocalizationMonitor, SlamLocalizer
from qc_warehouse.autonomy.mapping import SlamMap
from qc_warehouse.autonomy.odometry import OdometryNoise, WheelGyroOdometry
from qc_warehouse.autonomy.runner import SimRunner
from qc_warehouse.autonomy.slam_bridge import SlamBackend
from qc_warehouse.autonomy.slam_frontend import SlamFrontend
from qc_warehouse.autonomy.stretch import WHEEL_RADIUS, Stretch
from qc_warehouse.config import Layout
from qc_warehouse.robot import ROBOT_PREFIX


def dock_pose(layout: Layout) -> BasePose:
    """Where the map frame sits in the layout: the robot starts every run on its dock."""
    dock = layout.robot_dock
    return BasePose(dock.pos[0], dock.pos[1], math.radians(dock.yaw_deg))


def room_bounds(layout: Layout) -> tuple[float, float, float, float]:
    half_x, half_y = layout.room.size[0] / 2, layout.room.size[1] / 2
    return (-half_x, -half_y, half_x, half_y)


@dataclass
class SlamStack:
    backend: SlamBackend
    frontend: SlamFrontend
    localizer: SlamLocalizer
    maps: SlamMap
    monitor: LocalizationMonitor

    def report(self) -> dict[str, Any]:
        return {
            "mode": "slam",
            "error_vs_ground_truth": asdict(self.monitor.stats()),
            "scans": asdict(self.frontend.stats),
            "corrections": self.localizer.corrections,
        }

    def close(self) -> None:
        self.backend.close()


def attach_slam(
    robot: Stretch,
    runner: SimRunner,
    layout: Layout,
    backend: SlamBackend,
    inflation: float,
    seed: int = 0,
    odometry_noise: OdometryNoise = OdometryNoise(),
    lidar_spec: LidarSpec = LidarSpec(),
) -> SlamStack:
    """Switch `robot` from ground truth to SLAM: odometry + lidar feed `backend`, the robot
    drives on the fused estimate, and planning uses the live map. Sends the first scan."""
    rng = np.random.default_rng(seed)
    world_T_map = dock_pose(layout)
    odometry = WheelGyroOdometry(WHEEL_RADIUS, odometry_noise, rng)
    localizer = SlamLocalizer(odometry, world_T_map)
    lidar = Lidar2D(robot.model, ROBOT_PREFIX + "laser", ROBOT_PREFIX + "base_link", lidar_spec, rng)
    frontend = SlamFrontend(robot, backend, lidar, odometry, localizer)
    frontend.start()  # first scan; raises before anything is attached if SLAM is unreachable
    robot.use_localizer(localizer)
    monitor = LocalizationMonitor(robot.base_pose, robot.true_base_pose)
    runner.substep_hooks.append(frontend.gyro)
    runner.post_step.extend([frontend, monitor])
    maps = SlamMap(backend, lambda: frontend.last_processed_ns, world_T_map, room_bounds(layout), inflation)
    return SlamStack(backend, frontend, localizer, maps, monitor)
