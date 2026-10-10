"""The SLAM front end driven by the real robot model, against an offline fake slam_toolbox."""

import math

import mujoco
import numpy as np
import pytest

from qc_warehouse.autonomy.navigation import build_occupancy_grid
from qc_warehouse.autonomy.runner import SimRunner
from qc_warehouse.autonomy.slam_bridge import FakeSlamBackend, SlamUnavailableError, RosbridgeSlamBackend
from qc_warehouse.autonomy.slam_stack import attach_slam, dock_pose, room_bounds
from qc_warehouse.autonomy.stretch import Stretch
from qc_warehouse.config import load_layout
from qc_warehouse.constants import DEFAULT_LAYOUT, DEFAULT_SCENE, THIRD_PARTY_DIR

needs_scene = pytest.mark.skipif(
    not DEFAULT_SCENE.exists() or not (THIRD_PARTY_DIR / "stretch3").exists(), reason="scene/assets missing"
)


@pytest.fixture
def setup():
    layout = load_layout(DEFAULT_LAYOUT)
    model = mujoco.MjModel.from_xml_path(str(DEFAULT_SCENE))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("stow").id)
    mujoco.mj_forward(model, data)
    robot = Stretch(model, data)
    return layout, robot, SimRunner(model, data)


def _fake(layout, robot, **kwargs):
    grid = build_occupancy_grid(robot.model, robot.data, room_bounds(layout), 0.0)
    return FakeSlamBackend(robot.true_base_pose, dock_pose(layout), grid, **kwargs)


@needs_scene
def test_robot_drives_on_the_fused_estimate(setup):
    layout, robot, runner = setup
    stack = attach_slam(robot, runner, layout, _fake(layout, robot, xy_noise=0.005), inflation=0.34)
    assert robot.localizer is stack.localizer
    runner.run_until(lambda: False, 4.0, lambda: robot.drive(0.15, 0.4))
    stats = stack.monitor.stats()
    assert stats.samples > 150 and stats.max_xy_error < 0.03
    assert stack.frontend.stats.acks >= 2 and stack.frontend.stats.missed_acks == 0
    assert stack.localizer.corrections == stack.frontend.stats.acks
    report = stack.report()
    assert report["mode"] == "slam" and report["scans"]["scans"] == stack.frontend.stats.scans
    assert not stack.maps.refresh().is_free_xy((0.0, layout.shelves[0].pos[1]))  # shelf row is mapped


@needs_scene
def test_scans_are_paced_by_the_lidar_rate(setup):
    layout, robot, runner = setup
    backend = _fake(layout, robot)
    stack = attach_slam(robot, runner, layout, backend, inflation=0.34)
    runner.wait(1.0)
    assert backend.published == stack.frontend.stats.scans == 11  # t=0 plus 10 Hz for 1 s
    assert stack.frontend.stats.acks == 1                           # stationary: only the first is processed


@needs_scene
def test_missing_ack_falls_back_to_odometry(setup):
    layout, robot, runner = setup

    class Silent(FakeSlamBackend):
        def wait_ack(self, sim_ns, timeout):
            return None, []

    stack = attach_slam(robot, runner, layout, Silent(robot.true_base_pose, dock_pose(layout), build_occupancy_grid(robot.model, robot.data, room_bounds(layout), 0.0)), inflation=0.34)
    runner.run_until(lambda: False, 2.0, lambda: robot.drive(0.0, 0.5))
    assert stack.frontend.stats.missed_acks >= 2 and stack.localizer.corrections == 0
    est, true = robot.base_pose(), robot.true_base_pose()
    assert abs(math.remainder(est.yaw - true.yaw, 2 * math.pi)) < math.radians(2)  # gyro odometry alone
    assert np.linalg.norm(est.xy - true.xy) < 0.03


def test_rosbridge_url_is_validated():
    with pytest.raises(ValueError, match="ws://"):
        RosbridgeSlamBackend("http://localhost:9090")


def test_unreachable_rosbridge_raises_friendly_error():
    with pytest.raises(SlamUnavailableError, match="qcw-slam up"):
        RosbridgeSlamBackend("ws://127.0.0.1:9", connect_timeout=1.0)
