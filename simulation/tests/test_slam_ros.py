"""Integration with the real slam_toolbox container (marker `ros`; skipped when it isn't running).

Start it with `uv run qcw-slam up`. Each test session starts a fresh SLAM session.
"""

import math

import mujoco
import numpy as np
import pytest

from qc_warehouse import slam_cli
from qc_warehouse.autonomy import ros_messages as rm
from qc_warehouse.autonomy.motion import rotate_to
from qc_warehouse.autonomy.runner import SimRunner
from qc_warehouse.autonomy.slam_bridge import RosbridgeSlamBackend
from qc_warehouse.autonomy.slam_stack import attach_slam
from qc_warehouse.autonomy.stretch import Stretch
from qc_warehouse.config import load_layout
from qc_warehouse.constants import DEFAULT_LAYOUT, DEFAULT_SCENE
from conftest import ROS_MARKS, ROSBRIDGE_URL

URL = ROSBRIDGE_URL
pytestmark = ROS_MARKS


@pytest.fixture(scope="module")
def slam_run():
    """Fresh SLAM session; robot spins once on the dock (the same survey the picker does)."""
    slam_cli.compose("restart")
    layout = load_layout(DEFAULT_LAYOUT)
    model = mujoco.MjModel.from_xml_path(str(DEFAULT_SCENE))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("stow").id)
    mujoco.mj_forward(model, data)
    robot, runner = Stretch(model, data), SimRunner(model, data)
    stack = attach_slam(robot, runner, layout, RosbridgeSlamBackend(URL), inflation=0.34)
    start = robot.base_pose().yaw
    for k in range(1, 4):
        rotate_to(robot, runner, start + 2 * math.pi * k / 3)
    yield layout, robot, stack
    stack.close()


def test_every_predicted_scan_is_processed(slam_run):
    _, _, stack = slam_run
    stats = stack.frontend.stats
    assert stats.acks >= 10
    assert stats.missed_acks == 0 and stats.unexpected_acks == 0


def test_pose_tracks_ground_truth(slam_run):
    _, robot, stack = slam_run
    assert stack.monitor.stats().mean_xy_error < 0.05
    assert np.linalg.norm(robot.base_pose().xy - robot.true_base_pose().xy) < 0.08


def test_map_shows_walls_and_shelves(slam_run):
    layout, _, stack = slam_run
    stack.maps.refresh()
    values = stack.maps.values
    assert (values >= 0).mean() > 0.5  # most of the room observed from the dock
    grid = stack.maps.current()
    half_x = layout.room.size[0] / 2
    assert not grid.is_free_xy((-half_x + 0.05, 0.0))            # west wall
    assert not grid.is_free_xy(layout.shelves[0].pos)             # shelf A
    assert grid.is_free_xy(layout.robot_dock.pos)                 # robot's own spot


def test_map_stamp_is_recent(slam_run):
    _, _, stack = slam_run
    msg = stack.backend.latest_map(stack.frontend.last_processed_ns, 3.0)
    assert rm.parse_stamp(msg["header"]["stamp"]) >= stack.frontend.last_processed_ns
