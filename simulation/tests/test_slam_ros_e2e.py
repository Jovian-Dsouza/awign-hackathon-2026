"""One order fulfilled purely on SLAM (real slam_toolbox). Markers: slow + ros."""

import mujoco
import numpy as np
import pytest

from qc_warehouse import slam_cli
from qc_warehouse.autonomy.fulfill import OrderPicker
from qc_warehouse.autonomy.orders import Order
from qc_warehouse.autonomy.perception import HeadCamera
from qc_warehouse.autonomy.runner import SimRunner
from qc_warehouse.autonomy.slam_bridge import RosbridgeSlamBackend
from qc_warehouse.autonomy.slam_stack import attach_slam
from qc_warehouse.autonomy.stretch import Stretch
from qc_warehouse.autonomy.warehouse import Warehouse
from qc_warehouse.config import load_inventory, load_layout
from qc_warehouse.constants import DEFAULT_INVENTORY, DEFAULT_LAYOUT, DEFAULT_SCENE
from qc_warehouse.pick import NAV_INFLATION
from conftest import ROS_MARKS, ROSBRIDGE_URL as URL

pytestmark = [*ROS_MARKS, pytest.mark.slow]


def test_order_on_slam_only():
    slam_cli.compose("restart")
    layout = load_layout(DEFAULT_LAYOUT)
    inventory = load_inventory(DEFAULT_INVENTORY, layout)
    model = mujoco.MjModel.from_xml_path(str(DEFAULT_SCENE))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("stow").id)
    mujoco.mj_forward(model, data)
    robot, runner = Stretch(model, data), SimRunner(model, data)
    wh = Warehouse(model, data, layout, inventory)
    stack = attach_slam(robot, runner, layout, RosbridgeSlamBackend(URL), NAV_INFLATION)
    camera = HeadCamera(model, robot.head_camera)
    try:
        picker = OrderPicker(robot, runner, wh, stack.maps, camera, on_event=lambda _: None, survey=True)
        (result,) = picker.fulfil((Order("S-1", ("mustard",)),))
    finally:
        camera.close()
        stack.close()
    assert result.success, [r.detail for r in result.items]
    assert wh.item_in_tote("mustard", wh.tote(0))
    errors = stack.monitor.stats()
    assert errors.mean_xy_error < 0.04 and errors.max_xy_error < 0.12
    assert np.linalg.norm(robot.true_base_pose().xy - np.asarray(layout.robot_dock.pos)) < 0.2
