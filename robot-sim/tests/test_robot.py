import mujoco
import numpy as np
import pytest

from dsr_sim.build_scene import build_scene_xml
from dsr_sim.config import load_inventory, load_layout
from dsr_sim.constants import DEFAULT_INVENTORY, DEFAULT_LAYOUT
from dsr_sim.control import Robot, wheel_speeds


@pytest.fixture(scope="module")
def scene(tmp_path_factory):
    out = tmp_path_factory.mktemp("scene") / "scene.xml"
    layout = load_layout(DEFAULT_LAYOUT)
    build_scene_xml(layout, load_inventory(DEFAULT_INVENTORY, layout), out)
    model = mujoco.MjModel.from_xml_path(str(out))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    return model, data


def step(model, data, seconds):
    end = data.time + seconds
    while data.time < end:
        mujoco.mj_step(model, data)


def test_cameras_exist(scene):
    model, _ = scene
    for cam in ("wrist_cam", "chassis_cam", "overview"):
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, cam) >= 0


def test_forward_moves_along_x(scene):
    model, data = scene
    robot = Robot(model, data)
    x0, y0, _ = robot.pose
    robot.set_twist(0.2, 0, 0)
    step(model, data, 1.0)
    robot.stop()
    x1, y1, _ = robot.pose
    assert 0.1 < x1 - x0 < 0.25
    assert abs(y1 - y0) < 0.03


def test_strafe_is_sideways(scene):
    model, data = scene
    robot = Robot(model, data)
    x0, y0, _ = robot.pose
    robot.set_twist(0, 0.2, 0)
    step(model, data, 1.0)
    robot.stop()
    x1, y1, _ = robot.pose
    assert y1 - y0 > 0.1
    assert abs(x1 - x0) < 0.03


def test_mecanum_ik_signs():
    w = wheel_speeds(0.2, 0, 0)
    assert all(v > 0 for v in w.values())
    s = wheel_speeds(0, 0.2, 0)
    assert s["fl"] < 0 < s["fr"] and s["rl"] > 0 > s["rr"]
    r = wheel_speeds(0, 0, 1.0)
    assert r["fl"] < 0 < r["fr"] and r["rl"] < 0 < r["rr"]


def test_arm_tracks_command(scene):
    model, data = scene
    robot = Robot(model, data)
    robot.set_arm_deg("reach_low")
    step(model, data, 3.0)
    got = robot.arm_deg()
    assert abs(got["shoulder_lift"] - 10) < 8
    assert abs(got["gripper"] - 60) < 10
