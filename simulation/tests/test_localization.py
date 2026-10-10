import math

import mujoco
import numpy as np
import pytest

from qc_warehouse.autonomy.frames import BasePose, compose
from qc_warehouse.autonomy.localization import LocalizationMonitor, SlamLocalizer
from qc_warehouse.autonomy.odometry import EXACT, WheelGyroOdometry
from qc_warehouse.autonomy.stretch import WHEEL_RADIUS, Stretch
from qc_warehouse.constants import DEFAULT_SCENE, THIRD_PARTY_DIR

needs_scene = pytest.mark.skipif(
    not DEFAULT_SCENE.exists() or not (THIRD_PARTY_DIR / "stretch3").exists(), reason="scene/assets missing"
)


def _odometry_at(pose: BasePose) -> WheelGyroOdometry:
    odom = WheelGyroOdometry(WHEEL_RADIUS, EXACT)
    odom.reset(0, 0.0, 0.0, pose)
    return odom


def test_slam_localizer_composes_dock_correction_and_odometry():
    loc = SlamLocalizer(_odometry_at(BasePose(1.0, 0.0, 0.0)), world_T_map=BasePose(-3.0, -1.0, math.pi / 2))
    assert loc.base_pose().xy == pytest.approx([-3.0, 0.0])
    loc.apply_correction(BasePose(0.0, 0.5, 0.0))
    assert loc.map_pose().xy == pytest.approx([1.0, 0.5])
    assert loc.base_pose() == compose(BasePose(-3.0, -1.0, math.pi / 2), BasePose(1.0, 0.5, 0.0))


def test_freeze_defers_corrections_until_released():
    loc = SlamLocalizer(_odometry_at(BasePose(0.0, 0.0, 0.0)), BasePose(0.0, 0.0, 0.0))
    with loc.freeze():
        with loc.freeze():
            loc.apply_correction(BasePose(0.1, 0.0, 0.0))
        assert loc.base_pose().x == 0.0  # still frozen by the outer block
    assert loc.base_pose().x == pytest.approx(0.1)
    assert loc.corrections == 1


def test_monitor_stats():
    truth = iter([BasePose(0, 0, 0), BasePose(0, 0, 0)])
    est = iter([BasePose(0.03, 0.04, 0.0), BasePose(0.0, 0.0, math.radians(2))])
    monitor = LocalizationMonitor(lambda: next(est), lambda: next(truth))
    monitor()
    monitor()
    stats = monitor.stats()
    assert stats.samples == 2 and stats.max_xy_error == pytest.approx(0.05) and stats.mean_xy_error == pytest.approx(0.025)
    assert stats.max_yaw_error_deg == pytest.approx(2.0)
    assert LocalizationMonitor(lambda: None, lambda: None).stats().samples == 0


@pytest.fixture
def robot():
    model = mujoco.MjModel.from_xml_path(str(DEFAULT_SCENE))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("stow").id)
    mujoco.mj_forward(model, data)
    return Stretch(model, data)


class _FixedLocalizer:
    def __init__(self, pose):
        self.pose = pose

    def base_pose(self):
        return self.pose

    def freeze(self):
        import contextlib

        return contextlib.nullcontext()


@needs_scene
def test_estimated_kinematics_follow_the_estimate(robot):
    truth = robot.true_base_pose()
    true_pinch = robot.pinch_point().copy()
    true_cam = robot.data.cam_xpos[robot.model.camera(robot.head_camera).id].copy()

    robot.use_localizer(_FixedLocalizer(truth))
    assert robot.pinch_point() == pytest.approx(true_pinch, abs=1e-6)

    shifted = BasePose(truth.x + 0.05, truth.y - 0.02, truth.yaw)
    robot.use_localizer(_FixedLocalizer(shifted))
    assert robot.pinch_point() == pytest.approx(true_pinch + [0.05, -0.02, 0.0], abs=1e-6)
    cam = robot.estimated_data().cam_xpos[robot.model.camera(robot.head_camera).id]
    assert cam == pytest.approx(true_cam + [0.05, -0.02, 0.0], abs=1e-6)

    turned = BasePose(truth.x, truth.y, truth.yaw + math.pi / 2)
    robot.use_localizer(_FixedLocalizer(turned))
    local = truth.to_local(true_pinch[:2])
    assert robot.pinch_point()[:2] == pytest.approx(turned.to_world(local), abs=1e-6)
    assert robot.data.geom_xpos[0] is not None  # physical state untouched
    assert robot.true_base_pose() == truth


@needs_scene
def test_gyro_and_encoders_report_base_motion(robot):
    left0, right0 = robot.wheel_angles()
    for _ in range(100):
        robot.drive(0.0, 0.5)
        mujoco.mj_step(robot.model, robot.data, nstep=10)
    assert robot.yaw_rate() > 0.3                    # counter-clockwise turn reads positive
    left, right = robot.wheel_angles()
    assert left < left0 and right > right0           # wheels counter-rotate
    assert np.isfinite(robot.yaw_rate())
