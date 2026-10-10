import math

import numpy as np
import pytest

from qc_warehouse.autonomy.odometry import EXACT, GyroIntegrator, OdometryNoise, WheelGyroOdometry
from qc_warehouse.autonomy.stretch import WHEEL_RADIUS, WHEEL_SEPARATION

DT = 0.02
NS = 20_000_000


def _run(odom, v, w, ticks, start_tick=1, left=0.0, right=0.0):
    for k in range(start_tick, start_tick + ticks):
        left += (v - w * WHEEL_SEPARATION / 2) / WHEEL_RADIUS * DT
        right += (v + w * WHEEL_SEPARATION / 2) / WHEEL_RADIUS * DT
        odom.update(k * NS, left, right, w * DT)
    return left, right


def test_straight_line_is_exact_without_noise():
    odom = WheelGyroOdometry(WHEEL_RADIUS, EXACT)
    odom.reset(0, 0.0, 0.0)
    _run(odom, 0.2, 0.0, 50)
    assert odom.pose.x == pytest.approx(0.2) and odom.pose.y == pytest.approx(0.0, abs=1e-12)


def test_arc_matches_analytic_circle():
    odom = WheelGyroOdometry(WHEEL_RADIUS, EXACT)
    odom.reset(0, 0.0, 0.0)
    v, w, ticks = 0.2, 0.5, 100  # 2 s -> 1 rad of a 0.4 m radius circle
    _run(odom, v, w, ticks)
    r, theta = v / w, w * ticks * DT
    assert odom.pose.yaw == pytest.approx(theta)
    assert odom.pose.x == pytest.approx(r * math.sin(theta), abs=1e-5)
    assert odom.pose.y == pytest.approx(r * (1 - math.cos(theta)), abs=1e-5)


def test_yaw_comes_from_gyro_not_wheel_slip():
    odom = WheelGyroOdometry(WHEEL_RADIUS, EXACT)
    odom.reset(0, 0.0, 0.0)
    odom.update(NS, 0.1, 0.0, 0.0)  # one wheel slipped forward, gyro says no turn
    assert odom.pose.yaw == 0.0


def test_noise_is_seeded_and_biased():
    noisy = OdometryNoise(encoder_cpr=4096, radius_scale=1.01, gyro_sigma=0.002, gyro_bias=0.001)
    poses = []
    for _ in range(2):
        odom = WheelGyroOdometry(WHEEL_RADIUS, noisy, np.random.default_rng(7))
        odom.reset(0, 0.0, 0.0)
        _run(odom, 0.2, 0.0, 250)
        poses.append(odom.pose)
    assert poses[0] == poses[1]
    assert poses[0].x == pytest.approx(1.0 * 1.01, abs=2e-3)
    assert poses[0].yaw != 0.0


def test_history_lookup_and_trim():
    odom = WheelGyroOdometry(WHEEL_RADIUS, EXACT, history_seconds=0.1)
    odom.reset(0, 0.0, 0.0)
    _run(odom, 0.2, 0.0, 20)
    assert odom.pose_at(20 * NS) == odom.pose
    assert odom.pose_at(19 * NS).x == pytest.approx(0.2 * 19 * DT)
    with pytest.raises(KeyError):
        odom.pose_at(1 * NS)  # older than the history window


def test_update_before_reset_raises():
    with pytest.raises(RuntimeError, match="reset"):
        WheelGyroOdometry(WHEEL_RADIUS, EXACT).update(NS, 0.0, 0.0, 0.0)


def test_gyro_integrator_accumulates_and_resets():
    rates = iter([1.0, 2.0, 3.0])
    gyro = GyroIntegrator(lambda: next(rates), 0.5)
    gyro()
    gyro()
    assert gyro.take() == pytest.approx(1.5)
    gyro()
    assert gyro.take() == pytest.approx(1.5)
    assert gyro.take() == 0.0
