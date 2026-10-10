import math

import pytest

from qc_warehouse.autonomy.frames import IDENTITY, BasePose, compose, inverse, quat_to_yaw, yaw_to_quat


def _close(a: BasePose, b: BasePose, tol: float = 1e-9) -> bool:
    return abs(a.x - b.x) < tol and abs(a.y - b.y) < tol and abs(math.remainder(a.yaw - b.yaw, 2 * math.pi)) < tol


def test_compose_translates_in_parent_frame():
    a = BasePose(1.0, 2.0, math.pi / 2)
    assert _close(compose(a, BasePose(1.0, 0.0, 0.0)), BasePose(1.0, 3.0, math.pi / 2))


def test_inverse_round_trips():
    a = BasePose(0.4, -1.3, 2.7)
    assert _close(compose(a, inverse(a)), IDENTITY)
    assert _close(compose(inverse(a), a), IDENTITY)


def test_map_to_odom_correction_recovers_slam_pose():
    # map_T_odom = map_T_base * inv(odom_T_base) applied to a later odom pose.
    map_T_base, odom_T_base = BasePose(1.0, 1.0, 0.3), BasePose(0.9, 1.05, 0.25)
    map_T_odom = compose(map_T_base, inverse(odom_T_base))
    assert _close(compose(map_T_odom, odom_T_base), map_T_base)


@pytest.mark.parametrize("yaw", [0.0, 0.5, -2.0, math.pi - 1e-6])
def test_quaternion_round_trip(yaw):
    assert quat_to_yaw(yaw_to_quat(yaw)) == pytest.approx(yaw)
