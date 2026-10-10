import math

import numpy as np
import pytest

from qc_warehouse.autonomy import ros_messages as rm
from qc_warehouse.autonomy.frames import BasePose
from qc_warehouse.autonomy.lidar import LidarSpec


def test_stamp_offsets_and_round_trips():
    stamp = rm.stamp(1_500_000_001)
    assert stamp == {"sec": 1001, "nanosec": 500_000_001}
    assert rm.parse_stamp(stamp) == 1_500_000_001


def test_clock_and_tf_messages():
    assert rm.clock_msg(0) == {"clock": {"sec": 1000, "nanosec": 0}}
    tf = rm.tf_msg(20_000_000, "odom", "base_link", BasePose(1.0, -2.0, math.pi / 2))
    (t,) = tf["transforms"]
    assert t["header"] == {"stamp": {"sec": 1000, "nanosec": 20_000_000}, "frame_id": "odom"}
    assert t["child_frame_id"] == "base_link"
    assert t["transform"]["translation"] == {"x": 1.0, "y": -2.0, "z": 0.0}
    assert t["transform"]["rotation"]["z"] == pytest.approx(math.sqrt(0.5))


def test_scan_message_is_a_full_circle_and_json_safe():
    spec = LidarSpec(beams=4)
    msg = rm.scan_msg(0, np.array([1.23456, 2.0, 13.0, 0.5]), spec, frame_id="laser")
    assert msg["angle_min"] == pytest.approx(-math.pi)
    assert msg["angle_max"] == pytest.approx(-math.pi + 3 * msg["angle_increment"])
    assert msg["angle_increment"] == pytest.approx(math.pi / 2)
    assert msg["ranges"] == [1.235, 2.0, 13.0, 0.5]
    assert all(isinstance(r, float) for r in msg["ranges"])


def test_parse_pose():
    msg = {
        "header": {"stamp": {"sec": 1002, "nanosec": 5}, "frame_id": "map"},
        "pose": {"pose": {"position": {"x": 1.0, "y": 2.0, "z": 0.0}, "orientation": {"x": 0, "y": 0, "z": 1.0, "w": 0.0}}},
    }
    ns, pose = rm.parse_pose(msg)
    assert ns == 2_000_000_005
    assert (pose.x, pose.y) == (1.0, 2.0) and abs(pose.yaw) == pytest.approx(math.pi)


def _map_msg(width=4, height=3, res=0.5, origin=(-1.0, -0.5), data=None):
    return {
        "header": {"stamp": {"sec": 1001, "nanosec": 0}, "frame_id": "map"},
        "info": {
            "resolution": res, "width": width, "height": height,
            "origin": {"position": {"x": origin[0], "y": origin[1], "z": 0.0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
        },
        "data": data if data is not None else [-1] * (width * height),
    }


def test_map_resampling_is_row_major_and_frame_aware():
    data = [-1] * 12
    data[1 * 4 + 2] = 100      # row y=1, column x=2 -> map cell centre (0.25, 0.25)
    msg = _map_msg(data=data)
    # Identity map frame: query the layout point (0.25, 0.25).
    values = rm.map_to_layout(msg, BasePose(0, 0, 0), origin=(0.25, 0.25), shape=(1, 1), resolution=0.5)
    assert values[0, 0] == 100
    # Map frame shifted by +1 m in x (dock at x=1): the same cell now sits at layout x=1.25.
    shifted = rm.map_to_layout(msg, BasePose(1.0, 0, 0), origin=(1.25, 0.25), shape=(1, 1), resolution=0.5)
    assert shifted[0, 0] == 100
    # Rotated 90 deg: map (0.25, 0.25) -> layout (-0.25, 0.25).
    rotated = rm.map_to_layout(msg, BasePose(0, 0, math.pi / 2), origin=(-0.25, 0.25), shape=(1, 1), resolution=0.5)
    assert rotated[0, 0] == 100
    # Outside the ROS map is unknown.
    assert rm.map_to_layout(msg, BasePose(0, 0, 0), origin=(9.0, 9.0), shape=(1, 1), resolution=0.5)[0, 0] == -1


def test_map_stamp():
    assert rm.parse_stamp(_map_msg()["header"]["stamp"]) == 1_000_000_000


def test_map_rejects_inconsistent_size():
    with pytest.raises(ValueError, match="cells"):
        rm.map_to_layout(_map_msg(data=[0, 0]), BasePose(0, 0, 0), origin=(0, 0), shape=(1, 1), resolution=0.5)


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (lambda m: m.pop("header"), "malformed"),
        (lambda m: m["info"].update(width=10**5, height=10**5), "out of range"),
        (lambda m: m["info"].update(resolution=float("nan")), "finite"),
        (lambda m: m.update(data=[0] * 5), "cells"),
        (lambda m: m["info"]["origin"].pop("position"), "malformed"),
    ],
)
def test_untrusted_maps_are_rejected_before_allocation(mutate, message):
    msg = _map_msg()
    mutate(msg)
    with pytest.raises(ValueError, match=message):
        rm.validate_map(msg)


def test_out_of_range_cell_values_raise_value_error():
    with pytest.raises(ValueError, match="int8"):
        rm.map_to_layout(_map_msg(data=[1000] * 12), BasePose(0, 0, 0), origin=(0, 0), shape=(1, 1), resolution=0.5)


def test_downsampling_keeps_thin_walls():
    # 0.1 m ROS map, one occupied cell column at x in [0.1, 0.2); layout cells of 0.4 m centred at 0.0 and 0.4.
    data = [0] * 40
    for y in range(4):
        data[y * 10 + 1] = 100
    msg = _map_msg(width=10, height=4, res=0.1, origin=(0.0, 0.0), data=data)
    values = rm.map_to_layout(msg, BasePose(0, 0, 0), origin=(0.2, 0.2), shape=(2, 1), resolution=0.4)
    assert values[0, 0] == 100   # the wall lies inside cell 0's footprint, not at its centre
    assert values[1, 0] == 0
