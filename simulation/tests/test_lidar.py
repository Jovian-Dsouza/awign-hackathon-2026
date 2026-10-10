import math

import mujoco
import numpy as np
import pytest

from qc_warehouse.autonomy.lidar import Lidar2D, LidarSpec

# 4 x 6 m room (inner faces at x=+-2, y=+-3); a "robot" with a mast 0.3 m in front of its laser.
ROOM = """
<mujoco>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1"/>
    <geom name="east" type="box" size="0.1 3 0.5" pos="2.1 0 0.5"/>
    <geom name="west" type="box" size="0.1 3 0.5" pos="-2.1 0 0.5"/>
    <geom name="north" type="box" size="2 0.1 0.5" pos="0 3.1 0.5"/>
    <geom name="south" type="box" size="2 0.1 0.5" pos="0 -3.1 0.5"/>
    <body name="robot" pos="0 0 0">
      <freejoint/>
      <geom name="chassis" type="box" size="0.05 0.05 0.05" pos="0 0 0.05"/>
      <geom name="mast" type="box" size="0.02 0.02 0.5" pos="0.3 0 0.5"/>
      <body name="laser" pos="0 0 0.2" quat="0 0 0 1"/>
    </body>
  </worldbody>
</mujoco>
"""


@pytest.fixture
def scene():
    model = mujoco.MjModel.from_xml_string(ROOM)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def _beam(spec: LidarSpec, angle: float) -> int:
    return round((angle + math.pi) / (2 * math.pi / spec.beams)) % spec.beams


def test_exact_ranges_in_laser_frame(scene):
    model, data = scene
    spec = LidarSpec(beams=360, noise_sigma=0.0, dropout=0.0)
    lidar = Lidar2D(model, "laser", "robot", spec)
    ranges = lidar.scan(data)
    # The laser is yawed 180 deg: its +x beam (angle 0) points at the west wall.
    assert ranges[_beam(spec, 0.0)] == pytest.approx(2.0, abs=1e-6)
    assert ranges[_beam(spec, math.pi / 2)] == pytest.approx(3.0, abs=1e-6)   # laser +y = world -y
    assert ranges[_beam(spec, -math.pi / 2)] == pytest.approx(3.0, abs=1e-6)


def test_self_hits_are_no_return(scene):
    model, data = scene
    spec = LidarSpec(beams=360, noise_sigma=0.0, dropout=0.0)
    ranges = Lidar2D(model, "laser", "robot", spec).scan(data)
    # The mast sits along world +x = laser angle pi (beam 0 is -pi).
    assert ranges[0] == spec.no_return


def test_out_of_range_is_no_return(scene):
    model, data = scene
    spec = LidarSpec(beams=360, range_max=2.5, noise_sigma=0.0, dropout=0.0)
    ranges = Lidar2D(model, "laser", "robot", spec).scan(data)
    assert ranges[_beam(spec, math.pi / 2)] == spec.no_return
    assert ranges[_beam(spec, 0.0)] == pytest.approx(2.0, abs=1e-6)


def test_noise_is_seeded(scene):
    model, data = scene
    spec = LidarSpec(beams=90, noise_sigma=0.01, dropout=0.1)
    a = Lidar2D(model, "laser", "robot", spec, np.random.default_rng(3)).scan(data)
    b = Lidar2D(model, "laser", "robot", spec, np.random.default_rng(3)).scan(data)
    assert np.array_equal(a, b)
    assert np.any(a == spec.no_return)
    assert np.all(np.isfinite(a))


def test_unknown_body_raises(scene):
    model, _ = scene
    with pytest.raises(ValueError, match="nope"):
        Lidar2D(model, "nope", "robot")
