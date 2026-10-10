import mujoco
import numpy as np
import pytest

from qc_warehouse.autonomy.perception import HeadCamera, backproject, identify_items


def _scene() -> mujoco.MjModel:
    """A camera 1 m above the floor looking straight down at two boxes."""
    spec = mujoco.MjSpec()
    spec.worldbody.add_light(pos=[0, 0, 3])
    spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2, 2, 0.1])
    spec.worldbody.add_camera(name="cam", pos=[0, 0, 1.0], fovy=60)
    for name, pos, half in (("red", [0.15, 0.05, 0.05], [0.04, 0.03, 0.05]), ("blue", [-0.2, -0.1, 0.02], [0.05, 0.05, 0.02])):
        body = spec.worldbody.add_body(name=name, pos=pos)
        body.add_freejoint()
        body.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=half)
    return spec.compile()


def test_backproject_center_pixel_on_axis():
    depth = np.full((4, 4), 2.0)
    mask = np.zeros((4, 4), dtype=bool)
    mask[2, 2] = True
    cam_mat = np.eye(3)  # looks along world -z
    pts = backproject(depth, mask, np.array([0.0, 0.0, 3.0]), cam_mat, fovy_deg=90)
    # pixel (2,2) is half a pixel right/down of centre: focal = 2 px -> x = 0.5/2*2 = 0.5
    assert pts[0] == pytest.approx([0.5, -0.5, 1.0])


def test_identify_and_localise_items():
    model = _scene()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    cam = HeadCamera(model, "cam", width=320, height=240)
    try:
        frame = cam.capture(data)
    finally:
        cam.close()
    bodies = {"red": model.body("red").id, "blue": model.body("blue").id}
    found = identify_items(model, frame, bodies)
    assert set(found) == {"red", "blue"}
    red = found["red"]
    assert red.center[:2] == pytest.approx([0.15, 0.05], abs=0.01)
    assert red.max_xyz[2] == pytest.approx(0.10, abs=0.01)  # top face
    assert red.size[0] == pytest.approx(0.08, abs=0.015)
    assert found["blue"].center[:2] == pytest.approx([-0.2, -0.1], abs=0.01)


def test_unknown_camera_raises():
    with pytest.raises(ValueError, match="camera"):
        HeadCamera(_scene(), "nope")
