"""Integration tests: build the full scene from the default configs and simulate it."""

import mujoco
import numpy as np
import pytest

from qc_warehouse.build_scene import build_scene_xml
from qc_warehouse.config import load_inventory, load_layout
from qc_warehouse.constants import DEFAULT_INVENTORY, DEFAULT_LAYOUT, THIRD_PARTY_DIR
from qc_warehouse.shelf import slot_site_name, slot_width

pytestmark = pytest.mark.skipif(
    not (THIRD_PARTY_DIR / "hope").exists() or not (THIRD_PARTY_DIR / "ycb").exists(),
    reason="third-party assets missing: run `uv run python scripts/fetch_assets.py`",
)

SETTLE_SECONDS = 2.0
MAX_XY_DRIFT = 0.02
MAX_Z_DRIFT = 0.01


@pytest.fixture(scope="module")
def configs():
    layout = load_layout(DEFAULT_LAYOUT)
    return layout, load_inventory(DEFAULT_INVENTORY, layout)


@pytest.fixture(scope="module")
def model(tmp_path_factory, configs):
    layout, inventory = configs
    out = tmp_path_factory.mktemp("scene") / "scene.xml"
    build_scene_xml(layout, inventory, out)
    return mujoco.MjModel.from_xml_path(str(out))


def _id(model, obj, name):
    idx = mujoco.mj_name2id(model, obj, name)
    assert idx >= 0, f"{name} missing from scene"
    return idx


def test_scene_has_three_shelves_and_station(model, configs):
    layout, _ = configs
    for shelf in layout.shelves:
        _id(model, mujoco.mjtObj.mjOBJ_BODY, f"shelf_{shelf.name}")
    _id(model, mujoco.mjtObj.mjOBJ_BODY, "packing_station")
    _id(model, mujoco.mjtObj.mjOBJ_SITE, "drop_zone")
    _id(model, mujoco.mjtObj.mjOBJ_SITE, "robot_spawn")
    for cam in ("overview", "packing_station_cam", "shelf_A_cam", "shelf_B_cam", "shelf_C_cam"):
        _id(model, mujoco.mjtObj.mjOBJ_CAMERA, cam)


def test_station_is_separate_from_shelves(model, configs):
    layout, _ = configs
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    drop = data.site_xpos[_id(model, mujoco.mjtObj.mjOBJ_SITE, "drop_zone")]
    for shelf in layout.shelves:
        shelf_pos = data.xpos[_id(model, mujoco.mjtObj.mjOBJ_BODY, f"shelf_{shelf.name}")]
        assert np.linalg.norm(drop[:2] - shelf_pos[:2]) > 2.0


def test_every_item_is_a_free_body(model, configs):
    _, inventory = configs
    for item in inventory:
        body = _id(model, mujoco.mjtObj.mjOBJ_BODY, f"item_{item.name}")
        jnt = model.body_jntadr[body]
        assert jnt >= 0 and model.jnt_type[jnt] == mujoco.mjtJoint.mjJNT_FREE


def test_items_settle_on_their_shelves(model, configs):
    layout, inventory = configs
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    bodies = {i.name: _id(model, mujoco.mjtObj.mjOBJ_BODY, f"item_{i.name}") for i in inventory}
    start = {name: data.xpos[b].copy() for name, b in bodies.items()}

    while data.time < SETTLE_SECONDS:
        mujoco.mj_step(model, data)

    assert not np.any(np.isnan(data.qpos))
    for item in inventory:
        b = bodies[item.name]
        drift = data.xpos[b] - start[item.name]
        assert np.linalg.norm(drift[:2]) < MAX_XY_DRIFT, f"{item.name} slid {drift}"
        assert abs(drift[2]) < MAX_Z_DRIFT, f"{item.name} fell/bounced {drift}"
        site = data.site_xpos[_id(model, mujoco.mjtObj.mjOBJ_SITE, slot_site_name(item.shelf, item.level, item.slot))]
        assert np.linalg.norm(data.xpos[b][:2] - site[:2]) < 0.1
        assert np.linalg.norm(data.cvel[b]) < 0.05, f"{item.name} still moving"


def test_items_fit_in_their_slot(model, configs):
    """Item collision footprint must not overlap a neighbouring slot."""
    layout, inventory = configs
    width = slot_width(layout.shelf)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for item in inventory:
        b = _id(model, mujoco.mjtObj.mjOBJ_BODY, f"item_{item.name}")
        geoms = [g for g in range(model.ngeom) if model.body_rootid[model.geom_bodyid[g]] == b and model.geom_contype[g]]
        assert geoms, f"{item.name} has no collision geoms"
        xs = np.concatenate([data.geom_xpos[g][0] + np.array([-1, 1]) * model.geom_rbound[g] for g in geoms])
        assert xs.max() - xs.min() < width * 1.2, item.name
