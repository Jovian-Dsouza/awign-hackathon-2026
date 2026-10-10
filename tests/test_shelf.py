import mujoco
import numpy as np
import pytest

from qc_warehouse.config import load_layout
from qc_warehouse.constants import DEFAULT_LAYOUT
from qc_warehouse.materials import add_materials
from qc_warehouse.shelf import add_shelf_unit, slot_site_name


@pytest.fixture
def layout():
    return load_layout(DEFAULT_LAYOUT)


def _compiled_single_shelf(layout):
    spec = mujoco.MjSpec()
    add_materials(spec)
    add_shelf_unit(spec, layout.shelf, layout.shelves[0])
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def test_slot_site_name():
    assert slot_site_name("A", 1, 2) == "shelf_A_L1_S2"


def test_shelf_has_site_per_slot(layout):
    model, _ = _compiled_single_shelf(layout)
    cfg = layout.shelf
    for level in range(len(cfg.board_heights)):
        for slot in range(cfg.slots_per_level):
            assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, slot_site_name("A", level, slot)) >= 0


def test_slot_sites_sit_on_board_tops(layout):
    model, data = _compiled_single_shelf(layout)
    placement = layout.shelves[0]
    for level, height in enumerate(layout.shelf.board_heights):
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, slot_site_name("A", level, 0))
        pos = data.site_xpos[sid]
        assert pos[2] == pytest.approx(height)
        assert abs(pos[0] - placement.pos[0]) < layout.shelf.width / 2


def test_shelf_is_static(layout):
    model, _ = _compiled_single_shelf(layout)
    assert model.njnt == 0
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "shelf_A")
    assert body >= 0


def test_slots_ordered_left_to_right_facing_shelf(layout):
    """Facing the shelf (looking north, +y), slot 0 is on the left (smaller x)."""
    model, data = _compiled_single_shelf(layout)
    xs = [
        data.site_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, slot_site_name("A", 1, s))][0]
        for s in range(layout.shelf.slots_per_level)
    ]
    assert np.all(np.diff(xs) > 0)
