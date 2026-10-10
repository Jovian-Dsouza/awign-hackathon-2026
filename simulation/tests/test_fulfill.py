"""Warehouse model checks plus an end-to-end order run (slow: ~5 min of sim, ~1 min wall)."""

import mujoco
import numpy as np
import pytest

from qc_warehouse.autonomy.fulfill import OrderPicker
from qc_warehouse.autonomy.mapping import StaticMap
from qc_warehouse.autonomy.navigation import build_occupancy_grid
from qc_warehouse.autonomy.orders import Order
from qc_warehouse.autonomy.perception import HeadCamera
from qc_warehouse.autonomy.runner import SimRunner
from qc_warehouse.autonomy.stretch import Stretch
from qc_warehouse.autonomy.warehouse import Warehouse
from qc_warehouse.config import load_inventory, load_layout
from qc_warehouse.constants import DEFAULT_INVENTORY, DEFAULT_LAYOUT, DEFAULT_SCENE, THIRD_PARTY_DIR
from qc_warehouse.pick import NAV_INFLATION
from qc_warehouse.shelf import slot_site_name
from qc_warehouse.station import tote_site_name

pytestmark = pytest.mark.skipif(
    not DEFAULT_SCENE.exists() or not (THIRD_PARTY_DIR / "stretch3").exists(),
    reason="scene/assets missing: run fetch_assets.py and qcw-build",
)


@pytest.fixture
def world():
    layout = load_layout(DEFAULT_LAYOUT)
    inventory = load_inventory(DEFAULT_INVENTORY, layout)
    model = mujoco.MjModel.from_xml_path(str(DEFAULT_SCENE))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("stow").id)
    mujoco.mj_forward(model, data)
    return model, data, layout, Warehouse(model, data, layout, inventory)


def test_tote_geometry(world):
    _, _, layout, wh = world
    tote = wh.tote(0)
    assert tote.rim_z == pytest.approx(layout.station.table_size[2] + layout.station.tote_size[2], abs=1e-6)
    assert tote.floor_z < tote.rim_z
    with pytest.raises(IndexError):
        wh.tote(layout.station.num_totes)


def test_targets_come_from_config_and_match_the_scene(world):
    _, data, layout, wh = world
    for name, item in wh.inventory.items():
        site = data.site(slot_site_name(item.shelf, item.level, item.slot)).xpos
        assert wh.slot_position(item) == pytest.approx(site, abs=1e-6), name
    for index in range(layout.station.num_totes):
        site = data.site(tote_site_name(index)).xpos
        tote = wh.tote(index)
        assert [*tote.center, tote.floor_z] == pytest.approx(site, abs=1e-6)


def test_items_start_on_shelves_not_in_totes(world):
    _, _, _, wh = world
    for name in wh.inventory:
        assert not wh.item_in_tote(name, wh.tote(0))


def test_ceiling_is_board_above(world):
    _, _, layout, wh = world
    item = wh.item("ketchup")  # level 1
    assert wh.ceiling_z(item) == pytest.approx(layout.shelf.board_heights[2] - layout.shelf.board_thickness)


def test_unknown_item_raises(world):
    with pytest.raises(KeyError):
        world[3].item("caviar")


@pytest.mark.slow
def test_end_to_end_order(world):
    """Stretch fetches a side-grasp bottle and a top-down flat pack into tote_0."""
    model, data, layout, wh = world
    half_x, half_y = layout.room.size[0] / 2, layout.room.size[1] / 2
    grid = build_occupancy_grid(model, data, (-half_x, -half_y, half_x, half_y), NAV_INFLATION)
    robot = Stretch(model, data)
    camera = HeadCamera(model, robot.head_camera)
    try:
        picker = OrderPicker(robot, SimRunner(model, data), wh, StaticMap(grid), camera, on_event=lambda _: None)
        (result,) = picker.fulfil((Order("T-1", ("bbq_sauce", "butter")),))
    finally:
        camera.close()
    assert [r.grasp for r in result.items] == ["side", "top"]
    assert result.success, [r.detail for r in result.items]
    for name in ("bbq_sauce", "butter"):
        assert wh.item_in_tote(name, wh.tote(0))
    dock = np.asarray(layout.robot_dock.pos)
    assert np.linalg.norm(robot.base_pose().xy - dock) < 0.15  # returned to the dock


def test_lost_localization_stops_the_run_but_keeps_results(world, monkeypatch):
    from qc_warehouse.autonomy.localization import LocalizationLostError

    model, data, layout, wh = world
    robot = Stretch(model, data)
    grid = build_occupancy_grid(model, data, (-5, -4, 5, 4), NAV_INFLATION)
    events = []
    picker = OrderPicker(robot, SimRunner(model, data), wh, StaticMap(grid), camera=None, on_event=events.append)

    def lose(*_):
        raise LocalizationLostError("rosbridge went away")

    monkeypatch.setattr(picker, "_pick_and_pack", lose)
    monkeypatch.setattr(picker, "_return_to_dock", lambda: pytest.fail("must not drive without localization"))
    (result,) = picker.fulfil((Order("L-1", ("ketchup", "milk")),))
    assert [r.success for r in result.items] == [False, False]
    assert "localization lost" in result.items[0].detail
    assert result.items[1].detail.startswith("not attempted")
    assert any("stopped in place" in e for e in events)
